// rpicam_udp.cpp
#include <libcamera/libcamera.h>
#include <libcamera/controls.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <memory>
#include <vector>
#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <signal.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <sys/mman.h>
#include <cstring>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/opt.h>
#include <libswscale/swscale.h>
}

using namespace libcamera;
using namespace std::chrono_literals;

/* ---------- global state ---------- */
static std::shared_ptr<Camera> camera;
static volatile bool running = true;

/* UDP packet queue + sender thread */
static std::queue<std::vector<uint8_t>> packet_queue;
static std::mutex queue_mutex;
static std::condition_variable queue_cv;
static std::thread sender_thread;
static int udp_sock = -1;
static struct sockaddr_in dest_addr{};

/* OpenCV cascade */
static cv::CascadeClassifier face_cascade;

/* ---------- helper functions ---------- */
static void sigint_handler(int) { running = false; }

static void init_udp(const char *host, int port) {
    udp_sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_sock < 0) return;

    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port   = htons(port);
    inet_pton(AF_INET, host, &dest_addr.sin_addr);
}

static void sender_worker() {
    while (running) {
        std::unique_lock<std::mutex> lock(queue_mutex);
        queue_cv.wait(lock, [] { return !packet_queue.empty() || !running; });
        if (!running) break;

        auto pkt = std::move(packet_queue.front());
        packet_queue.pop();
        lock.unlock();

        sendto(udp_sock, pkt.data(), pkt.size(), 0,
               (struct sockaddr*)&dest_addr, sizeof(dest_addr));
    }
}

/* ---------- frame processing ---------- */
static void requestComplete(Request *request) {
    if (request->status() == Request::RequestCancelled || !running)
        return;

    const auto &buffers = request->buffers();
    for (const auto &pair : buffers) {
        FrameBuffer *buf = pair.second;
        const FrameMetadata &meta = buf->metadata();

        const auto &planes = buf->planes();
        if (planes.size() < 2) {
            request->reuse(Request::ReuseBuffers);
            camera->queueRequest(request);
            return;
        }

        /* ---- mmap NV12 planes ---- */
        size_t total = planes[1].offset + planes[1].length;
        long page    = sysconf(_SC_PAGESIZE);
        size_t aligned = (total + page - 1) & ~(page - 1);

        void *addr = mmap(nullptr, aligned, PROT_READ, MAP_SHARED,
                          planes[0].fd.get(), 0);
        if (addr == MAP_FAILED) {
            std::cerr << "mmap failed\n";
            request->reuse(Request::ReuseBuffers);
            camera->queueRequest(request);
            return;
        }

        uint8_t *y  = static_cast<uint8_t*>(addr) + planes[0].offset;
        size_t   ys = planes[0].length;
        uint8_t *uv = static_cast<uint8_t*>(addr) + planes[1].offset;
        size_t   uvs = planes[1].length;

        if (ys == 0 || uvs == 0) {
            munmap(addr, aligned);
            request->reuse(Request::ReuseBuffers);
            camera->queueRequest(request);
            return;
        }

        /* ---- NV12 to BGR (OpenCV) ---- */
        const int w = 320, h = 240;
        cv::Mat nv12(h * 3 / 2, w, CV_8UC1);
        memcpy(nv12.data,          y,  ys);
        memcpy(nv12.data + ys,    uv, uvs);

        cv::Mat bgr;
        cv::cvtColor(nv12, bgr, cv::COLOR_YUV2BGR_NV12);

        /* ---- face detection ---- */
        cv::Mat gray;
        cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);

        std::vector<cv::Rect> faces;
        face_cascade.detectMultiScale(gray, faces, 1.1, 5, 0,
                                      cv::Size(30, 30));

        for (const auto &f : faces)
            cv::rectangle(bgr, f, cv::Scalar(0, 255, 0), 2);

        /* ---- BGR to YUV420P for x264 ---- */
        static SwsContext *sws = nullptr;
        static bool sws_init = false;

        if (!sws_init) {
            sws = sws_getContext(w, h, AV_PIX_FMT_BGR24,
                                 w, h, AV_PIX_FMT_YUV420P,
                                 SWS_BILINEAR, nullptr, nullptr, nullptr);
            sws_init = true;
        }

        AVFrame *yuv = av_frame_alloc();
        yuv->format = AV_PIX_FMT_YUV420P;
        yuv->width  = w;
        yuv->height = h;
        av_frame_get_buffer(yuv, 0);
        yuv->linesize[0] = w;
        yuv->linesize[1] = yuv->linesize[2] = w / 2;

        uint8_t *bgr_src[4] = { bgr.data, nullptr, nullptr, nullptr };
        int      bgr_ls [4] = { (int)bgr.step, 0, 0, 0 };

        sws_scale(sws, bgr_src, bgr_ls, 0, h,
                  yuv->data, yuv->linesize);

        yuv->pts = meta.sequence;

        /* ---- H.264 encode (I-frame every frame) ---- */
        static AVCodecContext *cctx = nullptr;
        static AVPacket       *pkt  = nullptr;
        static bool ff_init = false;

        if (!ff_init) {
            const AVCodec *codec = avcodec_find_encoder(AV_CODEC_ID_H264);
            cctx = avcodec_alloc_context3(codec);
            cctx->width     = w;
            cctx->height    = h;
            cctx->pix_fmt   = AV_PIX_FMT_YUV420P;
            cctx->time_base = {1, 30};
            cctx->bit_rate  = 4'000'000;
            cctx->gop_size  = 1;

            av_opt_set(cctx->priv_data, "preset", "ultrafast", 0);
            av_opt_set(cctx->priv_data, "tune",   "zerolatency", 0);

            avcodec_open2(cctx, codec, nullptr);
            pkt = av_packet_alloc();
            ff_init = true;
        }

        avcodec_send_frame(cctx, yuv);
        while (avcodec_receive_packet(cctx, pkt) == 0) {
            std::vector<uint8_t> data(pkt->data, pkt->data + pkt->size);
            {
                std::lock_guard<std::mutex> lk(queue_mutex);
                packet_queue.push(std::move(data));
            }
            queue_cv.notify_one();
            av_packet_unref(pkt);
        }

        av_frame_free(&yuv);
        munmap(addr, aligned);
    }

    request->reuse(Request::ReuseBuffers);
    camera->queueRequest(request);
}

/* ---------- main ---------- */
int main(int, char **) {
    signal(SIGINT, sigint_handler);
    init_udp("10.10.10.99", 5000);
    sender_thread = std::thread(sender_worker);

    if (!face_cascade.load("/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml")) {
        std::cerr << "Failed to load cascade\n";
        return 1;
    }

    std::unique_ptr<CameraManager> cm = std::make_unique<CameraManager>();
    cm->start();

    auto cams = cm->cameras();
    if (cams.empty()) {
        std::cerr << "No cameras\n";
        return 1;
    }

    camera = cm->get(cams[0]->id());
    if (!camera) {
        std::cerr << "Cannot get camera\n";
        return 1;
    }

    camera->acquire();

    auto cfg = camera->generateConfiguration({StreamRole::VideoRecording});
    StreamConfiguration &sc = cfg->at(0);
    sc.size.width  = 320;
    sc.size.height = 240;
    sc.pixelFormat = formats::NV12;
    cfg->validate();
    camera->configure(cfg.get());

    ControlList ctrl;
    ctrl.set(controls::AeEnable, true);
    ctrl.set(controls::AwbEnable, true);
    ctrl.set(controls::Saturation, 1.0f);
    ctrl.set(controls::Sharpness, 1.5f);
    ctrl.set(controls::draft::NoiseReductionMode,
             controls::draft::NoiseReductionModeOff);

    FrameBufferAllocator alloc(camera);
    for (auto &s : *cfg)
        alloc.allocate(s.stream());

    camera->start(&ctrl);

    Stream *stream = sc.stream();
    const auto &bufs = alloc.buffers(stream);
    std::vector<std::unique_ptr<Request>> reqs;

    for (unsigned i = 0; i < bufs.size(); ++i) {
        auto r = camera->createRequest();
        r->addBuffer(stream, bufs[i].get());
        reqs.push_back(std::move(r));
    }

    camera->requestCompleted.connect(requestComplete);
    for (auto &r : reqs)
        camera->queueRequest(r.get());

    std::cout << "Streaming H.264 + face detection to 10.10.10.99:5000  (Ctrl-C to stop)\n";

    while (running)
        std::this_thread::sleep_for(100ms);

    /* ---- cleanup ---- */
    camera->stop();
    alloc.free(stream);
    camera->release();
    cm->stop();

    close(udp_sock);
    running = false;
    queue_cv.notify_all();
    sender_thread.join();

    return 0;
}
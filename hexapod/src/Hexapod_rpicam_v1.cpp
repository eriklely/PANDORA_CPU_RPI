// Hexapod_rpicam.cpp
#include "Hexapod.h"
#include "Hex_Cfg.h"
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
#include <fstream>
#include <libgen.h>
#include <limits.h>
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
/* OpenCV DNN */
static cv::dnn::Net face_net;
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
/* Resolve path to a data file: try cwd-relative and executable-relative paths. */
static std::string resolve_data_path(const std::string& filename) {
    auto file_exists = [](const std::string& p) {
        std::ifstream f(p);
        return f.good();
    };
    /* CWD-relative (run from hexapod/ or from hexapod/build/) */
    const std::string candidates[] = {
        "src/" + filename,
        "../src/" + filename,
    };
    for (const auto& p : candidates) {
        if (file_exists(p))
            return p;
    }
#ifdef __linux__
    /* Executable-relative (e.g. /home/pi/code/.../hexapod/build/hexapod -> .../hexapod/src/) */
    char exe_path[PATH_MAX];
    ssize_t n = readlink("/proc/self/exe", exe_path, sizeof(exe_path) - 1);
    if (n > 0) {
        exe_path[n] = '\0';
        std::string exe_dir(dirname(exe_path));
        std::string exe_rel[] = {
            exe_dir + "/../src/" + filename,
            exe_dir + "/src/" + filename,
        };
        for (const auto& p : exe_rel) {
            if (file_exists(p))
                return p;
        }
    }
#endif
    return "src/" + filename; /* fallback for clearer error from OpenCV */
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
        const int w = VIDEO_STREAM_WIDTH, h = VIDEO_STREAM_HEIGHT;
        cv::Mat nv12(h * 3 / 2, w, CV_8UC1);
        memcpy(nv12.data,          y,  ys);
        memcpy(nv12.data + ys,    uv, uvs);
        cv::Mat bgr;
        cv::cvtColor(nv12, bgr, cv::COLOR_YUV2BGR_NV12);
        /* ---- face detection with DNN (when Recognition Mode enabled) ---- */
        double max_area = 0.0;
        int centroid_x = 0, centroid_y = 0;
        if (g_fFaceDetectionEnabled) {
            std::vector<cv::Rect> faces;
            cv::Mat blob = cv::dnn::blobFromImage(bgr, 1.0, cv::Size(300, 300), cv::Scalar(104.0, 177.0, 123.0), false, false);
            face_net.setInput(blob);
            cv::Mat detection = face_net.forward();
            cv::Mat detectionMat(detection.size[2], detection.size[3], CV_32F, detection.ptr<float>());
            for (int i = 0; i < detectionMat.rows; i++) {
                float confidence = detectionMat.at<float>(i, 2);
                if (confidence < FACE_CONFIDENCE_THRESHOLD)
                    continue;
                int x1 = static_cast<int>(detectionMat.at<float>(i, 3) * w);
                int y1 = static_cast<int>(detectionMat.at<float>(i, 4) * h);
                int x2 = static_cast<int>(detectionMat.at<float>(i, 5) * w);
                int y2 = static_cast<int>(detectionMat.at<float>(i, 6) * h);
                int fw = x2 - x1, fh = y2 - y1;
                if (fw >= FACE_MIN_PIXELS && fh >= FACE_MIN_PIXELS)
                    faces.push_back(cv::Rect(x1, y1, fw, fh));
            }
            for (const auto &f : faces)
                cv::rectangle(bgr, f, cv::Scalar(0, 255, 0), 2);
            cv::Rect max_face;
            if (!faces.empty()) {
                for (const auto &f : faces) {
                    double area = f.width * f.height;
                    if (area > max_area) {
                        max_area = area;
                        max_face = f;
                    }
                }
                centroid_x = max_face.x + max_face.width / 2;
                centroid_y = max_face.y + max_face.height / 2;
                int cross_size = 5;
                cv::line(bgr, cv::Point(centroid_x - cross_size, centroid_y), cv::Point(centroid_x + cross_size, centroid_y), cv::Scalar(0, 0, 255), 2);
                cv::line(bgr, cv::Point(centroid_x, centroid_y - cross_size), cv::Point(centroid_x, centroid_y + cross_size), cv::Scalar(0, 0, 255), 2);
                std::string text = "X: " + std::to_string(centroid_x) + " Y: " + std::to_string(centroid_y);
                cv::putText(bgr, text, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);
            }
        }
        /* Temporal filter: only report face after FACE_CONFIRM_FRAMES consecutive frames with stable position */
        {
            static int pending_cx = 0, pending_cy = 0;
            static int confirm_count = 0;
            const int stable_dist = (w + h) / 16;
            bool have_detection = (max_area > 0);
            if (have_detection) {
                if (confirm_count > 0 && std::abs(centroid_x - pending_cx) <= stable_dist && std::abs(centroid_y - pending_cy) <= stable_dist) {
                    ++confirm_count;
                    pending_cx = centroid_x;
                    pending_cy = centroid_y;
                } else {
                    pending_cx = centroid_x;
                    pending_cy = centroid_y;
                    confirm_count = 1;
                }
            } else {
                confirm_count = 0;
            }
            bool report_face = have_detection && confirm_count >= FACE_CONFIRM_FRAMES;
            std::lock_guard<std::mutex> lock(face_mutex);
            if (g_fFaceDetectionEnabled) {
                latest_faces.num_faces = report_face ? 1 : 0;
                latest_faces.centroid_x = report_face ? pending_cx : 0;
                latest_faces.centroid_y = report_face ? pending_cy : 0;
            } else {
                latest_faces.num_faces = 0;
                latest_faces.centroid_x = 0;
                latest_faces.centroid_y = 0;
            }
        }
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
            cctx->bit_rate  = 8'000'000;
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
int hexapod_rpicam_main(int argc, char** argv)
{
    signal(SIGINT, sigint_handler);
    init_udp(VIDEO_STREAM_HOST, VIDEO_STREAM_PORT);
    sender_thread = std::thread(sender_worker);
    // Load DNN face detection model (paths resolved relative to cwd or executable)
    std::string prototxt = resolve_data_path("deploy.prototxt.txt");
    std::string caffemodel = resolve_data_path("res10_300x300_ssd_iter_140000.caffemodel");
    face_net = cv::dnn::readNetFromCaffe(prototxt, caffemodel);
    if (face_net.empty()) {
        std::cerr << "Failed to load DNN model. Tried prototxt=" << prototxt
                  << " caffemodel=" << caffemodel
                  << " (run from hexapod dir or ensure files exist in src/)\n";
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
    sc.size.width  = VIDEO_STREAM_WIDTH;
    sc.size.height = VIDEO_STREAM_HEIGHT;
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
    std::cout << "Hexapod camera streaming to " << VIDEO_STREAM_HOST << ":" << VIDEO_STREAM_PORT
              << " " << VIDEO_STREAM_WIDTH << "x" << VIDEO_STREAM_HEIGHT << "@" << VIDEO_STREAM_FPS << " fps (Ctrl-C to stop)\n";
    const int frame_ms = (VIDEO_STREAM_FPS >= 1) ? (1000 / VIDEO_STREAM_FPS) : 33;
    while (running)
        std::this_thread::sleep_for(std::chrono::milliseconds(frame_ms));
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
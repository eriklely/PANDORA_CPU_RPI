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
#include <mutex>  // ADD THIS for std::mutex + lock_guard

// Shared face data (thread-safe)
struct FaceData {
    int num_faces = 0;  // Number of detected faces
    int centroid_x = 0; // Average X position (for turning left/right)
    int centroid_y = 0; // Average Y position (for tilting up/down)
};

FaceData latest_faces;  // Global latest data
std::mutex face_mutex;  // Protects updates/reads

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
static cv::CascadeClassifier profile_cascade;  // NEW: For side profiles

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

        std::vector<cv::Rect> frontal_faces, profile_faces, faces;
        // Tune here for probability:   
        // - Higher sensitivity (more detections): scaleFactor=1.05, minNeighbors=3, minSize=cv::Size(20,20)
        // - Lower sensitivity (fewer, reliable): scaleFactor=1.3, minNeighbors=8, minSize=cv::Size(50,50)
        // - Balanced (current): scaleFactor=1.1, minNeighbors=5, minSize=cv::Size(30,30)
        face_cascade.detectMultiScale(gray, frontal_faces, 1.1, 5, 0, cv::Size(30, 30));

        // Detect profiles/sides (tuned stricter to reduce noise)
        profile_cascade.detectMultiScale(gray, profile_faces, 1.05, 3, 0, cv::Size(20, 20));

        // Merge: Append profiles to frontal list
        faces.insert(faces.end(), frontal_faces.begin(), frontal_faces.end());
        faces.insert(faces.end(), profile_faces.begin(), profile_faces.end());        

        // Optional: Draw on all (green for frontal, blue for profiles)
        for (const auto &f : frontal_faces)
            cv::rectangle(bgr, f, cv::Scalar(0, 255, 0), 2);  // Green
        for (const auto &f : profile_faces)
            cv::rectangle(bgr, f, cv::Scalar(255, 0, 0), 2);  // Blue

        // NEW: Select only the LARGEST face (max area) from merged list
        cv::Rect max_face;  // Will hold the selected face
        double max_area = 0.0;
        int centroid_x = 0, centroid_y = 0;  // Local vars for drawing
        if (!faces.empty()) {
            for (const auto &f : faces) {
                double area = f.width * f.height;
                if (area > max_area) {
                    max_area = area;
                    max_face = f;
                }
            }
            
            // REMOVED: No red bounding box, only the cross and text
            
            // Calculate centroid for drawing (same as below)
            centroid_x = max_face.x + max_face.width / 2;
            centroid_y = max_face.y + max_face.height / 2;
            
            // Optional: Draw a small red cross at the centroid
            int cross_size = 5;
            cv::line(bgr, cv::Point(centroid_x - cross_size, centroid_y), cv::Point(centroid_x + cross_size, centroid_y), cv::Scalar(0, 0, 255), 2);
            cv::line(bgr, cv::Point(centroid_x, centroid_y - cross_size), cv::Point(centroid_x, centroid_y + cross_size), cv::Scalar(0, 0, 255), 2);
            
            // Overlay centroid text on the frame (white, top-left; adjust position/color as needed)
            std::string text = "X: " + std::to_string(centroid_x) + " Y: " + std::to_string(centroid_y);
            cv::putText(bgr, text, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);
        }

        {
            std::lock_guard<std::mutex> lock(face_mutex);
            latest_faces.num_faces = (max_area > 0) ? 1 : 0;  // Report 1 if we have a max face, else 0

            if (max_area > 0) {
                // Use ONLY the max face's centroid (reuse locals)
                latest_faces.centroid_x = centroid_x;
                latest_faces.centroid_y = centroid_y;
            } else {
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
int hexapod_rpicam_main(int argc, char** argv)
{
    signal(SIGINT, sigint_handler);
    init_udp("10.10.10.4", 5000);
    sender_thread = std::thread(sender_worker);

    if (!face_cascade.load("/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml")) {
        std::cerr << "Failed to load cascade\n";
        return 1;
    }

    if (!profile_cascade.load("/usr/share/opencv4/haarcascades/haarcascade_profileface.xml")) {
        std::cerr << "Failed to load profile cascade\n";
        return 1;  // Or continue with warning if optional
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

    std::cout << "Hexapod camera streaming to 10.10.10.4:5000 (Ctrl-C to stop)\n";

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
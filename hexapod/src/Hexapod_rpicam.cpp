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
static std::atomic<bool> running{true};
/* UDP packet queue + sender thread */
static std::queue<std::vector<uint8_t>> packet_queue;
static std::mutex queue_mutex;
static std::condition_variable queue_cv;
static std::thread sender_thread;
static int udp_sock = -1;
static struct sockaddr_in dest_addr{};
/* OpenCV DNN */
static cv::dnn::Net face_net;
static bool face_net_loaded = false;
/* DNN worker thread — inference runs here, never in the libcamera callback */
static cv::Mat             dnn_input;
static bool                dnn_frame_ready = false;
static std::mutex          dnn_mx;
static std::condition_variable dnn_cv;
static std::thread         dnn_thread;
/* DNN overlay results — written by dnn_worker, read by requestComplete */
static std::vector<cv::Rect> dnn_overlay_faces;
static int                 dnn_cx = 0, dnn_cy = 0;
static std::mutex          dnn_result_mx;
/* H.264 encoder context — file-scope so hexapod_rpicam_main can flush on exit */
static AVCodecContext *g_cctx = nullptr;
static AVPacket       *g_pkt  = nullptr;
/* ---------- helper functions ---------- */
/* hexapod_rpicam_stop: called from main cleanup to signal this thread */
void hexapod_rpicam_stop() {
    running = false;
    queue_cv.notify_all();
    dnn_cv.notify_all();   // wake dnn_worker so it can exit
}
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
static void dnn_worker() {
    // Limit DNN to 2 threads — leaves 2 cores free for hexapod loop + servo serial
    cv::setNumThreads(2);
    face_net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    face_net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);

    double smooth_cx = 0, smooth_cy = 0;
    bool first_det = true;
    auto last_inference = std::chrono::steady_clock::now() - std::chrono::milliseconds(1000);

    while (running) {
        cv::Mat frame;
        {
            std::unique_lock<std::mutex> lk(dnn_mx);
            dnn_cv.wait(lk, [] { return dnn_frame_ready || !running; });
            if (!running) break;
            frame = std::move(dnn_input);
            dnn_frame_ready = false;
        }
        // Rate-limit: run DNN at most once per 250 ms to avoid saturating the CPU
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_inference).count() < 250) {
            continue;
        }
        last_inference = now;
        if (!face_net_loaded || !g_fFaceDetectionEnabled) {
            first_det = true;
            std::lock_guard<std::mutex> rlk(dnn_result_mx);
            dnn_overlay_faces.clear();
            dnn_cx = dnn_cy = 0;
            { std::lock_guard<std::mutex> flk(face_mutex); latest_faces = {}; }
            continue;
        }
        const int w = VIDEO_STREAM_WIDTH, h = VIDEO_STREAM_HEIGHT;
        cv::Mat blob = cv::dnn::blobFromImage(frame, 1.0, cv::Size(300,300),
                       cv::Scalar(104.0,177.0,123.0), false, false);
        face_net.setInput(blob);
        cv::Mat det = face_net.forward();
        cv::Mat detMat(det.size[2], det.size[3], CV_32F, det.ptr<float>());
        std::vector<cv::Rect> faces;
        double max_area = 0; cv::Rect max_face;
        for (int i = 0; i < detMat.rows; i++) {
            float conf = detMat.at<float>(i, 2);
            if (conf < FACE_CONFIDENCE_THRESHOLD) continue;
            int x1 = std::max(0, std::min((int)(detMat.at<float>(i,3)*w), w-1));
            int y1 = std::max(0, std::min((int)(detMat.at<float>(i,4)*h), h-1));
            int x2 = std::max(0, std::min((int)(detMat.at<float>(i,5)*w), w));
            int y2 = std::max(0, std::min((int)(detMat.at<float>(i,6)*h), h));
            int fw = x2-x1, fh = y2-y1;
            if (fw >= FACE_MIN_PIXELS && fh >= FACE_MIN_PIXELS) {
                faces.push_back(cv::Rect(x1,y1,fw,fh));
                double area = (double)fw*fh;
                if (area > max_area) { max_area = area; max_face = faces.back(); }
            }
        }
        int cx = 0, cy = 0;
        if (!faces.empty()) {
            cx = max_face.x + max_face.width/2;
            cy = max_face.y + max_face.height/2;
            if (first_det) { smooth_cx = cx; smooth_cy = cy; first_det = false; }
            else {
                smooth_cx = FACE_TRACKING_EMA_ALPHA*cx + (1.0-FACE_TRACKING_EMA_ALPHA)*smooth_cx;
                smooth_cy = FACE_TRACKING_EMA_ALPHA*cy + (1.0-FACE_TRACKING_EMA_ALPHA)*smooth_cy;
            }
            cx = (int)smooth_cx; cy = (int)smooth_cy;
        } else {
            first_det = true;
        }
        { std::lock_guard<std::mutex> rlk(dnn_result_mx);
          dnn_overlay_faces = faces; dnn_cx = cx; dnn_cy = cy; }
        { std::lock_guard<std::mutex> flk(face_mutex);
          latest_faces.num_faces = faces.empty() ? 0 : 1;
          latest_faces.centroid_x = cx; latest_faces.centroid_y = cy; }
    }
}
/* Resolve path to a data file: try several locations in priority order.
   Set HEXAPOD_DATA_DIR env var to override all search paths. */
static std::string resolve_data_path(const std::string& filename) {
    auto file_exists = [](const std::string& p) {
        std::ifstream f(p);
        return f.good();
    };
    /* 1. Environment variable override */
    if (const char* env = std::getenv("HEXAPOD_DATA_DIR")) {
        std::string p = std::string(env) + "/" + filename;
        if (file_exists(p)) return p;
    }
    /* 2. CWD-relative: bare filename (run from the same dir as models),
          src/ prefix (run from parent of src/), ../src/ (run from build/ subdir) */
    const std::string cwd_candidates[] = {
        filename,
        "src/" + filename,
        "../src/" + filename,
    };
    for (const auto& p : cwd_candidates) {
        if (file_exists(p)) return p;
    }
#ifdef __linux__
    /* 3. Executable-relative: same dir as binary, then src/ and ../src/ siblings */
    char exe_path[PATH_MAX];
    ssize_t n = readlink("/proc/self/exe", exe_path, sizeof(exe_path) - 1);
    if (n > 0) {
        exe_path[n] = '\0';
        std::string exe_dir(dirname(exe_path));
        const std::string exe_candidates[] = {
            exe_dir + "/" + filename,
            exe_dir + "/src/" + filename,
            exe_dir + "/../src/" + filename,
        };
        for (const auto& p : exe_candidates) {
            if (file_exists(p)) return p;
        }
    }
    /* 4. Known project location relative to $HOME */
    if (const char* home = std::getenv("HOME")) {
        std::string p = std::string(home) + "/code/Pandora/Cpu/hexapod_cpu_pandora/hexapod/src/" + filename;
        if (file_exists(p)) return p;
    }
#endif
    std::cerr << "resolve_data_path: '" << filename
              << "' not found; set HEXAPOD_DATA_DIR to its directory\n";
    return filename; /* last-resort: let OpenCV produce the error with the bare name */
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
        size_t   uvs = planes[1].length;  // UV plane size (NV12 contiguous with Y, used for size guard)
        if (ys == 0 || uvs == 0) {
            munmap(addr, aligned);
            request->reuse(Request::ReuseBuffers);
            camera->queueRequest(request);
            return;
        }
        /* ---- NV12 to BGR (OpenCV) ---- */
        const int w = VIDEO_STREAM_WIDTH, h = VIDEO_STREAM_HEIGHT;
        // OPT #2: Zero-copy NV12 — wrap mmap'd buffer directly (no memcpy)
        // This works when uv == y + ys (contiguous NV12 planes, standard for Pi cameras)
        cv::Mat nv12(h * 3 / 2, w, CV_8UC1, y);
        // OPT #3: Pre-allocate BGR Mat once (reused across frames)
        static cv::Mat bgr(h, w, CV_8UC3);
        cv::cvtColor(nv12, bgr, cv::COLOR_YUV2BGR_NV12);
        /* ---- submit frame to DNN worker (non-blocking; drop frame if worker busy) ---- */
        if (g_fFaceDetectionEnabled && face_net_loaded) {
            std::unique_lock<std::mutex> lk(dnn_mx, std::try_to_lock);
            if (lk.owns_lock() && !dnn_frame_ready) {
                dnn_input = bgr.clone();
                dnn_frame_ready = true;
                dnn_cv.notify_one();
            }
        }
        /* ---- overlay last DNN results on current frame (from previous inference) ---- */
        {
            std::lock_guard<std::mutex> rlk(dnn_result_mx);
            for (const auto &f : dnn_overlay_faces)
                cv::rectangle(bgr, f, cv::Scalar(0,255,0), 2);
            if (!dnn_overlay_faces.empty()) {
                cv::line(bgr, cv::Point(dnn_cx-5, dnn_cy), cv::Point(dnn_cx+5, dnn_cy), cv::Scalar(0,0,255), 2);
                cv::line(bgr, cv::Point(dnn_cx, dnn_cy-5), cv::Point(dnn_cx, dnn_cy+5), cv::Scalar(0,0,255), 2);
                std::string txt = "X:"+std::to_string(dnn_cx)+" Y:"+std::to_string(dnn_cy);
                cv::putText(bgr, txt, cv::Point(10,30), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255,255,255), 2);
            }
        }
        /* ---- BGR to YUV420P for x264 ---- */
        static SwsContext *sws = nullptr;
        static bool sws_init = false;
        if (!sws_init) {
            sws = sws_getContext(w, h, AV_PIX_FMT_BGR24,
                                 w, h, AV_PIX_FMT_YUV420P,
                                 SWS_FAST_BILINEAR, nullptr, nullptr, nullptr);
            sws_init = true;
        }
        // OPT #1: Pre-allocate AVFrame once (reused across frames, no per-frame malloc/free)
        static AVFrame *yuv = nullptr;
        static bool yuv_init = false;
        if (!yuv_init) {
            yuv = av_frame_alloc();
            yuv->format = AV_PIX_FMT_YUV420P;
            yuv->width  = w;
            yuv->height = h;
            av_frame_get_buffer(yuv, 0);
            yuv_init = true;
        }
        yuv->linesize[0] = w;
        yuv->linesize[1] = yuv->linesize[2] = w / 2;
        uint8_t *bgr_src[4] = { bgr.data, nullptr, nullptr, nullptr };
        int      bgr_ls [4] = { (int)bgr.step, 0, 0, 0 };
        sws_scale(sws, bgr_src, bgr_ls, 0, h,
                  yuv->data, yuv->linesize);
        yuv->pts = meta.sequence;
        /* ---- H.264 encode (I-frame every frame) ---- */
        static bool ff_init = false;
        if (!ff_init) {
            const AVCodec *codec = avcodec_find_encoder(AV_CODEC_ID_H264);
            g_cctx = avcodec_alloc_context3(codec);
            g_cctx->width     = w;
            g_cctx->height    = h;
            g_cctx->pix_fmt   = AV_PIX_FMT_YUV420P;
            g_cctx->time_base = {1, 30};
            g_cctx->bit_rate  = 8'000'000;
            g_cctx->gop_size  = 1;
            av_opt_set(g_cctx->priv_data, "preset", "ultrafast", 0);
            av_opt_set(g_cctx->priv_data, "tune",   "zerolatency", 0);
            if (avcodec_open2(g_cctx, codec, nullptr) < 0) {
                std::cerr << "Failed to open H264 codec\n";
                return;
            }
            g_pkt = av_packet_alloc();
            ff_init = true;
        }
        avcodec_send_frame(g_cctx, yuv);
        while (avcodec_receive_packet(g_cctx, g_pkt) == 0) {
            // OPT #4: Reserve capacity to reduce vector reallocation
            std::vector<uint8_t> data;
            data.reserve(g_pkt->size);
            data.assign(g_pkt->data, g_pkt->data + g_pkt->size);
            {
                std::lock_guard<std::mutex> lk(queue_mutex);
                packet_queue.push(std::move(data));
            }
            queue_cv.notify_one();
            av_packet_unref(g_pkt);
        }
        // OPT #1: AVFrame is pre-allocated and reused, no av_frame_free needed
        munmap(addr, aligned);
    }
    request->reuse(Request::ReuseBuffers);
    camera->queueRequest(request);
}
/* ---------- main ---------- */
int hexapod_rpicam_main(int argc, char** argv)
{
    // SIGINT is handled by Main.cpp's sigaction; hexapod_rpicam_stop() signals us to exit
    (void)argc; (void)argv;
    init_udp(VIDEO_STREAM_HOST, VIDEO_STREAM_PORT);
    sender_thread = std::thread(sender_worker);
    dnn_thread    = std::thread(dnn_worker);
    // RAII guard: join both worker threads on ALL exit paths
    struct WorkerGuard {
        ~WorkerGuard() {
            running = false;
            queue_cv.notify_all();
            dnn_cv.notify_all();
            if (sender_thread.joinable()) sender_thread.join();
            if (dnn_thread.joinable())    dnn_thread.join();
        }
    } worker_guard;

    // Load DNN face detection model — non-fatal: continue without face detection if missing
    std::string prototxt    = resolve_data_path("deploy.prototxt.txt");
    std::string caffemodel  = resolve_data_path("res10_300x300_ssd_iter_140000.caffemodel");
    try {
        face_net = cv::dnn::readNetFromCaffe(prototxt, caffemodel);
        if (face_net.empty())
            std::cerr << "DNN model loaded but empty — face detection disabled\n";
        else {
            face_net_loaded = true;
            std::cout << "DNN face detection model loaded\n";
        }
    } catch (const cv::Exception& e) {
        std::cerr << "DNN load failed (" << e.what() << ") — face detection disabled\n";
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
    /* ---- flush encoder before stopping camera ---- */
    if (g_cctx && g_pkt) {
        avcodec_send_frame(g_cctx, nullptr);  // signal end-of-stream
        while (avcodec_receive_packet(g_cctx, g_pkt) == 0) {
            std::vector<uint8_t> data(g_pkt->data, g_pkt->data + g_pkt->size);
            {
                std::lock_guard<std::mutex> lk(queue_mutex);
                packet_queue.push(std::move(data));
            }
            queue_cv.notify_one();
            av_packet_unref(g_pkt);
        }
    }
    /* ---- cleanup ---- */
    camera->stop();
    alloc.free(stream);
    camera->release();
    cm->stop();
    close(udp_sock);
    return 0;  // WorkerGuard destructor joins sender_thread and dnn_thread
}
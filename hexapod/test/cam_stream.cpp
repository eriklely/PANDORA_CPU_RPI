#include <iomanip>
#include <iostream>
#include <memory>
#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <signal.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <sys/mman.h>  // For mmap/munmap
#include <cstring>     // For memcpy/strerror

#include <libcamera/libcamera.h>
#include <libcamera/controls.h>  // For controls and draft namespace

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>  // For av_opt_set
#include <libswscale/swscale.h>
}

using namespace libcamera;
using namespace std::chrono_literals;

static std::shared_ptr<Camera> camera;
static volatile bool running = true;
static int udp_sock = -1;
static struct sockaddr_in dest_addr;

// Thread-safe queue for NV12 frames (y_plane, uv_plane, sizes, stride, pts)
struct FrameData {
    uint8_t *y_data;
    uint8_t *uv_data;
    size_t y_size, uv_size;
    int y_stride, uv_stride;
    int64_t pts;
    int width, height;
};
std::queue<FrameData> frame_queue;
std::mutex queue_mutex;
std::condition_variable queue_cv;
std::thread encode_thread;

// FFmpeg globals
static AVCodecContext *cctx = nullptr;
static AVFrame *frame = nullptr;
static AVPacket *pkt = nullptr;
static SwsContext *sws_ctx = nullptr;
static AVFrame *in_frame = nullptr;

static void sigint_handler(int) { running = false; }

static void init_udp(const char* host, int port) {
    udp_sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_sock < 0) { std::cerr << "UDP socket failed" << std::endl; return; }
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(port);
    inet_pton(AF_INET, host, &dest_addr.sin_addr);
}

static void send_udp(const uint8_t* data, size_t size) {
    if (udp_sock >= 0) sendto(udp_sock, data, size, 0, (struct sockaddr*)&dest_addr, sizeof(dest_addr));
}

static void cleanup_ffmpeg() {
    if (cctx) avcodec_free_context(&cctx);
    if (frame) av_frame_free(&frame);
    if (pkt) av_packet_free(&pkt);
    if (sws_ctx) sws_freeContext(sws_ctx);
    if (in_frame) av_frame_free(&in_frame);
}

// Worker thread: Dequeue, encode, send
static void encode_worker() {
    while (running) {
        std::unique_lock<std::mutex> lock(queue_mutex);
        queue_cv.wait(lock, [&] { return !frame_queue.empty() || !running; });
        if (!running) break;

        FrameData fd = frame_queue.front();
        frame_queue.pop();
        lock.unlock();

        // FFmpeg init (once)
        static bool ff_init = false;
        static const AVCodec *codec = nullptr;
        if (!ff_init) {
            codec = avcodec_find_encoder(AV_CODEC_ID_H264);
            if (!codec) { std::cerr << "No H.264 codec" << std::endl; return; }
            cctx = avcodec_alloc_context3(codec);
            cctx->width = fd.width;
            cctx->height = fd.height;
            cctx->pix_fmt = AV_PIX_FMT_YUV420P;
            cctx->time_base = {1, 30};
            cctx->bit_rate = 4000000;
            cctx->gop_size = 30;
            av_opt_set(cctx->priv_data, "preset", "ultrafast", AV_OPT_SEARCH_CHILDREN);
            av_opt_set(cctx->priv_data, "tune", "zerolatency", AV_OPT_SEARCH_CHILDREN);
            av_opt_set(cctx->priv_data, "keyint", "30", AV_OPT_SEARCH_CHILDREN);
            if (avcodec_open2(cctx, codec, nullptr) < 0) { std::cerr << "FFmpeg open failed" << std::endl; return; }

            frame = av_frame_alloc();
            frame->format = AV_PIX_FMT_YUV420P;
            frame->width = fd.width;
            frame->height = fd.height;
            av_frame_get_buffer(frame, 0);

            pkt = av_packet_alloc();

            in_frame = av_frame_alloc();
            in_frame->format = AV_PIX_FMT_NV12;
            in_frame->width = fd.width;
            in_frame->height = fd.height;
            av_frame_get_buffer(in_frame, 0);
            in_frame->linesize[0] = fd.y_stride;
            in_frame->linesize[1] = fd.uv_stride;

            sws_ctx = sws_getContext(fd.width, fd.height, AV_PIX_FMT_NV12, fd.width, fd.height, AV_PIX_FMT_YUV420P,
                                     SWS_BILINEAR, nullptr, nullptr, nullptr);
            ff_init = true;
        }

        // Expected sizes
        size_t expected_y = fd.width * fd.height;
        size_t expected_uv = expected_y / 2;

        // Row-by-row copy with stride
        for (int row = 0; row < fd.height; ++row) {
            memcpy(in_frame->data[0] + row * in_frame->linesize[0], fd.y_data + row * fd.y_stride, fd.width);
        }
        for (int row = 0; row < fd.height / 2; ++row) {
            memcpy(in_frame->data[1] + row * in_frame->linesize[1], fd.uv_data + row * fd.uv_stride, fd.width);
        }

        // Convert
        sws_scale(sws_ctx, in_frame->data, in_frame->linesize, 0, fd.height,
                  frame->data, frame->linesize);

        frame->pts = fd.pts;
        if (avcodec_send_frame(cctx, frame) < 0) {
            std::cerr << "Send frame failed" << std::endl;
        } else {
            while (avcodec_receive_packet(cctx, pkt) == 0) {
                send_udp(pkt->data, pkt->size);
                av_packet_unref(pkt);
            }
        }

        av_frame_unref(in_frame);

        // Free
        delete[] fd.y_data;
        delete[] fd.uv_data;
    }
}

static void requestComplete(Request *request) {
    if (request->status() == Request::RequestCancelled || !running) return;

    const std::map<const Stream *, FrameBuffer *> &buffers = request->buffers();
    for (auto bufferPair : buffers) {
        FrameBuffer *buffer = bufferPair.second;
        const FrameMetadata &metadata = buffer->metadata();

        const auto &planes = buffer->planes();
        if (planes.size() < 2) {
            request->reuse(Request::ReuseBuffers);
            camera->queueRequest(request);
            return;
        }

        // mmap
        size_t total_size = planes[1].offset + planes[1].length;
        long page_size = sysconf(_SC_PAGESIZE);
        if (page_size < 1) page_size = 4096;
        size_t aligned_size = (total_size + page_size - 1) & ~(page_size - 1);

        void *addr = mmap(nullptr, aligned_size, PROT_READ, MAP_SHARED, planes[0].fd.get(), 0);
        if (addr == MAP_FAILED) {
            std::cerr << "mmap failed: " << strerror(errno) << std::endl;
            request->reuse(Request::ReuseBuffers);
            camera->queueRequest(request);
            return;
        }

        // Planes
        uint8_t *y_plane = static_cast<uint8_t *>(addr) + planes[0].offset;
        size_t y_size = planes[0].length;
        uint8_t *uv_plane = static_cast<uint8_t *>(addr) + planes[1].offset;
        size_t uv_size = planes[1].length;

        std::cout << "Frame " << metadata.sequence << ": Y=" << y_size << ", UV=" << uv_size << std::endl;

        if (y_size == 0 || uv_size == 0) {
            munmap(addr, aligned_size);
            request->reuse(Request::ReuseBuffers);
            camera->queueRequest(request);
            return;
        }

        // Assume stride = width for Pi NV12 (no Plane.stride; fix if padded)
        int width = 1920;  // From config
        int height = 1080;
        int y_stride = width;
        int uv_stride = width;

        // Copy full planes to heap
        FrameData fd;
        fd.y_data = new uint8_t[y_size];
        fd.uv_data = new uint8_t[uv_size];
        memcpy(fd.y_data, y_plane, y_size);
        memcpy(fd.uv_data, uv_plane, uv_size);
        fd.y_size = y_size;
        fd.uv_size = uv_size;
        fd.y_stride = y_stride;
        fd.uv_stride = uv_stride;
        fd.pts = metadata.sequence;
        fd.width = width;
        fd.height = height;

        {
            std::lock_guard<std::mutex> lock(queue_mutex);
            frame_queue.push(std::move(fd));
        }
        queue_cv.notify_one();

        munmap(addr, aligned_size);
        request->reuse(Request::ReuseBuffers);
        camera->queueRequest(request);
    }
}

int main(int argc, char *argv[]) {
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <host> <port>\n";
        return 1;
    }
    init_udp(argv[1], std::stoi(argv[2]));
    signal(SIGINT, sigint_handler);
    atexit(cleanup_ffmpeg);

    // Start encode thread
    encode_thread = std::thread(encode_worker);

    std::unique_ptr<CameraManager> cm = std::make_unique<CameraManager>();
    cm->start();

    auto cameras = cm->cameras();
    if (cameras.empty()) {
        std::cerr << "No cameras available\n";
        running = false;
        queue_cv.notify_all();
        encode_thread.join();
        return 1;
    }

    std::string cameraId = cameras[0]->id();
    camera = cm->get(cameraId);
    if (!camera) {
        std::cerr << "Failed to acquire camera\n";
        running = false;
        queue_cv.notify_all();
        encode_thread.join();
        return 1;
    }

    camera->acquire();

    std::unique_ptr<CameraConfiguration> config = camera->generateConfiguration({ StreamRole::VideoRecording });
    if (!config) {
        std::cerr << "Failed to generate config\n";
        running = false;
        queue_cv.notify_all();
        encode_thread.join();
        return 1;
    }

    StreamConfiguration &streamConfig = config->at(0);
    streamConfig.size.width = 1920;
    streamConfig.size.height = 1080;
    streamConfig.pixelFormat = formats::NV12;

    if (config->validate() == CameraConfiguration::Invalid) {
        std::cerr << "Invalid config\n";
        running = false;
        queue_cv.notify_all();
        encode_thread.join();
        return 1;
    }
    camera->configure(config.get());

    // Controls
    ControlList controls;
    controls.set(controls::AeEnable, true);
    controls.set(controls::AwbEnable, true);
    controls.set(controls::Saturation, 1.0);
    controls.set(controls::Sharpness, 1.5);
    controls.set(libcamera::controls::draft::NoiseReductionMode, libcamera::controls::draft::NoiseReductionModeOff);

    FrameBufferAllocator *allocator = new FrameBufferAllocator(camera);
    for (StreamConfiguration &cfg : *config) {
        if (allocator->allocate(cfg.stream()) < 0) {
            std::cerr << "Buffer alloc failed\n";
            delete allocator;
            running = false;
            queue_cv.notify_all();
            encode_thread.join();
            return 1;
        }
    }

    Stream *stream = streamConfig.stream();
    const std::vector<std::unique_ptr<FrameBuffer>> &buffers = allocator->buffers(stream);
    std::vector<std::unique_ptr<Request>> requests;

    for (unsigned int i = 0; i < buffers.size(); ++i) {
        std::unique_ptr<Request> request = camera->createRequest();
        if (!request || request->addBuffer(stream, buffers[i].get()) < 0) {
            std::cerr << "Request creation failed\n";
            delete allocator;
            running = false;
            queue_cv.notify_all();
            encode_thread.join();
            return 1;
        }
        requests.push_back(std::move(request));
    }

    camera->requestCompleted.connect(requestComplete);
    if (camera->start(&controls) < 0) {
        std::cerr << "Camera start failed\n";
        delete allocator;
        running = false;
        queue_cv.notify_all();
        encode_thread.join();
        return 1;
    }

    for (std::unique_ptr<Request> &request : requests) {
        camera->queueRequest(request.get());
    }

    std::cout << "Streaming 1080p@30fps H.264 to " << argv[1] << ":" << argv[2] << "... Ctrl+C to stop.\n";
    while (running) {
        std::this_thread::sleep_for(100ms);
    }

    camera->stop();
    allocator->free(stream);
    delete allocator;
    camera->release();
    cm->stop();
    close(udp_sock);

    // Join thread and cleanup queue
    running = false;
    queue_cv.notify_all();
    encode_thread.join();
    cleanup_ffmpeg();
    return 0;
}

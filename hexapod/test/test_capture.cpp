#include <libcamera/libcamera.h>
#include <libcamera/controls.h>
#include <iostream>
#include <memory>
#include <thread>
#include <signal.h>
#include <sys/mman.h>
#include <cstring>

using namespace libcamera;
using namespace std::chrono_literals;

static std::shared_ptr<Camera> camera;
static volatile bool running = true;

static void sigint_handler(int) { running = false; }

static void requestComplete(Request *request) {
    if (request->status() == Request::RequestCancelled || !running) return;

    const std::map<const Stream *, FrameBuffer *> &buffers = request->buffers();
    for (auto bufferPair : buffers) {
        FrameBuffer *buffer = bufferPair.second;
        const FrameMetadata &metadata = buffer->metadata();

        const auto &planes = buffer->planes();
        if (planes.size() < 2) return;

        // mmap
        size_t total_size = planes[1].offset + planes[1].length;
        long page_size = sysconf(_SC_PAGESIZE);
        size_t aligned_size = (total_size + page_size - 1) & ~(page_size - 1);

        void *addr = mmap(nullptr, aligned_size, PROT_READ, MAP_SHARED, planes[0].fd.get(), 0);
        if (addr == MAP_FAILED) {
            std::cerr << "mmap failed" << std::endl;
            request->reuse(Request::ReuseBuffers);
            camera->queueRequest(request);
            return;
        }

        uint8_t *y_plane = static_cast<uint8_t *>(addr) + planes[0].offset;
        size_t y_size = planes[0].length;
        uint8_t *uv_plane = static_cast<uint8_t *>(addr) + planes[1].offset;
        size_t uv_size = planes[1].length;

        std::cout << "Frame " << metadata.sequence << ": Y=" << y_size << ", UV=" << uv_size << std::endl;

        munmap(addr, aligned_size);
        request->reuse(Request::ReuseBuffers);
        camera->queueRequest(request);
    }
}

int main() {
    signal(SIGINT, sigint_handler);

    std::unique_ptr<CameraManager> cm = std::make_unique<CameraManager>();
    cm->start();

    auto cameras = cm->cameras();
    if (cameras.empty()) {
        std::cerr << "No cameras" << std::endl;
        return 1;
    }

    std::string cameraId = cameras[0]->id();
    camera = cm->get(cameraId);
    if (!camera) {
        std::cerr << "Failed to get camera" << std::endl;
        return 1;
    }

    camera->acquire();

    std::unique_ptr<CameraConfiguration> config = camera->generateConfiguration({ StreamRole::VideoRecording });
    if (!config) {
        std::cerr << "Failed config" << std::endl;
        return 1;
    }

    StreamConfiguration &streamConfig = config->at(0);
    streamConfig.size.width = 1920;
    streamConfig.size.height = 1080;
    streamConfig.pixelFormat = formats::NV12;

    if (config->validate() == CameraConfiguration::Invalid) {
        std::cerr << "Invalid config" << std::endl;
        return 1;
    }
    camera->configure(config.get());

    ControlList controls;
    controls.set(controls::AeEnable, true);
    controls.set(controls::AwbEnable, true);
    camera->start(&controls);

    FrameBufferAllocator allocator(camera.get());
    for (StreamConfiguration &cfg : *config) {
        if (allocator.allocate(cfg.stream()) < 0) {
            std::cerr << "Alloc fail" << std::endl;
            return 1;
        }
    }

    Stream *stream = streamConfig.stream();
    const std::vector<std::unique_ptr<FrameBuffer>> &buffers = allocator.buffers(stream);
    std::vector<std::unique_ptr<Request>> requests;

    for (unsigned int i = 0; i < buffers.size(); ++i) {
        std::unique_ptr<Request> request = camera->createRequest();
        if (!request || request->addBuffer(stream, buffers[i].get()) < 0) {
            std::cerr << "Request fail" << std::endl;
            return 1;
        }
        requests.push_back(std::move(request));
    }

    camera->requestCompleted.connect(requestComplete);
    for (std::unique_ptr<Request> &request : requests) {
        camera->queueRequest(request.get());
    }

    std::cout << "Capturing frames for 10s (Ctrl+C to stop early)" << std::endl;
    std::this_thread::sleep_for(10s);

    camera->stop();
    allocator.free(stream);
    camera->release();
    cm->stop();
    return 0;
}

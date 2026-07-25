#include <unitree/robot/go2/video/video_client.hpp>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

namespace {

std::atomic<bool> stop_requested{false};

void handle_signal(int)
{
    stop_requested.store(true);
}

bool is_jpeg(const std::vector<uint8_t>& image)
{
    return image.size() >= 4 && image[0] == 0xff && image[1] == 0xd8 &&
           image[2] == 0xff;
}

bool fetch_jpeg(unitree::robot::go2::VideoClient& client,
                std::vector<uint8_t>& image,
                int attempts)
{
    for (int attempt = 1; attempt <= attempts && !stop_requested.load(); ++attempt) {
        image.clear();
        const int32_t result = client.GetImageSample(image);
        if (result == 0 && is_jpeg(image)) {
            return true;
        }
        std::cerr << "BUILTIN_CAMERA_FETCH_FAILED attempt=" << attempt
                  << " result=" << result << " bytes=" << image.size() << std::endl;
        if (attempt < attempts) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    return false;
}

int save_snapshot(unitree::robot::go2::VideoClient& client, const std::string& output)
{
    std::vector<uint8_t> image;
    if (!fetch_jpeg(client, image, 5)) {
        return 3;
    }

    std::ofstream stream(output, std::ios::binary | std::ios::trunc);
    if (!stream) {
        std::cerr << "BUILTIN_CAMERA_OPEN_FAILED output=" << output << std::endl;
        return 4;
    }
    stream.write(reinterpret_cast<const char*>(image.data()), image.size());
    if (!stream) {
        std::cerr << "BUILTIN_CAMERA_WRITE_FAILED output=" << output << std::endl;
        return 5;
    }
    std::cout << "BUILTIN_CAMERA_SNAPSHOT_OK bytes=" << image.size()
              << " output=" << output << std::endl;
    return 0;
}

int stream_mjpeg(unitree::robot::go2::VideoClient& client, double seconds, double fps)
{
    if (seconds <= 0.0 || fps <= 0.0 || fps > 30.0) {
        std::cerr << "duration and fps must be positive; fps must be <= 30" << std::endl;
        return 2;
    }

    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);
    std::signal(SIGPIPE, handle_signal);
    std::setvbuf(stdout, nullptr, _IOFBF, 1024 * 1024);

    const auto started = std::chrono::steady_clock::now();
    const auto deadline = started + std::chrono::duration<double>(seconds);
    const auto frame_period = std::chrono::duration<double>(1.0 / fps);
    uint64_t frames = 0;
    uint64_t errors = 0;

    while (!stop_requested.load() && std::chrono::steady_clock::now() < deadline) {
        const auto target = started + frame_period * static_cast<double>(frames + errors);
        std::vector<uint8_t> image;
        if (fetch_jpeg(client, image, 2)) {
            const std::size_t written = std::fwrite(image.data(), 1, image.size(), stdout);
            if (written != image.size() || std::fflush(stdout) != 0) {
                std::cerr << "BUILTIN_CAMERA_PIPE_CLOSED frames=" << frames << std::endl;
                return frames > 0 ? 0 : 6;
            }
            ++frames;
        } else {
            ++errors;
        }

        const auto next_target = target + frame_period;
        if (next_target > std::chrono::steady_clock::now()) {
            std::this_thread::sleep_until(next_target);
        }
    }

    const double elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - started).count();
    std::cerr << "BUILTIN_CAMERA_STREAM_DONE frames=" << frames
              << " errors=" << errors << " elapsed=" << elapsed
              << " fps=" << (elapsed > 0.0 ? frames / elapsed : 0.0) << std::endl;
    return frames > 0 ? 0 : 7;
}

void usage(const char* program)
{
    std::cerr << "usage: " << program
              << " probe [interface] | snapshot <output.jpg> [interface]"
                 " | stream <seconds> <fps> [interface]"
              << std::endl;
}

}  // namespace

int run(int argc, char** argv)
{
    if (argc < 2) {
        usage(argv[0]);
        return 2;
    }

    const std::string mode = argv[1];
    std::string interface = "eth0";
    if (mode == "probe" && argc >= 3) {
        interface = argv[2];
    } else if (mode == "snapshot" && argc >= 4) {
        interface = argv[3];
    } else if (mode == "stream" && argc >= 5) {
        interface = argv[4];
    }

    unitree::robot::ChannelFactory::Instance()->Init(0, interface);
    unitree::robot::go2::VideoClient client;
    client.SetTimeout(3.0f);
    client.Init();

    if (mode == "probe") {
        std::vector<uint8_t> image;
        if (!fetch_jpeg(client, image, 5)) {
            return 3;
        }
        std::cout << "BUILTIN_CAMERA_PROBE_OK interface=" << interface
                  << " bytes=" << image.size() << " encoding=jpeg" << std::endl;
        return 0;
    }
    if (mode == "snapshot" && argc >= 3) {
        return save_snapshot(client, argv[2]);
    }
    if (mode == "stream" && argc >= 4) {
        return stream_mjpeg(client, std::stod(argv[2]), std::stod(argv[3]));
    }

    usage(argv[0]);
    return 2;
}

int main(int argc, char** argv)
{
    try {
        return run(argc, argv);
    } catch (const std::exception& error) {
        std::cerr << "BUILTIN_CAMERA_FATAL error=" << error.what() << std::endl;
        return 8;
    } catch (...) {
        std::cerr << "BUILTIN_CAMERA_FATAL error=unknown_exception" << std::endl;
        return 9;
    }
}

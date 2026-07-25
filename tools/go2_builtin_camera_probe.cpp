#include <unitree/robot/go2/video/video_client.hpp>

#include <chrono>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <set>
#include <string>
#include <thread>
#include <vector>

namespace {

uint64_t fingerprint(const std::vector<uint8_t>& bytes)
{
    uint64_t value = 1469598103934665603ULL;
    for (uint8_t byte : bytes) {
        value ^= byte;
        value *= 1099511628211ULL;
    }
    return value;
}

}  // namespace

int main(int argc, char** argv)
{
    const std::string interface = argc > 1 ? argv[1] : "eth0";
    const std::string output = argc > 2 ? argv[2] : "/tmp/go2_builtin_camera.jpg";
    const int requested_samples = argc > 3 ? std::stoi(argv[3]) : 1;
    const int interval_ms = argc > 4 ? std::stoi(argv[4]) : 0;

    unitree::robot::ChannelFactory::Instance()->Init(0, interface);
    unitree::robot::go2::VideoClient video_client;
    video_client.SetTimeout(3.0f);
    video_client.Init();

    std::vector<uint8_t> image;
    std::set<uint64_t> fingerprints;
    int successful_samples = 0;
    const auto started = std::chrono::steady_clock::now();
    for (int index = 0; index < requested_samples; ++index) {
        std::vector<uint8_t> current_image;
        const auto call_started = std::chrono::steady_clock::now();
        const int32_t result = video_client.GetImageSample(current_image);
        const double call_seconds = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - call_started).count();
        std::cout << "sample=" << (index + 1) << " result=" << result
                  << " bytes=" << current_image.size()
                  << " call_seconds=" << call_seconds << std::endl;
        if (result == 0 && current_image.size() >= 4 &&
            current_image[0] == 0xff && current_image[1] == 0xd8 &&
            current_image[2] == 0xff) {
            image = std::move(current_image);
            fingerprints.insert(fingerprint(image));
            ++successful_samples;
        }
        if (interval_ms > 0 && index + 1 < requested_samples) {
            std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms));
        }
    }

    const double elapsed_seconds = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - started).count();
    std::cout << "summary requested=" << requested_samples
              << " successful=" << successful_samples
              << " unique=" << fingerprints.size()
              << " elapsed_seconds=" << elapsed_seconds
              << " effective_fps=" << successful_samples / elapsed_seconds
              << " interface=" << interface << std::endl;
    if (image.empty()) {
        return 2;
    }

    std::ofstream stream(output, std::ios::binary);
    if (!stream) {
        std::cerr << "cannot open output: " << output << std::endl;
        return 4;
    }
    stream.write(reinterpret_cast<const char*>(image.data()), image.size());
    if (!stream) {
        std::cerr << "cannot write output: " << output << std::endl;
        return 5;
    }
    std::cout << "saved=" << output << std::endl;
    return 0;
}

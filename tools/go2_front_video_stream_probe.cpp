#include <unitree/idl/go2/Go2FrontVideoData_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace {

using VideoMessage = unitree_go::msg::dds_::Go2FrontVideoData_;

std::atomic<uint64_t> sample_count{0};
std::atomic<uint64_t> first_timestamp{0};
std::atomic<uint64_t> last_timestamp{0};
std::chrono::steady_clock::time_point first_arrival;
std::chrono::steady_clock::time_point last_arrival;

std::string prefix(const std::vector<uint8_t>& bytes)
{
    std::ostringstream out;
    const std::size_t length = std::min<std::size_t>(bytes.size(), 12);
    for (std::size_t index = 0; index < length; ++index) {
        if (index != 0) {
            out << ' ';
        }
        out << std::hex << std::setfill('0') << std::setw(2)
            << static_cast<unsigned int>(bytes[index]);
    }
    return out.str();
}

std::string encoding(const std::vector<uint8_t>& bytes)
{
    if (bytes.size() >= 3 && bytes[0] == 0xff && bytes[1] == 0xd8 && bytes[2] == 0xff) {
        return "JPEG";
    }
    if (bytes.size() >= 4 && bytes[0] == 0x00 && bytes[1] == 0x00 &&
        ((bytes[2] == 0x00 && bytes[3] == 0x01) || bytes[2] == 0x01)) {
        return "H26x-AnnexB";
    }
    if (bytes.empty()) {
        return "empty";
    }
    return "unknown";
}

void handle_video(const void* raw_message)
{
    const auto& message = *static_cast<const VideoMessage*>(raw_message);
    const uint64_t count = sample_count.fetch_add(1) + 1;
    const auto now = std::chrono::steady_clock::now();
    if (count == 1) {
        first_timestamp.store(message.time_frame());
        first_arrival = now;
    }
    last_timestamp.store(message.time_frame());
    last_arrival = now;

    if (count <= 5 || count % 30 == 0) {
        const auto print_stream = [count, &message](const char* name,
                                                    const std::vector<uint8_t>& bytes) {
            std::cout << "sample=" << count
                      << " time_frame=" << message.time_frame()
                      << " stream=" << name
                      << " bytes=" << bytes.size()
                      << " encoding=" << encoding(bytes)
                      << " prefix=" << prefix(bytes) << std::endl;
        };
        print_stream("720p", message.video720p());
        print_stream("360p", message.video360p());
        print_stream("180p", message.video180p());
    }
}

}  // namespace

int main(int argc, char** argv)
{
    const std::string interface = argc > 1 ? argv[1] : "eth0";
    const int duration_seconds = argc > 2 ? std::stoi(argv[2]) : 8;

    unitree::robot::ChannelFactory::Instance()->Init(0, interface);
    auto subscriber = std::make_shared<unitree::robot::ChannelSubscriber<VideoMessage>>(
        "rt/frontvideostream");
    subscriber->InitChannel(handle_video, 1);

    std::cout << "listening topic=rt/frontvideostream interface=" << interface
              << " duration_seconds=" << duration_seconds << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(duration_seconds));
    subscriber->CloseChannel();

    const uint64_t count = sample_count.load();
    std::cout << "summary samples=" << count;
    if (count > 1) {
        const double elapsed =
            std::chrono::duration<double>(last_arrival - first_arrival).count();
        std::cout << " elapsed_seconds=" << elapsed
                  << " arrival_fps=" << static_cast<double>(count - 1) / elapsed
                  << " first_timestamp=" << first_timestamp.load()
                  << " last_timestamp=" << last_timestamp.load();
    }
    std::cout << std::endl;
    return count > 0 ? 0 : 2;
}

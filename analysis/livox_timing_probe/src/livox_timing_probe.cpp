#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>
#include <thread>
#include <vector>

#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "rclcpp/rclcpp.hpp"

namespace {

double percentile(std::vector<double> values, double p) {
  if (values.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  std::sort(values.begin(), values.end());
  const double index = (values.size() - 1) * p;
  const auto lower = static_cast<std::size_t>(std::floor(index));
  const auto upper = std::min(lower + 1, values.size() - 1);
  return values[lower] + (values[upper] - values[lower]) * (index - lower);
}

void print_summary(const char *name, const std::vector<double> &values) {
  if (values.empty()) {
    std::cout << name << " count=0\n";
    return;
  }
  const double sum = std::accumulate(values.begin(), values.end(), 0.0);
  const auto limits = std::minmax_element(values.begin(), values.end());
  std::cout << name
            << " count=" << values.size()
            << " min=" << *limits.first
            << " mean=" << sum / values.size()
            << " p50=" << percentile(values, 0.50)
            << " p95=" << percentile(values, 0.95)
            << " max=" << *limits.second << '\n';
}

class TimingProbe : public rclcpp::Node {
 public:
  TimingProbe() : Node("codex_livox_timing_probe") {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
    subscription_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
        "/livox/lidar", qos,
        [this](livox_ros_driver2::msg::CustomMsg::UniquePtr msg) {
          const auto now = std::chrono::system_clock::now();
          const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
              now.time_since_epoch()).count();
          const std::uint64_t frame_end_ns =
              msg->timebase +
              (msg->points.empty() ? 0 : msg->points.back().offset_time);

          ages_ms_.push_back(
              static_cast<double>(now_ns - static_cast<std::int64_t>(frame_end_ns)) /
              1e6);
          point_counts_.push_back(msg->points.size());

          if (last_frame_end_ns_ != 0) {
            sensor_gaps_ms_.push_back(
                static_cast<double>(frame_end_ns - last_frame_end_ns_) / 1e6);
          }
          const auto steady_now = std::chrono::steady_clock::now();
          if (received_ != 0) {
            receive_gaps_ms_.push_back(
                std::chrono::duration<double, std::milli>(
                    steady_now - last_receive_time_).count());
          } else {
            first_receive_time_ = steady_now;
          }
          last_receive_time_ = steady_now;
          last_frame_end_ns_ = frame_end_ns;
          ++received_;
        });
  }

  bool has_received() const { return received_ != 0; }

  double elapsed_seconds() const {
    if (!has_received()) {
      return 0.0;
    }
    return std::chrono::duration<double>(
        last_receive_time_ - first_receive_time_).count();
  }

  void report() const {
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "messages=" << received_
              << " elapsed_s=" << elapsed_seconds()
              << " effective_hz="
              << (elapsed_seconds() > 0.0
                      ? static_cast<double>(received_ - 1) / elapsed_seconds()
                      : 0.0)
              << '\n';
    print_summary("frame_end_age_ms", ages_ms_);
    print_summary("sensor_stamp_gap_ms", sensor_gaps_ms_);
    print_summary("local_receive_gap_ms", receive_gaps_ms_);
    if (!point_counts_.empty()) {
      const auto limits =
          std::minmax_element(point_counts_.begin(), point_counts_.end());
      std::cout << "point_count min=" << *limits.first
                << " max=" << *limits.second << '\n';
    }
  }

 private:
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr subscription_;
  std::vector<double> ages_ms_;
  std::vector<double> sensor_gaps_ms_;
  std::vector<double> receive_gaps_ms_;
  std::vector<std::size_t> point_counts_;
  std::uint64_t last_frame_end_ns_ = 0;
  std::size_t received_ = 0;
  std::chrono::steady_clock::time_point first_receive_time_;
  std::chrono::steady_clock::time_point last_receive_time_;
};

}  // namespace

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TimingProbe>();
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::seconds(15);
  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  node->report();
  rclcpp::shutdown();
  return 0;
}

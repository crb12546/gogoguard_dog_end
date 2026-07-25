#include <cassert>
#include <cstdint>

#include "comm/no_sync_timestamp_mapper.h"

int main() {
  constexpr uint64_t kDeviceAnchor = 10'000'000'000ULL;
  constexpr uint64_t kRosAnchor = 1'780'000'000'000'000'000ULL;

  livox_ros::NoSyncTimestampMapper mapper;
  assert(mapper.Map(kDeviceAnchor, kRosAnchor) == kRosAnchor);

  // Host callback delay must not change sensor elapsed time.
  assert(mapper.Map(kDeviceAnchor + 480'000ULL, kRosAnchor + 50'000'000ULL) ==
         kRosAnchor + 480'000ULL);
  assert(mapper.Map(kDeviceAnchor + 960'000ULL, kRosAnchor + 2'000'000ULL) ==
         kRosAnchor + 960'000ULL);

  // A small IMU/point-packet reordering keeps the same time base.
  assert(mapper.Map(kDeviceAnchor + 720'000ULL, kRosAnchor + 3'000'000ULL) ==
         kRosAnchor + 720'000ULL);

  // A lidar reboot re-anchors uptime to the current ROS epoch.
  constexpr uint64_t kBeforeRestart = kDeviceAnchor + 2'000'000'000ULL;
  mapper.Map(kBeforeRestart, kRosAnchor + 2'000'000'000ULL);
  constexpr uint64_t kRestartDeviceTime = 100'000'000ULL;
  constexpr uint64_t kRestartRosTime = kRosAnchor + 2'100'000'000ULL;
  assert(mapper.Map(kRestartDeviceTime, kRestartRosTime) == kRestartRosTime);
  assert(mapper.Map(kRestartDeviceTime + 480'000ULL, kRestartRosTime + 9'000'000ULL) ==
         kRestartRosTime + 480'000ULL);

  return 0;
}

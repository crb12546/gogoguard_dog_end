// Copyright 2026 Go2 Robotics Team
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <algorithm>
#include <cstdint>
#include <stdexcept>

namespace go2_map_localizer
{

// Monotonic-time gate requiring an explicit reset and a dwell before a runtime
// map replacement. Wall/ROS clock changes cannot shorten the quarantine.
class MapLoadQuarantine
{
public:
  explicit MapLoadQuarantine(std::int64_t minimum_dwell_ns)
  : minimum_dwell_ns_(minimum_dwell_ns)
  {
    if (minimum_dwell_ns_ <= 0) {
      throw std::invalid_argument("invalid map-load quarantine");
    }
  }

  void arm(std::int64_t now_ns)
  {
    armed_ = true;
    armed_at_ns_ = now_ns;
  }

  void disarm()
  {
    armed_ = false;
    armed_at_ns_ = 0;
  }

  bool armed() const {return armed_;}

  bool ready(std::int64_t now_ns) const
  {
    return armed_ && now_ns >= armed_at_ns_ &&
           now_ns - armed_at_ns_ >= minimum_dwell_ns_;
  }

  double remainingSeconds(std::int64_t now_ns) const
  {
    if (!armed_) {
      return 0.0;
    }
    if (now_ns < armed_at_ns_) {
      return static_cast<double>(minimum_dwell_ns_) * 1.0e-9;
    }
    return static_cast<double>(
      std::max<std::int64_t>(
        0, minimum_dwell_ns_ - (now_ns - armed_at_ns_))) * 1.0e-9;
  }

private:
  std::int64_t minimum_dwell_ns_;
  bool armed_{false};
  std::int64_t armed_at_ns_{0};
};

}  // namespace go2_map_localizer

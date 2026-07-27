// Copyright 2026 Go2 Robotics Team
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <algorithm>
#include <cstdint>
#include <stdexcept>

namespace go2_map_localizer
{

// A ROS-independent gate for promoting a provisional global pose. A start
// match counts as the first observation. Later observations must be strictly
// time-ordered; any rejection resets the consecutive streak while retaining
// provisional mode.
class ConfirmationGate
{
public:
  ConfirmationGate(std::uint32_t required_accepts, std::int64_t minimum_span_ns)
  : required_accepts_(required_accepts), minimum_span_ns_(minimum_span_ns)
  {
    if (required_accepts_ < 2 || minimum_span_ns_ <= 0) {
      throw std::invalid_argument("invalid confirmation gate configuration");
    }
  }

  void start(std::int64_t stamp_ns)
  {
    pending_ = true;
    accepts_ = 1;
    first_stamp_ns_ = stamp_ns;
    last_stamp_ns_ = stamp_ns;
  }

  void reject()
  {
    if (pending_) {
      accepts_ = 0;
      first_stamp_ns_ = 0;
      last_stamp_ns_ = 0;
    }
  }

  bool accept(std::int64_t stamp_ns)
  {
    if (!pending_) {
      return true;
    }
    if (accepts_ == 0) {
      accepts_ = 1;
      first_stamp_ns_ = stamp_ns;
      last_stamp_ns_ = stamp_ns;
      return false;
    }
    if (stamp_ns <= last_stamp_ns_) {
      reject();
      return false;
    }
    ++accepts_;
    last_stamp_ns_ = stamp_ns;
    if (accepts_ >= required_accepts_ &&
      last_stamp_ns_ - first_stamp_ns_ >= minimum_span_ns_)
    {
      pending_ = false;
      return true;
    }
    return false;
  }

  void reset()
  {
    pending_ = false;
    accepts_ = 0;
    first_stamp_ns_ = 0;
    last_stamp_ns_ = 0;
  }

  bool pending() const {return pending_;}
  std::uint32_t accepts() const {return accepts_;}
  double spanSeconds() const
  {
    if (accepts_ < 2) {
      return 0.0;
    }
    return static_cast<double>(std::max<std::int64_t>(
      0, last_stamp_ns_ - first_stamp_ns_)) * 1.0e-9;
  }

private:
  std::uint32_t required_accepts_;
  std::int64_t minimum_span_ns_;
  bool pending_{false};
  std::uint32_t accepts_{0};
  std::int64_t first_stamp_ns_{0};
  std::int64_t last_stamp_ns_{0};
};

}  // namespace go2_map_localizer

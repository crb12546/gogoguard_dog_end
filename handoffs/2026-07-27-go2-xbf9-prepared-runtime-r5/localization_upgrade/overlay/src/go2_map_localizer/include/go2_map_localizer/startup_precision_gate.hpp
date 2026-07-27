// Copyright 2026 Go2 Robotics Team
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>

namespace go2_map_localizer
{

struct StartupPrecisionPose
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double yaw{0.0};
};

// A ROS-independent, pre-motion repeatability gate. Every observation is
// compared with the first anchor, never with the preceding observation.
// Passing this gate proves only that repeated global solutions agree; it does
// not establish their absolute accuracy in the physical world.
class StartupPrecisionGate
{
public:
  StartupPrecisionGate(
    double maximum_translation_deviation_m,
    double maximum_yaw_deviation_rad)
  : maximum_translation_deviation_m_(maximum_translation_deviation_m),
    maximum_yaw_deviation_rad_(maximum_yaw_deviation_rad)
  {
    if (!std::isfinite(maximum_translation_deviation_m_) ||
      !std::isfinite(maximum_yaw_deviation_rad_) ||
      maximum_translation_deviation_m_ <= 0.0 ||
      maximum_yaw_deviation_rad_ <= 0.0 ||
      maximum_yaw_deviation_rad_ > kPi)
    {
      throw std::invalid_argument("invalid startup precision gate configuration");
    }
  }

  void start(const StartupPrecisionPose & anchor)
  {
    if (!finite(anchor)) {
      throw std::invalid_argument("startup precision anchor is non-finite");
    }
    anchor_ = anchor;
    active_ = true;
    verified_ = false;
    violated_ = false;
    observations_ = 1;
    maximum_observed_translation_deviation_m_ = 0.0;
    maximum_observed_yaw_deviation_rad_ = 0.0;
  }

  bool observe(const StartupPrecisionPose & candidate)
  {
    if (verified_) {
      return true;
    }
    if (!active_) {
      return false;
    }
    if (!finite(candidate)) {
      active_ = false;
      violated_ = true;
      maximum_observed_translation_deviation_m_ =
        std::numeric_limits<double>::infinity();
      maximum_observed_yaw_deviation_rad_ =
        std::numeric_limits<double>::infinity();
      return false;
    }

    const double dx = candidate.x - anchor_.x;
    const double dy = candidate.y - anchor_.y;
    const double dz = candidate.z - anchor_.z;
    const double translation_deviation = std::sqrt(dx * dx + dy * dy + dz * dz);
    const double yaw_deviation = std::abs(wrapAngle(candidate.yaw - anchor_.yaw));
    maximum_observed_translation_deviation_m_ = std::max(
      maximum_observed_translation_deviation_m_, translation_deviation);
    maximum_observed_yaw_deviation_rad_ = std::max(
      maximum_observed_yaw_deviation_rad_, yaw_deviation);

    if (translation_deviation > maximum_translation_deviation_m_ ||
      yaw_deviation > maximum_yaw_deviation_rad_)
    {
      active_ = false;
      violated_ = true;
      return false;
    }
    ++observations_;
    return true;
  }

  bool verify()
  {
    if (!active_ || violated_ || observations_ < 2) {
      return false;
    }
    active_ = false;
    verified_ = true;
    return true;
  }

  void reset()
  {
    anchor_ = StartupPrecisionPose{};
    active_ = false;
    verified_ = false;
    violated_ = false;
    observations_ = 0;
    maximum_observed_translation_deviation_m_ = 0.0;
    maximum_observed_yaw_deviation_rad_ = 0.0;
  }

  bool active() const {return active_;}
  bool verified() const {return verified_;}
  bool violated() const {return violated_;}
  std::uint32_t observations() const {return observations_;}
  double maximumTranslationDeviation() const
  {
    return maximum_observed_translation_deviation_m_;
  }
  double maximumYawDeviation() const
  {
    return maximum_observed_yaw_deviation_rad_;
  }
  double translationThreshold() const {return maximum_translation_deviation_m_;}
  double yawThreshold() const {return maximum_yaw_deviation_rad_;}

private:
  static constexpr double kPi = 3.14159265358979323846;

  static bool finite(const StartupPrecisionPose & pose)
  {
    return std::isfinite(pose.x) && std::isfinite(pose.y) &&
           std::isfinite(pose.z) && std::isfinite(pose.yaw);
  }

  static double wrapAngle(double angle)
  {
    return std::atan2(std::sin(angle), std::cos(angle));
  }

  double maximum_translation_deviation_m_;
  double maximum_yaw_deviation_rad_;
  StartupPrecisionPose anchor_;
  bool active_{false};
  bool verified_{false};
  bool violated_{false};
  std::uint32_t observations_{0};
  double maximum_observed_translation_deviation_m_{0.0};
  double maximum_observed_yaw_deviation_rad_{0.0};
};

}  // namespace go2_map_localizer

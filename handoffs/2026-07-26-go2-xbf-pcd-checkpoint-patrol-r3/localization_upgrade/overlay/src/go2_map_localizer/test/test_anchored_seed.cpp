// Copyright 2026 Go2 Robotics Team
// SPDX-License-Identifier: Apache-2.0
#include "go2_map_localizer/anchored_seed.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace
{

constexpr double kPi = 3.14159265358979323846;

void require(bool condition, const char * message)
{
  if (!condition) {
    throw std::runtime_error(message);
  }
}

bool near(double first, double second)
{
  return std::abs(first - second) <= 1.0e-12;
}

}  // namespace

int main()
{
  const std::vector<double> xy{0.0, -1.0, 1.0, -2.0, 2.0};
  const std::vector<double> yaw{
    0.0,
    -10.0 * kPi / 180.0,
    10.0 * kPi / 180.0,
    -20.0 * kPi / 180.0,
    20.0 * kPi / 180.0,
    -30.0 * kPi / 180.0,
    30.0 * kPi / 180.0};

  const auto limited =
    go2_map_localizer::makeAnchoredSeeds(10.0, -4.0, 0.25, xy, yaw, 2.1, 36);
  require(limited.size() == 36, "anchored seed count did not honor the limit");
  require(
    near(limited.front().x, 10.0) &&
    near(limited.front().y, -4.0) &&
    near(limited.front().yaw, 0.25),
    "exact route anchor must be the first seed");
  require(
    std::any_of(
      limited.begin(), limited.end(),
      [](const auto & seed) {
        return near(seed.x, 10.0) && near(seed.y, -4.0) &&
               near(seed.yaw, 0.25 + 30.0 * kPi / 180.0);
      }),
    "limited seed set must retain the 30 degree pure yaw hypothesis");
  require(
    std::any_of(
      limited.begin(), limited.end(),
      [](const auto & seed) {
        return near(seed.x, 12.0) && near(seed.y, -4.0) &&
               near(seed.yaw, 0.25);
      }),
    "limited seed set must retain the two metre pure translation hypothesis");
  for (const auto & seed : limited) {
    require(
      std::hypot(seed.x - 10.0, seed.y + 4.0) <= 2.1 + 1.0e-12,
      "seed escaped search radius");
  }

  const auto radius_filtered =
    go2_map_localizer::makeAnchoredSeeds(0.0, 0.0, 0.0, xy, {0.0}, 1.1, 100);
  require(radius_filtered.size() == 5, "radius filter should retain center and four axes");

  const std::vector<double> confirmation_xy{0.0, -0.25, 0.25};
  const std::vector<double> confirmation_yaw{
    0.0, -2.0 * kPi / 180.0, 2.0 * kPi / 180.0};
  const auto confirmation = go2_map_localizer::makeAnchoredSeeds(
    3.0, 4.0, 0.1,
    confirmation_xy, confirmation_yaw, 0.40, 7);
  require(
    confirmation.size() == 7,
    "confirmation seed budget must contain one exact and six fallback hypotheses");
  require(
    near(confirmation.front().x, 3.0) &&
    near(confirmation.front().y, 4.0) &&
    near(confirmation.front().yaw, 0.1),
    "confirmation must try the provisional prediction first");
  for (const auto & seed : confirmation) {
    const double translation = std::hypot(seed.x - 3.0, seed.y - 4.0);
    const double yaw_delta = std::abs(
      std::atan2(std::sin(seed.yaw - 0.1), std::cos(seed.yaw - 0.1)));
    require(
      translation <= 1.0e-12 || yaw_delta <= 1.0e-12,
      "confirmation fallback budget must not include combined offsets");
  }

  const auto wrapped = go2_map_localizer::makeAnchoredSeeds(
    0.0, 0.0, 179.0 * kPi / 180.0, {0.0}, {0.0, 10.0 * kPi / 180.0}, 1.0, 2);
  require(wrapped.size() == 2, "yaw wrapping test produced the wrong count");
  require(
    wrapped[1].yaw < -170.0 * kPi / 180.0,
    "yaw seed was not normalized across pi");

  bool rejected_missing_zero = false;
  try {
    static_cast<void>(
      go2_map_localizer::makeAnchoredSeeds(
        0.0, 0.0, 0.0, {1.0}, {0.0}, 2.0, 4));
  } catch (const std::invalid_argument &) {
    rejected_missing_zero = true;
  }
  require(rejected_missing_zero, "offset arrays without zero must be rejected");

  bool rejected_non_finite = false;
  try {
    static_cast<void>(
      go2_map_localizer::makeAnchoredSeeds(
        0.0, 0.0, 0.0, {0.0}, {0.0, INFINITY}, 2.0, 4));
  } catch (const std::invalid_argument &) {
    rejected_non_finite = true;
  }
  require(rejected_non_finite, "non-finite seed offsets must be rejected");
  return 0;
}

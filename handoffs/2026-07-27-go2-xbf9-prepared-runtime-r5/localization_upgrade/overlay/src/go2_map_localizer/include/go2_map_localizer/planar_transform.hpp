// Copyright 2026 Go2 Robotics Team
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cmath>

namespace go2_map_localizer
{

struct PlanarTransform
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

inline double normalizeAngle(double angle)
{
  constexpr double pi = 3.14159265358979323846;
  while (angle > pi) {
    angle -= 2.0 * pi;
  }
  while (angle <= -pi) {
    angle += 2.0 * pi;
  }
  return angle;
}

inline PlanarTransform compose(
  const PlanarTransform & parent_from_middle,
  const PlanarTransform & middle_from_child)
{
  const double cosine = std::cos(parent_from_middle.yaw);
  const double sine = std::sin(parent_from_middle.yaw);
  return {
    parent_from_middle.x + cosine * middle_from_child.x - sine * middle_from_child.y,
    parent_from_middle.y + sine * middle_from_child.x + cosine * middle_from_child.y,
    normalizeAngle(parent_from_middle.yaw + middle_from_child.yaw)};
}

inline PlanarTransform inverse(const PlanarTransform & parent_from_child)
{
  const double cosine = std::cos(parent_from_child.yaw);
  const double sine = std::sin(parent_from_child.yaw);
  return {
    -cosine * parent_from_child.x - sine * parent_from_child.y,
    sine * parent_from_child.x - cosine * parent_from_child.y,
    normalizeAngle(-parent_from_child.yaw)};
}

}  // namespace go2_map_localizer

// Copyright 2026 Go2 Robotics Team
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>

namespace go2_map_localizer
{

struct RigidPoseDifference
{
  double translation_m{std::numeric_limits<double>::infinity()};
  double rotation_rad{std::numeric_limits<double>::infinity()};
};

// Compare two poses expressed in the same parent frame. Translation is the
// Euclidean distance between their origins in that frame; rotation is the angle
// of reference_rotation.transpose() * candidate_rotation. In particular, the
// translation result is invariant when both poses receive the same map offset.
inline RigidPoseDifference rigidPoseDifference(
  const std::array<double, 3> & reference_translation,
  const std::array<double, 9> & reference_rotation_row_major,
  const std::array<double, 3> & candidate_translation,
  const std::array<double, 9> & candidate_rotation_row_major)
{
  const auto all_finite = [](const auto & values) {
      return std::all_of(
        values.begin(), values.end(),
        [](double value) {return std::isfinite(value);});
    };
  if (!all_finite(reference_translation) ||
    !all_finite(reference_rotation_row_major) ||
    !all_finite(candidate_translation) ||
    !all_finite(candidate_rotation_row_major))
  {
    return {};
  }

  const double dx = candidate_translation[0] - reference_translation[0];
  const double dy = candidate_translation[1] - reference_translation[1];
  const double dz = candidate_translation[2] - reference_translation[2];

  // trace(R_reference^T * R_candidate) is the Frobenius inner product
  // of the two row-major rotation matrices.
  double relative_rotation_trace = 0.0;
  for (std::size_t index = 0; index < reference_rotation_row_major.size(); ++index) {
    relative_rotation_trace +=
      reference_rotation_row_major[index] * candidate_rotation_row_major[index];
  }
  const double rotation_cosine = std::clamp(
    (relative_rotation_trace - 1.0) * 0.5, -1.0, 1.0);

  return {
    std::sqrt(dx * dx + dy * dy + dz * dz),
    std::acos(rotation_cosine)};
}

}  // namespace go2_map_localizer

// Copyright 2026 Go2 Robotics Team
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <functional>
#include <limits>
#include <vector>

namespace go2_map_localizer
{

struct LandmarkEvidenceConfig
{
  std::size_t minimum_matches{40};
  std::size_t minimum_unique_voxels{16};
  double minimum_support_match_ratio{0.10};
  double minimum_major_spread_m{1.5};
};

struct LandmarkEvidenceResult
{
  bool accepted{false};
  std::size_t matches{0};
  std::size_t unique_voxels{0};
  double support_match_ratio{0.0};
  double major_spread_m{0.0};
  double minor_spread_m{0.0};
};

/**
 * Evaluate already-associated scan points against the reviewed stable layer.
 *
 * Counting unique XYZ voxels prevents one dense patch from masquerading as
 * object-level evidence. The largest 3-D axis span accepts both horizontal
 * structure (walls/curbs) and vertical structure (poles), while rejecting a
 * tiny accidental overlap. Cleaned-map registration still computes the pose;
 * this independent channel decides whether approved fixed structure supports
 * it.
 */
inline LandmarkEvidenceResult evaluateLandmarkEvidence(
  std::size_t support_point_count,
  std::size_t match_count,
  const std::vector<std::array<double, 3>> & unique_matched_xyz,
  const LandmarkEvidenceConfig & config)
{
  LandmarkEvidenceResult result;
  result.matches = match_count;
  result.unique_voxels = unique_matched_xyz.size();
  if (support_point_count == 0 ||
    config.minimum_matches == 0 ||
    config.minimum_unique_voxels == 0 ||
    !std::isfinite(config.minimum_support_match_ratio) ||
    !std::isfinite(config.minimum_major_spread_m) ||
    config.minimum_support_match_ratio <= 0.0 ||
    config.minimum_support_match_ratio > 1.0 ||
    config.minimum_major_spread_m <= 0.0)
  {
    return result;
  }
  result.support_match_ratio =
    static_cast<double>(match_count) /
    static_cast<double>(support_point_count);
  if (unique_matched_xyz.empty()) {
    return result;
  }
  std::array<double, 3> minimum{
    std::numeric_limits<double>::infinity(),
    std::numeric_limits<double>::infinity(),
    std::numeric_limits<double>::infinity()};
  std::array<double, 3> maximum{
    -std::numeric_limits<double>::infinity(),
    -std::numeric_limits<double>::infinity(),
    -std::numeric_limits<double>::infinity()};
  for (const auto & point : unique_matched_xyz) {
    if (!std::all_of(point.begin(), point.end(), [](double value) {
        return std::isfinite(value);
      }))
    {
      return result;
    }
    for (std::size_t axis = 0; axis < 3; ++axis) {
      minimum[axis] = std::min(minimum[axis], point[axis]);
      maximum[axis] = std::max(maximum[axis], point[axis]);
    }
  }
  std::array<double, 3> spans{
    maximum[0] - minimum[0],
    maximum[1] - minimum[1],
    maximum[2] - minimum[2]};
  std::sort(spans.begin(), spans.end(), std::greater<double>());
  result.major_spread_m = spans[0];
  result.minor_spread_m = spans[1];
  result.accepted =
    result.matches >= config.minimum_matches &&
    result.unique_voxels >= config.minimum_unique_voxels &&
    result.support_match_ratio >= config.minimum_support_match_ratio &&
    result.major_spread_m >= config.minimum_major_spread_m;
  return result;
}

}  // namespace go2_map_localizer

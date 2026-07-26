// Copyright 2026 Go2 Robotics Team
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <vector>

namespace go2_map_localizer
{

struct AnchoredSeed2
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

inline double normalizeSeedAngle(double angle)
{
  if (!std::isfinite(angle)) {
    throw std::invalid_argument("anchored seed angle must be finite");
  }
  return std::atan2(std::sin(angle), std::cos(angle));
}

inline std::vector<double> uniqueFiniteOffsets(
  const std::vector<double> & offsets,
  const char * label)
{
  if (offsets.empty()) {
    throw std::invalid_argument(std::string(label) + " must not be empty");
  }
  std::vector<double> result;
  result.reserve(offsets.size());
  for (const double offset : offsets) {
    if (!std::isfinite(offset)) {
      throw std::invalid_argument(std::string(label) + " must contain finite values");
    }
    const bool duplicate = std::any_of(
      result.begin(), result.end(),
      [offset](double existing) {
        return std::abs(existing - offset) <= 1.0e-12;
      });
    if (!duplicate) {
      result.push_back(offset);
    }
  }
  return result;
}

// Generate deterministic SE2 hypotheses around a reviewed route anchor.
//
// The first hypothesis is always the exact anchor when both offset arrays
// contain zero. Pure position and pure yaw hypotheses are retained first;
// combined changes then follow in normalized cost order. max_count is applied
// after search-radius filtering and de-duplication.
inline std::vector<AnchoredSeed2> makeAnchoredSeeds(
  double anchor_x,
  double anchor_y,
  double anchor_yaw,
  const std::vector<double> & xy_offsets_m,
  const std::vector<double> & yaw_offsets_rad,
  double search_radius_m,
  std::size_t max_count)
{
  if (!std::isfinite(anchor_x) || !std::isfinite(anchor_y) ||
    !std::isfinite(anchor_yaw) || !std::isfinite(search_radius_m) ||
    search_radius_m <= 0.0 || max_count == 0)
  {
    throw std::invalid_argument("invalid anchored seed request");
  }
  const auto xy_offsets = uniqueFiniteOffsets(xy_offsets_m, "xy_offsets_m");
  const auto yaw_offsets = uniqueFiniteOffsets(yaw_offsets_rad, "yaw_offsets_rad");
  if (std::none_of(
      xy_offsets.begin(), xy_offsets.end(),
      [](double value) {return std::abs(value) <= 1.0e-12;}) ||
    std::none_of(
      yaw_offsets.begin(), yaw_offsets.end(),
      [](double value) {return std::abs(value) <= 1.0e-12;}))
  {
    throw std::invalid_argument("anchored seed offsets must include zero");
  }

  double position_scale = search_radius_m;
  for (const double offset : xy_offsets) {
    if (std::abs(offset) > 1.0e-12) {
      position_scale = std::min(position_scale, std::abs(offset));
    }
  }
  double yaw_scale = 1.0;
  bool found_yaw_scale = false;
  for (const double offset : yaw_offsets) {
    if (std::abs(offset) > 1.0e-12) {
      yaw_scale = found_yaw_scale ?
        std::min(yaw_scale, std::abs(offset)) : std::abs(offset);
      found_yaw_scale = true;
    }
  }

  struct RankedSeed
  {
    AnchoredSeed2 seed;
    double cost{0.0};
    double translation{0.0};
    double yaw_offset{0.0};
    bool pure_axis{false};
    std::size_t order{0};
  };
  std::vector<RankedSeed> ranked;
  std::size_t order = 0;
  for (const double dx : xy_offsets) {
    for (const double dy : xy_offsets) {
      const double translation = std::hypot(dx, dy);
      if (translation > search_radius_m + 1.0e-12) {
        continue;
      }
      for (const double yaw_offset : yaw_offsets) {
        RankedSeed candidate;
        candidate.seed = {
          anchor_x + dx,
          anchor_y + dy,
          normalizeSeedAngle(anchor_yaw + yaw_offset)};
        candidate.translation = translation;
        candidate.yaw_offset = std::abs(yaw_offset);
        candidate.cost =
          translation / position_scale +
          (found_yaw_scale ? candidate.yaw_offset / yaw_scale : 0.0);
        candidate.pure_axis =
          translation <= 1.0e-12 || candidate.yaw_offset <= 1.0e-12;
        candidate.order = order++;
        ranked.push_back(candidate);
      }
    }
  }
  std::stable_sort(
    ranked.begin(), ranked.end(),
    [](const RankedSeed & lhs, const RankedSeed & rhs) {
      if (lhs.pure_axis != rhs.pure_axis) {
        return lhs.pure_axis;
      }
      if (std::abs(lhs.cost - rhs.cost) > 1.0e-12) {
        return lhs.cost < rhs.cost;
      }
      if (std::abs(lhs.translation - rhs.translation) > 1.0e-12) {
        return lhs.translation < rhs.translation;
      }
      if (std::abs(lhs.yaw_offset - rhs.yaw_offset) > 1.0e-12) {
        return lhs.yaw_offset < rhs.yaw_offset;
      }
      return lhs.order < rhs.order;
    });

  std::vector<AnchoredSeed2> result;
  result.reserve(std::min(max_count, ranked.size()));
  for (const auto & candidate : ranked) {
    result.push_back(candidate.seed);
    if (result.size() >= max_count) {
      break;
    }
  }
  return result;
}

}  // namespace go2_map_localizer

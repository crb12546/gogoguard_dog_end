// Copyright 2026 Go2 Robotics Team
// SPDX-License-Identifier: Apache-2.0
//
// Clean-room polar descriptor. This implementation was designed for this
// project from first principles and does not contain Scan Context source code.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <vector>

namespace go2_map_localizer
{

constexpr double kPi = 3.141592653589793238462643383279502884;

struct Point3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct PolarDescriptorConfig
{
  std::size_t rings{20};
  std::size_t sectors{60};
  double max_radius_m{40.0};
  double min_radius_m{0.0};
  double min_z_m{-2.0};
  double max_z_m{4.0};
};

struct PolarDescriptor
{
  PolarDescriptorConfig config;
  std::vector<float> values;
  std::vector<float> ring_key;
  std::vector<float> sector_key;
  std::size_t accepted_points{0};
  std::size_t occupied_cells{0};
};

struct DescriptorMatch
{
  double distance{std::numeric_limits<double>::infinity()};
  double yaw_rad{0.0};
  int sector_shift{0};
  double overlap_ratio{0.0};
};

inline void validateConfig(const PolarDescriptorConfig & config)
{
  const double scalar_parameters[]{
    config.max_radius_m, config.min_radius_m, config.min_z_m, config.max_z_m};
  if (!std::all_of(
      std::begin(scalar_parameters), std::end(scalar_parameters),
      [](double value) {return std::isfinite(value);}) ||
    config.rings == 0 || config.sectors < 4 ||
    config.rings > 1000 || config.sectors > 4096 ||
    config.max_radius_m <= 0.0 ||
    config.min_radius_m < 0.0 || config.min_radius_m >= config.max_radius_m ||
    config.min_z_m >= config.max_z_m)
  {
    throw std::invalid_argument("invalid polar descriptor configuration");
  }
}

inline PolarDescriptor buildPolarDescriptor(
  const std::vector<Point3> & points,
  const PolarDescriptorConfig & config)
{
  validateConfig(config);
  PolarDescriptor result;
  result.config = config;
  result.values.assign(config.rings * config.sectors, 0.0F);
  result.ring_key.assign(config.rings, 0.0F);
  result.sector_key.assign(config.sectors, 0.0F);
  const double radius_span = config.max_radius_m - config.min_radius_m;
  const double height_span = config.max_z_m - config.min_z_m;
  for (const auto & point : points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      continue;
    }
    const double radius = std::hypot(point.x, point.y);
    if (radius < config.min_radius_m || radius > config.max_radius_m)
    {
      continue;
    }

    const auto ring = std::min(
      config.rings - 1,
      static_cast<std::size_t>(
        std::floor(
          (radius - config.min_radius_m) / radius_span *
          static_cast<double>(config.rings))));
    const double azimuth = std::atan2(point.y, point.x);
    const auto sector = static_cast<std::size_t>(
      std::floor(
        (azimuth + kPi) / (2.0 * kPi) * static_cast<double>(config.sectors) +
        1.0e-12)) %
      config.sectors;
    const std::size_t index = ring * config.sectors + sector;
    const float normalized_height = static_cast<float>(
      std::clamp((point.z - config.min_z_m) / height_span, 0.0, 1.0));
    result.values[index] = std::max(result.values[index], normalized_height);
    ++result.accepted_points;
  }

  for (std::size_t ring = 0; ring < config.rings; ++ring) {
    double sum = 0.0;
    for (std::size_t sector = 0; sector < config.sectors; ++sector) {
      const auto index = ring * config.sectors + sector;
      sum += result.values[index];
      result.occupied_cells += result.values[index] > 0.0F ? 1U : 0U;
    }
    result.ring_key[ring] =
      static_cast<float>(sum / static_cast<double>(config.sectors));
  }
  for (std::size_t sector = 0; sector < config.sectors; ++sector) {
    double sum = 0.0;
    for (std::size_t ring = 0; ring < config.rings; ++ring) {
      sum += result.values[ring * config.sectors + sector];
    }
    result.sector_key[sector] =
      static_cast<float>(sum / static_cast<double>(config.rings));
  }
  return result;
}

inline double normalizedKeyDistance(
  const std::vector<float> & lhs,
  const std::vector<float> & rhs)
{
  if (lhs.empty() || lhs.size() != rhs.size()) {
    return std::numeric_limits<double>::infinity();
  }
  double squared_error = 0.0;
  double squared_scale = 0.0;
  for (std::size_t i = 0; i < lhs.size(); ++i) {
    const double error = static_cast<double>(lhs[i]) - rhs[i];
    squared_error += error * error;
    squared_scale += std::max(
      static_cast<double>(lhs[i]) * lhs[i],
      static_cast<double>(rhs[i]) * rhs[i]);
  }
  return std::sqrt(squared_error / std::max(squared_scale, 1.0e-12));
}

inline DescriptorMatch matchPolarDescriptors(
  const PolarDescriptor & reference,
  const PolarDescriptor & query,
  double minimum_overlap_ratio = 0.0)
{
  const auto & config = reference.config;
  if (config.rings != query.config.rings || config.sectors != query.config.sectors ||
    reference.values.size() != config.rings * config.sectors ||
    query.values.size() != reference.values.size())
  {
    throw std::invalid_argument("descriptor dimensions do not match");
  }

  DescriptorMatch best;
  const auto total_cells = static_cast<double>(reference.values.size());
  for (std::size_t shift = 0; shift < config.sectors; ++shift) {
    double dot = 0.0;
    double reference_norm = 0.0;
    double query_norm = 0.0;
    std::size_t intersection = 0;
    std::size_t union_count = 0;
    for (std::size_t ring = 0; ring < config.rings; ++ring) {
      for (std::size_t sector = 0; sector < config.sectors; ++sector) {
        const auto query_index = ring * config.sectors + sector;
        const auto reference_sector = (sector + shift) % config.sectors;
        const auto reference_index = ring * config.sectors + reference_sector;
        const double a = query.values[query_index];
        const double b = reference.values[reference_index];
        const bool a_occupied = a > 0.0;
        const bool b_occupied = b > 0.0;
        intersection += (a_occupied && b_occupied) ? 1U : 0U;
        union_count += (a_occupied || b_occupied) ? 1U : 0U;
        dot += a * b;
        reference_norm += a * a;
        query_norm += b * b;
      }
    }
    const double overlap = union_count == 0 ? 0.0 :
      static_cast<double>(intersection) / static_cast<double>(union_count);
    const double coverage = static_cast<double>(union_count) / total_cells;
    if (overlap < minimum_overlap_ratio || coverage < minimum_overlap_ratio ||
      reference_norm < 1.0e-12 || query_norm < 1.0e-12)
    {
      continue;
    }
    const double cosine = std::clamp(
      dot / std::sqrt(reference_norm * query_norm), 0.0, 1.0);
    const double distance = 1.0 - cosine;
    if (distance < best.distance) {
      const int signed_shift = shift > config.sectors / 2 ?
        static_cast<int>(shift) - static_cast<int>(config.sectors) :
        static_cast<int>(shift);
      best.distance = distance;
      best.sector_shift = signed_shift;
      best.yaw_rad =
        static_cast<double>(signed_shift) * 2.0 * kPi /
        static_cast<double>(config.sectors);
      best.overlap_ratio = overlap;
    }
  }
  return best;
}

}  // namespace go2_map_localizer

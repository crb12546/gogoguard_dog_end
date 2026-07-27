// Copyright 2026 Go2 Robotics Team
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

namespace go2_map_localizer
{

// Keep the confidence scale fixed when an evidence channel is unavailable.
// Tracking registrations have no independent place descriptor, so they must
// neither receive nor renormalize its 0.20 weight.
inline double registrationConfidence(
  double inlier_ratio,
  double fitness_rmse_m,
  double maximum_fitness_rmse_m,
  double descriptor_distance = std::numeric_limits<double>::infinity())
{
  if (!std::isfinite(inlier_ratio) || !std::isfinite(fitness_rmse_m) ||
    !std::isfinite(maximum_fitness_rmse_m) ||
    inlier_ratio < 0.0 || inlier_ratio > 1.0 ||
    fitness_rmse_m < 0.0 || maximum_fitness_rmse_m <= 0.0)
  {
    return 0.0;
  }
  const double fitness_quality =
    std::exp(-fitness_rmse_m / maximum_fitness_rmse_m);
  const double descriptor_quality =
    std::isfinite(descriptor_distance) && descriptor_distance >= 0.0 ?
    std::exp(-descriptor_distance) : 0.0;
  return std::clamp(
    0.45 * inlier_ratio + 0.35 * fitness_quality +
    0.20 * descriptor_quality, 0.0, 1.0);
}

// Anchored registration already has an operator-reviewed route pose as an
// independent place prior. Re-scale the two scan-registration channels to the
// full [0, 1] confidence range instead of reserving 0.20 for a descriptor that
// this mode intentionally does not compute.
inline double descriptorlessRegistrationConfidence(
  double inlier_ratio,
  double fitness_rmse_m,
  double maximum_fitness_rmse_m)
{
  if (!std::isfinite(inlier_ratio) || !std::isfinite(fitness_rmse_m) ||
    !std::isfinite(maximum_fitness_rmse_m) ||
    inlier_ratio < 0.0 || inlier_ratio > 1.0 ||
    fitness_rmse_m < 0.0 || maximum_fitness_rmse_m <= 0.0)
  {
    return 0.0;
  }
  const double fitness_quality =
    std::exp(-fitness_rmse_m / maximum_fitness_rmse_m);
  constexpr double evidence_weight = 0.45 + 0.35;
  return std::clamp(
    (0.45 * inlier_ratio + 0.35 * fitness_quality) / evidence_weight,
    0.0, 1.0);
}

// Confidence is bounded to [0, 1]. Requiring both an absolute and a relative
// separation prevents a superficially large percentage at very low confidence
// and implements the safety-case "best is at least 20% better" criterion.
inline bool passesHypothesisMargin(
  double best_confidence,
  double runner_up_confidence,
  double minimum_absolute_margin,
  double minimum_relative_margin)
{
  if (!std::isfinite(best_confidence) || !std::isfinite(runner_up_confidence) ||
    best_confidence <= 0.0 || runner_up_confidence < 0.0 ||
    minimum_absolute_margin < 0.0 || minimum_relative_margin < 0.0)
  {
    return false;
  }
  const double absolute_margin = best_confidence - runner_up_confidence;
  const double relative_margin =
    absolute_margin / std::max(best_confidence, 1.0e-12);
  return absolute_margin >= minimum_absolute_margin &&
         relative_margin >= minimum_relative_margin;
}

inline bool hypothesesAreDistinct(
  double translation_m,
  double rotation_rad,
  double separation_m,
  double separation_rad)
{
  if (!std::isfinite(translation_m) || !std::isfinite(rotation_rad) ||
    !std::isfinite(separation_m) || !std::isfinite(separation_rad) ||
    translation_m < 0.0 || rotation_rad < 0.0 ||
    separation_m <= 0.0 || separation_rad <= 0.0)
  {
    return true;
  }
  return translation_m > separation_m || rotation_rad > separation_rad;
}

inline bool correctionWithinEnvelope(
  double translation_m,
  double rotation_rad,
  double maximum_translation_m,
  double maximum_rotation_rad)
{
  return std::isfinite(translation_m) && std::isfinite(rotation_rad) &&
         std::isfinite(maximum_translation_m) &&
         std::isfinite(maximum_rotation_rad) &&
         translation_m >= 0.0 && rotation_rad >= 0.0 &&
         maximum_translation_m > 0.0 && maximum_rotation_rad > 0.0 &&
         translation_m <= maximum_translation_m &&
         rotation_rad <= maximum_rotation_rad;
}

inline bool registrationQualityIsSafe(
  double fitness_rmse_m,
  double inlier_ratio,
  double maximum_fitness_rmse_m,
  double minimum_inlier_ratio)
{
  return std::isfinite(fitness_rmse_m) && std::isfinite(inlier_ratio) &&
         fitness_rmse_m >= 0.0 && fitness_rmse_m <= maximum_fitness_rmse_m &&
         inlier_ratio >= minimum_inlier_ratio && inlier_ratio <= 1.0;
}

inline bool timestampAgeIsAcceptable(
  double age_sec,
  double maximum_future_offset_sec,
  double maximum_age_sec)
{
  return std::isfinite(age_sec) &&
         age_sec >= -maximum_future_offset_sec &&
         age_sec <= maximum_age_sec;
}

}  // namespace go2_map_localizer

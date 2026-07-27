// Copyright 2026 Go2 Robotics Team
// SPDX-License-Identifier: Apache-2.0
#include "go2_map_localizer/landmark_evidence.hpp"

#include <array>
#include <stdexcept>
#include <vector>

namespace
{

void require(bool condition, const char * message)
{
  if (!condition) {
    throw std::runtime_error(message);
  }
}

}  // namespace

int main()
{
  using go2_map_localizer::LandmarkEvidenceConfig;
  using go2_map_localizer::evaluateLandmarkEvidence;

  LandmarkEvidenceConfig config;
  config.minimum_matches = 40;
  config.minimum_unique_voxels = 16;
  config.minimum_support_match_ratio = 0.10;
  config.minimum_major_spread_m = 1.5;

  std::vector<std::array<double, 3>> distributed;
  for (int index = 0; index < 20; ++index) {
    distributed.push_back({0.25 * index, 0.1 * (index % 2), 0.0});
  }
  const auto accepted =
    evaluateLandmarkEvidence(200, 60, distributed, config);
  require(accepted.accepted, "distributed stable matches should pass");
  require(accepted.major_spread_m > 1.5, "major spread was not measured");

  const auto too_few =
    evaluateLandmarkEvidence(200, 12, distributed, config);
  require(!too_few.accepted, "too few stable matches must fail");

  std::vector<std::array<double, 3>> vertical_pole;
  for (int index = 0; index < 20; ++index) {
    vertical_pole.push_back({3.0, 4.0, 0.15 * index});
  }
  const auto accepted_pole =
    evaluateLandmarkEvidence(200, 60, vertical_pole, config);
  require(accepted_pole.accepted, "vertical stable structure should pass");

  std::vector<std::array<double, 3>> tiny_patch(
    20, std::array<double, 3>{1.0, 1.0, 1.0});
  const auto concentrated =
    evaluateLandmarkEvidence(200, 80, tiny_patch, config);
  require(!concentrated.accepted, "one dense patch must not pass");
  return 0;
}

// Copyright 2026 Go2 Robotics Team
// SPDX-License-Identifier: Apache-2.0
#include "go2_map_localizer/quality_gates.hpp"

#include <cmath>
#include <stdexcept>

namespace
{

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
  using go2_map_localizer::descriptorlessRegistrationConfidence;
  using go2_map_localizer::registrationConfidence;

  require(
    near(descriptorlessRegistrationConfidence(1.0, 0.0, 0.4), 1.0),
    "perfect descriptorless evidence must map to confidence one");

  const double expected =
    (0.45 * 0.60 + 0.35 * std::exp(-0.20 / 0.40)) / 0.80;
  const double normalized =
    descriptorlessRegistrationConfidence(0.60, 0.20, 0.40);
  require(
    near(normalized, expected),
    "descriptorless confidence was not normalized over available evidence");
  require(
    normalized > registrationConfidence(0.60, 0.20, 0.40),
    "anchored confidence must not reserve weight for an absent descriptor");
  require(
    normalized >= 0.55,
    "representative safe anchored evidence should pass the handoff threshold");

  require(
    descriptorlessRegistrationConfidence(-0.1, 0.1, 0.4) == 0.0,
    "invalid inlier ratio must return zero confidence");
  require(
    descriptorlessRegistrationConfidence(0.8, -0.1, 0.4) == 0.0,
    "invalid fitness must return zero confidence");
  require(
    descriptorlessRegistrationConfidence(0.8, 0.1, 0.0) == 0.0,
    "invalid fitness scale must return zero confidence");
  return 0;
}

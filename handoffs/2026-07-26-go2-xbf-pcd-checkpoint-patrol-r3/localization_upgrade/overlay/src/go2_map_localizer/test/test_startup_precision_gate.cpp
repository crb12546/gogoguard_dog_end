// Copyright 2026 Go2 Robotics Team
// SPDX-License-Identifier: Apache-2.0
#include "go2_map_localizer/startup_precision_gate.hpp"

#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace
{

using go2_map_localizer::StartupPrecisionGate;
using go2_map_localizer::StartupPrecisionPose;

constexpr double kYawThreshold = 0.00523598776;
constexpr double kPi = 3.14159265358979323846;

bool near(double left, double right, double tolerance = 1.0e-12)
{
  return std::abs(left - right) <= tolerance;
}

void rejectsInvalidConfiguration()
{
  bool rejected = false;
  try {
    const StartupPrecisionGate invalid(0.0, kYawThreshold);
    static_cast<void>(invalid);
  } catch (const std::invalid_argument &) {
    rejected = true;
  }
  assert(rejected);

  rejected = false;
  try {
    const StartupPrecisionGate invalid(
      0.1, std::numeric_limits<double>::quiet_NaN());
    static_cast<void>(invalid);
  } catch (const std::invalid_argument &) {
    rejected = true;
  }
  assert(rejected);
}

void acceptsAConsistentFiveSolutionSequence()
{
  StartupPrecisionGate gate(0.1, kYawThreshold);
  gate.start({10.0, -4.0, 0.5, 1.0});
  assert(gate.active());
  assert(!gate.verified());
  assert(gate.observations() == 1);

  assert(gate.observe({10.02, -4.01, 0.5, 1.001}));
  assert(gate.observe({9.96, -4.02, 0.51, 0.998}));
  assert(gate.observe({10.08, -4.01, 0.5, 1.004}));
  assert(gate.observe({9.95, -3.97, 0.49, 0.996}));
  assert(gate.observations() == 5);
  assert(gate.maximumTranslationDeviation() <= 0.1);
  assert(gate.maximumYawDeviation() <= kYawThreshold);
  assert(gate.verify());
  assert(gate.verified());
  assert(!gate.active());

  // Once promoted, normal tracking observations are intentionally outside
  // this startup-only gate and cannot revoke its result.
  assert(gate.observe({100.0, 100.0, 100.0, -2.0}));
  assert(gate.verified());
}

void comparesEverySolutionWithTheFirstAnchor()
{
  StartupPrecisionGate gate(0.1, kYawThreshold);
  gate.start({0.0, 0.0, 0.0, 0.0});
  assert(gate.observe({0.08, 0.0, 0.0, 0.0}));
  assert(gate.observe({-0.08, 0.0, 0.0, 0.0}));
  assert(near(gate.maximumTranslationDeviation(), 0.08));

  // This candidate is only 0.03 m from the preceding one, but 0.11 m from
  // the first anchor, so chaining cannot hide the violation.
  assert(!gate.observe({-0.11, 0.0, 0.0, 0.0}));
  assert(gate.violated());
  assert(!gate.active());
  assert(!gate.verify());
  assert(near(gate.maximumTranslationDeviation(), 0.11));
}

void acceptsValuesExactlyAtTheConfiguredLimits()
{
  StartupPrecisionGate gate(0.1, kYawThreshold);
  gate.start({0.0, 0.0, 0.0, 0.0});
  assert(gate.observe({0.1, 0.0, 0.0, kYawThreshold}));
  assert(near(gate.maximumTranslationDeviation(), 0.1));
  assert(near(gate.maximumYawDeviation(), kYawThreshold));
}

void handlesYawWrapAndRejectsMoreThanPointThreeDegrees()
{
  StartupPrecisionGate gate(0.1, kYawThreshold);
  gate.start({0.0, 0.0, 0.0, kPi - 0.002});
  assert(gate.observe({0.0, 0.0, 0.0, -kPi + 0.002}));
  assert(near(gate.maximumYawDeviation(), 0.004, 1.0e-10));
  assert(!gate.observe({0.0, 0.0, 0.0, -kPi + 0.004}));
  assert(gate.violated());
  assert(gate.maximumYawDeviation() > kYawThreshold);
  assert(!gate.verified());
}

void includesVerticalTranslationAndResetsCleanly()
{
  StartupPrecisionGate gate(0.1, kYawThreshold);
  gate.start({1.0, 2.0, 3.0, 0.0});
  assert(!gate.observe({1.0, 2.0, 3.1001, 0.0}));
  assert(gate.violated());
  gate.reset();
  assert(!gate.active());
  assert(!gate.verified());
  assert(!gate.violated());
  assert(gate.observations() == 0);
  assert(near(gate.maximumTranslationDeviation(), 0.0));
  assert(near(gate.maximumYawDeviation(), 0.0));
}

}  // namespace

int main()
{
  rejectsInvalidConfiguration();
  acceptsAConsistentFiveSolutionSequence();
  comparesEverySolutionWithTheFirstAnchor();
  acceptsValuesExactlyAtTheConfiguredLimits();
  handlesYawWrapAndRejectsMoreThanPointThreeDegrees();
  includesVerticalTranslationAndResetsCleanly();
  std::cout << "startup precision gate tests passed\n";
  return 0;
}

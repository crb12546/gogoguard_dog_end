// Copyright 2026 Go2 Robotics Team
// SPDX-License-Identifier: Apache-2.0
#include "go2_map_localizer/rigid_pose_difference.hpp"

#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace
{

constexpr double kPi = 3.14159265358979323846;

void require(bool condition, const char * message)
{
  if (!condition) {
    throw std::runtime_error(message);
  }
}

bool near(double left, double right, double tolerance = 1.0e-12)
{
  return std::abs(left - right) <= tolerance;
}

std::array<double, 9> yawRotation(double yaw)
{
  const double cosine = std::cos(yaw);
  const double sine = std::sin(yaw);
  return {
    cosine, -sine, 0.0,
    sine, cosine, 0.0,
    0.0, 0.0, 1.0};
}

void commonMapTranslationDoesNotCreateTranslationError()
{
  constexpr double yaw_difference = 0.3 * kPi / 180.0;
  const auto reference_rotation = yawRotation(0.0);
  const auto candidate_rotation = yawRotation(yaw_difference);

  const auto at_origin = go2_map_localizer::rigidPoseDifference(
    {0.0, 0.0, 0.0}, reference_rotation,
    {0.0, 0.0, 0.0}, candidate_rotation);
  const auto shifted_one_hundred_metres =
    go2_map_localizer::rigidPoseDifference(
    {100.0, -40.0, 2.0}, reference_rotation,
    {100.0, -40.0, 2.0}, candidate_rotation);

  require(near(at_origin.translation_m, 0.0), "origin case invented translation");
  require(
    near(shifted_one_hundred_metres.translation_m, 0.0),
    "100 m map offset invented translation");
  require(
    near(at_origin.rotation_rad, yaw_difference, 1.0e-12),
    "origin yaw difference is incorrect");
  require(
    near(shifted_one_hundred_metres.rotation_rad, yaw_difference, 1.0e-12),
    "100 m map offset changed yaw difference");
}

void commonMapTranslationPreservesRealPoseDifference()
{
  const auto reference_rotation = yawRotation(-0.4);
  const auto candidate_rotation = yawRotation(0.2);
  const std::array<double, 3> reference{2.0, -1.0, 0.5};
  const std::array<double, 3> candidate{2.08, -1.06, 0.52};
  const std::array<double, 3> shifted_reference{102.0, 49.0, 10.5};
  const std::array<double, 3> shifted_candidate{102.08, 48.94, 10.52};

  const auto unshifted = go2_map_localizer::rigidPoseDifference(
    reference, reference_rotation, candidate, candidate_rotation);
  const auto shifted = go2_map_localizer::rigidPoseDifference(
    shifted_reference, reference_rotation, shifted_candidate, candidate_rotation);

  require(
    near(unshifted.translation_m, shifted.translation_m, 1.0e-12),
    "common map translation changed translation difference");
  require(
    near(unshifted.rotation_rad, shifted.rotation_rad, 1.0e-12),
    "common map translation changed rotation difference");
}

void invalidInputFailsClosed()
{
  const auto identity = yawRotation(0.0);
  const auto difference = go2_map_localizer::rigidPoseDifference(
    {0.0, 0.0, 0.0}, identity,
    {std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0}, identity);
  require(
    std::isinf(difference.translation_m) && std::isinf(difference.rotation_rad),
    "non-finite pose difference did not fail closed");
}

}  // namespace

int main()
{
  commonMapTranslationDoesNotCreateTranslationError();
  commonMapTranslationPreservesRealPoseDifference();
  invalidInputFailsClosed();
  std::cout << "rigid pose difference tests passed\n";
  return 0;
}

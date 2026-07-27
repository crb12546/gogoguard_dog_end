// Copyright 2026 Go2 Robotics Team
// SPDX-License-Identifier: Apache-2.0
#include "go2_map_localizer/confirmation_gate.hpp"
#include "go2_map_localizer/map_load_quarantine.hpp"
#include "go2_map_localizer/planar_transform.hpp"
#include "go2_map_localizer/polar_descriptor.hpp"
#include "go2_map_localizer/quality_gates.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
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

std::vector<go2_map_localizer::Point3> rotate(
  const std::vector<go2_map_localizer::Point3> & points, double angle)
{
  const double cosine = std::cos(angle);
  const double sine = std::sin(angle);
  std::vector<go2_map_localizer::Point3> result;
  result.reserve(points.size());
  for (const auto & point : points) {
    result.push_back(
      {cosine * point.x - sine * point.y,
        sine * point.x + cosine * point.y,
        point.z});
  }
  return result;
}

}  // namespace

int main()
{
  using namespace go2_map_localizer;
  PolarDescriptorConfig config;
  config.rings = 4;
  config.sectors = 12;
  config.max_radius_m = 12.0;
  config.min_radius_m = 0.0;
  config.min_z_m = -2.0;
  config.max_z_m = 4.0;

  {
    auto invalid_config = config;
    invalid_config.max_radius_m = std::numeric_limits<double>::quiet_NaN();
    bool rejected = false;
    try {
      validateConfig(invalid_config);
    } catch (const std::invalid_argument &) {
      rejected = true;
    }
    require(rejected, "non-finite descriptor configuration must be rejected");
  }
  {
    auto invalid_config = config;
    invalid_config.sectors = 5000;
    bool rejected = false;
    try {
      validateConfig(invalid_config);
    } catch (const std::invalid_argument &) {
      rejected = true;
    }
    require(rejected, "unbounded descriptor dimensions must be rejected");
  }

  const std::vector<Point3> map_points{
    {2.0, 0.0, 0.0}, {2.1, 0.1, 2.5}, {4.0, 2.0, 1.0},
    {-3.0, 5.0, 3.5}, {-6.0, -1.0, -0.5}, {1.0, -8.0, 1.5},
    {9.0, 3.0, 2.0}, {-8.0, 4.0, 0.5}};
  const auto reference = buildPolarDescriptor(map_points, config);
  require(reference.accepted_points == map_points.size(), "all fixture points should be accepted");
  require(reference.values.size() == 48, "row-major descriptor size is wrong");
  // Golden values are emitted by go2_map_tools.compute_polar_descriptor for
  // this fixture. They protect the on-robot C++ query from drifting away from
  // the offline Python index builder.
  const std::vector<std::pair<std::size_t, double>> golden_cells{
    {6, 0.75}, {18, 0.5}, {22, 11.0 / 12.0}, {24, 0.25},
    {27, 7.0 / 12.0}, {35, 5.0 / 12.0}, {42, 2.0 / 3.0}};
  std::size_t nonzero_cells = 0;
  for (std::size_t index = 0; index < reference.values.size(); ++index) {
    if (reference.values[index] > 0.0F) {
      ++nonzero_cells;
    }
  }
  require(nonzero_cells == golden_cells.size(), "offline/online occupied cells drifted");
  for (const auto & [index, expected] : golden_cells) {
    require(
      std::abs(reference.values[index] - expected) < 1.0e-6,
      "offline/online descriptor cell drifted");
  }
  const auto positive_pi = buildPolarDescriptor({{-2.0, 0.0, 1.0}}, config);
  const auto negative_pi = buildPolarDescriptor({{-2.0, -0.0, 1.0}}, config);
  require(
    positive_pi.values == negative_pi.values,
    "+pi/-pi must use the same half-open angular bin");

  const double robot_yaw = 2.0 * kPi / static_cast<double>(config.sectors);
  // A body-frame scan is the map geometry rotated by inverse robot yaw.
  const auto body_points = rotate(map_points, -robot_yaw);
  const auto query = buildPolarDescriptor(body_points, config);
  const auto match = matchPolarDescriptors(reference, query);
  require(match.distance < 1.0e-6, "rotated descriptor should match exactly");
  require(
    std::abs(normalizeAngle(match.yaw_rad - robot_yaw)) < 1.0e-6,
    "descriptor yaw sign/convention is wrong");
  require(
    normalizedKeyDistance(reference.ring_key, query.ring_key) < 1.0e-6,
    "ring key must be yaw invariant");
  PolarDescriptorConfig sparse_config;
  const auto sparse_reference =
    buildPolarDescriptor({{5.0, 0.0, 1.0}}, sparse_config);
  const auto sparse_query =
    buildPolarDescriptor({{5.0, 0.0, 1.0}}, sparse_config);
  require(
    !std::isfinite(
      matchPolarDescriptors(
        sparse_reference, sparse_query, 0.05).distance),
    "single-sector descriptor must fail the coverage gate");

  const PlanarTransform parent_from_middle{3.0, -2.0, 0.7};
  const PlanarTransform middle_from_child{-1.0, 4.0, -0.3};
  const auto parent_from_child = compose(parent_from_middle, middle_from_child);
  const auto recovered = compose(inverse(parent_from_middle), parent_from_child);
  require(std::abs(recovered.x - middle_from_child.x) < 1.0e-9, "inverse x failed");
  require(std::abs(recovered.y - middle_from_child.y) < 1.0e-9, "inverse y failed");
  require(
    std::abs(normalizeAngle(recovered.yaw - middle_from_child.yaw)) < 1.0e-9,
    "inverse yaw failed");

  ConfirmationGate confirmation(5, 2000000000LL);
  confirmation.start(1000000000LL);
  require(confirmation.pending() && confirmation.accepts() == 1, "global start not counted");
  require(!confirmation.accept(1500000000LL), "confirmed before required streak");
  require(!confirmation.accept(2000000000LL), "confirmed before required streak");
  require(!confirmation.accept(2500000000LL), "confirmed before minimum time span");
  require(confirmation.accept(3000000000LL), "5 accepts over 2 seconds must confirm");
  require(!confirmation.pending(), "confirmation gate remained pending");
  confirmation.start(4000000000LL);
  confirmation.accept(4500000000LL);
  confirmation.reject();
  require(
    confirmation.pending() && confirmation.accepts() == 0,
    "rejection must reset the provisional streak");
  require(!confirmation.accept(5000000000LL), "post-rejection streak restarted incorrectly");

  require(
    passesHypothesisMargin(0.70, 0.55, 0.10, 0.20),
    "clear global hypotheses should pass both margins");
  require(
    !passesHypothesisMargin(0.70, 0.58, 0.10, 0.20),
    "relative margin below 20 percent must fail");
  require(
    !passesHypothesisMargin(0.40, 0.31, 0.10, 0.20),
    "absolute margin below 0.10 must fail");
  require(
    !hypothesesAreDistinct(0.24, 0.08, 0.25, 0.0872665),
    "near-identical registration basins should be deduplicated");
  require(
    hypothesesAreDistinct(0.26, 0.08, 0.25, 0.0872665),
    "0.26 m competitor must enter the runner-up ambiguity gate");
  require(
    hypothesesAreDistinct(0.24, 0.09, 0.25, 0.0872665),
    "rotation beyond five degrees must enter the ambiguity gate");
  require(
    correctionWithinEnvelope(0.25, 0.12, 0.25, 0.122173),
    "confirmation envelope boundary must be inclusive");
  require(
    !correctionWithinEnvelope(0.26, 0.10, 0.25, 0.122173),
    "confirmation beyond the safe translation envelope must fail");
  require(
    !correctionWithinEnvelope(0.20, 0.13, 0.25, 0.122173),
    "confirmation beyond the safe rotation envelope must fail");
  require(
    registrationQualityIsSafe(0.40, 0.35, 0.40, 0.35),
    "registration quality boundary must be inclusive");
  require(
    !registrationQualityIsSafe(0.41, 0.55, 0.40, 0.35),
    "unsafe RMSE must fail closed");
  require(
    !registrationQualityIsSafe(0.20, 0.34, 0.40, 0.35),
    "unsafe overlap must fail closed");
  const double geometry_only_confidence =
    registrationConfidence(0.50, 0.40, 0.40);
  const double invalid_descriptor_confidence =
    registrationConfidence(
      0.50, 0.40, 0.40, std::numeric_limits<double>::quiet_NaN());
  const double perfect_descriptor_confidence =
    registrationConfidence(0.50, 0.40, 0.40, 0.0);
  require(
    std::abs(geometry_only_confidence - invalid_descriptor_confidence) < 1.0e-12,
    "missing descriptor evidence must contribute zero");
  require(
    geometry_only_confidence < perfect_descriptor_confidence,
    "missing descriptor evidence must score below real descriptor evidence");
  require(
    geometry_only_confidence < 0.55,
    "geometry-only confidence regression crossed the safe threshold");
  require(
    registrationConfidence(0.50, 0.40, 0.40, -0.1) ==
    geometry_only_confidence,
    "negative descriptor distance must be treated as unavailable");
  require(
    timestampAgeIsAcceptable(-0.10, 0.10, 0.25),
    "future tolerance boundary should be accepted at input only");
  require(
    !timestampAgeIsAcceptable(-0.1001, 0.10, 0.25),
    "excessively future-dated input must be rejected");
  require(
    timestampAgeIsAcceptable(0.25, 0.10, 0.25),
    "input-age boundary should be inclusive");
  require(
    !timestampAgeIsAcceptable(0.2501, 0.10, 0.25),
    "stale input must be rejected");

  MapLoadQuarantine quarantine(2000000000LL);
  require(!quarantine.ready(0), "unarmed map-load quarantine passed");
  quarantine.arm(1000000000LL);
  require(!quarantine.ready(2999999999LL), "map loaded before two-second dwell");
  require(quarantine.ready(3000000000LL), "map load blocked after two-second dwell");
  quarantine.disarm();
  require(!quarantine.ready(5000000000LL), "disarmed map-load quarantine passed");

  std::cout << "polar descriptor and planar transform tests passed\n";
  return EXIT_SUCCESS;
}

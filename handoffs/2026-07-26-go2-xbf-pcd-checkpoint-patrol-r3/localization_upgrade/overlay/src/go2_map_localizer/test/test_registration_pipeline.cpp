// Copyright 2026 Go2 Robotics Team
// SPDX-License-Identifier: Apache-2.0
#include "go2_map_localizer/registration_pipeline.hpp"

#include <pcl/common/transforms.h>

#include <Eigen/Geometry>

#include <cmath>
#include <cstdint>
#include <stdexcept>

namespace
{

using Cloud = pcl::PointCloud<pcl::PointXYZ>;

constexpr double kPi = 3.14159265358979323846;

void require(bool condition, const char * message)
{
  if (!condition) {
    throw std::runtime_error(message);
  }
}

Eigen::Matrix4f planarTransform(float x, float y, float z, float yaw)
{
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  transform.block<3, 3>(0, 0) =
    Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()).toRotationMatrix();
  transform(0, 3) = x;
  transform(1, 3) = y;
  transform(2, 3) = z;
  return transform;
}

Cloud::Ptr asymmetricCampusScene()
{
  Cloud::Ptr cloud(new Cloud());
  const auto append = [&cloud](float x, float y, float z) {
      cloud->push_back({x, y, z});
    };

  // Uneven ground returns.
  for (int x_index = -45; x_index <= 45; ++x_index) {
    for (int y_index = -36; y_index <= 36; ++y_index) {
      const float x = 0.28F * static_cast<float>(x_index);
      const float y = 0.28F * static_cast<float>(y_index);
      const float z =
        0.025F * std::sin(0.31F * x) + 0.018F * std::cos(0.47F * y);
      append(x, y, z);
    }
  }

  // Two non-symmetric building faces.
  for (int y_index = -34; y_index <= 16; ++y_index) {
    for (int z_index = 0; z_index <= 28; ++z_index) {
      append(
        -8.4F,
        0.24F * static_cast<float>(y_index),
        0.16F * static_cast<float>(z_index));
    }
  }
  for (int x_index = -10; x_index <= 42; ++x_index) {
    for (int z_index = 0; z_index <= 20; ++z_index) {
      append(
        0.23F * static_cast<float>(x_index),
        7.1F,
        0.18F * static_cast<float>(z_index));
    }
  }

  // Three poles at deliberately irregular locations.
  const float pole_centres[][2] = {
    {4.2F, -5.3F},
    {-2.1F, 3.7F},
    {9.0F, 1.4F},
  };
  for (const auto & centre : pole_centres) {
    for (int z_index = 0; z_index <= 36; ++z_index) {
      for (int angle_index = 0; angle_index < 10; ++angle_index) {
        const float angle =
          2.0F * static_cast<float>(kPi) *
          static_cast<float>(angle_index) / 10.0F;
        append(
          centre[0] + 0.10F * std::cos(angle),
          centre[1] + 0.10F * std::sin(angle),
          0.12F * static_cast<float>(z_index));
      }
    }
  }
  cloud->width = static_cast<std::uint32_t>(cloud->size());
  cloud->height = 1;
  cloud->is_dense = true;
  return cloud;
}

double yawOf(const Eigen::Matrix4f & transform)
{
  return std::atan2(
    static_cast<double>(transform(1, 0)),
    static_cast<double>(transform(0, 0)));
}

double wrappedAngle(double angle)
{
  return std::remainder(angle, 2.0 * kPi);
}

}  // namespace

int main()
{
  const auto target = asymmetricCampusScene();
  const Eigen::Matrix4f expected =
    planarTransform(2.4F, -1.3F, 0.08F, 0.12F);
  Cloud::Ptr source(new Cloud());
  pcl::transformPointCloud(*target, *source, expected.inverse());

  go2_map_localizer::RegistrationConfig config;
  config.source_coarse_voxel_m = 0.35;
  config.source_fine_voxel_m = 0.18;
  config.target_coarse_voxel_m = 0.35;
  config.target_fine_voxel_m = 0.18;
  config.ndt_coarse_resolution_m = 1.2;
  config.ndt_fine_resolution_m = 0.55;
  config.ndt_step_size_m = 0.15;
  config.transformation_epsilon = 0.001;
  config.ndt_coarse_iterations = 60;
  config.ndt_fine_iterations = 50;
  config.gicp_iterations = 70;
  config.gicp_max_correspondence_m = 1.0;
  config.inlier_distance_m = 0.35;
  config.maximum_source_range_m = 40.0;
  config.minimum_source_points = 1000;
  config.minimum_target_points = 1000;

  const Eigen::Matrix4f initial_guess =
    planarTransform(2.7F, -1.55F, 0.10F, 0.15F);
  const go2_map_localizer::RegistrationPipeline pipeline(config);
  const auto result = pipeline.align(source, target, initial_guess);

  require(result.converged, "known-truth registration did not converge");
  const double translation_error =
    static_cast<double>(
    (result.map_from_body.block<3, 1>(0, 3) -
    expected.block<3, 1>(0, 3)).norm());
  const double yaw_error =
    std::abs(wrappedAngle(yawOf(result.map_from_body) - yawOf(expected)));
  require(translation_error < 0.08, "known-truth translation error exceeded 8 cm");
  require(yaw_error < 0.3 * kPi / 180.0, "known-truth yaw error exceeded 0.3 degree");
  require(result.fitness < 0.08, "known-truth registration RMSE exceeded 8 cm");
  require(result.inlier_ratio > 0.90, "known-truth inlier ratio was below 90 percent");
  return 0;
}

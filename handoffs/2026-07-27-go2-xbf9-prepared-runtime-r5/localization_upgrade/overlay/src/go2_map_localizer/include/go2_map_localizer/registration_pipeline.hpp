// Copyright 2026 Go2 Robotics Team
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <Eigen/Core>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cstddef>
#include <string>

namespace go2_map_localizer
{

struct RegistrationConfig
{
  double source_coarse_voxel_m{0.8};
  double source_fine_voxel_m{0.35};
  double target_coarse_voxel_m{1.0};
  double target_fine_voxel_m{0.4};
  double ndt_coarse_resolution_m{2.0};
  double ndt_fine_resolution_m{0.8};
  double ndt_step_size_m{0.2};
  double transformation_epsilon{0.01};
  int ndt_coarse_iterations{35};
  int ndt_fine_iterations{25};
  int gicp_iterations{40};
  double gicp_max_correspondence_m{1.2};
  double inlier_distance_m{0.7};
  double maximum_source_range_m{30.0};
  std::size_t minimum_source_points{300};
  std::size_t minimum_target_points{1000};
};

struct RegistrationResult
{
  bool converged{false};
  Eigen::Matrix4f map_from_body{Eigen::Matrix4f::Identity()};
  // Root-mean-square GICP correspondence distance, in metres.
  double fitness{0.0};
  double inlier_ratio{0.0};
  std::size_t source_points{0};
  std::size_t target_points{0};
  std::string detail;
};

class RegistrationPipeline
{
public:
  explicit RegistrationPipeline(RegistrationConfig config);

  RegistrationResult align(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & source,
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & target,
    const Eigen::Matrix4f & initial_guess) const;

private:
  RegistrationConfig config_;
};

}  // namespace go2_map_localizer

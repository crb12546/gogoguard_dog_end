// Copyright 2026 Go2 Robotics Team
// SPDX-License-Identifier: Apache-2.0
#include "go2_map_localizer/registration_pipeline.hpp"

#include <pcl/common/point_tests.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/ndt.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <vector>

namespace go2_map_localizer
{
namespace
{

pcl::PointCloud<pcl::PointXYZ>::Ptr voxelized(
  const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & input, double leaf_size)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::VoxelGrid<pcl::PointXYZ> filter;
  filter.setLeafSize(
    static_cast<float>(leaf_size),
    static_cast<float>(leaf_size),
    static_cast<float>(leaf_size));
  filter.setInputCloud(input);
  filter.filter(*output);
  return output;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr croppedByRange(
  const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & input, double maximum_range)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>());
  output->reserve(input->size());
  const double squared_limit = maximum_range * maximum_range;
  for (const auto & point : input->points) {
    if (pcl::isFinite(point) &&
      static_cast<double>(point.x) * point.x +
      static_cast<double>(point.y) * point.y <= squared_limit)
    {
      output->push_back(point);
    }
  }
  output->width = static_cast<std::uint32_t>(output->size());
  output->height = 1;
  output->is_dense = true;
  return output;
}

double calculateInlierRatio(
  const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & source,
  const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & target,
  const Eigen::Matrix4f & transform,
  double maximum_distance)
{
  if (source->empty() || target->empty()) {
    return 0.0;
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::transformPointCloud(*source, *aligned, transform);
  pcl::KdTreeFLANN<pcl::PointXYZ> tree;
  tree.setInputCloud(target);
  const float threshold_squared =
    static_cast<float>(maximum_distance * maximum_distance);
  std::vector<int> indices(1);
  std::vector<float> squared_distances(1);
  std::size_t inliers = 0;
  for (const auto & point : aligned->points) {
    if (!pcl::isFinite(point)) {
      continue;
    }
    if (tree.nearestKSearch(point, 1, indices, squared_distances) == 1 &&
      squared_distances.front() <= threshold_squared)
    {
      ++inliers;
    }
  }
  return static_cast<double>(inliers) /
         static_cast<double>(source->size());
}

bool finiteTransform(const Eigen::Matrix4f & transform)
{
  if (!transform.allFinite() ||
    std::abs(transform(3, 0)) >= 1.0e-5F ||
    std::abs(transform(3, 1)) >= 1.0e-5F ||
    std::abs(transform(3, 2)) >= 1.0e-5F ||
    std::abs(transform(3, 3) - 1.0F) >= 1.0e-5F)
  {
    return false;
  }
  const Eigen::Matrix3f rotation = transform.block<3, 3>(0, 0);
  return std::abs(rotation.determinant() - 1.0F) < 0.05F &&
         (rotation.transpose() * rotation - Eigen::Matrix3f::Identity()).norm() < 0.05F;
}

}  // namespace

RegistrationPipeline::RegistrationPipeline(RegistrationConfig config)
: config_(std::move(config))
{
  const std::array scalar_parameters{
    config_.source_coarse_voxel_m,
    config_.source_fine_voxel_m,
    config_.target_coarse_voxel_m,
    config_.target_fine_voxel_m,
    config_.ndt_coarse_resolution_m,
    config_.ndt_fine_resolution_m,
    config_.ndt_step_size_m,
    config_.transformation_epsilon,
    config_.gicp_max_correspondence_m,
    config_.inlier_distance_m,
    config_.maximum_source_range_m};
  const bool all_scalars_finite = std::all_of(
    scalar_parameters.begin(), scalar_parameters.end(),
    [](double value) {return std::isfinite(value);});
  if (!all_scalars_finite ||
    config_.source_coarse_voxel_m <= 0.0 || config_.source_fine_voxel_m <= 0.0 ||
    config_.target_coarse_voxel_m <= 0.0 || config_.target_fine_voxel_m <= 0.0 ||
    config_.ndt_coarse_resolution_m <= 0.0 || config_.ndt_fine_resolution_m <= 0.0 ||
    config_.ndt_step_size_m <= 0.0 || config_.transformation_epsilon <= 0.0 ||
    config_.ndt_coarse_iterations <= 0 || config_.ndt_fine_iterations <= 0 ||
    config_.gicp_iterations <= 0 ||
    config_.ndt_coarse_iterations > 1000 || config_.ndt_fine_iterations > 1000 ||
    config_.gicp_iterations > 1000 ||
    config_.gicp_max_correspondence_m <= 0.0 ||
    config_.inlier_distance_m <= 0.0 || config_.maximum_source_range_m <= 0.0 ||
    config_.minimum_source_points == 0 ||
    config_.minimum_target_points == 0 ||
    config_.minimum_source_points > 10000000 ||
    config_.minimum_target_points > 10000000)
  {
    throw std::invalid_argument("invalid registration pipeline configuration");
  }
}

RegistrationResult RegistrationPipeline::align(
  const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & source,
  const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & target,
  const Eigen::Matrix4f & initial_guess) const
{
  RegistrationResult result;
  if (!source || !target) {
    result.detail = "null source or target cloud";
    return result;
  }
  const auto registration_source = croppedByRange(source, config_.maximum_source_range_m);
  result.source_points = registration_source->size();
  result.target_points = target->size();
  if (registration_source->size() < config_.minimum_source_points) {
    result.detail = "insufficient source points";
    return result;
  }
  if (target->size() < config_.minimum_target_points) {
    result.detail = "insufficient target points";
    return result;
  }
  if (!finiteTransform(initial_guess)) {
    result.detail = "non-finite initial transform";
    return result;
  }

  const auto source_coarse = voxelized(registration_source, config_.source_coarse_voxel_m);
  const auto target_coarse = voxelized(target, config_.target_coarse_voxel_m);
  const auto source_fine = voxelized(registration_source, config_.source_fine_voxel_m);
  const auto target_fine = voxelized(target, config_.target_fine_voxel_m);
  if (source_coarse->size() < 100 || target_coarse->size() < 300 ||
    source_fine->size() < 150 || target_fine->size() < 500)
  {
    result.detail = "voxel filtering left too few points";
    return result;
  }

  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt_coarse;
  ndt_coarse.setTransformationEpsilon(config_.transformation_epsilon);
  ndt_coarse.setStepSize(config_.ndt_step_size_m);
  ndt_coarse.setResolution(
    static_cast<float>(config_.ndt_coarse_resolution_m));
  ndt_coarse.setMaximumIterations(config_.ndt_coarse_iterations);
  ndt_coarse.setInputSource(source_coarse);
  ndt_coarse.setInputTarget(target_coarse);
  pcl::PointCloud<pcl::PointXYZ> coarse_output;
  ndt_coarse.align(coarse_output, initial_guess);
  if (!ndt_coarse.hasConverged() || !finiteTransform(ndt_coarse.getFinalTransformation())) {
    result.detail = "coarse NDT did not converge";
    return result;
  }

  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt_fine;
  ndt_fine.setTransformationEpsilon(config_.transformation_epsilon * 0.5);
  ndt_fine.setStepSize(config_.ndt_step_size_m * 0.5);
  ndt_fine.setResolution(
    static_cast<float>(config_.ndt_fine_resolution_m));
  ndt_fine.setMaximumIterations(config_.ndt_fine_iterations);
  ndt_fine.setInputSource(source_fine);
  ndt_fine.setInputTarget(target_fine);
  pcl::PointCloud<pcl::PointXYZ> fine_output;
  ndt_fine.align(fine_output, ndt_coarse.getFinalTransformation());
  if (!ndt_fine.hasConverged() || !finiteTransform(ndt_fine.getFinalTransformation())) {
    result.detail = "fine NDT did not converge";
    return result;
  }

  pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
  gicp.setMaximumIterations(config_.gicp_iterations);
  gicp.setMaxCorrespondenceDistance(config_.gicp_max_correspondence_m);
  gicp.setTransformationEpsilon(config_.transformation_epsilon * 0.25);
  gicp.setEuclideanFitnessEpsilon(1.0e-5);
  gicp.setRANSACIterations(0);
  gicp.setInputSource(source_fine);
  gicp.setInputTarget(target_fine);
  pcl::PointCloud<pcl::PointXYZ> gicp_output;
  gicp.align(gicp_output, ndt_fine.getFinalTransformation());
  if (!gicp.hasConverged() || !finiteTransform(gicp.getFinalTransformation())) {
    result.detail = "GICP did not converge";
    return result;
  }

  result.map_from_body = gicp.getFinalTransformation();
  // PCL returns a mean squared correspondence distance. Expose RMSE so the
  // public status value and all configured fitness gates are in metres.
  const double mean_squared_fitness =
    gicp.getFitnessScore(config_.gicp_max_correspondence_m);
  result.fitness = mean_squared_fitness >= 0.0 ?
    std::sqrt(mean_squared_fitness) : std::numeric_limits<double>::infinity();
  result.inlier_ratio = calculateInlierRatio(
    source_fine, target_fine, result.map_from_body, config_.inlier_distance_m);
  result.converged = std::isfinite(result.fitness) && std::isfinite(result.inlier_ratio);
  result.detail = "coarse NDT + fine NDT + GICP converged";
  return result;
}

}  // namespace go2_map_localizer

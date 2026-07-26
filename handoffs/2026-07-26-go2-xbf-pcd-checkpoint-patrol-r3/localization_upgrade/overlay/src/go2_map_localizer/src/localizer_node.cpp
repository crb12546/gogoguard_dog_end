// Copyright 2026 Go2 Robotics Team
// SPDX-License-Identifier: Apache-2.0
#include "go2_map_localizer/anchored_seed.hpp"
#include "go2_map_localizer/confirmation_gate.hpp"
#include "go2_map_localizer/map_load_quarantine.hpp"
#include "go2_map_localizer/map_manifest.hpp"
#include "go2_map_localizer/quality_gates.hpp"
#include "go2_map_localizer/registration_pipeline.hpp"
#include "go2_map_localizer/rigid_pose_difference.hpp"
#include "go2_map_localizer/startup_precision_gate.hpp"

#include <go2_nav_interfaces/msg/localization_status.hpp>
#include <go2_nav_interfaces/srv/global_relocalize.hpp>
#include <go2_nav_interfaces/srv/load_map.hpp>
#include <go2_nav_interfaces/srv/reset_localization.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <pcl/common/point_tests.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <functional>
#include <iterator>
#include <limits>
#include <list>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace go2_map_localizer
{
namespace
{

using Cloud = pcl::PointCloud<pcl::PointXYZ>;
using LocalizationStatus = go2_nav_interfaces::msg::LocalizationStatus;
using GlobalRelocalize = go2_nav_interfaces::srv::GlobalRelocalize;
using LoadMap = go2_nav_interfaces::srv::LoadMap;
using ResetLocalization = go2_nav_interfaces::srv::ResetLocalization;
using SetBool = std_srvs::srv::SetBool;

constexpr double kPiLocal = 3.14159265358979323846;

Eigen::Matrix4f poseToMatrix(const geometry_msgs::msg::Pose & pose)
{
  const Eigen::Quaternionf rotation(
    static_cast<float>(pose.orientation.w),
    static_cast<float>(pose.orientation.x),
    static_cast<float>(pose.orientation.y),
    static_cast<float>(pose.orientation.z));
  if (!rotation.coeffs().allFinite() || rotation.norm() < 1.0e-6F) {
    throw std::runtime_error("pose contains an invalid quaternion");
  }
  Eigen::Matrix4f result = Eigen::Matrix4f::Identity();
  result.block<3, 3>(0, 0) = rotation.normalized().toRotationMatrix();
  result(0, 3) = static_cast<float>(pose.position.x);
  result(1, 3) = static_cast<float>(pose.position.y);
  result(2, 3) = static_cast<float>(pose.position.z);
  if (!result.allFinite()) {
    throw std::runtime_error("pose contains non-finite values");
  }
  return result;
}

geometry_msgs::msg::Pose matrixToPose(const Eigen::Matrix4f & transform)
{
  geometry_msgs::msg::Pose pose;
  const Eigen::Quaternionf rotation(transform.block<3, 3>(0, 0));
  pose.position.x = transform(0, 3);
  pose.position.y = transform(1, 3);
  pose.position.z = transform(2, 3);
  pose.orientation.x = rotation.x();
  pose.orientation.y = rotation.y();
  pose.orientation.z = rotation.z();
  pose.orientation.w = rotation.w();
  return pose;
}

Eigen::Matrix4f interpolateTransform(
  const Eigen::Matrix4f & from, const Eigen::Matrix4f & to, double ratio)
{
  const float alpha = static_cast<float>(std::clamp(ratio, 0.0, 1.0));
  Eigen::Matrix4f result = Eigen::Matrix4f::Identity();
  result.block<3, 1>(0, 3) =
    (1.0F - alpha) * from.block<3, 1>(0, 3) + alpha * to.block<3, 1>(0, 3);
  const Eigen::Quaternionf from_rotation(from.block<3, 3>(0, 0));
  const Eigen::Quaternionf to_rotation(to.block<3, 3>(0, 0));
  result.block<3, 3>(0, 0) =
    from_rotation.normalized().slerp(alpha, to_rotation.normalized()).toRotationMatrix();
  return result;
}

RigidPoseDifference matrixPoseDifference(
  const Eigen::Matrix4f & reference,
  const Eigen::Matrix4f & candidate)
{
  return rigidPoseDifference(
    {
      static_cast<double>(reference(0, 3)),
      static_cast<double>(reference(1, 3)),
      static_cast<double>(reference(2, 3))},
    {
      static_cast<double>(reference(0, 0)),
      static_cast<double>(reference(0, 1)),
      static_cast<double>(reference(0, 2)),
      static_cast<double>(reference(1, 0)),
      static_cast<double>(reference(1, 1)),
      static_cast<double>(reference(1, 2)),
      static_cast<double>(reference(2, 0)),
      static_cast<double>(reference(2, 1)),
      static_cast<double>(reference(2, 2))},
    {
      static_cast<double>(candidate(0, 3)),
      static_cast<double>(candidate(1, 3)),
      static_cast<double>(candidate(2, 3))},
    {
      static_cast<double>(candidate(0, 0)),
      static_cast<double>(candidate(0, 1)),
      static_cast<double>(candidate(0, 2)),
      static_cast<double>(candidate(1, 0)),
      static_cast<double>(candidate(1, 1)),
      static_cast<double>(candidate(1, 2)),
      static_cast<double>(candidate(2, 0)),
      static_cast<double>(candidate(2, 1)),
      static_cast<double>(candidate(2, 2))});
}

StartupPrecisionPose startupPrecisionPose(const Eigen::Matrix4f & transform)
{
  if (!transform.allFinite()) {
    throw std::runtime_error("startup precision pose is non-finite");
  }
  StartupPrecisionPose pose;
  pose.x = static_cast<double>(transform(0, 3));
  pose.y = static_cast<double>(transform(1, 3));
  pose.z = static_cast<double>(transform(2, 3));
  pose.yaw = std::atan2(
    static_cast<double>(transform(1, 0)),
    static_cast<double>(transform(0, 0)));
  return pose;
}

std::pair<double, double> rollPitch(const Eigen::Matrix3f & rotation)
{
  const double pitch = std::asin(std::clamp(-static_cast<double>(rotation(2, 0)), -1.0, 1.0));
  const double roll = std::atan2(rotation(2, 1), rotation(2, 2));
  return {roll, pitch};
}

double secondsBetween(const rclcpp::Time & newer, const rclcpp::Time & older)
{
  return static_cast<double>((newer - older).nanoseconds()) * 1.0e-9;
}

std::int64_t steadyNowNanoseconds()
{
  return static_cast<std::int64_t>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count());
}

std::string stripFileUri(const std::string & input)
{
  constexpr const char * prefix = "file://";
  if (input.rfind(prefix, 0) == 0) {
    return input.substr(std::char_traits<char>::length(prefix));
  }
  return input;
}

}  // namespace

class MapLocalizerNode : public rclcpp::Node
{
public:
  MapLocalizerNode()
  : Node("go2_map_localizer"),
    registration_(declareRegistrationConfig())
  {
    declareRuntimeParameters();
    readRuntimeParameters();
    parameter_callback_handle_ = add_on_set_parameters_callback(
      [](const std::vector<rclcpp::Parameter> &) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = false;
        result.reason =
          "go2_map_localizer parameters are immutable after startup; "
          "update the reviewed YAML and restart";
        return result;
      });

    odom_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    cloud_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    service_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    timer_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions odom_options;
    odom_options.callback_group = odom_group_;
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(rclcpp::KeepLast(300)).best_effort(),
      std::bind(&MapLocalizerNode::odomCallback, this, std::placeholders::_1),
      odom_options);

    rclcpp::SubscriptionOptions cloud_options;
    cloud_options.callback_group = cloud_group_;
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, rclcpp::SensorDataQoS(),
      std::bind(&MapLocalizerNode::cloudCallback, this, std::placeholders::_1),
      cloud_options);

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      pose_topic_, rclcpp::QoS(10).reliable());
    odometry_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      odometry_topic_, rclcpp::QoS(10).reliable());
    status_pub_ = create_publisher<LocalizationStatus>(
      status_topic_, rclcpp::QoS(1).reliable().transient_local());
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    global_service_ = create_service<GlobalRelocalize>(
      global_service_name_,
      std::bind(
        &MapLocalizerNode::globalService, this,
        std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, service_group_);
    load_service_ = create_service<LoadMap>(
      load_service_name_,
      std::bind(
        &MapLocalizerNode::loadService, this,
        std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, service_group_);
    reset_service_ = create_service<ResetLocalization>(
      reset_service_name_,
      std::bind(
        &MapLocalizerNode::resetService, this,
        std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, service_group_);
    activation_service_ = create_service<SetBool>(
      activation_service_name_,
      std::bind(
        &MapLocalizerNode::activationService, this,
        std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, service_group_);

    const auto pose_period = std::chrono::duration<double>(1.0 / pose_publish_rate_hz_);
    pose_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(pose_period),
      std::bind(&MapLocalizerNode::publishPoseAndTf, this), timer_group_);
    const auto status_period = std::chrono::duration<double>(1.0 / status_publish_rate_hz_);
    status_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(status_period),
      std::bind(&MapLocalizerNode::publishStatus, this), timer_group_);

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      state_ = LocalizationStatus::STATE_INITIALIZING;
      state_reason_ = "waiting for map and synchronized LiDAR/odometry";
    }
    if (!map_manifest_parameter_.empty()) {
      std::string error;
      if (!loadMapTransactional(map_manifest_parameter_, verify_hashes_on_startup_, error)) {
        RCLCPP_ERROR(get_logger(), "Map auto-load failed: %s", error.c_str());
      }
    }
    RCLCPP_INFO(
      get_logger(),
      "Go2 map localizer ready: cloud=%s odom=%s corrected_odom=%s "
      "(ROS 2 Foxy / PCL 1.10 baseline)",
      cloud_topic_.c_str(), odom_topic_.c_str(), odometry_topic_.c_str());
  }

private:
  struct OdomSample
  {
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    Eigen::Matrix4f odom_from_body{Eigen::Matrix4f::Identity()};
    geometry_msgs::msg::TwistWithCovariance twist;
  };

  struct TimedScan
  {
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    Eigen::Matrix4f odom_from_body{Eigen::Matrix4f::Identity()};
    Cloud::Ptr cloud;
  };

  struct Candidate
  {
    const DescriptorRecord * record{nullptr};
    double ring_distance{std::numeric_limits<double>::infinity()};
    DescriptorMatch match;
  };

  struct AttemptOutcome
  {
    bool accepted{false};
    bool anchored{false};
    Eigen::Matrix4f map_from_body{Eigen::Matrix4f::Identity()};
    double fitness{std::numeric_limits<double>::infinity()};
    double inlier_ratio{0.0};
    double descriptor_distance{std::numeric_limits<double>::infinity()};
    double correction_translation{0.0};
    double correction_rotation{0.0};
    double confidence{0.0};
    std::string tile_id;
    std::string detail;
  };

  enum class AttemptKind
  {
    kTracking,
    kGlobalInitial,
    kGlobalConfirmation,
  };

  RegistrationConfig declareRegistrationConfig()
  {
    RegistrationConfig config;
    const auto declare_bounded_integer =
      [this](
      const char * name, std::int64_t default_value,
      std::int64_t minimum, std::int64_t maximum) {
        const auto value =
          declare_parameter<std::int64_t>(name, default_value);
        if (value < minimum || value > maximum) {
          throw std::invalid_argument(
                  std::string(name) + " is outside the supported range");
        }
        return value;
      };
    config.source_coarse_voxel_m =
      declare_parameter<double>("registration.source_coarse_voxel_m", 0.8);
    config.source_fine_voxel_m =
      declare_parameter<double>("registration.source_fine_voxel_m", 0.35);
    config.target_coarse_voxel_m =
      declare_parameter<double>("registration.target_coarse_voxel_m", 1.0);
    config.target_fine_voxel_m =
      declare_parameter<double>("registration.target_fine_voxel_m", 0.4);
    config.ndt_coarse_resolution_m =
      declare_parameter<double>("registration.ndt_coarse_resolution_m", 2.0);
    config.ndt_fine_resolution_m =
      declare_parameter<double>("registration.ndt_fine_resolution_m", 0.8);
    config.ndt_step_size_m =
      declare_parameter<double>("registration.ndt_step_size_m", 0.2);
    config.transformation_epsilon =
      declare_parameter<double>("registration.transformation_epsilon", 0.01);
    config.ndt_coarse_iterations = static_cast<int>(
      declare_bounded_integer(
        "registration.ndt_coarse_iterations", 35, 1, 1000));
    config.ndt_fine_iterations = static_cast<int>(
      declare_bounded_integer(
        "registration.ndt_fine_iterations", 25, 1, 1000));
    config.gicp_iterations = static_cast<int>(
      declare_bounded_integer(
        "registration.gicp_iterations", 40, 1, 1000));
    config.gicp_max_correspondence_m =
      declare_parameter<double>("registration.gicp_max_correspondence_m", 1.2);
    config.inlier_distance_m =
      declare_parameter<double>("registration.inlier_distance_m", 0.7);
    config.maximum_source_range_m =
      declare_parameter<double>("registration.maximum_source_range_m", 30.0);
    config.minimum_source_points = static_cast<std::size_t>(
      declare_bounded_integer(
        "registration.minimum_source_points", 300, 1, 10000000));
    config.minimum_target_points = static_cast<std::size_t>(
      declare_bounded_integer(
        "registration.minimum_target_points", 1000, 1, 10000000));
    return config;
  }

  void declareRuntimeParameters()
  {
    declare_parameter<std::string>("map_manifest_path", "");
    declare_parameter<bool>("verify_hashes_on_startup", true);
    declare_parameter<std::string>("cloud_topic", "/cloud_registered_body");
    declare_parameter<std::string>("odom_topic", "/Odometry");
    declare_parameter<std::string>("pose_topic", "/localization/pose");
    declare_parameter<std::string>("odometry_topic", "/localization/odometry");
    declare_parameter<std::string>("status_topic", "/localization/status");
    declare_parameter<std::string>("map_frame", "map");
    declare_parameter<std::string>("odom_frame", "odom");
    declare_parameter<std::string>("base_frame", "base_link");
    declare_parameter<bool>("input_extrinsics_verified", false);
    declare_parameter<std::string>(
      "services.global_relocalize", "/localization/global_relocalize");
    declare_parameter<std::string>("services.load_map", "/localization/load_map");
    declare_parameter<std::string>("services.reset", "/localization/reset");
    declare_parameter<std::string>(
      "services.set_active", "/localization/set_active");
    declare_parameter<double>("services.map_load_quarantine_sec", 2.0);
    declare_parameter<bool>("manual_activation.enabled", false);
    declare_parameter<bool>("manual_activation.start_active", true);
    declare_parameter<double>("sync.max_slop_sec", 0.08);
    declare_parameter<double>("sync.odom_buffer_sec", 3.0);
    declare_parameter<int>("sync.maximum_odom_samples", 1000);
    declare_parameter<double>("sync.maximum_future_offset_sec", 0.10);
    declare_parameter<double>("sync.maximum_input_age_sec", 0.15);
    declare_parameter<int>("accumulation.scan_count", 5);
    declare_parameter<double>("accumulation.max_age_sec", 1.5);
    declare_parameter<double>("accumulation.voxel_m", 0.15);
    declare_parameter<int>("accumulation.minimum_descriptor_points", 800);
    declare_parameter<double>("sensor.minimum_range_m", 1.0);
    declare_parameter<double>("sensor.maximum_range_m", 80.0);
    declare_parameter<double>("sensor.minimum_z_m", -2.5);
    declare_parameter<double>("sensor.maximum_z_m", 5.0);
    declare_parameter<double>("tracking.period_sec", 0.5);
    declare_parameter<bool>("tracking.enabled", true);
    declare_parameter<double>("tracking.local_map_radius_m", 35.0);
    declare_parameter<int>("tracking.maximum_tiles", 12);
    declare_parameter<double>("tracking.maximum_fitness", 0.40);
    declare_parameter<double>("tracking.minimum_inlier_ratio", 0.38);
    declare_parameter<double>("tracking.maximum_translation_correction_m", 0.5);
    declare_parameter<double>(
      "tracking.maximum_rotation_correction_rad", 15.0 * kPiLocal / 180.0);
    declare_parameter<double>("tracking.smoothing_alpha", 0.35);
    declare_parameter<bool>("global.auto_relocalize", true);
    declare_parameter<double>("global.retry_period_sec", 2.0);
    declare_parameter<int>("global.descriptor_top_k", 50);
    declare_parameter<int>("global.registration_candidates", 4);
    declare_parameter<int>("global.maximum_registration_attempts", 36);
    declare_parameter<double>("global.translation_seed_fraction", 0.333333333333);
    declare_parameter<double>("global.maximum_descriptor_distance", 0.75);
    declare_parameter<double>("global.minimum_descriptor_overlap_ratio", 0.05);
    declare_parameter<double>("global.maximum_fitness", 0.8);
    declare_parameter<double>("global.minimum_inlier_ratio", 0.30);
    declare_parameter<double>("global.minimum_hypothesis_confidence_margin", 0.10);
    declare_parameter<double>("global.minimum_hypothesis_relative_margin", 0.20);
    declare_parameter<double>("global.hypothesis_separation_m", 0.25);
    declare_parameter<double>("global.hypothesis_separation_rad", 0.0872665);
    declare_parameter<double>("global.maximum_roll_pitch_rad", 0.45);
    declare_parameter<double>("global.local_map_radius_m", 40.0);
    declare_parameter<int>("global.maximum_tiles", 16);
    declare_parameter<std::vector<double>>(
      "anchored.xy_offsets_m",
      std::vector<double>{0.0, -1.0, 1.0, -2.0, 2.0});
    declare_parameter<std::vector<double>>(
      "anchored.yaw_offsets_rad",
      std::vector<double>{
        0.0,
        -10.0 * kPiLocal / 180.0,
        10.0 * kPiLocal / 180.0,
        -20.0 * kPiLocal / 180.0,
        20.0 * kPiLocal / 180.0,
        -30.0 * kPiLocal / 180.0,
        30.0 * kPiLocal / 180.0});
    declare_parameter<int>("anchored.maximum_registration_attempts", 36);
    declare_parameter<double>("anchored.maximum_translation_correction_m", 3.0);
    declare_parameter<double>(
      "anchored.maximum_rotation_correction_rad", 20.0 * kPiLocal / 180.0);
    declare_parameter<double>("anchored.local_map_radius_m", 40.0);
    declare_parameter<int>("anchored.maximum_tiles", 16);
    declare_parameter<std::vector<double>>(
      "anchored.confirmation_xy_offsets_m",
      std::vector<double>{0.0, -0.25, 0.25});
    declare_parameter<std::vector<double>>(
      "anchored.confirmation_yaw_offsets_rad",
      std::vector<double>{
        0.0,
        -2.0 * kPiLocal / 180.0,
        2.0 * kPiLocal / 180.0});
    declare_parameter<int>("anchored.confirmation_maximum_registration_attempts", 7);
    declare_parameter<double>("anchored.confirmation_search_radius_m", 0.40);
    declare_parameter<double>(
      "anchored.confirmation_maximum_translation_correction_m", 0.50);
    declare_parameter<double>(
      "anchored.confirmation_maximum_rotation_correction_rad",
      5.0 * kPiLocal / 180.0);
    declare_parameter<int>("cache.maximum_tiles", 20);
    declare_parameter<int>("quality.degraded_rejections", 3);
    declare_parameter<int>("quality.lost_rejections", 10);
    declare_parameter<int>("quality.global_confirmation_accepts", 5);
    declare_parameter<double>("quality.global_confirmation_span_sec", 2.0);
    declare_parameter<double>(
      "quality.startup_maximum_translation_deviation_m", 0.10);
    declare_parameter<double>(
      "quality.startup_maximum_yaw_deviation_rad", 0.00523598776);
    declare_parameter<double>("quality.degraded_correction_age_sec", 3.0);
    declare_parameter<double>("quality.lost_correction_age_sec", 10.0);
    declare_parameter<double>("quality.safe_pose_age_sec", 0.15);
    declare_parameter<double>("quality.safe_correction_age_sec", 2.5);
    declare_parameter<double>("quality.safe_minimum_confidence", 0.55);
    declare_parameter<double>("quality.safe_maximum_fitness_rmse_m", 0.40);
    declare_parameter<double>("quality.safe_minimum_inlier_ratio", 0.35);
    declare_parameter<double>("quality.safe_maximum_correction_m", 0.25);
    declare_parameter<double>(
      "quality.safe_maximum_correction_rad", 7.0 * kPiLocal / 180.0);
    declare_parameter<double>("publish.pose_rate_hz", 20.0);
    declare_parameter<double>("publish.status_rate_hz", 20.0);
  }

  void readRuntimeParameters()
  {
    const auto bounded_integer_parameter =
      [this](const char * name, std::int64_t minimum, std::int64_t maximum) {
        const std::int64_t value = get_parameter(name).as_int();
        if (value < minimum || value > maximum) {
          throw std::runtime_error(
                  std::string(name) + " is outside the supported range");
        }
        return value;
      };
    map_manifest_parameter_ = get_parameter("map_manifest_path").as_string();
    verify_hashes_on_startup_ = get_parameter("verify_hashes_on_startup").as_bool();
    cloud_topic_ = get_parameter("cloud_topic").as_string();
    odom_topic_ = get_parameter("odom_topic").as_string();
    pose_topic_ = get_parameter("pose_topic").as_string();
    odometry_topic_ = get_parameter("odometry_topic").as_string();
    status_topic_ = get_parameter("status_topic").as_string();
    map_frame_ = get_parameter("map_frame").as_string();
    odom_frame_ = get_parameter("odom_frame").as_string();
    base_frame_ = get_parameter("base_frame").as_string();
    input_extrinsics_verified_ = get_parameter("input_extrinsics_verified").as_bool();
    global_service_name_ = get_parameter("services.global_relocalize").as_string();
    load_service_name_ = get_parameter("services.load_map").as_string();
    reset_service_name_ = get_parameter("services.reset").as_string();
    activation_service_name_ = get_parameter("services.set_active").as_string();
    map_load_quarantine_sec_ =
      get_parameter("services.map_load_quarantine_sec").as_double();
    manual_activation_enabled_ =
      get_parameter("manual_activation.enabled").as_bool();
    localization_active_.store(
      !manual_activation_enabled_ ||
      get_parameter("manual_activation.start_active").as_bool(),
      std::memory_order_relaxed);
    max_sync_slop_sec_ = get_parameter("sync.max_slop_sec").as_double();
    odom_buffer_sec_ = get_parameter("sync.odom_buffer_sec").as_double();
    maximum_odom_samples_ = static_cast<std::size_t>(
      bounded_integer_parameter("sync.maximum_odom_samples", 2, 1000000));
    maximum_future_offset_sec_ =
      get_parameter("sync.maximum_future_offset_sec").as_double();
    maximum_input_age_sec_ =
      get_parameter("sync.maximum_input_age_sec").as_double();
    accumulation_scan_count_ = static_cast<std::size_t>(
      bounded_integer_parameter("accumulation.scan_count", 1, 1000));
    accumulation_max_age_sec_ = get_parameter("accumulation.max_age_sec").as_double();
    accumulation_voxel_m_ = get_parameter("accumulation.voxel_m").as_double();
    minimum_descriptor_points_ = static_cast<std::size_t>(
      bounded_integer_parameter(
        "accumulation.minimum_descriptor_points", 1, 10000000));
    minimum_range_m_ = get_parameter("sensor.minimum_range_m").as_double();
    maximum_range_m_ = get_parameter("sensor.maximum_range_m").as_double();
    minimum_z_m_ = get_parameter("sensor.minimum_z_m").as_double();
    maximum_z_m_ = get_parameter("sensor.maximum_z_m").as_double();
    tracking_period_sec_ = get_parameter("tracking.period_sec").as_double();
    tracking_enabled_ = get_parameter("tracking.enabled").as_bool();
    tracking_local_map_radius_m_ = get_parameter("tracking.local_map_radius_m").as_double();
    tracking_maximum_tiles_ = static_cast<std::size_t>(
      bounded_integer_parameter("tracking.maximum_tiles", 1, 100000));
    tracking_maximum_fitness_ = get_parameter("tracking.maximum_fitness").as_double();
    tracking_minimum_inlier_ratio_ =
      get_parameter("tracking.minimum_inlier_ratio").as_double();
    tracking_maximum_translation_correction_m_ =
      get_parameter("tracking.maximum_translation_correction_m").as_double();
    tracking_maximum_rotation_correction_rad_ =
      get_parameter("tracking.maximum_rotation_correction_rad").as_double();
    tracking_smoothing_alpha_ = get_parameter("tracking.smoothing_alpha").as_double();
    auto_global_relocalize_ = get_parameter("global.auto_relocalize").as_bool();
    global_retry_period_sec_ = get_parameter("global.retry_period_sec").as_double();
    descriptor_top_k_ = static_cast<std::size_t>(
      bounded_integer_parameter("global.descriptor_top_k", 1, 100000));
    global_registration_candidates_ = static_cast<std::size_t>(
      bounded_integer_parameter("global.registration_candidates", 1, 100000));
    global_maximum_registration_attempts_ = static_cast<std::size_t>(
      bounded_integer_parameter(
        "global.maximum_registration_attempts", 1, 900000));
    global_translation_seed_fraction_ =
      get_parameter("global.translation_seed_fraction").as_double();
    global_maximum_descriptor_distance_ =
      get_parameter("global.maximum_descriptor_distance").as_double();
    global_minimum_descriptor_overlap_ratio_ =
      get_parameter("global.minimum_descriptor_overlap_ratio").as_double();
    global_maximum_fitness_ = get_parameter("global.maximum_fitness").as_double();
    global_minimum_inlier_ratio_ =
      get_parameter("global.minimum_inlier_ratio").as_double();
    global_minimum_hypothesis_confidence_margin_ =
      get_parameter("global.minimum_hypothesis_confidence_margin").as_double();
    global_minimum_hypothesis_relative_margin_ =
      get_parameter("global.minimum_hypothesis_relative_margin").as_double();
    global_hypothesis_separation_m_ =
      get_parameter("global.hypothesis_separation_m").as_double();
    global_hypothesis_separation_rad_ =
      get_parameter("global.hypothesis_separation_rad").as_double();
    global_maximum_roll_pitch_rad_ =
      get_parameter("global.maximum_roll_pitch_rad").as_double();
    global_local_map_radius_m_ = get_parameter("global.local_map_radius_m").as_double();
    global_maximum_tiles_ = static_cast<std::size_t>(
      bounded_integer_parameter("global.maximum_tiles", 1, 100000));
    anchored_xy_offsets_m_ =
      get_parameter("anchored.xy_offsets_m").as_double_array();
    anchored_yaw_offsets_rad_ =
      get_parameter("anchored.yaw_offsets_rad").as_double_array();
    anchored_maximum_registration_attempts_ = static_cast<std::size_t>(
      bounded_integer_parameter(
        "anchored.maximum_registration_attempts", 1, 10000));
    anchored_maximum_translation_correction_m_ =
      get_parameter("anchored.maximum_translation_correction_m").as_double();
    anchored_maximum_rotation_correction_rad_ =
      get_parameter("anchored.maximum_rotation_correction_rad").as_double();
    anchored_local_map_radius_m_ =
      get_parameter("anchored.local_map_radius_m").as_double();
    anchored_maximum_tiles_ = static_cast<std::size_t>(
      bounded_integer_parameter("anchored.maximum_tiles", 1, 100000));
    anchored_confirmation_xy_offsets_m_ =
      get_parameter("anchored.confirmation_xy_offsets_m").as_double_array();
    anchored_confirmation_yaw_offsets_rad_ =
      get_parameter("anchored.confirmation_yaw_offsets_rad").as_double_array();
    anchored_confirmation_maximum_registration_attempts_ =
      static_cast<std::size_t>(
        bounded_integer_parameter(
          "anchored.confirmation_maximum_registration_attempts", 1, 100));
    anchored_confirmation_search_radius_m_ =
      get_parameter("anchored.confirmation_search_radius_m").as_double();
    anchored_confirmation_maximum_translation_correction_m_ =
      get_parameter(
        "anchored.confirmation_maximum_translation_correction_m").as_double();
    anchored_confirmation_maximum_rotation_correction_rad_ =
      get_parameter(
        "anchored.confirmation_maximum_rotation_correction_rad").as_double();
    maximum_cached_tiles_ = static_cast<std::size_t>(
      bounded_integer_parameter("cache.maximum_tiles", 1, 100000));
    degraded_rejections_ = static_cast<std::uint32_t>(
      bounded_integer_parameter("quality.degraded_rejections", 1, 1000000));
    lost_rejections_ = static_cast<std::uint32_t>(
      bounded_integer_parameter("quality.lost_rejections", 2, 1000000));
    global_confirmation_required_accepts_ = static_cast<std::uint32_t>(
      bounded_integer_parameter(
        "quality.global_confirmation_accepts", 2, 1000000));
    global_confirmation_span_sec_ =
      get_parameter("quality.global_confirmation_span_sec").as_double();
    startup_maximum_translation_deviation_m_ =
      get_parameter("quality.startup_maximum_translation_deviation_m").as_double();
    startup_maximum_yaw_deviation_rad_ =
      get_parameter("quality.startup_maximum_yaw_deviation_rad").as_double();
    degraded_correction_age_sec_ =
      get_parameter("quality.degraded_correction_age_sec").as_double();
    lost_correction_age_sec_ =
      get_parameter("quality.lost_correction_age_sec").as_double();
    safe_pose_age_sec_ = get_parameter("quality.safe_pose_age_sec").as_double();
    safe_correction_age_sec_ =
      get_parameter("quality.safe_correction_age_sec").as_double();
    safe_minimum_confidence_ =
      get_parameter("quality.safe_minimum_confidence").as_double();
    safe_maximum_fitness_rmse_m_ =
      get_parameter("quality.safe_maximum_fitness_rmse_m").as_double();
    safe_minimum_inlier_ratio_ =
      get_parameter("quality.safe_minimum_inlier_ratio").as_double();
    safe_maximum_correction_m_ =
      get_parameter("quality.safe_maximum_correction_m").as_double();
    safe_maximum_correction_rad_ =
      get_parameter("quality.safe_maximum_correction_rad").as_double();
    pose_publish_rate_hz_ = get_parameter("publish.pose_rate_hz").as_double();
    status_publish_rate_hz_ = get_parameter("publish.status_rate_hz").as_double();

    const std::array runtime_scalars{
      map_load_quarantine_sec_,
      max_sync_slop_sec_,
      odom_buffer_sec_,
      maximum_future_offset_sec_,
      maximum_input_age_sec_,
      accumulation_max_age_sec_,
      accumulation_voxel_m_,
      minimum_range_m_,
      maximum_range_m_,
      minimum_z_m_,
      maximum_z_m_,
      tracking_period_sec_,
      tracking_local_map_radius_m_,
      tracking_maximum_fitness_,
      tracking_minimum_inlier_ratio_,
      tracking_maximum_translation_correction_m_,
      tracking_maximum_rotation_correction_rad_,
      tracking_smoothing_alpha_,
      global_retry_period_sec_,
      global_translation_seed_fraction_,
      global_maximum_descriptor_distance_,
      global_minimum_descriptor_overlap_ratio_,
      global_maximum_fitness_,
      global_minimum_inlier_ratio_,
      global_minimum_hypothesis_confidence_margin_,
      global_minimum_hypothesis_relative_margin_,
      global_hypothesis_separation_m_,
      global_hypothesis_separation_rad_,
      global_maximum_roll_pitch_rad_,
      global_local_map_radius_m_,
      anchored_maximum_translation_correction_m_,
      anchored_maximum_rotation_correction_rad_,
      anchored_local_map_radius_m_,
      anchored_confirmation_search_radius_m_,
      anchored_confirmation_maximum_translation_correction_m_,
      anchored_confirmation_maximum_rotation_correction_rad_,
      global_confirmation_span_sec_,
      startup_maximum_translation_deviation_m_,
      startup_maximum_yaw_deviation_rad_,
      degraded_correction_age_sec_,
      lost_correction_age_sec_,
      safe_pose_age_sec_,
      safe_correction_age_sec_,
      safe_minimum_confidence_,
      safe_maximum_fitness_rmse_m_,
      safe_minimum_inlier_ratio_,
      safe_maximum_correction_m_,
      safe_maximum_correction_rad_,
      pose_publish_rate_hz_,
      status_publish_rate_hz_};
    const bool all_runtime_scalars_finite = std::all_of(
      runtime_scalars.begin(), runtime_scalars.end(),
      [](double value) {return std::isfinite(value);});

    if (!all_runtime_scalars_finite ||
      map_frame_.empty() || odom_frame_.empty() || base_frame_.empty() ||
      pose_topic_.empty() || odometry_topic_.empty() ||
      odometry_topic_ == odom_topic_ ||
      lost_rejections_ <= degraded_rejections_ ||
      global_registration_candidates_ > descriptor_top_k_ ||
      global_maximum_registration_attempts_ >
      global_registration_candidates_ * 9 ||
      max_sync_slop_sec_ <= 0.0 || odom_buffer_sec_ <= max_sync_slop_sec_ ||
      maximum_future_offset_sec_ < 0.0 ||
      maximum_input_age_sec_ <= max_sync_slop_sec_ ||
      map_load_quarantine_sec_ < 2.0 ||
      accumulation_max_age_sec_ <= 0.0 || accumulation_voxel_m_ <= 0.0 ||
      tracking_period_sec_ <= 0.0 || global_retry_period_sec_ <= 0.0 ||
      tracking_local_map_radius_m_ <= 0.0 || global_local_map_radius_m_ <= 0.0 ||
      tracking_smoothing_alpha_ <= 0.0 || tracking_smoothing_alpha_ > 1.0 ||
      global_translation_seed_fraction_ < 0.0 ||
      global_translation_seed_fraction_ >= 0.5 ||
      global_maximum_descriptor_distance_ <= 0.0 ||
      global_minimum_descriptor_overlap_ratio_ <= 0.0 ||
      global_minimum_descriptor_overlap_ratio_ > 1.0 ||
      minimum_range_m_ < 0.0 || maximum_range_m_ <= minimum_range_m_ ||
      minimum_z_m_ >= maximum_z_m_ || tracking_maximum_fitness_ <= 0.0 ||
      global_maximum_fitness_ <= 0.0 || tracking_minimum_inlier_ratio_ < 0.0 ||
      tracking_minimum_inlier_ratio_ > 1.0 || global_minimum_inlier_ratio_ < 0.0 ||
      global_minimum_inlier_ratio_ > 1.0 ||
      global_minimum_hypothesis_confidence_margin_ < 0.0 ||
      global_minimum_hypothesis_confidence_margin_ > 1.0 ||
      global_minimum_hypothesis_relative_margin_ < 0.0 ||
      global_minimum_hypothesis_relative_margin_ > 1.0 ||
      global_hypothesis_separation_m_ <= 0.0 ||
      global_hypothesis_separation_rad_ <= 0.0 ||
      global_maximum_roll_pitch_rad_ <= 0.0 ||
      anchored_maximum_translation_correction_m_ <= 0.0 ||
      anchored_maximum_rotation_correction_rad_ <= 0.0 ||
      anchored_maximum_rotation_correction_rad_ > kPiLocal ||
      anchored_local_map_radius_m_ <= 0.0 ||
      anchored_xy_offsets_m_.size() > 64 ||
      anchored_yaw_offsets_rad_.size() > 64 ||
      anchored_confirmation_xy_offsets_m_.size() > 16 ||
      anchored_confirmation_yaw_offsets_rad_.size() > 16 ||
      std::any_of(
        anchored_xy_offsets_m_.begin(), anchored_xy_offsets_m_.end(),
        [](double value) {return !std::isfinite(value) || std::abs(value) > 20.0;}) ||
      std::any_of(
        anchored_yaw_offsets_rad_.begin(), anchored_yaw_offsets_rad_.end(),
        [](double value) {return !std::isfinite(value) || std::abs(value) > kPiLocal;}) ||
      std::any_of(
        anchored_confirmation_xy_offsets_m_.begin(),
        anchored_confirmation_xy_offsets_m_.end(),
        [](double value) {return !std::isfinite(value) || std::abs(value) > 2.0;}) ||
      std::any_of(
        anchored_confirmation_yaw_offsets_rad_.begin(),
        anchored_confirmation_yaw_offsets_rad_.end(),
        [](double value) {return !std::isfinite(value) || std::abs(value) > 0.5;}) ||
      anchored_confirmation_search_radius_m_ <= 0.0 ||
      anchored_confirmation_maximum_translation_correction_m_ <= 0.0 ||
      anchored_confirmation_maximum_rotation_correction_rad_ <= 0.0 ||
      anchored_confirmation_maximum_rotation_correction_rad_ > kPiLocal ||
      tracking_maximum_translation_correction_m_ <= 0.0 ||
      tracking_maximum_rotation_correction_rad_ <= 0.0 ||
      degraded_correction_age_sec_ <= 0.0 ||
      lost_correction_age_sec_ <= degraded_correction_age_sec_ ||
      global_confirmation_span_sec_ <= 0.0 ||
      startup_maximum_translation_deviation_m_ <= 0.0 ||
      startup_maximum_yaw_deviation_rad_ <= 0.0 ||
      startup_maximum_yaw_deviation_rad_ > kPiLocal ||
      safe_pose_age_sec_ <= 0.0 || safe_correction_age_sec_ <= 0.0 ||
      safe_minimum_confidence_ < 0.0 || safe_minimum_confidence_ > 1.0 ||
      safe_maximum_fitness_rmse_m_ <= 0.0 ||
      safe_minimum_inlier_ratio_ < 0.0 || safe_minimum_inlier_ratio_ > 1.0 ||
      safe_maximum_correction_m_ <= 0.0 || safe_maximum_correction_rad_ <= 0.0 ||
      pose_publish_rate_hz_ < 0.1 || status_publish_rate_hz_ < 0.1)
    {
      throw std::runtime_error("invalid go2_map_localizer runtime parameters");
    }
    try {
      static_cast<void>(
        makeAnchoredSeeds(
          0.0, 0.0, 0.0,
          anchored_xy_offsets_m_, anchored_yaw_offsets_rad_,
          100.0, anchored_maximum_registration_attempts_));
      static_cast<void>(
        makeAnchoredSeeds(
          0.0, 0.0, 0.0,
          anchored_confirmation_xy_offsets_m_,
          anchored_confirmation_yaw_offsets_rad_,
          anchored_confirmation_search_radius_m_,
          anchored_confirmation_maximum_registration_attempts_));
    } catch (const std::invalid_argument & error) {
      throw std::runtime_error(
              std::string("invalid anchored seed parameters: ") + error.what());
    }
    confirmation_gate_ = ConfirmationGate(
      global_confirmation_required_accepts_,
      static_cast<std::int64_t>(std::llround(global_confirmation_span_sec_ * 1.0e9)));
    startup_precision_gate_ = StartupPrecisionGate(
      startup_maximum_translation_deviation_m_,
      startup_maximum_yaw_deviation_rad_);
    map_load_quarantine_ = MapLoadQuarantine(
      static_cast<std::int64_t>(std::llround(map_load_quarantine_sec_ * 1.0e9)));
  }

  bool inputTimestampIsFresh(
    const rclcpp::Time & stamp,
    const rclcpp::Time & current_time,
    const char * source)
  {
    const double age = secondsBetween(current_time, stamp);
    if (timestampAgeIsAcceptable(
        age, maximum_future_offset_sec_, maximum_input_age_sec_))
    {
      return true;
    }
    if (invalid_timestamp_count_.fetch_add(1, std::memory_order_relaxed) % 100 == 0) {
      RCLCPP_WARN(
        get_logger(),
        "Rejecting %s timestamp: age=%.3fs, accepted range=[-%.3f, %.3f]s",
        source, age, maximum_future_offset_sec_, maximum_input_age_sec_);
    }
    return false;
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    if (message->header.frame_id != odom_frame_) {
      if (invalid_odom_frame_count_.fetch_add(1, std::memory_order_relaxed) % 100 == 0) {
        RCLCPP_WARN(
          get_logger(), "Rejecting odometry frame '%s'; expected '%s'",
          message->header.frame_id.c_str(), odom_frame_.c_str());
      }
      return;
    }
    if (message->child_frame_id != base_frame_) {
      if (invalid_odom_frame_count_.fetch_add(1, std::memory_order_relaxed) % 100 == 0) {
        RCLCPP_WARN(
          get_logger(), "Rejecting odometry child frame '%s'; expected '%s'",
          message->child_frame_id.c_str(), base_frame_.c_str());
      }
      return;
    }
    OdomSample sample;
    sample.stamp = rclcpp::Time(message->header.stamp);
    const rclcpp::Time current_time = now();
    if (!inputTimestampIsFresh(sample.stamp, current_time, "odometry")) {
      return;
    }
    try {
      sample.odom_from_body = poseToMatrix(message->pose.pose);
      sample.twist = message->twist;
    } catch (const std::exception & error) {
      RCLCPP_WARN(get_logger(), "Rejecting invalid odometry: %s", error.what());
      return;
    }
    {
      bool clock_epoch_reset = false;
      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (!odom_buffer_.empty() &&
          sample.stamp.nanoseconds() <= odom_buffer_.back().stamp.nanoseconds())
        {
          const double buffered_future_age =
            secondsBetween(odom_buffer_.back().stamp, current_time);
          if (buffered_future_age > maximum_future_offset_sec_) {
            odom_buffer_.clear();
            scan_buffer_.clear();
            latest_source_.reset();
            latest_odom_ = OdomSample();
            last_cloud_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
            clock_epoch_reset = true;
          } else {
            if (nonmonotonic_odom_count_.fetch_add(
                1, std::memory_order_relaxed) % 100 == 0)
            {
              RCLCPP_WARN(
                get_logger(),
                "Rejecting non-increasing odometry timestamp; pairing buffer is unchanged");
            }
            return;
          }
        }
        odom_buffer_.push_back(sample);
        while (odom_buffer_.size() > 2 &&
          secondsBetween(
            odom_buffer_.back().stamp,
            odom_buffer_.front().stamp) > odom_buffer_sec_)
        {
          odom_buffer_.pop_front();
        }
        while (odom_buffer_.size() > maximum_odom_samples_) {
          odom_buffer_.pop_front();
        }
      }
      if (clock_epoch_reset) {
        {
          std::lock_guard<std::mutex> lock(state_mutex_);
          pose_valid_ = false;
          provisional_pose_valid_ = false;
          confirmation_gate_.reset();
          startup_precision_gate_.reset();
          provisional_anchor_map_from_odom_ = Eigen::Matrix4f::Identity();
          state_ = LocalizationStatus::STATE_LOST;
          state_reason_ = "ROS clock epoch changed; localization invalidated";
          last_attempt_detail_ = state_reason_;
          last_pose_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
          last_correction_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
          last_attempt_completion_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
        }
        publishStatus();
      }
    }
  }

  std::optional<OdomSample> pairedOdom(const rclcpp::Time & stamp)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (odom_buffer_.empty()) {
      return std::nullopt;
    }
    auto upper = std::lower_bound(
      odom_buffer_.begin(), odom_buffer_.end(), stamp,
      [](const OdomSample & sample, const rclcpp::Time & target) {
        return sample.stamp < target;
      });
    if (upper == odom_buffer_.begin()) {
      if (std::abs(secondsBetween(upper->stamp, stamp)) <= max_sync_slop_sec_) {
        return *upper;
      }
      return std::nullopt;
    }
    if (upper == odom_buffer_.end()) {
      const auto & nearest = odom_buffer_.back();
      if (std::abs(secondsBetween(stamp, nearest.stamp)) <= max_sync_slop_sec_) {
        return nearest;
      }
      return std::nullopt;
    }
    const auto lower_sample = std::prev(upper);
    const double before = secondsBetween(stamp, lower_sample->stamp);
    const double after = secondsBetween(upper->stamp, stamp);
    if (before < -1.0e-6 || after < -1.0e-6 ||
      std::min(before, after) > max_sync_slop_sec_)
    {
      return std::nullopt;
    }
    const double span = before + after;
    OdomSample interpolated;
    interpolated.stamp = stamp;
    interpolated.odom_from_body = span <= 1.0e-9 ? lower_sample->odom_from_body :
      interpolateTransform(lower_sample->odom_from_body, upper->odom_from_body, before / span);
    interpolated.twist = before <= after ? lower_sample->twist : upper->twist;
    return interpolated;
  }

  Cloud::Ptr convertAndFilterCloud(const sensor_msgs::msg::PointCloud2 & message) const
  {
    Cloud::Ptr raw(new Cloud());
    pcl::fromROSMsg(message, *raw);
    Cloud::Ptr filtered(new Cloud());
    filtered->reserve(raw->size());
    for (const auto & point : raw->points) {
      if (!pcl::isFinite(point)) {
        continue;
      }
      const double range = std::hypot(point.x, point.y);
      if (range < minimum_range_m_ || range > maximum_range_m_ ||
        point.z < minimum_z_m_ || point.z > maximum_z_m_)
      {
        continue;
      }
      filtered->push_back(point);
    }
    filtered->width = static_cast<std::uint32_t>(filtered->size());
    filtered->height = 1;
    filtered->is_dense = true;
    return filtered;
  }

  Cloud::Ptr addAndAggregateScan(
    const rclcpp::Time & stamp,
    const OdomSample & current_odom,
    const Cloud::Ptr & cloud)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    last_cloud_stamp_ = stamp;
    scan_buffer_.push_back({stamp, current_odom.odom_from_body, cloud});
    while (scan_buffer_.size() > accumulation_scan_count_ ||
      (!scan_buffer_.empty() &&
      secondsBetween(stamp, scan_buffer_.front().stamp) > accumulation_max_age_sec_))
    {
      scan_buffer_.pop_front();
    }
    Cloud::Ptr accumulated(new Cloud());
    const Eigen::Matrix4f current_body_from_odom =
      current_odom.odom_from_body.inverse();
    for (const auto & scan : scan_buffer_) {
      Cloud transformed;
      pcl::transformPointCloud(
        *scan.cloud, transformed, current_body_from_odom * scan.odom_from_body);
      *accumulated += transformed;
    }
    if (accumulation_voxel_m_ > 0.0 && !accumulated->empty()) {
      Cloud::Ptr downsampled(new Cloud());
      pcl::VoxelGrid<pcl::PointXYZ> filter;
      const auto leaf = static_cast<float>(accumulation_voxel_m_);
      filter.setLeafSize(leaf, leaf, leaf);
      filter.setInputCloud(accumulated);
      filter.filter(*downsampled);
      accumulated = downsampled;
    }
    latest_source_ = accumulated;
    latest_odom_ = current_odom;
    latest_odom_.stamp = stamp;
    return accumulated;
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr message)
  {
    // Checkpoint mode keeps only the inexpensive odometry buffer alive while
    // the CSV follower is moving. Point conversion, scan accumulation, place
    // retrieval, NDT and GICP are all skipped until the coordinator explicitly
    // activates localization at startup or at a reviewed checkpoint.
    if (manual_activation_enabled_ &&
      !localization_active_.load(std::memory_order_relaxed))
    {
      return;
    }
    if (message->header.frame_id != base_frame_) {
      if (invalid_cloud_frame_count_.fetch_add(1, std::memory_order_relaxed) % 100 == 0) {
        RCLCPP_WARN(
          get_logger(), "Rejecting cloud frame '%s'; expected '%s'",
          message->header.frame_id.c_str(), base_frame_.c_str());
      }
      return;
    }
    const rclcpp::Time stamp(message->header.stamp);
    if (!inputTimestampIsFresh(stamp, now(), "point cloud")) {
      return;
    }
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      if (last_cloud_stamp_.nanoseconds() != 0 &&
        stamp.nanoseconds() <= last_cloud_stamp_.nanoseconds())
      {
        if (nonmonotonic_cloud_count_.fetch_add(
            1, std::memory_order_relaxed) % 100 == 0)
        {
          RCLCPP_WARN(
            get_logger(),
            "Rejecting non-increasing point-cloud timestamp");
        }
        return;
      }
    }
    const auto odom = pairedOdom(stamp);
    if (!odom) {
      synchronized_drop_count_.fetch_add(1, std::memory_order_relaxed);
      return;
    }
    Cloud::Ptr cloud;
    try {
      cloud = convertAndFilterCloud(*message);
    } catch (const std::exception & error) {
      RCLCPP_WARN(get_logger(), "Point cloud conversion failed: %s", error.what());
      return;
    }
    if (cloud->size() < 100) {
      return;
    }
    const auto source = addAndAggregateScan(stamp, *odom, cloud);
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      last_pose_stamp_ = stamp;
    }
    attemptAutomaticLocalization(source, *odom);
  }

  void attemptAutomaticLocalization(const Cloud::Ptr & source, const OdomSample & odom)
  {
    const auto now_steady = std::chrono::steady_clock::now();
    bool tracking = false;
    bool provisional = false;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (map_load_quarantine_.armed()) {
        return;
      }
      tracking = pose_valid_;
      provisional = provisional_pose_valid_ && confirmation_gate_.pending();
    }
    const double required_period = tracking ? tracking_period_sec_ : global_retry_period_sec_;
    if (std::chrono::duration<double>(now_steady - last_attempt_steady_).count() <
      required_period)
    {
      return;
    }
    std::unique_lock<std::mutex> attempt_lock(registration_mutex_, std::try_to_lock);
    if (!attempt_lock.owns_lock()) {
      return;
    }
    last_attempt_steady_ = now_steady;
    std::shared_ptr<const MapManifest> map;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      map = map_;
    }
    if (!input_extrinsics_verified_ || !map ||
      source->size() < minimum_descriptor_points_)
    {
      return;
    }

    AttemptOutcome outcome;
    AttemptKind kind = AttemptKind::kGlobalInitial;
    if (tracking && tracking_enabled_) {
      outcome = attemptTracking(*map, source, odom);
      kind = AttemptKind::kTracking;
    } else if (tracking) {
      return;
    } else if (auto_global_relocalize_) {
      outcome = attemptGlobal(*map, source, odom, std::nullopt, 0.0);
      kind = provisional ?
        AttemptKind::kGlobalConfirmation : AttemptKind::kGlobalInitial;
    } else {
      return;
    }
    recordOutcome(outcome, odom, kind);
  }

  Cloud::Ptr loadTile(const TileRecord & tile, bool verify_hash)
  {
    const auto cached = tile_cache_.find(tile.id);
    if (cached != tile_cache_.end()) {
      tile_lru_.remove(tile.id);
      tile_lru_.push_front(tile.id);
      return cached->second;
    }
    if (verify_hash && sha256File(tile.path) != tile.sha256) {
      throw std::runtime_error(
              "tile changed after map activation: " + tile.id);
    }
    Cloud::Ptr cloud(new Cloud());
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(tile.path.string(), *cloud) < 0) {
      throw std::runtime_error("PCL failed to load tile " + tile.path.string());
    }
    Cloud::Ptr finite_cloud(new Cloud());
    std::vector<int> kept_indices;
    pcl::removeNaNFromPointCloud(*cloud, *finite_cloud, kept_indices);
    cloud = finite_cloud;
    if (cloud->size() != tile.point_count) {
      throw std::runtime_error(
              "tile point count differs from manifest: " + tile.id +
              " expected=" + std::to_string(tile.point_count) +
              " actual=" + std::to_string(cloud->size()));
    }
    if (verify_hash && sha256File(tile.path) != tile.sha256) {
      throw std::runtime_error(
              "tile changed while being loaded: " + tile.id);
    }
    tile_cache_[tile.id] = cloud;
    tile_lru_.push_front(tile.id);
    while (tile_cache_.size() > maximum_cached_tiles_) {
      const auto evicted = tile_lru_.back();
      tile_lru_.pop_back();
      tile_cache_.erase(evicted);
    }
    return cloud;
  }

  Cloud::Ptr buildTarget(
    const MapManifest & map,
    double x, double y, double radius,
    std::size_t maximum_tiles,
    std::string & closest_tile)
  {
    const auto tiles = map.tilesNear(x, y, radius, maximum_tiles);
    if (tiles.empty()) {
      throw std::runtime_error("no map tiles overlap registration search area");
    }
    closest_tile = tiles.front()->id;
    Cloud::Ptr combined(new Cloud());
    for (const auto * tile : tiles) {
      *combined += *loadTile(*tile, map.hashes_verified);
    }
    combined->width = static_cast<std::uint32_t>(combined->size());
    combined->height = 1;
    combined->is_dense = true;
    return combined;
  }

  AttemptOutcome evaluateRegistration(
    const RegistrationResult & registration,
    const Eigen::Matrix4f & prediction,
    bool global,
    bool require_descriptor_evidence,
    double descriptor_distance,
    const std::string & tile_id) const
  {
    AttemptOutcome outcome;
    outcome.map_from_body = registration.map_from_body;
    outcome.fitness = registration.fitness;
    outcome.inlier_ratio = registration.inlier_ratio;
    outcome.descriptor_distance = descriptor_distance;
    outcome.tile_id = tile_id;
    outcome.detail = registration.detail;
    if (!registration.converged) {
      return outcome;
    }
    if (global && require_descriptor_evidence &&
      (!std::isfinite(descriptor_distance) || descriptor_distance < 0.0 ||
      descriptor_distance > global_maximum_descriptor_distance_))
    {
      outcome.detail = "global registration lacks valid place-descriptor evidence";
      return outcome;
    }
    const auto correction = matrixPoseDifference(prediction, registration.map_from_body);
    outcome.correction_translation = correction.translation_m;
    outcome.correction_rotation = correction.rotation_rad;
    const auto [roll, pitch] = rollPitch(registration.map_from_body.block<3, 3>(0, 0));
    const double maximum_fitness =
      global ? global_maximum_fitness_ : tracking_maximum_fitness_;
    const double minimum_inlier =
      global ? global_minimum_inlier_ratio_ : tracking_minimum_inlier_ratio_;
    if (registration.fitness > maximum_fitness) {
      outcome.detail = "fitness gate rejected: " + std::to_string(registration.fitness);
      return outcome;
    }
    if (registration.inlier_ratio < minimum_inlier) {
      outcome.detail = "inlier gate rejected: " + std::to_string(registration.inlier_ratio);
      return outcome;
    }
    if (std::abs(roll) > global_maximum_roll_pitch_rad_ ||
      std::abs(pitch) > global_maximum_roll_pitch_rad_)
    {
      outcome.detail = "roll/pitch plausibility gate rejected";
      return outcome;
    }
    if (!global &&
      (outcome.correction_translation > tracking_maximum_translation_correction_m_ ||
      outcome.correction_rotation > tracking_maximum_rotation_correction_rad_))
    {
      outcome.detail = "tracking correction jump gate rejected";
      return outcome;
    }
    outcome.confidence = registrationConfidence(
      registration.inlier_ratio, registration.fitness,
      maximum_fitness, descriptor_distance);
    outcome.accepted = true;
    return outcome;
  }

  AttemptOutcome evaluateAnchoredRegistration(
    const RegistrationResult & registration,
    const Eigen::Matrix4f & prediction,
    const std::string & tile_id,
    double maximum_translation_correction_m,
    double maximum_rotation_correction_rad,
    const char * accepted_detail) const
  {
    auto outcome = evaluateRegistration(
      registration, prediction, true, false,
      std::numeric_limits<double>::infinity(), tile_id);
    outcome.anchored = true;
    if (!outcome.accepted) {
      return outcome;
    }
    outcome.confidence = descriptorlessRegistrationConfidence(
      registration.inlier_ratio, registration.fitness,
      safe_maximum_fitness_rmse_m_);
    if (!registrationQualityIsSafe(
        registration.fitness, registration.inlier_ratio,
        safe_maximum_fitness_rmse_m_, safe_minimum_inlier_ratio_))
    {
      outcome.accepted = false;
      outcome.detail =
        "anchored registration did not meet motion-handoff RMSE/inlier thresholds";
      return outcome;
    }
    if (outcome.confidence < safe_minimum_confidence_) {
      outcome.accepted = false;
      outcome.detail =
        "anchored registration confidence is below motion-handoff threshold";
      return outcome;
    }
    if (!correctionWithinEnvelope(
        outcome.correction_translation,
        outcome.correction_rotation,
        maximum_translation_correction_m,
        maximum_rotation_correction_rad))
    {
      outcome.accepted = false;
      outcome.detail =
        "anchored registration correction exceeded seed envelope";
      return outcome;
    }
    outcome.detail = accepted_detail;
    return outcome;
  }

  AttemptOutcome attemptTracking(
    const MapManifest & map,
    const Cloud::Ptr & source,
    const OdomSample & odom)
  {
    Eigen::Matrix4f map_from_odom;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      map_from_odom = map_from_odom_;
    }
    const Eigen::Matrix4f prediction = map_from_odom * odom.odom_from_body;
    std::string tile_id;
    try {
      const auto target = buildTarget(
        map, prediction(0, 3), prediction(1, 3),
        tracking_local_map_radius_m_, tracking_maximum_tiles_, tile_id);
      const auto registration = registration_.align(source, target, prediction);
      return evaluateRegistration(
        registration, prediction, false, false,
        std::numeric_limits<double>::infinity(), tile_id);
    } catch (const std::exception & error) {
      AttemptOutcome outcome;
      outcome.detail = error.what();
      return outcome;
    }
  }

  std::vector<Candidate> descriptorCandidates(
    const MapManifest & map,
    const Cloud::Ptr & source,
    const Eigen::Matrix4f & odom_from_body,
    const std::optional<Eigen::Matrix4f> & initial_guess,
    double search_radius) const
  {
    const Eigen::Matrix3f odom_rotation = odom_from_body.block<3, 3>(0, 0);
    const float odom_yaw = std::atan2(odom_rotation(1, 0), odom_rotation(0, 0));
    const Eigen::Matrix3f odom_from_level =
      Eigen::AngleAxisf(odom_yaw, Eigen::Vector3f::UnitZ()).toRotationMatrix();
    const Eigen::Matrix3f level_from_body =
      odom_from_level.transpose() * odom_rotation;
    std::vector<Point3> points;
    points.reserve(source->size());
    for (const auto & point : source->points) {
      const Eigen::Vector3f level_point =
        level_from_body * Eigen::Vector3f(point.x, point.y, point.z);
      points.push_back({level_point.x(), level_point.y(), level_point.z()});
    }
    const auto query = buildPolarDescriptor(points, map.descriptor_config);
    std::vector<Candidate> candidates;
    candidates.reserve(map.descriptors.size());
    for (const auto & record : map.descriptors) {
      if (initial_guess && search_radius > 0.0) {
        const double dx = record.center[0] - (*initial_guess)(0, 3);
        const double dy = record.center[1] - (*initial_guess)(1, 3);
        if (std::hypot(dx, dy) > search_radius) {
          continue;
        }
      }
      Candidate candidate;
      candidate.record = &record;
      candidate.ring_distance =
        normalizedKeyDistance(record.descriptor.ring_key, query.ring_key);
      candidates.push_back(candidate);
    }
    std::sort(candidates.begin(), candidates.end(), [](const auto & lhs, const auto & rhs) {
        return lhs.ring_distance < rhs.ring_distance;
      });
    if (candidates.size() > descriptor_top_k_) {
      candidates.resize(descriptor_top_k_);
    }
    for (auto & candidate : candidates) {
      candidate.match = matchPolarDescriptors(
        candidate.record->descriptor, query,
        global_minimum_descriptor_overlap_ratio_);
    }
    std::sort(candidates.begin(), candidates.end(), [](const auto & lhs, const auto & rhs) {
        return lhs.match.distance < rhs.match.distance;
      });
    return candidates;
  }

  AttemptOutcome attemptGlobal(
    const MapManifest & map,
    const Cloud::Ptr & source,
    const OdomSample & odom,
    const std::optional<Eigen::Matrix4f> & initial_guess,
    double search_radius)
  {
    AttemptOutcome best;
    auto candidates = descriptorCandidates(
      map, source, odom.odom_from_body, initial_guess, search_radius);
    if (candidates.empty()) {
      best.detail = "descriptor search found no candidates";
      return best;
    }
    candidates.erase(
      std::remove_if(
        candidates.begin(), candidates.end(),
        [this](const Candidate & candidate) {
          return !std::isfinite(candidate.match.distance) ||
                 candidate.match.distance > global_maximum_descriptor_distance_;
        }),
      candidates.end());
    if (candidates.size() > global_registration_candidates_) {
      candidates.resize(global_registration_candidates_);
    }
    if (candidates.empty()) {
      best.detail = "all descriptor candidates failed descriptor quality gate";
      return best;
    }

    struct Hypothesis
    {
      const Candidate * candidate;
      Eigen::Matrix4f guess;
    };
    std::vector<Hypothesis> hypotheses;
    const std::vector<std::array<double, 2>> cardinal_offsets{
      {-1.0, 0.0}, {1.0, 0.0}, {0.0, -1.0}, {0.0, 1.0}};
    const std::vector<std::array<double, 2>> diagonal_offsets{
      {-1.0, -1.0}, {-1.0, 1.0}, {1.0, -1.0}, {1.0, 1.0}};
    const auto append_hypothesis =
      [&hypotheses, &map, this](
      const Candidate & candidate, const std::array<double, 2> & offset) {
        const auto & tile = map.tile(candidate.record->tile_id);
        const double width = tile.grid_max[0] - tile.grid_min[0];
        const double height = tile.grid_max[1] - tile.grid_min[1];
        Eigen::Matrix4f guess = Eigen::Matrix4f::Identity();
        const Eigen::AngleAxisf yaw(
          static_cast<float>(candidate.match.yaw_rad), Eigen::Vector3f::UnitZ());
        guess.block<3, 3>(0, 0) = yaw.toRotationMatrix();
        guess(0, 3) = static_cast<float>(
          candidate.record->center[0] +
          offset[0] * global_translation_seed_fraction_ * width);
        guess(1, 3) = static_cast<float>(
          candidate.record->center[1] +
          offset[1] * global_translation_seed_fraction_ * height);
        guess(2, 3) = static_cast<float>(candidate.record->center[2]);
        hypotheses.push_back({&candidate, guess});
      };
    if (initial_guess && !candidates.empty()) {
      // A manual pose is only a bounded search hint. It must still carry
      // descriptor evidence and compete with alternative global hypotheses.
      Eigen::Matrix4f seeded_guess = *initial_guess;
      const Eigen::AngleAxisf yaw(
        static_cast<float>(candidates.front().match.yaw_rad),
        Eigen::Vector3f::UnitZ());
      seeded_guess.block<3, 3>(0, 0) = yaw.toRotationMatrix();
      hypotheses.push_back({&candidates.front(), seeded_guess});
    }
    // First test every retrieved place at its center, then all cardinal and
    // diagonal seeds. The default 36-attempt budget fully covers this 3x3
    // lattice for all four registration candidates.
    for (const auto & candidate : candidates) {
      append_hypothesis(candidate, {0.0, 0.0});
    }
    for (const auto & candidate : candidates) {
      for (const auto & offset : cardinal_offsets) {
        append_hypothesis(candidate, offset);
      }
    }
    for (const auto & candidate : candidates) {
      for (const auto & offset : diagonal_offsets) {
        append_hypothesis(candidate, offset);
      }
    }

    AttemptOutcome runner_up;
    std::size_t attempted = 0;
    for (const auto & hypothesis : hypotheses) {
      const auto & candidate = *hypothesis.candidate;
      if (!std::isfinite(candidate.match.distance) ||
        candidate.match.distance > global_maximum_descriptor_distance_)
      {
        continue;
      }
      const Eigen::Matrix4f & guess = hypothesis.guess;
      std::string tile_id;
      try {
        const auto target = buildTarget(
          map, guess(0, 3), guess(1, 3),
          global_local_map_radius_m_, global_maximum_tiles_, tile_id);
        const auto registration = registration_.align(source, target, guess);
        auto outcome = evaluateRegistration(
          registration, guess, true, true, candidate.match.distance, tile_id);
        if (outcome.accepted && !best.accepted) {
          best = outcome;
        } else if (outcome.accepted) {
          const auto hypothesis_delta =
            matrixPoseDifference(best.map_from_body, outcome.map_from_body);
          const bool distinct_hypothesis =
            hypothesesAreDistinct(
              hypothesis_delta.translation_m,
              hypothesis_delta.rotation_rad,
              global_hypothesis_separation_m_,
              global_hypothesis_separation_rad_);
          if (!distinct_hypothesis) {
            if (outcome.confidence > best.confidence) {
              best = outcome;
            }
          } else if (outcome.confidence > best.confidence) {
            runner_up = best;
            best = outcome;
          } else if (!runner_up.accepted ||
            outcome.confidence > runner_up.confidence)
          {
            runner_up = outcome;
          }
        } else if (!best.accepted && outcome.fitness < best.fitness) {
          best = outcome;
        }
      } catch (const std::exception & error) {
        if (!best.accepted) {
          best.detail = error.what();
        }
      }
      if (++attempted >= global_maximum_registration_attempts_) {
        break;
      }
    }
    if (attempted == 0) {
      best.detail = "all descriptor candidates failed descriptor quality gate";
    }
    if (best.accepted && runner_up.accepted &&
      !passesHypothesisMargin(
        best.confidence, runner_up.confidence,
        global_minimum_hypothesis_confidence_margin_,
        global_minimum_hypothesis_relative_margin_))
    {
      const double absolute_margin = best.confidence - runner_up.confidence;
      const double relative_margin = best.confidence > 0.0 ?
        absolute_margin / best.confidence : 0.0;
      best.accepted = false;
      best.detail =
        "ambiguous global hypotheses: absolute confidence margin=" +
        std::to_string(absolute_margin) +
        ", relative margin=" + std::to_string(relative_margin);
    }
    return best;
  }

  AttemptOutcome attemptAnchored(
    const MapManifest & map,
    const Cloud::Ptr & source,
    const Eigen::Matrix4f & initial_guess,
    double search_radius)
  {
    AttemptOutcome best;
    AttemptOutcome runner_up;
    best.anchored = true;
    runner_up.anchored = true;
    const double initial_yaw = std::atan2(
      static_cast<double>(initial_guess(1, 0)),
      static_cast<double>(initial_guess(0, 0)));
    std::vector<AnchoredSeed2> seeds;
    try {
      seeds = makeAnchoredSeeds(
        static_cast<double>(initial_guess(0, 3)),
        static_cast<double>(initial_guess(1, 3)),
        initial_yaw,
        anchored_xy_offsets_m_,
        anchored_yaw_offsets_rad_,
        search_radius,
        anchored_maximum_registration_attempts_);
    } catch (const std::exception & error) {
      best.detail = error.what();
      return best;
    }
    if (seeds.empty()) {
      best.detail = "anchored search generated no hypotheses";
      return best;
    }

    std::size_t attempted = 0;
    for (const auto & seed : seeds) {
      Eigen::Matrix4f guess = Eigen::Matrix4f::Identity();
      guess.block<3, 3>(0, 0) =
        Eigen::AngleAxisf(
        static_cast<float>(seed.yaw), Eigen::Vector3f::UnitZ()).toRotationMatrix();
      guess(0, 3) = static_cast<float>(seed.x);
      guess(1, 3) = static_cast<float>(seed.y);
      guess(2, 3) = initial_guess(2, 3);

      std::string tile_id;
      try {
        const auto target = buildTarget(
          map, seed.x, seed.y,
          anchored_local_map_radius_m_, anchored_maximum_tiles_, tile_id);
        const auto registration = registration_.align(source, target, guess);
        auto outcome = evaluateAnchoredRegistration(
          registration, guess, tile_id,
          anchored_maximum_translation_correction_m_,
          anchored_maximum_rotation_correction_rad_,
          "anchored route-pose NDT/GICP converged without descriptor gating");
        if (outcome.accepted && !best.accepted) {
          best = outcome;
        } else if (outcome.accepted) {
          const auto hypothesis_delta =
            matrixPoseDifference(best.map_from_body, outcome.map_from_body);
          const bool distinct_hypothesis =
            hypothesesAreDistinct(
              hypothesis_delta.translation_m,
              hypothesis_delta.rotation_rad,
              global_hypothesis_separation_m_,
              global_hypothesis_separation_rad_);
          if (!distinct_hypothesis) {
            if (outcome.confidence > best.confidence) {
              best = outcome;
            }
          } else if (outcome.confidence > best.confidence) {
            runner_up = best;
            best = outcome;
          } else if (!runner_up.accepted ||
            outcome.confidence > runner_up.confidence)
          {
            runner_up = outcome;
          }
        } else if (!best.accepted && outcome.fitness < best.fitness) {
          best = outcome;
        }
      } catch (const std::exception & error) {
        if (!best.accepted) {
          best.detail = error.what();
        }
      }
      if (++attempted >= anchored_maximum_registration_attempts_) {
        break;
      }
    }
    if (attempted == 0) {
      best.detail = "anchored search attempted no registrations";
    }
    if (best.accepted && runner_up.accepted &&
      !passesHypothesisMargin(
        best.confidence, runner_up.confidence,
        global_minimum_hypothesis_confidence_margin_,
        global_minimum_hypothesis_relative_margin_))
    {
      const double absolute_margin = best.confidence - runner_up.confidence;
      const double relative_margin = best.confidence > 0.0 ?
        absolute_margin / best.confidence : 0.0;
      best.accepted = false;
      best.detail =
        "ambiguous anchored hypotheses: absolute confidence margin=" +
        std::to_string(absolute_margin) +
        ", relative margin=" + std::to_string(relative_margin);
    }
    return best;
  }

  AttemptOutcome attemptAnchoredConfirmation(
    const MapManifest & map,
    const Cloud::Ptr & source,
    const Eigen::Matrix4f & prediction)
  {
    AttemptOutcome best;
    AttemptOutcome runner_up;
    best.anchored = true;
    runner_up.anchored = true;
    const double predicted_yaw = std::atan2(
      static_cast<double>(prediction(1, 0)),
      static_cast<double>(prediction(0, 0)));
    std::vector<AnchoredSeed2> seeds;
    try {
      seeds = makeAnchoredSeeds(
        static_cast<double>(prediction(0, 3)),
        static_cast<double>(prediction(1, 3)),
        predicted_yaw,
        anchored_confirmation_xy_offsets_m_,
        anchored_confirmation_yaw_offsets_rad_,
        anchored_confirmation_search_radius_m_,
        anchored_confirmation_maximum_registration_attempts_);
    } catch (const std::exception & error) {
      best.detail = error.what();
      return best;
    }
    if (seeds.empty()) {
      best.detail = "anchored confirmation generated no hypotheses";
      return best;
    }

    std::size_t attempted = 0;
    for (const auto & seed : seeds) {
      Eigen::Matrix4f guess = Eigen::Matrix4f::Identity();
      guess.block<3, 3>(0, 0) =
        Eigen::AngleAxisf(
        static_cast<float>(seed.yaw), Eigen::Vector3f::UnitZ()).toRotationMatrix();
      guess(0, 3) = static_cast<float>(seed.x);
      guess(1, 3) = static_cast<float>(seed.y);
      guess(2, 3) = prediction(2, 3);

      std::string tile_id;
      AttemptOutcome outcome;
      outcome.anchored = true;
      try {
        const auto target = buildTarget(
          map, seed.x, seed.y,
          anchored_local_map_radius_m_, anchored_maximum_tiles_, tile_id);
        const auto registration = registration_.align(source, target, guess);
        outcome = evaluateAnchoredRegistration(
          registration, guess, tile_id,
          anchored_confirmation_maximum_translation_correction_m_,
          anchored_confirmation_maximum_rotation_correction_rad_,
          "fast provisional-pose confirmation converged");
      } catch (const std::exception & error) {
        outcome.detail = error.what();
      }
      ++attempted;

      // The provisional map<-odom prediction is already backed by the initial
      // multi-hypothesis ambiguity gate. A successful exact prediction needs
      // only one registration; repeatability across fresh scans is checked by
      // ConfirmationGate and StartupPrecisionGate.
      if (attempted == 1 && outcome.accepted) {
        return outcome;
      }
      if (outcome.accepted && !best.accepted) {
        best = outcome;
      } else if (outcome.accepted) {
        const auto hypothesis_delta =
          matrixPoseDifference(best.map_from_body, outcome.map_from_body);
        const bool distinct_hypothesis =
          hypothesesAreDistinct(
            hypothesis_delta.translation_m,
            hypothesis_delta.rotation_rad,
            global_hypothesis_separation_m_,
            global_hypothesis_separation_rad_);
        if (!distinct_hypothesis) {
          if (outcome.confidence > best.confidence) {
            best = outcome;
          }
        } else if (outcome.confidence > best.confidence) {
          runner_up = best;
          best = outcome;
        } else if (!runner_up.accepted ||
          outcome.confidence > runner_up.confidence)
        {
          runner_up = outcome;
        }
      } else if (!best.accepted && outcome.fitness < best.fitness) {
        best = outcome;
      }
      if (attempted >= anchored_confirmation_maximum_registration_attempts_) {
        break;
      }
    }
    if (best.accepted && runner_up.accepted &&
      !passesHypothesisMargin(
        best.confidence, runner_up.confidence,
        global_minimum_hypothesis_confidence_margin_,
        global_minimum_hypothesis_relative_margin_))
    {
      const double absolute_margin = best.confidence - runner_up.confidence;
      const double relative_margin = best.confidence > 0.0 ?
        absolute_margin / best.confidence : 0.0;
      best.accepted = false;
      best.detail =
        "ambiguous anchored confirmation hypotheses: absolute confidence margin=" +
        std::to_string(absolute_margin) +
        ", relative margin=" + std::to_string(relative_margin);
    }
    return best;
  }

  void recordOutcome(
    const AttemptOutcome & outcome,
    const OdomSample & odom,
    AttemptKind kind)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    last_attempt_detail_ = outcome.detail;
    last_fitness_ = outcome.fitness;
    last_inlier_ratio_ = outcome.inlier_ratio;
    last_descriptor_distance_ = outcome.descriptor_distance;
    last_correction_translation_ = outcome.correction_translation;
    last_correction_rotation_ = outcome.correction_rotation;
    last_attempt_anchored_ = outcome.anchored;
    last_attempt_completion_stamp_ = now();
    if (!outcome.accepted) {
      consecutive_accepts_ = 0;
      ++consecutive_rejects_;
      confirmation_gate_.reject();
      updateStateForAgeLocked(now());
      return;
    }

    map_load_quarantine_.disarm();
    const Eigen::Matrix4f desired_map_from_odom =
      outcome.map_from_body * odom.odom_from_body.inverse();
    if (kind == AttemptKind::kTracking && confirmation_gate_.pending()) {
      last_attempt_detail_ =
        "tracking cannot confirm a provisional global hypothesis";
      consecutive_accepts_ = 0;
      ++consecutive_rejects_;
      updateStateForAgeLocked(now());
      return;
    }
    if (kind == AttemptKind::kGlobalConfirmation &&
      !confirmation_gate_.pending())
    {
      last_attempt_detail_ =
        "global confirmation arrived without a pending hypothesis";
      consecutive_accepts_ = 0;
      ++consecutive_rejects_;
      pose_valid_ = false;
      provisional_pose_valid_ = false;
      state_ = LocalizationStatus::STATE_LOST;
      state_reason_ = last_attempt_detail_;
      startup_precision_gate_.reset();
      return;
    }
    if (kind == AttemptKind::kGlobalConfirmation) {
      const auto anchor_delta = matrixPoseDifference(
        provisional_anchor_map_from_odom_, desired_map_from_odom);
      if (!correctionWithinEnvelope(
          anchor_delta.translation_m, anchor_delta.rotation_rad,
          safe_maximum_correction_m_, safe_maximum_correction_rad_))
      {
        last_attempt_detail_ =
          "provisional confirmation inconsistent with global anchor";
        consecutive_accepts_ = 0;
        ++consecutive_rejects_;
        confirmation_gate_.reject();
        updateStateForAgeLocked(now());
        return;
      }
      const StartupPrecisionPose startup_candidate =
        startupPrecisionPose(desired_map_from_odom);
      if (!startup_precision_gate_.observe(startup_candidate)) {
        last_attempt_detail_ =
          "startup global solutions exceeded repeatability threshold: "
          "maximum translation deviation=" +
          std::to_string(startup_precision_gate_.maximumTranslationDeviation()) +
          " m, maximum yaw deviation=" +
          std::to_string(startup_precision_gate_.maximumYawDeviation()) +
          " rad; consistency does not establish absolute accuracy";
        consecutive_accepts_ = 0;
        ++consecutive_rejects_;
        confirmation_gate_.reset();
        provisional_pose_valid_ = false;
        pose_valid_ = false;
        map_from_odom_ = Eigen::Matrix4f::Identity();
        provisional_anchor_map_from_odom_ = Eigen::Matrix4f::Identity();
        state_ = LocalizationStatus::STATE_INITIALIZING;
        state_reason_ = last_attempt_detail_;
        return;
      }
    }
    if (kind == AttemptKind::kTracking) {
      map_from_odom_ = interpolateTransform(
        map_from_odom_, desired_map_from_odom, tracking_smoothing_alpha_);
    } else {
      map_from_odom_ = desired_map_from_odom;
    }
    if (kind == AttemptKind::kGlobalInitial) {
      confirmation_gate_.start(odom.stamp.nanoseconds());
      startup_precision_gate_.start(startupPrecisionPose(desired_map_from_odom));
      provisional_anchor_map_from_odom_ = desired_map_from_odom;
      provisional_pose_valid_ = true;
      pose_valid_ = false;
      state_ = LocalizationStatus::STATE_INITIALIZING;
      state_reason_ = "global pose is provisional; collecting independent confirmations";
    } else if (kind == AttemptKind::kGlobalConfirmation) {
      const bool confirmed = confirmation_gate_.accept(odom.stamp.nanoseconds());
      if (confirmed && !startup_precision_gate_.verify()) {
        last_attempt_detail_ =
          "startup precision gate could not verify the confirmed global sequence";
        consecutive_accepts_ = 0;
        ++consecutive_rejects_;
        pose_valid_ = false;
        provisional_pose_valid_ = false;
        map_from_odom_ = Eigen::Matrix4f::Identity();
        provisional_anchor_map_from_odom_ = Eigen::Matrix4f::Identity();
        state_ = LocalizationStatus::STATE_LOST;
        state_reason_ = last_attempt_detail_;
        return;
      }
      provisional_pose_valid_ = !confirmed;
      pose_valid_ = confirmed;
      state_ = confirmed ?
        LocalizationStatus::STATE_TRACKING :
        LocalizationStatus::STATE_INITIALIZING;
      state_reason_ = confirmed ?
        "global pose repeatability verified; tracking active "
        "(not a claim of absolute accuracy)" :
        "global pose is provisional; collecting independent confirmations";
    } else {
      provisional_pose_valid_ = false;
      pose_valid_ = true;
      state_ = LocalizationStatus::STATE_TRACKING;
      state_reason_ = "tracking correction accepted";
    }
    confidence_ = outcome.confidence;
    active_tile_id_ = outcome.tile_id;
    last_correction_stamp_ = odom.stamp;
    last_pose_stamp_ = odom.stamp;
    ++consecutive_accepts_;
    consecutive_rejects_ = 0;
    RCLCPP_INFO(
      get_logger(),
      "%s accepted: tile=%s confidence=%.3f fitness_rmse_m=%.3f inlier=%.3f "
      "confirmation=%u startup_max_translation=%.4f startup_max_yaw=%.6f "
      "startup_verified=%s",
      kind == AttemptKind::kTracking ? "Tracking" :
      (kind == AttemptKind::kGlobalInitial ?
      "Provisional global relocalization" : "Independent global confirmation"),
      active_tile_id_.c_str(), confidence_, last_fitness_, last_inlier_ratio_,
      confirmation_gate_.accepts(),
      startup_precision_gate_.maximumTranslationDeviation(),
      startup_precision_gate_.maximumYawDeviation(),
      startup_precision_gate_.verified() ? "true" : "false");
  }

  void updateStateForAgeLocked(const rclcpp::Time & current_time)
  {
    if (!input_extrinsics_verified_) {
      state_ = LocalizationStatus::STATE_LOST;
      state_reason_ = "input LiDAR/IMU/base extrinsics are not operator-verified";
      pose_valid_ = false;
      provisional_pose_valid_ = false;
      confirmation_gate_.reset();
      startup_precision_gate_.reset();
      provisional_anchor_map_from_odom_ = Eigen::Matrix4f::Identity();
      return;
    }
    if (!map_) {
      state_ = LocalizationStatus::STATE_INITIALIZING;
      state_reason_ = "map not loaded";
      pose_valid_ = false;
      provisional_pose_valid_ = false;
      confirmation_gate_.reset();
      startup_precision_gate_.reset();
      provisional_anchor_map_from_odom_ = Eigen::Matrix4f::Identity();
      return;
    }
    if (confirmation_gate_.pending() && provisional_pose_valid_) {
      // A full initial multi-hypothesis search may take longer than the
      // correction freshness budget. The provisional confirmation timeout
      // starts when that attempt completes; safe_to_move still uses the real
      // synchronized scan stamp after the fast final confirmation.
      const double confirmation_age =
        last_attempt_completion_stamp_.nanoseconds() == 0 ?
        std::numeric_limits<double>::infinity() :
        secondsBetween(current_time, last_attempt_completion_stamp_);
      if (confirmation_age < -maximum_future_offset_sec_) {
        state_ = LocalizationStatus::STATE_LOST;
        state_reason_ = "future-dated provisional attempt; localization invalidated";
        provisional_pose_valid_ = false;
        pose_valid_ = false;
        confirmation_gate_.reset();
        startup_precision_gate_.reset();
        provisional_anchor_map_from_odom_ = Eigen::Matrix4f::Identity();
      } else if (confirmation_age >= lost_correction_age_sec_ ||
        consecutive_rejects_ >= lost_rejections_)
      {
        state_ = LocalizationStatus::STATE_LOST;
        state_reason_ = "provisional global pose failed confirmation";
        provisional_pose_valid_ = false;
        pose_valid_ = false;
        confirmation_gate_.reset();
        startup_precision_gate_.reset();
        provisional_anchor_map_from_odom_ = Eigen::Matrix4f::Identity();
      } else {
        state_ = LocalizationStatus::STATE_INITIALIZING;
        state_reason_ =
          "global pose is provisional: " +
          std::to_string(confirmation_gate_.accepts()) + "/" +
          std::to_string(global_confirmation_required_accepts_);
        pose_valid_ = false;
      }
      return;
    }
    if (!pose_valid_) {
      state_ = LocalizationStatus::STATE_INITIALIZING;
      state_reason_ = last_attempt_detail_.empty() ?
        "waiting for global relocalization" : last_attempt_detail_;
      return;
    }
    const double correction_age =
      secondsBetween(current_time, last_correction_stamp_);
    if (correction_age < -maximum_future_offset_sec_) {
      state_ = LocalizationStatus::STATE_LOST;
      state_reason_ = "future-dated correction; localization invalidated";
      pose_valid_ = false;
      startup_precision_gate_.reset();
    } else if (correction_age >= lost_correction_age_sec_ ||
      consecutive_rejects_ >= lost_rejections_)
    {
      state_ = LocalizationStatus::STATE_LOST;
      state_reason_ = "localization correction lost: " + last_attempt_detail_;
      pose_valid_ = false;
      startup_precision_gate_.reset();
    } else if (correction_age >= degraded_correction_age_sec_ ||
      consecutive_rejects_ >= degraded_rejections_)
    {
      state_ = LocalizationStatus::STATE_DEGRADED;
      state_reason_ = "localization correction degraded: " + last_attempt_detail_;
    } else {
      state_ = LocalizationStatus::STATE_TRACKING;
    }
  }

  bool loadMapTransactional(
    const std::string & requested_path, bool verify_hashes, std::string & error)
  {
    if (requested_path.empty()) {
      error = "manifest path is empty";
      return false;
    }
    try {
      auto candidate = std::make_shared<MapManifest>(
        MapManifest::load(stripFileUri(requested_path), verify_hashes));
      if (candidate->frame_id != map_frame_) {
        throw std::runtime_error(
                "manifest frame_id '" + candidate->frame_id +
                "' differs from configured map_frame '" + map_frame_ + "'");
      }
      if (maximum_range_m_ + 1.0e-6 < candidate->descriptor_config.max_radius_m) {
        RCLCPP_WARN(
          get_logger(),
          "sensor.maximum_range_m=%.2f truncates descriptor radius %.2f; "
          "global retrieval quality will be reduced",
          maximum_range_m_, candidate->descriptor_config.max_radius_m);
      }
      std::lock_guard<std::mutex> lock(state_mutex_);
      map_ = std::move(candidate);
      map_from_odom_ = Eigen::Matrix4f::Identity();
      provisional_anchor_map_from_odom_ = Eigen::Matrix4f::Identity();
      pose_valid_ = false;
      provisional_pose_valid_ = false;
      confirmation_gate_.reset();
      startup_precision_gate_.reset();
      state_ = LocalizationStatus::STATE_INITIALIZING;
      state_reason_ = "map loaded; waiting for global relocalization";
      confidence_ = 0.0;
      last_fitness_ = std::numeric_limits<double>::infinity();
      last_inlier_ratio_ = 0.0;
      last_descriptor_distance_ = std::numeric_limits<double>::infinity();
      last_correction_translation_ = 0.0;
      last_correction_rotation_ = 0.0;
      last_attempt_anchored_ = false;
      last_pose_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      last_correction_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      last_attempt_completion_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      consecutive_accepts_ = 0;
      consecutive_rejects_ = 0;
      active_tile_id_.clear();
      last_attempt_detail_.clear();
      tile_cache_.clear();
      tile_lru_.clear();
      RCLCPP_INFO(
        get_logger(), "Loaded map '%s': %zu tiles, %zu descriptors, hash=%s",
        map_->map_id.c_str(), map_->tiles.size(), map_->descriptors.size(),
        map_->manifest_sha256.c_str());
      return true;
    } catch (const std::exception & exception) {
      error = exception.what();
      return false;
    }
  }

  void globalService(
    const std::shared_ptr<GlobalRelocalize::Request> request,
    std::shared_ptr<GlobalRelocalize::Response> response)
  {
    std::unique_lock<std::mutex> registration_lock(registration_mutex_);
    std::shared_ptr<const MapManifest> map;
    Cloud::Ptr source;
    OdomSample odom;
    bool provisional_confirmation = false;
    Eigen::Matrix4f provisional_map_from_odom = Eigen::Matrix4f::Identity();
    {
      std::lock_guard<std::mutex> state_lock(state_mutex_);
      if (manual_activation_enabled_ &&
        !localization_active_.load(std::memory_order_relaxed))
      {
        response->accepted = false;
        response->message =
          "manual checkpoint mode is inactive; call /localization/set_active first";
        return;
      }
      map = map_;
      provisional_confirmation =
        provisional_pose_valid_ && confirmation_gate_.pending();
      if (provisional_confirmation) {
        provisional_map_from_odom = provisional_anchor_map_from_odom_;
      }
      // A manual relocalization request explicitly cancels a pending map-load
      // maintenance window and resumes use of the current map.
      map_load_quarantine_.disarm();
    }
    {
      std::lock_guard<std::mutex> data_lock(data_mutex_);
      source = latest_source_;
      odom = latest_odom_;
    }
    if (!map) {
      response->accepted = false;
      response->message = "map not loaded";
      return;
    }
    if (!input_extrinsics_verified_) {
      response->accepted = false;
      response->message =
        "input_extrinsics_verified is false; refusing global localization";
      return;
    }
    if (!source || source->size() < minimum_descriptor_points_) {
      response->accepted = false;
      response->message = "not enough synchronized LiDAR points accumulated";
      return;
    }
    std::optional<Eigen::Matrix4f> initial_guess;
    try {
      if (request->use_initial_guess) {
        if (!request->initial_guess.header.frame_id.empty() &&
          request->initial_guess.header.frame_id != map_frame_)
        {
          throw std::runtime_error("initial guess must be expressed in map frame");
        }
        initial_guess = poseToMatrix(request->initial_guess.pose.pose);
      }
    } catch (const std::exception & error) {
      response->accepted = false;
      response->message = error.what();
      return;
    }
    const double radius = request->search_radius > 0.0F ?
      request->search_radius : global_local_map_radius_m_;
    const Eigen::Matrix4f confirmation_prediction =
      provisional_map_from_odom * odom.odom_from_body;
    const auto outcome = provisional_confirmation ?
      attemptAnchoredConfirmation(*map, source, confirmation_prediction) :
      (initial_guess ?
      attemptAnchored(*map, source, *initial_guess, radius) :
      attemptGlobal(*map, source, odom, std::nullopt, radius));
    recordOutcome(
      outcome, odom,
      provisional_confirmation ?
      AttemptKind::kGlobalConfirmation : AttemptKind::kGlobalInitial);
    bool pose_confirmed = false;
    {
      std::lock_guard<std::mutex> state_lock(state_mutex_);
      pose_confirmed = pose_valid_ && startup_precision_gate_.verified();
    }
    response->accepted = outcome.accepted;
    if (!outcome.accepted) {
      response->message = outcome.detail;
    } else if (pose_confirmed) {
      response->message =
        outcome.detail + "; global pose passed independent confirmation";
    } else {
      response->message =
        outcome.detail +
        "; provisional only: repeat this service after a fresh synchronized scan";
    }
    response->confidence = static_cast<float>(outcome.confidence);
    response->pose.header.stamp = odom.stamp;
    response->pose.header.frame_id = map_frame_;
    response->pose.pose.pose = matrixToPose(outcome.map_from_body);
  }

  void activationService(
    const std::shared_ptr<SetBool::Request> request,
    std::shared_ptr<SetBool::Response> response)
  {
    std::unique_lock<std::mutex> registration_lock(registration_mutex_);
    if (!manual_activation_enabled_) {
      response->success = false;
      response->message =
        "manual_activation.enabled is false; runtime activation is unavailable";
      return;
    }

    localization_active_.store(request->data, std::memory_order_relaxed);
    {
      std::lock_guard<std::mutex> state_lock(state_mutex_);
      map_from_odom_ = Eigen::Matrix4f::Identity();
      provisional_anchor_map_from_odom_ = Eigen::Matrix4f::Identity();
      pose_valid_ = false;
      provisional_pose_valid_ = false;
      confirmation_gate_.reset();
      startup_precision_gate_.reset();
      confidence_ = 0.0;
      last_fitness_ = std::numeric_limits<double>::infinity();
      last_inlier_ratio_ = 0.0;
      last_descriptor_distance_ = std::numeric_limits<double>::infinity();
      last_correction_translation_ = 0.0;
      last_correction_rotation_ = 0.0;
      last_attempt_anchored_ = false;
      last_pose_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      last_correction_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      last_attempt_completion_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      consecutive_accepts_ = 0;
      consecutive_rejects_ = 0;
      active_tile_id_.clear();
      last_attempt_detail_.clear();
      state_ = LocalizationStatus::STATE_INITIALIZING;
      state_reason_ = request->data ?
        "manual localization active; waiting for reset and stationary scans" :
        "manual localization inactive; downstream follower must use a frozen transform";
    }
    {
      std::lock_guard<std::mutex> data_lock(data_mutex_);
      scan_buffer_.clear();
      latest_source_.reset();
      latest_odom_ = OdomSample();
      last_cloud_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    }
    response->success = true;
    response->message = request->data ?
      "manual localization activated; collect stationary scans before relocalizing" :
      "manual localization deactivated; registration input and pose were invalidated";
    publishStatus();
  }

  void loadService(
    const std::shared_ptr<LoadMap::Request> request,
    std::shared_ptr<LoadMap::Response> response)
  {
    std::unique_lock<std::mutex> registration_lock(registration_mutex_);
    const std::string path =
      request->manifest_path.empty() ? map_manifest_parameter_ : request->manifest_path;
    const std::int64_t quarantine_now = steadyNowNanoseconds();
    std::string gate_rejection;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (pose_valid_ || provisional_pose_valid_ || confirmation_gate_.pending() ||
        state_ == LocalizationStatus::STATE_TRACKING ||
        state_ == LocalizationStatus::STATE_DEGRADED)
      {
        gate_rejection =
          "runtime map load rejected while localization is active; "
          "call /localization/reset first";
      } else if (!map_load_quarantine_.armed()) {
        gate_rejection =
          "runtime map load requires an explicit /localization/reset first";
      } else if (!map_load_quarantine_.ready(quarantine_now)) {
        gate_rejection =
          "map-load quarantine still active; retry after " +
          std::to_string(map_load_quarantine_.remainingSeconds(quarantine_now)) +
          " seconds";
      }
    }
    if (!gate_rejection.empty()) {
      response->success = false;
      response->message = gate_rejection;
      publishStatus();
      return;
    }
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      pose_valid_ = false;
      provisional_pose_valid_ = false;
      confirmation_gate_.reset();
      startup_precision_gate_.reset();
      state_ = LocalizationStatus::STATE_INITIALIZING;
      state_reason_ = "map load in progress; localization invalidated";
      confidence_ = 0.0;
      last_pose_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      last_correction_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      last_attempt_completion_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      consecutive_accepts_ = 0;
      consecutive_rejects_ = 0;
    }
    publishStatus();
    response->success = loadMapTransactional(path, request->verify_hashes, response->message);
    if (response->success) {
      std::lock_guard<std::mutex> lock(state_mutex_);
      map_load_quarantine_.disarm();
      response->message = "map loaded transactionally";
      response->map_id = map_->map_id;
      response->tile_count = static_cast<std::uint32_t>(map_->tiles.size());
    } else {
      std::lock_guard<std::mutex> lock(state_mutex_);
      state_ = LocalizationStatus::STATE_INITIALIZING;
      state_reason_ =
        "map load failed; old map retained but relocalization is required: " +
        response->message;
      last_attempt_detail_ = state_reason_;
    }
    publishStatus();
  }

  void resetService(
    const std::shared_ptr<ResetLocalization::Request> request,
    std::shared_ptr<ResetLocalization::Response> response)
  {
    std::unique_lock<std::mutex> registration_lock(registration_mutex_);
    {
      std::lock_guard<std::mutex> state_lock(state_mutex_);
      map_from_odom_ = Eigen::Matrix4f::Identity();
      provisional_anchor_map_from_odom_ = Eigen::Matrix4f::Identity();
      pose_valid_ = false;
      provisional_pose_valid_ = false;
      confirmation_gate_.reset();
      startup_precision_gate_.reset();
      confidence_ = 0.0;
      last_fitness_ = std::numeric_limits<double>::infinity();
      last_inlier_ratio_ = 0.0;
      last_descriptor_distance_ = std::numeric_limits<double>::infinity();
      last_correction_translation_ = 0.0;
      last_correction_rotation_ = 0.0;
      last_attempt_anchored_ = false;
      last_pose_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      last_correction_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      last_attempt_completion_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      consecutive_accepts_ = 0;
      consecutive_rejects_ = 0;
      active_tile_id_.clear();
      last_attempt_detail_.clear();
      if (request->clear_map) {
        map_.reset();
        tile_cache_.clear();
        tile_lru_.clear();
        state_reason_ = "localization and map cleared";
      } else {
        state_reason_ = map_ ? "localization reset; waiting for global relocalization" :
          "localization reset; map not loaded";
      }
      state_ = LocalizationStatus::STATE_INITIALIZING;
      map_load_quarantine_.arm(steadyNowNanoseconds());
    }
    {
      std::lock_guard<std::mutex> data_lock(data_mutex_);
      scan_buffer_.clear();
      latest_source_.reset();
    }
    response->success = true;
    response->message = request->clear_map ?
      "localization and map cleared; map-load quarantine armed" :
      "localization reset; map-load quarantine armed";
    publishStatus();
  }

  void publishPoseAndTf()
  {
    Eigen::Matrix4f map_from_odom;
    bool pose_valid = false;
    double fitness = 0.0;
    double confidence = 0.0;
    {
      std::lock_guard<std::mutex> state_lock(state_mutex_);
      updateStateForAgeLocked(now());
      map_from_odom = map_from_odom_;
      pose_valid = pose_valid_;
      fitness = last_fitness_;
      confidence = confidence_;
    }
    OdomSample odom;
    {
      std::lock_guard<std::mutex> data_lock(data_mutex_);
      if (odom_buffer_.empty()) {
        return;
      }
      odom = odom_buffer_.back();
    }
    if (!pose_valid) {
      return;
    }
    const Eigen::Matrix4f map_from_body = map_from_odom * odom.odom_from_body;
    geometry_msgs::msg::PoseWithCovarianceStamped pose;
    pose.header.stamp = odom.stamp;
    pose.header.frame_id = map_frame_;
    pose.pose.pose = matrixToPose(map_from_body);
    const double xy_variance = std::clamp(
      std::isfinite(fitness) ? fitness * fitness : 1.0, 0.01, 4.0);
    const double z_variance = std::clamp(2.0 * xy_variance, 0.04, 9.0);
    const double angle_variance = std::clamp(0.5 * (1.0 - confidence), 0.01, 1.0);
    pose.pose.covariance[0] = xy_variance;
    pose.pose.covariance[7] = xy_variance;
    pose.pose.covariance[14] = z_variance;
    pose.pose.covariance[21] = angle_variance;
    pose.pose.covariance[28] = angle_variance;
    pose.pose.covariance[35] = angle_variance;
    pose_pub_->publish(pose);

    nav_msgs::msg::Odometry corrected_odometry;
    corrected_odometry.header = pose.header;
    corrected_odometry.child_frame_id = base_frame_;
    corrected_odometry.pose = pose.pose;
    corrected_odometry.twist = odom.twist;
    odometry_pub_->publish(corrected_odometry);

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = odom.stamp;
    transform.header.frame_id = map_frame_;
    transform.child_frame_id = odom_frame_;
    transform.transform.translation.x = map_from_odom(0, 3);
    transform.transform.translation.y = map_from_odom(1, 3);
    transform.transform.translation.z = map_from_odom(2, 3);
    const Eigen::Quaternionf rotation(map_from_odom.block<3, 3>(0, 0));
    transform.transform.rotation.x = rotation.x();
    transform.transform.rotation.y = rotation.y();
    transform.transform.rotation.z = rotation.z();
    transform.transform.rotation.w = rotation.w();
    tf_broadcaster_->sendTransform(transform);
  }

  void publishStatus()
  {
    LocalizationStatus status;
    const auto current_time = now();
    std::lock_guard<std::mutex> lock(state_mutex_);
    updateStateForAgeLocked(current_time);
    status.header.stamp = current_time;
    status.header.frame_id = map_frame_;
    status.state = state_;
    status.map_valid = static_cast<bool>(map_);
    status.pose_valid = pose_valid_;
    status.confidence = static_cast<float>(confidence_);
    status.fitness_score = std::isfinite(last_fitness_) ?
      static_cast<float>(last_fitness_) : -1.0F;
    status.pose_age = last_pose_stamp_.nanoseconds() == 0 ? -1.0F :
      static_cast<float>(secondsBetween(current_time, last_pose_stamp_));
    status.correction_age = last_correction_stamp_.nanoseconds() == 0 ? -1.0F :
      static_cast<float>(secondsBetween(current_time, last_correction_stamp_));
    status.map_id = map_ ? map_->map_id : "";
    status.map_hash = map_ ? map_->manifest_sha256 : "";
    status.reason = state_reason_;
    status.descriptor_distance = std::isfinite(last_descriptor_distance_) ?
      static_cast<float>(last_descriptor_distance_) : -1.0F;
    status.inlier_ratio = std::isfinite(last_inlier_ratio_) ?
      static_cast<float>(last_inlier_ratio_) : -1.0F;
    status.correction_translation = static_cast<float>(last_correction_translation_);
    status.correction_rotation = static_cast<float>(last_correction_rotation_);
    status.consecutive_accepts = consecutive_accepts_;
    status.consecutive_rejects = consecutive_rejects_;
    status.active_tile_id = active_tile_id_;
    status.global_confirmation_pending = confirmation_gate_.pending();
    status.global_confirmation_accepts = confirmation_gate_.accepts();
    status.global_confirmation_span =
      static_cast<float>(confirmation_gate_.spanSeconds());
    status.startup_precision_verified = startup_precision_gate_.verified();
    status.startup_max_translation_deviation = static_cast<float>(
      startup_precision_gate_.maximumTranslationDeviation());
    status.startup_max_yaw_deviation = static_cast<float>(
      startup_precision_gate_.maximumYawDeviation());

    if (manual_activation_enabled_ &&
      !localization_active_.load(std::memory_order_relaxed))
    {
      status.reasons.push_back("manual_localization_inactive");
    }
    if (!status.map_valid) {
      status.reasons.push_back("map_not_loaded");
    }
    if (map_ && !map_->hashes_verified) {
      status.reasons.push_back("map_integrity_unverified");
    }
    if (!input_extrinsics_verified_) {
      status.reasons.push_back("input_extrinsics_unverified");
    }
    if (!status.pose_valid) {
      status.reasons.push_back("pose_invalid");
    }
    if (confirmation_gate_.pending()) {
      status.reasons.push_back("global_confirmation_pending");
    }
    if (!status.startup_precision_verified) {
      status.reasons.push_back("startup_precision_unverified");
    }
    if (map_load_quarantine_.armed()) {
      status.reasons.push_back("map_load_quarantine_armed");
    }
    if (status.correction_age < 0.0F) {
      status.reasons.push_back("correction_timestamp_in_future_or_unavailable");
    } else if (status.correction_age > safe_correction_age_sec_) {
      status.reasons.push_back("correction_stale");
    }
    if (status.pose_age < 0.0F) {
      status.reasons.push_back("pose_timestamp_in_future_or_unavailable");
    } else if (status.pose_age > safe_pose_age_sec_) {
      status.reasons.push_back("synchronized_pose_stale");
    }
    if (status.confidence < safe_minimum_confidence_) {
      status.reasons.push_back("confidence_below_safe_threshold");
    }
    if (!registrationQualityIsSafe(
        status.fitness_score, status.inlier_ratio,
        safe_maximum_fitness_rmse_m_, safe_minimum_inlier_ratio_))
    {
      if (status.fitness_score < 0.0F ||
        status.fitness_score > safe_maximum_fitness_rmse_m_)
      {
        status.reasons.push_back("registration_rmse_above_safe_threshold");
      }
      if (status.inlier_ratio < safe_minimum_inlier_ratio_ ||
        status.inlier_ratio > 1.0F)
      {
        status.reasons.push_back("registration_overlap_below_safe_threshold");
      }
    }
    const double safe_translation_correction_limit = last_attempt_anchored_ ?
      anchored_maximum_translation_correction_m_ : safe_maximum_correction_m_;
    const double safe_rotation_correction_limit = last_attempt_anchored_ ?
      anchored_maximum_rotation_correction_rad_ : safe_maximum_correction_rad_;
    if (status.correction_translation > safe_translation_correction_limit) {
      status.reasons.push_back("large_recent_translation_correction");
    }
    if (status.correction_rotation > safe_rotation_correction_limit) {
      status.reasons.push_back("large_recent_rotation_correction");
    }
    if (state_ != LocalizationStatus::STATE_TRACKING) {
      status.reasons.push_back("state_not_tracking");
    }
    status.safe_to_move = status.reasons.empty();
    status_pub_->publish(status);
  }

  // ROS entities.
  rclcpp::CallbackGroup::SharedPtr odom_group_;
  rclcpp::CallbackGroup::SharedPtr cloud_group_;
  rclcpp::CallbackGroup::SharedPtr service_group_;
  rclcpp::CallbackGroup::SharedPtr timer_group_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
  rclcpp::Publisher<LocalizationStatus>::SharedPtr status_pub_;
  rclcpp::Service<GlobalRelocalize>::SharedPtr global_service_;
  rclcpp::Service<LoadMap>::SharedPtr load_service_;
  rclcpp::Service<ResetLocalization>::SharedPtr reset_service_;
  rclcpp::Service<SetBool>::SharedPtr activation_service_;
  rclcpp::TimerBase::SharedPtr pose_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

  RegistrationPipeline registration_;
  std::mutex data_mutex_;
  std::mutex state_mutex_;
  std::mutex registration_mutex_;
  std::deque<OdomSample> odom_buffer_;
  std::deque<TimedScan> scan_buffer_;
  Cloud::Ptr latest_source_;
  OdomSample latest_odom_;
  rclcpp::Time last_cloud_stamp_{0, 0, RCL_ROS_TIME};
  std::shared_ptr<const MapManifest> map_;
  Eigen::Matrix4f map_from_odom_{Eigen::Matrix4f::Identity()};
  Eigen::Matrix4f provisional_anchor_map_from_odom_{Eigen::Matrix4f::Identity()};
  std::unordered_map<std::string, Cloud::Ptr> tile_cache_;
  std::list<std::string> tile_lru_;
  std::atomic<std::uint64_t> synchronized_drop_count_{0};
  std::atomic<std::uint64_t> invalid_odom_frame_count_{0};
  std::atomic<std::uint64_t> nonmonotonic_odom_count_{0};
  std::atomic<std::uint64_t> invalid_cloud_frame_count_{0};
  std::atomic<std::uint64_t> nonmonotonic_cloud_count_{0};
  std::atomic<std::uint64_t> invalid_timestamp_count_{0};
  std::atomic<bool> localization_active_{true};
  std::chrono::steady_clock::time_point last_attempt_steady_{};

  std::uint8_t state_{LocalizationStatus::STATE_UNKNOWN};
  bool pose_valid_{false};
  bool provisional_pose_valid_{false};
  ConfirmationGate confirmation_gate_{5, 2000000000LL};
  StartupPrecisionGate startup_precision_gate_{0.10, 0.00523598776};
  MapLoadQuarantine map_load_quarantine_{2000000000LL};
  double confidence_{0.0};
  double last_fitness_{std::numeric_limits<double>::infinity()};
  double last_inlier_ratio_{0.0};
  double last_descriptor_distance_{std::numeric_limits<double>::infinity()};
  double last_correction_translation_{0.0};
  double last_correction_rotation_{0.0};
  bool last_attempt_anchored_{false};
  std::uint32_t consecutive_accepts_{0};
  std::uint32_t consecutive_rejects_{0};
  std::string state_reason_;
  std::string last_attempt_detail_;
  std::string active_tile_id_;
  rclcpp::Time last_pose_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_correction_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_attempt_completion_stamp_{0, 0, RCL_ROS_TIME};

  // Parameters.
  std::string map_manifest_parameter_;
  bool verify_hashes_on_startup_{true};
  std::string cloud_topic_;
  std::string odom_topic_;
  std::string pose_topic_;
  std::string odometry_topic_;
  std::string status_topic_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  bool input_extrinsics_verified_{false};
  std::string global_service_name_;
  std::string load_service_name_;
  std::string reset_service_name_;
  std::string activation_service_name_;
  double map_load_quarantine_sec_{2.0};
  bool manual_activation_enabled_{false};
  double max_sync_slop_sec_{0.08};
  double odom_buffer_sec_{3.0};
  std::size_t maximum_odom_samples_{1000};
  double maximum_future_offset_sec_{0.10};
  double maximum_input_age_sec_{0.15};
  std::size_t accumulation_scan_count_{5};
  double accumulation_max_age_sec_{1.5};
  double accumulation_voxel_m_{0.15};
  std::size_t minimum_descriptor_points_{800};
  double minimum_range_m_{1.0};
  double maximum_range_m_{80.0};
  double minimum_z_m_{-2.5};
  double maximum_z_m_{5.0};
  double tracking_period_sec_{0.5};
  bool tracking_enabled_{true};
  double tracking_local_map_radius_m_{35.0};
  std::size_t tracking_maximum_tiles_{12};
  double tracking_maximum_fitness_{0.40};
  double tracking_minimum_inlier_ratio_{0.38};
  double tracking_maximum_translation_correction_m_{0.5};
  double tracking_maximum_rotation_correction_rad_{0.261799};
  double tracking_smoothing_alpha_{0.35};
  bool auto_global_relocalize_{true};
  double global_retry_period_sec_{2.0};
  std::size_t descriptor_top_k_{50};
  std::size_t global_registration_candidates_{4};
  std::size_t global_maximum_registration_attempts_{36};
  double global_translation_seed_fraction_{0.333333333333};
  double global_maximum_descriptor_distance_{0.75};
  double global_minimum_descriptor_overlap_ratio_{0.05};
  double global_maximum_fitness_{0.8};
  double global_minimum_inlier_ratio_{0.30};
  double global_minimum_hypothesis_confidence_margin_{0.10};
  double global_minimum_hypothesis_relative_margin_{0.20};
  double global_hypothesis_separation_m_{0.25};
  double global_hypothesis_separation_rad_{0.0872665};
  double global_maximum_roll_pitch_rad_{0.45};
  double global_local_map_radius_m_{40.0};
  std::size_t global_maximum_tiles_{16};
  std::vector<double> anchored_xy_offsets_m_{
    0.0, -1.0, 1.0, -2.0, 2.0};
  std::vector<double> anchored_yaw_offsets_rad_{
    0.0,
    -10.0 * kPiLocal / 180.0,
    10.0 * kPiLocal / 180.0,
    -20.0 * kPiLocal / 180.0,
    20.0 * kPiLocal / 180.0,
    -30.0 * kPiLocal / 180.0,
    30.0 * kPiLocal / 180.0};
  std::size_t anchored_maximum_registration_attempts_{36};
  double anchored_maximum_translation_correction_m_{3.0};
  double anchored_maximum_rotation_correction_rad_{20.0 * kPiLocal / 180.0};
  double anchored_local_map_radius_m_{40.0};
  std::size_t anchored_maximum_tiles_{16};
  std::vector<double> anchored_confirmation_xy_offsets_m_{
    0.0, -0.25, 0.25};
  std::vector<double> anchored_confirmation_yaw_offsets_rad_{
    0.0,
    -2.0 * kPiLocal / 180.0,
    2.0 * kPiLocal / 180.0};
  std::size_t anchored_confirmation_maximum_registration_attempts_{7};
  double anchored_confirmation_search_radius_m_{0.40};
  double anchored_confirmation_maximum_translation_correction_m_{0.50};
  double anchored_confirmation_maximum_rotation_correction_rad_{
    5.0 * kPiLocal / 180.0};
  std::size_t maximum_cached_tiles_{20};
  std::uint32_t degraded_rejections_{3};
  std::uint32_t lost_rejections_{10};
  std::uint32_t global_confirmation_required_accepts_{5};
  double global_confirmation_span_sec_{2.0};
  double startup_maximum_translation_deviation_m_{0.10};
  double startup_maximum_yaw_deviation_rad_{0.00523598776};
  double degraded_correction_age_sec_{3.0};
  double lost_correction_age_sec_{10.0};
  double safe_pose_age_sec_{0.15};
  double safe_correction_age_sec_{2.5};
  double safe_minimum_confidence_{0.55};
  double safe_maximum_fitness_rmse_m_{0.40};
  double safe_minimum_inlier_ratio_{0.35};
  double safe_maximum_correction_m_{0.25};
  double safe_maximum_correction_rad_{0.122173};
  double pose_publish_rate_hz_{20.0};
  double status_publish_rate_hz_{20.0};
};

}  // namespace go2_map_localizer

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<go2_map_localizer::MapLocalizerNode>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}

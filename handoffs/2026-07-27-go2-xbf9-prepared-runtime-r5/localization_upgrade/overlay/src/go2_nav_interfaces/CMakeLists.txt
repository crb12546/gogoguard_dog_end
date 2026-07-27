# Copyright 2026 Go2 Robotics Team
# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.5)
project(go2_nav_interfaces)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

set(msg_files
  "msg/GeofenceStatus.msg"
  "msg/Heartbeat.msg"
  "msg/LocalizationStatus.msg"
  "msg/MotionRequest.msg"
  "msg/ProximityStatus.msg"
  "msg/RouteStatus.msg"
  "msg/SafeMotionCommand.msg"
  "msg/SafetyStatus.msg"
)

set(srv_files
  "srv/GlobalRelocalize.srv"
  "srv/LoadMap.srv"
  "srv/ResetLocalization.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

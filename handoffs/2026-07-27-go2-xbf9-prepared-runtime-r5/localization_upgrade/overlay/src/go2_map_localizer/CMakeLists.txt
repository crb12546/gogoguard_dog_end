# Copyright 2026 Go2 Robotics Team
# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.5)
project(go2_map_localizer)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wconversion -Wshadow)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(go2_nav_interfaces REQUIRED)
find_package(jsoncpp CONFIG REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(PCL 1.10 REQUIRED COMPONENTS common filters io kdtree registration)
find_package(pcl_conversions REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)
add_definitions(${PCL_DEFINITIONS})

if(DEFINED ENV{ROS_DISTRO} AND NOT "$ENV{ROS_DISTRO}" STREQUAL "foxy")
  message(WARNING
    "go2_map_localizer is production-pinned to ROS 2 Foxy; "
    "current ROS_DISTRO='$ENV{ROS_DISTRO}'")
endif()

add_library(${PROJECT_NAME}_core
  src/map_manifest.cpp
  src/registration_pipeline.cpp
)
target_include_directories(${PROJECT_NAME}_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${PCL_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}_core
  Eigen3::Eigen
  jsoncpp_lib
  OpenSSL::Crypto
  yaml-cpp
  ${PCL_LIBRARIES}
)
add_executable(go2_map_localizer_node src/localizer_node.cpp)
target_include_directories(go2_map_localizer_node PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(go2_map_localizer_node
  ${PROJECT_NAME}_core
  Eigen3::Eigen
  ${PCL_LIBRARIES}
)
ament_target_dependencies(go2_map_localizer_node
  geometry_msgs
  go2_nav_interfaces
  nav_msgs
  pcl_conversions
  rcl_interfaces
  rclcpp
  sensor_msgs
  std_srvs
  tf2_ros
)

install(TARGETS
  ${PROJECT_NAME}_core
  go2_map_localizer_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/run_standalone_tests.sh DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Foxy's legacy copyright checker does not recognize SPDX-only Apache-2.0
  # headers, while cpplint rejects the project's deliberate pragma-once and
  # include grouping conventions. Uncrustify would rewrite those conventions.
  # Keep compiler warnings, cppcheck, CMake/XML/Python lint, and executable
  # contract tests enabled.
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  add_executable(test_polar_descriptor test/test_polar_descriptor.cpp)
  target_include_directories(test_polar_descriptor PRIVATE include)
  add_test(NAME polar_descriptor_standalone COMMAND test_polar_descriptor)
  add_executable(
    test_startup_precision_gate
    test/test_startup_precision_gate.cpp
  )
  target_include_directories(test_startup_precision_gate PRIVATE include)
  add_test(
    NAME startup_precision_gate_standalone
    COMMAND test_startup_precision_gate
  )
  add_executable(
    test_rigid_pose_difference
    test/test_rigid_pose_difference.cpp
  )
  target_include_directories(test_rigid_pose_difference PRIVATE include)
  add_test(
    NAME rigid_pose_difference_standalone
    COMMAND test_rigid_pose_difference
  )
  add_executable(test_anchored_seed test/test_anchored_seed.cpp)
  target_include_directories(test_anchored_seed PRIVATE include)
  add_test(NAME anchored_seed_standalone COMMAND test_anchored_seed)
  add_executable(test_quality_gates test/test_quality_gates.cpp)
  target_include_directories(test_quality_gates PRIVATE include)
  add_test(NAME quality_gates_standalone COMMAND test_quality_gates)
  add_executable(test_landmark_evidence test/test_landmark_evidence.cpp)
  target_include_directories(test_landmark_evidence PRIVATE include)
  add_test(NAME landmark_evidence_standalone COMMAND test_landmark_evidence)
  add_executable(test_map_manifest test/test_map_manifest.cpp)
  target_link_libraries(test_map_manifest ${PROJECT_NAME}_core)
  add_test(NAME map_manifest_contract COMMAND test_map_manifest)
  add_executable(
    test_registration_pipeline
    test/test_registration_pipeline.cpp
  )
  target_include_directories(test_registration_pipeline PRIVATE ${PCL_INCLUDE_DIRS})
  target_link_libraries(
    test_registration_pipeline
    ${PROJECT_NAME}_core
    Eigen3::Eigen
    ${PCL_LIBRARIES}
  )
  add_test(
    NAME registration_pipeline_known_truth
    COMMAND test_registration_pipeline
  )
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_core)
ament_export_dependencies(Eigen3 jsoncpp OpenSSL PCL yaml_cpp_vendor)
ament_package()

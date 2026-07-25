cmake_minimum_required(VERSION 3.5)
project(livox_timing_probe)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(livox_ros_driver2 REQUIRED)

add_executable(livox_timing_probe src/livox_timing_probe.cpp)
ament_target_dependencies(livox_timing_probe rclcpp livox_ros_driver2)

install(TARGETS livox_timing_probe DESTINATION lib/${PROJECT_NAME})
ament_package()

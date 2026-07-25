cmake_minimum_required(VERSION 3.5)
project(go2_map_manager)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

add_executable(submap_builder
  src/submap_builder.cpp
)

add_executable(route_relocalizer
  src/route_relocalizer.cpp
)

ament_target_dependencies(submap_builder
  rclcpp
  sensor_msgs
  nav_msgs
  pcl_ros
  pcl_conversions
  tf2
  tf2_ros
)

target_link_libraries(submap_builder
  ${PCL_LIBRARIES}
)

ament_target_dependencies(route_relocalizer
  rclcpp
  sensor_msgs
  nav_msgs
  pcl_ros
  pcl_conversions
)

target_link_libraries(route_relocalizer
  ${PCL_LIBRARIES}
)

install(TARGETS
  submap_builder
  route_relocalizer
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

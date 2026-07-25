cmake_minimum_required(VERSION 3.5)
project(go2_cmd_vel_bridge)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

set(UNITREE_SDK2_INSTALL "${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/unitree_sdk2_install")

add_executable(cmd_vel_udp_sender src/cmd_vel_udp_sender.cpp)
ament_target_dependencies(cmd_vel_udp_sender rclcpp geometry_msgs)

add_executable(go2_sdk2_udp_receiver src/go2_sdk2_udp_receiver.cpp)
target_include_directories(go2_sdk2_udp_receiver PRIVATE
  ${UNITREE_SDK2_INSTALL}/include
  ${UNITREE_SDK2_INSTALL}/include/ddscxx
)

target_link_libraries(go2_sdk2_udp_receiver
  ${UNITREE_SDK2_INSTALL}/lib/libunitree_sdk2.a
  ${UNITREE_SDK2_INSTALL}/lib/libddscxx.so
  ${UNITREE_SDK2_INSTALL}/lib/libddsc.so
  pthread
)

add_executable(go2_sdk2_motion_probe src/go2_sdk2_motion_probe.cpp)
target_include_directories(go2_sdk2_motion_probe PRIVATE
  ${UNITREE_SDK2_INSTALL}/include
  ${UNITREE_SDK2_INSTALL}/include/ddscxx
)

target_link_libraries(go2_sdk2_motion_probe
  ${UNITREE_SDK2_INSTALL}/lib/libunitree_sdk2.a
  ${UNITREE_SDK2_INSTALL}/lib/libddscxx.so
  ${UNITREE_SDK2_INSTALL}/lib/libddsc.so
  pthread
)

install(TARGETS
  cmd_vel_udp_sender
  go2_sdk2_udp_receiver
  go2_sdk2_motion_probe
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

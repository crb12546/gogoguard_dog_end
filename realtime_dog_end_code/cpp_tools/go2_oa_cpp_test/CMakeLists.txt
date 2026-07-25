cmake_minimum_required(VERSION 3.10)
project(go2_oa_cpp_test)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(UNITREE_SDK2_SRC "$ENV{HOME}/go2_fastlio_ws/third_party/unitree_sdk2")
set(UNITREE_SDK2_INSTALL "$ENV{HOME}/go2_fastlio_ws/third_party/unitree_sdk2_install")
set(CYCLONEDDS_INSTALL "$ENV{HOME}/cyclonedds_ws/install")

find_path(UNITREE_SDK2_INCLUDE_DIR
  NAMES unitree/robot/channel/channel_factory.hpp
  PATHS
    ${UNITREE_SDK2_INSTALL}/include
    ${UNITREE_SDK2_SRC}/include
  NO_DEFAULT_PATH
)

find_library(UNITREE_SDK2_LIBRARY
  NAMES unitree_sdk2 unitree_sdk2_static
  PATHS
    ${UNITREE_SDK2_INSTALL}/lib
    ${UNITREE_SDK2_SRC}/lib
    ${UNITREE_SDK2_SRC}/build
    ${UNITREE_SDK2_SRC}/build/lib
  NO_DEFAULT_PATH
)

find_path(CYCLONEDDS_CXX_INCLUDE_DIR
  NAMES dds/dds.hpp
  PATHS
    ${CYCLONEDDS_INSTALL}/include
    /usr/local/include
    /usr/include
  NO_DEFAULT_PATH
)

find_library(DDSCXX_LIBRARY
  NAMES ddscxx
  PATHS
    ${CYCLONEDDS_INSTALL}/lib
    /usr/local/lib
    /usr/lib
    /usr/lib/aarch64-linux-gnu
  NO_DEFAULT_PATH
)

find_library(DDSC_LIBRARY
  NAMES ddsc
  PATHS
    ${CYCLONEDDS_INSTALL}/lib
    /usr/local/lib
    /usr/lib
    /usr/lib/aarch64-linux-gnu
  NO_DEFAULT_PATH
)

if(NOT UNITREE_SDK2_INCLUDE_DIR)
  message(FATAL_ERROR "Cannot find unitree_sdk2 include dir")
endif()

if(NOT UNITREE_SDK2_LIBRARY)
  message(FATAL_ERROR "Cannot find unitree_sdk2 library")
endif()

if(NOT CYCLONEDDS_CXX_INCLUDE_DIR)
  message(FATAL_ERROR "Cannot find CycloneDDS C++ include dir: missing dds/dds.hpp")
endif()

if(NOT DDSCXX_LIBRARY)
  message(FATAL_ERROR "Cannot find libddscxx")
endif()

if(NOT DDSC_LIBRARY)
  message(FATAL_ERROR "Cannot find libddsc")
endif()

message(STATUS "UNITREE_SDK2_INCLUDE_DIR=${UNITREE_SDK2_INCLUDE_DIR}")
message(STATUS "UNITREE_SDK2_LIBRARY=${UNITREE_SDK2_LIBRARY}")
message(STATUS "CYCLONEDDS_CXX_INCLUDE_DIR=${CYCLONEDDS_CXX_INCLUDE_DIR}")
message(STATUS "DDSCXX_LIBRARY=${DDSCXX_LIBRARY}")
message(STATUS "DDSC_LIBRARY=${DDSC_LIBRARY}")

find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(spdlog REQUIRED)
find_package(fmt REQUIRED)

add_executable(oa_cpp_test src/oa_cpp_test.cpp)

target_include_directories(oa_cpp_test PRIVATE
  ${UNITREE_SDK2_INCLUDE_DIR}
  ${CYCLONEDDS_CXX_INCLUDE_DIR}
)

target_link_libraries(oa_cpp_test PRIVATE
  ${UNITREE_SDK2_LIBRARY}
  ${DDSCXX_LIBRARY}
  ${DDSC_LIBRARY}
  Threads::Threads
  yaml-cpp
  spdlog::spdlog
  fmt::fmt
)

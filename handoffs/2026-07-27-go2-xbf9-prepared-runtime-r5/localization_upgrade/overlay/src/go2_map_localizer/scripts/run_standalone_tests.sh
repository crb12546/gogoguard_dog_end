#!/usr/bin/env sh
# Copyright 2026 Go2 Robotics Team
# SPDX-License-Identifier: Apache-2.0
set -eu

script_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
package_dir=$(CDPATH= cd -- "${script_dir}/.." && pwd)
compiler=${CXX:-c++}
polar_output_path=${TMPDIR:-/tmp}/go2_map_localizer_polar_descriptor_test
startup_output_path=${TMPDIR:-/tmp}/go2_map_localizer_startup_precision_gate_test
pose_difference_output_path=${TMPDIR:-/tmp}/go2_map_localizer_rigid_pose_difference_test
anchored_seed_output_path=${TMPDIR:-/tmp}/go2_map_localizer_anchored_seed_test
quality_gates_output_path=${TMPDIR:-/tmp}/go2_map_localizer_quality_gates_test

"${compiler}" \
  -std=c++17 \
  -Wall -Wextra -Wpedantic -Wconversion -Wshadow -Werror \
  -I"${package_dir}/include" \
  "${package_dir}/test/test_polar_descriptor.cpp" \
  -o "${polar_output_path}"
"${polar_output_path}"

"${compiler}" \
  -std=c++17 \
  -Wall -Wextra -Wpedantic -Wconversion -Wshadow -Werror \
  -I"${package_dir}/include" \
  "${package_dir}/test/test_startup_precision_gate.cpp" \
  -o "${startup_output_path}"
"${startup_output_path}"

"${compiler}" \
  -std=c++17 \
  -Wall -Wextra -Wpedantic -Wconversion -Wshadow -Werror \
  -I"${package_dir}/include" \
  "${package_dir}/test/test_rigid_pose_difference.cpp" \
  -o "${pose_difference_output_path}"
"${pose_difference_output_path}"

"${compiler}" \
  -std=c++17 \
  -Wall -Wextra -Wpedantic -Wconversion -Wshadow -Werror \
  -I"${package_dir}/include" \
  "${package_dir}/test/test_anchored_seed.cpp" \
  -o "${anchored_seed_output_path}"
"${anchored_seed_output_path}"

"${compiler}" \
  -std=c++17 \
  -Wall -Wextra -Wpedantic -Wconversion -Wshadow -Werror \
  -I"${package_dir}/include" \
  "${package_dir}/test/test_quality_gates.cpp" \
  -o "${quality_gates_output_path}"
"${quality_gates_output_path}"

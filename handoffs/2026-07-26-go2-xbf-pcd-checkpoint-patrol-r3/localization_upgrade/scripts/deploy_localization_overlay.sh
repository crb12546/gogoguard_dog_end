#!/usr/bin/env bash
set -euo pipefail

# 只在本交付包的 overlay 目录内生成 build/install/log。
# rosdep 可能通过系统包管理器安装缺失的编译依赖，但本脚本不修改现有巡检源码。

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
BUNDLE_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd -P)"
OVERLAY_ROOT="${BUNDLE_ROOT}/overlay"
SOURCE_ROOT="${OVERLAY_ROOT}/src"

ROS_DISTRO_NAME="${GO2_LOCALIZATION_ROS_DISTRO:-foxy}"
ROS_SETUP="/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
GRAPH_SETUP="/unitree/module/graph_pid_ws/install/setup.bash"
FASTLIO_WORKSPACE="${GO2_FASTLIO_WS:-/home/unitree/go2_fastlio_ws}"
FASTLIO_SETUP="${FASTLIO_WORKSPACE}/install/setup.bash"

fail() {
  echo "错误：$*" >&2
  exit 1
}

source_setup() {
  local setup_file="$1"
  # ROS 2 Foxy 的部分 setup 脚本会读取未定义变量，因此 source 时暂时关闭 nounset。
  set +u
  # shellcheck disable=SC1090
  source "${setup_file}"
  set -u
}

[[ -f "${ROS_SETUP}" ]] || fail "找不到 ROS 2 环境：${ROS_SETUP}"
[[ -f "${SOURCE_ROOT}/go2_nav_interfaces/package.xml" ]] ||
  fail "缺少 go2_nav_interfaces 源码包"
[[ -f "${SOURCE_ROOT}/go2_map_localizer/package.xml" ]] ||
  fail "缺少 go2_map_localizer 源码包"
[[ -f "${SOURCE_ROOT}/go2_map_tools/package.xml" ]] ||
  fail "缺少 go2_map_tools 源码包"
[[ -f "${SOURCE_ROOT}/go2_checkpoint_patrol/package.xml" ]] ||
  fail "缺少 go2_checkpoint_patrol 源码包"

source_setup "${ROS_SETUP}"

if [[ -f "${GRAPH_SETUP}" ]]; then
  source_setup "${GRAPH_SETUP}"
else
  echo "提示：未发现 ${GRAPH_SETUP}，继续使用标准 ROS 2 环境。"
fi

if [[ -f "${FASTLIO_SETUP}" ]]; then
  source_setup "${FASTLIO_SETUP}"
else
  echo "提示：未发现 FAST-LIO underlay：${FASTLIO_SETUP}。"
  echo "      可以完成独立编译，但运行前必须确认真狗的 FAST-LIO 工作区。"
fi

command -v rosdep >/dev/null 2>&1 || fail "找不到 rosdep"
command -v colcon >/dev/null 2>&1 || fail "找不到 colcon"
command -v ros2 >/dev/null 2>&1 || fail "找不到 ros2"

echo "安装四个新增包所需的 ROS/系统依赖……"
rosdep install \
  --from-paths \
    "${SOURCE_ROOT}/go2_nav_interfaces" \
    "${SOURCE_ROOT}/go2_map_localizer" \
    "${SOURCE_ROOT}/go2_map_tools" \
    "${SOURCE_ROOT}/go2_checkpoint_patrol" \
  --ignore-src \
  --rosdistro "${ROS_DISTRO_NAME}" \
  -r -y

echo "在独立 overlay 中构建定位包……"
cd "${OVERLAY_ROOT}"
colcon build \
  --symlink-install \
  --packages-select \
    go2_nav_interfaces \
    go2_map_tools \
    go2_map_localizer \
    go2_checkpoint_patrol

[[ -f "${OVERLAY_ROOT}/install/setup.bash" ]] ||
  fail "构建结束但未生成 install/setup.bash"

source_setup "${OVERLAY_ROOT}/install/setup.bash"
ros2 pkg prefix go2_nav_interfaces >/dev/null
ros2 pkg prefix go2_map_tools >/dev/null
ros2 pkg prefix go2_map_localizer >/dev/null
ros2 pkg prefix go2_checkpoint_patrol >/dev/null

if ! ros2 pkg executables go2_map_tools |
  grep -Eq '(^|[[:space:]])go2-map$'; then
  fail "未找到 go2_map_tools/go2-map 地图发布/验证程序"
fi

if ! ros2 pkg executables go2_map_localizer |
  grep -Eq '(^|[[:space:]])go2_map_localizer_node$'; then
  fail "未找到 go2_map_localizer_node 可执行程序"
fi

if ! ros2 pkg executables go2_checkpoint_patrol |
  grep -Eq '(^|[[:space:]])checkpoint-localization-coordinator$'; then
  fail "未找到 checkpoint-localization-coordinator 可执行程序"
fi

echo
echo "构建成功：${OVERLAY_ROOT}/install"
echo "本脚本没有启动定位器，也没有启动或发送任何运动命令。"
echo "下一步请先阅读 ${BUNDLE_ROOT}/README.md，并准备审核通过的地图包。"

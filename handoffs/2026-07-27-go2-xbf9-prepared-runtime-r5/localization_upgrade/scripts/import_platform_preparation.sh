#!/usr/bin/env bash
set -euo pipefail

# Offline-only bridge from the browser export to immutable dog-runtime assets.
# It loads the built Python packages, but deliberately does not start ROS,
# create publishers, or invoke any Unitree motion program.

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
BUNDLE_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd -P)"
ROS_DISTRO_NAME="${GO2_LOCALIZATION_ROS_DISTRO:-foxy}"
ROS_SETUP="/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
OVERLAY_SETUP="${BUNDLE_ROOT}/overlay/install/setup.bash"

fail() {
  echo "错误：$*" >&2
  exit 1
}

source_setup() {
  local setup_file="$1"
  set +u
  # shellcheck disable=SC1090
  source "${setup_file}"
  set -u
}

[[ -f "${ROS_SETUP}" ]] || fail "找不到 ROS 2 环境：${ROS_SETUP}"
[[ -f "${OVERLAY_SETUP}" ]] ||
  fail "定位 overlay 尚未构建：请先运行 scripts/deploy_localization_overlay.sh"
[[ -x "${SCRIPT_DIR}/import_platform_preparation.py" ]] ||
  fail "准备包导入器不可执行"

source_setup "${ROS_SETUP}"
source_setup "${OVERLAY_SETUP}"

exec python3 "${SCRIPT_DIR}/import_platform_preparation.py" "$@"

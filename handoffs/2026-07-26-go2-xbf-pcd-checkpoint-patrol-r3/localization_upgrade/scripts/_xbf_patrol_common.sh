#!/usr/bin/env bash

# Shared helpers for the XBF production patrol scripts. This file is sourced;
# it deliberately does not enable or disable the caller's shell options.

XBF_SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
XBF_BUNDLE_ROOT="$(cd -- "${XBF_SCRIPT_DIR}/.." && pwd -P)"
XBF_OVERLAY_ROOT="${XBF_BUNDLE_ROOT}/overlay"
XBF_ROS_DISTRO="${GO2_LOCALIZATION_ROS_DISTRO:-foxy}"
XBF_ROS_SETUP="/opt/ros/${XBF_ROS_DISTRO}/setup.bash"
XBF_GRAPH_SETUP="/unitree/module/graph_pid_ws/install/setup.bash"
XBF_FASTLIO_WORKSPACE="${GO2_FASTLIO_WS:-/home/unitree/go2_fastlio_ws}"
XBF_FASTLIO_SETUP="${XBF_FASTLIO_WORKSPACE}/install/setup.bash"
XBF_OVERLAY_SETUP="${XBF_OVERLAY_ROOT}/install/setup.bash"
XBF_RUNTIME_DIR="${GO2_XBF_RUNTIME_DIR:-/tmp/go2_xbf_patrol}"

xbf_fail() {
  echo "错误：$*" >&2
  return 1
}

xbf_source_setup() {
  local setup_file="$1"
  set +u
  # shellcheck disable=SC1090
  source "${setup_file}"
  set -u
}

xbf_source_runtime() {
  [[ -f "${XBF_ROS_SETUP}" ]] ||
    xbf_fail "找不到 ROS 2 环境：${XBF_ROS_SETUP}" || return 1
  [[ -f "${XBF_FASTLIO_SETUP}" ]] ||
    xbf_fail "找不到 FAST-LIO underlay：${XBF_FASTLIO_SETUP}" || return 1
  [[ -f "${XBF_OVERLAY_SETUP}" ]] ||
    xbf_fail "定位 overlay 尚未构建：请先运行 scripts/deploy_localization_overlay.sh" ||
    return 1

  xbf_source_setup "${XBF_ROS_SETUP}"
  if [[ -f "${XBF_GRAPH_SETUP}" ]]; then
    xbf_source_setup "${XBF_GRAPH_SETUP}"
  fi
  xbf_source_setup "${XBF_FASTLIO_SETUP}"
  xbf_source_setup "${XBF_OVERLAY_SETUP}"
}

xbf_absolute_file() {
  local value="$1"
  local directory
  [[ -f "${value}" ]] || return 1
  directory="$(cd -- "$(dirname -- "${value}")" && pwd -P)"
  printf '%s/%s\n' "${directory}" "$(basename -- "${value}")"
}

xbf_absolute_directory() {
  local value="$1"
  [[ -d "${value}" ]] || return 1
  (cd -- "${value}" && pwd -P)
}

xbf_resolve_inputs() {
  local map_value="${1:-${GO2_XBF_MAP_ROOT:-${XBF_BUNDLE_ROOT}/maps/xbf-2026-07-26-map-reviewed-r2}}"
  local route_value="${2:-${GO2_XBF_ROUTE_FILE:-${XBF_BUNDLE_ROOT}/routes/xbf9_horizontal_clean.map-reviewed-r2.csv}}"
  local checkpoint_value="${3:-${GO2_XBF_CHECKPOINT_FILE:-${XBF_BUNDLE_ROOT}/routes/xbf9_horizontal_clean.map-reviewed-r2.checkpoints.json}}"

  XBF_MAP_ROOT="$(xbf_absolute_directory "${map_value}")" ||
    xbf_fail "找不到地图目录：${map_value}" || return 1
  XBF_ROUTE_FILE="$(xbf_absolute_file "${route_value}")" ||
    xbf_fail "找不到路线 CSV：${route_value}" || return 1
  XBF_CHECKPOINT_FILE="$(xbf_absolute_file "${checkpoint_value}")" ||
    xbf_fail "找不到 checkpoint sidecar：${checkpoint_value}" || return 1
  [[ "${XBF_ROUTE_FILE}" == *.csv ]] ||
    xbf_fail "路线文件必须以 .csv 结尾：${XBF_ROUTE_FILE}" || return 1
  XBF_ROUTE_METADATA="${XBF_ROUTE_FILE%.csv}.route.json"
  [[ -f "${XBF_ROUTE_METADATA}" ]] ||
    xbf_fail "路线缺少同名部署元数据：${XBF_ROUTE_METADATA}" || return 1

  XBF_MAP_MANIFEST="${XBF_MAP_ROOT}/manifest.json"
  XBF_MAP_MANIFEST_CHECKSUM="${XBF_MAP_ROOT}/manifest.sha256"
  XBF_MAP_PUBLICATION="${XBF_MAP_ROOT}/reviewed_map_publication.json"
  [[ -f "${XBF_MAP_MANIFEST}" ]] ||
    xbf_fail "地图缺少 manifest.json" || return 1
  [[ -f "${XBF_MAP_MANIFEST_CHECKSUM}" ]] ||
    xbf_fail "地图缺少 manifest.sha256" || return 1
  [[ -f "${XBF_MAP_PUBLICATION}" ]] ||
    xbf_fail "地图缺少 reviewed_map_publication.json" || return 1
}

xbf_topic_count() {
  local topic="$1"
  local kind="$2"
  local label
  case "${kind}" in
    publisher) label="Publisher count:" ;;
    subscriber) label="Subscription count:" ;;
    *) return 2 ;;
  esac
  { ros2 topic info "${topic}" 2>/dev/null || true; } |
    awk -v wanted="${label}" '
      index($0, wanted) == 1 {
        value = $NF
        if (value ~ /^[0-9]+$/) {
          print value
          found = 1
        }
      }
      END {
        if (!found) {
          print 0
        }
      }
    '
}

xbf_wait_for_service() {
  local service="$1"
  local timeout_sec="$2"
  local started now
  started="$(date +%s)"
  while true; do
    if ros2 service list 2>/dev/null | grep -Fxq "${service}"; then
      return 0
    fi
    now="$(date +%s)"
    if ((now - started >= timeout_sec)); then
      return 1
    fi
    sleep 1
  done
}

xbf_wait_for_publisher_count() {
  local topic="$1"
  local expected="$2"
  local timeout_sec="$3"
  local started now current
  started="$(date +%s)"
  while true; do
    current="$(xbf_topic_count "${topic}" publisher)"
    if [[ "${current}" == "${expected}" ]]; then
      return 0
    fi
    now="$(date +%s)"
    if ((now - started >= timeout_sec)); then
      return 1
    fi
    sleep 1
  done
}

xbf_wait_for_subscriber_count() {
  local topic="$1"
  local expected="$2"
  local timeout_sec="$3"
  local started now current
  started="$(date +%s)"
  while true; do
    current="$(xbf_topic_count "${topic}" subscriber)"
    if [[ "${current}" == "${expected}" ]]; then
      return 0
    fi
    now="$(date +%s)"
    if ((now - started >= timeout_sec)); then
      return 1
    fi
    sleep 1
  done
}

xbf_wait_for_min_subscriber_count() {
  local topic="$1"
  local minimum="$2"
  local timeout_sec="$3"
  local started now current
  started="$(date +%s)"
  while true; do
    current="$(xbf_topic_count "${topic}" subscriber)"
    if [[ "${current}" =~ ^[0-9]+$ ]] && ((current >= minimum)); then
      return 0
    fi
    now="$(date +%s)"
    if ((now - started >= timeout_sec)); then
      return 1
    fi
    sleep 1
  done
}

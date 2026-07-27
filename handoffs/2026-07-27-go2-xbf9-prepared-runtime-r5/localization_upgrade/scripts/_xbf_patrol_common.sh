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
XBF_RMW_IMPLEMENTATION="rmw_cyclonedds_cpp"
XBF_CYCLONEDDS_URI="${GO2_XBF_CYCLONEDDS_URI:-file:///tmp/go2_cyclonedds_eth0.xml}"

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
  local cyclonedds_config
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

  # 这台狗的稳定 Livox/FAST-LIO 运行栈使用 Cyclone DDS。实机验证表明，
  # 新节点若回退到默认 Fast DDS，会快速耗尽板载内存。因此必须在完整的
  # underlay/overlay 加载后重新固定 RMW；source setup 文件可能覆盖环境变量。
  [[ "${XBF_CYCLONEDDS_URI}" == file:///* ]] ||
    xbf_fail \
      "GO2_XBF_CYCLONEDDS_URI 必须是绝对 file:// URI：${XBF_CYCLONEDDS_URI}" ||
    return 1
  cyclonedds_config="${XBF_CYCLONEDDS_URI#file://}"
  [[ -r "${cyclonedds_config}" ]] ||
    xbf_fail "Cyclone DDS 配置不可读：${cyclonedds_config}" || return 1
  export RMW_IMPLEMENTATION="${XBF_RMW_IMPLEMENTATION}"
  export CYCLONEDDS_URI="${XBF_CYCLONEDDS_URI}"
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

xbf_process_start_ticks() {
  local pid="$1"
  [[ "${pid}" =~ ^[0-9]+$ && -r "/proc/${pid}/stat" ]] || return 1
  python3 - "${pid}" <<'PY'
import sys
from pathlib import Path

text = (Path("/proc") / sys.argv[1] / "stat").read_text(encoding="ascii")
closing_parenthesis = text.rfind(")")
if closing_parenthesis < 0:
    raise SystemExit(1)
fields_after_comm = text[closing_parenthesis + 2 :].split()
# `/proc/<pid>/stat` field 22 is process start time.  After removing PID and
# `(comm)`, field 3 is index 0, so starttime is index 19.
if len(fields_after_comm) <= 19 or not fields_after_comm[19].isdigit():
    raise SystemExit(1)
print(fields_after_comm[19])
PY
}

xbf_resolve_inputs() {
  local map_value="${1:-${GO2_XBF_MAP_ROOT:-${XBF_BUNDLE_ROOT}/maps/xbf9-horizontal-clean-r1}}"
  local route_value="${2:-${GO2_XBF_ROUTE_FILE:-${XBF_BUNDLE_ROOT}/routes/xbf9_horizontal_clean.aligned.csv}}"
  local checkpoint_value="${3:-${GO2_XBF_CHECKPOINT_FILE:-${XBF_BUNDLE_ROOT}/routes/xbf9_horizontal_clean.checkpoints.json}}"

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

xbf_udp_port_in_use() {
  local port="$1"
  [[ "${port}" =~ ^[0-9]+$ ]] || return 2
  ss -H -lun 2>/dev/null |
    awk -v port="${port}" '
      {
        for (field = 1; field <= NF; field++) {
          if ($field ~ (":" port "$")) {
            found = 1
            exit
          }
        }
      }
      END {
        exit found ? 0 : 1
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

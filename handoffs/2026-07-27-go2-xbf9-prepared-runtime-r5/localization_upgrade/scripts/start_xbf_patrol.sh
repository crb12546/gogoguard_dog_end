#!/usr/bin/env bash
set -euo pipefail

# shellcheck source=_xbf_patrol_common.sh
source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)/_xbf_patrol_common.sh"

usage() {
  cat >&2 <<'EOF'
用法：
  GO2_INPUT_EXTRINSICS_VERIFIED=1 \
    start_xbf_patrol.sh [reviewed-map目录] [路线CSV] [checkpoint.json]

不传参数时使用交付包内已由平台确认的 xbf9 地图、对齐路线和 8 个 checkpoint。
本脚本启动：
  go2_map_localizer
  checkpoint-localization-coordinator
  go2_cmd_vel_bridge/go2_sdk2_udp_receiver
  go2_cmd_vel_bridge/cmd_vel_udp_sender
  原 go2_fastlio_patrol/unitree_safe_cmd_node
  原 go2_fastlio_patrol/waypoint_follower_go2_2

它不重复启动 Livox 或 FAST-LIO；运动命令沿用狗端 SaaS 已验证的
127.0.0.1:5005 UDP -> SDK2 链路，不依赖 /api/sport/request。
首次现场可设置 GO2_XBF_CALIBRATION_ONLY=1，只定位但不产生非零运动。
EOF
}

if [[ $# -gt 3 ]]; then
  usage
  exit 2
fi

case "${GO2_INPUT_EXTRINSICS_VERIFIED:-0}" in
  1|true|TRUE|yes|YES) ;;
  *)
    xbf_fail \
      "生产巡检必须在核对 MID-360/IMU/base_link 外参后显式设置 GO2_INPUT_EXTRINSICS_VERIFIED=1"
    ;;
esac

xbf_resolve_inputs "$@"
xbf_source_runtime

"${XBF_SCRIPT_DIR}/preflight_xbf_patrol.sh" \
  "${XBF_MAP_ROOT}" "${XBF_ROUTE_FILE}" "${XBF_CHECKPOINT_FILE}"

localizer_template="${XBF_BUNDLE_ROOT}/config/localizer-u2-production.yaml"
coordinator_config="$(
  xbf_absolute_file \
    "${XBF_BUNDLE_ROOT}/overlay/src/go2_checkpoint_patrol/config/checkpoint-coordinator.production.yaml"
)" || xbf_fail "找不到 production coordinator 配置"
[[ -f "${localizer_template}" ]] ||
  xbf_fail "找不到 production localizer 配置：${localizer_template}"

sdk_interface="${GO2_SDK_IF:-eth0}"
sdk_receiver="$(
  xbf_absolute_file \
    "${XBF_FASTLIO_WORKSPACE}/build/go2_cmd_vel_bridge/go2_sdk2_udp_receiver"
)" || xbf_fail "找不到 SDK2 UDP receiver"
motion_probe="$(
  xbf_absolute_file \
    "${XBF_FASTLIO_WORKSPACE}/build/go2_cmd_vel_bridge/go2_sdk2_motion_probe"
)" || xbf_fail "找不到 SDK2 StopMove probe"

patrol_speed="${GO2_XBF_PATROL_SPEED:-0.20}"
max_yaw_rate="${GO2_XBF_MAX_YAW_RATE:-0.450}"
loop_mode="${GO2_XBF_LOOP_MODE:-once}"
calibration_only=0
case "${GO2_XBF_CALIBRATION_ONLY:-0}" in
  1|true|TRUE|yes|YES)
    calibration_only=1
    patrol_speed="0.0"
    max_yaw_rate="0.0"
    ;;
esac

manifest_sha="$(sha256sum "${XBF_MAP_MANIFEST}" | awk '{print $1}')"
runtime_identity="$(
  python3 - "${XBF_MAP_MANIFEST}" "${XBF_CHECKPOINT_FILE}" <<'PY'
import json
import sys
from pathlib import Path

manifest = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
sidecar = json.loads(Path(sys.argv[2]).read_text(encoding="utf-8"))
print(manifest["map_id"])
print(sidecar["source_csv_sha256"])
print(sidecar["source_pcd_sha256"])
PY
)"
map_id="$(printf '%s\n' "${runtime_identity}" | sed -n '1p')"
source_csv_sha="$(printf '%s\n' "${runtime_identity}" | sed -n '2p')"
source_pcd_sha="$(printf '%s\n' "${runtime_identity}" | sed -n '3p')"

umask 077
mkdir -p "${XBF_RUNTIME_DIR}"
log_dir="${XBF_RUNTIME_DIR}/logs"
mkdir -p "${log_dir}"
pid_file="${XBF_RUNTIME_DIR}/patrol.pids"
runtime_localizer_config="${XBF_RUNTIME_DIR}/localizer.runtime.yaml"
runtime_coordinator_config="${XBF_RUNTIME_DIR}/coordinator.runtime.yaml"
session_exec="${XBF_SCRIPT_DIR}/xbf_session_exec.py"
group_guard="${XBF_SCRIPT_DIR}/xbf_group_guard.py"
route_ready_waiter="${XBF_SCRIPT_DIR}/wait_route_ready.py"
timestamp_probe="${XBF_SCRIPT_DIR}/measure_input_timestamps.py"
for helper in \
  "${session_exec}" \
  "${group_guard}" \
  "${route_ready_waiter}" \
  "${timestamp_probe}"; do
  [[ -x "${helper}" ]] || xbf_fail "运行辅助程序不可执行：${helper}"
done

if [[ -f "${pid_file}" ]]; then
  old_pid_value() {
    local key="$1"
    awk -F= -v wanted="${key}" '$1 == wanted {print $2}' "${pid_file}"
  }
  old_supervisor="$(old_pid_value supervisor_pid)"
  old_supervisor_start_ticks="$(old_pid_value supervisor_start_ticks)"
  old_run_id="$(old_pid_value run_id)"
  if [[ "${old_supervisor}" =~ ^[0-9]+$ ]] &&
    kill -0 "${old_supervisor}" 2>/dev/null &&
    [[ -r "/proc/${old_supervisor}/cmdline" ]] &&
    tr '\0' ' ' <"/proc/${old_supervisor}/cmdline" |
      grep -Fq "start_xbf_patrol.sh"; then
    actual_start_ticks="$(
      xbf_process_start_ticks "${old_supervisor}" 2>/dev/null || true
    )"
    if [[ ! "${old_supervisor_start_ticks}" =~ ^[0-9]+$ ||
      "${actual_start_ticks}" == "${old_supervisor_start_ticks}" ]]; then
      xbf_fail \
        "已有 XBF 巡检监督进程 PID ${old_supervisor}；请先运行 stop_xbf_patrol.sh"
    fi
  fi
  [[ "${old_run_id}" =~ ^xbf-[0-9a-f]{24}$ ]] ||
    xbf_fail \
      "发现无法确认归属的旧运行记录 ${pid_file}；请先运行 stop_xbf_patrol.sh"
  stale_groups=()
  for old_key in \
    localizer_pgid \
    coordinator_pgid \
    sdk_receiver_pgid \
    cmd_vel_sender_pgid \
    safe_cmd_pgid \
    follower_pgid; do
    old_pgid="$(old_pid_value "${old_key}")"
    if [[ "${old_pgid}" =~ ^[0-9]+$ ]] &&
      "${group_guard}" status \
        "${old_pgid}" "${old_run_id}" >/dev/null 2>&1; then
      stale_groups+=("${old_key}=${old_pgid}")
    fi
  done
  if ((${#stale_groups[@]} > 0)); then
    xbf_fail \
      "发现上一次 XBF 遗留进程组（${stale_groups[*]}）；拒绝覆盖记录，请先运行 stop_xbf_patrol.sh"
  fi
  rm -f -- "${pid_file}"
fi

python3 - \
  "${localizer_template}" \
  "${runtime_localizer_config}" \
  "${coordinator_config}" \
  "${runtime_coordinator_config}" \
  "${XBF_ROUTE_FILE}" \
  "${XBF_CHECKPOINT_FILE}" \
  "${map_id}" \
  "${manifest_sha}" \
  "${source_csv_sha}" \
  "${source_pcd_sha}" \
  "${XBF_MAP_MANIFEST}" <<'PY'
import json
import re
import sys
from pathlib import Path

source = Path(sys.argv[1]).read_text(encoding="utf-8")
updated, count = re.subn(
    r"^(\s*input_extrinsics_verified:)\s*(?:true|false)\s*$",
    r"\1 true",
    source,
    count=1,
    flags=re.MULTILINE,
)
if count != 1:
    raise SystemExit("localizer 参数中没有唯一的 input_extrinsics_verified")
updated, count = re.subn(
    r"^(\s*map_manifest_path:)\s*.*$",
    lambda match: f"{match.group(1)} {json.dumps(sys.argv[11])}",
    updated,
    count=1,
    flags=re.MULTILINE,
)
if count != 1:
    raise SystemExit("localizer 参数中没有唯一的 map_manifest_path")
Path(sys.argv[2]).write_text(updated, encoding="utf-8")

coordinator_source = Path(sys.argv[3]).read_text(encoding="utf-8")
placeholder_counts = {
    "__RUNTIME_FROM_MANIFEST__": 1,
    "__RUNTIME_FROM_MANIFEST_SHA256__": 1,
    "__RUNTIME_FROM_CHECKPOINT_SIDECAR__": 2,
}
for placeholder, expected_count in placeholder_counts.items():
    if coordinator_source.count(placeholder) != expected_count:
        raise SystemExit(
            f"coordinator 参数模板中的 {placeholder} 数量错误"
        )

coordinator_parameters = {
    "route_file": sys.argv[5],
    "checkpoint_file": sys.argv[6],
    "expected_map_id": sys.argv[7],
    "expected_map_hash": sys.argv[8],
    "expected_source_csv_sha256": sys.argv[9],
    "expected_source_pcd_sha256": sys.argv[10],
}
coordinator_updated = coordinator_source
for name, value in coordinator_parameters.items():
    pattern = rf"^(\s*{re.escape(name)}:)\s*.*$"
    coordinator_updated, count = re.subn(
        pattern,
        lambda match, scalar=json.dumps(value): (
            f"{match.group(1)} {scalar}"
        ),
        coordinator_updated,
        count=1,
        flags=re.MULTILINE,
    )
    if count != 1:
        raise SystemExit(f"coordinator 参数中没有唯一的 {name}")
Path(sys.argv[4]).write_text(coordinator_updated, encoding="utf-8")
PY

localizer_pid=""
localizer_pgid=""
coordinator_pid=""
coordinator_pgid=""
sdk_receiver_pid=""
sdk_receiver_pgid=""
cmd_vel_sender_pid=""
cmd_vel_sender_pgid=""
safe_cmd_pid=""
safe_cmd_pgid=""
follower_pid=""
follower_pgid=""
cleanup_started=0
run_id="xbf-$(python3 -c 'import secrets; print(secrets.token_hex(12))')"
supervisor_start_ticks="$(xbf_process_start_ticks "$$")" ||
  xbf_fail "无法读取当前监督进程的 Linux starttime"

write_pid_file() {
  local temporary="${pid_file}.tmp.$$"
  {
    echo "supervisor_pid=$$"
    echo "supervisor_start_ticks=${supervisor_start_ticks}"
    echo "run_id=${run_id}"
    echo "localizer_pid=${localizer_pid}"
    echo "localizer_pgid=${localizer_pgid}"
    echo "coordinator_pid=${coordinator_pid}"
    echo "coordinator_pgid=${coordinator_pgid}"
    echo "sdk_receiver_pid=${sdk_receiver_pid}"
    echo "sdk_receiver_pgid=${sdk_receiver_pgid}"
    echo "cmd_vel_sender_pid=${cmd_vel_sender_pid}"
    echo "cmd_vel_sender_pgid=${cmd_vel_sender_pgid}"
    echo "safe_cmd_pid=${safe_cmd_pid}"
    echo "safe_cmd_pgid=${safe_cmd_pgid}"
    echo "follower_pid=${follower_pid}"
    echo "follower_pgid=${follower_pgid}"
    echo "sdk_interface=${sdk_interface}"
    echo "motion_probe=${motion_probe}"
  } >"${temporary}"
  chmod 600 "${temporary}"
  mv -f -- "${temporary}" "${pid_file}"
}

process_group_alive() {
  local pgid="$1"
  [[ "${pgid}" =~ ^[0-9]+$ ]] || return 1
  "${group_guard}" status "${pgid}" "${run_id}" >/dev/null 2>&1
}

spawn_component() {
  local label="$1"
  local log_file="$2"
  local pid_variable="$3"
  local pgid_variable="$4"
  local pid pgid sid attempt
  shift 4
  echo "启动 ${label}……"
  GO2_XBF_RUN_ID="${run_id}" "${session_exec}" "$@" \
    >"${log_file}" 2>&1 &
  pid=$!
  for attempt in $(seq 1 50); do
    if ! kill -0 "${pid}" 2>/dev/null; then
      wait "${pid}" 2>/dev/null || true
      xbf_fail "${label} 在建立独立进程组前退出；查看 ${log_file}"
      return 1
    fi
    pgid="$(ps -o pgid= -p "${pid}" 2>/dev/null | tr -d ' ')"
    sid="$(ps -o sid= -p "${pid}" 2>/dev/null | tr -d ' ')"
    if [[ "${pgid}" == "${pid}" && "${sid}" == "${pid}" ]] &&
      GO2_XBF_RUN_ID="${run_id}" \
        "${group_guard}" status "${pgid}" "${run_id}" >/dev/null 2>&1; then
      printf -v "${pid_variable}" '%s' "${pid}"
      printf -v "${pgid_variable}" '%s' "${pgid}"
      write_pid_file
      return 0
    fi
    sleep 0.1
  done
  kill -TERM "${pid}" 2>/dev/null || true
  wait "${pid}" 2>/dev/null || true
  xbf_fail "${label} 未在 5 秒内建立可确认的 PID=PGID=SID 会话"
}

wait_for_udp_receiver() {
  local timeout_sec="$1"
  local started now
  started="$(date +%s)"
  while true; do
    process_group_alive "${sdk_receiver_pgid}" || return 1
    if xbf_udp_port_in_use 5005; then
      return 0
    fi
    now="$(date +%s)"
    if ((now - started >= timeout_sec)); then
      return 1
    fi
    sleep 1
  done
}

require_pre_bridge_graph_clean() {
  local cmd_vel_publishers cmd_vel_subscribers
  local legacy_publishers legacy_subscribers
  cmd_vel_publishers="$(xbf_topic_count /cmd_vel publisher)"
  cmd_vel_subscribers="$(xbf_topic_count /cmd_vel subscriber)"
  legacy_publishers="$(xbf_topic_count /patrol_cmd publisher)"
  legacy_subscribers="$(xbf_topic_count /patrol_cmd subscriber)"
  [[ "${cmd_vel_publishers}" == "1" ]] ||
    xbf_fail \
      "运动桥接通前 /cmd_vel 发布者应只有本任务 safe cmd，实际 ${cmd_vel_publishers}"
  [[ "${cmd_vel_subscribers}" == "0" ]] ||
    xbf_fail \
      "运动桥接通前 /cmd_vel 已有 ${cmd_vel_subscribers} 个订阅者；可能残留旧 UDP sender"
  [[ "${legacy_publishers}" == "0" && "${legacy_subscribers}" == "0" ]] ||
    xbf_fail \
      "运动桥接通前检测到旧 /patrol_cmd 图（pub=${legacy_publishers}, sub=${legacy_subscribers}）"
}

stop_process() {
  local pgid="$1"
  local label="$2"
  local attempt
  [[ "${pgid}" =~ ^[0-9]+$ ]] || return 0
  process_group_alive "${pgid}" || return 0
  echo "停止 ${label}（进程组 ${pgid}）……"
  "${group_guard}" signal "${pgid}" "${run_id}" SIGINT >/dev/null 2>&1 || true
  for attempt in 1 2 3 4 5; do
    process_group_alive "${pgid}" || return 0
    sleep 1
  done
  "${group_guard}" signal "${pgid}" "${run_id}" SIGTERM >/dev/null 2>&1 || true
  sleep 1
  process_group_alive "${pgid}" &&
    "${group_guard}" signal "${pgid}" "${run_id}" SIGKILL >/dev/null 2>&1 || true
}

send_final_stopmove() {
  echo "通过 SDK2 motion probe 发送最终 StopMove……"
  "${motion_probe}" --iface "${sdk_interface}" stop \
    >>"${log_dir}/stopmove.log" 2>&1 || {
      echo "警告：StopMove probe 返回失败；查看 ${log_dir}/stopmove.log" >&2
      return 0
    }
}

cleanup() {
  local exit_code=$?
  if [[ "${cleanup_started}" == "1" ]]; then
    return
  fi
  cleanup_started=1
  trap - EXIT INT TERM

  # follower 先停止；协调器、safe cmd 和 UDP 链保持运行，让零速度持续
  # 到达 SDK2 receiver，避免 receiver 留住最后一帧非零命令。
  stop_process "${follower_pgid}" "原 waypoint follower"
  sleep 2
  stop_process "${localizer_pgid}" "地图定位器"
  sleep 1
  stop_process "${coordinator_pgid}" "checkpoint 协调器"
  sleep 1
  stop_process "${safe_cmd_pgid}" "原 Unitree safe cmd"
  stop_process "${cmd_vel_sender_pgid}" "cmd_vel UDP sender"
  stop_process "${sdk_receiver_pgid}" "SDK2 UDP receiver"
  send_final_stopmove

  rm -f -- \
    "${pid_file}" \
    "${runtime_localizer_config}" \
    "${runtime_coordinator_config}"
  echo "XBF 巡检进程已停止。日志保留在：${log_dir}"
  exit "${exit_code}"
}

signal_exit() {
  exit 130
}

trap cleanup EXIT
trap signal_exit INT TERM

timestamp_probe_duration="${GO2_XBF_TIMESTAMP_PROBE_SECONDS:-10}"
if ! "${timestamp_probe}" \
  --duration-sec "${timestamp_probe_duration}" \
  --maximum-age-sec 0.50 \
  --maximum-future-sec 0.10 \
  --output "${log_dir}/input_timestamps.json" \
  >"${log_dir}/input_timestamps.log" 2>&1; then
  case "${GO2_XBF_ALLOW_UNVERIFIED_TIMESTAMPS:-0}" in
    1|true|TRUE|yes|YES)
      echo "警告：输入时间戳预检未通过，已按显式 override 继续。" >&2
      ;;
    *)
      xbf_fail \
        "输入时间戳年龄未满足 localizer 的 [-0.10, 0.50] 秒约束；查看 ${log_dir}/input_timestamps.json"
      ;;
  esac
fi

spawn_component \
  "地图定位器" \
  "${log_dir}/localizer.log" \
  localizer_pid \
  localizer_pgid \
  ros2 launch go2_map_localizer localizer.launch.py \
  map_manifest:="${XBF_MAP_MANIFEST}" \
  params_file:="${runtime_localizer_config}"

if ! xbf_wait_for_service /localization/set_active 30; then
  xbf_fail "等待 /localization/set_active 超时；查看 ${log_dir}/localizer.log"
fi

spawn_component \
  "checkpoint 协调器（独占内部 gated command topic）" \
  "${log_dir}/coordinator.log" \
  coordinator_pid \
  coordinator_pgid \
  ros2 run go2_checkpoint_patrol checkpoint-localization-coordinator \
  --ros-args \
  --params-file "${runtime_coordinator_config}"

if ! xbf_wait_for_publisher_count /checkpoint_localization/gated_cmd 1 15; then
  xbf_fail \
    "内部 gated_cmd 未形成唯一协调器发布者；查看 ${log_dir}/coordinator.log"
fi

spawn_component \
  "原 unitree_safe_cmd_node（只订阅独占 gated command）" \
  "${log_dir}/safe_cmd.log" \
  safe_cmd_pid \
  safe_cmd_pgid \
  ros2 run go2_fastlio_patrol unitree_safe_cmd_node \
  --ros-args \
  -p cmd_topic:=/checkpoint_localization/gated_cmd \
  -p pointcloud_topic:=/cloud_registered_body \
  -p sport_request_topic:=/api/sport/request \
  -p output_cmd_topic:=/cmd_vel \
  -p max_vx:="${patrol_speed}" \
  -p max_vy:=0.000 \
  -p max_yaw_rate:="${max_yaw_rate}" \
  -p publish_rate:=20.0 \
  -p cmd_timeout:=0.500 \
  -p cloud_timeout:=1.000 \
  -p stop_distance:=0.80 \
  -p resume_distance:=1.00 \
  -p min_stop_points:=15 \
  -p roi_x_min:=0.35 \
  -p roi_x_max:=1.50 \
  -p roi_y_min:=-0.30 \
  -p roi_y_max:=0.30 \
  -p roi_z_min:=0.30 \
  -p roi_z_max:=0.90

if ! xbf_wait_for_min_subscriber_count /checkpoint_localization/gated_cmd 1 15; then
  xbf_fail \
    "safe cmd 没有订阅内部 gated_cmd；查看 ${log_dir}/safe_cmd.log"
fi
if ! xbf_wait_for_publisher_count /cmd_vel 1 15; then
  xbf_fail \
    "/cmd_vel 未形成唯一 safe cmd 发布者；查看 ${log_dir}/safe_cmd.log"
fi

require_pre_bridge_graph_clean
xbf_udp_port_in_use 5005 &&
  xbf_fail "运动桥接通前 UDP 5005 被其他进程占用"

spawn_component \
  "SDK2 UDP receiver（${sdk_interface}, 127.0.0.1:5005）" \
  "${log_dir}/sdk_receiver.log" \
  sdk_receiver_pid \
  sdk_receiver_pgid \
  env GO2_SDK_MAX_VY=0.020 \
  "${sdk_receiver}" "${sdk_interface}" 5005
if ! wait_for_udp_receiver 30; then
  xbf_fail \
    "SDK2 UDP receiver 未在 30 秒内监听 UDP 5005；查看 ${log_dir}/sdk_receiver.log"
fi

# receiver 已经能接收 SDK2 命令，但 sender 尚未启动；再次确认没有旧
# /cmd_vel 桥或 SaaS /patrol_cmd 在 preflight 之后抢入图，再接通唯一 sender。
require_pre_bridge_graph_clean

spawn_component \
  "cmd_vel UDP sender（限幅与当前巡检模式一致）" \
  "${log_dir}/cmd_vel_sender.log" \
  cmd_vel_sender_pid \
  cmd_vel_sender_pgid \
  ros2 run go2_cmd_vel_bridge cmd_vel_udp_sender \
  --ros-args \
  -p target_ip:=127.0.0.1 \
  -p target_port:=5005 \
  -p max_vx:="${patrol_speed}" \
  -p max_vy:=0.000 \
  -p max_vyaw:="${max_yaw_rate}"
if ! xbf_wait_for_min_subscriber_count /cmd_vel 1 15; then
  xbf_fail \
    "没有运动桥订阅 /cmd_vel；查看 ${log_dir}/cmd_vel_sender.log"
fi

spawn_component \
  "原 waypoint_follower_go2_2（控制算法不变，只重接 odom/cmd Topic）" \
  "${log_dir}/follower.log" \
  follower_pid \
  follower_pgid \
  ros2 run go2_fastlio_patrol waypoint_follower_go2_2 \
  --ros-args \
  -p route_file:="${XBF_ROUTE_FILE}" \
  -p odom_topic:=/checkpoint_localization/aligned_odometry \
  -p cmd_topic:=/checkpoint_localization/follower_cmd \
  -p v_base:="${patrol_speed}" \
  -p max_vx:="${patrol_speed}" \
  -p k_yaw:=0.900 \
  -p max_yaw_rate:="${max_yaw_rate}" \
  -p lookahead_distance:=0.600 \
  -p reach_distance:=0.400 \
  -p goal_distance:=0.250 \
  -p loop_mode:="${loop_mode}" \
  -p search_window:=6 \
  -p turn_in_place_angle:=1.000 \
  -p slow_down_angle:=0.500 \
  -p stuck_time:=3.000 \
  -p relocalize_distance:=1.500

if ! xbf_wait_for_publisher_count \
  /checkpoint_localization/follower_cmd 1 20; then
  xbf_fail \
    "follower_cmd 未形成唯一 follower 发布者；查看 ${log_dir}/follower.log"
fi
if ! xbf_wait_for_min_subscriber_count \
  /checkpoint_localization/aligned_odometry 1 20; then
  xbf_fail \
    "follower 未订阅 aligned_odometry；查看 ${log_dir}/follower.log"
fi

echo "等待起点定位完成并由 coordinator 明确进入 RUNNING……"
route_ready_timeout="${GO2_XBF_ROUTE_READY_TIMEOUT_SEC:-120}"
if ! "${route_ready_waiter}" \
  --timeout-sec "${route_ready_timeout}" \
  --output "${log_dir}/route_ready.json" \
  >"${log_dir}/route_ready.log" 2>&1; then
  xbf_fail \
    "起点定位未在 ${route_ready_timeout} 秒内进入 RUNNING；查看 ${log_dir}/route_ready.json 和 coordinator.log"
fi

echo
echo "XBF 巡检链已启动："
echo "  map_id: ${map_id}"
echo "  manifest_sha256: ${manifest_sha}"
echo "  route: ${XBF_ROUTE_FILE}"
echo "  logs: ${log_dir}"
echo "  运动链：gated_cmd -> safe_cmd -> /cmd_vel -> UDP 5005 -> SDK2"
echo "  SDK 网卡：${sdk_interface}"
if [[ "${calibration_only}" == "1" ]]; then
  echo "  模式：只校准不运动（follower 线速度/角速度上限均为 0）"
else
  echo "  模式：正式巡检，速度 ${patrol_speed} m/s"
fi
echo "  状态：ros2 topic echo /checkpoint_localization/route_status"
echo "按 Ctrl+C 或运行 scripts/stop_xbf_patrol.sh 停止。"

while true; do
  for process_spec in \
    "${localizer_pgid}:地图定位器" \
    "${coordinator_pgid}:checkpoint协调器" \
    "${sdk_receiver_pgid}:SDK2 UDP receiver" \
    "${cmd_vel_sender_pgid}:cmd_vel UDP sender" \
    "${safe_cmd_pgid}:Unitree safe cmd" \
    "${follower_pgid}:waypoint follower"; do
    process_pgid="${process_spec%%:*}"
    process_label="${process_spec#*:}"
    if ! process_group_alive "${process_pgid}"; then
      xbf_fail "${process_label} 已退出；请检查 ${log_dir}"
    fi
  done
  gated_count="$(
    xbf_topic_count /checkpoint_localization/gated_cmd publisher
  )"
  gated_subscriber_count="$(
    xbf_topic_count /checkpoint_localization/gated_cmd subscriber
  )"
  cmd_vel_count="$(xbf_topic_count /cmd_vel publisher)"
  cmd_vel_subscriber_count="$(xbf_topic_count /cmd_vel subscriber)"
  legacy_publisher_count="$(xbf_topic_count /patrol_cmd publisher)"
  legacy_subscriber_count="$(xbf_topic_count /patrol_cmd subscriber)"
  [[ "${gated_count}" == "1" ]] ||
    xbf_fail "内部 gated_cmd 发布者变为 ${gated_count}；停止以避免旁路"
  [[ "${gated_subscriber_count}" =~ ^[0-9]+$ ]] &&
    ((gated_subscriber_count >= 1)) ||
    xbf_fail "内部 gated_cmd 没有订阅者；停止以避免失去运动控制"
  [[ "${cmd_vel_count}" == "1" ]] ||
    xbf_fail "/cmd_vel 发布者变为 ${cmd_vel_count}；停止以避免双控制"
  [[ "${cmd_vel_subscriber_count}" =~ ^[0-9]+$ ]] &&
    ((cmd_vel_subscriber_count >= 1)) ||
    xbf_fail "/cmd_vel 没有订阅者；停止以避免命令没有送到 SDK2"
  [[ "${legacy_publisher_count}" == "0" ]] ||
    xbf_fail "检测到 SaaS 旧 follower 发布 /patrol_cmd；停止本任务以避免双巡检"
  [[ "${legacy_subscriber_count}" == "0" ]] ||
    xbf_fail "检测到 SaaS 旧 safe node 订阅 /patrol_cmd；停止本任务以避免双巡检"
  # coordinator 内部的 graph guard 以 40 Hz 立即锁停；这里每秒做一次
  # 进程级复核并负责结束整个任务，避免频繁启动 ros2 CLI 影响算力。
  sleep 1
done

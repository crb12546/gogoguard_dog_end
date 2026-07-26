#!/usr/bin/env bash
set -euo pipefail

# shellcheck source=_xbf_patrol_common.sh
source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)/_xbf_patrol_common.sh"

usage() {
  cat >&2 <<'EOF'
用法：
  GO2_INPUT_EXTRINSICS_VERIFIED=1 \
    start_xbf_patrol.sh [reviewed-map目录] [路线CSV] [checkpoint.json]

不传参数时使用交付包内的 R2 地图、路线和 checkpoint。
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

if [[ -f "${pid_file}" ]]; then
  old_supervisor="$(awk -F= '$1 == "supervisor_pid" {print $2}' "${pid_file}")"
  if [[ "${old_supervisor}" =~ ^[0-9]+$ ]] &&
    kill -0 "${old_supervisor}" 2>/dev/null; then
    xbf_fail "已有 XBF 巡检监督进程 PID ${old_supervisor}；请先运行 stop_xbf_patrol.sh"
  fi
  rm -f -- "${pid_file}"
fi

python3 - "${localizer_template}" "${runtime_localizer_config}" <<'PY'
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
Path(sys.argv[2]).write_text(updated, encoding="utf-8")
PY

localizer_pid=""
coordinator_pid=""
sdk_receiver_pid=""
cmd_vel_sender_pid=""
safe_cmd_pid=""
follower_pid=""
cleanup_started=0

write_pid_file() {
  {
    echo "supervisor_pid=$$"
    echo "localizer_pid=${localizer_pid}"
    echo "localizer_pgid=${localizer_pid}"
    echo "coordinator_pid=${coordinator_pid}"
    echo "coordinator_pgid=${coordinator_pid}"
    echo "sdk_receiver_pid=${sdk_receiver_pid}"
    echo "sdk_receiver_pgid=${sdk_receiver_pid}"
    echo "cmd_vel_sender_pid=${cmd_vel_sender_pid}"
    echo "cmd_vel_sender_pgid=${cmd_vel_sender_pid}"
    echo "safe_cmd_pid=${safe_cmd_pid}"
    echo "safe_cmd_pgid=${safe_cmd_pid}"
    echo "follower_pid=${follower_pid}"
    echo "follower_pgid=${follower_pid}"
    echo "sdk_interface=${sdk_interface}"
    echo "motion_probe=${motion_probe}"
  } >"${pid_file}"
}

process_group_alive() {
  local pgid="$1"
  [[ "${pgid}" =~ ^[0-9]+$ ]] || return 1
  kill -0 -- "-${pgid}" 2>/dev/null
}

wait_for_udp_receiver() {
  local timeout_sec="$1"
  local started now
  started="$(date +%s)"
  while true; do
    process_group_alive "${sdk_receiver_pid}" || return 1
    if ss -H -lun 2>/dev/null |
      awk '$5 ~ /:5005$/ {found=1} END {exit found ? 0 : 1}'; then
      return 0
    fi
    now="$(date +%s)"
    if ((now - started >= timeout_sec)); then
      return 1
    fi
    sleep 1
  done
}

stop_process() {
  local pgid="$1"
  local label="$2"
  local attempt
  [[ "${pgid}" =~ ^[0-9]+$ ]] || return 0
  process_group_alive "${pgid}" || return 0
  echo "停止 ${label}（进程组 ${pgid}）……"
  kill -INT -- "-${pgid}" 2>/dev/null || true
  for attempt in 1 2 3 4 5; do
    process_group_alive "${pgid}" || return 0
    sleep 1
  done
  kill -TERM -- "-${pgid}" 2>/dev/null || true
  sleep 1
  process_group_alive "${pgid}" &&
    kill -KILL -- "-${pgid}" 2>/dev/null || true
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
  stop_process "${follower_pid}" "原 waypoint follower"
  sleep 2
  stop_process "${localizer_pid}" "地图定位器"
  sleep 1
  stop_process "${coordinator_pid}" "checkpoint 协调器"
  sleep 1
  stop_process "${safe_cmd_pid}" "原 Unitree safe cmd"
  stop_process "${cmd_vel_sender_pid}" "cmd_vel UDP sender"
  stop_process "${sdk_receiver_pid}" "SDK2 UDP receiver"
  send_final_stopmove

  rm -f -- "${pid_file}" "${runtime_localizer_config}"
  echo "XBF 巡检进程已停止。日志保留在：${log_dir}"
  exit "${exit_code}"
}

signal_exit() {
  exit 130
}

trap cleanup EXIT
trap signal_exit INT TERM

echo "启动地图定位器……"
setsid ros2 launch go2_map_localizer localizer.launch.py \
  map_manifest:="${XBF_MAP_MANIFEST}" \
  params_file:="${runtime_localizer_config}" \
  >"${log_dir}/localizer.log" 2>&1 &
localizer_pid=$!
write_pid_file

if ! xbf_wait_for_service /localization/set_active 30; then
  xbf_fail "等待 /localization/set_active 超时；查看 ${log_dir}/localizer.log"
fi

echo "启动 checkpoint 协调器；它独占内部 gated command topic……"
setsid ros2 run go2_checkpoint_patrol checkpoint-localization-coordinator \
  --ros-args \
  --params-file "${coordinator_config}" \
  -p integration_enabled:=true \
  -p route_file:="${XBF_ROUTE_FILE}" \
  -p checkpoint_file:="${XBF_CHECKPOINT_FILE}" \
  -p expected_map_id:="${map_id}" \
  -p expected_map_hash:="${manifest_sha}" \
  -p expected_source_csv_sha256:="${source_csv_sha}" \
  -p expected_source_pcd_sha256:="${source_pcd_sha}" \
  -p gated_cmd_topic:=/checkpoint_localization/gated_cmd \
  -p actuator_command_topic:=/cmd_vel \
  >"${log_dir}/coordinator.log" 2>&1 &
coordinator_pid=$!
write_pid_file

if ! xbf_wait_for_publisher_count /checkpoint_localization/gated_cmd 1 15; then
  xbf_fail \
    "内部 gated_cmd 未形成唯一协调器发布者；查看 ${log_dir}/coordinator.log"
fi

echo "启动 SDK2 UDP receiver（${sdk_interface}, 127.0.0.1:5005）……"
setsid env GO2_SDK_MAX_VY=0.020 \
  "${sdk_receiver}" "${sdk_interface}" 5005 \
  >"${log_dir}/sdk_receiver.log" 2>&1 &
sdk_receiver_pid=$!
write_pid_file
if ! wait_for_udp_receiver 10; then
  xbf_fail \
    "SDK2 UDP receiver 未在 10 秒内监听 UDP 5005；查看 ${log_dir}/sdk_receiver.log"
fi

echo "启动 cmd_vel UDP sender；限幅与当前巡检模式一致……"
setsid ros2 run go2_cmd_vel_bridge cmd_vel_udp_sender \
  --ros-args \
  -p target_ip:=127.0.0.1 \
  -p target_port:=5005 \
  -p max_vx:="${patrol_speed}" \
  -p max_vy:=0.000 \
  -p max_vyaw:="${max_yaw_rate}" \
  >"${log_dir}/cmd_vel_sender.log" 2>&1 &
cmd_vel_sender_pid=$!
write_pid_file
if ! xbf_wait_for_min_subscriber_count /cmd_vel 1 15; then
  xbf_fail \
    "没有运动桥订阅 /cmd_vel；查看 ${log_dir}/cmd_vel_sender.log"
fi

echo "启动原 unitree_safe_cmd_node；只订阅本任务独占的 gated command……"
setsid ros2 run go2_fastlio_patrol unitree_safe_cmd_node \
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
  -p roi_z_max:=0.90 \
  >"${log_dir}/safe_cmd.log" 2>&1 &
safe_cmd_pid=$!
write_pid_file

if ! xbf_wait_for_min_subscriber_count /checkpoint_localization/gated_cmd 1 15; then
  xbf_fail \
    "safe cmd 没有订阅内部 gated_cmd；查看 ${log_dir}/safe_cmd.log"
fi
if ! xbf_wait_for_publisher_count /cmd_vel 1 15; then
  xbf_fail \
    "/cmd_vel 未形成唯一 safe cmd 发布者；查看 ${log_dir}/safe_cmd.log"
fi

echo "启动原 waypoint_follower_go2_2；控制算法不变，仅重接 odom/cmd Topic……"
setsid ros2 run go2_fastlio_patrol waypoint_follower_go2_2 \
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
  -p relocalize_distance:=1.500 \
  >"${log_dir}/follower.log" 2>&1 &
follower_pid=$!
write_pid_file

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
    "${localizer_pid}:地图定位器" \
    "${coordinator_pid}:checkpoint协调器" \
    "${sdk_receiver_pid}:SDK2 UDP receiver" \
    "${cmd_vel_sender_pid}:cmd_vel UDP sender" \
    "${safe_cmd_pid}:Unitree safe cmd" \
    "${follower_pid}:waypoint follower"; do
    process_pid="${process_spec%%:*}"
    process_label="${process_spec#*:}"
    if ! process_group_alive "${process_pid}"; then
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

#!/usr/bin/env bash
set -eo pipefail

WS=${GO2_WS:-/home/unitree/go2_fastlio_ws}
ENV="$WS/scripts/env_common.sh"
LOG_DIR="$WS/patrol_logs"
IOX_STUB="$WS/cpp_tools/legacy_iox_stub/liblegacy_iox_stub.so"
CYCLONE_NO_SHM="$WS/config/cyclonedds_no_shm_eth0.xml"
mkdir -p "$LOG_DIR"

source "$ENV"

if [[ ! -f "$IOX_STUB" ]]; then
  "$WS/scripts/build_legacy_iox_stub.sh" >/dev/null
fi

existing=$(ps -eo pid=,comm=,args= | awk '($2 == "ros2" && $0 ~ /[w]aypoint_follower|[u]nitree_safe_cmd_node|[u]nitree_go_safe_cmd_node|[u]nitree_cmd_node/) || $0 ~ /[g]o2_control_by_sdk.*send_cmd/ {print $1}' | tr '\n' ' ' || true)
if [[ -n "$existing" ]]; then
  echo "CONTROL_ALREADY_RUNNING $existing"
  exit 0
fi

setsid nohup env LD_PRELOAD="$IOX_STUB:${LD_PRELOAD:-}" CYCLONEDDS_URI="file://$CYCLONE_NO_SHM" ros2 run go2_control_by_sdk send_cmd \
  </dev/null >"$LOG_DIR/legacy_go2_sdk_bridge.log" 2>&1 &
echo "LEGACY_SDK_BRIDGE_STARTED pid=$!"

sleep 1

setsid nohup ros2 run go2_fastlio_patrol unitree_go_safe_cmd_node --ros-args \
  -p max_vx:=0.2 \
  -p max_yaw_rate:=0.45 \
  -p publish_rate:=20.0 \
  -p pointcloud_topic:=/cloud_registered_body \
  -p go2_cmd_topic:=/go2_cmd \
  -p cmd_mode:=2 \
  -p gait_type:=0 \
  -p speed_level:=0 \
  -p stop_distance:=0.40 \
  -p resume_distance:=0.50 \
  -p min_stop_points:=15 \
  -p roi_x_min:=0.35 \
  -p roi_x_max:=0.90 \
  -p roi_y_min:=-0.30 \
  -p roi_y_max:=0.30 \
  -p roi_z_min:=0.30 \
  -p roi_z_max:=0.90 \
  </dev/null >"$LOG_DIR/legacy_go2_safe_cmd.log" 2>&1 &
echo "LEGACY_GO2_SAFE_STARTED pid=$!"
#!/usr/bin/env bash
set -eo pipefail

WS=${GO2_WS:-/home/unitree/go2_fastlio_ws}
ENV="$WS/scripts/env_common.sh"
LOG_DIR="$WS/patrol_logs"
IOX_STUB="$WS/cpp_tools/legacy_iox_stub/liblegacy_iox_stub.so"
CYCLONE_NO_SHM="$WS/config/cyclonedds_no_shm_eth0.xml"
LOG="$LOG_DIR/legacy_send_cmd_start_check.log"

source "$ENV"
mkdir -p "$LOG_DIR"

if [[ ! -f "$IOX_STUB" ]]; then
  "$WS/scripts/build_legacy_iox_stub.sh" >/dev/null
fi

"$WS/scripts/stop_legacy_go2_cmd_bridge.sh" >/dev/null 2>&1 || true

setsid nohup env LD_PRELOAD="$IOX_STUB:${LD_PRELOAD:-}" CYCLONEDDS_URI="file://$CYCLONE_NO_SHM" ros2 run go2_control_by_sdk send_cmd \
  </dev/null >"$LOG" 2>&1 &
pid=$!
echo "STARTED pid=$pid"

sleep 3

echo "__PROCESS__"
ps -eo pid,ppid,comm,args | awk '/go2_control_by_sdk|send_cmd/ && !/awk/ {print}'

echo "__LOG__"
tail -80 "$LOG" 2>/dev/null || true

pids=$(ps -eo pid=,comm=,args= | awk '$2 == "send_cmd" || ($2 == "ros2" && $0 ~ /go2_control_by_sdk/) {print $1}' | tr '\n' ' ' || true)
if [[ -n "$pids" ]]; then
  kill -TERM $pids 2>/dev/null || true
fi
sleep 1

remaining=$(ps -eo pid=,comm=,args= | awk '$2 == "send_cmd" || ($2 == "ros2" && $0 ~ /go2_control_by_sdk/) {print $1}' | tr '\n' ' ' || true)
if [[ -n "$remaining" ]]; then
  kill -KILL $remaining 2>/dev/null || true
fi

echo "__REMAINING__"
ps -eo pid,ppid,comm,args | awk '/go2_control_by_sdk|send_cmd/ && !/awk/ {print}'
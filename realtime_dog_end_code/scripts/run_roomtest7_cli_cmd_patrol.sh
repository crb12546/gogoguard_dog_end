#!/usr/bin/env bash
set -eo pipefail

WS=${GO2_WS:-/home/unitree/go2_fastlio_ws}
ENV="$WS/scripts/env_common.sh"
LOG_DIR="$WS/patrol_logs"
ROUTE_FILE=${ROUTE_FILE:-$WS/src/go2_fastlio_patrol/routes/roomtest7.csv}
SPEED=${SPEED:-0.10}
MAX_YAW=${MAX_YAW:-0.45}
LOOP_MODE=${LOOP_MODE:-once}
LOOKAHEAD=${LOOKAHEAD:-0.6}

source "$ENV"
mkdir -p "$LOG_DIR"

cmd_pid=""
follower_pid=""

stop_all() {
  echo "[stop] stopping waypoint_follower / unitree_cmd_node"
  if [[ -n "$follower_pid" ]]; then kill -TERM "$follower_pid" 2>/dev/null || true; fi
  if [[ -n "$cmd_pid" ]]; then kill -TERM "$cmd_pid" 2>/dev/null || true; fi
  pkill -TERM -f "[w]aypoint_follower" 2>/dev/null || true
  pkill -TERM -f "[u]nitree_cmd_node" 2>/dev/null || true
  sleep 1
  pkill -KILL -f "[w]aypoint_follower|[u]nitree_cmd_node" 2>/dev/null || true
  ros2 topic pub -1 /api/sport/request unitree_api/msg/Request "{header: {identity: {id: 9999, api_id: 1003}, lease: {id: 0}, policy: {priority: 0, noreply: false}}, parameter: '{}', binary: []}" >/dev/null 2>&1 || true
}

trap stop_all EXIT INT TERM

echo "=== roomtest7 old CLI cmd-chain patrol ==="
echo "route: $ROUTE_FILE"
echo "speed: $SPEED m/s, yaw max: $MAX_YAW rad/s, loop: $LOOP_MODE"
echo "chain: waypoint_follower -> unitree_cmd_node -> /api/sport/request"

if [[ ! -f "$ROUTE_FILE" ]]; then
  echo "[error] route file not found: $ROUTE_FILE" >&2
  exit 2
fi

lines=$(wc -l < "$ROUTE_FILE")
if [[ "$lines" -lt 3 ]]; then
  echo "[error] route has fewer than 2 points: $ROUTE_FILE lines=$lines" >&2
  exit 2
fi

for topic in /livox/lidar /livox/imu /Odometry; do
  if ! timeout 5 ros2 topic list | grep -qx "$topic"; then
    echo "[error] required topic missing: $topic" >&2
    exit 3
  fi
done

echo
echo "Put Go2 at the SAME start pose used when recording roomtest7."
echo "Keep the remote controller in hand. Press Enter to start, Ctrl+C to abort."
read -r _

cmd_node=(
  ros2 run go2_fastlio_patrol unitree_cmd_node --ros-args
  -p max_vx:="$SPEED"
  -p max_yaw_rate:="$MAX_YAW"
  -p publish_rate:=20.0
)

follower_cmd=(
  ros2 run go2_fastlio_patrol waypoint_follower --ros-args
  -p route_file:="$ROUTE_FILE"
  -p v_base:="$SPEED"
  -p max_vx:="$SPEED"
  -p k_yaw:=0.9
  -p max_yaw_rate:="$MAX_YAW"
  -p lookahead_distance:="$LOOKAHEAD"
  -p reach_distance:=0.4
  -p goal_distance:=0.25
  -p loop_mode:="$LOOP_MODE"
  -p search_window:=6
  -p turn_in_place_angle:=1.0
  -p slow_down_angle:=0.5
  -p stuck_time:=3.0
  -p relocalize_distance:=1.5
)

"${cmd_node[@]}" >"$LOG_DIR/roomtest7_cli_cmd_node.log" 2>&1 &
cmd_pid=$!
sleep 1
"${follower_cmd[@]}" >"$LOG_DIR/roomtest7_cli_follower.log" 2>&1 &
follower_pid=$!

echo "[running] cmd log: $LOG_DIR/roomtest7_cli_cmd_node.log"
echo "[running] follower log: $LOG_DIR/roomtest7_cli_follower.log"
echo "Press Ctrl+C to stop. This script will send StopMove on exit."

while true; do
  if ! kill -0 "$cmd_pid" 2>/dev/null; then echo "[error] unitree_cmd_node exited"; break; fi
  if ! kill -0 "$follower_pid" 2>/dev/null; then echo "[info] follower exited"; break; fi
  sleep 1
done
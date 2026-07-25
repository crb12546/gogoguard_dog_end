#!/usr/bin/env bash
set -eo pipefail

WS=${GO2_WS:-/home/unitree/go2_fastlio_ws}
ENV="$WS/scripts/env_common.sh"
LOG_DIR="$WS/patrol_logs"

source "$ENV"
mkdir -p "$LOG_DIR"

PROBE_VX=${PROBE_VX:-0.10}
PROBE_VYAW=${PROBE_VYAW:-0.0}
PROBE_DURATION=${PROBE_DURATION:-3.0}

cleanup() {
  "$WS/scripts/stop_legacy_go2_cmd_bridge.sh" >/dev/null 2>&1 || true
}
trap cleanup EXIT

echo "[probe] stopping stale legacy/control processes"
cleanup

echo "[probe] starting legacy /go2_cmd bridge"
"$WS/scripts/start_legacy_go2_cmd_bridge.sh"
sleep 2

echo "[probe] active processes"
ps -eo pid,ppid,comm,args | awk '/go2_control_by_sdk|unitree_go_safe_cmd_node|waypoint_follower|unitree_safe_cmd|unitree_cmd_node/ && !/awk/ {print}'

echo "[probe] publish /patrol_cmd vx=$PROBE_VX vyaw=$PROBE_VYAW duration=${PROBE_DURATION}s"
export PROBE_VX PROBE_VYAW PROBE_DURATION
python3 - <<'PY'
import os
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

rclpy.init()
node = Node('legacy_go2_cmd_probe_pub')
pub = node.create_publisher(Twist, '/patrol_cmd', 10)
time.sleep(0.4)

cmd = Twist()
cmd.linear.x = float(os.environ.get('PROBE_VX', '0.10'))
cmd.angular.z = float(os.environ.get('PROBE_VYAW', '0.0'))
duration = float(os.environ.get('PROBE_DURATION', '3.0'))
count = max(1, int(duration / 0.05))
for _ in range(count):
    pub.publish(cmd)
    rclpy.spin_once(node, timeout_sec=0.0)
    time.sleep(0.05)

zero = Twist()
for _ in range(16):
    pub.publish(zero)
    rclpy.spin_once(node, timeout_sec=0.0)
    time.sleep(0.05)

node.destroy_node()
rclpy.shutdown()
print('PUBLISH_DONE')
PY

sleep 1

echo "[probe] stopping legacy /go2_cmd bridge"
cleanup
sleep 1

echo "[probe] remaining control processes"
ps -eo pid,ppid,comm,args | awk '/go2_control_by_sdk|unitree_go_safe_cmd_node|waypoint_follower|unitree_safe_cmd|unitree_cmd_node/ && !/awk/ {print}'

echo "[probe] legacy sdk bridge log"
tail -80 "$LOG_DIR/legacy_go2_sdk_bridge.log" 2>/dev/null || true

echo "[probe] legacy safe bridge log"
tail -80 "$LOG_DIR/legacy_go2_safe_cmd.log" 2>/dev/null || true
#!/usr/bin/env bash
set -eo pipefail

pids=$(ps -eo pid=,comm=,args= | awk '$2 ~ /^unitree_go_safe/ || $2 ~ /^waypoint_follow/ || $2 ~ /^unitree_safe_cm/ || $2 ~ /^unitree_cmd/ || ($2 == "send_cmd") || ($2 == "ros2" && $0 ~ /[w]aypoint_follower|[u]nitree_go_safe_cmd_node|[u]nitree_safe_cmd_node|[u]nitree_cmd_node|[g]o2_control_by_sdk/) {print $1}' | tr '\n' ' ' || true)
if [[ -z "$pids" ]]; then
  echo "NOT_RUNNING"
  exit 0
fi

kill -TERM $pids 2>/dev/null || true
sleep 1

remaining=$(ps -eo pid=,comm=,args= | awk '$2 ~ /^unitree_go_safe/ || $2 ~ /^waypoint_follow/ || $2 ~ /^unitree_safe_cm/ || $2 ~ /^unitree_cmd/ || ($2 == "send_cmd") || ($2 == "ros2" && $0 ~ /[w]aypoint_follower|[u]nitree_go_safe_cmd_node|[u]nitree_safe_cmd_node|[u]nitree_cmd_node|[g]o2_control_by_sdk/) {print $1}' | tr '\n' ' ' || true)
if [[ -n "$remaining" ]]; then
  kill -KILL $remaining 2>/dev/null || true
fi

echo "LEGACY_GO2_BRIDGE_STOPPED"
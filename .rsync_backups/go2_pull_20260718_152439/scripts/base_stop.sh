#!/usr/bin/env bash
set +e

echo "[base_stop] stopping base launch processes..."

pkill -INT -f "ros2 launch fast_lio mapping.launch.py"
pkill -INT -f "ros2 launch livox_ros_driver2 msg_MID360s_launch.py"

sleep 1

pkill -TERM -f "ros2 launch fast_lio mapping.launch.py"
pkill -TERM -f "ros2 launch livox_ros_driver2 msg_MID360s_launch.py"

echo "[base_stop] done."

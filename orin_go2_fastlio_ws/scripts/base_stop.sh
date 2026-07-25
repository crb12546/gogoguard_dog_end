#!/usr/bin/env bash
set +e

echo "[base_stop] stopping base launch processes..."

pkill -INT -f "ros2 launch fast_lio mapping.launch.py"
pkill -INT -f "fastlio_mapping"
pkill -INT -f "ros2 launch livox_ros_driver2 msg_MID360s_launch.py"
pkill -INT -f "livox_ros_driver2_node"

sleep 2

pkill -TERM -f "ros2 launch fast_lio mapping.launch.py"
pkill -TERM -f "fastlio_mapping"
pkill -TERM -f "ros2 launch livox_ros_driver2 msg_MID360s_launch.py"
pkill -TERM -f "livox_ros_driver2_node"

sleep 1

pkill -KILL -f "ros2 launch fast_lio mapping.launch.py"
pkill -KILL -f "fastlio_mapping"
pkill -KILL -f "ros2 launch livox_ros_driver2 msg_MID360s_launch.py"
pkill -KILL -f "livox_ros_driver2_node"

echo "[base_stop] done."

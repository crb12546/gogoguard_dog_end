#!/bin/bash
set -e

SESSION="level_scan"
PITCH=${1:-13.0}

tmux kill-session -t $SESSION 2>/dev/null || true

pkill -f odom_to_tf_map.py 2>/dev/null || true
pkill -f odom_to_tf_map_2d.py 2>/dev/null || true
pkill -f odom_to_tf_map_level_2d.py 2>/dev/null || true
pkill -f level_cloud_node.py 2>/dev/null || true
pkill -f pointcloud_to_laserscan 2>/dev/null || true

source /opt/ros/foxy/setup.bash
source ~/go2_fastlio_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

wait_for_topic() {
    local topic_name=$1
    echo "[WAIT] $topic_name"
    until ros2 topic list 2>/dev/null | grep -q "^${topic_name}$"; do
        sleep 1
    done
    echo "[OK] $topic_name"
}

run_window() {
    local name=$1
    local cmd=$2

    if [ "$name" = "tf" ]; then
        tmux new-session -d -s $SESSION -n "$name"
    else
        tmux new-window -t $SESSION -n "$name"
    fi

    tmux send-keys -t "$SESSION:$name" \
"cd ~/go2_fastlio_ws; source /opt/ros/foxy/setup.bash; source install/setup.bash; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; export ROS_DOMAIN_ID=0; export ROS_LOCALHOST_ONLY=0; $cmd" C-m
}

echo "[INFO] Using level pitch: +${PITCH} deg"

wait_for_topic "/Odometry"
wait_for_topic "/cloud_registered_body"

echo "[1/3] Start leveled planar TF"
run_window "tf" "python3 ~/go2_fastlio_ws/src/go2_loop_backend/go2_loop_backend/odom_to_tf_map_level_2d.py --ros-args -p odom_topic:=/Odometry -p parent_frame:=map -p child_frame:=base_link -p level_pitch_deg:=${PITCH}"

echo "[2/3] Start online level cloud"
run_window "level_cloud" "python3 ~/go2_fastlio_ws/src/go2_loop_backend/go2_loop_backend/level_cloud_node.py --ros-args -p input_topic:=/cloud_registered_body -p output_topic:=/cloud_registered_body_level -p output_frame:=base_link -p pitch_deg:=${PITCH} -p point_stride:=1"

wait_for_topic "/cloud_registered_body_level"

echo "[3/3] Start pointcloud_to_laserscan -> /scan_level"
run_window "scan_level" "ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args -r cloud_in:=/cloud_registered_body_level -r scan:=/scan_level -p target_frame:=base_link -p transform_tolerance:=0.2 -p min_height:=0.05 -p max_height:=1.20 -p angle_min:=-3.14159 -p angle_max:=3.14159 -p angle_increment:=0.0087 -p scan_time:=0.1 -p range_min:=0.3 -p range_max:=20.0 -p use_inf:=true -p inf_epsilon:=1.0"

wait_for_topic "/scan_level"

echo ""
echo "========================================="
echo "[OK] level TF + level cloud + scan_level started"
echo "Pitch: +${PITCH} deg"
echo "tmux attach: tmux a -t $SESSION"
echo "stop: tmux kill-session -t $SESSION"
echo "========================================="

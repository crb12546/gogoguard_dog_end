# Copyright 2026 Go2 Robotics Team
# SPDX-License-Identifier: Apache-2.0
uint8 IDLE=0
uint8 WAITING_LOCALIZATION=1
uint8 READY=2
uint8 RUNNING=3
uint8 PAUSED=4
uint8 COMPLETED=5
uint8 CANCELLED=6
uint8 FAULT=7

std_msgs/Header header
string mission_id
string route_id
uint8 state
uint32 nearest_index
uint32 target_index
uint32 waypoint_count
float32 cross_track_error
float32 distance_remaining
float32 target_distance
bool localization_ready
string reason

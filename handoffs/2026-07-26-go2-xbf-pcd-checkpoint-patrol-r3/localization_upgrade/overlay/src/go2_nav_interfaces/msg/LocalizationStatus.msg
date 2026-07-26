# Copyright 2026 Go2 Robotics Team
# SPDX-License-Identifier: Apache-2.0
# State values are intentionally stable API. Add new values only at the end.
uint8 STATE_UNKNOWN=0
uint8 STATE_INITIALIZING=1
uint8 STATE_TRACKING=2
uint8 STATE_DEGRADED=3
uint8 STATE_LOST=4

std_msgs/Header header
uint8 state
bool map_valid
bool pose_valid
float32 confidence
# GICP correspondence RMSE in metres; -1 means unavailable.
float32 fitness_score
float32 pose_age
float32 correction_age
string map_id
string map_hash
bool safe_to_move
string reason
string[] reasons

# Registration diagnostics used by the console and incident logs.
float32 descriptor_distance
float32 inlier_ratio
float32 correction_translation
float32 correction_rotation
uint32 consecutive_accepts
uint32 consecutive_rejects
string active_tile_id
bool global_confirmation_pending
uint32 global_confirmation_accepts
float32 global_confirmation_span

# Startup repeatability only; this does not establish absolute pose accuracy.
bool startup_precision_verified
# Maximum three-dimensional translation from the first anchor, in metres.
float32 startup_max_translation_deviation
# Maximum wrapped yaw difference from the first anchor, in radians.
float32 startup_max_yaw_deviation

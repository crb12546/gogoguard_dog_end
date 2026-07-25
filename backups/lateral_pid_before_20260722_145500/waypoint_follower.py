#!/usr/bin/env python3
import csv
import json
import math
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from .patrol_control import (
    corner_heading_command,
    cumulative_turn_candidate,
    lateral_velocity_command,
)


def normalize_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')

        self.declare_parameter('odom_topic', '/Odometry')
        self.declare_parameter('cmd_topic', '/patrol_cmd')
        self.declare_parameter('route_file', '/home/unitree/go2_fastlio_ws/src/go2_fastlio_patrol/routes/route_demo.csv')
        self.declare_parameter('route_transform_json', '')

        self.declare_parameter('v_base', 0.25)
        self.declare_parameter('max_vx', 0.25)
        self.declare_parameter('use_route_speed', True)
        self.declare_parameter('k_yaw', 0.6)
        self.declare_parameter('max_yaw_rate', 0.30)
        self.declare_parameter('max_non_corner_yaw_rate', 0.35)
        self.declare_parameter('lateral_yaw_gain', 0.06)
        self.declare_parameter('max_lateral_yaw_rate', 0.05)
        self.declare_parameter('lateral_deadband', 0.03)
        self.declare_parameter('lateral_velocity_gain', 0.75)
        self.declare_parameter('max_vy', 0.15)
        self.declare_parameter('lateral_velocity_deadband', 0.03)
        self.declare_parameter('lateral_velocity_heading_limit_deg', 25.0)

        self.declare_parameter('lookahead_distance', 0.8)
        self.declare_parameter('dynamic_lookahead', True)
        self.declare_parameter('lookahead_straight_distance', 0.70)
        self.declare_parameter('lookahead_corner_distance', 0.25)
        self.declare_parameter('lookahead_curve_distance', 0.85)
        self.declare_parameter('lookahead_corner_angle_deg', 60.0)
        self.declare_parameter('lookahead_corner_scan_distance', 1.00)
        self.declare_parameter('lookahead_curve_angle_deg', 20.0)
        self.declare_parameter('lookahead_curve_scan_distance', 4.00)
        self.declare_parameter('curve_heading_preview_distance', 1.20)
        self.declare_parameter('curve_heading_blend', 0.35)
        self.declare_parameter('corner_turn_distance', 0.55)
        self.declare_parameter('corner_exit_distance', 0.90)
        self.declare_parameter('corner_wiggle_distance', 1.60)
        self.declare_parameter('corner_wiggle_min_opposite_angle_deg', 30.0)
        self.declare_parameter('corner_heading_blend', 0.20)
        self.declare_parameter('corner_heading_conflict_blend', 0.65)
        self.declare_parameter('corner_lateral_yaw_gain_scale', 0.40)
        self.declare_parameter('reach_distance', 0.4)
        self.declare_parameter('goal_distance', 0.5)

        self.declare_parameter('loop_mode', 'once')   # once / pingpong
        self.declare_parameter('search_window', 6)

        self.declare_parameter('turn_in_place_angle', 1.0)
        self.declare_parameter('slow_down_angle', 0.5)
        self.declare_parameter('turn_crawl_vx', 0.06)

        self.declare_parameter('stuck_time', 3.0)
        self.declare_parameter('soft_unstick_yaw_rate', 0.20)
        self.declare_parameter('soft_unstick_min_moved', 0.12)
        self.declare_parameter('progress_s_epsilon', 0.08)
        self.declare_parameter('progress_backtrack_tolerance', 0.12)
        self.declare_parameter('progress_forward_tolerance', 0.45)
        self.declare_parameter('projection_speed_scale', 1.2)
        self.declare_parameter('projection_step_margin', 0.01)
        self.declare_parameter('projection_min_step', 0.015)
        self.declare_parameter('projection_max_step', 0.12)
        self.declare_parameter('relocalize_distance', 1.0)
        self.declare_parameter('start_policy', 'route_start')  # route_start / nearest
        self.declare_parameter('start_max_distance', 0.5)
        self.declare_parameter('start_max_yaw_error', 0.9)
        self.declare_parameter('allow_global_relocalize', False)
        self.declare_parameter('stuck_relocalize', False)
        self.declare_parameter('projection_fail_limit', 3)
        self.declare_parameter('stop_on_route_deviation', True)
        self.declare_parameter('recovery_distance', 1.0)
        self.declare_parameter('recovery_heading_distance', 0.35)
        self.declare_parameter('recovery_yaw_error', 0.75)
        self.declare_parameter('recovery_vx', 0.20)
        self.declare_parameter('max_route_deviation', 5.0)
        self.declare_parameter('max_valid_odom_abs', 10000.0)
        self.declare_parameter('max_odom_jump_distance', 0.80)
        self.declare_parameter('max_odom_jump_z', 0.50)
        self.declare_parameter('max_odom_jump_dt', 2.00)
        self.declare_parameter('yaw_deadband', 0.06)
        self.declare_parameter('max_vx_accel', 0.35)
        self.declare_parameter('max_vy_accel', 0.20)
        self.declare_parameter('max_yaw_accel', 0.90)
        self.declare_parameter('yaw_wrap_hysteresis', 0.35)

        self.odom_topic = self.get_parameter('odom_topic').value
        self.cmd_topic = self.get_parameter('cmd_topic').value
        self.route_file = self.get_parameter('route_file').value
        self.route_transform_json = str(self.get_parameter('route_transform_json').value).strip()

        self.v_base = float(self.get_parameter('v_base').value)
        self.max_vx = float(self.get_parameter('max_vx').value)
        self.use_route_speed = bool(self.get_parameter('use_route_speed').value)
        self.k_yaw = float(self.get_parameter('k_yaw').value)
        self.max_yaw_rate = float(self.get_parameter('max_yaw_rate').value)
        self.max_non_corner_yaw_rate = max(
            0.0,
            float(self.get_parameter('max_non_corner_yaw_rate').value),
        )
        self.lateral_yaw_gain = max(0.0, float(self.get_parameter('lateral_yaw_gain').value))
        self.max_lateral_yaw_rate = max(0.0, float(self.get_parameter('max_lateral_yaw_rate').value))
        self.lateral_deadband = max(0.0, float(self.get_parameter('lateral_deadband').value))
        self.lateral_velocity_gain = max(
            0.0,
            float(self.get_parameter('lateral_velocity_gain').value),
        )
        self.max_vy = max(0.0, float(self.get_parameter('max_vy').value))
        self.lateral_velocity_deadband = max(
            0.0,
            float(self.get_parameter('lateral_velocity_deadband').value),
        )
        self.lateral_velocity_heading_limit_deg = max(
            0.0,
            float(
                self.get_parameter(
                    'lateral_velocity_heading_limit_deg'
                ).value
            ),
        )

        self.lookahead_distance = max(0.05, float(self.get_parameter('lookahead_distance').value))
        self.dynamic_lookahead = bool(self.get_parameter('dynamic_lookahead').value)
        self.lookahead_straight_distance = max(
            self.lookahead_distance,
            float(self.get_parameter('lookahead_straight_distance').value)
        )
        self.lookahead_corner_distance = max(
            0.05,
            min(self.lookahead_distance, float(self.get_parameter('lookahead_corner_distance').value))
        )
        self.lookahead_curve_distance = max(
            self.lookahead_corner_distance,
            min(
                self.lookahead_straight_distance,
                float(self.get_parameter('lookahead_curve_distance').value),
            ),
        )
        self.lookahead_corner_angle = math.radians(
            max(0.0, float(self.get_parameter('lookahead_corner_angle_deg').value))
        )
        self.lookahead_corner_scan_distance = max(
            self.lookahead_straight_distance,
            float(self.get_parameter('lookahead_corner_scan_distance').value)
        )
        self.lookahead_curve_angle = math.radians(
            max(0.0, float(self.get_parameter('lookahead_curve_angle_deg').value))
        )
        self.lookahead_curve_scan_distance = max(
            self.lookahead_straight_distance,
            float(self.get_parameter('lookahead_curve_scan_distance').value),
        )
        self.curve_heading_preview_distance = max(
            0.05,
            float(self.get_parameter('curve_heading_preview_distance').value),
        )
        self.curve_heading_blend = max(
            0.0,
            min(1.0, float(self.get_parameter('curve_heading_blend').value)),
        )
        self.corner_turn_distance = max(0.05, float(self.get_parameter('corner_turn_distance').value))
        self.corner_exit_distance = max(0.0, float(self.get_parameter('corner_exit_distance').value))
        self.corner_wiggle_distance = max(
            0.0,
            float(self.get_parameter('corner_wiggle_distance').value),
        )
        self.corner_wiggle_min_opposite_angle = math.radians(max(
            0.0,
            float(self.get_parameter('corner_wiggle_min_opposite_angle_deg').value),
        ))
        self.corner_heading_blend = max(0.0, min(1.0, float(self.get_parameter('corner_heading_blend').value)))
        self.corner_heading_conflict_blend = max(
            self.corner_heading_blend,
            min(1.0, float(self.get_parameter('corner_heading_conflict_blend').value)),
        )
        self.corner_lateral_yaw_gain_scale = max(
            0.0,
            min(1.0, float(self.get_parameter('corner_lateral_yaw_gain_scale').value))
        )
        self.reach_distance = float(self.get_parameter('reach_distance').value)
        self.goal_distance = float(self.get_parameter('goal_distance').value)

        self.loop_mode = self.get_parameter('loop_mode').value
        self.search_window = int(self.get_parameter('search_window').value)

        self.turn_in_place_angle = float(self.get_parameter('turn_in_place_angle').value)
        self.slow_down_angle = float(self.get_parameter('slow_down_angle').value)
        self.turn_crawl_vx = max(0.0, float(self.get_parameter('turn_crawl_vx').value))

        self.stuck_time = float(self.get_parameter('stuck_time').value)
        self.soft_unstick_yaw_rate = max(0.0, float(self.get_parameter('soft_unstick_yaw_rate').value))
        self.soft_unstick_min_moved = max(0.0, float(self.get_parameter('soft_unstick_min_moved').value))
        self.progress_s_epsilon = max(0.0, float(self.get_parameter('progress_s_epsilon').value))
        self.progress_backtrack_tolerance = max(
            0.0,
            float(self.get_parameter('progress_backtrack_tolerance').value)
        )
        self.progress_forward_tolerance = max(
            self.progress_s_epsilon,
            float(self.get_parameter('progress_forward_tolerance').value)
        )
        self.projection_speed_scale = max(0.1, float(self.get_parameter('projection_speed_scale').value))
        self.projection_step_margin = max(0.0, float(self.get_parameter('projection_step_margin').value))
        self.projection_min_step = max(0.0, float(self.get_parameter('projection_min_step').value))
        self.projection_max_step = max(
            self.projection_min_step,
            float(self.get_parameter('projection_max_step').value)
        )
        self.relocalize_distance = float(self.get_parameter('relocalize_distance').value)
        self.start_policy = str(self.get_parameter('start_policy').value)
        self.start_max_distance = float(self.get_parameter('start_max_distance').value)
        self.start_max_yaw_error = float(self.get_parameter('start_max_yaw_error').value)
        self.allow_global_relocalize = bool(self.get_parameter('allow_global_relocalize').value)
        self.stuck_relocalize = bool(self.get_parameter('stuck_relocalize').value)
        self.projection_fail_limit = max(1, int(self.get_parameter('projection_fail_limit').value))
        self.stop_on_route_deviation = bool(self.get_parameter('stop_on_route_deviation').value)
        self.recovery_distance = float(self.get_parameter('recovery_distance').value)
        self.recovery_heading_distance = max(0.0, float(self.get_parameter('recovery_heading_distance').value))
        self.recovery_yaw_error = max(0.0, float(self.get_parameter('recovery_yaw_error').value))
        self.recovery_vx = float(self.get_parameter('recovery_vx').value)
        self.max_route_deviation = float(self.get_parameter('max_route_deviation').value)
        self.max_valid_odom_abs = float(self.get_parameter('max_valid_odom_abs').value)
        self.max_odom_jump_distance = max(0.0, float(self.get_parameter('max_odom_jump_distance').value))
        self.max_odom_jump_z = max(0.0, float(self.get_parameter('max_odom_jump_z').value))
        self.max_odom_jump_dt = max(0.1, float(self.get_parameter('max_odom_jump_dt').value))
        self.yaw_deadband = float(self.get_parameter('yaw_deadband').value)
        self.max_vx_accel = float(self.get_parameter('max_vx_accel').value)
        self.max_vy_accel = float(self.get_parameter('max_vy_accel').value)
        self.max_yaw_accel = float(self.get_parameter('max_yaw_accel').value)
        self.yaw_wrap_hysteresis = float(self.get_parameter('yaw_wrap_hysteresis').value)
        if self.start_policy not in ('route_start', 'nearest'):
            self.start_policy = 'route_start'

        self.route = self.load_route(self.route_file)
        self.route_transform, self.route_transform_active = self.load_route_transform(self.route_transform_json)
        self.route_transform_yaw = self.yaw_from_transform(self.route_transform)
        self.arc_s, self.segment_lengths, self.total_length = self.build_route_geometry(self.route)

        self.current_x = None
        self.current_y = None
        self.current_yaw = None
        self.current_s = 0.0
        self.last_raw_odom_x = None
        self.last_raw_odom_y = None
        self.last_raw_odom_z = None
        self.last_raw_odom_time = None
        self.projected_x = None
        self.projected_y = None
        self.projected_segment_index = 0
        self.projected_segment_t = 0.0
        self.raw_projected_s = 0.0
        self.raw_projected_segment_index = 0
        self.raw_projected_segment_t = 0.0
        self.projection_limited = False
        self.projection_limit_min_s = 0.0
        self.projection_limit_max_s = 0.0
        self.projection_step_limit = 0.0
        self.projection_dt = 0.0
        self.projection_fail_count = 0
        self.projection_expanded_search = False
        self.route_yaw = 0.0
        self.route_heading_error = 0.0
        self.lateral_error = 0.0
        self.lateral_yaw_rate = 0.0
        self.lateral_velocity = 0.0
        self.effective_lookahead_distance = self.lookahead_distance
        self.lookahead_corner_angle_ahead = 0.0
        self.lookahead_corner_distance_ahead = -1.0
        self.lookahead_corner_s_ahead = -1.0
        self.lookahead_sharp_corner_ahead = False
        self.lookahead_curve_angle_near = 0.0
        self.lookahead_curve_distance_ahead = -1.0
        self.lookahead_curve_active = False

        self.direction = 1
        self.nearest_index = 0
        self.target_index = 0
        self.target_x = None
        self.target_y = None
        self.finished = False
        self.initialized = False
        self.start_alignment_failed = False
        self.tracking_fault = False
        self.follow_state = 'FOLLOW'
        self.follow_state_reason = 'init'
        self.last_follow_state_log_time = 0.0

        self.last_progress_index = 0
        self.last_progress_s = 0.0
        self.last_progress_x = None
        self.last_progress_y = None
        self.last_progress_time = time.time()
        self.last_projection_s = None
        self.last_projection_time = None
        self.last_log_time = 0.0
        self.last_start_fault_log_time = 0.0
        self.last_route_fault_log_time = 0.0
        self.last_stuck_log_time = 0.0
        self.last_odom_fault_log_time = 0.0
        self.turn_alpha_sign = 0.0
        self.last_local_route_distance = 0.0
        self.last_cmd_vx = 0.0
        self.last_cmd_vy = 0.0
        self.last_cmd_yaw_rate = 0.0
        self.last_cmd_time = time.time()
        self.soft_unstick_sign = 0.0

        self.sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10
        )

        self.pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info('simple csv waypoint_follower started')
        self.get_logger().info(f'route_file: {self.route_file}')
        self.get_logger().info(f"route_transform_json: {self.route_transform_json or 'identity'}")
        self.get_logger().info(f"route_frame_mode: {self.route_transform_active}, yaw_old_from_current={self.route_transform_yaw:.4f}")
        self.get_logger().info(f'route points: {len(self.route)}')
        self.get_logger().info(f'v_base: {self.v_base}, max_vx: {self.max_vx}, use_route_speed: {self.use_route_speed}')
        self.get_logger().info(
            f'yaw_limits: max={self.max_yaw_rate}, '
            f'non_corner={self.max_non_corner_yaw_rate}'
        )
        self.get_logger().info(
            f'lateral_yaw_gain: {self.lateral_yaw_gain}, '
            f'max_lateral_yaw_rate: {self.max_lateral_yaw_rate}, '
            f'lateral_deadband: {self.lateral_deadband}'
        )
        self.get_logger().info(
            'lateral velocity correction: '
            f'gain={self.lateral_velocity_gain:.2f}, '
            f'max_vy={self.max_vy:.2f}m/s, '
            f'deadband={self.lateral_velocity_deadband:.2f}m, '
            f'heading_limit={self.lateral_velocity_heading_limit_deg:.1f}deg'
        )
        self.get_logger().info(f'loop_mode: {self.loop_mode}')
        self.get_logger().info(f'lookahead_distance: {self.lookahead_distance}')
        self.get_logger().info(
            f'dynamic_lookahead: enabled={self.dynamic_lookahead}, '
            f'straight={self.lookahead_straight_distance}, corner={self.lookahead_corner_distance}, '
            f'curve={self.lookahead_curve_distance}, '
            f'corner_angle_deg={math.degrees(self.lookahead_corner_angle):.1f}, '
            f'scan_distance={self.lookahead_corner_scan_distance}, '
            f'curve_angle_deg={math.degrees(self.lookahead_curve_angle):.1f}, '
            f'curve_scan_distance={self.lookahead_curve_scan_distance}, '
            f'curve_heading_preview={self.curve_heading_preview_distance}, '
            f'curve_heading_blend={self.curve_heading_blend}, '
            f'corner_turn_distance={self.corner_turn_distance}, '
            f'corner_exit_distance={self.corner_exit_distance}, '
            f'corner_wiggle_distance={self.corner_wiggle_distance}, '
            f'corner_wiggle_min_opposite_angle_deg={math.degrees(self.corner_wiggle_min_opposite_angle):.1f}, '
            f'corner_heading_blend={self.corner_heading_blend}, '
            f'corner_heading_conflict_blend={self.corner_heading_conflict_blend}, '
            f'corner_lateral_scale={self.corner_lateral_yaw_gain_scale}'
        )
        self.get_logger().info(f'reach_distance: {self.reach_distance}')
        self.get_logger().info(f'turn_crawl_vx: {self.turn_crawl_vx}')
        self.get_logger().info(
            f'odom_jump_limits: distance={self.max_odom_jump_distance}, '
            f'z={self.max_odom_jump_z}, dt={self.max_odom_jump_dt}'
        )
        self.get_logger().info(
            f'start_policy: {self.start_policy}, start_max_distance: {self.start_max_distance}, '
            f'start_max_yaw_error: {self.start_max_yaw_error}'
        )
        self.get_logger().info(
            f'allow_global_relocalize: {self.allow_global_relocalize}, '
            f'stuck_relocalize: {self.stuck_relocalize}, '
            f'recovery_distance: {self.recovery_distance}, '
            f'recovery_heading_distance: {self.recovery_heading_distance}, '
            f'recovery_yaw_error: {self.recovery_yaw_error}, recovery_vx: {self.recovery_vx}, '
            f'max_route_deviation: {self.max_route_deviation}'
        )
        self.get_logger().info(
            f'soft_unstick_yaw_rate: {self.soft_unstick_yaw_rate}, '
            f'soft_unstick_min_moved: {self.soft_unstick_min_moved}, '
            f'progress_s_epsilon: {self.progress_s_epsilon}'
        )
        self.get_logger().info(
            f'projection: local route window enabled; speed-step clamp disabled; '
            f'configured_speed_scale={self.projection_speed_scale}, '
            f'configured_max_step={self.projection_max_step}, '
            f'progress_forward_tolerance={self.progress_forward_tolerance}, '
            f'progress_backtrack_tolerance={self.progress_backtrack_tolerance}'
        )

    def reset_progress_watchdog(self, now=None):
        self.last_progress_index = self.nearest_index
        self.last_progress_s = self.current_s
        self.last_progress_x = self.current_x
        self.last_progress_y = self.current_y
        self.last_progress_time = time.time() if now is None else now
        self.soft_unstick_sign = 0.0

    def reset_projection_reference(self, now=None):
        self.last_projection_s = self.current_s
        self.last_projection_time = time.time() if now is None else now

    def set_follow_state(self, state, reason):
        if state != self.follow_state:
            self.get_logger().info(f'follow_state {self.follow_state} -> {state}: {reason}')
            self.follow_state = state
            self.last_follow_state_log_time = time.time()
        self.follow_state_reason = reason

    def select_follow_state(
        self,
        corner_turn_active,
        corner_exit_active,
        approach_corner_active,
        wiggle_active,
        curve_active,
        control_abs_alpha,
    ):
        if self.tracking_fault:
            return 'FAULT', 'tracking_fault'
        if self.projection_fail_count > 0 or self.projection_expanded_search:
            return 'RECOVER', 'projection_recovery'
        if corner_turn_active or corner_exit_active:
            return 'CORNER', 'corner_window'
        if wiggle_active:
            return 'WIGGLE', 'opposing_corner_pair'
        if approach_corner_active:
            return 'APPROACH_CORNER', 'corner_ahead'
        if control_abs_alpha > self.turn_in_place_angle:
            return 'RECOVER', 'heading_realign'
        if curve_active:
            return 'CURVE', 'cumulative_route_turn'
        return 'FOLLOW', 'csv_track'

    def has_route_progress(self):
        directed_s_delta = self.direction * (self.current_s - self.last_progress_s)
        directed_index_delta = self.direction * (self.nearest_index - self.last_progress_index)
        return directed_s_delta > self.progress_s_epsilon or directed_index_delta > 0

    def load_route_transform(self, path):
        identity = [
            1.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 1.0,
        ]
        if not path:
            return identity, False
        with open(path, 'r') as f:
            data = json.load(f)
        values = data.get('transformOldFromCurrent')
        if not isinstance(values, list) or len(values) != 16:
            raise RuntimeError('route_transform_json must contain transformOldFromCurrent[16]')
        return [float(v) for v in values], True

    def yaw_from_transform(self, m):
        return math.atan2(m[4], m[0])

    def transform_pose_to_route_frame(self, x, y, z, yaw):
        m = self.route_transform
        route_x = m[0] * x + m[1] * y + m[2] * z + m[3]
        route_y = m[4] * x + m[5] * y + m[6] * z + m[7]
        route_yaw = normalize_angle(yaw + self.route_transform_yaw)
        return route_x, route_y, route_yaw

    def load_route(self, path):
        route = []
        with open(path, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                route.append({
                    'x': float(row['x']),
                    'y': float(row['y']),
                    'yaw': float(row['yaw']),
                    'v': float(row['v'])
                })

        if len(route) < 2:
            raise RuntimeError('route file must contain at least 2 points')

        return route

    def build_route_geometry(self, route):
        arc_s = [0.0]
        segment_lengths = []
        total = 0.0
        for i in range(len(route) - 1):
            p0 = route[i]
            p1 = route[i + 1]
            seg = math.hypot(p1['x'] - p0['x'], p1['y'] - p0['y'])
            segment_lengths.append(seg)
            total += seg
            arc_s.append(total)
        return arc_s, segment_lengths, total

    def point_at_s(self, s):
        if s <= 0.0:
            p = self.route[0]
            return p['x'], p['y'], 0
        if s >= self.total_length:
            p = self.route[-1]
            return p['x'], p['y'], len(self.route) - 1

        for i, seg in enumerate(self.segment_lengths):
            s0 = self.arc_s[i]
            s1 = self.arc_s[i + 1]
            if s <= s1:
                if seg <= 1e-6:
                    p = self.route[i]
                    return p['x'], p['y'], i
                t = (s - s0) / seg
                p0 = self.route[i]
                p1 = self.route[i + 1]
                x = p0['x'] + (p1['x'] - p0['x']) * t
                y = p0['y'] + (p1['y'] - p0['y']) * t
                return x, y, i + (1 if t >= 0.5 else 0)

        p = self.route[-1]
        return p['x'], p['y'], len(self.route) - 1

    def segment_at_s(self, s):
        if not self.segment_lengths:
            return 0, 0.0
        if s <= 0.0:
            return 0, 0.0
        if s >= self.total_length:
            return len(self.segment_lengths) - 1, 1.0

        for i, seg in enumerate(self.segment_lengths):
            s0 = self.arc_s[i]
            s1 = self.arc_s[i + 1]
            if s <= s1:
                if seg <= 1e-6:
                    return i, 0.0
                return i, max(0.0, min(1.0, (s - s0) / seg))
        return len(self.segment_lengths) - 1, 1.0

    def route_segment_yaw(self, seg_i):
        if not self.segment_lengths:
            return 0.0
        seg_i = max(0, min(len(self.segment_lengths) - 1, int(seg_i)))
        p0 = self.route[seg_i]
        p1 = self.route[min(seg_i + 1, len(self.route) - 1)]
        return math.atan2(p1['y'] - p0['y'], p1['x'] - p0['x'])

    def route_yaw_at_s(self, s, direction=None):
        if direction is None:
            direction = self.direction
        if self.total_length <= 1e-6:
            i = 0
        elif s <= 0.0:
            i = 0
        elif s >= self.total_length:
            i = max(0, len(self.route) - 2)
        else:
            i = 0
            for seg_i in range(len(self.segment_lengths)):
                if s <= self.arc_s[seg_i + 1]:
                    i = seg_i
                    break

        yaw = self.route_segment_yaw(i)
        if direction < 0:
            yaw = normalize_angle(yaw + math.pi)
        return yaw

    def turn_angle_at_boundary(self, boundary_i):
        if boundary_i <= 0 or boundary_i >= len(self.segment_lengths):
            return 0.0
        return normalize_angle(self.route_segment_yaw(boundary_i) - self.route_segment_yaw(boundary_i - 1))

    def corner_is_short_opposing_wiggle(self, boundary_i, angle):
        """Detect a close, opposite sharp turn that should not be two corners.

        The route remains untouched. This only prevents the state machine from
        slowing and pivoting separately for an S-shaped pair such as +80/-80
        degrees recorded within a short travelled distance. Pure pursuit still
        follows the original x/y polyline with the normal yaw-rate limit.
        """
        if (
            self.corner_wiggle_distance <= 0.0
            or abs(angle) < self.lookahead_corner_angle
            or len(self.segment_lengths) < 2
        ):
            return False

        boundary_s = self.arc_s[boundary_i]
        first = max(1, boundary_i - 1)
        last = min(len(self.segment_lengths) - 1, boundary_i + 1)
        while first > 1 and boundary_s - self.arc_s[first - 1] <= self.corner_wiggle_distance:
            first -= 1
        while (
            last < len(self.segment_lengths) - 1
            and self.arc_s[last + 1] - boundary_s <= self.corner_wiggle_distance
        ):
            last += 1

        for other_i in range(first, last + 1):
            if other_i == boundary_i:
                continue
            other_angle = self.turn_angle_at_boundary(other_i)
            if (
                abs(other_angle) >= self.corner_wiggle_min_opposite_angle
                and angle * other_angle < 0.0
            ):
                return True
        return False

    def corner_ahead(self, s, direction, scan_distance):
        if len(self.segment_lengths) < 2 or scan_distance <= 0.0:
            return 0.0, -1.0, -1.0, False, False

        max_angle = 0.0
        best_angle = 0.0
        best_s = -1.0
        best_distance = -1.0
        sharp_found = False
        wiggle_angle = 0.0
        wiggle_s = -1.0
        wiggle_distance = -1.0
        wiggle_found = False
        _, current_seg_t = self.segment_at_s(s)

        if direction >= 0:
            start_boundary = self.segment_at_s(s)[0] + 1
            for boundary_i in range(max(1, start_boundary), len(self.segment_lengths)):
                boundary_s = self.arc_s[boundary_i]
                distance = boundary_s - s
                if distance < -1e-6:
                    continue
                if distance > scan_distance:
                    break
                signed_angle = self.turn_angle_at_boundary(boundary_i)
                angle = abs(signed_angle)
                if self.corner_is_short_opposing_wiggle(boundary_i, signed_angle):
                    max_angle = max(max_angle, angle)
                    if not wiggle_found:
                        wiggle_angle = angle
                        wiggle_s = boundary_s
                        wiggle_distance = max(0.0, distance)
                        wiggle_found = True
                    continue
                max_angle = max(max_angle, angle)
                if angle >= self.lookahead_corner_angle:
                    best_angle = angle
                    best_s = boundary_s
                    best_distance = max(0.0, distance)
                    sharp_found = True
                    break
        else:
            start_boundary = self.segment_at_s(s)[0]
            for boundary_i in range(min(len(self.segment_lengths) - 1, start_boundary), 0, -1):
                boundary_s = self.arc_s[boundary_i]
                distance = s - boundary_s
                if distance < -1e-6:
                    continue
                if distance > scan_distance:
                    break
                signed_angle = self.turn_angle_at_boundary(boundary_i)
                angle = abs(signed_angle)
                if self.corner_is_short_opposing_wiggle(boundary_i, signed_angle):
                    max_angle = max(max_angle, angle)
                    if not wiggle_found:
                        wiggle_angle = angle
                        wiggle_s = boundary_s
                        wiggle_distance = max(0.0, distance)
                        wiggle_found = True
                    continue
                max_angle = max(max_angle, angle)
                if angle >= self.lookahead_corner_angle:
                    best_angle = angle
                    best_s = boundary_s
                    best_distance = max(0.0, distance)
                    sharp_found = True
                    break

        if sharp_found:
            return best_angle, best_s, best_distance, True, False
        if wiggle_found:
            return wiggle_angle, wiggle_s, wiggle_distance, False, True
        return max_angle, -1.0, -1.0, False, False

    def curve_ahead(self, s, direction, scan_distance):
        """Detect a real arc made of several same-direction modest turns."""
        if len(self.segment_lengths) < 2 or scan_distance <= 0.0:
            return 0.0, -1.0, -1.0, False

        samples = []
        if direction >= 0:
            start_boundary = self.segment_at_s(s)[0] + 1
            boundaries = range(max(1, start_boundary), len(self.segment_lengths))
        else:
            start_boundary = self.segment_at_s(s)[0]
            boundaries = range(min(len(self.segment_lengths) - 1, start_boundary), 0, -1)

        for boundary_i in boundaries:
            boundary_s = self.arc_s[boundary_i]
            distance = boundary_s - s if direction >= 0 else s - boundary_s
            if distance < -1e-6:
                continue
            if distance > scan_distance:
                break
            signed_angle = self.turn_angle_at_boundary(boundary_i)
            if direction < 0:
                signed_angle = -signed_angle
            # A single sharp vertex belongs to the existing CORNER state, not
            # to cumulative arc handling, even when small lead-in noise exists.
            if abs(signed_angle) >= self.lookahead_corner_angle:
                break
            samples.append((boundary_s, max(0.0, distance), signed_angle))

        return cumulative_turn_candidate(samples, self.lookahead_curve_angle)

    def corner_aware_lookahead(self):
        if not self.dynamic_lookahead:
            return self.lookahead_distance, 0.0, -1.0, -1.0, False, False

        corner_angle, corner_s, corner_distance, sharp_corner, wiggle_ahead = self.corner_ahead(
            self.current_s,
            self.direction,
            self.lookahead_corner_scan_distance
        )
        if sharp_corner and corner_distance >= 0.0:
            if corner_distance > self.corner_turn_distance:
                # Before the corner, keep the target close to the vertex so
                # pure pursuit does not cut across the inside of the turn. Add
                # a tiny amount past the vertex so the target does not sit on
                # the exact corner for several seconds.
                lookahead = min(
                    self.lookahead_straight_distance,
                    max(self.lookahead_corner_distance, corner_distance + 0.05)
                )
            else:
                # At the corner, keep following the CSV polyline with a short
                # target ahead on the next segment instead of locking to the vertex.
                lookahead = min(
                    self.lookahead_straight_distance,
                    max(self.lookahead_corner_distance, corner_distance + self.lookahead_corner_distance)
                )
        else:
            lookahead = self.lookahead_straight_distance

        lookahead = max(self.lookahead_corner_distance, min(self.lookahead_straight_distance, lookahead))
        return lookahead, corner_angle, corner_s, corner_distance, sharp_corner, wiggle_ahead

    def corner_behind(self, s, direction, scan_distance):
        return self.corner_ahead(s, -direction, scan_distance)

    def update_lateral_error(self):
        if self.projected_x is None or self.projected_y is None:
            self.lateral_error = 0.0
            self.route_yaw = self.current_yaw if self.current_yaw is not None else 0.0
            self.route_heading_error = 0.0
            return
        self.route_yaw = self.route_yaw_at_s(self.current_s, self.direction)
        self.route_heading_error = normalize_angle(self.route_yaw - self.current_yaw)
        ux = math.cos(self.route_yaw)
        uy = math.sin(self.route_yaw)
        dx = self.current_x - self.projected_x
        dy = self.current_y - self.projected_y
        self.lateral_error = ux * dy - uy * dx

    def compute_lateral_yaw_rate(self, gain_scale=1.0):
        gain = self.lateral_yaw_gain * max(0.0, min(1.0, gain_scale))
        if gain <= 0.0:
            return 0.0
        error = self.lateral_error
        if abs(error) <= self.lateral_deadband:
            return 0.0
        correction = -gain * error
        if self.max_lateral_yaw_rate > 0.0:
            correction = max(-self.max_lateral_yaw_rate, min(self.max_lateral_yaw_rate, correction))
        return correction

    def project_to_route_segments(self, start, end):
        nseg = len(self.route) - 1
        if nseg <= 0:
            p = self.route[0]
            return 0, 0.0, p['x'], p['y'], 0.0, 0.0, 0

        start = max(0, min(nseg - 1, int(start)))
        end = max(start, min(nseg - 1, int(end)))
        best = None
        for i in range(start, end + 1):
            p0 = self.route[i]
            p1 = self.route[i + 1]
            vx = p1['x'] - p0['x']
            vy = p1['y'] - p0['y']
            seg2 = vx * vx + vy * vy
            if seg2 <= 1e-12:
                t = 0.0
                px = p0['x']
                py = p0['y']
            else:
                t = ((self.current_x - p0['x']) * vx + (self.current_y - p0['y']) * vy) / seg2
                t = max(0.0, min(1.0, t))
                px = p0['x'] + vx * t
                py = p0['y'] + vy * t
            d = math.hypot(self.current_x - px, self.current_y - py)
            route_s = self.arc_s[i] + self.segment_lengths[i] * t
            if best is None or d < best[5]:
                nearest_i = i + (1 if t >= 0.5 else 0)
                best = (nearest_i, route_s, px, py, t, d, i)
        return best

    def project_to_route_window(self):
        nseg = len(self.route) - 1
        if nseg <= 0:
            p = self.route[0]
            return 0, 0.0, p['x'], p['y'], 0.0, 0.0, 0

        if self.initialized:
            center_seg = self.segment_at_s(self.current_s)[0]
        else:
            center_seg = max(0, min(nseg - 1, self.nearest_index))

        if self.follow_state == 'RECOVER':
            local_radius = self.search_window
        elif self.follow_state in ('APPROACH_CORNER', 'CORNER'):
            local_radius = max(1, min(self.search_window, 2))
        else:
            local_radius = max(1, min(self.search_window, 2))
        local = self.project_to_route_segments(center_seg - local_radius, center_seg + local_radius)
        self.projection_expanded_search = False

        if local is None:
            self.projection_expanded_search = True
            return self.project_to_route_segments(center_seg - self.search_window, center_seg + self.search_window)

        if self.initialized and local[5] > self.relocalize_distance:
            self.projection_expanded_search = True
            expanded = self.project_to_route_segments(center_seg - self.search_window, center_seg + self.search_window)
            if expanded is not None and expanded[5] < local[5]:
                return expanded
        return local

    def odom_callback(self, msg):
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        z = float(msg.pose.pose.position.z)
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        if not self.odom_is_valid(x, y, z, yaw):
            self.tracking_fault = True
            self.publish_stop()
            now = time.time()
            if now - self.last_odom_fault_log_time > 1.0:
                self.last_odom_fault_log_time = now
                self.get_logger().error(
                    f'invalid odometry: x={x:.3f}, y={y:.3f}, z={z:.3f}, yaw={yaw:.3f}; hold stop'
                )
            return

        route_x, route_y, route_yaw = self.transform_pose_to_route_frame(x, y, z, yaw)
        if not self.odom_is_valid(route_x, route_y, 0.0, route_yaw):
            self.tracking_fault = True
            self.publish_stop()
            now = time.time()
            if now - self.last_odom_fault_log_time > 1.0:
                self.last_odom_fault_log_time = now
                self.get_logger().error(
                    f'invalid transformed odometry: x={route_x:.3f}, y={route_y:.3f}, yaw={route_yaw:.3f}; hold stop'
                )
            return

        now = time.time()
        stamp_time = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        odom_time = stamp_time if stamp_time > 0.0 else now
        if self.last_raw_odom_time is not None:
            dt = max(1e-3, odom_time - self.last_raw_odom_time)
            dx = x - self.last_raw_odom_x
            dy = y - self.last_raw_odom_y
            dz = z - self.last_raw_odom_z
            jump = math.sqrt(dx * dx + dy * dy + dz * dz)
            dynamic_jump_limit = max(
                self.max_odom_jump_distance,
                self.max_vx * max(dt, 0.0) * 2.5 + 0.30,
            )
            dynamic_z_limit = max(
                self.max_odom_jump_z,
                0.25 * max(dt, 0.0) + 0.20,
            )
            if dt <= self.max_odom_jump_dt and (
                jump > dynamic_jump_limit or abs(dz) > dynamic_z_limit
            ):
                self.tracking_fault = True
                self.publish_stop()
                if now - self.last_odom_fault_log_time > 1.0:
                    self.last_odom_fault_log_time = now
                    self.get_logger().error(
                        f'odometry jump: step={jump:.3f} dz={dz:.3f} dt={dt:.3f}; '
                        f'limit={dynamic_jump_limit:.3f} z_limit={dynamic_z_limit:.3f}; '
                        f'raw=({x:.3f},{y:.3f},{z:.3f}) '
                        f'last=({self.last_raw_odom_x:.3f},{self.last_raw_odom_y:.3f},{self.last_raw_odom_z:.3f}); hold stop'
                    )
                return
        self.last_raw_odom_x = x
        self.last_raw_odom_y = y
        self.last_raw_odom_z = z
        self.last_raw_odom_time = odom_time

        self.current_x = route_x
        self.current_y = route_y
        self.current_yaw = route_yaw

        if not self.initialized:
            if self.start_policy == 'route_start':
                if not self.check_start_alignment():
                    return
                self.nearest_index = 0
            else:
                self.nearest_index = self.find_nearest_global()
            projected = self.project_to_route_window()
            self.nearest_index = projected[0]
            self.current_s = projected[1]
            self.projected_x = projected[2]
            self.projected_y = projected[3]
            self.projected_segment_t = projected[4]
            self.last_local_route_distance = projected[5]
            self.projected_segment_index = projected[6]
            self.raw_projected_s = self.current_s
            self.raw_projected_segment_t = self.projected_segment_t
            self.raw_projected_segment_index = self.projected_segment_index
            self.target_index = self.nearest_index
            self.target_x = self.projected_x
            self.target_y = self.projected_y
            self.reset_progress_watchdog()
            self.reset_projection_reference()
            self.initialized = True
            self.start_alignment_failed = False

            d0 = self.distance_to_index(0)
            self.get_logger().info(f'init nearest_index={self.nearest_index}, distance_to_start={d0:.3f}')

    def odom_is_valid(self, x, y, z, yaw):
        values = (x, y, z, yaw)
        if not all(math.isfinite(v) for v in values):
            return False
        return all(abs(v) <= self.max_valid_odom_abs for v in (x, y, z))

    def check_start_alignment(self):
        start = self.route[0]
        distance = self.distance_to_index(0)
        yaw_error = normalize_angle(self.current_yaw - start['yaw'])
        if distance <= self.start_max_distance and abs(yaw_error) <= self.start_max_yaw_error:
            return True

        self.start_alignment_failed = True
        now = time.time()
        if now - self.last_start_fault_log_time > 2.0:
            self.last_start_fault_log_time = now
            self.get_logger().error(
                'route start mismatch: '
                f'current=({self.current_x:.3f},{self.current_y:.3f},{self.current_yaw:.3f}), '
                f'route_start=({start["x"]:.3f},{start["y"]:.3f},{start["yaw"]:.3f}), '
                f'distance={distance:.3f}/{self.start_max_distance:.3f}, '
                f'yaw_error={yaw_error:.3f}/{self.start_max_yaw_error:.3f}; hold stop'
            )
        return False

    def distance_to_index(self, i):
        p = self.route[i]
        return math.hypot(p['x'] - self.current_x, p['y'] - self.current_y)

    def find_nearest_global(self):
        best_i = 0
        best_d = 1e9
        for i, p in enumerate(self.route):
            d = math.hypot(p['x'] - self.current_x, p['y'] - self.current_y)
            if d < best_d:
                best_d = d
                best_i = i
        return best_i

    def find_nearest_window(self):
        n = len(self.route)

        start = max(0, self.nearest_index - self.search_window)
        end = min(n - 1, self.nearest_index + self.search_window)

        best_i = self.nearest_index
        best_d = 1e9

        for i in range(start, end + 1):
            p = self.route[i]
            d = math.hypot(p['x'] - self.current_x, p['y'] - self.current_y)
            if d < best_d:
                best_d = d
                best_i = i

        return best_i, best_d

    def projection_step_limits(self, now):
        if self.last_projection_time is None:
            dt = 0.05
        else:
            dt = max(0.02, min(0.20, now - self.last_projection_time))

        speed = max(0.0, abs(self.last_cmd_vx))
        dynamic_step = speed * self.projection_speed_scale * dt + self.projection_step_margin
        step = max(self.projection_min_step, dynamic_step)
        step = min(self.projection_max_step, step)
        forward_step = min(self.progress_forward_tolerance, step)
        backtrack_step = min(self.progress_backtrack_tolerance, step)
        return forward_step, backtrack_step, dt

    def update_nearest_index(self):
        now = time.time()
        projected = self.project_to_route_window()
        if projected is None:
            self.projection_fail_count += 1
            if self.projection_fail_count >= self.projection_fail_limit:
                self.tracking_fault = True
                self.get_logger().error('route projection failed repeatedly; hold stop')
                return False
            self.projection_limited = True
            return True

        new_i, raw_s, px, py, new_t, new_d, new_seg_i = projected

        self.raw_projected_s = raw_s
        self.raw_projected_segment_index = new_seg_i
        self.raw_projected_segment_t = new_t
        self.projection_limited = False
        self.projection_limit_min_s = 0.0
        self.projection_limit_max_s = self.total_length
        self.projection_step_limit = 0.0
        self.projection_dt = 0.0

        if self.stop_on_route_deviation and new_d > self.max_route_deviation:
            self.projection_fail_count += 1
            if self.projection_fail_count < self.projection_fail_limit:
                if now - self.last_route_fault_log_time > 1.0:
                    self.last_route_fault_log_time = now
                    self.get_logger().warn(
                        f'route projection outlier {self.projection_fail_count}/{self.projection_fail_limit}: '
                        f'local_d={new_d:.2f}/{self.max_route_deviation:.2f}, '
                        f'raw_s={raw_s:.2f}, current_s={self.current_s:.2f}; keep last valid projection'
                    )
                self.projection_limited = True
                self.last_local_route_distance = new_d
                return True

            self.tracking_fault = True
            if now - self.last_route_fault_log_time > 1.0:
                self.last_route_fault_log_time = now
                self.get_logger().error(
                    f'route deviation too large repeatedly: local_d={new_d:.2f}/{self.max_route_deviation:.2f}, '
                    f'nearest_index={self.nearest_index}, candidate={new_i}; hold stop'
                )
            return False
        self.projection_fail_count = 0

        # The nearest projection is already constrained to a small local route
        # window and validated by max_route_deviation above. A speed-derived
        # progress clamp can lag the pose, put the lookahead target behind the
        # robot, and incorrectly command a turn-around on a straight segment.
        # Always use this valid local projection for steering.
        new_s = raw_s

        self.last_local_route_distance = new_d
        self.nearest_index = new_i
        self.current_s = max(0.0, min(self.total_length, new_s))
        self.projected_x = px
        self.projected_y = py
        self.projected_segment_index = new_seg_i
        self.projected_segment_t = new_t
        self.last_projection_s = self.current_s
        self.last_projection_time = now
        self.update_lateral_error()

        if self.has_route_progress():
            self.reset_progress_watchdog()
        return True

    def compute_lookahead_index(self):
        n = len(self.route)
        i = self.nearest_index
        acc = 0.0

        while True:
            ni = i + self.direction

            if ni < 0 or ni >= n:
                return i

            p0 = self.route[i]
            p1 = self.route[ni]
            seg = math.hypot(p1['x'] - p0['x'], p1['y'] - p0['y'])
            acc += seg
            i = ni

            if acc >= self.lookahead_distance:
                return i

    def publish_stop(self):
        cmd = Twist()
        self.pub.publish(cmd)
        self.last_cmd_vx = 0.0
        self.last_cmd_vy = 0.0
        self.last_cmd_yaw_rate = 0.0
        self.last_cmd_time = time.time()

    def smooth_value(self, desired, previous, max_rate, dt):
        if max_rate <= 0.0:
            return desired
        max_step = max_rate * dt
        delta = max(-max_step, min(max_step, desired - previous))
        return previous + delta

    def handle_goal(self):
        n = len(self.route)

        if self.direction > 0:
            goal_i = n - 1
            if self.total_length - self.current_s > self.goal_distance:
                return False
        else:
            goal_i = 0
            if self.current_s > self.goal_distance:
                return False

        dist_goal = self.distance_to_index(goal_i)

        if dist_goal > self.goal_distance:
            return False

        if self.loop_mode == 'pingpong':
            if self.direction > 0:
                self.direction = -1
                self.nearest_index = n - 1
                self.current_s = self.total_length
                self.turn_alpha_sign = 0.0
                self.get_logger().info('reach end, switch to backward')
                self.reset_projection_reference()
            else:
                self.direction = 1
                self.nearest_index = 0
                self.current_s = 0.0
                self.turn_alpha_sign = 0.0
                self.get_logger().info('reach start, switch to forward')
                self.reset_projection_reference()

            self.reset_progress_watchdog()
            return False

        self.get_logger().info('goal reached, stop')
        self.finished = True
        self.publish_stop()
        return True

    def control_loop(self):
        if self.current_x is None:
            self.publish_stop()
            return

        if self.start_alignment_failed and not self.initialized:
            self.publish_stop()
            return

        if self.finished:
            self.publish_stop()
            return

        if self.tracking_fault:
            self.publish_stop()
            return

        if not self.update_nearest_index():
            self.publish_stop()
            return

        if self.handle_goal():
            return

        recovery_mode = False
        distance_recovery = False
        heading_recovery = False
        (
            lookahead_distance,
            corner_angle,
            corner_s,
            corner_distance,
            sharp_corner,
            wiggle_ahead,
        ) = self.corner_aware_lookahead()
        behind_angle, behind_s, behind_distance, recent_corner, recent_wiggle = self.corner_behind(
            self.current_s,
            self.direction,
            self.corner_exit_distance
        )
        curve_angle, curve_s, curve_distance, curve_ahead = self.curve_ahead(
            self.current_s,
            self.direction,
            self.lookahead_curve_scan_distance,
        )
        recent_curve_angle, _, _, recent_curve = self.curve_ahead(
            self.current_s,
            -self.direction,
            self.lookahead_curve_scan_distance,
        )

        corner_turn_active = (
            sharp_corner
            and corner_distance >= 0.0
            and corner_distance <= self.corner_turn_distance
        )
        corner_exit_active = (
            recent_corner
            and behind_distance >= 0.0
            and behind_distance <= self.corner_exit_distance
        )
        corner_control_active = corner_turn_active or corner_exit_active
        approach_corner_active = (
            sharp_corner
            and corner_distance >= 0.0
            and corner_distance <= self.lookahead_straight_distance
        )
        wiggle_active = wiggle_ahead or recent_wiggle
        curve_detected = curve_ahead or recent_curve
        curve_hysteresis = (
            self.follow_state == 'CURVE'
            and max(curve_angle, recent_curve_angle) >= self.lookahead_curve_angle * 0.50
        )
        curve_active = (
            not sharp_corner
            and not recent_corner
            and not wiggle_active
            and (curve_detected or curve_hysteresis)
        )
        if curve_active:
            lookahead_distance = min(lookahead_distance, self.lookahead_curve_distance)

        target_s = self.current_s + self.direction * lookahead_distance
        target_s = max(0.0, min(self.total_length, target_s))
        self.target_x, self.target_y, self.target_index = self.point_at_s(target_s)

        self.effective_lookahead_distance = abs(target_s - self.current_s)
        self.lookahead_corner_angle_ahead = corner_angle
        self.lookahead_corner_s_ahead = corner_s
        self.lookahead_corner_distance_ahead = corner_distance
        self.lookahead_sharp_corner_ahead = sharp_corner
        self.lookahead_curve_angle_near = max(curve_angle, recent_curve_angle)
        self.lookahead_curve_distance_ahead = curve_distance
        self.lookahead_curve_active = curve_active

        target_segment_index, target_segment_t = self.segment_at_s(target_s)
        target = self.route[self.target_index]
        dx = self.target_x - self.current_x
        dy = self.target_y - self.current_y
        dist_to_target = math.hypot(dx, dy)

        if dist_to_target > 1e-6:
            target_angle = math.atan2(dy, dx)
            target_alpha = normalize_angle(target_angle - self.current_yaw)
        else:
            target_alpha = 0.0

        corner_heading_error = 0.0
        curve_heading_error = 0.0
        heading_control_error = target_alpha
        heading_control_source = 'lookahead_target'
        if corner_control_active:
            if corner_turn_active and corner_s >= 0.0:
                probe_s = corner_s + self.direction * 1e-3
            else:
                probe_s = self.current_s + self.direction * max(0.05, self.lookahead_corner_distance)
            probe_s = max(0.0, min(self.total_length, probe_s))
            corner_heading = self.route_yaw_at_s(probe_s, self.direction)
            corner_heading_error = normalize_angle(corner_heading - self.current_yaw)
            heading_control_error = corner_heading_command(
                target_alpha,
                corner_heading_error,
                normal_blend=self.corner_heading_blend,
                conflict_blend=self.corner_heading_conflict_blend,
            )
            heading_control_source = 'csv_target+corner_tangent'
        elif curve_active:
            probe_s = self.current_s + self.direction * self.curve_heading_preview_distance
            probe_s = max(0.0, min(self.total_length, probe_s))
            curve_heading = self.route_yaw_at_s(probe_s, self.direction)
            curve_heading_error = normalize_angle(curve_heading - self.current_yaw)
            heading_control_error = corner_heading_command(
                target_alpha,
                curve_heading_error,
                normal_blend=self.curve_heading_blend,
                conflict_blend=max(0.55, self.curve_heading_blend),
            )
            heading_control_source = 'csv_target+curve_tangent'

        alpha = heading_control_error
        target_speed = self.v_base
        if self.use_route_speed:
            target_speed = min(self.v_base, max(0.0, float(target.get('v', self.v_base))))

        alpha_for_control = alpha
        if abs(alpha) > self.turn_in_place_angle:
            if abs(alpha) > math.pi - self.yaw_wrap_hysteresis:
                if self.turn_alpha_sign == 0.0:
                    if abs(self.last_cmd_yaw_rate) > 0.05:
                        self.turn_alpha_sign = 1.0 if self.last_cmd_yaw_rate > 0.0 else -1.0
                    else:
                        self.turn_alpha_sign = 1.0 if alpha >= 0.0 else -1.0
                alpha_for_control = self.turn_alpha_sign * abs(alpha)
            else:
                self.turn_alpha_sign = 1.0 if alpha >= 0.0 else -1.0
        elif abs(alpha) < self.slow_down_angle:
            self.turn_alpha_sign = 0.0

        heading_yaw_rate = self.k_yaw * alpha_for_control
        if abs(alpha_for_control) < self.yaw_deadband:
            heading_yaw_rate = 0.0
        control_abs_alpha = abs(alpha_for_control)

        next_state, state_reason = self.select_follow_state(
            corner_turn_active,
            corner_exit_active,
            approach_corner_active,
            wiggle_active,
            curve_active,
            control_abs_alpha
        )
        self.set_follow_state(next_state, state_reason)
        control_mode = self.follow_state

        if control_mode == 'CORNER':
            lateral_scale = self.corner_lateral_yaw_gain_scale
        elif control_mode == 'CURVE':
            lateral_scale = self.corner_lateral_yaw_gain_scale
        elif control_mode == 'WIGGLE':
            lateral_scale = max(self.corner_lateral_yaw_gain_scale, 0.75)
        elif control_mode == 'RECOVER':
            lateral_scale = max(self.corner_lateral_yaw_gain_scale, 0.50)
        else:
            lateral_scale = 1.0
        self.lateral_yaw_rate = self.compute_lateral_yaw_rate(lateral_scale)
        yaw_rate = heading_yaw_rate + self.lateral_yaw_rate
        yaw_limit = self.max_yaw_rate
        if control_mode != 'CORNER' and self.max_non_corner_yaw_rate > 0.0:
            yaw_limit = min(yaw_limit, self.max_non_corner_yaw_rate)
        yaw_rate = max(-yaw_limit, min(yaw_limit, yaw_rate))

        turn_crawl_active = False
        if control_mode == 'FAULT':
            vx = 0.0
            yaw_rate = 0.0
        elif control_mode == 'CORNER':
            crawl_vx = self.turn_crawl_vx if self.turn_crawl_vx > 0.0 else target_speed * 0.35
            corner_crawl_needed = corner_turn_active and (
                control_abs_alpha > self.slow_down_angle or abs(corner_heading_error) > self.slow_down_angle
            )
            if corner_crawl_needed:
                vx = min(target_speed, self.max_vx, max(0.18, crawl_vx))
                turn_crawl_active = True
            else:
                vx = min(target_speed, self.max_vx, max(0.30, target_speed * 0.80))
        elif control_mode == 'APPROACH_CORNER':
            vx = min(target_speed, self.max_vx, max(0.18, target_speed * 0.80))
        elif control_mode == 'CURVE':
            vx = min(target_speed, self.max_vx)
        elif control_mode == 'WIGGLE':
            vx = min(target_speed, self.max_vx, max(0.30, target_speed * 0.60))
        elif control_mode == 'RECOVER':
            if control_abs_alpha > self.turn_in_place_angle and self.turn_crawl_vx > 0.0:
                vx = min(target_speed, self.max_vx, self.turn_crawl_vx)
                turn_crawl_active = True
            else:
                vx = min(target_speed * 0.5, self.max_vx)
        elif control_abs_alpha > self.slow_down_angle:
            vx = min(target_speed * 0.5, self.max_vx)
        else:
            vx = min(target_speed, self.max_vx)
        if control_mode == 'FOLLOW':
            vy = lateral_velocity_command(
                self.lateral_error,
                self.route_yaw,
                self.current_yaw,
                gain=self.lateral_velocity_gain,
                max_vy=self.max_vy,
                deadband=self.lateral_velocity_deadband,
                heading_limit_deg=self.lateral_velocity_heading_limit_deg,
            )
        else:
            # Curves, corners, heading recovery and faults remain rotation-only.
            vy = 0.0
        now = time.time()
        dt = max(0.02, min(0.20, now - self.last_cmd_time))
        vx = self.smooth_value(vx, self.last_cmd_vx, self.max_vx_accel, dt)
        if control_mode == 'FOLLOW':
            vy = self.smooth_value(vy, self.last_cmd_vy, self.max_vy_accel, dt)
        else:
            vy = 0.0
        yaw_rate = self.smooth_value(yaw_rate, self.last_cmd_yaw_rate, self.max_yaw_accel, dt)
        self.last_cmd_vx = vx
        self.last_cmd_vy = vy
        self.last_cmd_yaw_rate = yaw_rate
        self.lateral_velocity = vy
        self.last_cmd_time = now

        expecting_route_progress = (
            abs(vx) > 0.05
            and control_mode in ('FOLLOW', 'APPROACH_CORNER', 'CORNER', 'WIGGLE')
        )
        if not expecting_route_progress:
            self.reset_progress_watchdog(now)

        stuck_elapsed = now - self.last_progress_time
        soft_unstick_active = False
        if expecting_route_progress and stuck_elapsed > self.stuck_time:
            moved_since_progress = 0.0
            if self.last_progress_x is not None and self.last_progress_y is not None:
                moved_since_progress = math.hypot(
                    self.current_x - self.last_progress_x,
                    self.current_y - self.last_progress_y
                )
            allow_soft_unstick = (
                control_mode == 'FOLLOW'
                and not sharp_corner
                and not corner_control_active
                and abs(heading_yaw_rate) <= self.soft_unstick_yaw_rate
            )
            if allow_soft_unstick and moved_since_progress < self.soft_unstick_min_moved and self.soft_unstick_yaw_rate > 0.0:
                if self.soft_unstick_sign == 0.0:
                    self.soft_unstick_sign = 1.0 if self.lateral_error <= 0.0 else -1.0
                phase = int(max(0.0, stuck_elapsed - self.stuck_time) / 2.0)
                unstick_sign = self.soft_unstick_sign if phase % 2 == 0 else -self.soft_unstick_sign
                yaw_rate = unstick_sign * max(abs(yaw_rate), self.soft_unstick_yaw_rate)
                yaw_rate = max(-self.max_yaw_rate, min(self.max_yaw_rate, yaw_rate))
                self.last_cmd_yaw_rate = yaw_rate
                soft_unstick_active = True
            if now - self.last_stuck_log_time > 2.0:
                self.last_stuck_log_time = now
                self.get_logger().warn(
                    f'no waypoint progress for {stuck_elapsed:.1f}s; csv follower applies soft unstick when movement is too small; '
                    f'pose=({self.current_x:.3f},{self.current_y:.3f},{self.current_yaw:.3f}), '
                    f'progress_s={self.current_s:.3f}, last_progress_s={self.last_progress_s:.3f}, '
                    f'moved_since_progress={moved_since_progress:.3f}, '
                    f'projected=({self.projected_x:.3f},{self.projected_y:.3f}), '
                    f'projected_segment_index={self.projected_segment_index}, '
                    f'segment_t={self.projected_segment_t:.3f}, raw_s={self.raw_projected_s:.3f}, '
                    f'raw_segment_index={self.raw_projected_segment_index}, raw_t={self.raw_projected_segment_t:.3f}, '
                    f'projection_limited={self.projection_limited}, '
                    f'target_s={target_s:.3f}, target=({self.target_x:.3f},{self.target_y:.3f}), '
                    f'lookahead_distance={self.effective_lookahead_distance:.3f}, '
                    f'corner_angle={self.lookahead_corner_angle_ahead:.3f}, '
                    f'corner_angle_deg={math.degrees(self.lookahead_corner_angle_ahead):.1f}, '
                    f'corner_s={self.lookahead_corner_s_ahead:.3f}, '
                    f'corner_distance={self.lookahead_corner_distance_ahead:.3f}, '
                    f'sharp_corner={self.lookahead_sharp_corner_ahead}, '
                    f'curve_angle_deg={math.degrees(self.lookahead_curve_angle_near):.1f}, '
                    f'curve_distance={self.lookahead_curve_distance_ahead:.3f}, '
                    f'curve_active={self.lookahead_curve_active}, '
                    f'target_segment_index={target_segment_index}, target_t={target_segment_t:.3f}, '
                    f'route_yaw={self.route_yaw:.3f}, target_alpha={target_alpha:.3f}, '
                    f'route_heading_error={self.route_heading_error:.3f}, '
                    f'heading_control={heading_control_source}:{heading_control_error:.3f}, '
                    f'mode={control_mode}, state_reason={self.follow_state_reason}, corner_turn={corner_turn_active}, '
                    f'corner_heading_error={corner_heading_error:.3f}, '
                    f'curve_heading_error={curve_heading_error:.3f}, '
                    f'lateral_error={self.lateral_error:.3f}, heading_yaw_rate={heading_yaw_rate:.3f}, '
                    f'lateral_yaw_rate={self.lateral_yaw_rate:.3f}, '
                    f'cmd=({vx:.3f},{vy:.3f},{yaw_rate:.3f}), dir={self.direction}, '
                    f'turn_crawl={turn_crawl_active}, soft_unstick={soft_unstick_active}, '
                    f'allow_soft_unstick={allow_soft_unstick}'
                )

        cmd = Twist()
        cmd.linear.x = float(vx)
        cmd.linear.y = float(vy)
        cmd.angular.z = float(yaw_rate)
        self.pub.publish(cmd)

        if now - self.last_log_time > 1.0:
            self.last_log_time = now
            self.get_logger().info(
                f'nearest={self.nearest_index}, target={self.target_index}, '
                f's={self.current_s:.2f}/{self.total_length:.2f}, '
                f'projected_segment_index={self.projected_segment_index}, segment_t={self.projected_segment_t:.2f}, '
                f'raw_s={self.raw_projected_s:.2f}, raw_segment_index={self.raw_projected_segment_index}, '
                f'raw_t={self.raw_projected_segment_t:.2f}, projection_limited={self.projection_limited}, '
                f'target_s={target_s:.2f}, target_xy=({self.target_x:.2f},{self.target_y:.2f}), '
                f'lookahead_distance={self.effective_lookahead_distance:.2f}, '
                f'corner_angle={self.lookahead_corner_angle_ahead:.2f}, '
                f'corner_angle_deg={math.degrees(self.lookahead_corner_angle_ahead):.1f}, '
                f'corner_s={self.lookahead_corner_s_ahead:.2f}, '
                f'corner_distance={self.lookahead_corner_distance_ahead:.2f}, '
                f'sharp_corner={self.lookahead_sharp_corner_ahead}, '
                f'curve_angle_deg={math.degrees(self.lookahead_curve_angle_near):.1f}, '
                f'curve_distance={self.lookahead_curve_distance_ahead:.2f}, '
                f'curve_active={self.lookahead_curve_active}, '
                f'target_segment_index={target_segment_index}, target_t={target_segment_t:.2f}, '
                f'route_yaw={self.route_yaw:.2f}, '
                f'dist={dist_to_target:.2f}, route_d={self.last_local_route_distance:.2f}, '
                f'lateral_error={self.lateral_error:.2f}, route_heading_error={self.route_heading_error:.2f}, '
                f'lateral_yaw_rate={self.lateral_yaw_rate:.2f}, '
                f'lateral_velocity={self.lateral_velocity:.2f}, '
                f'pose=({self.current_x:.2f},{self.current_y:.2f},{self.current_yaw:.2f}), '
                f'recovery={recovery_mode}, dist_recovery={distance_recovery}, heading_recovery={heading_recovery}, '
                f'target_alpha={target_alpha:.2f}, heading_control={heading_control_source}:{heading_control_error:.2f}, '
                f'mode={control_mode}, state_reason={self.follow_state_reason}, corner_turn={corner_turn_active}, '
                f'corner_heading_error={corner_heading_error:.2f}, '
                f'curve_heading_error={curve_heading_error:.2f}, '
                f'heading_yaw_rate={heading_yaw_rate:.2f}, '
                f'vx={vx:.2f}, vy={vy:.2f}, yaw_rate={yaw_rate:.2f}, dir={self.direction}, '
                f'turn_crawl={turn_crawl_active}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('waypoint_follower stopped by Ctrl+C')
        node.publish_stop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

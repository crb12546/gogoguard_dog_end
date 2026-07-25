#!/usr/bin/env python3
"""Deterministic sequential CSV route follower for Unitree Go2."""

import csv
import json
import math
import time
from collections import deque

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

from .patrol_control import (
    corner_turn_angle,
    displacement_course_heading,
    effective_forward_speed,
    feedback_motion_scale,
    line_follow_command,
    normalize_angle,
    ordered_upcoming_corner,
    ordered_route_heading,
    segment_metrics,
    stream_receive_age,
    waypoint_reached,
)


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def timing_summary(values):
    valid = sorted(
        float(value)
        for value in values
        if math.isfinite(float(value))
    )
    if not valid:
        return 0.0, 0.0, 0.0
    p95_index = max(0, int(math.ceil(len(valid) * 0.95)) - 1)
    return (
        sum(valid) / len(valid),
        valid[p95_index],
        valid[-1],
    )


class WaypointFollower(Node):
    """Follow CSV segments in order without nearest-route reprojection."""

    def __init__(self):
        super().__init__('waypoint_follower')

        self.declare_parameter('odom_topic', '/Odometry')
        self.declare_parameter('cmd_topic', '/patrol_cmd')
        self.declare_parameter(
            'route_file',
            '/home/unitree/go2_fastlio_ws/src/'
            'go2_fastlio_patrol/routes/route_demo.csv',
        )
        self.declare_parameter('route_transform_json', '')

        self.declare_parameter('v_base', 0.25)
        self.declare_parameter('max_vx', 0.25)
        self.declare_parameter('use_route_speed', True)

        self.declare_parameter('line_deadband', 0.030)
        self.declare_parameter('tracking_lookahead_distance', 0.50)
        self.declare_parameter('max_correction_angle_deg', 12.0)
        self.declare_parameter('course_heading_window', 0.80)
        self.declare_parameter('course_heading_min_distance', 0.12)
        self.declare_parameter('minimum_moving_speed', 0.35)

        self.declare_parameter('heading_gain', 1.20)
        self.declare_parameter('min_track_yaw_rate', 0.20)
        self.declare_parameter('max_track_yaw_rate', 0.60)
        self.declare_parameter('yaw_deadband', 0.03)
        self.declare_parameter('heading_slow_angle_deg', 20.0)
        self.declare_parameter('heading_stop_angle_deg', 75.0)

        self.declare_parameter('waypoint_reach_distance', 0.12)
        self.declare_parameter('corner_angle_deg', 30.0)
        self.declare_parameter('corner_reach_distance', 0.20)
        self.declare_parameter('corner_slowdown_distance', 0.90)
        self.declare_parameter('corner_min_speed', 0.35)

        self.declare_parameter('loop_mode', 'once')
        self.declare_parameter('start_max_distance', 0.35)
        self.declare_parameter('start_max_yaw_error', 0.35)
        self.declare_parameter('emergency_route_deviation', 1.50)
        self.declare_parameter('emergency_deviation_time', 1.00)
        self.declare_parameter('odom_normal_age', 0.60)
        self.declare_parameter('odom_slow_age', 1.20)
        self.declare_parameter('odom_hold_age', 2.00)
        self.declare_parameter('odom_slow_scale', 0.50)
        self.declare_parameter('odom_recovery_samples', 5)
        self.declare_parameter('max_valid_odom_abs', 10000.0)
        self.declare_parameter('max_odom_jump_distance', 0.80)
        self.declare_parameter('max_odom_jump_z', 0.50)
        self.declare_parameter('max_odom_jump_dt', 2.00)
        self.declare_parameter('max_vx_accel', 0.60)
        self.declare_parameter('max_vy_accel', 0.45)
        self.declare_parameter('max_yaw_accel', 1.00)
        self.declare_parameter('control_rate', 20.0)

        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.cmd_topic = str(self.get_parameter('cmd_topic').value)
        self.route_file = str(self.get_parameter('route_file').value)
        self.route_transform_json = str(
            self.get_parameter('route_transform_json').value
        ).strip()

        self.v_base = max(0.0, float(self.get_parameter('v_base').value))
        self.max_vx = max(0.0, float(self.get_parameter('max_vx').value))
        self.use_route_speed = bool(
            self.get_parameter('use_route_speed').value
        )

        self.line_deadband = max(
            0.0,
            float(self.get_parameter('line_deadband').value),
        )
        self.tracking_lookahead_distance = max(
            0.05,
            float(
                self.get_parameter(
                    'tracking_lookahead_distance'
                ).value
            ),
        )
        self.max_correction_angle = math.radians(max(
            0.0,
            float(
                self.get_parameter('max_correction_angle_deg').value
            ),
        ))
        self.course_heading_window = max(
            0.20,
            float(
                self.get_parameter('course_heading_window').value
            ),
        )
        self.course_heading_min_distance = max(
            0.03,
            float(
                self.get_parameter(
                    'course_heading_min_distance'
                ).value
            ),
        )
        self.minimum_moving_speed = max(
            0.0,
            float(
                self.get_parameter('minimum_moving_speed').value
            ),
        )

        self.heading_gain = max(
            0.0,
            float(self.get_parameter('heading_gain').value),
        )
        self.min_track_yaw_rate = max(
            0.0,
            float(self.get_parameter('min_track_yaw_rate').value),
        )
        self.max_track_yaw_rate = max(
            0.0,
            float(self.get_parameter('max_track_yaw_rate').value),
        )
        self.yaw_deadband = max(
            0.0,
            float(self.get_parameter('yaw_deadband').value),
        )
        self.heading_slow_angle = math.radians(max(
            0.0,
            float(
                self.get_parameter('heading_slow_angle_deg').value
            ),
        ))
        self.heading_stop_angle = math.radians(max(
            math.degrees(self.heading_slow_angle) + 1.0,
            float(
                self.get_parameter('heading_stop_angle_deg').value
            ),
        ))

        self.waypoint_reach_distance = max(
            0.01,
            float(
                self.get_parameter('waypoint_reach_distance').value
            ),
        )
        self.corner_angle = math.radians(max(
            1.0,
            float(self.get_parameter('corner_angle_deg').value),
        ))
        self.corner_reach_distance = max(
            0.01,
            float(
                self.get_parameter('corner_reach_distance').value
            ),
        )
        self.corner_slowdown_distance = max(
            self.corner_reach_distance,
            float(
                self.get_parameter('corner_slowdown_distance').value
            ),
        )
        self.corner_min_speed = max(
            0.0,
            float(self.get_parameter('corner_min_speed').value),
        )

        self.loop_mode = str(self.get_parameter('loop_mode').value)
        if self.loop_mode not in ('once', 'pingpong'):
            self.loop_mode = 'once'
        self.start_max_distance = max(
            0.0,
            float(self.get_parameter('start_max_distance').value),
        )
        self.start_max_yaw_error = max(
            0.0,
            float(self.get_parameter('start_max_yaw_error').value),
        )
        self.emergency_route_deviation = max(
            self.line_deadband + 0.20,
            float(
                self.get_parameter(
                    'emergency_route_deviation'
                ).value
            ),
        )
        self.emergency_deviation_time = max(
            0.20,
            float(
                self.get_parameter('emergency_deviation_time').value
            ),
        )
        self.odom_normal_age = max(
            0.05,
            float(self.get_parameter('odom_normal_age').value),
        )
        self.odom_slow_age = max(
            self.odom_normal_age + 0.05,
            float(self.get_parameter('odom_slow_age').value),
        )
        self.odom_hold_age = max(
            self.odom_slow_age + 0.10,
            float(self.get_parameter('odom_hold_age').value),
        )
        self.odom_slow_scale = max(
            0.0,
            min(
                1.0,
                float(self.get_parameter('odom_slow_scale').value),
            ),
        )
        self.odom_recovery_samples = max(
            1,
            int(self.get_parameter('odom_recovery_samples').value),
        )
        self.max_valid_odom_abs = max(
            1.0,
            float(self.get_parameter('max_valid_odom_abs').value),
        )
        self.max_odom_jump_distance = max(
            0.0,
            float(
                self.get_parameter('max_odom_jump_distance').value
            ),
        )
        self.max_odom_jump_z = max(
            0.0,
            float(self.get_parameter('max_odom_jump_z').value),
        )
        self.max_odom_jump_dt = max(
            0.1,
            float(self.get_parameter('max_odom_jump_dt').value),
        )
        self.max_vx_accel = max(
            0.0,
            float(self.get_parameter('max_vx_accel').value),
        )
        self.max_vy_accel = max(
            0.0,
            float(self.get_parameter('max_vy_accel').value),
        )
        self.max_yaw_accel = max(
            0.0,
            float(self.get_parameter('max_yaw_accel').value),
        )
        self.control_rate = max(
            5.0,
            float(self.get_parameter('control_rate').value),
        )

        self.route = self.load_route(self.route_file)
        self.arc_s, self.total_length = self.build_route_geometry(
            self.route
        )
        (
            self.route_transform,
            self.route_transform_active,
        ) = self.load_route_transform(self.route_transform_json)
        self.route_transform_yaw = self.yaw_from_transform(
            self.route_transform
        )

        self.current_x = None
        self.current_y = None
        self.current_yaw = None
        self.motion_world_vx = 0.0
        self.motion_world_vy = 0.0
        self.motion_valid = False
        self.course_samples = deque()
        self.course_yaw = 0.0
        self.course_speed = 0.0
        self.course_valid = False

        self.last_raw_odom_x = None
        self.last_raw_odom_y = None
        self.last_raw_odom_z = None
        self.last_raw_odom_time = None
        self.last_odom_receive_time = 0.0
        self.last_odom_stamp_age = float('inf')
        self.last_odom_receive_age = float('inf')
        self.fresh_odom_samples = 0
        self.last_route_odom_x = None
        self.last_route_odom_y = None
        self.last_route_odom_time = None

        self.direction = 1
        self.anchor_index = 0
        self.target_index = 0
        self.state = 'WAIT_ODOM'
        self.state_reason = 'waiting_for_odometry'
        self.initialized = False
        self.finished = False
        self.tracking_fault = False
        self.emergency_deviation_since = None
        self.corner_preparing = False
        self.feedback_hold = True
        self.feedback_mode = 'WAIT_FRESH_ODOM'
        self.feedback_scale = 0.0
        self.last_metrics = None
        self.last_turn_angle = 0.0
        self.last_corner_remaining = float('inf')
        self.last_cross_velocity = 0.0
        self.last_predicted_lateral_error = 0.0
        self.last_correction_angle = 0.0
        self.last_route_guide_yaw = 0.0
        self.last_target_heading = 0.0
        self.last_heading_error = 0.0
        self.last_heading_feedback_yaw = 0.0
        self.last_heading_feedback_source = 'body'
        self.last_body_course_slip = 0.0
        self.last_course_route_error = 0.0

        self.last_cmd_vx = 0.0
        self.last_cmd_vy = 0.0
        self.last_cmd_yaw_rate = 0.0
        self.last_cmd_time = time.time()
        self.last_log_time = 0.0
        self.last_odom_fault_log_time = 0.0
        self.last_feedback_log_time = 0.0
        self.timing_control_period = 1.0 / self.control_rate
        self.timing_last_control_mono = 0.0
        self.timing_last_odom_mono = 0.0
        self.timing_last_report_mono = time.monotonic()
        self.timing_last_alert_mono = 0.0
        self.timing_loop_gaps_ms = []
        self.timing_control_compute_ms = []
        self.timing_odom_gaps_ms = []
        self.timing_odom_compute_ms = []
        self.timing_odom_stamp_age_ms = []
        self.timing_odom_to_cmd_ms = []
        self.timing_publish_ms = []
        self.timing_cmd_count = 0
        self.timing_stop_count = 0
        self.timing_deadline_misses = 0

        self.pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10,
        )
        self.timer = self.create_timer(
            1.0 / self.control_rate,
            self.control_loop,
        )

        self.get_logger().info(
            'simple csv waypoint_follower started: '
            'sequential line-lock controller'
        )
        self.get_logger().info(
            f'route_file={self.route_file}, '
            f'points={len(self.route)}, length={self.total_length:.2f}m, '
            f'loop_mode={self.loop_mode}'
        )
        self.get_logger().info(
            'single_controller: ordered CSV tangent plus continuous '
            'cross-track correction, vy=0, '
            f'deadband={self.line_deadband:.3f}m, '
            f'lookahead={self.tracking_lookahead_distance:.2f}m, '
            f'max_correction='
            f'{math.degrees(self.max_correction_angle):.1f}deg, '
            f'course_window={self.course_heading_window:.2f}s, '
            f'course_min_distance='
            f'{self.course_heading_min_distance:.2f}m, '
            f'minimum_moving_speed={self.minimum_moving_speed:.2f}m/s'
        )
        self.get_logger().info(
            f'corner: angle={math.degrees(self.corner_angle):.1f}deg, '
            f'slowdown={self.corner_slowdown_distance:.2f}m, '
            f'reach={self.corner_reach_distance:.2f}m; '
            'same controller remains active through corners'
        )
        self.get_logger().info(
            'progress policy: advance every passed ordered CSV gate; '
            'nearest-route projection and lateral gate lock disabled'
        )
        self.get_logger().info(
            'feedback policy: hard freshness uses local receive time; '
            'message stamp age is diagnostic only; '
            f'normal<={self.odom_normal_age:.2f}s, '
            f'slow<={self.odom_slow_age:.2f}s, '
            f'hold>={self.odom_hold_age:.2f}s, '
            f'recovery_samples={self.odom_recovery_samples}'
        )
        self.get_logger().info(
            'deviation policy: '
            f'emergency={self.emergency_route_deviation:.2f}m/'
            f'{self.emergency_deviation_time:.1f}s'
        )
        self.get_logger().info(
            'fault policy: continuously publish zero command until '
            'patrol is explicitly stopped'
        )

    @staticmethod
    def load_route(path):
        route = []
        with open(path, 'r', newline='') as handle:
            reader = csv.DictReader(handle)
            required = {'x', 'y', 'yaw', 'v'}
            missing = required - set(reader.fieldnames or [])
            if missing:
                raise RuntimeError(
                    'route CSV missing columns: '
                    + ','.join(sorted(missing))
                )
            for line_number, row in enumerate(reader, start=2):
                try:
                    point = {
                        'x': float(row['x']),
                        'y': float(row['y']),
                        'yaw': float(row['yaw']),
                        'v': float(row['v']),
                    }
                except (TypeError, ValueError) as exc:
                    raise RuntimeError(
                        f'invalid route value at line {line_number}: {exc}'
                    ) from exc
                if not all(math.isfinite(value) for value in point.values()):
                    raise RuntimeError(
                        f'non-finite route value at line {line_number}'
                    )
                route.append(point)
        if len(route) < 2:
            raise RuntimeError('route requires at least two points')
        return route

    @staticmethod
    def build_route_geometry(route):
        arc_s = [0.0]
        for previous, current in zip(route, route[1:]):
            arc_s.append(
                arc_s[-1]
                + math.hypot(
                    current['x'] - previous['x'],
                    current['y'] - previous['y'],
                )
            )
        if arc_s[-1] <= 1e-6:
            raise RuntimeError('route has no non-zero segments')
        return arc_s, arc_s[-1]

    @staticmethod
    def load_route_transform(path):
        identity = [
            1.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 1.0,
        ]
        if not path:
            return identity, False
        with open(path, 'r') as handle:
            data = json.load(handle)
        values = data.get('transformOldFromCurrent')
        if not isinstance(values, list) or len(values) != 16:
            raise RuntimeError(
                'route_transform_json must contain '
                'transformOldFromCurrent[16]'
            )
        return [float(value) for value in values], True

    @staticmethod
    def yaw_from_transform(matrix):
        return math.atan2(matrix[4], matrix[0])

    def transform_pose_to_route_frame(self, x, y, z, yaw):
        matrix = self.route_transform
        route_x = (
            matrix[0] * x
            + matrix[1] * y
            + matrix[2] * z
            + matrix[3]
        )
        route_y = (
            matrix[4] * x
            + matrix[5] * y
            + matrix[6] * z
            + matrix[7]
        )
        return (
            route_x,
            route_y,
            normalize_angle(yaw + self.route_transform_yaw),
        )

    def odom_is_valid(self, x, y, z, yaw):
        values = (x, y, z, yaw)
        return (
            all(math.isfinite(value) for value in values)
            and abs(x) <= self.max_valid_odom_abs
            and abs(y) <= self.max_valid_odom_abs
            and abs(z) <= self.max_valid_odom_abs
        )

    def update_course_estimate(self, x, y, odom_time):
        sample = (float(odom_time), float(x), float(y))
        if (
            self.course_samples
            and sample[0] <= self.course_samples[-1][0]
        ):
            self.course_samples.clear()
        self.course_samples.append(sample)

        cutoff = sample[0] - self.course_heading_window
        while (
            len(self.course_samples) >= 2
            and self.course_samples[1][0] <= cutoff
        ):
            self.course_samples.popleft()

        start_time, start_x, start_y = self.course_samples[0]
        elapsed = sample[0] - start_time
        distance = math.hypot(
            sample[1] - start_x,
            sample[2] - start_y,
        )
        course_yaw = displacement_course_heading(
            start_x,
            start_y,
            sample[1],
            sample[2],
            self.course_heading_min_distance,
        )
        if elapsed > 0.0 and course_yaw is not None:
            self.course_yaw = course_yaw
            self.course_speed = distance / elapsed
            self.course_valid = True
        else:
            self.course_speed = 0.0
            self.course_valid = False

    def reset_course_estimate(self):
        self.course_samples.clear()
        self.course_speed = 0.0
        self.course_valid = False

    def set_fault(self, reason):
        if not self.tracking_fault:
            self.get_logger().error(
                f'patrol fault: {reason}; hold zero command until '
                'patrol is explicitly stopped'
            )
        self.tracking_fault = True
        self.state = 'FAULT'
        self.state_reason = reason
        self.publish_stop()

    def odom_callback(self, msg):
        callback_start = time.monotonic()
        if self.timing_last_odom_mono > 0.0:
            gap_ms = (
                callback_start - self.timing_last_odom_mono
            ) * 1000.0
            self.timing_odom_gaps_ms.append(gap_ms)
        self.timing_last_odom_mono = callback_start
        try:
            self._odom_callback_impl(msg)
        finally:
            callback_ms = (
                time.monotonic() - callback_start
            ) * 1000.0
            self.timing_odom_compute_ms.append(callback_ms)
            if math.isfinite(self.last_odom_stamp_age):
                self.timing_odom_stamp_age_ms.append(
                    self.last_odom_stamp_age * 1000.0
                )

    def _odom_callback_impl(self, msg):
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        z = float(msg.pose.pose.position.z)
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        if not self.odom_is_valid(x, y, z, yaw):
            self.set_fault(
                f'invalid odometry x={x:.3f} y={y:.3f} '
                f'z={z:.3f} yaw={yaw:.3f}'
            )
            return

        now = time.time()
        previous_receive_time = self.last_odom_receive_time
        self.last_odom_receive_time = now
        self.last_odom_receive_age = 0.0
        stamp_time = (
            float(msg.header.stamp.sec)
            + float(msg.header.stamp.nanosec) * 1e-9
        )
        odom_time = stamp_time if stamp_time > 0.0 else now
        if odom_time > now + 0.50:
            self.last_odom_stamp_age = float('inf')
        else:
            self.last_odom_stamp_age = max(0.0, now - odom_time)
        receive_gap = stream_receive_age(now, previous_receive_time)
        if receive_gap <= self.odom_normal_age:
            self.fresh_odom_samples += 1
        else:
            self.fresh_odom_samples = 1

        if self.last_raw_odom_time is not None:
            dt = odom_time - self.last_raw_odom_time
            if 0.0 < dt <= self.max_odom_jump_dt:
                dx = x - self.last_raw_odom_x
                dy = y - self.last_raw_odom_y
                dz = z - self.last_raw_odom_z
                jump = math.sqrt(dx * dx + dy * dy + dz * dz)
                dynamic_xy_limit = max(
                    self.max_odom_jump_distance,
                    self.max_vx * dt * 2.5 + 0.30,
                )
                dynamic_z_limit = max(
                    self.max_odom_jump_z,
                    0.25 * dt + 0.20,
                )
                if (
                    jump > dynamic_xy_limit
                    or abs(dz) > dynamic_z_limit
                ):
                    self.set_fault(
                        f'odometry jump={jump:.3f}m '
                        f'dz={dz:.3f}m dt={dt:.3f}s'
                    )
                    return

        self.last_raw_odom_x = x
        self.last_raw_odom_y = y
        self.last_raw_odom_z = z
        self.last_raw_odom_time = odom_time

        route_x, route_y, route_yaw = (
            self.transform_pose_to_route_frame(x, y, z, yaw)
        )
        if not self.odom_is_valid(
            route_x,
            route_y,
            0.0,
            route_yaw,
        ):
            self.set_fault('invalid transformed odometry')
            return

        if self.last_route_odom_time is not None:
            dt = odom_time - self.last_route_odom_time
            if 0.01 <= dt <= 0.50:
                measured_vx = (
                    route_x - self.last_route_odom_x
                ) / dt
                measured_vy = (
                    route_y - self.last_route_odom_y
                ) / dt
                alpha = 0.35
                if not self.motion_valid:
                    self.motion_world_vx = measured_vx
                    self.motion_world_vy = measured_vy
                else:
                    self.motion_world_vx = (
                        (1.0 - alpha) * self.motion_world_vx
                        + alpha * measured_vx
                    )
                    self.motion_world_vy = (
                        (1.0 - alpha) * self.motion_world_vy
                        + alpha * measured_vy
                    )
                self.motion_valid = True

        self.last_route_odom_x = route_x
        self.last_route_odom_y = route_y
        self.last_route_odom_time = odom_time
        self.current_x = route_x
        self.current_y = route_y
        self.current_yaw = route_yaw
        self.update_course_estimate(route_x, route_y, odom_time)

        if not self.initialized and not self.tracking_fault:
            if self.fresh_odom_samples >= self.odom_recovery_samples:
                self.initialize_at_route_start()
            elif now - self.last_odom_fault_log_time > 1.0:
                self.last_odom_fault_log_time = now
                self.get_logger().warn(
                    'waiting for fresh odometry before patrol: '
                    f'receive_gap={receive_gap:.3f}s, '
                    f'stamp_age={self.last_odom_stamp_age:.3f}s, '
                    f'fresh={self.fresh_odom_samples}/'
                    f'{self.odom_recovery_samples}'
                )

    def find_next_distinct(self, index, direction):
        anchor = self.route[index]
        candidate = index + direction
        while 0 <= candidate < len(self.route):
            point = self.route[candidate]
            if math.hypot(
                point['x'] - anchor['x'],
                point['y'] - anchor['y'],
            ) > 1e-4:
                return candidate
            candidate += direction
        return None

    def initialize_at_route_start(self):
        target_index = self.find_next_distinct(0, 1)
        if target_index is None:
            self.set_fault('route has no usable first segment')
            return

        start = self.route[0]
        target = self.route[target_index]
        distance = math.hypot(
            self.current_x - start['x'],
            self.current_y - start['y'],
        )
        start_yaw = math.atan2(
            target['y'] - start['y'],
            target['x'] - start['x'],
        )
        yaw_error = normalize_angle(start_yaw - self.current_yaw)
        if (
            distance > self.start_max_distance
            or abs(yaw_error) > self.start_max_yaw_error
        ):
            self.set_fault(
                f'route start mismatch: distance={distance:.3f}/'
                f'{self.start_max_distance:.3f}m, '
                f'yaw_error={math.degrees(yaw_error):.1f}/'
                f'{math.degrees(self.start_max_yaw_error):.1f}deg'
            )
            return

        self.direction = 1
        self.anchor_index = 0
        self.target_index = target_index
        self.state = 'TRACK'
        self.state_reason = 'route_start_aligned'
        self.initialized = True
        self.feedback_hold = False
        self.feedback_mode = 'NORMAL'
        self.feedback_scale = 1.0
        self.get_logger().info(
            f'route start aligned: distance={distance:.3f}m, '
            f'yaw_error={math.degrees(yaw_error):.1f}deg, '
            f'first_segment=0->{target_index}'
        )

    def current_segment_metrics(self):
        start = self.route[self.anchor_index]
        target = self.route[self.target_index]
        return segment_metrics(
            self.current_x,
            self.current_y,
            start['x'],
            start['y'],
            target['x'],
            target['y'],
        )

    def corner_at_target(self):
        next_index = self.find_next_distinct(
            self.target_index,
            self.direction,
        )
        if next_index is None:
            return False, 0.0, None
        start = self.route[self.anchor_index]
        corner = self.route[self.target_index]
        end = self.route[next_index]
        angle = corner_turn_angle(
            (start['x'], start['y']),
            (corner['x'], corner['y']),
            (end['x'], end['y']),
        )
        return abs(angle) >= self.corner_angle, angle, next_index

    def upcoming_sharp_corner(self, metrics, search_distance=None):
        """Find the next ordered corner in the preparation horizon."""
        if search_distance is None:
            search_distance = self.corner_slowdown_distance
        try:
            return ordered_upcoming_corner(
                route=self.route,
                anchor_index=self.anchor_index,
                target_index=self.target_index,
                direction=self.direction,
                current_segment_remaining=metrics['remaining'],
                corner_angle=self.corner_angle,
                search_distance=search_distance,
            )
        except RuntimeError as exc:
            self.set_fault(str(exc))
            return None

    def handle_reached_target(self, sharp_corner, turn_angle, next_index):
        reached_index = self.target_index
        if next_index is None:
            if self.loop_mode == 'once':
                self.finished = True
                self.state = 'DONE'
                self.state_reason = 'route_endpoint_reached'
                self.get_logger().info(
                    'goal reached; hold zero command until patrol is '
                    'explicitly stopped'
                )
                self.publish_stop()
                return

            self.direction *= -1
            self.anchor_index = reached_index
            reverse_target = self.find_next_distinct(
                self.anchor_index,
                self.direction,
            )
            if reverse_target is None:
                self.set_fault('cannot reverse at route endpoint')
                return
            self.target_index = reverse_target
            self.state = 'TRACK'
            self.state_reason = 'pingpong_direction_reversed'
            self.last_turn_angle = math.pi
            self.reset_course_estimate()
            self.get_logger().info(
                'pingpong endpoint: same continuous controller '
                f'reverses toward segment={self.anchor_index}->'
                f'{self.target_index}'
            )
            return

        self.anchor_index = reached_index
        self.target_index = next_index
        if sharp_corner:
            self.state_reason = 'passed_csv_corner_continuously'
            self.last_turn_angle = turn_angle
        else:
            self.state_reason = 'next_csv_segment'
        self.state = 'TRACK'

    def route_progress_s(self, metrics):
        along = max(
            0.0,
            min(float(metrics['length']), float(metrics['along'])),
        )
        if self.direction > 0:
            return self.arc_s[self.anchor_index] + along
        return self.arc_s[self.anchor_index] - along

    def update_feedback_state(self):
        now = time.time()
        receive_age = stream_receive_age(
            now,
            self.last_odom_receive_time,
        )
        self.last_odom_receive_age = receive_age
        if receive_age > self.odom_normal_age:
            self.fresh_odom_samples = 0

        scale = feedback_motion_scale(
            receive_age,
            normal_age=self.odom_normal_age,
            slow_age=self.odom_slow_age,
            hold_age=self.odom_hold_age,
            slow_scale=self.odom_slow_scale,
        )
        previous_mode = self.feedback_mode

        if self.feedback_hold:
            if (
                receive_age <= self.odom_normal_age
                and self.fresh_odom_samples
                >= self.odom_recovery_samples
            ):
                self.feedback_hold = False
                self.feedback_mode = 'NORMAL'
                scale = 1.0
                self.get_logger().info(
                    'fresh odometry recovered; patrol resumes '
                    f'after {self.fresh_odom_samples} samples'
                )
            else:
                self.feedback_mode = 'WAIT_FRESH_ODOM'
                scale = 0.0
        elif receive_age >= self.odom_hold_age:
            self.feedback_hold = True
            self.feedback_mode = 'WAIT_FRESH_ODOM'
            scale = 0.0
            self.get_logger().error(
                'odometry stream stopped; smoothly hold and wait for '
                f'automatic recovery: receive_age={receive_age:.3f}s, '
                f'stamp_age={self.last_odom_stamp_age:.3f}s'
            )
        elif receive_age > self.odom_slow_age:
            self.feedback_mode = 'DECELERATE'
        elif receive_age > self.odom_normal_age:
            self.feedback_mode = 'DEGRADED'
        else:
            self.feedback_mode = 'NORMAL'

        self.feedback_scale = scale
        if (
            self.feedback_mode != previous_mode
            and self.feedback_mode not in ('NORMAL', 'WAIT_FRESH_ODOM')
        ):
            self.get_logger().warn(
                f'feedback mode={self.feedback_mode}, '
                f'odom_receive_age={receive_age:.3f}s, '
                f'odom_stamp_age={self.last_odom_stamp_age:.3f}s, '
                f'motion_scale={scale:.2f}'
            )
        if (
            self.feedback_hold
            and now - self.last_feedback_log_time > 1.0
        ):
            self.last_feedback_log_time = now
            self.get_logger().warn(
                'patrol waiting for fresh odometry: '
                f'receive_age={receive_age:.3f}s, '
                f'stamp_age={self.last_odom_stamp_age:.3f}s, '
                f'fresh={self.fresh_odom_samples}/'
                f'{self.odom_recovery_samples}; auto-resume enabled'
            )
        return scale

    def control_track(self):
        for _ in range(32):
            metrics = self.current_segment_metrics()
            sharp, turn_angle, next_index = self.corner_at_target()
            reach_distance = (
                self.corner_reach_distance
                if sharp
                else self.waypoint_reach_distance
            )
            if not waypoint_reached(
                metrics,
                reach_distance,
            ):
                break
            self.handle_reached_target(
                sharp,
                turn_angle,
                next_index,
            )
            if self.state != 'TRACK':
                return
        else:
            self.set_fault('too many CSV waypoint gates in one cycle')
            return

        metrics = self.current_segment_metrics()
        self.last_metrics = metrics
        lateral_error = float(metrics['lateral_error'])
        abs_lateral_error = abs(lateral_error)
        now = time.time()
        if abs_lateral_error >= self.emergency_route_deviation:
            if self.emergency_deviation_since is None:
                self.emergency_deviation_since = now
                self.get_logger().error(
                    'extreme route deviation detected; trying recovery '
                    f'before final stop: error={abs_lateral_error:.3f}m'
                )
            elif (
                now - self.emergency_deviation_since
                >= self.emergency_deviation_time
            ):
                self.set_fault(
                    f'extreme route deviation='
                    f'{abs_lateral_error:.3f}m for '
                    f'{now - self.emergency_deviation_since:.1f}s '
                    f'on segment {self.anchor_index}->'
                    f'{self.target_index}'
                )
                return
        else:
            self.emergency_deviation_since = None

        if self.motion_valid:
            cross_velocity = (
                -math.sin(metrics['yaw']) * self.motion_world_vx
                + math.cos(metrics['yaw']) * self.motion_world_vy
            )
        else:
            cross_velocity = 0.0
        self.last_cross_velocity = cross_velocity

        upcoming_corner = self.upcoming_sharp_corner(metrics)
        if self.tracking_fault:
            return
        corner_preparing = (
            upcoming_corner is not None
            and upcoming_corner['distance']
            <= self.corner_slowdown_distance
        )
        if corner_preparing and not self.corner_preparing:
            self.get_logger().info(
                'enter continuous corner slowdown: '
                f'corner_index={upcoming_corner["index"]}, '
                f'remaining={upcoming_corner["distance"]:.3f}m; '
                'same tracking controller remains active'
            )
        self.corner_preparing = corner_preparing

        guide_yaw = ordered_route_heading(
            route=self.route,
            arc_s=self.arc_s,
            progress_s=self.route_progress_s(metrics),
            direction=self.direction,
            lookahead_distance=self.tracking_lookahead_distance,
            fallback_yaw=metrics['yaw'],
        )
        self.last_route_guide_yaw = guide_yaw
        self.last_course_route_error = (
            normalize_angle(self.course_yaw - guide_yaw)
            if self.course_valid
            else float('inf')
        )

        if corner_preparing:
            self.state_reason = 'continuous_csv_corner_slowdown'
            self.last_turn_angle = upcoming_corner['turn_angle']
            self.last_corner_remaining = upcoming_corner['distance']
        else:
            self.state_reason = 'continuous_csv_tracking'
            self.last_turn_angle = 0.0
            self.last_corner_remaining = float('inf')

        target_speed = min(self.v_base, self.max_vx)
        if self.use_route_speed:
            target_speed = min(
                target_speed,
                max(0.0, float(self.route[self.target_index]['v'])),
            )

        if corner_preparing:
            ratio = max(
                0.0,
                min(
                    1.0,
                    upcoming_corner['distance']
                    / self.corner_slowdown_distance,
                ),
            )
            target_speed = min(
                target_speed,
                self.corner_min_speed
                + (target_speed - self.corner_min_speed) * ratio,
            )

        command = line_follow_command(
            lateral_error=lateral_error,
            lateral_velocity=cross_velocity,
            route_yaw=guide_yaw,
            current_yaw=self.current_yaw,
            forward_speed=target_speed,
            motion_yaw=(
                self.course_yaw
                if self.course_valid
                else None
            ),
            line_deadband=self.line_deadband,
            correction_lookahead_distance=(
                self.tracking_lookahead_distance
            ),
            correction_prediction_time=0.20,
            max_correction_angle=self.max_correction_angle,
            heading_gain=self.heading_gain,
            yaw_deadband=self.yaw_deadband,
            min_yaw_rate=self.min_track_yaw_rate,
            max_track_yaw_rate=self.max_track_yaw_rate,
            max_vx=self.max_vx,
        )

        abs_heading_error = abs(command['heading_error'])
        if abs_heading_error > self.heading_slow_angle:
            heading_scale = (
                self.heading_stop_angle - abs_heading_error
            ) / (
                self.heading_stop_angle - self.heading_slow_angle
            )
            command['vx'] *= max(0.0, min(1.0, heading_scale))

        self.last_predicted_lateral_error = command[
            'predicted_lateral_error'
        ]
        self.last_correction_angle = command['correction_angle']
        self.last_target_heading = command['target_yaw']
        self.last_heading_error = command['heading_error']
        self.last_heading_feedback_yaw = command.get(
            'heading_feedback_yaw',
            self.current_yaw,
        )
        self.last_heading_feedback_source = command.get(
            'heading_feedback_source',
            'body',
        )
        self.last_body_course_slip = (
            normalize_angle(self.course_yaw - self.current_yaw)
            if self.course_valid
            else 0.0
        )
        usable_floor = (
            self.minimum_moving_speed
            if abs_heading_error <= math.radians(45.0)
            else 0.0
        )
        output_vx = effective_forward_speed(
            requested_speed=command['vx'],
            target_speed=target_speed,
            motion_scale=self.feedback_scale,
            minimum_moving_speed=usable_floor,
        )
        self.publish_smoothed_command(
            output_vx,
            command['vy'] * self.feedback_scale,
            command['yaw_rate'] * self.feedback_scale,
        )

    @staticmethod
    def smooth_value(desired, previous, max_rate, dt):
        step = max(0.0, float(max_rate)) * max(0.0, float(dt))
        delta = float(desired) - float(previous)
        if abs(delta) <= step:
            return float(desired)
        return float(previous) + math.copysign(step, delta)

    def publish_smoothed_command(self, vx, vy, yaw_rate):
        now = time.time()
        dt = max(0.02, min(0.20, now - self.last_cmd_time))
        vx = self.smooth_value(
            vx,
            self.last_cmd_vx,
            self.max_vx_accel,
            dt,
        )
        vy = self.smooth_value(
            vy,
            self.last_cmd_vy,
            self.max_vy_accel,
            dt,
        )
        yaw_rate = self.smooth_value(
            yaw_rate,
            self.last_cmd_yaw_rate,
            self.max_yaw_accel,
            dt,
        )
        self.last_cmd_vx = vx
        self.last_cmd_vy = vy
        self.last_cmd_yaw_rate = yaw_rate
        self.last_cmd_time = now

        command = Twist()
        command.linear.x = float(vx)
        command.linear.y = float(vy)
        command.angular.z = float(yaw_rate)
        self.publish_command_message(command, is_stop=False)

    def publish_stop(self):
        command = Twist()
        self.publish_command_message(command, is_stop=True)
        self.last_cmd_vx = 0.0
        self.last_cmd_vy = 0.0
        self.last_cmd_yaw_rate = 0.0
        self.last_cmd_time = time.time()

    def publish_command_message(self, command, is_stop):
        publish_start = time.monotonic()
        self.pub.publish(command)
        self.timing_publish_ms.append(
            (time.monotonic() - publish_start) * 1000.0
        )
        self.timing_cmd_count += 1
        if is_stop:
            self.timing_stop_count += 1
        if self.last_odom_receive_time > 0.0:
            odom_to_cmd_ms = max(
                0.0,
                time.time() - self.last_odom_receive_time,
            ) * 1000.0
            self.timing_odom_to_cmd_ms.append(odom_to_cmd_ms)

    def control_loop(self):
        loop_start = time.monotonic()
        if self.timing_last_control_mono > 0.0:
            loop_gap_ms = (
                loop_start - self.timing_last_control_mono
            ) * 1000.0
            self.timing_loop_gaps_ms.append(loop_gap_ms)
            if loop_gap_ms > self.timing_control_period * 1500.0:
                self.timing_deadline_misses += 1
        self.timing_last_control_mono = loop_start
        try:
            self.control_cycle()
        finally:
            self.timing_control_compute_ms.append(
                (time.monotonic() - loop_start) * 1000.0
            )
            self.maybe_log_timing(time.monotonic())

    def control_cycle(self):
        if self.current_x is None or not self.initialized:
            self.publish_stop()
            return
        if self.tracking_fault or self.finished:
            self.publish_stop()
            return

        self.update_feedback_state()
        if self.feedback_hold:
            self.publish_smoothed_command(0.0, 0.0, 0.0)
            return

        if self.state == 'TRACK':
            self.control_track()
        else:
            self.set_fault(f'unexpected control state {self.state}')
            return

        now = time.time()
        if now - self.last_log_time > 1.0:
            self.last_log_time = now
            metrics = self.last_metrics or self.current_segment_metrics()
            mode = (
                'CORNER_SLOW'
                if self.state == 'TRACK' and self.corner_preparing
                else self.state
            )
            corner_remaining = (
                self.last_corner_remaining
                if math.isfinite(self.last_corner_remaining)
                else -1.0
            )
            course_yaw_for_log = (
                self.course_yaw
                if self.course_valid
                else float('nan')
            )
            self.get_logger().info(
                f'mode={mode}, reason={self.state_reason}, '
                f'feedback={self.feedback_mode}, '
                f'odom_receive_age='
                f'{self.last_odom_receive_age:.3f}s, '
                f'odom_stamp_age={self.last_odom_stamp_age:.3f}s, '
                f'motion_scale={self.feedback_scale:.2f}, '
                f'dir={self.direction}, '
                f'segment={self.anchor_index}->{self.target_index}, '
                f's={self.route_progress_s(metrics):.2f}/'
                f'{self.total_length:.2f}, '
                f'pose=({self.current_x:.3f},{self.current_y:.3f},'
                f'{self.current_yaw:.3f}), '
                f'route_yaw={metrics["yaw"]:.3f}, '
                f'guide_yaw={self.last_route_guide_yaw:.3f}, '
                f'line_error={metrics["lateral_error"]:.3f}m, '
                f'cross_velocity={self.last_cross_velocity:.3f}m/s, '
                f'predicted_line_error='
                f'{self.last_predicted_lateral_error:.3f}m, '
                f'correction_angle='
                f'{math.degrees(self.last_correction_angle):.1f}deg, '
                f'target_heading={self.last_target_heading:.3f}, '
                f'heading_feedback='
                f'{self.last_heading_feedback_source}:'
                f'{self.last_heading_feedback_yaw:.3f}, '
                f'course_yaw={course_yaw_for_log:.3f}, '
                f'course_speed={self.course_speed:.3f}m/s, '
                f'body_course_slip='
                f'{math.degrees(self.last_body_course_slip):.1f}deg, '
                f'course_route_error='
                f'{math.degrees(self.last_course_route_error):.1f}deg, '
                f'heading_error='
                f'{math.degrees(self.last_heading_error):.1f}deg, '
                f'remaining={metrics["remaining"]:.3f}m, '
                f'corner_remaining={corner_remaining:.3f}m, '
                f'next_turn={math.degrees(self.last_turn_angle):.1f}deg, '
                f'cmd=({self.last_cmd_vx:.3f},'
                f'{self.last_cmd_vy:.3f},'
                f'{self.last_cmd_yaw_rate:.3f})'
            )

    def maybe_log_timing(self, now_mono):
        if now_mono - self.timing_last_report_mono < 1.0:
            return

        loop_avg, loop_p95, loop_max = timing_summary(
            self.timing_loop_gaps_ms
        )
        compute_avg, compute_p95, compute_max = timing_summary(
            self.timing_control_compute_ms
        )
        odom_gap_avg, odom_gap_p95, odom_gap_max = timing_summary(
            self.timing_odom_gaps_ms
        )
        odom_compute_avg, odom_compute_p95, odom_compute_max = (
            timing_summary(self.timing_odom_compute_ms)
        )
        stamp_avg, stamp_p95, stamp_max = timing_summary(
            self.timing_odom_stamp_age_ms
        )
        odom_cmd_avg, odom_cmd_p95, odom_cmd_max = timing_summary(
            self.timing_odom_to_cmd_ms
        )
        publish_avg, publish_p95, publish_max = timing_summary(
            self.timing_publish_ms
        )
        self.get_logger().info(
            'TIMING_FOLLOWER '
            f'period_ms={self.timing_control_period * 1000.0:.1f} '
            f'loop_gap_ms={loop_avg:.3f}/{loop_p95:.3f}/{loop_max:.3f} '
            f'compute_ms={compute_avg:.3f}/{compute_p95:.3f}/'
            f'{compute_max:.3f} '
            f'odom_gap_ms={odom_gap_avg:.3f}/{odom_gap_p95:.3f}/'
            f'{odom_gap_max:.3f} '
            f'odom_callback_ms={odom_compute_avg:.3f}/'
            f'{odom_compute_p95:.3f}/{odom_compute_max:.3f} '
            f'odom_stamp_age_ms={stamp_avg:.3f}/{stamp_p95:.3f}/'
            f'{stamp_max:.3f} '
            f'odom_to_cmd_ms={odom_cmd_avg:.3f}/{odom_cmd_p95:.3f}/'
            f'{odom_cmd_max:.3f} '
            f'ros_publish_ms={publish_avg:.3f}/{publish_p95:.3f}/'
            f'{publish_max:.3f} '
            f'deadline_miss={self.timing_deadline_misses} '
            f'cmd_count={self.timing_cmd_count} '
            f'stop_count={self.timing_stop_count} '
            f'state={self.state} feedback={self.feedback_mode}'
        )

        alert = (
            loop_max > self.timing_control_period * 1500.0
            or compute_max > self.timing_control_period * 500.0
            or odom_gap_max > self.odom_normal_age * 1000.0
        )
        if alert and now_mono - self.timing_last_alert_mono >= 1.0:
            self.timing_last_alert_mono = now_mono
            self.get_logger().warn(
                'TIMING_ALERT_FOLLOWER '
                f'loop_gap_max_ms={loop_max:.3f} '
                f'compute_max_ms={compute_max:.3f} '
                f'odom_gap_max_ms={odom_gap_max:.3f} '
                f'odom_stamp_age_max_ms={stamp_max:.3f} '
                f'odom_to_cmd_max_ms={odom_cmd_max:.3f} '
                f'state={self.state} feedback={self.feedback_mode}'
            )

        self.timing_loop_gaps_ms.clear()
        self.timing_control_compute_ms.clear()
        self.timing_odom_gaps_ms.clear()
        self.timing_odom_compute_ms.clear()
        self.timing_odom_stamp_age_ms.clear()
        self.timing_odom_to_cmd_ms.clear()
        self.timing_publish_ms.clear()
        self.timing_cmd_count = 0
        self.timing_stop_count = 0
        self.timing_deadline_misses = 0
        self.timing_last_report_mono = now_mono

    def destroy_node(self):
        try:
            self.publish_stop()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(
            'waypoint_follower stopped by Ctrl+C'
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

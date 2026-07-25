#!/usr/bin/env python3
"""Go2_2 enhanced nearest-point/lookahead route follower.

The control decisions and default parameters are kept from the Go2_2
implementation.  Timing and odometry diagnostics are observational only.
"""

import csv
import json
import math
import os
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from .go2_course_control import (
    MotionCourseEstimator,
    closest_route_projection,
    route_heading_at,
    straight_target_course,
    use_straight_course_feedback,
)


def normalize_angle(value):
    while value > math.pi:
        value -= 2.0 * math.pi
    while value < -math.pi:
        value += 2.0 * math.pi
    return value


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


def finite_or_none(value):
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


class WaypointFollowerGo22(Node):
    """Original Go2_2 nearest-index controller with diagnostics."""

    def __init__(self):
        super().__init__('waypoint_follower')

        self.declare_parameter('odom_topic', '/Odometry')
        self.declare_parameter('cmd_topic', '/patrol_cmd')
        self.declare_parameter(
            'route_file',
            '/home/unitree/go2_fastlio_ws/src/'
            'go2_fastlio_patrol/routes/route_demo.csv',
        )

        self.declare_parameter('v_base', 0.25)
        self.declare_parameter('max_vx', 0.25)
        self.declare_parameter('k_yaw', 0.6)
        self.declare_parameter('max_yaw_rate', 0.30)

        self.declare_parameter('lookahead_distance', 0.8)
        self.declare_parameter('reach_distance', 0.4)
        self.declare_parameter('goal_distance', 0.5)

        self.declare_parameter('loop_mode', 'once')
        self.declare_parameter('search_window', 6)

        self.declare_parameter('turn_in_place_angle', 1.0)
        self.declare_parameter('slow_down_angle', 0.5)

        self.declare_parameter('stuck_time', 3.0)
        self.declare_parameter('relocalize_distance', 1.0)
        self.declare_parameter('course_feedback_enabled', True)
        self.declare_parameter('course_feedback_deadband', 0.03)
        self.declare_parameter('course_feedback_minimum_motion', 0.12)
        self.declare_parameter('course_feedback_maximum_age', 0.80)
        self.declare_parameter('course_feedback_smoothing', 0.60)
        self.declare_parameter('course_feedback_max_route_turn_deg', 20.0)
        self.declare_parameter('course_feedback_max_course_deg', 22.0)
        self.declare_parameter('course_feedback_max_yaw_rate', 0.35)
        self.declare_parameter('trace_file', '')

        self.odom_topic = self.get_parameter('odom_topic').value
        self.cmd_topic = self.get_parameter('cmd_topic').value
        self.route_file = self.get_parameter('route_file').value
        self.trace_file = str(
            self.get_parameter('trace_file').value
        ).strip()

        self.v_base = float(self.get_parameter('v_base').value)
        self.max_vx = float(self.get_parameter('max_vx').value)
        self.k_yaw = float(self.get_parameter('k_yaw').value)
        self.max_yaw_rate = float(
            self.get_parameter('max_yaw_rate').value
        )

        self.lookahead_distance = float(
            self.get_parameter('lookahead_distance').value
        )
        self.reach_distance = float(
            self.get_parameter('reach_distance').value
        )
        self.goal_distance = float(
            self.get_parameter('goal_distance').value
        )

        self.loop_mode = self.get_parameter('loop_mode').value
        self.search_window = int(
            self.get_parameter('search_window').value
        )

        self.turn_in_place_angle = float(
            self.get_parameter('turn_in_place_angle').value
        )
        self.slow_down_angle = float(
            self.get_parameter('slow_down_angle').value
        )

        self.stuck_time = float(
            self.get_parameter('stuck_time').value
        )
        self.relocalize_distance = float(
            self.get_parameter('relocalize_distance').value
        )
        self.course_feedback_enabled = bool(
            self.get_parameter('course_feedback_enabled').value
        )
        self.course_feedback_max_course = math.radians(
            float(
                self.get_parameter(
                    'course_feedback_max_course_deg'
                ).value
            )
        )
        self.course_feedback_max_yaw_rate = float(
            self.get_parameter(
                'course_feedback_max_yaw_rate'
            ).value
        )
        self.course_feedback_deadband = float(
            self.get_parameter(
                'course_feedback_deadband'
            ).value
        )
        self.course_feedback_max_route_turn = math.radians(
            float(
                self.get_parameter(
                    'course_feedback_max_route_turn_deg'
                ).value
            )
        )
        self.course_estimator = MotionCourseEstimator(
            minimum_distance=float(
                self.get_parameter(
                    'course_feedback_minimum_motion'
                ).value
            ),
            smoothing=float(
                self.get_parameter(
                    'course_feedback_smoothing'
                ).value
            ),
            maximum_age=float(
                self.get_parameter(
                    'course_feedback_maximum_age'
                ).value
            ),
        )

        self.route = self.load_route(self.route_file)

        self.current_x = None
        self.current_y = None
        self.current_z = None
        self.current_yaw = None

        self.direction = 1
        self.nearest_index = 0
        self.target_index = 0
        self.finished = False
        self.initialized = False

        self.last_progress_index = 0
        self.last_progress_time = time.time()
        self.last_log_time = 0.0
        self.last_nearest_distance = float('inf')
        self.last_target_distance = float('inf')
        self.last_alpha = 0.0
        self.last_cmd_vx = 0.0
        self.last_cmd_yaw_rate = 0.0
        self.last_body_alpha = 0.0
        self.last_course_alpha = 0.0
        self.last_cross_track = 0.0
        self.last_route_turn = 0.0
        self.last_control_source = 'body'
        self.last_projection_segment = None
        self.last_projection_x = None
        self.last_projection_y = None
        self.last_projection_heading = None
        self.last_target_x = None
        self.last_target_y = None
        self.last_target_angle = None
        self.last_course_valid = False
        self.last_course_heading = None

        self.last_odom_receive_time = 0.0
        self.last_odom_receive_mono = 0.0
        self.last_odom_source_stamp = None
        self.odom_callback_sequence = 0
        self.control_sequence = 0
        self.control_loop_start_mono = 0.0
        self.last_odom_stamp_age = float('inf')
        self.last_motion_x = None
        self.last_motion_y = None
        self.last_motion_receive_time = None
        self.measured_speed = 0.0

        self.control_period = 0.05
        self.timing_last_control_mono = 0.0
        self.timing_last_odom_mono = 0.0
        self.timing_last_report_mono = time.monotonic()
        self.timing_last_alert_mono = 0.0
        self.timing_loop_gaps_ms = []
        self.timing_control_compute_ms = []
        self.timing_odom_gaps_ms = []
        self.timing_odom_callback_ms = []
        self.timing_odom_stamp_age_ms = []
        self.timing_odom_to_cmd_ms = []
        self.timing_publish_ms = []
        self.timing_cmd_count = 0
        self.timing_stop_count = 0
        self.timing_deadline_misses = 0
        self.trace_handle = None
        self.trace_write_error_reported = False
        self.open_trace()

        odom_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            odom_qos,
        )
        self.pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.timer = self.create_timer(
            self.control_period,
            self.control_loop,
        )

        self.get_logger().info(
            'GO2_2 enhanced waypoint_follower started'
        )
        self.get_logger().info(
            'odometry QoS: keep_last=1 reliability=best_effort'
        )
        self.get_logger().info(f'route_file: {self.route_file}')
        self.get_logger().info(f'route points: {len(self.route)}')
        self.get_logger().info(f'loop_mode: {self.loop_mode}')
        self.get_logger().info(
            f'lookahead_distance: {self.lookahead_distance}'
        )
        self.get_logger().info(
            f'reach_distance: {self.reach_distance}'
        )
        self.get_logger().info(
            'GO2_2 parameters: '
            f'v_base={self.v_base:.3f}, max_vx={self.max_vx:.3f}, '
            f'k_yaw={self.k_yaw:.3f}, '
            f'max_yaw_rate={self.max_yaw_rate:.3f}, '
            f'goal_distance={self.goal_distance:.3f}, '
            f'search_window={self.search_window}, '
            f'turn_in_place_angle={self.turn_in_place_angle:.3f}, '
            f'slow_down_angle={self.slow_down_angle:.3f}, '
            f'stuck_time={self.stuck_time:.3f}, '
            f'relocalize_distance={self.relocalize_distance:.3f}'
        )
        self.get_logger().info(
            'single-source straight feedback: '
            f'enabled={self.course_feedback_enabled}, '
            f'deadband={self.course_feedback_deadband:.3f}m, '
            f'minimum_motion={self.course_estimator.minimum_distance:.3f}m, '
            f'maximum_age={self.course_estimator.maximum_age:.3f}s, '
            f'max_route_turn='
            f'{math.degrees(self.course_feedback_max_route_turn):.1f}deg, '
            f'max_course='
            f'{math.degrees(self.course_feedback_max_course):.1f}deg, '
            f'max_yaw_rate={self.course_feedback_max_yaw_rate:.3f}rad/s'
        )
        if self.trace_handle is not None:
            self.get_logger().info(
                f'control trace enabled: {self.trace_file}'
            )

    def open_trace(self):
        if not self.trace_file:
            return
        try:
            parent = os.path.dirname(
                os.path.abspath(self.trace_file)
            )
            if parent:
                os.makedirs(parent, exist_ok=True)
            self.trace_handle = open(
                self.trace_file,
                'w',
                buffering=1,
            )
            self.write_trace_record(
                {
                    'kind': 'trace_start',
                    'wall_time': time.time(),
                    'monotonic_s': time.monotonic(),
                    'pid': os.getpid(),
                    'route_file': self.route_file,
                    'route_points': len(self.route),
                    'control_period_s': self.control_period,
                    'odom_qos': {
                        'history': 'keep_last',
                        'depth': 1,
                        'reliability': 'best_effort',
                    },
                }
            )
        except OSError as exc:
            self.trace_handle = None
            self.get_logger().error(
                f'cannot open control trace {self.trace_file}: {exc}'
            )

    def write_trace_record(self, record):
        if self.trace_handle is None:
            return
        try:
            self.trace_handle.write(
                json.dumps(
                    record,
                    sort_keys=True,
                    separators=(',', ':'),
                    allow_nan=False,
                )
                + '\n'
            )
        except (OSError, TypeError, ValueError) as exc:
            if not self.trace_write_error_reported:
                self.trace_write_error_reported = True
                self.get_logger().error(
                    f'control trace write failed: {exc}'
                )

    def write_control_trace(self, vx, yaw_rate, is_stop):
        target = (
            self.route[self.target_index]
            if self.route
            and 0 <= self.target_index < len(self.route)
            else {}
        )
        now_mono = time.monotonic()
        self.write_trace_record(
            {
                'kind': 'control',
                'wall_time': time.time(),
                'monotonic_s': now_mono,
                'control_sequence': self.control_sequence,
                'odom_callback_sequence': self.odom_callback_sequence,
                'odom_source_stamp': finite_or_none(
                    self.last_odom_source_stamp
                ),
                'odom_receive_monotonic_s': finite_or_none(
                    self.last_odom_receive_mono
                ),
                'odom_to_control_ms': (
                    finite_or_none(
                        (
                            now_mono
                            - self.last_odom_receive_mono
                        )
                        * 1000.0
                    )
                    if self.last_odom_receive_mono > 0.0
                    else None
                ),
                'odom_stamp_age_ms': (
                    finite_or_none(
                        self.last_odom_stamp_age * 1000.0
                    )
                ),
                'pose': {
                    'x': finite_or_none(self.current_x),
                    'y': finite_or_none(self.current_y),
                    'z': finite_or_none(self.current_z),
                    'yaw': finite_or_none(self.current_yaw),
                },
                'route': {
                    'nearest_index': self.nearest_index,
                    'target_index': self.target_index,
                    'direction': self.direction,
                    'nearest_distance_m': finite_or_none(
                        self.last_nearest_distance
                    ),
                    'target_distance_m': finite_or_none(
                        self.last_target_distance
                    ),
                    'target_x': finite_or_none(
                        target.get('x')
                    ),
                    'target_y': finite_or_none(
                        target.get('y')
                    ),
                    'target_yaw': finite_or_none(
                        target.get('yaw')
                    ),
                    'projection_segment': (
                        self.last_projection_segment
                    ),
                    'projection_x': finite_or_none(
                        self.last_projection_x
                    ),
                    'projection_y': finite_or_none(
                        self.last_projection_y
                    ),
                    'projection_heading': finite_or_none(
                        self.last_projection_heading
                    ),
                },
                'control': {
                    'source': self.last_control_source,
                    'cross_track_m': finite_or_none(
                        self.last_cross_track
                    ),
                    'target_angle': finite_or_none(
                        self.last_target_angle
                    ),
                    'body_alpha': finite_or_none(
                        self.last_body_alpha
                    ),
                    'course_alpha': finite_or_none(
                        self.last_course_alpha
                    ),
                    'selected_alpha': finite_or_none(
                        self.last_alpha
                    ),
                    'route_turn': finite_or_none(
                        self.last_route_turn
                    ),
                    'course_valid': self.last_course_valid,
                    'course_heading': finite_or_none(
                        self.last_course_heading
                    ),
                    'measured_speed_mps': finite_or_none(
                        self.measured_speed
                    ),
                    'finished': self.finished,
                    'is_stop': bool(is_stop),
                    'cmd_vx': finite_or_none(vx),
                    'cmd_vy': 0.0,
                    'cmd_yaw_rate': finite_or_none(yaw_rate),
                },
                'compute_to_trace_ms': (
                    finite_or_none(
                        (
                            now_mono
                            - self.control_loop_start_mono
                        )
                        * 1000.0
                    )
                    if self.control_loop_start_mono > 0.0
                    else None
                ),
            }
        )

    @staticmethod
    def load_route(path):
        route = []
        with open(path, 'r') as handle:
            reader = csv.DictReader(handle)
            for row in reader:
                route.append({
                    'x': float(row['x']),
                    'y': float(row['y']),
                    'yaw': float(row['yaw']),
                    'v': float(row['v']),
                })

        if len(route) < 2:
            raise RuntimeError(
                'route file must contain at least 2 points'
            )
        return route

    def odom_callback(self, msg):
        callback_start = time.monotonic()
        self.odom_callback_sequence += 1
        self.last_odom_receive_mono = callback_start
        if self.timing_last_odom_mono > 0.0:
            self.timing_odom_gaps_ms.append(
                (
                    callback_start - self.timing_last_odom_mono
                ) * 1000.0
            )
        self.timing_last_odom_mono = callback_start
        try:
            self.process_odometry(msg)
        finally:
            self.timing_odom_callback_ms.append(
                (time.monotonic() - callback_start) * 1000.0
            )
            if math.isfinite(self.last_odom_stamp_age):
                self.timing_odom_stamp_age_ms.append(
                    self.last_odom_stamp_age * 1000.0
                )

    def process_odometry(self, msg):
        now = time.time()
        current_x = float(msg.pose.pose.position.x)
        current_y = float(msg.pose.pose.position.y)
        current_z = float(msg.pose.pose.position.z)
        current_yaw = yaw_from_quaternion(msg.pose.pose.orientation)

        stamp_time = (
            float(msg.header.stamp.sec)
            + float(msg.header.stamp.nanosec) * 1e-9
        )
        self.last_odom_source_stamp = stamp_time
        if stamp_time <= 0.0:
            self.last_odom_stamp_age = 0.0
        elif stamp_time > now + 0.50:
            self.last_odom_stamp_age = float('inf')
        else:
            self.last_odom_stamp_age = max(0.0, now - stamp_time)

        if self.last_motion_receive_time is not None:
            dt = now - self.last_motion_receive_time
            if 0.001 < dt < 2.0:
                self.measured_speed = math.hypot(
                    current_x - self.last_motion_x,
                    current_y - self.last_motion_y,
                ) / dt
        self.last_motion_x = current_x
        self.last_motion_y = current_y
        self.last_motion_receive_time = now
        self.last_odom_receive_time = now
        self.course_estimator.update(current_x, current_y, now)

        self.current_x = current_x
        self.current_y = current_y
        self.current_z = current_z
        self.current_yaw = current_yaw

        if not self.initialized:
            self.nearest_index = self.find_nearest_global()
            self.target_index = self.nearest_index
            self.last_progress_index = self.nearest_index
            self.last_progress_time = now
            self.initialized = True

            distance_to_start = self.distance_to_index(0)
            self.get_logger().info(
                f'init nearest_index={self.nearest_index}, '
                f'distance_to_start={distance_to_start:.3f}'
            )

    def distance_to_index(self, index):
        point = self.route[index]
        return math.hypot(
            point['x'] - self.current_x,
            point['y'] - self.current_y,
        )

    def find_nearest_global(self):
        best_index = 0
        best_distance = 1e9
        for index, point in enumerate(self.route):
            distance = math.hypot(
                point['x'] - self.current_x,
                point['y'] - self.current_y,
            )
            if distance < best_distance:
                best_distance = distance
                best_index = index
        return best_index

    def find_nearest_window(self):
        route_size = len(self.route)
        start = max(0, self.nearest_index - self.search_window)
        end = min(
            route_size - 1,
            self.nearest_index + self.search_window,
        )

        best_index = self.nearest_index
        best_distance = 1e9
        for index in range(start, end + 1):
            point = self.route[index]
            distance = math.hypot(
                point['x'] - self.current_x,
                point['y'] - self.current_y,
            )
            if distance < best_distance:
                best_distance = distance
                best_index = index
        return best_index, best_distance

    def update_nearest_index(self):
        new_index, new_distance = self.find_nearest_window()

        if self.direction > 0:
            if new_index >= self.nearest_index:
                self.nearest_index = min(
                    new_index,
                    self.nearest_index + 1,
                )
        elif new_index <= self.nearest_index:
            self.nearest_index = max(
                new_index,
                self.nearest_index - 1,
            )

        if new_distance > self.relocalize_distance:
            global_index = self.find_nearest_global()
            self.get_logger().warn(
                f'far from local route: d={new_distance:.2f}, '
                f'relocalize nearest_index '
                f'{self.nearest_index} -> {global_index}'
            )
            self.nearest_index = global_index

        self.last_nearest_distance = self.distance_to_index(
            self.nearest_index
        )
        if self.nearest_index != self.last_progress_index:
            self.last_progress_index = self.nearest_index
            self.last_progress_time = time.time()

    def compute_lookahead_index(self):
        route_size = len(self.route)
        index = self.nearest_index
        accumulated = 0.0

        while True:
            next_index = index + self.direction
            if next_index < 0 or next_index >= route_size:
                return index

            point = self.route[index]
            next_point = self.route[next_index]
            accumulated += math.hypot(
                next_point['x'] - point['x'],
                next_point['y'] - point['y'],
            )
            index = next_index
            if accumulated >= self.lookahead_distance:
                return index

    def publish_command(self, vx, yaw_rate, is_stop=False):
        command = Twist()
        command.linear.x = float(vx)
        command.linear.y = 0.0
        command.angular.z = float(yaw_rate)

        publish_start = time.monotonic()
        self.pub.publish(command)
        self.timing_publish_ms.append(
            (time.monotonic() - publish_start) * 1000.0
        )
        self.timing_cmd_count += 1
        if is_stop:
            self.timing_stop_count += 1
        if self.last_odom_receive_time > 0.0:
            self.timing_odom_to_cmd_ms.append(
                max(
                    0.0,
                    time.time() - self.last_odom_receive_time,
                ) * 1000.0
            )

        self.last_cmd_vx = float(vx)
        self.last_cmd_yaw_rate = float(yaw_rate)
        self.write_control_trace(vx, yaw_rate, is_stop)

    def publish_stop(self):
        self.publish_command(0.0, 0.0, is_stop=True)

    def handle_goal(self):
        route_size = len(self.route)
        if self.direction > 0:
            goal_index = route_size - 1
            if self.nearest_index < max(1, route_size - 2):
                return False
        else:
            goal_index = 0
            if self.nearest_index > min(route_size - 2, 1):
                return False

        distance_to_goal = self.distance_to_index(goal_index)
        if distance_to_goal > self.goal_distance:
            return False

        if self.loop_mode == 'pingpong':
            if self.direction > 0:
                self.direction = -1
                self.nearest_index = route_size - 1
                self.get_logger().info(
                    'reach end, switch to backward'
                )
            else:
                self.direction = 1
                self.nearest_index = 0
                self.get_logger().info(
                    'reach start, switch to forward'
                )

            self.last_progress_index = self.nearest_index
            self.last_progress_time = time.time()
            return False

        self.get_logger().info('goal reached, stop')
        self.finished = True
        self.publish_stop()
        return True

    def control_loop(self):
        loop_start = time.monotonic()
        self.control_sequence += 1
        self.control_loop_start_mono = loop_start
        if self.timing_last_control_mono > 0.0:
            loop_gap_ms = (
                loop_start - self.timing_last_control_mono
            ) * 1000.0
            self.timing_loop_gaps_ms.append(loop_gap_ms)
            if loop_gap_ms > self.control_period * 1500.0:
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
        if self.current_x is None:
            self.publish_stop()
            return
        if self.finished:
            self.publish_stop()
            return

        self.update_nearest_index()
        if self.handle_goal():
            return

        self.target_index = self.compute_lookahead_index()
        target = self.route[self.target_index]
        delta_x = target['x'] - self.current_x
        delta_y = target['y'] - self.current_y
        distance_to_target = math.hypot(delta_x, delta_y)
        target_angle = math.atan2(delta_y, delta_x)
        body_alpha = normalize_angle(target_angle - self.current_yaw)

        projection = closest_route_projection(
            self.route,
            self.current_x,
            self.current_y,
            self.nearest_index,
            self.search_window,
            self.direction,
        )
        target_route_heading = route_heading_at(
            self.route,
            self.target_index,
            self.direction,
        )
        route_turn = normalize_angle(
            target_route_heading - projection.route_heading
        )
        course_valid = self.course_estimator.valid(time.time())
        straight_course_active = use_straight_course_feedback(
            self.course_feedback_enabled,
            course_valid,
            route_turn,
            self.course_feedback_max_route_turn,
            body_alpha,
            self.turn_in_place_angle,
        )

        course_alpha = body_alpha
        control_source = 'body_fallback'
        if straight_course_active:
            desired_course = straight_target_course(
                projection.route_heading,
                target_angle,
                projection.signed_distance,
                self.course_feedback_deadband,
                self.course_feedback_max_course,
            )
            course_alpha = normalize_angle(
                desired_course - self.course_estimator.course
            )
            alpha = course_alpha
            control_source = 'course_straight'
        else:
            alpha = body_alpha
            if (
                abs(route_turn)
                > self.course_feedback_max_route_turn
                or abs(body_alpha) > self.turn_in_place_angle
            ):
                control_source = 'body_corner'

        yaw_rate = self.k_yaw * alpha
        yaw_rate_limit = self.max_yaw_rate
        if control_source == 'course_straight':
            yaw_rate_limit = min(
                yaw_rate_limit,
                self.course_feedback_max_yaw_rate,
            )
        yaw_rate = max(
            -yaw_rate_limit,
            min(yaw_rate_limit, yaw_rate),
        )

        if control_source == 'course_straight':
            speed_alpha = abs(alpha)
        else:
            speed_alpha = abs(body_alpha)
        if abs(body_alpha) > self.turn_in_place_angle:
            velocity_x = 0.0
        elif speed_alpha > self.slow_down_angle:
            velocity_x = min(self.v_base * 0.5, self.max_vx)
        else:
            velocity_x = min(self.v_base, self.max_vx)

        now = time.time()
        if now - self.last_progress_time > self.stuck_time:
            old_index = self.nearest_index
            self.nearest_index = self.find_nearest_global()
            self.last_progress_index = self.nearest_index
            self.last_progress_time = now
            self.get_logger().warn(
                f'stuck recovery: nearest_index '
                f'{old_index} -> {self.nearest_index}'
            )

        self.last_target_distance = distance_to_target
        self.last_alpha = alpha
        self.last_body_alpha = body_alpha
        self.last_course_alpha = course_alpha
        self.last_cross_track = projection.signed_distance
        self.last_route_turn = route_turn
        self.last_control_source = control_source
        self.last_projection_segment = projection.segment_index
        self.last_projection_x = projection.x
        self.last_projection_y = projection.y
        self.last_projection_heading = projection.route_heading
        self.last_target_x = target['x']
        self.last_target_y = target['y']
        self.last_target_angle = target_angle
        self.last_course_valid = course_valid
        self.last_course_heading = (
            self.course_estimator.course
            if course_valid
            else None
        )
        self.publish_command(velocity_x, yaw_rate)

        if now - self.last_log_time > 1.0:
            self.last_log_time = now
            course_heading_log = (
                self.course_estimator.course
                if course_valid
                else float('nan')
            )
            self.get_logger().info(
                f'GO2_2 nearest={self.nearest_index}, '
                f'target={self.target_index}, '
                f'nearest_distance={self.last_nearest_distance:.3f}m, '
                f'target_distance={distance_to_target:.3f}m, '
                f'pose=({self.current_x:.3f},{self.current_y:.3f},'
                f'{self.current_yaw:.3f}), '
                f'alpha={alpha:.3f}, body_alpha={body_alpha:.3f}, '
                f'course_alpha={course_alpha:.3f}, '
                f'course_heading={course_heading_log:.3f}, '
                f'cross_track={projection.signed_distance:.3f}m, '
                f'route_turn={route_turn:.3f}, '
                f'control_source={control_source}, '
                f'cmd=({velocity_x:.3f},0.000,{yaw_rate:.3f}), '
                f'measured_speed={self.measured_speed:.3f}m/s, '
                f'odom_stamp_age={self.last_odom_stamp_age:.3f}s, '
                f'dir={self.direction}'
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
        odom_cb_avg, odom_cb_p95, odom_cb_max = timing_summary(
            self.timing_odom_callback_ms
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
            'TIMING_FOLLOWER_GO2_2 '
            f'period_ms={self.control_period * 1000.0:.1f} '
            f'loop_gap_ms={loop_avg:.3f}/{loop_p95:.3f}/'
            f'{loop_max:.3f} '
            f'compute_ms={compute_avg:.3f}/{compute_p95:.3f}/'
            f'{compute_max:.3f} '
            f'odom_gap_ms={odom_gap_avg:.3f}/{odom_gap_p95:.3f}/'
            f'{odom_gap_max:.3f} '
            f'odom_callback_ms={odom_cb_avg:.3f}/{odom_cb_p95:.3f}/'
            f'{odom_cb_max:.3f} '
            f'odom_stamp_age_ms={stamp_avg:.3f}/{stamp_p95:.3f}/'
            f'{stamp_max:.3f} '
            f'odom_to_cmd_ms={odom_cmd_avg:.3f}/{odom_cmd_p95:.3f}/'
            f'{odom_cmd_max:.3f} '
            f'ros_publish_ms={publish_avg:.3f}/{publish_p95:.3f}/'
            f'{publish_max:.3f} '
            f'deadline_miss={self.timing_deadline_misses} '
            f'cmd_count={self.timing_cmd_count} '
            f'stop_count={self.timing_stop_count} '
            f'nearest={self.nearest_index} target={self.target_index}'
        )

        alert = (
            loop_max > self.control_period * 1500.0
            or compute_max > self.control_period * 500.0
            or odom_gap_max > 600.0
        )
        if alert and now_mono - self.timing_last_alert_mono >= 1.0:
            self.timing_last_alert_mono = now_mono
            self.get_logger().warn(
                'TIMING_ALERT_FOLLOWER_GO2_2 '
                f'loop_gap_max_ms={loop_max:.3f} '
                f'compute_max_ms={compute_max:.3f} '
                f'odom_gap_max_ms={odom_gap_max:.3f} '
                f'odom_stamp_age_max_ms={stamp_max:.3f} '
                f'odom_to_cmd_max_ms={odom_cmd_max:.3f}'
            )

        self.timing_loop_gaps_ms.clear()
        self.timing_control_compute_ms.clear()
        self.timing_odom_gaps_ms.clear()
        self.timing_odom_callback_ms.clear()
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
        if self.trace_handle is not None:
            try:
                self.write_trace_record(
                    {
                        'kind': 'trace_stop',
                        'wall_time': time.time(),
                        'monotonic_s': time.monotonic(),
                        'control_sequence': self.control_sequence,
                        'odom_callback_sequence': (
                            self.odom_callback_sequence
                        ),
                    }
                )
                self.trace_handle.flush()
                os.fsync(self.trace_handle.fileno())
                self.trace_handle.close()
            except OSError:
                pass
            self.trace_handle = None
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollowerGo22()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(
            'GO2_2 waypoint_follower stopped by Ctrl+C'
        )
        node.publish_stop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

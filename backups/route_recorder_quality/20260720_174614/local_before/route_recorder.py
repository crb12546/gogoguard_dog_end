#!/usr/bin/env python3
import math
import os
import csv
import signal
import shutil
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def angle_difference(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


def route_point(row):
    try:
        x = float(row[1])
        y = float(row[2])
        yaw = float(row[3])
    except (IndexError, TypeError, ValueError):
        return None

    if not all(math.isfinite(value) for value in (x, y, yaw)):
        return None
    return x, y, yaw


def find_isolated_spikes(
    rows,
    min_turn_angle_deg,
    min_lateral_deviation,
    max_rejoin_heading_mismatch_deg,
):
    spike_indexes = []
    for index in range(1, len(rows) - 1):
        previous = route_point(rows[index - 1])
        current = route_point(rows[index])
        following = route_point(rows[index + 1])
        if previous is None or current is None or following is None:
            continue

        previous_heading = math.atan2(
            current[1] - previous[1],
            current[0] - previous[0],
        )
        following_heading = math.atan2(
            following[1] - current[1],
            following[0] - current[0],
        )
        direct_x = following[0] - previous[0]
        direct_y = following[1] - previous[1]
        direct_length = math.hypot(direct_x, direct_y)
        if direct_length == 0.0:
            continue

        turn_angle_deg = math.degrees(
            abs(angle_difference(following_heading, previous_heading))
        )
        lateral_deviation = abs(
            (current[0] - previous[0]) * direct_y
            - (current[1] - previous[1]) * direct_x
        ) / direct_length
        direct_heading = math.atan2(direct_y, direct_x)
        rejoin_heading_mismatch_deg = math.degrees(
            abs(angle_difference(direct_heading, following[2]))
        )

        if (
            turn_angle_deg >= min_turn_angle_deg
            and lateral_deviation >= min_lateral_deviation
            and rejoin_heading_mismatch_deg <= max_rejoin_heading_mismatch_deg
        ):
            spike_indexes.append(index)
    return spike_indexes


def sanitize_route_file(
    route_file,
    min_turn_angle_deg,
    min_lateral_deviation,
    max_rejoin_heading_mismatch_deg,
):
    with open(route_file, newline='') as file:
        rows = list(csv.reader(file))

    if not rows or rows[0] != ['id', 'x', 'y', 'yaw', 'v']:
        return [], None

    spike_indexes = find_isolated_spikes(
        rows[1:],
        min_turn_angle_deg,
        min_lateral_deviation,
        max_rejoin_heading_mismatch_deg,
    )
    if not spike_indexes:
        return [], None

    dropped_ids = [rows[index + 1][0] for index in spike_indexes]
    spike_indexes = set(index + 1 for index in spike_indexes)
    temporary_file = f'{route_file}.sanitizing'
    backup_file = (
        f'{route_file}.before_sanitize_'
        f'{time.strftime("%Y%m%d_%H%M%S")}.bak'
    )
    shutil.copy2(route_file, backup_file)
    with open(temporary_file, 'w', newline='') as file:
        writer = csv.writer(file)
        for index, row in enumerate(rows):
            if index not in spike_indexes:
                writer.writerow(row)
    os.replace(temporary_file, route_file)
    return dropped_ids, backup_file


class RouteRecorder(Node):
    def __init__(self):
        super().__init__('route_recorder')

        self.declare_parameter('odom_topic', '/Odometry')
        self.declare_parameter('route_file', '/home/unitree/go2_fastlio_ws/src/go2_fastlio_patrol/routes/route_demo.csv')
        self.declare_parameter('min_distance', 0.4)
        self.declare_parameter('default_speed', 0.20)
        self.declare_parameter('warmup_seconds', 4.0)
        self.declare_parameter('min_stable_samples', 5)
        self.declare_parameter('max_odom_step', 1.0)
        self.declare_parameter('max_record_step', 0.0)
        self.declare_parameter('max_heading_mismatch_deg', 0.0)
        self.declare_parameter('sanitize_on_close', False)
        self.declare_parameter('spike_turn_angle_deg', 90.0)
        self.declare_parameter('spike_lateral_deviation', 0.20)
        self.declare_parameter('spike_rejoin_heading_mismatch_deg', 30.0)

        self.odom_topic = self.get_parameter('odom_topic').value
        self.route_file = self.get_parameter('route_file').value
        self.min_distance = float(self.get_parameter('min_distance').value)
        self.default_speed = float(self.get_parameter('default_speed').value)
        self.warmup_seconds = max(0.0, float(self.get_parameter('warmup_seconds').value))
        self.min_stable_samples = max(1, int(self.get_parameter('min_stable_samples').value))
        self.max_odom_step = max(0.0, float(self.get_parameter('max_odom_step').value))
        self.max_record_step = max(0.0, float(self.get_parameter('max_record_step').value))
        self.max_heading_mismatch_deg = max(
            0.0,
            float(self.get_parameter('max_heading_mismatch_deg').value),
        )
        self.sanitize_on_close = bool(
            self.get_parameter('sanitize_on_close').value
        )
        self.spike_turn_angle_deg = max(
            0.0,
            float(self.get_parameter('spike_turn_angle_deg').value),
        )
        self.spike_lateral_deviation = max(
            0.0,
            float(self.get_parameter('spike_lateral_deviation').value),
        )
        self.spike_rejoin_heading_mismatch_deg = max(
            0.0,
            float(
                self.get_parameter('spike_rejoin_heading_mismatch_deg').value
            ),
        )

        os.makedirs(os.path.dirname(self.route_file), exist_ok=True)

        self.file = open(self.route_file, 'w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow(['id', 'x', 'y', 'yaw', 'v'])
        self.file.flush()

        self.point_id = 0
        self.last_x = None
        self.last_y = None
        self.last_observed_x = None
        self.last_observed_y = None
        self.stable_samples = 0
        self.started_at = time.monotonic()
        self.recording_ready = False
        self.last_rejection_reason = None
        self.last_rejection_log_at = 0.0
        self.route_finalized = False

        self.sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10
        )

        self.get_logger().info(f'route_recorder started')
        self.get_logger().info(f'odom_topic: {self.odom_topic}')
        self.get_logger().info(f'route_file: {self.route_file}')
        self.get_logger().info(f'min_distance: {self.min_distance}')
        self.get_logger().info(
            'quality gate: '
            f'warmup={self.warmup_seconds:.1f}s, '
            f'stable_samples={self.min_stable_samples}, '
            f'max_odom_step={self.max_odom_step:.2f}m, '
            f'max_record_step={self.max_record_step:.2f}m (0=disabled), '
            f'global_heading_gate={self.max_heading_mismatch_deg:.1f}deg'
        )
        self.get_logger().info(
            'close sanitizer: '
            f'enabled={self.sanitize_on_close} (disabled by default), '
            f'turn={self.spike_turn_angle_deg:.1f}deg, '
            f'lateral={self.spike_lateral_deviation:.2f}m, '
            f'rejoin_heading={self.spike_rejoin_heading_mismatch_deg:.1f}deg'
        )

    def log_rejection(self, reason):
        now = time.monotonic()
        if (
            reason != self.last_rejection_reason
            or now - self.last_rejection_log_at >= 2.0
        ):
            self.get_logger().warn(reason)
            self.last_rejection_reason = reason
            self.last_rejection_log_at = now

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)

        if not all(math.isfinite(value) for value in (x, y, yaw)):
            self.stable_samples = 0
            self.last_observed_x = None
            self.last_observed_y = None
            self.log_rejection('ignored non-finite odometry sample')
            return

        observation_step = 0.0
        observation_stable = True
        if self.last_observed_x is not None:
            observation_step = math.hypot(
                x - self.last_observed_x,
                y - self.last_observed_y,
            )
            if self.max_odom_step > 0.0 and observation_step > self.max_odom_step:
                observation_stable = False
                self.log_rejection(
                    'ignored odometry jump: '
                    f'step={observation_step:.3f}m exceeds '
                    f'max_odom_step={self.max_odom_step:.3f}m'
                )

        self.last_observed_x = x
        self.last_observed_y = y
        self.stable_samples = self.stable_samples + 1 if observation_stable else 1

        if time.monotonic() - self.started_at < self.warmup_seconds:
            return

        if self.stable_samples < self.min_stable_samples:
            return

        if not self.recording_ready:
            self.recording_ready = True
            self.get_logger().info(
                'quality gate ready; recording route points from the current pose'
            )

        if self.last_x is not None:
            distance = math.hypot(x - self.last_x, y - self.last_y)
            if distance < self.min_distance:
                return

            if self.max_record_step > 0.0 and distance > self.max_record_step:
                self.log_rejection(
                    'ignored route discontinuity: '
                    f'step={distance:.3f}m exceeds '
                    f'max_record_step={self.max_record_step:.3f}m'
                )
                return

            if self.max_heading_mismatch_deg > 0.0:
                segment_heading = math.atan2(y - self.last_y, x - self.last_x)
                heading_mismatch_deg = math.degrees(
                    abs(angle_difference(segment_heading, yaw))
                )
                if heading_mismatch_deg > self.max_heading_mismatch_deg:
                    self.log_rejection(
                        'ignored route point with heading mismatch: '
                        f'mismatch={heading_mismatch_deg:.1f}deg exceeds '
                        f'max_heading_mismatch_deg={self.max_heading_mismatch_deg:.1f}'
                    )
                    return

        self.writer.writerow([
            self.point_id,
            f'{x:.6f}',
            f'{y:.6f}',
            f'{yaw:.6f}',
            f'{self.default_speed:.3f}'
        ])
        self.file.flush()

        self.get_logger().info(
            f'saved point {self.point_id}: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}'
        )

        self.point_id += 1
        self.last_x = x
        self.last_y = y

    def finalize_route_file(self):
        if self.route_finalized:
            return
        self.route_finalized = True

        try:
            if not self.file.closed:
                self.file.flush()
                self.file.close()
        except Exception:
            pass

        if not self.sanitize_on_close:
            return
        try:
            dropped_ids, backup_file = sanitize_route_file(
                self.route_file,
                self.spike_turn_angle_deg,
                self.spike_lateral_deviation,
                self.spike_rejoin_heading_mismatch_deg,
            )
        except Exception as exc:
            self.get_logger().error(f'route sanitizer failed: {exc}')
            return

        if dropped_ids:
            preview = ', '.join(dropped_ids[:8])
            suffix = '...' if len(dropped_ids) > 8 else ''
            self.get_logger().warn(
                f'route sanitizer removed {len(dropped_ids)} isolated spike(s): '
                f'{preview}{suffix}'
            )
            self.get_logger().warn(
                f'original route backup retained at: {backup_file}'
            )

    def destroy_node(self):
        self.finalize_route_file()
        super().destroy_node()


def handle_shutdown_signal(_signum, _frame):
    raise KeyboardInterrupt


def main(args=None):
    rclpy.init(args=args)
    node = RouteRecorder()
    previous_sigterm_handler = signal.getsignal(signal.SIGTERM)
    signal.signal(signal.SIGTERM, handle_shutdown_signal)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('route_recorder stopped')
    finally:
        node.destroy_node()
        signal.signal(signal.SIGTERM, previous_sigterm_handler)
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

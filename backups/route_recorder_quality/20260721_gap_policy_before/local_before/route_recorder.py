#!/usr/bin/env python3
import math
import os
import csv
import json
import signal
import shutil
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from nav_msgs.msg import Odometry

from .route_quality import (
    AdaptiveRouteSampler,
    OdometryQualityGate,
    RouteSample,
    build_clean_route,
    validate_route_report,
)


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
        self.declare_parameter('min_distance', 0.20)
        self.declare_parameter('turn_distance', 0.10)
        self.declare_parameter('turn_enter_deg', 5.0)
        self.declare_parameter('turn_confirm_samples', 2)
        self.declare_parameter('turn_anchor_deg', 2.0)
        self.declare_parameter('turn_exit_deg', 2.0)
        self.declare_parameter('turn_exit_distance', 0.25)
        self.declare_parameter('turn_zone_distance', 0.30)
        self.declare_parameter('simplify_tolerance', 0.03)
        self.declare_parameter('spike_lateral_threshold', 0.06)
        self.declare_parameter('default_speed', 0.20)
        self.declare_parameter('warmup_seconds', 4.0)
        self.declare_parameter('min_stable_samples', 5)
        self.declare_parameter('max_odom_step', 0.30)
        self.declare_parameter('max_odom_speed', 4.0)
        self.declare_parameter('odom_step_margin', 0.10)
        self.declare_parameter('hard_odom_speed', 6.0)
        self.declare_parameter('hard_odom_step', 3.0)
        self.declare_parameter('max_odom_gap_seconds', 1.0)
        self.declare_parameter('odom_recovery_samples', 3)
        self.declare_parameter('odom_failure_samples', 3)
        self.declare_parameter('odom_failure_seconds', 0.50)
        self.declare_parameter('max_record_step', 2.50)
        self.declare_parameter('min_route_points', 5)
        self.declare_parameter('min_route_length', 1.0)
        self.declare_parameter('max_rejection_ratio', 0.10)
        self.declare_parameter('max_heading_mismatch_deg', 0.0)
        self.declare_parameter('sanitize_on_close', False)
        self.declare_parameter('spike_turn_angle_deg', 90.0)
        self.declare_parameter('spike_lateral_deviation', 0.20)
        self.declare_parameter('spike_rejoin_heading_mismatch_deg', 30.0)

        self.odom_topic = self.get_parameter('odom_topic').value
        self.route_file = self.get_parameter('route_file').value
        self.min_distance = float(self.get_parameter('min_distance').value)
        self.turn_distance = float(self.get_parameter('turn_distance').value)
        self.turn_enter_deg = float(self.get_parameter('turn_enter_deg').value)
        self.turn_confirm_samples = max(
            1,
            int(self.get_parameter('turn_confirm_samples').value),
        )
        self.turn_anchor_deg = float(self.get_parameter('turn_anchor_deg').value)
        self.turn_exit_deg = float(self.get_parameter('turn_exit_deg').value)
        self.turn_exit_distance = float(
            self.get_parameter('turn_exit_distance').value
        )
        self.turn_zone_distance = float(
            self.get_parameter('turn_zone_distance').value
        )
        self.simplify_tolerance = float(
            self.get_parameter('simplify_tolerance').value
        )
        self.spike_lateral_threshold = float(
            self.get_parameter('spike_lateral_threshold').value
        )
        self.default_speed = float(self.get_parameter('default_speed').value)
        self.warmup_seconds = max(0.0, float(self.get_parameter('warmup_seconds').value))
        self.min_stable_samples = max(1, int(self.get_parameter('min_stable_samples').value))
        self.max_odom_step = max(0.0, float(self.get_parameter('max_odom_step').value))
        self.max_odom_speed = max(
            0.01,
            float(self.get_parameter('max_odom_speed').value),
        )
        self.odom_step_margin = max(
            0.0,
            float(self.get_parameter('odom_step_margin').value),
        )
        self.hard_odom_speed = max(
            self.max_odom_speed,
            float(self.get_parameter('hard_odom_speed').value),
        )
        self.hard_odom_step = max(
            self.max_odom_step,
            float(self.get_parameter('hard_odom_step').value),
        )
        self.max_odom_gap_seconds = max(
            0.01,
            float(self.get_parameter('max_odom_gap_seconds').value),
        )
        self.odom_recovery_samples = max(
            1,
            int(self.get_parameter('odom_recovery_samples').value),
        )
        self.odom_failure_samples = max(
            1,
            int(self.get_parameter('odom_failure_samples').value),
        )
        self.odom_failure_seconds = max(
            0.0,
            float(self.get_parameter('odom_failure_seconds').value),
        )
        self.max_record_step = max(0.0, float(self.get_parameter('max_record_step').value))
        self.min_route_points = max(
            2,
            int(self.get_parameter('min_route_points').value),
        )
        self.min_route_length = max(
            0.0,
            float(self.get_parameter('min_route_length').value),
        )
        self.max_rejection_ratio = min(
            1.0,
            max(0.0, float(self.get_parameter('max_rejection_ratio').value)),
        )
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

        route_directory = os.path.dirname(self.route_file) or '.'
        route_basename = os.path.splitext(os.path.basename(self.route_file))[0]
        self.raw_file = os.path.join(
            route_directory,
            'raw',
            f'{route_basename}.raw.csv',
        )
        self.quality_report_file = os.path.join(
            route_directory,
            'quality',
            f'{route_basename}.quality.json',
        )
        os.makedirs(route_directory, exist_ok=True)
        os.makedirs(os.path.dirname(self.raw_file), exist_ok=True)
        os.makedirs(os.path.dirname(self.quality_report_file), exist_ok=True)

        self.file = open(self.route_file, 'w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow(['id', 'x', 'y', 'yaw', 'v'])
        self.file.flush()

        self.raw_handle = open(self.raw_file, 'w', newline='')
        self.raw_writer = csv.writer(self.raw_handle)
        self.raw_writer.writerow(['id', 'x', 'y', 'yaw', 'v'])
        self.raw_handle.flush()

        self.point_id = 0
        self.raw_point_id = 0
        self.raw_samples = []
        self.last_x = None
        self.last_y = None
        self.stable_samples = 0
        self.total_observations = 0
        self.accepted_observations = 0
        self.rejected_observations = 0
        self.max_observed_step = 0.0
        self.max_observed_speed = 0.0
        self.runtime_failure = ''
        self.stop_requested = False
        self.route_valid = False
        self.started_at = time.monotonic()
        self.recording_ready = False
        self.last_rejection_reason = None
        self.last_rejection_log_at = 0.0
        self.route_finalized = False
        self.sampler = AdaptiveRouteSampler(
            normal_spacing=self.min_distance,
            turn_spacing=self.turn_distance,
            turn_enter_deg=self.turn_enter_deg,
            turn_confirm_samples=self.turn_confirm_samples,
            turn_anchor_deg=self.turn_anchor_deg,
            turn_exit_deg=self.turn_exit_deg,
            turn_exit_distance=self.turn_exit_distance,
        )
        self.odom_gate = OdometryQualityGate(
            base_step_m=self.max_odom_step,
            max_speed_mps=self.max_odom_speed,
            step_margin_m=self.odom_step_margin,
            hard_speed_mps=self.hard_odom_speed,
            hard_step_m=self.hard_odom_step,
            max_gap_s=self.max_odom_gap_seconds,
            recovery_samples=self.odom_recovery_samples,
            failure_samples=self.odom_failure_samples,
            failure_duration_s=self.odom_failure_seconds,
        )

        self.sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10
        )

        self.get_logger().info('route_recorder started')
        self.get_logger().info(f'odom_topic: {self.odom_topic}')
        self.get_logger().info(f'route_file: {self.route_file}')
        self.get_logger().info(f'raw_file: {self.raw_file}')
        self.get_logger().info(
            'adaptive sampling: '
            f'straight={self.min_distance:.2f}m, '
            f'turn={self.turn_distance:.2f}m, '
            f'enter={self.turn_enter_deg:.1f}deg/'
            f'{self.turn_confirm_samples} samples, '
            f'exit={self.turn_exit_deg:.1f}deg over '
            f'{self.turn_exit_distance:.2f}m'
        )
        self.get_logger().info(
            'final route cleanup: '
            f'xy_tolerance={self.simplify_tolerance:.3f}m, '
            f'spike_lateral={self.spike_lateral_threshold:.3f}m, '
            f'turn_zone={self.turn_zone_distance:.2f}m'
        )
        self.get_logger().info(
            'quality gate: '
            f'warmup={self.warmup_seconds:.1f}s, '
            f'stable_samples={self.min_stable_samples}, '
            f'base_step={self.max_odom_step:.2f}m, '
            f'max_speed={self.max_odom_speed:.2f}m/s, '
            f'step_margin={self.odom_step_margin:.2f}m, '
            f'hard_speed={self.hard_odom_speed:.2f}m/s, '
            f'failure_samples={self.odom_failure_samples}, '
            f'max_record_step={self.max_record_step:.2f}m (0=disabled), '
            f'global_heading_gate={self.max_heading_mismatch_deg:.1f}deg'
        )
        self.get_logger().info(
            'final quality validation: '
            f'min_points={self.min_route_points}, '
            f'min_length={self.min_route_length:.2f}m, '
            f'max_rejection_ratio={self.max_rejection_ratio:.1%}, '
            f'max_source_gap={self.max_record_step:.2f}m'
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

    def write_raw_sample(self, sample):
        self.raw_writer.writerow([
            self.raw_point_id,
            f'{sample.x:.6f}',
            f'{sample.y:.6f}',
            f'{sample.yaw:.6f}',
            f'{self.default_speed:.3f}',
        ])
        self.raw_handle.flush()
        self.raw_samples.append(sample)
        self.raw_point_id += 1

    def write_route_sample(self, sample):
        if self.last_x is not None:
            distance = math.hypot(sample.x - self.last_x, sample.y - self.last_y)
            if self.max_record_step > 0.0 and distance > self.max_record_step:
                self.fail_recording(
                    'live route discontinuity %.3fm exceeds %.3fm'
                    % (distance, self.max_record_step)
                )
                return

            if self.max_heading_mismatch_deg > 0.0:
                segment_heading = math.atan2(
                    sample.y - self.last_y,
                    sample.x - self.last_x,
                )
                heading_mismatch_deg = math.degrees(
                    abs(angle_difference(segment_heading, sample.yaw))
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
            f'{sample.x:.6f}',
            f'{sample.y:.6f}',
            f'{sample.yaw:.6f}',
            f'{self.default_speed:.3f}',
        ])
        self.file.flush()
        self.get_logger().info(
            f'saved point {self.point_id}: '
            f'x={sample.x:.3f}, y={sample.y:.3f}, yaw={sample.yaw:.3f}, '
            f'mode={self.sampler.mode}'
        )
        self.point_id += 1
        self.last_x = sample.x
        self.last_y = sample.y

    def fail_recording(self, reason):
        if self.runtime_failure:
            return
        self.runtime_failure = str(reason)
        self.stop_requested = True
        self.get_logger().error(
            f'RECORDING_FAILED: {self.runtime_failure}; '
            'stopping recorder and retaining diagnostics'
        )

    @staticmethod
    def odometry_sample_time(msg):
        stamp = msg.header.stamp
        sample_time = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        if sample_time > 0.0 and math.isfinite(sample_time):
            return sample_time
        return time.monotonic()

    def odom_callback(self, msg):
        elapsed_since_start = time.monotonic() - self.started_at
        quality_window = elapsed_since_start >= self.warmup_seconds
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        if not math.isfinite(yaw):
            x = float('nan')
        decision = self.odom_gate.evaluate(
            x,
            y,
            z,
            self.odometry_sample_time(msg),
        )
        if quality_window:
            self.total_observations += 1
        if math.isfinite(decision.step_m):
            self.max_observed_step = max(
                self.max_observed_step,
                decision.step_m,
            )
        if math.isfinite(decision.speed_mps):
            self.max_observed_speed = max(
                self.max_observed_speed,
                decision.speed_mps,
            )
        if not decision.accepted:
            if quality_window:
                self.rejected_observations += 1
            self.stable_samples = 0
            self.log_rejection(decision.reason)
            if decision.should_fail:
                self.fail_recording(
                    '%s; consecutive_anomalies=%d'
                    % (decision.reason, decision.consecutive_anomalies)
                )
            return

        if quality_window:
            self.accepted_observations += 1
        if decision.recovered:
            self.get_logger().info(decision.reason)
        self.stable_samples += 1

        if not quality_window:
            return

        if self.stable_samples < self.min_stable_samples:
            return

        if not self.recording_ready:
            self.recording_ready = True
            self.get_logger().info(
                'quality gate ready; recording route points from the current pose'
            )

        sample = RouteSample(x, y, yaw)
        self.write_raw_sample(sample)
        for selected_sample in self.sampler.add(sample):
            self.write_route_sample(selected_sample)

    def rewrite_clean_route(self, samples, route_file=None):
        route_file = route_file or self.route_file
        temporary_file = f'{route_file}.cleaning'
        with open(temporary_file, 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['id', 'x', 'y', 'yaw', 'v'])
            for point_id, sample in enumerate(samples):
                writer.writerow([
                    point_id,
                    f'{sample.x:.6f}',
                    f'{sample.y:.6f}',
                    f'{sample.yaw:.6f}',
                    f'{self.default_speed:.3f}',
                ])
        os.replace(temporary_file, route_file)

    def retain_invalid_route(self):
        invalid_directory = os.path.join(
            os.path.dirname(self.route_file) or '.',
            'invalid',
        )
        os.makedirs(invalid_directory, exist_ok=True)
        basename = os.path.splitext(os.path.basename(self.route_file))[0]
        invalid_file = os.path.join(
            invalid_directory,
            '%s.invalid_%s.csv'
            % (basename, time.strftime('%Y%m%d_%H%M%S')),
        )
        os.replace(self.route_file, invalid_file)
        return invalid_file

    def write_quality_report(self, report):
        report.update({
            'route_file': self.route_file,
            'raw_file': self.raw_file,
            'generated_at': time.strftime('%Y-%m-%dT%H:%M:%S%z'),
            'max_odom_step_m': self.max_odom_step,
            'max_odom_speed_mps': self.max_odom_speed,
            'hard_odom_speed_mps': self.hard_odom_speed,
            'max_record_step_m': self.max_record_step,
            'turn_confirm_samples': self.turn_confirm_samples,
        })
        temporary_file = f'{self.quality_report_file}.writing'
        with open(temporary_file, 'w') as file:
            json.dump(report, file, indent=2, sort_keys=True)
            file.write('\n')
        os.replace(temporary_file, self.quality_report_file)

    def finalize_route_file(self):
        if self.route_finalized:
            return
        self.route_finalized = True

        try:
            if not self.file.closed:
                self.file.flush()
                self.file.close()
            if not self.raw_handle.closed:
                self.raw_handle.flush()
                self.raw_handle.close()
        except Exception:
            pass

        try:
            clean_samples, report = build_clean_route(
                self.raw_samples,
                normal_spacing=self.min_distance,
                turn_spacing=self.turn_distance,
                turn_angle_deg=self.turn_enter_deg,
                turn_zone_distance=self.turn_zone_distance,
                simplify_tolerance=self.simplify_tolerance,
                spike_lateral_threshold=self.spike_lateral_threshold,
            )
            rejection_ratio = (
                float(self.rejected_observations) / self.total_observations
                if self.total_observations
                else 1.0
            )
            report.update({
                'observed_samples': self.total_observations,
                'accepted_observations': self.accepted_observations,
                'rejected_observations': self.rejected_observations,
                'rejection_ratio': rejection_ratio,
                'max_observed_step_m': self.max_observed_step,
                'max_observed_speed_mps': self.max_observed_speed,
                'runtime_failure': self.runtime_failure,
            })
            quality_reasons = validate_route_report(
                report,
                min_route_points=self.min_route_points,
                min_route_length_m=self.min_route_length,
                max_rejection_ratio=self.max_rejection_ratio,
                max_raw_gap_m=(
                    self.max_record_step
                    if self.max_record_step > 0.0
                    else float('inf')
                ),
                runtime_failure=self.runtime_failure,
            )
            self.rewrite_clean_route(clean_samples)
            report['valid'] = not quality_reasons
            report['status'] = 'valid' if not quality_reasons else 'failed'
            report['quality_failure_reasons'] = quality_reasons
            if quality_reasons:
                invalid_file = self.retain_invalid_route()
                report['invalid_route_file'] = invalid_file
                self.get_logger().error(
                    'RECORDING_FAILED: final CSV quality validation failed: '
                    + '; '.join(quality_reasons)
                )
                self.get_logger().error(
                    f'invalid route retained at: {invalid_file}'
                )
            else:
                self.route_valid = True
            self.write_quality_report(report)
            if self.route_valid:
                self.get_logger().info(
                    'final clean route saved and validated: '
                    f'raw={report["raw_samples"]}, '
                    f'route={report["route_points"]}, '
                    f'length={report["route_length_m"]:.3f}m, '
                    f'rejection_ratio={rejection_ratio:.1%}, '
                    f'max_gap={report["max_route_gap_m"]:.3f}m'
                )
            self.get_logger().info(
                f'quality report saved: {self.quality_report_file}'
            )
            if not self.route_valid:
                self.get_logger().error(
                    'RECORDING_FAILED: '
                    + '; '.join(quality_reasons)
                )
        except Exception as exc:
            self.runtime_failure = (
                self.runtime_failure
                or f'final route cleanup failed: {exc}'
            )
            self.get_logger().error(
                f'RECORDING_FAILED: {self.runtime_failure}'
            )
            report = {
                'status': 'failed',
                'valid': False,
                'runtime_failure': self.runtime_failure,
                'quality_failure_reasons': [self.runtime_failure],
                'observed_samples': self.total_observations,
                'accepted_observations': self.accepted_observations,
                'rejected_observations': self.rejected_observations,
                'rejection_ratio': (
                    float(self.rejected_observations) / self.total_observations
                    if self.total_observations
                    else 1.0
                ),
            }
            try:
                if os.path.exists(self.route_file):
                    report['invalid_route_file'] = self.retain_invalid_route()
                self.write_quality_report(report)
            except Exception as retain_exc:
                self.get_logger().error(
                    'failed to retain invalid route diagnostics: '
                    f'{retain_exc}'
                )

        if not self.route_valid or not self.sanitize_on_close:
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
    # SIGTERM does not wake a Foxy executor when the handler only raises a
    # Python exception. Shutting down the context wakes the wait set so the
    # ``finally`` block can rebuild and atomically save the clean CSV.
    if rclpy.ok():
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = RouteRecorder()
    previous_sigterm_handler = signal.getsignal(signal.SIGTERM)
    signal.signal(signal.SIGTERM, handle_shutdown_signal)
    try:
        # A finite wait guarantees that a queued SIGTERM is handled even when
        # /Odometry has stopped and no ROS callback is available to wake Foxy's
        # executor wait set.
        while rclpy.ok() and not node.stop_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().info('route_recorder stopped')
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        signal.signal(signal.SIGTERM, previous_sigterm_handler)
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

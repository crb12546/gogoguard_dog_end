#!/usr/bin/env python3
"""Fail fast when the Livox stream is unsafe for FAST-LIO."""

import argparse
import math
import re
import statistics
import sys
import time
from pathlib import Path

import rclpy
from livox_ros_driver2.msg import CustomMsg
from rclpy.node import Node
from sensor_msgs.msg import Imu


HEALTH_PREFIX = 'LIVOX_STREAM_HEALTH '


def check_driver_log(args):
    log_path = Path(args.driver_log)
    deadline = time.monotonic() + args.duration
    latest_detail = 'waiting for driver health reports'
    while time.monotonic() < deadline:
        try:
            text = log_path.read_text(errors='replace')
        except OSError as exc:
            latest_detail = 'cannot read %s: %s' % (log_path, exc)
            time.sleep(0.20)
            continue

        if 'Dropping malformed Livox frame' in text:
            return False, 'driver dropped a malformed frame'
        if 'Livox point packet backlog' in text:
            return False, 'driver point packet queue is backlogged'

        lines = [line for line in text.splitlines() if HEALTH_PREFIX in line]
        if len(lines) < args.min_reports:
            latest_detail = 'only %d health reports, need %d' % (
                len(lines), args.min_reports
            )
            time.sleep(0.20)
            continue

        reports = []
        for line in lines[-args.min_reports:]:
            values = {
                key: float(value)
                for key, value in re.findall(r'([a-z_]+)=([0-9]+(?:\.[0-9]+)?)', line)
            }
            reports.append(values)

        required = {
            'frame_hz', 'imu_hz', 'span_ms', 'points',
            'lidar_imu_offset_ms', 'queue_depth',
        }
        if any(not required.issubset(report) for report in reports):
            return False, 'incomplete driver health report'

        for report in reports:
            if report['frame_hz'] < args.min_delivery_hz or report['frame_hz'] > 13.0:
                return False, 'bad driver frame rate %.3fHz' % report['frame_hz']
            if report['imu_hz'] < 100.0 or report['imu_hz'] > 300.0:
                return False, 'bad driver IMU rate %.3fHz' % report['imu_hz']
            if report['span_ms'] < 80.0 or report['span_ms'] > 130.0:
                return False, 'bad driver frame span %.3fms' % report['span_ms']
            if report['points'] < args.min_points:
                return False, 'low driver point count %.0f' % report['points']
            if report['lidar_imu_offset_ms'] > 100.0:
                return False, 'lidar/IMU offset too large %.3fms' % report['lidar_imu_offset_ms']
            if report['queue_depth'] > 512.0:
                return False, 'driver queue depth too high %.0f' % report['queue_depth']

        frame_hz = statistics.mean(report['frame_hz'] for report in reports)
        imu_hz = statistics.mean(report['imu_hz'] for report in reports)
        span_ms = statistics.mean(report['span_ms'] for report in reports)
        min_points = min(report['points'] for report in reports)
        max_offset = max(report['lidar_imu_offset_ms'] for report in reports)
        max_queue = max(report['queue_depth'] for report in reports)
        return True, (
            'reports=%d frame_hz=%.3f imu_hz=%.3f span_ms=%.3f '
            'min_points=%.0f max_lidar_imu_offset_ms=%.3f max_queue_depth=%.0f'
            % (
                len(reports), frame_hz, imu_hz, span_ms,
                min_points, max_offset, max_queue,
            )
        )

    return False, latest_detail


def stamp_seconds(msg):
    return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9


class LivoxStreamCheck(Node):
    def __init__(self, args):
        super().__init__('go2_livox_stream_check')
        self.args = args
        self.samples = []
        self.imu_samples = []
        self.errors = []
        self.create_subscription(CustomMsg, args.topic, self.on_lidar, 10)
        self.create_subscription(Imu, args.imu_topic, self.on_imu, 100)

    def on_lidar(self, msg):
        wall_time = time.monotonic()
        stamp = stamp_seconds(msg)
        point_num = int(msg.point_num)
        points_size = len(msg.points)

        if not math.isfinite(stamp) or stamp <= 0.0:
            self.errors.append('invalid stamp %.9f' % stamp)
        if point_num < self.args.min_points or points_size < self.args.min_points:
            self.errors.append(
                'low point count point_num=%d points_size=%d' % (point_num, points_size)
            )
        if point_num != points_size:
            self.errors.append(
                'point count mismatch point_num=%d points_size=%d' % (point_num, points_size)
            )

        if self.samples:
            stamp_dt = stamp - self.samples[-1][1]
            if stamp_dt < self.args.min_stamp_dt or stamp_dt > self.args.max_stamp_dt:
                self.errors.append('bad stamp dt %.6fs' % stamp_dt)

        self.samples.append((wall_time, stamp, point_num))

    def on_imu(self, msg):
        wall_time = time.monotonic()
        stamp = stamp_seconds(msg)
        if not math.isfinite(stamp) or stamp <= 0.0:
            self.errors.append('invalid IMU stamp %.9f' % stamp)
        if self.imu_samples:
            stamp_dt = stamp - self.imu_samples[-1][1]
            if stamp_dt <= 0.0 or stamp_dt > self.args.max_imu_stamp_dt:
                self.errors.append('bad IMU stamp dt %.6fs' % stamp_dt)
        self.imu_samples.append((wall_time, stamp))

    def result(self):
        if len(self.samples) < self.args.min_frames:
            return False, 'only %d frames, need %d' % (len(self.samples), self.args.min_frames)
        if len(self.imu_samples) < self.args.min_imu_samples:
            return False, 'only %d IMU samples, need %d' % (
                len(self.imu_samples), self.args.min_imu_samples
            )
        if self.errors:
            return False, '; '.join(self.errors[:8])

        wall_span = self.samples[-1][0] - self.samples[0][0]
        stamp_span = self.samples[-1][1] - self.samples[0][1]
        if wall_span <= 0.0 or stamp_span <= 0.0:
            return False, 'non-positive stream span'

        delivery_hz = (len(self.samples) - 1) / wall_span
        clock_ratio = stamp_span / wall_span
        imu_wall_span = self.imu_samples[-1][0] - self.imu_samples[0][0]
        imu_stamp_span = self.imu_samples[-1][1] - self.imu_samples[0][1]
        if imu_wall_span <= 0.0 or imu_stamp_span <= 0.0:
            return False, 'non-positive IMU stream span'
        imu_delivery_hz = (len(self.imu_samples) - 1) / imu_wall_span
        imu_clock_ratio = imu_stamp_span / imu_wall_span
        lidar_imu_offset = abs(self.samples[-1][1] - self.imu_samples[-1][1])
        stamp_dts = [
            self.samples[index][1] - self.samples[index - 1][1]
            for index in range(1, len(self.samples))
        ]
        min_points = min(sample[2] for sample in self.samples)
        summary = (
            'frames=%d delivery_hz=%.2f stamp_dt_median=%.6f '
            'stamp_dt_min=%.6f stamp_dt_max=%.6f clock_ratio=%.3f min_points=%d '
            'imu_samples=%d imu_hz=%.2f imu_clock_ratio=%.3f lidar_imu_offset=%.3f'
            % (
                len(self.samples),
                delivery_hz,
                statistics.median(stamp_dts),
                min(stamp_dts),
                max(stamp_dts),
                clock_ratio,
                min_points,
                len(self.imu_samples),
                imu_delivery_hz,
                imu_clock_ratio,
                lidar_imu_offset,
            )
        )
        if delivery_hz < self.args.min_delivery_hz:
            return False, summary + ' delivery rate too low'
        if clock_ratio < self.args.min_clock_ratio or clock_ratio > self.args.max_clock_ratio:
            return False, summary + ' sensor time and wall time diverge'
        if imu_delivery_hz < self.args.min_imu_delivery_hz:
            return False, summary + ' IMU delivery rate too low'
        if imu_clock_ratio < self.args.min_clock_ratio or imu_clock_ratio > self.args.max_clock_ratio:
            return False, summary + ' IMU sensor time and wall time diverge'
        if lidar_imu_offset > self.args.max_lidar_imu_offset:
            return False, summary + ' lidar/IMU clocks are not aligned'
        return True, summary


def parse_args():
    parser = argparse.ArgumentParser(description='Validate Livox timing before starting FAST-LIO')
    parser.add_argument('--topic', default='/livox/lidar')
    parser.add_argument('--imu-topic', default='/livox/imu')
    parser.add_argument('--driver-log', default='')
    parser.add_argument('--min-reports', type=int, default=3)
    parser.add_argument('--duration', type=float, default=4.0)
    parser.add_argument('--min-frames', type=int, default=25)
    parser.add_argument('--min-points', type=int, default=5000)
    parser.add_argument('--min-stamp-dt', type=float, default=0.04)
    parser.add_argument('--max-stamp-dt', type=float, default=0.25)
    parser.add_argument('--min-delivery-hz', type=float, default=7.0)
    parser.add_argument('--min-imu-samples', type=int, default=100)
    parser.add_argument('--min-imu-delivery-hz', type=float, default=25.0)
    parser.add_argument('--max-imu-stamp-dt', type=float, default=0.20)
    parser.add_argument('--max-lidar-imu-offset', type=float, default=0.50)
    parser.add_argument('--min-clock-ratio', type=float, default=0.70)
    parser.add_argument('--max-clock-ratio', type=float, default=1.30)
    return parser.parse_args()


def main():
    args = parse_args()
    if args.driver_log:
        ok, detail = check_driver_log(args)
        if ok:
            print('LIVOX_STREAM_OK ' + detail)
            return 0
        print('LIVOX_STREAM_FAILED ' + detail, file=sys.stderr)
        return 2

    rclpy.init()
    node = LivoxStreamCheck(args)
    deadline = time.monotonic() + args.duration
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.10)

    ok, detail = node.result()
    node.destroy_node()
    rclpy.shutdown()
    if ok:
        print('LIVOX_STREAM_OK ' + detail)
        return 0
    print('LIVOX_STREAM_FAILED ' + detail, file=sys.stderr)
    return 2


if __name__ == '__main__':
    raise SystemExit(main())

#!/usr/bin/env python3
import argparse
import csv
import math
import os
import time
from datetime import datetime

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, PointCloud2

try:
    from livox_ros_driver2.msg import CustomMsg as LivoxCustomMsg
except Exception:  # noqa: BLE001
    LivoxCustomMsg = None


WS = os.environ.get('GO2_WS', '/home/unitree/go2_fastlio_ws')


def stamp_sec(msg):
    header = getattr(msg, 'header', None)
    stamp = getattr(header, 'stamp', None)
    if stamp is None:
        return 0.0
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def angle_diff(a, b):
    return (a - b + math.pi) % (2.0 * math.pi) - math.pi


class CsvFile:
    def __init__(self, path, header):
        self.path = path
        self.file = open(path, 'w', newline='', buffering=1)
        self.writer = csv.writer(self.file)
        self.writer.writerow(header)

    def row(self, values):
        self.writer.writerow(values)

    def close(self):
        self.file.close()


class LioTraceRecorder(Node):
    def __init__(self, args):
        super().__init__('go2_lio_trace_recorder')
        self.args = args
        os.makedirs(args.output_dir, exist_ok=True)
        prefix = datetime.now().strftime('lio_trace_%Y%m%d_%H%M%S')
        self.odom_csv = CsvFile(
            os.path.join(args.output_dir, prefix + '_odom.csv'),
            [
                'wall_time', 'stamp', 'x', 'y', 'z', 'yaw',
                'dt', 'dxy', 'dz', 'dyaw',
            ],
        )
        self.cloud_csv = CsvFile(
            os.path.join(args.output_dir, prefix + '_cloud.csv'),
            [
                'wall_time', 'topic', 'stamp', 'dt', 'stamp_dt',
                'width', 'height', 'points', 'point_step', 'row_step',
                'data_len', 'is_dense',
            ],
        )
        self.livox_csv = CsvFile(
            os.path.join(args.output_dir, prefix + '_livox.csv'),
            ['wall_time', 'topic', 'stamp', 'dt', 'stamp_dt', 'point_num', 'timebase'],
        )
        self.imu_csv = CsvFile(
            os.path.join(args.output_dir, prefix + '_imu.csv'),
            [
                'wall_time', 'stamp', 'dt', 'stamp_dt',
                'angular_x', 'angular_y', 'angular_z',
                'linear_x', 'linear_y', 'linear_z',
            ],
        )
        self.events_csv = CsvFile(
            os.path.join(args.output_dir, prefix + '_events.csv'),
            ['wall_time', 'event', 'detail'],
        )

        self.last_odom = None
        self.last_wall = {}
        self.last_stamp = {}
        self.last_seen = {}
        self.last_write = {}
        self.start_time = time.time()

        self.create_subscription(Odometry, args.odom_topic, self.on_odom, 50)
        self.create_subscription(PointCloud2, args.cloud_topic, lambda m: self.on_cloud(args.cloud_topic, m), 20)
        self.create_subscription(PointCloud2, args.cloud_body_topic, lambda m: self.on_cloud(args.cloud_body_topic, m), 20)
        if args.include_imu:
            self.create_subscription(Imu, args.imu_topic, self.on_imu, 100)
        else:
            self.event('IMU_TRACE_DISABLED', args.imu_topic)
        if args.include_raw_livox and LivoxCustomMsg is not None:
            self.create_subscription(
                LivoxCustomMsg,
                args.livox_topic,
                lambda m: self.on_livox(args.livox_topic, m),
                20,
            )
        elif args.include_raw_livox:
            self.event('LIVOX_MSG_IMPORT_FAILED', 'livox_ros_driver2.msg.CustomMsg unavailable')
        else:
            self.event('RAW_LIVOX_TRACE_DISABLED', args.livox_topic)

        self.create_timer(1.0, self.on_timer)
        self.get_logger().info('lio trace recorder started: %s' % args.output_dir)

    def event(self, name, detail):
        self.events_csv.row(['%.6f' % time.time(), name, detail])
        self.get_logger().warn('%s: %s' % (name, detail))

    def update_gap(self, topic, wall, stamp, gap_limit, stamp_gap_limit=None):
        last_wall = self.last_wall.get(topic)
        last_stamp = self.last_stamp.get(topic)
        self.last_wall[topic] = wall
        self.last_stamp[topic] = stamp
        self.last_seen[topic] = wall
        dt = 0.0 if last_wall is None else wall - last_wall
        stamp_dt = 0.0 if last_stamp is None or stamp == 0.0 or last_stamp == 0.0 else stamp - last_stamp
        if last_wall is not None and dt > gap_limit:
            self.event('TOPIC_GAP', '%s dt=%.3f limit=%.3f' % (topic, dt, gap_limit))
        if stamp_gap_limit is not None and stamp_dt > stamp_gap_limit:
            self.event('STAMP_GAP', '%s stamp_dt=%.3f limit=%.3f' % (topic, stamp_dt, stamp_gap_limit))
        return dt, stamp_dt

    def on_odom(self, msg):
        wall = time.time()
        stamp = stamp_sec(msg)
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        x = float(p.x)
        y = float(p.y)
        z = float(p.z)
        yaw = yaw_from_quat(q)
        dt, _ = self.update_gap(self.args.odom_topic, wall, stamp, self.args.odom_gap)

        dxy = 0.0
        dz = 0.0
        dyaw = 0.0
        if self.last_odom is not None:
            _, px, py, pz, pyaw = self.last_odom
            dxy = math.hypot(x - px, y - py)
            dz = z - pz
            dyaw = angle_diff(yaw, pyaw)
            if (
                dxy > self.args.odom_jump_distance
                or abs(dz) > self.args.odom_jump_z
                or abs(dyaw) > self.args.odom_jump_yaw
            ):
                self.event(
                    'ODOM_JUMP',
                    'dt=%.3f dxy=%.3f dz=%.3f dyaw=%.3f pos=(%.3f,%.3f,%.3f) yaw=%.3f'
                    % (dt, dxy, dz, dyaw, x, y, z, yaw),
                )
        if (
            not all(math.isfinite(v) for v in (x, y, z, yaw))
            or abs(x) > self.args.max_abs_xy
            or abs(y) > self.args.max_abs_xy
            or abs(z) > self.args.max_abs_z
        ):
            self.event('ODOM_ABS_INVALID', 'pos=(%.3f,%.3f,%.3f) yaw=%.3f' % (x, y, z, yaw))

        self.odom_csv.row([
            '%.6f' % wall,
            '%.6f' % stamp,
            '%.6f' % x,
            '%.6f' % y,
            '%.6f' % z,
            '%.6f' % yaw,
            '%.6f' % dt,
            '%.6f' % dxy,
            '%.6f' % dz,
            '%.6f' % dyaw,
        ])
        self.last_odom = (wall, x, y, z, yaw)

    def on_cloud(self, topic, msg):
        wall = time.time()
        stamp = stamp_sec(msg)
        dt, stamp_dt = self.update_gap(topic, wall, stamp, self.args.cloud_gap, self.args.cloud_stamp_gap)
        points = int(msg.width) * int(msg.height)
        data_len = len(msg.data)
        if points <= 0 or data_len <= 0:
            self.event('CLOUD_EMPTY', '%s points=%d data_len=%d' % (topic, points, data_len))
        self.cloud_csv.row([
            '%.6f' % wall,
            topic,
            '%.6f' % stamp,
            '%.6f' % dt,
            '%.6f' % stamp_dt,
            int(msg.width),
            int(msg.height),
            points,
            int(msg.point_step),
            int(msg.row_step),
            data_len,
            int(bool(msg.is_dense)),
        ])

    def on_livox(self, topic, msg):
        wall = time.time()
        stamp = stamp_sec(msg)
        dt, stamp_dt = self.update_gap(topic, wall, stamp, self.args.livox_gap, self.args.livox_stamp_gap)
        point_num = int(getattr(msg, 'point_num', 0))
        timebase = int(getattr(msg, 'timebase', 0))
        if point_num <= 0:
            self.event('LIVOX_EMPTY', '%s point_num=%d' % (topic, point_num))
        last_write = self.last_write.get(topic, 0.0)
        if wall - last_write < self.args.livox_write_period:
            return
        self.last_write[topic] = wall
        self.livox_csv.row([
            '%.6f' % wall,
            topic,
            '%.6f' % stamp,
            '%.6f' % dt,
            '%.6f' % stamp_dt,
            point_num,
            timebase,
        ])

    def on_imu(self, msg):
        wall = time.time()
        stamp = stamp_sec(msg)
        dt, stamp_dt = self.update_gap(self.args.imu_topic, wall, stamp, self.args.imu_gap, self.args.imu_stamp_gap)
        last_write = self.last_write.get(self.args.imu_topic, 0.0)
        if wall - last_write < self.args.imu_write_period:
            return
        self.last_write[self.args.imu_topic] = wall
        av = msg.angular_velocity
        la = msg.linear_acceleration
        self.imu_csv.row([
            '%.6f' % wall,
            '%.6f' % stamp,
            '%.6f' % dt,
            '%.6f' % stamp_dt,
            '%.6f' % av.x,
            '%.6f' % av.y,
            '%.6f' % av.z,
            '%.6f' % la.x,
            '%.6f' % la.y,
            '%.6f' % la.z,
        ])

    def on_timer(self):
        now = time.time()
        if self.args.duration > 0 and now - self.start_time > self.args.duration:
            self.event('RECORDER_DONE', 'duration reached')
            raise KeyboardInterrupt
        for topic, limit in (
            (self.args.odom_topic, self.args.no_odom_timeout),
            (self.args.cloud_topic, self.args.no_cloud_timeout),
            (self.args.cloud_body_topic, self.args.no_cloud_timeout),
            (self.args.livox_topic, self.args.no_livox_timeout),
            (self.args.imu_topic, self.args.no_imu_timeout),
        ):
            last = self.last_seen.get(topic)
            if last is not None and now - last > limit:
                self.event('TOPIC_STALE', '%s age=%.3f limit=%.3f' % (topic, now - last, limit))
                self.last_seen[topic] = now

    def close(self):
        for f in (self.odom_csv, self.cloud_csv, self.livox_csv, self.imu_csv, self.events_csv):
            f.close()


def parse_args():
    parser = argparse.ArgumentParser(description='Record lightweight FAST-LIO input/output diagnostics.')
    parser.add_argument('--output-dir', default=os.path.join(WS, 'patrol_logs/diagnostics'))
    parser.add_argument('--duration', type=float, default=0.0)
    parser.add_argument('--odom-topic', default='/Odometry')
    parser.add_argument('--cloud-topic', default='/cloud_registered')
    parser.add_argument('--cloud-body-topic', default='/cloud_registered_body')
    parser.add_argument('--livox-topic', default='/livox/lidar')
    parser.add_argument('--imu-topic', default='/livox/imu')
    parser.add_argument('--include-raw-livox', action='store_true')
    parser.add_argument('--include-imu', action='store_true', default=True)
    parser.add_argument('--livox-write-period', type=float, default=0.20)
    parser.add_argument('--imu-write-period', type=float, default=0.20)
    parser.add_argument('--odom-jump-distance', type=float, default=0.50)
    parser.add_argument('--odom-jump-z', type=float, default=0.30)
    parser.add_argument('--odom-jump-yaw', type=float, default=0.70)
    parser.add_argument('--max-abs-xy', type=float, default=100.0)
    parser.add_argument('--max-abs-z', type=float, default=5.0)
    parser.add_argument('--odom-gap', type=float, default=1.00)
    parser.add_argument('--cloud-gap', type=float, default=1.00)
    parser.add_argument('--cloud-stamp-gap', type=float, default=1.00)
    parser.add_argument('--livox-gap', type=float, default=1.00)
    parser.add_argument('--livox-stamp-gap', type=float, default=1.00)
    parser.add_argument('--imu-gap', type=float, default=0.50)
    parser.add_argument('--imu-stamp-gap', type=float, default=0.50)
    parser.add_argument('--no-odom-timeout', type=float, default=2.0)
    parser.add_argument('--no-cloud-timeout', type=float, default=2.0)
    parser.add_argument('--no-livox-timeout', type=float, default=2.0)
    parser.add_argument('--no-imu-timeout', type=float, default=2.0)
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = LioTraceRecorder(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

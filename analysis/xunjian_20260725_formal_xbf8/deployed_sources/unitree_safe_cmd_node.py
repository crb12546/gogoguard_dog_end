#!/usr/bin/env python3
import json
import math
import struct
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud2, PointField
from unitree_api.msg import Request

from .patrol_control import (
    limit_planar_command,
    point_in_lateral_motion_roi,
    stream_receive_age,
)


SPORT_API_ID_MOVE = 1008


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


class UnitreeSafeCmdNode(Node):
    def __init__(self):
        super().__init__('unitree_safe_cmd_node')

        self.declare_parameter('cmd_topic', '/patrol_cmd')
        self.declare_parameter('pointcloud_topic', '/cloud_registered_body')
        self.declare_parameter('sport_request_topic', '/api/sport/request')
        self.declare_parameter('output_cmd_topic', '')

        self.declare_parameter('max_vx', 0.5)
        self.declare_parameter('max_vy', 0.15)
        self.declare_parameter('max_yaw_rate', 0.45)
        self.declare_parameter('publish_rate', 40.0)
        self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('cloud_timeout', 1.0)

        self.declare_parameter('roi_x_min', 0.35)
        self.declare_parameter('roi_x_max', 1.20)
        self.declare_parameter('roi_y_min', -0.45)
        self.declare_parameter('roi_y_max', 0.45)
        self.declare_parameter('roi_z_min', 0.25)
        self.declare_parameter('roi_z_max', 0.90)

        self.declare_parameter('lateral_cmd_deadband', 0.02)
        self.declare_parameter('lateral_roi_x_min', 0.10)
        self.declare_parameter('lateral_roi_x_max', 1.00)
        self.declare_parameter('lateral_roi_inner_y', 0.30)
        self.declare_parameter('lateral_roi_outer_y', 0.65)
        self.declare_parameter('lateral_min_stop_points', 12)

        self.declare_parameter('stop_distance', 0.70)
        self.declare_parameter('resume_distance', 0.95)
        self.declare_parameter('min_stop_points', 12)
        self.declare_parameter('stop_frames', 1)
        self.declare_parameter('clear_frames', 5)
        self.declare_parameter('point_skip', 2)
        self.declare_parameter('max_cloud_process_rate', 20.0)

        self.cmd_topic = self.get_parameter('cmd_topic').value
        self.pointcloud_topic = self.get_parameter('pointcloud_topic').value
        self.sport_request_topic = self.get_parameter('sport_request_topic').value
        self.output_cmd_topic = str(self.get_parameter('output_cmd_topic').value)

        self.max_vx = float(self.get_parameter('max_vx').value)
        self.max_vy = float(self.get_parameter('max_vy').value)
        self.max_yaw_rate = float(self.get_parameter('max_yaw_rate').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)
        self.cloud_timeout = float(self.get_parameter('cloud_timeout').value)

        self.roi_x_min = float(self.get_parameter('roi_x_min').value)
        self.roi_x_max = float(self.get_parameter('roi_x_max').value)
        self.roi_y_min = float(self.get_parameter('roi_y_min').value)
        self.roi_y_max = float(self.get_parameter('roi_y_max').value)
        self.roi_z_min = float(self.get_parameter('roi_z_min').value)
        self.roi_z_max = float(self.get_parameter('roi_z_max').value)

        self.lateral_cmd_deadband = float(self.get_parameter('lateral_cmd_deadband').value)
        self.lateral_roi_x_min = float(self.get_parameter('lateral_roi_x_min').value)
        self.lateral_roi_x_max = float(self.get_parameter('lateral_roi_x_max').value)
        self.lateral_roi_inner_y = float(self.get_parameter('lateral_roi_inner_y').value)
        self.lateral_roi_outer_y = float(self.get_parameter('lateral_roi_outer_y').value)
        self.lateral_min_stop_points = int(self.get_parameter('lateral_min_stop_points').value)

        self.stop_distance = float(self.get_parameter('stop_distance').value)
        self.resume_distance = float(self.get_parameter('resume_distance').value)
        self.min_stop_points = int(self.get_parameter('min_stop_points').value)
        self.stop_frames = int(self.get_parameter('stop_frames').value)
        self.clear_frames = int(self.get_parameter('clear_frames').value)
        self.point_skip = max(1, int(self.get_parameter('point_skip').value))
        self.max_cloud_process_rate = float(self.get_parameter('max_cloud_process_rate').value)

        self.last_vx = 0.0
        self.last_vy = 0.0
        self.last_yaw_rate = 0.0
        self.last_cmd_time = 0.0
        self.last_log_time = 0.0
        self.last_cloud_time = 0.0
        self.last_cloud_stamp_age = float('inf')
        self.last_cloud_process_time = 0.0
        self.timing_period = 1.0 / self.publish_rate
        self.timing_last_timer_mono = 0.0
        self.timing_last_cmd_mono = 0.0
        self.timing_last_cloud_mono = 0.0
        self.timing_last_report_mono = time.monotonic()
        self.timing_last_alert_mono = 0.0
        self.timing_timer_gaps_ms = []
        self.timing_timer_compute_ms = []
        self.timing_cmd_gaps_ms = []
        self.timing_cmd_callback_ms = []
        self.timing_cloud_gaps_ms = []
        self.timing_cloud_callback_ms = []
        self.timing_publish_ms = []
        self.timing_cmd_ages_ms = []
        self.timing_cloud_ages_ms = []
        self.timing_cmd_count = 0
        self.timing_output_count = 0
        self.timing_override_count = 0
        self.timing_deadline_misses = 0

        self.obstacle_stop = False
        self.stop_frame_count = 0
        self.clear_frame_count = 0

        self.last_stop_count = 0
        self.last_roi_count = 0
        self.last_nearest_x = float('inf')
        self.last_lateral_count = 0
        self.last_nearest_lateral_y = float('inf')

        self.pub = self.create_publisher(Request, self.sport_request_topic, 10)
        self.twist_pub = None
        if self.output_cmd_topic:
            self.twist_pub = self.create_publisher(Twist, self.output_cmd_topic, 10)
        self.create_subscription(Twist, self.cmd_topic, self.cmd_callback, 10)
        self.create_subscription(PointCloud2, self.pointcloud_topic, self.cloud_callback, 10)
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)

        self.get_logger().info(
            f'unitree_safe_cmd_node ACTIVE: obstacle -> Move(0,0,0), '
            f'resume after {self.clear_frames} clear frames; lateral swept ROI enabled'
        )
        self.get_logger().info(
            'cloud safety freshness uses local receive time; '
            'message header stamp is diagnostic only, '
            f'receive_timeout={self.cloud_timeout:.2f}s'
        )
        if self.twist_pub is not None:
            self.get_logger().info(f'output mode: Twist -> {self.output_cmd_topic}')
        else:
            self.get_logger().info(
                'output mode: unitree_api Request -> '
                f'{self.sport_request_topic}'
            )

    def clamp(self, v, lo, hi):
        return max(lo, min(hi, v))

    def cmd_callback(self, msg):
        callback_start = time.monotonic()
        if self.timing_last_cmd_mono > 0.0:
            self.timing_cmd_gaps_ms.append(
                (callback_start - self.timing_last_cmd_mono) * 1000.0
            )
        self.timing_last_cmd_mono = callback_start
        try:
            (
                self.last_vx,
                self.last_vy,
                self.last_yaw_rate,
            ) = limit_planar_command(
                msg.linear.x,
                msg.linear.y,
                msg.angular.z,
                self.max_vx,
                self.max_vy,
                self.max_yaw_rate,
            )
            self.last_cmd_time = time.time()
            self.timing_cmd_count += 1
        finally:
            self.timing_cmd_callback_ms.append(
                (time.monotonic() - callback_start) * 1000.0
            )

    def make_move_request(self, vx, vy, yaw_rate):
        req = Request()
        req.header.identity.id = 9100
        req.header.identity.api_id = SPORT_API_ID_MOVE
        req.header.lease.id = 0
        req.header.policy.priority = 0
        req.header.policy.noreply = False
        req.parameter = json.dumps({
            'x': float(vx),
            'y': float(vy),
            'z': float(yaw_rate)
        })
        req.binary = []
        return req

    def publish_move(self, vx, vy, yaw_rate):
        publish_start = time.monotonic()
        if self.twist_pub is not None:
            cmd = Twist()
            cmd.linear.x = float(vx)
            cmd.linear.y = float(vy)
            cmd.angular.z = float(yaw_rate)
            self.twist_pub.publish(cmd)
        else:
            self.pub.publish(self.make_move_request(vx, vy, yaw_rate))
        self.timing_publish_ms.append(
            (time.monotonic() - publish_start) * 1000.0
        )
        self.timing_output_count += 1

    def get_xyz_offsets(self, msg):
        offsets = {}
        datatypes = {}

        for f in msg.fields:
            if f.name in ('x', 'y', 'z'):
                offsets[f.name] = f.offset
                datatypes[f.name] = f.datatype

        if not all(k in offsets for k in ('x', 'y', 'z')):
            return None
        if not all(datatypes[k] == PointField.FLOAT32 for k in ('x', 'y', 'z')):
            return None

        return offsets

    def in_roi(self, x, y, z):
        return (
            self.roi_x_min <= x <= self.roi_x_max
            and self.roi_y_min <= y <= self.roi_y_max
            and self.roi_z_min <= z <= self.roi_z_max
        )

    def cloud_callback(self, msg):
        callback_start = time.monotonic()
        if self.timing_last_cloud_mono > 0.0:
            self.timing_cloud_gaps_ms.append(
                (callback_start - self.timing_last_cloud_mono) * 1000.0
            )
        self.timing_last_cloud_mono = callback_start
        try:
            self.process_cloud_message(msg)
        finally:
            self.timing_cloud_callback_ms.append(
                (time.monotonic() - callback_start) * 1000.0
            )

    def process_cloud_message(self, msg):
        now = time.time()
        self.last_cloud_time = now
        stamp_time = (
            float(msg.header.stamp.sec)
            + float(msg.header.stamp.nanosec) * 1e-9
        )
        if stamp_time <= 0.0:
            stamp_time = now
        if stamp_time > now + 0.50:
            self.last_cloud_stamp_age = float('inf')
        else:
            self.last_cloud_stamp_age = max(0.0, now - stamp_time)

        min_period = 1.0 / max(self.max_cloud_process_rate, 1.0)
        if now - self.last_cloud_process_time < min_period:
            return
        self.last_cloud_process_time = now

        offsets = self.get_xyz_offsets(msg)
        if offsets is None:
            self.get_logger().warn('PointCloud2 missing float32 x/y/z fields')
            return

        endian = '>' if msg.is_bigendian else '<'
        data = msg.data
        point_step = msg.point_step
        total_points = msg.width * msg.height

        roi_count = 0
        stop_count = 0
        nearest_x = float('inf')
        lateral_count = 0
        nearest_lateral_y = float('inf')

        for i in range(0, total_points, self.point_skip):
            base = i * point_step
            try:
                x = struct.unpack_from(endian + 'f', data, base + offsets['x'])[0]
                y = struct.unpack_from(endian + 'f', data, base + offsets['y'])[0]
                z = struct.unpack_from(endian + 'f', data, base + offsets['z'])[0]
            except struct.error:
                continue

            if math.isnan(x) or math.isnan(y) or math.isnan(z):
                continue

            if self.in_roi(x, y, z):
                roi_count += 1
                nearest_x = min(nearest_x, x)
                if x <= self.stop_distance:
                    stop_count += 1

            if point_in_lateral_motion_roi(
                x,
                y,
                z,
                self.last_vy,
                cmd_deadband=self.lateral_cmd_deadband,
                x_min=self.lateral_roi_x_min,
                x_max=self.lateral_roi_x_max,
                inner_y=self.lateral_roi_inner_y,
                outer_y=self.lateral_roi_outer_y,
                z_min=self.roi_z_min,
                z_max=self.roi_z_max,
            ):
                lateral_count += 1
                nearest_lateral_y = min(nearest_lateral_y, abs(y))

        self.last_roi_count = roi_count
        self.last_stop_count = stop_count
        self.last_nearest_x = nearest_x
        self.last_lateral_count = lateral_count
        self.last_nearest_lateral_y = nearest_lateral_y

        unsafe = (
            stop_count >= self.min_stop_points
            or lateral_count >= self.lateral_min_stop_points
        )

        if unsafe:
            self.stop_frame_count += 1
            self.clear_frame_count = 0
        else:
            self.clear_frame_count += 1
            self.stop_frame_count = 0

        if not self.obstacle_stop and self.stop_frame_count >= self.stop_frames:
            self.obstacle_stop = True
            nearest_text = 'inf' if nearest_x == float('inf') else f'{nearest_x:.2f}'
            self.get_logger().warn(
                f'OBSTACLE DETECTED: stop_count={stop_count}, roi_count={roi_count}, '
                f'lateral_count={lateral_count}, nearest_x={nearest_text}'
            )

        # 关键：连续 clear_frames 帧没有危险点，才恢复
        if self.obstacle_stop and self.clear_frame_count >= self.clear_frames:
            self.obstacle_stop = False
            self.get_logger().info(
                f'obstacle cleared after {self.clear_frame_count} clear frames, resume move'
            )

    def timer_callback(self):
        loop_start = time.monotonic()
        if self.timing_last_timer_mono > 0.0:
            gap_ms = (
                loop_start - self.timing_last_timer_mono
            ) * 1000.0
            self.timing_timer_gaps_ms.append(gap_ms)
            if gap_ms > self.timing_period * 1500.0:
                self.timing_deadline_misses += 1
        self.timing_last_timer_mono = loop_start
        try:
            self.publish_safe_cycle()
        finally:
            self.timing_timer_compute_ms.append(
                (time.monotonic() - loop_start) * 1000.0
            )
            self.maybe_log_timing(time.monotonic())

    def publish_safe_cycle(self):
        now = time.time()
        command_age = stream_receive_age(now, self.last_cmd_time)
        cloud_receive_age = stream_receive_age(
            now,
            self.last_cloud_time,
        )

        out_vx = self.last_vx
        out_vy = self.last_vy
        out_yaw_rate = self.last_yaw_rate
        reason = 'normal'

        if command_age > self.cmd_timeout:
            out_vx, out_vy, out_yaw_rate = (0.0, 0.0, 0.0)
            reason = 'cmd_timeout'

        elif cloud_receive_age > self.cloud_timeout:
            out_vx, out_vy, out_yaw_rate = (0.0, 0.0, 0.0)
            reason = 'cloud_timeout'

        elif self.obstacle_stop:
            out_vx, out_vy, out_yaw_rate = (0.0, 0.0, 0.0)
            reason = 'obstacle'

        if math.isfinite(command_age):
            self.timing_cmd_ages_ms.append(command_age * 1000.0)
        if math.isfinite(cloud_receive_age):
            self.timing_cloud_ages_ms.append(
                cloud_receive_age * 1000.0
            )
        if reason != 'normal':
            self.timing_override_count += 1
        self.publish_move(out_vx, out_vy, out_yaw_rate)

        if now - self.last_log_time > 0.5:
            self.last_log_time = now
            nearest_text = (
                'inf'
                if self.last_nearest_x == float('inf')
                else f'{self.last_nearest_x:.2f}'
            )

            if reason == 'normal':
                self.get_logger().info(
                    f'Move x={out_vx:.3f}, y={out_vy:.3f}, z={out_yaw_rate:.3f}, '
                    f'stop_count={self.last_stop_count}, roi_count={self.last_roi_count}, '
                    f'lateral_count={self.last_lateral_count}, '
                    f'nearest_x={nearest_text}, clear_frames={self.clear_frame_count}, '
                    f'cloud_receive_age={cloud_receive_age:.3f}s, '
                    f'cloud_stamp_age={self.last_cloud_stamp_age:.3f}s'
                )
            else:
                self.get_logger().warn(
                    f'SAFE OVERRIDE {reason}: Move x=0.000, y=0.000, z=0.000, '
                    f'raw_x={self.last_vx:.3f}, raw_y={self.last_vy:.3f}, '
                    f'raw_z={self.last_yaw_rate:.3f}, '
                    f'stop_count={self.last_stop_count}, roi_count={self.last_roi_count}, '
                    f'lateral_count={self.last_lateral_count}, '
                    f'nearest_x={nearest_text}, clear_frames={self.clear_frame_count}, '
                    f'cloud_receive_age={cloud_receive_age:.3f}s, '
                    f'cloud_stamp_age={self.last_cloud_stamp_age:.3f}s'
                )

    def maybe_log_timing(self, now_mono):
        if now_mono - self.timing_last_report_mono < 1.0:
            return

        timer_avg, timer_p95, timer_max = timing_summary(
            self.timing_timer_gaps_ms
        )
        compute_avg, compute_p95, compute_max = timing_summary(
            self.timing_timer_compute_ms
        )
        cmd_gap_avg, cmd_gap_p95, cmd_gap_max = timing_summary(
            self.timing_cmd_gaps_ms
        )
        cmd_cb_avg, cmd_cb_p95, cmd_cb_max = timing_summary(
            self.timing_cmd_callback_ms
        )
        cloud_gap_avg, cloud_gap_p95, cloud_gap_max = timing_summary(
            self.timing_cloud_gaps_ms
        )
        cloud_cb_avg, cloud_cb_p95, cloud_cb_max = timing_summary(
            self.timing_cloud_callback_ms
        )
        publish_avg, publish_p95, publish_max = timing_summary(
            self.timing_publish_ms
        )
        cmd_age_avg, cmd_age_p95, cmd_age_max = timing_summary(
            self.timing_cmd_ages_ms
        )
        cloud_age_avg, cloud_age_p95, cloud_age_max = timing_summary(
            self.timing_cloud_ages_ms
        )
        self.get_logger().info(
            'TIMING_SAFE '
            f'period_ms={self.timing_period * 1000.0:.1f} '
            f'timer_gap_ms={timer_avg:.3f}/{timer_p95:.3f}/'
            f'{timer_max:.3f} '
            f'compute_ms={compute_avg:.3f}/{compute_p95:.3f}/'
            f'{compute_max:.3f} '
            f'cmd_gap_ms={cmd_gap_avg:.3f}/{cmd_gap_p95:.3f}/'
            f'{cmd_gap_max:.3f} '
            f'cmd_callback_ms={cmd_cb_avg:.3f}/{cmd_cb_p95:.3f}/'
            f'{cmd_cb_max:.3f} '
            f'cmd_age_ms={cmd_age_avg:.3f}/{cmd_age_p95:.3f}/'
            f'{cmd_age_max:.3f} '
            f'cloud_gap_ms={cloud_gap_avg:.3f}/{cloud_gap_p95:.3f}/'
            f'{cloud_gap_max:.3f} '
            f'cloud_callback_ms={cloud_cb_avg:.3f}/{cloud_cb_p95:.3f}/'
            f'{cloud_cb_max:.3f} '
            f'cloud_age_ms={cloud_age_avg:.3f}/{cloud_age_p95:.3f}/'
            f'{cloud_age_max:.3f} '
            f'ros_publish_ms={publish_avg:.3f}/{publish_p95:.3f}/'
            f'{publish_max:.3f} '
            f'deadline_miss={self.timing_deadline_misses} '
            f'cmd_count={self.timing_cmd_count} '
            f'output_count={self.timing_output_count} '
            f'override_count={self.timing_override_count}'
        )

        alert = (
            timer_max > self.timing_period * 1500.0
            or compute_max > self.timing_period * 500.0
            or cmd_gap_max > self.cmd_timeout * 500.0
            or cloud_cb_max > self.timing_period * 1000.0
        )
        if alert and now_mono - self.timing_last_alert_mono >= 1.0:
            self.timing_last_alert_mono = now_mono
            self.get_logger().warn(
                'TIMING_ALERT_SAFE '
                f'timer_gap_max_ms={timer_max:.3f} '
                f'compute_max_ms={compute_max:.3f} '
                f'cmd_gap_max_ms={cmd_gap_max:.3f} '
                f'cmd_age_max_ms={cmd_age_max:.3f} '
                f'cloud_callback_max_ms={cloud_cb_max:.3f} '
                f'cloud_age_max_ms={cloud_age_max:.3f}'
            )

        self.timing_timer_gaps_ms.clear()
        self.timing_timer_compute_ms.clear()
        self.timing_cmd_gaps_ms.clear()
        self.timing_cmd_callback_ms.clear()
        self.timing_cloud_gaps_ms.clear()
        self.timing_cloud_callback_ms.clear()
        self.timing_publish_ms.clear()
        self.timing_cmd_ages_ms.clear()
        self.timing_cloud_ages_ms.clear()
        self.timing_cmd_count = 0
        self.timing_output_count = 0
        self.timing_override_count = 0
        self.timing_deadline_misses = 0
        self.timing_last_report_mono = now_mono

    def destroy_node(self):
        try:
            self.publish_move(0.0, 0.0, 0.0)
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UnitreeSafeCmdNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Ctrl+C, publish Move(0,0,0)')
        node.publish_move(0.0, 0.0, 0.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

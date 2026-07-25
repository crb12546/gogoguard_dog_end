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


SPORT_API_ID_MOVE = 1008


class UnitreeSafeCmdNode(Node):
    def __init__(self):
        super().__init__('unitree_safe_cmd_node')

        self.declare_parameter('cmd_topic', '/patrol_cmd')
        self.declare_parameter('pointcloud_topic', '/cloud_registered_body')
        self.declare_parameter('sport_request_topic', '/api/sport/request')

        self.declare_parameter('max_vx', 0.5)
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

        self.max_vx = float(self.get_parameter('max_vx').value)
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

        self.stop_distance = float(self.get_parameter('stop_distance').value)
        self.resume_distance = float(self.get_parameter('resume_distance').value)
        self.min_stop_points = int(self.get_parameter('min_stop_points').value)
        self.stop_frames = int(self.get_parameter('stop_frames').value)
        self.clear_frames = int(self.get_parameter('clear_frames').value)
        self.point_skip = max(1, int(self.get_parameter('point_skip').value))
        self.max_cloud_process_rate = float(self.get_parameter('max_cloud_process_rate').value)

        self.last_vx = 0.0
        self.last_yaw_rate = 0.0
        self.last_cmd_time = 0.0
        self.last_log_time = 0.0
        self.last_cloud_time = 0.0
        self.last_cloud_process_time = 0.0

        self.obstacle_stop = False
        self.stop_frame_count = 0
        self.clear_frame_count = 0

        self.last_stop_count = 0
        self.last_roi_count = 0
        self.last_nearest_x = float('inf')

        self.pub = self.create_publisher(Request, self.sport_request_topic, 10)
        self.create_subscription(Twist, self.cmd_topic, self.cmd_callback, 10)
        self.create_subscription(PointCloud2, self.pointcloud_topic, self.cloud_callback, 10)
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)

        self.get_logger().info(
            f'unitree_safe_cmd_node ACTIVE: obstacle -> Move(0,0,0), '
            f'resume after {self.clear_frames} clear frames'
        )

    def clamp(self, v, lo, hi):
        return max(lo, min(hi, v))

    def cmd_callback(self, msg):
        self.last_vx = self.clamp(float(msg.linear.x), -self.max_vx, self.max_vx)
        self.last_yaw_rate = self.clamp(float(msg.angular.z), -self.max_yaw_rate, self.max_yaw_rate)
        self.last_cmd_time = time.time()

    def make_move_request(self, vx, yaw_rate):
        req = Request()
        req.header.identity.id = 9100
        req.header.identity.api_id = SPORT_API_ID_MOVE
        req.header.lease.id = 0
        req.header.policy.priority = 0
        req.header.policy.noreply = False
        req.parameter = json.dumps({
            'x': float(vx),
            'y': 0.0,
            'z': float(yaw_rate)
        })
        req.binary = []
        return req

    def publish_move(self, vx, yaw_rate):
        self.pub.publish(self.make_move_request(vx, yaw_rate))

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
            self.roi_x_min <= x <= self.roi_x_max and
            self.roi_y_min <= y <= self.roi_y_max and
            self.roi_z_min <= z <= self.roi_z_max
        )

    def cloud_callback(self, msg):
        now = time.time()
        self.last_cloud_time = now

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

            if not self.in_roi(x, y, z):
                continue

            roi_count += 1
            nearest_x = min(nearest_x, x)

            if x <= self.stop_distance:
                stop_count += 1

        self.last_roi_count = roi_count
        self.last_stop_count = stop_count
        self.last_nearest_x = nearest_x

        unsafe = stop_count >= self.min_stop_points

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
                f'OBSTACLE DETECTED: stop_count={stop_count}, roi_count={roi_count}, nearest_x={nearest_text}'
            )

        # 关键：连续 clear_frames 帧没有危险点，才恢复
        if self.obstacle_stop and self.clear_frame_count >= self.clear_frames:
            self.obstacle_stop = False
            self.get_logger().info(
                f'obstacle cleared after {self.clear_frame_count} clear frames, resume move'
            )

    def timer_callback(self):
        now = time.time()

        out_vx = self.last_vx
        out_yaw_rate = self.last_yaw_rate
        reason = 'normal'

        if now - self.last_cmd_time > self.cmd_timeout:
            out_vx = 0.0
            out_yaw_rate = 0.0
            reason = 'cmd_timeout'

        elif now - self.last_cloud_time > self.cloud_timeout:
            out_vx = 0.0
            out_yaw_rate = 0.0
            reason = 'cloud_timeout'

        elif self.obstacle_stop:
            out_vx = 0.0
            out_yaw_rate = 0.0
            reason = 'obstacle'

        self.publish_move(out_vx, out_yaw_rate)

        if now - self.last_log_time > 0.5:
            self.last_log_time = now
            nearest_text = 'inf' if self.last_nearest_x == float('inf') else f'{self.last_nearest_x:.2f}'

            if reason == 'normal':
                self.get_logger().info(
                    f'Move x={out_vx:.3f}, z={out_yaw_rate:.3f}, '
                    f'stop_count={self.last_stop_count}, roi_count={self.last_roi_count}, '
                    f'nearest_x={nearest_text}, clear_frames={self.clear_frame_count}'
                )
            else:
                self.get_logger().warn(
                    f'SAFE OVERRIDE {reason}: Move x=0.000, z=0.000, '
                    f'raw_x={self.last_vx:.3f}, raw_z={self.last_yaw_rate:.3f}, '
                    f'stop_count={self.last_stop_count}, roi_count={self.last_roi_count}, '
                    f'nearest_x={nearest_text}, clear_frames={self.clear_frame_count}'
                )

    def destroy_node(self):
        try:
            self.publish_move(0.0, 0.0)
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
        node.publish_move(0.0, 0.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

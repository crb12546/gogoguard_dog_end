#!/usr/bin/env python3
import math
import struct
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from unitree_go.msg import SportModeCmd

from .patrol_control import limit_planar_command, point_in_lateral_motion_roi


class UnitreeGoSafeCmdNode(Node):
    def __init__(self):
        super().__init__('unitree_go_safe_cmd_node')

        self.declare_parameter('cmd_topic', '/patrol_cmd')
        self.declare_parameter('go2_cmd_topic', '/go2_cmd')
        self.declare_parameter('pointcloud_topic', '/cloud_registered_body')

        self.declare_parameter('cmd_mode', 2)
        self.declare_parameter('gait_type', 0)
        self.declare_parameter('speed_level', 0)

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
        self.go2_cmd_topic = self.get_parameter('go2_cmd_topic').value
        self.pointcloud_topic = self.get_parameter('pointcloud_topic').value

        self.cmd_mode = int(self.get_parameter('cmd_mode').value)
        self.gait_type = int(self.get_parameter('gait_type').value)
        self.speed_level = int(self.get_parameter('speed_level').value)

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
        self.last_cloud_process_time = 0.0

        self.obstacle_stop = False
        self.stop_frame_count = 0
        self.clear_frame_count = 0
        self.last_stop_count = 0
        self.last_roi_count = 0
        self.last_nearest_x = float('inf')
        self.last_lateral_count = 0
        self.last_nearest_lateral_y = float('inf')

        self.pub = self.create_publisher(SportModeCmd, self.go2_cmd_topic, 10)
        self.create_subscription(Twist, self.cmd_topic, self.cmd_callback, 10)
        self.create_subscription(PointCloud2, self.pointcloud_topic, self.cloud_callback, 10)
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)

        self.get_logger().info(
            f'unitree_go_safe_cmd_node ACTIVE: {self.cmd_topic} -> {self.go2_cmd_topic}, '
            f'mode={self.cmd_mode}, gait_type={self.gait_type}, speed_level={self.speed_level}; '
            f'lateral swept ROI enabled'
        )

    def clamp(self, value, low, high):
        return max(low, min(high, value))

    def cmd_callback(self, msg):
        self.last_vx, self.last_vy, self.last_yaw_rate = limit_planar_command(
            msg.linear.x,
            msg.linear.y,
            msg.angular.z,
            self.max_vx,
            self.max_vy,
            self.max_yaw_rate,
        )
        self.last_cmd_time = time.time()

    def make_cmd(self, vx, vy, yaw_rate):
        cmd = SportModeCmd()
        cmd.mode = self.cmd_mode
        cmd.gait_type = self.gait_type
        cmd.speed_level = self.speed_level
        cmd.velocity[0] = float(vx)
        cmd.velocity[1] = float(vy)
        cmd.yaw_speed = float(yaw_rate)
        return cmd

    def publish_move(self, vx, vy, yaw_rate):
        self.pub.publish(self.make_cmd(vx, vy, yaw_rate))

    def get_xyz_offsets(self, msg):
        offsets = {}
        datatypes = {}
        for field in msg.fields:
            if field.name in ('x', 'y', 'z'):
                offsets[field.name] = field.offset
                datatypes[field.name] = field.datatype
        if not all(key in offsets for key in ('x', 'y', 'z')):
            return None
        if not all(datatypes[key] == PointField.FLOAT32 for key in ('x', 'y', 'z')):
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
        lateral_count = 0
        nearest_lateral_y = float('inf')

        for index in range(0, total_points, self.point_skip):
            base = index * point_step
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
        if self.obstacle_stop and self.clear_frame_count >= self.clear_frames:
            self.obstacle_stop = False
            self.get_logger().info(f'obstacle cleared after {self.clear_frame_count} clear frames, resume move')

    def timer_callback(self):
        now = time.time()
        out_vx = self.last_vx
        out_vy = self.last_vy
        out_yaw_rate = self.last_yaw_rate
        reason = 'normal'

        if now - self.last_cmd_time > self.cmd_timeout:
            out_vx, out_vy, out_yaw_rate = (0.0, 0.0, 0.0)
            reason = 'cmd_timeout'
        elif now - self.last_cloud_time > self.cloud_timeout:
            out_vx, out_vy, out_yaw_rate = (0.0, 0.0, 0.0)
            reason = 'cloud_timeout'
        elif self.obstacle_stop:
            out_vx, out_vy, out_yaw_rate = (0.0, 0.0, 0.0)
            reason = 'obstacle'

        self.publish_move(out_vx, out_vy, out_yaw_rate)

        if now - self.last_log_time > 0.5:
            self.last_log_time = now
            nearest_text = 'inf' if self.last_nearest_x == float('inf') else f'{self.last_nearest_x:.2f}'
            if reason == 'normal':
                self.get_logger().info(
                    f'Go2Cmd mode={self.cmd_mode} vx={out_vx:.3f}, '
                    f'vy={out_vy:.3f}, yaw={out_yaw_rate:.3f}, '
                    f'stop_count={self.last_stop_count}, roi_count={self.last_roi_count}, '
                    f'lateral_count={self.last_lateral_count}, '
                    f'nearest_x={nearest_text}, clear_frames={self.clear_frame_count}'
                )
            else:
                self.get_logger().warn(
                    f'SAFE OVERRIDE {reason}: Go2Cmd vx=0.000, vy=0.000, yaw=0.000, '
                    f'raw_x={self.last_vx:.3f}, raw_y={self.last_vy:.3f}, '
                    f'raw_z={self.last_yaw_rate:.3f}, '
                    f'stop_count={self.last_stop_count}, roi_count={self.last_roi_count}, '
                    f'lateral_count={self.last_lateral_count}, '
                    f'nearest_x={nearest_text}, clear_frames={self.clear_frame_count}'
                )

    def destroy_node(self):
        try:
            self.publish_move(0.0, 0.0, 0.0)
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UnitreeGoSafeCmdNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Ctrl+C, publish zero Go2Cmd')
        node.publish_move(0.0, 0.0, 0.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

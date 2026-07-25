#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


def quat_to_rot(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1.0 - 2.0*y*y - 2.0*z*z, 2.0*x*y - 2.0*z*w,       2.0*x*z + 2.0*y*w],
        [2.0*x*y + 2.0*z*w,       1.0 - 2.0*x*x - 2.0*z*z, 2.0*y*z - 2.0*x*w],
        [2.0*x*z - 2.0*y*w,       2.0*y*z + 2.0*x*w,       1.0 - 2.0*x*x - 2.0*y*y],
    ], dtype=np.float64)


def rot_y(deg):
    th = math.radians(deg)
    c = math.cos(th)
    s = math.sin(th)
    return np.array([
        [ c, 0.0,  s],
        [0.0, 1.0, 0.0],
        [-s, 0.0,  c],
    ], dtype=np.float64)


def yaw_from_rot(R):
    return math.atan2(R[1, 0], R[0, 0])


def quat_from_yaw(yaw):
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


class OdomToTFLevel2D(Node):
    def __init__(self):
        super().__init__("odom_to_tf_odom_level_2d")

        self.declare_parameter("odom_topic", "/Odometry")
        self.declare_parameter("parent_frame", "odom")
        self.declare_parameter("child_frame", "base_link")
        self.declare_parameter("level_pitch_deg", 13.0)

        self.odom_topic = self.get_parameter("odom_topic").value
        self.parent_frame = self.get_parameter("parent_frame").value
        self.child_frame = self.get_parameter("child_frame").value
        self.level_pitch_deg = float(self.get_parameter("level_pitch_deg").value)

        self.R_level = rot_y(self.level_pitch_deg)
        self.br = TransformBroadcaster(self)

        self.sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.cb,
            50,
        )

        self.get_logger().info(f"odom_topic: {self.odom_topic}")
        self.get_logger().info(f"publishing TF: {self.parent_frame} -> {self.child_frame}")
        self.get_logger().info(f"level_pitch_deg: {self.level_pitch_deg}")
        self.get_logger().info("Nav2/AMCL mode: odom -> base_link only, no map -> base_link")

    def cb(self, msg):
        p_raw = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        ], dtype=np.float64)

        R_raw = quat_to_rot(msg.pose.pose.orientation)

        # 与离线地图和在线点云保持一致：先绕 Y 轴做水平校正
        p_level = self.R_level @ p_raw
        R_level_body = self.R_level @ R_raw

        # Nav2 / AMCL 使用 2D TF：只保留 x, y, yaw
        yaw = yaw_from_rot(R_level_body)
        qx, qy, qz, qw = quat_from_yaw(yaw)

        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self.parent_frame
        t.child_frame_id = self.child_frame

        t.transform.translation.x = float(p_level[0])
        t.transform.translation.y = float(p_level[1])
        t.transform.translation.z = 0.0

        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.br.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToTFLevel2D()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

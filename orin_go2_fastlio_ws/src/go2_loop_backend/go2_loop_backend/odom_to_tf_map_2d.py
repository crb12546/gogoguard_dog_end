#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


def yaw_from_quat(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def quat_from_yaw(yaw):
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


class OdomToTF2D(Node):
    def __init__(self):
        super().__init__("odom_to_tf_map_2d")

        self.declare_parameter("odom_topic", "/Odometry")
        self.declare_parameter("parent_frame", "map")
        self.declare_parameter("child_frame", "base_link")
        self.declare_parameter("zero_z", True)

        self.odom_topic = self.get_parameter("odom_topic").value
        self.parent_frame = self.get_parameter("parent_frame").value
        self.child_frame = self.get_parameter("child_frame").value
        self.zero_z = bool(self.get_parameter("zero_z").value)

        self.br = TransformBroadcaster(self)

        self.sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.cb,
            50,
        )

        self.get_logger().info(f"odom_topic: {self.odom_topic}")
        self.get_logger().info(f"publishing planar TF: {self.parent_frame} -> {self.child_frame}")
        self.get_logger().info("roll = 0, pitch = 0, z = 0, yaw kept from /Odometry")

    def cb(self, msg):
        yaw = yaw_from_quat(msg.pose.pose.orientation)
        qx, qy, qz, qw = quat_from_yaw(yaw)

        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self.parent_frame
        t.child_frame_id = self.child_frame

        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = 0.0 if self.zero_z else msg.pose.pose.position.z

        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.br.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToTF2D()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Publish a deterministic L-shaped /Odometry route for recorder smoke tests."""

import math
import time

import rclpy
from nav_msgs.msg import Odometry


def publish_pose(publisher, node, x, y, yaw):
    message = Odometry()
    message.header.stamp = node.get_clock().now().to_msg()
    message.header.frame_id = 'camera_init'
    message.child_frame_id = 'body'
    message.pose.pose.position.x = x
    message.pose.pose.position.y = y
    message.pose.pose.orientation.z = math.sin(yaw / 2.0)
    message.pose.pose.orientation.w = math.cos(yaw / 2.0)
    publisher.publish(message)
    rclpy.spin_once(node, timeout_sec=0.0)
    time.sleep(0.01)


def main():
    rclpy.init()
    node = rclpy.create_node('fake_odom_route_publisher')
    publisher = node.create_publisher(Odometry, '/Odometry', 10)
    time.sleep(0.8)

    for _ in range(20):
        publish_pose(publisher, node, 0.0, 0.0, 0.0)

    for index in range(1, 51):
        x = index * 0.02
        y = 0.012 * math.sin(index * 0.31)
        if index == 25:
            y += 0.20
        publish_pose(publisher, node, x, y, 0.0)

    for degrees in range(10, 100, 10):
        publish_pose(publisher, node, 1.0, 0.0, math.radians(degrees))
        publish_pose(publisher, node, 1.0, 0.0, math.radians(degrees))

    for index in range(1, 51):
        publish_pose(
            publisher,
            node,
            1.0 + 0.012 * math.sin(index * 0.27),
            index * 0.02,
            math.pi / 2.0,
        )

    time.sleep(0.5)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomToTF(Node):
    def __init__(self):
        super().__init__("odom_to_tf_map")

        self.declare_parameter("odom_topic", "/Odometry")
        self.declare_parameter("parent_frame", "map")
        self.declare_parameter("child_frame", "base_link")

        self.odom_topic = self.get_parameter("odom_topic").value
        self.parent_frame = self.get_parameter("parent_frame").value
        self.child_frame = self.get_parameter("child_frame").value

        self.br = TransformBroadcaster(self)

        self.sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.cb,
            50,
        )

        self.get_logger().info(f"odom_topic: {self.odom_topic}")
        self.get_logger().info(f"publishing TF: {self.parent_frame} -> {self.child_frame}")

    def cb(self, msg):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self.parent_frame
        t.child_frame_id = self.child_frame

        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z

        t.transform.rotation = msg.pose.pose.orientation

        self.br.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

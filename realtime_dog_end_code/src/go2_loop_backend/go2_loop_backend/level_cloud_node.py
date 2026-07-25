#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2


class LevelCloudNode(Node):
    def __init__(self):
        super().__init__("level_cloud_node")

        self.declare_parameter("input_topic", "/cloud_registered_body")
        self.declare_parameter("output_topic", "/cloud_registered_body_level")
        self.declare_parameter("output_frame", "base_link")
        self.declare_parameter("pitch_deg", 12.3)
        self.declare_parameter("point_stride", 1)

        self.input_topic = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.output_frame = self.get_parameter("output_frame").value
        self.pitch_deg = float(self.get_parameter("pitch_deg").value)
        self.point_stride = max(1, int(self.get_parameter("point_stride").value))

        th = math.radians(self.pitch_deg)
        c = math.cos(th)
        s = math.sin(th)

        # 绕 Y 轴旋转，把雷达倾斜平面校正到 GO2 水平平面
        self.R = np.array([
            [ c, 0.0,  s],
            [0.0, 1.0, 0.0],
            [-s, 0.0,  c],
        ], dtype=np.float64)

        self.sub = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self.cb,
            10,
        )

        self.pub = self.create_publisher(
            PointCloud2,
            self.output_topic,
            10,
        )

        self.get_logger().info(f"input_topic: {self.input_topic}")
        self.get_logger().info(f"output_topic: {self.output_topic}")
        self.get_logger().info(f"output_frame: {self.output_frame}")
        self.get_logger().info(f"pitch_deg: {self.pitch_deg}")
        self.get_logger().info(f"point_stride: {self.point_stride}")

    def cb(self, msg: PointCloud2):
        field_names = [f.name for f in msg.fields]
        has_intensity = "intensity" in field_names

        if has_intensity:
            read_fields = ("x", "y", "z", "intensity")
        else:
            read_fields = ("x", "y", "z")

        out_points = []

        for i, p in enumerate(point_cloud2.read_points(msg, field_names=read_fields, skip_nans=True)):
            if i % self.point_stride != 0:
                continue

            xyz = np.array([float(p[0]), float(p[1]), float(p[2])], dtype=np.float64)
            xyz2 = self.R @ xyz

            if has_intensity:
                out_points.append((float(xyz2[0]), float(xyz2[1]), float(xyz2[2]), float(p[3])))
            else:
                out_points.append((float(xyz2[0]), float(xyz2[1]), float(xyz2[2])))

        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = self.output_frame

        if has_intensity:
            fields = [
                PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
            ]
        else:
            fields = [
                PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            ]

        out_msg = point_cloud2.create_cloud(header, fields, out_points)
        self.pub.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LevelCloudNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

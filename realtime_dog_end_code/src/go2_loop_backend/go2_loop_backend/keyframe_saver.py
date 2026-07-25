#!/usr/bin/env python3
import math
import os
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


def quat_to_yaw(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_diff(a: float, b: float) -> float:
    d = a - b
    while d > math.pi:
        d -= 2.0 * math.pi
    while d < -math.pi:
        d += 2.0 * math.pi
    return abs(d)


class KeyframeSaver(Node):
    def __init__(self):
        super().__init__("keyframe_saver")

        self.declare_parameter("odom_topic", "/Odometry")
        self.declare_parameter("cloud_topic", "/cloud_registered_body")
        self.declare_parameter("output_dir", "/home/unitree/go2_fastlio_ws/maps/loop_backend/keyframes")
        self.declare_parameter("distance_thresh", 1.0)
        self.declare_parameter("yaw_thresh_deg", 10.0)
        self.declare_parameter("point_stride", 1)

        self.odom_topic = self.get_parameter("odom_topic").value
        self.cloud_topic = self.get_parameter("cloud_topic").value
        self.output_dir = self.get_parameter("output_dir").value
        self.distance_thresh = float(self.get_parameter("distance_thresh").value)
        self.yaw_thresh = math.radians(float(self.get_parameter("yaw_thresh_deg").value))
        self.point_stride = max(1, int(self.get_parameter("point_stride").value))

        os.makedirs(self.output_dir, exist_ok=True)
        self.pose_file_path = os.path.join(self.output_dir, "poses_raw.txt")
        self.pose_file = open(self.pose_file_path, "w")
        self.pose_file.write("# idx stamp x y z qx qy qz qw yaw pcd_file\n")
        self.pose_file.flush()

        self.latest_odom: Optional[Odometry] = None
        self.last_key_pose: Optional[Tuple[float, float, float, float]] = None
        self.key_idx = 0

        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 50)
        self.create_subscription(PointCloud2, self.cloud_topic, self.cloud_callback, 10)

        self.get_logger().info(f"odom_topic: {self.odom_topic}")
        self.get_logger().info(f"cloud_topic: {self.cloud_topic}")
        self.get_logger().info(f"output_dir: {self.output_dir}")
        self.get_logger().info(f"distance_thresh: {self.distance_thresh} m")
        self.get_logger().info(f"yaw_thresh: {math.degrees(self.yaw_thresh):.1f} deg")
        self.get_logger().info(f"point_stride: {self.point_stride}")

    def odom_callback(self, msg: Odometry):
        self.latest_odom = msg

    def cloud_callback(self, cloud: PointCloud2):
        if self.latest_odom is None:
            return

        pose = self.latest_odom.pose.pose
        x = pose.position.x
        y = pose.position.y
        z = pose.position.z
        q = pose.orientation
        yaw = quat_to_yaw(q)

        if self.last_key_pose is not None:
            lx, ly, lz, lyaw = self.last_key_pose
            dist = math.sqrt((x - lx) ** 2 + (y - ly) ** 2 + (z - lz) ** 2)
            dyaw = yaw_diff(yaw, lyaw)

            if dist < self.distance_thresh and dyaw < self.yaw_thresh:
                return

        stamp = cloud.header.stamp.sec + cloud.header.stamp.nanosec * 1e-9
        pcd_name = f"keyframe_{self.key_idx:06d}.pcd"
        pcd_path = os.path.join(self.output_dir, pcd_name)

        n_points = self.write_pcd_ascii(cloud, pcd_path)

        self.pose_file.write(
            f"{self.key_idx} {stamp:.9f} "
            f"{x:.6f} {y:.6f} {z:.6f} "
            f"{q.x:.9f} {q.y:.9f} {q.z:.9f} {q.w:.9f} "
            f"{yaw:.9f} {pcd_name}\n"
        )
        self.pose_file.flush()

        self.last_key_pose = (x, y, z, yaw)

        self.get_logger().info(
            f"Saved keyframe {self.key_idx:06d}: "
            f"points={n_points}, pose=({x:.2f}, {y:.2f}, {z:.2f}, yaw={math.degrees(yaw):.1f})"
        )

        self.key_idx += 1

    def write_pcd_ascii(self, cloud: PointCloud2, path: str) -> int:
        field_names_available = [f.name for f in cloud.fields]
        use_intensity = "intensity" in field_names_available

        if use_intensity:
            fields = ("x", "y", "z", "intensity")
        else:
            fields = ("x", "y", "z")

        points = []
        for i, p in enumerate(point_cloud2.read_points(cloud, field_names=fields, skip_nans=True)):
            if i % self.point_stride != 0:
                continue
            points.append(p)

        with open(path, "w") as f:
            if use_intensity:
                f.write("# .PCD v0.7 - Point Cloud Data file format\n")
                f.write("VERSION 0.7\n")
                f.write("FIELDS x y z intensity\n")
                f.write("SIZE 4 4 4 4\n")
                f.write("TYPE F F F F\n")
                f.write("COUNT 1 1 1 1\n")
                f.write(f"WIDTH {len(points)}\n")
                f.write("HEIGHT 1\n")
                f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
                f.write(f"POINTS {len(points)}\n")
                f.write("DATA ascii\n")
                for p in points:
                    f.write(f"{p[0]:.6f} {p[1]:.6f} {p[2]:.6f} {float(p[3]):.6f}\n")
            else:
                f.write("# .PCD v0.7 - Point Cloud Data file format\n")
                f.write("VERSION 0.7\n")
                f.write("FIELDS x y z\n")
                f.write("SIZE 4 4 4\n")
                f.write("TYPE F F F\n")
                f.write("COUNT 1 1 1\n")
                f.write(f"WIDTH {len(points)}\n")
                f.write("HEIGHT 1\n")
                f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
                f.write(f"POINTS {len(points)}\n")
                f.write("DATA ascii\n")
                for p in points:
                    f.write(f"{p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n")

        return len(points)

    def destroy_node(self):
        try:
            self.pose_file.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = KeyframeSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

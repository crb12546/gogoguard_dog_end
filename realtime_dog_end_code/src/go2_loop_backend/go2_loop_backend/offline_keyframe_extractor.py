#!/usr/bin/env python3
import argparse
import math
import os
import sqlite3
from typing import Optional, Tuple

import rclpy
from rclpy.serialization import deserialize_message
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


def write_pcd_ascii(cloud: PointCloud2, path: str, point_stride: int) -> int:
    field_names = [f.name for f in cloud.fields]
    use_intensity = "intensity" in field_names

    if use_intensity:
        fields = ("x", "y", "z", "intensity")
    else:
        fields = ("x", "y", "z")

    points = []
    for i, p in enumerate(point_cloud2.read_points(cloud, field_names=fields, skip_nans=True)):
        if i % point_stride != 0:
            continue
        points.append(p)

    with open(path, "w") as f:
        f.write("# .PCD v0.7 - Point Cloud Data file format\n")
        f.write("VERSION 0.7\n")

        if use_intensity:
            f.write("FIELDS x y z intensity\n")
            f.write("SIZE 4 4 4 4\n")
            f.write("TYPE F F F F\n")
            f.write("COUNT 1 1 1 1\n")
        else:
            f.write("FIELDS x y z\n")
            f.write("SIZE 4 4 4\n")
            f.write("TYPE F F F\n")
            f.write("COUNT 1 1 1\n")

        f.write(f"WIDTH {len(points)}\n")
        f.write("HEIGHT 1\n")
        f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        f.write(f"POINTS {len(points)}\n")
        f.write("DATA ascii\n")

        if use_intensity:
            for p in points:
                f.write(f"{p[0]:.6f} {p[1]:.6f} {p[2]:.6f} {float(p[3]):.6f}\n")
        else:
            for p in points:
                f.write(f"{p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n")

    return len(points)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", required=True, help="Path to rosbag directory or .db3 file")
    parser.add_argument("--out", default="/home/unitree/go2_fastlio_ws/maps/loop_backend/keyframes")
    parser.add_argument("--odom_topic", default="/Odometry")
    parser.add_argument("--cloud_topic", default="/cloud_registered_body")
    parser.add_argument("--distance_thresh", type=float, default=1.0)
    parser.add_argument("--yaw_thresh_deg", type=float, default=10.0)
    parser.add_argument("--point_stride", type=int, default=2)
    args = parser.parse_args()

    if os.path.isdir(args.bag):
        db_files = [f for f in os.listdir(args.bag) if f.endswith(".db3")]
        if not db_files:
            raise RuntimeError(f"No .db3 file found in {args.bag}")
        db_path = os.path.join(args.bag, sorted(db_files)[0])
    else:
        db_path = args.bag

    os.makedirs(args.out, exist_ok=True)
    pose_path = os.path.join(args.out, "poses_raw.txt")

    conn = sqlite3.connect(db_path)
    cur = conn.cursor()

    topic_rows = cur.execute("SELECT id, name, type FROM topics").fetchall()
    topic_map = {row[0]: (row[1], row[2]) for row in topic_rows}

    print("[INFO] Topics in bag:")
    for tid, (name, typ) in topic_map.items():
        print(f"  id={tid}, name={name}, type={typ}")

    odom_topic_id = None
    cloud_topic_id = None

    for tid, (name, typ) in topic_map.items():
        if name == args.odom_topic:
            odom_topic_id = tid
        if name == args.cloud_topic:
            cloud_topic_id = tid

    if odom_topic_id is None:
        raise RuntimeError(f"Cannot find odom topic: {args.odom_topic}")
    if cloud_topic_id is None:
        raise RuntimeError(f"Cannot find cloud topic: {args.cloud_topic}")

    latest_odom: Optional[Odometry] = None
    last_key_pose: Optional[Tuple[float, float, float, float]] = None
    key_idx = 0
    yaw_thresh = math.radians(args.yaw_thresh_deg)

    with open(pose_path, "w") as pose_file:
        pose_file.write("# idx stamp x y z qx qy qz qw yaw pcd_file\n")

        query = """
            SELECT topic_id, timestamp, data
            FROM messages
            WHERE topic_id = ? OR topic_id = ?
            ORDER BY timestamp ASC
        """

        for topic_id, timestamp, data in cur.execute(query, (odom_topic_id, cloud_topic_id)):
            if topic_id == odom_topic_id:
                latest_odom = deserialize_message(data, Odometry)
                continue

            if topic_id != cloud_topic_id:
                continue

            if latest_odom is None:
                continue

            cloud = deserialize_message(data, PointCloud2)

            pose = latest_odom.pose.pose
            x = pose.position.x
            y = pose.position.y
            z = pose.position.z
            q = pose.orientation
            yaw = quat_to_yaw(q)

            if last_key_pose is not None:
                lx, ly, lz, lyaw = last_key_pose
                dist = math.sqrt((x - lx) ** 2 + (y - ly) ** 2 + (z - lz) ** 2)
                dyaw = yaw_diff(yaw, lyaw)

                if dist < args.distance_thresh and dyaw < yaw_thresh:
                    continue

            stamp = cloud.header.stamp.sec + cloud.header.stamp.nanosec * 1e-9
            pcd_name = f"keyframe_{key_idx:06d}.pcd"
            pcd_path = os.path.join(args.out, pcd_name)

            n_points = write_pcd_ascii(cloud, pcd_path, max(1, args.point_stride))

            pose_file.write(
                f"{key_idx} {stamp:.9f} "
                f"{x:.6f} {y:.6f} {z:.6f} "
                f"{q.x:.9f} {q.y:.9f} {q.z:.9f} {q.w:.9f} "
                f"{yaw:.9f} {pcd_name}\n"
            )
            pose_file.flush()

            last_key_pose = (x, y, z, yaw)

            print(
                f"[KEYFRAME] {key_idx:06d} "
                f"points={n_points} "
                f"pose=({x:.2f}, {y:.2f}, {z:.2f}, yaw={math.degrees(yaw):.1f})"
            )

            key_idx += 1

    conn.close()
    print(f"[DONE] saved keyframes: {key_idx}")
    print(f"[DONE] poses: {pose_path}")


if __name__ == "__main__":
    rclpy.init(args=None)
    main()
    rclpy.shutdown()

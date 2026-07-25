#!/usr/bin/env python3
import argparse
import os
import sqlite3
import numpy as np

import rclpy
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


def voxel_downsample(points, voxel):
    if len(points) == 0 or voxel <= 0:
        return points
    keys = np.floor(points / voxel).astype(np.int64)
    _, idx = np.unique(keys, axis=0, return_index=True)
    return points[idx]


def write_pcd_xyz(path, points):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as f:
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
            f.write(f"{p[0]:.4f} {p[1]:.4f} {p[2]:.4f}\n")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", required=True)
    parser.add_argument("--topic", default="/cloud_registered")
    parser.add_argument("--out", required=True)
    parser.add_argument("--frame_stride", type=int, default=3)
    parser.add_argument("--point_stride", type=int, default=2)
    parser.add_argument("--voxel", type=float, default=0.08)
    args = parser.parse_args()

    if os.path.isdir(args.bag):
        db_files = [f for f in os.listdir(args.bag) if f.endswith(".db3")]
        if not db_files:
            raise RuntimeError(f"No .db3 file found in {args.bag}")
        db_path = os.path.join(args.bag, sorted(db_files)[0])
    else:
        db_path = args.bag

    conn = sqlite3.connect(db_path)
    cur = conn.cursor()

    topics = cur.execute("SELECT id, name, type FROM topics").fetchall()
    topic_id = None

    print("[INFO] topics:")
    for tid, name, typ in topics:
        print(f"  {tid}: {name} [{typ}]")
        if name == args.topic:
            topic_id = tid

    if topic_id is None:
        raise RuntimeError(f"Cannot find topic: {args.topic}")

    rows = cur.execute(
        "SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp ASC",
        (topic_id,)
    )

    all_points = []
    frame_count = 0
    used_frame_count = 0

    for _, data in rows:
        if frame_count % args.frame_stride != 0:
            frame_count += 1
            continue

        msg = deserialize_message(data, PointCloud2)
        pts = []

        for i, p in enumerate(point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)):
            if i % args.point_stride != 0:
                continue
            pts.append([float(p[0]), float(p[1]), float(p[2])])

        if pts:
            all_points.append(np.asarray(pts, dtype=np.float64))

        if used_frame_count % 20 == 0:
            print(f"[INFO] used frame {used_frame_count}, total input frame {frame_count}")

        used_frame_count += 1
        frame_count += 1

    conn.close()

    if not all_points:
        raise RuntimeError("No points exported")

    merged = np.vstack(all_points)
    print(f"[INFO] merged points before voxel: {len(merged)}")

    merged = voxel_downsample(merged, args.voxel)
    print(f"[INFO] points after voxel {args.voxel}: {len(merged)}")

    write_pcd_xyz(args.out, merged)
    print(f"[DONE] saved: {args.out}")


if __name__ == "__main__":
    rclpy.init(args=None)
    main()
    rclpy.shutdown()

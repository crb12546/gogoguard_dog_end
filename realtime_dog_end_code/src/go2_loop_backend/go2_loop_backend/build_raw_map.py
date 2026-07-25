#!/usr/bin/env python3
import argparse
import math
import os
import numpy as np


def quat_to_rot(qx, qy, qz, qw):
    x, y, z, w = qx, qy, qz, qw
    return np.array([
        [1 - 2*y*y - 2*z*z,     2*x*y - 2*z*w,         2*x*z + 2*y*w],
        [2*x*y + 2*z*w,         1 - 2*x*x - 2*z*z,     2*y*z - 2*x*w],
        [2*x*z - 2*y*w,         2*y*z + 2*x*w,         1 - 2*x*x - 2*y*y],
    ], dtype=np.float64)


def load_pcd_xyz(path):
    points = []
    data = False
    fields = None

    with open(path, "r") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            if line.startswith("FIELDS"):
                fields = line.split()[1:]
            elif line.startswith("DATA"):
                data = True
                continue
            elif data:
                a = line.split()
                if len(a) >= 3:
                    points.append([float(a[0]), float(a[1]), float(a[2])])

    return np.asarray(points, dtype=np.float64)


def voxel_downsample(points, voxel):
    if len(points) == 0 or voxel <= 0:
        return points
    keys = np.floor(points / voxel).astype(np.int64)
    _, idx = np.unique(keys, axis=0, return_index=True)
    return points[idx]


def write_pcd_xyz(path, points):
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
    parser.add_argument("--keyframes", required=True)
    parser.add_argument("--out", default="/home/unitree/go2_fastlio_ws/maps/loop_backend/raw_map.pcd")
    parser.add_argument("--voxel", type=float, default=0.10)
    args = parser.parse_args()

    poses_path = os.path.join(args.keyframes, "poses_raw.txt")
    all_points = []

    with open(poses_path, "r") as f:
        for line in f:
            if line.startswith("#") or not line.strip():
                continue

            a = line.split()
            idx = int(a[0])
            x, y, z = map(float, a[2:5])
            qx, qy, qz, qw = map(float, a[5:9])
            pcd_name = a[10]

            pcd_path = os.path.join(args.keyframes, pcd_name)
            pts = load_pcd_xyz(pcd_path)

            if len(pts) == 0:
                continue

            R = quat_to_rot(qx, qy, qz, qw)
            t = np.array([x, y, z], dtype=np.float64)
            pts_w = pts @ R.T + t

            all_points.append(pts_w)

            if idx % 10 == 0:
                print(f"[INFO] loaded keyframe {idx}, points={len(pts)}")

    if not all_points:
        raise RuntimeError("No points loaded")

    points = np.vstack(all_points)
    print(f"[INFO] merged raw points: {len(points)}")

    points = voxel_downsample(points, args.voxel)
    print(f"[INFO] after voxel {args.voxel}: {len(points)}")

    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    write_pcd_xyz(args.out, points)
    print(f"[DONE] saved raw map: {args.out}")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
import argparse
import os
import math
import numpy as np


def load_poses(path):
    poses = {}
    with open(path, "r") as f:
        for line in f:
            if line.startswith("#") or not line.strip():
                continue
            a = line.split()
            idx = int(a[0])
            poses[idx] = {
                "idx": idx,
                "stamp": float(a[1]),
                "x": float(a[2]),
                "y": float(a[3]),
                "z": float(a[4]),
                "qx": float(a[5]),
                "qy": float(a[6]),
                "qz": float(a[7]),
                "qw": float(a[8]),
                "yaw": float(a[9]),
                "pcd": a[10],
            }
    return poses


def quat_to_rot(qx, qy, qz, qw):
    x, y, z, w = qx, qy, qz, qw
    return np.array([
        [1 - 2*y*y - 2*z*z,     2*x*y - 2*z*w,         2*x*z + 2*y*w],
        [2*x*y + 2*z*w,         1 - 2*x*x - 2*z*z,     2*y*z - 2*x*w],
        [2*x*z - 2*y*w,         2*y*z + 2*x*w,         1 - 2*x*x - 2*y*y],
    ], dtype=np.float64)


def rotz(yaw):
    c = math.cos(yaw)
    s = math.sin(yaw)
    return np.array([
        [c, -s, 0],
        [s,  c, 0],
        [0,  0, 1],
    ], dtype=np.float64)


def load_pcd_xyz(path):
    pts = []
    data = False
    with open(path, "r") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            if line.startswith("DATA"):
                data = True
                continue
            if data:
                a = line.split()
                if len(a) >= 3:
                    pts.append([float(a[0]), float(a[1]), float(a[2])])
    return np.asarray(pts, dtype=np.float64)


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
    parser.add_argument("--raw", required=True)
    parser.add_argument("--opt", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--voxel", type=float, default=0.10)
    args = parser.parse_args()

    raw_poses = load_poses(args.raw)
    opt_poses = load_poses(args.opt)

    all_points = []

    for idx in sorted(raw_poses.keys()):
        if idx not in opt_poses:
            continue

        raw = raw_poses[idx]
        opt = opt_poses[idx]

        pcd_path = os.path.join(args.keyframes, raw["pcd"])
        pts = load_pcd_xyz(pcd_path)

        if len(pts) == 0:
            continue

        # 原始完整姿态：包含 roll/pitch/yaw
        R_raw = quat_to_rot(raw["qx"], raw["qy"], raw["qz"], raw["qw"])

        # 只用 PGO 优化出来的 yaw 去修正原始 yaw
        yaw_delta = opt["yaw"] - raw["yaw"]
        R_delta = rotz(yaw_delta)

        # 保留原始 roll/pitch，只修正全局 yaw
        R_corr = R_delta @ R_raw

        # x/y 用优化结果；z 暂时保留原始 z
        t_corr = np.array([opt["x"], opt["y"], raw["z"]], dtype=np.float64)

        pts_w = pts @ R_corr.T + t_corr
        all_points.append(pts_w)

        if idx % 10 == 0:
            print(f"[INFO] keyframe {idx:06d}, points={len(pts)}")

    if not all_points:
        raise RuntimeError("No points loaded")

    merged = np.vstack(all_points)
    print(f"[INFO] merged points: {len(merged)}")

    merged = voxel_downsample(merged, args.voxel)
    print(f"[INFO] after voxel {args.voxel}: {len(merged)}")

    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    write_pcd_xyz(args.out, merged)
    print(f"[DONE] saved: {args.out}")


if __name__ == "__main__":
    main()

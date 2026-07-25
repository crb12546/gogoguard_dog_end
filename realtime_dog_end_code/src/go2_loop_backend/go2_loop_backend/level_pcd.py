#!/usr/bin/env python3
import argparse
import math
import os
import numpy as np


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


def write_pcd_xyz(path, pts):
    os.makedirs(os.path.dirname(path), exist_ok=True)

    with open(path, "w") as f:
        f.write("# .PCD v0.7 - Point Cloud Data file format\n")
        f.write("VERSION 0.7\n")
        f.write("FIELDS x y z\n")
        f.write("SIZE 4 4 4\n")
        f.write("TYPE F F F\n")
        f.write("COUNT 1 1 1\n")
        f.write(f"WIDTH {len(pts)}\n")
        f.write("HEIGHT 1\n")
        f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        f.write(f"POINTS {len(pts)}\n")
        f.write("DATA ascii\n")

        for x, y, z in pts:
            f.write(f"{x:.4f} {y:.4f} {z:.4f}\n")


def rot_x(deg):
    th = math.radians(deg)
    c = math.cos(th)
    s = math.sin(th)
    return np.array([
        [1.0, 0.0, 0.0],
        [0.0, c, -s],
        [0.0, s,  c],
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


def rot_z(deg):
    th = math.radians(deg)
    c = math.cos(th)
    s = math.sin(th)
    return np.array([
        [c, -s, 0.0],
        [s,  c, 0.0],
        [0.0, 0.0, 1.0],
    ], dtype=np.float64)


def voxel_downsample(points, voxel):
    if len(points) == 0 or voxel <= 0:
        return points

    keys = np.floor(points / voxel).astype(np.int64)
    _, idx = np.unique(keys, axis=0, return_index=True)
    return points[idx]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--in_pcd", required=True)
    parser.add_argument("--out_pcd", required=True)

    # 默认固定为实测水平校正角：Y 轴 +12.3°
    parser.add_argument("--roll_deg", type=float, default=0.0)
    parser.add_argument("--pitch_deg", type=float, default=13.0)
    parser.add_argument("--yaw_deg", type=float, default=0.0)

    # 默认把低位高度平移到 0 附近，便于后续 2D 投影
    parser.add_argument("--shift_z_to_percentile", type=float, default=5.0)

    # 默认不额外降采样；需要时手动传 --voxel 0.05
    parser.add_argument("--voxel", type=float, default=0.0)

    args = parser.parse_args()

    pts = load_pcd_xyz(args.in_pcd)
    if len(pts) == 0:
        raise RuntimeError(f"No points loaded from {args.in_pcd}")

    print(f"[INFO] loaded points: {len(pts)}")
    print(f"[INFO] input: {args.in_pcd}")
    print(f"[INFO] leveling rotation: roll={args.roll_deg}, pitch(Y)={args.pitch_deg}, yaw={args.yaw_deg}")

    R = rot_z(args.yaw_deg) @ rot_y(args.pitch_deg) @ rot_x(args.roll_deg)
    pts_out = pts @ R.T

    if args.shift_z_to_percentile is not None:
        z_ref = np.percentile(pts_out[:, 2], args.shift_z_to_percentile)
        pts_out[:, 2] -= z_ref
        print(f"[INFO] shifted z by percentile {args.shift_z_to_percentile}: {z_ref:.4f}")

    if args.voxel > 0:
        before = len(pts_out)
        pts_out = voxel_downsample(pts_out, args.voxel)
        print(f"[INFO] voxel downsample {args.voxel}: {before} -> {len(pts_out)}")

    print(f"[INFO] output z range: {pts_out[:,2].min():.3f} ~ {pts_out[:,2].max():.3f}")

    write_pcd_xyz(args.out_pcd, pts_out)
    print(f"[DONE] saved: {args.out_pcd}")


if __name__ == "__main__":
    main()

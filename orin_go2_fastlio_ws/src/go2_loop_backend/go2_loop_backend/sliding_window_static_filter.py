#!/usr/bin/env python3
import argparse
import os
from collections import defaultdict

import numpy as np


def quat_to_rot(qx, qy, qz, qw):
    x, y, z, w = qx, qy, qz, qw
    return np.array([
        [1 - 2*y*y - 2*z*z,     2*x*y - 2*z*w,         2*x*z + 2*y*w],
        [2*x*y + 2*z*w,         1 - 2*x*x - 2*z*z,     2*y*z - 2*x*w],
        [2*x*z - 2*y*w,         2*y*z + 2*x*w,         1 - 2*x*x - 2*y*y],
    ], dtype=np.float64)


def load_poses(path):
    poses = []
    with open(path, "r") as f:
        for line in f:
            if line.startswith("#") or not line.strip():
                continue
            a = line.split()
            poses.append({
                "idx": int(a[0]),
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
            })
    return poses


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


def voxel_keys(points, voxel):
    return np.floor(points / voxel).astype(np.int64)


def tuple_keys(keys_np):
    return [tuple(k) for k in keys_np]


def voxel_downsample(points, voxel):
    if len(points) == 0 or voxel <= 0:
        return points

    keys = np.floor(points / voxel).astype(np.int64)
    _, idx = np.unique(keys, axis=0, return_index=True)
    return points[idx]


def max_neighbor_support(key, support_map, neighbor_radius):
    if neighbor_radius <= 0:
        return support_map.get(key, 0)

    kx, ky, kz = key
    best = 0

    for dx in range(-neighbor_radius, neighbor_radius + 1):
        for dy in range(-neighbor_radius, neighbor_radius + 1):
            for dz in range(-neighbor_radius, neighbor_radius + 1):
                nk = (kx + dx, ky + dy, kz + dz)
                v = support_map.get(nk, 0)
                if v > best:
                    best = v

    return best


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--keyframes", required=True)
    parser.add_argument("--out_static", default="/home/unitree/go2_fastlio_ws/maps/loop_backend/static_map_sliding.pcd")
    parser.add_argument("--out_removed", default="/home/unitree/go2_fastlio_ws/maps/loop_backend/dynamic_removed_sliding.pcd")

    parser.add_argument("--voxel_size", type=float, default=0.05)
    parser.add_argument("--output_voxel", type=float, default=0.04)

    parser.add_argument("--window_radius", type=int, default=2)
    parser.add_argument("--min_frame_support", type=int, default=2)
    parser.add_argument("--neighbor_radius_vox", type=int, default=1)

    parser.add_argument("--z_min", type=float, default=-0.30)
    parser.add_argument("--z_max", type=float, default=2.50)
    parser.add_argument("--ego_radius", type=float, default=0.50)
    parser.add_argument("--max_range", type=float, default=25.0)

    args = parser.parse_args()

    poses_path = os.path.join(args.keyframes, "poses_raw.txt")
    poses = load_poses(poses_path)
    n = len(poses)

    print(f"[INFO] loaded poses: {n}")
    print(f"[INFO] keyframes: {args.keyframes}")
    print(f"[INFO] voxel_size: {args.voxel_size}")
    print(f"[INFO] output_voxel: {args.output_voxel}")
    print(f"[INFO] window_radius: {args.window_radius}")
    print(f"[INFO] min_frame_support: {args.min_frame_support}")
    print(f"[INFO] neighbor_radius_vox: {args.neighbor_radius_vox}")

    frame_points = []
    frame_keys = []
    frame_unique_keys = []

    total_raw_points = 0
    total_used_points = 0

    print("[INFO] loading and transforming keyframes...")

    for i, pose in enumerate(poses):
        pcd_path = os.path.join(args.keyframes, pose["pcd"])
        pts_local = load_pcd_xyz(pcd_path)
        total_raw_points += len(pts_local)

        if len(pts_local) == 0:
            frame_points.append(np.zeros((0, 3), dtype=np.float64))
            frame_keys.append([])
            frame_unique_keys.append(set())
            continue

        R = quat_to_rot(pose["qx"], pose["qy"], pose["qz"], pose["qw"])
        t = np.array([pose["x"], pose["y"], pose["z"]], dtype=np.float64)

        pts_world = pts_local @ R.T + t

        local_xy_range = np.linalg.norm(pts_local[:, :2], axis=1)

        mask = (
            (pts_world[:, 2] >= args.z_min) &
            (pts_world[:, 2] <= args.z_max) &
            (local_xy_range >= args.ego_radius) &
            (local_xy_range <= args.max_range)
        )

        pts_world = pts_world[mask]
        total_used_points += len(pts_world)

        keys_np = voxel_keys(pts_world, args.voxel_size)
        keys_list = tuple_keys(keys_np)
        unique_keys = set(keys_list)

        frame_points.append(pts_world)
        frame_keys.append(keys_list)
        frame_unique_keys.append(unique_keys)

        if i % 20 == 0:
            print(
                f"[INFO] loaded frame {i:04d}: "
                f"raw={len(pts_local)}, used={len(pts_world)}, unique_voxels={len(unique_keys)}"
            )

    print(f"[INFO] total_raw_points: {total_raw_points}")
    print(f"[INFO] total_used_points_after_crop: {total_used_points}")

    static_points_chunks = []
    removed_points_chunks = []

    total_static = 0
    total_removed = 0

    print("[INFO] sliding-window filtering...")

    for i in range(n):
        start = max(0, i - args.window_radius)
        end = min(n, i + args.window_radius + 1)

        support = defaultdict(int)

        # 每个 voxel 统计窗口内有多少个不同 keyframe 命中过。
        for j in range(start, end):
            for k in frame_unique_keys[j]:
                support[k] += 1

        pts = frame_points[i]
        keys = frame_keys[i]

        if len(pts) == 0:
            continue

        keep_mask = np.zeros(len(pts), dtype=bool)

        for pi, k in enumerate(keys):
            s = max_neighbor_support(k, support, args.neighbor_radius_vox)
            if s >= args.min_frame_support:
                keep_mask[pi] = True

        static_pts = pts[keep_mask]
        removed_pts = pts[~keep_mask]

        if len(static_pts) > 0:
            static_points_chunks.append(static_pts)
        if len(removed_pts) > 0:
            removed_points_chunks.append(removed_pts)

        total_static += len(static_pts)
        total_removed += len(removed_pts)

        if i % 20 == 0:
            print(
                f"[INFO] frame {i:04d}: "
                f"window=[{start},{end}), "
                f"static={len(static_pts)}, removed={len(removed_pts)}"
            )

    if static_points_chunks:
        static_points = np.vstack(static_points_chunks)
    else:
        static_points = np.zeros((0, 3), dtype=np.float64)

    if removed_points_chunks:
        removed_points = np.vstack(removed_points_chunks)
    else:
        removed_points = np.zeros((0, 3), dtype=np.float64)

    print(f"[INFO] static raw output points: {len(static_points)}")
    print(f"[INFO] removed raw output points: {len(removed_points)}")

    static_points_ds = voxel_downsample(static_points, args.output_voxel)
    removed_points_ds = voxel_downsample(removed_points, args.output_voxel)

    print(f"[INFO] static after output voxel {args.output_voxel}: {len(static_points_ds)}")
    print(f"[INFO] removed after output voxel {args.output_voxel}: {len(removed_points_ds)}")

    write_pcd_xyz(args.out_static, static_points_ds)
    write_pcd_xyz(args.out_removed, removed_points_ds)

    print("\n[DONE]")
    print(f"static map:  {args.out_static}")
    print(f"removed map: {args.out_removed}")


if __name__ == "__main__":
    main()

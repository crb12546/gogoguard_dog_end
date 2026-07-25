#!/usr/bin/env python3
import argparse
import math
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


def voxel_key(p, voxel):
    return tuple(np.floor(p / voxel).astype(np.int64))


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


def raycast_free_voxels(origin, point, voxel, max_range, step):
    vec = point - origin
    dist = float(np.linalg.norm(vec))

    if dist < 1e-6 or dist > max_range:
        return []

    direction = vec / dist

    # 不标记最后一个 voxel，避免把命中点也当 free。
    end_dist = max(0.0, dist - voxel * 1.5)
    if end_dist <= step:
        return []

    keys = []
    s = step
    while s < end_dist:
        p = origin + direction * s
        keys.append(voxel_key(p, voxel))
        s += step

    return keys


def neighbor_support(key, hit_frame_count, min_neighbor_hit_frames):
    kx, ky, kz = key
    count = 0

    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            for dz in (-1, 0, 1):
                if dx == 0 and dy == 0 and dz == 0:
                    continue
                nk = (kx + dx, ky + dy, kz + dz)
                if hit_frame_count.get(nk, 0) >= min_neighbor_hit_frames:
                    count += 1

    return count


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--keyframes", required=True)
    parser.add_argument("--out_static", default="/home/unitree/go2_fastlio_ws/maps/loop_backend/static_map_filtered.pcd")
    parser.add_argument("--out_removed", default="/home/unitree/go2_fastlio_ws/maps/loop_backend/dynamic_removed.pcd")

    parser.add_argument("--voxel_size", type=float, default=0.10)
    parser.add_argument("--z_min", type=float, default=-0.30)
    parser.add_argument("--z_max", type=float, default=2.50)
    parser.add_argument("--ego_radius", type=float, default=0.50)

    parser.add_argument("--static_hit_frames", type=int, default=3)
    parser.add_argument("--candidate_hit_frames", type=int, default=2)
    parser.add_argument("--min_neighbor_support", type=int, default=4)
    parser.add_argument("--neighbor_min_hit_frames", type=int, default=2)

    parser.add_argument("--raycast", action="store_true")
    parser.add_argument("--ray_point_stride", type=int, default=8)
    parser.add_argument("--ray_step_multiplier", type=float, default=2.0)
    parser.add_argument("--max_ray_range", type=float, default=15.0)
    parser.add_argument("--free_conflict_ratio", type=float, default=5.0)

    args = parser.parse_args()

    poses_path = os.path.join(args.keyframes, "poses_raw.txt")
    poses = load_poses(poses_path)

    print(f"[INFO] loaded poses: {len(poses)}")
    print(f"[INFO] keyframes: {args.keyframes}")
    print(f"[INFO] voxel_size: {args.voxel_size}")
    print(f"[INFO] raycast: {args.raycast}")

    hit_count = defaultdict(int)
    hit_frames = defaultdict(set)
    free_count = defaultdict(int)
    sum_points = defaultdict(lambda: np.zeros(3, dtype=np.float64))

    total_points = 0
    used_points = 0

    for frame_i, pose in enumerate(poses):
        pcd_path = os.path.join(args.keyframes, pose["pcd"])
        pts_local = load_pcd_xyz(pcd_path)

        R = quat_to_rot(pose["qx"], pose["qy"], pose["qz"], pose["qw"])
        t = np.array([pose["x"], pose["y"], pose["z"]], dtype=np.float64)

        if pts_local.shape[0] == 0:
            continue

        pts_world = pts_local @ R.T + t
        total_points += pts_world.shape[0]

        local_range = np.linalg.norm(pts_local[:, :2], axis=1)
        mask = (
            (pts_world[:, 2] >= args.z_min) &
            (pts_world[:, 2] <= args.z_max) &
            (local_range >= args.ego_radius)
        )

        pts_world = pts_world[mask]
        pts_local_valid = pts_local[mask]
        used_points += pts_world.shape[0]

        for p in pts_world:
            k = voxel_key(p, args.voxel_size)
            hit_count[k] += 1
            hit_frames[k].add(frame_i)
            sum_points[k] += p

        if args.raycast:
            stride = max(1, int(args.ray_point_stride))
            step = max(args.voxel_size, args.voxel_size * args.ray_step_multiplier)

            for p in pts_world[::stride]:
                for fk in raycast_free_voxels(
                    t,
                    p,
                    args.voxel_size,
                    args.max_ray_range,
                    step,
                ):
                    free_count[fk] += 1

        if frame_i % 20 == 0:
            print(
                f"[INFO] frame {frame_i:04d}, "
                f"raw_points={len(pts_local)}, used_points={len(pts_world)}, "
                f"hit_voxels={len(hit_count)}"
            )

    print(f"[INFO] total raw points: {total_points}")
    print(f"[INFO] total used points after crop: {used_points}")
    print(f"[INFO] total hit voxels: {len(hit_count)}")
    print(f"[INFO] total free voxels: {len(free_count)}")

    hit_frame_count = {k: len(v) for k, v in hit_frames.items()}

    static_points = []
    removed_points = []

    num_static_direct = 0
    num_static_neighbor = 0
    num_removed_free_conflict = 0
    num_removed_weak = 0

    for k, cnt in hit_count.items():
        hf = hit_frame_count[k]
        fc = free_count.get(k, 0)
        centroid = sum_points[k] / float(cnt)

        ns = neighbor_support(k, hit_frame_count, args.neighbor_min_hit_frames)

        # 高置信静态：多帧直接命中。
        if hf >= args.static_hit_frames:
            static_points.append(centroid)
            num_static_direct += 1
            continue

        # 中等置信：命中帧数较少，但周围结构稳定，保留。
        if hf >= args.candidate_hit_frames and ns >= args.min_neighbor_support:
            static_points.append(centroid)
            num_static_neighbor += 1
            continue

        # freespace 反证强，删除。
        if args.raycast and fc >= args.free_conflict_ratio * max(1, hf):
            removed_points.append(centroid)
            num_removed_free_conflict += 1
            continue

        # 单帧/弱支持点，删除。
        removed_points.append(centroid)
        num_removed_weak += 1

    static_points = np.asarray(static_points, dtype=np.float64)
    removed_points = np.asarray(removed_points, dtype=np.float64)

    write_pcd_xyz(args.out_static, static_points)
    write_pcd_xyz(args.out_removed, removed_points)

    print("\n[RESULT]")
    print(f"static voxels direct:   {num_static_direct}")
    print(f"static voxels neighbor: {num_static_neighbor}")
    print(f"removed free conflict:  {num_removed_free_conflict}")
    print(f"removed weak support:   {num_removed_weak}")
    print(f"static points:          {len(static_points)}")
    print(f"removed points:         {len(removed_points)}")
    print(f"[DONE] static map:      {args.out_static}")
    print(f"[DONE] removed map:     {args.out_removed}")


if __name__ == "__main__":
    main()

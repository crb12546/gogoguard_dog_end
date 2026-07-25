#!/usr/bin/env python3
import argparse
import math
import os
from collections import defaultdict

import numpy as np


UNKNOWN = 205
FREE = 254
OCCUPIED = 0


def load_pcd_xyz(path):
    points = []
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
                    points.append([float(a[0]), float(a[1]), float(a[2])])

    return np.asarray(points, dtype=np.float64)


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

        for x, y, z in points:
            f.write(f"{x:.4f} {y:.4f} {z:.4f}\n")


def write_pgm(path, img):
    os.makedirs(os.path.dirname(path), exist_ok=True)

    h, w = img.shape
    with open(path, "wb") as f:
        f.write(f"P5\n{w} {h}\n255\n".encode("ascii"))
        f.write(img.astype(np.uint8).tobytes())


def neighbor_count(cell, cell_set, radius=1):
    ix, iy = cell
    c = 0
    for dx in range(-radius, radius + 1):
        for dy in range(-radius, radius + 1):
            if dx == 0 and dy == 0:
                continue
            if (ix + dx, iy + dy) in cell_set:
                c += 1
    return c


def cell_center(ix, iy, min_x, min_y, resolution, z=0.0):
    return [
        min_x + (ix + 0.5) * resolution,
        min_y + (iy + 0.5) * resolution,
        z,
    ]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--in_pcd", required=True)
    parser.add_argument("--out_dir", required=True)
    parser.add_argument("--map_name", default="campus_level_y12p3")

    parser.add_argument("--resolution", type=float, default=0.05)
    parser.add_argument("--padding", type=float, default=2.0)

    # 全局高度裁剪：先去掉明显无关的高空/低空噪声
    parser.add_argument("--z_global_min", type=float, default=-0.40)
    parser.add_argument("--z_global_max", type=float, default=2.50)

    # 局部地面估计
    parser.add_argument("--ground_percentile", type=float, default=10.0)
    parser.add_argument("--ground_neighbor_radius", type=int, default=4)
    parser.add_argument("--ground_candidate_max_z", type=float, default=0.60)

    # 相对地面高度分类
    parser.add_argument("--ground_rel_min", type=float, default=-0.10)
    parser.add_argument("--ground_rel_max", type=float, default=0.15)
    parser.add_argument("--obstacle_rel_min", type=float, default=0.20)
    parser.add_argument("--obstacle_rel_max", type=float, default=1.50)

    # 点数阈值
    parser.add_argument("--min_ground_points", type=int, default=2)
    parser.add_argument("--min_obstacle_points", type=int, default=3)

    # 孤立障碍过滤
    parser.add_argument("--remove_isolated_obstacles", action="store_true")
    parser.add_argument("--obstacle_neighbor_radius", type=int, default=1)
    parser.add_argument("--min_obstacle_neighbors", type=int, default=1)
    parser.add_argument("--strong_obstacle_points", type=int, default=10)

    args = parser.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)

    print(f"[INFO] loading: {args.in_pcd}")
    pts = load_pcd_xyz(args.in_pcd)

    if len(pts) == 0:
        raise RuntimeError("No points loaded")

    print(f"[INFO] raw points: {len(pts)}")
    print(f"[INFO] raw z range: {pts[:,2].min():.3f} ~ {pts[:,2].max():.3f}")

    mask = (
        (pts[:, 2] >= args.z_global_min) &
        (pts[:, 2] <= args.z_global_max)
    )
    pts = pts[mask]

    if len(pts) == 0:
        raise RuntimeError("No points after z crop")

    print(f"[INFO] points after z crop: {len(pts)}")
    print(f"[INFO] cropped z range: {pts[:,2].min():.3f} ~ {pts[:,2].max():.3f}")

    min_x = float(np.min(pts[:, 0]) - args.padding)
    max_x = float(np.max(pts[:, 0]) + args.padding)
    min_y = float(np.min(pts[:, 1]) - args.padding)
    max_y = float(np.max(pts[:, 1]) + args.padding)

    width = int(math.ceil((max_x - min_x) / args.resolution))
    height = int(math.ceil((max_y - min_y) / args.resolution))

    print(f"[INFO] bounds x: {min_x:.2f} ~ {max_x:.2f}")
    print(f"[INFO] bounds y: {min_y:.2f} ~ {max_y:.2f}")
    print(f"[INFO] grid: {width} x {height}, resolution={args.resolution}")

    ix = np.floor((pts[:, 0] - min_x) / args.resolution).astype(np.int32)
    iy = np.floor((pts[:, 1] - min_y) / args.resolution).astype(np.int32)

    valid = (ix >= 0) & (ix < width) & (iy >= 0) & (iy < height)
    pts = pts[valid]
    ix = ix[valid]
    iy = iy[valid]

    cell_zs = defaultdict(list)

    for x_idx, y_idx, z in zip(ix, iy, pts[:, 2]):
        cell_zs[(int(x_idx), int(y_idx))].append(float(z))

    print(f"[INFO] occupied-by-points cells: {len(cell_zs)}")

    # 每个 cell 的低分位高度
    low_z = {}
    count_z = {}
    for cell, zs in cell_zs.items():
        arr = np.asarray(zs, dtype=np.float64)
        count_z[cell] = len(arr)
        low_z[cell] = float(np.percentile(arr, args.ground_percentile))

    # 候选地面 cell：低分位高度不能太高，避免墙面高处当成地面
    ground_candidates = {
        cell: z for cell, z in low_z.items()
        if z <= args.ground_candidate_max_z
    }

    global_ground = float(np.percentile(pts[:, 2], args.ground_percentile))
    print(f"[INFO] global ground estimate p{args.ground_percentile}: {global_ground:.3f}")
    print(f"[INFO] ground candidate cells: {len(ground_candidates)}")

    ground_cache = {}

    def estimate_ground(cell):
        if cell in ground_cache:
            return ground_cache[cell]

        cx, cy = cell
        values = []

        r = args.ground_neighbor_radius
        for dx in range(-r, r + 1):
            for dy in range(-r, r + 1):
                nc = (cx + dx, cy + dy)
                if nc in ground_candidates:
                    values.append(ground_candidates[nc])

        if values:
            # 用邻域低高度的中位数，避免单个低噪声把地面拉低
            g = float(np.median(values))
        elif cell in low_z:
            g = float(low_z[cell])
        else:
            g = global_ground

        ground_cache[cell] = g
        return g

    free_cells = set()
    occupied_cells = set()

    free_debug = []
    occ_debug = []
    unknown_debug = []

    cell_stats = {}

    for cell, zs in cell_zs.items():
        arr = np.asarray(zs, dtype=np.float64)
        g = estimate_ground(cell)
        rel = arr - g

        ground_count = int(np.sum(
            (rel >= args.ground_rel_min) &
            (rel <= args.ground_rel_max)
        ))

        obstacle_count = int(np.sum(
            (rel >= args.obstacle_rel_min) &
            (rel <= args.obstacle_rel_max)
        ))

        cell_stats[cell] = {
            "ground_z": g,
            "ground_count": ground_count,
            "obstacle_count": obstacle_count,
            "point_count": len(arr),
        }

        ix0, iy0 = cell

        # occupied 优先级高于 free
        if obstacle_count >= args.min_obstacle_points:
            occupied_cells.add(cell)
        elif ground_count >= args.min_ground_points:
            free_cells.add(cell)

    print(f"[INFO] raw occupied cells: {len(occupied_cells)}")
    print(f"[INFO] raw free cells: {len(free_cells)}")

    if args.remove_isolated_obstacles:
        occ_before = len(occupied_cells)
        new_occupied = set()

        for cell in occupied_cells:
            nb = neighbor_count(
                cell,
                occupied_cells,
                radius=args.obstacle_neighbor_radius,
            )
            obstacle_count = cell_stats[cell]["obstacle_count"]

            if nb >= args.min_obstacle_neighbors or obstacle_count >= args.strong_obstacle_points:
                new_occupied.add(cell)

        occupied_cells = new_occupied
        print(f"[INFO] isolated obstacle filter: {occ_before} -> {len(occupied_cells)}")

    # 如果某 cell 同时 free 和 occupied，occupied 优先
    free_cells = free_cells - occupied_cells

    print(f"[INFO] final occupied cells: {len(occupied_cells)}")
    print(f"[INFO] final free cells: {len(free_cells)}")

    img = np.full((height, width), UNKNOWN, dtype=np.uint8)

    for cell in free_cells:
        x_idx, y_idx = cell
        row = height - 1 - y_idx
        col = x_idx
        img[row, col] = FREE
        gz = cell_stats[cell]["ground_z"]
        free_debug.append(cell_center(x_idx, y_idx, min_x, min_y, args.resolution, gz))

    for cell in occupied_cells:
        x_idx, y_idx = cell
        row = height - 1 - y_idx
        col = x_idx
        img[row, col] = OCCUPIED
        gz = cell_stats[cell]["ground_z"]
        occ_debug.append(cell_center(x_idx, y_idx, min_x, min_y, args.resolution, gz + 0.5))

    # 少量 unknown debug：只输出有点但未分类的 cell，避免巨大文件
    point_cells = set(cell_zs.keys())
    unknown_observed_cells = point_cells - free_cells - occupied_cells
    for cell in unknown_observed_cells:
        x_idx, y_idx = cell
        gz = cell_stats[cell]["ground_z"]
        unknown_debug.append(cell_center(x_idx, y_idx, min_x, min_y, args.resolution, gz + 0.25))

    pgm_path = os.path.join(args.out_dir, f"{args.map_name}.pgm")
    yaml_path = os.path.join(args.out_dir, f"{args.map_name}.yaml")

    write_pgm(pgm_path, img)

    with open(yaml_path, "w") as f:
        f.write(f"image: {os.path.basename(pgm_path)}\n")
        f.write("mode: trinary\n")
        f.write(f"resolution: {args.resolution}\n")
        f.write(f"origin: [{min_x:.6f}, {min_y:.6f}, 0.0]\n")
        f.write("negate: 0\n")
        f.write("occupied_thresh: 0.65\n")
        f.write("free_thresh: 0.25\n")

    debug_free_path = os.path.join(args.out_dir, f"{args.map_name}_debug_free.pcd")
    debug_occ_path = os.path.join(args.out_dir, f"{args.map_name}_debug_occupied.pcd")
    debug_unknown_path = os.path.join(args.out_dir, f"{args.map_name}_debug_unknown_observed.pcd")

    write_pcd_xyz(debug_free_path, np.asarray(free_debug, dtype=np.float64))
    write_pcd_xyz(debug_occ_path, np.asarray(occ_debug, dtype=np.float64))
    write_pcd_xyz(debug_unknown_path, np.asarray(unknown_debug, dtype=np.float64))

    print("\n[DONE]")
    print(f"map pgm:   {pgm_path}")
    print(f"map yaml:  {yaml_path}")
    print(f"debug free:     {debug_free_path}")
    print(f"debug occupied: {debug_occ_path}")
    print(f"debug unknown:  {debug_unknown_path}")


if __name__ == "__main__":
    main()

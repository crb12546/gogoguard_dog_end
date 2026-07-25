#!/usr/bin/env python3
import argparse
import math
import os
import numpy as np
from scipy import ndimage


UNKNOWN = 205
FREE = 254
OCCUPIED = 0


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


def write_pgm(path, img):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    h, w = img.shape
    with open(path, "wb") as f:
        f.write(f"P5\n{w} {h}\n255\n".encode("ascii"))
        f.write(img.astype(np.uint8).tobytes())


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


def disk_structure(radius):
    if radius <= 0:
        return np.ones((1, 1), dtype=bool)
    r = int(radius)
    yy, xx = np.mgrid[-r:r + 1, -r:r + 1]
    return (xx * xx + yy * yy) <= r * r


def remove_small_components(mask, min_cells):
    if min_cells <= 1:
        return mask
    labels, n = ndimage.label(mask)
    if n == 0:
        return mask
    sizes = np.bincount(labels.ravel())
    keep = sizes >= min_cells
    keep[0] = False
    return keep[labels]


def keep_large_components(mask, min_cells):
    if min_cells <= 1:
        return mask
    labels, n = ndimage.label(mask)
    if n == 0:
        return mask
    sizes = np.bincount(labels.ravel())
    keep = sizes >= min_cells
    keep[0] = False
    return keep[labels]


def grid_to_debug_pcd(mask, min_x, min_y, res, z):
    ys, xs = np.nonzero(mask)
    pts = np.zeros((len(xs), 3), dtype=np.float64)
    pts[:, 0] = min_x + (xs + 0.5) * res
    pts[:, 1] = min_y + (ys + 0.5) * res
    pts[:, 2] = z
    return pts


def compute_cell_percentile(linear, z, size, percentile):
    """
    Fast grouped percentile per cell.
    Sort by (cell_id, z), then select percentile index inside each group.
    """
    order = np.lexsort((z, linear))
    lin_s = linear[order]
    z_s = z[order]

    unique, start, count = np.unique(lin_s, return_index=True, return_counts=True)

    # percentile index in each sorted group
    frac = percentile / 100.0
    offset = np.floor((count - 1) * frac).astype(np.int64)
    idx = start + offset

    out = np.full(size, np.nan, dtype=np.float64)
    out[unique] = z_s[idx]
    return out, unique, count


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--in_pcd", required=True)
    parser.add_argument("--out_dir", required=True)
    parser.add_argument("--map_name", default="campus_level_y12p3_fast")

    parser.add_argument("--resolution", type=float, default=0.05)
    parser.add_argument("--padding", type=float, default=2.0)

    # level_pcd 后低位高度已接近 0，这里保留足够高度用于花坛/树干/墙体
    parser.add_argument("--z_min", type=float, default=-0.35)
    parser.add_argument("--z_max", type=float, default=2.30)

    # 局部地面估计
    parser.add_argument("--ground_percentile", type=float, default=8.0)
    parser.add_argument("--ground_candidate_max_z", type=float, default=0.45)
    parser.add_argument("--ground_fill_max_dist_cells", type=int, default=80)
    parser.add_argument("--ground_smooth_radius", type=int, default=2)

    # relative height 判定
    parser.add_argument("--ground_rel_min", type=float, default=-0.08)
    parser.add_argument("--ground_rel_max", type=float, default=0.18)
    parser.add_argument("--obs_rel_min", type=float, default=0.18)
    parser.add_argument("--obs_rel_max", type=float, default=1.60)

    parser.add_argument("--min_ground_points", type=int, default=1)
    parser.add_argument("--min_obs_points", type=int, default=2)

    # free 通路连续化
    parser.add_argument("--free_close_radius", type=int, default=4)
    parser.add_argument("--free_dilate_radius", type=int, default=1)
    parser.add_argument("--min_free_component_cells", type=int, default=80)

    # obstacle 过滤和连续化
    parser.add_argument("--obs_close_radius", type=int, default=1)
    parser.add_argument("--min_obs_component_cells", type=int, default=60)
    parser.add_argument("--obs_dilate_radius", type=int, default=1)

    args = parser.parse_args()
    os.makedirs(args.out_dir, exist_ok=True)

    pts = load_pcd_xyz(args.in_pcd)
    if len(pts) == 0:
        raise RuntimeError("No points loaded")

    print(f"[INFO] loaded points: {len(pts)}")
    print(f"[INFO] raw z range: {pts[:,2].min():.3f} ~ {pts[:,2].max():.3f}")

    z_crop = (pts[:, 2] >= args.z_min) & (pts[:, 2] <= args.z_max)
    pts = pts[z_crop]

    if len(pts) == 0:
        raise RuntimeError("No points after z crop")

    print(f"[INFO] after z crop: {len(pts)}")
    print(f"[INFO] crop z range: {pts[:,2].min():.3f} ~ {pts[:,2].max():.3f}")

    min_x = float(np.min(pts[:, 0]) - args.padding)
    max_x = float(np.max(pts[:, 0]) + args.padding)
    min_y = float(np.min(pts[:, 1]) - args.padding)
    max_y = float(np.max(pts[:, 1]) + args.padding)

    width = int(math.ceil((max_x - min_x) / args.resolution))
    height = int(math.ceil((max_y - min_y) / args.resolution))
    size = width * height

    print(f"[INFO] bounds x: {min_x:.2f} ~ {max_x:.2f}")
    print(f"[INFO] bounds y: {min_y:.2f} ~ {max_y:.2f}")
    print(f"[INFO] grid: {width} x {height}, res={args.resolution}")

    ix = np.floor((pts[:, 0] - min_x) / args.resolution).astype(np.int64)
    iy = np.floor((pts[:, 1] - min_y) / args.resolution).astype(np.int64)

    valid = (ix >= 0) & (ix < width) & (iy >= 0) & (iy < height)
    pts = pts[valid]
    ix = ix[valid]
    iy = iy[valid]

    linear = iy * width + ix
    z = pts[:, 2]

    # 1. 每个 cell 估计低分位高度
    low_z_flat, observed_cells, observed_counts = compute_cell_percentile(
        linear, z, size, args.ground_percentile
    )

    point_count = np.zeros(size, dtype=np.int32)
    point_count[observed_cells] = observed_counts

    low_z_grid = low_z_flat.reshape((height, width))
    observed_grid = ~np.isnan(low_z_grid)

    global_ground = float(np.nanpercentile(low_z_flat, args.ground_percentile))
    print(f"[INFO] global ground estimate: {global_ground:.3f}")

    # 候选地面 cell：只用低位高度较低的 cell 作为地面锚点
    ground_candidate = observed_grid & (low_z_grid <= args.ground_candidate_max_z)

    if np.sum(ground_candidate) == 0:
        raise RuntimeError("No ground candidate cells. Try increasing --ground_candidate_max_z")

    print(f"[INFO] ground candidate cells: {int(np.sum(ground_candidate))}")

    # 2. 用最近地面候选填充 ground_z
    # 对非候选 cell 找最近候选 cell 的 low_z
    dist, inds = ndimage.distance_transform_edt(
        ~ground_candidate,
        return_distances=True,
        return_indices=True
    )

    ground_grid = low_z_grid[inds[0], inds[1]]
    ground_grid[dist > args.ground_fill_max_dist_cells] = global_ground

    if args.ground_smooth_radius > 0:
        k = args.ground_smooth_radius * 2 + 1
        ground_grid = ndimage.median_filter(ground_grid, size=k)

    # 3. 所有点按 local ground 做 relative height
    ground_for_points = ground_grid[iy, ix]
    rel_z = z - ground_for_points

    ground_mask = (rel_z >= args.ground_rel_min) & (rel_z <= args.ground_rel_max)
    obs_mask = (rel_z >= args.obs_rel_min) & (rel_z <= args.obs_rel_max)

    ground_count = np.bincount(linear[ground_mask], minlength=size).reshape((height, width))
    obs_count = np.bincount(linear[obs_mask], minlength=size).reshape((height, width))

    free_raw = ground_count >= args.min_ground_points
    obs_raw = obs_count >= args.min_obs_points

    print(f"[INFO] free_raw cells: {int(np.sum(free_raw))}")
    print(f"[INFO] obs_raw cells:  {int(np.sum(obs_raw))}")

    # 4. free 通路修复：填小洞、连断点、保留大通路
    free = free_raw.copy()

    if args.free_close_radius > 0:
        free = ndimage.binary_closing(
            free,
            structure=disk_structure(args.free_close_radius)
        )

    if args.free_dilate_radius > 0:
        free = ndimage.binary_dilation(
            free,
            structure=disk_structure(args.free_dilate_radius)
        )

    free = keep_large_components(free, args.min_free_component_cells)

    # 5. obstacle 修复：连接花坛/墙体，删除行人小块，再轻微膨胀
    obs = obs_raw.copy()

    if args.obs_close_radius > 0:
        obs = ndimage.binary_closing(
            obs,
            structure=disk_structure(args.obs_close_radius)
        )

    obs = remove_small_components(obs, args.min_obs_component_cells)

    if args.obs_dilate_radius > 0:
        obs = ndimage.binary_dilation(
            obs,
            structure=disk_structure(args.obs_dilate_radius)
        )

    # occupied 优先
    free = free & (~obs)

    print(f"[INFO] free_final cells: {int(np.sum(free))}")
    print(f"[INFO] obs_final cells:  {int(np.sum(obs))}")

    img = np.full((height, width), UNKNOWN, dtype=np.uint8)

    fy, fx = np.nonzero(free)
    oy, ox = np.nonzero(obs)

    img[height - 1 - fy, fx] = FREE
    img[height - 1 - oy, ox] = OCCUPIED

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

    write_pcd_xyz(
        os.path.join(args.out_dir, f"{args.map_name}_debug_free.pcd"),
        grid_to_debug_pcd(free, min_x, min_y, args.resolution, 0.0)
    )
    write_pcd_xyz(
        os.path.join(args.out_dir, f"{args.map_name}_debug_occupied.pcd"),
        grid_to_debug_pcd(obs, min_x, min_y, args.resolution, 0.5)
    )

    # ground debug：只输出地面候选，检查地面估计是否合理
    write_pcd_xyz(
        os.path.join(args.out_dir, f"{args.map_name}_debug_ground_candidates.pcd"),
        grid_to_debug_pcd(ground_candidate, min_x, min_y, args.resolution, 0.1)
    )

    print("\n[DONE]")
    print(f"map pgm:  {pgm_path}")
    print(f"map yaml: {yaml_path}")
    print(f"debug free: {os.path.join(args.out_dir, f'{args.map_name}_debug_free.pcd')}")
    print(f"debug occ:  {os.path.join(args.out_dir, f'{args.map_name}_debug_occupied.pcd')}")


if __name__ == "__main__":
    main()

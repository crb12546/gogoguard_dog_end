#!/usr/bin/env python3
import argparse
import math
import os
import numpy as np


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
    return np.asarray(points, dtype=np.float32)


def load_poses(poses_path):
    poses = []
    with open(poses_path, "r") as f:
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


def make_scan_context(
    points,
    num_rings=20,
    num_sectors=60,
    max_radius=30.0,
    min_z=-1.5,
    max_z=2.5,
):
    """
    Scan Context descriptor:
    rows: radial rings
    cols: angular sectors
    value: max height in each bin
    """
    if points.shape[0] == 0:
        return np.zeros((num_rings, num_sectors), dtype=np.float32)

    x = points[:, 0]
    y = points[:, 1]
    z = points[:, 2]

    r = np.sqrt(x * x + y * y)
    mask = (r > 0.5) & (r < max_radius) & (z > min_z) & (z < max_z)

    x = x[mask]
    y = y[mask]
    z = z[mask]
    r = r[mask]

    desc = np.zeros((num_rings, num_sectors), dtype=np.float32)

    if len(r) == 0:
        return desc

    theta = np.arctan2(y, x)
    theta[theta < 0] += 2.0 * math.pi

    ring_idx = np.floor(r / max_radius * num_rings).astype(np.int32)
    sector_idx = np.floor(theta / (2.0 * math.pi) * num_sectors).astype(np.int32)

    ring_idx = np.clip(ring_idx, 0, num_rings - 1)
    sector_idx = np.clip(sector_idx, 0, num_sectors - 1)

    height_value = z - min_z

    for ri, si, hv in zip(ring_idx, sector_idx, height_value):
        if hv > desc[ri, si]:
            desc[ri, si] = hv

    max_val = np.max(desc)
    if max_val > 1e-6:
        desc = desc / max_val

    return desc


def sc_distance_with_shift(desc1, desc2):
    """
    Return best distance and best sector shift.
    Distance lower means more similar.
    """
    num_sectors = desc1.shape[1]
    best_dist = 1e9
    best_shift = 0

    for shift in range(num_sectors):
        d2 = np.roll(desc2, shift=shift, axis=1)

        sims = []
        for c in range(num_sectors):
            v1 = desc1[:, c]
            v2 = d2[:, c]
            n1 = np.linalg.norm(v1)
            n2 = np.linalg.norm(v2)

            if n1 < 1e-6 or n2 < 1e-6:
                continue

            sims.append(float(np.dot(v1, v2) / (n1 * n2)))

        if len(sims) < 3:
            dist = 1.0
        else:
            dist = 1.0 - float(np.mean(sims))

        if dist < best_dist:
            best_dist = dist
            best_shift = shift

    return best_dist, best_shift


def sector_shift_to_yaw_deg(shift, num_sectors):
    return shift * 360.0 / num_sectors


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--keyframes", required=True)
    parser.add_argument("--out", default="/home/unitree/go2_fastlio_ws/maps/loop_backend/loop_candidates.txt")
    parser.add_argument("--num_rings", type=int, default=20)
    parser.add_argument("--num_sectors", type=int, default=60)
    parser.add_argument("--max_radius", type=float, default=30.0)
    parser.add_argument("--min_z", type=float, default=-1.5)
    parser.add_argument("--max_z", type=float, default=2.5)
    parser.add_argument("--min_separation", type=int, default=15)
    parser.add_argument("--threshold", type=float, default=0.35)
    args = parser.parse_args()

    poses_path = os.path.join(args.keyframes, "poses_raw.txt")
    poses = load_poses(poses_path)

    print(f"[INFO] loaded poses: {len(poses)}")
    print("[INFO] building scan context descriptors...")

    descs = []
    for p in poses:
        pcd_path = os.path.join(args.keyframes, p["pcd"])
        pts = load_pcd_xyz(pcd_path)

        desc = make_scan_context(
            pts,
            num_rings=args.num_rings,
            num_sectors=args.num_sectors,
            max_radius=args.max_radius,
            min_z=args.min_z,
            max_z=args.max_z,
        )
        descs.append(desc)

        if p["idx"] % 10 == 0:
            print(f"[INFO] descriptor for keyframe {p['idx']:06d}, points={len(pts)}")

    candidates = []

    print("[INFO] detecting loop candidates...")
    for i in range(len(poses)):
        if i < args.min_separation:
            continue

        best_j = -1
        best_dist = 1e9
        best_shift = 0

        for j in range(0, i - args.min_separation):
            dist, shift = sc_distance_with_shift(descs[i], descs[j])

            if dist < best_dist:
                best_dist = dist
                best_j = j
                best_shift = shift

        yaw_shift_deg = sector_shift_to_yaw_deg(best_shift, args.num_sectors)

        if best_dist < args.threshold:
            candidates.append((i, best_j, best_dist, yaw_shift_deg))
            print(
                f"[LOOP] current={i:06d}, candidate={best_j:06d}, "
                f"sc_dist={best_dist:.4f}, yaw_shift={yaw_shift_deg:.1f} deg"
            )
        else:
            print(
                f"[BEST] current={i:06d}, candidate={best_j:06d}, "
                f"sc_dist={best_dist:.4f}, yaw_shift={yaw_shift_deg:.1f} deg"
            )

    os.makedirs(os.path.dirname(args.out), exist_ok=True)

    with open(args.out, "w") as f:
        f.write("# current_idx candidate_idx sc_distance yaw_shift_deg\n")
        for c in candidates:
            f.write(f"{c[0]} {c[1]} {c[2]:.6f} {c[3]:.3f}\n")

    print(f"[DONE] candidates: {len(candidates)}")
    print(f"[DONE] saved: {args.out}")


if __name__ == "__main__":
    main()

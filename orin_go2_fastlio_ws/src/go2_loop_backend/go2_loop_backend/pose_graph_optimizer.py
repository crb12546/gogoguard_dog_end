#!/usr/bin/env python3
import argparse
import math
import os
import numpy as np
from scipy.optimize import least_squares


def norm_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def relative_pose(p_i, p_j):
    """
    SE(2) relative pose from i to j.
    p = [x, y, yaw]
    """
    xi, yi, thi = p_i
    xj, yj, thj = p_j

    dx = xj - xi
    dy = yj - yi

    c = math.cos(thi)
    s = math.sin(thi)

    rel_x = c * dx + s * dy
    rel_y = -s * dx + c * dy
    rel_th = norm_angle(thj - thi)

    return np.array([rel_x, rel_y, rel_th], dtype=np.float64)


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


def load_loops(path):
    loops = []
    with open(path, "r") as f:
        for line in f:
            if line.startswith("#") or not line.strip():
                continue
            a = line.split()
            cur = int(a[0])
            cand = int(a[1])
            sc_dist = float(a[2])
            yaw_shift = float(a[3])
            loops.append((cur, cand, sc_dist, yaw_shift))
    return loops


def yaw_to_quat(yaw):
    qz = math.sin(yaw * 0.5)
    qw = math.cos(yaw * 0.5)
    return 0.0, 0.0, qz, qw


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--keyframes", required=True)
    parser.add_argument("--loops", required=True)
    parser.add_argument("--out", default="/home/unitree/go2_fastlio_ws/maps/loop_backend/keyframes/poses_optimized.txt")

    parser.add_argument("--odom_weight_xy", type=float, default=1.0)
    parser.add_argument("--odom_weight_yaw", type=float, default=1.0)

    parser.add_argument("--loop_weight_xy", type=float, default=8.0)
    parser.add_argument("--loop_weight_yaw", type=float, default=3.0)

    parser.add_argument("--max_nfev", type=int, default=200)
    args = parser.parse_args()

    poses_path = os.path.join(args.keyframes, "poses_raw.txt")
    poses = load_poses(poses_path)
    loops = load_loops(args.loops)

    n = len(poses)
    if n < 2:
        raise RuntimeError("Not enough poses")

    print(f"[INFO] loaded poses: {n}")
    print(f"[INFO] loaded loop edges: {len(loops)}")

    raw = np.zeros((n, 3), dtype=np.float64)
    for i, p in enumerate(poses):
        raw[i, 0] = p["x"]
        raw[i, 1] = p["y"]
        raw[i, 2] = p["yaw"]

    # Odometry edges from raw FAST-LIO relative motion.
    odom_edges = []
    for i in range(n - 1):
        meas = relative_pose(raw[i], raw[i + 1])
        odom_edges.append((i, i + 1, meas))

    # Optimize all poses except pose 0, which is fixed.
    x0 = raw[1:].reshape(-1)

    def unpack(x):
        est = np.zeros((n, 3), dtype=np.float64)
        est[0] = raw[0]
        est[1:] = x.reshape((n - 1, 3))
        return est

    def residuals(x):
        est = unpack(x)
        res = []

        # Odometry constraints.
        for i, j, meas in odom_edges:
            pred = relative_pose(est[i], est[j])
            e = pred - meas
            e[2] = norm_angle(e[2])

            res.append(args.odom_weight_xy * e[0])
            res.append(args.odom_weight_xy * e[1])
            res.append(args.odom_weight_yaw * e[2])

        # Loop constraints.
        # First version: high-confidence loop candidate means same place / near same heading.
        # Later we will replace this zero-relative loop measurement with ICP/GICP measurement.
        for cur, cand, sc_dist, yaw_shift in loops:
            if cur < 0 or cur >= n or cand < 0 or cand >= n:
                continue

            pred = relative_pose(est[cand], est[cur])

            res.append(args.loop_weight_xy * pred[0])
            res.append(args.loop_weight_xy * pred[1])
            res.append(args.loop_weight_yaw * norm_angle(pred[2]))

        return np.asarray(res, dtype=np.float64)

    def print_loop_stats(label, arr):
        print(f"\n[{label}] loop residuals:")
        for cur, cand, sc_dist, yaw_shift in loops:
            rel = relative_pose(arr[cand], arr[cur])
            dist_xy = math.hypot(rel[0], rel[1])
            dyaw = abs(math.degrees(norm_angle(rel[2])))
            print(
                f"  {cur:03d}->{cand:03d} "
                f"xy={dist_xy:.3f} m, yaw={dyaw:.2f} deg, sc={sc_dist:.4f}"
            )

    print_loop_stats("BEFORE", raw)

    print("\n[INFO] optimizing...")
    result = least_squares(
        residuals,
        x0,
        loss="huber",
        f_scale=1.0,
        max_nfev=args.max_nfev,
        verbose=1,
    )

    opt = unpack(result.x)

    print(f"[INFO] success: {result.success}")
    print(f"[INFO] cost: {result.cost:.6f}")
    print(f"[INFO] iterations: {result.nfev}")

    print_loop_stats("AFTER", opt)

    os.makedirs(os.path.dirname(args.out), exist_ok=True)

    with open(args.out, "w") as f:
        f.write("# idx stamp x y z qx qy qz qw yaw pcd_file\n")
        for i, p in enumerate(poses):
            x, y, yaw = opt[i]
            z = p["z"]

            qx, qy, qz, qw = yaw_to_quat(yaw)

            f.write(
                f"{p['idx']} {p['stamp']:.9f} "
                f"{x:.6f} {y:.6f} {z:.6f} "
                f"{qx:.9f} {qy:.9f} {qz:.9f} {qw:.9f} "
                f"{yaw:.9f} {p['pcd']}\n"
            )

    print(f"\n[DONE] saved optimized poses: {args.out}")


if __name__ == "__main__":
    main()

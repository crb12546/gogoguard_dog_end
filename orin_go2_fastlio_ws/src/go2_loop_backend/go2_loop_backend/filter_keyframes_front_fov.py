#!/usr/bin/env python3
import argparse
import math
import os
import shutil
from pathlib import Path


def read_ascii_pcd(path):
    header = []
    data_lines = []
    in_data = False

    with open(path, "r") as f:
        for line in f:
            if not in_data:
                header.append(line)
                if line.strip().startswith("DATA"):
                    if "ascii" not in line:
                        raise RuntimeError(f"Only ascii PCD is supported: {path}")
                    in_data = True
            else:
                if line.strip():
                    data_lines.append(line)

    fields = None
    for line in header:
        if line.startswith("FIELDS"):
            fields = line.strip().split()[1:]
            break

    if fields is None:
        raise RuntimeError(f"No FIELDS line in {path}")

    try:
        ix = fields.index("x")
        iy = fields.index("y")
        iz = fields.index("z")
    except ValueError:
        raise RuntimeError(f"PCD has no x/y/z fields: {path}")

    return header, data_lines, ix, iy, iz


def update_header_count(header, n):
    new_header = []
    for line in header:
        if line.startswith("WIDTH"):
            new_header.append(f"WIDTH {n}\n")
        elif line.startswith("POINTS"):
            new_header.append(f"POINTS {n}\n")
        else:
            new_header.append(line)
    return new_header


def write_ascii_pcd(path, header, data_lines):
    header = update_header_count(header, len(data_lines))
    with open(path, "w") as f:
        for line in header:
            f.write(line)
        for line in data_lines:
            f.write(line if line.endswith("\n") else line + "\n")


def filter_pcd(src, dst, front_deg, yaw_offset_deg, range_min, range_max):
    header, lines, ix, iy, iz = read_ascii_pcd(src)

    half = math.radians(front_deg * 0.5)
    yaw_offset = math.radians(yaw_offset_deg)

    kept = []
    removed = 0

    for line in lines:
        a = line.strip().split()
        if len(a) <= max(ix, iy, iz):
            removed += 1
            continue

        try:
            x = float(a[ix])
            y = float(a[iy])
            z = float(a[iz])
        except ValueError:
            removed += 1
            continue

        r_xy = math.hypot(x, y)
        if r_xy < range_min:
            removed += 1
            continue
        if range_max > 0 and r_xy > range_max:
            removed += 1
            continue

        angle = math.atan2(y, x) - yaw_offset

        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi

        if abs(angle) <= half:
            kept.append(line)
        else:
            removed += 1

    write_ascii_pcd(dst, header, kept)
    return len(lines), len(kept), removed


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--in_dir", required=True)
    parser.add_argument("--out_dir", required=True)

    parser.add_argument("--front_deg", type=float, default=270.0)
    parser.add_argument("--yaw_offset_deg", type=float, default=0.0)

    parser.add_argument("--range_min", type=float, default=0.0)
    parser.add_argument("--range_max", type=float, default=0.0)

    args = parser.parse_args()

    in_dir = Path(args.in_dir)
    out_dir = Path(args.out_dir)

    if not in_dir.exists():
        raise RuntimeError(f"Input dir not found: {in_dir}")

    if out_dir.exists():
        shutil.rmtree(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    # 复制 poses_raw.txt 和其它非 pcd 文件
    for p in in_dir.iterdir():
        if p.is_file() and p.suffix != ".pcd":
            shutil.copy2(p, out_dir / p.name)

    pcds = sorted(in_dir.glob("*.pcd"))
    if not pcds:
        raise RuntimeError(f"No PCD files found in {in_dir}")

    total_in = 0
    total_keep = 0
    total_remove = 0

    print(f"[INFO] input:  {in_dir}")
    print(f"[INFO] output: {out_dir}")
    print(f"[INFO] front_deg={args.front_deg}, yaw_offset_deg={args.yaw_offset_deg}")
    print(f"[INFO] keep angle: ±{args.front_deg * 0.5:.1f} deg around local +X")

    for i, src in enumerate(pcds):
        dst = out_dir / src.name
        n_in, n_keep, n_remove = filter_pcd(
            src,
            dst,
            args.front_deg,
            args.yaw_offset_deg,
            args.range_min,
            args.range_max,
        )

        total_in += n_in
        total_keep += n_keep
        total_remove += n_remove

        if i % 20 == 0:
            print(
                f"[INFO] {src.name}: "
                f"in={n_in}, keep={n_keep}, remove={n_remove}"
            )

    print("\n[DONE]")
    print(f"frames:        {len(pcds)}")
    print(f"points input:  {total_in}")
    print(f"points kept:   {total_keep}")
    print(f"points removed:{total_remove}")
    print(f"keep ratio:    {100.0 * total_keep / max(1, total_in):.2f}%")
    print(f"out dir:       {out_dir}")


if __name__ == "__main__":
    main()

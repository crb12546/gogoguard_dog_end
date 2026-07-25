#!/usr/bin/env python3
"""Recreate the original Go2 route-recorder distance sampling exactly."""

import argparse
import csv
import hashlib
import json
import math
import os
from datetime import datetime, timezone
from pathlib import Path


def file_sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def read_raw(path):
    samples = []
    with open(path, "r", newline="") as handle:
        reader = csv.DictReader(handle)
        required = {"x", "y", "yaw"}
        if not required.issubset(reader.fieldnames or []):
            raise RuntimeError("raw CSV must contain x, y and yaw columns")
        for raw_id, row in enumerate(reader):
            x = float(row["x"])
            y = float(row["y"])
            yaw = float(row["yaw"])
            if not all(math.isfinite(value) for value in (x, y, yaw)):
                raise RuntimeError(
                    f"raw CSV contains a non-finite row: {raw_id}"
                )
            samples.append((raw_id, x, y, yaw))
    if len(samples) < 2:
        raise RuntimeError("raw CSV must contain at least two rows")
    return samples


def original_distance_sample(samples, minimum_distance):
    """Match Go2_2's recorder: save first, then save at distance >= limit."""
    selected = [samples[0]]
    last_x = samples[0][1]
    last_y = samples[0][2]
    for sample in samples[1:]:
        _, x, y, _ = sample
        if math.hypot(x - last_x, y - last_y) >= minimum_distance:
            selected.append(sample)
            last_x = x
            last_y = y
    return selected


def densify_geometry(samples, maximum_gap):
    """Add points on sampled line segments without moving any geometry."""
    if maximum_gap <= 0.0:
        return list(samples)
    output = [samples[0]]
    for start, end in zip(samples, samples[1:]):
        _, ax, ay, ayaw = start
        raw_id, bx, by, byaw = end
        distance = math.hypot(bx - ax, by - ay)
        subdivisions = max(1, int(math.ceil(distance / maximum_gap)))
        yaw_delta = math.atan2(
            math.sin(byaw - ayaw),
            math.cos(byaw - ayaw),
        )
        for step in range(1, subdivisions + 1):
            ratio = step / subdivisions
            output.append(
                (
                    raw_id if step == subdivisions else -1,
                    ax + ratio * (bx - ax),
                    ay + ratio * (by - ay),
                    math.atan2(
                        math.sin(ayaw + ratio * yaw_delta),
                        math.cos(ayaw + ratio * yaw_delta),
                    ),
                )
            )
    return output


def write_route(path, samples, speed):
    temporary = Path(str(path) + ".tmp")
    with open(temporary, "w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["id", "x", "y", "yaw", "v"])
        for point_id, (_, x, y, yaw) in enumerate(samples):
            writer.writerow(
                [
                    point_id,
                    f"{x:.6f}",
                    f"{y:.6f}",
                    f"{yaw:.6f}",
                    f"{speed:.3f}",
                ]
            )
    os.replace(temporary, path)


def write_json(path, value):
    temporary = Path(str(path) + ".tmp")
    with open(temporary, "w") as handle:
        json.dump(value, handle, ensure_ascii=False, indent=2)
        handle.write("\n")
    os.replace(temporary, path)


def write_preview(path, raw_samples, route_samples, route_label, title):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    raw_x = [sample[1] for sample in raw_samples]
    raw_y = [sample[2] for sample in raw_samples]
    route_x = [sample[1] for sample in route_samples]
    route_y = [sample[2] for sample in route_samples]
    figure, axis = plt.subplots(figsize=(9, 11), dpi=150)
    axis.plot(
        raw_x,
        raw_y,
        color="#ef4444",
        linewidth=0.7,
        alpha=0.55,
        label="raw xbf4",
    )
    axis.plot(
        route_x,
        route_y,
        color="#2563eb",
        linewidth=1.1,
        alpha=0.95,
        label=route_label,
    )
    axis.scatter(
        [route_x[0]],
        [route_y[0]],
        color="#16a34a",
        s=24,
        label="start",
        zorder=3,
    )
    axis.set_aspect("equal", adjustable="box")
    axis.set_xlabel("x (m)")
    axis.set_ylabel("y (m)")
    axis.set_title(title)
    axis.grid(True, alpha=0.2)
    axis.legend(loc="best")
    figure.tight_layout()
    figure.savefig(path)
    plt.close(figure)


def route_statistics(samples):
    gaps = [
        math.hypot(
            samples[index][1] - samples[index - 1][1],
            samples[index][2] - samples[index - 1][2],
        )
        for index in range(1, len(samples))
    ]
    turns = []
    for index in range(1, len(samples) - 1):
        incoming = math.atan2(
            samples[index][2] - samples[index - 1][2],
            samples[index][1] - samples[index - 1][1],
        )
        outgoing = math.atan2(
            samples[index + 1][2] - samples[index][2],
            samples[index + 1][1] - samples[index][1],
        )
        turn = math.atan2(
            math.sin(outgoing - incoming),
            math.cos(outgoing - incoming),
        )
        turns.append(abs(math.degrees(turn)))
    return {
        "route_length_m": sum(gaps),
        "minimum_gap_m": min(gaps, default=0.0),
        "mean_gap_m": sum(gaps) / len(gaps) if gaps else 0.0,
        "maximum_gap_m": max(gaps, default=0.0),
        "maximum_adjacent_turn_deg": max(turns, default=0.0),
        "adjacent_turns_over_90deg": sum(
            value > 90.0 for value in turns
        ),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("raw_csv")
    parser.add_argument("output_csv")
    parser.add_argument("--report")
    parser.add_argument("--preview")
    parser.add_argument("--minimum-distance", type=float, default=0.40)
    parser.add_argument(
        "--densify-maximum-gap",
        type=float,
        default=0.0,
        help=(
            "preserve the original sampled polyline and insert points so "
            "no segment exceeds this distance; 0 disables densification"
        ),
    )
    parser.add_argument("--speed", type=float, default=0.20)
    args = parser.parse_args()

    if args.minimum_distance <= 0.0:
        raise RuntimeError("minimum distance must be positive")

    raw_path = Path(args.raw_csv).resolve()
    output_path = Path(args.output_csv).resolve()
    report_path = (
        Path(args.report).resolve()
        if args.report
        else output_path.with_suffix(".quality.json")
    )
    preview_path = Path(args.preview).resolve() if args.preview else None
    output_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.parent.mkdir(parents=True, exist_ok=True)
    if preview_path is not None:
        preview_path.parent.mkdir(parents=True, exist_ok=True)

    raw_samples = read_raw(raw_path)
    route_samples = original_distance_sample(
        raw_samples,
        args.minimum_distance,
    )
    geometry_anchors = list(route_samples)
    route_samples = densify_geometry(
        route_samples,
        args.densify_maximum_gap,
    )
    if len(route_samples) < 2:
        raise RuntimeError("distance sampling produced fewer than two points")

    write_route(output_path, route_samples, args.speed)
    report = {
        "status": "valid",
        "valid": True,
        "generated_at": datetime.now(timezone.utc)
        .astimezone()
        .isoformat(timespec="seconds"),
        "method": (
            "original_go2_distance_sampling_then_geometry_densify"
            if args.densify_maximum_gap > 0.0
            else "original_go2_distance_sampling"
        ),
        "source_raw_file": raw_path.name,
        "source_raw_sha256": file_sha256(raw_path),
        "output_route_file": output_path.name,
        "output_route_sha256": file_sha256(output_path),
        "raw_samples": len(raw_samples),
        "route_points": len(route_samples),
        "minimum_distance_m": args.minimum_distance,
        "default_speed_mps": args.speed,
        "despiking": False,
        "simplification": False,
        "resampling": args.densify_maximum_gap > 0.0,
        "geometry_anchor_points": len(geometry_anchors),
        "geometry_anchors_preserved": True,
        "densify_maximum_gap_m": args.densify_maximum_gap,
        "straightening": False,
        "final_point_forced": False,
        "yaw_source": "raw_odometry",
        "first_source_raw_id": route_samples[0][0],
        "last_source_raw_id": route_samples[-1][0],
    }
    report.update(route_statistics(route_samples))
    write_json(report_path, report)
    if preview_path is not None:
        if args.densify_maximum_gap > 0.0:
            route_label = (
                "original Go2 0.4 m geometry, densified to <= "
                f"{args.densify_maximum_gap:.2f} m"
            )
            title = "xbf4 raw vs geometry-preserving dense route"
        else:
            route_label = "original Go2 0.4 m sampling"
            title = "xbf4 raw vs original Go2 0.4 m sampling"
        write_preview(
            preview_path,
            raw_samples,
            route_samples,
            route_label,
            title,
        )

    print(
        "ORIGINAL_GO2_ROUTE_BUILT "
        f"raw={len(raw_samples)} route_points={len(route_samples)} "
        f"length_m={report['route_length_m']:.3f} "
        f"mean_gap_m={report['mean_gap_m']:.3f} "
        f"max_turn_deg={report['maximum_adjacent_turn_deg']:.1f}"
    )
    print(f"OUTPUT={output_path}")
    print(f"REPORT={report_path}")
    if preview_path is not None:
        print(f"PREVIEW={preview_path}")


if __name__ == "__main__":
    main()

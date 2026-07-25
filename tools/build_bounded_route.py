#!/usr/bin/env python3
"""Build a resampled route while strictly bounding deviation from raw XY."""

import argparse
import csv
import hashlib
import importlib.util
import json
import math
import os
from datetime import datetime, timezone
from pathlib import Path

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
ROUTE_QUALITY_PATH = (
    REPO_ROOT
    / "orin_go2_fastlio_ws/src/go2_fastlio_patrol"
    / "go2_fastlio_patrol/route_quality.py"
)


def load_route_quality():
    spec = importlib.util.spec_from_file_location(
        "go2_route_quality",
        ROUTE_QUALITY_PATH,
    )
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load route quality module")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def file_sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def read_raw(path, route_quality):
    samples = []
    speeds = []
    with open(path, "r", newline="") as handle:
        reader = csv.DictReader(handle)
        required = {"x", "y"}
        if not required.issubset(reader.fieldnames or []):
            raise RuntimeError("raw CSV must contain x and y columns")
        for row in reader:
            x = float(row["x"])
            y = float(row["y"])
            yaw = float(row.get("yaw", "0") or 0.0)
            speed = float(row.get("v", "0.2") or 0.2)
            if not all(math.isfinite(value) for value in (x, y, yaw, speed)):
                continue
            samples.append(route_quality.RouteSample(x, y, yaw))
            speeds.append(speed)
    if len(samples) < 3:
        raise RuntimeError("raw CSV must contain at least three finite rows")
    return samples, speeds


def movement_sample(samples, minimum_distance, route_quality):
    if minimum_distance <= 0.0:
        return list(samples)
    output = [samples[0]]
    for sample in samples[1:]:
        if (
            route_quality.sample_distance(output[-1], sample)
            >= minimum_distance
        ):
            output.append(sample)
    if (
        route_quality.sample_distance(output[-1], samples[-1])
        >= 0.005
    ):
        output.append(samples[-1])
    return output


def write_route(path, samples, speed):
    temporary = Path(str(path) + ".tmp")
    with open(temporary, "w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["id", "x", "y", "yaw", "v"])
        for index, sample in enumerate(samples):
            writer.writerow(
                [
                    index,
                    f"{sample.x:.6f}",
                    f"{sample.y:.6f}",
                    f"{sample.yaw:.6f}",
                    f"{speed:.3f}",
                ]
            )
    os.replace(temporary, path)


def raw_to_route_deviations(raw_samples, route_samples):
    raw_xy = np.asarray(
        [(sample.x, sample.y) for sample in raw_samples],
        dtype=float,
    )
    route_xy = np.asarray(
        [(sample.x, sample.y) for sample in route_samples],
        dtype=float,
    )
    starts = route_xy[:-1]
    directions = route_xy[1:] - starts
    denominators = np.sum(directions * directions, axis=1)
    if np.any(denominators <= 1e-12):
        raise RuntimeError("generated route contains a zero-length segment")

    deviations = []
    for point in raw_xy:
        ratios = np.sum((point - starts) * directions, axis=1)
        ratios = np.clip(ratios / denominators, 0.0, 1.0)
        projections = starts + ratios[:, None] * directions
        distances = np.linalg.norm(point - projections, axis=1)
        deviations.append(float(np.min(distances)))
    return np.asarray(deviations)


def write_report(path, report):
    temporary = Path(str(path) + ".tmp")
    with open(temporary, "w") as handle:
        json.dump(report, handle, ensure_ascii=False, indent=2)
        handle.write("\n")
    os.replace(temporary, path)


def write_preview(path, raw_samples, route_samples):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    raw_xy = np.asarray([(sample.x, sample.y) for sample in raw_samples])
    route_xy = np.asarray([(sample.x, sample.y) for sample in route_samples])
    figure, axis = plt.subplots(figsize=(9, 11), dpi=150)
    axis.plot(
        raw_xy[:, 0],
        raw_xy[:, 1],
        color="#ef4444",
        linewidth=0.7,
        alpha=0.55,
        label="raw xbf4",
    )
    axis.plot(
        route_xy[:, 0],
        route_xy[:, 1],
        color="#2563eb",
        linewidth=1.1,
        alpha=0.95,
        label="bounded 3 cm route",
    )
    axis.scatter(
        [route_xy[0, 0]],
        [route_xy[0, 1]],
        color="#16a34a",
        s=24,
        label="start",
        zorder=3,
    )
    axis.set_aspect("equal", adjustable="box")
    axis.set_xlabel("x (m)")
    axis.set_ylabel("y (m)")
    axis.set_title("xbf4 raw vs bounded 3 cm route")
    axis.grid(True, alpha=0.2)
    axis.legend(loc="best")
    figure.tight_layout()
    figure.savefig(path)
    plt.close(figure)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("raw_csv")
    parser.add_argument("output_csv")
    parser.add_argument("--report")
    parser.add_argument("--preview")
    parser.add_argument("--max-deviation", type=float, default=0.03)
    parser.add_argument("--spacing", type=float, default=0.20)
    parser.add_argument("--source-min-distance", type=float, default=0.30)
    parser.add_argument("--max-turn-deg", type=float, default=90.0)
    parser.add_argument("--speed", type=float, default=0.20)
    args = parser.parse_args()

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

    route_quality = load_route_quality()
    raw_samples, _ = read_raw(raw_path, route_quality)
    movement_samples = movement_sample(
        raw_samples,
        args.source_min_distance,
        route_quality,
    )
    deduplicated_movement = route_quality._deduplicate(
        movement_samples
    )
    geometry_samples, prefilter_spikes_removed = (
        route_quality.remove_isolated_spikes(
            deduplicated_movement,
            lateral_threshold=0.06,
        )
    )
    route_samples, build_report = route_quality.build_clean_route(
        geometry_samples,
        normal_spacing=args.spacing,
        turn_spacing=args.spacing,
        turn_angle_deg=5.0,
        turn_zone_distance=0.50,
        simplify_tolerance=args.max_deviation,
        spike_lateral_threshold=0.06,
        straighten_stable_sections=False,
    )
    source_deviations = raw_to_route_deviations(
        geometry_samples,
        route_samples,
    )
    raw_deviations = raw_to_route_deviations(raw_samples, route_samples)
    maximum_source_deviation = float(np.max(source_deviations))
    if maximum_source_deviation > args.max_deviation + 1e-6:
        raise RuntimeError(
            "generated route violates deviation bound: "
            f"{maximum_source_deviation:.6f} > "
            f"{args.max_deviation:.6f}"
        )

    turn_degrees = []
    for index in range(len(route_samples) - 1):
        turn_degrees.append(
            abs(
                math.degrees(
                    route_quality.angle_difference(
                        route_samples[index + 1].yaw,
                        route_samples[index].yaw,
                    )
                )
            )
        )
    maximum_turn = max(turn_degrees, default=0.0)
    if maximum_turn > args.max_turn_deg + 1e-6:
        raise RuntimeError(
            "generated route contains a reverse spike: "
            f"{maximum_turn:.3f}deg > {args.max_turn_deg:.3f}deg"
        )

    write_route(output_path, route_samples, args.speed)
    report = dict(build_report)
    report.update(
        {
            "status": "valid",
            "valid": True,
            "generated_at": datetime.now(timezone.utc)
            .astimezone()
            .isoformat(timespec="seconds"),
            "method": "bounded_rdp_no_global_fit",
            "source_raw_file": raw_path.name,
            "source_raw_sha256": file_sha256(raw_path),
            "output_route_file": output_path.name,
            "output_route_sha256": file_sha256(output_path),
            "body_yaw_used_for_geometry": False,
            "global_line_regression": False,
            "corner_intersection_replacement": False,
            "source_motion_samples": len(movement_samples),
            "bounded_geometry_samples": len(geometry_samples),
            "source_min_distance_m": args.source_min_distance,
            "prefilter_isolated_spikes_removed": (
                prefilter_spikes_removed
            ),
            "requested_max_deviation_m": args.max_deviation,
            "requested_max_deviation_applies_to": (
                "movement_sampled_source_after_stationary_noise_filter"
            ),
            "source_to_route_mean_m": float(
                np.mean(source_deviations)
            ),
            "source_to_route_p95_m": float(
                np.quantile(source_deviations, 0.95)
            ),
            "source_to_route_max_m": maximum_source_deviation,
            "raw_to_route_mean_m": float(np.mean(raw_deviations)),
            "raw_to_route_median_m": float(np.median(raw_deviations)),
            "raw_to_route_p95_m": float(
                np.quantile(raw_deviations, 0.95)
            ),
            "raw_to_route_p99_m": float(
                np.quantile(raw_deviations, 0.99)
            ),
            "raw_to_route_max_m": float(np.max(raw_deviations)),
            "raw_points_over_3cm_ratio": float(
                np.mean(raw_deviations > 0.03)
            ),
            "maximum_adjacent_turn_deg": maximum_turn,
            "adjacent_turns_over_10deg": sum(
                value > 10.0 for value in turn_degrees
            ),
            "adjacent_turns_over_30deg": sum(
                value > 30.0 for value in turn_degrees
            ),
            "adjacent_turns_over_90deg": sum(
                value > 90.0 for value in turn_degrees
            ),
        }
    )
    write_report(report_path, report)
    if preview_path is not None:
        write_preview(preview_path, raw_samples, route_samples)

    print(
        "BOUNDED_ROUTE_BUILT "
        f"raw={len(raw_samples)} movement={len(movement_samples)} "
        f"source={len(geometry_samples)} "
        f"route_points={len(route_samples)} "
        f"vertices={build_report['simplified_vertices']} "
        f"length_m={build_report['route_length_m']:.3f} "
        f"mean_deviation_m={report['raw_to_route_mean_m']:.4f} "
        f"p95_deviation_m={report['raw_to_route_p95_m']:.4f} "
        f"source_max_deviation_m={maximum_source_deviation:.4f} "
        f"max_turn_deg={maximum_turn:.1f}"
    )
    print(f"OUTPUT={output_path}")
    print(f"REPORT={report_path}")
    if preview_path is not None:
        print(f"PREVIEW={preview_path}")


if __name__ == "__main__":
    main()

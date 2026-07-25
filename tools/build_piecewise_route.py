#!/usr/bin/env python3
"""Build a patrol-ready piecewise-linear route from a dense raw odometry CSV.

The fitter uses only x/y geometry.  It deliberately does not use body yaw to
project the path because a quadruped can move with a persistent crab angle.
"""

import argparse
import csv
import hashlib
import json
import math
import os
from datetime import datetime, timezone
from pathlib import Path

import numpy as np


def angle_difference(left, right):
    return (left - right + math.pi) % (2.0 * math.pi) - math.pi


def distance(left, right):
    return float(np.linalg.norm(left - right))


def read_raw(path):
    points = []
    with open(path, "r", newline="") as handle:
        reader = csv.DictReader(handle)
        required = {"x", "y"}
        if not required.issubset(reader.fieldnames or []):
            raise RuntimeError("raw CSV must contain x and y columns")
        for row in reader:
            point = np.array([float(row["x"]), float(row["y"])], dtype=float)
            if not np.all(np.isfinite(point)):
                continue
            points.append(point)
    if len(points) < 3:
        raise RuntimeError("raw CSV must contain at least three finite points")
    return np.asarray(points)


def deduplicate(points, minimum_distance):
    output = [points[0]]
    for point in points[1:]:
        if distance(point, output[-1]) >= minimum_distance:
            output.append(point)
    return np.asarray(output)


def cumulative_distance(points):
    if len(points) <= 1:
        return np.zeros(len(points), dtype=float)
    steps = np.linalg.norm(np.diff(points, axis=0), axis=1)
    return np.concatenate(([0.0], np.cumsum(steps)))


def resample_by_arc(points, spacing):
    arc = cumulative_distance(points)
    locations = np.arange(0.0, arc[-1], spacing)
    if len(locations) == 0 or arc[-1] - locations[-1] > 1e-9:
        locations = np.append(locations, arc[-1])
    return np.column_stack(
        (
            np.interp(locations, arc, points[:, 0]),
            np.interp(locations, arc, points[:, 1]),
        )
    )


class SegmentCosts:
    def __init__(self, points):
        self.points = points
        self.count = len(points)
        self.xy = np.vstack((np.zeros(2), np.cumsum(points, axis=0)))
        self.xx = np.concatenate(([0.0], np.cumsum(points[:, 0] ** 2)))
        self.yy = np.concatenate(([0.0], np.cumsum(points[:, 1] ** 2)))
        self.cross = np.concatenate(
            ([0.0], np.cumsum(points[:, 0] * points[:, 1]))
        )

    def orthogonal_squared_error(self, start, end):
        count = end - start
        sum_x, sum_y = self.xy[end] - self.xy[start]
        scatter_xx = (
            self.xx[end] - self.xx[start] - sum_x * sum_x / count
        )
        scatter_yy = (
            self.yy[end] - self.yy[start] - sum_y * sum_y / count
        )
        scatter_xy = (
            self.cross[end]
            - self.cross[start]
            - sum_x * sum_y / count
        )
        trace = scatter_xx + scatter_yy
        discriminant = math.sqrt(
            max(
                0.0,
                (scatter_xx - scatter_yy) ** 2
                + 4.0 * scatter_xy * scatter_xy,
            )
        )
        return max(0.0, 0.5 * (trace - discriminant))


def segment_route(points, penalty, minimum_points):
    costs = SegmentCosts(points)
    count = len(points)
    best = np.full(count + 1, np.inf)
    previous = np.full(count + 1, -1, dtype=int)
    best[0] = -penalty

    for end in range(2, count + 1):
        for start in range(0, end - 1):
            if start > 0 and end - start < minimum_points:
                continue
            candidate = (
                best[start]
                + costs.orthogonal_squared_error(start, end)
                + penalty
            )
            if candidate < best[end]:
                best[end] = candidate
                previous[end] = start

    segments = []
    end = count
    while end > 0:
        start = int(previous[end])
        if start < 0:
            raise RuntimeError("piecewise fit failed to cover the route")
        segments.append([start, end])
        end = start
    segments.reverse()
    return segments


def fit_line(points, start, end, anchor=None):
    samples = points[start:end]
    center = np.mean(samples, axis=0)
    _, _, vectors = np.linalg.svd(samples - center, full_matrices=False)
    direction = vectors[0]
    if float(np.dot(direction, samples[-1] - samples[0])) < 0.0:
        direction = -direction
    if anchor == "start":
        center = points[start]
    elif anchor == "end":
        center = points[end - 1]
    return center, direction


def line_angle(left, right):
    left_heading = math.atan2(left[1], left[0])
    right_heading = math.atan2(right[1], right[0])
    return abs(angle_difference(right_heading, left_heading))


def merge_nearly_parallel(points, segments, minimum_turn_angle):
    segments = [list(segment) for segment in segments]
    changed = True
    while changed and len(segments) > 1:
        changed = False
        fits = [fit_line(points, *segment) for segment in segments]
        for index in range(len(segments) - 1):
            if (
                line_angle(fits[index][1], fits[index + 1][1])
                < minimum_turn_angle
            ):
                segments[index : index + 2] = [
                    [segments[index][0], segments[index + 1][1]]
                ]
                changed = True
                break
    return segments


def line_intersection(first, second):
    first_center, first_direction = first
    second_center, second_direction = second
    cross = float(np.cross(first_direction, second_direction))
    if abs(cross) <= 1e-9:
        return None
    offset = second_center - first_center
    scale = float(np.cross(offset, second_direction)) / cross
    return first_center + scale * first_direction


def build_vertices(points, segments, maximum_intersection_distance):
    fits = []
    for index, (start, end) in enumerate(segments):
        anchor = None
        if index == 0:
            anchor = "start"
        elif index == len(segments) - 1:
            anchor = "end"
        fits.append(fit_line(points, start, end, anchor=anchor))

    vertices = [points[0]]
    intersection_fallbacks = 0
    for index in range(len(segments) - 1):
        boundary = points[segments[index][1] - 1]
        vertex = line_intersection(fits[index], fits[index + 1])
        if (
            vertex is None
            or distance(vertex, boundary) > maximum_intersection_distance
        ):
            vertex = boundary
            intersection_fallbacks += 1
        if distance(vertex, vertices[-1]) >= 0.005:
            vertices.append(vertex)
    if distance(points[-1], vertices[-1]) >= 0.005:
        vertices.append(points[-1])
    return np.asarray(vertices), intersection_fallbacks


def vertex_turns(vertices):
    turns = [False] * len(vertices)
    angles = [0.0] * len(vertices)
    for index in range(1, len(vertices) - 1):
        incoming = vertices[index] - vertices[index - 1]
        outgoing = vertices[index + 1] - vertices[index]
        angle = line_angle(incoming, outgoing)
        angles[index] = angle
    return turns, angles


def resample_polyline(
    vertices,
    normal_spacing,
    turn_spacing,
    turn_angle,
    turn_zone_distance,
):
    _, angles = vertex_turns(vertices)
    turns = [angle >= turn_angle for angle in angles]
    output = [vertices[0]]

    for index in range(len(vertices) - 1):
        start = vertices[index]
        end = vertices[index + 1]
        length = distance(start, end)
        if length < 0.005:
            continue
        travelled = 0.0
        while travelled < length - 1e-9:
            near_start_turn = (
                turns[index] and travelled < turn_zone_distance
            )
            near_end_turn = (
                turns[index + 1]
                and length - travelled <= turn_zone_distance
            )
            spacing = (
                turn_spacing
                if near_start_turn or near_end_turn
                else normal_spacing
            )
            next_distance = min(length, travelled + spacing)
            point = start + (end - start) * (next_distance / length)
            if distance(point, output[-1]) >= 0.005:
                output.append(point)
            travelled = next_distance
    return np.asarray(output)


def point_segment_distance(point, start, end):
    direction = end - start
    denominator = float(np.dot(direction, direction))
    if denominator <= 1e-12:
        return distance(point, start)
    ratio = float(np.dot(point - start, direction)) / denominator
    ratio = min(1.0, max(0.0, ratio))
    projection = start + ratio * direction
    return distance(point, projection)


def route_deviations(points, vertices):
    values = []
    for point in points:
        values.append(
            min(
                point_segment_distance(
                    point,
                    vertices[index],
                    vertices[index + 1],
                )
                for index in range(len(vertices) - 1)
            )
        )
    return np.asarray(values)


def write_route(path, points, speed):
    temporary = Path(str(path) + ".tmp")
    with open(temporary, "w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["id", "x", "y", "yaw", "v"])
        for index, point in enumerate(points):
            if index < len(points) - 1:
                other = points[index + 1]
                yaw = math.atan2(
                    other[1] - point[1],
                    other[0] - point[0],
                )
            else:
                other = points[index - 1]
                yaw = math.atan2(
                    point[1] - other[1],
                    point[0] - other[0],
                )
            writer.writerow(
                [
                    index,
                    f"{point[0]:.6f}",
                    f"{point[1]:.6f}",
                    f"{yaw:.6f}",
                    f"{speed:.3f}",
                ]
            )
    os.replace(temporary, path)


def write_json(path, data):
    temporary = Path(str(path) + ".tmp")
    with open(temporary, "w") as handle:
        json.dump(data, handle, ensure_ascii=False, indent=2)
        handle.write("\n")
    os.replace(temporary, path)


def file_sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("raw_csv")
    parser.add_argument("output_csv")
    parser.add_argument("--report")
    parser.add_argument("--fit-spacing", type=float, default=0.50)
    parser.add_argument("--segment-penalty", type=float, default=3.0)
    parser.add_argument("--minimum-segment-length", type=float, default=2.0)
    parser.add_argument("--merge-angle-deg", type=float, default=5.0)
    parser.add_argument("--maximum-intersection-distance", type=float, default=8.0)
    parser.add_argument("--normal-spacing", type=float, default=0.40)
    parser.add_argument("--turn-spacing", type=float, default=0.30)
    parser.add_argument("--turn-angle-deg", type=float, default=5.0)
    parser.add_argument("--turn-zone-distance", type=float, default=0.50)
    parser.add_argument("--speed", type=float, default=0.20)
    args = parser.parse_args()

    raw_path = Path(args.raw_csv).resolve()
    output_path = Path(args.output_csv).resolve()
    report_path = (
        Path(args.report).resolve()
        if args.report
        else output_path.with_suffix(".quality.json")
    )
    output_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.parent.mkdir(parents=True, exist_ok=True)

    raw = read_raw(raw_path)
    deduplicated = deduplicate(raw, 0.005)
    fit_points = resample_by_arc(deduplicated, args.fit_spacing)
    minimum_points = max(
        3,
        int(math.ceil(args.minimum_segment_length / args.fit_spacing)),
    )
    segments = segment_route(
        fit_points,
        args.segment_penalty,
        minimum_points,
    )
    segments = merge_nearly_parallel(
        fit_points,
        segments,
        math.radians(args.merge_angle_deg),
    )
    vertices, intersection_fallbacks = build_vertices(
        fit_points,
        segments,
        args.maximum_intersection_distance,
    )
    route = resample_polyline(
        vertices,
        args.normal_spacing,
        args.turn_spacing,
        math.radians(args.turn_angle_deg),
        args.turn_zone_distance,
    )
    deviations = route_deviations(deduplicated, vertices)
    route_steps = np.linalg.norm(np.diff(route, axis=0), axis=1)
    _, angles = vertex_turns(vertices)
    turn_degrees = [
        math.degrees(value)
        for value in angles[1:-1]
        if value > 1e-9
    ]

    write_route(output_path, route, args.speed)
    report = {
        "status": "valid",
        "valid": True,
        "generated_at": datetime.now(timezone.utc).astimezone().isoformat(
            timespec="seconds"
        ),
        "source_raw_file": raw_path.name,
        "source_raw_sha256": file_sha256(raw_path),
        "output_route_file": output_path.name,
        "output_route_sha256": file_sha256(output_path),
        "method": "xy_piecewise_orthogonal_regression",
        "body_yaw_used_for_geometry": False,
        "raw_points": int(len(raw)),
        "deduplicated_points": int(len(deduplicated)),
        "fit_points": int(len(fit_points)),
        "control_vertices": int(len(vertices)),
        "route_points": int(len(route)),
        "source_length_m": float(cumulative_distance(deduplicated)[-1]),
        "route_length_m": float(np.sum(route_steps)),
        "maximum_route_gap_m": float(np.max(route_steps)),
        "mean_route_gap_m": float(np.mean(route_steps)),
        "raw_to_centerline_mean_m": float(np.mean(deviations)),
        "raw_to_centerline_p95_m": float(np.quantile(deviations, 0.95)),
        "raw_to_centerline_max_m": float(np.max(deviations)),
        "segment_penalty": args.segment_penalty,
        "fit_spacing_m": args.fit_spacing,
        "minimum_segment_length_m": args.minimum_segment_length,
        "merged_below_angle_deg": args.merge_angle_deg,
        "intersection_fallbacks": intersection_fallbacks,
        "normal_spacing_m": args.normal_spacing,
        "turn_spacing_m": args.turn_spacing,
        "turn_zone_distance_m": args.turn_zone_distance,
        "corner_count": len(turn_degrees),
        "corner_angles_deg": turn_degrees,
        "control_vertex_xy": [
            [float(point[0]), float(point[1])] for point in vertices
        ],
    }
    write_json(report_path, report)
    print(
        "ROUTE_BUILT"
        f" raw={len(raw)} deduplicated={len(deduplicated)}"
        f" vertices={len(vertices)} route_points={len(route)}"
        f" length_m={report['route_length_m']:.3f}"
        f" p95_deviation_m={report['raw_to_centerline_p95_m']:.3f}"
        f" max_deviation_m={report['raw_to_centerline_max_m']:.3f}"
    )
    print(f"OUTPUT={output_path}")
    print(f"REPORT={report_path}")


if __name__ == "__main__":
    main()

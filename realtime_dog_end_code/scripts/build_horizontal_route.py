#!/usr/bin/env python3
"""Build a gravity-level route from an exact route-recording black box.

The output is not a guessed 2-D correction.  Every CSV waypoint is paired back
to its recorded full 3-D FAST-LIO pose.  A plane is fitted to the complete
recording trajectory, and the route plus full pose orientations are rigidly
rotated into that measured horizontal plane.
"""

from __future__ import annotations

import argparse
import bisect
import csv
import hashlib
import json
import math
from pathlib import Path

import numpy as np

from horizontal_frame import (
    angle_between,
    normalize_angle,
    normalize_vector,
    quaternion_conjugate,
    quaternion_from_two_vectors,
    quaternion_multiply,
    quaternion_rotate,
    quaternion_yaw,
    vector_norm,
)


def sha256_file(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        while True:
            chunk = handle.read(1024 * 1024)
            if not chunk:
                break
            digest.update(chunk)
    return digest.hexdigest()


def load_route(path):
    with Path(path).open(newline="") as handle:
        rows = list(csv.DictReader(handle))
    route = []
    for index, row in enumerate(rows):
        route.append(
            {
                "id": int(row.get("id", index)),
                "x": float(row["x"]),
                "y": float(row["y"]),
                "yaw": float(row["yaw"]),
                "v": float(row["v"]),
            }
        )
    if len(route) < 2:
        raise RuntimeError("route must contain at least two points")
    return route


def load_telemetry(path):
    selected = {"odom": [], "livox_imu": [], "sport": []}
    with Path(path).open() as handle:
        for line in handle:
            record = json.loads(line)
            kind = record.get("kind")
            if kind in selected:
                selected[kind].append(record)
    for records in selected.values():
        records.sort(key=lambda item: float(item["wall_time"]))
    if not all(selected.values()):
        missing = [kind for kind, records in selected.items() if not records]
        raise RuntimeError("missing telemetry kinds: %s" % ",".join(missing))
    return selected


def orientation_quaternion(record):
    orientation = record["data"]["orientation"]
    return (
        float(orientation["qx"]),
        float(orientation["qy"]),
        float(orientation["qz"]),
        float(orientation["qw"]),
    )


def position_xyz(record):
    position = record["data"]["position"]
    return tuple(float(position[axis]) for axis in "xyz")


def nearest_record(records, times, wall_time, maximum_gap=0.08):
    insertion = bisect.bisect_left(times, wall_time)
    candidates = [
        index
        for index in (insertion - 1, insertion)
        if 0 <= index < len(records)
    ]
    if not candidates:
        return None
    index = min(candidates, key=lambda item: abs(times[item] - wall_time))
    if abs(times[index] - wall_time) > maximum_gap:
        return None
    return records[index]


def pair_route_to_odometry(route, odometry):
    """Greedily recover the exact Odometry sample used by route_recorder."""
    pairs = []
    cursor = 0
    position_errors = []
    yaw_errors = []
    for point in route:
        best = None
        first_exact = None
        for index in range(cursor, len(odometry)):
            record = odometry[index]
            position = position_xyz(record)
            position_error = math.hypot(
                position[0] - point["x"], position[1] - point["y"]
            )
            yaw_error = abs(
                normalize_angle(
                    quaternion_yaw(orientation_quaternion(record))
                    - point["yaw"]
                )
            )
            score = position_error + yaw_error
            if best is None or score < best[0]:
                best = (score, index, position_error, yaw_error)
            if position_error <= 2e-5 and yaw_error <= 2e-4:
                first_exact = (score, index, position_error, yaw_error)
                break
        match = first_exact or best
        if match is None:
            raise RuntimeError("cannot pair route point %s" % point["id"])
        _, index, position_error, yaw_error = match
        if position_error > 0.002 or yaw_error > 0.002:
            raise RuntimeError(
                "route/telemetry pair exceeds proof tolerance at id=%s: "
                "position=%.6fm yaw=%.6frad"
                % (point["id"], position_error, yaw_error)
            )
        pairs.append(odometry[index])
        position_errors.append(position_error)
        yaw_errors.append(yaw_error)
        cursor = index + 1
    return pairs, {
        "count": len(pairs),
        "maximum_position_error_m": max(position_errors),
        "maximum_yaw_error_deg": math.degrees(max(yaw_errors)),
    }


def fit_plane(records):
    positions = np.asarray([position_xyz(record) for record in records])
    design = np.column_stack(
        [positions[:, 0], positions[:, 1], np.ones(len(positions))]
    )
    coefficients = np.linalg.lstsq(
        design, positions[:, 2], rcond=None
    )[0]
    predicted = design @ coefficients
    residual = positions[:, 2] - predicted
    total = np.sum((positions[:, 2] - positions[:, 2].mean()) ** 2)
    residual_sum = np.sum(residual**2)
    normal = np.asarray(
        [-coefficients[0], -coefficients[1], 1.0], dtype=float
    )
    normal /= np.linalg.norm(normal)
    return {
        "coefficients": tuple(float(value) for value in coefficients),
        "normal": tuple(float(value) for value in normal),
        "r_squared": float(1.0 - residual_sum / total),
        "residual_p95_m": float(np.percentile(np.abs(residual), 95)),
        "residual_max_m": float(np.max(np.abs(residual))),
        "tilt_deg": math.degrees(
            math.atan(math.hypot(coefficients[0], coefficients[1]))
        ),
        "samples": len(records),
    }


def median_unit(vectors):
    array = np.asarray(vectors, dtype=float)
    median = np.median(array, axis=0)
    result = normalize_vector(median)
    if result is None:
        raise RuntimeError("cannot normalize median vector")
    return result


def static_calibration(
    telemetry,
    plane_normal,
    static_start,
    static_end,
):
    odometry = [
        record
        for record in telemetry["odom"]
        if static_start <= float(record["wall_time"]) <= static_end
    ]
    livox = telemetry["livox_imu"]
    sport = telemetry["sport"]
    livox_times = [float(record["wall_time"]) for record in livox]
    sport_times = [float(record["wall_time"]) for record in sport]

    lidar_measured = []
    body_measured = []
    sensor_targets = []
    pairing_gaps = []
    for odom in odometry:
        wall_time = float(odom["wall_time"])
        livox_record = nearest_record(livox, livox_times, wall_time)
        sport_record = nearest_record(sport, sport_times, wall_time)
        if livox_record is None or sport_record is None:
            continue
        lidar_data = livox_record["data"]
        sport_imu = sport_record["data"]["imu"]
        lidar_gyro = tuple(
            float(lidar_data["angular_velocity"][axis]) for axis in "xyz"
        )
        body_gyro = tuple(float(value) for value in sport_imu["gyroscope"])
        if vector_norm(lidar_gyro) > 0.04 or vector_norm(body_gyro) > 0.04:
            continue
        lidar_acceleration = tuple(
            float(lidar_data["linear_acceleration"][axis])
            for axis in "xyz"
        )
        body_acceleration = tuple(
            float(value) for value in sport_imu["accelerometer"]
        )
        lidar_up = normalize_vector(lidar_acceleration)
        body_up = normalize_vector(body_acceleration)
        q_map_from_sensor = orientation_quaternion(odom)
        target_sensor_up = normalize_vector(
            quaternion_rotate(
                quaternion_conjugate(q_map_from_sensor), plane_normal
            )
        )
        if lidar_up is None or body_up is None or target_sensor_up is None:
            continue
        lidar_measured.append(lidar_up)
        body_measured.append(body_up)
        sensor_targets.append(target_sensor_up)
        pairing_gaps.append(
            max(
                abs(float(livox_record["wall_time"]) - wall_time),
                abs(float(sport_record["wall_time"]) - wall_time),
            )
        )
    if len(sensor_targets) < 20:
        raise RuntimeError(
            "fewer than 20 quiet paired calibration samples: %d"
            % len(sensor_targets)
        )

    lidar_up = median_unit(lidar_measured)
    body_up = median_unit(body_measured)
    target_sensor_up = median_unit(sensor_targets)
    q_lidar_correction = quaternion_from_two_vectors(
        lidar_up, target_sensor_up
    )
    q_sensor_from_body = quaternion_from_two_vectors(
        body_up, target_sensor_up
    )
    corrected_lidar_up = quaternion_rotate(q_lidar_correction, lidar_up)
    body_derived_sensor_up = quaternion_rotate(
        q_sensor_from_body, body_up
    )
    return {
        "sample_count": len(sensor_targets),
        "static_window_epoch": [static_start, static_end],
        "maximum_pairing_gap_s": max(pairing_gaps),
        "lidar_gravity_measured_unit_xyz": lidar_up,
        "body_gravity_measured_unit_xyz": body_up,
        "plane_derived_sensor_up_unit_xyz": target_sensor_up,
        "q_lidar_gravity_correction_xyzw": q_lidar_correction,
        "q_sensor_from_body_xyzw": q_sensor_from_body,
        "post_correction_lidar_error_deg": math.degrees(
            angle_between(corrected_lidar_up, target_sensor_up)
        ),
        "post_calibration_body_error_deg": math.degrees(
            angle_between(body_derived_sensor_up, target_sensor_up)
        ),
        "measured_body_lidar_gravity_angle_deg": math.degrees(
            angle_between(body_up, lidar_up)
        ),
    }


def write_route(path, route):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(("id", "x", "y", "yaw", "v"))
        for index, point in enumerate(route):
            writer.writerow(
                (
                    index,
                    "%.9f" % point["x"],
                    "%.9f" % point["y"],
                    "%.9f" % point["yaw"],
                    "%.3f" % point["v"],
                )
            )


def build(args):
    source_route = Path(args.route).resolve()
    telemetry_path = Path(args.telemetry).resolve()
    output_route = Path(args.output_route).resolve()
    output_metadata = Path(args.output_metadata).resolve()
    route = load_route(source_route)
    telemetry = load_telemetry(telemetry_path)
    plane_odometry = [
        record
        for record in telemetry["odom"]
        if args.plane_start
        <= float(record["wall_time"])
        <= args.plane_end
    ]
    if len(plane_odometry) < 100:
        raise RuntimeError("plane fitting has fewer than 100 odometry samples")
    plane = fit_plane(plane_odometry)
    if plane["r_squared"] < 0.99:
        raise RuntimeError(
            "recording trajectory is not a stable plane: R2=%.6f"
            % plane["r_squared"]
        )

    paired_odometry, pairing = pair_route_to_odometry(
        route, telemetry["odom"]
    )
    calibration = static_calibration(
        telemetry,
        plane["normal"],
        args.static_start,
        args.static_end,
    )
    q_ground_from_map = quaternion_from_two_vectors(
        plane["normal"], (0.0, 0.0, 1.0)
    )
    q_sensor_from_body = calibration["q_sensor_from_body_xyzw"]
    origin = position_xyz(paired_odometry[0])
    horizontal_route = []
    leveled_z = []
    for point, odom in zip(route, paired_odometry):
        position = position_xyz(odom)
        relative = tuple(
            position[index] - origin[index] for index in range(3)
        )
        ground_position = quaternion_rotate(q_ground_from_map, relative)
        body_orientation = quaternion_multiply(
            quaternion_multiply(
                q_ground_from_map, orientation_quaternion(odom)
            ),
            q_sensor_from_body,
        )
        horizontal_route.append(
            {
                "x": ground_position[0],
                "y": ground_position[1],
                "yaw": quaternion_yaw(body_orientation),
                "v": point["v"],
            }
        )
        leveled_z.append(ground_position[2])

    write_route(output_route, horizontal_route)
    metadata = {
        "schema": "go2.horizontal_route.v1",
        "method": (
            "exact_3d_waypoint_reconstruction_plus_flat_floor_rigid_rotation"
        ),
        "source_route": str(source_route),
        "source_route_sha256": sha256_file(source_route),
        "source_telemetry": str(telemetry_path),
        "horizontal_route": str(output_route),
        "horizontal_route_sha256": sha256_file(output_route),
        "route_points": len(horizontal_route),
        "pairing_proof": pairing,
        "recording_plane": {
            "model": "z=a*x+b*y+c",
            "a": plane["coefficients"][0],
            "b": plane["coefficients"][1],
            "c": plane["coefficients"][2],
            "normal_xyz": plane["normal"],
            "tilt_deg": plane["tilt_deg"],
            "r_squared": plane["r_squared"],
            "residual_p95_m": plane["residual_p95_m"],
            "residual_max_m": plane["residual_max_m"],
            "samples": plane["samples"],
            "q_ground_from_recording_map_xyzw": q_ground_from_map,
        },
        "mount_and_gravity_calibration": calibration,
        "horizontal_route_z_check_m": {
            "minimum": min(leveled_z),
            "maximum": max(leveled_z),
            "range": max(leveled_z) - min(leveled_z),
            "p95_absolute": float(
                np.percentile(np.abs(np.asarray(leveled_z)), 95)
            ),
        },
        "runtime_contract": {
            "position": "p_ground=q_ground_from_map*p_fastlio_map",
            "orientation": (
                "q_ground_from_body=q_ground_from_map*"
                "q_fastlio_map_from_sensor*q_sensor_from_body"
            ),
            "map_leveling": (
                "freeze one startup rotation after paired lidar/body "
                "gravity agreement; never substitute a scalar body yaw"
            ),
        },
    }
    output_metadata.parent.mkdir(parents=True, exist_ok=True)
    output_metadata.write_text(
        json.dumps(metadata, indent=2, ensure_ascii=False) + "\n"
    )
    return metadata


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--route", required=True)
    parser.add_argument("--telemetry", required=True)
    parser.add_argument("--output-route", required=True)
    parser.add_argument("--output-metadata", required=True)
    parser.add_argument("--plane-start", required=True, type=float)
    parser.add_argument("--plane-end", required=True, type=float)
    parser.add_argument("--static-start", required=True, type=float)
    parser.add_argument("--static-end", required=True, type=float)
    return parser.parse_args()


def main():
    metadata = build(parse_args())
    print(json.dumps(metadata, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()

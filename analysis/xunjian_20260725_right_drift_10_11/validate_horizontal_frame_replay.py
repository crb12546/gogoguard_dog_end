#!/usr/bin/env python3
"""Replay the production horizontal-frame startup gate on saved runs."""

from __future__ import annotations

import bisect
import json
import math
import sys
from pathlib import Path


HERE = Path(__file__).resolve().parent
PROJECT = HERE.parents[1]
SCRIPTS = PROJECT / "orin_go2_fastlio_ws" / "scripts"
sys.path.insert(0, str(SCRIPTS))

from horizontal_frame import (  # noqa: E402
    HorizontalFrameEstimator,
    angle_between,
)


FORMAL = PROJECT / "analysis" / "xunjian_20260725_formal_xbf8"
METADATA = FORMAL / "routes" / "xbf8.csv.horizontal.json"
PLANE_EVIDENCE = HERE / "rigid_frame_evidence.json"
RUNS = {
    "recording": (
        FORMAL / "recording" / "experiment_telemetry.jsonl",
        1784960669.242379,
        "recording",
    ),
    "08_raw_lio_yaw": (
        FORMAL / "patrol" / "experiment_telemetry.jsonl",
        1784961384.956508,
        "08_raw_lio_yaw",
    ),
    "10_body_yaw_first": (
        HERE / "xunjian-20260725-10" / "experiment_telemetry.jsonl",
        1784968744.0242708,
        "10_body_yaw_first",
    ),
    "11_body_yaw_second": (
        HERE / "xunjian-20260725-11" / "experiment_telemetry.jsonl",
        1784969369.6975238,
        "11_body_yaw_second",
    ),
}


def load_window(path, start):
    selected = {"odom": [], "livox_imu": [], "sport": []}
    with path.open() as handle:
        for line in handle:
            record = json.loads(line)
            if not start - 15.0 <= float(record["wall_time"]) <= start:
                continue
            if record.get("kind") in selected:
                selected[record["kind"]].append(record)
    for records in selected.values():
        records.sort(key=lambda item: float(item["wall_time"]))
    return selected


def nearest(records, times, wall_time):
    insertion = bisect.bisect_left(times, wall_time)
    candidates = [
        index
        for index in (insertion - 1, insertion)
        if 0 <= index < len(records)
    ]
    if not candidates:
        return None
    index = min(candidates, key=lambda item: abs(times[item] - wall_time))
    if abs(times[index] - wall_time) > 0.08:
        return None
    return records[index]


def replay(path, motion_start, plane_normal, calibration):
    data = load_window(path, motion_start)
    livox_times = [
        float(record["wall_time"]) for record in data["livox_imu"]
    ]
    sport_times = [
        float(record["wall_time"]) for record in data["sport"]
    ]
    estimator = HorizontalFrameEstimator(
        calibration["q_sensor_from_body_xyzw"],
        calibration["q_lidar_gravity_correction_xyzw"],
        minimum_samples=15,
        maximum_spread_rad=math.radians(1.5),
        maximum_source_disagreement_rad=math.radians(3.0),
        maximum_gyro_rad_s=0.08,
    )
    locked_at = None
    for odometry in data["odom"]:
        wall_time = float(odometry["wall_time"])
        lidar = nearest(data["livox_imu"], livox_times, wall_time)
        sport = nearest(data["sport"], sport_times, wall_time)
        if lidar is None or sport is None:
            continue
        orientation = odometry["data"]["orientation"]
        lidar_data = lidar["data"]
        sport_imu = sport["data"]["imu"]
        locked = estimator.add_sample(
            (
                orientation["qx"],
                orientation["qy"],
                orientation["qz"],
                orientation["qw"],
            ),
            [
                lidar_data["linear_acceleration"][axis]
                for axis in "xyz"
            ],
            sport_imu["accelerometer"],
            [
                lidar_data["angular_velocity"][axis]
                for axis in "xyz"
            ],
            sport_imu["gyroscope"],
        )
        if locked:
            locked_at = wall_time
            break
    diagnostics = estimator.diagnostics()
    diagnostics["locked_before_motion_s"] = (
        motion_start - locked_at if locked_at is not None else None
    )
    diagnostics["startup_up_vs_full_trajectory_plane_deg"] = (
        math.degrees(angle_between(estimator.map_up, plane_normal))
        if estimator.ready
        else None
    )
    return diagnostics


def main():
    metadata = json.loads(METADATA.read_text())
    calibration = metadata["mount_and_gravity_calibration"]
    plane_evidence = json.loads(PLANE_EVIDENCE.read_text())[
        "data_only_claims"
    ]["flat_floor_lio_plane"]
    results = {}
    for name, (path, motion_start, plane_key) in RUNS.items():
        results[name] = replay(
            path,
            motion_start,
            plane_evidence[plane_key]["normal_xyz"],
            calibration,
        )
    output = {
        "schema": "go2.horizontal_frame_replay.v1",
        "production_thresholds": {
            "minimum_samples": 15,
            "maximum_spread_deg": 1.5,
            "maximum_lidar_body_source_disagreement_deg": 3.0,
            "maximum_gyro_rad_s": 0.08,
        },
        "runs": results,
        "all_runs_lock": all(item["ready"] for item in results.values()),
        "maximum_replay_plane_residual_deg": max(
            item["startup_up_vs_full_trajectory_plane_deg"]
            for item in results.values()
        ),
    }
    destination = HERE / "horizontal_frame_replay.json"
    destination.write_text(
        json.dumps(output, indent=2, ensure_ascii=False) + "\n"
    )
    print(json.dumps(output, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()

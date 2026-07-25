#!/usr/bin/env python3
"""Reproducible comparison of patrol runs 06 and 07.

The script reads ROS 2 CDR records directly from the downloaded sqlite bags, so
it does not require a ROS installation on the analysis computer.
"""

from __future__ import annotations

import csv
import hashlib
import json
import math
import re
import sqlite3
import struct
from dataclasses import dataclass
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


ROOT = Path(__file__).resolve().parent
RUN_NAMES = ("xunjian-20260725-06", "xunjian-20260725-07")


class CdrReader:
    def __init__(self, data: bytes):
        if len(data) < 4:
            raise ValueError("short CDR buffer")
        self.data = data
        self.base = 4
        self.pos = 4
        # ROS 2 bags from this robot use CDR little endian (encapsulation 0x0001).
        self.endian = "<" if data[1] == 1 else ">"

    def align(self, size: int) -> None:
        relative = self.pos - self.base
        self.pos += (-relative) % size

    def unpack(self, fmt: str, alignment: int):
        self.align(alignment)
        size = struct.calcsize(fmt)
        value = struct.unpack_from(self.endian + fmt, self.data, self.pos)
        self.pos += size
        return value[0] if len(value) == 1 else value

    def i32(self) -> int:
        return int(self.unpack("i", 4))

    def u32(self) -> int:
        return int(self.unpack("I", 4))

    def f64(self) -> float:
        return float(self.unpack("d", 8))

    def string(self) -> str:
        length = self.u32()
        raw = self.data[self.pos : self.pos + length]
        self.pos += length
        if raw.endswith(b"\0"):
            raw = raw[:-1]
        return raw.decode("utf-8", errors="replace")


def parse_odometry(data: bytes) -> dict[str, float]:
    r = CdrReader(data)
    sec = r.i32()
    nanosec = r.u32()
    frame_id = r.string()
    child_frame_id = r.string()
    x, y, z = (r.f64() for _ in range(3))
    qx, qy, qz, qw = (r.f64() for _ in range(4))
    # Skip pose covariance.
    for _ in range(36):
        r.f64()
    vx, vy, vz, wx, wy, wz = (r.f64() for _ in range(6))
    return {
        "stamp": sec + nanosec * 1e-9,
        "frame_id": frame_id,
        "child_frame_id": child_frame_id,
        "x": x,
        "y": y,
        "z": z,
        "qx": qx,
        "qy": qy,
        "qz": qz,
        "qw": qw,
        "vx": vx,
        "vy": vy,
        "vz": vz,
        "wx": wx,
        "wy": wy,
        "wz": wz,
    }


def parse_twist(data: bytes) -> np.ndarray:
    r = CdrReader(data)
    return np.asarray([r.f64() for _ in range(6)], dtype=float)


def load_topic(db_path: Path, topic: str, parser):
    connection = sqlite3.connect(str(db_path))
    try:
        topic_row = connection.execute(
            "SELECT id FROM topics WHERE name = ?", (topic,)
        ).fetchone()
        if topic_row is None:
            raise KeyError(f"{topic} missing from {db_path}")
        rows = connection.execute(
            "SELECT timestamp, data FROM messages WHERE topic_id = ? "
            "ORDER BY timestamp",
            (topic_row[0],),
        )
        result = []
        for timestamp, data in rows:
            result.append((timestamp * 1e-9, parser(data)))
        return result
    finally:
        connection.close()


def load_route(path: Path) -> np.ndarray:
    with path.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    return np.asarray(
        [[float(row["x"]), float(row["y"]), float(row["yaw"])] for row in rows]
    )


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def quaternion_to_euler(q: np.ndarray) -> np.ndarray:
    qx, qy, qz, qw = q
    sinr = 2.0 * (qw * qx + qy * qz)
    cosr = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr, cosr)
    sinp = 2.0 * (qw * qy - qz * qx)
    pitch = math.asin(float(np.clip(sinp, -1.0, 1.0)))
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny, cosy)
    return np.asarray([roll, pitch, yaw])


def quaternion_relative_angle(q1: np.ndarray, q2: np.ndarray) -> float:
    q1 = q1 / np.linalg.norm(q1)
    q2 = q2 / np.linalg.norm(q2)
    return 2.0 * math.acos(float(np.clip(abs(np.dot(q1, q2)), -1.0, 1.0)))


def fit_plane(points: np.ndarray) -> dict[str, np.ndarray | float]:
    center = points.mean(axis=0)
    _, _, vh = np.linalg.svd(points - center, full_matrices=False)
    normal = vh[-1]
    if normal[2] < 0:
        normal = -normal
    normal /= np.linalg.norm(normal)
    signed = (points - center) @ normal
    # z = ax + by + c
    design = np.column_stack((points[:, 0], points[:, 1], np.ones(len(points))))
    coefficients, *_ = np.linalg.lstsq(design, points[:, 2], rcond=None)
    return {
        "normal": normal,
        "tilt_deg": math.degrees(math.acos(float(np.clip(normal[2], -1.0, 1.0)))),
        "rms_m": float(np.sqrt(np.mean(signed * signed))),
        "max_m": float(np.max(np.abs(signed))),
        "z_coefficients": coefficients,
    }


def route_transform(original: np.ndarray, runtime: np.ndarray):
    """Fit runtime_xy = R @ original_xy + t and return inverse mapper."""
    source = original[:, :2]
    target = runtime[:, :2]
    source_center = source.mean(axis=0)
    target_center = target.mean(axis=0)
    u, _, vt = np.linalg.svd((source - source_center).T @ (target - target_center))
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0:
        vt[-1, :] *= -1
        rotation = vt.T @ u.T
    translation = target_center - rotation @ source_center
    residual = target - (source @ rotation.T + translation)

    def inverse(points: np.ndarray) -> np.ndarray:
        return (points - translation) @ rotation

    return rotation, translation, inverse, float(np.max(np.linalg.norm(residual, axis=1)))


def project_monotonic(points: np.ndarray, route: np.ndarray):
    """Project points to a forward-only polyline using a local search window."""
    xy = route[:, :2]
    segment = xy[1:] - xy[:-1]
    lengths = np.linalg.norm(segment, axis=1)
    cumulative = np.concatenate(([0.0], np.cumsum(lengths)))
    indices = np.zeros(len(points), dtype=int)
    fractions = np.zeros(len(points))
    signed = np.zeros(len(points))
    distances = np.zeros(len(points))
    progress = np.zeros(len(points))
    previous = 0
    for row, point in enumerate(points):
        lo = max(0, previous - 3)
        hi = min(len(segment), previous + 14)
        best = None
        for index in range(lo, hi):
            vector = segment[index]
            denom = float(vector @ vector)
            fraction = 0.0 if denom == 0 else float((point - xy[index]) @ vector / denom)
            fraction = float(np.clip(fraction, 0.0, 1.0))
            closest = xy[index] + fraction * vector
            delta = point - closest
            distance = float(np.linalg.norm(delta))
            if best is None or distance < best[0]:
                cross = vector[0] * delta[1] - vector[1] * delta[0]
                best = (
                    distance,
                    index,
                    fraction,
                    math.copysign(distance, cross) if cross else 0.0,
                )
        assert best is not None
        distance, index, fraction, side = best
        # Keep route progress monotonic. This matches the forward-only run mode.
        scalar = cumulative[index] + fraction * lengths[index]
        if row and scalar < progress[row - 1]:
            scalar = progress[row - 1]
        else:
            previous = max(previous, index)
        indices[row] = index
        fractions[row] = fraction
        signed[row] = side
        distances[row] = distance
        progress[row] = scalar
    return {
        "index": indices,
        "fraction": fractions,
        "signed": signed,
        "distance": distances,
        "progress": progress,
        "route_arc": cumulative,
    }


def percentile_stats(values: np.ndarray) -> dict[str, float]:
    values = np.asarray(values)
    return {
        "mean": float(np.mean(values)),
        "median": float(np.median(values)),
        "p90": float(np.percentile(values, 90)),
        "p95": float(np.percentile(values, 95)),
        "max": float(np.max(values)),
    }


def parse_follower_log(path: Path) -> dict[str, float | int]:
    expression = re.compile(
        r"nearest=(?P<nearest>\d+), target=(?P<target>\d+), "
        r"dist=(?P<dist>[-0-9.]+), alpha=(?P<alpha>[-0-9.]+), "
        r"vx=(?P<vx>[-0-9.]+), yaw_rate=(?P<yaw>[-0-9.]+)"
    )
    records = []
    for line in path.read_text(errors="replace").splitlines():
        match = expression.search(line)
        if match:
            records.append({key: float(value) for key, value in match.groupdict().items()})
    return {
        "samples": len(records),
        "last_nearest": int(records[-1]["nearest"]),
        "max_abs_alpha_rad": max(abs(row["alpha"]) for row in records),
        "max_abs_yaw_rate": max(abs(row["yaw"]) for row in records),
    }


@dataclass
class Run:
    name: str
    directory: Path
    route_original: np.ndarray
    route_runtime: np.ndarray
    anchor: dict
    odom_receive_time: np.ndarray
    odom: np.ndarray
    odom_header_time: np.ndarray
    patrol_receive_time: np.ndarray
    patrol_cmd: np.ndarray
    motion_mask: np.ndarray
    motion: np.ndarray
    motion_time: np.ndarray
    motion_original_xy: np.ndarray
    projection: dict
    plane_all: dict
    plane_first_leg: dict
    follower: dict


def load_run(name: str) -> Run:
    directory = ROOT / "runs" / name
    db = directory / "rosbag" / "rosbag_0.db3"
    odom_rows = load_topic(db, "/Odometry", parse_odometry)
    patrol_rows = load_topic(db, "/patrol_cmd", parse_twist)
    odom_receive_time = np.asarray([row[0] for row in odom_rows])
    odom_header_time = np.asarray([row[1]["stamp"] for row in odom_rows])
    odom = np.asarray(
        [
            [
                row[1][key]
                for key in ("x", "y", "z", "qx", "qy", "qz", "qw", "vx", "vy", "vz")
            ]
            for row in odom_rows
        ]
    )
    patrol_receive_time = np.asarray([row[0] for row in patrol_rows])
    patrol_cmd = np.asarray([row[1] for row in patrol_rows])
    active = np.linalg.norm(patrol_cmd[:, [0, 1, 5]], axis=1) > 1e-6
    active_times = patrol_receive_time[active]
    # Include odometry from the first nonzero command through the last one.
    motion_mask = (odom_receive_time >= active_times[0]) & (
        odom_receive_time <= active_times[-1]
    )
    motion = odom[motion_mask]
    motion_time = odom_receive_time[motion_mask] - active_times[0]
    original = load_route(directory / "route_original.csv")
    runtime = load_route(directory / "route_runtime.csv")
    _, _, inverse, transform_residual = route_transform(original, runtime)
    if transform_residual > 2e-6:
        raise RuntimeError(f"route transform residual too high: {transform_residual}")
    motion_original_xy = inverse(motion[:, :2])
    projection = project_monotonic(motion_original_xy, original)
    first_leg_end = projection["route_arc"][124]
    first_leg_mask = projection["progress"] <= first_leg_end
    return Run(
        name=name,
        directory=directory,
        route_original=original,
        route_runtime=runtime,
        anchor=json.loads((directory / "manual_anchor.json").read_text()),
        odom_receive_time=odom_receive_time,
        odom=odom,
        odom_header_time=odom_header_time,
        patrol_receive_time=patrol_receive_time,
        patrol_cmd=patrol_cmd,
        motion_mask=motion_mask,
        motion=motion,
        motion_time=motion_time,
        motion_original_xy=motion_original_xy,
        projection=projection,
        plane_all=fit_plane(motion[:, :3]),
        plane_first_leg=fit_plane(motion[first_leg_mask, :3]),
        follower=parse_follower_log(directory / "waypoint_follower.log"),
    )


def odom_timing(run: Run) -> dict[str, float | int]:
    delta = np.diff(run.odom_header_time)
    return {
        "samples": len(run.odom),
        "mean_hz": float(1.0 / np.mean(delta)),
        "median_period_ms": float(np.median(delta) * 1000.0),
        "p95_period_ms": float(np.percentile(delta, 95) * 1000.0),
        "max_period_ms": float(np.max(delta) * 1000.0),
        "gaps_over_150ms": int(np.sum(delta > 0.15)),
        "gaps_over_250ms": int(np.sum(delta > 0.25)),
        "estimated_missing_100ms_periods": int(
            np.sum(np.maximum(0, np.rint(delta / 0.1).astype(int) - 1))
        ),
        "receive_minus_header_ms_mean": float(
            np.mean(run.odom_receive_time - run.odom_header_time) * 1000.0
        ),
        "receive_minus_header_ms_p95": float(
            np.percentile(run.odom_receive_time - run.odom_header_time, 95) * 1000.0
        ),
        "receive_minus_header_ms_max": float(
            np.max(run.odom_receive_time - run.odom_header_time) * 1000.0
        ),
    }


def command_stats(run: Run) -> dict[str, float | int]:
    cmd = run.patrol_cmd
    active = np.linalg.norm(cmd[:, [0, 1, 5]], axis=1) > 1e-6
    return {
        "samples": len(cmd),
        "active_samples": int(np.sum(active)),
        "max_abs_vx": float(np.max(np.abs(cmd[:, 0]))),
        "max_abs_vy": float(np.max(np.abs(cmd[:, 1]))),
        "max_abs_yaw_rate": float(np.max(np.abs(cmd[:, 5]))),
    }


def receiver_stats(path: Path) -> dict[str, float | int]:
    text = path.read_text(errors="replace")
    rows = []
    triple_names = (
        "udp_gap_ms",
        "udp_transit_ms",
        "move_call_ms",
        "sender_to_move_done_ms",
    )
    integer_names = ("count", "seq_gaps", "invalid", "last_seq")
    for line in text.splitlines():
        if "TIMING_RECEIVER" not in line:
            continue
        row = {}
        for name in triple_names:
            match = re.search(
                rf"{name}=([0-9.-]+)/([0-9.-]+)/([0-9.-]+)", line
            )
            if match:
                row[name] = tuple(float(value) for value in match.groups())
        for name in integer_names:
            match = re.search(rf"{name}=([0-9]+)", line)
            if match:
                row[name] = int(match.group(1))
        if all(name in row for name in triple_names + integer_names):
            rows.append(row)
    return {
        "reports": len(rows),
        "max_seq_gaps": max(row["seq_gaps"] for row in rows),
        "max_invalid": max(row["invalid"] for row in rows),
        "last_seq": max(row["last_seq"] for row in rows),
        "max_udp_gap_ms": max(row["udp_gap_ms"][2] for row in rows),
        "max_udp_transit_ms": max(row["udp_transit_ms"][2] for row in rows),
        "max_move_call_ms": max(row["move_call_ms"][2] for row in rows),
        "max_sender_to_move_done_ms": max(
            row["sender_to_move_done_ms"][2] for row in rows
        ),
    }


def plane_json(plane: dict) -> dict:
    return {
        "normal": np.asarray(plane["normal"]).tolist(),
        "tilt_deg": plane["tilt_deg"],
        "rms_cm": plane["rms_m"] * 100.0,
        "max_cm": plane["max_m"] * 100.0,
        "z_equals_ax_by_c": np.asarray(plane["z_coefficients"]).tolist(),
    }


def compare_progress(run_a: Run, run_b: Run, last_index: int) -> dict:
    samples = np.linspace(
        0.0, min(run_a.projection["route_arc"][last_index], run_b.projection["route_arc"][last_index]), 800
    )
    positions = []
    for run in (run_a, run_b):
        progress = run.projection["progress"]
        unique_progress, unique_index = np.unique(progress, return_index=True)
        positions.append(
            np.column_stack(
                [
                    np.interp(samples, unique_progress, run.motion_original_xy[unique_index, axis])
                    for axis in range(2)
                ]
            )
        )
    delta = np.linalg.norm(positions[0] - positions[1], axis=1)
    return {
        "through_waypoint": last_index,
        "route_distance_m": float(samples[-1]),
        "xy_difference_m": percentile_stats(delta),
    }


def make_plot(runs: list[Run]) -> None:
    fig, axes = plt.subplots(1, 2, figsize=(14, 6), constrained_layout=True)
    route = runs[0].route_original
    for run in runs:
        label = run.name[-2:]
        axes[0].plot(
            run.motion_original_xy[:, 0],
            run.motion_original_xy[:, 1],
            linewidth=1.2,
            label=f"run {label} odometry",
        )
    axes[0].plot(route[:, 0], route[:, 1], "k--", linewidth=0.9, label="CSV")
    axes[0].axis("equal")
    axes[0].set_title("All downloaded motion (mapped to original CSV frame)")
    axes[0].set_xlabel("x [m]")
    axes[0].set_ylabel("y [m]")
    axes[0].legend()
    for run in runs:
        label = run.name[-2:]
        first = run.projection["progress"] <= run.projection["route_arc"][124]
        axes[1].plot(
            run.motion_original_xy[first, 0],
            run.motion_original_xy[first, 1],
            linewidth=1.4,
            label=f"run {label} odometry",
        )
    axes[1].plot(
        route[:125, 0], route[:125, 1], "k--", linewidth=0.9, label="CSV"
    )
    axes[1].axis("equal")
    axes[1].set_title("First leg through waypoint 124")
    axes[1].set_xlabel("x [m]")
    axes[1].set_ylabel("y [m]")
    axes[1].legend()
    fig.savefig(ROOT / "run06_vs_run07_trajectory.png", dpi=180)
    plt.close(fig)


def main() -> None:
    runs = [load_run(name) for name in RUN_NAMES]
    result = {"runs": {}, "comparison": {}}
    for run in runs:
        anchor_q = np.asarray(
            [
                run.anchor["current_anchor"][key]
                for key in ("qx", "qy", "qz", "qw")
            ]
        )
        euler_deg = np.degrees(quaternion_to_euler(anchor_q))
        first_leg = run.projection["progress"] <= run.projection["route_arc"][124]
        reached = int(np.max(run.projection["index"]))
        first_leg_xyz = run.motion[first_leg, :3]
        result["runs"][run.name] = {
            "anchor": {
                "x": run.anchor["current_anchor"]["x"],
                "y": run.anchor["current_anchor"]["y"],
                "z": run.anchor["current_anchor"]["z"],
                "quaternion_xyzw": anchor_q.tolist(),
                "euler_roll_pitch_yaw_deg": euler_deg.tolist(),
                "saved_2d_yaw_deg": math.degrees(run.anchor["current_anchor"]["yaw"]),
                "stability": run.anchor["stability"],
            },
            "motion": {
                "duration_s": float(run.motion_time[-1] - run.motion_time[0]),
                "odom_samples": len(run.motion),
                "last_projected_route_segment": reached,
                "delta_xyz_m": (run.motion[-1, :3] - run.motion[0, :3]).tolist(),
                "first_leg_time_s": float(
                    run.motion_time[first_leg][-1] - run.motion_time[first_leg][0]
                ),
                "first_leg_csv_distance_m": float(
                    run.projection["route_arc"][124]
                ),
                "first_leg_3d_cumulative_distance_m": float(
                    np.sum(np.linalg.norm(np.diff(first_leg_xyz, axis=0), axis=1))
                ),
            },
            "route_error_all_reached_m": percentile_stats(
                run.projection["distance"]
            ),
            "route_error_first_leg_m": percentile_stats(
                run.projection["distance"][first_leg]
            ),
            "route_signed_error_first_leg_m": percentile_stats(
                run.projection["signed"][first_leg]
            ),
            "plane_all": plane_json(run.plane_all),
            "plane_first_leg": plane_json(run.plane_first_leg),
            "odom_timing": odom_timing(run),
            "patrol_commands": command_stats(run),
            "udp_sdk_delivery": receiver_stats(
                run.directory / "go2_sdk2_udp_receiver.log"
            ),
            "follower_log": run.follower,
        }

    q = []
    for run in runs:
        q.append(
            np.asarray(
                [
                    run.anchor["current_anchor"][key]
                    for key in ("qx", "qy", "qz", "qw")
                ]
            )
        )
    eulers = [quaternion_to_euler(item) for item in q]
    normals = [np.asarray(run.plane_first_leg["normal"]) for run in runs]
    route_hashes = {
        run.name: sha256(run.directory / "route_original.csv") for run in runs
    }
    original_follower = (
        Path.home()
        / "Desktop/go2_original_linux_code_20260708/source_original_candidate"
        / "orin_go2_fastlio_ws/src/go2_fastlio_patrol/go2_fastlio_patrol"
        / "waypoint_follower.py"
    )
    captured_follower = (
        ROOT / "previous_boot/remote_source/waypoint_follower_go2_2.py"
    )
    result["comparison"] = {
        "route_original_sha256": route_hashes,
        "same_original_csv_sha256": len(set(route_hashes.values())) == 1,
        "runtime_follower_sha256": sha256(captured_follower),
        "desktop_original_follower_sha256": (
            sha256(original_follower) if original_follower.exists() else None
        ),
        "runtime_follower_matches_desktop_original": (
            original_follower.exists()
            and sha256(captured_follower) == sha256(original_follower)
        ),
        "anchor_saved_2d_yaw_difference_deg": abs(
            math.degrees(
                runs[0].anchor["current_anchor"]["yaw"]
                - runs[1].anchor["current_anchor"]["yaw"]
            )
        ),
        "anchor_euler_difference_roll_pitch_yaw_deg": np.degrees(
            eulers[1] - eulers[0]
        ).tolist(),
        "anchor_full_quaternion_relative_angle_deg": math.degrees(
            quaternion_relative_angle(q[0], q[1])
        ),
        "first_leg_plane_normal_angle_deg": math.degrees(
            math.acos(float(np.clip(normals[0] @ normals[1], -1.0, 1.0)))
        ),
        "first_leg_plane_tilt_difference_deg": abs(
            runs[0].plane_first_leg["tilt_deg"]
            - runs[1].plane_first_leg["tilt_deg"]
        ),
        "cross_run_progress_comparison_first_leg": compare_progress(
            runs[0], runs[1], 124
        ),
        "cross_run_progress_comparison_shared_run06_extent": compare_progress(
            runs[0], runs[1], 190
        ),
    }
    make_plot(runs)
    output = ROOT / "run06_vs_run07_metrics.json"
    output.write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n")
    print(json.dumps(result, indent=2, ensure_ascii=False))
    print(f"\nWrote {output}")
    print(f"Wrote {ROOT / 'run06_vs_run07_trajectory.png'}")


if __name__ == "__main__":
    main()

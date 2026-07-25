#!/usr/bin/env python3
"""Reproducible forensic analysis for the 2026-07-25 xbf8 record/patrol pair.

All inputs are the read-only evidence copied from the robot.  The script writes
only derived JSON/CSV/PNG files next to itself.
"""

from __future__ import annotations

import csv
import hashlib
import json
import math
import re
from pathlib import Path
from typing import Any, Iterable

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


ROOT = Path(__file__).resolve().parent
PATROL = ROOT / "patrol"
RECORDING = ROOT / "recording"
ROUTES = ROOT / "routes"


def read_jsonl(path: Path, wanted: set[str] | None = None) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    with path.open(encoding="utf-8") as fh:
        for line in fh:
            if not line.strip():
                continue
            row = json.loads(line)
            if wanted is None or row.get("kind") in wanted:
                rows.append(row)
    return rows


def read_route(path: Path) -> dict[str, np.ndarray]:
    cols: dict[str, list[float]] = {"id": [], "x": [], "y": [], "yaw": [], "v": []}
    with path.open(newline="", encoding="utf-8-sig") as fh:
        for row in csv.DictReader(fh):
            for key in cols:
                cols[key].append(float(row[key]))
    return {key: np.asarray(value, dtype=float) for key, value in cols.items()}


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as fh:
        for block in iter(lambda: fh.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def pct(values: Iterable[float], percentiles: tuple[int, ...] = (50, 95, 100)) -> dict[str, float | int | None]:
    a = np.asarray(list(values), dtype=float)
    a = a[np.isfinite(a)]
    if not len(a):
        return {"count": 0, **{f"p{p}": None for p in percentiles}}
    result: dict[str, float | int | None] = {"count": int(len(a))}
    for p in percentiles:
        result[f"p{p}"] = float(np.percentile(a, p))
    return result


def wrap_angle(value: np.ndarray | float) -> np.ndarray | float:
    return (value + np.pi) % (2.0 * np.pi) - np.pi


def route_geometry(points: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    vec = np.diff(points, axis=0)
    seg_len = np.linalg.norm(vec, axis=1)
    cumulative = np.r_[0.0, np.cumsum(seg_len)]
    return vec, seg_len, cumulative


def project_to_polyline(
    points: np.ndarray, route: np.ndarray, chunk_size: int = 1000
) -> dict[str, np.ndarray]:
    """Globally project 2-D points onto a route polyline."""
    points = np.asarray(points, dtype=float)
    route = np.asarray(route, dtype=float)
    vec, seg_len, cumulative = route_geometry(route)
    denom = np.sum(vec * vec, axis=1)
    all_dist: list[np.ndarray] = []
    all_signed: list[np.ndarray] = []
    all_progress: list[np.ndarray] = []
    all_seg: list[np.ndarray] = []
    all_frac: list[np.ndarray] = []
    all_proj: list[np.ndarray] = []
    for start in range(0, len(points), chunk_size):
        p = points[start : start + chunk_size]
        rel = p[:, None, :] - route[:-1][None, :, :]
        frac = np.clip(np.einsum("nsi,si->ns", rel, vec) / denom[None, :], 0.0, 1.0)
        projected = route[:-1][None, :, :] + frac[:, :, None] * vec[None, :, :]
        delta = p[:, None, :] - projected
        dist2 = np.einsum("nsi,nsi->ns", delta, delta)
        seg = np.argmin(dist2, axis=1)
        idx = np.arange(len(p))
        chosen_frac = frac[idx, seg]
        chosen_delta = delta[idx, seg]
        chosen_vec = vec[seg]
        signed = (
            chosen_vec[:, 0] * chosen_delta[:, 1] - chosen_vec[:, 1] * chosen_delta[:, 0]
        ) / seg_len[seg]
        all_dist.append(np.sqrt(dist2[idx, seg]))
        all_signed.append(signed)
        all_progress.append(cumulative[seg] + chosen_frac * seg_len[seg])
        all_seg.append(seg)
        all_frac.append(chosen_frac)
        all_proj.append(projected[idx, seg])
    return {
        "distance": np.concatenate(all_dist),
        "signed": np.concatenate(all_signed),
        "progress": np.concatenate(all_progress),
        "segment": np.concatenate(all_seg),
        "fraction": np.concatenate(all_frac),
        "projected": np.concatenate(all_proj),
    }


def fit_plane(points_xyz: np.ndarray) -> dict[str, Any]:
    design = np.c_[points_xyz[:, :2], np.ones(len(points_xyz))]
    coeff = np.linalg.lstsq(design, points_xyz[:, 2], rcond=None)[0]
    residual = points_xyz[:, 2] - design @ coeff
    normal = np.asarray([-coeff[0], -coeff[1], 1.0], dtype=float)
    normal /= np.linalg.norm(normal)
    if normal[2] < 0:
        normal *= -1
    return {
        "coeff": coeff,
        "normal": normal,
        "residual": residual,
        "tilt_deg": math.degrees(math.atan(math.hypot(coeff[0], coeff[1]))),
        "rms_m": float(np.sqrt(np.mean(residual * residual))),
        "abs_residual_m": pct(np.abs(residual)),
    }


def quaternion(row: dict[str, Any]) -> np.ndarray:
    return np.asarray([row["qw"], row["qx"], row["qy"], row["qz"]], dtype=float)


def relative_rotation_angle(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    q1 = q1 / np.linalg.norm(q1, axis=1, keepdims=True)
    q2 = q2 / np.linalg.norm(q2, axis=1, keepdims=True)
    dot = np.abs(np.sum(q1 * q2, axis=1))
    return 2.0 * np.arccos(np.clip(dot, 0.0, 1.0))


def attitude_invariant(
    odom: list[dict[str, Any]],
    sport: list[dict[str, Any]],
    active_start: float,
    active_end: float,
) -> dict[str, Any]:
    odom = [r for r in odom if active_start <= r["wall_time"] <= active_end]
    sport = [r for r in sport if active_start <= r["wall_time"] <= active_end]
    sport_t = np.asarray([r["wall_time"] for r in sport])
    paired_t: list[float] = []
    paired_odom: list[np.ndarray] = []
    paired_body: list[np.ndarray] = []
    gaps: list[float] = []
    for row in odom:
        t = row["wall_time"]
        j = int(np.searchsorted(sport_t, t))
        candidates = [k for k in (j - 1, j) if 0 <= k < len(sport)]
        if not candidates:
            continue
        k = min(candidates, key=lambda candidate: abs(sport_t[candidate] - t))
        gap = abs(sport_t[k] - t)
        if gap > 0.1:
            continue
        paired_t.append(t)
        paired_odom.append(quaternion(row["data"]["orientation"]))
        paired_body.append(quaternion(sport[k]["data"]["imu"]["orientation"]))
        gaps.append(gap)
    t = np.asarray(paired_t)
    q_odom = np.asarray(paired_odom)
    q_body = np.asarray(paired_body)
    intervals: dict[str, Any] = {}
    for seconds in (1.0, 5.0, 10.0):
        left: list[int] = []
        right: list[int] = []
        for i, t0 in enumerate(t):
            j = int(np.searchsorted(t, t0 + seconds))
            if j < len(t) and abs((t[j] - t0) - seconds) <= 0.15:
                left.append(i)
                right.append(j)
        li = np.asarray(left, dtype=int)
        ri = np.asarray(right, dtype=int)
        lidar = np.degrees(relative_rotation_angle(q_odom[li], q_odom[ri]))
        body = np.degrees(relative_rotation_angle(q_body[li], q_body[ri]))
        intervals[f"{seconds:g}s"] = {
            "pair_count": int(len(li)),
            "lidar_rotation_deg": pct(lidar),
            "body_rotation_deg": pct(body),
            "absolute_rotation_difference_deg": pct(np.abs(lidar - body)),
        }
    return {
        "paired_sample_count": int(len(t)),
        "pair_receive_time_gap_s": pct(gaps),
        "method": (
            "Compare relative rotation magnitudes over fixed intervals. This is invariant "
            "to constant world-frame and rigid mount rotations."
        ),
        "intervals": intervals,
    }


def phase_records(active: list[dict[str, Any]]) -> list[list[dict[str, Any]]]:
    phases: list[list[dict[str, Any]]] = []
    current: list[dict[str, Any]] = []
    direction: int | None = None
    for row in active:
        new_direction = int(row["control"]["direction"])
        if direction is not None and new_direction != direction:
            phases.append(current)
            current = []
        current.append(row)
        direction = new_direction
    if current:
        phases.append(current)
    return phases


def unique_odom(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    seen: set[int] = set()
    result: list[dict[str, Any]] = []
    for row in rows:
        seq = int(row["odom_callback_sequence"])
        if seq in seen:
            continue
        seen.add(seq)
        result.append(row)
    return result


def binned_metrics(
    progress: np.ndarray,
    signed: np.ndarray,
    distance: np.ndarray,
    width: float = 5.0,
) -> list[dict[str, Any]]:
    maximum = float(np.nanmax(progress)) if len(progress) else 0.0
    rows: list[dict[str, Any]] = []
    for lo in np.arange(0.0, maximum + width, width):
        keep = (progress >= lo) & (progress < lo + width)
        if not np.any(keep):
            continue
        rows.append(
            {
                "progress_start_m": float(lo),
                "progress_end_m": float(lo + width),
                "count": int(np.sum(keep)),
                "signed_median_m": float(np.median(signed[keep])),
                "abs_median_m": float(np.median(distance[keep])),
                "abs_p95_m": float(np.percentile(distance[keep], 95)),
                "abs_max_m": float(np.max(distance[keep])),
            }
        )
    return rows


def section_metrics(
    progress: np.ndarray,
    signed: np.ndarray,
    distance: np.ndarray,
    sections: tuple[tuple[float, float], ...] = ((0.0, 55.0), (60.0, 140.0), (145.0, 159.0)),
) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for lo, hi in sections:
        keep = (progress >= lo) & (progress < hi)
        if not np.any(keep):
            continue
        rows.append(
            {
                "progress_start_m": lo,
                "progress_end_m": hi,
                "count": int(np.sum(keep)),
                "signed_m": pct(signed[keep], (5, 50, 95)),
                "absolute_m": pct(distance[keep]),
            }
        )
    return rows


def json_safe(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(k): json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(v) for v in value]
    if isinstance(value, np.ndarray):
        return [json_safe(v) for v in value.tolist()]
    if isinstance(value, (np.floating, np.integer)):
        return value.item()
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def main() -> None:
    source_route = read_route(ROUTES / "xbf8.csv")
    runtime_route = read_route(PATROL / "route_runtime.csv")
    source_xy = np.c_[source_route["x"], source_route["y"]]
    runtime_xy = np.c_[runtime_route["x"], runtime_route["y"]]
    _, runtime_seg_len, runtime_cumulative = route_geometry(runtime_xy)

    # Recording motion window: point 0 was captured while the dog waited.  Point
    # 1 is the first sample after real walking began.
    recorder_log = (RECORDING / "route_recorder.log").read_text(encoding="utf-8")
    saved_times = np.asarray(
        [
            float(value)
            for value in re.findall(
                r"\[INFO\] \[(\d+\.\d+)\] \[route_recorder\]: saved point \d+:", recorder_log
            )
        ]
    )
    record_start = float(saved_times[1] - 1.0)
    record_end = float(saved_times[-1] + 1.0)

    recording_rows = read_jsonl(
        RECORDING / "experiment_telemetry.jsonl", {"odom", "sport", "low", "sport_request"}
    )
    record_all_odom = [row for row in recording_rows if row["kind"] == "odom"]
    record_all_sport = [row for row in recording_rows if row["kind"] == "sport"]
    record_odom = [
        row
        for row in record_all_odom
        if record_start <= row["wall_time"] <= record_end
    ]
    record_xyz = np.asarray(
        [
            [
                row["data"]["position"]["x"],
                row["data"]["position"]["y"],
                row["data"]["position"]["z"],
            ]
            for row in record_odom
        ]
    )
    recording_plane = fit_plane(record_xyz)
    recording_move_requests = [
        row
        for row in recording_rows
        if row["kind"] == "sport_request"
        and record_start <= row["wall_time"] <= record_end
        and row["data"].get("api_id") == 1008
        and isinstance(row["data"].get("parameter_json"), dict)
    ]
    recording_requested_x = np.asarray(
        [row["data"]["parameter_json"].get("x", 0.0) for row in recording_move_requests],
        dtype=float,
    )
    recording_requested_y = np.asarray(
        [row["data"]["parameter_json"].get("y", 0.0) for row in recording_move_requests],
        dtype=float,
    )
    recording_requested_yaw = np.asarray(
        [row["data"]["parameter_json"].get("z", 0.0) for row in recording_move_requests],
        dtype=float,
    )

    source_heading = np.arctan2(np.diff(source_xy[:, 1]), np.diff(source_xy[:, 0]))
    source_yaw_track_error = np.asarray(wrap_angle(source_heading - source_route["yaw"][:-1]))
    _, _, source_cumulative = route_geometry(source_xy)
    yaw_track_sections: list[dict[str, Any]] = []
    for lo, hi in ((0.0, 55.0), (60.0, 140.0), (145.0, 159.0)):
        keep = (source_cumulative[:-1] >= lo) & (source_cumulative[:-1] < hi)
        error_deg = np.degrees(source_yaw_track_error[keep])
        yaw_track_sections.append(
            {
                "progress_start_m": lo,
                "progress_end_m": hi,
                "count": int(np.sum(keep)),
                "signed_error_deg": pct(error_deg, (5, 50, 95)),
                "absolute_error_deg": pct(np.abs(error_deg)),
            }
        )

    # Unitree's body IMU has an arbitrary yaw origin, so align it by one
    # constant offset on the first straight.  After that alignment, a rigid
    # body-yaw source should continue to agree with the route tangent in every
    # direction.  This separates a dog-body heading problem from a LIO Euler
    # yaw/frame problem.
    all_odom_xy = np.asarray(
        [
            [row["data"]["position"]["x"], row["data"]["position"]["y"]]
            for row in record_all_odom
        ]
    )
    all_odom_time = np.asarray([row["wall_time"] for row in record_all_odom])
    all_sport_time = np.asarray([row["wall_time"] for row in record_all_sport])
    all_body_yaw = np.asarray(
        [row["data"]["imu"]["orientation"]["yaw"] for row in record_all_sport]
    )
    matched_body_yaw: list[float] = []
    matched_receive_gap: list[float] = []
    for point in source_xy:
        odom_index = int(np.argmin(np.sum((all_odom_xy - point) ** 2, axis=1)))
        t = all_odom_time[odom_index]
        sport_index = int(np.argmin(np.abs(all_sport_time - t)))
        matched_body_yaw.append(float(all_body_yaw[sport_index]))
        matched_receive_gap.append(float(abs(all_sport_time[sport_index] - t)))
    matched_body_yaw_a = np.asarray(matched_body_yaw)
    track_minus_body = np.asarray(
        wrap_angle(source_heading - matched_body_yaw_a[:-1])
    )
    track_minus_lio = source_yaw_track_error
    first_straight = (source_cumulative[:-1] >= 0.0) & (source_cumulative[:-1] < 55.0)
    body_constant_offset = float(np.median(track_minus_body[first_straight]))
    lio_constant_offset = float(np.median(track_minus_lio[first_straight]))
    body_aligned_error = np.asarray(wrap_angle(track_minus_body - body_constant_offset))
    lio_aligned_error = np.asarray(wrap_angle(track_minus_lio - lio_constant_offset))
    body_lio_sections: list[dict[str, Any]] = []
    for lo, hi in ((0.0, 55.0), (60.0, 140.0), (145.0, 159.0)):
        keep = (source_cumulative[:-1] >= lo) & (source_cumulative[:-1] < hi)
        body_error_deg = np.degrees(body_aligned_error[keep])
        lio_error_deg = np.degrees(lio_aligned_error[keep])
        body_lio_sections.append(
            {
                "progress_start_m": lo,
                "progress_end_m": hi,
                "body_imu_aligned_track_error_deg": {
                    "signed": pct(body_error_deg, (5, 50, 95)),
                    "absolute": pct(np.abs(body_error_deg)),
                },
                "lio_yaw_aligned_track_error_deg": {
                    "signed": pct(lio_error_deg, (5, 50, 95)),
                    "absolute": pct(np.abs(lio_error_deg)),
                },
            }
        )

    trace = read_jsonl(PATROL / "follower_control_trace.jsonl")
    controls = [row for row in trace if row.get("kind") == "control"]
    active = [
        row
        for row in controls
        if row["control"].get("motion_enabled") and row["pose"].get("x") is not None
    ]
    phases = phase_records(active)
    active_start = float(active[0]["wall_time"])
    active_end = float(active[-1]["wall_time"])

    patrol_rows = read_jsonl(
        PATROL / "experiment_telemetry.jsonl", {"odom", "sport", "low", "cmd_vel", "patrol_cmd"}
    )
    patrol_odom = [row for row in patrol_rows if row["kind"] == "odom"]
    patrol_sport = [row for row in patrol_rows if row["kind"] == "sport"]

    # The route lies on a very consistent plane in the LIO world.  Build a
    # coordinate basis on that plane to estimate distances in the physical
    # floor plane instead of the tilted LIO x-y projection.
    coeff = recording_plane["coeff"]
    normal = recording_plane["normal"]
    route_xyz = np.c_[
        runtime_xy,
        coeff[0] * runtime_xy[:, 0] + coeff[1] * runtime_xy[:, 1] + coeff[2],
    ]
    first_direction = route_xyz[min(30, len(route_xyz) - 1)] - route_xyz[0]
    first_direction -= normal * np.dot(first_direction, normal)
    e1 = first_direction / np.linalg.norm(first_direction)
    e2 = np.cross(normal, e1)
    origin = route_xyz[0]
    route_plane_xy = np.c_[
        (route_xyz - origin) @ e1,
        (route_xyz - origin) @ e2,
    ]
    _, route_plane_seg_len, route_plane_cumulative = route_geometry(route_plane_xy)

    phase_metrics: list[dict[str, Any]] = []
    phase_plot: list[dict[str, Any]] = []
    all_unique: list[dict[str, Any]] = []
    for phase_id, rows in enumerate(phases):
        unique = unique_odom(rows)
        all_unique.extend(unique)
        xy = np.asarray([[r["pose"]["x"], r["pose"]["y"]] for r in unique])
        xyz = np.asarray([[r["pose"]["x"], r["pose"]["y"], r["pose"]["z"]] for r in unique])
        progress = np.asarray(
            [
                runtime_cumulative[int(r["projection"]["segment"])]
                + float(r["projection"]["fraction"])
                * runtime_seg_len[int(r["projection"]["segment"])]
                for r in unique
            ]
        )
        signed = np.asarray([r["control"]["signed_cross_track_m"] for r in unique])
        distance = np.asarray([r["control"]["cross_track_m"] for r in unique])
        odom_age = np.asarray([r["odom_stamp_age_ms"] for r in unique], dtype=float)
        receive_age = np.asarray([r["odom_to_control_ms"] for r in unique], dtype=float)
        command_vx = np.asarray([r["control"]["cmd_vx"] for r in unique], dtype=float)
        command_yaw = np.asarray([r["control"]["cmd_yaw_rate"] for r in unique], dtype=float)
        target_distance = np.asarray(
            [r["control"]["target_distance_m"] for r in unique], dtype=float
        )
        selected_alpha = np.asarray(
            [r["control"]["selected_alpha"] for r in unique], dtype=float
        )

        plane_points = np.c_[(xyz - origin) @ e1, (xyz - origin) @ e2]
        plane_projection = project_to_polyline(plane_points, route_plane_xy)
        plane_fit = fit_plane(xyz)

        route_sections = section_metrics(progress, signed, distance)
        for section in route_sections:
            keep = (progress >= section["progress_start_m"]) & (
                progress < section["progress_end_m"]
            )
            geometric_angle = np.degrees(
                np.arcsin(np.clip(distance[keep] / target_distance[keep], -1.0, 1.0))
            )
            section["target_distance_m"] = pct(target_distance[keep])
            section["selected_alpha_rad"] = pct(selected_alpha[keep], (5, 50, 95))
            section["cross_track_over_target_as_angle_deg"] = pct(geometric_angle)

        phase_metrics.append(
            {
                "phase": phase_id,
                "direction": int(rows[0]["control"]["direction"]),
                "control_cycle_count": len(rows),
                "unique_odom_count": len(unique),
                "start_epoch": rows[0]["wall_time"],
                "end_epoch": rows[-1]["wall_time"],
                "duration_s": rows[-1]["wall_time"] - rows[0]["wall_time"],
                "route_progress_m": {
                    "start": float(progress[0]),
                    "end": float(progress[-1]),
                    "min": float(np.min(progress)),
                    "max": float(np.max(progress)),
                },
                "lio_xy_cross_track_m": {
                    "absolute": pct(distance),
                    "signed": pct(signed, (5, 50, 95)),
                },
                "floor_plane_cross_track_m": {
                    "absolute": pct(plane_projection["distance"]),
                    "signed": pct(plane_projection["signed"], (5, 50, 95)),
                    "interpretation": (
                        "Distance after rotating the fitted flat-route plane level; "
                        "still LIO-relative, not an external ground-truth measurement."
                    ),
                },
                "odom_source_stamp_age_ms": pct(odom_age),
                "odom_receive_to_control_ms": pct(receive_age),
                "command_vx_mps": pct(command_vx, (5, 50, 95, 100)),
                "command_abs_yaw_rate_rps": pct(np.abs(command_yaw), (50, 95, 100)),
                "plane_fit": {
                    "z_equals_a_x_plus_b_y_plus_c": plane_fit["coeff"],
                    "tilt_deg": plane_fit["tilt_deg"],
                    "rms_residual_m": plane_fit["rms_m"],
                    "abs_residual_m": plane_fit["abs_residual_m"],
                },
                "progress_bins_5m": binned_metrics(progress, signed, distance),
                "route_sections": route_sections,
            }
        )
        phase_plot.append(
            {
                "phase": phase_id,
                "xy": xy,
                "xyz": xyz,
                "progress": progress,
                "signed": signed,
                "distance": distance,
                "plane_xy": plane_points,
                "plane_projection": plane_projection,
                "odom_age": odom_age,
                "receive_age": receive_age,
            }
        )

    # All controller cycles are used for controller-reported aggregate error.
    controller_distance = np.asarray([r["control"]["cross_track_m"] for r in active])
    controller_signed = np.asarray([r["control"]["signed_cross_track_m"] for r in active])
    controller_odom_age = np.asarray([r["odom_stamp_age_ms"] for r in active], dtype=float)
    controller_receive_age = np.asarray([r["odom_to_control_ms"] for r in active], dtype=float)
    correlations = {
        "abs_cross_track_vs_source_stamp_age": float(
            np.corrcoef(controller_distance, controller_odom_age)[0, 1]
        ),
        "abs_cross_track_vs_receive_to_control": float(
            np.corrcoef(controller_distance, controller_receive_age)[0, 1]
        ),
    }

    # Initial alignment at the exact instant motion was released.
    first = active[0]
    initial_position_error = math.hypot(
        first["pose"]["x"] - runtime_route["x"][0],
        first["pose"]["y"] - runtime_route["y"][0],
    )
    initial_yaw_error = abs(
        float(wrap_angle(first["pose"]["yaw"] - runtime_route["yaw"][0]))
    )

    anchor = json.loads((PATROL / "manual_anchor.json").read_text(encoding="utf-8"))
    recording_audit = json.loads(
        (RECORDING / "route_recording_audit.json").read_text(encoding="utf-8")
    )
    manifest = (PATROL / "manifest.txt").read_text(encoding="utf-8")
    speed_match = re.search(r"^speed=(\S+)$", manifest, re.MULTILINE)
    loop_match = re.search(r"^loop=(\S+)$", manifest, re.MULTILINE)

    post_corner_yaw_section = yaw_track_sections[1]
    forward_post_corner_section = phase_metrics[0]["route_sections"][1]
    coordinate_heading_bias_deg = abs(
        float(post_corner_yaw_section["signed_error_deg"]["p50"])
    )
    median_target_distance = float(forward_post_corner_section["target_distance_m"]["p50"])
    predicted_cross_track = median_target_distance * math.sin(
        math.radians(coordinate_heading_bias_deg)
    )
    observed_cross_track = float(forward_post_corner_section["absolute_m"]["p50"])

    # Route turn positions.
    segment_heading = np.arctan2(np.diff(runtime_xy[:, 1]), np.diff(runtime_xy[:, 0]))
    turn = np.asarray(wrap_angle(np.diff(segment_heading)))
    strongest = np.argsort(np.abs(turn))[::-1][:10] + 1
    turn_rows = [
        {
            "waypoint_index": int(index),
            "route_progress_m": float(runtime_cumulative[index]),
            "turn_deg": float(np.degrees(turn[index - 1])),
        }
        for index in strongest
    ]

    # Top controller-error events, kept distinct in time.
    order = np.argsort(controller_distance)[::-1]
    top_events: list[dict[str, Any]] = []
    for index in order:
        row = active[int(index)]
        if any(abs(row["wall_time"] - event["epoch"]) < 1.0 for event in top_events):
            continue
        segment = int(row["projection"]["segment"])
        progress = runtime_cumulative[segment] + row["projection"]["fraction"] * runtime_seg_len[segment]
        top_events.append(
            {
                "epoch": row["wall_time"],
                "phase_direction": int(row["control"]["direction"]),
                "route_progress_m": float(progress),
                "cross_track_m": float(row["control"]["cross_track_m"]),
                "signed_cross_track_m": float(row["control"]["signed_cross_track_m"]),
                "cmd_vx_mps": float(row["control"]["cmd_vx"]),
                "cmd_yaw_rate_rps": float(row["control"]["cmd_yaw_rate"]),
                "selected_alpha_rad": float(row["control"]["selected_alpha"]),
                "odom_source_stamp_age_ms": float(row["odom_stamp_age_ms"]),
                "odom_receive_to_control_ms": float(row["odom_to_control_ms"]),
            }
        )
        if len(top_events) == 10:
            break

    attitude = attitude_invariant(patrol_odom, patrol_sport, active_start, active_end)

    # Battery and motor temperature are useful health context, not localization evidence.
    patrol_low = [
        row
        for row in patrol_rows
        if row["kind"] == "low" and active_start <= row["wall_time"] <= active_end
    ]
    battery_soc = [row["data"]["bms"]["soc"] for row in patrol_low]
    motor_temp = [
        motor["temperature_c"] for row in patrol_low for motor in row["data"]["motors"]
    ]

    hashes = {
        str(path.relative_to(ROOT)): sha256(path)
        for path in [
            ROUTES / "xbf8.csv",
            ROUTES / "xbf2.csv",
            PATROL / "route_original.csv",
            RECORDING / "route_recorded.csv",
        ]
    }

    metrics: dict[str, Any] = {
        "analysis_version": 1,
        "evidence_scope": {
            "recording_motion_window_epoch": [record_start, record_end],
            "patrol_motion_window_epoch": [active_start, active_end],
            "patrol_phase_count": len(phases),
        },
        "route_identity": {
            "sha256": hashes,
            "all_content_identical": len(set(hashes.values())) == 1,
        },
        "recording_integrity": {
            "waypoint_count": int(len(source_xy)),
            "path_length_lio_xy_m": float(np.sum(np.linalg.norm(np.diff(source_xy, axis=0), axis=1))),
            "csv_velocity_values_mps": sorted(set(float(v) for v in source_route["v"])),
            "recorder_reproduction": recording_audit["recorder_reproduction"],
            "spacing_m": recording_audit["route_geometry"]["spacing_m"],
            "manual_move_requests": {
                "api_id": 1008,
                "count": int(len(recording_move_requests)),
                "forward_x": pct(recording_requested_x, (5, 50, 95, 100)),
                "lateral_y": pct(recording_requested_y, (5, 50, 95, 100)),
                "yaw_z": pct(recording_requested_yaw, (5, 50, 95, 100)),
                "lateral_nonzero_over_1e_4_count": int(
                    np.sum(np.abs(recording_requested_y) > 1e-4)
                ),
            },
            "recorded_lio_yaw_vs_xy_track_heading": {
                "definition": "wrapped(track heading - recorded LIO Euler yaw)",
                "overall_signed_deg": pct(np.degrees(source_yaw_track_error), (5, 50, 95)),
                "overall_absolute_deg": pct(np.abs(np.degrees(source_yaw_track_error))),
                "route_sections": yaw_track_sections,
            },
            "body_imu_vs_lio_yaw_direction_test": {
                "method": (
                    "Match each recorded waypoint to body IMU yaw by local receive time, "
                    "then remove one constant yaw-origin offset using the 0-55 m straight."
                ),
                "waypoint_count": int(len(matched_body_yaw_a)),
                "odom_to_body_receive_pair_gap_s": pct(matched_receive_gap),
                "body_yaw_origin_alignment_deg": math.degrees(body_constant_offset),
                "lio_yaw_origin_alignment_deg": math.degrees(lio_constant_offset),
                "route_sections": body_lio_sections,
            },
        },
        "manual_anchor_and_initial_alignment": {
            "manual_anchor_delta_yaw_deg": math.degrees(anchor["transform"]["delta_yaw"]),
            "manual_anchor_translation_m": math.hypot(
                anchor["transform"]["current_start"]["x"] - anchor["transform"]["source_start"]["x"],
                anchor["transform"]["current_start"]["y"] - anchor["transform"]["source_start"]["y"],
            ),
            "anchor_stability": anchor["stability"],
            "motion_release_position_error_to_runtime_start_m": initial_position_error,
            "motion_release_yaw_error_to_runtime_start_deg": math.degrees(initial_yaw_error),
        },
        "execution": {
            "requested_speed_mps": float(speed_match.group(1)) if speed_match else None,
            "loop_mode": loop_match.group(1) if loop_match else None,
            "runtime_route_length_lio_xy_m": float(runtime_cumulative[-1]),
            "runtime_route_length_floor_plane_m": float(route_plane_cumulative[-1]),
            "phase_metrics": phase_metrics,
            "controller_reported_all_cycles": {
                "cycle_count": int(len(active)),
                "absolute_cross_track_m": pct(controller_distance),
                "signed_cross_track_m": pct(controller_signed, (5, 50, 95)),
                "odom_source_stamp_age_ms": pct(controller_odom_age),
                "odom_receive_to_control_ms": pct(controller_receive_age),
            },
            "latency_error_correlations": correlations,
            "top_cross_track_events": top_events,
            "strongest_route_turns": turn_rows,
            "post_corner_geometric_consistency_check": {
                "section_progress_m": [60.0, 140.0],
                "recorded_track_heading_minus_lio_yaw_median_abs_deg": coordinate_heading_bias_deg,
                "controller_target_distance_median_m": median_target_distance,
                "cross_track_predicted_by_target_distance_times_sin_bias_m": predicted_cross_track,
                "cross_track_observed_forward_median_m": observed_cross_track,
                "absolute_prediction_difference_m": abs(predicted_cross_track - observed_cross_track),
                "interpretation": (
                    "The simple lookahead controller settles where its target bearing agrees "
                    "with the biased LIO Euler yaw.  Its measured lateral offset is therefore "
                    "expected to be approximately target_distance*sin(heading_bias)."
                ),
            },
        },
        "tilted_lio_plane": {
            "recording_fit": {
                "point_count": int(len(record_xyz)),
                "z_equals_a_x_plus_b_y_plus_c": recording_plane["coeff"],
                "tilt_deg": recording_plane["tilt_deg"],
                "rms_residual_m": recording_plane["rms_m"],
                "abs_residual_m": recording_plane["abs_residual_m"],
            },
            "plane_normal_lio_coordinates": normal,
            "interpretation": (
                "On the user's physically flat route, recording and every patrol pass lie on "
                "the same ~23-degree plane in the LIO world with centimetre-scale residuals. "
                "This proves a fixed tilted LIO/world plane; it does not by itself prove drift "
                "or a loose sensor."
            ),
        },
        "rigid_attitude_consistency": attitude,
        "health_context": {
            "battery_soc_percent": {
                "start": battery_soc[0] if battery_soc else None,
                "end": battery_soc[-1] if battery_soc else None,
                "min": min(battery_soc) if battery_soc else None,
            },
            "motor_temperature_c": pct(motor_temp),
            "fastlio_recording_health_events": recording_audit["fastlio_health"],
        },
    }
    (ROOT / "derived_metrics.json").write_text(
        json.dumps(json_safe(metrics), ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )

    # Compact per-phase 5 m table for independent inspection.
    with (ROOT / "cross_track_by_progress.csv").open("w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(
            fh,
            fieldnames=[
                "phase",
                "direction",
                "progress_start_m",
                "progress_end_m",
                "count",
                "signed_median_m",
                "abs_median_m",
                "abs_p95_m",
                "abs_max_m",
            ],
        )
        writer.writeheader()
        for phase in phase_metrics:
            for row in phase["progress_bins_5m"]:
                writer.writerow(
                    {
                        "phase": phase["phase"],
                        "direction": phase["direction"],
                        **row,
                    }
                )

    # Figure 1: route and each pass in raw LIO x-y.
    colors = ["#1877d2", "#e15759", "#59a14f"]
    labels = ["forward pass 1", "reverse pass", "forward pass 2 (stopped)"]
    fig, ax = plt.subplots(figsize=(9, 8))
    ax.plot(runtime_xy[:, 0], runtime_xy[:, 1], color="black", lw=2.2, label="runtime CSV")
    for info, color, label in zip(phase_plot, colors, labels):
        ax.plot(info["xy"][:, 0], info["xy"][:, 1], color=color, lw=1.1, alpha=0.9, label=label)
    ax.scatter(runtime_xy[0, 0], runtime_xy[0, 1], marker="o", s=55, color="#00a878", label="start")
    ax.scatter(runtime_xy[-1, 0], runtime_xy[-1, 1], marker="X", s=70, color="#7b2cbf", label="end")
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("LIO x (m)")
    ax.set_ylabel("LIO y (m)")
    ax.set_title("Recorded route vs patrol trajectory (raw LIO x-y)")
    ax.grid(alpha=0.25)
    ax.legend(fontsize=8, loc="best")
    fig.tight_layout()
    fig.savefig(ROOT / "trajectory_xy.png", dpi=180)
    plt.close(fig)

    # Figure 2: signed cross-track by route progress.  Downsample for readability.
    fig, ax = plt.subplots(figsize=(12, 5.5))
    for info, color, label in zip(phase_plot, colors, labels):
        step = max(1, len(info["progress"]) // 2500)
        ax.scatter(
            info["progress"][::step],
            info["signed"][::step],
            s=3,
            alpha=0.22,
            color=color,
        )
        bins = binned_metrics(info["progress"], info["signed"], info["distance"])
        centers = [(row["progress_start_m"] + row["progress_end_m"]) / 2 for row in bins]
        medians = [row["signed_median_m"] for row in bins]
        ax.plot(centers, medians, color=color, lw=2.2, label=f"{label}: 5 m median")
    for turn_row in turn_rows[:3]:
        ax.axvline(turn_row["route_progress_m"], color="#8c8c8c", ls="--", lw=1)
        ax.text(
            turn_row["route_progress_m"],
            0.36,
            f"turn {turn_row['turn_deg']:.0f}°",
            rotation=90,
            va="top",
            ha="right",
            fontsize=7,
            color="#555555",
        )
    ax.axhline(0, color="black", lw=0.8)
    ax.set_ylim(-0.38, 0.38)
    ax.set_xlabel("Route progress (m)")
    ax.set_ylabel("Signed cross-track in LIO x-y (m)")
    ax.set_title("Tracking error is location/direction dependent, not cumulative frame backlog")
    ax.grid(alpha=0.25)
    ax.legend(fontsize=8, loc="lower left")
    fig.tight_layout()
    fig.savefig(ROOT / "cross_track_progress.png", dpi=180)
    plt.close(fig)

    # Figure 3: tilted plane and the effect of leveling it.
    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    record_projection = project_to_polyline(record_xyz[:, :2], source_xy)
    axes[0].scatter(
        record_projection["progress"],
        record_xyz[:, 2],
        s=3,
        alpha=0.25,
        color="#4e79a7",
        label="recording odometry",
    )
    for info, color, label in zip(phase_plot[:2], colors[:2], labels[:2]):
        axes[0].scatter(
            info["progress"],
            info["xyz"][:, 2],
            s=2,
            alpha=0.18,
            color=color,
            label=label,
        )
    axes[0].set_ylabel("LIO z (m)")
    axes[0].set_title(
        f"Physically flat route appears on a {recording_plane['tilt_deg']:.2f}° plane in LIO coordinates"
    )
    axes[0].grid(alpha=0.25)
    axes[0].legend(fontsize=8)
    for info, color, label in zip(phase_plot, colors, labels):
        axes[1].scatter(
            info["progress"],
            info["distance"],
            s=3,
            alpha=0.18,
            color=color,
        )
        bins_raw = binned_metrics(info["progress"], info["signed"], info["distance"])
        centers = [(r["progress_start_m"] + r["progress_end_m"]) / 2 for r in bins_raw]
        raw = [r["abs_median_m"] for r in bins_raw]
        axes[1].plot(centers, raw, color=color, lw=2, label=f"{label}: raw x-y")
        plane = info["plane_projection"]
        comparable_plane_progress = (
            plane["progress"] * runtime_cumulative[-1] / route_plane_cumulative[-1]
        )
        bins_plane = binned_metrics(
            comparable_plane_progress, plane["signed"], plane["distance"]
        )
        centers_plane = [(r["progress_start_m"] + r["progress_end_m"]) / 2 for r in bins_plane]
        leveled = [r["abs_median_m"] for r in bins_plane]
        axes[1].plot(
            centers_plane,
            leveled,
            color=color,
            lw=1.4,
            ls="--",
            label=f"{label}: leveled plane",
        )
    axes[1].set_xlabel("Route progress (m)")
    axes[1].set_ylabel("Absolute cross-track (m)")
    axes[1].set_title("Raw LIO x-y error versus fitted-floor-plane distance")
    axes[1].grid(alpha=0.25)
    axes[1].legend(fontsize=7, ncol=2)
    fig.tight_layout()
    fig.savefig(ROOT / "orientation_z_progress.png", dpi=180)
    plt.close(fig)

    # Figure 4: stale source stamps are real, but do not correlate with error location.
    all_progress = np.concatenate([info["progress"] for info in phase_plot])
    all_distance = np.concatenate([info["distance"] for info in phase_plot])
    all_odom_age = np.concatenate([info["odom_age"] for info in phase_plot])
    all_receive_age = np.concatenate([info["receive_age"] for info in phase_plot])
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.8))
    sample = np.arange(0, len(all_distance), max(1, len(all_distance) // 5000))
    sc = axes[0].scatter(
        all_odom_age[sample],
        all_distance[sample],
        c=all_progress[sample],
        s=5,
        alpha=0.25,
        cmap="viridis",
    )
    axes[0].set_xlabel("Odom source-stamp age (ms)")
    axes[0].set_ylabel("Absolute cross-track (m)")
    axes[0].set_title(f"r = {correlations['abs_cross_track_vs_source_stamp_age']:.3f}")
    fig.colorbar(sc, ax=axes[0], label="route progress (m)")
    axes[1].scatter(
        all_receive_age[sample],
        all_distance[sample],
        c=all_progress[sample],
        s=5,
        alpha=0.25,
        cmap="viridis",
    )
    axes[1].set_xlabel("Local odom receive-to-control (ms)")
    axes[1].set_ylabel("Absolute cross-track (m)")
    axes[1].set_title(f"r = {correlations['abs_cross_track_vs_receive_to_control']:.3f}")
    for ax in axes:
        ax.grid(alpha=0.25)
    fig.suptitle("Frame age is measurable, but it does not explain where tracking error occurs")
    fig.tight_layout()
    fig.savefig(ROOT / "latency_and_error.png", dpi=180)
    plt.close(fig)

    print(json.dumps(json_safe(metrics), ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()

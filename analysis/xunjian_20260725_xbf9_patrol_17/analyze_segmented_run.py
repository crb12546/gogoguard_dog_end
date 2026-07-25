#!/usr/bin/env python3
"""Reproducible segmented analysis for patrol xunjian-20260725-17.

The source evidence is kept read-only.  This script derives:

* an independent projection of every consumed odometry update onto the exact
  horizontal route used by the follower;
* safety-override and physical-remote timelines;
* tracking metrics for uncontaminated automatic and intervention phases;
* performance/latency correlation checks; and
* compact JSON/CSV/PNG artifacts for manual review.
"""

from __future__ import annotations

import csv
import json
import math
from datetime import datetime
from pathlib import Path
from typing import Any, Iterable

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


ROOT = Path(__file__).resolve().parent
TRACE = ROOT / "follower_control_trace.jsonl"
TELEMETRY = ROOT / "experiment_telemetry.jsonl"
ROUTE = ROOT / "route_horizontal.csv"
RUNTIME_ROUTE = ROOT / "route_runtime.csv"
PERFORMANCE = ROOT / "performance_monitor.log"


def read_jsonl(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    with path.open(encoding="utf-8") as handle:
        for line in handle:
            if line.strip():
                rows.append(json.loads(line))
    return rows


def read_route(path: Path) -> dict[str, np.ndarray]:
    columns: dict[str, list[float]] = {"id": [], "x": [], "y": [], "yaw": [], "v": []}
    with path.open(newline="", encoding="utf-8-sig") as handle:
        for row in csv.DictReader(handle):
            for key in columns:
                columns[key].append(float(row[key]))
    return {key: np.asarray(values, dtype=float) for key, values in columns.items()}


def percentile(values: Iterable[float], ps: tuple[int, ...] = (50, 95, 99, 100)) -> dict[str, Any]:
    array = np.asarray(list(values), dtype=float)
    array = array[np.isfinite(array)]
    result: dict[str, Any] = {"count": int(len(array))}
    for p in ps:
        result[f"p{p}"] = float(np.percentile(array, p)) if len(array) else None
    return result


def wrap_angle(value: np.ndarray | float) -> np.ndarray | float:
    return (value + np.pi) % (2.0 * np.pi) - np.pi


def project_to_polyline(points: np.ndarray, route: np.ndarray, chunk_size: int = 800) -> dict[str, np.ndarray]:
    vectors = np.diff(route, axis=0)
    lengths = np.linalg.norm(vectors, axis=1)
    cumulative = np.r_[0.0, np.cumsum(lengths)]
    denom = np.sum(vectors * vectors, axis=1)
    output: dict[str, list[np.ndarray]] = {
        "distance": [],
        "signed": [],
        "progress": [],
        "segment": [],
        "fraction": [],
        "projected": [],
    }
    for start in range(0, len(points), chunk_size):
        point_chunk = points[start : start + chunk_size]
        relative = point_chunk[:, None, :] - route[:-1][None, :, :]
        fraction = np.clip(
            np.einsum("nsi,si->ns", relative, vectors) / denom[None, :],
            0.0,
            1.0,
        )
        projected = route[:-1][None, :, :] + fraction[:, :, None] * vectors[None, :, :]
        delta = point_chunk[:, None, :] - projected
        distance_squared = np.einsum("nsi,nsi->ns", delta, delta)
        segment = np.argmin(distance_squared, axis=1)
        row_index = np.arange(len(point_chunk))
        chosen_fraction = fraction[row_index, segment]
        chosen_delta = delta[row_index, segment]
        chosen_vector = vectors[segment]
        signed = (
            chosen_vector[:, 0] * chosen_delta[:, 1]
            - chosen_vector[:, 1] * chosen_delta[:, 0]
        ) / lengths[segment]
        output["distance"].append(np.sqrt(distance_squared[row_index, segment]))
        output["signed"].append(signed)
        output["progress"].append(cumulative[segment] + chosen_fraction * lengths[segment])
        output["segment"].append(segment)
        output["fraction"].append(chosen_fraction)
        output["projected"].append(projected[row_index, segment])
    return {key: np.concatenate(values) for key, values in output.items()}


def nearest_waypoint_distance(points: np.ndarray, route: np.ndarray, chunk_size: int = 1000) -> np.ndarray:
    result: list[np.ndarray] = []
    for start in range(0, len(points), chunk_size):
        chunk = points[start : start + chunk_size]
        distance_squared = np.sum((chunk[:, None, :] - route[None, :, :]) ** 2, axis=2)
        result.append(np.sqrt(np.min(distance_squared, axis=1)))
    return np.concatenate(result)


def unique_odom(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    seen: set[int] = set()
    result: list[dict[str, Any]] = []
    for row in rows:
        sequence = int(row["odom_callback_sequence"])
        if sequence <= 0 or sequence in seen:
            continue
        seen.add(sequence)
        result.append(row)
    return result


def vector_from_twist(row: dict[str, Any]) -> np.ndarray:
    data = row["data"]
    return np.asarray(
        [data["linear"]["x"], data["linear"]["y"], data["angular"]["z"]],
        dtype=float,
    )


def nearest_pair_flags(
    raw_rows: list[dict[str, Any]],
    safe_rows: list[dict[str, Any]],
    start: float,
    end: float,
) -> tuple[np.ndarray, np.ndarray]:
    raw_rows = [row for row in raw_rows if start <= row["wall_time"] <= end]
    safe_rows = [row for row in safe_rows if start <= row["wall_time"] <= end]
    safe_times = np.asarray([row["wall_time"] for row in safe_rows], dtype=float)
    safe_vectors = np.asarray([vector_from_twist(row) for row in safe_rows], dtype=float)
    paired_times: list[float] = []
    flags: list[bool] = []
    for row in raw_rows:
        timestamp = float(row["wall_time"])
        index = int(np.searchsorted(safe_times, timestamp))
        candidates = [candidate for candidate in (index - 1, index) if 0 <= candidate < len(safe_times)]
        if not candidates:
            continue
        chosen = min(candidates, key=lambda candidate: abs(safe_times[candidate] - timestamp))
        if abs(safe_times[chosen] - timestamp) > 0.08:
            continue
        raw_vector = vector_from_twist(row)
        raw_nonzero = bool(np.max(np.abs(raw_vector)) > 0.01)
        safe_zero = bool(np.max(np.abs(safe_vectors[chosen])) <= 0.01)
        paired_times.append(timestamp)
        flags.append(raw_nonzero and safe_zero)
    return np.asarray(paired_times), np.asarray(flags, dtype=bool)


def true_intervals(times: np.ndarray, flags: np.ndarray, max_gap: float = 0.25) -> list[tuple[float, float]]:
    active = times[flags]
    if not len(active):
        return []
    intervals: list[tuple[float, float]] = []
    begin = float(active[0])
    previous = float(active[0])
    for value in active[1:]:
        timestamp = float(value)
        if timestamp - previous > max_gap:
            intervals.append((begin, previous + 0.05))
            begin = timestamp
        previous = timestamp
    intervals.append((begin, previous + 0.05))
    return intervals


def remote_is_active(row: dict[str, Any]) -> bool:
    data = row["data"]
    axes = [abs(float(data.get(key, 0.0))) for key in ("lx", "ly", "rx", "ry")]
    return int(data.get("keys", 0)) != 0 or max(axes, default=0.0) > 0.05


def group_active_remote(rows: list[dict[str, Any]], max_gap: float = 5.0) -> list[tuple[float, float]]:
    active = [float(row["wall_time"]) for row in rows if remote_is_active(row)]
    if not active:
        return []
    groups: list[tuple[float, float]] = []
    start = active[0]
    previous = active[0]
    for timestamp in active[1:]:
        if timestamp - previous > max_gap:
            groups.append((start, previous))
            start = timestamp
        previous = timestamp
    groups.append((start, previous))
    return groups


def find_stable_start(
    times: np.ndarray,
    errors: np.ndarray,
    after: float,
    threshold_m: float = 0.10,
    hold_s: float = 5.0,
) -> float:
    candidates = np.where(times >= after)[0]
    for index in candidates:
        end = int(np.searchsorted(times, times[index] + hold_s, side="right"))
        if end <= index or times[end - 1] - times[index] < hold_s - 0.25:
            continue
        if float(np.max(errors[index:end])) < threshold_m:
            return float(times[index])
    return float(times[candidates[-1]]) if len(candidates) else after


def phase_metrics(
    name: str,
    mask: np.ndarray,
    times: np.ndarray,
    progress: np.ndarray,
    distance: np.ndarray,
    signed: np.ndarray,
    waypoint_distance: np.ndarray,
    csv_heading_error_deg: np.ndarray,
    tangent_heading_error_deg: np.ndarray,
    odom_age_ms: np.ndarray,
    source_stamps: np.ndarray,
) -> dict[str, Any]:
    indices = np.where(mask)[0]
    if not len(indices):
        return {"name": name, "count": 0}
    selected_times = times[indices]
    selected_progress = progress[indices]
    adjacent = np.diff(indices) == 1
    gaps = np.diff(source_stamps[indices])[adjacent]
    regressions = np.diff(selected_progress)[adjacent]
    worst_local = int(np.argmax(distance[indices]))
    worst_index = int(indices[worst_local])
    return {
        "name": name,
        "count": int(len(indices)),
        "start_epoch": float(selected_times[0]),
        "end_epoch": float(selected_times[-1]),
        "duration_s": float(selected_times[-1] - selected_times[0]),
        "route_progress_start_m": float(selected_progress[0]),
        "route_progress_end_m": float(selected_progress[-1]),
        "route_progress_delta_m": float(selected_progress[-1] - selected_progress[0]),
        "cross_track_abs_m": percentile(distance[indices]),
        "cross_track_signed_mean_m": float(np.mean(signed[indices])),
        "cross_track_signed_median_m": float(np.median(signed[indices])),
        "nearest_waypoint_m": percentile(waypoint_distance[indices]),
        "heading_error_vs_csv_yaw_deg": percentile(csv_heading_error_deg[indices]),
        "heading_error_vs_geometric_tangent_deg": percentile(tangent_heading_error_deg[indices]),
        "odom_source_stamp_age_ms": percentile(odom_age_ms[indices]),
        "odom_source_gap_s": percentile(gaps, (50, 95, 100)),
        "progress_regression_over_5cm_count": int(np.sum(regressions < -0.05)),
        "minimum_progress_step_m": float(np.min(regressions)) if len(regressions) else None,
        "worst_cross_track": {
            "epoch": float(times[worst_index]),
            "error_m": float(distance[worst_index]),
            "progress_m": float(progress[worst_index]),
        },
    }


def parse_performance() -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    with PERFORMANCE.open(encoding="utf-8") as handle:
        for line in handle:
            if not line.startswith("PERF_SAMPLE "):
                continue
            row = json.loads(line[len("PERF_SAMPLE ") :])
            row["epoch"] = datetime.fromisoformat(row["wall_time"]).timestamp()
            rows.append(row)
    return rows


def performance_metrics(rows: list[dict[str, Any]], start: float, end: float) -> dict[str, Any]:
    selected = [row for row in rows if start <= row["epoch"] <= end]
    if not selected:
        return {"count": 0}

    def values(path: tuple[str, ...]) -> list[float]:
        output: list[float] = []
        for row in selected:
            value: Any = row
            for key in path:
                if not isinstance(value, dict) or key not in value:
                    value = None
                    break
                value = value[key]
            if isinstance(value, (int, float)):
                output.append(float(value))
        return output

    network_errors: dict[str, int] = {}
    for row in selected:
        for interface, counters in row.get("network", {}).items():
            total = sum(
                int(counters.get(key, 0))
                for key in ("rx_dropped_delta", "rx_errors_delta", "tx_dropped_delta", "tx_errors_delta")
            )
            network_errors[interface] = network_errors.get(interface, 0) + total
    udp_totals: dict[str, int] = {}
    for row in selected:
        for key, value in row.get("udp_deltas", {}).items():
            udp_totals[key] = udp_totals.get(key, 0) + int(value)
    return {
        "count": len(selected),
        "system_cpu_pct": percentile(values(("system_cpu_pct",))),
        "temperature_c": percentile(values(("max_temperature_c",))),
        "monitor_wake_late_ms": percentile(values(("monitor_wake_late_ms",))),
        "process_cpu_pct": {
            name: percentile(
                [
                    float(row["process_cpu_pct"][name])
                    for row in selected
                    if name in row.get("process_cpu_pct", {})
                ]
            )
            for name in ("fastlio", "livox", "follower", "safe", "experiment_telemetry")
        },
        "online_cpu_sets": sorted({tuple(row.get("online_cpus", [])) for row in selected}),
        "network_interface_error_drop_totals": network_errors,
        "udp_kernel_counter_totals": udp_totals,
    }


def performance_tracking_correlation(
    performance_rows: list[dict[str, Any]],
    times: np.ndarray,
    errors: np.ndarray,
    ages_ms: np.ndarray,
    start: float,
    end: float,
) -> dict[str, Any]:
    selected = [row for row in performance_rows if start <= row["epoch"] <= end]
    cpu: list[float] = []
    matched_error: list[float] = []
    matched_age: list[float] = []
    matched_time: list[float] = []
    for row in selected:
        index = int(np.argmin(np.abs(times - row["epoch"])))
        if abs(times[index] - row["epoch"]) > 0.8:
            continue
        cpu.append(float(row["system_cpu_pct"]))
        matched_error.append(float(errors[index]))
        matched_age.append(float(ages_ms[index]))
        matched_time.append(float(row["epoch"]))
    cpu_array = np.asarray(cpu)
    error_array = np.asarray(matched_error)
    age_array = np.asarray(matched_age)
    time_array = np.asarray(matched_time)
    high = cpu_array >= 90.0
    normal = ~high
    high_intervals = true_intervals(time_array, high, max_gap=2.0)
    return {
        "matched_samples": int(len(cpu_array)),
        "system_cpu_at_or_above_90pct_fraction": float(np.mean(high)),
        "pearson_system_cpu_vs_cross_track": float(np.corrcoef(cpu_array, error_array)[0, 1]),
        "cross_track_during_cpu_below_90pct_m": percentile(error_array[normal]),
        "cross_track_during_cpu_at_or_above_90pct_m": percentile(error_array[high]),
        "odom_age_during_cpu_below_90pct_ms": percentile(age_array[normal]),
        "odom_age_during_cpu_at_or_above_90pct_ms": percentile(age_array[high]),
        "high_cpu_burst_count": len(high_intervals),
        "high_cpu_burst_duration_s": percentile(
            [end_value - begin for begin, end_value in high_intervals]
        ),
    }


def local_time(epoch: float) -> str:
    return datetime.fromtimestamp(epoch).astimezone().isoformat(timespec="milliseconds")


def json_safe(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(key): json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    if isinstance(value, np.ndarray):
        return [json_safe(item) for item in value.tolist()]
    if isinstance(value, (np.integer, np.floating)):
        return value.item()
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def main() -> None:
    trace_rows = read_jsonl(TRACE)
    controls_all = [row for row in trace_rows if row.get("kind") == "control"]
    active_controls = [
        row
        for row in controls_all
        if row["control"].get("source") == "deployed_go2_2_unified_horizontal_frame"
        and bool(row["control"].get("motion_enabled"))
    ]
    consumed = unique_odom(active_controls)
    anchor = next(row for row in trace_rows if row.get("kind") == "horizontal_route_anchored")
    stop = next(row for row in reversed(trace_rows) if row.get("kind") == "trace_stop")

    route = read_route(ROUTE)
    route_xy = np.c_[route["x"], route["y"]]
    theta = math.radians(float(anchor["route_rotation_deg"]))
    rotation = np.asarray(
        [[math.cos(theta), -math.sin(theta)], [math.sin(theta), math.cos(theta)]],
        dtype=float,
    )
    canonical_start = np.asarray(
        [anchor["canonical_start"]["x"], anchor["canonical_start"]["y"]],
        dtype=float,
    )
    anchor_xy = np.asarray([anchor["anchor_x"], anchor["anchor_y"]], dtype=float)
    aligned_route = (route_xy - canonical_start) @ rotation.T + anchor_xy
    aligned_route_yaw = np.asarray(wrap_angle(route["yaw"] + theta), dtype=float)

    times = np.asarray([row["wall_time"] for row in consumed], dtype=float)
    source_stamps = np.asarray([row["odom_source_stamp"] for row in consumed], dtype=float)
    odom_age_ms = np.asarray([row["odom_stamp_age_ms"] for row in consumed], dtype=float)
    poses = np.asarray([[row["pose"]["x"], row["pose"]["y"]] for row in consumed], dtype=float)
    pose_yaw = np.asarray([row["pose"]["yaw"] for row in consumed], dtype=float)
    raw_pose_xyz = np.asarray(
        [
            [row["pose"]["raw_lio_x"], row["pose"]["raw_lio_y"], row["pose"]["raw_lio_z"]]
            for row in consumed
        ],
        dtype=float,
    )
    horizontal_z = np.asarray([row["pose"]["z"] for row in consumed], dtype=float)

    projection = project_to_polyline(poses, aligned_route)
    waypoint_distance = nearest_waypoint_distance(poses, aligned_route)
    segment = projection["segment"].astype(int)
    fraction = projection["fraction"]
    yaw_delta = np.asarray(wrap_angle(aligned_route_yaw[segment + 1] - aligned_route_yaw[segment]))
    interpolated_csv_yaw = np.asarray(
        wrap_angle(aligned_route_yaw[segment] + fraction * yaw_delta),
        dtype=float,
    )
    csv_heading_error_deg = np.abs(np.degrees(wrap_angle(pose_yaw - interpolated_csv_yaw)))
    tangent = np.arctan2(
        aligned_route[segment + 1, 1] - aligned_route[segment, 1],
        aligned_route[segment + 1, 0] - aligned_route[segment, 0],
    )
    tangent_heading_error_deg = np.abs(np.degrees(wrap_angle(pose_yaw - tangent)))

    trace_distance = np.asarray([row["control"]["cross_track_m"] for row in consumed], dtype=float)
    trace_signed = np.asarray([row["control"]["signed_cross_track_m"] for row in consumed], dtype=float)
    projection_validation = {
        "independent_vs_trace_abs_cross_track_max_difference_m": float(
            np.max(np.abs(projection["distance"] - trace_distance))
        ),
        "independent_vs_trace_signed_cross_track_max_difference_m": float(
            np.max(np.abs(projection["signed"] - trace_signed))
        ),
    }

    telemetry = read_jsonl(TELEMETRY)
    raw_commands = [row for row in telemetry if row.get("kind") == "patrol_cmd"]
    safe_commands = [row for row in telemetry if row.get("kind") == "cmd_vel"]
    wireless = [row for row in telemetry if row.get("kind") == "wireless"]
    sport = [row for row in telemetry if row.get("kind") == "sport"]

    motion_start = float(active_controls[0]["wall_time"])
    motion_end = float(active_controls[-1]["wall_time"])
    paired_times, override_flags = nearest_pair_flags(raw_commands, safe_commands, motion_start, motion_end)
    override_intervals = true_intervals(paired_times, override_flags)
    first_override = override_intervals[0][0]
    remote_groups = group_active_remote(wireless)

    main_remote = remote_groups[0]
    second_remote = remote_groups[1]
    final_remote = remote_groups[-1]
    significant_axis_rows = [
        row
        for row in wireless
        if max(abs(float(row["data"].get(key, 0.0))) for key in ("lx", "ly", "rx", "ry"))
        > 0.05
    ]
    first_axis = next(row for row in significant_axis_rows if row["wall_time"] >= first_override)
    recovery_one = find_stable_start(
        times,
        projection["distance"],
        after=main_remote[1],
        threshold_m=0.10,
        hold_s=5.0,
    )
    recovery_two = find_stable_start(
        times,
        projection["distance"],
        after=second_remote[1],
        threshold_m=0.10,
        hold_s=5.0,
    )

    phase_bounds = [
        ("pure_auto_before_first_obstacle", motion_start, first_override),
        ("obstacle_manual_intervention_and_recovery", first_override, recovery_one),
        ("recovered_auto_before_brief_correction", recovery_one, second_remote[0]),
        ("brief_manual_correction_and_recovery", second_remote[0], recovery_two),
        ("recovered_auto_before_final_stop", recovery_two, final_remote[0]),
        ("final_remote_stop_to_sigterm", final_remote[0], float(stop["wall_time"])),
    ]
    phases: list[dict[str, Any]] = []
    for name, begin, end in phase_bounds:
        mask = (times >= begin) & (times < end)
        phases.append(
            phase_metrics(
                name,
                mask,
                times,
                projection["progress"],
                projection["distance"],
                projection["signed"],
                waypoint_distance,
                csv_heading_error_deg,
                tangent_heading_error_deg,
                odom_age_ms,
                source_stamps,
            )
        )

    clean_auto_mask = (
        ((times >= motion_start) & (times < first_override))
        | ((times >= recovery_one) & (times < second_remote[0]))
        | ((times >= recovery_two) & (times < final_remote[0]))
    )
    clean_auto = phase_metrics(
        "all_clean_automatic_segments",
        clean_auto_mask,
        times,
        projection["progress"],
        projection["distance"],
        projection["signed"],
        waypoint_distance,
        csv_heading_error_deg,
        tangent_heading_error_deg,
        odom_age_ms,
        source_stamps,
    )

    source_gaps = np.diff(source_stamps)
    wall_gaps = np.diff(times)
    max_gap_index = int(np.argmax(source_gaps)) + 1
    gap_epoch = float(times[max_gap_index])
    neighborhood = (times >= gap_epoch - 2.0) & (times <= gap_epoch + 2.0)
    gap_events: list[dict[str, Any]] = []
    for index in np.where(source_gaps > 0.30)[0]:
        following = index + 1
        local = (times >= times[following] - 1.0) & (times <= times[following] + 2.0)
        gap_events.append(
            {
                "epoch": float(times[following]),
                "source_gap_s": float(source_gaps[index]),
                "wall_gap_s": float(wall_gaps[index]),
                "cross_track_before_m": float(projection["distance"][index]),
                "cross_track_after_m": float(projection["distance"][following]),
                "max_cross_track_within_minus1_plus2s_m": float(
                    np.max(projection["distance"][local])
                ),
                "signed_cross_track_step_m": float(
                    projection["signed"][following] - projection["signed"][index]
                ),
            }
        )

    pure_age = odom_age_ms[clean_auto_mask]
    pure_error = projection["distance"][clean_auto_mask]
    age_cut = float(np.percentile(pure_age, 95))
    latency_correlation = {
        "clean_auto_pearson_age_vs_abs_cross_track": float(np.corrcoef(pure_age, pure_error)[0, 1]),
        "clean_auto_high_age_threshold_ms": age_cut,
        "clean_auto_cross_track_when_age_below_p95_m": percentile(pure_error[pure_age < age_cut]),
        "clean_auto_cross_track_when_age_at_or_above_p95_m": percentile(pure_error[pure_age >= age_cut]),
        "source_gap_over_300ms_events": gap_events,
        "maximum_source_gap_event": {
            "epoch": gap_epoch,
            "source_gap_s": float(source_gaps[max_gap_index - 1]),
            "wall_gap_s": float(wall_gaps[max_gap_index - 1]),
            "cross_track_before_m": float(projection["distance"][max_gap_index - 1]),
            "cross_track_after_m": float(projection["distance"][max_gap_index]),
            "max_cross_track_within_2s_m": float(np.max(projection["distance"][neighborhood])),
        },
    }

    all_control_times = np.asarray([row["wall_time"] for row in active_controls], dtype=float)
    pure_control_mask = all_control_times < first_override
    pure_yaw_rate = np.abs(
        np.asarray([row["control"]["cmd_yaw_rate"] for row in active_controls], dtype=float)[
            pure_control_mask
        ]
    )
    pure_vx = np.asarray(
        [row["control"]["cmd_vx"] for row in active_controls], dtype=float
    )[pure_control_mask]

    after_obstacle = np.where(times >= first_override)[0]
    first_over_10cm_index = next(
        int(index) for index in after_obstacle if projection["distance"][index] > 0.10
    )
    intervention_mask = (times >= first_override) & (times < recovery_one)
    intervention_indices = np.where(intervention_mask)[0]
    intervention_peak_index = int(
        intervention_indices[np.argmax(projection["distance"][intervention_indices])]
    )
    intervention_causality = {
        "first_safety_zero_epoch": first_override,
        "cross_track_at_first_safety_zero_m": float(
            projection["distance"][int(np.argmin(np.abs(times - first_override)))]
        ),
        "first_physical_remote_event_epoch": float(main_remote[0]),
        "cross_track_at_first_physical_remote_event_m": float(
            projection["distance"][int(np.argmin(np.abs(times - main_remote[0])))]
        ),
        "first_significant_remote_axis_epoch": float(first_axis["wall_time"]),
        "cross_track_at_first_significant_remote_axis_m": float(
            projection["distance"][
                int(np.argmin(np.abs(times - float(first_axis["wall_time"]))))
            ]
        ),
        "first_cross_track_over_10cm_epoch": float(times[first_over_10cm_index]),
        "first_cross_track_over_10cm_m": float(projection["distance"][first_over_10cm_index]),
        "intervention_peak_epoch": float(times[intervention_peak_index]),
        "intervention_peak_cross_track_m": float(
            projection["distance"][intervention_peak_index]
        ),
        "main_remote_end_epoch": float(main_remote[1]),
        "stable_below_10cm_for_5s_epoch": recovery_one,
        "route_segment_at_first_safety_zero": int(
            projection["segment"][int(np.argmin(np.abs(times - first_override)))]
        ),
        "route_segment_at_recovery": int(
            projection["segment"][int(np.argmin(np.abs(times - recovery_one)))]
        ),
    }

    route_vectors = np.diff(aligned_route, axis=0)
    route_length = float(np.sum(np.linalg.norm(route_vectors, axis=1)))
    final_progress = float(projection["progress"][-1])

    def nearest_progress(epoch: float) -> dict[str, float]:
        index = int(np.argmin(np.abs(times - epoch)))
        return {
            "epoch": float(times[index]),
            "progress_m": float(projection["progress"][index]),
            "cross_track_m": float(projection["distance"][index]),
        }

    final_override = override_intervals[-1]
    final_stop_timeline = {
        "final_remote_group": [float(final_remote[0]), float(final_remote[1])],
        "at_final_remote_start": nearest_progress(final_remote[0]),
        "final_safety_override_interval": [float(final_override[0]), float(final_override[1])],
        "at_final_safety_override_start": nearest_progress(final_override[0]),
        "trace_stop_epoch": float(stop["wall_time"]),
        "trace_stop_reason": stop["reason"],
        "at_trace_stop": nearest_progress(float(stop["wall_time"])),
        "route_finished_flag_seen": any(bool(row["control"].get("finished")) for row in active_controls),
    }

    sport_times = np.asarray([row["wall_time"] for row in sport], dtype=float)
    sport_speed = np.asarray(
        [
            math.hypot(
                float(row["data"]["velocity"][0]),
                float(row["data"]["velocity"][1]),
            )
            for row in sport
        ],
        dtype=float,
    )
    final_window = (sport_times >= final_remote[0] - 2.0) & (
        sport_times <= float(stop["wall_time"])
    )
    final_stop_timeline["sport_horizontal_speed_mps_final_window"] = percentile(
        sport_speed[final_window]
    )
    after_last_key_before_safety = (sport_times >= final_remote[1]) & (
        sport_times < final_override[0]
    )
    during_final_safety = (sport_times >= final_override[0]) & (
        sport_times <= float(stop["wall_time"])
    )
    final_stop_timeline["sport_speed_after_last_key_before_safety_mps"] = percentile(
        sport_speed[after_last_key_before_safety]
    )
    final_stop_timeline["sport_speed_during_final_safety_zero_mps"] = percentile(
        sport_speed[during_final_safety]
    )
    final_stop_timeline["progress_after_last_remote_group_before_safety_m"] = float(
        nearest_progress(final_override[0])["progress_m"]
        - nearest_progress(final_remote[1])["progress_m"]
    )

    performance_rows = parse_performance()
    performance = {
        "pure_auto_before_first_obstacle": performance_metrics(
            performance_rows, motion_start, first_override
        ),
        "full_motion": performance_metrics(performance_rows, motion_start, motion_end),
        "tracking_correlation_during_pure_auto": performance_tracking_correlation(
            performance_rows,
            times,
            projection["distance"],
            odom_age_ms,
            motion_start,
            first_override,
        ),
        "at_first_obstacle_window": performance_metrics(
            performance_rows, first_override - 3.0, first_override + 5.0
        ),
        "at_maximum_odom_gap_window": performance_metrics(
            performance_rows, gap_epoch - 2.0, gap_epoch + 2.0
        ),
    }

    runtime_route = read_route(RUNTIME_ROUTE)
    runtime_xy = np.c_[runtime_route["x"], runtime_route["y"]]
    runtime_length = float(np.sum(np.linalg.norm(np.diff(runtime_xy, axis=0), axis=1)))

    output = {
        "schema": "go2.patrol_segmented_forensics.v1",
        "source_run": "xunjian-20260725-17",
        "timeline": {
            "motion_start_epoch": motion_start,
            "motion_start_local": local_time(motion_start),
            "first_safety_override_epoch": first_override,
            "first_safety_override_local": local_time(first_override),
            "first_remote_input_epoch": float(remote_groups[0][0]),
            "first_remote_input_local": local_time(float(remote_groups[0][0])),
            "main_remote_end_epoch": float(main_remote[1]),
            "stable_recovery_one_epoch": recovery_one,
            "stable_recovery_one_local": local_time(recovery_one),
            "brief_remote_group": [float(second_remote[0]), float(second_remote[1])],
            "stable_recovery_two_epoch": recovery_two,
            "final_remote_group": [float(final_remote[0]), float(final_remote[1])],
            "trace_stop_epoch": float(stop["wall_time"]),
            "trace_stop_local": local_time(float(stop["wall_time"])),
            "trace_stop_reason": stop["reason"],
        },
        "route": {
            "waypoints": int(len(route_xy)),
            "horizontal_route_length_m": route_length,
            "runtime_xy_route_length_m": runtime_length,
            "route_rotation_deg": float(anchor["route_rotation_deg"]),
            "final_progress_m": final_progress,
            "final_progress_fraction": final_progress / route_length,
            "final_nearest_route_segment": int(projection["segment"][-1]),
            "raw_lio_coordinate_absolute_max_m": float(np.max(np.abs(raw_pose_xyz))),
            "horizontal_z_m": percentile(horizontal_z),
        },
        "projection_validation": projection_validation,
        "safety_override_intervals": [
            {
                "start_epoch": float(begin),
                "end_epoch": float(end),
                "start_local": local_time(float(begin)),
                "end_local": local_time(float(end)),
                "duration_s": float(end - begin),
            }
            for begin, end in override_intervals
        ],
        "remote_active_groups": [
            {
                "start_epoch": float(begin),
                "end_epoch": float(end),
                "start_local": local_time(float(begin)),
                "end_local": local_time(float(end)),
            }
            for begin, end in remote_groups
        ],
        "phases": phases,
        "all_clean_automatic_segments": clean_auto,
        "pure_auto_control_output": {
            "vx_mps": percentile(pure_vx),
            "absolute_yaw_rate_radps": percentile(pure_yaw_rate),
            "yaw_rate_saturation_count": int(np.sum(pure_yaw_rate >= 0.45 - 1e-9)),
        },
        "intervention_causality": intervention_causality,
        "latency_correlation": latency_correlation,
        "performance": performance,
        "final_stop": final_stop_timeline,
        "audit_coordinate_warning": {
            "automatic_audit_used_runtime_xy_route_length_m": runtime_length,
            "follower_used_horizontal_route_length_m": route_length,
            "automatic_audit_route_tracking_alarm_is_not_valid_in_horizontal_frame_mode": True,
            "proof": (
                "The independent projection onto route_horizontal.csv agrees with the "
                "follower trace to the numerical difference reported above."
            ),
        },
        "ground_truth_limit": (
            "Onboard route/pose agreement proves tracking in the frozen FAST-LIO horizontal "
            "frame. It cannot alone prove centimetre-level physical floor truth because no "
            "independent fixed-camera/floor-marker measurement was recorded."
        ),
    }
    (ROOT / "segmented_metrics.json").write_text(
        json.dumps(json_safe(output), ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )

    with (ROOT / "segmented_samples.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            [
                "wall_time",
                "local_time",
                "x",
                "y",
                "route_progress_m",
                "cross_track_m",
                "signed_cross_track_m",
                "heading_error_vs_csv_yaw_deg",
                "odom_stamp_age_ms",
            ]
        )
        for index in range(len(times)):
            writer.writerow(
                [
                    f"{times[index]:.6f}",
                    local_time(float(times[index])),
                    f"{poses[index, 0]:.6f}",
                    f"{poses[index, 1]:.6f}",
                    f"{projection['progress'][index]:.6f}",
                    f"{projection['distance'][index]:.6f}",
                    f"{projection['signed'][index]:.6f}",
                    f"{csv_heading_error_deg[index]:.6f}",
                    f"{odom_age_ms[index]:.3f}",
                ]
            )

    phase_colors = {
        "pure_auto_before_first_obstacle": "#15803d",
        "obstacle_manual_intervention_and_recovery": "#dc2626",
        "recovered_auto_before_brief_correction": "#2563eb",
        "brief_manual_correction_and_recovery": "#f97316",
        "recovered_auto_before_final_stop": "#0891b2",
        "final_remote_stop_to_sigterm": "#7c3aed",
    }
    fig, axis = plt.subplots(figsize=(11, 8))
    axis.plot(aligned_route[:, 0], aligned_route[:, 1], color="#b9bec7", linewidth=1.5, label="CSV horizontal route")
    for name, begin, end in phase_bounds:
        mask = (times >= begin) & (times < end)
        axis.plot(
            poses[mask, 0],
            poses[mask, 1],
            color=phase_colors[name],
            linewidth=1.2,
            label=name.replace("_", " "),
        )
    axis.scatter(
        poses[np.argmin(np.abs(times - first_override)), 0],
        poses[np.argmin(np.abs(times - first_override)), 1],
        marker="x",
        s=75,
        color="black",
        label="first safety override",
        zorder=4,
    )
    axis.set_aspect("equal", adjustable="datalim")
    axis.set_xlabel("Horizontal-frame X (m)")
    axis.set_ylabel("Horizontal-frame Y (m)")
    axis.set_title("Patrol 17: executed trajectory versus exact controller CSV")
    axis.grid(alpha=0.2)
    axis.legend(fontsize=7, loc="best")
    fig.tight_layout()
    fig.savefig(ROOT / "trajectory_segmented.png", dpi=180)
    plt.close(fig)

    elapsed_min = (times - motion_start) / 60.0
    fig, axes = plt.subplots(2, 1, figsize=(13, 8), sharex=True)
    axes[0].plot(elapsed_min, projection["distance"], color="#0f4c81", linewidth=0.8)
    axes[0].axhline(0.10, color="#dc2626", linestyle="--", linewidth=0.9, label="10 cm")
    axes[0].set_ylabel("Cross-track error (m)")
    axes[0].set_ylim(bottom=0)
    axes[0].grid(alpha=0.2)
    axes[0].legend(loc="upper left")
    axes[1].plot(elapsed_min, odom_age_ms, color="#6b21a8", linewidth=0.7)
    axes[1].set_ylabel("Consumed odom stamp age (ms)")
    axes[1].set_xlabel("Elapsed patrol time (min)")
    axes[1].grid(alpha=0.2)
    for begin, end in override_intervals:
        for axis in axes:
            axis.axvspan(
                (begin - motion_start) / 60.0,
                (end - motion_start) / 60.0,
                color="#fca5a5",
                alpha=0.25,
            )
    for begin, _ in remote_groups:
        for axis in axes:
            axis.axvline(
                (begin - motion_start) / 60.0,
                color="#f97316",
                linestyle=":",
                linewidth=1.0,
            )
    axes[0].set_title(
        "Patrol 17 timeline (red shading: safety zero; orange lines: physical remote input)"
    )
    fig.tight_layout()
    fig.savefig(ROOT / "error_latency_timeline.png", dpi=180)
    plt.close(fig)

    print(json.dumps(json_safe(output), ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()

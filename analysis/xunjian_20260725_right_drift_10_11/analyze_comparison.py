#!/usr/bin/env python3
"""Compare the 2026-07-25 recording and patrol runs 08, 10, and 11.

The script reads only copied evidence.  It reconstructs the outbound pass in a
common source-route frame, quantifies the heading adapter, and separates the
run-11 obstacle stop from the later ping-pong endpoint reversal.
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
FORMAL = ROOT.parent / "xunjian_20260725_formal_xbf8"
RUNS = {
    "08_raw_lio_yaw": FORMAL / "patrol",
    "10_body_yaw_first": ROOT / "xunjian-20260725-10",
    "11_body_yaw_second": ROOT / "xunjian-20260725-11",
}
SECTIONS = ((0.0, 55.0), (60.0, 140.0), (145.0, 159.0))


def read_jsonl(
    path: Path,
    wanted: set[str] | None = None,
) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    with path.open(encoding="utf-8") as handle:
        for line in handle:
            if not line.strip():
                continue
            row = json.loads(line)
            if wanted is None or row.get("kind") in wanted:
                rows.append(row)
    return rows


def read_route(path: Path) -> dict[str, np.ndarray]:
    columns: dict[str, list[float]] = {
        "id": [],
        "x": [],
        "y": [],
        "yaw": [],
        "v": [],
    }
    with path.open(newline="", encoding="utf-8-sig") as handle:
        for row in csv.DictReader(handle):
            for key in columns:
                columns[key].append(float(row[key]))
    return {
        key: np.asarray(values, dtype=float)
        for key, values in columns.items()
    }


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def wrap_angle(value: np.ndarray | float) -> np.ndarray | float:
    return (value + np.pi) % (2.0 * np.pi) - np.pi


def summary(
    values: Iterable[float],
    percentiles: tuple[int, ...] = (5, 50, 95, 100),
) -> dict[str, float | int | None]:
    array = np.asarray(list(values), dtype=float)
    array = array[np.isfinite(array)]
    result: dict[str, float | int | None] = {"count": int(len(array))}
    if not len(array):
        return {
            **result,
            "mean": None,
            **{f"p{percentile}": None for percentile in percentiles},
        }
    result["mean"] = float(np.mean(array))
    for percentile in percentiles:
        result[f"p{percentile}"] = float(
            np.percentile(array, percentile)
        )
    return result


def route_geometry(
    route_xy: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    vectors = np.diff(route_xy, axis=0)
    lengths = np.linalg.norm(vectors, axis=1)
    cumulative = np.r_[0.0, np.cumsum(lengths)]
    return vectors, lengths, cumulative


def progress_for(
    row: dict[str, Any],
    segment_lengths: np.ndarray,
    cumulative: np.ndarray,
) -> float:
    projection = row["projection"]
    segment = int(projection["segment"])
    fraction = float(projection["fraction"])
    return float(
        cumulative[segment] + fraction * segment_lengths[segment]
    )


def unique_odom(
    rows: list[dict[str, Any]],
) -> list[dict[str, Any]]:
    seen: set[int] = set()
    result: list[dict[str, Any]] = []
    for row in rows:
        sequence = int(row["odom_callback_sequence"])
        if sequence in seen:
            continue
        seen.add(sequence)
        result.append(row)
    return result


def outbound_controls(
    trace_rows: list[dict[str, Any]],
) -> list[dict[str, Any]]:
    active = [
        row
        for row in trace_rows
        if row.get("kind") == "control"
        and row["control"].get("motion_enabled")
        and row["pose"].get("x") is not None
    ]
    result: list[dict[str, Any]] = []
    for row in active:
        if int(row["control"]["direction"]) != 1:
            break
        result.append(row)
    return result


def inverse_anchor_transform(
    xy: np.ndarray,
    anchor: dict[str, Any],
) -> np.ndarray:
    transform = anchor["transform"]
    current = transform["current_start"]
    source = transform["source_start"]
    delta = float(transform["delta_yaw"])
    translated = xy - np.asarray([current["x"], current["y"]])
    cosine = math.cos(-delta)
    sine = math.sin(-delta)
    rotation = np.asarray([[cosine, -sine], [sine, cosine]])
    return translated @ rotation.T + np.asarray(
        [source["x"], source["y"]]
    )


def section_mask(
    progress: np.ndarray,
    low: float,
    high: float,
) -> np.ndarray:
    return (progress >= low) & (progress < high)


def binned(
    progress: np.ndarray,
    values: np.ndarray,
    width: float = 5.0,
) -> list[dict[str, Any]]:
    result: list[dict[str, Any]] = []
    for low in np.arange(0.0, 160.0, width):
        keep = section_mask(progress, float(low), float(low + width))
        if not np.any(keep):
            continue
        result.append(
            {
                "start_m": float(low),
                "end_m": float(low + width),
                "count": int(np.sum(keep)),
                "median": float(np.median(values[keep])),
                "p5": float(np.percentile(values[keep], 5)),
                "p95": float(np.percentile(values[keep], 95)),
            }
        )
    return result


def parse_follower_events(run_dir: Path) -> dict[str, Any]:
    text = (run_dir / "waypoint_follower.log").read_text(
        encoding="utf-8",
        errors="replace",
    )
    body_match = re.search(
        r"FOLLOWER_BODY_YAW_READY "
        r"offset_deg=([-+0-9.]+) "
        r"spread_deg=([-+0-9.]+) "
        r"samples=(\d+)",
        text,
    )
    stuck = [
        {
            "epoch": float(epoch),
            "nearest_before": int(before),
            "nearest_after": int(after),
        }
        for epoch, before, after in re.findall(
            r"\[([0-9.]+)\].*stuck recovery: "
            r"nearest_index (\d+) -> (\d+)",
            text,
        )
    ]
    result: dict[str, Any] = {"stuck_recovery": stuck}
    if body_match:
        result["body_yaw_alignment"] = {
            "offset_deg": float(body_match.group(1)),
            "spread_deg": float(body_match.group(2)),
            "samples": int(body_match.group(3)),
        }
    return result


def analyze_run(
    label: str,
    run_dir: Path,
) -> tuple[dict[str, Any], dict[str, np.ndarray]]:
    route = read_route(run_dir / "route_runtime.csv")
    route_xy = np.c_[route["x"], route["y"]]
    _, segment_lengths, cumulative = route_geometry(route_xy)
    trace = read_jsonl(run_dir / "follower_control_trace.jsonl")
    outbound_all = outbound_controls(trace)
    outbound = unique_odom(outbound_all)
    anchor = json.loads(
        (run_dir / "manual_anchor.json").read_text(encoding="utf-8")
    )
    audit = json.loads(
        (run_dir / "experiment_audit.json").read_text(encoding="utf-8")
    )
    events = parse_follower_events(run_dir)

    progress = np.asarray(
        [
            progress_for(row, segment_lengths, cumulative)
            for row in outbound
        ]
    )
    xy = np.asarray(
        [[row["pose"]["x"], row["pose"]["y"]] for row in outbound]
    )
    source_xy = inverse_anchor_transform(xy, anchor)
    signed = np.asarray(
        [row["control"]["signed_cross_track_m"] for row in outbound],
        dtype=float,
    )
    absolute = np.asarray(
        [row["control"]["cross_track_m"] for row in outbound],
        dtype=float,
    )
    used_yaw = np.asarray(
        [row["pose"]["yaw"] for row in outbound],
        dtype=float,
    )
    raw_yaw = np.asarray(
        [
            (
                row["pose"].get("raw_lio_yaw")
                if row["pose"].get("raw_lio_yaw") is not None
                else row["pose"]["yaw"]
            )
            for row in outbound
        ],
        dtype=float,
    )
    route_heading = np.asarray(
        [row["projection"]["heading"] for row in outbound],
        dtype=float,
    )
    used_minus_raw = np.degrees(wrap_angle(used_yaw - raw_yaw))
    track_minus_used = np.degrees(
        wrap_angle(route_heading - used_yaw)
    )
    track_minus_raw = np.degrees(
        wrap_angle(route_heading - raw_yaw)
    )
    target_distance = np.asarray(
        [row["control"]["target_distance_m"] for row in outbound],
        dtype=float,
    )
    selected_alpha = np.degrees(
        np.asarray(
            [row["control"]["selected_alpha"] for row in outbound],
            dtype=float,
        )
    )
    predicted_signed = target_distance * np.sin(
        np.radians(track_minus_used)
    )
    source_age = np.asarray(
        [row["odom_stamp_age_ms"] for row in outbound],
        dtype=float,
    )
    receive_age = np.asarray(
        [row["odom_to_control_ms"] for row in outbound],
        dtype=float,
    )
    command_yaw = np.asarray(
        [row["control"]["cmd_yaw_rate"] for row in outbound],
        dtype=float,
    )

    sections: list[dict[str, Any]] = []
    for low, high in SECTIONS:
        keep = section_mask(progress, low, high)
        sections.append(
            {
                "progress_m": [low, high],
                "count": int(np.sum(keep)),
                "signed_cross_track_m": summary(signed[keep]),
                "absolute_cross_track_m": summary(absolute[keep]),
                "used_minus_raw_yaw_deg": summary(
                    used_minus_raw[keep]
                ),
                "route_tangent_minus_used_yaw_deg": summary(
                    track_minus_used[keep]
                ),
                "route_tangent_minus_raw_yaw_deg": summary(
                    track_minus_raw[keep]
                ),
                "selected_alpha_deg": summary(selected_alpha[keep]),
                "target_distance_m": summary(target_distance[keep]),
                "predicted_signed_cross_track_m": summary(
                    predicted_signed[keep]
                ),
            }
        )

    active_all = [
        row
        for row in trace
        if row.get("kind") == "control"
        and row["control"].get("motion_enabled")
    ]
    direction_changes: list[dict[str, Any]] = []
    last_direction: int | None = None
    for row in active_all:
        direction = int(row["control"]["direction"])
        if direction == last_direction:
            continue
        direction_changes.append(
            {
                "epoch": float(row["wall_time"]),
                "direction": direction,
                "nearest_index": row.get("nearest", {}).get("index"),
                "target_index": row.get("target", {}).get("index"),
                "cmd_vx": row["control"].get("cmd_vx"),
                "cmd_yaw_rate": row["control"].get("cmd_yaw_rate"),
                "x": row["pose"].get("x"),
                "y": row["pose"].get("y"),
            }
        )
        last_direction = direction

    first = outbound[0]
    transform = anchor["transform"]
    metrics = {
        "label": label,
        "run_dir": str(run_dir),
        "route_sha256": sha256(run_dir / "route_original.csv"),
        "controller_source_counts": audit["follower_control_trace"][
            "control_source_counts"
        ],
        "anchor": {
            "translation_from_recorded_start_m": math.hypot(
                transform["current_start"]["x"]
                - transform["source_start"]["x"],
                transform["current_start"]["y"]
                - transform["source_start"]["y"],
            ),
            "delta_yaw_deg": math.degrees(
                float(transform["delta_yaw"])
            ),
            "stability_translation_span_m": anchor["stability"][
                "translation_span_m"
            ],
            "stability_yaw_span_deg": math.degrees(
                anchor["stability"]["yaw_span_rad"]
            ),
            "motion_release_position_error_m": math.hypot(
                first["pose"]["x"] - route["x"][0],
                first["pose"]["y"] - route["y"][0],
            ),
            "motion_release_used_yaw_error_deg": math.degrees(
                abs(
                    float(
                        wrap_angle(
                            first["pose"]["yaw"] - route["yaw"][0]
                        )
                    )
                )
            ),
        },
        "follower_events": events,
        "outbound": {
            "start_epoch": float(outbound_all[0]["wall_time"]),
            "end_epoch": float(outbound_all[-1]["wall_time"]),
            "duration_s": float(
                outbound_all[-1]["wall_time"]
                - outbound_all[0]["wall_time"]
            ),
            "control_cycles": len(outbound_all),
            "unique_odom": len(outbound),
            "progress_start_m": float(progress[0]),
            "progress_end_m": float(progress[-1]),
            "progress_max_m": float(np.max(progress)),
            "signed_cross_track_m": summary(signed),
            "absolute_cross_track_m": summary(absolute),
            "used_minus_raw_yaw_deg": summary(used_minus_raw),
            "route_tangent_minus_used_yaw_deg": summary(
                track_minus_used
            ),
            "route_tangent_minus_raw_yaw_deg": summary(
                track_minus_raw
            ),
            "source_stamp_age_ms": summary(source_age),
            "receive_to_control_ms": summary(receive_age),
            "absolute_command_yaw_rate_rps": summary(
                np.abs(command_yaw)
            ),
            "sections": sections,
            "signed_cross_track_bins_5m": binned(progress, signed),
            "used_minus_raw_yaw_bins_5m": binned(
                progress,
                used_minus_raw,
            ),
        },
        "direction_changes": direction_changes,
        "performance_context": {
            "best_response_lag_s": audit["measured_response"][
                "best_response_lag_s"
            ],
            "vx_command_vs_sport_error_mps": audit[
                "measured_response"
            ]["vx_command_vs_sport_error_mps"],
            "yaw_command_vs_sport_error_radps": audit[
                "measured_response"
            ]["yaw_command_vs_sport_error_radps"],
            "motor_temperature_c": audit["robot_state"][
                "motor_temperature_c"
            ],
            "battery_voltage_v": audit["robot_state"][
                "battery_voltage_v"
            ],
        },
    }
    arrays = {
        "progress": progress,
        "xy": xy,
        "source_xy": source_xy,
        "signed": signed,
        "absolute": absolute,
        "used_minus_raw": used_minus_raw,
        "track_minus_used": track_minus_used,
        "track_minus_raw": track_minus_raw,
    }
    return metrics, arrays


def nearest_values(
    source_times: np.ndarray,
    source_values: np.ndarray,
    target_times: np.ndarray,
) -> np.ndarray:
    indices = np.searchsorted(source_times, target_times)
    indices = np.clip(indices, 0, len(source_times) - 1)
    previous = np.clip(indices - 1, 0, len(source_times) - 1)
    choose_previous = (
        np.abs(source_times[previous] - target_times)
        <= np.abs(source_times[indices] - target_times)
    )
    indices = np.where(choose_previous, previous, indices)
    return source_values[indices]


def merge_intervals(
    times: np.ndarray,
    active: np.ndarray,
    max_merge_gap_s: float = 1.0,
) -> list[tuple[float, float]]:
    selected = times[active]
    if not len(selected):
        return []
    raw: list[list[float]] = [[float(selected[0]), float(selected[0])]]
    for value in selected[1:]:
        value = float(value)
        if value - raw[-1][1] <= 0.2:
            raw[-1][1] = value
        else:
            raw.append([value, value])
    merged: list[list[float]] = []
    for start, end in raw:
        if merged and start - merged[-1][1] <= max_merge_gap_s:
            merged[-1][1] = end
        else:
            merged.append([start, end])
    return [(start, end) for start, end in merged]


def analyze_run11_timeline(
    metrics: dict[str, Any],
) -> tuple[dict[str, Any], dict[str, np.ndarray]]:
    run_dir = RUNS["11_body_yaw_second"]
    trace = read_jsonl(run_dir / "follower_control_trace.jsonl")
    active = [
        row
        for row in trace
        if row.get("kind") == "control"
        and row["control"].get("motion_enabled")
    ]
    route = read_route(run_dir / "route_runtime.csv")
    route_xy = np.c_[route["x"], route["y"]]
    _, segment_lengths, cumulative = route_geometry(route_xy)
    control_time = np.asarray([row["wall_time"] for row in active])
    patrol_vx = np.asarray(
        [row["control"]["cmd_vx"] for row in active],
        dtype=float,
    )
    direction = np.asarray(
        [row["control"]["direction"] for row in active],
        dtype=int,
    )
    progress = np.asarray(
        [
            progress_for(row, segment_lengths, cumulative)
            for row in active
        ]
    )
    nearest_index = np.asarray(
        [row.get("nearest", {}).get("index", -1) for row in active],
        dtype=int,
    )

    telemetry = read_jsonl(
        run_dir / "experiment_telemetry.jsonl",
        {"cmd_vel", "sport"},
    )
    command_rows = [
        row for row in telemetry if row["kind"] == "cmd_vel"
    ]
    sport_rows = [
        row for row in telemetry if row["kind"] == "sport"
    ]
    command_time = np.asarray(
        [row["wall_time"] for row in command_rows],
        dtype=float,
    )
    command_vx = np.asarray(
        [row["data"]["linear"]["x"] for row in command_rows],
        dtype=float,
    )
    sport_time = np.asarray(
        [row["wall_time"] for row in sport_rows],
        dtype=float,
    )
    sport_speed = np.asarray(
        [
            math.hypot(
                row["data"]["velocity"][0],
                row["data"]["velocity"][1],
            )
            for row in sport_rows
        ],
        dtype=float,
    )
    safe_vx = nearest_values(command_time, command_vx, control_time)
    body_speed = nearest_values(sport_time, sport_speed, control_time)

    suppressed = (
        (direction == 1)
        & (patrol_vx >= 0.4)
        & (safe_vx <= 0.05)
    )
    intervals = merge_intervals(control_time, suppressed)
    obstacle_interval = max(
        intervals,
        key=lambda interval: interval[1] - interval[0],
    )
    obstacle_keep = (
        (control_time >= obstacle_interval[0] - 0.1)
        & (control_time <= obstacle_interval[1] + 0.1)
    )

    safe_text = (run_dir / "unitree_safe_cmd_node.log").read_text(
        encoding="utf-8",
        errors="replace",
    )
    detected = [
        {
            "epoch": float(epoch),
            "stop_count": int(stop_count),
            "roi_count": int(roi_count),
            "nearest_x_m": float(nearest_x),
        }
        for epoch, stop_count, roi_count, nearest_x in re.findall(
            r"\[([0-9.]+)\].*OBSTACLE DETECTED: "
            r"stop_count=(\d+), roi_count=(\d+), "
            r"lateral_count=\d+, nearest_x=([0-9.]+)",
            safe_text,
        )
    ]

    reversal = next(
        change
        for change in metrics["direction_changes"]
        if change["direction"] == -1
    )
    reversal_epoch = float(reversal["epoch"])
    reverse_move_candidates = [
        row
        for row in active
        if row["wall_time"] >= reversal_epoch
        and int(row["control"]["direction"]) == -1
        and float(row["control"]["cmd_vx"]) > 0.1
    ]
    reverse_move_epoch = float(
        reverse_move_candidates[0]["wall_time"]
    )

    obstacle_progress_start = float(
        progress[np.flatnonzero(obstacle_keep)[0]]
    )
    obstacle_progress_end = float(
        progress[np.flatnonzero(obstacle_keep)[-1]]
    )
    timeline_metrics = {
        "safe_node_obstacle_detections": detected,
        "follower_command_suppressed_interval_epoch": list(
            obstacle_interval
        ),
        "suppressed_duration_s": float(
            obstacle_interval[1] - obstacle_interval[0]
        ),
        "body_speed_mps_during_suppression": summary(
            body_speed[obstacle_keep]
        ),
        "route_progress_m_during_suppression": [
            obstacle_progress_start,
            obstacle_progress_end,
        ],
        "nearest_index_during_suppression": [
            int(np.min(nearest_index[obstacle_keep])),
            int(np.max(nearest_index[obstacle_keep])),
        ],
        "direction_during_suppression": sorted(
            set(int(value) for value in direction[obstacle_keep])
        ),
        "stuck_recovery_events_near_obstacle": [
            event
            for event in metrics["follower_events"]["stuck_recovery"]
            if obstacle_interval[0] - 1.0
            <= event["epoch"]
            <= obstacle_interval[1] + 1.0
        ],
        "endpoint_reversal": {
            **reversal,
            "route_length_m": float(cumulative[-1]),
            "seconds_after_obstacle_suppression_ended": float(
                reversal_epoch - obstacle_interval[1]
            ),
            "first_reverse_translation_epoch": reverse_move_epoch,
            "turn_in_place_duration_s": float(
                reverse_move_epoch - reversal_epoch
            ),
        },
    }
    arrays = {
        "time": control_time,
        "patrol_vx": patrol_vx,
        "safe_vx": safe_vx,
        "body_speed": body_speed,
        "direction": direction,
        "progress": progress,
    }
    return timeline_metrics, arrays


def json_safe(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(key): json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    if isinstance(value, np.ndarray):
        return [json_safe(item) for item in value.tolist()]
    if isinstance(value, (np.floating, np.integer)):
        return value.item()
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def section(
    run_metrics: dict[str, Any],
    low: float,
    high: float,
) -> dict[str, Any]:
    return next(
        item
        for item in run_metrics["outbound"]["sections"]
        if item["progress_m"] == [low, high]
    )


def main() -> None:
    run_metrics: dict[str, dict[str, Any]] = {}
    arrays: dict[str, dict[str, np.ndarray]] = {}
    for label, run_dir in RUNS.items():
        run_metrics[label], arrays[label] = analyze_run(label, run_dir)

    timeline, timeline_arrays = analyze_run11_timeline(
        run_metrics["11_body_yaw_second"]
    )

    old_long = section(run_metrics["08_raw_lio_yaw"], 60.0, 140.0)
    first_long = section(
        run_metrics["10_body_yaw_first"],
        60.0,
        140.0,
    )
    second_long = section(
        run_metrics["11_body_yaw_second"],
        60.0,
        140.0,
    )
    first_all = run_metrics["10_body_yaw_first"]["outbound"]
    second_all = run_metrics["11_body_yaw_second"]["outbound"]
    comparison = {
        "route_identity": {
            label: metrics["route_sha256"]
            for label, metrics in run_metrics.items()
        },
        "all_route_hashes_identical": (
            len(
                {
                    metrics["route_sha256"]
                    for metrics in run_metrics.values()
                }
            )
            == 1
        ),
        "old_to_new_long_section_sign_flip": {
            "old_run08_signed_median_m": old_long[
                "signed_cross_track_m"
            ]["p50"],
            "new_run10_signed_median_m": first_long[
                "signed_cross_track_m"
            ]["p50"],
            "new_run11_signed_median_m": second_long[
                "signed_cross_track_m"
            ]["p50"],
        },
        "run11_minus_run10_outbound": {
            "signed_median_change_m": (
                second_all["signed_cross_track_m"]["p50"]
                - first_all["signed_cross_track_m"]["p50"]
            ),
            "absolute_p95_change_m": (
                second_all["absolute_cross_track_m"]["p95"]
                - first_all["absolute_cross_track_m"]["p95"]
            ),
            "absolute_max_change_m": (
                second_all["absolute_cross_track_m"]["p100"]
                - first_all["absolute_cross_track_m"]["p100"]
            ),
            "body_yaw_alignment_offset_change_deg": (
                run_metrics["11_body_yaw_second"]["follower_events"][
                    "body_yaw_alignment"
                ]["offset_deg"]
                - run_metrics["10_body_yaw_first"]["follower_events"][
                    "body_yaw_alignment"
                ]["offset_deg"]
            ),
            "long_section_route_tangent_minus_used_yaw_change_deg": (
                second_long["route_tangent_minus_used_yaw_deg"]["p50"]
                - first_long["route_tangent_minus_used_yaw_deg"]["p50"]
            ),
        },
        "run11_obstacle_and_endpoint": timeline,
    }
    result = {
        "schema": "go2.patrol_comparison.20260725.v1",
        "run_metrics": run_metrics,
        "comparison": comparison,
    }
    (ROOT / "comparison_metrics.json").write_text(
        json.dumps(json_safe(result), ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )

    with (ROOT / "cross_track_comparison.csv").open(
        "w",
        newline="",
        encoding="utf-8",
    ) as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=[
                "run",
                "progress_start_m",
                "progress_end_m",
                "count",
                "signed_median_m",
                "signed_p5_m",
                "signed_p95_m",
            ],
        )
        writer.writeheader()
        for label, metrics in run_metrics.items():
            for row in metrics["outbound"][
                "signed_cross_track_bins_5m"
            ]:
                writer.writerow(
                    {
                        "run": label,
                        "progress_start_m": row["start_m"],
                        "progress_end_m": row["end_m"],
                        "count": row["count"],
                        "signed_median_m": row["median"],
                        "signed_p5_m": row["p5"],
                        "signed_p95_m": row["p95"],
                    }
                )

    source_route = read_route(
        RUNS["08_raw_lio_yaw"] / "route_original.csv"
    )
    source_route_xy = np.c_[source_route["x"], source_route["y"]]
    colors = {
        "08_raw_lio_yaw": "#4e79a7",
        "10_body_yaw_first": "#f28e2b",
        "11_body_yaw_second": "#e15759",
    }
    names = {
        "08_raw_lio_yaw": "run 08: raw FAST-LIO yaw",
        "10_body_yaw_first": "run 10: body yaw, first",
        "11_body_yaw_second": "run 11: body yaw, second",
    }

    fig, ax = plt.subplots(figsize=(10, 8))
    ax.plot(
        source_route_xy[:, 0],
        source_route_xy[:, 1],
        color="black",
        lw=2.3,
        label="recorded CSV",
    )
    for label, values in arrays.items():
        ax.plot(
            values["source_xy"][:, 0],
            values["source_xy"][:, 1],
            color=colors[label],
            lw=1.3,
            label=names[label],
        )
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("source-route x (m)")
    ax.set_ylabel("source-route y (m)")
    ax.set_title("Three outbound passes transformed into the recorded CSV frame")
    ax.grid(alpha=0.25)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(ROOT / "trajectory_comparison_source_frame.png", dpi=180)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(13, 5.5))
    for label, metrics in run_metrics.items():
        bins = metrics["outbound"]["signed_cross_track_bins_5m"]
        centers = [
            (row["start_m"] + row["end_m"]) / 2.0 for row in bins
        ]
        medians = [row["median"] for row in bins]
        ax.plot(
            centers,
            medians,
            color=colors[label],
            lw=2.2,
            marker="o",
            ms=3,
            label=names[label],
        )
    ax.axhline(0.0, color="black", lw=0.8)
    ax.axvspan(60.0, 140.0, color="#999999", alpha=0.08)
    ax.set_xlabel("route progress (m)")
    ax.set_ylabel("signed cross-track median per 5 m (m)")
    ax.set_title("The body-yaw change reverses the systematic side bias")
    ax.grid(alpha=0.25)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(ROOT / "signed_cross_track_comparison.png", dpi=180)
    plt.close(fig)

    fig, axes = plt.subplots(2, 1, figsize=(13, 8), sharex=True)
    for label in ("10_body_yaw_first", "11_body_yaw_second"):
        values = arrays[label]
        step = max(1, len(values["progress"]) // 3000)
        axes[0].scatter(
            values["progress"][::step],
            values["used_minus_raw"][::step],
            s=3,
            alpha=0.15,
            color=colors[label],
        )
        bins = run_metrics[label]["outbound"][
            "used_minus_raw_yaw_bins_5m"
        ]
        centers = [
            (row["start_m"] + row["end_m"]) / 2.0 for row in bins
        ]
        medians = [row["median"] for row in bins]
        axes[0].plot(
            centers,
            medians,
            color=colors[label],
            lw=2.2,
            label=names[label],
        )
        cross_bins = run_metrics[label]["outbound"][
            "signed_cross_track_bins_5m"
        ]
        axes[1].plot(
            [
                (row["start_m"] + row["end_m"]) / 2.0
                for row in cross_bins
            ],
            [row["median"] for row in cross_bins],
            color=colors[label],
            lw=2.2,
            marker="o",
            ms=3,
            label=names[label],
        )
    axes[0].axhline(0.0, color="black", lw=0.8)
    axes[0].set_ylabel("used body yaw − raw LIO yaw (deg)")
    axes[0].set_title("Runtime heading correction produced by the body-yaw adapter")
    axes[1].axhline(0.0, color="black", lw=0.8)
    axes[1].set_xlabel("route progress (m)")
    axes[1].set_ylabel("signed cross-track (m)")
    axes[1].set_title("Resulting side offset")
    for axis in axes:
        axis.axvspan(60.0, 140.0, color="#999999", alpha=0.08)
        axis.grid(alpha=0.25)
        axis.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(ROOT / "heading_adapter_comparison.png", dpi=180)
    plt.close(fig)

    obstacle_start, obstacle_end = timeline[
        "follower_command_suppressed_interval_epoch"
    ]
    reversal_epoch = timeline["endpoint_reversal"]["epoch"]
    windows = [
        (obstacle_start - 3.0, obstacle_end + 5.0, "Obstacle stop"),
        (reversal_epoch - 5.0, reversal_epoch + 10.0, "Endpoint reversal"),
    ]
    fig, axes = plt.subplots(2, 1, figsize=(13, 8))
    for axis, (low, high, title) in zip(axes, windows):
        keep = (
            (timeline_arrays["time"] >= low)
            & (timeline_arrays["time"] <= high)
        )
        relative = timeline_arrays["time"][keep] - low
        axis.plot(
            relative,
            timeline_arrays["patrol_vx"][keep],
            lw=2,
            label="follower requested vx",
        )
        axis.plot(
            relative,
            timeline_arrays["safe_vx"][keep],
            lw=2,
            label="safe-node output vx",
        )
        axis.plot(
            relative,
            timeline_arrays["body_speed"][keep],
            lw=1.5,
            label="Unitree reported planar speed",
        )
        axis.set_title(title)
        axis.set_xlabel(f"seconds since epoch {low:.3f}")
        axis.set_ylabel("m/s")
        axis.grid(alpha=0.25)
        axis.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(ROOT / "run11_obstacle_endpoint_timeline.png", dpi=180)
    plt.close(fig)

    print(json.dumps(json_safe(result), ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()

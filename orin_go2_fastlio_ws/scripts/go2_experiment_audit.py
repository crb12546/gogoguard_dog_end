#!/usr/bin/env python3
"""Build an evidence-based audit for one completed Go2 patrol run.

This script uses only captured files.  It does not contact ROS or the robot and
therefore cannot alter the run it is evaluating.
"""

from __future__ import print_function

import argparse
import csv
import datetime
import json
import math
import os
import re
import tempfile
from pathlib import Path


NUMBER_RE = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?"
KEY_VALUE_RE = re.compile(
    r"([A-Za-z][A-Za-z0-9_]*)=("
    + NUMBER_RE
    + r"(?:/"
    + NUMBER_RE
    + r")*)"
)
ROS_TIME_RE = re.compile(r"^\[[A-Z]+\]\s+\[(" + NUMBER_RE + r")\]")


def iso_time():
    return datetime.datetime.now().astimezone().isoformat(
        timespec="milliseconds"
    )


def normalize_angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def percentile(values, fraction):
    finite = sorted(
        float(value)
        for value in values
        if value is not None and math.isfinite(float(value))
    )
    if not finite:
        return None
    position = (len(finite) - 1) * fraction
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return finite[lower]
    weight = position - lower
    return finite[lower] * (1.0 - weight) + finite[upper] * weight


def summary(values):
    finite = [
        float(value)
        for value in values
        if value is not None and math.isfinite(float(value))
    ]
    if not finite:
        return {
            "count": 0,
            "min": None,
            "mean": None,
            "median": None,
            "p95": None,
            "max": None,
        }
    return {
        "count": len(finite),
        "min": min(finite),
        "mean": sum(finite) / len(finite),
        "median": percentile(finite, 0.50),
        "p95": percentile(finite, 0.95),
        "max": max(finite),
    }


def atomic_write(path, text):
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(
        prefix=".%s." % output.name,
        suffix=".tmp",
        dir=str(output.parent),
        text=True,
    )
    try:
        with os.fdopen(fd, "w") as handle:
            handle.write(text)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, str(output))
    except Exception:
        try:
            os.unlink(temporary)
        except OSError:
            pass
        raise


def atomic_write_json(path, payload):
    atomic_write(
        path,
        json.dumps(
            payload,
            ensure_ascii=False,
            indent=2,
            sort_keys=True,
        )
        + "\n",
    )


def load_json(path, default=None):
    try:
        return json.loads(Path(path).read_text())
    except (OSError, ValueError):
        return default


def load_route(path):
    route = []
    try:
        handle = open(path, "r", newline="")
    except OSError:
        return route
    with handle:
        reader = csv.DictReader(handle)
        for index, row in enumerate(reader):
            try:
                route.append(
                    {
                        "index": index,
                        "x": float(row["x"]),
                        "y": float(row["y"]),
                        "yaw": float(row["yaw"]),
                        "v": (
                            float(row["v"])
                            if row.get("v", "") != ""
                            else None
                        ),
                    }
                )
            except (KeyError, TypeError, ValueError):
                continue
    return route


def load_telemetry(path):
    records = {}
    malformed = 0
    try:
        handle = open(path, "r")
    except OSError:
        return records, malformed
    with handle:
        for line in handle:
            try:
                record = json.loads(line)
                kind = str(record["kind"])
                wall_time = float(record["wall_time"])
            except (KeyError, TypeError, ValueError):
                malformed += 1
                continue
            record["wall_time"] = wall_time
            records.setdefault(kind, []).append(record)
    for values in records.values():
        values.sort(key=lambda item: item["wall_time"])
    return records, malformed


def load_control_trace(path):
    records = []
    malformed = 0
    clean_stop = False
    try:
        handle = open(path, "r")
    except OSError:
        return records, malformed, clean_stop
    with handle:
        for line in handle:
            try:
                record = json.loads(line)
            except (TypeError, ValueError):
                malformed += 1
                continue
            if record.get("kind") == "trace_stop":
                clean_stop = True
            if record.get("kind") != "control":
                continue
            try:
                record["wall_time"] = float(record["wall_time"])
            except (KeyError, TypeError, ValueError):
                malformed += 1
                continue
            records.append(record)
    records.sort(key=lambda item: item["wall_time"])
    return records, malformed, clean_stop


def control_trace_audit(path, telemetry_records):
    records, malformed, clean_stop = load_control_trace(path)
    if not records:
        return {
            "available": False,
            "reason": "follower control trace is unavailable",
            "malformed_lines": malformed,
            "clean_stop": clean_stop,
        }
    odom_to_control = []
    odom_stamp_age = []
    compute_to_trace = []
    cross_track = []
    selected_alpha = []
    source_counts = {}
    odom_sequences = []
    active_records = []
    for record in records:
        control = record.get("control", {})
        try:
            sequence = int(
                record.get("odom_callback_sequence", 0)
            )
        except (TypeError, ValueError):
            sequence = 0
        if sequence > 0:
            odom_sequences.append(sequence)
        for target, value in (
            (odom_to_control, record.get("odom_to_control_ms")),
            (odom_stamp_age, record.get("odom_stamp_age_ms")),
            (compute_to_trace, record.get("compute_to_trace_ms")),
            (cross_track, control.get("cross_track_m")),
            (selected_alpha, control.get("selected_alpha")),
        ):
            try:
                value = float(value)
                if math.isfinite(value):
                    target.append(value)
            except (TypeError, ValueError):
                pass
        source = str(control.get("source", "unknown"))
        source_counts[source] = source_counts.get(source, 0) + 1
        if not control.get("is_stop", False):
            active_records.append(record)

    consecutive_reuse = []
    current_sequence = None
    current_count = 0
    distinct_sequence_records = []
    for record in records:
        try:
            sequence = int(
                record.get("odom_callback_sequence", 0)
            )
        except (TypeError, ValueError):
            continue
        if sequence <= 0:
            continue
        if sequence == current_sequence:
            current_count += 1
            continue
        if current_count:
            consecutive_reuse.append(current_count)
        current_sequence = sequence
        current_count = 1
        distinct_sequence_records.append(record)
    if current_count:
        consecutive_reuse.append(current_count)

    callback_sequence_steps = []
    source_stamp_steps = []
    for previous, current in zip(
        distinct_sequence_records,
        distinct_sequence_records[1:],
    ):
        try:
            callback_sequence_steps.append(
                int(current["odom_callback_sequence"])
                - int(previous["odom_callback_sequence"])
            )
        except (KeyError, TypeError, ValueError):
            pass
        try:
            gap = float(current["odom_source_stamp"]) - float(
                previous["odom_source_stamp"]
            )
            if math.isfinite(gap) and gap >= 0.0:
                source_stamp_steps.append(gap)
        except (KeyError, TypeError, ValueError):
            pass

    patrol_commands = telemetry_records.get("patrol_cmd", [])
    command_errors = [[], []]
    matched_commands = 0
    command_index = 0
    for trace in records:
        if not patrol_commands:
            break
        target_time = trace["wall_time"]
        while (
            command_index + 1 < len(patrol_commands)
            and abs(
                patrol_commands[command_index + 1]["wall_time"]
                - target_time
            )
            <= abs(
                patrol_commands[command_index]["wall_time"]
                - target_time
            )
        ):
            command_index += 1
        command_record = patrol_commands[command_index]
        if abs(command_record["wall_time"] - target_time) > 0.075:
            continue
        emitted = record_twist(command_record)
        control = trace.get("control", {})
        try:
            command_errors[0].append(
                abs(float(control["cmd_vx"]) - emitted[0])
            )
            command_errors[1].append(
                abs(
                    float(control["cmd_yaw_rate"])
                    - emitted[2]
                )
            )
            matched_commands += 1
        except (KeyError, TypeError, ValueError):
            continue

    unique_sequences = len(set(odom_sequences))
    return {
        "available": True,
        "control_records": len(records),
        "active_control_records": len(active_records),
        "malformed_lines": malformed,
        "clean_stop": clean_stop,
        "unique_odom_callback_sequences_used": unique_sequences,
        "control_records_reusing_an_odom_callback": max(
            0,
            len(odom_sequences) - unique_sequences,
        ),
        "consecutive_control_cycles_per_odom": summary(
            consecutive_reuse
        ),
        "odom_callback_sequence_step": summary(
            callback_sequence_steps
        ),
        "consumed_odom_source_stamp_gap_s": summary(
            source_stamp_steps
        ),
        "odom_receive_to_control_ms": summary(odom_to_control),
        "odom_source_stamp_age_ms": summary(odom_stamp_age),
        "control_compute_to_trace_ms": summary(compute_to_trace),
        "internal_cross_track_m": summary(cross_track),
        "selected_alpha_rad": summary(selected_alpha),
        "control_source_counts": source_counts,
        "trace_to_patrol_cmd_matched_records": matched_commands,
        "trace_to_patrol_cmd_absolute_error": {
            "vx": summary(command_errors[0]),
            "yaw_rate": summary(command_errors[1]),
        },
        "interpretation": (
            "Reusing one odometry sample for two 20 Hz control cycles is "
            "normal when FAST-LIO publishes at 10 Hz. Risk is indicated by "
            "large odom age or long consecutive reuse, not by dropping "
            "intermediate sensor frames by itself."
        ),
    }


def route_cumulative_lengths(route):
    cumulative = [0.0]
    for previous, current in zip(route, route[1:]):
        cumulative.append(
            cumulative[-1]
            + math.hypot(
                current["x"] - previous["x"],
                current["y"] - previous["y"],
            )
        )
    return cumulative


def nearest_route_projection(x, y, route, cumulative):
    best = None
    for index, (start, end) in enumerate(zip(route, route[1:])):
        dx = end["x"] - start["x"]
        dy = end["y"] - start["y"]
        length_squared = dx * dx + dy * dy
        if length_squared <= 1e-12:
            fraction = 0.0
        else:
            fraction = max(
                0.0,
                min(
                    1.0,
                    (
                        (x - start["x"]) * dx
                        + (y - start["y"]) * dy
                    )
                    / length_squared,
                ),
            )
        projected_x = start["x"] + fraction * dx
        projected_y = start["y"] + fraction * dy
        error_x = x - projected_x
        error_y = y - projected_y
        distance = math.hypot(error_x, error_y)
        length = math.sqrt(length_squared)
        signed = (
            (dx * error_y - dy * error_x) / length
            if length > 1e-9
            else 0.0
        )
        candidate = {
            "segment": index,
            "fraction": fraction,
            "distance_m": distance,
            "signed_cross_track_m": signed,
            "along_route_m": cumulative[index]
            + fraction * length,
        }
        if best is None or candidate["distance_m"] < best["distance_m"]:
            best = candidate
    if best is None and route:
        return {
            "segment": 0,
            "fraction": 0.0,
            "distance_m": math.hypot(
                x - route[0]["x"],
                y - route[0]["y"],
            ),
            "signed_cross_track_m": 0.0,
            "along_route_m": 0.0,
        }
    return best


def record_twist(record):
    data = record.get("data", {})
    linear = data.get("linear", {})
    angular = data.get("angular", {})
    try:
        return (
            float(linear.get("x", 0.0)),
            float(linear.get("y", 0.0)),
            float(angular.get("z", 0.0)),
        )
    except (TypeError, ValueError):
        return (0.0, 0.0, 0.0)


def motion_window(records):
    active_times = []
    for kind in ("cmd_vel", "patrol_cmd"):
        for record in records.get(kind, []):
            command = record_twist(record)
            if (
                abs(command[0]) > 0.03
                or abs(command[1]) > 0.03
                or abs(command[2]) > 0.03
            ):
                active_times.append(record["wall_time"])
    if not active_times:
        return None
    return min(active_times) - 0.25, max(active_times) + 0.50


def extract_odom(records, window=None):
    result = []
    for record in records.get("odom", []):
        if window and not window[0] <= record["wall_time"] <= window[1]:
            continue
        data = record.get("data", {})
        position = data.get("position", {})
        orientation = data.get("orientation", {})
        linear = data.get("linear_velocity", {})
        angular = data.get("angular_velocity", {})
        try:
            item = {
                "time": record["wall_time"],
                "source_stamp": record.get("source_stamp"),
                "x": float(position["x"]),
                "y": float(position["y"]),
                "z": float(position["z"]),
                "roll": float(orientation["roll"]),
                "pitch": float(orientation["pitch"]),
                "yaw": float(orientation["yaw"]),
                "vx": float(linear.get("x", 0.0)),
                "vy": float(linear.get("y", 0.0)),
                "yaw_rate": float(angular.get("z", 0.0)),
            }
        except (KeyError, TypeError, ValueError):
            continue
        if all(
            math.isfinite(item[key])
            for key in (
                "time",
                "x",
                "y",
                "z",
                "roll",
                "pitch",
                "yaw",
            )
        ):
            result.append(item)
    return result


def verify_manual_transform(original, runtime, anchor_metadata):
    if not original or not runtime:
        return {
            "available": False,
            "reason": "original or runtime route is missing",
        }
    transform = (anchor_metadata or {}).get("transform")
    if not isinstance(transform, dict):
        return {
            "available": False,
            "reason": "manual anchor transform metadata is missing",
        }
    current = transform.get("current_start", {})
    source = transform.get("source_start", {})
    try:
        anchor_x = float(current["x"])
        anchor_y = float(current["y"])
        anchor_yaw = float(current["yaw"])
        source_x = float(source["x"])
        source_y = float(source["y"])
        source_yaw = float(source["yaw"])
    except (KeyError, TypeError, ValueError):
        return {
            "available": False,
            "reason": "manual anchor transform metadata is invalid",
        }
    delta_yaw = normalize_angle(anchor_yaw - source_yaw)
    cosine = math.cos(delta_yaw)
    sine = math.sin(delta_yaw)
    position_errors = []
    yaw_errors = []
    for source_point, runtime_point in zip(original, runtime):
        dx = source_point["x"] - source_x
        dy = source_point["y"] - source_y
        expected_x = anchor_x + cosine * dx - sine * dy
        expected_y = anchor_y + sine * dx + cosine * dy
        expected_yaw = normalize_angle(
            source_point["yaw"] + delta_yaw
        )
        position_errors.append(
            math.hypot(
                runtime_point["x"] - expected_x,
                runtime_point["y"] - expected_y,
            )
        )
        yaw_errors.append(
            abs(
                normalize_angle(
                    runtime_point["yaw"] - expected_yaw
                )
            )
        )
    count_match = len(original) == len(runtime)
    exact = (
        count_match
        and bool(position_errors)
        and max(position_errors) <= 2e-6
        and max(yaw_errors) <= 2e-6
    )
    return {
        "available": True,
        "source_waypoints": len(original),
        "runtime_waypoints": len(runtime),
        "count_match": count_match,
        "delta_yaw_rad": delta_yaw,
        "delta_yaw_deg": math.degrees(delta_yaw),
        "position_error_m": summary(position_errors),
        "yaw_error_deg": summary(
            [math.degrees(value) for value in yaw_errors]
        ),
        "exact_within_csv_precision": exact,
    }


def tracking_audit(route, odom):
    if len(route) < 2 or not odom:
        return {
            "available": False,
            "reason": "runtime route or motion odometry is unavailable",
        }
    cumulative = route_cumulative_lengths(route)
    projections = [
        nearest_route_projection(
            sample["x"],
            sample["y"],
            route,
            cumulative,
        )
        for sample in odom
    ]
    cross_track = [item["distance_m"] for item in projections]
    signed = [item["signed_cross_track_m"] for item in projections]
    progress = [item["along_route_m"] for item in projections]
    first_5m = [
        item["distance_m"]
        for item in projections
        if item["along_route_m"] <= 5.0
    ]
    regressions = [
        progress[index - 1] - progress[index]
        for index in range(1, len(progress))
        if progress[index] + 0.20 < progress[index - 1]
    ]
    initial = odom[0]
    initial_position_error = math.hypot(
        initial["x"] - route[0]["x"],
        initial["y"] - route[0]["y"],
    )
    initial_yaw_error = abs(
        normalize_angle(initial["yaw"] - route[0]["yaw"])
    )
    path_steps = [
        math.sqrt(
            (current["x"] - previous["x"]) ** 2
            + (current["y"] - previous["y"]) ** 2
            + (current["z"] - previous["z"]) ** 2
        )
        for previous, current in zip(odom, odom[1:])
    ]
    wall_gaps = [
        current["time"] - previous["time"]
        for previous, current in zip(odom, odom[1:])
    ]
    stamp_gaps = []
    for previous, current in zip(odom, odom[1:]):
        try:
            gap = float(current["source_stamp"]) - float(
                previous["source_stamp"]
            )
        except (TypeError, ValueError):
            continue
        if gap >= 0.0 and math.isfinite(gap):
            stamp_gaps.append(gap)
    return {
        "available": True,
        "motion_odom_samples": len(odom),
        "route_length_m": cumulative[-1],
        "estimated_motion_path_length_m": sum(path_steps),
        "initial_position_error_m": initial_position_error,
        "initial_yaw_error_rad": initial_yaw_error,
        "initial_yaw_error_deg": math.degrees(initial_yaw_error),
        "cross_track_error_m": summary(cross_track),
        "signed_cross_track_error_m": summary(signed),
        "first_5m_cross_track_error_m": summary(first_5m),
        "max_progress_m": max(progress),
        "final_progress_m": progress[-1],
        "progress_regression_over_0_20m_count": len(regressions),
        "progress_regression_m": summary(regressions),
        "odom_wall_gap_s": summary(wall_gaps),
        "odom_source_stamp_gap_s": summary(stamp_gaps),
        "z_m": summary([sample["z"] for sample in odom]),
        "roll_deg": summary(
            [math.degrees(sample["roll"]) for sample in odom]
        ),
        "pitch_deg": summary(
            [math.degrees(sample["pitch"]) for sample in odom]
        ),
    }


def latest_preceding(records, timestamp, max_age=0.20):
    best = None
    for record in records:
        if record["wall_time"] > timestamp:
            break
        best = record
    if best is None or timestamp - best["wall_time"] > max_age:
        return None
    return best


def command_chain_audit(records):
    raw = records.get("patrol_cmd", [])
    safe = records.get("cmd_vel", [])
    differences = [[], [], []]
    ages = []
    overrides = 0
    paired = 0
    for output in safe:
        source = latest_preceding(raw, output["wall_time"])
        if source is None:
            continue
        source_command = record_twist(source)
        output_command = record_twist(output)
        paired += 1
        ages.append(output["wall_time"] - source["wall_time"])
        for index in range(3):
            differences[index].append(
                abs(output_command[index] - source_command[index])
            )
        if (
            max(abs(value) for value in source_command) > 0.03
            and max(abs(value) for value in output_command) <= 0.01
        ):
            overrides += 1
    return {
        "patrol_cmd_records": len(raw),
        "cmd_vel_records": len(safe),
        "paired_records": paired,
        "raw_to_safe_receive_age_s": summary(ages),
        "absolute_difference": {
            "vx": summary(differences[0]),
            "vy": summary(differences[1]),
            "yaw_rate": summary(differences[2]),
        },
        "nonzero_to_zero_override_records": overrides,
        "override_fraction_of_pairs": (
            float(overrides) / paired if paired else None
        ),
    }


def sport_samples(records):
    result = []
    for record in records.get("sport", []):
        data = record.get("data", {})
        velocity = data.get("velocity", [])
        try:
            result.append(
                {
                    "time": record["wall_time"],
                    "vx": float(velocity[0]),
                    "vy": float(velocity[1]),
                    "yaw_rate": float(data["yaw_speed"]),
                    "position": [
                        float(value)
                        for value in data.get("position", [])[:3]
                    ],
                }
            )
        except (KeyError, IndexError, TypeError, ValueError):
            continue
    return result


def nearest_by_time(records, target_time, start_index=0):
    if not records:
        return None, start_index
    index = max(0, min(start_index, len(records) - 1))
    while (
        index + 1 < len(records)
        and abs(records[index + 1]["time"] - target_time)
        <= abs(records[index]["time"] - target_time)
    ):
        index += 1
    return records[index], index


def measured_response_audit(records):
    commands = records.get("cmd_vel", [])
    sport = sport_samples(records)
    if not commands or not sport:
        return {
            "available": False,
            "reason": "cmd_vel or sport-state telemetry is unavailable",
        }
    lag_trials = []
    for step in range(0, 21):
        lag = step * 0.025
        vx_errors = []
        yaw_errors = []
        index = 0
        for command_record in commands:
            command = record_twist(command_record)
            measured, index = nearest_by_time(
                sport,
                command_record["wall_time"] + lag,
                index,
            )
            if measured is None:
                continue
            if (
                abs(
                    measured["time"]
                    - command_record["wall_time"]
                    - lag
                )
                > 0.10
            ):
                continue
            vx_errors.append(abs(measured["vx"] - command[0]))
            yaw_errors.append(
                abs(measured["yaw_rate"] - command[2])
            )
        score = (
            (sum(vx_errors) / len(vx_errors))
            + (sum(yaw_errors) / len(yaw_errors))
            if vx_errors and yaw_errors
            else float("inf")
        )
        lag_trials.append(
            {
                "lag_s": lag,
                "score": score,
                "vx_errors": vx_errors,
                "yaw_errors": yaw_errors,
            }
        )
    best = min(lag_trials, key=lambda item: item["score"])
    return {
        "available": math.isfinite(best["score"]),
        "sport_records": len(sport),
        "command_records": len(commands),
        "best_response_lag_s": best["lag_s"],
        "combined_mae_score": (
            best["score"] if math.isfinite(best["score"]) else None
        ),
        "vx_command_vs_sport_error_mps": summary(
            best["vx_errors"]
        ),
        "yaw_command_vs_sport_error_radps": summary(
            best["yaw_errors"]
        ),
        "note": (
            "SportModeState is an onboard feedback source, not external "
            "metric ground truth."
        ),
    }


def vector_norm(values):
    try:
        return math.sqrt(sum(float(value) ** 2 for value in values))
    except (TypeError, ValueError):
        return None


def quaternion_from_orientation(orientation):
    try:
        quaternion = [
            float(orientation["qx"]),
            float(orientation["qy"]),
            float(orientation["qz"]),
            float(orientation["qw"]),
        ]
    except (KeyError, TypeError, ValueError):
        return None
    norm = math.sqrt(sum(value * value for value in quaternion))
    if norm < 1e-6 or not math.isfinite(norm):
        return None
    return [value / norm for value in quaternion]


def quaternion_conjugate(quaternion):
    return [
        -quaternion[0],
        -quaternion[1],
        -quaternion[2],
        quaternion[3],
    ]


def quaternion_multiply(left, right):
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return [
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    ]


def quaternion_euler(quaternion):
    x, y, z, w = quaternion
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (w * y - z * x)
    pitch = (
        math.copysign(math.pi / 2.0, sinp)
        if abs(sinp) >= 1.0
        else math.asin(sinp)
    )
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def sensor_body_alignment_audit(records):
    body_samples = []
    for record in records.get("sport", []):
        orientation = (
            record.get("data", {})
            .get("imu", {})
            .get("orientation", {})
        )
        quaternion = quaternion_from_orientation(orientation)
        if quaternion is not None:
            body_samples.append(
                {
                    "time": record["wall_time"],
                    "quaternion": quaternion,
                }
            )
    lidar_samples = []
    for record in records.get("odom", []):
        orientation = (
            record.get("data", {})
            .get("orientation", {})
        )
        quaternion = quaternion_from_orientation(orientation)
        if quaternion is not None:
            lidar_samples.append(
                {
                    "time": record["wall_time"],
                    "quaternion": quaternion,
                }
            )
    if not body_samples or not lidar_samples:
        return {
            "available": False,
            "reason": (
                "FAST-LIO odometry or body IMU quaternion is unavailable"
            ),
        }

    relative_euler = []
    pair_ages = []
    body_index = 0
    for lidar in lidar_samples:
        body, body_index = nearest_by_time(
            body_samples,
            lidar["time"],
            body_index,
        )
        if body is None:
            continue
        age = abs(body["time"] - lidar["time"])
        if age > 0.10:
            continue
        # Assuming both messages express local-frame orientation in a world
        # frame, q_body^-1 * q_lidar is the fixed sensor-to-body rotation.
        relative = quaternion_multiply(
            quaternion_conjugate(body["quaternion"]),
            lidar["quaternion"],
        )
        relative_euler.append(quaternion_euler(relative))
        pair_ages.append(age * 1000.0)
    if not relative_euler:
        return {
            "available": False,
            "reason": "no body/FAST-LIO orientation pair was within 100 ms",
        }

    reference = relative_euler[0]
    drift = [
        [
            math.degrees(
                normalize_angle(value[index] - reference[index])
            )
            for value in relative_euler
        ]
        for index in range(3)
    ]
    return {
        "available": True,
        "paired_samples": len(relative_euler),
        "pair_time_difference_ms": summary(pair_ages),
        "relative_roll_deg": summary(
            [math.degrees(value[0]) for value in relative_euler]
        ),
        "relative_pitch_deg": summary(
            [math.degrees(value[1]) for value in relative_euler]
        ),
        "relative_yaw_deg": summary(
            [math.degrees(value[2]) for value in relative_euler]
        ),
        "relative_rotation_change_from_first_deg": {
            "roll": summary(drift[0]),
            "pitch": summary(drift[1]),
            "yaw": summary(drift[2]),
        },
        "absolute_relative_rotation_change_from_first_deg": {
            "roll": summary([abs(value) for value in drift[0]]),
            "pitch": summary([abs(value) for value in drift[1]]),
            "yaw": summary([abs(value) for value in drift[2]]),
        },
        "interpretation": (
            "A stable relative rotation supports a rigid mount and "
            "consistent onboard attitude sources. Its absolute value is "
            "not external floor truth and may include differing yaw origins."
        ),
    }


def robot_state_audit(records):
    voltages = []
    currents = []
    electrical_power = []
    battery_soc = []
    motor_temperatures = []
    motor_torques = []
    motor_lost_nonzero = 0
    low_imu_temperature = []
    foot_force_spreads = []
    for record in records.get("low", []):
        data = record.get("data", {})
        try:
            voltage = float(data.get("power_v"))
            current = float(data.get("power_a"))
            voltages.append(voltage)
            currents.append(current)
            electrical_power.append(voltage * current)
        except (TypeError, ValueError):
            pass
        try:
            battery_soc.append(float(data["bms"]["soc"]))
        except (KeyError, TypeError, ValueError):
            pass
        try:
            low_imu_temperature.append(
                float(data["imu"]["temperature_c"])
            )
        except (KeyError, TypeError, ValueError):
            pass
        foot_forces = []
        for value in data.get("foot_force", []):
            try:
                foot_forces.append(float(value))
            except (TypeError, ValueError):
                pass
        if foot_forces:
            foot_force_spreads.append(
                max(foot_forces) - min(foot_forces)
            )
        for motor in data.get("motors", []):
            try:
                motor_temperatures.append(
                    float(motor["temperature_c"])
                )
                motor_torques.append(abs(float(motor["tau_est"])))
                if int(motor.get("lost", 0)) != 0:
                    motor_lost_nonzero += 1
            except (KeyError, TypeError, ValueError):
                continue

    sport_error_codes = []
    sport_modes = []
    sport_gaits = []
    sport_imu_temperature = []
    for record in records.get("sport", []):
        data = record.get("data", {})
        try:
            sport_error_codes.append(int(data.get("error_code", 0)))
            sport_modes.append(int(data.get("mode", 0)))
            sport_gaits.append(int(data.get("gait_type", 0)))
        except (TypeError, ValueError):
            pass
        try:
            sport_imu_temperature.append(
                float(data["imu"]["temperature_c"])
            )
        except (KeyError, TypeError, ValueError):
            pass

    acceleration_norms = []
    angular_velocity_norms = []
    for record in records.get("livox_imu", []):
        data = record.get("data", {})
        acceleration = data.get("linear_acceleration", {})
        angular = data.get("angular_velocity", {})
        acceleration_norm = vector_norm(
            [
                acceleration.get("x"),
                acceleration.get("y"),
                acceleration.get("z"),
            ]
        )
        angular_norm = vector_norm(
            [
                angular.get("x"),
                angular.get("y"),
                angular.get("z"),
            ]
        )
        if acceleration_norm is not None:
            acceleration_norms.append(acceleration_norm)
        if angular_norm is not None:
            angular_velocity_norms.append(angular_norm)
    return {
        "low_state_records": len(records.get("low", [])),
        "sport_state_records": len(records.get("sport", [])),
        "livox_imu_records": len(records.get("livox_imu", [])),
        "battery_voltage_v": summary(voltages),
        "battery_current_a": summary(currents),
        "battery_electrical_power_w": summary(electrical_power),
        "battery_soc_percent": summary(battery_soc),
        "motor_temperature_c": summary(motor_temperatures),
        "absolute_motor_torque_estimate": summary(motor_torques),
        "motor_lost_nonzero_records": motor_lost_nonzero,
        "low_imu_temperature_c": summary(low_imu_temperature),
        "sport_imu_temperature_c": summary(sport_imu_temperature),
        "foot_force_max_minus_min": summary(foot_force_spreads),
        "sport_nonzero_error_code_records": sum(
            1 for value in sport_error_codes if value != 0
        ),
        "sport_modes_seen": sorted(set(sport_modes)),
        "sport_gaits_seen": sorted(set(sport_gaits)),
        "livox_acceleration_norm_mps2": summary(
            acceleration_norms
        ),
        "livox_angular_velocity_norm_radps": summary(
            angular_velocity_norms
        ),
    }


def parse_marker(path, marker, start_time=None, end_time=None):
    fields = {}
    count = 0
    alert_count = 0
    try:
        handle = open(path, "r", errors="replace")
    except OSError:
        return {
            "count": 0,
            "alert_count": 0,
            "fields": {},
        }
    with handle:
        for line in handle:
            if marker not in line:
                continue
            time_match = ROS_TIME_RE.search(line)
            if time_match:
                line_time = float(time_match.group(1))
                if start_time is not None and line_time < start_time - 2.0:
                    continue
                if end_time is not None and line_time > end_time + 2.0:
                    continue
            count += 1
            if "ALERT" in line or "ERROR" in line:
                alert_count += 1
            for key, raw_value in KEY_VALUE_RE.findall(line):
                parts = []
                for token in raw_value.split("/"):
                    try:
                        parts.append(float(token))
                    except ValueError:
                        pass
                if not parts:
                    continue
                if len(parts) == 1:
                    fields.setdefault(key, []).append(parts[0])
                else:
                    labels = (
                        ("avg", "p95", "max")
                        if len(parts) == 3
                        else tuple(
                            "part%d" % index
                            for index in range(len(parts))
                        )
                    )
                    for label, value in zip(labels, parts):
                        fields.setdefault(
                            "%s_%s" % (key, label),
                            [],
                        ).append(value)
    return {
        "count": count,
        "alert_count": alert_count,
        "fields": {
            key: summary(values)
            for key, values in sorted(fields.items())
        },
    }


def log_timing_audit(run_dir, start_time, end_time):
    paths_and_markers = (
        (
            "follower",
            Path(run_dir) / "waypoint_follower.log",
            "TIMING_FOLLOWER",
        ),
        (
            "safety",
            Path(run_dir) / "unitree_safe_cmd_node.log",
            "TIMING_SAFE",
        ),
        (
            "udp_sender",
            Path(run_dir) / "cmd_vel_udp_sender.log",
            "TIMING_SENDER",
        ),
        (
            "udp_receiver",
            Path(run_dir) / "go2_sdk2_udp_receiver.log",
            "TIMING_RECEIVER",
        ),
        (
            "livox_stream",
            Path(run_dir) / "base_logs/livox.log",
            "LIVOX_STREAM_HEALTH",
        ),
        (
            "livox_publish",
            Path(run_dir) / "base_logs/livox.log",
            "LIVOX_PUBLISH_TIMING",
        ),
        (
            "fastlio_input",
            Path(run_dir) / "base_logs/fast_lio.log",
            "FAST_LIO_INPUT_TIMING",
        ),
        (
            "fastlio_output",
            Path(run_dir) / "base_logs/fast_lio.log",
            "FAST_LIO_OUTPUT_TIMING",
        ),
    )
    result = {}
    for label, path, marker in paths_and_markers:
        result[label] = parse_marker(
            path,
            marker,
            start_time,
            end_time,
        )
        result[label]["path"] = str(path)
        result[label]["marker"] = marker
    for label, filename, pattern in (
        (
            "safety_overrides",
            "unitree_safe_cmd_node.log",
            r"SAFE OVERRIDE",
        ),
        (
            "follower_stale_or_stop",
            "waypoint_follower.log",
            r"stale|timeout|stop",
        ),
        (
            "udp_alerts",
            "go2_sdk2_udp_receiver.log",
            r"TIMING_ALERT|invalid|gap",
        ),
    ):
        try:
            text = (Path(run_dir) / filename).read_text(
                errors="replace"
            )
        except OSError:
            text = ""
        result[label] = {
            "matching_lines": len(
                re.findall(pattern, text, flags=re.IGNORECASE)
            )
        }
    return result


def rosbag_audit(run_dir):
    info_path = Path(run_dir) / "rosbag_info.txt"
    try:
        text = info_path.read_text(errors="replace")
    except OSError:
        text = ""
    topics = {}
    for match in re.finditer(
        r"Topic:\s*(/[^\s|]+).*?Count:\s*(\d+)",
        text,
        flags=re.IGNORECASE,
    ):
        topics[match.group(1)] = int(match.group(2))
    critical_topics = (
        "/Odometry",
        "/livox/imu",
        "/lf/sportmodestate",
        "/patrol_cmd",
        "/cmd_vel",
        "/api/sport/request",
    )
    missing_or_empty = [
        topic
        for topic in critical_topics
        if topics.get(topic, 0) <= 0
    ]
    database_files = []
    for path in sorted((Path(run_dir) / "rosbag").glob("*.db3")):
        try:
            database_files.append(
                {
                    "name": path.name,
                    "size_bytes": path.stat().st_size,
                }
            )
        except OSError:
            continue
    return {
        "available": bool(database_files),
        "info_available": bool(text),
        "database_files": database_files,
        "topic_message_counts": topics,
        "critical_topics_missing_or_empty": missing_or_empty,
        "critical_topics_complete": (
            bool(database_files)
            and bool(text)
            and not missing_or_empty
        ),
    }


def performance_audit(path):
    samples = []
    try:
        handle = open(path, "r")
    except OSError:
        return {"samples": 0}
    with handle:
        for line in handle:
            if "PERF_SAMPLE " not in line:
                continue
            try:
                samples.append(
                    json.loads(line.split("PERF_SAMPLE ", 1)[1])
                )
            except (ValueError, TypeError):
                continue
    per_cpu = {}
    process_cpu = {}
    max_temperature = []
    wake_late = []
    pressure_cpu = []
    network_drops = {}
    udp_deltas = {}
    hwmon = {}
    scheduling = {}
    for item in samples:
        wake_late.append(item.get("monitor_wake_late_ms"))
        max_temperature.append(item.get("max_temperature_c"))
        pressure_cpu.append(
            item.get("pressure_cpu", {}).get("some_avg10")
        )
        for key, value in item.get("per_cpu_pct", {}).items():
            per_cpu.setdefault(key, []).append(value)
        for key, value in item.get("process_cpu_pct", {}).items():
            process_cpu.setdefault(key, []).append(value)
        for interface, counters in item.get("network", {}).items():
            drops = (
                counters.get("rx_dropped_delta", 0)
                + counters.get("tx_dropped_delta", 0)
                + counters.get("rx_errors_delta", 0)
                + counters.get("tx_errors_delta", 0)
            )
            network_drops[interface] = (
                network_drops.get(interface, 0) + drops
            )
        for key, value in item.get("udp_deltas", {}).items():
            udp_deltas[key] = udp_deltas.get(key, 0) + value
        for device, sensors in item.get("hwmon", {}).items():
            for key, value in sensors.items():
                hwmon.setdefault(
                    "%s/%s" % (device, key),
                    [],
                ).append(value)
        for process, detail in item.get(
            "process_scheduling",
            {},
        ).items():
            target = scheduling.setdefault(
                process,
                {
                    "cpu_allowed_lists": set(),
                    "observed_last_cpus": set(),
                    "max_threads": 0,
                    "max_threads_observed_per_cpu": {},
                },
            )
            target["cpu_allowed_lists"].update(
                detail.get("cpu_allowed_lists", [])
            )
            target["observed_last_cpus"].update(
                detail.get("last_cpus", [])
            )
            target["max_threads"] = max(
                target["max_threads"],
                int(detail.get("threads", 0)),
            )
            for cpu_id, count in detail.get(
                "thread_last_cpu_counts",
                {},
            ).items():
                target["max_threads_observed_per_cpu"][cpu_id] = max(
                    target["max_threads_observed_per_cpu"].get(
                        cpu_id,
                        0,
                    ),
                    int(count),
                )
    return {
        "samples": len(samples),
        "monitor_wake_late_ms": summary(wake_late),
        "max_temperature_c": summary(max_temperature),
        "cpu_pressure_some_avg10": summary(pressure_cpu),
        "per_cpu_pct": {
            key: summary(values)
            for key, values in sorted(per_cpu.items())
        },
        "process_cpu_pct": {
            key: summary(values)
            for key, values in sorted(process_cpu.items())
        },
        "network_error_drop_totals": network_drops,
        "udp_counter_deltas_total": udp_deltas,
        "hwmon": {
            key: summary(values)
            for key, values in sorted(hwmon.items())
        },
        "process_scheduling": {
            key: {
                "cpu_allowed_lists": sorted(
                    value["cpu_allowed_lists"]
                ),
                "observed_last_cpus": sorted(
                    value["observed_last_cpus"]
                ),
                "max_threads": value["max_threads"],
                "max_threads_observed_per_cpu": dict(
                    sorted(
                        value[
                            "max_threads_observed_per_cpu"
                        ].items()
                    )
                ),
            }
            for key, value in sorted(scheduling.items())
        },
    }


def command_stdout(snapshot, label):
    return (
        snapshot.get("commands", {})
        .get(label, {})
        .get("stdout", "")
    )


def configuration_text(snapshot, suffix):
    configurations = snapshot.get("configuration_files", {})
    matches = [
        (path, item.get("text", ""))
        for path, item in configurations.items()
        if path.endswith(suffix) and isinstance(item, dict)
    ]
    if not matches:
        return "", ""
    # The installed share file is what ros2 launch resolves at runtime.
    matches.sort(
        key=lambda item: (
            0 if item[0].startswith("install/") else 1,
            item[0],
        )
    )
    return matches[0]


def parameter_token(text, key):
    match = re.search(
        r"(?m)^\s*(?:[A-Za-z0-9_.]+\.)?"
        + re.escape(key)
        + r"\s*:\s*(\[[^\]]*\]|[^\n#]+)",
        text or "",
    )
    return match.group(1).strip() if match else ""


def parameter_float(text, key):
    token = parameter_token(text, key)
    try:
        return float(token)
    except (TypeError, ValueError):
        return None


def parameter_int(text, key):
    value = parameter_float(text, key)
    return int(value) if value is not None else None


def parameter_bool(text, key):
    token = parameter_token(text, key).strip().lower()
    if token in ("true", "1", "yes", "on"):
        return True
    if token in ("false", "0", "no", "off"):
        return False
    return None


def parameter_array(text, key):
    token = parameter_token(text, key)
    if not token:
        return []
    return [
        float(value)
        for value in re.findall(NUMBER_RE, token)
    ]


def localization_configuration_audit(snapshot):
    runtime_fastlio = command_stdout(
        snapshot,
        "ros_params_fastlio",
    )
    config_path, config_text = configuration_text(
        snapshot,
        "go2_mid360s.yaml",
    )
    fastlio_text = runtime_fastlio or config_text
    rotation = parameter_array(fastlio_text, "extrinsic_R")
    translation = parameter_array(fastlio_text, "extrinsic_T")
    identity_error = None
    if len(rotation) == 9:
        identity = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        identity_error = max(
            abs(value - expected)
            for value, expected in zip(rotation, identity)
        )

    livox_path, livox_text = configuration_text(
        snapshot,
        "MID360s_config.json",
    )
    livox_extrinsic = {}
    try:
        livox_json = json.loads(livox_text)
        lidar_configs = livox_json.get("lidar_configs", [])
        if lidar_configs:
            livox_extrinsic = lidar_configs[0].get(
                "extrinsic_parameter",
                {},
            )
    except (TypeError, ValueError):
        pass
    runtime_livox = command_stdout(
        snapshot,
        "ros_params_livox",
    )
    return {
        "fastlio_parameter_source": (
            "runtime_ros_parameter_dump"
            if runtime_fastlio
            else config_path or "unavailable"
        ),
        "fastlio_config_path": config_path,
        "extrinsic_estimation_enabled": parameter_bool(
            fastlio_text,
            "extrinsic_est_en",
        ),
        "lidar_to_imu_translation_m": translation,
        "lidar_to_imu_rotation_matrix": rotation,
        "lidar_to_imu_rotation_identity_max_error": identity_error,
        "lidar_qos_depth": parameter_int(
            fastlio_text,
            "lidar_qos_depth",
        ),
        "imu_qos_depth": parameter_int(
            fastlio_text,
            "imu_qos_depth",
        ),
        "time_sync_enabled": parameter_bool(
            fastlio_text,
            "time_sync_en",
        ),
        "lidar_to_imu_time_offset_s": parameter_float(
            fastlio_text,
            "time_offset_lidar_to_imu",
        ),
        "scan_rate_hz": parameter_float(
            fastlio_text,
            "scan_rate",
        ),
        "livox_parameter_dump_available": bool(runtime_livox),
        "livox_config_path": livox_path,
        "livox_configured_extrinsic": livox_extrinsic,
        "mounting_note": (
            "These values describe the LiDAR-to-LiDAR-IMU/configured "
            "transform. They do not independently measure the physical "
            "LiDAR-to-dog-body mounting angle."
        ),
    }


def system_configuration_audit(run_dir):
    start = load_json(Path(run_dir) / "system_start.json", {})
    end = load_json(Path(run_dir) / "system_end.json", {})
    start_files = {
        item.get("relative_path", item.get("path", "")): item.get(
            "sha256",
            "",
        )
        for item in start.get("files", [])
        if isinstance(item, dict)
    }
    end_files = {
        item.get("relative_path", item.get("path", "")): item.get(
            "sha256",
            "",
        )
        for item in end.get("files", [])
        if isinstance(item, dict)
    }
    changed_files = [
        key
        for key in sorted(set(start_files).intersection(end_files))
        if start_files[key] != end_files[key]
    ]
    start_cpu = start.get("cpu", {})
    process_affinity = {}
    for process in start.get("processes", []):
        command = process.get("command", "")
        if "fastlio_mapping" in command:
            label = "fastlio"
        elif "livox_ros_driver" in command:
            label = "livox"
        elif "waypoint_follower" in command:
            label = "follower"
        elif "unitree_safe_cmd_node" in command:
            label = "safety"
        else:
            continue
        process_affinity.setdefault(label, []).append(
            {
                "pid": process.get("pid"),
                "cpu_allowed_list": process.get(
                    "cpu_allowed_list",
                    "",
                ),
                "threads": process.get("threads"),
                "scheduler": process.get("scheduler"),
                "nice": process.get("nice"),
            }
        )
    nvpmodel = (
        start.get("commands", {})
        .get("nvpmodel", {})
        .get("stdout", "")
        .strip()
    )
    return {
        "start_snapshot_available": bool(start),
        "end_snapshot_available": bool(end),
        "boot_id_start": start.get("boot_id"),
        "boot_id_end": end.get("boot_id"),
        "same_boot": (
            start.get("boot_id") == end.get("boot_id")
            if start and end
            else None
        ),
        "nvpmodel": nvpmodel
        or start.get("nvpmodel_status_file", ""),
        "online_cpu_range": start_cpu.get("online", ""),
        "cpu_count_with_details": len(start_cpu.get("by_cpu", {})),
        "process_affinity_at_start": process_affinity,
        "localization_configuration": (
            localization_configuration_audit(start)
        ),
        "source_or_executable_files_changed_during_run": changed_files,
        "source_file_count_start": len(start_files),
        "source_file_count_end": len(end_files),
    }


def evidence_health(records, malformed, run_dir=None):
    health = records.get("recorder_health", [])
    last = health[-1].get("data", {}) if health else {}
    ages = last.get("receive_age_s", {})
    required = (
        "odom",
        "sport",
        "low",
        "patrol_cmd",
        "cmd_vel",
    )
    missing = [
        kind
        for kind in required
        if not records.get(kind)
    ]
    missing_artifacts = []
    if run_dir is not None:
        root = Path(run_dir)
        required_artifacts = (
            "route_original.csv",
            "route_runtime.csv",
            "manual_anchor.json",
            "experiment_telemetry.jsonl",
            "follower_control_trace.jsonl",
            "performance_monitor.log",
            "system_start.json",
            "system_end.json",
            "localization_session_end.json",
            "waypoint_follower.log",
            "unitree_safe_cmd_node.log",
            "cmd_vel_udp_sender.log",
            "go2_sdk2_udp_receiver.log",
            "localization_session_guard.log",
            "base_logs/livox.log",
            "base_logs/fast_lio.log",
        )
        missing_artifacts.extend(
            relative
            for relative in required_artifacts
            if not (root / relative).is_file()
        )
        if not any((root / "rosbag").glob("*.db3")):
            missing_artifacts.append("rosbag/*.db3")
    return {
        "malformed_jsonl_lines": malformed,
        "record_counts": {
            kind: len(values)
            for kind, values in sorted(records.items())
        },
        "last_receive_age_s": ages,
        "missing_required_streams": missing,
        "complete_for_onboard_chain": not missing,
        "missing_required_artifacts": missing_artifacts,
        "complete_for_full_audit": (
            not missing and not missing_artifacts
        ),
        "raw_pointcloud_deliberately_not_subscribed": True,
    }


def session_identity_equal(left, right):
    if not isinstance(left, dict) or not isinstance(right, dict):
        return None
    required = ("boot_id", "pid", "start_ticks")
    if not all(key in left and key in right for key in required):
        return None
    return all(left.get(key) == right.get(key) for key in required)


def localization_session_audit(
    run_dir,
    anchor_metadata,
    recording_link,
):
    patrol_start = (anchor_metadata or {}).get(
        "localization_session",
        {},
    )
    patrol_end_metadata = load_json(
        Path(run_dir) / "localization_session_end.json",
        {},
    )
    patrol_end = patrol_end_metadata.get(
        "localization_session",
        {},
    )
    recording_start = recording_link.get(
        "localization_session_start",
        {},
    )
    recording_end = recording_link.get(
        "localization_session_end",
        {},
    )
    try:
        guard_text = (
            Path(run_dir) / "localization_session_guard.log"
        ).read_text(errors="replace")
    except OSError:
        guard_text = ""
    return {
        "recording_start_to_end_same": session_identity_equal(
            recording_start,
            recording_end,
        ),
        "recording_end_to_patrol_start_same": session_identity_equal(
            recording_end,
            patrol_start,
        ),
        "patrol_start_to_end_same": session_identity_equal(
            patrol_start,
            patrol_end,
        ),
        "recording_start": recording_start,
        "recording_end": recording_end,
        "patrol_start": patrol_start,
        "patrol_end": patrol_end,
        "session_guard_abort_detected": (
            "LOCALIZATION_SESSION_ABORTED" in guard_text
        ),
        "session_guard_started": (
            "SESSION_GUARD_STARTED" in guard_text
        ),
    }


def classify(report):
    findings = []
    transform = report["coordinate_transform"]
    tracking = report["route_tracking"]
    control_trace = report["follower_control_trace"]
    evidence = report["evidence_health"]
    recording = report["route_recording_link"]
    sessions = report["localization_sessions"]
    system = report["system_configuration"]
    robot = report["robot_state"]
    rosbag = report["rosbag"]
    alignment = report["sensor_body_alignment"]

    if not evidence["complete_for_onboard_chain"]:
        findings.append(
            {
                "severity": "error",
                "stage": "evidence",
                "conclusion": "onboard evidence is incomplete",
                "detail": ",".join(
                    evidence["missing_required_streams"]
                ),
            }
        )
    if evidence.get("missing_required_artifacts"):
        findings.append(
            {
                "severity": "error",
                "stage": "evidence_artifacts",
                "conclusion": (
                    "full experiment artifacts are incomplete"
                ),
                "detail": ",".join(
                    evidence["missing_required_artifacts"]
                ),
            }
        )
    if not rosbag.get("critical_topics_complete"):
        findings.append(
            {
                "severity": "error",
                "stage": "rosbag",
                "conclusion": (
                    "the canonical full-rate bag is missing or has an "
                    "empty critical topic"
                ),
                "missing_topics": rosbag.get(
                    "critical_topics_missing_or_empty",
                    [],
                ),
            }
        )
    if transform.get("available") and not transform.get(
        "exact_within_csv_precision"
    ):
        findings.append(
            {
                "severity": "error",
                "stage": "coordinate_transform",
                "conclusion": (
                    "runtime CSV is not the exact rigid transform "
                    "described by manual_anchor.json"
                ),
            }
        )
    elif transform.get("exact_within_csv_precision"):
        findings.append(
            {
                "severity": "ok",
                "stage": "coordinate_transform",
                "conclusion": (
                    "original-to-runtime CSV conversion is exactly "
                    "reproducible within CSV precision"
                ),
            }
        )

    if tracking.get("available"):
        p95 = tracking["cross_track_error_m"].get("p95")
        maximum = tracking["cross_track_error_m"].get("max")
        if p95 is not None and p95 > 0.30:
            severity = "error"
            conclusion = (
                "FAST-LIO trajectory itself deviated materially from "
                "the runtime CSV"
            )
        elif maximum is not None and maximum > 0.30:
            severity = "warning"
            conclusion = (
                "FAST-LIO trajectory had localized large deviation "
                "from the runtime CSV"
            )
        else:
            severity = "ok"
            conclusion = (
                "FAST-LIO trajectory stayed close to the runtime CSV"
            )
        findings.append(
            {
                "severity": severity,
                "stage": "route_tracking",
                "conclusion": conclusion,
                "p95_cross_track_m": p95,
                "max_cross_track_m": maximum,
            }
        )

    if control_trace.get("available"):
        age_p95 = control_trace.get(
            "odom_source_stamp_age_ms",
            {},
        ).get("p95")
        reuse_max = control_trace.get(
            "consecutive_control_cycles_per_odom",
            {},
        ).get("max")
        command_error = control_trace.get(
            "trace_to_patrol_cmd_absolute_error",
            {},
        ).get("yaw_rate", {}).get("max")
        if age_p95 is not None and age_p95 > 500.0:
            trace_severity = "error"
            trace_conclusion = (
                "the follower made control decisions from materially "
                "stale odometry"
            )
        elif (
            (age_p95 is not None and age_p95 > 250.0)
            or (reuse_max is not None and reuse_max > 8)
        ):
            trace_severity = "warning"
            trace_conclusion = (
                "the follower had elevated odometry age or prolonged "
                "reuse of one odometry update"
            )
        elif command_error is not None and command_error > 0.001:
            trace_severity = "error"
            trace_conclusion = (
                "the command recorded at the follower decision does "
                "not match the published patrol command"
            )
        else:
            trace_severity = "ok"
            trace_conclusion = (
                "each follower decision is linked to its exact odometry "
                "callback and published command"
            )
        findings.append(
            {
                "severity": trace_severity,
                "stage": "follower_frame_usage",
                "conclusion": trace_conclusion,
                "odom_stamp_age_p95_ms": age_p95,
                "max_control_cycles_per_odom": reuse_max,
                "trace_to_command_yaw_error_max": command_error,
            }
        )

    if recording.get("available"):
        findings.append(
            {
                "severity": "ok",
                "stage": "recording_link",
                "conclusion": (
                    "executed CSV hash matches its route-recording "
                    "blackbox sidecar"
                ),
            }
        )
    else:
        findings.append(
            {
                "severity": "warning",
                "stage": "recording_link",
                "conclusion": (
                    "this patrol cannot be cryptographically linked "
                    "to a route-recording blackbox"
                ),
            }
        )

    if (
        sessions.get("patrol_start_to_end_same") is False
        or sessions.get("session_guard_abort_detected")
    ):
        findings.append(
            {
                "severity": "error",
                "stage": "localization_session",
                "conclusion": (
                    "FAST-LIO session changed or the session guard "
                    "aborted during patrol"
                ),
            }
        )
    elif sessions.get("patrol_start_to_end_same") is True:
        findings.append(
            {
                "severity": "ok",
                "stage": "localization_session",
                "conclusion": (
                    "FAST-LIO process identity stayed constant for "
                    "the complete patrol"
                ),
            }
        )
    if sessions.get("recording_end_to_patrol_start_same") is False:
        findings.append(
            {
                "severity": "warning",
                "stage": "recording_to_patrol_session",
                "conclusion": (
                    "FAST-LIO was restarted between recording and "
                    "patrol; manual anchoring still transforms the "
                    "route, but this run cannot isolate session drift "
                    "as cleanly"
                ),
            }
        )

    if system.get("source_or_executable_files_changed_during_run"):
        findings.append(
            {
                "severity": "error",
                "stage": "code_identity",
                "conclusion": (
                    "source or executable hashes changed during the run"
                ),
                "files": system[
                    "source_or_executable_files_changed_during_run"
                ],
            }
        )
    elif (
        system.get("start_snapshot_available")
        and system.get("end_snapshot_available")
    ):
        findings.append(
            {
                "severity": "ok",
                "stage": "code_identity",
                "conclusion": (
                    "captured source/executable identities stayed "
                    "unchanged during the run"
                ),
            }
        )

    if (
        robot.get("motor_lost_nonzero_records", 0) > 0
        or robot.get("sport_nonzero_error_code_records", 0) > 0
    ):
        findings.append(
            {
                "severity": "error",
                "stage": "robot_low_level_state",
                "conclusion": (
                    "Unitree state reported a motor-lost count or a "
                    "nonzero sport error code"
                ),
                "motor_lost_nonzero_records": robot.get(
                    "motor_lost_nonzero_records"
                ),
                "sport_nonzero_error_code_records": robot.get(
                    "sport_nonzero_error_code_records"
                ),
            }
        )

    if alignment.get("available"):
        absolute_change = alignment.get(
            "absolute_relative_rotation_change_from_first_deg",
            {},
        )
        p95_changes = [
            absolute_change.get(axis, {}).get("p95")
            for axis in ("roll", "pitch", "yaw")
        ]
        maximum_changes = [
            absolute_change.get(axis, {}).get("max")
            for axis in ("roll", "pitch", "yaw")
        ]
        largest_p95 = max(
            (
                value
                for value in p95_changes
                if value is not None
            ),
            default=None,
        )
        largest_maximum = max(
            (
                value
                for value in maximum_changes
                if value is not None
            ),
            default=None,
        )
        if largest_p95 is not None and largest_p95 > 8.0:
            alignment_severity = "error"
            alignment_conclusion = (
                "FAST-LIO and body IMU relative orientation changed "
                "materially during the run"
            )
        elif (
            (largest_p95 is not None and largest_p95 > 3.0)
            or (largest_maximum is not None and largest_maximum > 8.0)
        ):
            alignment_severity = "warning"
            alignment_conclusion = (
                "FAST-LIO and body IMU relative orientation had "
                "elevated variation during the run"
            )
        else:
            alignment_severity = "ok"
            alignment_conclusion = (
                "FAST-LIO and body IMU relative orientation stayed "
                "stable during the run"
            )
        findings.append(
            {
                "severity": alignment_severity,
                "stage": "sensor_body_alignment",
                "conclusion": alignment_conclusion,
                "paired_samples": alignment.get("paired_samples"),
                "largest_p95_absolute_change_deg": largest_p95,
                "largest_max_absolute_change_deg": largest_maximum,
            }
        )
    else:
        findings.append(
            {
                "severity": "warning",
                "stage": "sensor_body_alignment",
                "conclusion": (
                    "could not compare FAST-LIO orientation with the "
                    "dog body IMU"
                ),
                "detail": alignment.get("reason"),
            }
        )

    findings.append(
        {
            "severity": "needs_external_measurement",
            "stage": "physical_ground_truth",
            "conclusion": (
                "onboard logs cannot prove whether FAST-LIO's reported "
                "XY equals the dog's true floor position; use the fixed "
                "camera/floor-marker record for that final distinction"
            ),
        }
    )
    return findings


def format_value(value, digits=3):
    if value is None:
        return "无"
    if isinstance(value, float):
        return ("%.*f" % (digits, value)).rstrip("0").rstrip(".")
    return str(value)


def build_markdown(report):
    tracking = report["route_tracking"]
    transform = report["coordinate_transform"]
    control_trace = report["follower_control_trace"]
    command = report["command_chain"]
    rosbag = report["rosbag"]
    performance = report["performance"]
    system = report["system_configuration"]
    localization_config = system.get(
        "localization_configuration",
        {},
    )
    fastlio_scheduling = performance.get(
        "process_scheduling",
        {},
    ).get("fastlio", {})
    fastlio_cpu = performance.get(
        "process_cpu_pct",
        {},
    ).get("fastlio", {})
    robot = report["robot_state"]
    alignment = report["sensor_body_alignment"]
    alignment_change = alignment.get(
        "absolute_relative_rotation_change_from_first_deg",
        {},
    )
    lines = [
        "# Go2 巡检全链路自动体检",
        "",
        "- 生成时间：%s" % report["created_at"],
        "- 运行目录：`%s`" % report["run_dir"],
        "- 证据流是否完整：%s"
        % (
            "是"
            if report["evidence_health"][
                "complete_for_onboard_chain"
            ]
            else "否"
        ),
        "- 完整体检材料是否齐全：%s"
        % (
            "是"
            if report["evidence_health"].get(
                "complete_for_full_audit"
            )
            else "否"
        ),
        "",
        "## 关键结论",
        "",
    ]
    for finding in report["findings"]:
        lines.append(
            "- [%s] %s：%s"
            % (
                finding["severity"],
                finding["stage"],
                finding["conclusion"],
            )
        )
    lines.extend(
        [
            "",
            "## 核心数值",
            "",
            "- 坐标转换可精确复算：%s"
            % (
                "是"
                if transform.get("exact_within_csv_precision")
                else "否/无证据"
            ),
            "- 起步位置误差：%s m；起步朝向误差：%s°"
            % (
                format_value(
                    tracking.get("initial_position_error_m")
                ),
                format_value(
                    tracking.get("initial_yaw_error_deg")
                ),
            ),
            "- 全程横向误差：中位 %s m，P95 %s m，最大 %s m"
            % (
                format_value(
                    tracking.get("cross_track_error_m", {}).get(
                        "median"
                    )
                ),
                format_value(
                    tracking.get("cross_track_error_m", {}).get(
                        "p95"
                    )
                ),
                format_value(
                    tracking.get("cross_track_error_m", {}).get(
                        "max"
                    )
                ),
            ),
            "- 前 5m 横向误差：P95 %s m，最大 %s m"
            % (
                format_value(
                    tracking.get(
                        "first_5m_cross_track_error_m",
                        {},
                    ).get("p95")
                ),
                format_value(
                    tracking.get(
                        "first_5m_cross_track_error_m",
                        {},
                    ).get("max")
                ),
            ),
            "- 跟线器逐周期证据：%s 条控制决策、使用 %s 个里程计回调；同一里程计连续复用最多 %s 个控制周期；里程计时间戳年龄 P95 %s ms"
            % (
                control_trace.get("control_records", 0),
                control_trace.get(
                    "unique_odom_callback_sequences_used",
                    0,
                ),
                format_value(
                    control_trace.get(
                        "consecutive_control_cycles_per_odom",
                        {},
                    ).get("max")
                ),
                format_value(
                    control_trace.get(
                        "odom_source_stamp_age_ms",
                        {},
                    ).get("p95"),
                    1,
                ),
            ),
            "- 全速 rosbag 关键话题：%s；各话题消息数=%s"
            % (
                (
                    "齐全"
                    if rosbag.get("critical_topics_complete")
                    else "缺失/为空"
                ),
                json.dumps(
                    rosbag.get("topic_message_counts", {}),
                    ensure_ascii=False,
                    sort_keys=True,
                ),
            ),
            "- 安全层把非零命令改成零：%s 次（配对命令 %s 条）"
            % (
                command.get("nonzero_to_zero_override_records", 0),
                command.get("paired_records", 0),
            ),
            "- 性能采样：%s 条；最高温度 %s°C；监控唤醒延迟 P95 %s ms"
            % (
                performance.get("samples", 0),
                format_value(
                    performance.get("max_temperature_c", {}).get(
                        "max"
                    ),
                    1,
                ),
                format_value(
                    performance.get(
                        "monitor_wake_late_ms",
                        {},
                    ).get("p95"),
                    1,
                ),
            ),
            "- Orin 功耗模式：`%s`；在线 CPU：`%s`；识别到 %s 个 CPU 核配置"
            % (
                (system.get("nvpmodel") or "未读到").replace(
                    "\n",
                    " / ",
                ),
                system.get("online_cpu_range") or "未知",
                system.get("cpu_count_with_details", 0),
            ),
            "- FAST-LIO 多核证据：进程 CPU 峰值 %s%%，线程数最多 %s，线程实际落到的 CPU=%s，允许 CPU=%s"
            % (
                format_value(fastlio_cpu.get("max"), 1),
                format_value(
                    fastlio_scheduling.get("max_threads")
                ),
                json.dumps(
                    fastlio_scheduling.get(
                        "observed_last_cpus",
                        [],
                    )
                ),
                json.dumps(
                    fastlio_scheduling.get(
                        "cpu_allowed_lists",
                        [],
                    )
                ),
            ),
            "- FAST-LIO 实际配置：外参在线估计=%s，LiDAR→IMU 平移=%s，旋转矩阵=%s，LiDAR/IMU 队列深度=%s/%s"
            % (
                format_value(
                    localization_config.get(
                        "extrinsic_estimation_enabled"
                    )
                ),
                json.dumps(
                    localization_config.get(
                        "lidar_to_imu_translation_m",
                        [],
                    )
                ),
                json.dumps(
                    localization_config.get(
                        "lidar_to_imu_rotation_matrix",
                        [],
                    )
                ),
                format_value(
                    localization_config.get("lidar_qos_depth")
                ),
                format_value(
                    localization_config.get("imu_qos_depth")
                ),
            ),
            "- Livox 配置外参：`%s`（注意：这是雷达/雷达内置 IMU 的配置，不能替代雷达相对狗身安装角的实测）"
            % json.dumps(
                localization_config.get(
                    "livox_configured_extrinsic",
                    {},
                ),
                ensure_ascii=False,
                sort_keys=True,
            ),
            "- FAST-LIO 与狗身 IMU 相对姿态：%s；相对 R/P/Y 中位数=%s°/%s°/%s°；相对姿态变化 P95=%s°/%s°/%s°"
            % (
                (
                    "可比较"
                    if alignment.get("available")
                    else "无有效配对"
                ),
                format_value(
                    alignment.get("relative_roll_deg", {}).get(
                        "median"
                    ),
                    2,
                ),
                format_value(
                    alignment.get("relative_pitch_deg", {}).get(
                        "median"
                    ),
                    2,
                ),
                format_value(
                    alignment.get("relative_yaw_deg", {}).get(
                        "median"
                    ),
                    2,
                ),
                format_value(
                    alignment_change.get("roll", {}).get("p95"),
                    2,
                ),
                format_value(
                    alignment_change.get("pitch", {}).get("p95"),
                    2,
                ),
                format_value(
                    alignment_change.get("yaw", {}).get("p95"),
                    2,
                ),
            ),
            "- 狗底层：电机最高温度 %s°C；电池功率 P95 %sW；Sport 非零错误 %s 条；motor lost %s 条"
            % (
                format_value(
                    robot.get("motor_temperature_c", {}).get("max"),
                    1,
                ),
                format_value(
                    robot.get(
                        "battery_electrical_power_w",
                        {},
                    ).get("p95"),
                    1,
                ),
                robot.get("sport_nonzero_error_code_records", 0),
                robot.get("motor_lost_nonzero_records", 0),
            ),
            "",
            "## 解释边界",
            "",
            "横向误差使用 FAST-LIO 自己报告的轨迹与运行时 CSV 比较。"
            "如果这个误差很小、但现场视频里狗明显偏离地面路线，"
            "问题就在定位真值而不是跟线器；这一步必须用固定机位视频或"
            "地面标记作为外部真值，任何纯车载日志都无法独立证明。",
            "",
        ]
    )
    return "\n".join(lines)


def build_report(run_dir):
    run_dir = Path(run_dir).resolve()
    original = load_route(run_dir / "route_original.csv")
    runtime = load_route(run_dir / "route_runtime.csv")
    anchor = load_json(run_dir / "manual_anchor.json", {})
    recording_link = load_json(
        run_dir / "route_recording.json",
        {
            "available": False,
            "reason": "route_recording.json not present",
        },
    )
    if isinstance(recording_link, dict) and "available" not in recording_link:
        recording_link = dict(recording_link)
        recording_link["available"] = True
    records, malformed = load_telemetry(
        run_dir / "experiment_telemetry.jsonl"
    )
    window = motion_window(records)
    odom = extract_odom(records, window)
    all_times = [
        record["wall_time"]
        for values in records.values()
        for record in values
    ]
    start_time = min(all_times) if all_times else None
    end_time = max(all_times) if all_times else None
    report = {
        "schema": "go2.patrol_experiment_audit.v1",
        "created_at": iso_time(),
        "run_dir": str(run_dir),
        "motion_window_epoch": list(window) if window else None,
        "evidence_health": evidence_health(
            records,
            malformed,
            run_dir,
        ),
        "route_recording_link": recording_link,
        "coordinate_transform": verify_manual_transform(
            original,
            runtime,
            anchor,
        ),
        "localization_sessions": localization_session_audit(
            run_dir,
            anchor,
            recording_link,
        ),
        "route_tracking": tracking_audit(runtime, odom),
        "follower_control_trace": control_trace_audit(
            run_dir / "follower_control_trace.jsonl",
            records,
        ),
        "command_chain": command_chain_audit(records),
        "measured_response": measured_response_audit(records),
        "robot_state": robot_state_audit(records),
        "sensor_body_alignment": sensor_body_alignment_audit(
            records
        ),
        "rosbag": rosbag_audit(run_dir),
        "timing_logs": log_timing_audit(
            run_dir,
            start_time,
            end_time,
        ),
        "performance": performance_audit(
            run_dir / "performance_monitor.log"
        ),
        "system_configuration": system_configuration_audit(run_dir),
        "external_ground_truth": {
            "available": False,
            "required_artifact": (
                "fixed-camera video with visible floor reference marks"
            ),
            "reason": (
                "All onboard position channels can share localization "
                "or inertial biases."
            ),
        },
    }
    report["findings"] = classify(report)
    return report


def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-dir", required=True)
    parser.add_argument("--output", default="")
    parser.add_argument("--markdown-output", default="")
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)
    run_dir = Path(args.run_dir).resolve()
    output = (
        Path(args.output)
        if args.output
        else run_dir / "experiment_audit.json"
    )
    markdown_output = (
        Path(args.markdown_output)
        if args.markdown_output
        else run_dir / "experiment_audit.md"
    )
    report = build_report(run_dir)
    atomic_write_json(output, report)
    atomic_write(markdown_output, build_markdown(report))
    print(
        "EXPERIMENT_AUDIT_READY output=%s markdown=%s findings=%d"
        % (output, markdown_output, len(report["findings"])),
        flush=True,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

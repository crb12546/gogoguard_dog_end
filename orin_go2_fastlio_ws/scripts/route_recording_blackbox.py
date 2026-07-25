#!/usr/bin/env python3
"""Manage a complete, low-observer-effect route-recording experiment.

One invocation starts the recorder plus its evidence collectors and exits.
Another invocation stops the exact saved process groups and finalizes the
evidence.  The state file makes this safe across SSH connections.
"""

from __future__ import print_function

import argparse
import csv
import datetime
import hashlib
import json
import math
import os
import re
import shutil
import signal
import subprocess
import tempfile
import time
from pathlib import Path


DEFAULT_WS = os.environ.get(
    "GO2_WS",
    "/home/unitree/go2_fastlio_ws",
)
DEFAULT_STATE_FILE = "/tmp/go2_route_recording_blackbox.json"
DEFAULT_ACTIVE_FILE = "/tmp/go2_console_recorder.active"
DEFAULT_CONSOLE_LOG = "/tmp/console_recorder.log"
LIGHTWEIGHT_BAG_TOPICS = (
    "/Odometry",
    "/livox/imu",
    "/lf/wirelesscontroller",
    "/wirelesscontroller",
    "/tf",
    "/tf_static",
)
CONFLICT_PATTERN = re.compile(
    r"route_recorder|go2map_capture|waypoint_follower|"
    r"unitree_safe_cmd_node|cmd_vel_udp_sender|"
    r"go2_sdk2_udp_receiver|go2_saas_agent.py video-loop|"
    r"go2_experiment_telemetry|patrol_performance_monitor|"
    r"ros2 bag record|localization_session_guard.py.*--mode recorder"
)


def iso_time():
    return datetime.datetime.now().astimezone().isoformat(
        timespec="milliseconds"
    )


def atomic_write_json(path, payload):
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
            json.dump(payload, handle, indent=2, sort_keys=True)
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, str(output))
    except Exception:
        try:
            os.unlink(temporary)
        except OSError:
            pass
        raise


def atomic_write_text(path, text):
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


def remove_file(path):
    try:
        os.unlink(path)
    except FileNotFoundError:
        pass


def sha256_file(path):
    digest = hashlib.sha256()
    try:
        with open(path, "rb") as handle:
            while True:
                chunk = handle.read(1024 * 1024)
                if not chunk:
                    break
                digest.update(chunk)
    except OSError:
        return ""
    return digest.hexdigest()


def run_command(argv, timeout=30):
    try:
        return subprocess.run(
            argv,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=timeout,
        )
    except subprocess.TimeoutExpired as exc:
        return subprocess.CompletedProcess(
            argv,
            124,
            stdout=exc.stdout or "",
            stderr=(exc.stderr or "") + "\ncommand timed out",
        )
    except OSError as exc:
        return subprocess.CompletedProcess(
            argv,
            127,
            stdout="",
            stderr=repr(exc),
        )


def load_json(path):
    with open(path, "r") as handle:
        return json.load(handle)


def is_within(path, parent):
    try:
        Path(path).resolve().relative_to(Path(parent).resolve())
        return True
    except (OSError, ValueError):
        return False


def process_group_alive(pgid):
    try:
        os.killpg(int(pgid), 0)
        return True
    except (OSError, ValueError):
        return False


def find_conflicts():
    matches = []
    own_pid = os.getpid()
    for process_dir in Path("/proc").glob("[0-9]*"):
        try:
            pid = int(process_dir.name)
            if pid == own_pid:
                continue
            command = (
                (process_dir / "cmdline")
                .read_bytes()
                .replace(b"\0", b" ")
                .decode("utf-8", errors="replace")
                .strip()
            )
            comm = (process_dir / "comm").read_text().strip()
        except (OSError, ValueError):
            continue
        if comm in ("bash", "sh", "dash", "ssh", "timeout"):
            continue
        if CONFLICT_PATTERN.search(command):
            matches.append({"pid": pid, "command": command})
    return matches


def spawn_process(name, argv, log_path, state):
    log_handle = open(log_path, "a", buffering=1)
    try:
        process = subprocess.Popen(
            argv,
            stdin=subprocess.DEVNULL,
            stdout=log_handle,
            stderr=subprocess.STDOUT,
            start_new_session=True,
            close_fds=True,
        )
    finally:
        log_handle.close()
    state["processes"][name] = {
        "pid": process.pid,
        "pgid": process.pid,
        "argv": argv,
        "log": str(log_path),
        "started_at": iso_time(),
    }
    atomic_write_json(state["state_file"], state)
    return process.pid


def signal_group(process_info, signum):
    if not process_info:
        return
    try:
        os.killpg(int(process_info["pgid"]), signum)
    except (OSError, ValueError):
        pass


def wait_group(process_info, timeout):
    if not process_info:
        return True
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if not process_group_alive(process_info["pgid"]):
            return True
        time.sleep(0.10)
    return not process_group_alive(process_info["pgid"])


def stop_group(process_info, first_signal, first_wait):
    if not process_info or not process_group_alive(process_info["pgid"]):
        return "not_running"
    signal_group(process_info, first_signal)
    if wait_group(process_info, first_wait):
        return "graceful"
    signal_group(process_info, signal.SIGTERM)
    if wait_group(process_info, 2.0):
        return "terminated"
    signal_group(process_info, signal.SIGKILL)
    wait_group(process_info, 1.0)
    return "killed"


def wait_for(predicate, timeout, interval=0.10):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(interval)
    return bool(predicate())


def file_contains(path, marker):
    try:
        return marker in Path(path).read_text(errors="replace")
    except OSError:
        return False


def capture_localization_session(
    workspace,
    metadata_path,
    log_path,
):
    command = [
        "python3",
        "-u",
        str(Path(workspace) / "scripts/manual_route_anchor.py"),
        "--capture-only",
        "--metadata",
        str(metadata_path),
        "--timeout",
        "12.0",
        "--stable-seconds",
        "1.0",
        "--min-updates",
        "8",
        "--max-local-gap",
        "0.35",
        "--max-translation",
        "0.08",
        "--max-yaw",
        "0.08",
    ]
    result = run_command(command, timeout=20)
    atomic_write_text(
        log_path,
        result.stdout + result.stderr,
    )
    if result.returncode != 0:
        raise RuntimeError(
            "localization session capture failed rc=%d: %s"
            % (
                result.returncode,
                (result.stderr or result.stdout).strip(),
            )
        )
    return load_json(metadata_path)


def take_snapshot(workspace, output, phase, route_file, label):
    command = [
        "python3",
        "-u",
        str(Path(workspace) / "scripts/go2_experiment_snapshot.py"),
        "--output",
        str(output),
        "--phase",
        phase,
        "--workspace",
        str(workspace),
        "--route-file",
        str(route_file),
        "--label",
        label,
    ]
    result = run_command(command, timeout=50)
    atomic_write_text(
        str(output) + ".log",
        result.stdout + result.stderr,
    )
    if result.returncode != 0:
        raise RuntimeError(
            "experiment snapshot failed rc=%d: %s"
            % (
                result.returncode,
                (result.stderr or result.stdout).strip(),
            )
        )


def percentile(values, fraction):
    if not values:
        return None
    ordered = sorted(values)
    position = (len(ordered) - 1) * fraction
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def normalize_angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def summarize_values(values):
    finite = [float(value) for value in values if math.isfinite(float(value))]
    if not finite:
        return {
            "count": 0,
            "min": None,
            "median": None,
            "p95": None,
            "max": None,
        }
    return {
        "count": len(finite),
        "min": min(finite),
        "median": percentile(finite, 0.50),
        "p95": percentile(finite, 0.95),
        "max": max(finite),
    }


def read_route_points(route_file):
    with open(route_file, "r", newline="") as handle:
        reader = csv.DictReader(handle)
        required = {"x", "y", "yaw"}
        if not required.issubset(set(reader.fieldnames or [])):
            raise RuntimeError("route CSV is missing x/y/yaw columns")
        points = []
        for index, row in enumerate(reader):
            try:
                point = {
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
            except (TypeError, ValueError) as exc:
                raise RuntimeError(
                    "invalid route row %d: %s" % (index, exc)
                )
            if not all(
                math.isfinite(point[key])
                for key in ("x", "y", "yaw")
            ):
                raise RuntimeError(
                    "non-finite route row %d" % index
                )
            points.append(point)
    if not points:
        raise RuntimeError("route CSV contains no waypoint")
    return points


def read_telemetry_odom(path):
    samples = []
    health = []
    try:
        handle = open(path, "r")
    except OSError:
        return samples, health
    with handle:
        for line in handle:
            try:
                record = json.loads(line)
            except (TypeError, ValueError):
                continue
            if record.get("kind") == "recorder_health":
                health.append(record)
                continue
            if record.get("kind") != "odom":
                continue
            data = record.get("data", {})
            position = data.get("position", {})
            orientation = data.get("orientation", {})
            try:
                sample = {
                    "wall_time": float(record["wall_time"]),
                    "source_stamp": record.get("source_stamp"),
                    "x": float(position["x"]),
                    "y": float(position["y"]),
                    "z": float(position["z"]),
                    "roll": float(orientation["roll"]),
                    "pitch": float(orientation["pitch"]),
                    "yaw": float(orientation["yaw"]),
                }
            except (KeyError, TypeError, ValueError):
                continue
            if all(
                math.isfinite(sample[key])
                for key in (
                    "wall_time",
                    "x",
                    "y",
                    "z",
                    "roll",
                    "pitch",
                    "yaw",
                )
            ):
                samples.append(sample)
    return samples, health


def route_geometry(points):
    spacings = []
    segment_headings = []
    for previous, current in zip(points, points[1:]):
        dx = current["x"] - previous["x"]
        dy = current["y"] - previous["y"]
        spacings.append(math.hypot(dx, dy))
        segment_headings.append(math.atan2(dy, dx))

    turns = []
    for index in range(1, len(segment_headings)):
        angle = normalize_angle(
            segment_headings[index] - segment_headings[index - 1]
        )
        turns.append(
            {
                "waypoint_index": index,
                "turn_rad": angle,
                "turn_deg": math.degrees(angle),
            }
        )
    strongest_turns = sorted(
        turns,
        key=lambda item: abs(item["turn_rad"]),
        reverse=True,
    )[:20]
    yaw_track_errors = []
    for index, heading in enumerate(segment_headings):
        yaw_track_errors.append(
            abs(normalize_angle(points[index]["yaw"] - heading))
        )
    xs = [point["x"] for point in points]
    ys = [point["y"] for point in points]
    return {
        "waypoint_count": len(points),
        "path_length_m": sum(spacings),
        "spacing_m": summarize_values(spacings),
        "spacing_above_0_60m": sum(
            1 for spacing in spacings if spacing > 0.60
        ),
        "spacing_above_1_00m": sum(
            1 for spacing in spacings if spacing > 1.00
        ),
        "body_yaw_vs_track_error_deg": summarize_values(
            [math.degrees(value) for value in yaw_track_errors]
        ),
        "strongest_turns": strongest_turns,
        "bounding_box": {
            "min_x": min(xs),
            "max_x": max(xs),
            "min_y": min(ys),
            "max_y": max(ys),
        },
        "start": points[0],
        "end": points[-1],
    }


def telemetry_route_comparison(points, odom, min_distance):
    if not odom:
        return {
            "available": False,
            "reason": "no odometry records in telemetry",
        }
    first = points[0]
    start_index = min(
        range(len(odom)),
        key=lambda index: math.hypot(
            odom[index]["x"] - first["x"],
            odom[index]["y"] - first["y"],
        ),
    )
    start_error = math.hypot(
        odom[start_index]["x"] - first["x"],
        odom[start_index]["y"] - first["y"],
    )
    reconstructed = []
    last = None
    for sample in odom[start_index:]:
        if last is None or math.hypot(
            sample["x"] - last["x"],
            sample["y"] - last["y"],
        ) >= min_distance:
            reconstructed.append(sample)
            last = sample

    pair_count = min(len(points), len(reconstructed))
    position_errors = []
    yaw_errors = []
    for point, sample in zip(points[:pair_count], reconstructed[:pair_count]):
        position_errors.append(
            math.hypot(
                sample["x"] - point["x"],
                sample["y"] - point["y"],
            )
        )
        yaw_errors.append(
            abs(normalize_angle(sample["yaw"] - point["yaw"]))
        )
    exact = (
        start_error <= 0.002
        and len(points) == len(reconstructed)
        and bool(position_errors)
        and max(position_errors) <= 0.002
        and max(yaw_errors) <= 0.002
    )
    return {
        "available": True,
        "telemetry_start_index": start_index,
        "route_start_to_nearest_telemetry_m": start_error,
        "reconstructed_waypoint_count": len(reconstructed),
        "csv_waypoint_count": len(points),
        "paired_waypoint_count": pair_count,
        "position_error_m": summarize_values(position_errors),
        "yaw_error_deg": summarize_values(
            [math.degrees(value) for value in yaw_errors]
        ),
        "exact_within_2mm_0_002rad": exact,
        "interpretation": (
            "exact recorder reproduction from sampled telemetry"
            if exact
            else (
                "telemetry is independently depth-1 sampled; use the "
                "full /Odometry rosbag as canonical input if counts differ"
            )
        ),
    }


def build_route_audit(route_file, telemetry_file, min_distance):
    points = read_route_points(route_file)
    odom, health = read_telemetry_odom(telemetry_file)
    source_stamp_gaps = []
    for previous, current in zip(odom, odom[1:]):
        before = previous.get("source_stamp")
        after = current.get("source_stamp")
        if before is None or after is None:
            continue
        try:
            gap = float(after) - float(before)
        except (TypeError, ValueError):
            continue
        if math.isfinite(gap) and gap >= 0.0:
            source_stamp_gaps.append(gap)
    spatial_steps = []
    planar_steps = []
    for previous, current in zip(odom, odom[1:]):
        dx = current["x"] - previous["x"]
        dy = current["y"] - previous["y"]
        dz = current["z"] - previous["z"]
        planar_steps.append(math.hypot(dx, dy))
        spatial_steps.append(math.sqrt(dx * dx + dy * dy + dz * dz))
    received = (
        health[-1].get("data", {}).get("received", {})
        if health
        else {}
    )
    written = (
        health[-1].get("data", {}).get("written", {})
        if health
        else {}
    )
    return {
        "schema": "go2.route_recording_audit.v1",
        "created_at": iso_time(),
        "route_file": str(route_file),
        "route_sha256": sha256_file(route_file),
        "min_distance_m": min_distance,
        "route_geometry": route_geometry(points),
        "telemetry": {
            "odom_samples": len(odom),
            "health_records": len(health),
            "received_at_end": received,
            "written_at_end": written,
            "source_stamp_gap_s": summarize_values(
                source_stamp_gaps
            ),
            "z_m": summarize_values(
                [sample["z"] for sample in odom]
            ),
            "roll_deg": summarize_values(
                [math.degrees(sample["roll"]) for sample in odom]
            ),
            "pitch_deg": summarize_values(
                [math.degrees(sample["pitch"]) for sample in odom]
            ),
            "planar_motion_m": sum(planar_steps),
            "spatial_motion_m": sum(spatial_steps),
        },
        "recorder_reproduction": telemetry_route_comparison(
            points,
            odom,
            min_distance,
        ),
    }


def base_evidence_sources(workspace):
    return (
        Path(workspace) / "patrol_logs/livox.log",
        Path(workspace) / "patrol_logs/fast_lio.log",
        Path(workspace)
        / "patrol_logs/service/go2-base-health-watchdog.log",
        Path("/tmp/console_base.log"),
        Path("/tmp/go2_saas_base.log"),
        Path("/tmp/go2_saas_loop.log"),
    )


def capture_log_offsets(workspace):
    offsets = {}
    for source in base_evidence_sources(workspace):
        try:
            offsets[str(source)] = source.stat().st_size
        except OSError:
            offsets[str(source)] = 0
    return offsets


def copy_base_evidence(workspace, run_dir, offsets=None):
    sources = base_evidence_sources(workspace)
    offsets = offsets or {}
    output_dir = Path(run_dir) / "base_logs"
    output_dir.mkdir(parents=True, exist_ok=True)
    copied = []
    for source in sources:
        if not source.is_file():
            continue
        destination = output_dir / source.name
        if destination.exists():
            destination = output_dir / (
                source.parent.name + "_" + source.name
            )
        try:
            start_offset = max(0, int(offsets.get(str(source), 0)))
            current_size = source.stat().st_size
            truncated_since_start = current_size < start_offset
            if truncated_since_start:
                start_offset = 0
            with open(source, "rb") as input_handle:
                input_handle.seek(start_offset)
                with open(destination, "wb") as output_handle:
                    shutil.copyfileobj(input_handle, output_handle)
                    output_handle.flush()
                    os.fsync(output_handle.fileno())
        except OSError:
            continue
        copied.append(
            {
                "source": str(source),
                "copy": str(destination.relative_to(run_dir)),
                "size_bytes": destination.stat().st_size,
                "sha256": sha256_file(str(destination)),
                "start_offset_bytes": start_offset,
                "source_size_at_stop_bytes": current_size,
                "source_truncated_since_start": truncated_since_start,
            }
        )
    return copied


def build_run_dir(workspace, route_file):
    day = time.strftime("%Y%m%d")
    stem = re.sub(
        r"[^A-Za-z0-9_-]+",
        "_",
        Path(route_file).stem,
    )[:40]
    name = "record-%s-%s-%s-%d" % (
        time.strftime("%Y%m%d-%H%M%S"),
        stem,
        "blackbox",
        os.getpid(),
    )
    path = (
        Path(workspace)
        / "patrol_logs"
        / "recordings"
        / day
        / name
    )
    path.mkdir(parents=True, exist_ok=False)
    return path


def validate_route_target(workspace, route_file):
    routes_root = (
        Path(workspace) / "src/go2_fastlio_patrol/routes"
    ).resolve()
    target = Path(route_file).resolve()
    if target.suffix.lower() != ".csv":
        raise RuntimeError("route target must end in .csv")
    if not is_within(target, routes_root):
        raise RuntimeError(
            "route target must stay under %s" % routes_root
        )
    return target


def write_console_symlink(target):
    remove_file(DEFAULT_CONSOLE_LOG)
    try:
        os.symlink(str(target), DEFAULT_CONSOLE_LOG)
    except OSError:
        pass


def cleanup_started_processes(state):
    processes = state.get("processes", {})
    remove_file(state.get("active_file", DEFAULT_ACTIVE_FILE))
    results = {}
    for name, signum, wait_seconds in (
        ("route_recorder", signal.SIGINT, 3.0),
        ("rosbag", signal.SIGINT, 5.0),
        ("telemetry", signal.SIGTERM, 2.0),
        ("performance", signal.SIGTERM, 2.0),
        ("session_guard", signal.SIGTERM, 2.0),
    ):
        results[name] = stop_group(
            processes.get(name),
            signum,
            wait_seconds,
        )
    return results


def start_recording(args):
    workspace = Path(args.workspace).resolve()
    route_file = validate_route_target(workspace, args.route_file)
    state_path = Path(args.state_file)
    if state_path.exists():
        previous = load_json(state_path)
        live = [
            name
            for name, info in previous.get("processes", {}).items()
            if process_group_alive(info.get("pgid", -1))
        ]
        if live:
            raise RuntimeError(
                "recording blackbox already active: %s"
                % ",".join(live)
            )
        remove_file(str(state_path))
    conflicts = find_conflicts()
    if conflicts:
        raise RuntimeError(
            "mode conflict: %s"
            % json.dumps(conflicts, sort_keys=True)
        )
    if route_file.exists() and not args.overwrite:
        raise RuntimeError("route already exists: %s" % route_file)

    run_dir = build_run_dir(workspace, route_file)
    state = {
        "schema": "go2.route_recording_blackbox_state.v1",
        "status": "starting",
        "created_at": iso_time(),
        "workspace": str(workspace),
        "route_file": str(route_file),
        "min_distance_m": float(args.min_distance),
        "run_dir": str(run_dir),
        "state_file": str(state_path),
        "active_file": str(args.active_file),
        "processes": {},
        "base_log_offsets": capture_log_offsets(workspace),
    }
    atomic_write_json(str(state_path), state)
    manifest = {
        "schema": "go2.route_recording_run.v1",
        "status": "starting",
        "created_at": state["created_at"],
        "run_dir": str(run_dir),
        "route_file": str(route_file),
        "min_distance_m": float(args.min_distance),
        "observer_policy": {
            "raw_pointcloud_subscriber": False,
            "pointcloud_evidence": (
                "Livox and FAST-LIO in-process timing logs"
            ),
            "rosbag_topics": list(LIGHTWEIGHT_BAG_TOPICS),
        },
    }
    atomic_write_json(run_dir / "manifest.json", manifest)

    try:
        session_start = capture_localization_session(
            workspace,
            run_dir / "localization_session_start.json",
            run_dir / "localization_session_start.log",
        )
        manifest["localization_session_start"] = session_start
        take_snapshot(
            workspace,
            run_dir / "system_start.json",
            "start",
            route_file,
            run_dir.name,
        )

        if route_file.exists():
            remove_file(str(route_file))
        route_file.parent.mkdir(parents=True, exist_ok=True)
        Path(args.active_file).touch()

        telemetry_log = run_dir / "experiment_telemetry.log"
        telemetry_jsonl = run_dir / "experiment_telemetry.jsonl"
        performance_log = run_dir / "performance_monitor.log"
        rosbag_log = run_dir / "rosbag_record.log"
        guard_log = run_dir / "localization_session_guard.log"
        recorder_log = run_dir / "route_recorder.log"

        spawn_process(
            "telemetry",
            [
                "python3",
                "-u",
                str(workspace / "scripts/go2_experiment_telemetry.py"),
                "--profile",
                "recording",
                "--output",
                str(telemetry_jsonl),
            ],
            telemetry_log,
            state,
        )
        spawn_process(
            "performance",
            [
                "python3",
                "-u",
                str(workspace / "scripts/patrol_performance_monitor.py"),
                "--interval",
                "1.0",
                "--filesystem-path",
                str(run_dir),
            ],
            performance_log,
            state,
        )
        spawn_process(
            "rosbag",
            [
                "ros2",
                "bag",
                "record",
                "-o",
                str(run_dir / "rosbag"),
            ]
            + list(LIGHTWEIGHT_BAG_TOPICS),
            rosbag_log,
            state,
        )

        if not wait_for(
            lambda: (
                process_group_alive(
                    state["processes"]["telemetry"]["pgid"]
                )
                and file_contains(
                    telemetry_jsonl,
                    '"kind":"recorder_ready"',
                )
            ),
            10.0,
        ):
            raise RuntimeError("experiment telemetry did not become ready")
        if not wait_for(
            lambda: (
                process_group_alive(
                    state["processes"]["performance"]["pgid"]
                )
                and file_contains(
                    performance_log,
                    "PERF_MONITOR_START",
                )
            ),
            8.0,
        ):
            raise RuntimeError("performance monitor did not become ready")
        if not wait_for(
            lambda: (
                process_group_alive(
                    state["processes"]["rosbag"]["pgid"]
                )
                and any(
                    (run_dir / "rosbag").glob("*.db3")
                )
            ),
            12.0,
        ):
            raise RuntimeError("lightweight rosbag did not become ready")

        spawn_process(
            "session_guard",
            [
                "python3",
                "-u",
                str(workspace / "scripts/localization_session_guard.py"),
                "--metadata",
                str(run_dir / "localization_session_start.json"),
                "--active-file",
                str(args.active_file),
                "--mode",
                "recorder",
                "--event-log",
                str(guard_log),
                "--invalidate-output",
                str(route_file),
            ],
            guard_log,
            state,
        )
        if not wait_for(
            lambda: (
                process_group_alive(
                    state["processes"]["session_guard"]["pgid"]
                )
                and file_contains(
                    guard_log,
                    "SESSION_GUARD_STARTED",
                )
            ),
            8.0,
        ):
            raise RuntimeError(
                "localization session guard did not become ready"
            )

        write_console_symlink(recorder_log)
        spawn_process(
            "route_recorder",
            [
                "ros2",
                "run",
                "go2_fastlio_patrol",
                "route_recorder",
                "--ros-args",
                "-p",
                "route_file:=%s" % route_file,
                "-p",
                "min_distance:=%.6f" % args.min_distance,
            ],
            recorder_log,
            state,
        )
        if not wait_for(
            lambda: (
                process_group_alive(
                    state["processes"]["route_recorder"]["pgid"]
                )
                and route_file.is_file()
                and route_file.stat().st_size > 5
                and file_contains(
                    recorder_log,
                    "route_recorder started",
                )
            ),
            10.0,
        ):
            raise RuntimeError("route recorder did not become ready")

        state["status"] = "recording"
        state["ready_at"] = iso_time()
        atomic_write_json(str(state_path), state)
        manifest["status"] = "recording"
        manifest["ready_at"] = state["ready_at"]
        manifest["processes"] = state["processes"]
        manifest["evidence"] = {
            "telemetry": "experiment_telemetry.jsonl",
            "performance": "performance_monitor.log",
            "rosbag": "rosbag",
            "route_recorder": "route_recorder.log",
            "session_guard": "localization_session_guard.log",
            "system_start": "system_start.json",
        }
        atomic_write_json(run_dir / "manifest.json", manifest)
    except Exception as exc:
        state["status"] = "failed_start"
        state["failure"] = repr(exc)
        state["cleanup"] = cleanup_started_processes(state)
        atomic_write_json(str(state_path), state)
        manifest["status"] = "failed_start"
        manifest["failure"] = repr(exc)
        manifest["cleanup"] = state["cleanup"]
        manifest["failed_at"] = iso_time()
        atomic_write_json(run_dir / "manifest.json", manifest)
        try:
            take_snapshot(
                workspace,
                run_dir / "system_failure.json",
                "failure",
                route_file,
                run_dir.name,
            )
        except Exception as snapshot_exc:
            manifest["failure_snapshot_error"] = repr(snapshot_exc)
            atomic_write_json(run_dir / "manifest.json", manifest)
        remove_file(str(state_path))
        raise

    print(
        "RECORDING_BLACKBOX_STARTED route=%s run_dir=%s "
        "session=%s:%s"
        % (
            route_file,
            run_dir,
            session_start["localization_session"]["pid"],
            session_start["localization_session"]["start_ticks"],
        ),
        flush=True,
    )


def finalize_rosbag(run_dir):
    rosbag_dir = Path(run_dir) / "rosbag"
    if not rosbag_dir.is_dir():
        return {"available": False}
    result = run_command(
        ["ros2", "bag", "info", str(rosbag_dir)],
        timeout=20,
    )
    atomic_write_text(
        Path(run_dir) / "rosbag_info.txt",
        result.stdout + result.stderr,
    )
    return {
        "available": True,
        "returncode": result.returncode,
        "files": [
            {
                "name": path.name,
                "size_bytes": path.stat().st_size,
                "sha256": sha256_file(str(path)),
            }
            for path in sorted(rosbag_dir.glob("*"))
            if path.is_file()
        ],
    }


def stop_recording(args):
    state_path = Path(args.state_file)
    if not state_path.exists():
        raise RuntimeError(
            "no active recording blackbox state: %s" % state_path
        )
    state = load_json(state_path)
    workspace = Path(state["workspace"])
    run_dir = Path(state["run_dir"])
    route_file = Path(state["route_file"])
    manifest_path = run_dir / "manifest.json"
    try:
        manifest = load_json(manifest_path)
    except (OSError, ValueError):
        manifest = {
            "schema": "go2.route_recording_run.v1",
            "run_dir": str(run_dir),
            "route_file": str(route_file),
        }

    remove_file(state.get("active_file", DEFAULT_ACTIVE_FILE))
    processes = state.get("processes", {})
    stop_results = {}
    stop_results["route_recorder"] = stop_group(
        processes.get("route_recorder"),
        signal.SIGINT,
        5.0,
    )
    stop_results["rosbag"] = stop_group(
        processes.get("rosbag"),
        signal.SIGINT,
        8.0,
    )
    stop_results["telemetry"] = stop_group(
        processes.get("telemetry"),
        signal.SIGTERM,
        3.0,
    )
    stop_results["performance"] = stop_group(
        processes.get("performance"),
        signal.SIGTERM,
        3.0,
    )
    stop_results["session_guard"] = stop_group(
        processes.get("session_guard"),
        signal.SIGTERM,
        2.0,
    )

    errors = []
    session_end = None
    try:
        session_end = capture_localization_session(
            workspace,
            run_dir / "localization_session_end.json",
            run_dir / "localization_session_end.log",
        )
    except Exception as exc:
        errors.append("end_session:%r" % exc)
    try:
        take_snapshot(
            workspace,
            run_dir / "system_end.json",
            "stop",
            route_file,
            run_dir.name,
        )
    except Exception as exc:
        errors.append("end_snapshot:%r" % exc)

    route_copy = run_dir / "route_recorded.csv"
    audit = None
    if route_file.is_file():
        try:
            shutil.copy2(str(route_file), str(route_copy))
            audit = build_route_audit(
                str(route_file),
                str(run_dir / "experiment_telemetry.jsonl"),
                float(state["min_distance_m"]),
            )
            atomic_write_json(
                run_dir / "route_recording_audit.json",
                audit,
            )
        except Exception as exc:
            errors.append("route_audit:%r" % exc)
    else:
        errors.append("route_missing:%s" % route_file)

    rosbag = finalize_rosbag(run_dir)
    base_logs = copy_base_evidence(
        workspace,
        run_dir,
        offsets=state.get("base_log_offsets", {}),
    )
    if (
        not rosbag.get("available")
        or not any(
            item.get("name", "").endswith(".db3")
            and item.get("size_bytes", 0) > 0
            for item in rosbag.get("files", [])
        )
    ):
        errors.append("rosbag_missing_or_empty")
    telemetry_path = run_dir / "experiment_telemetry.jsonl"
    if (
        not file_contains(telemetry_path, '"kind":"recorder_ready"')
        or not file_contains(telemetry_path, '"kind":"recorder_stop"')
    ):
        errors.append("telemetry_not_cleanly_finalized")
    performance_path = run_dir / "performance_monitor.log"
    if (
        not file_contains(performance_path, "PERF_MONITOR_START")
        or not file_contains(performance_path, "PERF_MONITOR_STOP")
    ):
        errors.append("performance_monitor_not_cleanly_finalized")
    copied_source_names = {
        Path(item.get("source", "")).name
        for item in base_logs
    }
    for required_base_log in ("livox.log", "fast_lio.log"):
        if required_base_log not in copied_source_names:
            errors.append(
                "base_log_missing:%s" % required_base_log
            )
    if audit is not None and not (
        audit.get("recorder_reproduction", {}).get(
            "exact_within_2mm_0_002rad"
        )
    ):
        errors.append("route_sampler_reproduction_mismatch")
    route_sha256 = sha256_file(str(route_file))
    line_count = 0
    if route_file.is_file():
        try:
            with open(route_file, "r") as handle:
                line_count = sum(1 for _line in handle)
        except OSError:
            line_count = 0

    start_session_identity = (
        manifest.get("localization_session_start", {})
        .get("localization_session", {})
    )
    end_session_identity = (
        session_end.get("localization_session", {})
        if session_end
        else {}
    )
    same_session = bool(
        start_session_identity
        and end_session_identity
        and start_session_identity.get("boot_id")
        == end_session_identity.get("boot_id")
        and start_session_identity.get("pid")
        == end_session_identity.get("pid")
        and start_session_identity.get("start_ticks")
        == end_session_identity.get("start_ticks")
    )
    if not same_session:
        errors.append("fastlio_session_changed_during_recording")
    for process_name in ("route_recorder", "rosbag", "telemetry"):
        if stop_results.get(process_name) == "killed":
            errors.append(
                "unclean_process_stop:%s" % process_name
            )

    completed_at = iso_time()
    sidecar = {
        "schema": "go2.route_recording_link.v1",
        "status": "complete" if not errors else "complete_with_warnings",
        "recording_run_dir": str(run_dir),
        "route_file": str(route_file),
        "route_sha256": route_sha256,
        "route_line_count": line_count,
        "min_distance_m": float(state["min_distance_m"]),
        "created_at": state.get("created_at"),
        "completed_at": completed_at,
        "same_fastlio_session_at_start_and_stop": same_session,
        "localization_session_start": start_session_identity,
        "localization_session_end": end_session_identity,
        "audit": "route_recording_audit.json"
        if audit is not None
        else "",
        "errors": errors,
    }
    if route_file.is_file():
        atomic_write_json(
            str(route_file) + ".recording.json",
            sidecar,
        )
    atomic_write_json(run_dir / "route_link.json", sidecar)

    manifest.update(
        {
            "status": sidecar["status"],
            "completed_at": completed_at,
            "stop_results": stop_results,
            "localization_session_end": session_end,
            "same_fastlio_session_at_start_and_stop": same_session,
            "route_sha256": route_sha256,
            "route_line_count": line_count,
            "route_copy": (
                route_copy.name if route_copy.is_file() else ""
            ),
            "route_audit": (
                "route_recording_audit.json"
                if audit is not None
                else ""
            ),
            "rosbag": rosbag,
            "base_logs": base_logs,
            "errors": errors,
        }
    )
    try:
        stat = os.statvfs(str(run_dir))
        manifest["filesystem_available_bytes_at_stop"] = (
            stat.f_bavail * stat.f_frsize
        )
    except OSError:
        pass
    atomic_write_json(manifest_path, manifest)
    remove_file(str(state_path))
    print(
        "RECORDING_BLACKBOX_STOPPED route=%s run_dir=%s "
        "lines=%d sha256=%s status=%s"
        % (
            route_file,
            run_dir,
            line_count,
            route_sha256,
            sidecar["status"],
        ),
        flush=True,
    )
    if errors:
        print(
            "RECORDING_BLACKBOX_WARNINGS "
            + json.dumps(errors, ensure_ascii=False),
            flush=True,
        )


def show_status(args):
    path = Path(args.state_file)
    if not path.exists():
        print(
            json.dumps(
                {"active": False, "state_file": str(path)},
                sort_keys=True,
            )
        )
        return
    state = load_json(path)
    state["active"] = any(
        process_group_alive(info.get("pgid", -1))
        for info in state.get("processes", {}).values()
    )
    state["process_alive"] = {
        name: process_group_alive(info.get("pgid", -1))
        for name, info in state.get("processes", {}).items()
    }
    print(json.dumps(state, sort_keys=True))


def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--workspace",
        default=DEFAULT_WS,
    )
    parser.add_argument(
        "--state-file",
        default=DEFAULT_STATE_FILE,
    )
    parser.add_argument(
        "--active-file",
        default=DEFAULT_ACTIVE_FILE,
    )
    subparsers = parser.add_subparsers(
        dest="command",
        required=True,
    )
    start = subparsers.add_parser("start")
    start.add_argument("--route-file", required=True)
    start.add_argument("--min-distance", type=float, default=0.40)
    start.add_argument("--overwrite", action="store_true")
    subparsers.add_parser("stop")
    subparsers.add_parser("status")
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)
    if args.command == "start":
        if not 0.05 <= args.min_distance <= 2.0:
            raise SystemExit("--min-distance must be in [0.05, 2.0]")
        try:
            start_recording(args)
        except Exception as exc:
            print(
                "RECORDING_BLACKBOX_START_FAILED %s" % exc,
                file=os.sys.stderr,
                flush=True,
            )
            return 2
        return 0
    if args.command == "stop":
        try:
            stop_recording(args)
        except Exception as exc:
            print(
                "RECORDING_BLACKBOX_STOP_FAILED %s" % exc,
                file=os.sys.stderr,
                flush=True,
            )
            return 2
        return 0
    show_status(args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

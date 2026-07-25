#!/usr/bin/env python3
"""Create a patrol route in the current FAST-LIO odometry frame.

The source CSV is never modified.  The first source waypoint is treated as the
manually placed physical start pose, and the complete route is transformed
rigidly so that waypoint zero matches the dog's current, stationary odometry
pose.

The metadata file also records the exact FAST-LIO process session.  A separate
runtime guard uses that identity to stop a task if FAST-LIO is restarted.
"""

import argparse
import csv
import json
import math
import os
import tempfile
import time
from pathlib import Path


DEFAULT_ODOM_SNAPSHOT = "/dev/shm/go2_fastlio_latest_odom.txt"
REQUIRED_ODOM_KEYS = {"stamp", "x", "y", "z", "qx", "qy", "qz", "qw"}


def normalize_angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def yaw_from_quaternion(qx, qy, qz, qw):
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def _read_text(path):
    with open(path, "r") as handle:
        return handle.read().strip()


def _process_start_ticks(pid):
    # /proc/<pid>/stat field 22 is starttime.  The comm field may contain
    # spaces/parentheses, so split only after the final closing parenthesis.
    raw = _read_text("/proc/%d/stat" % pid)
    tail = raw[raw.rfind(")") + 2 :].split()
    if len(tail) < 20:
        raise RuntimeError("incomplete /proc/%d/stat" % pid)
    return int(tail[19])


def _fastlio_pids():
    matches = []
    for name in os.listdir("/proc"):
        if not name.isdigit():
            continue
        pid = int(name)
        try:
            comm = _read_text("/proc/%d/comm" % pid)
            executable = os.path.basename(
                os.readlink("/proc/%d/exe" % pid)
            )
        except OSError:
            continue
        if comm == "fastlio_mapping" or executable == "fastlio_mapping":
            matches.append(pid)
    return sorted(set(matches))


def current_fastlio_session():
    pids = _fastlio_pids()
    if len(pids) != 1:
        raise RuntimeError(
            "expected exactly one fastlio_mapping process, found %d: %s"
            % (len(pids), ",".join(str(pid) for pid in pids) or "none")
        )
    pid = pids[0]
    try:
        boot_id = _read_text("/proc/sys/kernel/random/boot_id")
    except OSError:
        boot_id = ""
    try:
        executable = os.readlink("/proc/%d/exe" % pid)
    except OSError:
        executable = ""
    return {
        "boot_id": boot_id,
        "pid": pid,
        "start_ticks": _process_start_ticks(pid),
        "executable": executable,
    }


def sessions_equal(expected, actual):
    return (
        str(expected.get("boot_id", "")) == str(actual.get("boot_id", ""))
        and int(expected.get("pid", -1)) == int(actual.get("pid", -2))
        and int(expected.get("start_ticks", -1))
        == int(actual.get("start_ticks", -2))
    )


def read_odom_snapshot(path):
    with open(path, "r") as handle:
        tokens = handle.readline().strip().split()
        stat = os.fstat(handle.fileno())
    values = {}
    for token in tokens:
        key, separator, value = token.partition("=")
        if separator:
            values[key] = float(value)
    if not REQUIRED_ODOM_KEYS.issubset(values):
        missing = sorted(REQUIRED_ODOM_KEYS.difference(values))
        raise ValueError("incomplete odometry snapshot, missing %s" % ",".join(missing))
    finite = [values[key] for key in sorted(REQUIRED_ODOM_KEYS)]
    if not all(math.isfinite(value) for value in finite):
        raise ValueError("odometry snapshot contains non-finite values")
    quaternion_norm = math.sqrt(
        values["qx"] ** 2
        + values["qy"] ** 2
        + values["qz"] ** 2
        + values["qw"] ** 2
    )
    if not 0.5 <= quaternion_norm <= 1.5:
        raise ValueError("invalid odometry quaternion norm %.6f" % quaternion_norm)
    values["yaw"] = yaw_from_quaternion(
        values["qx"], values["qy"], values["qz"], values["qw"]
    )
    values["snapshot_mtime_ns"] = int(stat.st_mtime_ns)
    values["snapshot_signature"] = (
        int(stat.st_mtime_ns),
        float(values["stamp"]),
    )
    return values


def collect_stationary_anchor(
    odom_snapshot=DEFAULT_ODOM_SNAPSHOT,
    timeout=12.0,
    stable_seconds=1.0,
    min_updates=8,
    max_local_gap=0.35,
    max_translation=0.08,
    max_yaw=0.08,
):
    expected_session = current_fastlio_session()
    deadline = time.monotonic() + timeout
    samples = []
    last_signature = None
    last_update_at = None
    largest_gap = 0.0
    last_error = ""

    while time.monotonic() < deadline:
        try:
            actual_session = current_fastlio_session()
            if not sessions_equal(expected_session, actual_session):
                raise RuntimeError(
                    "FAST-LIO session changed while capturing the manual anchor"
                )
            odom = read_odom_snapshot(odom_snapshot)
            signature = odom["snapshot_signature"]
            if signature == last_signature:
                time.sleep(0.03)
                continue

            now = time.monotonic()
            if last_update_at is not None:
                gap = now - last_update_at
                if gap > max_local_gap:
                    samples = []
                    largest_gap = 0.0
                else:
                    largest_gap = max(largest_gap, gap)
            last_update_at = now
            last_signature = signature
            samples.append((now, odom))
            cutoff = now - stable_seconds
            samples = [sample for sample in samples if sample[0] >= cutoff]

            if len(samples) < min_updates:
                continue
            if samples[-1][0] - samples[0][0] < stable_seconds * 0.85:
                continue

            reference = samples[-1][1]
            translation_span = max(
                math.sqrt(
                    (sample[1]["x"] - reference["x"]) ** 2
                    + (sample[1]["y"] - reference["y"]) ** 2
                    + (sample[1]["z"] - reference["z"]) ** 2
                )
                for sample in samples
            )
            yaw_span = max(
                abs(normalize_angle(sample[1]["yaw"] - reference["yaw"]))
                for sample in samples
            )
            if translation_span > max_translation or yaw_span > max_yaw:
                last_error = (
                    "dog not stationary: translation_span=%.3f/%.3f "
                    "yaw_span=%.3f/%.3f"
                    % (translation_span, max_translation, yaw_span, max_yaw)
                )
                continue

            final_session = current_fastlio_session()
            if not sessions_equal(expected_session, final_session):
                raise RuntimeError(
                    "FAST-LIO session changed after capturing the manual anchor"
                )
            anchor = dict(reference)
            anchor.pop("snapshot_signature", None)
            return anchor, final_session, {
                "samples": len(samples),
                "duration_seconds": samples[-1][0] - samples[0][0],
                "max_local_gap_seconds": largest_gap,
                "translation_span_m": translation_span,
                "yaw_span_rad": yaw_span,
            }
        except RuntimeError:
            raise
        except (OSError, ValueError) as exc:
            last_error = str(exc)
        time.sleep(0.03)

    raise RuntimeError(
        "stationary odometry anchor unavailable within %.1fs%s"
        % (timeout, ": " + last_error if last_error else "")
    )


def read_route(path):
    with open(path, "r", newline="") as handle:
        reader = csv.DictReader(handle)
        fieldnames = list(reader.fieldnames or [])
        rows = list(reader)
    required = {"x", "y", "yaw"}
    if not required.issubset(fieldnames):
        raise RuntimeError(
            "route csv missing columns: %s"
            % ",".join(sorted(required.difference(fieldnames)))
        )
    if len(rows) < 2:
        raise RuntimeError("route csv needs at least two waypoint rows")
    for index, row in enumerate(rows):
        try:
            values = (float(row["x"]), float(row["y"]), float(row["yaw"]))
        except (TypeError, ValueError) as exc:
            raise RuntimeError(
                "route csv has invalid pose at waypoint %d: %s" % (index, exc)
            )
        if not all(math.isfinite(value) for value in values):
            raise RuntimeError(
                "route csv has non-finite pose at waypoint %d" % index
            )
    return fieldnames, rows


def transform_route_rows(fieldnames, rows, anchor_x, anchor_y, anchor_yaw):
    del fieldnames  # Kept in the public signature for test/readability symmetry.
    source_x = float(rows[0]["x"])
    source_y = float(rows[0]["y"])
    source_yaw = float(rows[0]["yaw"])
    delta_yaw = normalize_angle(anchor_yaw - source_yaw)
    cosine = math.cos(delta_yaw)
    sine = math.sin(delta_yaw)
    transformed = []

    for row in rows:
        source_point_x = float(row["x"])
        source_point_y = float(row["y"])
        dx = source_point_x - source_x
        dy = source_point_y - source_y
        output = dict(row)
        output["x"] = "%.6f" % (
            anchor_x + cosine * dx - sine * dy
        )
        output["y"] = "%.6f" % (
            anchor_y + sine * dx + cosine * dy
        )
        output["yaw"] = "%.6f" % normalize_angle(
            float(row["yaw"]) + delta_yaw
        )
        transformed.append(output)

    return transformed, {
        "source_start": {
            "x": source_x,
            "y": source_y,
            "yaw": source_yaw,
        },
        "current_start": {
            "x": anchor_x,
            "y": anchor_y,
            "yaw": anchor_yaw,
        },
        "delta_yaw": delta_yaw,
    }


def _atomic_write_csv(path, fieldnames, rows):
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(
        prefix=".%s." % output.name,
        suffix=".tmp",
        dir=str(output.parent),
        text=True,
    )
    try:
        with os.fdopen(fd, "w", newline="") as handle:
            writer = csv.DictWriter(
                handle,
                fieldnames=fieldnames,
                extrasaction="ignore",
            )
            writer.writeheader()
            writer.writerows(rows)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, str(output))
    except Exception:
        try:
            os.unlink(temporary)
        except OSError:
            pass
        raise


def _atomic_write_json(path, payload):
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


def build_parser():
    parser = argparse.ArgumentParser(
        description="Capture a stationary FAST-LIO session and optionally anchor a route"
    )
    parser.add_argument("--route-file")
    parser.add_argument("--output-route")
    parser.add_argument("--metadata", required=True)
    parser.add_argument("--capture-only", action="store_true")
    parser.add_argument("--odom-snapshot", default=DEFAULT_ODOM_SNAPSHOT)
    parser.add_argument("--timeout", type=float, default=12.0)
    parser.add_argument("--stable-seconds", type=float, default=1.0)
    parser.add_argument("--min-updates", type=int, default=8)
    parser.add_argument("--max-local-gap", type=float, default=0.35)
    parser.add_argument("--max-translation", type=float, default=0.08)
    parser.add_argument("--max-yaw", type=float, default=0.08)
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)
    if args.capture_only:
        if args.route_file or args.output_route:
            raise SystemExit(
                "--capture-only cannot be combined with --route-file/--output-route"
            )
    elif not args.route_file or not args.output_route:
        raise SystemExit(
            "--route-file and --output-route are required unless --capture-only is used"
        )
    if args.min_updates < 2:
        raise SystemExit("--min-updates must be at least 2")

    try:
        anchor, session, stability = collect_stationary_anchor(
            odom_snapshot=args.odom_snapshot,
            timeout=args.timeout,
            stable_seconds=args.stable_seconds,
            min_updates=args.min_updates,
            max_local_gap=args.max_local_gap,
            max_translation=args.max_translation,
            max_yaw=args.max_yaw,
        )
        metadata = {
            "schema": "go2.manual_anchor.v1",
            "created_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
            "mode": "session_only" if args.capture_only else "manual_route",
            "odom_snapshot": args.odom_snapshot,
            "localization_session": session,
            "current_anchor": {
                key: anchor[key]
                for key in (
                    "stamp",
                    "x",
                    "y",
                    "z",
                    "qx",
                    "qy",
                    "qz",
                    "qw",
                    "yaw",
                    "snapshot_mtime_ns",
                )
            },
            "stability": stability,
        }

        if not args.capture_only:
            fieldnames, rows = read_route(args.route_file)
            transformed, transform = transform_route_rows(
                fieldnames,
                rows,
                anchor["x"],
                anchor["y"],
                anchor["yaw"],
            )
            _atomic_write_csv(args.output_route, fieldnames, transformed)
            metadata.update(
                {
                    "source_route": str(Path(args.route_file).resolve()),
                    "runtime_route": str(Path(args.output_route).resolve()),
                    "waypoint_count": len(rows),
                    "transform": transform,
                }
            )

        _atomic_write_json(args.metadata, metadata)
    except Exception as exc:  # noqa: BLE001
        print("MANUAL_ANCHOR_FAILED %s" % exc, file=os.sys.stderr)
        return 2

    if args.capture_only:
        print(
            "LOCALIZATION_SESSION_CAPTURED pid=%s start_ticks=%s "
            "anchor=(%.3f,%.3f,%.3f)"
            % (
                session["pid"],
                session["start_ticks"],
                anchor["x"],
                anchor["y"],
                anchor["yaw"],
            )
        )
    else:
        print(
            "MANUAL_ANCHOR_READY route=%s points=%d "
            "source=(%.3f,%.3f,%.3f) current=(%.3f,%.3f,%.3f) "
            "delta_yaw=%.3f session=%s:%s"
            % (
                args.output_route,
                metadata["waypoint_count"],
                transform["source_start"]["x"],
                transform["source_start"]["y"],
                transform["source_start"]["yaw"],
                transform["current_start"]["x"],
                transform["current_start"]["y"],
                transform["current_start"]["yaw"],
                transform["delta_yaw"],
                session["pid"],
                session["start_ticks"],
            )
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

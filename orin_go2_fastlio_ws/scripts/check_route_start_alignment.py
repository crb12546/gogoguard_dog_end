#!/usr/bin/env python3
import argparse
import csv
import math
import os
import sys
import time

def normalize_angle(value):
    while value > math.pi:
        value -= 2.0 * math.pi
    while value < -math.pi:
        value += 2.0 * math.pi
    return value


def yaw_from_quaternion(qx, qy, qz, qw):
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def read_route_start(path):
    with open(path, "r", newline="") as handle:
        reader = csv.DictReader(handle)
        row = next(reader, None)
    if not row:
        raise RuntimeError("route csv has no waypoint rows: %s" % path)
    return float(row["x"]), float(row["y"]), float(row["yaw"])


def collect_odom_snapshot(path, timeout, max_local_gap, min_updates):
    deadline = time.monotonic() + timeout
    latest = None
    last_signature = None
    last_update_time = None
    consecutive_updates = 0
    local_gap_max = 0.0
    while time.monotonic() < deadline:
        try:
            with open(path, "r") as handle:
                tokens = handle.readline().strip().split()
                snapshot_stat = os.fstat(handle.fileno())
            values = {}
            for token in tokens:
                key, separator, value = token.partition("=")
                if separator:
                    values[key] = float(value)
            required = {"stamp", "x", "y", "z", "qx", "qy", "qz", "qw"}
            if not required.issubset(values):
                raise ValueError("incomplete snapshot")
            signature = (snapshot_stat.st_mtime_ns, values["stamp"])
            if signature != last_signature:
                now = time.monotonic()
                local_gap = 0.0
                if last_update_time is None:
                    consecutive_updates = 1
                    local_gap_max = 0.0
                else:
                    local_gap = now - last_update_time
                    if local_gap <= max_local_gap:
                        consecutive_updates += 1
                        local_gap_max = max(local_gap_max, local_gap)
                    else:
                        consecutive_updates = 1
                        local_gap_max = 0.0
                last_signature = signature
                last_update_time = now
                values["clock_offset"] = time.time() - values["stamp"]
                values["local_gap"] = local_gap
                values["local_gap_max"] = local_gap_max
                values["local_updates"] = consecutive_updates
                latest = values
                if consecutive_updates >= min_updates:
                    return values
        except (OSError, ValueError):
            pass
        time.sleep(0.05)
    if latest is not None and last_update_time is not None:
        latest["local_silence"] = time.monotonic() - last_update_time
    return latest


def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument("--route-file", required=True)
    parser.add_argument(
        "--odom-snapshot",
        default="/dev/shm/go2_fastlio_latest_odom.txt",
    )
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument("--max-distance", type=float, default=0.8)
    parser.add_argument("--max-yaw-error", type=float, default=0.9)
    parser.add_argument(
        "--max-odom-age",
        dest="max_local_gap",
        type=float,
        default=0.35,
        help="maximum local gap between consecutive odometry snapshot updates",
    )
    parser.add_argument("--min-odom-updates", type=int, default=3)
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)
    try:
        route_x, route_y, route_yaw = read_route_start(args.route_file)
    except Exception as exc:  # noqa: BLE001
        print("ROUTE_START_CHECK_ERROR %s" % exc, file=sys.stderr)
        return 4

    odom = collect_odom_snapshot(
        args.odom_snapshot,
        args.timeout,
        args.max_local_gap,
        args.min_odom_updates,
    )

    if odom is None:
        print(
            "ROUTE_START_CHECK_NO_ODOM snapshot=%s timeout=%.1f"
            % (args.odom_snapshot, args.timeout),
            file=sys.stderr,
        )
        return 3

    current_x = odom["x"]
    current_y = odom["y"]
    current_yaw = yaw_from_quaternion(
        odom["qx"],
        odom["qy"],
        odom["qz"],
        odom["qw"],
    )
    odom_stamp = odom["stamp"]
    clock_offset = odom["clock_offset"]
    local_gap_max = odom["local_gap_max"]
    local_updates = int(odom["local_updates"])
    finite_values = (
        current_x,
        current_y,
        odom["z"],
        odom["qx"],
        odom["qy"],
        odom["qz"],
        odom["qw"],
        current_yaw,
        odom_stamp,
        clock_offset,
        local_gap_max,
    )
    if not all(math.isfinite(value) for value in finite_values):
        print("ROUTE_START_CHECK_INVALID_ODOM non_finite_pose_or_stamp", file=sys.stderr)
        return 5
    if local_updates < args.min_odom_updates:
        print(
            "ROUTE_START_CHECK_STALE_ODOM local_updates=%d/%d "
            "local_silence_ms=%.1f max_local_gap_ms=%.1f "
            "stamp_offset_ms=%.1f"
            % (
                local_updates,
                args.min_odom_updates,
                odom.get("local_silence", 0.0) * 1000.0,
                args.max_local_gap * 1000.0,
                clock_offset * 1000.0,
            ),
            file=sys.stderr,
        )
        return 6
    distance = math.hypot(current_x - route_x, current_y - route_y)
    yaw_error = normalize_angle(current_yaw - route_yaw)

    message = (
        "current=(%.3f,%.3f,%.3f) route_start=(%.3f,%.3f,%.3f) "
        "distance=%.3f/%.3f yaw_error=%.3f/%.3f "
        "local_updates=%d/%d local_gap_max_ms=%.1f stamp_offset_ms=%.1f"
        % (
            current_x,
            current_y,
            current_yaw,
            route_x,
            route_y,
            route_yaw,
            distance,
            args.max_distance,
            yaw_error,
            args.max_yaw_error,
            local_updates,
            args.min_odom_updates,
            local_gap_max * 1000.0,
            clock_offset * 1000.0,
        )
    )
    if distance > args.max_distance or abs(yaw_error) > args.max_yaw_error:
        print("ROUTE_START_NOT_ALIGNED " + message, file=sys.stderr)
        return 2

    print("ROUTE_START_ALIGNED " + message)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

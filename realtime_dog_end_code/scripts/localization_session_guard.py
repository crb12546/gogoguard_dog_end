#!/usr/bin/env python3
"""Abort an active operation when its FAST-LIO coordinate session changes."""

import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path

from manual_route_anchor import (
    current_fastlio_session,
    read_odom_snapshot,
    sessions_equal,
)


WS = os.environ.get("GO2_WS", "/home/unitree/go2_fastlio_ws")


def append_line(path, text):
    if not path:
        return
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    with open(str(output), "a") as handle:
        handle.write(text.rstrip() + "\n")
        handle.flush()


def shell(command, timeout=12):
    try:
        return subprocess.run(
            ["bash", "-lc", command],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=timeout,
        )
    except Exception as exc:  # noqa: BLE001
        return subprocess.CompletedProcess(
            ["bash", "-lc", command],
            255,
            stdout="",
            stderr=repr(exc),
        )


def invalidate_output(path, reason):
    if not path:
        return ""
    source = Path(path)
    suffix = time.strftime(".invalid.%Y%m%d_%H%M%S")
    invalid = Path(str(source) + suffix)
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline and not source.exists():
        time.sleep(0.10)
    if source.exists():
        source.replace(invalid)
    append_line(str(invalid) + ".failure.txt", reason)
    return str(invalid)


def stop_patrol(enable_file=""):
    if enable_file:
        try:
            os.unlink(enable_file)
        except OSError:
            pass
    command = r'''
rm -f /home/unitree/go2_fastlio_ws/patrol_logs/run/patrol_video.active /home/unitree/go2_fastlio_ws/patrol_logs/run/video.run
pkill -TERM -f '[g]o2_saas_agent.py video-loop' 2>/dev/null || true
pkill -TERM -f '[w]aypoint_follower|[u]nitree_safe_cmd_node|[u]nitree_cmd_node|[c]md_vel_udp_sender|[g]o2_sdk2_udp_receiver|[p]atrol_performance_monitor.py|[g]o2_experiment_telemetry.py' 2>/dev/null || true
pkill -INT -f '[r]os2 bag record.*/patrol_logs/runs/' 2>/dev/null || true
sleep 1
pkill -KILL -f '[w]aypoint_follower|[u]nitree_safe_cmd_node|[u]nitree_cmd_node|[c]md_vel_udp_sender|[g]o2_sdk2_udp_receiver|[p]atrol_performance_monitor.py|[g]o2_experiment_telemetry.py' 2>/dev/null || true
ros2 topic pub -1 /api/sport/request unitree_api/msg/Request "{header: {identity: {id: 9999, api_id: 1003}, lease: {id: 0}, policy: {priority: 0, noreply: false}}, parameter: '{}', binary: []}" >/dev/null 2>&1 || true
sdk_if=${GO2_SDK_IF:-eth0}
''' + "%s/build/go2_cmd_vel_bridge/go2_sdk2_motion_probe" % WS + r''' --iface "$sdk_if" stop >/dev/null 2>&1 || true
'''
    return shell(command, timeout=12)


def stop_recording(mode):
    if mode == "recorder":
        result = shell(
            "p=$(ps -eo pid=,comm=,args= | "
            "awk '$2 != \"bash\" && $2 != \"sh\" && "
            "($2 == \"route_recorder\" || "
            "($2 == \"ros2\" && $0 ~ /[r]oute_recorder/)) "
            "{print $1}' | tr '\\n' ' ' || true); "
            "[ -n \"$p\" ] && kill -INT $p 2>/dev/null || true; "
            "sleep 2; "
            "p=$(ps -eo pid=,comm=,args= | "
            "awk '$2 != \"bash\" && $2 != \"sh\" && "
            "($2 == \"route_recorder\" || "
            "($2 == \"ros2\" && $0 ~ /[r]oute_recorder/)) "
            "{print $1}' | tr '\\n' ' ' || true); "
            "[ -n \"$p\" ] && kill -TERM $p 2>/dev/null || true",
            timeout=6,
        )
    else:
        result = shell(
            "pkill -TERM -f '[g]o2map_capture' 2>/dev/null || true; sleep 3; "
            "pkill -KILL -f '[g]o2map_capture' 2>/dev/null || true",
            timeout=7,
        )
    return result


def abort_operation(args, reason):
    try:
        os.unlink(args.active_file)
    except OSError:
        pass
    result = (
        stop_patrol(args.enable_file)
        if args.mode == "patrol"
        else stop_recording(args.mode)
    )
    invalid_path = invalidate_output(args.invalidate_output, reason)
    timestamp = time.strftime("%Y-%m-%dT%H:%M:%S%z")
    event = (
        "LOCALIZATION_SESSION_ABORTED mode=%s at=%s reason=%s rc=%s"
        % (args.mode, timestamp, reason, result.returncode)
    )
    if invalid_path:
        event += " invalid_output=%s" % invalid_path
    append_line(args.event_log, event)
    if args.manifest:
        append_line(args.manifest, "aborted_at=%s" % timestamp)
        append_line(args.manifest, "abort_reason=localization_session_changed")
        append_line(args.manifest, "abort_detail=%s" % reason.replace("\n", " "))
        # Keep current_patrol_run pointing at this aborted experiment.  The
        # regular stop/finalize path needs that pointer to capture the ending
        # snapshot, close the bag, slice base logs and build the audit report.
        # It is removed only after finalization is complete.
    print(event, file=sys.stderr, flush=True)


def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument("--metadata", required=True)
    parser.add_argument("--active-file", required=True)
    parser.add_argument(
        "--mode",
        choices=("patrol", "recorder", "pcd"),
        required=True,
    )
    parser.add_argument("--enable-file", default="")
    parser.add_argument("--manifest", default="")
    parser.add_argument("--event-log", default="")
    parser.add_argument("--invalidate-output", default="")
    parser.add_argument(
        "--odom-snapshot",
        default="/dev/shm/go2_fastlio_latest_odom.txt",
    )
    parser.add_argument("--max-odom-silence", type=float, default=2.0)
    parser.add_argument("--interval", type=float, default=0.20)
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)
    try:
        with open(args.metadata, "r") as handle:
            metadata = json.load(handle)
        expected = metadata["localization_session"]
    except Exception as exc:  # noqa: BLE001
        print("SESSION_GUARD_METADATA_ERROR %s" % exc, file=sys.stderr)
        return 3

    if not os.path.exists(args.active_file):
        print("SESSION_GUARD_NOT_ACTIVE file=%s" % args.active_file)
        return 0

    last_snapshot_signature = None
    last_snapshot_at = time.monotonic()
    last_odom_stamp = None
    print(
        "SESSION_GUARD_STARTED mode=%s session=%s:%s active=%s"
        % (
            args.mode,
            expected.get("pid"),
            expected.get("start_ticks"),
            args.active_file,
        ),
        flush=True,
    )

    while os.path.exists(args.active_file):
        reason = ""
        try:
            actual = current_fastlio_session()
            if not sessions_equal(expected, actual):
                reason = "fastlio_session_changed expected=%s actual=%s" % (
                    json.dumps(expected, sort_keys=True),
                    json.dumps(actual, sort_keys=True),
                )
        except Exception as exc:  # noqa: BLE001
            reason = "fastlio_session_unavailable:%s" % exc

        if not reason:
            try:
                odom = read_odom_snapshot(args.odom_snapshot)
                signature = odom["snapshot_signature"]
                if signature != last_snapshot_signature:
                    if (
                        last_odom_stamp is not None
                        and odom["stamp"] + 1e-6 < last_odom_stamp
                    ):
                        reason = (
                            "odometry_stamp_moved_backward previous=%.9f current=%.9f"
                            % (last_odom_stamp, odom["stamp"])
                        )
                    last_snapshot_signature = signature
                    last_snapshot_at = time.monotonic()
                    last_odom_stamp = odom["stamp"]
            except (OSError, ValueError):
                pass

        if (
            not reason
            and time.monotonic() - last_snapshot_at > args.max_odom_silence
        ):
            reason = "odometry_snapshot_silent_for_%.2fs" % (
                time.monotonic() - last_snapshot_at
            )

        if reason:
            abort_operation(args, reason)
            return 10
        time.sleep(max(0.05, args.interval))

    print("SESSION_GUARD_STOPPED mode=%s active_file_removed" % args.mode)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

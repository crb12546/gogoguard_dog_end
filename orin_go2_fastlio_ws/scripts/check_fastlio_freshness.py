#!/usr/bin/env python3
"""Require consecutive, current FAST-LIO output frames before declaring ready."""

import argparse
import math
import sys
import time


def read_snapshot_stamp(path):
    with open(path, "r") as handle:
        tokens = handle.readline().strip().split()

    for token in tokens:
        key, separator, value = token.partition("=")
        if key == "stamp" and separator:
            stamp = float(value)
            if math.isfinite(stamp):
                return stamp
            break
    raise ValueError("snapshot has no finite stamp")


def build_parser():
    parser = argparse.ArgumentParser(
        description=(
            "Wait for consecutive unique FAST-LIO snapshots whose sensor "
            "timestamps are close to the host wall clock."
        )
    )
    parser.add_argument(
        "--snapshot",
        default="/dev/shm/go2_fastlio_latest_odom.txt",
    )
    parser.add_argument("--frames", type=int, default=10)
    parser.add_argument("--max-age-ms", type=float, default=250.0)
    parser.add_argument("--min-age-ms", type=float, default=-100.0)
    parser.add_argument("--min-frame-dt-ms", type=float, default=50.0)
    parser.add_argument("--max-frame-dt-ms", type=float, default=150.0)
    parser.add_argument("--poll-ms", type=float, default=20.0)
    parser.add_argument("--timeout", type=float, default=120.0)
    return parser


def validate_args(parser, args):
    if args.frames < 1:
        parser.error("--frames must be at least 1")
    if args.timeout <= 0:
        parser.error("--timeout must be positive")
    if args.poll_ms <= 0:
        parser.error("--poll-ms must be positive")
    if args.min_age_ms > args.max_age_ms:
        parser.error("--min-age-ms must not exceed --max-age-ms")
    if args.min_frame_dt_ms <= 0:
        parser.error("--min-frame-dt-ms must be positive")
    if args.min_frame_dt_ms > args.max_frame_dt_ms:
        parser.error("--min-frame-dt-ms must not exceed --max-frame-dt-ms")


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    validate_args(parser, args)

    deadline = time.monotonic() + args.timeout
    previous_stamp = None
    last_seen_stamp = None
    last_age_ms = None
    consecutive = 0

    while time.monotonic() < deadline:
        try:
            stamp = read_snapshot_stamp(args.snapshot)
        except (OSError, ValueError):
            time.sleep(args.poll_ms / 1000.0)
            continue

        if stamp == last_seen_stamp:
            time.sleep(args.poll_ms / 1000.0)
            continue

        last_seen_stamp = stamp
        age_ms = (time.time() - stamp) * 1000.0
        last_age_ms = age_ms
        frame_dt_ms = None
        sequential = previous_stamp is None
        if previous_stamp is not None:
            frame_dt_ms = (stamp - previous_stamp) * 1000.0
            sequential = (
                args.min_frame_dt_ms
                <= frame_dt_ms
                <= args.max_frame_dt_ms
            )

        fresh = args.min_age_ms <= age_ms <= args.max_age_ms
        if fresh:
            if previous_stamp is None or not sequential:
                consecutive = 1
            else:
                consecutive += 1
        else:
            consecutive = 0

        dt_text = "first" if frame_dt_ms is None else "%.3f" % frame_dt_ms
        print(
            "FASTLIO_FRESH_SAMPLE "
            "stamp=%.6f age_ms=%.3f frame_dt_ms=%s "
            "fresh=%d sequential=%d consecutive=%d/%d"
            % (
                stamp,
                age_ms,
                dt_text,
                int(fresh),
                int(sequential),
                consecutive,
                args.frames,
            ),
            flush=True,
        )

        if consecutive >= args.frames:
            print(
                "FASTLIO_FRESH_READY "
                "frames=%d max_age_ms=%.3f "
                "frame_dt_ms_range=%.3f..%.3f"
                % (
                    args.frames,
                    args.max_age_ms,
                    args.min_frame_dt_ms,
                    args.max_frame_dt_ms,
                ),
                flush=True,
            )
            return 0

        previous_stamp = stamp
        time.sleep(args.poll_ms / 1000.0)

    age_text = "unknown" if last_age_ms is None else "%.3f" % last_age_ms
    print(
        "FASTLIO_FRESH_TIMEOUT "
        "timeout_s=%.3f consecutive=%d/%d last_age_ms=%s snapshot=%s"
        % (
            args.timeout,
            consecutive,
            args.frames,
            age_text,
            args.snapshot,
        ),
        file=sys.stderr,
        flush=True,
    )
    return 1


if __name__ == "__main__":
    sys.exit(main())

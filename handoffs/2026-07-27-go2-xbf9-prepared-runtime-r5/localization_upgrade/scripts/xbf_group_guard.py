#!/usr/bin/env python3
"""Validate or signal only a process group belonging to one XBF run."""

from __future__ import annotations

import os
from pathlib import Path
import signal
import sys
from typing import Iterable


MARKER = b"GO2_XBF_RUN_ID="


def numeric_pids() -> Iterable[int]:
    for child in Path("/proc").iterdir():
        if child.name.isdigit():
            yield int(child.name)


def marked_group_members(pgid: int, run_id: str) -> list[int]:
    expected = MARKER + run_id.encode("ascii")
    result: list[int] = []
    for pid in numeric_pids():
        try:
            if os.getpgid(pid) != pgid:
                continue
            values = (Path("/proc") / str(pid) / "environ").read_bytes().split(b"\0")
            if expected in values:
                result.append(pid)
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
    return result


def main() -> int:
    if len(sys.argv) not in (4, 5):
        print(
            "usage: xbf_group_guard.py status PGID RUN_ID | "
            "signal PGID RUN_ID SIGNAL",
            file=sys.stderr,
        )
        return 64
    operation, pgid_text, run_id = sys.argv[1:4]
    if not pgid_text.isdigit() or int(pgid_text) <= 1:
        print("invalid PGID", file=sys.stderr)
        return 65
    if not run_id or len(run_id) > 128 or not run_id.isascii():
        print("invalid run id", file=sys.stderr)
        return 65
    pgid = int(pgid_text)
    members = marked_group_members(pgid, run_id)
    if operation == "status":
        if not members:
            return 1
        print(" ".join(str(pid) for pid in sorted(members)))
        return 0
    if operation != "signal" or len(sys.argv) != 5:
        print("invalid operation", file=sys.stderr)
        return 64
    if not members:
        return 1
    signal_name = sys.argv[4]
    try:
        signal_number = getattr(signal, signal_name)
    except AttributeError:
        print(f"unsupported signal: {signal_name}", file=sys.stderr)
        return 65
    try:
        os.killpg(pgid, signal_number)
    except ProcessLookupError:
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

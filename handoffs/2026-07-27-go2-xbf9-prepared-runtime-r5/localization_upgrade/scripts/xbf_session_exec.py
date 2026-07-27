#!/usr/bin/env python3
"""Exec one patrol component as an exact new Linux session leader.

Unlike ``setsid command &``, the PID observed by the parent shell is retained
through ``exec``.  The parent can therefore wait until PID == PGID == SID
before recording the component and can never mistake the transient setsid
wrapper for the real process group.
"""

from __future__ import annotations

import os
import sys


def main() -> int:
    if len(sys.argv) < 2:
        print("usage: xbf_session_exec.py COMMAND [ARG ...]", file=sys.stderr)
        return 64
    run_id = os.environ.get("GO2_XBF_RUN_ID", "")
    if not run_id or len(run_id) > 128:
        print("GO2_XBF_RUN_ID is missing or invalid", file=sys.stderr)
        return 65
    try:
        os.setsid()
    except OSError as error:
        print(f"cannot create patrol session: {error}", file=sys.stderr)
        return 66
    os.execvpe(sys.argv[1], sys.argv[1:], os.environ)
    return 127


if __name__ == "__main__":
    raise SystemExit(main())

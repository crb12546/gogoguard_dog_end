#!/usr/bin/env bash
set -euo pipefail

if [[ "$(uname -s)" != "Linux" || ! -d /proc ]]; then
  echo "SKIP: process-group lifecycle test requires Linux /proc"
  exit 77
fi

root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
session_exec="${root}/scripts/xbf_session_exec.py"
group_guard="${root}/scripts/xbf_group_guard.py"
run_id="xbf-lifecycle-regression"

GO2_XBF_RUN_ID="${run_id}" "${session_exec}" sleep 60 &
pid=$!
cleanup() {
  "${group_guard}" signal "${pid}" "${run_id}" SIGKILL >/dev/null 2>&1 || true
  wait "${pid}" 2>/dev/null || true
}
trap cleanup EXIT

pair=""
for _attempt in $(seq 1 50); do
  pair="$(
    python3 - "${pid}" <<'PY' 2>/dev/null || true
import os
import sys
pid = int(sys.argv[1])
print(os.getpgid(pid), os.getsid(pid))
PY
  )"
  [[ "${pair}" == "${pid} ${pid}" ]] && break
  sleep 0.05
done
[[ "${pair}" == "${pid} ${pid}" ]] || {
  echo "component did not establish PID=PGID=SID" >&2
  exit 1
}

"${group_guard}" status "${pid}" "${run_id}" >/dev/null
if "${group_guard}" status "${pid}" "xbf-wrong-run" >/dev/null 2>&1; then
  echo "group guard accepted the wrong run id" >&2
  exit 1
fi
"${group_guard}" signal "${pid}" "${run_id}" SIGTERM >/dev/null
wait "${pid}" 2>/dev/null || true
if "${group_guard}" status "${pid}" "${run_id}" >/dev/null 2>&1; then
  echo "group remained alive after exact termination" >&2
  exit 1
fi
trap - EXIT
echo "process group lifecycle regression passed"

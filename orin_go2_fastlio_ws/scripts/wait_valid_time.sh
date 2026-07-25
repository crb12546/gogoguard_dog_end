#!/usr/bin/env bash
set -euo pipefail

TIMEOUT_SECONDS=${1:-${GO2_TIME_WAIT_TIMEOUT:-0}}
MIN_VALID_EPOCH=${GO2_MIN_VALID_EPOCH:-1704067200}
SLEEP_SECONDS=${GO2_TIME_WAIT_SLEEP:-2}

is_valid_time() {
  local now
  now=$(date +%s)
  [ "$now" -ge "$MIN_VALID_EPOCH" ]
}

elapsed=0
while ! is_valid_time; do
  echo "[wait_valid_time] wall clock not synced yet: $(date '+%Y-%m-%d %H:%M:%S %Z')" >&2
  if [ "$TIMEOUT_SECONDS" -gt 0 ] && [ "$elapsed" -ge "$TIMEOUT_SECONDS" ]; then
    echo "[wait_valid_time] timeout after ${elapsed}s" >&2
    exit 75
  fi
  sleep "$SLEEP_SECONDS"
  elapsed=$((elapsed + SLEEP_SECONDS))
done

echo "[wait_valid_time] wall clock ready: $(date '+%Y-%m-%d %H:%M:%S %Z')" >&2

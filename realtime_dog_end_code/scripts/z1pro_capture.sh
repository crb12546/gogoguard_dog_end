#!/usr/bin/env bash
set -euo pipefail

MODE=${1:-record}
SEGMENT_SECONDS=${2:-20}
RTSP_URL=${Z1PRO_RTSP_URL:-rtsp://192.168.144.108/}
OUT_DIR=${Z1PRO_OUT_DIR:-/home/unitree/go2_fastlio_ws/patrol_logs/videos}
WAIT_VALID_TIME=${Z1PRO_WAIT_VALID_TIME:-1}
WAIT_VALID_TIME_SCRIPT=${GO2_WAIT_VALID_TIME_SCRIPT:-/home/unitree/go2_fastlio_ws/scripts/wait_valid_time.sh}
TIME_WAIT_TIMEOUT=${GO2_TIME_WAIT_TIMEOUT:-180}

mkdir -p "$OUT_DIR"

gst_pid=""
tmp=""

stop_gst() {
  local pid=$1
  if [ -z "$pid" ] || ! kill -0 "$pid" 2>/dev/null; then
    return 0
  fi

  kill -INT "$pid" 2>/dev/null || true
  for _ in 1 2 3 4 5 6 7 8 9 10; do
    if ! kill -0 "$pid" 2>/dev/null; then
      wait "$pid" 2>/dev/null || true
      return 0
    fi
    sleep 0.5
  done

  kill -TERM "$pid" 2>/dev/null || true
  sleep 2
  if kill -0 "$pid" 2>/dev/null; then
    kill -KILL "$pid" 2>/dev/null || true
  fi
  wait "$pid" 2>/dev/null || true
}

abort_record() {
  stop_gst "${gst_pid:-}"
  if [ -n "${tmp:-}" ]; then
    rm -f "$tmp"
  fi
  exit 130
}

wait_valid_time() {
  if [ "$WAIT_VALID_TIME" != "1" ]; then
    return 0
  fi
  if [ -x "$WAIT_VALID_TIME_SCRIPT" ]; then
    "$WAIT_VALID_TIME_SCRIPT" "$TIME_WAIT_TIMEOUT"
    return $?
  fi
  local waited=0
  local min_epoch=${GO2_MIN_VALID_EPOCH:-1704067200}
  while [ "$(date +%s)" -lt "$min_epoch" ]; do
    echo "[z1pro_capture] waiting for valid wall clock: $(date '+%Y-%m-%d %H:%M:%S %Z')" >&2
    if [ "$TIME_WAIT_TIMEOUT" -gt 0 ] && [ "$waited" -ge "$TIME_WAIT_TIMEOUT" ]; then
      echo "[z1pro_capture] valid wall clock timeout after ${waited}s" >&2
      return 75
    fi
    sleep 2
    waited=$((waited + 2))
  done
}

timestamp() {
  date +%Y%m%d_%H%M%S
}

case "$MODE" in
  record)
    wait_valid_time
    ts=$(timestamp)
    out="$OUT_DIR/z1pro_${ts}_${SEGMENT_SECONDS}s.mp4"
    tmp="$OUT_DIR/.z1pro_${ts}_${SEGMENT_SECONDS}s.tmp.mp4"
    trap abort_record INT TERM
    echo "$out"
    gst-launch-1.0 -e -q \
      rtspsrc location="$RTSP_URL" protocols=tcp latency=100 \
      ! rtph264depay ! h264parse config-interval=-1 ! mp4mux faststart=true ! filesink location="$tmp" &
    gst_pid=$!
    sleep "$SEGMENT_SECONDS"
    stop_gst "$gst_pid"
    trap - INT TERM
    if [ -s "$tmp" ]; then
      mv "$tmp" "$out"
    fi
    rm -f "$tmp"
    test -s "$out"
    ;;
  snapshot)
    wait_valid_time
    out="$OUT_DIR/z1pro_snapshot_$(timestamp).jpg"
    echo "$out"
    timeout -s INT 8 gst-launch-1.0 -q \
      rtspsrc location="$RTSP_URL" protocols=tcp latency=100 \
      ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! jpegenc \
      ! multifilesink max-files=1 location="$out" || true
    test -s "$out"
    ;;
  probe)
    python3 - <<PY
import socket
host = "192.168.144.108"
req = ("OPTIONS rtsp://%s/ RTSP/1.0\\r\\nCSeq: 1\\r\\nUser-Agent: z1pro-probe\\r\\n\\r\\n" % host).encode()
s = socket.create_connection((host, 554), timeout=5)
s.sendall(req)
print(s.recv(4096).decode(errors="replace"))
s.close()
PY
    ;;
  *)
    echo "usage: $0 [record [seconds]|snapshot|probe]" >&2
    exit 2
    ;;
esac

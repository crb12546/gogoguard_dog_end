#!/usr/bin/env bash
set -euo pipefail

MODE=${1:-record}
SECONDS=${2:-20}
RTSP_URL=${Z1PRO_RTSP_URL:-rtsp://192.168.144.108/}
OUT_DIR=${Z1PRO_OUT_DIR:-/home/unitree/go2_fastlio_ws/patrol_logs/videos}

mkdir -p "$OUT_DIR"

timestamp() {
  date +%Y%m%d_%H%M%S
}

case "$MODE" in
  record)
    out="$OUT_DIR/z1pro_$(timestamp)_${SECONDS}s.mp4"
    tmp="$OUT_DIR/.z1pro_$(timestamp)_${SECONDS}s.tmp.mp4"
    echo "$out"
    gst-launch-1.0 -e -q \
      rtspsrc location="$RTSP_URL" protocols=tcp latency=100 \
      ! rtph264depay ! h264parse config-interval=-1 ! mp4mux faststart=true ! filesink location="$tmp" &
    pid=$!
    sleep "$SECONDS"
    kill -INT "$pid" 2>/dev/null || true
    wait "$pid" 2>/dev/null || true
    if [ -s "$tmp" ]; then
      mv "$tmp" "$out"
    fi
    rm -f "$tmp"
    test -s "$out"
    ;;
  snapshot)
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
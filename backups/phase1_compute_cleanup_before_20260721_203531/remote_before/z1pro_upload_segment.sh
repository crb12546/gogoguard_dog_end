#!/usr/bin/env bash
set -euo pipefail

SEGMENT_SECONDS=${1:-20}
ROBOT_ID=${GO2_ROBOT_ID:-go2-tju-01}
BACKEND_BASE=${GO2_BACKEND_BASE:-https://39.96.37.187/api/v1}
UPLOAD=${Z1PRO_UPLOAD:-0}
CAPTURE_SCRIPT=${Z1PRO_CAPTURE_SCRIPT:-/home/unitree/go2_fastlio_ws/scripts/z1pro_capture.sh}
SAAS_AGENT=${GO2_SAAS_AGENT:-/home/unitree/go2_fastlio_ws/scripts/go2_saas_agent.py}
WAIT_VALID_TIME=${Z1PRO_WAIT_VALID_TIME:-1}
WAIT_VALID_TIME_SCRIPT=${GO2_WAIT_VALID_TIME_SCRIPT:-/home/unitree/go2_fastlio_ws/scripts/wait_valid_time.sh}
TIME_WAIT_TIMEOUT=${GO2_TIME_WAIT_TIMEOUT:-180}

if ! [[ "$SEGMENT_SECONDS" =~ ^[0-9]+$ ]] || [ "$SEGMENT_SECONDS" -le 0 ]; then
  echo "usage: $0 [seconds]" >&2
  exit 2
fi

recorded_at() {
  date '+%Y-%m-%d %H:%M:%S'
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
    echo "[z1pro_upload_segment] waiting for valid wall clock: $(date '+%Y-%m-%d %H:%M:%S %Z')" >&2
    if [ "$TIME_WAIT_TIMEOUT" -gt 0 ] && [ "$waited" -ge "$TIME_WAIT_TIMEOUT" ]; then
      echo "[z1pro_upload_segment] valid wall clock timeout after ${waited}s" >&2
      exit 75
    fi
    sleep 2
    waited=$((waited + 2))
  done
}

file_size() {
  stat -c '%s' "$1"
}

wait_valid_time
video_file=$("$CAPTURE_SCRIPT" record "$SEGMENT_SECONDS" | tail -1)
if [ ! -s "$video_file" ]; then
  echo "record failed or empty file: $video_file" >&2
  exit 1
fi

end_time=$(recorded_at)
name=$(basename "$video_file")
size=$(file_size "$video_file")
meta=$(printf '{"segmentSeconds":%s,"recordedAt":"%s","source":"z1pro","rtspUrl":"rtsp://192.168.144.108/"}' "$SEGMENT_SECONDS" "$end_time")
url="${BACKEND_BASE%/}/robot/video/upload"

echo "video_file=$video_file"
echo "robotId=$ROBOT_ID"
echo "upload_url=$url"
echo "meta=$meta"

if [ "$UPLOAD" != "1" ]; then
  echo "upload skipped; set Z1PRO_UPLOAD=1 to POST this segment"
  exit 0
fi

if [ -f "$SAAS_AGENT" ]; then
  if command -v ionice >/dev/null 2>&1; then
    exec nice -n "${Z1PRO_UPLOAD_NICE:-10}" ionice -c2 -n "${Z1PRO_UPLOAD_IONICE:-7}" python3 -u "$SAAS_AGENT" upload-once --video "$video_file" --upload --no-heartbeat
  fi
  exec nice -n "${Z1PRO_UPLOAD_NICE:-10}" python3 -u "$SAAS_AGENT" upload-once --video "$video_file" --upload --no-heartbeat
fi

echo "missing executable SaaS agent: $SAAS_AGENT" >&2
exit 1

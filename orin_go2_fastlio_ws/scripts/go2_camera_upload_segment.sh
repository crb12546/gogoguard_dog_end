#!/usr/bin/env bash
set -euo pipefail

WS=${GO2_WS:-/home/unitree/go2_fastlio_ws}
CONFIG_FILE=${GO2_CAMERA_CONFIG:-${WS}/config/camera.env}
SOURCE_OVERRIDE=${GO2_CAMERA_SOURCE:-}
if [ -f "$CONFIG_FILE" ]; then
  # shellcheck disable=SC1090
  set -a
  source "$CONFIG_FILE"
  set +a
fi
if [ -n "$SOURCE_OVERRIDE" ]; then
  GO2_CAMERA_SOURCE=$SOURCE_OVERRIDE
fi

SEGMENT_SECONDS=${1:-20}
ROBOT_ID=${GO2_ROBOT_ID:-go2-tju-01}
BACKEND_BASE=${GO2_BACKEND_BASE:-https://39.96.37.187/api/v1}
UPLOAD=${GO2_CAMERA_UPLOAD:-${Z1PRO_UPLOAD:-0}}
CAPTURE_SCRIPT=${GO2_CAMERA_CAPTURE_SCRIPT:-${WS}/scripts/go2_camera_capture.sh}
SAAS_AGENT=${GO2_SAAS_AGENT:-${WS}/scripts/go2_saas_agent.py}
WAIT_VALID_TIME_SCRIPT=${GO2_WAIT_VALID_TIME_SCRIPT:-${WS}/scripts/wait_valid_time.sh}
TIME_WAIT_TIMEOUT=${GO2_TIME_WAIT_TIMEOUT:-180}

if ! [[ "$SEGMENT_SECONDS" =~ ^[0-9]+$ ]] || [ "$SEGMENT_SECONDS" -le 0 ]; then
  echo "usage: $0 [seconds]" >&2
  exit 2
fi

if [ "${GO2_CAMERA_WAIT_VALID_TIME:-1}" = "1" ] && [ -x "$WAIT_VALID_TIME_SCRIPT" ]; then
  "$WAIT_VALID_TIME_SCRIPT" "$TIME_WAIT_TIMEOUT"
fi

video_file=$("$CAPTURE_SCRIPT" record "$SEGMENT_SECONDS" | tail -1)
if [ ! -s "$video_file" ]; then
  echo "record failed or empty file: $video_file" >&2
  exit 1
fi

end_time=$(date '+%Y-%m-%d %H:%M:%S')
name=$(basename "$video_file")
size=$(stat -c '%s' "$video_file")
source_name=${GO2_CAMERA_SOURCE:-z1pro}

echo "video_file=$video_file"
echo "robotId=$ROBOT_ID"
echo "upload_url=${BACKEND_BASE%/}/robot/video/upload"
echo "meta={\"segmentSeconds\":${SEGMENT_SECONDS},\"recordedAt\":\"${end_time}\",\"source\":\"${source_name}\"}"

if [ "$UPLOAD" != "1" ]; then
  echo "upload skipped; set GO2_CAMERA_UPLOAD=1 to upload this segment"
  exit 0
fi

if [ ! -f "$SAAS_AGENT" ]; then
  echo "missing SaaS agent: $SAAS_AGENT" >&2
  exit 1
fi

nice_level=${GO2_CAMERA_UPLOAD_NICE:-${Z1PRO_UPLOAD_NICE:-10}}
ionice_level=${GO2_CAMERA_UPLOAD_IONICE:-${Z1PRO_UPLOAD_IONICE:-7}}
if command -v ionice >/dev/null 2>&1; then
  exec nice -n "$nice_level" ionice -c2 -n "$ionice_level" \
    python3 -u "$SAAS_AGENT" upload-once --video "$video_file" --upload --no-heartbeat
fi
exec nice -n "$nice_level" \
  python3 -u "$SAAS_AGENT" upload-once --video "$video_file" --upload --no-heartbeat

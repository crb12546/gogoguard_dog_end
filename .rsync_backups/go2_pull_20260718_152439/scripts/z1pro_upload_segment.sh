#!/usr/bin/env bash
set -euo pipefail

SEGMENT_SECONDS=${1:-20}
ROBOT_ID=${GO2_ROBOT_ID:-go2-tju-01}
BACKEND_BASE=${GO2_BACKEND_BASE:-https://39.96.37.187/api/v1}
UPLOAD=${Z1PRO_UPLOAD:-0}
CAPTURE_SCRIPT=${Z1PRO_CAPTURE_SCRIPT:-/home/unitree/go2_fastlio_ws/scripts/z1pro_capture.sh}

if ! [[ "$SEGMENT_SECONDS" =~ ^[0-9]+$ ]] || [ "$SEGMENT_SECONDS" -le 0 ]; then
  echo "usage: $0 [seconds]" >&2
  exit 2
fi

recorded_at() {
  date '+%Y-%m-%d %H:%M:%S'
}

file_size() {
  stat -c '%s' "$1"
}

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

curl -4 -k -sS -i \
  --connect-timeout 10 \
  --max-time 120 \
  -F "file=@${video_file};type=video/mp4" \
  -F "robotId=${ROBOT_ID}" \
  -F "fileName=${name}" \
  -F "fileSize=${size}" \
  -F "time=${end_time}" \
  -F "meta=${meta}" \
  "$url"
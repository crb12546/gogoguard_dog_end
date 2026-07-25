#!/usr/bin/env bash
set -euo pipefail

# Backward-compatible entry point. Camera selection now belongs to the shared
# camera backend, so legacy callers also follow config/camera.env.
WS=${GO2_WS:-/home/unitree/go2_fastlio_ws}
exec "${GO2_CAMERA_UPLOAD_SCRIPT:-${WS}/scripts/go2_camera_upload_segment.sh}" "$@"

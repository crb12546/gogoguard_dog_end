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

CAMERA_SOURCE=${GO2_CAMERA_SOURCE:-z1pro}
PRESET=${1:-probe}
case "$CAMERA_SOURCE" in
  z1pro|z1|z-1pro)
    exec "${GO2_Z1PRO_PRESET_SCRIPT:-${WS}/scripts/z1pro_preset.sh}" "$PRESET"
    ;;
  unitree_builtin|unitree|builtin|go2)
    case "$PRESET" in
      probe)
        exec "${GO2_CAMERA_CAPTURE_SCRIPT:-${WS}/scripts/go2_camera_capture.sh}" probe
        ;;
      front|home)
        echo "CAMERA_PRESET_OK source=unitree_builtin preset=${PRESET} fixed_front=true"
        ;;
      down|up|left|right|rear)
        echo "CAMERA_PRESET_UNSUPPORTED source=unitree_builtin preset=${PRESET} fixed_front=true" >&2
        exit 3
        ;;
      *)
        echo "usage: $0 [front|down|up|left|right|home|probe|rear]" >&2
        exit 2
        ;;
    esac
    ;;
  *)
    echo "unsupported GO2_CAMERA_SOURCE: $CAMERA_SOURCE" >&2
    exit 2
    ;;
esac

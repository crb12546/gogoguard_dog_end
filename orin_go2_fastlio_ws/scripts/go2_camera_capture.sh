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
case "$CAMERA_SOURCE" in
  unitree|builtin|go2)
    CAMERA_SOURCE=unitree_builtin
    ;;
  z1|z-1pro)
    CAMERA_SOURCE=z1pro
    ;;
esac

MODE=${1:-record}
SEGMENT_SECONDS=${2:-20}
OUT_DIR=${GO2_CAMERA_OUT_DIR:-${GO2_VIDEO_DIR:-${Z1PRO_OUT_DIR:-${WS}/patrol_logs/videos}}}
Z1PRO_CAPTURE=${GO2_Z1PRO_CAPTURE_SCRIPT:-${WS}/scripts/z1pro_capture.sh}
BUILTIN_BIN=${GO2_BUILTIN_CAMERA_BIN:-${WS}/bin/go2_builtin_camera_capture}
BUILTIN_INTERFACE=${GO2_BUILTIN_CAMERA_INTERFACE:-auto}
BUILTIN_MAC=${GO2_BUILTIN_CAMERA_MAC:-${GO2_WIRED_MAC:-4c:bb:47:ab:e4:c2}}
BUILTIN_FPS=${GO2_BUILTIN_CAMERA_FPS:-15}
BUILTIN_BITRATE=${GO2_BUILTIN_CAMERA_BITRATE:-4000000}
BUILTIN_ENCODER=${GO2_BUILTIN_CAMERA_ENCODER:-nvidia}
WAIT_VALID_TIME_SCRIPT=${GO2_WAIT_VALID_TIME_SCRIPT:-${WS}/scripts/wait_valid_time.sh}
TIME_WAIT_TIMEOUT=${GO2_TIME_WAIT_TIMEOUT:-180}

timestamp() {
  date +%Y%m%d_%H%M%S
}

wait_valid_time() {
  if [ "${GO2_CAMERA_WAIT_VALID_TIME:-1}" != "1" ]; then
    return 0
  fi
  if [ -x "$WAIT_VALID_TIME_SCRIPT" ]; then
    "$WAIT_VALID_TIME_SCRIPT" "$TIME_WAIT_TIMEOUT"
  fi
}

require_builtin_camera() {
  if [ ! -x "$BUILTIN_BIN" ]; then
    echo "missing built-in camera helper: $BUILTIN_BIN" >&2
    echo "build it with: ${WS}/scripts/build_go2_builtin_camera_capture.sh" >&2
    exit 1
  fi
}

resolve_builtin_interface() {
  local requested=${BUILTIN_INTERFACE:-auto}
  local iface address target

  if [ "$requested" != "auto" ] && [ -d "/sys/class/net/$requested" ]; then
    BUILTIN_INTERFACE=$requested
    return 0
  fi
  if [ "$requested" != "auto" ]; then
    echo "configured camera interface '$requested' is absent; resolving by MAC" >&2
  fi

  if [ -n "${GO2_SDK_IF:-}" ] && [ -d "/sys/class/net/${GO2_SDK_IF}" ]; then
    BUILTIN_INTERFACE=$GO2_SDK_IF
    echo "BUILTIN_CAMERA_INTERFACE_RESOLVED interface=$BUILTIN_INTERFACE source=GO2_SDK_IF" >&2
    return 0
  fi

  target=$(printf '%s' "$BUILTIN_MAC" | tr 'A-Z' 'a-z')
  for iface in /sys/class/net/*; do
    [ -r "$iface/address" ] || continue
    address=$(tr 'A-Z' 'a-z' <"$iface/address" 2>/dev/null || true)
    if [ "$address" = "$target" ]; then
      BUILTIN_INTERFACE=$(basename "$iface")
      echo "BUILTIN_CAMERA_INTERFACE_RESOLVED interface=$BUILTIN_INTERFACE source=mac mac=$target" >&2
      return 0
    fi
  done

  if command -v ip >/dev/null 2>&1; then
    iface=$(ip -o -4 addr show 2>/dev/null \
      | awk '$4 ~ /^192\.168\.123\./ {print $2; exit}')
    if [ -n "$iface" ] && [ -d "/sys/class/net/$iface" ]; then
      BUILTIN_INTERFACE=$iface
      echo "BUILTIN_CAMERA_INTERFACE_RESOLVED interface=$BUILTIN_INTERFACE source=go2_subnet" >&2
      return 0
    fi
  fi

  echo "cannot resolve built-in camera interface (requested=$requested mac=$target)" >&2
  return 1
}

record_builtin() {
  if ! [[ "$SEGMENT_SECONDS" =~ ^[0-9]+$ ]] || [ "$SEGMENT_SECONDS" -le 0 ]; then
    echo "record seconds must be a positive integer" >&2
    exit 2
  fi
  if ! [[ "$BUILTIN_FPS" =~ ^[0-9]+$ ]] || [ "$BUILTIN_FPS" -le 0 ] || [ "$BUILTIN_FPS" -gt 30 ]; then
    echo "GO2_BUILTIN_CAMERA_FPS must be an integer from 1 to 30" >&2
    exit 2
  fi

  wait_valid_time
  mkdir -p "$OUT_DIR"
  local ts out tmp
  ts=$(timestamp)
  out="${OUT_DIR}/unitree_builtin_${ts}_${SEGMENT_SECONDS}s.mp4"
  tmp="${OUT_DIR}/.unitree_builtin_${ts}_${SEGMENT_SECONDS}s.tmp.mp4"
  trap 'rm -f "$tmp"' EXIT INT TERM

  case "$BUILTIN_ENCODER" in
    nvidia)
      "$BUILTIN_BIN" stream "$SEGMENT_SECONDS" "$BUILTIN_FPS" "$BUILTIN_INTERFACE" \
        | gst-launch-1.0 -e -q \
          fdsrc fd=0 do-timestamp=true \
          ! "image/jpeg,framerate=${BUILTIN_FPS}/1" \
          ! jpegparse \
          ! nvv4l2decoder mjpeg=1 \
          ! nvvidconv \
          ! 'video/x-raw(memory:NVMM),format=NV12' \
          ! nvv4l2h264enc bitrate="$BUILTIN_BITRATE" control-rate=1 \
            insert-sps-pps=true iframeinterval="$((BUILTIN_FPS * 2))" \
          ! h264parse config-interval=-1 \
          ! mp4mux faststart=true \
          ! filesink location="$tmp"
      ;;
    x264)
      "$BUILTIN_BIN" stream "$SEGMENT_SECONDS" "$BUILTIN_FPS" "$BUILTIN_INTERFACE" \
        | gst-launch-1.0 -e -q \
          fdsrc fd=0 do-timestamp=true \
          ! "image/jpeg,framerate=${BUILTIN_FPS}/1" \
          ! jpegparse \
          ! jpegdec \
          ! videoconvert \
          ! x264enc tune=zerolatency speed-preset=ultrafast bitrate="$((BUILTIN_BITRATE / 1000))" \
            key-int-max="$((BUILTIN_FPS * 2))" \
          ! h264parse config-interval=-1 \
          ! mp4mux faststart=true \
          ! filesink location="$tmp"
      ;;
    *)
      echo "unsupported GO2_BUILTIN_CAMERA_ENCODER: $BUILTIN_ENCODER" >&2
      exit 2
      ;;
  esac

  if [ ! -s "$tmp" ]; then
    echo "built-in camera produced an empty video" >&2
    exit 1
  fi
  mv "$tmp" "$out"
  trap - EXIT INT TERM
  echo "$out"
}

snapshot_builtin() {
  wait_valid_time
  mkdir -p "$OUT_DIR"
  local ts out tmp
  ts=$(timestamp)
  out="${OUT_DIR}/unitree_builtin_snapshot_${ts}.jpg"
  tmp="${OUT_DIR}/.unitree_builtin_snapshot_${ts}.tmp.jpg"
  trap 'rm -f "$tmp"' EXIT INT TERM
  "$BUILTIN_BIN" snapshot "$tmp" "$BUILTIN_INTERFACE" >&2
  if [ ! -s "$tmp" ]; then
    echo "built-in camera produced an empty snapshot" >&2
    exit 1
  fi
  mv "$tmp" "$out"
  trap - EXIT INT TERM
  echo "$out"
}

case "$CAMERA_SOURCE" in
  z1pro)
    if [ ! -x "$Z1PRO_CAPTURE" ]; then
      echo "missing Z-1Pro capture script: $Z1PRO_CAPTURE" >&2
      exit 1
    fi
    exec "$Z1PRO_CAPTURE" "$MODE" "$SEGMENT_SECONDS"
    ;;
  unitree_builtin)
    require_builtin_camera
    resolve_builtin_interface
    case "$MODE" in
      record)
        record_builtin
        ;;
      snapshot)
        snapshot_builtin
        ;;
      probe)
        "$BUILTIN_BIN" probe "$BUILTIN_INTERFACE"
        ;;
      *)
        echo "usage: $0 [record [seconds]|snapshot|probe]" >&2
        exit 2
        ;;
    esac
    ;;
  *)
    echo "unsupported GO2_CAMERA_SOURCE: $CAMERA_SOURCE (expected unitree_builtin or z1pro)" >&2
    exit 2
    ;;
esac

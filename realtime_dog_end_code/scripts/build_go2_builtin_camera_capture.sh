#!/usr/bin/env bash
set -euo pipefail

WS=${GO2_WS:-/home/unitree/go2_fastlio_ws}
SDK=${GO2_UNITREE_SDK2_DIR:-${WS}/third_party/unitree_sdk2}
SOURCE=${GO2_BUILTIN_CAMERA_SOURCE_FILE:-${WS}/cpp_tools/go2_builtin_camera_capture.cpp}
OUTPUT=${GO2_BUILTIN_CAMERA_BIN:-${WS}/bin/go2_builtin_camera_capture}
ARCH=${GO2_UNITREE_SDK2_ARCH:-aarch64}

if [ ! -f "$SOURCE" ]; then
  echo "missing source: $SOURCE" >&2
  exit 1
fi
if [ ! -f "${SDK}/lib/${ARCH}/libunitree_sdk2.a" ]; then
  echo "missing Unitree SDK2 library under: ${SDK}/lib/${ARCH}" >&2
  exit 1
fi

mkdir -p "$(dirname "$OUTPUT")"
tmp=$(mktemp "${OUTPUT}.tmp.XXXXXX")
trap 'rm -f "$tmp"' EXIT

g++ -std=c++17 -O2 "$SOURCE" \
  -I"${SDK}/include" \
  -I"${SDK}/thirdparty/include" \
  -I"${SDK}/thirdparty/include/ddscxx" \
  "${SDK}/lib/${ARCH}/libunitree_sdk2.a" \
  -L"${SDK}/thirdparty/lib/${ARCH}" \
  -Wl,-rpath,"${SDK}/thirdparty/lib/${ARCH}" \
  -lddsc -lddscxx -lpthread \
  -o "$tmp"

chmod 0755 "$tmp"
mv "$tmp" "$OUTPUT"
trap - EXIT
echo "BUILTIN_CAMERA_BUILD_OK output=$OUTPUT"

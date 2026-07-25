#!/usr/bin/env bash
set -eo pipefail

WS=${GO2_WS:-/home/unitree/go2_fastlio_ws}
SRC="$WS/cpp_tools/legacy_iox_stub/free_iox_chunk_stub.c"
OUT="$WS/cpp_tools/legacy_iox_stub/liblegacy_iox_stub.so"

mkdir -p "$(dirname "$OUT")"
gcc -shared -fPIC -O2 "$SRC" -o "$OUT"

echo "$OUT"
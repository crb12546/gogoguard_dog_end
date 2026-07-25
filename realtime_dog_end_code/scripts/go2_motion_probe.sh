#!/usr/bin/env bash
set -euo pipefail

WS=${GO2_WS:-/home/unitree/go2_fastlio_ws}
SDK_LIB="$WS/third_party/unitree_sdk2_install/lib"
PROBE="$WS/install/go2_cmd_vel_bridge/lib/go2_cmd_vel_bridge/go2_sdk2_motion_probe"

export LD_LIBRARY_PATH="$SDK_LIB:${LD_LIBRARY_PATH:-}"

if [[ ! -x "$PROBE" ]]; then
  echo "motion probe not found: $PROBE" >&2
  echo "Build it first: cd $WS && source /opt/ros/foxy/setup.bash && colcon build --packages-select go2_cmd_vel_bridge --symlink-install" >&2
  exit 1
fi

exec "$PROBE" "$@"
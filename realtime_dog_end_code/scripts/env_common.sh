#!/usr/bin/env bash
set -e

export WS=/home/unitree/go2_fastlio_ws
export PATROL_PKG=go2_fastlio_patrol
export ROUTE_ROOT=${WS}/src/go2_fastlio_patrol/routes
export RECORD_ROOT=${ROUTE_ROOT}/records
export LOG_DIR=${WS}/patrol_logs
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

export GO2_WIRED_MAC=${GO2_WIRED_MAC:-4c:bb:47:ab:e4:c2}

find_iface_by_mac() {
  local target iface addr
  target=$(printf '%s' "${1:-}" | tr 'A-Z' 'a-z')
  [ -n "$target" ] || return 1
  for iface in /sys/class/net/*; do
    [ -r "$iface/address" ] || continue
    addr=$(cat "$iface/address" 2>/dev/null | tr 'A-Z' 'a-z')
    if [ "$addr" = "$target" ]; then
      basename "$iface"
      return 0
    fi
  done
  return 1
}

export GO2_WIRED_IF=${GO2_WIRED_IF:-$(find_iface_by_mac "$GO2_WIRED_MAC" 2>/dev/null || printf eth1)}
export GO2_SDK_IF=${GO2_SDK_IF:-$GO2_WIRED_IF}
export GO2_CYCLONEDDS_CONFIG=${GO2_CYCLONEDDS_CONFIG:-/tmp/go2_cyclonedds_${GO2_WIRED_IF}.xml}

if [ -n "$GO2_WIRED_IF" ]; then
  cat >"$GO2_CYCLONEDDS_CONFIG" <<EOF
<CycloneDDS>
  <Domain id='any'>
    <General>
      <NetworkInterfaceAddress>$GO2_WIRED_IF</NetworkInterfaceAddress>
      <AllowMulticast>true</AllowMulticast>
    </General>
  </Domain>
</CycloneDDS>
EOF
  export CYCLONEDDS_URI=${CYCLONEDDS_URI:-file://${GO2_CYCLONEDDS_CONFIG}}
fi

source /opt/ros/foxy/setup.bash

if [ -f /unitree/module/graph_pid_ws/install/setup.bash ]; then
  source /unitree/module/graph_pid_ws/install/setup.bash
fi

source ${WS}/install/setup.bash
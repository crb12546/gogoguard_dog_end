#!/usr/bin/env bash
set -euo pipefail

CTL=${Z1PRO_GCU_CTL:-/home/unitree/go2_fastlio_ws/scripts/z1pro_gcu_control.py}

case "${1:-front}" in
  front)
    "$CTL" angle --roll 0 --pitch 0 --yaw 0
    ;;
  down)
    "$CTL" angle --roll 0 --pitch -10 --yaw 0
    ;;
  up)
    "$CTL" angle --roll 0 --pitch 10 --yaw 0
    ;;
  left)
    "$CTL" angle --roll 0 --pitch 0 --yaw -15
    ;;
  right)
    "$CTL" angle --roll 0 --pitch 0 --yaw 15
    ;;
  home)
    "$CTL" home
    ;;
  probe)
    "$CTL" probe
    ;;
  rear)
    if [ "${Z1PRO_ALLOW_REAR:-0}" != "1" ]; then
      echo "rear is disabled by default because it is a large move; set Z1PRO_ALLOW_REAR=1 to allow it" >&2
      exit 2
    fi
    "$CTL" angle --roll 0 --pitch 0 --yaw 180
    ;;
  *)
    echo "usage: $0 [front|down|up|left|right|home|probe|rear]" >&2
    exit 2
    ;;
esac
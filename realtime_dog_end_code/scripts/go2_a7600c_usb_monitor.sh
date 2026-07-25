#!/usr/bin/env bash
set -u

# Lightweight A7600C USB link monitor. It is read-only except for /tmp logs.

INTERVAL=${GO2_A7600C_MONITOR_INTERVAL:-5}
LOG_DIR=${GO2_A7600C_MONITOR_LOG_DIR:-/tmp}
SNAPSHOT_LOG=${GO2_A7600C_SNAPSHOT_LOG:-$LOG_DIR/go2_a7600c_usb_snapshot.log}
KERNEL_LOG=${GO2_A7600C_KERNEL_LOG:-$LOG_DIR/go2_a7600c_kernel_watch.log}
EVENT_LOG=${GO2_A7600C_EVENT_LOG:-$LOG_DIR/go2_a7600c_usb_events.log}
SNAPSHOT_PID=${GO2_A7600C_SNAPSHOT_PID:-/tmp/go2_a7600c_usb_snapshot.pid}
KERNEL_PID=${GO2_A7600C_KERNEL_PID:-/tmp/go2_a7600c_kernel_watch.pid}
TARGET_HOST=${GO2_4G_TARGET_HOST:-39.96.37.187}
USB_PATH=${GO2_A7600C_USB_PATH:-1-2.2}
USB_PARENT=${GO2_A7600C_USB_PARENT:-1-2}
NET_IF=${GO2_A7600C_NET_IF:-go2_4g}

PATTERN='usb 1-2|1-2\.2|go2_4g|eth1|cdc_ether|ttyUSB|disconnect|reset|error -71|Cannot enable|xhci|tegra-xusb|descriptor|enumerat|over-current|under-voltage|unregister'
EVENT_PATTERN='disconnect|reset|error -71|Cannot enable|unregister|descriptor|enumerat|over-current|under-voltage'

ts() {
  date '+%F %T'
}

append_cmd() {
  local label=$1
  shift
  {
    printf -- '--- %s ---\n' "$label"
    "$@" 2>&1 || true
  } >>"$SNAPSHOT_LOG"
}

snapshot_loop() {
  mkdir -p "$LOG_DIR"
  printf '[%s] snapshot monitor started interval=%ss usb=%s net=%s\n' "$(ts)" "$INTERVAL" "$USB_PATH" "$NET_IF" >>"$SNAPSHOT_LOG"
  while true; do
    {
      printf '\n===== %s =====\n' "$(ts)"
      uptime
    } >>"$SNAPSHOT_LOG"
    append_cmd "ip -br addr" ip -br addr
    append_cmd "ip route target" ip route get "$TARGET_HOST"
    append_cmd "ip -s link $NET_IF" ip -s link show "$NET_IF"
    append_cmd "net sysfs $NET_IF" sh -c "readlink -f /sys/class/net/$NET_IF/device 2>/dev/null; cat /sys/class/net/$NET_IF/address /sys/class/net/$NET_IF/operstate /sys/class/net/$NET_IF/carrier 2>/dev/null"
    append_cmd "usb tree" lsusb -t
    append_cmd "usb devices" lsusb
    append_cmd "usb sysfs $USB_PARENT/$USB_PATH" sh -c "for f in /sys/bus/usb/devices/$USB_PARENT/product /sys/bus/usb/devices/$USB_PARENT/idVendor /sys/bus/usb/devices/$USB_PARENT/idProduct /sys/bus/usb/devices/$USB_PARENT/speed /sys/bus/usb/devices/$USB_PATH/product /sys/bus/usb/devices/$USB_PATH/manufacturer /sys/bus/usb/devices/$USB_PATH/idVendor /sys/bus/usb/devices/$USB_PATH/idProduct /sys/bus/usb/devices/$USB_PATH/speed /sys/bus/usb/devices/$USB_PATH/busnum /sys/bus/usb/devices/$USB_PATH/devnum; do printf '%s=' \"\$f\"; cat \"\$f\" 2>/dev/null || true; done"
    append_cmd "ttyUSB" sh -c "ls -l /dev/ttyUSB* 2>/dev/null || true"
    append_cmd "watchdog tail" sh -c "tail -n 8 /tmp/go2_connectivity_watchdog.log 2>/dev/null || true"
    sleep "$INTERVAL"
  done
}

capture_event_bundle() {
  local trigger=$1
  {
    printf '\n===== USB EVENT %s =====\n' "$(ts)"
    printf 'trigger=%s\n' "$trigger"
    printf '\n--- recent snapshots ---\n'
    tail -n 260 "$SNAPSHOT_LOG" 2>/dev/null || true
    printf '\n--- current usb tree ---\n'
    lsusb -t 2>&1 || true
    printf '\n--- current usb devices ---\n'
    lsusb 2>&1 || true
    printf '\n--- current net ---\n'
    ip -br addr 2>&1 || true
    ip -s link show "$NET_IF" 2>&1 || true
    ip route get "$TARGET_HOST" 2>&1 || true
    printf '\n--- kernel around now ---\n'
    journalctl -b --no-pager -k --since '-90 seconds' 2>&1 | grep -Ei "$PATTERN" || true
    printf '\n--- NetworkManager around now ---\n'
    journalctl -b --no-pager -u NetworkManager --since '-90 seconds' 2>&1 | grep -Ei "$PATTERN|dhcp|carrier|link|managed" || true
    printf '\n--- watchdog around now ---\n'
    tail -n 80 /tmp/go2_connectivity_watchdog.log 2>/dev/null || true
  } >>"$EVENT_LOG"
}

kernel_watch_loop() {
  mkdir -p "$LOG_DIR"
  printf '[%s] kernel watch started usb=%s net=%s\n' "$(ts)" "$USB_PATH" "$NET_IF" >>"$KERNEL_LOG"
  journalctl -b -k -f --no-pager 2>&1 | grep --line-buffered -Ei "$PATTERN" | while IFS= read -r line; do
    printf '[%s] %s\n' "$(ts)" "$line" >>"$KERNEL_LOG"
    if printf '%s\n' "$line" | grep -Eiq "$EVENT_PATTERN"; then
      capture_event_bundle "$line"
    fi
  done
}

is_running() {
  local pidfile=$1
  [ -s "$pidfile" ] || return 1
  kill -0 "$(cat "$pidfile")" 2>/dev/null
}

start_monitor() {
  mkdir -p "$LOG_DIR"
  if ! is_running "$SNAPSHOT_PID"; then
    nohup "$0" snapshot >/tmp/go2_a7600c_snapshot.nohup 2>&1 &
    echo "$!" >"$SNAPSHOT_PID"
  fi
  if ! is_running "$KERNEL_PID"; then
    nohup "$0" kernel-watch >/tmp/go2_a7600c_kernel.nohup 2>&1 &
    echo "$!" >"$KERNEL_PID"
  fi
  status_monitor
}

stop_monitor() {
  for pidfile in "$SNAPSHOT_PID" "$KERNEL_PID"; do
    if is_running "$pidfile"; then
      kill "$(cat "$pidfile")" 2>/dev/null || true
    fi
    rm -f "$pidfile"
  done
}

status_monitor() {
  printf 'snapshot_pid='
  if is_running "$SNAPSHOT_PID"; then cat "$SNAPSHOT_PID"; else printf 'stopped\n'; fi
  printf 'kernel_pid='
  if is_running "$KERNEL_PID"; then cat "$KERNEL_PID"; else printf 'stopped\n'; fi
  printf 'snapshot_log=%s\nkernel_log=%s\nevent_log=%s\n' "$SNAPSHOT_LOG" "$KERNEL_LOG" "$EVENT_LOG"
}

case "${1:-status}" in
  start)
    start_monitor
    ;;
  stop)
    stop_monitor
    ;;
  snapshot)
    snapshot_loop
    ;;
  kernel-watch)
    kernel_watch_loop
    ;;
  status)
    status_monitor
    ;;
  *)
    echo "usage: $0 {start|stop|status|snapshot|kernel-watch}" >&2
    exit 2
    ;;
esac

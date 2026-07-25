#!/usr/bin/env bash
set -u

# Long-running A7600C connectivity watchdog.
# It treats the 4G modem as an unreliable bearer: USB may disappear, the modem
# may reboot, and the carrier may drop. The script waits for stable USB
# enumeration before attempting to restore PPP/ECM networking.

TARGET_HOST=${GO2_4G_TARGET_HOST:-39.96.37.187}
TARGET_PORT=${GO2_4G_TARGET_PORT:-443}
USB_VENDOR=${GO2_4G_USB_VENDOR:-1e0e}
USB_PRODUCT=${GO2_4G_USB_PRODUCT:-9011}
MODE=${GO2_4G_MODE:-auto}                 # auto | ppp | ecm | nmcli | passive
INTERVAL=${GO2_4G_WATCH_INTERVAL:-5}
STABLE_SECONDS=${GO2_4G_USB_STABLE_SECONDS:-30}
RECOVERY_COOLDOWN=${GO2_4G_RECOVERY_COOLDOWN:-20}
PPP_PEER=${GO2_4G_PPP_PEER:-go2-a7600c}
PPP_IF=${GO2_4G_PPP_IF:-ppp0}
PPP_TTY=${GO2_4G_PPP_TTY:-/dev/ttyUSB2}
PPP_TTY_CANDIDATES=${GO2_4G_PPP_TTY_CANDIDATES:-"/dev/ttyUSB2 /dev/ttyUSB1 /dev/ttyUSB3 /dev/ttyUSB0"}
PPP_USB_INTERFACE=${GO2_4G_PPP_USB_INTERFACE:-05}
PPP_BAUD=${GO2_4G_PPP_BAUD:-115200}
PPP_RUNTIME_PEER=${GO2_4G_PPP_RUNTIME_PEER:-/tmp/go2-a7600c-runtime-peer}
PPP_PRE_DIAL_DELAY=${GO2_4G_PPP_PRE_DIAL_DELAY:-5}
PPP_AT_PROBE_TIMEOUT=${GO2_4G_PPP_AT_PROBE_TIMEOUT:-6}
PPP_SKIP_AT_PROBE=${GO2_4G_PPP_SKIP_AT_PROBE:-0}
ENABLE_PPP_MODEM_RECOVERY=${GO2_4G_ENABLE_PPP_MODEM_RECOVERY:-1}
PPP_MODEM_RECOVERY_MIN_INTERVAL=${GO2_4G_PPP_MODEM_RECOVERY_MIN_INTERVAL:-300}
LAST_PPP_MODEM_RECOVERY=${GO2_4G_LAST_PPP_MODEM_RECOVERY:-/run/go2-4g-last-ppp-modem-recovery}
ECM_IF=${GO2_4G_ECM_IF:-go2_4g}
ECM_GW=${GO2_4G_ECM_GW:-192.168.0.1}
ECM_MAC=${GO2_4G_ECM_MAC:-28:e3:57:c3:ed:9a}
WIRED_MAC=${GO2_WIRED_MAC:-4c:bb:47:ab:e4:c2}
ECM_AT_CONFIG=${GO2_4G_ECM_AT_CONFIG:-1}
ECM_AT_CONFIG_MARKER=${GO2_4G_ECM_AT_CONFIG_MARKER:-/run/go2-4g-ecm-at-configured}
NM_CONNECTION=${GO2_GSM_CONNECTION:-go2-4g}
STOP_MODEMMANAGER_FOR_PPP=${GO2_4G_STOP_MODEMMANAGER_FOR_PPP:-1}
ENABLE_USB_RESET=${GO2_4G_ENABLE_USB_RESET:-0}
ENABLE_USB_POWER_CONTROL=${GO2_4G_ENABLE_USB_POWER_CONTROL:-0}
USB_RESET_MIN_INTERVAL=${GO2_4G_USB_RESET_MIN_INTERVAL:-180}
LAST_USB_RESET=${GO2_4G_LAST_USB_RESET:-/run/go2-4g-last-usb-reset}
ENABLE_USB_BUS_REBIND=${GO2_4G_ENABLE_USB_BUS_REBIND:-1}
USB_BUS_DEV=${GO2_4G_USB_BUS_DEV:-usb1}
USB_ABSENT_REBIND_AFTER=${GO2_4G_USB_ABSENT_REBIND_AFTER:-60}
USB_BOOT_ABSENT_REBIND_AFTER=${GO2_4G_USB_BOOT_ABSENT_REBIND_AFTER:-20}
USB_BOOT_ABSENT_WINDOW=${GO2_4G_USB_BOOT_ABSENT_WINDOW:-180}
USB_PRESENT_REBIND_AFTER=${GO2_4G_USB_PRESENT_REBIND_AFTER:-180}
USB_UNSTABLE_REBIND_AFTER=${GO2_4G_USB_UNSTABLE_REBIND_AFTER:-45}
USB_BUS_REBIND_MIN_INTERVAL=${GO2_4G_USB_BUS_REBIND_MIN_INTERVAL:-60}
USB_BOOT_BUS_REBIND_MIN_INTERVAL=${GO2_4G_USB_BOOT_BUS_REBIND_MIN_INTERVAL:-45}
USB_BUS_REBIND_MAX_ATTEMPTS=${GO2_4G_USB_BUS_REBIND_MAX_ATTEMPTS:-3}
USB_BOOT_BUS_REBIND_MAX_ATTEMPTS=${GO2_4G_USB_BOOT_BUS_REBIND_MAX_ATTEMPTS:-5}
USB_BUS_REBIND_BACKOFF=${GO2_4G_USB_BUS_REBIND_BACKOFF:-300}
LAST_USB_BUS_REBIND=${GO2_4G_LAST_USB_BUS_REBIND:-/run/go2-4g-last-usb-bus-rebind}
USB_HOST_CONTROLLER=${GO2_4G_USB_HOST_CONTROLLER:-3610000.xhci}
XHCI_DEAD_MARKER=${GO2_4G_XHCI_DEAD_MARKER:-/run/go2-4g-xhci-dead}
XHCI_DEAD_LOG_MIN_INTERVAL=${GO2_4G_XHCI_DEAD_LOG_MIN_INTERVAL:-60}
LAST_XHCI_DEAD_LOG=${GO2_4G_LAST_XHCI_DEAD_LOG:-/run/go2-4g-last-xhci-dead-log}
NETDEV_STUCK_IF=${GO2_4G_NETDEV_STUCK_IF:-"go2_4g eth0 eth1"}
NETDEV_STUCK_RECENT_SECONDS=${GO2_4G_NETDEV_STUCK_RECENT_SECONDS:-75}
NETDEV_STUCK_CLEANUP_MIN_INTERVAL=${GO2_4G_NETDEV_STUCK_CLEANUP_MIN_INTERVAL:-30}
NETDEV_STUCK_REBOOT_AFTER=${GO2_4G_NETDEV_STUCK_REBOOT_AFTER:-0}
LAST_NETDEV_STUCK_CLEANUP=${GO2_4G_LAST_NETDEV_STUCK_CLEANUP:-/run/go2-4g-last-netdev-stuck-cleanup}
NETDEV_STUCK_MARKER=${GO2_4G_NETDEV_STUCK_MARKER:-/run/go2-4g-netdev-stuck}
ENABLE_TIME_BOOTSTRAP=${GO2_4G_ENABLE_TIME_BOOTSTRAP:-1}
TIME_BOOTSTRAP_MIN_YEAR=${GO2_4G_TIME_BOOTSTRAP_MIN_YEAR:-2024}
RUN_ONCE=${GO2_4G_RUN_ONCE:-0}
SELECTED_PPP_TTY=""
ACTIVE_ECM_IF="$ECM_IF"
START_TS=$(awk '{print int($1)}' /proc/uptime 2>/dev/null || date +%s)
USB_ABSENT_SINCE=0
USB_PRESENT_FAILURE_SINCE=0
USB_UNSTABLE_SINCE=0
USB_BUS_REBIND_ATTEMPTS=0
NETDEV_STUCK_SINCE=0

log() {
  printf '[go2-connectivity] %s %s\n' "$(date '+%F %T')" "$*"
}

run_root() {
  if [ "$(id -u)" = "0" ]; then
    "$@"
  elif command -v sudo >/dev/null 2>&1; then
    sudo "$@"
  else
    log "need root for: $*"
    return 1
  fi
}

monotonic_seconds() {
  awk '{print int($1)}' /proc/uptime 2>/dev/null || date +%s
}

usb_devices() {
  local d
  for d in /sys/bus/usb/devices/*; do
    [ -r "$d/idVendor" ] || continue
    [ -r "$d/idProduct" ] || continue
    if [ "$(cat "$d/idVendor" 2>/dev/null)" = "$USB_VENDOR" ] && [ "$(cat "$d/idProduct" 2>/dev/null)" = "$USB_PRODUCT" ]; then
      basename "$d"
    fi
  done
}

usb_present() {
  [ -n "$(usb_devices | head -1)" ]
}

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

usb_device_signature() {
  local dev d busnum devnum serial
  dev=$(usb_devices | sort | head -1)
  [ -n "$dev" ] || return 1
  d="/sys/bus/usb/devices/$dev"
  busnum=$(cat "$d/busnum" 2>/dev/null || echo "?")
  devnum=$(cat "$d/devnum" 2>/dev/null || echo "?")
  # Some A7600C firmware states can block indefinitely while reading the USB serial sysfs node.
  # The device path plus bus/dev number is enough to detect re-enumeration stability.
  serial=""
  printf '%s:bus%s:dev%s:%s' "$dev" "$busnum" "$devnum" "$serial"
}

usb_root_present() {
  [ -e "/sys/bus/usb/devices/${USB_BUS_DEV}" ] && return 0
  ls /sys/bus/usb/devices/usb* >/dev/null 2>&1
}

recent_xhci_dead_fault() {
  dmesg 2>/dev/null | tail -n 500 | grep -Eq \
    "xHCI host controller not responding|HC died|XHCI Controller not ready|failed to load firmware: -5|probe of .*xhci failed with error -5"
}

recent_usb_bus_fault() {
  dmesg 2>/dev/null | tail -n 500 | grep -Eq \
    "Abort failed to stop command ring|usb usb1: can't set config #1, error -22|Not enough bandwidth for altsetting"
}

netdev_unregister_stuck_if() {
  local ifaces ifname pattern logs
  ifaces=${1:-$NETDEV_STUCK_IF}
  if command -v journalctl >/dev/null 2>&1; then
    logs=$(journalctl -k --since "${NETDEV_STUCK_RECENT_SECONDS} seconds ago" --no-pager 2>/dev/null || true)
    for ifname in $ifaces; do
      pattern="unregister_netdevice: waiting for ${ifname} to become free"
      printf '%s\n' "$logs" | grep -Fq "$pattern" && {
        printf '%s\n' "$ifname"
        return 0
      }
    done
  fi
  logs=$(dmesg 2>/dev/null | tail -n 120 || true)
  for ifname in $ifaces; do
    pattern="unregister_netdevice: waiting for ${ifname} to become free"
    printf '%s\n' "$logs" | grep -Fq "$pattern" && {
      printf '%s\n' "$ifname"
      return 0
    }
  done
  return 1
}

recent_netdev_unregister_stuck() {
  netdev_unregister_stuck_if "$@" >/dev/null
}

cleanup_netdev_unregister_stuck() {
  local now last elapsed ifname
  ifname=${1:-$NETDEV_STUCK_IF}
  now=$(monotonic_seconds)
  if [ "$NETDEV_STUCK_SINCE" -le 0 ] 2>/dev/null; then
    NETDEV_STUCK_SINCE="$now"
  fi
  elapsed=$((now - NETDEV_STUCK_SINCE))
  echo "$now $ifname" | run_root tee "$NETDEV_STUCK_MARKER" >/dev/null 2>&1 || true

  last=$(cat "$LAST_NETDEV_STUCK_CLEANUP" 2>/dev/null || echo 0)
  if [ "$last" -gt "$now" ] 2>/dev/null; then
    last=0
  fi
  if [ $((now - last)) -lt "$NETDEV_STUCK_CLEANUP_MIN_INTERVAL" ]; then
    log "USB_NETDEV_UNREGISTER_STUCK: $ifname still unregistering (${elapsed}s); cleanup cooldown active"
  else
    echo "$now" | run_root tee "$LAST_NETDEV_STUCK_CLEANUP" >/dev/null 2>&1 || true
    log "USB_NETDEV_UNREGISTER_STUCK: $ifname unregister stuck for ${elapsed}s; clearing user-space holders"
    run_root pkill -TERM -f "dhclient .*${ifname}" 2>/dev/null || true
    run_root pkill -TERM -f "udhcpc .* -i ${ifname}" 2>/dev/null || true
    run_root pkill -TERM -f "pppd .*(${PPP_TTY}|${PPP_RUNTIME_PEER}|${PPP_PEER})" 2>/dev/null || true
    if command -v nmcli >/dev/null 2>&1; then
      run_root timeout 5 nmcli dev disconnect "$ifname" >/dev/null 2>&1 || true
      run_root timeout 5 nmcli dev set "$ifname" managed no >/dev/null 2>&1 || true
    fi
    run_root ip route del default dev "$ifname" 2>/dev/null || true
    run_root ip addr flush dev "$ifname" 2>/dev/null || true
  fi

  if [ "$NETDEV_STUCK_REBOOT_AFTER" -gt 0 ] 2>/dev/null && [ "$elapsed" -ge "$NETDEV_STUCK_REBOOT_AFTER" ] 2>/dev/null; then
    log "USB_NETDEV_UNREGISTER_STUCK_REBOOTING: $ifname stuck for ${elapsed}s >= ${NETDEV_STUCK_REBOOT_AFTER}s"
    run_root sync 2>/dev/null || true
    run_root systemctl reboot --message="A7600C ${ifname} unregister stuck for ${elapsed}s" 2>/dev/null || \
      run_root reboot 2>/dev/null || true
  else
    log "USB_NETDEV_UNREGISTER_STUCK_REBOOT_REQUIRED: $ifname stuck for ${elapsed}s; set GO2_4G_NETDEV_STUCK_REBOOT_AFTER>0 to allow automatic reboot"
  fi
}

clear_netdev_stuck_state_if_gone() {
  if recent_netdev_unregister_stuck; then
    return 0
  fi
  if [ "$NETDEV_STUCK_SINCE" -gt 0 ] 2>/dev/null; then
    log "USB_NETDEV_UNREGISTER_STUCK_CLEARED: ${NETDEV_STUCK_IF} unregister warning disappeared"
  fi
  NETDEV_STUCK_SINCE=0
  run_root rm -f "$NETDEV_STUCK_MARKER" "$LAST_NETDEV_STUCK_CLEANUP" 2>/dev/null || true
}

xhci_dead() {
  usb_root_present && return 1
  recent_xhci_dead_fault
}

log_xhci_dead_once() {
  local now last
  now=$(monotonic_seconds)
  last=$(cat "$LAST_XHCI_DEAD_LOG" 2>/dev/null || echo 0)
  if [ "$last" -gt "$now" ] 2>/dev/null; then
    last=0
  fi
  echo "$now" | run_root tee "$XHCI_DEAD_MARKER" >/dev/null 2>&1 || true
  if [ $((now - last)) -lt "$XHCI_DEAD_LOG_MIN_INTERVAL" ]; then
    return 0
  fi
  echo "$now" | run_root tee "$LAST_XHCI_DEAD_LOG" >/dev/null 2>&1 || true
  log "USB_HOST_CONTROLLER_DEAD_REBOOT_REQUIRED: ${USB_HOST_CONTROLLER} root hub is missing after xHCI fault; skipping host reset"
}

set_usb_power_on() {
  local dev path current parent
  [ "$ENABLE_USB_POWER_CONTROL" = "1" ] || return 0
  for dev in $(usb_devices); do
    path="/sys/bus/usb/devices/$dev/power/control"
    if [ -w "$path" ]; then
      current=$(cat "$path" 2>/dev/null || true)
      if [ "$current" != "on" ] && echo on >"$path" 2>/dev/null; then
        log "USB device power/control forced on for $dev"
      fi
    fi
    parent="$dev"
    while printf '%s' "$parent" | grep -q '\.'; do
      parent=${parent%.*}
      path="/sys/bus/usb/devices/$parent/power/control"
      if [ -w "$path" ]; then
        current=$(cat "$path" 2>/dev/null || true)
        if [ "$current" != "on" ] && echo on >"$path" 2>/dev/null; then
          log "USB parent hub power/control forced on for $parent"
        fi
      fi
    done
  done
}

set_usb_host_power_on() {
  local dev path current
  [ "$ENABLE_USB_POWER_CONTROL" = "1" ] || return 0
  dev="/sys/bus/platform/devices/${USB_HOST_CONTROLLER}"
  path="$dev/power/control"
  if [ -w "$path" ]; then
    current=$(cat "$path" 2>/dev/null || true)
    if [ "$current" != "on" ] && echo on >"$path" 2>/dev/null; then
      log "USB host power/control forced on for ${USB_HOST_CONTROLLER}"
    fi
  fi
  path="/sys/bus/usb/devices/${USB_BUS_DEV}/power/control"
  if [ -w "$path" ]; then
    current=$(cat "$path" 2>/dev/null || true)
    if [ "$current" != "on" ] && echo on >"$path" 2>/dev/null; then
      log "USB bus power/control forced on for ${USB_BUS_DEV}"
    fi
  fi
}

disable_usb_autosuspend() {
  if [ -w /sys/module/usbcore/parameters/autosuspend ]; then
    echo -1 >/sys/module/usbcore/parameters/autosuspend 2>/dev/null || true
    log "usbcore autosuspend disabled"
  fi
}

unload_usbnet_drivers_for_ppp() {
  [ "$MODE" = "ppp" ] || return 0
  run_root modprobe -r rndis_host cdc_ether cdc_subset usbnet 2>/dev/null || true
  if [ -x /usr/local/sbin/go2-a7600c-usbnet-unbind ]; then
    run_root /usr/local/sbin/go2-a7600c-usbnet-unbind 2>/dev/null || true
  fi
}

wait_usb_stable() {
  local stable sig last_sig stuck_if
  stuck_if=$(netdev_unregister_stuck_if || true)
  if [ -n "$stuck_if" ]; then
    log "A7600C_USB_SYSFS_STALE_REBOOT_REQUIRED: ${stuck_if} unregister stuck after USB disconnect"
    return 1
  fi
  stable=0
  last_sig=""
  while [ "$stable" -lt "$STABLE_SECONDS" ]; do
    sig=$(usb_device_signature 2>/dev/null || true)
    if [ -z "$sig" ]; then
      return 1
    fi
    if [ "$sig" != "$last_sig" ]; then
      if [ -n "$last_sig" ]; then
        log "A7600C USB signature changed while settling: $last_sig -> $sig"
      fi
      last_sig="$sig"
      stable=0
    fi
    sleep 1
    stable=$((stable + 1))
  done
  log "A7600C USB stable: $last_sig for ${STABLE_SECONDS}s"
  return 0
}

tcp_ok() {
  timeout 4 bash -c ":</dev/tcp/${TARGET_HOST}/${TARGET_PORT}" >/dev/null 2>&1
}

route_uses_4g() {
  local route_dev
  route_dev=$(ip route get "$TARGET_HOST" 2>/dev/null | sed -n 's/.* dev \([^ ]*\).*/\1/p' | head -1)
  [ -n "$route_dev" ] || return 1

  case "$route_dev" in
    "$PPP_IF"|ppp[0-9]*|wwan[0-9]*|wwp*|usb[0-9]*)
      return 0
      ;;
  esac

  detect_ecm_if >/dev/null 2>&1 || return 1
  [ "$route_dev" = "$ACTIVE_ECM_IF" ]
}

online() {
  route_uses_4g && tcp_ok
}

bootstrap_time_if_needed() {
  local year header
  [ "$ENABLE_TIME_BOOTSTRAP" = "1" ] || return 0
  year=$(date +%Y 2>/dev/null || echo 0)
  [ "$year" -lt "$TIME_BOOTSTRAP_MIN_YEAR" ] || return 0
  command -v curl >/dev/null 2>&1 || return 0

  header=$(timeout 10 curl -skI --connect-timeout 5 --max-time 8 "https://${TARGET_HOST}/" 2>/dev/null | \
    awk -F': ' 'tolower($1) == "date" {print $2; exit}' | tr -d '\r')
  [ -n "$header" ] || return 1
  if run_root date -s "$header" >/dev/null 2>&1; then
    log "system time bootstrapped from HTTPS Date: $header"
  fi
}

ppp_has_ip() {
  ip -4 addr show dev "$PPP_IF" 2>/dev/null | grep -q 'inet '
}

ecm_has_ip() {
  detect_ecm_if >/dev/null 2>&1 || return 1
  ip -4 addr show dev "$ACTIVE_ECM_IF" 2>/dev/null | grep -q 'inet '
}

stop_ecm_dhcp_clients() {
  detect_ecm_if >/dev/null 2>&1 || return 0
  run_root pkill -TERM -f "dhclient .*${ACTIVE_ECM_IF}" 2>/dev/null || true
  run_root pkill -TERM -f "udhcpc .* -i ${ACTIVE_ECM_IF}" 2>/dev/null || true
}

usb_present_rebind_due() {
  local reason now elapsed
  reason=${1:-bearer recovery}
  now=$(monotonic_seconds)
  if [ "$USB_PRESENT_FAILURE_SINCE" -le 0 ] 2>/dev/null; then
    USB_PRESENT_FAILURE_SINCE="$now"
  fi
  elapsed=$((now - USB_PRESENT_FAILURE_SINCE))
  if [ "$elapsed" -lt "$USB_PRESENT_REBIND_AFTER" ]; then
    log "$reason failed; USB is still present/stable, retrying without bus rebind (${elapsed}/${USB_PRESENT_REBIND_AFTER}s)"
    return 1
  fi
  log "$reason failed for ${elapsed}s with stable USB; allowing bus rebind"
  return 0
}

usb_absent_rebind_after() {
  if usb_boot_absent_window_active; then
    printf '%s\n' "$USB_BOOT_ABSENT_REBIND_AFTER"
    return 0
  fi
  printf '%s\n' "$USB_ABSENT_REBIND_AFTER"
}

usb_boot_absent_window_active() {
  [ "$USB_ABSENT_SINCE" -gt 0 ] 2>/dev/null || return 1
  [ "$USB_ABSENT_SINCE" -le "$USB_BOOT_ABSENT_WINDOW" ] 2>/dev/null
}

usb_bus_rebind_min_interval() {
  if usb_boot_absent_window_active; then
    printf '%s\n' "$USB_BOOT_BUS_REBIND_MIN_INTERVAL"
    return 0
  fi
  printf '%s\n' "$USB_BUS_REBIND_MIN_INTERVAL"
}

usb_bus_rebind_max_attempts() {
  if usb_boot_absent_window_active; then
    printf '%s\n' "$USB_BOOT_BUS_REBIND_MAX_ATTEMPTS"
    return 0
  fi
  printf '%s\n' "$USB_BUS_REBIND_MAX_ATTEMPTS"
}

replace_ecm_default_route() {
  detect_ecm_if >/dev/null 2>&1 || return 1
  if ip route show default 2>/dev/null | \
    awk -v gw="$ECM_GW" -v dev="$ACTIVE_ECM_IF" \
      '$1 == "default" && $2 == "via" && $3 == gw && $4 == "dev" && $5 == dev { found = 1 } END { exit !found }'; then
    return 0
  fi
  run_root ip route replace default via "$ECM_GW" dev "$ACTIVE_ECM_IF" metric 50 2>/dev/null || true
}

ppp_running() {
  pgrep -f "pppd .*(${PPP_TTY}|${PPP_RUNTIME_PEER}|${PPP_PEER})" >/dev/null 2>&1
}

stop_ppp_if_running() {
  if ppp_running; then
    log "stopping pppd because A7600C USB is absent"
    run_root pkill -TERM -f "pppd .*(${PPP_TTY}|${PPP_RUNTIME_PEER}|${PPP_PEER})" 2>/dev/null || true
  fi
}

nmcli_busy() {
  command -v nmcli >/dev/null 2>&1 || return 1
  nmcli -t -f DEVICE,TYPE,STATE,CONNECTION device status 2>/dev/null | \
    awk -F: -v conn="$NM_CONNECTION" '$2 == "gsm" && $4 == conn {print $3; exit}' | \
    grep -Eq 'connected|connecting|deactivating|disconnecting|prepare|config|ip-config|secondaries|activated'
}

disable_wired_default_route() {
  local wired_if
  wired_if=$(find_iface_by_mac "$WIRED_MAC") || return 0
  while ip route show default dev "$wired_if" 2>/dev/null | grep -q '^default '; do
    run_root ip route del default dev "$wired_if" 2>/dev/null || break
  done
  while ip route show default via 192.168.123.1 dev "$wired_if" 2>/dev/null | grep -q '^default '; do
    run_root ip route del default via 192.168.123.1 dev "$wired_if" 2>/dev/null || break
  done
}

usb_reset_once() {
  local now last dev usbreset_bin
  [ "$ENABLE_USB_RESET" = "1" ] || return 1
  dev=$(usb_devices | head -1)
  [ -n "$dev" ] || return 1
  now=$(date +%s)
  last=$(cat "$LAST_USB_RESET" 2>/dev/null || echo 0)
  if [ $((now - last)) -lt "$USB_RESET_MIN_INTERVAL" ]; then
    log "USB reset cooldown active"
    return 1
  fi
  usbreset_bin=$(command -v usbreset 2>/dev/null || true)
  if [ -z "$usbreset_bin" ] && [ -x /usr/bin/usbreset ]; then
    usbreset_bin=/usr/bin/usbreset
  fi
  if [ -n "$usbreset_bin" ]; then
    log "resetting A7600C through $usbreset_bin"
  else
    log "resetting USB device $dev through sysfs rebind"
  fi
  echo "$now" | run_root tee "$LAST_USB_RESET" >/dev/null 2>&1 || true
  if [ -n "$usbreset_bin" ] && run_root "$usbreset_bin" "${USB_VENDOR}:${USB_PRODUCT}" >/dev/null 2>&1; then
    sleep "$STABLE_SECONDS"
    return 0
  fi
  echo "$dev" | run_root tee /sys/bus/usb/drivers/usb/unbind >/dev/null 2>&1 || true
  sleep 3
  echo "$dev" | run_root tee /sys/bus/usb/drivers/usb/bind >/dev/null 2>&1 || true
  sleep "$STABLE_SECONDS"
}

usb_bus_rebind_once() {
  local now last min_interval max_attempts
  if recent_netdev_unregister_stuck; then
    log USB_NETDEV_UNREGISTER_STUCK_REBOOT_REQUIRED: kernel is still unregistering a USB netdev, skipping USB bus rebind
    return 1
  fi
  [ "$ENABLE_USB_BUS_REBIND" = "1" ] || return 1
  if xhci_dead; then
    log_xhci_dead_once
    return 1
  fi
  [ -e "/sys/bus/usb/devices/${USB_BUS_DEV}" ] || {
    log "USB bus device not found: $USB_BUS_DEV"
    return 1
  }
  now=$(monotonic_seconds)
  last=$(cat "$LAST_USB_BUS_REBIND" 2>/dev/null || echo 0)
  if [ "$last" -gt "$now" ] 2>/dev/null; then
    last=0
  fi
  max_attempts=$(usb_bus_rebind_max_attempts)
  if [ "$USB_BUS_REBIND_ATTEMPTS" -ge "$max_attempts" ] 2>/dev/null; then
    if [ $((now - last)) -lt "$USB_BUS_REBIND_BACKOFF" ]; then
      log "USB bus rebind extended backoff active after ${USB_BUS_REBIND_ATTEMPTS}/${max_attempts} attempts"
      return 1
    fi
    USB_BUS_REBIND_ATTEMPTS=0
  fi
  min_interval=$(usb_bus_rebind_min_interval)
  if [ $((now - last)) -lt "$min_interval" ]; then
    log "USB bus rebind cooldown active ($((now - last))/${min_interval}s)"
    return 1
  fi

  log "rebinding USB bus $USB_BUS_DEV to recover A7600C USB stack"
  echo "$now" | run_root tee "$LAST_USB_BUS_REBIND" >/dev/null 2>&1 || true
  echo "$USB_BUS_DEV" | run_root tee /sys/bus/usb/drivers/usb/unbind >/dev/null 2>&1 || true
  sleep 2
  echo "$USB_BUS_DEV" | run_root tee /sys/bus/usb/drivers/usb/bind >/dev/null 2>&1 || true
  USB_BUS_REBIND_ATTEMPTS=$((USB_BUS_REBIND_ATTEMPTS + 1))
  sleep "$STABLE_SECONDS"
}

at_ok() {
  local tty=$1
  [ -e "$tty" ] || return 1
  command -v python3 >/dev/null 2>&1 || return 1
  timeout "$PPP_AT_PROBE_TIMEOUT" python3 - "$tty" <<'PY'
import os
import select
import sys
import termios
import time

path = sys.argv[1]
fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
try:
    attrs = termios.tcgetattr(fd)
    attrs[0] = 0
    attrs[1] = 0
    attrs[2] = termios.B115200 | termios.CS8 | termios.CREAD | termios.CLOCAL
    attrs[3] = 0
    attrs[4] = termios.B115200
    attrs[5] = termios.B115200
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    termios.tcflush(fd, termios.TCIOFLUSH)
    try:
        os.write(fd, b"AT\r")
    except BlockingIOError:
        sys.exit(1)
    deadline = time.monotonic() + 2.0
    data = b""
    while time.monotonic() < deadline:
        readable, _, _ = select.select([fd], [], [], 0.2)
        if not readable:
            continue
        try:
            chunk = os.read(fd, 4096)
        except BlockingIOError:
            continue
        if chunk:
            data += chunk
    sys.exit(0 if b"OK" in data else 1)
finally:
    os.close(fd)
PY
}

at_command() {
  local tty=$1
  shift
  [ -e "$tty" ] || return 1
  [ "$#" -gt 0 ] || return 1
  command -v python3 >/dev/null 2>&1 || return 1
  timeout "$PPP_AT_PROBE_TIMEOUT" python3 - "$tty" "$@" <<'PY'
import os
import select
import sys
import termios
import time

path = sys.argv[1]
cmds = sys.argv[2:]
fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)

def read_until(deadline):
    data = b""
    while time.monotonic() < deadline:
        readable, _, _ = select.select([fd], [], [], 0.2)
        if not readable:
            continue
        try:
            chunk = os.read(fd, 4096)
        except BlockingIOError:
            continue
        if chunk:
            data += chunk
            if b"\r\nOK\r\n" in data or b"\r\nERROR\r\n" in data:
                break
    return data

try:
    attrs = termios.tcgetattr(fd)
    attrs[0] = 0
    attrs[1] = 0
    attrs[2] = termios.B115200 | termios.CS8 | termios.CREAD | termios.CLOCAL
    attrs[3] = 0
    attrs[4] = termios.B115200
    attrs[5] = termios.B115200
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    termios.tcflush(fd, termios.TCIOFLUSH)
    output = b""
    ok = True
    for cmd in cmds:
        os.write(fd, cmd.encode("ascii") + b"\r")
        data = read_until(time.monotonic() + 2.5)
        output += data
        if b"OK" not in data:
            ok = False
            break
        time.sleep(0.1)
    sys.stdout.write(output.decode("ascii", "replace"))
    sys.exit(0 if ok else 1)
finally:
    os.close(fd)
PY
}

find_ppp_tty_by_usb_interface() {
  local iface parent ifnum tty
  [ -n "$PPP_USB_INTERFACE" ] || return 1
  for iface in /sys/bus/usb/devices/*:*; do
    [ -r "$iface/bInterfaceNumber" ] || continue
    ifnum=$(cat "$iface/bInterfaceNumber" 2>/dev/null || true)
    [ "$ifnum" = "$PPP_USB_INTERFACE" ] || continue
    parent=${iface%%:*}
    [ "$(cat "$parent/idVendor" 2>/dev/null || true)" = "$USB_VENDOR" ] || continue
    [ "$(cat "$parent/idProduct" 2>/dev/null || true)" = "$USB_PRODUCT" ] || continue
    for tty in "$iface"/tty/ttyUSB*; do
      [ -e "$tty" ] || continue
      printf '/dev/%s\n' "${tty##*/}"
      return 0
    done
  done
  return 1
}

list_a7600c_ttys() {
  local iface parent tty
  for iface in /sys/bus/usb/devices/*:*; do
    [ -r "$iface/bInterfaceNumber" ] || continue
    parent=${iface%%:*}
    [ "$(cat "$parent/idVendor" 2>/dev/null || true)" = "$USB_VENDOR" ] || continue
    [ "$(cat "$parent/idProduct" 2>/dev/null || true)" = "$USB_PRODUCT" ] || continue
    for tty in "$iface"/tty/ttyUSB*; do
      [ -e "$tty" ] || continue
      printf '/dev/%s\n' "${tty##*/}"
    done
  done
}

select_ppp_tty() {
  local tty preferred_tty seen candidates
  SELECTED_PPP_TTY=""
  preferred_tty=$(find_ppp_tty_by_usb_interface 2>/dev/null || true)
  if [ -n "$preferred_tty" ] && [ -e "$preferred_tty" ]; then
    if [ "$PPP_SKIP_AT_PROBE" = "1" ]; then
      SELECTED_PPP_TTY="$preferred_tty"
      log "using A7600C USB interface ${PPP_USB_INTERFACE} PPP port without preflight AT probe: $SELECTED_PPP_TTY"
      return 0
    fi
    if at_ok "$preferred_tty"; then
      SELECTED_PPP_TTY="$preferred_tty"
      log "selected A7600C USB interface ${PPP_USB_INTERFACE} PPP port $SELECTED_PPP_TTY"
      return 0
    fi
    log "PPP AT probe failed on A7600C USB interface ${PPP_USB_INTERFACE} port $preferred_tty"
  fi
  candidates="$PPP_TTY $PPP_TTY_CANDIDATES $(list_a7600c_ttys 2>/dev/null || true)"
  seen=""
  for tty in $candidates; do
    [ -n "$tty" ] || continue
    case " $seen " in
      *" $tty "*) continue ;;
    esac
    seen="$seen $tty"
    [ -e "$tty" ] || continue
    if [ "$PPP_SKIP_AT_PROBE" = "1" ]; then
      SELECTED_PPP_TTY="$tty"
      log "using PPP port without preflight AT probe: $SELECTED_PPP_TTY"
      return 0
    fi
    if at_ok "$tty"; then
      SELECTED_PPP_TTY="$tty"
      log "selected PPP AT port $SELECTED_PPP_TTY"
      return 0
    fi
    log "PPP AT probe failed on $tty"
  done
  return 1
}

recent_ppp_dial_rejected() {
  command -v journalctl >/dev/null 2>&1 || return 1
  journalctl --since "90 seconds ago" --no-pager 2>/dev/null | \
    grep -Eq 'chat\[[0-9]+\]: (ERROR|NO CARRIER)|Failed \((ERROR|NO CARRIER)\)'
}

recover_ppp_modem_state() {
  local tty=$1 now last out
  [ "$ENABLE_PPP_MODEM_RECOVERY" = "1" ] || return 1
  [ -e "$tty" ] || return 1
  recent_ppp_dial_rejected || return 1

  now=$(monotonic_seconds)
  last=$(cat "$LAST_PPP_MODEM_RECOVERY" 2>/dev/null || echo 0)
  if [ "$last" -gt "$now" ] 2>/dev/null; then
    last=0
  fi
  if [ $((now - last)) -lt "$PPP_MODEM_RECOVERY_MIN_INTERVAL" ]; then
    log "PPP modem recovery cooldown active ($((now - last))/${PPP_MODEM_RECOVERY_MIN_INTERVAL}s)"
    return 1
  fi

  echo "$now" | run_root tee "$LAST_PPP_MODEM_RECOVERY" >/dev/null 2>&1 || true
  out=$(at_command "$tty" "AT+DIALMODE?" 'AT$MYCONFIG?' "AT+CGATT?" "AT+CGACT?" 2>/dev/null || true)
  log "PPP modem recovery readback on $tty: $(printf '%s' "$out" | tr '\r\n' ' ' | sed 's/[[:space:]][[:space:]]*/ /g' | tail -c 260)"
  log "PPP dial was rejected by modem; forcing manual dial mode and rebooting A7600C baseband"
  at_command "$tty" "AT+DIALMODE=1" >/dev/null 2>&1 || true
  at_command "$tty" "AT+CFUN=1,1" >/dev/null 2>&1 || true
  return 0
}

detect_ecm_if() {
  local target candidate
  target=$(printf '%s' "$ECM_MAC" | tr 'A-Z' 'a-z')
  if [ -n "$target" ]; then
    candidate=$(find_iface_by_mac "$target") || return 1
    if [ "$ACTIVE_ECM_IF" != "$candidate" ]; then
      log "A7600C ECM interface detected by MAC: $candidate ($target)"
    fi
    ACTIVE_ECM_IF="$candidate"
    return 0
  fi

  ACTIVE_ECM_IF="$ECM_IF"
  ip link show "$ACTIVE_ECM_IF" >/dev/null 2>&1
}

ensure_ecm_at_config() {
  local changed out tty candidate candidates seen
  changed=0
  [ "$ECM_AT_CONFIG" = "1" ] || return 0
  [ -e "$ECM_AT_CONFIG_MARKER" ] && return 0
  tty=""
  out=""
  candidates="$PPP_TTY $PPP_TTY_CANDIDATES"
  seen=""
  for candidate in $candidates; do
    [ -n "$candidate" ] || continue
    case " $seen " in
      *" $candidate "*) continue ;;
    esac
    seen="$seen $candidate"
    [ -e "$candidate" ] || continue
    if ! at_ok "$candidate"; then
      log "PPP AT probe failed on $candidate"
      continue
    fi
    SELECTED_PPP_TTY="$candidate"
    log "selected PPP AT port $SELECTED_PPP_TTY"
    out=$(at_command "$candidate" "AT+DIALMODE?" 'AT$MYCONFIG?' 2>/dev/null || true)
    log "ECM AT config readback on $candidate: $(printf '%s' "$out" | tr '\r\n' ' ' | sed 's/[[:space:]][[:space:]]*/ /g' | tail -c 220)"
    case "$out" in
      *DIALMODE*|*MYCONFIG*|*usbnetmode*|*USBNETMODE*)
        tty="$candidate"
        break
        ;;
      *)
        log "ECM AT config readback empty on $candidate; trying next AT port"
        ;;
    esac
  done
  if [ -z "$tty" ]; then
    log "ECM AT config skipped: no AT port returned config readback"
    return 1
  fi
  case "$out" in
    *"+DIALMODE: 0"*|*"+DIALMODE:0"*) ;;
    *)
      log "enabling USBNET auto dial with AT+DIALMODE=0"
      at_command "$tty" "AT+DIALMODE=0" >/dev/null 2>&1 || true
      changed=1
      ;;
  esac
  case "$out" in
    *'"usbnetmode",1'*|*'"USBNETMODE",1'*)
      ;;
    *)
      log "requesting ECM USBNET mode with AT\$MYCONFIG=\"USBNETMODE\",1"
      at_command "$tty" 'AT$MYCONFIG="USBNETMODE",1' >/dev/null 2>&1 || true
      changed=1
      ;;
  esac
  run_root mkdir -p "$(dirname "$ECM_AT_CONFIG_MARKER")" 2>/dev/null || true
  echo "$(monotonic_seconds)" | run_root tee "$ECM_AT_CONFIG_MARKER" >/dev/null 2>&1 || true
  if [ "$changed" = "1" ]; then
    log "ECM AT config changed; waiting for modem USB re-enumeration"
    return 2
  fi
}

connect_ppp() {
  local selected_tty
  if ppp_has_ip; then
    run_root ip route replace default dev "$PPP_IF" metric 50 2>/dev/null || true
    return 0
  fi
  [ -e "/etc/ppp/peers/${PPP_PEER}" ] || return 1

  if ppp_running; then
    log "pppd is running but $PPP_IF has no IPv4; restarting gently"
    run_root pkill -TERM -f "pppd .*(${PPP_TTY}|${PPP_RUNTIME_PEER}|${PPP_PEER})" 2>/dev/null || true
    sleep 3
  fi

  if [ "$STOP_MODEMMANAGER_FOR_PPP" = "1" ] && command -v systemctl >/dev/null 2>&1; then
    run_root systemctl stop ModemManager 2>/dev/null || true
  fi

  run_root rm -f /var/lock/LCK..ttyUSB0 /var/lock/LCK..ttyUSB1 /var/lock/LCK..ttyUSB2 /var/lock/LCK..ttyUSB3 2>/dev/null || true
  awk '
    /^[[:space:]]*\/dev\// {next}
    /^[[:space:]]*[0-9]+[[:space:]]*$/ {next}
    {print}
  ' "/etc/ppp/peers/${PPP_PEER}" >"$PPP_RUNTIME_PEER"
  if [ "$PPP_PRE_DIAL_DELAY" != "0" ]; then
    log "settling modem before PPP dial for ${PPP_PRE_DIAL_DELAY}s"
    sleep "$PPP_PRE_DIAL_DELAY"
  fi

  if ! select_ppp_tty; then
    log "no AT-responsive PPP tty found"
    return 1
  fi
  selected_tty="$SELECTED_PPP_TTY"

  log "starting pppd $selected_tty $PPP_BAUD file $PPP_RUNTIME_PEER"
  run_root pppd "$selected_tty" "$PPP_BAUD" file "$PPP_RUNTIME_PEER" >/tmp/go2-connectivity-pppd-start.log 2>&1 || {
    log "pppd start failed: $(tail -c 300 /tmp/go2-connectivity-pppd-start.log 2>/dev/null | tr '\n' ' ')"
    return 1
  }

  sleep 12
  if ppp_has_ip; then
    run_root ip route replace default dev "$PPP_IF" metric 50 2>/dev/null || true
    log "PPP online: $(ip -br addr show "$PPP_IF" 2>/dev/null)"
    return 0
  fi
  run_root pkill -TERM -f "pppd .*(${PPP_TTY}|${PPP_RUNTIME_PEER}|${PPP_PEER})" 2>/dev/null || true
  recover_ppp_modem_state "$selected_tty" || true
  return 1
}

connect_ecm() {
  local cfg_rc
  detect_ecm_if || return 1
  ensure_ecm_at_config
  cfg_rc=$?
  if [ "$cfg_rc" = "2" ]; then
    return 1
  fi
  run_root ip link set "$ACTIVE_ECM_IF" up 2>/dev/null || true
  if ! ecm_has_ip; then
    log "requesting DHCP on $ACTIVE_ECM_IF"
    stop_ecm_dhcp_clients
    sleep 1
    if command -v dhclient >/dev/null 2>&1; then
      run_root timeout 20 dhclient -1 -d -v "$ACTIVE_ECM_IF" >/tmp/go2-connectivity-dhclient.log 2>&1 || true
    elif command -v udhcpc >/dev/null 2>&1; then
      run_root timeout 15 udhcpc -n -q -i "$ACTIVE_ECM_IF" >/tmp/go2-connectivity-udhcpc.log 2>&1 || true
    fi
  fi
  if ecm_has_ip; then
    stop_ecm_dhcp_clients
    replace_ecm_default_route
    log "ECM online: $(ip -br addr show "$ACTIVE_ECM_IF" 2>/dev/null)"
    return 0
  fi
  return 1
}

connect_nmcli() {
  command -v nmcli >/dev/null 2>&1 || return 1
  nmcli -t -f NAME con show 2>/dev/null | grep -qx "$NM_CONNECTION" || return 1
  if nmcli_busy; then
    log "$NM_CONNECTION is busy; not interrupting"
    return 0
  fi
  log "activating nmcli connection $NM_CONNECTION"
  run_root nmcli con mod "$NM_CONNECTION" connection.autoconnect no connection.interface-name "" ipv4.never-default no ipv4.route-metric 50 ipv6.method ignore >/dev/null 2>&1 || true
  run_root nmcli con up "$NM_CONNECTION" >/tmp/go2-connectivity-nmcli-up.log 2>&1 || {
    log "nmcli up failed: $(tail -c 300 /tmp/go2-connectivity-nmcli-up.log 2>/dev/null | tr '\n' ' ')"
    return 1
  }
}

recover_once() {
  local now absent_after absent_elapsed stuck_if unstable_elapsed
  stuck_if=$(netdev_unregister_stuck_if || true)
  if [ -n "$stuck_if" ]; then
    cleanup_netdev_unregister_stuck "$stuck_if"
    return 1
  fi
  clear_netdev_stuck_state_if_gone
  disable_wired_default_route
  if ecm_has_ip; then
    replace_ecm_default_route
  fi
  set_usb_host_power_on || true
  unload_usbnet_drivers_for_ppp
  if online; then
    USB_PRESENT_FAILURE_SINCE=0
    bootstrap_time_if_needed || true
    log "online: $(ip route get "$TARGET_HOST" 2>/dev/null | head -1)"
    return 0
  fi

  if ! usb_present; then
    log "A7600C USB absent; waiting for hardware re-enumeration"
    stop_ppp_if_running
    now=$(monotonic_seconds)
    if [ "$USB_ABSENT_SINCE" -le 0 ] 2>/dev/null; then
      USB_ABSENT_SINCE="$now"
    fi
    USB_PRESENT_FAILURE_SINCE=0
    absent_after=$(usb_absent_rebind_after)
    absent_elapsed=$((now - USB_ABSENT_SINCE))
    if [ "$absent_elapsed" -lt "$absent_after" ]; then
      log "USB absent grace active before bus rebind (${absent_elapsed}/${absent_after}s)"
    elif xhci_dead; then
      log_xhci_dead_once
    else
      usb_bus_rebind_once || true
    fi
    return 1
  fi

  USB_ABSENT_SINCE=0
  USB_BUS_REBIND_ATTEMPTS=0
  run_root rm -f "$XHCI_DEAD_MARKER" "$LAST_XHCI_DEAD_LOG" 2>/dev/null || true
  set_usb_power_on || true
  if ! wait_usb_stable; then
    now=$(monotonic_seconds)
    if [ "$USB_UNSTABLE_SINCE" -le 0 ] 2>/dev/null; then
      USB_UNSTABLE_SINCE="$now"
    fi
    unstable_elapsed=$((now - USB_UNSTABLE_SINCE))
    if recent_usb_bus_fault; then
      log "A7600C USB is unstable and recent usb/xhci bus fault was observed"
    fi
    log "A7600C USB is not stable for ${STABLE_SECONDS}s (${unstable_elapsed}/${USB_UNSTABLE_REBIND_AFTER}s)"
    if [ "$unstable_elapsed" -ge "$USB_UNSTABLE_REBIND_AFTER" ] 2>/dev/null; then
      log "A7600C USB unstable for ${unstable_elapsed}s; allowing bus rebind"
      usb_bus_rebind_once || true
      USB_UNSTABLE_SINCE=0
    fi
    return 1
  fi
  USB_UNSTABLE_SINCE=0

  case "$MODE" in
    passive)
      log "passive mode; not changing bearer"
      ;;
    ppp)
      connect_ppp || usb_bus_rebind_once || usb_reset_once || true
      ;;
    ecm)
      connect_ecm || {
        if usb_present_rebind_due "ECM recovery"; then
          usb_bus_rebind_once || usb_reset_once || true
        fi
      }
      ;;
    nmcli)
      connect_nmcli || usb_bus_rebind_once || usb_reset_once || true
      ;;
    auto|*)
      if detect_ecm_if >/dev/null 2>&1; then
        connect_ecm || connect_ppp || connect_nmcli || usb_bus_rebind_once || usb_reset_once || true
      else
        connect_ppp || connect_ecm || connect_nmcli || usb_bus_rebind_once || usb_reset_once || true
      fi
      ;;
  esac

  if online; then
    USB_PRESENT_FAILURE_SINCE=0
    bootstrap_time_if_needed || true
    log "recovered: $(ip route get "$TARGET_HOST" 2>/dev/null | head -1)"
    return 0
  fi
  log "still offline; usb=$(usb_devices | tr '\n' ',' | sed 's/,$//') tty=$(ls /dev/ttyUSB* 2>/dev/null | tr '\n' ',' | sed 's/,$//') route=$(ip route get "$TARGET_HOST" 2>/dev/null | head -1)"
  return 1
}

main() {
  log "starting mode=$MODE target=${TARGET_HOST}:${TARGET_PORT}"
  disable_usb_autosuspend
  set_usb_host_power_on || true
  unload_usbnet_drivers_for_ppp
  [ "$MODE" = "ppp" ] && log "usbnet/RNDIS drivers guarded for PPP mode"
  while true; do
    recover_once || true
    [ "$RUN_ONCE" = "1" ] && break
    sleep "$INTERVAL"
  done
}

main "$@"

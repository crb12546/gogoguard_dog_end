#!/usr/bin/env bash
set -u

ETH_IF=${GO2_ETH_IF:-eth0}
ETH_MAC=${GO2_ETH_MAC:-4c:bb:47:ab:e4:c2}
GO2_ADDR=${GO2_ETH_ADDR:-192.168.123.18/24}
LIVOX_ADDR=${GO2_LIVOX_ADDR:-192.168.1.5/24}
FOURG_IF=${GO2_4G_IF:-eth1}
FOURG_MAC=${GO2_4G_MAC:-28:e3:57:c3:ed:9a}
FOURG_GW=${GO2_4G_GW:-192.168.0.1}
GSM_CONNECTION=${GO2_GSM_CONNECTION:-go2-4g}
ETH_CONNECTION=${GO2_ETH_CONNECTION:-Wired connection 1}
EXTRA_ETH_CONNECTION=${GO2_EXTRA_ETH_CONNECTION:-Wired connection 2}
WAIT_SECONDS=${GO2_4G_WAIT_SECONDS:-15}
USB_RESET_STAMP=${GO2_4G_USB_RESET_STAMP:-/run/go2-network-4g-usb-reset.done}
ENABLE_USB_RESET=${GO2_4G_ENABLE_USB_RESET:-0}

log() {
  printf '[go2-network] %s %s\n' "$(date '+%F %T')" "$*"
}

has_addr() {
  ip -4 addr show dev "$1" | grep -q " $2"
}

find_iface_by_mac() {
  local want_mac
  local fallback
  local path
  local name
  want_mac=$(printf '%s' "$1" | tr 'A-F' 'a-f')
  fallback=$2

  for path in /sys/class/net/*/address; do
    [ -r "$path" ] || continue
    if [ "$(tr 'A-F' 'a-f' <"$path")" = "$want_mac" ]; then
      name=$(basename "$(dirname "$path")")
      printf '%s\n' "$name"
      return 0
    fi
  done

  printf '%s\n' "$fallback"
}

find_4g_iface() {
  local by_mac
  local by_addr
  local by_ppp
  by_mac=$(find_iface_by_mac "$FOURG_MAC" "$FOURG_IF")
  if [ "$by_mac" != "$ETH_IF" ] && ip link show "$by_mac" >/dev/null 2>&1; then
    printf '%s\n' "$by_mac"
    return 0
  fi

  by_ppp=$(ip -o -4 addr show 2>/dev/null | awk '$2 ~ /^ppp[0-9]+$/ {print $2; exit}')
  if [ -n "$by_ppp" ]; then
    printf '%s\n' "$by_ppp"
    return 0
  fi

  by_addr=$(ip -o -4 addr show 2>/dev/null | awk -v eth_if="$ETH_IF" '$2 != eth_if && $4 ~ /^192\.168\.0\./ {print $2; exit}')
  if [ -n "$by_addr" ]; then
    printf '%s\n' "$by_addr"
    return 0
  fi

  if [ "$FOURG_IF" != "$ETH_IF" ] && ip link show "$FOURG_IF" >/dev/null 2>&1; then
    printf '%s\n' "$FOURG_IF"
    return 0
  fi

  printf '\n'
}

find_4g_usb_device() {
  local d
  for d in /sys/bus/usb/devices/*; do
    [ -r "$d/idVendor" ] || continue
    [ -r "$d/idProduct" ] || continue
    if [ "$(cat "$d/idVendor" 2>/dev/null)" = "1e0e" ] && [ "$(cat "$d/idProduct" 2>/dev/null)" = "9011" ]; then
      basename "$d"
      return 0
    fi
  done
  return 1
}

reset_4g_usb_once() {
  local dev
  if [ "$ENABLE_USB_RESET" != "1" ]; then
    log "A7600C USB reset disabled; use hot-plug or delayed USB power if cold boot leaves modem disabled"
    return 1
  fi

  dev=$(find_4g_usb_device || true)
  if [ -z "$dev" ]; then
    log "A7600C USB device not found for reset"
    return 1
  fi
  if [ -e "$USB_RESET_STAMP" ]; then
    log "A7600C USB reset already attempted this boot"
    return 1
  fi

  log "resetting A7600C USB device $dev"
  touch "$USB_RESET_STAMP" 2>/dev/null || true
  echo "$dev" >/sys/bus/usb/drivers/usb/unbind 2>/dev/null || true
  sleep 3
  echo "$dev" >/sys/bus/usb/drivers/usb/bind 2>/dev/null || true
  sleep 8
  return 0
}

ensure_eth0() {
  if ! ip link show "$ETH_IF" >/dev/null 2>&1; then
    log "$ETH_IF not found"
    return 0
  fi

  ip link set "$ETH_IF" up || true

  if ! has_addr "$ETH_IF" "$GO2_ADDR"; then
    log "adding $GO2_ADDR to $ETH_IF"
    ip addr add "$GO2_ADDR" dev "$ETH_IF" 2>/dev/null || true
  fi

  if ! has_addr "$ETH_IF" "$LIVOX_ADDR"; then
    log "adding $LIVOX_ADDR to $ETH_IF"
    ip addr add "$LIVOX_ADDR" dev "$ETH_IF" 2>/dev/null || true
  fi

  if command -v nmcli >/dev/null 2>&1; then
    nmcli con mod "$ETH_CONNECTION" connection.interface-name "$ETH_IF" >/dev/null 2>&1 || true
    nmcli con mod "$ETH_CONNECTION" ipv4.gateway "" ipv4.never-default yes ipv6.never-default yes >/dev/null 2>&1 || true
    nmcli con mod "$EXTRA_ETH_CONNECTION" connection.autoconnect no >/dev/null 2>&1 || true
  fi

  while ip route show default dev "$ETH_IF" | grep -q '^default '; do
    ip route del default dev "$ETH_IF" 2>/dev/null || break
  done
  while ip route show default via 192.168.123.1 dev "$ETH_IF" | grep -q '^default '; do
    ip route del default via 192.168.123.1 dev "$ETH_IF" 2>/dev/null || break
  done
}

gsm_device_state() {
  command -v nmcli >/dev/null 2>&1 || return 0
  nmcli -t -f DEVICE,TYPE,STATE,CONNECTION device status 2>/dev/null | \
    awk -F: -v conn="$GSM_CONNECTION" '$2 == "gsm" && $4 == conn {print $3; exit}'
}

gsm_should_not_interrupt() {
  local state
  state=$(gsm_device_state)
  case "$state" in
    connected*|connecting*|deactivating*|disconnecting*|prepare*|config*|ip-config*|secondaries*|activated*)
      log "$GSM_CONNECTION already busy/state=$state; not starting another activation"
      return 0
      ;;
  esac
  return 1
}

try_start_4g_profile() {
  local enable_failed=0
  if command -v mmcli >/dev/null 2>&1; then
    local modem_path
    local modem_id
    modem_path=$(mmcli -L 2>/dev/null | awk '/Modem\// {print $1; exit}')
    if [ -n "$modem_path" ]; then
      modem_id=${modem_path##*/}
      log "enabling modem $modem_id"
      if ! mmcli -m "$modem_id" --enable >/tmp/go2-network-mmcli-enable.log 2>&1; then
        log "mmcli enable failed: $(tr '\n' ' ' </tmp/go2-network-mmcli-enable.log | tail -c 300)"
        enable_failed=1
      fi
    fi
  fi

  if [ "$enable_failed" = "1" ]; then
    reset_4g_usb_once || true
  fi

  if command -v nmcli >/dev/null 2>&1 && nmcli -t -f NAME con show | grep -qx "$GSM_CONNECTION"; then
    if gsm_should_not_interrupt; then
      return 0
    fi
    nmcli con mod "$GSM_CONNECTION" connection.autoconnect no connection.interface-name "" ipv4.never-default no ipv4.route-metric 50 ipv6.method ignore >/dev/null 2>&1 || true
    log "activating $GSM_CONNECTION"
    if ! nmcli con up "$GSM_CONNECTION" >/tmp/go2-network-nmcli-up.log 2>&1; then
      log "nmcli up failed: $(tr '\n' ' ' </tmp/go2-network-nmcli-up.log | tail -c 300)"
    fi
  fi
}

ensure_4g_route() {
  try_start_4g_profile

  local waited=0
  while [ "$waited" -lt "$WAIT_SECONDS" ]; do
    FOURG_IF=$(find_4g_iface)
    if [ -n "$FOURG_IF" ] && ip -4 addr show dev "$FOURG_IF" 2>/dev/null | grep -q 'inet '; then
      break
    fi
    sleep 3
    waited=$((waited + 3))
  done

  if [ -z "$FOURG_IF" ] || ! ip -4 addr show dev "$FOURG_IF" 2>/dev/null | grep -q 'inet '; then
    reset_4g_usb_once || true
    try_start_4g_profile

    waited=0
    while [ "$waited" -lt "$WAIT_SECONDS" ]; do
      FOURG_IF=$(find_4g_iface)
      if [ -n "$FOURG_IF" ] && ip -4 addr show dev "$FOURG_IF" 2>/dev/null | grep -q 'inet '; then
        break
      fi
      sleep 3
      waited=$((waited + 3))
    done
  fi

  if [ -z "$FOURG_IF" ] || ! ip -4 addr show dev "$FOURG_IF" 2>/dev/null | grep -q 'inet '; then
    log "$FOURG_IF has no IPv4 yet; leaving default route unchanged"
    return 0
  fi

  if printf '%s\n' "$FOURG_IF" | grep -q '^ppp'; then
    ip route replace default dev "$FOURG_IF" metric 50 2>/dev/null || true
  else
    ip route replace default via "$FOURG_GW" dev "$FOURG_IF" metric 50 2>/dev/null || true
  fi
  log "default route: $(ip route get 8.8.8.8 2>/dev/null | head -1 || true)"
}

main() {
  ETH_IF=$(find_iface_by_mac "$ETH_MAC" "$ETH_IF")
  FOURG_IF=$(find_4g_iface)
  log "using eth_if=$ETH_IF 4g_if=$FOURG_IF"
  ensure_eth0
  ensure_4g_route
  ip -br addr show "$ETH_IF" 2>/dev/null || true
  ip -br addr show "$FOURG_IF" 2>/dev/null || true
  ip route | sed -n '1,8p'
}

main "$@"
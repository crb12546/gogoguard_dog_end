#!/usr/bin/env bash
set -u

WIRED_MAC=${GO2_WIRED_MAC:-4c:bb:47:ab:e4:c2}
WIRED_IPS=${GO2_WIRED_IPS:-"192.168.123.18/24 192.168.1.5/24 192.168.144.100/24"}
INTERVAL=${GO2_WIRED_RESCUE_INTERVAL:-5}
RUN_ONCE=${GO2_WIRED_RESCUE_RUN_ONCE:-0}

log() {
  printf '[go2-wired-ssh-rescue] %s %s\n' "$(date '+%F %T')" "$*"
}

lower() {
  printf '%s' "$1" | tr 'A-Z' 'a-z'
}

find_wired_if() {
  local iface addr target
  target=$(lower "$WIRED_MAC")
  for iface in /sys/class/net/*; do
    [ -r "$iface/address" ] || continue
    addr=$(lower "$(cat "$iface/address" 2>/dev/null)")
    if [ "$addr" = "$target" ]; then
      basename "$iface"
      return 0
    fi
  done
  return 1
}

ip_on_iface() {
  local cidr iface
  cidr=$1
  iface=$2
  ip -4 addr show dev "$iface" 2>/dev/null | awk '{print $2}' | grep -Fxq "$cidr"
}

remove_ip_from_other_ifaces() {
  local cidr target iface
  cidr=$1
  target=$2
  ip -o -4 addr show 2>/dev/null | awk -v cidr="$cidr" '$4 == cidr {print $2}' | sort -u | while read -r iface; do
    [ -n "$iface" ] || continue
    [ "$iface" = "$target" ] && continue
    if ip addr del "$cidr" dev "$iface" 2>/dev/null; then
      log "removed wired SSH address $cidr from non-wired interface $iface"
    fi
  done
}

ensure_wired_ssh_once() {
  local iface cidr changed
  iface=$(find_wired_if) || {
    log "wired SSH interface absent; waiting for MAC $WIRED_MAC"
    return 1
  }

  ip link set "$iface" up 2>/dev/null || true

  changed=0
  for cidr in $WIRED_IPS; do
    remove_ip_from_other_ifaces "$cidr" "$iface"
    if ! ip_on_iface "$cidr" "$iface"; then
      if ip addr add "$cidr" dev "$iface" 2>/dev/null; then
        log "added wired SSH address $cidr to $iface"
        changed=1
      fi
    fi
  done

  if [ "$changed" = "1" ]; then
    log "wired SSH ready: $(ip -br addr show dev "$iface" 2>/dev/null)"
  fi
  return 0
}

if [ "$RUN_ONCE" = "1" ]; then
  ensure_wired_ssh_once
  exit $?
fi

log "starting wired SSH rescue for MAC $WIRED_MAC"
while true; do
  ensure_wired_ssh_once || true
  sleep "$INTERVAL"
done

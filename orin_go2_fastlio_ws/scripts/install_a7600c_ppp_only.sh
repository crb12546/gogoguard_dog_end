#!/usr/bin/env bash
set -euo pipefail

WS=${GO2_WS:-/home/unitree/go2_fastlio_ws}
SERVICE_NAME=${GO2_CONNECTIVITY_SERVICE:-go2-connectivity-watchdog.service}

sudo_run() {
  if [ -n "${SUDO_PASSWORD:-}" ]; then
    printf '%s\n' "$SUDO_PASSWORD" | sudo -S -p '' "$@"
  else
    sudo "$@"
  fi
}

install_text() {
  local mode=$1
  local path=$2
  local tmp
  tmp=$(mktemp)
  cat >"$tmp"
  sudo_run install -m "$mode" "$tmp" "$path"
  rm -f "$tmp"
}

install_text 0644 /etc/modprobe.d/go2-a7600c-no-usbnet.conf <<'EOF'
# Keep the A7600C from binding as a USB Ethernet/RNDIS device.
# On this Orin build that path has repeatedly destabilized tegra-xusb.
blacklist cdc_ether
blacklist rndis_host
blacklist cdc_subset
blacklist usbnet
install cdc_ether /bin/false
install rndis_host /bin/false
install cdc_subset /bin/false
install usbnet /bin/false
EOF

install_text 0644 /etc/udev/rules.d/78-go2-a7600c-ignore-ttyusb1.rules <<'EOF'
ACTION!="add|change|move|bind", GOTO="go2_a7600c_end"
SUBSYSTEM!="tty", GOTO="go2_a7600c_end"
ATTRS{idVendor}=="1e0e", ATTRS{idProduct}=="9011", ATTRS{bInterfaceNumber}=="04", ENV{ID_MM_PORT_IGNORE}="1"
LABEL="go2_a7600c_end"
EOF

install_text 0755 /usr/local/sbin/go2-a7600c-usbnet-unbind <<'EOF'
#!/usr/bin/env bash
set -u

log() {
  logger -t go2-a7600c-usbnet "$*" 2>/dev/null || true
}

unbind_one() {
  local name=$1
  local path parent driver_path driver
  name=${name#/sys/bus/usb/devices/}
  path="/sys/bus/usb/devices/${name}"
  [ -d "$path" ] || return 0
  [ -r "$path/bInterfaceNumber" ] || return 0
  parent=${name%%:*}
  [ -r "/sys/bus/usb/devices/${parent}/idVendor" ] || return 0
  [ -r "/sys/bus/usb/devices/${parent}/idProduct" ] || return 0
  [ "$(cat "/sys/bus/usb/devices/${parent}/idVendor" 2>/dev/null)" = "1e0e" ] || return 0
  [ "$(cat "/sys/bus/usb/devices/${parent}/idProduct" 2>/dev/null)" = "9011" ] || return 0

  driver_path=$(readlink -f "$path/driver" 2>/dev/null || true)
  [ -n "$driver_path" ] || return 0
  driver=$(basename "$driver_path")
  case "$driver" in
    cdc_ether|rndis_host|cdc_subset|usbnet)
      printf '%s' "$name" >"${driver_path}/unbind" 2>/dev/null || true
      log "unbound ${driver} from ${name}"
      ;;
  esac
}

if [ "$#" -gt 0 ]; then
  for name in "$@"; do
    unbind_one "$name"
  done
else
  for iface in /sys/bus/usb/devices/*:*; do
    [ -e "$iface" ] || continue
    unbind_one "$(basename "$iface")"
  done
fi
EOF

install_text 0644 /etc/udev/rules.d/77-go2-a7600c-no-usbnet-bind.rules <<'EOF'
ACTION=="bind", SUBSYSTEM=="usb", DEVTYPE=="usb_interface", ATTRS{idVendor}=="1e0e", ATTRS{idProduct}=="9011", RUN+="/usr/local/sbin/go2-a7600c-usbnet-unbind %k"
ACTION=="add|change", SUBSYSTEM=="net", ATTR{address}=="28:e3:57:c3:ed:9a", ENV{NM_UNMANAGED}="1"
EOF

sudo_run mkdir -p /etc/NetworkManager/conf.d
install_text 0644 /etc/NetworkManager/conf.d/99-go2-a7600c-unmanaged.conf <<'EOF'
[keyfile]
unmanaged-devices=mac:28:E3:57:C3:ED:9A
EOF

sudo_run udevadm control --reload-rules || true
if command -v nmcli >/dev/null 2>&1; then
  sudo_run nmcli general reload 2>/dev/null || true
fi

if command -v systemctl >/dev/null 2>&1; then
  sudo_run systemctl disable --now ModemManager.service 2>/dev/null || true
fi

sudo_run modprobe -r rndis_host cdc_ether cdc_subset usbnet 2>/dev/null || true
sudo_run /usr/local/sbin/go2-a7600c-usbnet-unbind || true

if [ ! -e /etc/ppp/peers/go2-a7600c ]; then
  echo "missing /etc/ppp/peers/go2-a7600c; install PPP chat/peer config first" >&2
  exit 1
fi

if [ ! -x "${WS}/scripts/install_connectivity_watchdog.sh" ]; then
  chmod +x "${WS}/scripts/install_connectivity_watchdog.sh" 2>/dev/null || true
fi

SUDO_PASSWORD=${SUDO_PASSWORD:-} \
GO2_WS="$WS" \
GO2_CONNECTIVITY_SERVICE="$SERVICE_NAME" \
GO2_4G_MODE=ppp \
GO2_4G_ENABLE_USB_RESET=1 \
GO2_4G_ENABLE_USB_POWER_CONTROL=0 \
GO2_4G_ENABLE_USB_BUS_REBIND=0 \
GO2_4G_USB_RESET_MIN_INTERVAL=300 \
GO2_4G_ENABLE_HOST_RESET=0 \
GO2_4G_USB_BUS_REBIND_MAX_ATTEMPTS=3 \
GO2_4G_USB_BUS_REBIND_BACKOFF=300 \
GO2_4G_XHCI_DEAD_LOG_MIN_INTERVAL=60 \
"${WS}/scripts/install_connectivity_watchdog.sh"

sudo_run mkdir -p "/etc/systemd/system/${SERVICE_NAME}.d"
install_text 0644 "/etc/systemd/system/${SERVICE_NAME}.d/zz-go2-4g-ppp-primary.conf" <<'EOF'
[Service]
# ECM/DHCP causes complete A7600C USB resets on this Orin. Keep USB bus
# recovery disabled; permit one device-level reset every five minutes only
# after PPP itself cannot recover.
Environment=GO2_4G_MODE=ppp
Environment=GO2_4G_ENABLE_USB_POWER_CONTROL=0
Environment=GO2_4G_ENABLE_USB_BUS_REBIND=0
Environment=GO2_4G_ENABLE_USB_RESET=1
Environment=GO2_4G_USB_RESET_MIN_INTERVAL=300
Environment=GO2_4G_ECM_AT_CONFIG=0
Environment=GO2_4G_PPP_PRE_DIAL_DELAY=8
Environment=GO2_4G_PPP_SKIP_AT_PROBE=0
Environment=GO2_4G_PPP_USB_INTERFACE=05
Environment=GO2_4G_ENABLE_PPP_MODEM_RECOVERY=1
Environment=GO2_4G_PPP_MODEM_RECOVERY_MIN_INTERVAL=300
Environment="GO2_4G_PPP_TTY_CANDIDATES=/dev/ttyUSB2 /dev/ttyUSB3 /dev/ttyUSB1 /dev/ttyUSB0 /dev/ttyUSB4"
StandardOutput=journal
StandardError=journal
EOF

sudo_run systemctl daemon-reload
sudo_run systemctl restart "$SERVICE_NAME"
sudo_run systemctl --no-pager --full status "$SERVICE_NAME" || true

#!/usr/bin/env bash
set -euo pipefail

WS=${GO2_WS:-/home/unitree/go2_fastlio_ws}
SERVICE_NAME=${GO2_CONNECTIVITY_SERVICE:-go2-connectivity-watchdog.service}
ECM_MAC=${GO2_4G_ECM_MAC:-28:e3:57:c3:ed:9a}
ECM_IF=${GO2_4G_ECM_IF:-go2_4g}

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

install_text 0644 /etc/udev/rules.d/77-go2-a7600c-usbnet-ecm.rules <<EOF
ACTION=="add|change", SUBSYSTEM=="net", ATTR{address}=="${ECM_MAC}", ENV{NM_UNMANAGED}="1"
EOF

sudo_run mkdir -p /etc/systemd/network
install_text 0644 /etc/systemd/network/10-go2-a7600c.link <<EOF
[Match]
MACAddress=${ECM_MAC}

[Link]
Name=${ECM_IF}
EOF

sudo_run rm -f /etc/udev/rules.d/77-go2-a7600c-no-usbnet-bind.rules
sudo_run mkdir -p /etc/NetworkManager/conf.d
install_text 0644 /etc/NetworkManager/conf.d/99-go2-a7600c-unmanaged.conf <<EOF
[keyfile]
unmanaged-devices=mac:${ECM_MAC};interface-name:${ECM_IF}
EOF

sudo_run udevadm control --reload-rules || true
if command -v nmcli >/dev/null 2>&1; then
  sudo_run nmcli general reload 2>/dev/null || true
fi

if command -v systemctl >/dev/null 2>&1; then
  sudo_run systemctl disable --now ModemManager.service 2>/dev/null || true
fi

if [ ! -x "${WS}/scripts/install_connectivity_watchdog.sh" ]; then
  chmod +x "${WS}/scripts/install_connectivity_watchdog.sh" 2>/dev/null || true
fi

SUDO_PASSWORD=${SUDO_PASSWORD:-} \
GO2_WS="$WS" \
GO2_CONNECTIVITY_SERVICE="$SERVICE_NAME" \
GO2_4G_MODE=ecm \
GO2_4G_ENABLE_USB_RESET=0 \
GO2_4G_ENABLE_USB_POWER_CONTROL=${GO2_4G_ENABLE_USB_POWER_CONTROL:-1} \
GO2_4G_ENABLE_HOST_RESET=0 \
GO2_4G_ECM_IF=${ECM_IF} \
GO2_4G_ECM_MAC="$ECM_MAC" \
GO2_4G_ECM_GW=${GO2_4G_ECM_GW:-192.168.0.1} \
GO2_4G_ECM_AT_CONFIG=${GO2_4G_ECM_AT_CONFIG:-1} \
GO2_4G_USB_BUS_REBIND_MAX_ATTEMPTS=${GO2_4G_USB_BUS_REBIND_MAX_ATTEMPTS:-3} \
GO2_4G_USB_BUS_REBIND_BACKOFF=${GO2_4G_USB_BUS_REBIND_BACKOFF:-300} \
GO2_4G_XHCI_DEAD_LOG_MIN_INTERVAL=${GO2_4G_XHCI_DEAD_LOG_MIN_INTERVAL:-60} \
"${WS}/scripts/install_connectivity_watchdog.sh"

sudo_run mkdir -p "/etc/systemd/system/${SERVICE_NAME}.d"
install_text 0644 "/etc/systemd/system/${SERVICE_NAME}.d/override.conf" <<EOF
[Service]
Environment=GO2_4G_MODE=ecm
Environment=GO2_4G_ENABLE_HOST_RESET=0
Environment=GO2_4G_ENABLE_USB_POWER_CONTROL=${GO2_4G_ENABLE_USB_POWER_CONTROL:-1}
Environment=GO2_4G_ECM_IF=${ECM_IF}
Environment=GO2_4G_ECM_MAC=${ECM_MAC}
Environment=GO2_4G_ECM_GW=${GO2_4G_ECM_GW:-192.168.0.1}
Environment=GO2_4G_ECM_AT_CONFIG=${GO2_4G_ECM_AT_CONFIG:-1}
Environment=GO2_4G_USB_BUS_REBIND_MAX_ATTEMPTS=${GO2_4G_USB_BUS_REBIND_MAX_ATTEMPTS:-3}
Environment=GO2_4G_USB_BUS_REBIND_BACKOFF=${GO2_4G_USB_BUS_REBIND_BACKOFF:-300}
Environment=GO2_4G_XHCI_DEAD_LOG_MIN_INTERVAL=${GO2_4G_XHCI_DEAD_LOG_MIN_INTERVAL:-60}
EOF

sudo_run systemctl daemon-reload
sudo_run systemctl restart "$SERVICE_NAME"
sudo_run systemctl --no-pager --full status "$SERVICE_NAME" || true

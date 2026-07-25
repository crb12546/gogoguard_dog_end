#!/usr/bin/env bash
set -euo pipefail

# Install the single-owner A7600C ECM auto-connect manager. Run as root.

if [[ "${EUID}" -ne 0 ]]; then
  echo "run this installer as root" >&2
  exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SOURCE_MANAGER="${SCRIPT_DIR}/go2_4g_manager.py"
INSTALL_ROOT="${GO2_INSTALL_ROOT:-/home/unitree/go2_fastlio_ws}"
INSTALL_SCRIPTS="${INSTALL_ROOT}/scripts"
INSTALLED_MANAGER="${INSTALL_SCRIPTS}/go2_4g_manager.py"
BACKUP_BASE="${INSTALL_ROOT}/backups"
TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
BACKUP_DIR="${BACKUP_BASE}/4g_manager_${TIMESTAMP}"
ECM_MAC="${GO2_4G_ECM_MAC:-28:e3:57:c3:ed:9a}"
ECM_IF="${GO2_4G_ECM_INTERFACE:-go2_4g}"

if [[ ! -f "${SOURCE_MANAGER}" ]]; then
  echo "missing ${SOURCE_MANAGER}" >&2
  exit 1
fi
if [[ "${ECM_IF}" == "eth0" || "${ECM_IF}" == "eth1" ]]; then
  echo "refusing to use reserved interface ${ECM_IF}" >&2
  exit 1
fi

mkdir -p "${INSTALL_SCRIPTS}" "${BACKUP_DIR}" /var/lib/dhcp

backup_path() {
  local source_path="$1"
  if [[ -e "${source_path}" || -L "${source_path}" ]]; then
    cp -a --parents "${source_path}" "${BACKUP_DIR}"
    echo "${source_path}" >>"${BACKUP_DIR}/present-before-install.txt"
  fi
}

BACKUP_PATHS=(
  /etc/systemd/system/go2-4g-manager.service
  /etc/systemd/system/go2-xhci-keep-awake.service
  /etc/systemd/system/go2-4g-rndis-block.service
  /etc/systemd/system/go2-4g-serial-isolation.service
  /etc/systemd/system/go2-4g-cfun-test.service
  /etc/systemd/system/go2-connectivity-watchdog.service
  /etc/systemd/system/go2-connectivity-watchdog.service.d
  /etc/systemd/system/go2-network-recover.timer
  /etc/systemd/system/go2-network-recover.service
  /etc/modprobe.d/go2-a7600c-no-usbnet.conf
  /etc/modprobe.d/go2-a7600c-no-rndis.conf
  /etc/udev/rules.d/40-go2-a7600c-no-mtp.rules
  /etc/udev/rules.d/39-go2-xhci-keep-awake.rules
  /etc/udev/rules.d/77-go2-a7600c-usbnet-ecm.rules
  /etc/udev/rules.d/77-go2-a7600c-no-usbnet-bind.rules
  /etc/udev/rules.d/77-go2-a7600c-ppp.rules
  /etc/udev/rules.d/77-go2-a7600c-ppp.rules.disabled
  /etc/udev/rules.d/78-go2-a7600c-ignore-ttyusb1.rules
  /etc/systemd/network/10-go2-a7600c.link
  /etc/NetworkManager/conf.d/99-go2-a7600c-unmanaged.conf
  /usr/local/sbin/go2-a7600c-usbnet-unbind
  /usr/local/sbin/go2-a7600c-ppp-unbind-usbnet
  /etc/ppp/peers/a7600c
  /etc/ppp/peers/go2-a7600c
  /etc/chatscripts/go2-4g-manager
  /etc/chatscripts/go2-4g-manager-disconnect
  "${INSTALLED_MANAGER}"
)

for path in "${BACKUP_PATHS[@]}"; do
  backup_path "${path}"
done

cat >"${BACKUP_DIR}/README.txt" <<EOF
Backup made before installing the vendor-mode ECM manager at ${TIMESTAMP}.
The manager never selects or configures the robot's eth0 or eth1 interfaces.
EOF
echo "backup=${BACKUP_DIR}"

# Stop every former owner of the modem before changing persistent settings.
systemctl disable --now go2-connectivity-watchdog.service 2>/dev/null || true
systemctl disable --now go2-network-recover.timer 2>/dev/null || true
systemctl stop go2-network-recover.service 2>/dev/null || true
systemctl disable --now go2-4g-manager.service 2>/dev/null || true
systemctl stop go2-4g-serial-isolation.service 2>/dev/null || true
systemctl stop go2-4g-cfun-test.service 2>/dev/null || true
systemctl disable --now ModemManager.service 2>/dev/null || true
pkill -TERM -f '(^|/)pppd( .*call (a7600c|go2-a7600c)| .*ttyUSB)' 2>/dev/null || true

dhclient_pid_file="/run/go2-4g-dhclient-${ECM_IF}.pid"
if [[ -f "${dhclient_pid_file}" ]]; then
  dhclient_pid="$(tr -cd '0-9' <"${dhclient_pid_file}" || true)"
  if [[ -n "${dhclient_pid}" && -r "/proc/${dhclient_pid}/cmdline" ]] &&
     tr '\0' ' ' <"/proc/${dhclient_pid}/cmdline" | grep -q "${ECM_IF}"; then
    kill -TERM "${dhclient_pid}" 2>/dev/null || true
  fi
fi
sleep 1

# Remove the accumulated PPP/ECM rules which competed for the same device.
rm -rf /etc/systemd/system/go2-connectivity-watchdog.service.d
rm -f /etc/systemd/system/go2-connectivity-watchdog.service
rm -f /etc/systemd/system/go2-4g-serial-isolation.service
rm -f /etc/systemd/system/go2-4g-cfun-test.service
rm -f /etc/modprobe.d/go2-a7600c-no-usbnet.conf
rm -f /etc/udev/rules.d/77-go2-a7600c-no-usbnet-bind.rules
rm -f /etc/udev/rules.d/77-go2-a7600c-ppp.rules
rm -f /etc/udev/rules.d/77-go2-a7600c-ppp.rules.disabled
rm -f /etc/udev/rules.d/78-go2-a7600c-ignore-ttyusb1.rules
rm -f /usr/local/sbin/go2-a7600c-usbnet-unbind
rm -f /usr/local/sbin/go2-a7600c-ppp-unbind-usbnet

install -m 0755 "${SOURCE_MANAGER}" "${INSTALLED_MANAGER}"

# Keep the Tegra USB host out of ELPG runtime suspend.  After a storm of
# A7600C error -71 disconnects this Jetson has repeatedly failed to reload the
# XUSB Falcon firmware on resume, leaving runtime_status=error until reboot.
# Keep NVIDIA's HCD remove/probe recovery disabled.  On this R35.3.1 kernel it
# emits IRQ-domain warnings and can leave the controller firmware inaccessible.
cat >/etc/systemd/system/go2-xhci-keep-awake.service <<'EOF'
[Unit]
Description=Keep the Go2 Tegra XUSB host awake
After=systemd-udevd.service
Before=go2-4g-manager.service

[Service]
Type=oneshot
ExecStart=/bin/sh -c 'for p in /sys/devices/platform/3610000.xhci/power/control /sys/bus/usb/devices/usb1/power/control; do [ ! -w "$p" ] || printf "on\n" >"$p"; done; p=/sys/module/xhci_tegra/parameters/en_hcd_reinit; [ ! -w "$p" ] || printf "N\n" >"$p"'
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF

# Apply power/control=on as soon as udev sees the platform controller and root
# hub.  This runs several seconds before the regular manager service on boot.
cat >/etc/udev/rules.d/39-go2-xhci-keep-awake.rules <<'EOF'
ACTION=="add", SUBSYSTEM=="platform", KERNEL=="3610000.xhci", TEST=="power/control", ATTR{power/control}="on"
ACTION=="add", SUBSYSTEM=="usb", KERNEL=="usb1", TEST=="power/control", ATTR{power/control}="on"
EOF

# If the modem was left in Windows/RNDIS mode, keep that host driver detached
# just long enough for the manager to restore Linux/ECM mode through the AT
# port. ECM itself uses the Jetson's built-in cdc_ether driver.
cat >/etc/modprobe.d/go2-a7600c-no-rndis.conf <<'EOF'
blacklist rndis_host
install rndis_host /bin/false
EOF
chmod 0644 /etc/modprobe.d/go2-a7600c-no-rndis.conf

cat >/etc/systemd/system/go2-4g-rndis-block.service <<'EOF'
[Unit]
Description=Keep recovery RNDIS detached until A7600C returns to Linux ECM
After=systemd-modules-load.service
Before=go2-4g-manager.service

[Service]
Type=oneshot
ExecStart=/bin/sh -c 'if grep -q "^rndis_host " /proc/modules; then /sbin/modprobe -r rndis_host; fi'
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF

cat >/etc/udev/rules.d/40-go2-a7600c-no-mtp.rules <<'EOF'
# The A7600C is owned by go2-4g-manager, not MTP or ModemManager.
SUBSYSTEM=="usb", ATTR{idVendor}=="1e0e", ATTR{idProduct}=="9011", ENV{ID_MTP_DEVICE}="0", ENV{ID_MM_DEVICE_IGNORE}="1"
EOF

cat >/etc/udev/rules.d/77-go2-a7600c-usbnet-ecm.rules <<EOF
# Keep NetworkManager away from only the A7600C ECM MAC.
ACTION=="add|change", SUBSYSTEM=="net", ATTR{address}=="${ECM_MAC}", ENV{NM_UNMANAGED}="1"
EOF

mkdir -p /etc/systemd/network
cat >/etc/systemd/network/10-go2-a7600c.link <<EOF
[Match]
MACAddress=${ECM_MAC}

[Link]
Name=${ECM_IF}
EOF

mkdir -p /etc/NetworkManager/conf.d
cat >/etc/NetworkManager/conf.d/99-go2-a7600c-unmanaged.conf <<EOF
[keyfile]
unmanaged-devices=mac:${ECM_MAC};interface-name:${ECM_IF}
EOF

cat >/etc/systemd/system/go2-4g-manager.service <<EOF
[Unit]
Description=Go2 A7600C vendor-mode ECM auto-connect manager
After=systemd-udevd.service go2-xhci-keep-awake.service go2-4g-rndis-block.service network-pre.target
Wants=go2-xhci-keep-awake.service go2-4g-rndis-block.service network-pre.target
Conflicts=go2-connectivity-watchdog.service
StartLimitIntervalSec=0

[Service]
Type=simple
User=root
ExecStartPre=/sbin/modprobe option
ExecStart=/usr/bin/python3 -u ${INSTALLED_MANAGER}
Restart=always
RestartSec=3
TimeoutStopSec=20
KillMode=control-group
Environment=PYTHONUNBUFFERED=1
Environment=GO2_4G_XHCI_DEVICE=3610000.xhci
Environment=GO2_4G_USB_BUS_DEVICE=usb1
Environment=GO2_4G_USB_VID=1e0e
Environment=GO2_4G_USB_PID=9011
Environment=GO2_4G_SWITCH_VID=2ecc
Environment=GO2_4G_SWITCH_PID=3001
Environment=GO2_4G_ECM_INTERFACE=${ECM_IF}
Environment=GO2_4G_ECM_GATEWAY=192.168.0.1
Environment=GO2_4G_APN=cmnet
Environment=GO2_4G_CLOUD_HOST=39.96.37.187
Environment=GO2_4G_CLOUD_PORT=443
Environment=GO2_4G_FALLBACK_HOST=223.5.5.5
Environment=GO2_4G_FALLBACK_PORT=53
Environment=GO2_4G_LOOP_SECONDS=5
Environment=GO2_4G_USB_SETTLE_SECONDS=3
Environment=GO2_4G_AT_RETRY_SECONDS=20
Environment=GO2_4G_DHCP_RETRY_SECONDS=10
Environment=GO2_4G_DHCP_TIMEOUT_SECONDS=25
Environment=GO2_4G_HEALTH_FAILURES=3
Environment=GO2_4G_LINK_RESTART_AFTER=45
Environment=GO2_4G_MODEM_RESET_AFTER=180
Environment=GO2_4G_MODEM_RESET_COOLDOWN=300
Environment=GO2_4G_RAPID_USB_LIMIT=3
Environment=GO2_4G_RAPID_USB_WINDOW=90
Environment=GO2_4G_RADIO_OFF_SECONDS=30
Environment=GO2_4G_STATE_FILE=/run/go2-4g-manager-state.json

[Install]
WantedBy=multi-user.target
EOF

udevadm control --reload-rules
systemctl daemon-reload
systemctl enable --now go2-xhci-keep-awake.service
systemctl enable --now go2-4g-rndis-block.service
systemctl enable --now go2-4g-manager.service

echo "installed_service=go2-4g-manager.service"
echo "mode=ecm-auto"
echo "interface=${ECM_IF}"
echo "state_file=/run/go2-4g-manager-state.json"

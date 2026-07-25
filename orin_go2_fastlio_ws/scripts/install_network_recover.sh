#!/usr/bin/env bash
set -euo pipefail

WS=${GO2_WS:-/home/unitree/go2_fastlio_ws}
SCRIPT=${WS}/scripts/go2_network_recover.sh

if [ ! -f "$SCRIPT" ]; then
  echo "missing $SCRIPT"
  echo "copy scripts/go2_network_recover.sh to the Orin workspace first"
  exit 1
fi

chmod +x "$SCRIPT"

echo "[install] installing go2-network-recover.service and timer"

sudo tee /etc/systemd/system/go2-network-recover.service >/dev/null <<SERVICE
[Unit]
Description=GO2 network recovery for eth0 and A7600C 4G
After=NetworkManager.service ModemManager.service
Wants=NetworkManager.service ModemManager.service

[Service]
Type=oneshot
TimeoutStartSec=60
ExecStart=${SCRIPT}
SERVICE

sudo tee /etc/systemd/system/go2-network-recover.timer >/dev/null <<'TIMER'
[Unit]
Description=Run GO2 network recovery periodically

[Timer]
OnBootSec=20s
OnUnitActiveSec=30s
AccuracySec=5s
Unit=go2-network-recover.service

[Install]
WantedBy=timers.target
TIMER

sudo systemctl daemon-reload
sudo systemctl enable go2-network-recover.timer
sudo systemctl restart go2-network-recover.timer
sudo systemctl start go2-network-recover.service

echo "[install] installed"
echo "  sudo systemctl status go2-network-recover.service --no-pager"
echo "  sudo systemctl status go2-network-recover.timer --no-pager"
echo "  journalctl -u go2-network-recover.service -n 100 --no-pager"
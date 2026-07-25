#!/usr/bin/env bash
set -e

echo "[install] installing root-level lidar network service..."

sudo tee /etc/systemd/system/go2-lidar-network.service >/dev/null <<'SERVICE'
[Unit]
Description=Configure wired MAC static IPv4 for MID-360 lidar
DefaultDependencies=no
Before=network-online.target
Wants=network-online.target

[Service]
Type=oneshot
RemainAfterExit=yes
Environment=GO2_WIRED_MAC=4c:bb:47:ab:e4:c2
ExecStart=/bin/bash -lc 'target=$(printf %s "${GO2_WIRED_MAC}" | tr A-Z a-z); iface=$(for i in /sys/class/net/*; do [ -r "$i/address" ] || continue; addr=$(cat "$i/address" 2>/dev/null | tr A-Z a-z); [ "$addr" = "$target" ] && basename "$i" && break; done); [ -n "$iface" ] || { echo "wired interface not found for MAC $target" >&2; exit 1; }; /usr/sbin/ip link set "$iface" up; /usr/sbin/ip -4 addr show dev "$iface" | grep -q "192.168.1.5/24" || /usr/sbin/ip addr add 192.168.1.5/24 dev "$iface"; /usr/sbin/ip -4 addr show dev "$iface"'

[Install]
WantedBy=multi-user.target
SERVICE

sudo systemctl daemon-reload
sudo systemctl enable go2-lidar-network.service
sudo systemctl restart go2-lidar-network.service

echo "[install] installing user-level GO2 FAST-LIO base service..."

mkdir -p ~/.config/systemd/user

cat > ~/.config/systemd/user/go2-fastlio-base.service <<'SERVICE'
[Unit]
Description=GO2 FAST-LIO base bringup: Livox then FAST-LIO
After=default.target

[Service]
Type=simple
ExecStart=/home/unitree/go2_fastlio_ws/scripts/base_bringup.sh
ExecStop=/home/unitree/go2_fastlio_ws/scripts/base_stop.sh
Restart=on-failure
RestartSec=5

[Install]
WantedBy=default.target
SERVICE

systemctl --user daemon-reload
systemctl --user enable go2-fastlio-base.service

echo ""
echo "已安装："
echo "  1. go2-lidar-network.service       root-level, configure 192.168.1.5/24 by wired MAC"
echo "  2. go2-fastlio-base.service        用户级，启动 Livox + FAST-LIO"
echo ""
echo "现在可以测试："
echo "  sudo systemctl status go2-lidar-network.service"
echo "  systemctl --user start go2-fastlio-base.service"
echo "  systemctl --user status go2-fastlio-base.service"
echo ""
echo "查看日志："
echo "  journalctl -u go2-lidar-network.service -n 100 --no-pager"
echo "  journalctl --user -u go2-fastlio-base.service -f"
echo ""
echo "如果需要开机不登录也启动用户服务，执行："
echo "  sudo loginctl enable-linger unitree"

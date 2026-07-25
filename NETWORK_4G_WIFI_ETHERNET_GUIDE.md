# Go2 网络配置经验：网线、WiFi/热点、A7600C 4G

更新时间: 2026-07-08

本文整理 Go2 + Orin 巡检项目里关于网线、WiFi/热点、A7600C 4G 的配置经验、实际做法、原理和踩坑记录。信息来源包括项目文档、脚本、交接记录和仓库记忆。

结论先行：

```text
网线 = 最可靠的本地维护/兜底链路，低延迟，不依赖公网。
WiFi/热点 = 现场默认人机调试链路，Mac 和狗进同一个局域网。
4G = 狗端自己上公网，用于 GoGoGuard 心跳/视频/命令；不适合作为本地 SSH 主链路。
```

---

## 1. 总体网络拓扑

### 1.1 设备和地址

当前项目里几个网络是叠在 Orin 上的：

```text
Mac / 本地控制端
  en16 / AX88179A USB 网卡: 192.168.123.222/24
  WiFi: 家里 WiFi 或 iPhone 热点

Orin / 狗背板 Linux
  eth0:
    192.168.123.18/24     # Go2 内部网 / 有线维护网
    192.168.1.5/24        # Livox MID-360S 雷达网
    192.168.144.100/24    # Z-1Pro 摄像头网
  wlan0:
    172.20.10.2           # iPhone 热点下常见地址
    192.168.0.122         # 家里 WiFi 历史地址
  4G:
    eth1 / 192.168.0.100  # A7600C NAT/CDC ECM 模式曾出现
    ppp0 / 10.x.x.x/32    # A7600C PPP/GSM 模式成功状态

Go2 本体:       192.168.123.161
Livox MID-360S: 192.168.1.161
Z-1Pro:         192.168.144.108
GoGoGuard:      39.96.37.187
```

### 1.2 三类链路的分工

```text
go2wired -> 192.168.123.18
  Mac 用网线直连 Orin。最快、最稳，适合维护、救援、重配网络。

go2 -> 172.20.10.2
  Orin 和 Mac 同时连 iPhone 热点。现场默认无线调试链路。

go2home -> 192.168.0.122
  Orin 和 Mac 同时连家里/办公室 WiFi。室内调试方便。

4G / go2-4g
  Orin 自己连公网，给 GoGoGuard 上传心跳/视频/命令结果。
  它不解决 Mac 到狗的本地 SSH，除非另做公网穿透/VPN。
```

管理台后端的 SSH 探测顺序也是这个思路：

```text
go2wired -> go2 -> go2home
```

也就是说插上网线时自动优先用网线；现场没网线时走热点；家里调试可走 `go2home`。

---

## 2. 网线链路

### 2.1 我们怎么做的

Mac 侧 USB 网卡是 AX88179A，接口名记录为 `en16`。已把它持久配置成：

```text
192.168.123.222/24
```

对应命令：

```bash
networksetup -setmanual "AX88179A" 192.168.123.222 255.255.255.0
```

Orin 侧有线维护地址：

```text
192.168.123.18/24
```

SSH alias：

```text
ssh go2wired
```

### 2.2 原理

Go2 内部网络本来就在 `192.168.123.0/24`。Orin `eth0` 上的 `192.168.123.18/24` 既能和 Go2 本体 `192.168.123.161` 通信，也能让 Mac 通过同网段直连 Orin。

网线链路不需要 DHCP、不需要路由器、不需要公网。只要：

```text
Mac en16 = 192.168.123.222/24
Orin eth0 = 192.168.123.18/24
```

二者就可以直接 SSH。

### 2.3 为什么不要让 eth0 抢默认路由

`eth0` 同时承载 Go2/Livox/Z-1Pro 的内部网络，不应该拿默认网关。否则 Orin 访问公网时可能把流量错误地扔到内部网，导致 4G/WiFi 互联网不可用。

我们对有线连接的原则是：

```text
eth0 只管局域网设备，不管公网默认路由。
WiFi 或 4G 才负责默认路由。
```

NetworkManager 层面的方向：

```bash
sudo nmcli con mod "Wired connection 1" ipv4.gateway "" ipv4.never-default yes ipv6.never-default yes
sudo nmcli con mod "Wired connection 2" connection.autoconnect no
```

### 2.4 Orin eth0 多 IP

Orin 的 `eth0` 不是只配一个 IP，而是同时挂多个子网：

```bash
sudo ip addr add 192.168.123.18/24 dev eth0   # Go2 / Mac 维护网
sudo ip addr add 192.168.1.5/24 dev eth0      # Livox 网
sudo ip addr add 192.168.144.100/24 dev eth0  # Z-1Pro 网
```

`base_bringup.sh` 至少会确保 `192.168.1.5/24` 存在，因为 Livox 驱动依赖它。

早期 `install_autostart.sh` 还安装过 root 级服务：

```text
go2-lidar-network.service
```

它的作用是开机时给 `eth0` 加 `192.168.1.5/24`，保证 Livox 网络在 FAST-LIO 前准备好。

### 2.5 验证命令

Mac：

```bash
ssh go2wired 'hostname; ip -br addr; ip route | sed -n "1,8p"'
```

Orin 到内部设备：

```bash
ssh go2wired 'ping -c 2 192.168.123.161; ping -c 2 192.168.1.161; ping -c 2 192.168.144.108'
```

预期：

```text
192.168.123.161  # Go2 本体可 ping，但 SSH 被拒是正常的
192.168.1.161    # Livox 可 ping
192.168.144.108  # Z-1Pro 可 ping
```

### 2.6 经验和坑

```text
1. go2wired 是救援优先级最高的链路。
2. Mac 静态 IP 要持久配置，否则重插网卡后会丢。
3. eth0 不要设置默认网关。
4. Go2 本体 192.168.123.161 可以 ping，但 SSH refused 正常，不要把它当 Linux 主机。
5. ubuntu.local 在热点下不可靠，它会通告内部 eth0 IP；直接用 ssh alias。
```

---

## 3. WiFi / iPhone 热点链路

### 3.1 我们怎么做的

Orin 上使用 USB WiFi：

```text
UGREEN CM763
芯片/驱动: AIC 8800D80 / aic8800_fdrv
接口: wlan0
```

配置过 iPhone 热点 profile：

```text
connection name: iphone_hotspot
SSID: Constantine‘s iPhone 14 Pro Ma
```

注意：iOS 热点 SSID 会被截断到 32 字节，所以记录里没有最后的 `x`。

最终成功状态：

```text
Orin 热点 IP: 172.20.10.2
Mac 热点 IP:  172.20.10.5
SSH alias: ssh go2
```

家里 WiFi 也验证过：

```text
Orin wlan0: 192.168.0.122
SSH alias: ssh go2home
```

### 3.2 iPhone 热点关键配置

iPhone 热点踩坑主要不是密码错，而是 WPA/PMF 兼容性。

关键配置：

```text
wifi-sec.key-mgmt = wpa-psk
wifi-sec.pmf = disable
802-11-wireless.bssid = ""
connection.autoconnect = yes
```

原因：

```text
1. iPhone 热点倾向 WPA2/WPA3 兼容模式。
2. AIC 8800D80 驱动在 WPA3/PMF 协商时会关联后约 10 秒被踢，报 Secrets were required。
3. 禁用 PMF 后稳定。
4. iPhone 热点 BSSID 会变化，不能绑定固定 BSSID。
```

模板命令：

```bash
sudo nmcli con add type wifi ifname wlan0 con-name iphone_hotspot ssid '<iPhone热点SSID>'
sudo nmcli con mod iphone_hotspot \
  wifi-sec.key-mgmt wpa-psk \
  wifi-sec.psk '<热点密码>' \
  wifi-sec.pmf disable \
  802-11-wireless.bssid "" \
  connection.autoconnect yes
sudo nmcli con up iphone_hotspot
```

不要把真实热点密码写进仓库或文档。

### 3.3 现场开机顺序

推荐：

```text
1. 先打开 iPhone 个人热点，并停留在个人热点设置页。
2. 再开狗/Orin。
3. 等 Orin 自动连上热点。
4. Mac 也连同一个 iPhone 热点。
5. 用 ssh go2 或管理台连接。
```

原因：

```text
iPhone 热点在没有客户端时可能休眠；一旦狗连上，热点一般会保持醒着。
```

### 3.4 macOS 的坑

```text
1. macOS 可能自动跳到自己的 iPhone 热点或别的 WiFi。
2. 狗和 Mac 必须在同一个热点/同一个 WiFi。
3. 如果 ssh go2 不通，先确认 Mac 当前 WiFi 是不是同一个热点。
4. 如果狗的热点 IP 不是 172.20.10.2，检查 iPhone 客户端列表或扫 172.20.10.2-14。
```

### 3.5 验证命令

```bash
ssh go2 'hostname; ip -br addr; iw dev wlan0 link; ip route | sed -n "1,8p"'
```

管理台也会抓 WiFi 信号：

```bash
iw dev wlan0 link | grep -oE "signal: -[0-9]+"
```

### 3.6 家里 WiFi

家里调试时使用：

```text
ssh go2home
```

历史地址：

```text
192.168.0.122
```

家里 WiFi 的原则和热点相同：Mac 与 Orin 必须在同一局域网；不要依赖 `ubuntu.local`。

---

## 4. A7600C 4G 卡

### 4.1 目标和边界

4G 的目标是让 Orin 自己上公网，主要服务：

```text
1. GoGoGuard heartbeat
2. 视频上传
3. command-loop 拉平台命令并回传结果
```

4G 不等于 Mac 可以直接 SSH 狗。蜂窝网络通常在运营商 NAT 后面，没有公网入站能力。除非另做 VPN/反向隧道，否则本地维护仍要用：

```text
go2wired / go2 / go2home
```

### 4.2 硬件和识别状态

4G 卡：A7600C。

USB ID：

```text
1e0e:9011
```

曾观察到：

```text
ModemManager: /Modem/0
primary port: ttyUSB2
drivers: cdc_ether + option + usbserial
运营商: China Mobile LTE
```

出现过两种网络形态：

```text
NAT/CDC ECM:
  eth1 = 192.168.0.100/24
  gateway = 192.168.0.1

PPP/GSM:
  ppp0 = 10.x.x.x/32
  default dev ppp0
```

最终 live GoGoGuard 测试里，4G 直连公网走的是 `ppp0`。

### 4.3 推荐配置

NetworkManager profile：

```text
connection name: go2-4g
APN: cmnet
IPv6: ignore
autoconnect: no
interface-name: 空 / 不绑定
ipv4.never-default: no
ipv4.route-metric: 50
DNS: 223.5.5.5, 8.8.8.8
```

模板命令：

```bash
sudo nmcli con add type gsm ifname "*" con-name go2-4g apn cmnet
sudo nmcli con mod go2-4g \
  connection.autoconnect no \
  connection.interface-name "" \
  ipv4.method auto \
  ipv4.never-default no \
  ipv4.route-metric 50 \
  ipv4.dns "223.5.5.5 8.8.8.8" \
  ipv6.method ignore
```

为什么 `autoconnect no`：

```text
A7600C 在 Orin 上热插后可用，但 NetworkManager 自动激活和人工/脚本激活会互相打架。
我们遇到过 prepare -> deactivating、connected -> disconnecting，然后 ttyUSB2 timeout，modem invalid。
稳定做法是手动只 up 一次。
```

### 4.4 稳定启动流程

已经验证相对稳定的流程：

```text
1. 不插 4G，先启动 Orin。
2. 通过 go2wired / go2 / go2home 确认能 SSH。
3. 热插 A7600C。
4. 等 ModemManager 看到 /Modem/N。
5. 手动只执行一次 sudo nmcli con up go2-4g。
6. 检查 ip route 和 ping。
```

命令：

```bash
mmcli -L
mmcli -m 0
sudo nmcli con up go2-4g
nmcli -t -f DEVICE,TYPE,STATE,CONNECTION device status
ip -br addr
ip route
ip route get 39.96.37.187
ping -c 3 39.96.37.187
```

成功状态可能是：

```text
ttyUSB2:gsm:connected:go2-4g
ppp0: 10.x.x.x/32
default dev ppp0
ping 39.96.37.187: OK
```

或者 NAT 模式：

```text
eth1: 192.168.0.100/24
default via 192.168.0.1 dev eth1
```

### 4.5 冷启动插着 4G 的坑

非常重要：

```text
A7600C 插着再冷启动，在当前 Orin USB 路径上不稳定。
```

现象：

```text
modem remains disabled
mmcli --enable -> No such device
kernel: usb_wwan_open ... failed: -19
lsusb / sysfs 可能卡住
```

经验结论：

```text
不要依赖“插着 4G 冷启动”。
现场稳定流程是先开 Orin，再热插 4G。
如果要无人值守，需要延迟上电 USB hub、可控 USB 电源，或换更稳定的 modem。
```

### 4.6 go2-network-recover 的经验

脚本路径：

```text
orin_go2_fastlio_ws/scripts/go2_network_recover.sh
```

设计目的：

```text
1. 根据 MAC 找回 Orin 有线口。
2. 确保 eth0 有 192.168.123.18/24 和 192.168.1.5/24。
3. 尝试恢复 4G 默认路由。
```

安装脚本：

```text
orin_go2_fastlio_ws/scripts/install_network_recover.sh
```

它会安装：

```text
go2-network-recover.service
go2-network-recover.timer
```

timer 默认：

```text
OnBootSec=20s
OnUnitActiveSec=30s
```

但是我们后续学到：

```text
现场调 4G 时，不建议让它周期运行。
```

原因：

```text
1. 重复 nmcli con up 会打断正在连接/已连接的 A7600C。
2. 自动激活和手动激活会竞争，导致 ttyUSB2 timeout / modem invalid。
3. 旧版本只按 eth1/192.168.0.1 NAT 模式写路由，不能覆盖 ppp0 场景。
```

特别注意：当前本地仓库里的 `go2_network_recover.sh` 仍能看到会把 `go2-4g` 改成 `autoconnect yes`、`never-default yes` 并执行 `nmcli con up` 的逻辑。这和后续 handoff 里记录的“已修复版本”不完全一致。也就是说：

```text
不要盲目重新安装/启动本地这版 go2-network-recover.timer。
如果要用，先按最终经验改掉自动反复 up 4G 的逻辑，并确认支持 ppp0。
```

推荐现场状态：

```bash
sudo systemctl stop go2-network-recover.timer go2-network-recover.service
sudo systemctl disable go2-network-recover.timer
```

除非你明确需要它恢复 eth0 地址。

### 4.7 GoGoGuard 域名/IP 经验

4G 下发现：

```text
gogoguard.cn 可能因 ICP/TLS/域名链路被 reset 或 blocked。
```

稳定测试用：

```text
https://39.96.37.187/api/v1
```

后续 `go2_saas_agent.py` 里还做过 routeUrl rewrite：平台给 `https://gogoguard.cn/...` 时，狗端按 `GO2_BACKEND_BASE` 改写到 `39.96.37.187`。

---

## 5. SSH alias 和管理台策略

### 5.1 SSH alias

三个常用 alias：

```text
go2wired -> 192.168.123.18
go2      -> 172.20.10.2
go2home  -> 192.168.0.122
```

建议 `~/.ssh/config` 里设置较短 keepalive：

```text
Host go2wired
  HostName 192.168.123.18
  User unitree
  IdentityFile ~/.ssh/go2_orin_ed25519
  ServerAliveInterval 5
  ServerAliveCountMax 2

Host go2
  HostName 172.20.10.2
  User unitree
  IdentityFile ~/.ssh/go2_orin_ed25519
  ServerAliveInterval 5
  ServerAliveCountMax 2

Host go2home
  HostName 192.168.0.122
  User unitree
  IdentityFile ~/.ssh/go2_orin_ed25519
  ServerAliveInterval 5
  ServerAliveCountMax 2
```

### 5.2 管理台自动探测

本地管理台后端使用：

```text
HOSTS = ["go2wired", "go2", "go2home"]
```

也就是：

```text
1. 有网线先用网线。
2. 没网线就用 iPhone 热点。
3. 家里调试可用 go2home。
```

断线后约 10 秒会重新探测 SSH，自动切换。

---

## 6. 推荐现场网络流程

### 6.1 只做本地采集/巡检调试

```text
1. iPhone 开热点，并停在个人热点设置页。
2. 开狗，等 Orin 连上热点。
3. Mac 连同一个热点。
4. ssh go2 测试。
5. 启动本地管理台。
6. 网线带着，出问题插线走 go2wired。
```

### 6.2 家里/办公室调试

```text
1. Orin 连家里 WiFi。
2. Mac 连同一个 WiFi。
3. ssh go2home。
4. 如果需要摄像头，确认 eth0 仍有 192.168.144.100/24。
5. 如果需要 Livox，确认 eth0 仍有 192.168.1.5/24。
```

### 6.3 需要 GoGoGuard 联调/公网

```text
1. 先用 go2wired 或 WiFi 确认 Orin 可维护。
2. 不插 4G 冷启动。
3. Orin 启动稳定后，热插 A7600C。
4. mmcli -L 看到 modem。
5. sudo nmcli con up go2-4g 只执行一次。
6. ip route get 39.96.37.187 确认走 ppp0 或 4G eth1。
7. 再启动 GoGoGuard patrol-loop / command-loop。
```

### 6.4 故障救援优先级

```text
1. 插网线，ssh go2wired。
2. 如果网线不通，确认 Mac en16 是否还在 192.168.123.222/24。
3. 如果热点不通，确认 Mac 和狗是否在同一热点、iPhone 热点是否休眠、Orin IP 是否变了。
4. 如果 4G 不通，不要反复 nmcli con up；先查 mmcli -L / nmcli device / journalctl。
5. 4G 冷启动异常时，拔插 4G 或重启 Orin 后热插。
```

---

## 7. 常用诊断命令

### 7.1 总览

```bash
ssh go2wired 'date; hostname; ip -br addr; ip route; nmcli -t -f DEVICE,TYPE,STATE,CONNECTION device status'
```

### 7.2 eth0 内部设备

```bash
ssh go2wired 'ping -c 2 192.168.123.161; ping -c 2 192.168.1.161; ping -c 2 192.168.144.108'
```

### 7.3 WiFi

```bash
ssh go2 'iw dev wlan0 link; ip -br addr show wlan0; ip route | sed -n "1,8p"'
```

### 7.4 4G

```bash
ssh go2wired 'mmcli -L; nmcli -t -f DEVICE,TYPE,STATE,CONNECTION device status; ip -br addr; ip route; ip route get 39.96.37.187'
```

### 7.5 NetworkManager profile

```bash
ssh go2wired 'nmcli con show go2-4g | egrep "connection.autoconnect|connection.interface-name|ipv4.never-default|ipv4.route-metric|ipv6.method"'
```

### 7.6 systemd 网络恢复状态

```bash
ssh go2wired 'systemctl status go2-network-recover.timer --no-pager; systemctl status go2-network-recover.service --no-pager'
```

---

## 8. 最关键的经验总结

```text
1. 网线是救命绳：Mac 静态 192.168.123.222，Orin 192.168.123.18。
2. eth0 承载内部设备，不应抢默认路由。
3. WiFi/热点用于人机调试，iPhone 热点必须处理 WPA/PMF 和 BSSID 变化。
4. ubuntu.local 不可靠，用 ssh alias。
5. A7600C 适合热插，不适合插着冷启动。
6. go2-4g 要手动只 up 一次，避免 NetworkManager/脚本反复激活。
7. 4G 是狗上公网，不是本地 SSH 入口。
8. gogoguard.cn 在 4G 链路上不稳，优先用 39.96.37.187。
9. go2-network-recover 的思路对 eth0 有用，但对 4G 自动恢复要谨慎，旧脚本会制造连接竞争。
10. 做任何运动测试前，网络只是基础；还要确认 FAST-LIO 未漂、电量足、控制节点没有残留。
```

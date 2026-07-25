# 09 · 网络与 4G(几种连接方式 + 怎么保活)

> 原则同 00。核心文件:`scripts/go2_4g_manager.py`(1800 行,ECM/PPP 单一所有者管理器)、
> `scripts/install_a7600c_ecm_only.sh`、`scripts/install_a7600c_ppp_only.sh`、
> `scripts/go2_connectivity_watchdog.sh` / `install_connectivity_watchdog.sh`、
> `scripts/go2_a7600c_usb_monitor.sh`、`scripts/go2_network_recover.sh`、`scripts/go2_wired_ssh_rescue.sh`。
> **深度**:install 脚本 + manager 的 docstring/Config/main/关键函数 docstring 已读;1800 行 USB 恢复细节按需再深读。

## 一、整机三条网络路径
| 路径 | 网卡/设备 | 用途 |
|---|---|---|
| **有线以太网** | `eth1`(按 MAC `4c:bb:47:ab:e4:c2` 找,`env_common.sh`) | 接 **Livox 雷达**(192.168.1.5↔161)、Go2 SDK、SSH 救援 |
| **WiFi** | 板载 | 现场本地调试 |
| **4G** | SIMCom **A7600C**(USB `1e0e:9011`) | 野外远程联网(连 llyj SaaS 云端) |

## 二、4G 一共两种连接方式(互斥安装,回答你的问题)

模块是 **SIMCom A7600C**。两种模式二选一:

### ① ECM(USB 网卡,厂商默认) — `install_a7600c_ecm_only.sh`
- 模块以 **USB CDC-ECM** 呈现一个 USB 以太网设备,用 systemd `.link` 按 MAC `28:e3:57:c3:ed:9a` **重命名为 `go2_4g`**,走 **DHCP**,网关 `192.168.0.1`。
- 厂商 **自动拨号**(`AT+DIALMODE=0`);NetworkManager 设为不管(`NM_UNMANAGED`),禁用 `ModemManager`。
- 启动环境:`GO2_4G_MODE=ecm`,USB reset 关、USB 电源控制开、host reset 关。

### ② PPP(串口拨号) — `install_a7600c_ppp_only.sh`
- **显式拉黑 USB 网卡驱动**(`cdc_ether/rndis_host/cdc_subset/usbnet` blacklist + `install ... /bin/false`)。
  > ⚠️ 脚本注释原话:*"On this Orin build that [USB Ethernet] path has repeatedly destabilized tegra-xusb"* —— **ECM/DHCP 会导致 A7600C 整个 USB 复位、搞崩 Orin 的 Tegra XHCI 控制器**。
- 改用 **PPP over 串口**:`/etc/ppp/peers/go2-a7600c` 拨号,PPP USB 接口 `05`,tty 候选 `ttyUSB2/3/1/0/4`。
- 启动环境:`GO2_4G_MODE=ppp`,USB 总线 rebind 关、USB 电源控制关,**只允许每 5 分钟一次设备级 reset**(且仅在 PPP 自己恢复不了时)。

> **这就是"乱"的一处历史根因**:ECM 更简单但在这台 Orin 上不稳定 → 引入 PPP 作为更稳的替代。两套 install 脚本互斥,现场用哪套要看部署机器。

## 三、`go2_4g_manager.py`:单一所有者连接管理器
- 作为 `go2-connectivity-watchdog.service` 常驻运行(`Manager(Config.from_env()).run()`),`--once` 可单次、`--self-test` 自检。
- **AT 探测模块健康**:`AT+CPIN?`(SIM)、`AT+CSQ`(信号)、`AT+CEREG?`(注册)、`AT+DIALMODE?`、`AT$MYCONFIG?`(usbnet 配置)。
- **保活**:ECM 模式确保厂商自动拨号 + DHCP 拿到 `go2_4g` 的 IP;PPP 模式确保拨号进程在。
- **确保厂商 Linux 模式**:`ensure_vendor_linux_mode` 若发现 `DIALMODE≠0` 会 `AT+CGDCONT=1,"IP",<APN>` + `AT+DIALMODE=0` 复原。
- **谨慎恢复**(多重保护,从函数 docstring 核实):
  - USB 重新枚举 / XHCI 复位 / 模块复位,各带**冷却**(`modem_reset_after=180s`,`cooldown=300s`,`rapid_usb_limit`)。
  - **巡检作业期间禁止破坏性恢复**:`Return robot jobs during which an automatic reboot is unsafe` / `Block disruptive recovery during and immediately after robot jobs` —— 巡检中不重启/不拔 USB,避免打断任务。
  - **不信 Jetson 的 RTC**(开机 NTP 前时钟可能是错的)做限速。
- 状态机 `set_state("MODE_CONFIG", ...)` 等。

## 四、配套脚本
- `go2_a7600c_usb_monitor.sh`:监视 A7600C 的 USB 出现/消失。
- `go2_connectivity_watchdog.sh` + `install_*`:把 manager 装成 systemd 服务、注入模式环境变量。
- `go2_network_recover.sh` + `install_network_recover.sh`:网络恢复(重置网卡/路由)。
- `go2_wired_ssh_rescue.sh`:有线 SSH 救援(4G/WiFi 都挂时从有线进狗)。

## 五、留待坐实(按需深读 1800 行 manager)
- USB 总线 rebind / XHCI dead 判定 / 电源控制 的**具体 sysfs 操作序列**。
- PPP 拨号失败后的模块恢复流程细节(`ENABLE_PPP_MODEM_RECOVERY`)。
- `NETWORK_4G_WIFI_ETHERNET_GUIDE.md` 与代码的逐条一致性核对(文档按惯例不可信)。

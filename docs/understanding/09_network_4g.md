# 09 · 网络与 4G(几种连接方式 + 怎么保活)

> 原则同 00。核心文件(真实路径前缀 `S/` = `orin_go2_fastlio_ws/scripts/`):
> `S/go2_4g_manager.py`(**1594 行**,**ECM-only** 单一所有者管理器)、`S/install_go2_4g_manager.sh`(它的真正安装器/服务)、
> `S/go2_connectivity_watchdog.sh` / `S/install_connectivity_watchdog.sh`(**另一套** ECM/PPP/nmcli 多模 watchdog)、
> `S/install_a7600c_ecm_only.sh`、`S/install_a7600c_ppp_only.sh`(给 watchdog 装 ECM/PPP 模式的 drop-in)、
> `S/go2_a7600c_usb_monitor.sh`、`S/go2_network_recover.sh` / `S/install_network_recover.sh`(旧 nmcli 恢复)、`S/go2_wired_ssh_rescue.sh`、`S/env_common.sh`、`S/base_bringup.sh`。
> **深度**:install 脚本 + 两套实现的 docstring/Config/main/关键函数已读并逐条核过磁盘源码;1594 行里 USB 恢复的 sysfs 细节按需再深读。

## 核验状态

> 本轮已对**磁盘源码逐条核对**(`S/` 下 12 个网络相关文件 + Livox config + 巡检 manifest),核对基线见文末「## 核验台账」。
> **一句话结论**:数值/阈值/AT 指令/ECM-vs-PPP 安装分裂等细节大多**属实(CONFIRMED)**;但原文有一处**贯穿全篇的结构性错误**——把 Python 的 `go2_4g_manager.py`(纯 ECM、由 `go2-4g-manager.service` 跑、`install_go2_4g_manager.sh` 装)和 Bash 的 `go2_connectivity_watchdog.sh`(ECM/PPP/nmcli 多模、由 `go2-connectivity-watchdog.service` 跑、`install_connectivity_watchdog.sh` 装)**混为一体**。二者是**显式 `Conflicts` 的两套独立实现/服务**,`install_go2_4g_manager.sh` 安装时会 disable 并删掉 watchdog。本轮已把它们拆开(见 §二~§五)。
> **本轮已更正 7 处**:①行数 1800→1594;②manager 是 ECM-only(非 ECM/PPP);③manager 跑在 `go2-4g-manager.service`(非 `go2-connectivity-watchdog.service`);④PPP 保活/恢复归 Bash watchdog(非 manager);⑤`install_connectivity_watchdog.sh` 的 ExecStart 是 Bash 脚本本身(非 python manager);⑥`ENABLE_PPP_MODEM_RECOVERY` 在 Bash watchdog;⑦有线网卡狗上实为 `eth0`(非 `eth1`)。另**补齐遗漏文件** `install_go2_4g_manager.sh`。
> **仍无法验证【DOG_UNKNOWN】**:狗上到底跑哪一套(三套安装器/服务并存且互斥,无任一网络文件的狗上副本,manifest 只含巡检运行期字段)。**所有 12 个网络源文件均【无狗上对照】**——不得默认等同狗上。
> **源标签**:【默认 code:F:L】磁盘源码默认值 · 【生产 saas:F:L】install 脚本注入/生产值 · 【狗上 dog:证据】狗上运行期实证 · 【README对照】仅文档有代码里无 · 【推断-未验】。文中 `F` 省略 `S/` 前缀时即指 `orin_go2_fastlio_ws/scripts/`。

## 一、整机三条网络路径
| 路径 | 网卡/设备 | 用途 |
|---|---|---|
| **有线以太网** | **`eth0`**(按 MAC `4c:bb:47:ab:e4:c2` 找) | 接 **Livox 雷达**(192.168.1.5↔161)、Go2 SDK、SSH 救援 |
| **WiFi** | 板载 | 现场本地调试 |
| **4G** | SIMCom **A7600C**(USB `1e0e:9011`) | 野外远程联网(连云端 SaaS) |

- **有线网卡名 = `eth0`(狗上实证)**,原文写 `eth1` 有误。【狗上 dog:manifest `xunjian-20260725-06` `sdk_if=eth0`】【默认 code:go2_network_recover.sh:4 `ETH_IF=…eth0`】。
  - `eth1` 只是两处**兜底字面量**、不是有线口真名:`env_common.sh:28` 在按 MAC 找不到网卡时 `||printf eth1`【默认 code:env_common.sh:28】;而 `go2_network_recover.sh:8 FOURG_IF=…eth1` 里 `eth1` 恰恰是 **4G** 的兜底名【默认 code:go2_network_recover.sh:8】。所以"有线=eth1"既与狗上矛盾、又和 4G 兜底名撞车。
  - 按 MAC 查找这一机制本身是对的:`env_common.sh:11 GO2_WIRED_MAC=4c:bb:47:ab:e4:c2` + `find_iface_by_mac`【默认 code:env_common.sh:11-28】。
- **Livox 雷达**:`192.168.1.161`(MID360)↔ 主机侧 `192.168.1.5`。【默认 code:base_bringup.sh:33,49 `ip route replace 192.168.1.161/32 dev <iface> src 192.168.1.5`】【默认 code:go2_network_recover.sh:7 `LIVOX_ADDR=192.168.1.5/24`】【默认 code:src/Livox-SDK2/samples/logger/config.json host_ip 192.168.1.5】。
- **Go2 SDK / SSH 救援**同走有线口:`env_common.sh:29 GO2_SDK_IF=GO2_WIRED_IF`【默认 code:env_common.sh:29】;救援脚本按 MAC 加 `192.168.123.18 / 192.168.1.5 / 192.168.144.100`【默认 code:go2_wired_ssh_rescue.sh:4-5】。
- **4G 模块** SIMCom **A7600C**,USB `1e0e:9011`。【默认 code:go2_4g_manager.py:152-153】【生产 saas:install_a7600c_ppp_only.sh:41,63-64,90】【默认 code:go2_connectivity_watchdog.sh:11-12】。
- **"llyj SaaS 云端"** 中的 `llyj` 只是文档命名,**代码里无**。可核验的云端端点是 `39.96.37.187:443`(fallback `223.5.5.5:53`)。【README对照:llyj 仅见 docs 00/09】【默认 code:go2_4g_manager.py:159-160,161-162】【默认 code:go2_connectivity_watchdog.sh:9-10】。

## 二、三套互斥的 4G 管理栈(核心澄清 — 原文最大结构错误在此)

狗上历史沉淀了**三条独立、互相冲突**的 4G 管理实现。原文把其中两条(Bash watchdog / Python manager)当成一条讲,导致后文张冠李戴。真实分布:

| 栈 | 脚本 | 安装器 → systemd 服务 | 模式 | 说明 |
|---|---|---|---|---|
| **A · Bash watchdog** | `go2_connectivity_watchdog.sh` | `install_connectivity_watchdog.sh` → **`go2-connectivity-watchdog.service`**(ExecStart=**Bash 脚本本身**) | **ECM / PPP / nmcli / auto** 多模,靠 `GO2_4G_MODE` env 选 | ECM/PPP 双模逻辑、PPP 保活与恢复**都在这**。模式由 §三 两个 `install_a7600c_*_only.sh` drop-in 决定 |
| **B · Python manager** | `go2_4g_manager.py`(**1594 行**) | `install_go2_4g_manager.sh` → **`go2-4g-manager.service`**(ExecStart=`python3 -u …go2_4g_manager.py`) | **ECM-only**(硬编码 `ecm-auto`) | 看起来是**最新的"单一所有者"合并**:安装时 **disable+删除 watchdog**、删 PPP/ECM 规则。`Conflicts=go2-connectivity-watchdog.service` |
| **C · 旧 nmcli 恢复** | `go2_network_recover.sh` | `install_network_recover.sh` → `go2-network-recover.service`+timer(30s) | nmcli/ModemManager 路径(wired=eth0,4G=eth1) | 与 A/B 不是同一套栈;定时重置网卡/路由 |

- **A 与 B 显式互斥**:【生产 saas:install_go2_4g_manager.sh:197 `Conflicts=go2-connectivity-watchdog.service`】,且安装 B 时主动停/禁 A【生产 saas:install_go2_4g_manager.sh:82 `disable --now go2-connectivity-watchdog.service`】、删 A 的 unit 与 drop-in【生产 saas:install_go2_4g_manager.sh:103】、删累积的 PPP/ECM 竞争规则【生产 saas:install_go2_4g_manager.sh:108-112】、mode 打印 `ecm-auto`【生产 saas:install_go2_4g_manager.sh:259】。
- **A 的 ExecStart 是 Bash 脚本本身,不是 python "manager"**:【生产 saas:install_connectivity_watchdog.sh:64 `ExecStart=${WS}/scripts/go2_connectivity_watchdog.sh`】,并注入 `GO2_4G_MODE` 等一批 env【生产 saas:install_connectivity_watchdog.sh:44】。
- **B 的 ExecStart 是 python**:【生产 saas:install_go2_4g_manager.sh:256-258 `ExecStart=/usr/bin/python3 -u ${INSTALLED_MANAGER}`】;manager 主循环 `Manager(Config.from_env()).run(once=…)`【默认 code:go2_4g_manager.py:1590】。
- **B 是 ECM-only**:docstring 原话 `Single-owner A7600C ECM connection manager`【默认 code:go2_4g_manager.py:2】,自检打印 `ECM self-test`【默认 code:go2_4g_manager.py:1574】,全文件**无任何 ppp 代码**。
- 【DOG_UNKNOWN】**狗上到底启用 A / B / C 哪一套无法验证**——三者并存互斥、无任一文件狗上副本、manifest 只有巡检运行期字段。B(`install_go2_4g_manager.sh`)从"停删 watchdog + 删 PPP 规则 + ecm-auto"看**像是意图取代整套 ECM/PPP watchdog 的最新方案**,但**无法证实已部署**【推断-未验】。**若 B 为准,则下文 §三②PPP 段整体描述的是"非活路径"。**

> **历史根因(仍成立)**:ECM 更简单但在这台 Orin 上不稳定 → 引入 PPP 作为更稳替代(§三)→ 之后又出现 python "单一所有者" manager 想一统。这就是"乱"的来源:三套栈叠在一个仓库里。

## 三、ECM vs PPP:两种连接方式(**配置的是 §二栈 A 那个 Bash watchdog**)

模块是 **SIMCom A7600C**。栈 A(watchdog)的模式二选一,由下面两个 install 脚本各写一个 **drop-in 到 `go2-connectivity-watchdog.service`** 决定(两者互斥)。【生产 saas:install_a7600c_ecm_only.sh:5 `SERVICE_NAME=…go2-connectivity-watchdog.service`,:77 写 `${SERVICE_NAME}.d/override.conf`】【生产 saas:install_a7600c_ppp_only.sh:5 同,:136 写 `${SERVICE_NAME}.d/zz-go2-4g-ppp-primary.conf`】。

> 注:栈 B 的 python manager 也是 ECM,但它**不吃**这两个 drop-in,mode 自己硬编码 `ecm-auto`。下述 ECM 的 `.link`/DHCP/网关等**在 A、B 两栈里都存在**(各自实现),PPP 则**仅** A 栈有。

### ① ECM(USB 网卡,厂商默认) — `install_a7600c_ecm_only.sh`(栈 A 的 ECM drop-in)
- 模块以 **USB CDC-ECM** 呈现一个 USB 以太网设备,用 systemd `.link` 按 MAC `28:e3:57:c3:ed:9a` **重命名为 `go2_4g`**。【生产 saas:install_a7600c_ecm_only.sh:6-7,32-38】(栈 B 里同样有此 `.link`:【生产 saas:install_go2_4g_manager.sh:178-184】)。
- 走 **DHCP**【默认 code:go2_connectivity_watchdog.sh:910-914 dhclient/udhcpc】【默认 code:go2_4g_manager.py:1138-1160】,网关 `192.168.0.1`(代码默认==生产,无分歧)【生产 saas:install_a7600c_ecm_only.sh:69,84】【默认 code:go2_connectivity_watchdog.sh:31,446】【默认 code:go2_4g_manager.py:158】【生产 saas:install_go2_4g_manager.sh:217】。
- 厂商 **自动拨号**(`AT+DIALMODE=0`);NetworkManager 设为不管(`NM_UNMANAGED`),禁用 `ModemManager`。【默认 code:go2_4g_manager.py:1060-1064】【默认 code:go2_connectivity_watchdog.sh:820-824】【生产 saas:install_a7600c_ecm_only.sh:28,42-44(NM_UNMANAGED),:53(disable ModemManager)】。
- 启动环境:`GO2_4G_MODE=ecm`、USB reset **关**、USB 电源控制**开**、host reset **关**。【生产 saas:install_a7600c_ecm_only.sh:63,79 MODE=ecm;:64 `USB_RESET=0`;:65,81 `USB_POWER_CONTROL` 默认 1(开);:66,80 `HOST_RESET=0`】。
  > ⚠️ **死参数**:`GO2_4G_ENABLE_HOST_RESET=0` 被 install 脚本写进 service 并注入,但 **watchdog 与 manager 两个消费端都不读它**(grep 无命中),属**注了不生效的死 env**——别把它当成"host reset 关是靠它实现的"。【默认 code:go2_connectivity_watchdog.sh / go2_4g_manager.py 均无 `GO2_4G_ENABLE_HOST_RESET` 读取】。

### ② PPP(串口拨号) — `install_a7600c_ppp_only.sh`(栈 A 的 PPP drop-in)
- **显式拉黑 USB 网卡驱动**(`cdc_ether/rndis_host/cdc_subset/usbnet` blacklist + `install … /bin/false`)。【生产 saas:install_a7600c_ppp_only.sh:28-35】。
  > ⚠️ 脚本注释原话:*"On this Orin build that path has repeatedly destabilized tegra-xusb"*(`[USB Ethernet]` 为文档编辑性插入)—— **ECM/DHCP 会导致 A7600C 整个 USB 复位、搞崩 Orin 的 Tegra XHCI**。【默认 code:install_a7600c_ppp_only.sh:27(tegra-xusb 原文)】【生产 saas:install_a7600c_ppp_only.sh:138-140 drop-in 注释 `ECM/DHCP causes complete A7600C USB resets on this Orin`】。
- 改用 **PPP over 串口**:`/etc/ppp/peers/go2-a7600c` 拨号,PPP USB 接口 `05`,tty 候选 `ttyUSB2/3/1/0/4`。【生产 saas:install_a7600c_ppp_only.sh:112-113(peer),:149(iface 05)→ 消费 go2_connectivity_watchdog.sh:21,663-667;:152(tty 候选)→ 消费 :20,711】。
  > **默认≠生产(已核)**:tty 候选顺序在 watchdog **脚本默认**是另一套且**无 `ttyUSB4`**【默认 code:install_connectivity_watchdog.sh:23】,PPP drop-in 覆盖为 `ttyUSB2 3 1 0 4`【生产 saas:install_a7600c_ppp_only.sh:152】——**PPP 模式下以 drop-in 覆盖值生效**。
- 启动环境:`GO2_4G_MODE=ppp`、USB 总线 rebind **关**、USB 电源控制**关**,**只允许每 5 分钟一次设备级 reset**(且仅在 PPP 自己恢复不了时)。【生产 saas:install_a7600c_ppp_only.sh:124,141 MODE=ppp;:127,143 `BUS_REBIND=0`;:126,142 `POWER_CONTROL=0`;:125,144 `USB_RESET=1` + :128,145 `USB_RESET_MIN_INTERVAL=300`(=5min)→ 消费 go2_connectivity_watchdog.sh:40,485】;"仅自恢复不了时"见 `recover_once`:【默认 code:go2_connectivity_watchdog.sh:1009 `connect_ppp||usb_bus_rebind_once||usb_reset_once`】。
  > **默认≠生产(已核)**:ECM AT 配置开关 `GO2_4G_ECM_AT_CONFIG` 在 ECM drop-in 为 `1`【生产 saas:install_a7600c_ecm_only.sh:85】、PPP drop-in 为 `0`【生产 saas:install_a7600c_ppp_only.sh:146】——随模式而反。

> **互斥性(已核)**:ECM drop-in 删 `no-usbnet-bind` 规则【生产 saas:install_a7600c_ecm_only.sh:40】,PPP drop-in 反过来建它【生产 saas:install_a7600c_ppp_only.sh:89-92】;两者各装 `MODE=ecm/ppp` 的 drop-in,现场用哪套看部署。⚠️ 但这只是**栈 A 内部**的二选一;真正的第三条路径(栈 B 的 python ECM manager)原文完全没提。

## 四、栈 A 实现 · `go2_connectivity_watchdog.sh`:ECM/PPP/nmcli 保活与恢复

> 原文把下面这些 PPP/恢复行为算在了 python manager 头上,**实为 Bash watchdog 的职责**,已归位。

- **PPP 保活**:确保拨号进程在(`ppp_running` / `connect_ppp`)。【默认 code:go2_connectivity_watchdog.sh:449-451 `ppp_running`,:845-895 `connect_ppp`】。
- **ECM 保活**:`connect_ecm`(其 `ensure_ecm_at_config` 只做 `DIALMODE=0`+`MYCONFIG`,**无 `CGDCONT`**,与栈 B manager 不同,见 §五)。【默认 code:go2_connectivity_watchdog.sh:897 `connect_ecm`,:820-833 `ensure_ecm_at_config`】。
- **nmcli 兜底**:多级降级链 `connect_ecm||connect_ppp||connect_nmcli||usb_bus_rebind_once||usb_reset_once`。【默认 code:go2_connectivity_watchdog.sh:1023-1025】。
- **PPP 拨号失败后的模块恢复**:`ENABLE_PPP_MODEM_RECOVERY` 开关 + `recover_ppp_modem_state`。**在此脚本,不在 python manager**。【默认 code:go2_connectivity_watchdog.sh:27 `ENABLE_PPP_MODEM_RECOVERY=…`,:741-764 `recover_ppp_modem_state`】;PPP drop-in 把它设为 1【生产 saas:install_a7600c_ppp_only.sh:150-151】。

## 五、栈 B 实现 · `go2_4g_manager.py`:**ECM-only** 单一所有者(**1594 行**)

- **作为 `go2-4g-manager.service` 常驻运行**(`Manager(Config.from_env()).run(once=…)`),`--once` 单次、`--self-test` 自检。【生产 saas:install_go2_4g_manager.sh:192-204,256-258(service+python ExecStart)】【默认 code:go2_4g_manager.py:1580-1581,1588-1590】。**注意:不是 `go2-connectivity-watchdog.service`(那是栈 A)。**
- **AT 探测模块健康**:`AT+CPIN?`(SIM)、`AT+CSQ`(信号)、`AT+CEREG?`(注册)、`AT+DIALMODE?`、`AT$MYCONFIG?`(usbnet 配置)。【默认 code:go2_4g_manager.py:1027(五条 AT 完全一致)】。
- **保活**:ECM 模式确保厂商自动拨号 + DHCP 拿到 `go2_4g` 的 IP。(**PPP 保活不在这里**——python manager 全文无 ppp 代码;见 §四。)【默认 code:go2_4g_manager.py:1138-1160(DHCP),849 mode 硬编码 `ecm-auto`】。
- **确保厂商 Linux 模式**:`ensure_vendor_linux_mode` 若发现 `DIALMODE≠0` 会 `AT+CGDCONT=1,"IP",<APN>` + `AT+DIALMODE=0` 复原(APN 默认 `cmnet`)。**此含 `CGDCONT` 的确切序列仅在 python manager**(栈 A watchdog 的等价函数无 `CGDCONT`)。【默认 code:go2_4g_manager.py:1054,1060-1066;apn :157】。
- **谨慎恢复**(多重保护,函数 docstring 核实):
  - USB 重新枚举 / XHCI 复位 / 模块复位,各带**冷却**:`modem_reset_after=180s`、`modem_reset_cooldown=300s`、`rapid_usb_limit=3`(代码默认==生产,无分歧)。【默认 code:go2_4g_manager.py:170,171,172】【生产 saas:install_go2_4g_manager.sh:230,231,232】。
  - **巡检作业期间禁止破坏性恢复**:docstring `Return robot jobs during which an automatic reboot is unsafe` / `Block disruptive recovery during and immediately after robot jobs` —— 巡检中不重启/不拔 USB,避免打断任务。【默认 code:go2_4g_manager.py:467(active_workloads),488(recovery_workload_guard)】。
  - **不信 Jetson 的 RTC**(开机 NTP 前时钟可能错)做限速:改用"完成的 boot 次数"而非 wall-clock。【默认 code:go2_4g_manager.py:651-660(fatal_reboot_policy docstring)】。
- 状态机 `set_state("MODE_CONFIG", …)` 等。【默认 code:go2_4g_manager.py:1373】。

## 六、配套脚本
- `S/install_go2_4g_manager.sh`:**栈 B 的安装器(原文遗漏)**——建 `go2-4g-manager.service`、停删 watchdog、清 PPP/ECM 规则、`Conflicts` watchdog(见 §二)。【生产 saas:install_go2_4g_manager.sh:82,103,108-112,192-204,256-259】。
- `S/go2_a7600c_usb_monitor.sh`:监视 A7600C 的 USB 出现/消失(`snapshot_loop`+`kernel_watch_loop`,事件含 disconnect/reset/enumerat)。【默认 code:go2_a7600c_usb_monitor.sh:4,18-19,79-88】。
- `S/install_connectivity_watchdog.sh`:**栈 A 的安装器**——把 **Bash 脚本 `go2_connectivity_watchdog.sh` 本身**装成 `go2-connectivity-watchdog.service`,并注入 `GO2_4G_MODE` 等 env。⚠️ 原文"把 manager 装成服务"措辞误导:装的是 Bash 脚本、不是 python manager。【生产 saas:install_connectivity_watchdog.sh:44,64】。
- `S/go2_network_recover.sh` + `S/install_network_recover.sh`:**栈 C**,旧 nmcli/ModemManager 路径的网络恢复(重置网卡/路由,`ensure_eth0` 加 `192.168.123.18/192.168.1.5`、`ensure_4g_route`),装 `go2-network-recover.service`+timer(`OnUnitActiveSec=30s`)。**与栈 A/B 不是同一套栈**(此处 wired=eth0、4G=eth1)。【默认 code:go2_network_recover.sh:108-138,169-204】【默认 code:install_network_recover.sh:17-46】。
- `S/go2_wired_ssh_rescue.sh`:有线 SSH 救援(4G/WiFi 都挂时从有线进狗),按 MAC 找有线口加 `192.168.123.18/192.168.1.5/192.168.144.100`。【默认 code:go2_wired_ssh_rescue.sh:4-5,51-75】。

## 七、留待坐实
- 栈 A/B 里 USB 总线 rebind / XHCI dead 判定 / 电源控制的**具体 sysfs 操作序列**(按需再深读)。
- PPP 拨号失败后的模块恢复流程细节 `ENABLE_PPP_MODEM_RECOVERY` / `recover_ppp_modem_state`——**在 `go2_connectivity_watchdog.sh`(栈 A)**,原文误置于 python manager。【默认 code:go2_connectivity_watchdog.sh:27,741-764】。
- **【DOG_UNKNOWN】狗上真正启用哪套栈(A/B/C)**——需登狗 `systemctl is-active go2-4g-manager.service` vs `go2-connectivity-watchdog.service` vs `go2-network-recover.service` 才能定;当前无任一网络文件狗上副本。
- `NETWORK_4G_WIFI_ETHERNET_GUIDE.md` 与代码的逐条一致性核对(文档按惯例不可信;本文已按磁盘源码校正)。

## 狗上对照状态
> 本轮**无任一网络源文件有狗上副本**(狗上仅有 laserMapping/lddc/lds/waypoint_follower_go2_2 四份非网络副本)。下列全部**【无狗上对照】**,不得默认等同狗上:

| 文件 | 狗上状态 |
|---|---|
| `S/go2_4g_manager.py` | 【无狗上对照】狗上是否运行此文件未知 |
| `S/go2_connectivity_watchdog.sh` | 【无狗上对照】与 manager 互斥,跑哪套未知 |
| `S/install_go2_4g_manager.sh` | 【无狗上对照】原文漏提的关键文件 |
| `S/install_connectivity_watchdog.sh` | 【无狗上对照】 |
| `S/install_a7600c_ecm_only.sh` | 【无狗上对照】 |
| `S/install_a7600c_ppp_only.sh` | 【无狗上对照】 |
| `S/go2_a7600c_usb_monitor.sh` | 【无狗上对照】 |
| `S/go2_network_recover.sh` | 【无狗上对照】 |
| `S/install_network_recover.sh` | 【无狗上对照】 |
| `S/go2_wired_ssh_rescue.sh` | 【无狗上对照】 |
| `S/env_common.sh` | 【无狗上对照】但狗上 manifest `sdk_if=eth0` 与原文"eth1"矛盾 |
| `S/base_bringup.sh` | 【无狗上对照】Livox 192.168.1.161↔.5 的来源 |

## 核验台账
> claim → 证据(file:line)→ 判定。`S/` = `orin_go2_fastlio_ws/scripts/`。

| # | claim(原文说法) | 证据 | 判定 |
|---|---|---|---|
| 1 | manager 1800 行 | `go2_4g_manager.py:1594 raise SystemExit(main())`;`wc -l`=1594(姊妹文档 16 写 1595) | **CORRECTED → 1594** |
| 2 | manager 是 ECM/PPP 单一所有者 | `go2_4g_manager.py:2` docstring `…ECM connection manager`;:1574 `ECM self-test`;:849 硬编码 `ecm-auto`;全文无 ppp。PPP 在 `go2_connectivity_watchdog.sh:845 connect_ppp` | **CORRECTED → ECM-only** |
| 3 | 作为 `go2-connectivity-watchdog.service` 常驻 | 实由 `go2-4g-manager.service` 跑:install_go2_4g_manager.sh:256-258 python ExecStart,:197 `Conflicts=…watchdog`,:82 disable watchdog,:103 rm unit;watchdog 的 ExecStart 是 Bash:install_connectivity_watchdog.sh:64 | **CORRECTED → go2-4g-manager.service** |
| 4 | PPP 保活归 manager | manager 全文无 ppp;PPP 在 `go2_connectivity_watchdog.sh:449-451,845-895` | **CORRECTED → 栈 A watchdog** |
| 5 | install_* 把 manager 装成服务并注入 env | `install_connectivity_watchdog.sh:44,64` ExecStart 是 **Bash 脚本本身**,非 python | **CORRECTED** |
| 6 | `ENABLE_PPP_MODEM_RECOVERY` 在 manager | `go2_connectivity_watchdog.sh:27,741-764`;PPP env `install_a7600c_ppp_only.sh:150` | **CORRECTED → 栈 A watchdog** |
| 7 | 有线网卡 `eth1` | 狗上 `sdk_if=eth0`(manifest);`go2_network_recover.sh:4 ETH_IF=eth0`;eth1 仅 `env_common.sh:28` fallback、且 `go2_network_recover.sh:8` 里 eth1 是 4G 兜底名 | **CORRECTED → eth0** |
| 8 | 有线口接 Livox(192.168.1.5↔161) | `base_bringup.sh:33,49`;`go2_network_recover.sh:7`;Livox config host_ip | CONFIRMED |
| 9 | 有线口接 Go2 SDK / SSH 救援 | `go2_wired_ssh_rescue.sh:4-5,51-75`;`env_common.sh:29` | CONFIRMED |
| 10 | A7600C USB `1e0e:9011` | `install_a7600c_ppp_only.sh:41,63-64,90`;`go2_4g_manager.py:152-153`;`go2_connectivity_watchdog.sh:11-12` | CONFIRMED |
| 11 | 连 llyj SaaS 云端 | `llyj` 仅 docs;代码端点 `39.96.37.187:443`(`go2_4g_manager.py:159-160`;`go2_connectivity_watchdog.sh:9-10`) | UNVERIFIABLE(llyj 为文档命名) |
| 12 | ECM `.link` 按 MAC `28:e3:57:c3:ed:9a` 改名 `go2_4g` | `install_a7600c_ecm_only.sh:6-7,32-38`;`install_go2_4g_manager.sh:178-184` | CONFIRMED |
| 13 | ECM 走 DHCP、网关 `192.168.0.1` | DHCP `go2_connectivity_watchdog.sh:910-914` / `go2_4g_manager.py:1138-1160`;GW `install_a7600c_ecm_only.sh:69,84` 等(默认==生产) | CONFIRMED |
| 14 | ECM 自动拨号 `AT+DIALMODE=0`;NM_UNMANAGED;禁 ModemManager | `go2_4g_manager.py:1060-1064` / `go2_connectivity_watchdog.sh:820-824`;`install_a7600c_ecm_only.sh:28,42-44,53` | CONFIRMED |
| 15 | ECM 环境:MODE=ecm、USB reset 关、电源控制开、host reset 关 | `install_a7600c_ecm_only.sh:63,79 / :64 / :65,81 / :66,80` | CONFIRMED(但 `GO2_4G_ENABLE_HOST_RESET` 无消费者=死 env) |
| 16 | PPP 拉黑 `cdc_ether/rndis_host/cdc_subset/usbnet` | `install_a7600c_ppp_only.sh:28-35` | CONFIRMED |
| 17 | 注释 `…destabilized tegra-xusb` | `install_a7600c_ppp_only.sh:27`(`[USB Ethernet]` 为文档插入) | CONFIRMED |
| 18 | ECM/DHCP 致 A7600C USB 复位、崩 Tegra XHCI | `install_a7600c_ppp_only.sh:138-140` drop-in 注释 | CONFIRMED |
| 19 | PPP:peer `go2-a7600c`、iface `05`、tty `ttyUSB2/3/1/0/4` | peer `install_a7600c_ppp_only.sh:112-113`;iface05 :149→`go2_connectivity_watchdog.sh:21,663-667`;tty :152→:20,711 | CONFIRMED(tty 为 PPP drop-in 覆盖值;默认 `install_connectivity_watchdog.sh:23` 无 ttyUSB4) |
| 20 | PPP 环境:MODE=ppp、bus rebind 关、电源控制关、每 5min 一次设备 reset(仅自恢复不了时) | `install_a7600c_ppp_only.sh:124,141 / :127,143 / :126,142 / :125,144 + :128,145(300s)`→`go2_connectivity_watchdog.sh:40,485`;`recover_once:1009` | CONFIRMED |
| 21 | AT 探测 `CPIN?/CSQ/CEREG?/DIALMODE?/$MYCONFIG?` | `go2_4g_manager.py:1027` | CONFIRMED |
| 22 | `ensure_vendor_linux_mode`:`CGDCONT`+`DIALMODE=0` | `go2_4g_manager.py:1054,1060-1066`(apn `cmnet` :157);watchdog 等价函数无 CGDCONT | CONFIRMED(CGDCONT 仅 python manager) |
| 23 | 冷却 `modem_reset_after=180 / cooldown=300 / rapid_usb_limit` | `go2_4g_manager.py:170,171,172`;生产 `install_go2_4g_manager.sh:230-232`(无分歧) | CONFIRMED |
| 24 | 巡检期禁破坏性恢复(两句 docstring) | `go2_4g_manager.py:467,488` | CONFIRMED |
| 25 | 不信 Jetson RTC 做限速 | `go2_4g_manager.py:651-660` | CONFIRMED |
| 26 | 状态机 `MODE_CONFIG`;`--once/--self-test` | `go2_4g_manager.py:1373;1580-1581;1588-1590` | CONFIRMED |
| 27 | `go2_a7600c_usb_monitor.sh` 监视 USB 出现/消失 | `go2_a7600c_usb_monitor.sh:4,18-19,79-88` | CONFIRMED |
| 28 | `go2_network_recover.sh`+install 网络恢复(重置网卡/路由) | `go2_network_recover.sh:108-138,169-204`;`install_network_recover.sh:17-46`(timer 30s) | CONFIRMED(旧 nmcli 路径,栈 C) |
| 29 | `go2_wired_ssh_rescue.sh` 有线 SSH 救援 | `go2_wired_ssh_rescue.sh:4-5,51-75` | CONFIRMED |
| 30 | 4G 两种连接方式互斥 ECM/PPP | `install_a7600c_ecm_only.sh:40` vs `install_a7600c_ppp_only.sh:89-92` | CONFIRMED(但不完整:栈 B `install_go2_4g_manager.sh` 是第三条路径,原文未提) |
| 31 | 狗上实际跑哪套 4G 管理 | 三安装器/服务并存互斥,无狗上网络副本,manifest 无 4G 字段;B 像最新合并(:82,103,108-112,259)但无法证实部署 | **DOG_UNKNOWN** |

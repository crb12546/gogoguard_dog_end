# 11 · 开机自启动 & 执行链(狗一通电到底怎么跑起来)

> 原则同 00。核心文件(**已逐行读**,路径均在 `orin_go2_fastlio_ws/` 下):
> `orin_go2_fastlio_ws/scripts/install_autostart.sh`、`install_saas_autostart.sh`、
> `install_connectivity_watchdog.sh`、`install_network_recover.sh`、`install_go2_4g_manager.sh`、
> `go2_connectivity_watchdog.sh`、`base_bringup.sh`、`go2_saas_agent.py`、
> `deploy/systemd_user/go2-fastlio-base.service`。
>
> **源标签约定**:【默认 code file:line】=仓库脚本/默认代码路径;【生产 saas dog:证据】=狗上 manifest/快照实测;
> 【推断-未验】=无在盘证据、仅推断;【无狗上对照】=仓库有、狗上无副本可 sha 对比(本轮绝大多数文件如此)。

---

## 核验状态(本轮做了什么、什么还没法验)

- **已对磁盘源码逐条核**:systemd 单元表(一)、生命周期骨架(二)、"哪个文件在执行"(三)全部落到 install 脚本与 `go2_saas_agent.py` 的真实行号;三处原文错误已修:
  1. **watchdog ↔ go2_4g_manager.py 混淆**(原文表一/二/三共 3 处)——已改正。
  2. **巡检链顺序错**(遥测错置 + 漏 `performance_monitor`)——已改正。
  3. **follower 默认≠狗上生产**(仓库默认跑 `_trace.py` 包装器,狗上实跑 330 行裸版)——已补生产值。
- **仍无法验证(证据已从盘上删除,partial-clone 下 blob 不可取回、网络受限)**:
  - section 四头号结论"狗上分 `livox.service` + `fast_lio.service`"所依赖的 `previous_boot/livox.service.log`、`fast_lio.service.log`、`current_boot/*.service.log` **在工作树已不存在**(`find analysis -name '*.service.log'` 为空;`analysis/xunjian_20260725_shutdown_capture/` 下只剩 `previous_boot/`、`runs/`,连 `current_boot/` 目录都没有)。git 历史 `b5b87fc` 的树引用这些 log 的 blob,但 `git cat-file -e` 返回 1(partial clone `.git/objects/pack/*.promisor`,blob 本地缺失)。→ **该结论无任何在盘证据可核,本轮降级为"待取证",不作为已坐实。**
- **狗上无副本可 sha 对比的文件**:本篇所有 install_*.sh / go2_saas_agent.py / base_bringup.sh / go2_connectivity_watchdog.sh 等,狗上 `remote_source/` 只落了 4 份副本,其余均【无狗上对照】——不许默认等同狗上。
- **一处结构性存疑(见 section 五-补)**:狗 manifest 的键集与本仓库 `go2_saas_agent.py` 产出的 manifest **对不上** → 狗上真实巡检启动器很可能**不是**本仓库这份 `go2_saas_agent.py`。文档自身的"仓库≠狗上"主题,其实也适用于整条 saas 启动链。

---

## 一、开机自启动的 systemd 单元(全部来自 install 脚本的内联定义)

### root 级(`/etc/systemd/system/`)

| 单元 | 类型 | ExecStart | 依赖/时机 | 源标签 |
|---|---|---|---|---|
| `go2-lidar-network.service` | oneshot, RemainAfterExit | 内联:按 MAC `4c:bb:47:ab:e4:c2` 找网卡 → `ip addr add 192.168.1.5/24` | `Before=network-online.target` | 【默认 code install_autostart.sh:6,10,14-17】✅CONFIRMED |
| `go2-saas-command.service` | simple, Restart=always | `python3 -u go2_saas_agent.py command-loop --interval 5 --run-file … --execute-safe` | After `network-online.target time-sync.target`;ExecStartPre `wait_valid_time.sh 0` 等有效墙钟 | 【默认 code install_saas_autostart.sh:48,55,70,44-45,53】✅CONFIRMED |
| `go2-saas-video.service` | simple, Restart=always | `python3 -u go2_saas_agent.py video-loop --seconds 20 --upload` | 同上 | 【默认 code install_saas_autostart.sh:72,8】✅CONFIRMED |
| `go2-saas-outbox.service` | simple, Restart=always | `python3 -u go2_saas_agent.py outbox-loop --interval 10 --max-jobs 2` | 同上 | 【默认 code install_saas_autostart.sh:74】✅CONFIRMED |
| `go2-connectivity-watchdog.service` | simple, Restart=always | `go2_connectivity_watchdog.sh`(**自成一体的 4G 看门狗**,注入全部 `GO2_4G_*` env 供其自己消费;**不调用 `go2_4g_manager.py`**) | After network | 【默认 code install_connectivity_watchdog.sh:64】⚠️CORRECTED(见下) |
| `go2-network-recover.service` + `.timer` | oneshot + timer | `go2_network_recover.sh`(`OnBootSec=20s`,`OnUnitActiveSec=30s`) | After NetworkManager/ModemManager | 【默认 code install_network_recover.sh:24,26,34-35,21】✅CONFIRMED |

**⚠️ 参数生效说明(默认可被 env 覆盖 / 硬编码 区分):**
- `--interval 5`(command)= 默认 `GO2_SAAS_COMMAND_INTERVAL:-5`【install_saas_autostart.sh:7】,env 可覆盖;脚本未额外覆盖 → 落地即 5。
- `--execute-safe`(command)= 默认 `GO2_SAAS_EXECUTE_SAFE:-1`【:6,20-24】,env 可覆盖;未覆盖 → 落地带 `--execute-safe`。
- `--seconds 20`(video)= 默认 `GO2_SAAS_VIDEO_SECONDS:-20`【:8】,env 可覆盖。
- `--interval 10 --max-jobs 2`(outbox)= **脚本内硬编码字面量,非 env 变量**【:74】,云端下发也覆盖不了这两个数。

**⚠️ CORRECTED —— `go2-connectivity-watchdog` 与 `go2_4g_manager.py` 是两套互斥栈,不是同一条链:**
- `go2_connectivity_watchdog.sh` **自己**做 PPP / ECM / nmcli 恢复:`main()→recover_once`,全脚本 grep 无 `go2_4g_manager` 字样【默认 code go2_connectivity_watchdog.sh:1040-1053;grep -n go2_4g_manager 返回 1(无匹配)】。它 ExecStart 于 watchdog.service【install_connectivity_watchdog.sh:64】。
- 注入的 `GO2_4G_*` env **确被 watchdog 本身消费**(`MODE=${GO2_4G_MODE:-auto}`:13、`ECM_IF=${GO2_4G_ECM_IF:-go2_4g}`:30、`ECM_MAC=${GO2_4G_ECM_MAC:-…}`:32)——消费者是 watchdog,不是 `go2_4g_manager.py`。
- `go2_4g_manager.py` 属于**另一套独立且互斥**的 `go2-4g-manager.service`(由 `install_go2_4g_manager.sh` 装配):`Conflicts=go2-connectivity-watchdog.service`【:197】、`ExecStart=/usr/bin/python3 -u …/go2_4g_manager.py`【:204】;而且它的安装器会 **disable + rm 掉 watchdog**(`systemctl disable --now go2-connectivity-watchdog.service`:82、`rm -f …/go2-connectivity-watchdog.service`:103)。
- → **二者不会同时存在**。狗上到底跑 watchdog 还是 go2-4g-manager.service,**仓库判不出**【推断-未验;两文件狗上均【无狗上对照】】。

### 用户级(`~/.config/systemd/user/`,需 `loginctl enable-linger unitree` 免登录启动)

| 单元 | ExecStart / ExecStop | 干什么 | 源标签 |
|---|---|---|---|
| `go2-fastlio-base.service` | ExecStart=**`base_bringup.sh`**,ExecStop=`base_stop.sh`,Restart=on-failure | 起 Livox 驱动 + FAST-LIO(基础层,见 01) | 【默认 code deploy/systemd_user/go2-fastlio-base.service:7-9;install_autostart.sh:29,38-40,64-65】✅CONFIRMED |

`base_bringup.sh` 出 `/Odometry`、`/cloud_registered_body`:【默认 code base_bringup.sh:177(livox launch),220(fastlio mapping.launch.py),226-227(wait_topic /Odometry、/cloud_registered_body)】✅CONFIRMED。

## 二、"狗一通电"的完整生命周期

```
上电
 ├─(root)go2-lidar-network      → 配雷达网卡 192.168.1.5/24         【默认 install_autostart.sh:16-17】
 ├─(root)go2-connectivity-watchdog → 4G 保活(go2_connectivity_watchdog.sh 自成一体) 【默认 install_connectivity_watchdog.sh:64】
 │        ⚠️ 或改由互斥的 go2-4g-manager.service(go2_4g_manager.py)接管,二者只跑其一 —— 狗上跑哪套【推断-未验】
 ├─(root)go2-network-recover.timer → 周期网络自愈                   【默认 install_network_recover.sh:34-35】
 ├─(user)go2-fastlio-base       → base_bringup.sh → Livox + FAST-LIO(出 /Odometry, /cloud_registered_body) 【默认 base_bringup.sh:177,220,226-227】
 └─(root)go2-saas-command/video/outbox → go2_saas_agent.py 三循环(等有效墙钟后) 【默认 install_saas_autostart.sh:53,70-74】
        │
        │  待命(不自动巡检)
        ▼
   command-loop 每 5s POST /robot/heartbeat 取命令  【默认 go2_saas_agent.py:2853,2856,3023,3083】
        │  收到 start_patrol → run_start_patrol → start_patrol_command  【默认 :51,2565-2574,1475】
        ▼
   start_patrol_command 现场组装巨型 bash(commands[]),分级拉起 —— 实际顺序(⚠️已订正):
   ensure_base → 遥测(experiment_telemetry) → performance_monitor → sdk2_receiver → cmd_vel_sender
     → safe → (重定位 route_prepare + check_route_start_alignment) → rosbag
     → video 门开 → session_guard → 稳定门(--patrol-start-gate) → follower
        │        【默认 go2_saas_agent.py commands[]:2203 / 2225 / 2259 / 2292 / 2314 / 2319 / 2330 / 2354 / 2387 / 2403 / 2434 / 2439】
        │
        │  ⚠️ follower 默认=waypoint_follower_go2_2_trace.py(仓库);狗上生产实跑=waypoint_follower_go2_2(见下)
        ▼
   巡检运行(doc 02/03/04 的闭环) ── 收到 stop_patrol → stop_patrol_command 收尾取证
```

**⚠️ CORRECTED —— commands[] 真实启动顺序:** 原文写成 `…→ sdk2_receiver → cmd_vel_sender → safe → 遥测 → …`,把遥测错置到 safe 之后,且漏了 `performance_monitor`。实际**遥测(experiment_telemetry)与 performance_monitor 都在 sdk2_receiver 之前**就已拉起:

| 序 | 命令 | 行号(【默认 code go2_saas_agent.py】) |
|---|---|---|
| 1 | ensure_base(`ensure_base_ready.sh`) | 2203 |
| 2 | **遥测** `experiment_telemetry`(`go2_experiment_telemetry.py`) | 2225 |
| 3 | **`performance_monitor`**(`patrol_performance_monitor.py`,原文漏) | 2259 |
| 4 | sdk2_receiver(`go2_sdk2_udp_receiver … 5005`) | 2292 |
| 5 | cmd_vel_sender(`cmd_vel_udp_sender`) | 2314 |
| 6 | safe(`unitree_safe_cmd_node`) | 2319 |
| 7 | 重定位(`route_prepare` + `check_route_start_alignment`) | 2328-2333 |
| 8 | rosbag(`ros2 bag record`) | 2354 |
| 9 | video 门开 | 2385-2387 |
| 10 | session_guard(`localization_session_guard.py`) | 2400-2403 |
| 11 | 稳定门(`--patrol-start-gate`) | 2433-2434 |
| 12 | follower | 2439(等 `FOLLOWER_EXACT_TRACE_READY`:2442) |

> 注:video门开 / session_guard / 稳定门 / follower 的相对次序,以及"重定位在 safe 之后、rosbag 之前"均**原文正确**;唯"遥测位置 + 漏 performance_monitor"两点订正。

## 三、回答"狗启动了是哪个文件在执行"

- **待命阶段**:
  - `base_bringup.sh`(基础层)【默认 base_bringup.sh:177,220】
  - `go2_saas_agent.py`(command / video / outbox 三个进程)【默认 install_saas_autostart.sh:70-74】
  - **4G 保活**:`go2_connectivity_watchdog.sh`(自成一体)**或**互斥的 `go2_4g_manager.py`(go2-4g-manager.service),**只跑其一**【默认 install_connectivity_watchdog.sh:64 / install_go2_4g_manager.sh:197,204;狗上跑哪套=推断-未验】。
    ⚠️ 原文此处写 `go2_4g_manager.py` 是把两套互斥栈当成了一条链——见 section 一 CORRECTED。
- **巡检阶段**:上面的 command-loop 再拉起:
  - **follower** —— ⚠️ **默认≠生产**:
    - 【默认 code】= `orin_go2_fastlio_ws/scripts/waypoint_follower_go2_2_trace.py`(427 行,sha `86745b40`,class `TracedWaypointFollower`);仓库 `go2_saas_agent.py` 装配的启动串就是它(`follower_trace_script`:1743、`follower_cmd`:2061-2079、manifest 写 `controller_executable=waypoint_follower_go2_2_trace.py`:1830、等 `FOLLOWER_EXACT_TRACE_READY`:2442)。
    - 【生产 dog】= `waypoint_follower_go2_2`(**330 行**,sha `d205a596` = `remote_source` 副本版);狗 20260725-06/07 manifest 实跑此版:`controller_executable=waypoint_follower_go2_2`(无 `_trace`)、`controller_reference_sha256=d205a596…`、`controller=go2_2_enhanced_nearest_lookahead`【生产 dog runs/xunjian-20260725-06/manifest.txt:14-16;07 同】。
    - → **狗上没跑 trace 包装器**。(注:仓库 `src/…/waypoint_follower_go2_2.py` 是 1043 行 / sha `009cb25b` 的另一版,与狗上 330 行版也不同 → repo≠dog,sha 验。)
  - `unitree_safe_cmd_node`【默认 go2_saas_agent.py:2051】✅
  - `cmd_vel_udp_sender`【默认 :2045】✅
  - `go2_sdk2_udp_receiver`(端口 5005)【默认 :2022-2024】✅
  - (+ rosbag / telemetry / performance_monitor / session_guard 旁路)
- **触发者**:云端下发 `start_patrol`,不是开机自动【默认 go2_saas_agent.py:51,2565-2574】✅。

## 四、⚠️ "仓库 ≠ 狗上"

### 4.1 仓库侧(✅已坐实):install_autostart.sh 建的是**单个**服务
- **仓库** `install_autostart.sh` 建的是**单个** `go2-fastlio-base.service`,内部 `base_bringup.sh` 同时把 livox + fastlio 拉为**子进程**:【默认 code install_autostart.sh:31-45(单个单元内联);base_bringup.sh:177-178(livox 后台 &),220-221(fastlio 后台 &),238(wait LIVOX_PID FASTLIO_PID)】✅CONFIRMED。

### 4.2 狗上侧(❓待取证,**本轮无法坐实**):"狗上分 livox.service + fast_lio.service"
- 原文结论:狗上真实开机日志是**分开的两个 systemd 服务** `livox.service` + `fast_lio.service`(据 `previous_boot/livox.service.log`、`fast_lio.service.log`)。
- **⚠️ UNVERIFIABLE**:该结论所依赖的 `*.service.log` 证据**已从工作树删除**——`find analysis -name '*.service.log'` 为空;`analysis/xunjian_20260725_shutdown_capture/` 下只有 `previous_boot/`、`runs/`,无 `current_boot/`。git `b5b87fc` 树虽引用这些 log 的 blob,但 partial-clone 下 blob 本地缺失(`git cat-file -e` rc=1、`.git/objects/pack/*.promisor`),网络受限取不回。两份在盘 `manifest.txt` 也未涉及 systemd 单元拆分。
- → **无任何在盘/可取证据支撑此结论,本轮不作为"已坐实"。** 结构上"狗上自启动≠仓库脚本"很可能成立(这是本篇 4.3、及全项目反复出现的主题),但"到底拆成哪两个 service、ExecStart 是什么"**必须重新取证**(重新拉狗上 `/etc/systemd/system/` 真实单元或 boot 日志)后才能写死。

### 4.3 ⚠️ 补:saas 启动链本身也可能"仓库 ≠ 狗上"(原文未指出)
本篇 section 二/三 默认把 `go2_saas_agent.py` 当作**狗上**的巡检启动器,但**狗 manifest 的键集与本仓库 `go2_saas_agent.py` 产出的 manifest 对不上**:

| 键 | 【生产 dog】manifest(runs/…-06/07) | 【默认 code】go2_saas_agent.py 产出 |
|---|---|---|
| `controller_executable` | `waypoint_follower_go2_2`(无 `_trace`) | `waypoint_follower_go2_2_trace.py`(:1830) |
| `controller` | `go2_2_enhanced_nearest_lookahead` | `deployed_go2_2_nearest_lookahead_unchanged`(:1829) |
| `controller_reference_sha256` | `d205a596…`(存在) | 该键**不产出**(产出的是 `controller_source_sha256` / `controller_trace_wrapper_sha256`:1831-1832) |
| `trace_wrapper` / `route_sha256` / `fast_lio_freshness_gate` | **均无**(dog manifest grep -c = 0) | **均有**(:1832-1833 / :1818 / :1849) |

→ **狗上 runs 06/07 并非跑本仓库这份 `go2_saas_agent.py`**【生产 dog runs/xunjian-20260725-06/manifest.txt vs 默认 code go2_saas_agent.py:1818,1829-1833,1849;dog 侧 go2_saas_agent.py【无狗上对照】(remote_source 仅 4 份,无此文件副本)】。→ 本篇二/三的启动链描述**在语义/架构层可作理解地图**,但**不能默认逐字节等同狗上实跑**;真实巡检启动器很可能是狗上另一份(改过的)agent。

## 五、留待坐实(下一步逐行读 / 重新取证)

- **[头号取证]** 狗上 `/etc/systemd/system/` 真实单元:验证 section 4.2 的 `livox.service` / `fast_lio.service` 拆分是否真存在、ExecStart 为何(原证据 `*.service.log` 已删,需重新从狗上拉)。
- **[头号取证]** 狗上真实巡检启动器(section 4.3):狗上那份产出 06/07 manifest 的 agent 到底是哪份、与仓库 `go2_saas_agent.py` diff 在哪。
- 狗上启用的是 `go2-connectivity-watchdog.service` 还是互斥的 `go2-4g-manager.service`(section 一 CORRECTED,仓库判不出)。
- 巡检生命周期旁路逐行:`ensure_base_ready.sh` / `base_stop.sh` / `wait_valid_time.sh` / `localization_session_guard.py` / `patrol_performance_monitor.py` / `go2_experiment_telemetry.py` / `go2_base_health_watchdog.py` / `check_route_start_alignment.py`。

---

## 核验台账(claim → 证据 file:line → 判定)

> 判定图例:✅CONFIRMED=与在盘源码一致;⚠️CORRECTED=原文错、已用更正值改写;🔀DEFAULT_VS_PROD=默认与狗上生产不同、两值并列;❓UNVERIFIABLE=证据缺失、不可核。
> 狗上状态:repo==dog(sha验)/ repo≠dog(sha验)/【无狗上对照】(仓库有、狗上无副本)。

| # | claim(原文断言) | 证据 file:line | 判定 | 狗上状态 |
|---|---|---|---|---|
| 1 | watchdog → `go2_4g_manager.py`(表一/二/三,3 处) | go2_connectivity_watchdog.sh:1040-1053(main→recover_once,全脚本无 go2_4g_manager);:13/:30/:32(自己消费 GO2_4G_* env);install_connectivity_watchdog.sh:64(ExecStart=…watchdog.sh);install_go2_4g_manager.sh:197(Conflicts=)、204(ExecStart go2_4g_manager.py)、82/103(disable+rm watchdog) | ⚠️CORRECTED:watchdog 自成一体、不调 go2_4g_manager.py;后者属互斥的 go2-4g-manager.service | 两文件均【无狗上对照】;狗上跑哪套=推断-未验 |
| 2 | 狗上分 `livox.service` + `fast_lio.service`(section 四头号) | find analysis 无任何 *.service.log、无 current_boot(仅 previous_boot/runs);git b5b87fc 引 blob 但 cat-file -e rc=1(partial-clone promisor);manifest.txt 未涉单元拆分 | ❓UNVERIFIABLE:证据已删+blob 不可取,降级为待取证 | 【无狗上对照】(证据 log 已从工作树删除、不可读) |
| 3 | 巡检 follower = `waypoint_follower_go2_2_trace.py`(二/三) | 默认:go2_saas_agent.py:1743,2061-2079,1830,2442(427 行 sha 86745b40);生产:runs/…-06/manifest.txt:14-16(waypoint_follower_go2_2、d205a596、go2_2_enhanced_nearest_lookahead) | 🔀DEFAULT_VS_PROD:默认跑 _trace 包装器,狗上实跑 330 行裸版 | repo≠dog(sha 验:trace 427/86745b40 vs dog 330/d205a596) |
| 4 | start_patrol commands[] 顺序:…sdk2_receiver→cmd_vel→safe→遥测→… | go2_saas_agent.py commands[]:2203/2225/2259/2292/2314/2319/2328-2333/2354/2387/2403/2434/2439 | ⚠️CORRECTED:遥测+performance_monitor 在 sdk2_receiver **之前**;原文错置遥测、漏 performance_monitor | 【无狗上对照】(且 4.3:狗上 agent 疑非本文件) |
| 5 | go2-lidar-network.service(MAC→ip add,Before=network-online) | install_autostart.sh:6,10,14-17 | ✅CONFIRMED | 【无狗上对照】 |
| 6 | go2-saas-command.service(command-loop --interval 5 --execute-safe) | install_saas_autostart.sh:48,55,70,44-45,53;5=默认:7、--execute-safe=默认:6 | ✅CONFIRMED(两参数为 env 可覆盖默认,未覆盖→落地生效) | 【无狗上对照】 |
| 7 | go2-saas-video.service(video-loop --seconds 20 --upload) | install_saas_autostart.sh:72;20=默认 VIDEO_SECONDS:8 | ✅CONFIRMED(env 可覆盖默认) | 【无狗上对照】 |
| 8 | go2-saas-outbox.service(outbox-loop --interval 10 --max-jobs 2) | install_saas_autostart.sh:74 | ✅CONFIRMED(10/2 为**硬编码字面量**,非 env,云端覆盖不了) | 【无狗上对照】 |
| 9 | go2-network-recover.service + .timer(20s/30s,After NM/MM) | install_network_recover.sh:24,26,34-35,21 | ✅CONFIRMED | 【无狗上对照】 |
| 10 | go2-fastlio-base.service(user;bringup/stop;linger) | deploy/systemd_user/go2-fastlio-base.service:7-9;install_autostart.sh:29,38-40,64-65 | ✅CONFIRMED | 【无狗上对照】 |
| 11 | base_bringup.sh → Livox+FAST-LIO(/Odometry,/cloud_registered_body) | base_bringup.sh:177,220,226-227 | ✅CONFIRMED | 【无狗上对照】 |
| 12 | 仓库 install_autostart 建**单个** base 服务(内部子进程拉 livox+fastlio) | install_autostart.sh:31-45;base_bringup.sh:177-178,220-221,238 | ✅CONFIRMED(仓库侧;与 4.2 狗上侧存疑并存) | 【无狗上对照】 |
| 13 | command-loop 每 5s POST /robot/heartbeat 取命令 | go2_saas_agent.py:2853,2856,2861-2879,3023,3083 | ✅CONFIRMED(5s 为默认,可覆盖) | 【无狗上对照】 |
| 14 | 收到 start_patrol → start_patrol_command | go2_saas_agent.py:51,2565-2574,1475 | ✅CONFIRMED | 【无狗上对照】 |
| 15 | 巡检进程 unitree_safe_cmd_node + cmd_vel_udp_sender + go2_sdk2_udp_receiver | go2_saas_agent.py:2051,2045,2022-2024(:5005) | ✅CONFIRMED(进程名对仓库正确;follower 见 #3) | 【无狗上对照】 |
| 16 | (补)狗 manifest 键集 ≠ 仓库 go2_saas_agent.py 产出 → 狗上启动器疑非本文件 | dog runs/…-06/manifest.txt(无 trace_wrapper/route_sha256/fast_lio_freshness_gate,grep -c=0)vs go2_saas_agent.py:1818,1829-1833,1849 | ⚠️新增结论(原文未指出) | repo≠dog(键集差异);dog 侧 agent【无狗上对照】 |

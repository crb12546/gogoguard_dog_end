# 11 · 开机自启动 & 执行链(狗一通电到底怎么跑起来)

> 原则同 00。核心文件(**已逐行读**):`scripts/install_autostart.sh`、`scripts/install_saas_autostart.sh`、
> `scripts/install_connectivity_watchdog.sh`、`scripts/install_network_recover.sh`、`deploy/systemd_user/go2-fastlio-base.service`。
> 证据:`analysis/xunjian_20260725_shutdown_capture/{current_boot,previous_boot}/*.service.log`(狗真实开机日志)。

## 一、开机自启动的 systemd 单元(全部来自 install 脚本的内联定义)

### root 级(`/etc/systemd/system/`)
| 单元 | 类型 | ExecStart | 依赖/时机 |
|---|---|---|---|
| `go2-lidar-network.service` | oneshot, RemainAfterExit | 内联:按 MAC `4c:bb:47:ab:e4:c2` 找网卡 → `ip addr add 192.168.1.5/24` | `Before=network-online.target` |
| `go2-saas-command.service` | simple, Restart=always | `python3 -u go2_saas_agent.py command-loop --interval 5 --run-file … --execute-safe` | After `network-online.target time-sync.target`;ExecStartPre 等有效墙钟 |
| `go2-saas-video.service` | simple, Restart=always | `python3 -u go2_saas_agent.py video-loop --seconds 20 --upload` | 同上 |
| `go2-saas-outbox.service` | simple, Restart=always | `python3 -u go2_saas_agent.py outbox-loop --interval 10 --max-jobs 2` | 同上 |
| `go2-connectivity-watchdog.service` | simple, Restart=always | `go2_connectivity_watchdog.sh`(→ `go2_4g_manager.py`,注入全部 4G env) | After network |
| `go2-network-recover.service` + `.timer` | oneshot + timer | `go2_network_recover.sh`(`OnBootSec=20s`,`OnUnitActiveSec=30s`) | After NetworkManager/ModemManager |

### 用户级(`~/.config/systemd/user/`,需 `loginctl enable-linger unitree` 免登录启动)
| 单元 | ExecStart / ExecStop | 干什么 |
|---|---|---|
| `go2-fastlio-base.service` | ExecStart=**`base_bringup.sh`**,ExecStop=`base_stop.sh`,Restart=on-failure | 起 Livox 驱动 + FAST-LIO(基础层,见 01) |

## 二、"狗一通电"的完整生命周期
```
上电
 ├─(root)go2-lidar-network      → 配雷达网卡 192.168.1.5/24
 ├─(root)go2-connectivity-watchdog → 4G 保活(go2_4g_manager.py)
 ├─(root)go2-network-recover.timer → 周期网络自愈
 ├─(user)go2-fastlio-base       → base_bringup.sh → Livox + FAST-LIO(出 /Odometry, /cloud_registered_body)
 └─(root)go2-saas-command/video/outbox → go2_saas_agent.py 三循环(等有效墙钟后)
        │
        │  待命(不自动巡检)
        ▼
   command-loop 每 5s POST /robot/heartbeat 取命令
        │  收到 start_patrol
        ▼
   start_patrol_command 现场组装巨型 bash,分级拉起:
   ensure_base → sdk2_receiver → cmd_vel_sender → safe → 遥测 → (重定位) → rosbag
     → video 门开 → session_guard → 稳定门 → follower(waypoint_follower_go2_2_trace.py)
        │
        ▼
   巡检运行(doc 02/03/04 的闭环) ── 收到 stop_patrol → stop_patrol_command 收尾取证
```

## 三、回答"狗启动了是哪个文件在执行"
- **待命阶段**:`base_bringup.sh`(基础层)+ `go2_saas_agent.py`(command/video/outbox 三个进程)+ `go2_4g_manager.py`。
- **巡检阶段**:上面的 command-loop 再拉起 `waypoint_follower_go2_2_trace.py` + `unitree_safe_cmd_node` + `cmd_vel_udp_sender` + `go2_sdk2_udp_receiver`(+ rosbag/telemetry/session_guard 旁路)。
- **触发者**:云端下发 `start_patrol`,不是开机自动。

## 四、⚠️ 又一处"仓库 ≠ 狗上"(重要)
- **仓库** `install_autostart.sh` 建的是**单个** `go2-fastlio-base.service`(内部 `base_bringup.sh` 同时拉 livox+fastlio 为子进程)。
- **狗上真实开机日志**却是**分开的两个 systemd 服务**:`livox.service` + `fast_lio.service`(见 `previous_boot/livox.service.log`、`fast_lio.service.log`)。
- → 狗上的实际自启动结构与仓库脚本不一致。**要复现狗的开机行为,不能只看 install_autostart.sh。** 需进一步比对狗上 `/etc/systemd/system/` 真实单元(可从 `current_boot` 日志与 `analysis` 快照反推)。

## 五、留待坐实(下一步逐行读)
- 狗上 `livox.service` / `fast_lio.service` 的真实 ExecStart(反推自 boot 日志)。
- `go2_connectivity_watchdog.sh`(4G 看门狗外壳,还没逐行读)。
- 巡检生命周期旁路:`ensure_base_ready.sh` / `base_stop.sh` / `wait_valid_time.sh` / `localization_session_guard.py` / `go2_base_health_watchdog.py` / `check_route_start_alignment.py`。

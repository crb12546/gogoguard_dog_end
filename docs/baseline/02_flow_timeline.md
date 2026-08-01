# 02 · 端到端时序

> 从通电到狗迈出第一步,再到停止收尾。每一步给**行号 · 动作 · 失败退出码**。
> 源:`go2_saas_agent.py` 的 `start_patrol_command`(`:1678` 起)中的 `commands` 列表(`:2590–2924`)。

---

## 阶段 0 · 开机(systemd,4 个单元)

位置:`system_config/etc/systemd/system/`(**不在 mirror 下**)

| 单元 | 启动什么 | 策略 |
|---|---|---|
| `go2-saas-command.service` | `go2_saas_agent.py command-loop --interval 5 --execute-safe` | `Restart=always` |
| `go2-saas-outbox.service` | `go2_saas_agent.py outbox-loop --interval 10 --max-jobs 2` | `Restart=always` |
| `go2-saas-video.service` | `go2_saas_agent.py video-loop --seconds 20 --upload` | `Restart=always` |
| `go2-wired-ssh-rescue.service` | `scripts/go2_wired_ssh_rescue.sh` | `Restart=always` |

`go2_test.service` 存在但**未 enable**(宇树自带 `/unitree/module/test/test.py`,非本项目)。

### ★ 开机第一环:`ExecStartPre` 等时间有效

```
go2-saas-command.service:14   ExecStartPre=/bin/bash -lc '.../scripts/wait_valid_time.sh 0'
go2-saas-command.service:15   ExecStart=...
```
**saas 启动之前必须先等系统时间有效**。狗断电后 RTC 可能不准,若在 NTP 同步前启动,
所有日志与证据的时间戳都会错乱(saas 常量 `MIN_VALID_EPOCH = 1704067200` 即 2024-01-01,
`MIN_VALID_YEAR = 2024`)。`wait_valid_time.sh`(26 行)是这道闸。

**基础层不由 systemd 拉起**,由 saas 的 base bringup 启动:
```
livox_ros_driver2_node      (巡检时 pid 5275)
fastlio_mapping             (巡检时 pid 5628)
go2_base_health_watchdog.py (saas :2783 setsid nohup)
```

**此时狗知道自己在哪,但绝不会动** —— `.motion_enabled` 不存在。

---

## 阶段 1 · 收到命令(每 5 秒轮询一次)

```
command-loop → 云端 /devices/plan → 解析 action
   ├─ start_patrol / patrol_start / follow_route / start_route / auto_patrol / auto_inspection
   ├─ stop_patrol  / patrol_stop  / stop_route
   ├─ 安全命令: ping / noop / status / start_base / stop_base / camera_start_loop / camera_stop_loop
   └─ move/walk/go/goto/navigate  →  **直接拒绝**(:3175-3176 "goto/free navigation is not implemented in v1.5")
```

⚠️ `handle_commands:3266` 只在累计 `rc == 0` 时才把 cmd_id 记入 `seen`
→ 一条命令失败会影响后续 id 的记录行为。

---

## 阶段 2 · 巡检启动 · **57 步**(commands 列表 `:2590-2924`,顺序执行,任一失败即回滚)

> 步数由 **AST 精确解析** `commands = [...]` 得出 = **57 个元素**。
> (早期人工统计曾误记为 34/39,已订正。)

### 2.1 冲突检测(步 1-2)

| 步 | 行 | 动作 | 失败 |
|---:|---|---|---|
| 1 | `:2592` | 检测 `route_recorder` / `go2map_capture` / `z1pro_video_loop.sh` | `PATROL_MODE_CONFLICT` **exit 4** |
| 2 | `:2602` | 检测是否已有 `waypoint_follower` 等在跑 | `PATROL_ALREADY_RUNNING` **exit 4** |

### 2.2 run 目录与文件准备(步 3-11)

| 步 | 行 | 动作 | 失败 |
|---:|---|---|---|
| 3 | `:2612` | `mkdir -p <日期目录>` | — |
| 4 | `:2613` | **`mkdir <run 目录>`**(非 `-p`,靠原子性防重入) | `PATROL_LOG_DIR_EXISTS` **exit 5** |
| 5 | `:2617` | **`rm -f .patrol_active .motion_enabled`** —— 启动前预清门控 | — |
| 6 | `:2618` | `cp -- <路线> route_original.csv` | **exit 5** |
| 7-8 | `:2619/:2620` | (续) 路线相关拷贝 | **exit 5** |
| 9 | `:2621` | `printf '%%s' <manifest 内容> > manifest.txt` | — |
| 10 | `:2622` | (续) | — |
| 11 | `:2624` | `mkdir -p <run 目录>; printf '%%s\n' <run 路径> > current_patrol_run` | — |

### 2.3 ★ 建立 /tmp 日志软链(步 12-18)—— 7 个

| 步 | 行 | 软链 |
|---:|---|---|
| 12 | `:2627` | `/tmp/go2_saas_sdk_receiver.log` |
| 13 | `:2629` | `/tmp/go2_saas_cmd_vel_sender.log` |
| 14 | `:2631` | `/tmp/go2_saas_safe.log` |
| 15 | `:2632` | `/tmp/go2_saas_follower.log` |
| 16 | `:2634` | `/tmp/go2_saas_rosbag.log` |
| 17 | `:2636` | `/tmp/go2_saas_performance.log` |
| 18 | `:2638` | `/tmp/go2_saas_experiment_telemetry.log` |

全部用 `ln -sfn` 指向本次 run 目录内对应日志。
→ **运维价值**:不必知道 run 目录名,固定路径即可查看"当前巡检"的任一进程日志。

### 2.4 基础层与观察层(步 19-27)

| 步 | 行 | 动作 | 失败 |
|---:|---|---|---|
| 19 | `:2640` | `echo PATROL_LOG_DIR=<run 目录>` | — |
| 20 | `:2642` | **`bash <base bringup>`** —— 拉起 livox + fastlio;失败时 `rm -f` 并**透传原退出码** | 透传 |
| 21 | `:2646` | `go2_experiment_snapshot.py --phase start` → `system_start.json` | `EXPERIMENT_START_SNAPSHOT_FAILED` **exit 48** |
| 22 | `:2661` | 启动 `go2_experiment_telemetry.py --profile patrol` | — |
| 23 | `:2670` | 轮询 `telemetry_ready` | — |
| 24 | `:2685` | 未就绪 | `EXPERIMENT_TELEMETRY_NOT_READY` **exit 49** |
| 25 | `:2695` | 启动 `patrol_performance_monitor.py --interval 1.0` | — |
| 26 | `:2704` | 轮询 `performance_ready` | — |
| 27 | `:2719` | 未就绪 | `PERFORMANCE_MONITOR_NOT_READY` **exit 50** |

### 2.5 运动末端(步 28-32,自下而上)

| 步 | 行 | 动作 | 失败 |
|---:|---|---|---|
| 28 | `:2729` | `GO2_SDK_MAX_VY=0.020 build/go2_cmd_vel_bridge/go2_sdk2_udp_receiver <if> 5005`,**sleep 4** | — |
| 29 | `:2736` | `grep -Eq 'StandUp ret=[1-9][0-9]*\|BalanceStand ret=[1-9][0-9]*'` | `MOTION_SDK_NOT_READY` **exit 42** |
| 30 | `:2743` | 检查 receiver 是否提前退出 | `MOTION_SDK_NOT_READY` **exit 42** |
| 31 | `:2751` | `ros2 run go2_cmd_vel_bridge cmd_vel_udp_sender`,sleep 1 | — |
| 32 | `:2756` | `ros2 run go2_fastlio_patrol unitree_safe_cmd_node`,sleep 1 | — |

> ⚠️ 第 29 步的 grep 期待 `StandUp ret=<非零>`,而 `go2_sdk2_udp_receiver.cpp:91,93`
> **调用后不打印返回值** —— 该门禁永远匹配不到。见 `10_open_questions.md`。

### 2.6 定位与路线坐标系(步 33-35)

| 步 | 行 | 动作 | 失败 |
|---:|---|---|---|
| 33 | `:2762` | `bash <check_fastlio_freshness> --fresh-only` | `FASTLIO_NOT_FRESH_AFTER_STARTUP` **exit 44** |
| 34 | `:2767` | 路线坐标系准备:`manual_route_anchor.py` +(水平模式)`build_horizontal_route.py` | `ROUTE_FRAME_PREPARATION_FAILED` **exit 47** |
| 35 | `:2772` | `check_route_start_alignment.py --max-distance --max-yaw-error`,失败**透传退出码** | 透传 |

**34° 倾角在第 34 步被处理掉**,详见 `03_dataflow.md`。

### 2.7 守护与录包(步 36-40)

| 步 | 行 | 动作 | 失败 |
|---:|---|---|---|
| 36 | `:2783` | 若未在跑则 `setsid nohup` 拉起 `go2_base_health_watchdog.py` | — |
| 37 | `:2791` | `nice -n 10 ros2 bag record -o rosbag <8 话题>` | — |
| 38 | `:2796` | 轮询 `rosbag_ready`(`PATROL_ROSBAG_READY_ATTEMPTS=50` × `..._POLL_SECONDS=0.2` 秒) | — |
| 39 | `:2809` | 未就绪 | `ROSBAG_NOT_READY` **exit 41** |
| 40 | `:2818` | `ROSBAG_READY attempt=$startup_attempt` | — |

### 2.8 门控开启与熔断器(步 41-47)

| 步 | 行 | 动作 | 失败 |
|---:|---|---|---|
| 41 | `:2820` | `mkdir -p <video run 目录>; touch <video.run>` | — |
| 42 | `:2825` | 视频循环未在跑则 `touch patrol_video.active` 并 `setsid nohup ... video-loop --seconds 20 --upload` | — |
| 43 | `:2838` | **`touch .patrol_active`** → 熔断器存活条件成立 | — |
| 44 | `:2839` | 启动 `localization_session_guard.py` | — |
| 45 | `:2848` | 轮询 `guard_ready`(`PATROL_GUARD_READY_ATTEMPTS=50` × `..._POLL_SECONDS=0.2` 秒) | — |
| 46 | `:2863` | 未就绪 | `LOCALIZATION_SESSION_GUARD_NOT_READY` **exit 46** |
| 47 | `:2869` | `LOCALIZATION_SESSION_GUARD_READY attempt=$startup_attempt` | — |

### 2.9 ★ 控制器启动与放行(步 48-57)

| 步 | 行 | 动作 | 失败 |
|---:|---|---|---|
| 48 | `:2872` | `detached_command(follower_cmd)` → `python3 <WS>/scripts/waypoint_follower_go2_2_trace.py` | — |
| 49 | `:2874` | **等 30 次 × 0.5 秒 = 最长 15 秒**,须同时满足三信号:<br>`FOLLOWER_EXACT_TRACE_READY` + `FOLLOWER_ODOM_READY` + `follower_frame_ready_probe` | — |
| 50 | `:2885` | 三信号未齐 | `FOLLOWER_NOT_READY` **exit 43** |
| 51 | `:2892` | `FOLLOWER_WARM_READY` | — |
| 52 | `:2895` | `bash ensure_base.sh --patrol-start-gate`(CPU / FAST-LIO / follower 稳定性) | `PATROL_START_GATE_FAILED` **exit 45** |
| **53** | **`:2900`** | **`touch .motion_enabled`** ← **真正的松刹车** | — |
| 54 | `:2902` | 等 20 次 × 0.1 秒 = 最长 2 秒,确认 `FOLLOWER_MOTION_INTERLOCK_RELEASED` | — |
| 55 | `:2908` | 未确认释放 | `FOLLOWER_MOTION_RELEASE_FAILED` **exit 51** |
| 56 | `:2913` | `FOLLOWER_STARTED motion_interlock=released` | — |
| 57 | `:2916` | `PATROL_STARTED route=... speed=... loop=... log_dir=...` | — |

> `follower_frame_ready_probe` 是**互斥三分支**(saas `:2505-2516`):
> ```python
> grep FOLLOWER_HORIZONTAL_FRAME_READY   if use_horizontal_frame
> else grep FOLLOWER_BODY_YAW_READY      if use_body_yaw_alignment
> else "true"
> ```

### 实测启动耗时(7/26 最后一次,trace 的 monotonic 时间戳)

```
9657.67  首条 control 记录     source=startup_interlock_zero, motion_enabled=false
9661.97  horizontal_frame_ready   (+4.3 秒,重力标定完成)
9662.02  horizontal_route_anchored(+0.05 秒,路线锚定, route_rotation_deg=-15.67°)
9669.71  motion_interlock_released(+7.7 秒,过 odom 门禁)
─────────  从首条控制到放行共 12.0 秒
9999.11  trace_stop  reason=signal_15   (总时长 341.4 秒)
```

### 失败回滚

任一步失败都执行 `startup_cleanup`(`:2568-2589`):
```
rm -f .patrol_active .motion_enabled patrol_video.active video.run
kill -TERM <控制类进程>
终止 performance / telemetry / rosbag / guard 各进程组
kill -INT <rosbag 进程>
```
→ **门控文件先删,再杀进程** —— 顺序保证 follower 在被杀之前就已停止发指令。

---

## 阶段 3 · 巡检运行中(20 Hz 闭环)

```
每个控制周期(50 ms):
  ① follower 读 .motion_enabled 是否存在                    (trace.py:896)
  ② 合成 motion_enabled = 文件在 且 已上报释放/释放新鲜 且 …  (trace.py:918-921)
  ③ 若为真 → super().control_loop() 走纯追踪               (trace.py:977)
     若为假 → 发布零速 Twist                                (trace.py:981)
  ④ 逐周期写 trace jsonl(含 pose/target/nearest/projection/control/horizontal_frame)
```

**实测**:6828 条 control 记录(含启动互锁期 240 条)/ 3093 次 odom 回调 → 全程复用比 **2.21**;
仅 active 期为 6588 / 3025 → **2.18**。audit 报告正文用的是后者。

---

## 阶段 4 · 停止与收尾(`stop_patrol_command`,`:2928` 起)

```
1. 杀进程:控制类 kill -TERM,rosbag kill -INT(优雅收尾,保证 db3 完整)
2. 删门控:.patrol_active / .motion_enabled / patrol_video.active / video.run
          + /tmp/go2_saas_video.run(:2936 两个历史路径都删)
3. 删 current_patrol_run(:3014/:3022)
4. manual_route_anchor.py --capture-only  → 记录停止时刻位姿(:2964)
5. go2_experiment_snapshot.py --phase end → system_end.json(:2970)
6. go2_experiment_audit.py                → 全链路体检(saas :2998 调用)
7. sha256sum 5 个证据文件 → evidence_sha256.txt
```

**启动失败时的特殊保护**(`:3059-3070`):
```python
if rc != 0:
    current_run = CURRENT_PATROL_RUN_FILE.read_text().strip()
    # 注释:只 finalize 本次尝试分配的目录。
    #       重复启动被拒绝时,绝不能停掉已在运行的巡检。
    if expected_run and current_run == expected_run:
        stop_patrol_command()
```

---

## 阶段 5 · 录制链(与巡检互斥,由第 1 步的冲突检测保证)

`route_recording_blackbox.py` 编排,启动 **5 个进程**:

| 进程 | 命令 |
|---|---|
| telemetry | `go2_experiment_telemetry.py --profile **recording**` |
| performance | `patrol_performance_monitor.py --interval 1.0` |
| rosbag | `ros2 bag record` × **9 个话题** |
| session_guard | `localization_session_guard.py` |
| route_recorder | **`ros2 run go2_fastlio_patrol route_recorder`** `-p min_distance:=0.400 -p startup_enable_file:=...` |

**录制录 9 个话题**(比巡检多手柄、少控制指令):
`/Odometry` `/livox/imu` **`/lf/lowstate`** `/lf/sportmodestate`
**`/lf/wirelesscontroller`** **`/wirelesscontroller`** **`/wirelesscontroller_unprocessed`** `/tf` `/tf_static`

**录制独有的两道门禁**:`observer_warmup_gate`(预热)→ `formal_recording_gate`(正式录)
→ 预热期丢弃 15 帧后才开始记点。

---

## 阶段 6 · 视频(常驻服务 + 门控)

```
go2-saas-video.service 常驻存活
  → video-loop 每轮检查 patrol_video.active 是否存在
     不存在 → print("VIDEO_LOOP_IDLE waiting_for_patrol") 空转     (:3396-3398)
     存在   → 录 20 秒一段 → 上传
```

**运行证据**:7/26 共 24 段视频,**100% 落在两次巡检窗口内**;
xbf9 录制窗口(18:59:29–19:12:18)内 **0 段**。

> `route_recording_blackbox.py` 全文 grep `video` = **0 处** ——
> 录制脚本**不需要主动停视频**,因为视频默认就不录。

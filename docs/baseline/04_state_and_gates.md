# 04 · 状态与门控

> ⚠️ **基准:2026-07-27 整机快照(删除阶段之前)。**
> 本篇描述的是**删代码之前**的狗端状态,内容在该基准下成立。
> 读时注意两点:
> 1. **有些组件现在已从仓库删除**(4G 治理整套、legacy 命令桥、22 个脚本等)——
>    见 `13_file_accounting.md` 与 `14_dog_verification.md`。文档没写错,是那个时间点确实有。
> 2. **`go2_saas_agent.py` 的行号已漂移**(3630 → 3633 行,删除阶段改过 4 处)。
>    换算:引用 `1–52` 不用调 · `53–480` **+3** · **`481–3184` +5**(绝大多数引用在此段) · `3186` 之后 +3。
>    其他文件行号不受影响。
> 已用已知答案自检:本文若写 `commands` 列表在 `:2590`,当前仓库实测在 `:2595`,差 5 ✓

> 这个系统的进程之间**没有函数调用**,靠"文件存在与否"来同步状态。
> 本文列出全部 7 条共享状态,每条给**谁创建 · 谁读 · 谁删**。

---

## 一、系统状态机

```
        ┌──────────────┐
        │  通电 / 开机   │
        └──────┬───────┘
               │ systemd 拉起 4 个单元
               ▼
        ┌──────────────────────────────┐
        │ 空闲(saas 三循环常驻)          │  video-loop 空转:VIDEO_LOOP_IDLE
        └──────┬───────────────────────┘
               │ saas base bringup
               ▼
        ┌──────────────────────────────┐
        │ 基础层就绪                     │  livox + fastlio + health_watchdog
        │ /dev/shm 快照开始刷新           │  **狗知道自己在哪,但不会动**
        └──────┬───────────────┬───────┘
               │               │
   云端 start_patrol      人工发起录制
               │               │
               ▼               ▼
    ┌──────────────────┐  ┌──────────────────┐
    │ 巡检启动中(57 步)  │  │ 录制中            │  两者互斥
    │ 任一步失败→回滚    │  │ (第 1 步冲突检测)  │
    └────────┬─────────┘  └──────────────────┘
             │ :2900 touch .motion_enabled
             ▼
    ┌──────────────────────────────────┐
    │ 巡检运行中(20 Hz 闭环)             │
    │ 门控: .motion_enabled 存在          │
    └────────┬──────────────┬──────────┘
             │              │ 熔断:FAST-LIO 会话身份变化
     云端 stop_patrol       ▼
             │      ┌──────────────────┐
             ▼      │ session_guard 删除 │
    ┌──────────────┤ 门控文件 → 强制停止 │
    │ 停止与收尾     │└──────────────────┘
    │ 杀进程→删门控   │
    │ →快照→体检     │
    └──────────────┘
```

---

## 二、7 条共享状态的完整生命周期

### ★ 1. `.motion_enabled` —— 运动放行的唯一开关

| | |
|---|---|
| 位置 | `<run_dir>/.motion_enabled` |
| **创建** | saas `:2900` `touch`(**57 步的第 53 步**,全部门禁通过之后) |
| **读** | `waypoint_follower_go2_2_trace.py:896` `os.path.isfile()` —— **20 Hz 轮询** |
| **删** | saas `:2617`(启动前预清)· saas `:2947`(停止)· `startup_cleanup`(任一步失败) |
| 传递 | saas `:2484` `-p motion_enable_file:=` 给 follower;`:2525` `--enable-file` 给 guard |

follower 侧的合成逻辑(`:916-921`)不只看文件:
```python
motion_requested = os.path.isfile(self.motion_enable_file)          # :896
motion_enabled   = motion_requested and (motion_release_reported or release_fresh) and ...
```
→ 文件在**且**已上报过释放(或释放足够新鲜)才真正发指令,否则 `:981` 发零速 Twist。

### ★ 2. `.patrol_active` —— 熔断器的存活条件

| | |
|---|---|
| **创建** | saas `:2838` `touch` |
| **读** | `localization_session_guard.py:192` **`while os.path.exists(args.active_file):`** ← 主循环条件 |
| **删** | saas `:2617`/`:2946` · **guard 自己 `:114` `os.unlink()`** |

⚠️ **双删除者**。guard 既以它为存活条件,又能主动删它 —— 这是熔断动作的一部分。
guard `:174-175` 启动时若文件不存在则打印 `SESSION_GUARD_NOT_ACTIVE`;guard `:236` 退出时 `SESSION_GUARD_STOPPED ... active_file_removed`。

### ★ 3. `patrol_video.active` —— 视频录制门控

| | |
|---|---|
| 位置 | `patrol_logs/run/patrol_video.active`(saas `:38` 定义) |
| **创建** | saas `:2822`(**57 步的第 42 步**) |
| **读** | saas `:3396` video-loop:不存在则 `print("VIDEO_LOOP_IDLE waiting_for_patrol")` 空转 |
| **删** | saas `:2580`/停止流程 · **`localization_session_guard.py:72` 也删** |
| 其他 | `:486` 心跳上报 `status["camera_loop"] |= 存在?`;`:3415-3417` 检查 mtime 年龄防残留长录 |

⚠️ **双删除者**(saas + guard)。熔断时视频与巡检一起停。

**运行证据**:7/26 共 24 段视频,**100% 落在巡检窗口内**;两次录制窗口内 **0 段**。

### 4. `current_patrol_run` —— 防误停的身份凭证

| | |
|---|---|
| **创建/写** | saas `:2625`(写入本次 run 目录路径) |
| **读** | saas `:3061` `read_text()` 做身份核对 · `:2643` 传给 `ensure_base` |
| **删** | saas `:3014` / `:3022`(停止流程) |

用途(`:3059-3070`,注释原文):
> *Only finalize the directory allocated by this exact attempt.
> A duplicate-start rejection must never stop an already running patrol.*

→ 启动失败要收尾时,先核对"当前 run 目录是不是本次分配的",**不是就不碰**。

### 5. `video.run` —— 服务循环标识

| | |
|---|---|
| **创建** | `install_saas_autostart.sh:16` `${RUN_DIR}/video.run`<br>`start_saas_loops.sh:9` **`/tmp/go2_saas_video.run`** |
| **读** | saas `:3388` `managed_service_loop = (args.run_file == SERVICE_VIDEO_RUN_FILE)` |
| **删** | saas `:2936` **两个路径都删** · guard `:72` |

⚠️ **双路径历史遗留**:两个安装/启动脚本用了不同位置,saas 停止时对两个都做 `rm -f` 兼容。

### 6. `.route_recording_enabled` —— 录制版的运动放行

| | |
|---|---|
| **创建** | `route_recording_blackbox.py:1005` `run_dir / ".route_recording_enabled"` |
| **传递** | `:1217` `-p startup_enable_file:=` 给 route_recorder |
| **读** | `route_recorder.py:66-67` `if self.startup_enable_file and not os.path.isfile(...)` |
| 记录 | `:1294` 写进 manifest 的 `route_writer_gate` 字段 |

`route_recorder.py:34`:`self.recording_enabled = not self.startup_enable_file`
→ **未传该参数则直接开录**;传了就必须等文件出现。
对应 manifest 的 `formal_recording_gate`(观察器预热通过后才正式记点)。

### 7. `/dev/shm/go2_fastlio_latest_odom.txt` —— 定位快照(数据通道)

| | |
|---|---|
| **写** | `laserMapping.cpp:747-763`:先写 `.tmp`,`snapshot.close()` 后 **`std::rename`** —— 原子 |
| **读** | **6 个**:`base_bringup.sh:8` · `ensure_base_ready.sh:10` · `check_fastlio_freshness.py:33` · `manual_route_anchor.py:23` · `localization_session_guard.py:157` · `check_route_start_alignment.py:89` |
| **删** | 无 —— `/dev/shm` 是内存文件系统,重启自动清空 |

**一写六读,是全系统最重要的数据通道。** 用 `rename` 保证读者永远看不到半截内容。

---

## 三、门控之外的进程管理

| 机制 | 用途 |
|---|---|
| `*.pgid` 文件 | 记录 detached 进程组 id,停止时按组终止(`performance_monitor.pgid` / `rosbag.pgid` / `experiment_telemetry.pgid` / `localization_session_guard.pgid`) |
| `heartbeat-safe.pid` | ⚠️ **残留** —— 该服务只在 2026-07-19 16:43→18:40 活过 2 小时,之后再未运行,pid 文件留存至今 |
| `outbox.run.paused_20260716_2150`<br>`video.run.paused_20260716_2150` | 2026-07-16 21:50 的暂停标记,留存至今 |

---

## 四、失败回滚的顺序 ★

`startup_cleanup`(saas `:2568-2589`),57 步中任一步失败都执行:

```
1. rm -f .patrol_active .motion_enabled patrol_video.active video.run
2. kill -TERM <控制类进程>
3. 终止 performance / telemetry / rosbag / guard 各进程组
4. kill -INT <rosbag 进程>   ← 用 INT 而非 TERM,保证 db3 正常收尾
```

> **先删门控,再杀进程** —— 这个顺序保证 follower 在被杀之前就已经因为
> `.motion_enabled` 消失而停止发指令,不会出现"进程死了但最后一条速度指令还在路上"。

---

## 五、互斥关系

| A | B | 由谁保证 |
|---|---|---|
| 巡检 | 录制 / 建图 / 手动视频 | saas `:2598` 第 1 步冲突检测 → **exit 4** |
| 巡检 | 另一个巡检 | saas `:2609` 第 2 步 + `:2614` `mkdir` 原子性 → **exit 4 / 5** |
| 视频录制 | 非巡检时段 | `patrol_video.active` 门控 |
| **基础层重启** | 任何活跃作业 | saas `start_base` 先扫 7 类进程(`route_recorder` · `go2map_capture` · `waypoint_follower` · `unitree_safe_cmd_node` · `cmd_vel_udp_sender` · `go2_sdk2_udp_receiver` · `localization_session_guard`),有则 `BASE_RESTART_BLOCKED_ACTIVE_MODE` → **exit 4** |

> 即:**巡检或录制进行中,云端下发 `start_base` 会被拒绝**,不会把正在跑的作业连根拔起。

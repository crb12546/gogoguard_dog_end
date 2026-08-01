# 06 · 不变量清单 ★

> **改造时不能破坏的约束。** 每条给:约束 · 为什么 · 违反后果 · 证据位置。
>
> 这份文档的价值在于:这些约束**读单个文件读不出来**,必须把整条因果链走通才知道。
> 一旦丢失,需要重走整条链才能找回 —— 而且往往是在已经改坏之后才发现。

---

## I 类 · 坐标系与定位(违反 = 狗走偏,且不报错)

### I-1 · 34° 修正必须发生在 FAST-LIO **输出之后**,不得去改 FAST-LIO 参数

**约束**:`extrinsic_R` 必须保持单位阵,`extrinsic_est_en` 必须保持 `false`。

**为什么**:这两个参数描述的是**雷达 ↔ 雷达内置 IMU**(同一设备内),
单位阵是**正确**的。而 34° 是**雷达 ↔ 狗身**的安装角,是另一个量。
> audit `:1695-1699` `mounting_note` 原文:
> *这些值描述的是雷达/雷达内置 IMU 的配置变换,不独立测量物理的雷达→狗身安装角度。*

**违反后果**:若把 34° 塞进 `extrinsic_R`,FAST-LIO 内部的 IMU 预积分与去畸变
会建立在错误的设备间关系上;同时双 IMU 互校验(`disagreement ≤ 3°`)会失去意义 ——
因为两个源的参考基准都被动过了。**且不会报错。**

**证据**:生产 yaml `:37-41`;`horizontal_frame.py:243-311`;`xbf.leveling.json`

---

### I-2 · `q_ground_from_map` 锁定后必须冻结,不得在跑动中更新

**约束**:一次会话内只标定一次,`ready = True` 之后永不改变。

**为什么**:路线 CSV 是在**某一个**水平系下变换好的。若旋转在跑动中变化,
路线与位姿会落进两个不同的系。

**违反后果**:狗跑着跑着"世界"在转,横向偏差持续漂移,无任何报警。

**证据**:`horizontal_frame.py` `add_sample()` 首行 `if self.ready: return False`

---

### I-3 · `q_ground_from_map` 的 **qz 分量必须为 0**

**约束**:只旋 roll/pitch,绝不动 yaw。

**为什么**:yaw 是路线朝向的基准。动了 yaw 等于把整条路线转了个角度。

**违反后果**:狗沿着一条被整体旋转过的路线走 —— 起点看着对,越走偏得越多。

**证据**:实测 `q_ground_from_map = [-0.03344, 0.29317, **0.0**, 0.95547]`;
`frame_contract` 明示 `yaw_normalization_applied: false`

---

### I-4 · 水平化的四道门禁不得放宽

| 门禁 | 阈值 | 实测余量 |
|---|---:|---:|
| 双源重力夹角 `disagreement` | ≤ **3.0°** | 0.91°(3 次独立运行:1.00 / 0.52 / 0.70) |
| 样本离散度 `spread` | ≤ **1.5°** | 0.76°(0.83 / 0.75 / 0.60) |
| 最少样本数 | ≥ **15** | 15 |
| 静止判据 `gyro` | ≤ **0.08** | 三个拒绝计数均为 0 |

**为什么**:这是唯一能发现"某个 IMU 出问题"的机制。两源互为裁判。

**违反后果**:单个 IMU 漂移时不再被拦截,直接锁进一个错误的水平系,**整个会话都是错的**。

**证据**:`xbf.leveling.json.level_frame` 与 `independent_runtime_validation`

---

### I-5 · FAST-LIO 会话身份必须用**四元组**判定

**约束**:`boot_id` + `pid` + `start_ticks` + `executable`,缺一不可。

**为什么**:pid 会被复用;只看 pid 会把"新进程恰好拿到同一个 pid"误判为同一会话。

**违反后果**:定位进程重启后未被熔断,狗按旧坐标系继续跑。

**证据**:`route_link.json.localization_session_start/end`;`localization_session_guard.py`

---

## II 类 · 运动安全(违反 = 狗可能失控)

### II-1 · `.motion_enabled` 是**唯一**的运动放行点,不得绕过

**约束**:任何重构都必须保留"文件存在才发指令"这条路径。

**为什么**:它是整个 57 步启动链最后一道闸,前面所有门禁的意义都汇聚于此。

**违反后果**:狗在定位未就绪/未稳定时就开始动。

**证据**:创建 saas `:2900`;读 `trace.py:896`(20 Hz);
合成条件 `:916-921`(文件在 **且** 已上报释放 **或** 释放新鲜)

---

### II-2 · 失败回滚必须**先删门控,再杀进程**

**约束**:`startup_cleanup` 的四步顺序不可颠倒。

**为什么**:先删门控,follower 在下一个 50 ms 周期就会自己发零速;
若先杀进程,可能出现"进程死了,但最后一条非零速度指令已经在 UDP 路上"。

**违反后果**:狗在收尾过程中带着残余速度继续动。

**证据**:saas `:2568-2589`

---

### II-3 · rosbag 必须用 `kill -INT`,不能用 `TERM`

**约束**:停止 rosbag 一律 SIGINT。

**为什么**:SIGINT 让 `ros2 bag` 正常收尾并 flush sqlite;TERM 会留下损坏的 db3。

**违反后果**:证据链断裂,事后无法复盘。

**证据**:saas `startup_cleanup` 与 `stop_patrol_command`

---

### II-4 · 限幅必须走完整条关卡链

**约束**:分析或修改任何速度分量时,必须同时检查三道关卡。

```
① unitree_safe_cmd_node   -p max_vx / max_vy / max_yaw_rate
② cmd_vel_udp_sender      -p max_vx / max_vy / max_vyaw  → limit()
③ go2_sdk2_udp_receiver   硬编码 ±0.5f / ±0.10f / ±0.5f
```

**为什么**:最紧的那道才是生效值。

**违反后果**:得出错误结论。**本项目已实际发生过**:
只看 ③ 的 `±0.10f` 会以为横向速度上限是 0.10,
而 ①② 都是 `max_vy:=0.000` —— **实际全链路为 0,狗根本不做横移**
(manifest `command_vy=0.000` 是直接证据)。

---

### II-5 · 观察者必须先于被观察者启动

**约束**:57 步中 telemetry / performance_monitor 的启动**必须**排在
运动末端(receiver / sender / safe node)之前。

**为什么**:否则运动链最初几秒的行为没有任何记录。

**违反后果**:出问题时最关键的启动瞬间是盲区。

**证据**:saas 57 步顺序 —— 观察层步 22/25(`:2661`/`:2695`)早于运动末端步 28/31/32(`:2729`/`:2751`/`:2756`)

---

### II-6 · ⚠️ `/cloud_registered_body` 的 frame_id 名不副实 —— 现有 ROI 是斜切片

**事实**(第一步交付漏记,方案审查时补入):
```cpp
laserMapping.cpp publish_frame_body():
    RGBpointBodyLidarToIMU(&feats_undistort->points[i], ...)   // 只做 雷达→IMU
    laserCloudmsg.header.frame_id = "base_link";               // 却标称机身系

RGBpointBodyLidarToIMU():
    p_body_imu = offset_R_L_I * p_body_lidar + offset_T_L_I
                 ↑ = 生产 yaml 的 extrinsic_R = 【单位阵】
```

→ 雷达系与 IMU 系外参是单位阵(仅差 `extrinsic_T` 几厘米平移),
  **故 `/cloud_registered_body` 实质就是雷达系,带着 33–34° 安装倾角,却标称 `base_link`**。

**后果**:`unitree_safe_cmd_node` 的 ROI 盒
`x∈[0.35,1.50] · y∈[-0.30,0.30] · z∈[0.30,0.90]`
被套用在这片倾斜点云上,**在真实机身坐标下是一个绕轴转了约 34° 的斜切片**。

**尚未确定的是方向**:
- 若倾斜使地面点落入 `z∈[0.30,0.90]` → **误停**(把地面当障碍)
- 若倾斜使真实障碍落到 ROI 之外 → **漏停**(危险)

实测 341 秒触发 69 次(占配对命令 1.66%),未见持续误报,
推测 `min_stop_points=15` 的阈值滤掉了零星点。**但这只是推测,必须实测。**

**约束**:任何对避障的改造,**必须先把点云扶正到真实机身系再套 ROI**,
且扶正要用**运行时姿态**(不能用冻结的安装角 —— 实测 FAST-LIO 与机身 IMU 的
相对姿态变化 P95 达 38.47°/31.39°/25.81°,见 `09_measured_baseline.md`)。

**验证手段**:用 `patrol_logs/runs/20260723/xunjian-20260723-01` 的 bag
(全狗仅 2 份含原始点云之一)离线回放,对比扶正前后的 `would_stop` 帧数。

---

## III 类 · 数据完整性(违反 = 悄悄用到错数据)

### III-1 · `/dev/shm` 快照必须**先写 .tmp 再 rename**

**约束**:保持 `std::rename` 原子替换,不得改成直接写目标文件。

**为什么**:**6 个读者**随时可能在读。非原子写会让读者拿到半截内容。

**违反后果**:锚定/新鲜度检查读到残缺位姿,可能得到看似合法的错误坐标。

**证据**:`laserMapping.cpp:747-763`,`snapshot.close()` 后 `std::rename`

---

### III-2 · 路线认定必须**靠 sha256,不靠文件名**

**约束**:保留 content-alias 机制。

**为什么**:这不是洁癖 —— `routes/` 里**已经存在内容错配的文件**:
```
xbf2.csv                 1278 行  sha 7f4312a12935c451
xbf2.csv.horizontal.csv   359 行  sha 6778d8c4086677f4   ← 行数对不上
xbf8.csv.horizontal.csv   359 行  sha 6778d8c4086677f4   ← 与上完全相同
```
**`xbf2.csv.horizontal.csv` 里装的是 xbf8 的水平化结果。**

**违反后果**:按文件名取用会跑错路线。

**证据**:saas `:1249-1368`,注释 *never trust a filename or a sidecar hash alone*

---

### III-3 · run 目录必须用 `mkdir`(非 `mkdir -p`)

**约束**:保留"目录已存在即失败"的原子性。

**为什么**:这是防止两个巡检写进同一目录的最后一道保险。

**违反后果**:并发巡检互相覆盖证据。

**证据**:saas `:2614` → `PATROL_LOG_DIR_EXISTS` **exit 5**

---

### III-4 · 启动失败收尾前必须核对 `current_patrol_run`

**约束**:保留身份核对逻辑。

**为什么**:注释原文 —
> *Only finalize the directory allocated by this exact attempt.
> A duplicate-start rejection must never stop an already running patrol.*

**违反后果**:第二次误触"开始巡检"被拒后,会把**正在跑的第一次**给停掉。

**证据**:saas `:3059-3070`

---

## IV 类 · 跨组件阈值一致性

### IV-1 · **5000 点**阈值必须三处一致

| 层 | 位置 | 值 |
|---|---|---|
| Livox 驱动 | `pub_handler.cpp:274` | `points < 5000` → 丢帧 |
| 启动检查器 | `check_livox_stream.py` | `points >= 5000` |
| FAST-LIO | `laserMapping.cpp:108` | `kMinLivoxFramePoints = 5000` |

**为什么**:三层对"什么算一帧好数据"必须有相同定义。

**违反后果**:出现"驱动放行但 FAST-LIO 拒绝"(或反之)的灰区,
表现为定位间歇性中断且无明确原因。

---

### IV-2 · 驱动层丢帧判据不得放宽

**约束**:`frame_span > publish_interval × 2 || points < 5000` 必须保留。

**为什么**:代码注释给出了根因 —
> *its per-point offsets would make IMU undistortion integrate across the discontinuity*

跨周期帧会让 FAST-LIO 的 IMU 去畸变**跨不连续点积分**。

**违反后果**:位姿在异常帧后跳变。

**证据**:`pub_handler.cpp:271-279`

---

### IV-3 · QoS 组合 `lidar_depth=2` + `imu_depth=400` 是配套的

**约束**:两者须一起考虑,不可单独调整。

**为什么**:生产 yaml `:15-16` 注释 —
> *Patrol needs the freshest scan rather than replaying a deep FIFO.
> Keep enough IMU history to propagate the state across skipped scans.*

**违反后果**:雷达队列加深 → 用到陈旧扫描;IMU 队列变浅 → 跳帧时无法传播状态。

**证据**:manifest `fast_lio_input_qos=lidar_2_best_effort_imu_400_reliable`

---

## V 类 · 证据链

### V-1 · 巡检启动必须校验录制 sidecar

**约束**:`route_link.json` 的 `status` 只接受 `complete` / `complete_with_warnings`。

**违反后果**:用一份未完成或已损坏的录制去跑巡检。

**证据**:saas `route_recording_evidence`;manifest `route_recording_evidence=linked_hash_verified`

---

### V-2 · 停止时的 sha256 固化不得省略

**约束**:保留 `evidence_sha256.txt`(5 个证据文件的哈希)。

**为什么**:这是事后证明"分析用的就是当时那份数据"的唯一手段。

**证据**:saas `:2928-3023` stop 流程

---

## 使用建议

改造前,对照本清单逐条自问:**"我这个改动会不会碰到其中任何一条?"**

其中 **I-1 / I-2 / I-3 / II-1 / II-4** 是最容易在"看起来无害的重构"中被破坏的五条。

# 01 · 术语表

> ⚠️ **基准:2026-07-27 整机快照(删除阶段之前)。**
> 本篇描述的是**删代码之前**的狗端状态,内容在该基准下成立。
> 读时注意两点:
> 1. **有些组件现在已从仓库删除**(4G 治理整套、legacy 命令桥、22 个脚本等)——
>    见 `13_file_accounting.md` 与 `14_dog_verification.md`。文档没写错,是那个时间点确实有。
> 2. **`go2_saas_agent.py` 的行号已漂移**(3630 → 3633 行,删除阶段改过 4 处)。
>    换算:引用 `1–52` 不用调 · `53–480` **+3** · **`481–3184` +5**(绝大多数引用在此段) · `3186` 之后 +3。
>    其他文件行号不受影响。
> 已用已知答案自检:本文若写 `commands` 列表在 `:2590`,当前仓库实测在 `:2595`,差 5 ✓

> 本系统自造概念的统一定义。每条给出**定义 · 代码出处 · 为什么存在**。
> 工程根 `WS = /home/unitree/go2_fastlio_ws`。

---

## 一、定位与坐标系

### FAST-LIO 会话(session)
一次 `fastlio_mapping` 进程的生命周期。**它的坐标原点 = 进程启动那一刻狗所在的位置**。
进程一重启,原点就换地方。
> 身份由四元组标识:`boot_id` + `pid` + `start_ticks` + `executable`
> —— `route_link.json` 的 `localization_session_start/end`

**为什么重要**:录路线时记的绝对坐标,重启后失去意义。整个"锚定"设计都源于此。

### manual_anchor(手动锚点)
**当前唯一在用的定位模式**。把狗摆到路线物理起点,取当前 FAST-LIO 位姿作锚,
把整条路线 CSV 做 **SE(2) 刚体变换**搬进当前会话。
> `go2_saas_agent.py:602-702` `localization_mode_from_params`
> 产物:`manual_anchor.json`,schema `go2.manual_anchor.v1`

### origin / direct / none
**历史模式,现已是 manual_anchor 的别名**。原意是"直接用录制时的绝对坐标"。
> `go2_saas_agent.py:609-611` 注释原文:
> *Comparing their old absolute CSV coordinates with a new FAST-LIO origin after a reboot is not meaningful.*
>
> 运行证据:7/23–7/25 有 16 次 `mode=origin` 且**全部无 `manual_anchor.json`**;
> 7/25 之后 19 次 `mode=manual_anchor` 其中 17 次有该文件。**切换是真实发生的。**

### pcd 模式 / route_relocalizer
用**事先建好的点云地图**做 ICP 匹配来确定位置。
> `src/go2_map_manager/src/route_relocalizer.cpp`(1004 行),saas `:2386` 启动
>
> **运行证据:132 次巡检记录中,命中 0 次。从未运行过。**

⚠️ 即使启用,saas 传的 `anchor_route_start:=true` 会让它在 `:956-957`
**把 ICP 算出的平移再叠加一个修正,强制路线起点落回狗当前位置**;
若 ICP 位置与当前位置差超阈值则 `RELOCALIZE_FAILED` 退出码 4。
→ 它**不是**"狗放哪都能自己找到位置",同样要求摆在起点。

### session_only
第三种模式,只保证"同一个 FAST-LIO 进程会话内有效",进程重启即失效。
> `maps/console/xbf.leveling.json` 的 `localization_session.mode`

### horizontal_frame(重力水平系)
把倾斜的 FAST-LIO 世界系旋正到重力水平的一个**冻结刚体旋转**。
> `scripts/horizontal_frame.py`(349 行);运行产物 `route_horizontal.csv` / `route_horizontal.json`

### q_ground_from_map
上述旋转本身。**qz 分量恒为 0** —— 只旋 roll/pitch,不动 yaw。
> 实测值 `xbf.leveling.json`:`[-0.03344, 0.29317, **0.0**, 0.95547]`
> `frame_contract` 明示:`translation_applied:false` · `yaw_normalization_applied:false` · `z_percentile_shift_applied:false`

### tilt_removed_deg
被旋掉的倾角,即 FAST-LIO 世界系 z 轴与真实重力的夹角。
> **实测 34.3240°**(`xbf.leveling.json.proof`)。独立复算 `arccos(0.8259)` = 34.3240° ✅

⚠️ 与之相关但**不同**的三个量,全部落在 32–34° 区间,互相印证:
| 量 | 值 | 出处 |
|---|---:|---|
| `tilt_removed_deg` | **34.32°** | 要旋掉的量 |
| 机身↔传感器夹角 | **33.10°** | 由 `q_sensor_from_body` 复算 |
| 双 IMU 重力夹角 | 31.87 / 31.98° | `build_horizontal_route` 的 `measured_body_lidar_gravity_angle_deg` |
| 录制轨迹平面拟合倾角 | 23–24° | `tilt_deg`,受路面本身起伏影响,**不是安装角** |

**❌ 12.3° / 13.0° 不是实测值**,是早期未部署工具里的人估参数
(`level_cloud_node.py:19`、`level_pcd.py:99`),
而且 `maps/extrinsic_est_false_check/level_sweep/` 里有 9 个 PCD,
文件名 `pos10.0`…`pos16.0` —— 证明当年是**肉眼从 10° 试到 16°** 挑的。

---

## 二、门控与状态

### 门控文件(gate file)
一个**存在即为真**的空文件,用于跨进程传递开关状态。本系统的核心同步机制。
> 全部 7 个见 `04_state_and_gates.md`

### `.motion_enabled`
**运动放行的唯一开关**。它存在,follower 才发速度指令;不存在就发零。
> 创建 saas `:2900 touch` · 读 `waypoint_follower_go2_2_trace.py:896`(20 Hz `os.path.isfile`)
> 删 saas `:2617`(启动前清)/`:2947`(停止)

### 启动互锁(startup motion interlock)
follower 起来后**先不动**,直到通过全部就绪门禁才放行。
> manifest:`startup_motion_interlock=trace_wrapper_zero_until_post_follower_gate`
> 首条 control 记录:`source="startup_interlock_zero"`, `motion_enabled=false`
> **实测:启动到放行 12.0 秒**(重力标定 4.3s → 锚定 → 再等 7.7s 过 odom 门禁)

### 熔断器(session_guard)
`scripts/localization_session_guard.py`。监视 FAST-LIO 会话身份,
一旦发现进程变了(重启/换 pid),**删除门控文件,连带停掉巡检与视频**。
> 读 `.patrol_active`(`:192` while 主循环条件)· 删 `.patrol_active`(`:114`)
> 删 `patrol_video.active`(`:72`)· 读 `.motion_enabled`(saas `:2525` 传 `--enable-file`)
> 对应 manifest:`localization_restart_policy=abort_current_patrol`

**注意**:此角色**无法从调用关系看出**,它由"删哪几个文件"定义。

### 观察器预热门禁(observer_warmup_gate)
录制专有。CPU 与连续 FAST-LIO 帧数达标后,才开始正式录制。
> `manifest.json.observer_policy`:`formal_recording_after_warmup_gate: true`
> 实测:`{"available":true,"ready":true,"returncode":0}`,预热期丢弃 15 帧

### 正式录制门禁(formal_recording_gate)
录制专有。与上者配套,产出 `formal_recording_gate.log`。

---

### 云端命令词表(saas 顶层常量)

| 常量 | 值 |
|---|---|
| `START_PATROL_COMMANDS` `:51` | `start_patrol` `patrol_start` `follow_route` `start_route` `auto_patrol` `auto_inspection` |
| `STOP_PATROL_COMMANDS` `:52` | `stop_patrol` `patrol_stop` `stop_route` |
| `SAFE_COMMANDS` `:53` | `ping` `noop` `status` `start_base` `stop_base` `camera_start_loop` `camera_stop_loop` |
| `GOTO_COMMANDS` `:50` | `move` `walk` `go` `goto` `navigate` —— **一律拒绝**(`:3175-3176` "not implemented in v1.5") |
| `ROUTE_NAME_RE` `:55` | `^[A-Za-z0-9_\-]{1,80}(\.csv)?$` —— 路线名白名单 |
| `MEDIA_TIMESTAMP_RE` `:48` | `(?:^|[_-])(?P<date>\d{8})[_-]\d{6}` —— 媒体文件时间戳解析 |
| `DEFAULT_BACKEND_BASE` `:40` | `https://39.96.37.187/api/v1` |
| `DEFAULT_ROBOT_ID` `:41` | `go2-tju-01`(可被 `GO2_ROBOT_ID` 覆盖;云端应答里的 `LLYJ0001` 是云侧 ID) |
| `OUTBOX_MAX_BACKOFF` `:45` | 300 秒 —— 上传失败退避上限 |

## 三、证据与产物

### run 目录
一次巡检的全部证据容器。
> `patrol_logs/runs/<日期>/xunjian-<日期>-<序号>/`
> **共 132 次记录**:89 次老格式(2 层,7/17–7/22)+ 43 次新格式(3 层,7/23–7/26)

### manifest(清单)
saas 写的巡检参数与身份快照,**纯文本 key=value**。
> `manifest.txt`。录制侧对应 `manifest.json`(JSON,20 KB)

### sidecar(路线出生证明)
录制产出的、与路线 CSV 绑定的凭证。巡检启动时会校验。
> `route_link.json`,schema `go2.route_recording_link.v1`
> saas 只接受 `status` 为 `complete` 或 `complete_with_warnings`

### content-alias(内容别名)
**用 sha256 而非文件名来认定路线**。
> saas `:1249-1368` `route_recording_evidence`,注释原文:
> *never trust a filename or a sidecar hash alone*
>
> **这不是洁癖**:`routes/` 里 `xbf2.csv.horizontal.csv`(359 行)装的其实是
> **xbf8 的水平化结果**(与 `xbf8.csv.horizontal.csv` sha 完全相同),
> 而 `xbf2.csv` 本体是 1278 行。按文件名取用就会拿到错的路线。

### 黑盒录制(blackbox)
`scripts/route_recording_blackbox.py`(1761 行)编排的三阶段录制,产出全套证据。
> 目录名形如 `record-<时间>-<路线名>-blackbox-<pid>`

### 体检(experiment_audit)
每次巡检**停止时自动执行**的全链路诊断,出 JSON + 中文 Markdown。
> `scripts/go2_experiment_audit.py`(2807 行),saas `:2998` 调用
> 产物:`experiment_audit.json`(60 KB)+ `experiment_audit.md`

---

## 四、控制链

### trace wrapper(跟踪外壳)
实际执行的 follower。它**继承**基类控制器,在外面加就绪门禁、水平化、逐周期记录。
> `scripts/waypoint_follower_go2_2_trace.py`(1259 行),**不在 ROS 包内**
> `:34` `from go2_fastlio_patrol import waypoint_follower_go2_2 as base_module`
> `:148` `class TracedWaypointFollower(BaseFollower)` · `:977` `super().control_loop()`
>
> ⚠️ **同一条链上两种启动机制**:follower 走 `python3 <WS>/scripts/...`(saas `:2060`),
> safe 节点走 `ros2 run go2_fastlio_patrol unitree_safe_cmd_node`(saas `:2449`)

### 纯追踪(pure pursuit)
部署的控制算法。取路线上距当前位置 `lookahead_distance` 处的点为目标,
按航向角误差 `alpha` 出转向速度。
> `waypoint_follower_go2_2.py:255-313` `control_loop`
> `yaw_rate = clamp(k_yaw * alpha, ±max_yaw_rate)`;vx 按 |alpha| 三档

### cross_track(横向偏差)
狗到路线的垂直距离。**有两个互相矛盾的口径**,见 `10_open_questions.md`。
> follower 自算:`trace.py:82-130` `nearest_local_projection`,只写进 trace 文件
> audit 另算:拿 FAST-LIO 轨迹比 `route_runtime.csv`

### G2CM 包
运动指令的 UDP 载荷。**"G2CM" 魔数 + 版本 2 + 44 字节定长**。
> `cmd_vel_udp_sender.cpp:23-39` 结构体定义

# 一条线走完整个系统 —— 细到代码

> **先读这篇。** 01–11 是按主题切开的(时序、数据流、不变量…),
> 分开看容易变成"一块一块",连不起来。
> 这篇只有**一条因果链**:从"你想让狗走一条固定路线"出发,
> 每一步都是上一步逼出来的,每一步都指回具体 file:line。
>
> **基准**:`gogoguard_dog_go2_real_code` @ `8b3d789`(2026-08-01,已完成删除阶段)。
> 行号以该 commit 为准。删除前的原始基线见 01–11(基于 2026-07-27 整机快照)。

---

## 起点

> 你想让狗自己走一条固定路线。

下面每一节,都是被上一节逼出来的。

---

## ⓪ 线的起点不是巡检,是五条常驻循环

`systemd/` + `deploy/systemd_user/` 共 5 个 unit,开机各拉起一条**互不知情的循环**:

| unit | 拉起 | 干什么 |
|---|---|---|
| `go2-saas-command.service` | `go2_saas_agent.py command-loop` | 每 5 秒问云端要命令 |
| `go2-saas-video.service` | `video-loop` | 视频分段 |
| `go2-saas-outbox.service` | `outbox-loop` | 补传积压文件 |
| `go2-wired-ssh-rescue.service` | `go2_wired_ssh_rescue.sh` | 有线口保底 SSH |
| `deploy/systemd_user/go2-fastlio-base.service` | 底座 | 雷达 + FAST-LIO |

**巡检不是常驻的。** 它是 `command-loop` 收到一条命令后临时拉起的第六摊东西,
跑完就拆。

---

## ① 录制 —— 路线怎么变成 CSV

狗不知道路线在哪 → **得有人先带着走一遍**。

`src/go2_fastlio_patrol/go2_fastlio_patrol/route_recorder.py`,订阅 `/Odometry`,只做一件事:

```python
:23   declare_parameter('min_distance', 0.4)
:90   if dist >= self.min_distance:            # 离上一个记录点 ≥ 0.4 米
:94       self.writer.writerow([id, x, y, yaw, v])
```

**5 个字段:`id, x, y, yaw, v`。**

⚠️ 其中 `v` **不是实测速度**,是常数 `default_speed=0.20`(:24)直接写进去的。
这就是为什么跟线器**从不读 CSV 里的 v** —— 它读了也没意义。

**分支①在这里岔开**:录制与巡检**互斥**,靠 `.route_recording_enabled` 门文件挡。

---

## ② 定位 —— 位置从哪来

Livox MID-360 → CustomMsg → `src/FAST_LIO/src/laserMapping.cpp`(IESKF + ikd-Tree)→ 发 `/Odometry`。

**这是唯一的位置来源。全系统没有第二个位置源做交叉验证。**

配套探针:`check_livox_stream.py`(雷达有没有数)、`check_fastlio_freshness.py`(定位新不新)。

---

## ③ 扶正 —— 为什么必须有这一步

FAST-LIO 把"雷达的上"当成"世界的上",而**雷达斜装 34°**(实测 `tilt_removed_deg=34.324`)。
歪的世界里,平地被虚假拉高;在歪的系里记的路线,拿去走会拧。

`scripts/waypoint_follower_go2_2_trace.py` 里的 `horizontal_estimator`:

```
:214  horizontal_frame_samples = 15                        ← 攒够 15 个一致样本才锁
:223  horizontal_frame_max_source_disagreement_deg = 3.0   ← 两源差 > 3° 丢弃
:318  q_lidar_gravity_correction_xyzw                      ← 标定文件里的挂载修正
```

**两个 IMU 的全部用途就在这里:**
- `lidar_imu_callback`(:546)收雷达 IMU
- `body_yaw_callback`(:515)收狗身 IMU

各测一次重力,互相对答案。差 > 3° 丢弃,攒够 15 个才敢锁。
锁完**冻死**(frozen),整趟不再更新 —— 因为路线是在某个系里记的,系一变就对不上。

配套:`horizontal_frame.py`、`body_yaw_alignment.py`。

---

## ④ 路线对齐 —— 把路线转到狗身上

狗放下的位置不可能和录制起点完全一样。所以做 SE(2) 刚体变换,
**把整条路线平移+旋转到狗当前位置**。

`go2_saas_agent.py:2319` 是个**二选一**:

```python
if   relocalization["manualAnchor"]:       → manual_route_anchor.py    ← 默认走这条
elif relocalization["pcdRelocalization"]:  → route_relocalizer(C++)    ← ICP,默认不走
```

默认是 `manual_anchor`(`:3581  default="manual_anchor"`)。

产出链:
```
源路线 CSV
   ↓ manual_route_anchor.py (--max-translation 0.08 --max-yaw 0.08)
route_runtime.csv        ← 已平移旋转
   ↓ build_horizontal_route.py
route_horizontal.csv     ← 再扶正,跟线器读的是这份
```

### ⚠️ 这里藏着全系统最要命的结构性盲区

**路线是转过来迁就狗的。狗摆歪 1°,路线就跟着转 1°。**
于是跟得再准(实测 cross-track 0.0295 m),相对真实地面整条路都偏 1°。

`check_route_start_alignment.py`(204 行)本来是那道验收闸:

```python
:195  if distance > max_distance or abs(yaw_error) > max_yaw_error:
:196      print("ROUTE_START_NOT_ALIGNED ...")   → 非零退出 → 巡检整体回滚
```

**但它挡不住这件事,原因在执行顺序:**

```
saas:2772   route_prepare_cmd            ← manual_anchor 把路线转到狗身上
                  ↓ 产出 route_runtime.csv (:2326 --output-route)
saas:2777   check_route_start_alignment --route-file route_runtime.csv
                  ↑ 检查的就是刚被转过来的那份 (:1956 route_arg = route_runtime.csv)
```

**它量的是"manual_anchor 干活干成了没有",不是"狗摆得正不正"。**
路线已经被转到迁就狗了,这道闸**在结构上只可能通过**。

这不是概率问题,是代码顺序上的死结:`route_arg`(:1956)与 manual_anchor 的
`--output-route`(:2326)指向同一个文件。

**1° 的代价**:@50 m = 0.87 m,@100 m = 1.75 m,@436 m = 7.61 m。
**链路上没有任何一个环节能发现它。**

---

## ⑤ 跟线器基类 —— 331 行,四步

`src/go2_fastlio_patrol/go2_fastlio_patrol/waypoint_follower_go2_2.py`
50 ms 一拍(`:98 create_timer(0.05)`)。

### 第一步 · 找我在路上的哪 `:153 find_nearest_window()`
```python
start = max(0, nearest_index - 6)      # search_window = 6
end   = min(n-1, nearest_index + 6)    # 只在这 13 个点里找,不全局搜
```
`:177` **强制单调**:`nearest_index = min(new_i, nearest_index + 1)`
—— 一拍最多前进 1 格,防止跳点。
`:183` 偏离超过 `relocalize_distance` 才允许全局重搜。

### 第二步 · 找前面 0.6 米那个点 `:195 compute_lookahead_index()`
```python
acc = 0.0
while True:
    seg = hypot(p1.x-p0.x, p1.y-p0.y)   # 沿路线逐段累加弧长
    acc += seg
    if acc >= lookahead_distance: return i
```
**不是画圆求交点,是沿路线累加弧长。** 所以弯道上直线距离会短于 0.6 m。

### 第三步 · 算转多快 `:279`
```python
alpha    = normalize_angle(atan2(dy,dx) - current_yaw)
yaw_rate = clamp(k_yaw * alpha, ±max_yaw_rate)
```

### 第四步 · 三档定速 `:283`
```
|alpha| > turn_in_place_angle → vx = 0          原地转
|alpha| > slow_down_angle     → vx = 速度 × 0.5  减半
否则                          → vx = 速度       正常
```

### 兜底 `:292`
`stuck_time` 秒索引没进展 → 全局重找最近点。

**整个算法:四步 + 一个兜底。没有 PID,没有轨迹优化,没有预测。**

### 运行时真正生效的参数(`go2_saas_agent.py:2482`)
```
-p k_yaw:=0.900          -p max_yaw_rate:=0.450
-p lookahead_distance:=0.600
-p reach_distance:=0.400 -p goal_distance:=0.250
-p search_window:=6
-p turn_in_place_angle:=1.000   (57.3°)
-p slow_down_angle:=0.500       (28.6°)
-p stuck_time:=3.000  -p relocalize_distance:=1.500
```

⚠️ **除 `v_base`/`max_vx`(= speed)和 `loop_mode` 外,全部写死。**
云端在 `:1685–1790` 读进来算了一大堆(`k_yaw` 默认 1.20、
`tracking_lookahead_distance` 默认 0.50、`max_yaw_rate` 默认 0.60),
**算完就扔,没有一个进命令行**。

⚠️ `reach_distance` 基类只在日志里打印(`:105`),**从不参与计算** —— 死参数。

---

## ⑥ 外壳 —— 1259 行,只干两件事

`scripts/waypoint_follower_go2_2_trace.py` **继承**上面那个类
(manifest 记为 `controller_trace_wrapper_policy=subclass_calls_base_control_with_selected_heading_feedback_after_interlock_release`),
只覆盖 `odom_callback`(:724)和 `control_loop`(:889)。

### 第一件事:在算法前面加一道闸 `:896`
```python
motion_requested = os.path.isfile('.motion_enabled')     # 门文件在不在

release_fresh = (odom_ready
    and 0.0   <= 收包龄   <= 0.35     # 数据不能超过 350 ms 没来
    and -0.10 <= 时间戳龄 <= 0.50     # 时间戳不能太旧
    and body_yaw 新鲜
    and 扶正已锁)

if motion_enabled:
    super().control_loop()          # ← 放行,才调那 331 行
else:
    self.pub.publish(Twist())       # ← 不放行,直接发零
```

**关键在 `else` 那一行**(:981):不放行时它发零,而**不调基类** ——
这样基类的卡住计时器和索引推进不会在静止时空转。**这是有意为之,改造时不能顺手"简化"掉。**

### 第二件事:每一拍写一条 JSON `:1065`
一条 trace 记 **40+ 字段**:位姿(扶正后 / 原始 LIO / body yaw 三份)、
最近点、目标点、投影、有符号横向误差、alpha、实发速度、各级延迟。

一趟 6828 条 → `follower_control_trace.jsonl`。
**电脑端做分析看的就是这个文件。**

---

## ⑦ 安全层 —— 唯一会否决算法的地方

`src/go2_fastlio_patrol/go2_fastlio_patrol/unitree_safe_cmd_node.py`
收 `/patrol_cmd` → 吐 `/cmd_vel`,订阅 `/cloud_registered_body`。

### 前向 ROI(一个盒子) `:55`
```
x: 0.35 ~ 1.20 m    身前 35 cm 到 1.2 m
y: -0.45 ~ +0.45 m  左右各 45 cm
z: 0.25 ~ 0.90 m    离地 25 cm 到 90 cm(避开地面与天花板)
```
```
min_stop_points = 12    盒子里 ≥12 个点就停
stop_frames  = 1        1 帧就停(反应快)
clear_frames = 5        连续 5 帧干净才恢复(恢复慢)
point_skip   = 2        每 2 点取 1,省算力
max_cloud_process_rate = 20.0
```

**"停"是纯二值:要么全速,要么零。没有减速,没有绕行。**

### 限速(最后一道) `:48`
```
max_vx = 0.50   max_vy = 0.15   max_yaw_rate = 0.45
cmd_timeout   = 0.5 s   跟线器 0.5 秒不说话 → 零
cloud_timeout = 1.0 s   点云 1 秒不来 → 零
```

⚠️ 本文件 `:13` 从 `patrol_control.py` import 三个函数:
`limit_planar_command`(:189 限速)、`point_in_lateral_motion_roi`(:324 ROI 判定)、
`stream_receive_age`(:393)。
**`patrol_control.py` 是运动链核心,不是工具库** —— 曾差点被当"未部署"归档,那会直接把狗弄瘫。

---

## ⑧ 出 ROS,进 SDK —— 两个搬运工 + 一把独立刹车

### 发送端 `src/go2_cmd_vel_bridge/src/cmd_vel_udp_sender.cpp`
把 `/cmd_vel` 打成 **44 字节**发到 `127.0.0.1:5005`:
```cpp
magic = 0x4732434d  // "G2CM"      4 字节
version = 2                        2
packet_size = 44                   2
sequence                           8   ← 序号,查丢包用
send_steady_ns                     8   ← 单调钟
send_system_ns                     8   ← 墙钟
vx, vy, vyaw                      12   ← 真正的载荷
```
**44 字节里只有 12 字节是命令,32 字节是给事后追查用的。**

### 接收端 `go2_sdk2_udp_receiver.cpp`
三重校验(magic + version + size,`:195-198`)→ 调宇树 SDK:
```cpp
:91   sport_client.StandUp();
:93   sport_client.BalanceStand();
:219  sport_client.Move(last_vx, last_vy, last_vyaw);   ← 循环里唯一让狗动的一行
:241  sport_client.StopMove();
```

**整条链走到最后,就是 `Move()` 这一行。**

### 独立刹车 `go2_sdk2_motion_probe`(C++)
**绕过整条 ROS 链、直接对宇树 SDK 喊停**。三处调用,全是 `stop`:
```
go2_saas_agent.py:2960              停止巡检时
localization_session_guard.py:80    熔断时
go2_base_health_watchdog.py:50      健康看门狗触发时
```
**前面那条 `Move()` 链要是卡死,靠的就是它。**

---

## ⑨ 编排 —— 57 步

`go2_saas_agent.py` 的 `commands = [...]`(:2590–2924),57 条,
每条带**探针**,失败即整体回滚(`startup_cleanup`)。

关键顺序:
```
建目录 → 存证据哈希 → 起 UDP 接收 → 起 UDP 发送 → 起安全层 → 起跟线器
  → :2767 等 FAST-LIO fresh (失败 exit 44)
  → :2772 路线对齐 (失败 exit 47)
  → :2777 起点对齐检查 (失败即回滚)
  → :2788 拉起 base_health_watchdog(若未在跑)
  → :2796 起 rosbag
  → :2512 等日志出现 FOLLOWER_HORIZONTAL_FRAME_READY
  → 起观测三件套
  → 最后 touch .motion_enabled
```

**最后一步那个空文件是整条链的总闸。**
前 56 步做完狗还站着不动;这个文件一出现,第 ⑥ 步的闸打开,狗才走。

---

## ⑩ 云端 —— 为什么只能狗主动

狗背 4G,**没有固定公网 IP,平台连不进来**。
所以只能狗每 5 秒 POST 一次 `/robot/heartbeat`(`:3494`):
**请求体带状态上去,响应体带命令下来。一个接口两个方向。**

四条 loop 并行:`heartbeat-loop`(:3278)、`command-loop`(:3342)、
`video-loop`(:3388)、`outbox-loop`(:3474)。

---

## ⑪ 观测 —— 电脑端分析的数据从这来

同时四份记录:

| 谁 | 频率 | 产物 |
|---|---|---|
| follower trace(外壳) | 20 Hz | `follower_control_trace.jsonl` 6828 条 × 40+ 字段 |
| `go2_experiment_telemetry.py` | 1 Hz | CPU / 温度 / 功耗 |
| `patrol_performance_monitor.py` | 1 Hz | 各进程存活 |
| `ros2 bag record` | 原速 | 8 个话题原始数据 |

停止时 `go2_experiment_audit.py` 合成体检报告。

---

## ⑫ 监督 —— 两条独立的看门狗线

### `localization_session_guard.py`(241 行,巡检期间临时)
盯 FAST-LIO 的 pid:
```python
:185  expected.get("pid")             ← 记下开始时的 pid
:232  abort_operation(args, reason)   ← pid 变了 = 定位重启过 = 坐标系换了
```
一旦发现,`:73-77` 直接 `pkill` 掉整条运动链,并往 manifest 写
`abort_reason=localization_session_changed`。

**理由**:FAST-LIO 一重启原点就变,之前对齐的路线全部作废,继续走会撞。
**宁可停,不冒险。**

### `go2_base_health_watchdog.py`(常驻,不随巡检退出)
`saas:2788` 起,**若未在跑则拉起**。盯底座健康,触发时同样调 `motion_probe stop`。

**这两条 + 两把刹车,是全系统唯一有冗余的地方。**
定位、路线、朝向,全是单点。

---

## ⑬ 证据链

`go2_experiment_snapshot.py` 在停止时把关键文件哈希存档。
路线**认哈希不认文件名**(带内容别名回退)。

---

# 主线之外的五条分支

```
主线 ── 巡检 ①→⑬
  │
  ├─ 分支1  录制链    route_recorder + route_recording_blackbox
  │                  ⚠️ 与巡检互斥,靠 .route_recording_enabled 挡
  │
  ├─ 分支2  相机链    go2-saas-video.service(常驻)+ 8 个 sh/py
  │                  巡检时靠 patrol_video.active 门文件启停
  │                  ⚠️ 狗上实跑 unitree_builtin,不是 z1pro(config/camera.env)
  │
  ├─ 分支3  上传链    go2-saas-outbox.service(常驻)
  │                  4G 断了攒着,通了补传
  │
  ├─ 分支4  救援链    go2-wired-ssh-rescue.service
  │
  └─ 分支5  离线建图  go2_loop_backend/ 18 个 py
                     ⚠️ 巡检期间完全不跑
```

**五条与主线的关系各不相同**:1 互斥,2/3/4 并行常驻,5 压根不参与。

---

# 代码量沿这条线的分布

| 环节 | 行数 | 干的事 |
|---|---:|---|
| ① 录制 | 1,894 | 每 0.4 m 写一行 CSV |
| ③④ 扶正+对齐 | 1,885 | 锁一个旋转矩阵 + 转一条路线 |
| ⑤ 算法 | **331** | 四步 + 一个兜底 |
| ⑥ 外壳 | **1,259** | 一道闸 + 每拍写 40 字段 ← **算法的 3.8 倍** ★ |
| ⑦ 安全 | 570 | 一个盒子 + 数点 |
| ⑧ 搬运 | 492 | 打包/解包 44 字节 |
| ⑨⑩ 编排+云端 | **3,634** | 一个文件,六件事 ★ |
| ⑪ 观测 | **5,370** | 比整条运动链(3,125)还大 72% ★ |
| ⑫ 监督 | 241 + watchdog | |
| 分支5 离线建图 | **4,228** | 巡检根本不跑 ★ |

**四颗星 = 第 3 步重构要动的四处。**
这条线本身是干净的 —— 每一环都是上一环逼出来的,没有一环是多余的。
**问题不在这条线,在代码没按这条线摆。**

---

# 这次走线挖出的三件事(均已 file:line 实证)

**① 起点对齐检查是自证的。** 它在路线被旋转之后才量,量的是自己刚做完的事。
代码顺序上是死结(`:2772` → `:2777`,`route_arg` = manual_anchor 的输出),不是概率问题。

**② nav2 / AMCL / slam_toolbox 五份配置零引用。** 前人把配置放好了,没有一行代码读它们。

**③ 全系统只有"停"这件事有冗余。** 两条看门狗 + 两把刹车。
定位单点(只有 FAST-LIO)、路线单点(只有 manual_anchor)、朝向单点(锁完就冻)。

---

**下一篇 → `13_file_accounting.md`:213 个团队文件逐个归位,
证明这条线能装下全部代码 —— 装不下的,逐个点名。**

# 03 · 数据流

> 一帧点云进来,到狗腿动起来,中间经过什么。
> 重点是**坐标变换链**和**限幅链** —— 34° 倾角就在前者里被处理掉。

---

## 全景

```
Livox MID-360 硬件
   │  UDP
   ▼
livox_ros_driver2_node ────────────────────── 团队补丁 7 处
   │  ① 丢弃畸形帧: frame_span > 2×interval 或 points < 5000
   │  ② 队列积压告警 >1024   ③ 每秒 LIVOX_STREAM_HEALTH   ④ LIVOX_PUBLISH_TIMING
   ├──► /livox/lidar (CustomMsg)   QoS depth=2  best_effort
   └──► /livox/imu                 QoS depth=400 reliable
        │
        ▼
   fastlio_mapping (laserMapping.cpp) ─────── 团队补丁 8 处
        │  健康门禁: 连续坏帧≥3 / 点数<5000 / span∉[0.08,0.13] / 可恢复间隔≤1.00
        │  IESKF(esekfom) + ikd-Tree,生产参数见下
        ├──► /Odometry              ← 控制链用这个
        ├──► /cloud_registered      ← submap_builder(未运行)
        ├──► /cloud_registered_body ← safe node 用这个做避障
        ├──► /Laser_map /cloud_effected /path   ← yaml 里三个 false,实际不发布
        └──► /dev/shm/go2_fastlio_latest_odom.txt  (写 .tmp 再 rename,原子)
                  └─► 6 个读者:base_bringup.sh / ensure_base_ready.sh /
                      check_fastlio_freshness.py / manual_route_anchor.py /
                      localization_session_guard.py / check_route_start_alignment.py
```

---

## 一、坐标变换链 ★ 这是整个系统最关键的一段

### 1.1 三个坐标系

| 系 | 定义 | z 轴指向 |
|---|---|---|
| **雷达/IMU 系** | Livox 内部 | 雷达自身的"上" |
| **FAST-LIO map 系** | 原点 = 进程启动那一刻狗的位置 | **= 雷达 z 轴,与重力偏 34.32°** |
| **ground 系** | 重力水平系 | 真正的"上" |

### 1.2 为什么 map 系是歪的

生产 yaml 里:
```yaml
extrinsic_est_en: false                    # 不在线估计外参
extrinsic_T: [-0.011, -0.02329, 0.04412]   # 雷达↔雷达内置IMU 的小平移
extrinsic_R: [1,0,0, 0,1,0, 0,0,1]         # 单位阵
```

**这个单位阵是正确的** —— 它描述"雷达↔雷达内置 IMU"的关系,同一设备内确实对齐。
audit 的 `mounting_note`(`:1695-1699`)也点明:
> 这些值描述的是雷达/雷达内置 IMU 的配置变换,**不独立测量物理的雷达→狗身安装角度**。

**但雷达是斜着装在狗身上的**,而 FAST-LIO 不知道 ——
于是它把"雷达的上"当成了"世界的上",整个地图跟着歪 34.32°。

**危害的量化**(同一片 1,085,459 点的云):
| | z 最小 | z 最大 | z 跨度 |
|---|---:|---:|---:|
| 未旋正 | -5.99 | 181.16 | **187.15 m** |
| 旋正后 | -4.19 | 32.18 | **36.37 m** |
→ **虚假拉伸 5.1 倍**。这也是 `pcd_to_nav2_map` 那种"按离地高度分类障碍"在未旋正图上无法工作的原因。

### 1.3 怎么旋正:双 IMU 求重力,不改任何一方参数

`horizontal_frame.py:243-311` `add_sample()`,启动时执行,**锁定后冻结**:

```
① 两个源各自量重力
   lidar_up = normalize(雷达IMU加速度)
   body_up  = normalize(狗身IMU加速度)

② 各自套一层标定常量(外面乘四元数,不改设备参数)
   corrected_lidar_up  = q_lidar_gravity_correction × lidar_up
   sensor_up_from_body = q_sensor_from_body        × body_up
   实测值(xbf.leveling.json):
     q_lidar_gravity_correction = [0.01929, -0.00888, 0.01241, 0.99970]  ← 接近单位
     q_sensor_from_body         = [0.02284, -0.28391, 0.00429, 0.95857]  ← 复算 33.10°

③ 各自用 FAST-LIO 当前姿态搬到 map 系
   map_up_from_lidar = q_map_from_sensor × corrected_lidar_up
   map_up_from_body  = q_map_from_sensor × sensor_up_from_body

④ 互相裁判
   disagreement = angle_between(两者) > 3.0° → 丢弃该样本,rejected_source_samples++

⑤ 融合 = 角平分线
   fused = normalize(map_up_from_lidar + map_up_from_body)   ← 两源等权,无滤波

⑥⑦ 攒够 15 个 → mean_up = normalize(全部相加)
    spread = max(每个样本与 mean_up 夹角) > 1.5° → 不锁定,继续攒

⑧ 锁定并冻结
   q_ground_from_map = quaternion_from_two_vectors(mean_up, (0,0,1))
   ready = True  →  add_sample 首行 `if self.ready: return False`,永不更新
```

**三个拒绝计数分开记**:`rejected_invalid_samples` / `rejected_motion_samples` / `rejected_source_samples`
→ 能区分"数据坏了"/"狗在动"/"两源打架"。**实测三者均为 0。**

**自证**(`xbf.leveling.json.proof`):
```
rotation_determinant            = 1.0
rotation_orthogonality_max_error= 4.85e-17
maximum_pair_distance_error_m   = 1.14e-13   (512 个点对验证刚体性)
residual_gravity_tilt_deg       = 0.0
```

### 1.4 变换在哪里施加 ★

**不是改控制算法,而是改数据入口。** `trace.py:661-722` `horizontal_odometry()`:

```python
def horizontal_odometry(self, message):
    q_map_from_sensor = self.odometry_quaternion(message)
    position_map      = self.odometry_position(message)
    position_ground   = transform_position(position_map)          # 位置转 ground
    q_ground_from_body= transform_body_orientation(q_map_from_sensor)  # 姿态转 ground
    body_yaw = quaternion_yaw(q_ground_from_body)

    if not self.horizontal_route_anchored:                        # 只做一次
        self.route, self.horizontal_anchor_rotation = align_route_to_pose(
            self.canonical_route, position_ground[0], position_ground[1], body_yaw)
        self.horizontal_route_anchored = True

    transformed = copy.deepcopy(message)
    transformed.pose.pose.position    = position_ground
    transformed.pose.pose.orientation = q_ground_from_body
    return transformed                                            # 喂给基类
```

**基类 `waypoint_follower_go2_2.py`(330 行)完全不知道有倾斜这回事** ——
它拿到的 odom 已经是扶正的,路线也已经在同一个系里。

> **这是本系统最好的一处设计**:第三方(FAST-LIO/Livox)零改动,
> 修正全部发生在它们的输出之后,且下游无感。

---

## 二、路线的四次变换

| 阶段 | 产物 | 变换 | 实测 |
|---|---|---|---|
| 录制 | `routes/xbf9.csv` | 原始 FAST-LIO map 系绝对坐标 | 1278 行,首点 (6.736, -6.469) |
| 巡检启动 · 锚定 | `route_runtime.csv` | `manual_route_anchor.py` 的 **SE(2) 刚体变换**,起点归零 | 首点 (-0.002, -0.038) |
| 巡检启动 · 水平化 | `route_horizontal.csv` | 施加 `q_ground_from_map` | 首点 (0.000, 0.000) |
| 运行时 · 再对齐 | 内存中的 `self.route` | `align_route_to_pose` 绕起点旋转,对齐狗当前朝向 | **`route_rotation_deg = -15.67°`** |

⚠️ **runtime 与 horizontal 是两个不同的系**:实测逐点距离**中位 67.0 m / 最大 133.3 m**
(路径总长 577.0 m)。这是 34° 旋转的必然结果,不是错误。

⚠️ **但 audit 评估用的是 `route_runtime.csv`,follower 实际跑的是 `route_horizontal.csv`** ——
这导致两个口径的横向偏差差 176 倍,详见 `10_open_questions.md`。

---

## 三、控制链

```
/Odometry (map 系)
   │
   ▼ trace.py:661 horizontal_odometry()  → ground 系
   │
   ▼ trace.py:977 super().control_loop()
   │
   ├─ 最近点搜索(search_window=6,含 stuck 时全局回捞 find_nearest_global)
   ├─ 前视点: 沿路线累计弧长 ≥ lookahead_distance(0.600) 处
   ├─ alpha = 目标方向 − 当前朝向
   ├─ yaw_rate = clamp(k_yaw(0.900) × alpha, ±max_yaw_rate(0.450))
   └─ vx 三档:
        |alpha| > turn_in_place_angle(1.000)  → vx = 0        原地转
        |alpha| > slow_down_angle  (0.500)    → vx = min(v_base×0.5, max_vx)
        否则                                   → vx = min(v_base,     max_vx)
   │
   ▼ /patrol_cmd (Twist)
```

**门控**:每周期先查 `.motion_enabled`(`:896`),不满足则发零速(`:981`)。

---

## 四、安全链

```
/patrol_cmd ──┐
              ├─► unitree_safe_cmd_node.py
/cloud_registered_body ──┘
   │
   ├─ ROI 盒过滤(roi_x/y/z_min/max),Python 逐点 struct.unpack_from,point_skip=2
   │    ⚠️ **该点云未扶正**:`/cloud_registered_body` 的 frame_id 虽标 `base_link`,
   │       但 `publish_frame_body()` 只做雷达→IMU 变换,而 `extrinsic_R` 是单位阵
   │       ⇒ 它实际是【雷达系,带 33–34° 倾角】,ROI 盒在真实机身系里是斜切片
   │       详见 `06_invariants.md` II-6
   ├─ 障碍判定: stop_frames=1(立即停) / clear_frames=5(连续 5 帧无障碍才恢复)
   ├─ 优先级(publish_safe_cycle:404-414):
   │     cmd_timeout(0.5s) > cloud_timeout(1.0s) > 障碍
   └─ 限幅: max_vx / **max_vy=0.000** / max_yaw_rate
   │
   ▼ output_cmd_topic ⇒ /cmd_vel (Twist)     ← saas :2454 传 /cmd_vel
      (若该参数为空串则改走 /api/sport/request 宇树原生,见 07_design_intent.md)
```

**实测**:该层把非零命令改成零 **69 次**(配对命令 4148 条)。

---

## 五、传输链

```
/cmd_vel
   ▼ cmd_vel_udp_sender.cpp:103 订阅(话题名硬编码 "/cmd_vel")
   │  :144  pkt.vx   = limit(linear.x,  max_vx_)
   │  :146  pkt.vy   = unitree_vy_sign_ × limit(linear.y, max_vy_)
   │  :147  pkt.vyaw = limit(angular.z, max_vyaw_)
   ▼ G2CM 包(魔数"G2CM" + 版本 2 + 44 字节定长,:23-39)
   │  sendto → 127.0.0.1:5005
   ▼ go2_sdk2_udp_receiver  ← 直接跑 build/go2_cmd_vel_bridge/ 下的二进制,非 ROS 节点
   │  :214-216 硬编码 clamp  vx ±0.5f / vy ±0.10f / vyaw ±0.5f
   │  :219     sport_client.Move(vx, vy, vyaw)
   ▼ 宇树 SDK → 狗腿
```

---

## 六、限幅链 ★ 同一物理量的多道关卡

**任何一个速度分量都要过三道**,最紧的那道才是生效值:

| 分量 | ① safe node | ② sender | ③ receiver(硬编码) | **实际生效** |
|---|---|---|---|---|
| **vx** | `max_vx:=<speed>` | `max_vx:=<speed>` | ±0.5f | 取决于云端 speed(实测 0.5) |
| **vy** | **0.000** | **0.000** | ±0.10f | **0 —— 狗不做横移** |
| **vyaw** | `max_yaw_rate:=0.450` | `max_vyaw:=0.450` | ±0.5f | **0.450** |

> ⚠️ 分析任何一个量时**必须走完整条关卡链**。
> 只看末端会得出错误结论 —— 例如只看 receiver 的 ±0.10f,
> 会误以为横向速度上限是 0.10,而实际上游两道已经把它设成了 0。
> manifest 的 `command_vy=0.000` 是这一点的直接证据。

---

## 七、观测链(不参与控制,只留证据)

```
follower ──► follower_control_trace.jsonl   6828 条 control(含 cross_track,仅此一处)
telemetry ─► experiment_telemetry.jsonl     15 MB,20 Hz,profile=patrol/recording
monitor ───► performance_monitor.log        1.9 MB,1 Hz(CPU/温度/功耗/唤醒延迟)
rosbag ────► rosbag_0.db3                   34 MB,8 话题,**不含点云**
FAST-LIO ──► FAST_LIO_INPUT/OUTPUT_TIMING   进程内计时
livox ─────► LIVOX_STREAM_HEALTH / LIVOX_PUBLISH_TIMING  每秒
停止时 ────► experiment_audit.json + .md    全链路体检
```

⚠️ **rosbag 不录点云**(`observer_policy.raw_pointcloud_subscriber = false`),
点云证据依赖 `pointcloud_evidence = "Livox and FAST-LIO in-process timing logs"`。
→ 后果:**黑盒无法离线重放点云匹配过程**。

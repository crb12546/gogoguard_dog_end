# 01 · 基础层 Bringup(雷达 → FAST-LIO → 定位/点云)

> 原则同 00:结论只对代码负责,引用标 `文件:行`。
> 涉及文件:`scripts/base_bringup.sh`、`scripts/env_common.sh`、`scripts/check_livox_stream.py`、
> `scripts/check_fastlio_freshness.py`、`src/FAST_LIO/launch/mapping.launch.py`、
> `src/FAST_LIO/config/go2_mid360s.yaml`、`src/livox_ros_driver2/config/MID360s_config.json`。

## 一句话
基础层负责把**激光雷达**拉起来、喂给 **FAST-LIO** 做激光惯性 SLAM,产出两条巡检命脉:
- **`/Odometry`** —— 狗的实时位姿(定位),巡检跟随器靠它知道"我现在在哪"。
- **`/cloud_registered_body`** —— 机体系点云,安全节点靠它做前方急停。
并且在放行巡检前,用两道**健康门**卡住:Livox 流时序 + FAST-LIO 输出新鲜度。

---

## 二、雷达怎么接入(硬件 → 话题)

**网络**(`MID360s_config.json` + `base_bringup.sh:19-56`):
- 雷达 IP `192.168.1.161`,Orin 主机 IP `192.168.1.5/24`,走有线网卡(`env_common.sh:11-28` 按 MAC `4c:bb:47:ab:e4:c2` 找网卡,找不到回退 `eth1`)。
- UDP 端口:点云 `56300→56301`、IMU `56400→56401`、控制 `56100→56101` 等。
- 雷达外参在驱动配置里全为 0(`lidar_configs[].extrinsic_parameter`),`pcl_data_type=1`(笛卡尔点)。

**驱动**(`base_bringup.sh:168-196`):`ros2 launch livox_ros_driver2 msg_MID360s_launch.py`
- 产出 `/livox/lidar`(Livox `CustomMsg` 自定义点云)+ `/livox/imu`(`sensor_msgs/Imu`)。
- **5 次重试**,每次:停旧进程 → 起驱动 → `wait_livox_topics`(45s 内等两话题出现,日志出现 `bind failed / init fail` 立即判失败)→ `validate_livox_stream`。

**DDS**:全程 CycloneDDS(`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`),`env_common.sh:30-44` 动态生成一份把 DDS 绑定到有线网卡的 XML,避免多网卡串扰。

---

## 三、Livox 流健康门(`check_livox_stream.py`)
两种模式,base_bringup 用的是 **driver-log 模式**(`base_bringup.sh:156-166`,8s、至少 3 份报告):
- 解析驱动日志里的 `LIVOX_STREAM_HEALTH ...` 行,逐条卡:
  `frame_hz∈[min,13]`(MID360 ~10Hz)、`imu_hz∈[100,300]`、`span_ms∈[80,130]`(≈一帧)、
  `points≥min`、`lidar_imu_offset_ms≤100`、`queue_depth≤512`;
  日志出现 `Dropping malformed Livox frame` 或 `point packet backlog` 直接判失败。
- 另有 **live 模式**(订阅 `CustomMsg`/`Imu`):校验交付率≥7Hz、`clock_ratio`(传感器时钟/墙钟)∈[0.7,1.3]、点数、帧间隔 [0.04,0.25]s、雷达/IMU 时钟偏差≤0.5s。
> 作用:**开跑前先证明雷达流是实时、时序正常、点数够的**,否则 SLAM 会漂/崩。

---

## 四、FAST-LIO 起法与关键配置(`mapping.launch.py` + `go2_mid360s.yaml`)
- launch 起 `fast_lio` 包的 `fastlio_mapping`,base_bringup 传 `config_file:=go2_mid360s.yaml rviz:=false`(`base_bringup.sh:220`)。
- **`go2_mid360s.yaml` 里为"巡检"做的关键取舍**:
  - `lidar_qos_depth: 2`(注释明写:巡检要**最新一帧**,不要回放深 FIFO 积压)、`imu_qos_depth: 400`。
  - `preprocess`:`lidar_type:1`(Livox)、`scan_line:4`、`blind:0.5`(0.5m 内盲区)、`scan_rate:10`。
  - `mapping`:`fov_degree:360`、`det_range:100`、IMU-雷达外参 `extrinsic_T:[-0.011,-0.02329,0.04412]`、`extrinsic_est_en:false`(不在线估外参)、`max_iteration:3`、体素 `filter_size_*:0.5`、`cube_side_length:1000`。
  - **`publish` 段(直接决定输出哪些话题)**:
    - `scan_bodyframe_pub_en: true` → 产出 **`/cloud_registered_body`**(机体系点云,喂安全节点急停)。
    - `path_en:false` / `map_en:false` / `dense_publish_en:false` / `effect_map_en:false` → 巡检不消费 `/path`、`/Laser_map`,关掉省序列化。
  - **`pcd_save: pcd_save_en:false`** → **FAST-LIO 本身在巡检配置下不存 pcd**。⚠️ 这条重要:pcd 地图**不是**由 FAST-LIO 自带保存产生的,而是另有离线流程(见 06)。

---

## 五、FAST-LIO 新鲜度门(`check_fastlio_freshness.py`,巡检放行的最后一道闸)
- 读快照文件 `/dev/shm/go2_fastlio_latest_odom.txt`(取其中 `stamp=` 字段)。
  ⚠️ 该快照由 FAST-LIO(疑似被改过的 `laserMapping.cpp`)写入 `/dev/shm`,**写入点待 06/建图章节坐实**。
- 逻辑(`base_bringup.sh:84-106` 调用,默认 `--frames 10`):要求连续 **10 帧**满足:
  - 单帧"年龄"`age=墙钟-stamp ∈ [-100, 250]ms`(足够实时);
  - 相邻帧 `dt ∈ [50,150]ms`(≈10Hz,均匀不跳);
  - 一旦某帧陈旧/跳变,计数**清零重来**;总超时 120s。
> 作用:**证明 FAST-LIO 在稳定实时地出位姿**,才允许开跑,避免"定位还没热身就巡逻"。

---

## 六、启动总时序(`base_bringup.sh` 主流程)
```
ensure_lidar_network(配 192.168.1.5/24 + 到雷达的路由)
  → 清理旧的 fastlio/livox 进程
  → start_livox_driver(5次重试, 每次: 等话题 + 流校验)
  → 起 FAST-LIO(go2_mid360s.yaml)
  → wait_topic /Odometry (90s) + wait_topic /cloud_registered_body (90s)
  → wait_fastlio_freshness(连续10帧新鲜)
  → 就绪; 阻塞 wait 两个子进程
退出时 trap: 停 fastlio + 停 livox
```

## 七、雷达一共被用在哪(回答"用雷达了么、怎么用")
| 用途 | 数据 | 去向 |
|---|---|---|
| **定位** | 雷达点 + IMU → FAST-LIO 融合 | `/Odometry`(狗在 FAST-LIO 局部坐标系里的位姿)→ 跟随器 |
| **急停** | FAST-LIO 输出的机体系点云 | `/cloud_registered_body` → `unitree_safe_cmd_node` 前方/侧向 ROI 判障 |
| **建图/重定位** | 点云累积 | 离线出 pcd(06)、巡检前 route×map 对齐(07) |

FAST-LIO 输出的是**局部里程计坐标系**(以开机静止初始化点为原点),因此 README 反复强调"固定起点、固定朝向"——因为路线 CSV 的坐标系必须和当前 FAST-LIO 坐标系一致(除非走生产链的 route_relocalizer 重定位,见 07)。

## 八、留待坐实
- `/dev/shm/go2_fastlio_latest_odom.txt` 的**写入方**(哪段代码、写入频率、字段格式)——需读被改过的 `laserMapping.cpp`。
- Livox `CustomMsg` → FAST-LIO 内部预处理(`preprocess.cpp`)的点筛选细节(第三方,按需)。

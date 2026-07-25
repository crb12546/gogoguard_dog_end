# 01 · 基础层 Bringup(雷达 → FAST-LIO → 定位/点云)

> 原则同 00:结论只对代码负责,引用标 `文件:行`。
> 工作区根:`orin_go2_fastlio_ws/`(下文短路径 `scripts/`、`src/FAST_LIO/`、`src/livox_ros_driver2/` 均相对此根)。
> 涉及文件:`scripts/base_bringup.sh`、`scripts/env_common.sh`、`scripts/check_livox_stream.py`、
> `scripts/check_fastlio_freshness.py`、`scripts/ensure_base_ready.sh`、`scripts/go2_saas_agent.py`、
> `src/FAST_LIO/launch/mapping.launch.py`、`src/FAST_LIO/config/go2_mid360s.yaml`、
> `src/FAST_LIO/src/laserMapping.cpp`、`src/livox_ros_driver2/config/MID360s_config.json`、
> `src/livox_ros_driver2/launch_ROS2/msg_MID360s_launch.py`、
> `src/go2_fastlio_patrol/go2_fastlio_patrol/unitree_safe_cmd_node.py`。

## 一句话
基础层负责把**激光雷达**拉起来、喂给 **FAST-LIO** 做激光惯性 SLAM,产出两条巡检命脉:
- **`/Odometry`** —— 狗的实时位姿(定位),巡检跟随器靠它知道"我现在在哪"。
- **`/cloud_registered_body`** —— 机体系点云,安全节点靠它做前方/侧向 ROI 急停。
并且在放行巡检前,用两道**健康门**卡住:Livox 流时序 + FAST-LIO 输出新鲜度。

---

## 核验状态
本轮已把本文逐条对 `orin_go2_fastlio_ws/` **磁盘源码逐行核过**(见文末「核验台账」),无一处硬性数值错误;仅修了 4 处结构性/精度瑕疵(见下)。源标签约定:

- **【默认 code F:L】** = 仓库磁盘源码字面(本轮已核),但**无狗上二进制逐字节对照**,取其 argparse/`declare` 默认值。
- **【生产 saas F:L】** = 生产启动串/生产脚本实际传参(启动时覆盖了代码默认,狗上真正生效的那个)。
- **【狗上 dog:证据】** = 狗上运行副本 / 运行 manifest / 抓包坐实(最强证据)。
- **【README对照】** / **【推断-未验】** = 来自 README 描述或逻辑推断,未在源码字面直证。

**本轮修正的 4 处**:
1. §四 结构性归类:`max_iteration/filter_size_surf/filter_size_map/cube_side_length` 是 yaml **顶层 `ros__parameters`(:5-8)**,不在 `mapping:` 子块(:30-41)内 —— 数值全对,仅原文层级标签误导。
2. §三 `--duration 8s`、§四 `extrinsic_est_en/path_en/dense_publish_en=false` 均属**「生产值覆盖了代码默认」**,现两值并列并点明狗上生效哪个。
3. §五「最后一道闸」定位:base_bringup 的新鲜度门在**「基础层起来」**时跑;start_patrol **放行前**那道闸其实是 `ensure_base_ready.sh --fresh-only`(同一 checker、同参),已更正。
4. §五/§八 `/dev/shm` 快照写入方**从「疑似/待坐实」升级为「已证实」**:确由 `laserMapping.cpp::publish_odometry` 写入(自研改造)。

**仍无法直证的局限**:
- 狗上**没有**这批脚本(`base_bringup.sh`/`env_common.sh`/`check_*.py`/`ensure_base_ready.sh`/yaml/json/launch/`unitree_safe_cmd_node.py`)的源码副本 —— 只能对**仓库**核,标 **【无狗上对照】**。
- **有**狗上运行副本者:`laserMapping.cpp`(`remote_source/`,狗真跑版)、`waypoint_follower_go2_2.py`(`remote_source/`);运行 manifest 印证了 QoS。
- `laserMapping.cpp` **仓库版 sha `5fec8282…` ≠ 狗上版 `e4cd05cb…`**(`analysis/xunjian_20260725_shutdown_capture/previous_boot/remote_source/`)。基础层相关行为两版**结构一致**(均在 `publish_odometry` 写快照、均发 `/Odometry`+`/cloud_registered_body`、publish 门控与 declare 默认相同),但仓库源并非狗上二进制的逐字节对应源。

**文件 × 狗上对照状态**:

| 文件 | 狗上状态 |
|---|---|
| `scripts/base_bringup.sh`、`env_common.sh`、`check_livox_stream.py`、`ensure_base_ready.sh`、`mapping.launch.py`、`MID360s_config.json`、`msg_MID360s_launch.py`、`unitree_safe_cmd_node.py` | 【无狗上对照】(仅对仓库核) |
| `scripts/check_fastlio_freshness.py` | 【无狗上对照】;但 base_bringup 与 ensure_base_ready **两处同参调用**它 |
| `src/FAST_LIO/config/go2_mid360s.yaml` | 【无狗上对照】(文件本身);但运行 manifest(`runs/xunjian-20260725-06/07`)**运行期印证** `lidar_qos_depth=2`/`imu_qos_depth=400` |
| `src/FAST_LIO/src/laserMapping.cpp` | **repo≠dog(sha 验)**:`5fec8282…` ≠ `e4cd05cb…`;基础层相关行为两版结构一致 |
| `remote_source/waypoint_follower_go2_2.py` | 【狗上 dog】狗真跑版(330 行 `class WaypointFollower`) |

---

## 二、雷达怎么接入(硬件 → 话题)

**网络**(`MID360s_config.json` + `base_bringup.sh:19-56`)【无狗上对照】:
- 雷达 IP `192.168.1.161`,Orin 主机 IP `192.168.1.5/24`,走有线网卡。【默认 code `MID360s_config.json:26`(ip)/`:15`(host_ip);`base_bringup.sh:21`(ip_cidr 192.168.1.5/24),`:33`/`:49`(路由到 .161)】
- 找网卡:`env_common.sh:11-28` 按 MAC `4c:bb:47:ab:e4:c2` 找,找不到回退 `eth1`。【默认 code `env_common.sh:11`(GO2_WIRED_MAC),`:13-26`(find_iface_by_mac),`:28`(||printf eth1)】
- UDP 端口:点云 `56300→56301`、IMU `56400→56401`、控制 `56100→56101`(=`cmd_data_port`)等。【默认 code `MID360s_config.json:9/18`(point),`:10/19`(imu),`:7/16`(cmd)】另有 push `56200/56201`、log `56500/56501`(此处"等"涵盖)。
- 雷达外参在驱动配置里全为 0(`lidar_configs[].extrinsic_parameter` 的 roll/pitch/yaw/x/y/z),`pcl_data_type=1`(Livox 笛卡尔点)。【默认 code `MID360s_config.json:29-36`(全 0),`:27`(pcl_data_type 1)】

**驱动**(`base_bringup.sh:168-196`):`ros2 launch livox_ros_driver2 msg_MID360s_launch.py`【无狗上对照】
- 产出 `/livox/lidar`(Livox `CustomMsg` 自定义点云)+ `/livox/imu`(`sensor_msgs/Imu`)。【默认 code `base_bringup.sh:177`;`msg_MID360s_launch.py:8`(xfer_format=1=CustomMsg),`:19`(用 MID360s_config.json)】话题名在驱动节点内**硬编码**,launch 未显式列出,但 yaml/校验脚本均以 `/livox/lidar`、`/livox/imu` 消费,一致(`check_livox_stream.py:13/15`)。
- **5 次重试**,每次:停旧进程 → 起驱动 → `wait_livox_topics`(45s 内等两话题出现)→ `validate_livox_stream`。【默认 code `base_bringup.sh:171`(for 1..5),`:173`(stop),`:177`(start),`:183`(wait 45 && validate)】
- 失败模式:日志出现三串之一即立即判失败 —— `bind failed` / `Init lds lidar fail` / `Failed to init livox lidar sdk`(原文"bind failed / init fail"是宽松转述,`init fail` 覆盖后两串)。【默认 code `base_bringup.sh:141`】

**DDS**:全程 CycloneDDS(`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`),`env_common.sh:30-44` 动态生成一份把 DDS 绑定到有线网卡的 XML(用**网卡名**而非地址),避免多网卡串扰。【默认 code `env_common.sh:9`(RMW),`:30-42`(heredoc 写 NetworkInterfaceAddress=GO2_WIRED_IF),`:43`(CYCLONEDDS_URI)】

---

## 三、Livox 流健康门(`check_livox_stream.py`)【无狗上对照】
两种模式,base_bringup 用的是 **driver-log 模式**(`base_bringup.sh:156-166`):
- 生产实际传 `--duration 8.0 --min-reports 3`(外层再套 `timeout 12`)。**默认≠生产**:argparse 默认 `--duration 4.0`、`--min-reports 3` —— 狗上 base_bringup **显式传 8.0s**,故实际生效 **8s / 至少 3 份报告**;min-reports 两值恰同为 3。【生产 saas `base_bringup.sh:158-159`(8.0/3)】覆盖【默认 code `check_livox_stream.py:208`(duration 4.0),`:207`(min-reports 3)】
- 解析驱动日志里的 `LIVOX_STREAM_HEALTH ...` 行,逐条卡:
  `frame_hz∈[min,13]`、`imu_hz∈[100,300]`、`span_ms∈[80,130]`(≈一帧)、
  `points≥min`、`lidar_imu_offset_ms≤100`、`queue_depth≤512`;其中 **`min`=frame_hz≥7.0Hz(MID360 ~10Hz)、points≥5000**。【默认 code `check_livox_stream.py:62/64/66/68/70/72`(六条阈值);`:213`(min_delivery_hz 7.0),`:210`(min_points 5000)】
- 日志出现 `Dropping malformed Livox frame` 或 `Livox point packet backlog`(原文略写"point packet backlog")直接判失败。【默认 code `check_livox_stream.py:33-34`/`:35-36`】
- ⚠️ **另有 live 模式**(订阅 `CustomMsg`/`Imu`):交付率≥7Hz、`clock_ratio`(传感器时钟/墙钟)∈[0.7,1.3]、点数、帧间隔 [0.04,0.25]s、雷达/IMU 时钟偏差≤0.5s。**但生产 base_bringup 只走 driver-log 模式**,live 模式这些阈值在狗上**不执行**——属"存在但不跑"的旁路,勿当运行时门。【默认 code `check_livox_stream.py:104-105`(订阅),`:189`(delivery),`:191`(clock_ratio),`:126`(stamp_dt),`:197`(offset),均为 argparse 默认】
> 作用:**开跑前先证明雷达流是实时、时序正常、点数够的**,否则 SLAM 会漂/崩。

---

## 四、FAST-LIO 起法与关键配置(`mapping.launch.py` + `go2_mid360s.yaml`)【无狗上对照(QoS 项有运行 manifest 印证)】
- launch 起 `fast_lio` 包的 `fastlio_mapping`;launch **默认 config 是 `mid360.yaml`**(`mapping.launch.py:34`),base_bringup **覆盖**为 `go2_mid360s.yaml rviz:=false`。【默认 code `mapping.launch.py:47-48`(package/executable)】【生产 saas `base_bringup.sh:220`(config_file 覆盖)】
- **`go2_mid360s.yaml` 里为"巡检"做的关键取舍**:
  - `lidar_qos_depth: 2`(注释明写:巡检要**最新一帧**,不回放深 FIFO 积压)、`imu_qos_depth: 400`。**运行期强证据**:manifest 直接印证。【狗上 dog `manifest.txt:30` `fast_lio_input_qos=lidar_2_best_effort_imu_400_reliable`(runs/xunjian-20260725-06/07)】+【默认 code `go2_mid360s.yaml:17/:18`】
  - `preprocess`:`lidar_type:1`(Livox)、`scan_line:4`、`blind:0.5`(0.5m 内盲区)、`scan_rate:10`。【默认 code `go2_mid360s.yaml:24-28`】
  - **雷达/建图核心参数**(⚠️ **归类更正**):`fov_degree:360`、`det_range:100`、IMU-雷达外参 `extrinsic_T:[-0.011,-0.02329,0.04412]` 三项确在 `mapping:` 块内(`:35/:36/:38`);但 **`max_iteration:3`、体素 `filter_size_surf/map:0.5`、`cube_side_length:1000` 是 yaml 顶层 `ros__parameters`(`:5-8`),不在 `mapping:` 子块**(原文把这四项列在 `mapping:` 下有误,数值全对)。【默认 code `go2_mid360s.yaml:35/36/38`(mapping 内),`:5-8`(顶层)】
  - `extrinsic_est_en:false`(不在线估外参)。**默认≠生产**:FAST-LIO 节点 `declare` 默认 `true`(`laserMapping.cpp:1005/1047`,全局初值 `:77` 亦 true),巡检 yaml 在 launch 时**覆盖为 false**;狗上加载 yaml,实际 **false**。【生产 `go2_mid360s.yaml:37`(false)】覆盖【默认 code `laserMapping.cpp:1005/1047/77`(true)】
  - **`publish` 段(直接决定输出哪些话题)**:
    - `scan_bodyframe_pub_en: true` → 产出 **`/cloud_registered_body`**(机体系点云,喂安全节点急停)。需 `scan_publish_en`(`:47` true)与 `scan_bodyframe_pub_en`(`:49` true)**同时成立**才发布。【默认 code `go2_mid360s.yaml:47/49`;`laserMapping.cpp:1119`(建 publisher),`:1290`(门控 `if scan_pub_en && scan_body_pub_en`)】
    - `path_en:false` / `map_en:false` / `dense_publish_en:false` / `effect_map_en:false` → 巡检不消费 `/path`、`/Laser_map`,关掉省序列化。**默认≠生产**:`path_en`、`dense_publish_en` 节点 `declare` 默认 `true`(`laserMapping.cpp:1011/1015`)被 yaml 覆盖为 false;`map_en/effect_map_en` 节点默认已是 false(`:974/975`)。`/Laser_map` **仅在退出时**(`map_pub_en`)可能发,`map_en:false` 下**从不发**。【默认 code `go2_mid360s.yaml:44/46/48/45`;`laserMapping.cpp:1288/1291/1328`(逐帧 `:1292` 已注释)】
  - **`pcd_save: pcd_save_en:false`** → **FAST-LIO 本身在巡检配置下不存 pcd**。⚠️ pcd 地图**不是**由 FAST-LIO 自带保存产生的,而是另有离线流程(见 06)。【默认 code `go2_mid360s.yaml:52`;`laserMapping.cpp:77`(全局 false)】

---

## 五、FAST-LIO 新鲜度门(`check_fastlio_freshness.py`)【无狗上对照(两处同参调用)】
> ⚠️ **定位更正**:本门有两处调用、**同一 checker、同参**(frames10 / age[-100,250] / dt[50,150] / 超时120s):
> - base_bringup 主流程调用(`:229` `wait_fastlio_freshness`,函数体 `:84-106`)—— 在**「基础层起来」**时把关;
> - **真正在 start_patrol 放行前的"最后一道闸"**是 `ensure_base_ready.sh --fresh-only`。【生产 saas `go2_saas_agent.py:2323-2324`(`bash … --fresh-only`,失败 `FASTLIO_NOT_FRESH_AFTER_STARTUP` exit 44)→ `ensure_base_ready.sh:139-142`(`wait_fastlio_fresh 120`)】
> 因二者同参,阈值结论不变;仅"哪个脚本是最后一闸"由原文的 base_bringup 更正为 ensure_base_ready。

- 读快照文件 `/dev/shm/go2_fastlio_latest_odom.txt`(取其中 `stamp=` 字段),默认 `--frames 10`。【默认 code `check_fastlio_freshness.py:33`(路径),`:14-17`(解析 stamp= token),`:35`(frames 10);`base_bringup.sh:8`(FASTLIO_SNAPSHOT 同路径),`:10`(FRESH_FRAMES 10)】code 默认 == base_bringup == ensure_base_ready 三处一致。
- ✅ **写入方(已证实,升级)**:该快照由 `laserMapping.cpp::publish_odometry` **每次发里程计时写入** `/dev/shm/…tmp`(内容 `stamp=lidar_end_time` + `x/y/z/qx/qy/qz/qw`),再 `rename` 成 `.txt`。这是**自研改造**(原版 FAST-LIO 无此写盘),原文"疑似被改过/待 06 坐实"可去。【狗上 dog `remote_source/laserMapping.cpp:738-745`(狗真跑版)】+【默认 code 仓库 `laserMapping.cpp:746-764`,`stamp=` 在 `:753`】
- 逻辑:要求连续 **10 帧**满足:
  - 单帧"年龄"`age=墙钟-stamp ∈ [-100, 250]ms`(足够实时);
  - 相邻帧 `dt ∈ [50,150]ms`(≈10Hz,均匀不跳);
  - 总超时 120s。【默认 code `check_fastlio_freshness.py:83`(age),`:95`(age 判定),`:88-92`(dt 判定),`:36-41`(默认 250/-100/50/150/120);`base_bringup.sh:11-16`(传相同值)】code 默认 == 生产,无分叉。
  - 一旦某帧陈旧/跳变,计数**基本清零重来**。细微差:**陈旧**(非 fresh)→归 **0**;**跳变**(fresh 但 dt 越界)→重置为 **1**(本帧仍计入)。【推断-未验(逻辑读出)`check_fastlio_freshness.py:95-102`】
> 作用:**证明 FAST-LIO 在稳定实时地出位姿**,才允许开跑,避免"定位还没热身就巡逻"。

---

## 六、启动总时序(`base_bringup.sh` 主流程)【无狗上对照】
```
ensure_lidar_network(配 192.168.1.5/24 + 到雷达的路由)         # :206
  → 清理旧的 fastlio/livox 进程                                  # :209-210
  → start_livox_driver(5次重试, 每次: 等话题45s + 流校验8s)     # :214
  → 起 FAST-LIO(go2_mid360s.yaml)                              # :220
  → wait_topic /Odometry (90s) + wait_topic /cloud_registered_body (90s)  # :226-227
  → wait_fastlio_freshness(连续10帧新鲜)                       # :229
  → 就绪; 阻塞 wait 两个子进程(FASTLIO_PID/LIVOX_PID)          # :238
退出时 trap cleanup_children: 停 fastlio + 停 livox              # :204 / :198-202
```
【默认 code `base_bringup.sh:206/209-210/214/220/226-227/229/238`;trap `:204`/`:198-202`】时序与主流程完全一致。

## 七、雷达一共被用在哪(回答"用雷达了么、怎么用")
| 用途 | 数据 | 去向 | 源标签 |
|---|---|---|---|
| **定位** | 雷达点 + IMU → FAST-LIO 融合 | `/Odometry`(狗在 FAST-LIO 局部坐标系里的位姿)→ 跟随器 | 【狗上 dog `remote_source/waypoint_follower_go2_2.py:30`(declare odom_topic 默认 `/Odometry`),`:90-93`(订阅 Odometry),`:124`(odom_callback)】 |
| **急停** | FAST-LIO 输出的机体系点云 | `/cloud_registered_body` → `unitree_safe_cmd_node` 前方/侧向 ROI 判障 | 【生产 saas `go2_saas_agent.py:2051`(`ros2 run go2_fastlio_patrol unitree_safe_cmd_node`);`unitree_safe_cmd_node.py:46`(默认 `/cloud_registered_body`),`:59-60`(前向 ROI),`:67-70`(侧向 ROI)】 |
| **建图/重定位** | 点云累积 | 离线出 pcd(06)、巡检前 route×map 对齐(07) | 见 06/07 |

> ⚠️ 急停端**生产确用 `unitree_safe_cmd_node`**(本文所指),**非**同目录的 `unitree_go_safe_cmd_node`;前向 + 侧向双 ROI 均存在。ROI 具体阈值属 02/03 章,此处只验消费链成立。

FAST-LIO 输出的是**局部里程计坐标系**(frame_id `odom` / child `base_link`),因此 README 反复强调"固定起点、固定朝向"——因为路线 CSV 的坐标系必须和当前 FAST-LIO 坐标系一致(除非走生产链的 `route_relocalizer` 重定位,见 07)。【默认 code `laserMapping.cpp:741-742`(odom/base_link)】+【README对照:固定起点/朝向】+【推断-未验:"开机静止初始化点为原点"=FAST-LIO 特性;`route_relocalizer` 存在于 `go2_map_manager/src/route_relocalizer.cpp`,重定位细节留待 07】

## 八、留待坐实
- ~~`/dev/shm/go2_fastlio_latest_odom.txt` 的写入方~~ —— **已坐实**(见 §五):`laserMapping.cpp::publish_odometry`,每次发里程计时写(仓库 `:746-764` / 狗上 `:738-745`),自研改造。
- Livox `CustomMsg` → FAST-LIO 内部预处理(`preprocess.cpp`)的点筛选细节(第三方,按需)。【未核】
- **可靠性提醒**:本文前向引用的 `laserMapping.cpp`,仓库版 sha `5fec8282…` ≠ 狗上真跑版 `e4cd05cb…`;基础层相关行为两版结构一致,但仓库源并非狗上二进制的逐字节对应源(重定位/建图章节引用时须记此分叉)。

---

## 核验台账
> claim → 证据(`file:line`)→ 判定 / 源标签。CONFIRMED=源码字面直证;DEFAULT_VS_PROD=默认与生产分叉(已补生产值);inferred=逻辑推断。

| # | 断言 | 证据(`file:line`) | 判定 |
|---|---|---|---|
| 1 | 雷达 IP 192.168.1.161 / 主机 192.168.1.5/24 | `MID360s_config.json:26,15`;`base_bringup.sh:21,33,49` | CONFIRMED · code |
| 2 | 按 MAC 4c:bb:47:ab:e4:c2 找网卡,回退 eth1 | `env_common.sh:11,13-26,28` | CONFIRMED · code |
| 3 | UDP 点云56300→56301/IMU56400→56401/控制56100→56101 | `MID360s_config.json:9/18,10/19,7/16` | CONFIRMED · code |
| 4 | 外参全 0,pcl_data_type=1(笛卡尔) | `MID360s_config.json:29-36,27` | CONFIRMED · code |
| 5 | 驱动产出 /livox/lidar(CustomMsg)+/livox/imu(Imu) | `base_bringup.sh:177`;`msg_MID360s_launch.py:8,19`;`check_livox_stream.py:13,15` | CONFIRMED · code |
| 6 | 5 次重试;三串日志立即判失败 | `base_bringup.sh:171,173,177,183,141` | CONFIRMED · code |
| 7 | 全程 CycloneDDS,绑有线网卡 XML | `env_common.sh:9,30-42,43` | CONFIRMED · code |
| 8 | driver-log:8s、至少 3 份报告 | `base_bringup.sh:158-159`(8.0/3);`check_livox_stream.py:208`(默认4.0),`:207`(默认3) | **DEFAULT_VS_PROD** · 生产 8s 生效 |
| 9 | 阈值 frame[min,13]/imu[100,300]/span[80,130]/pts≥min/off≤100/q≤512;min=7.0Hz&5000 | `check_livox_stream.py:62,64,66,68,70,72,213,210` | CONFIRMED · code |
| 10 | 日志 Dropping malformed / Livox point packet backlog 判失败 | `check_livox_stream.py:33-34,35-36` | CONFIRMED · code |
| 11 | live 模式:≥7Hz/clock_ratio[0.7,1.3]/帧间隔[0.04,0.25]/偏差≤0.5s | `check_livox_stream.py:104-105,189,191,126,197` | CONFIRMED · code(**生产不跑此模式**) |
| 12 | launch fastlio_mapping,base 传 go2_mid360s.yaml rviz:=false | `mapping.launch.py:47-48,34`;`base_bringup.sh:220` | CONFIRMED · code(默认 mid360.yaml 被覆盖) |
| 13 | lidar_qos_depth:2 / imu_qos_depth:400 | `go2_mid360s.yaml:17,18`;`manifest.txt:30`(lidar_2…imu_400) | CONFIRMED · **dog 运行印证** |
| 14 | preprocess lidar_type1/scan_line4/blind0.5/scan_rate10 | `go2_mid360s.yaml:24-28` | CONFIRMED · code |
| 15 | fov360/det_range100/extrinsic_T…;max_iter3/filter0.5/cube1000 | `go2_mid360s.yaml:35,36,38`(mapping)/`5-8`(**顶层**) | CONFIRMED · code(**归类更正**) |
| 16 | extrinsic_est_en:false | `go2_mid360s.yaml:37`;`laserMapping.cpp:1005,1047,77`(默认 true) | **DEFAULT_VS_PROD** · 生产 false 生效 |
| 17 | scan_bodyframe_pub_en:true → /cloud_registered_body | `go2_mid360s.yaml:47,49`;`laserMapping.cpp:1119,1290` | CONFIRMED · code |
| 18 | path/map/dense/effect_map=false → 不发 /path、/Laser_map | `go2_mid360s.yaml:44,46,48,45`;`laserMapping.cpp:1011,1015,974,975,1288,1291,1328` | CONFIRMED · code(path/dense 覆盖默认 true) |
| 19 | pcd_save_en:false → FAST-LIO 不自存 pcd | `go2_mid360s.yaml:52`;`laserMapping.cpp:77` | CONFIRMED · code |
| 20 | 读快照 /dev/shm/…_odom.txt,默认 --frames 10 | `check_fastlio_freshness.py:33,14-17,35`;`base_bringup.sh:8,10` | CONFIRMED · code |
| 21 | 快照由 laserMapping.cpp::publish_odometry 写入 | 仓库 `laserMapping.cpp:746-764`(stamp `:753`);狗上 `:738-745` | CONFIRMED · **dog(已升级坐实)** |
| 22 | 连续10帧:age[-100,250]ms/dt[50,150]ms/超时120s | `check_fastlio_freshness.py:83,95,88-92,36-41`;`base_bringup.sh:11-16` | CONFIRMED · code |
| 23 | 陈旧/跳变计数清零重来 | `check_fastlio_freshness.py:95-102` | inferred(陈旧→0;跳变→1) |
| 24 | 启动总时序 + 退出 trap 停 fastlio/livox | `base_bringup.sh:206,209-210,214,220,226-227,229,238,204,198-202` | CONFIRMED · code |
| 25 | /Odometry → 跟随器 | `remote_source/waypoint_follower_go2_2.py:30,90-93,124` | CONFIRMED · **dog** |
| 26 | /cloud_registered_body → unitree_safe_cmd_node 双 ROI | `go2_saas_agent.py:2051`;`unitree_safe_cmd_node.py:46,59-60,67-70` | CONFIRMED · **生产 saas** |
| 27 | FAST-LIO 局部里程计系;route_relocalizer 见 07 | `laserMapping.cpp:741-742`;`route_relocalizer.cpp` 存在 | CONFIRMED · code + README/推断 |
| — | start_patrol 放行最后一闸 = ensure_base_ready --fresh-only | `go2_saas_agent.py:2323-2324`→`ensure_base_ready.sh:139-142` | CONFIRMED · **生产 saas(§五定位更正)** |

# 00 · 系统总览

> **本文档原则**:所有结论基于当前仓库真实代码(仓库 `HEAD = 6ec2382`;源码与 `b5b87fc · Initial import` 一致 —— 其后 `fe0b43a` / `6ec2382` 两次均为 docs-only 提交,未动源码)。
> 引用一律标 `文件:行`,并加**源标签**(见下节图例)。与 README / 旧 `.md` 文档冲突处,**以代码为准**。
> 旧文档(README、HANDOFF、各 *_NOTES.md)只作线索,数值/状态描述均已被证实不可信。

---

## 〇、核验状态(本轮做了什么、还有什么没法验)

**本轮已做**:对磁盘上仓库源码**逐条核对** `文件:行`、默认值、话题名、entry_points、包内可执行数、节点名,并交叉比对狗端实证:
- 狗端运行 manifest:`analysis/xunjian_20260725_shutdown_capture/runs/xunjian-20260725-06,07/manifest.txt`(2026-07-25 两次实跑)。
- 狗上实跑代码副本:`analysis/.../previous_boot/remote_source/`(4 个文件:`laserMapping.cpp` / `lddc.cpp` / `lds.cpp` / `waypoint_follower_go2_2.py`)。

**头号更正**:原文把 `scripts/waypoint_follower_go2_2_trace.py` 定为"狗上真跑的生产真身"——**这是过度断言**。狗端 06/07 manifest 证实狗上实跑的是 `waypoint_follower_go2_2` **基类**(无 trace 包装),详见 §二 / §四.1。

**源标签图例**(每个可证伪数值/断言都带):
- 【默认 `file:line`】= 源码 `declare_parameter` 的代码默认值。
- 【生产·仓库saas `file:line`】= 仓库 `go2_saas_agent.py` 装配时的 `-p` 启动串值。⚠️ **狗上 saas 是另一版本**(manifest schema 不同,见下),故此值**不必然等于狗上实跑**。
- 【狗上 dog:证据】= 狗端 manifest / `remote_source` 实证(**最高可信度**,优先信它)。
- 【README `file:line`】= README 的数值(通常已过时)。
- 【推断-未验】= 合理推断,但仓库源码中无直接依据。

**狗上对照状态图例**(每个文件/子系统都标,不许默认等同狗上):
- 🟰 **repo==dog**(sha 一致,已验)
- ❌ **repo≠dog**(sha 不一致,已验,**以狗版为准**)
- ❔ **无狗上对照**(大多数自研文件如此,仅能核仓库源码,**不代表狗上一致**)

**仍无法验证**:
- 狗上 `go2_saas_agent.py` 的具体内容(❔ 无直接副本,只能从 manifest schema 反推它≠仓库 saas)。
- 狗上大量自研节点(unitree_safe_cmd_node / route_relocalizer / base_bringup 等 ~117 个文件)的实跑版本(❔ 无对照,结论均基于仓库源码)。
- `Ubuntu 20.04 + ROS2 Foxy`(【推断-未验】:Go2 EDU + Orin 常见组合,但源码无直接佐证)。
- 云端名 `llyj`(【推断-未验】:saas docstring 自称 "GoGoGuard SaaS" `go2_saas_agent.py:2`,代码中未见 `llyj`)。

---

## 一、这是什么系统

宇树 **Go2 EDU** 四足机器狗的**固定路线巡检(xunjian)**程序,运行在狗背上的 **Orin**,工程根 `/home/unitree/go2_fastlio_ws`【狗上 dog:manifest `route=/home/unitree/go2_fastlio_ws/src/...` 确证】。系统为 **Ubuntu 20.04 + ROS2 Foxy**【推断-未验:源码无佐证】。

- **感知/定位**:Livox **MID-360** 激光雷达 → **FAST-LIO** 激光惯性里程计(SLAM),输出机体位姿 `/Odometry` 和机体系点云 `/cloud_registered_body`【默认 `FAST_LIO/src/laserMapping.cpp:1122`(`/cloud_registered_body`)、`:1119`(`/Odometry`)】。❌ **repo≠dog**:仓库 `laserMapping.cpp`(1414 行,sha `5fec8282`)≠ 狗上 `remote_source`(1395 行,sha `e4cd05cb`);但两版均发这两个话题(狗版 `:1091/:1094`),**本文所依赖的话题结论两版一致**。
- **巡检方式**:人工遥控录一条路线(x,y,yaw 的 CSV)→ 之后自动沿线巡逻(纯跟踪)。
- **云端**:`go2_saas_agent.py` 接 SaaS 云端,收命令、回传视频与结果。命令词表属实【默认 `go2_saas_agent.py:50` `GOTO_COMMANDS`(含 `move`)、`:51` `START_PATROL_COMMANDS`(含 `start_patrol`)、`:52` `STOP_PATROL_COMMANDS`】。⚠️ **精度更正**:停止命令实际 token 为 `stop_patrol`(`:52`),**无裸 `stop`**;云端名 `llyj` 为文档命名,代码 docstring 自称 "GoGoGuard SaaS"(`:2`)【推断-未验】。❔ **无狗上对照**:狗上 saas 为另一版本(见 §二说明)。

## 二、两条巡检链(理解全局的钥匙)

⚠️ **测试链 ≠ 生产链**——两者用的是不同的跟随器文件、不同的运动输出路径。

> **关于"生产链"的重要前提**:下图基于**仓库版** `go2_saas_agent.py` 的装配逻辑。但狗端 06/07 manifest 证实**狗上 saas 是另一版本**:狗 manifest 用 `controller_executable=waypoint_follower_go2_2` + `controller_reference_sha256`;而仓库 saas 写出的 manifest 用 `controller_executable=waypoint_follower_go2_2_trace.py` + `controller_source_sha256` + `controller_trace_wrapper_sha256`(`go2_saas_agent.py:1830-1832`)。**schema 不同 ⇒ 狗上 saas ≠ 仓库 saas**。故下图的 trace 包装是**仓库装配**,**狗上实跑的是基类、无 trace**(见 §四.1)。

### 🟢 生产链(云端 `start_patrol` 触发;装配见【生产·仓库saas `go2_saas_agent.py:1475 start_patrol_command`】)
```
SaaS云端 ──start_patrol+route──▶ saas command-loop ──下载 route.csv / 找同名 .pcd 地图──┐
（docstring: GoGoGuard SaaS）                                                          ▼
   route_relocalizer            (route.csv × 地图.pcd × /Odometry, ICP 粗+精对齐) → route_runtime.csv
   [go2_map_manager]             【生产·仓库saas :1989 ros2 run … route_relocalizer;out_route_file:=route_runtime.csv :1990/:1660】
                                 【默认 route_relocalizer.cpp:445-458 run_icp,:481-488 coarse,:862-872 refined】
                                 （"relocalized.csv" 仅概念标签,真实产物名是 route_runtime.csv）        │
                                                                                      ▼
   ┌ 仓库 saas 装配:waypoint_follower_go2_2_trace.py(TracedWaypointFollower 子类,包装基类)
   │   【生产·仓库saas :1743 trace 路径,:2061-2073 follower_cmd 启 follower_trace_script】
   └ 狗上 06/07 实跑:waypoint_follower_go2_2 基类(class WaypointFollower,330 行)  ★狗上真身★
       【狗上 dog:manifest controller_executable=waypoint_follower_go2_2 +
        controller_reference_sha256=d205a596(=remote_source 330 行 WaypointFollower)】—— 无 trace 包装
                                 (读 route_runtime.csv + /Odometry, 纯跟踪)                │  Twist → /patrol_cmd
                                                                                      ▼
   unitree_safe_cmd_node        (限速 + 前方/侧向 ROI 点云急停; 订阅 /cloud_registered_body)
   [go2_fastlio_patrol]          【生产·仓库saas :2055 pointcloud_topic:=/cloud_registered_body;默认 unitree_safe_cmd_node.py:174-179 订阅,:220 limit_planar_command 限速,:184 lateral swept ROI】
        ● 二选一(非双发):设了 output_cmd_topic:=/cmd_vel ⇒ 只发 Twist → /cmd_vel(走 UDP/SDK2),不发 sport Request
          【默认 unitree_safe_cmd_node.py:252-259 publish_move if/else;生产·仓库saas :2056 output_cmd_topic:=/cmd_vel】
                    │
                    └─ Twist → /cmd_vel ─▶ cmd_vel_udp_sender ─UDP:127.0.0.1:5005─▶ go2_sdk2_udp_receiver ─▶ Go2 SDK2(驱动腿)
                       【生产·仓库saas :2045-2049 target_ip:=127.0.0.1 target_port:=5005;:2022-2025 receiver 收 5005】
   旁路: patrol_performance_monitor / go2_experiment_telemetry / snapshot / rosbag record / localization_session_guard
        【生产·仓库saas :2026 / :2029 / :2035 / :1787 / :2080】
```
> ⚠️ **原图已更正**:旧图在此处写"sport-API 与 UDP **疑似双发**,哪条真驱动待深挖"——**不成立**。`publish_move` 是 `if twist_pub is not None: 发 Twist / else: 发 Request` 的**二选一**(`unitree_safe_cmd_node.py:252-259`)。生产设了 `output_cmd_topic:=/cmd_vel`,故**只走 Twist/UDP**;`self.pub`(Request)虽在 `:164` 恒创建,但生产**从不 publish**。此结论与 §六①一致。

### 🔵 测试链(README §10 / `scripts/run_roomtest7_readme_safe_patrol.sh`,手动跑)
```
waypoint_follower.py ─Twist /patrol_cmd─▶ unitree_safe_cmd_node ─Request─▶ /api/sport/request ─▶ Go2 sport API
（无重定位、无 UDP 桥、无遥测;跟随器是另一个文件 waypoint_follower.py,
  safe 节点 output_cmd_topic 默认空 → 走 publish_move else 分支发 Request,不发 /cmd_vel）
```
【默认 `run_roomtest7_readme_safe_patrol.sh:35` "chain: waypoint_follower -> unitree_safe_cmd_node -> /api/sport/request",`:79` 启 waypoint_follower 未传 output_cmd_topic;`unitree_safe_cmd_node.py:48` output_cmd_topic 默认 `''`,`:259` 空则发 Request】 ❔ 测试链跟随器/脚本均**无狗上对照**。

### ⚙️ 基础层(两条链共用;`scripts/base_bringup.sh`)
```
配雷达网(eth1 192.168.1.5/24 → 雷达 192.168.1.161)
  → Livox MID360 驱动(5 次重试+流校验) → /livox/lidar, /livox/imu
  → FAST-LIO(mapping.launch.py, config=go2_mid360s.yaml) → /Odometry, /cloud_registered_body
  → "10 帧连续新鲜" 门槛(check_fastlio_freshness.py)才算 ready
```
【默认 `base_bringup.sh:20-21`(eth1 192.168.1.5/24)、`:33`(192.168.1.161)、`:171 for attempt in 1 2 3 4 5`、`:136`(/livox/lidar+/livox/imu)、`:220 mapping.launch.py config_file:=go2_mid360s.yaml`;`check_fastlio_freshness.py:35 --frames 默认 10`】。"10 帧"既是代码默认也是实际调用值【默认+生产 `base_bringup.sh:10/93` 传 `FASTLIO_FRESH_FRAMES=10`,除非 env `GO2_FASTLIO_FRESH_FRAMES` 覆盖】。❔ base_bringup / check_fastlio_freshness **无狗上对照**;但 Livox 驱动 `lddc.cpp`/`lds.cpp` 🟰 **repo==dog**(sha `b5811eaf` / `8345dfde` 一致)。

## 三、节点 / 包清单(自研为主)

> **狗上状态列说明**:🟰=sha 一致 / ❌=sha 不一致(狗版为准) / ❔=无狗上对照(仅核仓库)。

| 包 | 文件 / 节点 | 角色 | 状态 | 狗上对照 |
|---|---|---|---|---|
| **go2_fastlio_patrol** (py) | `waypoint_follower.py` | 测试/README 跟随器(node 名 "waypoint_follower",`:56`) | 活·测试链 | ❔ |
| | `waypoint_follower_go2_2.py` | 生产跟随器**家族**;仓库版是 `class WaypointFollowerGo22`(1043 行,`:67`)。⚠️ **狗上实跑的是同名文件的另一版本**:`class WaypointFollower`(330 行) | 活·生产(家族) | ❌ repo≠dog(仓库 sha `009cb25b` ≠ 狗 sha `d205a596`) |
| | `waypoint_follower_old.py` | 旧跟随器(node 名 "waypoint_follower",`:27`) | **死**(不在 entry_points,`setup.py:22-29`) | ❔ |
| | `unitree_safe_cmd_node.py` | 安全过滤 + **二选一**输出(Twist 或 sport Request,非双发) | 活·两链都用 | ❔ |
| | `unitree_cmd_node.py` | 无点云急停,仅 `limit_planar_command` 限幅,直发 sport Request(`:8/:45/:80`) | 存疑(在 entry_points,但生产链未启它) | ❔ |
| | `unitree_go_safe_cmd_node.py` | 发 `SportModeCmd`(另一种控制接口,`:10/:110`) | 变体·用途存疑 | ❔ |
| | `route_recorder.py` | 录制路线 CSV | 活·录制时 | ❔ |
| | `patrol_control.py` / `go2_course_control.py` | 控制库?(各自带 test:`test/test_patrol_control.py`、`test_go2_course_control.py`) | 待定 | ❔ |
| | `route_quality.py` | 路线质量评估工具 | 工具 | ❔ |
| **scripts/** | `waypoint_follower_go2_2_trace.py` | **仓库 saas 装配的 trace 包装器**(`TracedWaypointFollower`)。⚠️ **非狗上真跑**:狗 06/07 跑基类、无 trace。且它 `:25 getattr(base_module,"WaypointFollower")`,`:28-30` 缺失即抛 `BASE_FOLLOWER_CLASS_MISSING` —— 仓库基类只有 `WaypointFollowerGo22`,**故此包装器对仓库自身会崩**,只能对狗上 330 行 `WaypointFollower` 生效 | 仓库装配·**狗上未跑** | ❔(无狗上副本;manifest 06/07 未跑 trace) |
| **go2_cmd_vel_bridge** (C++) | `cmd_vel_udp_sender` / `go2_sdk2_udp_receiver` / `go2_sdk2_motion_probe` | `/cmd_vel`→UDP→SDK2 运动桥(3 个 add_executable,`CMakeLists.txt:12,15,28`) | 活·生产 | ❔ |
| **go2_map_manager** (C++) | `route_relocalizer` / `submap_builder` | 路线×地图重定位、子图构建(`CMakeLists.txt:46,50`) | 活·生产 | ❔ |
| **go2_loop_backend** (py) | 11 个可执行(keyframe_saver / scan_context_detector / pose_graph_optimizer / pcd_to_nav2_map …,`setup.py:23-33` 恰 11 条) | 离线建图 / 回环闭合工具集 | 不在实时控制环 | ❔ |
| **unitree_api** | Request/Response 等 8 个 msg(`msg/` 恰 8 个) | 宇树运动 API 消息接口 | 依赖 | ❔ |
| 三方 | FAST_LIO / Livox-SDK2 / livox_ros_driver2 | 激光 SLAM + 雷达驱动 | 依赖 | ❌ FAST_LIO repo≠dog;🟰 livox_ros_driver2 的 lddc/lds repo==dog |

## 四、"乱"的结构性根源(全部有代码依据)

1. **测试链 ≠ 生产链,且"生产真身"要看狗上 manifest,不是仓库 saas 的 trace 装配**:
   - 照 README 读到的是 `waypoint_follower.py`;
   - 仓库 saas 装配的是 `scripts/waypoint_follower_go2_2_trace.py`(trace 包装器)【生产·仓库saas `:2061,1743`】;
   - **狗上 06/07 实跑的却是 `waypoint_follower_go2_2` 基类(`class WaypointFollower`,330 行,sha `d205a596`),无 trace**【狗上 dog:manifest `controller_executable` + `controller_reference_sha256`】。
   - **"查巡检 bug 连定位都费劲",头号原因极可能是读错文件**——而且连"读仓库 saas 的 trace 链"都可能读错,**得对狗上 manifest 才作数**。
   - ⚠️ **仓库自相矛盾(原文档未发现)**:trace 包装器 `:25` 需 `base_module.WaypointFollower`,而仓库 `waypoint_follower_go2_2.py:67` 只定义 `WaypointFollowerGo22`、无裸 `WaypointFollower` ⇒ **仓库这两个文件互不兼容**,trace 对仓库自身会抛 `BASE_FOLLOWER_CLASS_MISSING`。强烈提示仓库 `_go2_2.py`(WaypointFollowerGo22)是**分叉/实验版**,实际部署基类是狗上 330 行版。
2. **3 个跟随器 + 3 个命令节点**,且 `waypoint_follower.py / _go2_2.py / _old.py` 的 **node 名全叫 `waypoint_follower`**【默认 `:56 / :71 / :27`】,`ros2 node list` 无法区分。
3. **两条运动输出路径**:sport API(`/api/sport/request`)与 UDP/SDK2(`/cmd_vel`→桥)。生产里 safe 节点同时**配**了 `output_cmd_topic:=/cmd_vel` 和 `sport_request_topic:=/api/sport/request`【生产·仓库saas `:2055-2056`】,但运行时 `publish_move` **二选一**只发 Twist(见 §二更正 / §六①),**非双发**。
4. **同一参数多处不同默认值,且部分云端下发值其实"算了不用/被硬编码盖过"**——文档数值一律不可信,**以代码默认值 + 实际启动串 + 狗上 manifest 为准**:
   - `stop_distance`:【默认 `unitree_safe_cmd_node.py:73` 0.70】/【README `README.md:424` 0.40】/【生产·仓库saas `:2057` 0.80】。⚠️ **狗上实际值无法确证**:狗 06/07 manifest **未记录** stop_distance,且狗 saas ≠ 仓库 saas,故**只有代码默认 0.70 可确认**,0.80 仅是仓库 saas 会传的值。
   - `resume_distance`:【默认 `:74` 0.95】/【生产·仓库saas `:2057` 1.00】(原文档未提)。
   - `k_yaw`:【默认·云端 bounded_float `go2_saas_agent.py:1511-1516` 默认 1.20】**但被硬编码盖过** → follower_cmd【生产·仓库saas `:2064` 硬编码 `-p k_yaw:=0.900`】;且 `:1684 k_yaw_arg = shlex.quote("%.3f" % k_yaw)` **计算后全文件再无引用 = 死参**(云端算的 k_yaw 根本没进 follower)。【狗上 dog:manifest `go2_2_k_yaw=0.900`】—— **狗上确用 0.900**。
   - `max_yaw_rate`:**消费端三处分叉**:follower【生产·仓库saas `:2064` 硬编码 0.450】、safe 节点与 cmd_vel_sender 却用**计算值** `max_yaw_rate_arg`【生产·仓库saas `:2060`(safe)/`:2048`(sender),默认 0.60】——**三处上限不一致**。【狗上 dog:manifest `go2_2_max_yaw_rate=0.450`】—— **狗上 follower 侧用 0.450**。
5. **README 自认过时**【README `README.md:7` "本 README 的部分'当前状态'描述早于该交接记录";`:3-6` 是指向 HANDOFF 第 0 节的引导,非自认句本身】;`analysis/xunjian_20260725_shutdown_capture/previous_boot/remote_source/` 存有从狗上抓取的实跑代码副本(4 个文件),可用于校验"狗上 vs 仓库"——本轮已用它抓出 §四.1 的头号更正。

## 五、后续子系统文档(逐个深读沉淀)

| 编号 | 主题 | 回答的核心问题 |
|---|---|---|
| 01 | 基础层 bringup | 雷达怎么进系统、FAST-LIO 怎么出定位与点云 |
| 02 | CSV 巡检执行(生产跟随器) | 狗执行 CSV 时到底什么逻辑、怎么用 /Odometry(**注意对狗上 330 行 WaypointFollower,非仓库 1043 行版**) |
| 03 | 安全节点急停内部 | 雷达点云怎么做前方/侧向急停 |
| 04 | 运动输出(sport API vs UDP/SDK2) | 到底哪条真驱动腿(已定:生产走 Twist/UDP,二选一) |
| 05 | 路线录制 route_recorder | 整体怎么录制一条路线 |
| 06 | 地图 / PCD 生成 | pcd 怎么"打点拼合"出来 |
| 07 | 重定位 route_relocalizer | 巡检前怎么把路线对齐到地图(产物 `route_runtime.csv`) |
| 08 | SaaS agent | 云端命令词表、生产链装配、视频/outbox(**仓库 saas ≠ 狗上 saas,须分辨**) |
| 09 | 网络 4G | 一共几种连接方式、怎么切换 |
| 10 | 相机 z1pro | 云台/拍摄/上传 |
| 90 | 乱象与代码分叉审计 | 重复节点 / .bak / 死代码 / 狗上 vs 仓库 sha |

## 六、尚未坐实、留待深挖(不 overclaim)

- ✅ **已解决(见 03)**:生产里安全节点输出是**二选一**——设了 `output_cmd_topic:=/cmd_vel` 就**只发 Twist 到 `/cmd_vel`(走 UDP/SDK2)**,不发 sport API【默认 `unitree_safe_cmd_node.py:252-259`;生产·仓库saas `:2056`】。sport API 仅测试链用。(此结论已回填 §二图,原图"疑似双发"作废。)
- ✅ **已解决(本轮)**:`waypoint_follower_go2_2_trace.py` 与 `_go2_2.py` 的**包装关系**——trace 是 `TracedWaypointFollower(BaseFollower)` 子类,`import` 基类而非拷贝改(`:25 getattr`,`:130 class TracedWaypointFollower(BaseFollower)`)。但**仓库版基类类名不匹配**(需 `WaypointFollower`,仓库只有 `WaypointFollowerGo22`),包装器只能对狗上 330 行版生效。
- ✅ **已解决(本轮)**:**狗上到底跑哪个跟随器**——`waypoint_follower_go2_2` 基类(330 行 `WaypointFollower`),**非 trace**【狗上 dog:manifest 06/07】。
- ❔ `patrol_control.py` / `go2_course_control.py` / `unitree_cmd_node.py` 是否仍被调用(生产链未启 unitree_cmd_node;另两者为控制库,待定)。
- ❔ 各跟随器 `cmd_topic` 默认值是否都对得上 `/patrol_cmd`。
- ❔ **狗上 saas 的真实内容**:目前只能从 manifest schema(`controller_reference_sha256` vs 仓库的 `controller_source_sha256`+`controller_trace_wrapper_sha256`)反推它≠仓库 saas,**无直接副本**,狗上 saas 的参数装配(如 stop_distance)无法确证。

---

## 附 · 核验台账(claim → 证据 → 判定)

> 判定值:**CONFIRMED**=属实 / **CORRECTED**=有错已更正 / **DEFAULT_VS_PROD**=默认≠生产已补 / **UNVERIFIABLE**=无法证实。

| # | 断言(摘要) | 证据 `file:line` | 判定 | 更正 / 备注 |
|---|---|---|---|---|
| 1 | 基于 commit `b5b87fc` Initial import | git log:`b5b87fc` 存在,HEAD=`6ec2382` | **CORRECTED** | HEAD 实为 `6ec2382`;其后仅 docs-only,源码等同 b5b87fc |
| 2 | MID-360→FAST-LIO→`/Odometry`+`/cloud_registered_body` | `laserMapping.cpp:1119/1122`;`go2_mid360s.yaml` 存在 | **CONFIRMED** | 狗版副本亦发同名话题(`:1091/1094`);❌ repo≠dog(话题结论两版一致) |
| 3 | saas 接 SaaS,收 start_patrol/move/stop | `go2_saas_agent.py:50/51/52` | **CONFIRMED** | stop 实为 `stop_patrol`(无裸 stop);`llyj` 未见于代码(docstring 自称 GoGoGuard SaaS `:2`),UNVERIFIABLE |
| 4 | start_patrol 装配 `:1475 start_patrol_command` | `go2_saas_agent.py:1475` | **CONFIRMED** | — |
| 5 | route_relocalizer(ICP 粗+精,找同名 pcd)→relocalized.csv | `go2_saas_agent.py:1989`;`route_relocalizer.cpp:445-458/481-488/862-872`;saas `:706/724/744` | **CORRECTED** | 输出文件名实为 **`route_runtime.csv`**(`out_route_file` `:1990/1660`);"relocalized.csv" 仅概念标签 |
| 6 | `waypoint_follower_go2_2_trace.py` ★生产真身★ 狗上真跑 | repo saas `:1743/:2061-2073`;**dog manifest 06/07 `controller_executable=waypoint_follower_go2_2` + `controller_reference_sha256=d205a596`** | **CORRECTED** | 狗上实跑基类(330 行 `WaypointFollower`)、**无 trace**;trace 仅仓库装配,狗 saas 是另一版本(schema 不同) |
| 7 | safe 节点限速+ROI 点云急停,订阅 `/cloud_registered_body` | saas `:2055`;`unitree_safe_cmd_node.py:174-179/220/184`;saas `:2058` roi | **CONFIRMED** | — |
| 8 | Twist→/cmd_vel→UDP:127.0.0.1:5005→receiver→SDK2 | saas `:2045-2049/:2022-2025` | **CONFIRMED** | — |
| 9 | safe 另发 Request→/api/sport/request,疑似双发 | `unitree_safe_cmd_node.py:250-259` publish_move if/else | **CORRECTED** | **非双发**;设了 output_cmd_topic 只发 Twist。§二图与 §六 曾内部矛盾,已同步 |
| 10 | 旁路 5 件(monitor/telemetry/snapshot/rosbag/session_guard) | saas `:2026/:2029/:2035/:1787/:2080` | **CONFIRMED** | — |
| 11 | 测试链 waypoint_follower→safe→sport;output_cmd_topic 默认空不发 /cmd_vel | `run_roomtest7_readme_safe_patrol.sh:35/79`;`unitree_safe_cmd_node.py:48/259` | **CONFIRMED** | — |
| 12 | 基础层网络/雷达/5 重试/10 帧新鲜 | `base_bringup.sh:20-21/33/171/136/220`;`check_fastlio_freshness.py:35`;`base_bringup.sh:10` | **CONFIRMED** | 10 帧既是默认也是实际调用值 |
| 13 | 3 跟随器 node 名全叫 waypoint_follower | `:56 / :71 / :27` | **CONFIRMED** | — |
| 14 | waypoint_follower_old.py 死代码 | `setup.py:22-29` console_scripts 仅 6 项无 old | **CONFIRMED** | — |
| 15 | `_go2_2.py` = 生产 trace 版基底 | `waypoint_follower_go2_2.py:67 class WaypointFollowerGo22`;trace `:22/25 getattr WaypointFollower`,`:26-30` 缺失即崩 | **CORRECTED** | 仓库该文件类名不匹配,**非** trace 所需基底;真基底是狗上 330 行 `WaypointFollower`。仓库自相矛盾 |
| 16 | unitree_cmd_node 无安全,直发 sport Request | `unitree_cmd_node.py:8/45/80`;`setup.py:26` | **CONFIRMED** | 无点云订阅;是否仍被调用为开放项(生产链未启) |
| 17 | unitree_go_safe_cmd_node 发 SportModeCmd | `unitree_go_safe_cmd_node.py:10/110`;`setup.py:28` | **CONFIRMED** | — |
| 18 | patrol_control / go2_course_control 各带 test | `go2_fastlio_patrol/test/` 二文件 | **CONFIRMED** | — |
| 19 | go2_cmd_vel_bridge 3 可执行 | `CMakeLists.txt:12/15/28` | **CONFIRMED** | — |
| 20 | go2_map_manager 2 可执行 | `CMakeLists.txt:46/50`;`submap_builder.cpp:22` | **CONFIRMED** | — |
| 21 | go2_loop_backend 11 可执行 | `setup.py:23-33` 恰 11 条 | **CONFIRMED** | — |
| 22 | unitree_api 8 msg | `unitree_api/msg/` 恰 8 个 | **CONFIRMED** | — |
| 23 | stop_distance 默认 0.70/README 0.40/生产 0.80 | `unitree_safe_cmd_node.py:73`;`README.md:424`;saas `:2057` | **CONFIRMED** | 三值全中;**狗上实际无法确证**(manifest 未记 + 狗 saas 另版),仅默认 0.70 可确认;resume_distance 默认 0.95/生产 1.00 |
| 24 | README 自认过时;remote_source 存狗上副本 | `README.md:7`(非 :3-6);remote_source 4 文件 | **CONFIRMED** | 自认句在 :7;副本:lddc/lds 与仓库一致,laserMapping/waypoint_follower_go2_2 不一致 |
| 25 | 设 output_cmd_topic:=/cmd_vel 只发 Twist,sport 仅测试链 | `unitree_safe_cmd_node.py:252-259`;saas `:2056` | **CONFIRMED** | 与 §二图旧"双发"矛盾,图已更正 |
| 26 | 生产跟随器角速度/增益参数(任务追消费端) | saas `:1511-1516/:1684/:2064`;manifest `go2_2_k_yaw=0.900`/`go2_2_max_yaw_rate=0.450` | **DEFAULT_VS_PROD** | `k_yaw`:云端默认 1.20 但 follower 硬编码 **0.900**,`:1684 k_yaw_arg` **死参**;`max_yaw_rate`:follower 硬编码 **0.450**,safe/sender 用计算值默认 0.60(三处不一致);**狗上确用 0.900/0.450** |
| 27 | 工程根 /home/unitree/go2_fastlio_ws;Ubuntu20.04+Foxy | dog manifest `route=/home/unitree/go2_fastlio_ws/...`;Ubuntu/Foxy 无佐证 | **UNVERIFIABLE**(部分) | 工程根 CONFIRMED;Ubuntu20.04+Foxy 属【推断-未验】 |

### 狗上对照状态一览(sha 验)

| 文件 | 状态 | 备注 |
|---|---|---|
| `remote_source/waypoint_follower_go2_2.py` | ❌ **repo≠dog** | 仓库 1043 行 `WaypointFollowerGo22` sha `009cb25b` ≠ 狗上 330 行 `WaypointFollower` sha `d205a596`(=manifest 引用)。**生产真身是狗版** |
| `scripts/go2_saas_agent.py` | ❌ **repo≠dog**(推) | 无直接副本;但狗 manifest schema(`controller_reference_sha256`)≠ 仓库 saas 写出的(`controller_source_sha256`+`controller_trace_wrapper_sha256` `:1830-1832`)⇒ 狗 saas 另一版本、直接启基类无 trace |
| `FAST_LIO/src/laserMapping.cpp` | ❌ **repo≠dog** | 仓库 sha `5fec8282`(1414 行)≠ 狗 sha `e4cd05cb`(1395 行);但均发 `/Odometry`+`/cloud_registered_body`,话题结论两版一致 |
| `livox_ros_driver2/src/lddc.cpp` | 🟰 **repo==dog** | sha `b5811eaf` 一致 |
| `livox_ros_driver2/src/lds.cpp` | 🟰 **repo==dog** | sha `8345dfde` 一致 |
| `scripts/waypoint_follower_go2_2_trace.py` | ❔ **无狗上对照** | 无副本;manifest 06/07 未跑 trace。仓库该包装器 `:25` 需 `WaypointFollower`,仓库只有 `WaypointFollowerGo22`→对仓库自身会崩,仅兼容狗上 `WaypointFollower` |
| `unitree_safe_cmd_node.py` / `waypoint_follower.py` / `waypoint_follower_old.py` / `route_relocalizer.cpp` / `base_bringup.sh` / `check_fastlio_freshness.py` / `setup.py` 等 | ❔ **无狗上对照** | ~117 个自研文件之一,结论均基于仓库源码,**不代表狗上一致** |

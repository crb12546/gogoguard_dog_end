# 00 · 系统总览

> **本文档原则**:所有结论 100% 基于当前仓库真实代码(commit `b5b87fc` · Initial import)。
> 引用一律标 `文件:行`。与 README / 旧 `.md` 文档冲突处,**以代码为准**。
> 旧文档(README、HANDOFF、各 *_NOTES.md)只作线索,数值/状态描述均已被证实不可信。

## 一、这是什么系统

宇树 **Go2 EDU** 四足机器狗的**固定路线巡检(xunjian)**程序,运行在狗背上的 **Orin**(Ubuntu 20.04 + ROS2 Foxy,工程根 `/home/unitree/go2_fastlio_ws`)。

- **感知/定位**:Livox **MID-360** 激光雷达 → **FAST-LIO** 激光惯性里程计(SLAM),输出机体位姿 `/Odometry` 和机体系点云 `/cloud_registered_body`。
- **巡检方式**:人工遥控录一条路线(x,y,yaw 的 CSV)→ 之后自动沿线巡逻(纯跟踪)。
- **云端**:`go2_saas_agent.py` 接 llyj SaaS 云端,收命令(start_patrol / move / stop…)、回传视频与结果。

## 二、两条巡检链(理解全局的钥匙)

⚠️ **测试链 ≠ 生产链**——两者用的是不同的跟随器文件、不同的运动输出路径。

### 🟢 生产链(云端 `start_patrol` 触发;装配见 `go2_saas_agent.py:1475 start_patrol_command`)
```
llyj云端 ──start_patrol+route──▶ saas command-loop ──下载 route.csv / 找同名 .pcd 地图──┐
                                                                                      ▼
   route_relocalizer            (route.csv × 地图.pcd × /Odometry, ICP粗+精对齐) → relocalized.csv
   [go2_map_manager]                                                                  │  go2_saas_agent.py:1989
                                                                                      ▼
   waypoint_follower_go2_2_trace.py   ★生产真身★  (读 relocalized.csv + /Odometry, 纯跟踪)
   [scripts/, go2_saas_agent.py:1743,2061]                                            │  Twist → /patrol_cmd
                                                                                      ▼
   unitree_safe_cmd_node        (限速 + 前方/侧向 ROI 点云急停; 订阅 /cloud_registered_body)
   [go2_fastlio_patrol]              ├─ Twist → /cmd_vel ─▶ cmd_vel_udp_sender ─UDP:127.0.0.1:5005─▶ go2_sdk2_udp_receiver ─▶ Go2 SDK2(驱动腿)
                                     └─ Request → /api/sport/request   (运动API;与上面 UDP 路径疑似双发, 哪条真驱动待深挖)
   旁路: patrol_performance_monitor / go2_experiment_telemetry / snapshot / rosbag record / localization_session_guard
```

### 🔵 测试链(README §10 / `scripts/run_roomtest7_readme_safe_patrol.sh`,手动跑)
```
waypoint_follower.py ─Twist /patrol_cmd─▶ unitree_safe_cmd_node ─Request─▶ /api/sport/request ─▶ Go2 sport API
（无重定位、无 UDP 桥、无遥测;跟随器是另一个文件 waypoint_follower.py,safe 节点 output_cmd_topic 默认空 → 不发 /cmd_vel）
```

### ⚙️ 基础层(两条链共用;`scripts/base_bringup.sh`)
```
配雷达网(eth1 192.168.1.5/24 → 雷达 192.168.1.161)
  → Livox MID360 驱动(5次重试+流校验) → /livox/lidar, /livox/imu
  → FAST-LIO(mapping.launch.py, config=go2_mid360s.yaml) → /Odometry, /cloud_registered_body
  → "10 帧连续新鲜" 门槛(check_fastlio_freshness.py)才算 ready
```

## 三、节点 / 包清单(自研为主)

| 包 | 文件 / 节点 | 角色 | 状态 |
|---|---|---|---|
| **go2_fastlio_patrol** (py) | `waypoint_follower.py` | 测试/README 跟随器(node 名 "waypoint_follower") | 活·测试链 |
| | `waypoint_follower_go2_2.py` | 生产 trace 版的基底 | 活·生产 |
| | `waypoint_follower_old.py` | 旧跟随器 | **死**(不在 entry_points) |
| | `unitree_safe_cmd_node.py` | 安全过滤 + 双路输出(sport Request / Twist) | 活·两链都用 |
| | `unitree_cmd_node.py` | 无安全,直发 sport Request | 存疑(在 entry_points) |
| | `unitree_go_safe_cmd_node.py` | 发 `SportModeCmd`(另一种控制接口) | 变体·用途存疑 |
| | `route_recorder.py` | 录制路线 CSV | 活·录制时 |
| | `patrol_control.py` / `go2_course_control.py` | 控制库?(各自带 test) | 待定 |
| | `route_quality.py` | 路线质量评估工具 | 工具 |
| **scripts/** | `waypoint_follower_go2_2_trace.py` | **★生产实际跑的跟随器★** | 活·生产 |
| **go2_cmd_vel_bridge** (C++) | `cmd_vel_udp_sender` / `go2_sdk2_udp_receiver` / `go2_sdk2_motion_probe` | `/cmd_vel`→UDP→SDK2 运动桥 | 活·生产 |
| **go2_map_manager** (C++) | `route_relocalizer` / `submap_builder` | 路线×地图重定位、子图构建 | 活·生产 |
| **go2_loop_backend** (py) | 11 个可执行(keyframe_saver / scan_context_detector / pose_graph_optimizer / pcd_to_nav2_map …) | 离线建图 / 回环闭合工具集 | 不在实时控制环 |
| **unitree_api** | Request/Response 等 8 个 msg | 宇树运动 API 消息接口 | 依赖 |
| 三方 | FAST_LIO / Livox-SDK2 / livox_ros_driver2 | 激光 SLAM + 雷达驱动 | 依赖 |

## 四、"乱"的结构性根源(全部有代码依据)

1. **测试链 ≠ 生产链**:照 README 读到的是 `waypoint_follower.py`,狗上真跑的是 `scripts/waypoint_follower_go2_2_trace.py`(`go2_saas_agent.py:2061,1743`)。**"查巡检 bug 连定位都费劲",头号原因极可能是读错了文件。**
2. **3 个跟随器 + 3 个命令节点**,且 `waypoint_follower.py / _go2_2.py / _old.py` 的 **node 名全叫 `waypoint_follower`**(`:56 / :71 / :27`),`ros2 node list` 无法区分。
3. **两条运动输出路径**:sport API(`/api/sport/request`)与 UDP/SDK2(`/cmd_vel`→桥)。生产里 safe 节点**同时**配了 `output_cmd_topic:=/cmd_vel` 和 `sport_request_topic:=/api/sport/request`(`go2_saas_agent.py:2055-2056`)。
4. **同一参数多处不同默认值**:如 safe 节点 `stop_distance` 代码默认 0.70(`unitree_safe_cmd_node.py:73`)、README 写 0.40、生产传 0.80 → 文档数值一律不可信,以代码默认值 + 实际启动参数为准。
5. **README 自认过时**(`README.md:3-6`);`analysis/xunjian_20260725_shutdown_capture/previous_boot/remote_source/` 存有从狗上抓取的实跑代码副本,可用于校验"狗上 vs 仓库"是否一致。

## 五、后续子系统文档(逐个深读沉淀)

| 编号 | 主题 | 回答的核心问题 |
|---|---|---|
| 01 | 基础层 bringup | 雷达怎么进系统、FAST-LIO 怎么出定位与点云 |
| 02 | CSV 巡检执行(生产跟随器) | 狗执行 CSV 时到底什么逻辑、怎么用 /Odometry |
| 03 | 安全节点急停内部 | 雷达点云怎么做前方/侧向急停 |
| 04 | 运动输出(sport API vs UDP/SDK2) | 到底哪条真驱动腿 |
| 05 | 路线录制 route_recorder | 整体怎么录制一条路线 |
| 06 | 地图 / PCD 生成 | pcd 怎么"打点拼合"出来 |
| 07 | 重定位 route_relocalizer | 巡检前怎么把路线对齐到地图 |
| 08 | SaaS agent | 云端命令词表、生产链装配、视频/outbox |
| 09 | 网络 4G | 一共几种连接方式、怎么切换 |
| 10 | 相机 z1pro | 云台/拍摄/上传 |
| 90 | 乱象与代码分叉审计 | 重复节点 / .bak / 死代码 / 狗上 vs 仓库 |

## 六、尚未坐实、留待深挖(不 overclaim)

- ✅ **已解决(见 03)**:生产里安全节点输出是**二选一**——设了 `output_cmd_topic:=/cmd_vel` 就**只发 Twist 到 `/cmd_vel`(走 UDP/SDK2)**,不发 sport API。sport API 仅测试链用。
- `waypoint_follower_go2_2_trace.py` 与 `_go2_2.py` 的**包装关系**(import 还是拷贝改)。
- `patrol_control.py` / `go2_course_control.py` / `unitree_cmd_node.py` 是否仍被调用。
- 各跟随器 `cmd_topic` 默认值是否都对得上 `/patrol_cmd`。

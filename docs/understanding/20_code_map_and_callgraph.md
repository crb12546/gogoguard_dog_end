# 20 · 代码地图 & 调用图(专给重构用)

> 原则同 00:只认代码。**本篇全部基于 2026-07-25 对工作树的实测 grep/ls/CMake/setup.py**,不是记忆。
> 目的:给"重新整理代码"提供一张 grounded 的侦察图——包/文件归属、死代码、运动控制全调用点、话题耦合契约。

## 〇、核验状态
- 【今日实测】= 本篇写作时对 `orin_go2_fastlio_ws/` 直接 grep/ls 得到,附命令可复现。
- 【狗 dog】= 有狗上副本/manifest 佐证(仅 4 文件 + manifest,见 [[project_gogoguard_dog]] 与 doc 00 核验状态)。
- 【无狗上对照】= 仅仓库可见,狗上是否同构未验(**结构性调用图在 repo 与 dog 间大体一致,但 follower/laserMapping 两处 repo≠dog,已单独标**)。
- ⚠️ 本篇是"仓库代码的地图";狗上实跑差异集中在:follower(基类 330 行,无 trace/无课程反馈)、laserMapping(另一 build)。重构以哪份为基线**必须先定**(见 doc 00 / 90)。
- ⚠️ **本篇初稿写完即提交,未先核 → 被 50 答核验工作流回查揪出 6 处错并已订正**:submap 点云话题名、third_party 计数(git树 vs 本地)、/Odometry 消费口径、/cloud_registered_body 控制路、原生避障"并存"、生产运动图。以下正文均为订正后版本,细节见文末核验台账。

---

## 一、包/目录分布总图【今日实测 ls + package.xml】

```
orin_go2_fastlio_ws/
├─ src/                              # 6 个 ROS 包
│  ├─ FAST_LIO/             cmake C++  三方 SLAM(改过:健康门+/dev/shm 写)
│  ├─ livox_ros_driver2/    cmake C++  三方雷达驱动(改过:健康埋点+跳帧)
│  ├─ Livox-SDK2/           cmake C++  三方 SDK(未改)
│  ├─ go2_cmd_vel_bridge/   cmake C++  ★运动桥(3 可执行)
│  ├─ go2_fastlio_patrol/   py         ★巡检节点(11 py:3 follower+3 cmd+recorder+2 库+quality)
│  ├─ go2_loop_backend/     py         离线建图/回环(11 entry,19 文件)
│  ├─ go2_map_manager/      cmake C++  重定位+子图(2 可执行)
│  └─ unitree_api/          cmake      纯 8 个 msg(Request/Response…)
├─ scripts/     50 文件   ★★操作层大头(saas/4G/相机/遥测/守卫/录制/install)——不在 ROS 包
├─ config/      8 文件    camera.env + DDS×2 + amcl×2 + slam_toolbox×2 + nav2_params(⚠️非 FAST-LIO 配置)
├─ cpp_tools/   2 文件    go2_builtin_camera_capture / go2_oa_cpp_test(内置相机/避障探针)
├─ deploy/      1 文件    systemd_user/go2-fastlio-base.service
├─ launch/      —         ⚠️全系统只有 1 个 launch:src/FAST_LIO/launch/mapping.launch.py
└─ third_party/ (git树1545,⚠️未checkout到本地)unitree_sdk2(+install),SportClient 来自这
                本地现存三方仅 FAST_LIO35+Livox-SDK2×35+livox_ros_driver2×97=167,自研 py/cpp/sh 实测119
```

**重构第一洞察**:
1. **`scripts/`(50 文件)是杂物抽屉**——操作层绝大部分逻辑在扁平脚本里,不是 ROS 包。**最该 package 化/分类的就是它**。
2. **全系统几乎不用 ros2 launch**(只有 FAST-LIO 一个);实际编排靠 **systemd + saas 现场拼的巨型 bash + install 脚本**。重构编排层要认这个事实。
3. FAST-LIO 的运行配置(`go2_mid360s.yaml`)在 **`src/FAST_LIO/config/`**,不在顶层 `config/`;顶层 `config/` 是 nav2/amcl/slam_toolbox/DDS(旁支路线,非当前巡检主链)。

## 二、可执行/节点清单【今日实测 setup.py + CMakeLists】

| 包 | 可执行/节点 | 备注 |
|---|---|---|
| **go2_fastlio_patrol**(entry_points 6) | `route_recorder` `waypoint_follower` `waypoint_follower_go2_2` `unitree_cmd_node` `unitree_safe_cmd_node` `unitree_go_safe_cmd_node` | `waypoint_follower_old` **不在**(死);trace 壳在 scripts/ 非 entry |
| **go2_cmd_vel_bridge**(add_executable 3) | `cmd_vel_udp_sender` `go2_sdk2_udp_receiver` `go2_sdk2_motion_probe` | receiver=裸 main+SDK2,非 ROS 节点 |
| **go2_map_manager**(add_executable 2) | `route_relocalizer` `submap_builder` | submap_builder 订 /Odometry + **/cloud_registered(非 _body)**,按 **10m 位移阈值**存子图 pcd + trajectory【submap_builder.cpp:24-25,55】 |
| **go2_loop_backend**(entry 11) | keyframe_saver / build_raw_map / scan_context_detector / pose_graph_optimizer / dynamic_map_filter / sliding_window_static_filter / export_registered_cloud_map / level_pcd / pcd_to_nav2_map(_fast) / offline_keyframe_extractor | ⚠️非"全离线":**keyframe_saver 是实时节点**(订 /Odometry+/cloud、rclpy.spin);其余10个 argparse 离线。这11个无 saas 编排,但包内 `go2_start_level_scan.sh` 会编排**非-entry 模块**(odom_to_tf_map_level_2d / level_cloud_node) |
| **unitree_api** | (纯 msg) | Request/RequestHeader/Identity/Lease/Policy + Response×3 |

- go2_fastlio_patrol 的 2 个**库文件**(非节点):`patrol_control.py`(被引用 5 处=`waypoint_follower.py` 那套)、`go2_course_control.py`(被引用 2 处=仓库 `waypoint_follower_go2_2`)。【今日实测 grep -rl】

## 三、scripts/ 50 文件功能分类(重构分桶用)【今日实测】

| 桶 | 数量 | 代表 |
|---|---|---|
| 4G/网络 | 11 | `go2_4g_manager.py` `go2_connectivity_watchdog.sh` `install_a7600c_{ecm,ppp}_only.sh` `go2_network_recover.sh` `go2_wired_ssh_rescue.sh` |
| 相机 | 8 | `z1pro_{gcu_control,capture,upload_segment,preset}.*` `go2_camera_*` |
| 遥测/证据/审计 | 9 | `go2_experiment_{telemetry,snapshot,audit}.py` `patrol_performance_monitor.py` `go2_lio_trace_recorder.py` |
| install/部署 | 7 | `install_{autostart,saas_autostart,connectivity_watchdog,network_recover,go2_4g_manager}.sh` |
| 录制/路线 | 5 | `route_recording_blackbox.py` `manual_route_anchor.py` |
| 定位/会话守卫 | (散在上面) | `localization_session_guard.py` `check_{fastlio_freshness,route_start_alignment}.py` `go2_base_health_watchdog.py` `ensure_base_ready.sh` |
| base/编排 | — | `base_bringup.sh` `base_stop.sh` `start_saas_loops.sh` `go2_saas_agent.py`(3152 行) |
| **legacy 可归档** | — | `*_legacy_go2_cmd_bridge.sh`(start/stop/probe) `patrol_cli.disabled_before_cmd_rework.py` `build_legacy_iox_stub.sh` `run_roomtest7_*.sh` `go2_motion_probe.sh` `env_common.sh` |

## 四、死/遗留代码清单(可先归档)【今日实测 grep 佐证】

| 文件 | 证据 | 结论 |
|---|---|---|
| `waypoint_follower_old.py` | entry_points=**0**、被引用=**0** | **真死**,可归档 |
| `patrol_cli.disabled_before_cmd_rework.py` | 文件名自述 disabled | 死(旧 CLI) |
| `{start,stop,probe}_legacy_go2_cmd_bridge.sh` + `build_legacy_iox_stub.sh` | legacy 命名 | 死(旧命令桥) |
| `patrol_control.py` | entry=0 但被引用 **5** | **不是死**——是 `waypoint_follower.py`(测试链)的库,随它去留 |
| `go2_course_control.py` | entry=0 但被引用 **2** | **不是死**——仓库 `waypoint_follower_go2_2` 的课程反馈库;⚠️**但狗上 330 行版不 import 它**,狗端等于没用 |
| `.bak_*` 快照 | 遍布仓库/backups/ | 手改存档,无版本管理,可清 |

## 五、运动控制全调用图(你点名的:都在哪调用)【今日实测 grep】

**三种运动接口 → 生产的"腿驱动"只经①的 receiver;但②的节点(安全节点)生产也在跑,只是被切成 Twist 模式、其 api_id-1008 路径被绕过:**

| # | 接口 | 调用点 | 状态 |
|---|---|---|---|
| ① | **SDK2 `SportClient` 直连** | `go2_sdk2_udp_receiver.cpp`: `StandUp():91` `BalanceStand():93` `Move():219` `StopMove():241`(**生产唯一腿驱动**);`go2_sdk2_motion_probe.cpp`: Move/Stand/Stop:104-140(诊断) | ★生产 + 诊断 |
| ② | **sport API `Request`** | `unitree_safe_cmd_node.py:238`(`api_id=1008` Move,output 空时);`unitree_cmd_node.py`(1008/1003) | 测试链 |
| ③ | **`SportModeCmd` 话题** | `unitree_go_safe_cmd_node.py:110` pub | 变体,生产不用 |

- **api_id 常量**:`MOVE=1008`、`STOPMOVE=1003`【unitree_safe_cmd_node:22 / unitree_cmd_node:13-14】。
- **StopMove 多重保险**(急停):receiver:241、motion_probe:111/140、以及 sport API 1003 直发 3 处(`localization_session_guard.py:78`、`go2_saas_agent.py:2484` stop_patrol、`patrol_cli.disabled:140`)。
- **Go2 原生避障**:`scripts/enable_oa_only.py:25` `ObstaclesAvoidClient.SwitchSet(True)`,机制独立于安全节点点云 ROI;⚠️**但该脚本零引用、未接进生产**(saas 全无引用)→ 生产实际**只有安全节点 ROI 在跑**,原生避障需人工手动执行才开,**不是自动"并存"**。
- ⚠️ **抢腿风险**:`go2_sdk2_motion_probe` 也会 `StandUp/Move`,巡检中误跑会和 receiver 抢同一 SportClient——**重构时应把 probe 明确隔离/加互斥**。
- **生产完整链**:`follower →Twist /patrol_cmd → unitree_safe_cmd_node →Twist /cmd_vel → cmd_vel_udp_sender →UDP 127.0.0.1:5005 → go2_sdk2_udp_receiver → SportClient.Move`。

## 六、话题耦合契约(重构前应冻结)【今日实测 grep】

| 话题 | 生产者 | 消费者 | 重构提示 |
|---|---|---|---|
| **`/Odometry`** | `laserMapping.cpp:745` | **活订阅~15**:3 follower + route_recorder + map_manager×2 + loop_backend内5(keyframe_saver+odom_to_tf×4) + telemetry + lio_trace + saas + watchdog;⚠️另 audit/blackbox/snapshot 是**录包/topic查询非订阅**、offline_keyframe_extractor 离线读bag → 引用**总计~28 文件** | **最脆耦合面**:改名/改 QoS 波及 ~28 文件,先冻结契约(名+QoS+frame `odom→base_link`) |
| **`/cloud_registered_body`** | `laserMapping.cpp:1119` | 生产控制路 `unitree_safe_cmd_node`(点云急停)**+ 遗留变体 `unitree_go_safe_cmd_node` 也订(:112)**;其余 map/diag(relocalizer/level_cloud/keyframe/export/snapshot/saas/lio_trace)。⚠️ **submap_builder 订的是 /cloud_registered 非本话题** | 生产控制路=安全节点;go_safe 变体非生产 |
| **`/patrol_cmd`** | 运行时 1 个 follower | 运行时 1 个 safe 节点(但 **6 个文件**都声明 `cmd_topic='/patrol_cmd'`) | node 名三重撞车(全 `waypoint_follower`),重构应改 node 名 |
| **`/cmd_vel`** | `unitree_safe_cmd_node:167`(output 非空) | `cmd_vel_udp_sender.cpp:104` | 二选一输出,生产走这条 |
| **`/dev/shm/go2_fastlio_latest_odom.txt`** | `laserMapping.cpp`(自研改造,原子写) | **4 读**:`check_fastlio_freshness` `localization_session_guard` `manual_route_anchor` `check_route_start_alignment` | ROS 外低开销位姿总线;重构别忽略这条隐形依赖 |
| **`/api/sport/request`** | 测试链 safe/cmd 节点 + 停车直发(session_guard/saas) | Go2 板载 sport 服务 | 生产不走(仅停车 1003) |

## 七、重构建议(仅侦察结论,非执行)
1. **先定基线**:仓库版 vs 狗上版(follower/laserMapping 已 sha 证不同)——不定基线,重构对象都不明确。
2. **package 化 `scripts/`**:按第三节 8 个桶拆成子包(network / camera / telemetry / localization_guard / orchestration / install / legacy-archive)。
3. **冻结话题契约**:`/Odometry`(~15 活订阅/~28 引用文件)、`/dev/shm` 快照(4 读)、`/cloud_registered_body`(生产安全节点 + go_safe 变体)——改动前立契约测试。
4. **归档死代码**:第四节清单(waypoint_follower_old / legacy cmd_bridge / patrol_cli.disabled / .bak_*)。
5. **收敛运动接口**:三种接口留 ①,归档 ②③;probe 加互斥防抢腿。
6. **node 名去重**:3 个 follower 改掉同名 `waypoint_follower`。

## 核验台账(claim → 证据 → 判定)
| # | claim | 证据【今日实测】 | 判定 |
|---|---|---|---|
| 1 | src/ 6 ROS 包 + 各 build 类型 | `ls src/*/package.xml` + `<build_type>` | CONFIRMED |
| 2 | scripts/ 50 文件 | `find scripts -type f` | CONFIRMED |
| 3 | config/ **8** 文件(非 5) | `ls config/` | CORRECTED(先前口误 5) |
| 4 | 全系统仅 1 launch | `find -name *.launch.py` 非三方=1 | CONFIRMED |
| 5 | go2_fastlio_patrol entry 6 + old 不在 | `setup.py` | CONFIRMED |
| 6 | bridge 3 / map_manager 2 可执行 | `CMakeLists add_executable` | CONFIRMED |
| 7 | 腿驱动唯一点 receiver:219 | grep SportClient/Move | CONFIRMED |
| 8 | 三种运动接口 | grep SportClient/Request/SportModeCmd | CONFIRMED |
| 9 | api_id 1008/1003 | `unitree_*cmd_node:13-22` | CONFIRMED |
| 10 | ~~/Odometry ~20 消费~~ → **活订阅~15 / 引用~28** | audit/blackbox/snapshot 是录包/查询非订阅 | **IMPRECISE→已订正** |
| 11 | /dev/shm 写1读4 | grep go2_fastlio_latest_odom | CONFIRMED |
| 12 | waypoint_follower_old 真死 | entry=0 & ref=0 | CONFIRMED |
| 13 | patrol_control/course_control 是库非死 | ref=5 / ref=2 | CONFIRMED |
| 14 | 狗上 follower=基类无 trace/无课程反馈 | manifest + remote_source sha | 【狗 dog】CONFIRMED |

### 本篇被 50 答核验工作流揪出并已订正的 6 处
| 处 | 原错 | 订正 |
|---|---|---|
| submap_builder 话题 | 写成订 `/cloud_registered_body` | 实为 `/cloud_registered`(submap_builder.cpp:25) |
| third_party 计数 | "1545 文件"当本地存在 | git树1545 但**未 checkout**;本地现存三方仅 167 |
| /Odometry 消费口径 | "~20 消费" | 活订阅~15,audit/blackbox/snapshot 非订阅,引用共~28 |
| /cloud_registered_body 控制路 | "只有安全节点" | 还有遗留变体 `unitree_go_safe_cmd_node`(:112) |
| 原生避障 | "两套避障并存" | enable_oa_only 零引用手动脚本,生产不自动开 |
| 生产运动图 | "生产只用①receiver" | 安全节点(②的节点)生产也跑,只是切 Twist 模式 |

**狗上对照**:本篇结构图【无狗上对照】(仅 follower/laserMapping 两处 repo≠dog 已 sha 证);调用图结构在两版间预期一致,但**具体行号以仓库为准,狗上未逐一验**。

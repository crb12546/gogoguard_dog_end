# 90 · 乱象与代码分叉审计(给下一个开发者的"定位地图")

> 原则同 00:全部是代码事实。本篇不"改",只把"乱在哪、什么在跑、什么是死的、仓库和狗上差在哪"钉清楚,让人能快速定位。

## 一、最重要的一件事:仓库 ≠ 狗上,且生产跟随器有三套

**狗上真跑的**(`analysis/xunjian_20260725_shutdown_capture/previous_boot/remote_source/waypoint_follower_go2_2.py`):
- **330 行,`class WaypointFollower`,自包含**(不 import course_control / patrol_control)。

**仓库里的** `src/.../waypoint_follower_go2_2.py`:
- **1043 行,`class WaypointFollowerGo22`**,import `go2_course_control`,是一次大重写并**改了类名**。

**后果(纯代码推理)**:生产用 `waypoint_follower_go2_2_trace.py` 拉起,而它 `getattr(module, "WaypointFollower")`(`trace.py:25`):
- 对**狗上版**(类名 `WaypointFollower`)→ 能实例化 ✅
- 对**仓库版**(类名 `WaypointFollowerGo22`)→ 取不到 → 抛 `BASE_FOLLOWER_CLASS_MISSING` 崩溃 ❌

→ **仓库里这条生产链是断的;仓库提交的跟随器从未在狗上原样运行过。** 谁要改巡检逻辑,第一步必须先对齐"仓库 ↔ 狗上"到底以哪份为准。

## 二、"什么在跑" vs "README/新人会读到什么"

| | 生产真身(狗上/SaaS 装配) | README/测试/新人默认读到 |
|---|---|---|
| 跟随器 | `scripts/waypoint_follower_go2_2_trace.py` → `waypoint_follower_go2_2.py` | `src/.../waypoint_follower.py` |
| 控制设计 | 最近点 + 前视纯跟踪 + 课程纠偏 | 顺序段锁定 + line_follow 横向纠偏 |
| 运动输出 | Twist→`/cmd_vel`→UDP→SDK2 `SportClient.Move` | Request→`/api/sport/request` sport API |
| 重定位 | route_relocalizer 预变换 CSV | follower 自己吃 route_transform_json |
> **新人读错文件、走错链路,几乎是必然的。** 这是"连定位都费劲"的直接原因。

## 三、节点动物园(重复实现清单)

### 跟随器 ×3(node 名全叫 `waypoint_follower`,ros2 node list 分不清)
| 文件 | 行 | 设计 | 用 patrol_control? | 吃 CSV 'v'? | 起点 | 脱轨 | 状态 |
|---|---|---|---|---|---|---|---|
| `waypoint_follower_go2_2.py` | 1043 | 最近点+前视(course_control) | 否 | 否 | 容忍(全局最近) | 自动重定位 | **生产基底**(但类名已偏离狗上) |
| `waypoint_follower.py` | 1425 | 顺序段锁定(patrol_control) | **是** | **是**(use_route_speed) | **要求对齐否则 FAULT** | emergency→FAULT 停死 | 测试/README |
| `waypoint_follower_old.py` | 209 | 最早简版 | - | - | - | - | **死**(不在 entry_points) |

### 命令/运动输出节点 ×3(三种不同运动接口!)
| 文件 | 行 | 输出接口 | 安全急停 | 状态 |
|---|---|---|---|---|
| `unitree_safe_cmd_node.py` | 636 | `Request`(sport API)**或** Twist(`/cmd_vel`) | 有 | **生产/测试都用** |
| `unitree_cmd_node.py` | 152 | `Request`(Move 1008/StopMove 1003) | **无** | 裸命令节点(存疑) |
| `unitree_go_safe_cmd_node.py` | 312 | `unitree_go/SportModeCmd`(另一种低层接口) | 有 | 变体(SportModeCmd 路线) |

→ **运动接口就有三条**:`Request`/`/api/sport/request`、`SportModeCmd` 话题、Twist→UDP→SDK2 `SportClient`。生产只用最后一条(见 04)。

## 四、patrol_control.py 的真相(此前存疑,现已坐实)
- **不是死代码**:它是 **`waypoint_follower.py`(测试版)的控制库**(line_follow_command / segment_metrics / ordered_route_heading / corner 检测 / feedback_motion_scale …)。
- 但**生产跟随器(go2_2)完全不用它**,只用 `go2_course_control`。
- 安全节点只用它 3 个函数(限幅/侧向ROI/流龄)。
→ 结论:patrol_control 属于"顺序段锁定"那一套(测试链),和生产的"最近点+前视"是**两套并存的控制哲学**。

## 五、参数错配(SaaS ↔ 生产跟随器)
`go2_saas_agent.start_patrol_command` 计算了 `cornerAngleDeg / headingSlowAngleDeg / lineDeadband / minimumMovingSpeed / maxCorrectionAngleDeg / courseHeadingWindow …` 一大批参数,但**真正传给 `waypoint_follower_go2_2` 的只有 14 个**(v_base/max_vx/k_yaw/max_yaw_rate/lookahead/reach/goal/loop/search_window/turn_in_place/slow_down/stuck/relocalize/trace)。
→ 多算的那批是 **`waypoint_follower.py`(line_follow 那套)的参数**,在生产路径上**计算了但不消费**。云端下发那些参数**不会影响生产巡检行为**——一个极易误判的坑。

## 六、其它"乱"的来源
- **文档不可信**:README 自认过时(`README.md:3-6`);同一参数(如 `stop_distance`)代码默认 0.70 / README 0.40 / 生产 0.80。**一律以代码默认 + 实际启动串为准。**
- **`.bak_*` 泛滥**:仓库/备份里几十个带日期后缀的手改快照(`*.bak_20260716_lateral` / `*.bak_before_patrol_runs_log` …),是狗上"改前手动存档"的产物,无版本管理。
- **建图无编排**:`go2_loop_backend` 11 个工具全靠手动跑,没有脚本串起来(见 06);现场到底用哪条造图路不明。
- **两套定位模式并存**:`pcd`(route_relocalizer)与 `manual_anchor`(手动锚点),SaaS 按下发选。
- **狗上实跑副本**里连 `laserMapping.cpp`(58.9KB)也是改过的 FAST-LIO(疑似它写 `/dev/shm/go2_fastlio_latest_odom.txt` 供新鲜度门用,见 01 待坐实)。

## 七、给下一个开发者的"先做这几步"(仅建议,非本次执行)
1. **定基线**:确认生产以"仓库版 go2_2(WaypointFollowerGo22,1043行)"还是"狗上版(WaypointFollower,330行)"为准,统一之(并修 trace 外壳的类名取值)。
2. **收敛跟随器**:三套跟随器留一套,删/归档 `waypoint_follower_old.py`;明确 `waypoint_follower.py`(测试)是否还需要。
3. **对齐 README 与实际生产链**(README 现在指向测试链,严重误导)。
4. **清理参数错配**:SaaS 只算/传生产跟随器真正消费的参数。
5. **固化建图流程**:把 `go2_loop_backend` 的实际造图步骤写成一个脚本 + 说明。

## 八、附:本次梳理已逐行核实的范围
docs 00–09 覆盖:基础层、CSV 执行(生产 go2_2)、安全急停、运动输出、录制、pcd 建图、重定位、SaaS、4G。**未逐行**:三方 FAST_LIO/Livox 内部、`go2_loop_backend` 部分工具(scan_context/pose_graph/level/nav2)、SaaS 中段若干旁路脚本、相机(见 10)。这些已在各文档"留待坐实"标注。

# 90 · 乱象与代码分叉审计(给下一个开发者的"定位地图")

> 原则同 00:全部是代码事实。本篇不"改",只把"乱在哪、什么在跑、什么是死的、仓库和狗上差在哪"钉清楚,让人能快速定位。
>
> **本轮已按核验台账对磁盘源码逐条复核**(源码 + sha + 狗端 manifest + file:line),并给每条可证伪断言标注【源标签】。判定语义:✅坐实 / ⚠默认≠生产(已注生效值)/ ✳原文有误已更正 / 【推断-未验】。

## 〇、核验状态(本轮已核 / 仍无法直接验)

**源标签约定**(下文每个数值/断言尽量带一个):
- 【默认 code path:line】= 仓库源码里的 `declare_parameter` 默认值 / 类定义 / 行数。
- 【生产 saas path:line】= `go2_saas_agent.py` 装配出的实际启动串(`-p` 覆盖)。
- 【狗上 dog:证据】= 狗端 `remote_source` 副本 / manifest 佐证(sha 或 manifest 字段)。
- 【README对照】= README 示例值(常与生产不符)。
- 【推断-未验】= 基于源码逻辑的确定性推理,**非运行验证**。

**sha 已验(全仓仅两处有狗上副本可比)**:
- `waypoint_follower_go2_2.py`:仓库版 sha `009cb25b…`(1043 行,`WaypointFollowerGo22`) **≠** 狗上真跑版 sha `d205a596…`(330 行,`WaypointFollower`)。后者 == 狗端 manifest `controller_reference_sha256`(06/07 两次跑),坐实狗上跑的是 `remote_source` 那份 330 行版。**→ repo ≠ dog(sha 验)**。
- `FAST_LIO/src/laserMapping.cpp`:仓库版 `5fec8282…`(61919 字节) **≠** 狗上版 `e4cd05cb…`(60358 字节 = 58.9KB)。**→ repo ≠ dog(sha 验)**。

**【无狗上对照】(其余全部,不许默认等同狗上)**:`waypoint_follower_go2_2_trace.py` / `waypoint_follower.py` / `waypoint_follower_old.py` / `unitree_safe_cmd_node.py` / `unitree_cmd_node.py` / `unitree_go_safe_cmd_node.py` / `patrol_control.py` / `go2_course_control.py` / `go2_saas_agent.py` / `go2_sdk2_udp_receiver.cpp` / `cmd_vel_udp_sender.cpp` / `route_relocalizer.cpp` / 两个 `setup.py` —— 狗端 `remote_source` 只抓到 go2_2 follower 与 laserMapping 两份副本,**这些文件狗上一致性无法直接验证**,行数/类名/import/函数清单均按仓库源码核实。**唯一佐证**:`go2_saas_agent.py` 输出的狗端 manifest(06/07)证明 `-p` 硬编码值确实到达狗上(`k_yaw=0.900 / speed=0.5 / loop=pingpong / localization_mode=manual_anchor / stop_distance=0.80 / udp_sdk_vx_limit=0.500`),但 saas 脚本本体的狗上一致性仍无法直接验证。

**仍属推断/未验**:第一节"仓库版会崩溃"是代码逻辑【推断-未验】(非运行验证);`laserMapping` 写 shm 的**消费端**(新鲜度门,见 01)未验。

---

## 一、最重要的一件事:仓库 ≠ 狗上,且生产跟随器有三套

**狗上真跑的**【狗上 dog:`analysis/xunjian_20260725_shutdown_capture/previous_boot/remote_source/waypoint_follower_go2_2.py`,sha `d205a596…` == manifest】:
- **330 行,`class WaypointFollower`**(:26),**自包含**(import 仅 csv/math/time/rclpy/Node/Odometry/Twist,1–9 行,**不** import course_control / patrol_control)。

**仓库里的** `src/.../waypoint_follower_go2_2.py`【默认 code:67,sha `009cb25b…`】:
- **1043 行,`class WaypointFollowerGo22`**(:67),import `go2_course_control`(:20–26),是一次大重写并**改了类名**。

**后果**【推断-未验,由 sha 不等 + 类名不匹配两条硬事实支撑】:生产用 `waypoint_follower_go2_2_trace.py` 拉起【生产 saas:2061–2073 follower_cmd 用 follower_trace_script】,而 trace 外壳 `getattr(base_module, "WaypointFollower", None)`【默认 code trace.py:25;:22 `import waypoint_follower_go2_2 as base_module`】:
- 对**狗上版**(类名 `WaypointFollower`)→ 能实例化 ✅
- 对**仓库版**(类名 `WaypointFollowerGo22`,全文无独立 `WaypointFollower` 符号)→ getattr 返回 `None` → `raise RuntimeError("BASE_FOLLOWER_CLASS_MISSING…")`【默认 code trace.py:26–30】崩溃 ❌

→ **仓库里这条生产链是断的;仓库提交的跟随器从未在狗上原样运行过。**("会崩溃"是【推断-未验】;狗上跑的是 remote_source 那份含 `WaypointFollower` 的 330 行版,故可实例化——这是 sha 坐实的。)谁要改巡检逻辑,第一步必须先对齐"仓库 ↔ 狗上"到底以哪份为准。

## 二、"什么在跑" vs "README/新人会读到什么"

| | 生产真身(狗上 / SaaS 装配) | README / 测试 / 新人默认读到 |
|---|---|---|
| 跟随器 | `scripts/waypoint_follower_go2_2_trace.py` → `waypoint_follower_go2_2.py`【生产 saas:2073;trace.py:22】 | `src/.../waypoint_follower.py`【README对照 README.md:482】 |
| 控制设计 | 最近点 + 前视纯跟踪 + 课程纠偏【默认 code go2_2:784 compute_lookahead_index / :792 closest_route_projection / :809 use_straight_course_feedback】 | 顺序段锁定 + line_follow 横向纠偏【默认 code waypoint_follower.py:53 docstring "…in order without nearest-route reprojection" / :15 line_follow_command】 |
| 运动输出 | Twist→`/cmd_vel`→UDP→SDK2 `SportClient.Move`【生产 saas:2056 output_cmd_topic:=/cmd_vel;默认 code go2_sdk2_udp_receiver.cpp:80/219 sport_client.Move】 | Request→`/api/sport/request` sport API【默认 code unitree_cmd_node.py:45】 |
| 重定位 | route_relocalizer 预变换 CSV【生产 saas:1666–1668 "give it the already transformed runtime CSV";route_relocalizer.cpp 存在】 | follower 自己吃 route_transform_json【默认 code waypoint_follower.py:65】 |

> **新人读错文件、走错链路,几乎是必然的。** 这是"连定位都费劲"的直接原因。
> 注:上表右列(测试链诸文件)均**【无狗上对照】**,按仓库源码核实。

## 三、节点动物园(重复实现清单)

### 跟随器 ×3(node 名全叫 `waypoint_follower`,`ros2 node list` 分不清)
四份文件(含狗上版)节点名都是 `waypoint_follower`【默认 code go2_2:71 / waypoint_follower.py:56 / old:27;狗上 dog remote_source:28】。

| 文件 | 行 | 设计 | 用 patrol_control? | 吃 CSV 'v'? | 起点 | 脱轨 | 状态 |
|---|---|---|---|---|---|---|---|
| `waypoint_follower_go2_2.py`(仓库版) | 1043 | 最近点+前视(course_control) | 否(:20 无 patrol_control import) | 否 ¹ | 容忍(全局最近,:580 find_nearest_global) | 自动重定位(:648 relocalize / :866 stuck recovery) | **生产基底**(但仓库类名 `WaypointFollowerGo22` 已偏离狗上 `WaypointFollower`,见一) |
| `waypoint_follower.py` | 1425 | 顺序段锁定(patrol_control,:15) | **是** | **是**(:69 `use_route_speed=True`) | **要求对齐否则 FAULT**(:755 "route has no usable first segment") | emergency→FAULT 停死(:94/1002 emergency_route_deviation → :597 `state='FAULT'`) | 测试 / README |
| `waypoint_follower_old.py` | 209 | 最早简版 | - | - | - | - | **死**(不在 entry_points,setup.py:22–29 无它) |

¹ go2_2 的 `load_route`(:511)**加载了** CSV 'v' 字段,但 `control_cycle`(:863)实际取 `v_base / max_vx`,**不消费** 'v' → 故记"否"。【全部 默认 code:上述 file:line;三份文件均**【无狗上对照】**,仓库版 go2_2 更已 sha 验为 repo≠dog】

### 命令 / 运动输出节点 ×3(三种不同运动接口!)
| 文件 | 行 | 输出接口 | 安全急停 | 状态 |
|---|---|---|---|---|
| `unitree_safe_cmd_node.py` | 636 | `Request`(sport API,:164)**或** Twist(`/cmd_vel`,:166–167) | 有(:73 stop_distance / ROI) | **生产 / 测试都用**【生产 saas:2050–2051;README对照 README.md:419/466】 |
| `unitree_cmd_node.py` | 152 | `Request`(Move 1008 / StopMove 1003,:13–14) | **无 ²**(仅 :113 cmd_timeout→StopMove) | 裸命令节点(生产链未启动它) |
| `unitree_go_safe_cmd_node.py` | 312 | `unitree_go/SportModeCmd`(:10,另一低层接口) | 有(:48 stop_distance / ROI) | 变体(saas grep 命中 0,生产不启动) |

² "无安全急停"指**无障碍 / 点云急停**,仅有指令超时停(StopMove)。【全部 默认 code:上述 file:line;三份均**【无狗上对照】**】

→ **运动接口就有三条**:`Request`/`/api/sport/request`、`SportModeCmd` 话题、Twist→UDP→SDK2 `SportClient`。**生产只用最后一条**(follower→safe→cmd_vel_udp_sender→sdk2_receiver:219,见 04)。

## 四、patrol_control.py 的真相(此前存疑,现已坐实)
- **不是死代码**:它是 **`waypoint_follower.py`(测试版)的控制库**【默认 code patrol_control.py:line_follow_command:344 / segment_metrics:28 / ordered_route_heading:161 / corner_turn_angle:68 / ordered_upcoming_corner:81 / feedback_motion_scale:288】,被 `waypoint_follower.py:15` + 三个 unitree 节点共 **4 处 import**。
- 但**生产跟随器(go2_2)完全不用它**,只用 `go2_course_control`(go2_2 全文唯一 import 在 :20;且 course_control 也**仅被 go2_2** import)。
- 安全节点只用它 **3 个函数**【默认 code unitree_safe_cmd_node.py:15–19:`limit_planar_command`(限幅)/ `point_in_lateral_motion_roi`(侧向 ROI)/ `stream_receive_age`(流龄)】。

→ 结论:patrol_control 属于"顺序段锁定"那一套(测试链),和生产的"最近点+前视"是**两套并存的控制哲学**。(patrol_control / course_control 均**【无狗上对照】**,按仓库核实。)

## 五、参数错配(SaaS ↔ 生产跟随器)——含"算了不用 / 被硬编码覆盖"的旗舰坑

`go2_saas_agent.start_patrol_command` 计算了一大批参数(`cornerAngleDeg / headingSlowAngleDeg / lineDeadband / minimumMovingSpeed / maxCorrectionAngleDeg / courseHeadingWindow …`)【生产 saas:1532/1547/1565/1571/1598/1619】,但真正 `-p` 传给 `waypoint_follower_go2_2` 的只有一小撮。

**先纠两处此前不精确**:
1. **follower_cmd 实际有 15 个 `-p`**(此前文档写"14 个"**漏计了 `route_file`**)【生产 saas:2063–2071】。
2. 这批 `-p` **多数是硬编码字面量,不是云端算出来的值**,分三类:
   - **云端真算了并传进去的**:`v_base / max_vx`(= speed_arg)、`loop_mode`(= loop_arg),外加 `trace_file / route_file` 路径。
   - **`-p` 硬编码字面量**(与云端算的同名 `_arg` 无关):`k_yaw:=0.900 / max_yaw_rate:=0.450 / lookahead_distance / reach_distance / goal_distance / search_window / turn_in_place_angle / slow_down_angle / stuck_time / relocalize_distance`【生产 saas:2064 等】。
   - 把这些笼统说成"传入的 14 个"会**掩盖两点**:①它们是**硬编码**;②云端算的**同名 `_arg` 其实是死参**。

**多算不消费(死参)**:`cornerAngleDeg` 那批算进各自 `_arg`【生产 saas:1688/1691/1697/1698/1705/1711】,但每个 `_arg` 全文**引用计数 = 1(仅定义,从不使用)= 死参**;这些名字在 `waypoint_follower.py:71–86` 全声明,而 go2_2 里 grep 命中 0。→ 云端下发那批 line_follow 参数**不影响生产巡检行为**,一个极易误判的坑。

**旗舰坑:`k_yaw`(云端算了却被硬编码盖掉)**【⚠DEFAULT_VS_PROD,已坐实】:
- 代码默认 `0.6`【默认 code waypoint_follower_go2_2.py:83 `declare_parameter('k_yaw', 0.6)`】
- 云端 `k_yaw` 走 bounded_float,**默认 1.20**,算进 `k_yaw_arg`【生产 saas:1511–1516;:1684】——但 `k_yaw_arg` 全文**仅此 1 处引用 = 死参,从不喂给 follower**
- 生产 `-p` **硬编码 `k_yaw:=0.900`**【生产 saas:2064】盖掉一切
- **狗上实跑 `0.900`**【狗上 dog:manifest go2_2_k_yaw = 0.900,06/07 双跑皆 0.900】
- → **云端那个"能调 k_yaw"的旋钮(bounded 默认 1.20)对生产根本不生效**,狗上永远是硬编码 0.900。谁在 SaaS 面板上调 k_yaw 会以为生效,实则被 `-p` 覆盖。

**同型:`max_yaw_rate`**:follower `-p` 硬编码 `0.450`【生产 saas:2064】,代码默认 `0.30`【默认 code waypoint_follower_go2_2.py:84】;云端 `max_yaw_rate_arg` 只喂 safe / cmd_vel_sender,**不喂 follower**。

## 六、其它"乱"的来源
- **文档不可信**:README 自认过时【README对照 README.md:6(引用块 3–6)"…本 README 的部分'当前状态'描述早于该交接记录"】。同一参数 `stop_distance` 三值全对但要分清生效者【⚠DEFAULT_VS_PROD】:代码默认 `0.70`【默认 code unitree_safe_cmd_node.py:73】 / README 示例 `0.40`【README对照 README.md:424】 / 生产 `-p 0.80`【生产 saas:2057】 → **狗上实跑 0.80**(saas 覆盖,manifest stop_distance=0.80)。**一律以生产 `-p` 启动串 + 狗上 manifest 为准。**
- **`.bak_*`(纠正此前"泛滥"的过度断言)**【✳CORRECTED】:**当前工作树 / HEAD 为 0 个 `.bak`**(`find -iname '*bak*'` = 0、`git ls-files | grep .bak_` = 0、`orin_go2_fastlio_ws/backups` 目录不存在)。只有 **git 历史**里曾有 **47 个** `.bak_`(含所引两例 `go2_saas_agent.py.bak_20260716_lateral` / `…bak_before_patrol_runs_log`)。故 present-tense"仓库里泛滥"对当前 checkout 是**误导**;它们本就在 git 历史里,"无版本管理"表述亦不严谨 → 应表述为"**git 历史中曾有 47 个手改快照,现已从工作树移除**"。
- **建图无编排**:`go2_loop_backend` **11 个可跑工具**(setup.py:23–33 注册的 console_scripts:keyframe_saver / offline_keyframe_extractor / build_raw_map / scan_context_detector / pose_graph_optimizer / dynamic_map_filter / sliding_window_static_filter / export_registered_cloud_map / level_pcd / pcd_to_nav2_map / pcd_to_nav2_map_fast)全靠手动跑,没有脚本串起来(见 06)。**口径澄清**:目录里另有 **6 个未注册 `.py` 模块**(odom_to_tf_map / odom_to_tf_map_2d / odom_to_tf_map_level_2d / odom_to_tf_odom_level_2d / level_cloud_node / filter_keyframes_front_fov)→ 论"可跑工具"是 **11**,论"模块文件数"是 **17**。现场到底用哪条造图路不明。
- **两套定位模式并存**:`pcd`(route_relocalizer)与 `manual_anchor`(手动锚点),SaaS 按下发归一选一【生产 saas:621–631】。**狗上两次跑(06/07)都用 `manual_anchor`**【狗上 dog:manifest localization_mode = manual_anchor】。
- **狗上实跑副本里 `laserMapping.cpp`(58.9KB)是改过的 FAST-LIO**【狗上 dog:remote_source/laserMapping.cpp = 60358 字节,`e4cd05cb…`;:738–739 写 `/dev/shm/go2_fastlio_latest_odom.txt`】——**但这条 shm 写入不是狗上独有**:仓库版 `FAST_LIO/src/laserMapping.cpp`(`5fec8282…`,61919 字节)**同样含该写入**(:747–748)。即狗版与仓库版内容不同(sha 不等,repo≠dog),但 shm 快照改动**两边都在**;只是狗版与仓库版仍有其它差异。其**消费端**(新鲜度门,见 01)本次仍**未坐实**【推断-未验】。

## 七、给下一个开发者的"先做这几步"(仅建议,非本次执行)
1. **定基线**:确认生产以"仓库版 go2_2(`WaypointFollowerGo22`,1043 行)"还是"狗上版(`WaypointFollower`,330 行)"为准,统一之(并修 trace 外壳 :25 的类名取值)。
2. **收敛跟随器**:三套跟随器留一套,删 / 归档 `waypoint_follower_old.py`;明确 `waypoint_follower.py`(测试)是否还需要。
3. **对齐 README 与实际生产链**(README 现在指向测试链,严重误导)。
4. **清理参数错配**:SaaS 只算 / 传生产跟随器真正消费的参数;并把 `-p` 硬编码值(k_yaw / max_yaw_rate / stop_distance …)与云端可调项区分——**现在云端调 k_yaw / max_yaw_rate 是假旋钮**(被 `-p` 覆盖)。
5. **固化建图流程**:把 `go2_loop_backend` 的实际造图步骤写成一个脚本 + 说明。

## 八、附:本次梳理已逐行核实的范围
docs 00–09 覆盖:基础层、CSV 执行(生产 go2_2)、安全急停、运动输出、录制、pcd 建图、重定位、SaaS、4G【README对照 docs/understanding/ 00–09 一一对应;`10_camera_z1pro` 亦在】。**未逐行**:三方 FAST_LIO / Livox 内部、`go2_loop_backend` 部分工具(scan_context / pose_graph / level / nav2)、SaaS 中段若干旁路脚本、相机(见 10)。这些已在各文档"留待坐实"标注。

---

## 九、核验台账(claim → 证据 file:line → 判定)

> 判定:✅ = CONFIRMED(坐实);⚠ = DEFAULT_VS_PROD(默认≠生产,已注生效值);✳ = CORRECTED(原文有误已更正)。`dog` = 有狗上副本 sha 验证;未标 `dog` 者**【无狗上对照】**,按仓库源码核实。

| # | 断言 | 证据 file:line | 判定 |
|---|---|---|---|
| 1 | 狗上真跑 go2_2 = 330 行 `class WaypointFollower` 自包含 | remote_source/waypoint_follower_go2_2.py:26;wc=330;import 1–9 行无 course/patrol | ✅ dog(sha d205a596 == manifest) |
| 2 | 仓库 go2_2 = 1043 行 `WaypointFollowerGo22` import course_control | 仓库同名:67;:20–26;wc=1043;sha 009cb25b | ✅(repo ≠ dog,sha 验) |
| 3 | 生产用 trace.py 拉起,`getattr(module,"WaypointFollower")` | trace.py:25;:22;saas:2061–2073 | ✅ |
| 4 | 狗上版可实例化 / 仓库版取不到 → BASE_FOLLOWER_CLASS_MISSING 崩溃 | trace.py:26–30;仓库无独立 `WaypointFollower` 符号 | ✅【推断-未验:非运行验证】 |
| 5 | 仓库生产链断,提交版从未在狗上原样跑 | sha 009cb25b ≠ d205a596 + 类名不匹配 | ✅(mixed) |
| 6 | 生产 trace → go2_2;README / 测试 waypoint_follower | saas:2073;trace.py:22;README.md:482/466 | ✅ |
| 7 | 控制:生产 最近点+前视+课程纠偏;测试 顺序段+line_follow | go2_2:784/792/809;waypoint_follower.py:53/15 | ✅ |
| 8 | 运动:生产 Twist→/cmd_vel→UDP→SDK2 Move;测试 Request→/api/sport/request | sdk2_receiver:80/219;saas:2056;unitree_cmd_node.py:45 | ✅ |
| 9 | 重定位:生产 route_relocalizer 预变换;测试 follower 吃 json | saas:1666–1668;route_relocalizer.cpp;waypoint_follower.py:65 | ✅ |
| 10 | 三套(含狗上共四份)跟随器 node 名全 `waypoint_follower` | go2_2:71 / waypoint_follower.py:56 / old:27 / dog remote:28 | ✅ dog |
| 11 | go2_2:1043 / course_control / 不用 patrol / 不消费 CSV 'v' / 全局最近 / 自动重定位 / 生产基底 | :20 / 511 / 863 / 580 / 648 / 866 | ✅ |
| 12 | waypoint_follower.py:1425 / patrol_control / use_route_speed / 对齐否则 FAULT / emergency→FAULT | :15 / 69 / 597 / 755 / 94 / 1002 | ✅ |
| 13 | waypoint_follower_old.py:209 / 死(不在 entry_points) | 209 行;setup.py:22–29 无它 | ✅ |
| 14 | unitree_safe_cmd_node.py:636 / Request 或 Twist / 有急停 / 生产测试都用 | :164 / 166–167 / 73;saas:2050–2051;README:419/466 | ✅ |
| 15 | unitree_cmd_node.py:152 / Move 1008 StopMove 1003 / 无障碍急停 / 生产未启 | :13–14 / 45 / 113 | ✅ |
| 16 | unitree_go_safe_cmd_node.py:312 / SportModeCmd / 有急停 / 变体 | :10 / 48;saas grep = 0 | ✅ |
| 17 | 运动接口三条,生产只用 Twist→UDP→SDK2 | unitree_cmd/safe(Request)/ go_safe(SportModeCmd)/ sdk2:219 | ✅ |
| 18 | patrol_control 非死代码,是 waypoint_follower.py 控制库 | patrol_control.py:344/28/161/68/81/288;import ×4 | ✅ |
| 19 | go2_2 全不用 patrol_control 只用 course_control;安全节点只用 3 函数 | go2_2:20;unitree_safe:15–19 | ✅ |
| 20 | saas 多算 line_follow 参数,真传少数,死参 `_arg` 引用 = 1 | saas:1532… / 1688–1711;waypoint_follower.py:71–86;go2_2 命中 0 | ✅(补:实为 15 个 -p + 硬编码字面量,见五) |
| 21 | k_yaw:代码默认 0.6 / 云端 bounded 默认 1.20(死参 k_yaw_arg)/ 生产 -p 0.900 / 狗上 0.900 | 默认 go2_2:83;saas:1511–1516 / 1684 / 2064;manifest 0.900 | ⚠ dog(狗上实跑 0.900;云端 1.20 死) |
| 22 | README 自认过时 | README.md:6(引用块 3–6) | ✅ |
| 23 | stop_distance:默认 0.70 / README 0.40 / 生产 0.80 | unitree_safe:73;README:424;saas:2057 | ⚠ dog(狗上实跑 0.80) |
| 24 | `.bak_*` 泛滥 | 工作树 / HEAD = 0;backups 目录不存在;git 历史 = 47(含 2 例) | ✳ CORRECTED(现工作树 0 个) |
| 25 | go2_loop_backend 11 工具手动跑 | setup.py:23–33 = 11 console_scripts;另 6 未注册模块 | ✅(11 可跑 / 17 模块) |
| 26 | 两套定位 pcd / manual_anchor,SaaS 选 | saas:621–631;manifest = manual_anchor | ✅ dog(狗上 manual_anchor) |
| 27 | laserMapping.cpp 改过 FAST-LIO,写 shm | dog remote:60358 字节 / 738–739;仓库 5fec8282 / 61919 / 747–748 | ✅ dog(repo≠dog;shm 仓库版也在;消费端未验) |
| 28 | docs 00–09(+10 相机)覆盖 | docs/understanding/ 00–10 一一对应 | ✅ |

# 03 · 安全节点:限速 + 雷达点云急停(`unitree_safe_cmd_node`)

> 原则同 00。核心文件:
> `src/go2_fastlio_patrol/go2_fastlio_patrol/unitree_safe_cmd_node.py`、
> 依赖 `patrol_control.py`(`limit_planar_command` / `point_in_lateral_motion_roi` / `stream_receive_age`)。
> 定时发布频率:默认 40Hz【默认 code:53】/ 生产 20Hz【生产 saas:2053】。

## 核验状态
本轮已对**磁盘仓库源码逐条核验**(unitree_safe_cmd_node.py / patrol_control.py / go2_saas_agent.py / 两个 follower / unitree_cmd_node.py / FAST_LIO yaml),调用链、行号、阈值、兜底优先级、"生产只发 `/cmd_vel`"结论均与真实代码一致。下面每个可证伪数值都带【源标签】:

- **【默认 code:NN】** = `unitree_safe_cmd_node.py` 第 NN 行 `declare_parameter` 的代码默认值。
- **【生产 saas:NN】** = `orin_go2_fastlio_ws/scripts/go2_saas_agent.py` 第 NN 行的 `-p` 启动串装配值。
- **【ctrl:NN】** = `patrol_control.py`。
- **【狗上 manifest】** = `analysis/xunjian_20260725_shutdown_capture/runs/xunjian-20260725-0X/manifest.txt`(真机落盘)。
- **【狗上 dog】** = `analysis/.../previous_boot/remote_source/…` 真跑副本。
- **【推断-未验】** = 逻辑推断,无独立证据。

**必须先讲清的证据边界(否则会误读所有"生产值"):**
1. **安全节点三份源码(`unitree_safe_cmd_node.py` / `patrol_control.py` / `go2_saas_agent.py`)都【无狗上对照】** —— 不在 4 份狗上源码副本内,狗上是否与仓库一致**无法验证**。所以本文所有【生产 saas】阈值都是 **repo-only**(仅 repo 的 `go2_saas_agent.py -p` 装配可证)。
2. **狗上 manifest 只佐证 4 个字段**:`speed=0.5`(→ `max_vx`)、`command_vy=0.000`(→ `max_vy`)、`udp_sdk_vx_limit=0.500`、`cloud_freshness_source=local_receive_time`。**除这 4 项外,`publish_rate` / `roi_*` / `stop_distance` / `min_stop_points` / `cmd_timeout` / `cloud_timeout` 等阈值狗端实际生效值均无独立对照** —— 本文以"生产"口吻陈述它们,依据是 saas `-p`,读者需知狗端无法二次确证。
3. **生产跟随器 repo≠dog(sha 已验)**:狗上真跑的是 330 行 `class WaypointFollower`(sha `d205a596`),不是仓库里 1043 行的 `class WaypointFollowerGo22`(sha `009cb25b`)。涉及跟随器的结论按狗上 330 行版为准(见第六节)。

## 一、它在链路里的位置
跟随器只管"往哪走",**安全节点是狗和运动执行之间的最后一道闸**:接住跟随器的速度,限幅、按雷达点云判急停、按流超时兜底,再输出给运动层。

- **输入**:`/patrol_cmd`(Twist,来自跟随器【默认 code:45,168】)+ `/cloud_registered_body`(PointCloud2,来自 FAST-LIO,QoS best_effort/depth=2【默认 code:169-179】)。`pointcloud_topic` 生产值 `=/cloud_registered_body`【生产 saas:2055】**恰等于默认值**【默认 code:46】。
- **输出(二选一,`publish_move:250-259`【默认 code:250-259】)**:
  - `output_cmd_topic` 非空(**生产=`/cmd_vel`**【生产 saas:2056】,狗上无对照)→ 发 **Twist** 到该话题(→ UDP 运动桥),`twist_pub` 由 `output_cmd_topic` 非空创建【默认 code:166-167】。
  - `output_cmd_topic` 为空(**测试**)→ 发 **unitree_api `Request`**(Move,`api_id=1008`【默认 code:22,238】,parameter=JSON`{x,y,z}`【默认 code:242-246】)到 `/api/sport/request`。
  - ✅ **结论:生产只发 `/cmd_vel`,不发 sport API**;sport API 仅测试链用。总览"双发悬念"就此关闭。**注意**:该结论依赖 `output_cmd_topic:=/cmd_vel` 这条 saas `-p`,狗上无 manifest 对照,分支逻辑本身【默认 code:250-259】已核实。

## 二、接指令:限幅(`cmd_callback:208`)
每收到一条 `/patrol_cmd`,用 `limit_planar_command`【ctrl:420】把 `vx/vy/yaw_rate` 各自 clamp 到 `±max_vx / ±max_vy / ±max_yaw_rate`【ctrl:433-441】,存为 `last_*` 并记 `last_cmd_time`【默认 code:208,216-228】。**注意:安全节点自己不算控制,只限幅+兜底透传。**
> `max_vy` **生产=0.000**【生产 saas:2052】,且**狗上 manifest 佐证 `command_vy=0.000`** —— 这是少数有狗上对照的字段;它直接决定侧向 ROI 恒不触发(见三节)。`max_vx` 由狗上 manifest `speed=0.5` 佐证。

## 三、雷达点云判障(`process_cloud_message:302`)——回答"雷达怎么做急停"
- 处理限速到 `max_cloud_process_rate` **20Hz**【默认 code:79;saas 未设 → 运行时取默认】【默认 code:302,325-328】。
- 直接解析 `PointCloud2` 原始字节(`struct.unpack` float32 x/y/z【默认 code:349-351】),**每 `point_skip` 个点取一个**降负载 —— `point_skip` **默认 2**【默认 code:78,346;⚠️ 不在任何 saas `-p` 串,原文标"生产2"有误,来源是代码默认】。
- **前方 ROI(盒子)**:`in_roi` 判点是否落在 `x∈[roi_x_min,roi_x_max], y∈[roi_y_min,roi_y_max], z∈[roi_z_min,roi_z_max]`(机体系,x 向前)【默认 code:281-286】。
  - **默认**:x[0.35,1.20] y[-0.45,0.45] z[0.25,0.90]【默认 code:59-64】。
  - **生产**:x[0.35,1.50] y[-0.30,0.30] z[0.30,0.90]【生产 saas:2058-2059,狗上无对照】。
  - (注:`roi_x_min`=0.35、`roi_z_max`=0.90 默认=生产;其余 4 项默认≠生产,生产盒子更长更窄更高。)
  - 落框计 `roi_count`;其中 `x ≤ stop_distance`(**默认 0.70**【默认 code:73】/ **生产 0.80m**【生产 saas:2057】)的计 `stop_count`;记 `nearest_x`【默认 code:360-362】。
- **侧向 ROI(`point_in_lateral_motion_roi`)**:⚠️ **正常巡检时是死的**——它开头 `if abs(vy)≤deadband: return False`【ctrl:444,459-460】,`deadband=0.02`【默认 code:66 / ctrl:449;代码默认,不在 `-p`】;而 `last_vy` 恒 0(`max_vy` 生产=0.000【生产 saas:2052】+ 狗上 manifest `command_vy=0.000` 佐证 + 跟随器发 vy=0),故侧向计数恒 0、从不触发【默认 code:364-378】。只有当有横移指令时才按 vy 符号选左/右侧检测。
- **判危险**:`unsafe = stop_count ≥ min_stop_points 或 lateral_count ≥ lateral_min_stop_points`【默认 code:386-389】。
  - `min_stop_points`:**默认 12**【默认 code:75】/ **生产 15**【生产 saas:2057】。
  - `lateral_min_stop_points`:**默认 12**【默认 code:71;saas 未设 → 运行时取默认 12】(原文写"12"无生产标签,正确)。因侧向 ROI 恒不触发,此阈值实际上不起作用。
- **去抖(关键)**:连续 `stop_frames` 帧危险 → 置 `obstacle_stop=True`(**1 帧就停,快**);置位后需连续 `clear_frames` 帧无危险 → 才解除(**恢复保守**)【默认 code:398-399,407-408】。
  - `stop_frames`:**默认 1**【默认 code:76;⚠️ 不在 saas `-p`,原文标"生产1"有误,来源是代码默认】。
  - `clear_frames`:**默认 5**【默认 code:77;saas 未设 → 运行时取默认 5】。

## 四、定时输出与兜底(`publish_safe_cycle:431`,每 1/publish_rate 秒)
`create_timer(1.0/publish_rate)` 触发【默认 code:180,413-424,431】。按优先级,任一命中就输出零速 `Move(0,0,0)`【默认 code:463-491】:
1. **startup_interlock**:设了 `startup_enable_file` 且文件还没出现 → 停(开机联锁,等使能文件)【默认 code:156,433-438,463-465】。
   > ⚠️ **生产链下此兜底恒不触发**:saas `safe_cmd` **未设 `startup_enable_file`** → 取默认 `''`【默认 code:57;saas 未设】→ `motion_interlock_released` 开机即 `True`【默认 code:156】。也就是说,**开机联锁在生产下等于关闭**,这条优先级第 1 的兜底在真机上形同虚设。(机制本身代码正确,只是生产未启用。)
2. **cmd_timeout**:距上次 `/patrol_cmd` > `cmd_timeout`(**默认 0.5s**【默认 code:54】=**生产 0.5s**【生产 saas:2054】)→ 停(**跟随器挂了/卡了,狗自动停**)。
3. **cloud_timeout**:距上次点云 > `cloud_timeout`(**默认 1.0s**【默认 code:55】=**生产 1.0s**【生产 saas:2054】)→ 停(**雷达/FAST-LIO 断流,狗自动停**)。
4. **cloud_recovery**:点云超时恢复后,需连续 `cloud_recovery_frames`(**默认 5**【默认 code:56;saas 未设 → 运行时取默认 5】)帧新鲜才放行,其间 → 停。
5. **obstacle**:`obstacle_stop=True` → 停。
否则透传限幅后的 `last_vx/vy/yaw_rate`【默认 code:491+】。
> 流新鲜度用**本地接收时间**判(`stream_receive_age`【ctrl:318】,`now - last_receive`),不信消息头 stamp(`:186-191` 明说 header stamp 仅诊断)【默认 code:186-191,443-447】。**狗上 manifest `cloud_freshness_source=local_receive_time` 佐证**(少数有狗上对照的字段之一)。
>
> (`cmd_timeout` / `cloud_timeout` 生产=默认,是巧合相等;但仍以 saas `-p` 为运行时依据,狗上无 manifest 对照。)

## 五、安全哲学小结
- **正向**:只有前方盒子里近处点够多才停(点数阈值抗噪,生产 `min_stop_points=15`【生产 saas:2057】),停得快(`stop_frames=1`【默认 code:76】)、恢复慢(`clear_frames=5`【默认 code:77】)。
- **兜底**:指令断流、点云断流一律零速——**故障默认停**。**但"开机未使能"这条在生产实为空转**(`startup_enable_file` 未配,见四·1)。
- **盲区**:
  - 侧向避障事实上不生效(`last_vy` 恒 0,见三节)。
  - 高度 ROI 下限 **生产 0.30m**【生产 saas:2059】(默认 0.25m【默认 code:63】)会忽略很矮的障碍。
  - 雷达盲区 **0.5m**(FAST-LIO `blind`)内无点【FAST_LIO/config/mid360.yaml:22、go2_mid360s.yaml:26 均为 0.5】。⚠️ **FAST_LIO config 无狗上副本**,Go2 用 Livox MID360、加载 mid360 系配置合理,但狗上具体加载哪份 yaml **无法从本盘独立确证【推断-未验】**。

## 六、`patrol_control.py` 的现状(顺带审计)
该文件是一大堆平面控制 helper(`line_follow_command`/`heading_drive_command`/`ordered_route_heading`/`corner_turn_angle`/`feedback_motion_scale`…),但:

- **生产跟随器与 `patrol_control` 的关系(已按狗上真跑版修正)**:
  - "**完全不用 `patrol_control`**"—— ✅ 对 **repo 版(1043行)与狗上 330 行版都成立**。
  - 但原文"**只 import `go2_course_control`**"**仅对 repo 版成立**:repo `waypoint_follower_go2_2.py`(sha `009cb25b`,`class WaypointFollowerGo22`)确从 `.go2_course_control` import【repo :20-26】。
  - **狗上真跑版**是另一个文件:330 行 `class WaypointFollower`(sha `d205a596`【狗上 dog: previous_boot/remote_source/waypoint_follower_go2_2.py:2-9】,manifest `controller_reference_sha256=d205a596` 佐证【狗上 manifest】),**既不 import `go2_course_control` 也不 import `patrol_control`**,imports 仅 `csv/math/time/rclpy/Node/Odometry/Twist`。→ **狗上跟随器完全不碰 `patrol_control` 与 `go2_course_control` 两套 helper**。
  - 狗上状态:`waypoint_follower_go2_2.py` **repo≠dog(sha 已验)**;仓库版是分析用参考,不是狗上真跑代码。
- **安全节点**只用其中 3 个:`limit_planar_command`、`point_in_lateral_motion_roi`、`stream_receive_age`【默认 code:15-19,精确无误】。
- 其余大量函数(整条 `line_follow_command` 直线跟踪方案)在**生产路径上无人调用**(生产跟随器 = 狗上 330 行版,不 import patrol_control)→ 属于另一套/更早的跟随设计遗留。**消费者已坐实(原文"疑似"可去)**:
  - `line_follow_command`(及 `corner_turn_angle`/`feedback_motion_scale`/`ordered_route_heading`)的实际使用者是 **`waypoint_follower.py`**【:16-23 import,:818/906/1050/1096 调用】—— 非生产跟随器,**无狗上对照**。
  - 原文"或 `unitree_cmd_node`"**对 `line_follow_command` 不成立**:`unitree_cmd_node.py` 只 `from .patrol_control import limit_planar_command`【:10】,不使用 `line_follow_command`。**无狗上对照**。

## 七、留待坐实
- `/cmd_vel` 之后:`cmd_vel_udp_sender` → UDP → `go2_sdk2_udp_receiver` → SDK2 如何变成狗腿动作(见 04)。
- ~~`patrol_control.line_follow_command` 等到底谁在用~~ **已坐实 = `waypoint_follower.py`**(见六节);仍待坐实的是 `waypoint_follower.py` 本身是否在任何链路上被拉起(它与生产跟随器 `waypoint_follower_go2_2` 同名族但不同文件),**该文件无狗上副本**。
- 安全节点三份源码(`unitree_safe_cmd_node.py`/`patrol_control.py`/`go2_saas_agent.py`)**均无狗上副本**:狗上是否与仓库一致、生产 `-p` 阈值狗端是否真的生效,待抓取狗上 `/orin_go2_fastlio_ws` 实体后二次核验。

## 核验台账
> claim → 证据 file:line → 判定。缩写:code=`unitree_safe_cmd_node.py`,ctrl=`patrol_control.py`,saas=`go2_saas_agent.py`,dog=`previous_boot/remote_source/…`,manifest=`runs/xunjian-20260725-0X/manifest.txt`。

| # | 断言 | 证据 file:line | 判定 |
|---|------|----------------|------|
| 1 | 核心文件 code,依赖 ctrl 的 `limit_planar_command`/`point_in_lateral_motion_roi`/`stream_receive_age` | code:15-19;ctrl:420,444,318 | ✅ CONFIRMED |
| 2 | 发布频率 默认40 / 生产20 | code:53(40.0);saas:2053(`-p publish_rate:=20.0`) | ⚠️ 默认≠生产,狗上无对照 |
| 3 | 输入 `/patrol_cmd`+`/cloud_registered_body`(best_effort/depth=2) | code:45-46,168,169-179;saas:2055 | ✅ CONFIRMED(pointcloud_topic 生产=默认) |
| 4 | 输出二选一,生产只发 `/cmd_vel`(publish_move) | code:250-259,166-167,22/238,242-246;saas:2056 | ✅ CONFIRMED(依赖 saas `-p`) |
| 5 | 限幅 `cmd_callback:208` → `limit_planar_command` clamp | code:208,216-228;ctrl:420,433-441 | ✅ CONFIRMED |
| 6 | `process_cloud_message:302` 限速 `max_cloud_process_rate`=20Hz | code:302,325-328,79;saas 未设 | ✅ CONFIRMED(代码默认,运行时=20) |
| 7 | `point_skip` 取点降负载,值=2 | code:78,346,349-351;saas grep=0 | ⚠️ 值对,但**"生产2"应改"默认2"** |
| 8 | 前方 ROI 盒子 生产 x[0.35,1.50] y[-0.30,0.30] z[0.30,0.90] | code:59-64,281-286;saas:2058-2059 | ⚠️ 默认≠生产(给的是生产值,对);狗上无对照 |
| 9 | `x≤stop_distance` 计 stop_count,生产0.80 | code:73,360-362;saas:2057 | ⚠️ 默认0.70/生产0.80 |
| 10 | 侧向 ROI 恒死:`abs(vy)≤deadband(0.02) return False`+last_vy恒0 | ctrl:444,459-460,449;code:66,364-378;saas:2052+manifest command_vy=0.000 | ✅ CONFIRMED |
| 11 | `unsafe = stop_count≥min_stop_points(生产15) 或 lateral_count≥lateral_min(12)` | code:386-389,75,71;saas:2057,grep lateral=0 | ⚠️ min_stop_points 默认12/生产15;lateral 默认12(-p未设) |
| 12 | 去抖 `stop_frames` 帧危险→停,`clear_frames` 帧清才解除 | code:398-399,407-408,76,77;saas grep stop_frames=0 | ⚠️ 逻辑/值对,但 stop_frames **"生产1"应改"默认1"**;clear_frames 默认5 |
| 13 | `publish_safe_cycle:431`,每 1/publish_rate 秒 | code:431,180,413-424 | ✅ CONFIRMED |
| 14 | 兜底优先级 startup→cmd_timeout(0.5)→cloud_timeout(1.0)→cloud_recovery(5)→obstacle→透传 | code:463-491,54,55,56;saas:2054 | ✅ CONFIRMED(cmd/cloud_timeout 生产=默认) |
| 15 | startup_interlock:设了 enable_file 且未出现→停 | code:156,433-438,463-465,57;saas grep startup_enable_file=0 | ⚠️ 机制对,但**生产未设 enable_file→恒不触发**,原文未注明 |
| 16 | 流新鲜度用本地接收时间(stream_receive_age),不信 header stamp | code:186-191,443-447;ctrl:318;manifest cloud_freshness_source=local_receive_time | ✅ CONFIRMED |
| 17 | 盲区:侧向不生效/ROI z下限0.30忽略矮障/FAST-LIO blind 0.5 | 见#10;saas:2059;mid360.yaml:22,go2_mid360s.yaml:26 | ✅ CONFIRMED(yaml 无狗上副本,加载哪份未验) |
| 18 | 生产跟随器 只 import go2_course_control,完全不用 patrol_control | repo:20-26;**dog:2-9(330行,sha d205a596,仅stdlib+rclpy+msgs)**;manifest sha=d205a596 | 🔧 CORRECTED:"不用 patrol_control"对两版都成立;"只 import go2_course_control"仅 repo 版;狗上330行版两者都不 import |
| 19 | 安全节点只用 patrol_control 中3个 | code:15-19 | ✅ CONFIRMED |
| 20 | 其余函数(line_follow_command 方案)生产无人调用,疑似 waypoint_follower.py 或 unitree_cmd_node | waypoint_follower.py:16-23,818/906/1050/1096;unitree_cmd_node.py:10 | 🔧 CORRECTED:消费者坐实=`waypoint_follower.py`;`unitree_cmd_node` 只用 `limit_planar_command`,"或 unitree_cmd_node"对 line_follow_command 不成立 |

**狗上对照状态汇总**:
- `unitree_safe_cmd_node.py` / `patrol_control.py` / `go2_saas_agent.py` / `waypoint_follower.py` / `unitree_cmd_node.py` / `go2_course_control.py` / `FAST_LIO/config/*.yaml` —— **【无狗上对照】**(不在狗上副本内,一致性未验)。
- `waypoint_follower_go2_2.py` —— **repo≠dog(sha 已验)**:repo `009cb25b`(1043行)≠ dog `d205a596`(330行),manifest 确认狗上跑 330 行版。
- `analysis/.../remote_source/waypoint_follower_go2_2.py` —— **repo==dog(即狗上真跑副本本体,sha `d205a596`)**。
- 狗上 manifest 仅佐证:`speed=0.5`(→max_vx)、`command_vy=0.000`(→max_vy)、`udp_sdk_vx_limit=0.500`、`cloud_freshness_source=local_receive_time`;其余安全阈值狗端无对照。

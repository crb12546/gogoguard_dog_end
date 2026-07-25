# 14 · 证据与诊断层(仓库版 saas/blackbox 的"飞行记录仪";狗端 2026-07-25 实测仅 performance + rosbag)

> 原则同 00。核心文件(均已逐行读)。真实路径为 `orin_go2_fastlio_ws/scripts/`,文档沿用 GO2_WS 工作区相对写法 `scripts/`(snapshot 自身 `SOURCE_CANDIDATES` 亦用 `scripts/` 前缀,非错误)【README对照】:
> `scripts/go2_experiment_telemetry.py`、`scripts/go2_experiment_snapshot.py`、
> `scripts/patrol_performance_monitor.py`、`scripts/go2_lio_trace_recorder.py`。
> 消费者 `scripts/go2_experiment_audit.py`(停止时的大审计)见 §五。

## 核验状态(本轮 2026-07-25)
- **对磁盘仓库源码逐条核过**:四个记录器 + `go2_experiment_audit.py` + `go2_saas_agent.py` + `route_recording_blackbox.py` 的订阅话题、限速/QoS、记录字段、就绪/健康/停止标志、系统/进程指标、快照代码取证、审计消费链——**逐行对得上,几乎无硬错**。修正项集中在少数措辞(见 §二/台账)。
- **狗上无源码对照**:上述脚本**均不在狗上那 4 份源码副本内**,源码是否与仓库一致【无狗上对照】。
- **狗端运行证据**仅来自 2026-07-25 两次真跑 manifest(runs 06/07):
  - **只有 `performance_monitor` + `rosbag` 经狗端证实运行**【狗上 dog:manifest:33-36】。
  - **telemetry / snapshot / audit / follower-trace 在狗端 manifest 完全缺席**(无 `experiment_telemetry*` / `system_snapshot_start/end` / `follower_control_trace` 行),且 controller 非 trace 包装版 → **狗上跑的是不含遥测/快照层的旧版 saas**【狗上 dog:manifest:14-16 + 缺失行】。
- 结论:本篇除 `performance_monitor` 外的"每次巡检/录制都产"叙述,**均为仓库代码事实、狗端未证实**。下文逐节标源。

### 文件 × 狗上状态
| 文件 | 狗上状态 |
|---|---|
| `go2_experiment_telemetry.py` | 【无狗上对照】+ manifest 无 telemetry 行,未见运行证据 |
| `go2_experiment_snapshot.py` | 【无狗上对照】+ manifest 无 snapshot 行,未见运行证据 |
| `patrol_performance_monitor.py` | 【无狗上对照(源码)】,但**运行存在性经 manifest 证实** |
| `go2_lio_trace_recorder.py` | 【无狗上对照】+ 任何链路均未拉起,无运行证据 |
| `go2_experiment_audit.py` | 【无狗上对照】+ 依赖 telemetry/snapshot 产物(狗上缺席)→ 无运行证据 |
| `go2_saas_agent.py` | **repo≠dog**(manifest 字段揭示,非脚本 sha 直验):狗跑旧版 |
| `route_recording_blackbox.py` | 【无狗上对照】+ 7/25 为巡检(patrol)非录制,无对应证据 |

## 一、为什么有这一层
巡检/录制在户外一次性发生,出了问题要能事后复盘。所以**(仓库版)** 每次运行都并行拉起一组**低观测负载**的记录器,把"传感器、控制、系统资源、以及当时到底跑的哪份代码"全留证。**关键设计:热路径记录器一律不订阅点云**(在 Python 里反序列化整帧点云会改变它要测的时序)——telemetry 全文无 `PointCloud2`、perf 纯 procfs/sysfs 无 ROS、snapshot `observer_policy.raw_pointcloud_subscriber=False`【默认 snapshot.py:504-510】。
> ⚠️ 狗端 caveat:此"每次运行都拉起一组记录器"仅对**仓库版 saas/blackbox** 成立。狗上旧版 saas 只拉 `performance_monitor` + `rosbag`,不含 telemetry/snapshot/audit(见 §三)。

## 二、四个记录器

### `go2_experiment_telemetry.py`(跨层遥测 → JSONL)　【无狗上对照;manifest 无 telemetry 行,狗端未见运行】
- 订阅:`/Odometry`【默认 telemetry.py:187】、`/livox/imu`【:193】、`/lf/sportmodestate`【:204】、`/lf/lowstate`【:210】、无线手柄(`/lf/wirelesscontroller` 和 `/wirelesscontroller` **两名都订**,均映射 `kind='wireless'`【:215/228,213-235】)、`/patrol_cmd`【:238】、`/cmd_vel`【:248】、`/api/sport/request`【:258】。
- **每流限速**(`QoSProfile` KEEP_LAST / depth=1 / BEST_EFFORT【默认 telemetry.py:180-184】,另有 periods 节流【:165-178,306-311】),记:
  - **位姿+速度+协方差**:`pose`/`twist`/`*_covariance_diagonal`【默认 telemetry.py:360-367】;
  - **sport**:足力 `foot_force` / 足位 `foot_position_body` / 步态 `gait_type` / `range_obstacle` / 体 IMU。**⚠️ 更正:sport 无"电机温"**——其 `temperature` 来自 `imu_state`(**体 IMU 温度**,经 `imu_state_dict`【默认 telemetry.py:108】),电机温不在此流【默认 sport_payload:387-403,无 motor temperature】;
  - **LowState 全量**:12 电机 `q/dq/tau_est/temperature_c`(**电机温在这里**)【默认 telemetry.py:408-417】、BMS `soc/current/cell_vol`【:438-443】、体 IMU【:435】;
  - **手柄摇杆**:`lx/ly/rx/ry/keys`【默认 telemetry.py:449-456】。
- 标志:`recorder_start`【:268】 / `recorder_ready`(必需流 odom/livox_imu/sport/low 到齐【:155-160】;recording profile 还需手柄输入 wireless/wireless_raw【:161-164,330-335,525-538】)【:541】 / `recorder_health`(每秒 received/written/receive_age_s,timer=1.0s)【:266,501,503-521】 / `recorder_stop`【:576】。
  - → 这些标志正是黑盒/saas 启动时 `wait_for` 的对象【生产 saas.py:2237(recorder_ready)+2271(PERF_MONITOR_START);blackbox.py:925 + 938】。**仅对仓库版成立**;狗上旧版 saas 不拉 telemetry,无此等待。
- profile:`recording` vs `patrol`【默认 argparse choices telemetry.py:590-592,default='recording'】。
  - **默认≠生产**:黑盒传 `--profile recording`【生产 blackbox.py:883】、saas 巡检传 `--profile patrol`【生产 saas.py:2030】。
  - **狗上实际**:7/25 是巡检(patrol),但**旧版 saas 根本未拉起 telemetry**,故该 profile 在狗上不生效。

### `go2_experiment_snapshot.py`(边界快照,不连续)　【无狗上对照;manifest 无 snapshot 行,狗端未见运行】
- **拍照时机(默认≠生产,分链不同)**:
  - **录制黑盒链** start / stop / failure **各一次**【生产 blackbox.py:859/1168/1055;phase choices=start/stop/failure/manual snapshot.py:519】;
  - **saas 巡检链只 start + stop,无 failure 相**【生产 saas.py:2036(start)+2501(stop)】;
  - 第 4 个 `manual` 相未被这两调用方使用。
- 记 platform/boot_id/uptime/loadavg/kernel cmdline【默认 snapshot.py:479-484】、**nvpmodel(Jetson 功耗档)**(读状态文件 + 跑 `nvpmodel -q --verbose`)【:485,368】、CPU 调速器/频率【:199-206】、内存【:491】、温度【:492】、PSI 压力(cpu/memory/io)【:495-498】、中断【:494】。
- **代码身份铁证**:对一长串源码/可执行/配置(`base_bringup.sh`、`waypoint_follower_go2_2.py`、`unitree_safe_cmd_node.py`、`cmd_vel_udp_sender.cpp`、`go2_sdk2_udp_receiver.cpp`、`go2_mid360s.yaml`、`laserMapping.cpp`、install 产物…)记 **sha256 + size + mtime_ns**【默认 SOURCE_CANDIDATES:43-75 / EXECUTABLE_CANDIDATES:77-95 / file_identity:145-150】;对关键进程记 pid/command/executable/**executable_sha256**/start_ticks/cpu_allowed_list/scheduler/nice【默认 process_snapshot:298-312,KEY_PROCESS_PATTERNS:27-32】。
- 并发跑 ROS CLI 探针(`ThreadPoolExecutor` max_workers=4【:444】):node/topic list、关键话题 QoS、`/laser_mapping` 的 param dump【:432】、以及 **`/livox_lidar_publisher`**(文档旧写"/livox"是简写,真实节点名如此)的 param dump【默认 snapshot.py:436】。
- **环境变量:并非"完全不 dump"**。⚠️ 更正:docstring 自称因 SaaS token 不 dump env【snapshot.py:7】,但 `build_snapshot` 实际输出 **6 键安全白名单 `SAFE_ENV_KEYS`**(`ROS_DOMAIN_ID` / `RMW_IMPLEMENTATION` / `CYCLONEDDS_URI` / `FASTRTPS_DEFAULT_PROFILES_FILE` / `GO2_WS` / `GO2_SDK_IF`)【默认 snapshot.py:34-41,486-489】。精确说法:**不 dump 全量/含 token 环境,仅取安全白名单**(docstring 与实现有出入,以实现为准)。

### `patrol_performance_monitor.py`(每秒系统/进程画像)　【无狗上对照(源码);运行经 dog manifest 证实:`performance_monitor_interval=1.0` / `performance_log=performance_monitor.log`】
- 纯 procfs/sysfs【默认 docstring:4】:系统 & 每核 CPU%【:689/690】、CPU 频率【:695】、**GPU/EMC**【:196-210】、**hwmon INA3221 功耗轨(整机功耗)**【:228-267】、温度【:713】、load【:698】、procs_running/blocked【:703-704】、上下文切换【:705】、PSI【:709-711】、内存【:712】、磁盘 I/O【:720】、网络速率 + 丢包/错误 delta【:364-410】、**UDP 计数**【:726】,以及各关键进程 CPU%/RSS/线程/末核/亲和/上下文切换【:727-738】。
  - **UDP 计数**读 `/proc/net/snmp` Udp 段【默认 perf.py:413-429】——这是**系统级全量 UDP**,含但不限于 cmd_vel 运动路;文档旧写"对应 cmd_vel UDP 运动路"是解读,系统计数不区分通道【推断-未验】。
- 标志:`PERF_MONITOR_START`【:633】 / `PERF_SAMPLE`【:741】 / `PERF_MONITOR_STOP`【:756】。
- 用途:把控制时序尖峰和系统饱和对齐着看。**这是狗端唯一经证实在跑的证据记录器。**

### `go2_lio_trace_recorder.py`(FAST-LIO 输入输出 CSV,独立诊断工具)　【无狗上对照;任何链路均未拉起,狗端无运行证据】
- 订 odom / cloud / cloud_body / **livox raw** / imu → 分别写 5 个 CSV【默认 subscriptions lio_trace.py:100-117;CSV:61-91】。**⚠️ 默认状态**:raw livox 需 `--include-raw-livox`(`action=store_true`,默认 **False → 默认不录**)【:290】,imu 默认开【:291】;odom/cloud/cloud_body 常订(此为唯一订点云者,故非热路径【:101-102】)。
- 检 `TOPIC_GAP`/`STAMP_GAP`/`ODOM_JUMP`/`TOPIC_STALE`/`CLOUD_EMPTY` 事件【:135/137/165/273/198】。
- **非默认巡检链的一部分**:全仓 `grep lio_trace_recorder` 除自身文件外无任何 saas/blackbox/launch 引用,按需单跑【默认:grep 证实】。

## 三、和运行链的关系(默认 / 生产 / 狗上,三层都写)

**仓库版(源码事实,08 saas + 13 blackbox)**:
- saas(08)启动:detached 拉起 telemetry【生产 saas.py:2225】+ performance【:2259】+ rosbag【:2357】,各等 ready(`recorder_ready`:2237 / `PERF_MONITOR_START`:2271);**停止时跑 audit**【:2520】。manifest 会写 `experiment_telemetry`【:1860】/`experiment_telemetry_profile`【:1861】/`system_snapshot_start`+`_end`【:1862-1863】/`follower_control_trace`【:1857】,且 controller=`deployed_go2_2_nearest_lookahead_unchanged`、executable=`waypoint_follower_go2_2_trace.py`、带 `controller_source_sha256` + `controller_trace_wrapper_sha256`【:1829-1833】。
- blackbox(13)录制:同理拉 telemetry【:881】+ performance【:895】+ wait【:918-944】,snapshot 三相【:859/1055/1168】。

**狗上(生产实测,2026-07-25 runs 06/07 manifest)** —— **优先信 manifest**:
- **只有** `performance_monitor_interval=1.0` / `performance_log=performance_monitor.log`【狗上 dog:manifest:33-34】+ `rosbag_profile=control_light` / `rosbag_topics=/Odometry /patrol_cmd /cmd_vel /api/sport/request /tf /tf_static`【狗上 dog:manifest:35-36】。
- **完全没有** `experiment_telemetry*` / `system_snapshot_start/end` / `follower_control_trace` 行。
- controller=**`go2_2_enhanced_nearest_lookahead`**、executable=**`waypoint_follower_go2_2`**(**非 trace 包装版**)、只有单个 `controller_reference_sha256=d205a596…`【狗上 dog:manifest:14-16】。
- → **狗上跑的是旧版 saas**:遥测/快照/audit/trace 层整套缺席。**狗端实际生效的证据层 = `performance_monitor` + `rosbag` 二者而已。**

**产物落点**:`patrol_logs/runs/<日>/xunjian-…/`【狗端证实:`follower_route=…/patrol_logs/runs/20260725/xunjian-20260725-07/route_runtime.csv` dog:manifest:5】;`patrol_logs/recordings/…` 为黑盒(录制)侧【无狗上对照:7/25 是巡检非录制】。二者再被同步进 `analysis/`(仓库里那些海量日志的来源)。

## 四、这层告诉我们的工程取向
**(仓库版)** 整套系统对"**可复现、可取证**"极其执着:每次跑都锁定代码 sha、进程身份、FAST-LIO 会话、系统资源、全量遥测。这也是"乱"的另一面——**大量精力花在诊断/证据基建上**(experiment_*/performance/telemetry/blackbox/snapshot 合计数千行:telemetry 637 + snapshot 552 + audit 2591 + performance 763 + blackbox 1429 ≈ 5972 行【默认 wc -l】),控制本体反而只是其中一小块。
> ⚠️ 但**狗端现役旧版 saas 尚未接入这套遥测/快照/审计**,只留 `performance_monitor` + `rosbag`——即"证据基建"的完全体在仓库、**狗上是缩水版**。上文"每次跑都锁定…全量遥测"描述的是仓库理想态,非狗端现状。

## 五、消费者:`go2_experiment_audit.py`(停止时的大审计)　【无狗上对照;依赖 telemetry/snapshot 产物,后者狗上缺席 → 狗端无运行证据】
- **80KB 属实**:82737 bytes(≈80.8KiB)、2591 行【默认 wc/stat】。
- 停止时消费上述证据出审计报告:读 `system_start/end.json`【audit.py:1605-1606】、`experiment_telemetry.jsonl`【:1707/2490】、`performance_monitor.log`【:1709/2540】、`PERF_SAMPLE`【:1331】;saas 于 stop 触发【生产 saas.py:2520】。
- **狗端**:因 telemetry/snapshot 在狗上不产,audit 即便存在也"无米下锅" → 狗端无运行证据。

## 核验台账(claim → 证据 file:line → 判定)
> 记录器脚本真实路径均在 `orin_go2_fastlio_ws/scripts/`;下表省略前缀。

- 核心 4 文件已逐行读、文档以 `scripts/` 前缀引用 → **CONFIRMED**(GO2_WS 相对写法,非错误)【README对照】
- audit 80KB / 停止消费证据出报告 → **CONFIRMED**(82737B/2591 行;消费链 audit:1605/1707/1709/1331;saas:2520 触发)
- 热路径不订点云 → **CONFIRMED**(telemetry 无 PointCloud2;perf 无 ROS;snapshot:504-510 raw_pointcloud_subscriber=False)
- telemetry 订阅 9 类话题、两名 wireless 都订 → **CONFIRMED**(telemetry:187/193/204/210/215/228/238/248/258;213-235)
- 每流 KEEP_LAST depth=1 / best_effort → **CONFIRMED**(telemetry:180-184;节流 165-178,306-311)
- 记 pose+twist+协方差 / IMU / LowState 全量(12 电机 q/dq/tau/温度、BMS、体 IMU)/ 手柄 → **CONFIRMED**(telemetry:360-367/408-417/438-443/435/449-456)
- **sport 含"电机温"** → **CORRECTED**:sport 无电机温,其 temperature 是**体 IMU 温度**(imu_state,telemetry:108),**电机温仅在 LowState**(:417);sport_payload:387-403 无 motor temperature
- 标志 recorder_start/ready/health/stop(ready 需必需流到齐;recording 还需手柄)→ **CONFIRMED**(telemetry:268/541/501/576;required 155-160;controller_input 161-164,330-335,525-538)
- 这些标志是黑盒/saas 的 wait_for 对象 → **CONFIRMED(仅仓库版)**(saas:2237/2271;blackbox:925/938;狗上旧版 saas 不拉 telemetry)
- profile recording vs patrol → **CONFIRMED**(choices telemetry:590-592 default recording;黑盒 recording blackbox:883;saas 巡检 patrol saas:2030)
- snapshot start/stop/failure 各拍一次 → **CONFIRMED(分链有别)**:三相仅**录制黑盒**(blackbox:859/1055/1168);**saas 巡检只 start+stop**(saas:2036/2501),无 failure;`manual` 相未被调用
- snapshot 记 platform…nvpmodel…PSI…中断 → **CONFIRMED**(snapshot:479-484/485+368/199-206/491/492/495-498/494)
- 对源码/可执行/配置记 sha256+size+mtime → **CONFIRMED**(SOURCE 43-75/EXEC 77-95/identity 145-150)
- 对进程记 pid/executable_sha256/亲和/调度/nice → **CONFIRMED**(process 298-312;patterns 27-32)
- 并发 ROS CLI 探针含 `/laser_mapping` 与 `/livox` param dump → **CONFIRMED(节点名更正)**:真实节点名 **`/livox_lidar_publisher`**(snapshot:436);/laser_mapping 正确(:432);ThreadPool 444
- **故意不 dump 环境变量** → **CORRECTED**:实 dump 了 6 键白名单 `SAFE_ENV_KEYS`(snapshot:34-41,486-489);docstring:7 自称不 dump 与实现出入,应为"不 dump 全量/含 token 环境,仅取白名单"
- perf 纯 procfs 记全套系统/进程指标 → **CONFIRMED**(perf docstring:4;各指标 689-738 等)
- **UDP 计数对应 cmd_vel 运动路** → **CONFIRMED(解读)**:读 /proc/net/snmp Udp(perf:413-429),系统级全量 UDP,含但不限于该路【推断-未验】
- perf 标志 PERF_MONITOR_START/SAMPLE/STOP → **CONFIRMED**(perf:633/741/756)
- lio_trace 订 odom/cloud/cloud_body/livox raw/imu → 5 CSV;检 5 事件 → **CONFIRMED(默认更正)**:**raw livox 默认关**(--include-raw-livox store_true 默认 False,:290),imu 默认开(:291);订阅 100-117,CSV 61-91,事件 135/137/165/273/198
- lio_trace 非默认链、按需单跑 → **CONFIRMED**(全仓 grep 无 launch/saas/blackbox 引用)
- 巡检(08)/录制(13)都拉 telemetry+performance(+rosbag)并等 ready、停止跑审计 → **DEFAULT_VS_PROD**:仓库版确如此(saas:2225/2259/2357/2520;blackbox:881/895/918-944);**狗上 runs 06/07 manifest 仅 performance+rosbag,无 telemetry/snapshot/trace 行 → 狗跑旧版 saas**,仅 performance 经狗端证实(dog:manifest:14-16,33-36)
- 产物落 `patrol_logs/runs/<日>/xunjian-…/` 或 `recordings/…` → **CONFIRMED**:runs 路径经狗端 manifest 证实(dog:manifest:5);recordings 为黑盒侧【无狗上对照】
- 证据代码合计数千行 → **CONFIRMED**(637+552+2591+763+1429 ≈ 5972 行)

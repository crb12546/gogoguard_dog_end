# 08 · SaaS Agent(云端总编排,把一切串起来)

> 原则同 00。核心文件:`scripts/go2_saas_agent.py`(仓库版 3152 行,纯标准库)。
> **已逐行读**:1–460(HTTP/outbox/配置)、1475–1835 + 2340–3152(start_patrol 装配头、stop、分发、循环、CLI、main);
> 中段(路线解析/下载/上传/遥测/heartbeat_payload、start_patrol 中段)为 grep 级,细节按需再深读。

## 核验状态(2026-07-25 本轮,严格对磁盘源码逐条核过)

**源标签约定**(下文可证伪数值/断言都带):
- 【默认 code:N】= `go2_saas_agent.py` 里的 argparse/常量默认值(N=行号);
- 【生产 saas:N】= `start_saas_loops.sh` 启动串实际传的值;【systemd install:N】= `install_saas_autostart.sh`;
- 【狗上 06:N / 07:N】= 两份狗运行期 manifest(`analysis/.../runs/xunjian-20260725-06/07/manifest.txt`)实测,狗自己那份 agent 生成 = 生产真相参照物;
- 【推断】= 未在源码坐实;【无狗上对照】= 仓库能核但狗上无对照物,**不许默认等同狗上**。

**⚠️ 头号纠正(系统性):本文旧版把"仓库代码值"当成了"狗上生产真相",这是错的。** 两份狗 manifest(06/07,逐字一致)证明**狗上跑的是另一个更老的 `go2_saas_agent.py` + 跟随器**,与本仓库版本显著不同:

| 维度 | 仓库版会写/会跑 | 狗上 manifest 实测 |
|---|---|---|
| 跟随器可执行 | `waypoint_follower_go2_2_trace.py`【默认 code:1830】 | `waypoint_follower_go2_2`(无 `_trace`)【狗上 06:15】 |
| 跟随器 sha 字段 | `controller_source_sha256`【默认 code:1831】 | `controller_reference_sha256=d205a596…`【狗上 06:16】 |
| controller 标签 | `deployed_go2_2_nearest_lookahead_unchanged`【默认 code:1829】 | `go2_2_enhanced_nearest_lookahead`【狗上 06:14】 |
| rosbag 话题 | 8 个【默认 code:56-65】 | 6 个(缺 `/livox/imu`、`/lf/sportmodestate`)【狗上 06:36】 |
| route_sha256 | 有【默认 code:1818】 | **无**【狗上 06 全文 grep 无此字段】 |
| 十余个仓库字段 | 有(见下§三.2) | **全缺** |

`d205a596…` = `analysis/.../previous_boot/remote_source/waypoint_follower_go2_2.py`(330 行,`class WaypointFollower`)的实测 sha —— **狗上真正在跑的跟随器就是这个 330 行版**,不是仓库那个 1043 行的 `WaypointFollowerGo22`,也不是 `_trace.py` 包装器。

**本轮能坐实的**:命令流 / 分发表 / 停车链 / outbox / 视频门控 / GOTO 拒绝 / 启动分级等**逻辑骨架**都对得到仓库精确 file:line(见文末台账);manifest 各字段值也对得上仓库模板。
**本轮无法验证的**:① 狗上究竟走哪个 launcher(`start_saas_loops.sh` vs systemd `install_saas_autostart.sh`)—— 二者 seen 文件路径分歧,狗上无对照;② 狗上 `go2_saas_agent.py` 逐行源码(remote_source 仅 4 份副本,不含本文件,只能由 manifest 输出反证 repo≠dog);③ 狗上真实 robotId / 后端地址(env 可覆盖,manifest 无佐证)。

**三处必须以狗为准的段落**:§三.1「云端可下发覆盖」(大打折扣,见下)、§三.2「证据链/跟随器身份」(狗上指向另一对象)、§三.4「rosbag 话题」(狗上 6 非 8)。

**狗上状态速览**:
- `scripts/go2_saas_agent.py` → **repo≠dog**(manifest 输出反证,无直接源码 diff)
- `src/…/waypoint_follower_go2_2.py` → **repo≠dog(sha)**:仓库 `009cb25b…` 1043 行 `WaypointFollowerGo22` / 狗上 `d205a596…` 330 行 `WaypointFollower`
- `scripts/waypoint_follower_go2_2_trace.py` → **repo≠dog**:狗未启动它;且其 `:25` 用 `getattr(base_module,'WaypointFollower')` 取基类,而仓库 src 只有 `WaypointFollowerGo22`、根本没有 `WaypointFollower` —— **仓库内部就不自洽,这个 trace 包装器面向的是狗上 330 行版,不是仓库 src 版**
- `scripts/start_saas_loops.sh`、`scripts/install_saas_autostart.sh` → **【无狗上对照】**
- `runs/xunjian-20260725-06/07/manifest.txt` → 狗端运行期真相文件本身(参照基准),07 与 06 逐字一致

---

## 一、它是什么
一个**子命令 CLI**(argparse),纯 Python 标准库【默认 code:1-27,import 全标准库】(狗上无需 pip)。生产由 `start_saas_loops.sh` 起三个常驻循环【生产 saas:50-52】(**注:狗上是否真用此 launcher 还是 systemd 版无对照**):
- **command-loop**(`--interval 5`【生产 saas:46】):轮询云端拿命令并执行;
- **video-loop**(`--seconds 20 --upload`【生产 saas:51】):巡检时录 20s 视频段上传;
- **outbox-loop**(`--interval 10 --max-jobs 2`【生产 saas:52】):把上传任务队列带重试地发出去。

后端:`https://39.96.37.187/api/v1`【默认 code:40,可被 `GO2_BACKEND_BASE` 覆盖:124】(阿里云北京段 IP【推断,未在代码坐实】),机器人 `go2-tju-01`【默认 code:41,可被 `GO2_ROBOT_ID` 覆盖:134;狗上真实 robotId 无 manifest 佐证 → 未知】,**TLS 默认不校验**(`GO2_BACKEND_VERIFY_TLS=0`【默认 code:152-156】)。密钥从环境(`~/.config/go2_saas.env`【生产 saas:5】)读,不入库(token 读取 code:137-149)。

## 二、命令怎么来、怎么执行(command-loop)
- `cmd_command_loop`(每 `interval`=5s;【默认 code:3083 argparse 5.0】与【生产 saas:46 --interval 5】一致,两处生产值皆 5s)→ `cmd_command_poll_once`:**POST `/robot/heartbeat`(心跳)**【默认 code:2852-2858,endpoint 默认 /robot/heartbeat:3023】,**响应体里取 `data.get('commands',[])`**——命令搭心跳一次往返下发。
- `handle_commands`:逐条按 `cmd_id` **去重**(仅 rc==0 才 `seen.add`【code:2780-2789】,seen 集合持久化)→ `run_safe_command(action, params)` → 结果 POST `/robot/command/result`(失败转 outbox)【默认 code:2737-2747,endpoint 默认:3024】。
  - ⚠️ **seen 文件路径二义**:【默认 code:3029】与【生产 saas 未传 --seen-file → 用默认】皆 `/tmp/go2_saas_seen_commands.json`;但【systemd install:13,70】改传 `--seen-file $WS/patrol_logs/go2_saas_seen_commands.json`。**狗上究竟走哪个 launcher 无对照 → 未知**,本文两条都列。
- **门槛**:`run_file` 哨兵消失即 `break` 退出(stop 机制)【code:2866-2868】;`wall_time_ready` 无效则 `continue`(时钟没同步不动)【code:2869】。

### 命令词表与分发(`run_safe_command:2690`)
分发顺序 ping/noop/status → start → stop → goto(拒绝)→ safe → unknown(拒绝),四个命令集合定义在【默认 code:50-53】:

| 命令 | 行为 |
|---|---|
| `start_patrol/patrol_start/follow_route/start_route/auto_patrol/auto_inspection` | **run_start_patrol**(装配并启动整条巡检栈)【code:2693-2694】 |
| `stop_patrol/patrol_stop/stop_route` | **run_stop_patrol**【code:2695-2696】 |
| `start_base` / `stop_base` | 起/停 `base_bringup.sh` / `base_stop.sh`【code:2705-2706】 |
| `camera_start_loop` / `camera_stop_loop` | 相机循环开关【code:2707-2708】 |
| `ping/noop/status` | 接受【code:2691-2692】 |
| `move/walk/go/goto/navigate` | ⚠️ **"goto/free navigation is not implemented in v1.5" → 拒绝**【code:2697-2698,拒绝语句逐字一致】。只有固定路线巡检,没有自由导航。 |
| 其它 | 拒绝(unknown command action)【code:2699-2700】 |
- ⚠️ **execute_safe 门**:不带 `--execute-safe` 时一切只 dry-run(接受不执行,分支 code:2692/2702)；生产带(`EXECUTE_SAFE` 默认 1【生产 saas:14,47-49】、【systemd install:6,20-21】追加 `--execute-safe`)。

## 三、start_patrol 到底做什么(`start_patrol_command:1475` 装配一个巨型 bash)

### 1. 参数与"云端可覆盖"——**必须打折扣看**
一大批 `bounded_float`(各带 min/max 夹取),speed **硬上限 0.50**(`bounded_float(...,0.50,0.05,0.8)` 再 `min(speed,0.50)`【默认 code:1476-1483】,狗 manifest `speed=0.5`【狗上 06:6】佐证)。

⚠️ **"云端可下发覆盖"只对少数参数成立**——真正拼进 `follower_cmd` 交给跟随器的 15 个 `-p`【code:2061-2079】里,**只有 4 项随云端动**:`v_base` / `max_vx`(都=speed)、`loop_mode`、`route_file`/`trace_file`;**其余 10 项是硬编码字面量**,云端下发了也不生效:

| 参数 | 【默认 bounded_float】算出的值 | 【生产 follower -p 硬编码】实际到跟随器 | 狗上佐证 |
|---|---|---|---|
| `k_yaw` | 默认 1.20,夹 [0.10,1.20]【code:1684】 | **`:=0.900`**【code:2064】 | `go2_2_k_yaw=0.900`【狗上 06:19】 |
| `max_yaw_rate` | 默认 0.60【算出流向别处,见下】 | **`:=0.450`**【code:2064】 | `go2_2_max_yaw_rate=0.450`【狗上 06:20】 |
| `lookahead_distance` | —— | **`:=0.600`**【code:2065】 | 【狗上 06:21】 |
| `reach_distance` | —— | **`:=0.400`**【code:2066】 | 【狗上 06:22】 |
| `goal_distance` | —— | **`:=0.250`**【code:2066】 | 【狗上 06:23】 |
| `search_window` | —— | **`:=6`**【code:2067】 | 【狗上 06:24】 |
| `turn_in_place_angle` | —— | **`:=1.000`**【code:2068】 | 【狗上 06:25】 |
| `slow_down_angle` | —— | **`:=0.500`**【code:2069】 | 【狗上 06:26】 |
| `stuck_time` | —— | **`:=3.000`**【code:2070】 | 【狗上 06:27】 |
| `relocalize_distance` | —— | **`:=1.500`**【code:2070】 | 【狗上 06:28】 |

- **`k_yaw_arg` 全文件仅出现 1 次(1684)= 死参**:云端 `kYaw` 即便算进 bounded_float,也被 `follower_cmd` 的 `k_yaw:=0.900` 盖掉。旧版把 `k_yaw` 列进"传给 go2_2 的可配参数"是误导。
- **`max_yaw_rate` 的岔路**:算出的 `max_yaw_rate_arg`(默认 0.60)只流向 `cmd_vel_sender`(`-p max_vyaw`【code:2049】)与 `safe_node`(`-p max_yaw_rate`【code:2060】);而**跟随器拿到的是硬编码 0.450**【code:2064】。两个 max_yaw_rate 不是一回事。
- 因此**真正随云端动的只有 speed(v_base/max_vx)、loop、以及重定位/起点对齐那批**;k_yaw 等约 10 个跟随器参数**云端下发不生效**。

⚠️ **参数错配(核心 mess,旧版淡化了)**:start_patrol 里还算了 **16 个 `_arg`**(`max_non_corner_yaw_rate_arg` / `tracking_lookahead_distance_arg` / `max_correction_angle_deg_arg` / `course_heading_window_arg` / `course_heading_min_distance_arg` / `line_deadband_arg` / `minimum_moving_speed_arg` / `min_track_yaw_rate_arg` / `yaw_deadband_arg` / `heading_slow_angle_deg_arg` / `heading_stop_angle_deg_arg` / `corner_angle_deg_arg` / `corner_slowdown_distance_arg` / `corner_min_speed_arg` / `use_route_speed_arg` / `k_yaw_arg`)【code:1683-1716】,**每个全文件各只出现 1 次 = 纯死参,从未拼进任何命令**——"算了不用"。旧版说这是"给另一套跟随器 line_follow 的遗留",但 `go2_saas_agent.py` 全文件无 `line_follow` 字样,**"line_follow 遗留"属推断,不可坐实**【推断】;能坐实的只是"这批参数无人消费"。

### 2. 运行目录与 manifest——**证据链在狗上指向另一对象**
运行目录实为 `patrol_logs/runs/<YYYYMMDD>/xunjian-<YYYYMMDD>-NN/`(**日期 + 序号两级**,非旧版写的单一 `<时间戳>`)【code:35,1343-1359,1659】,放 route_runtime.csv、各日志、`manifest.txt`、trace、rosbag;狗 `follower_route=.../runs/20260725/xunjian-20260725-06/route_runtime.csv`【狗上 06:5】印证。

⚠️ **旧版"第三处独立证实生产跟随器身份 + 强证据链"是误导**:
- **仓库模板**会写 `controller_executable=waypoint_follower_go2_2_trace.py`【默认 code:1830】、`controller_source_sha256=%s`【默认 code:1831】、`route_sha256=%s`【默认 code:1818】、以及 `controller_trace_wrapper_sha256/_policy`【code:1832-1833】、`route_recording_evidence`【code:1819】、`fast_lio_freshness_gate`【code:1849】、`startup_motion_interlock/cpu_max_pct/stable_samples`【code:1850-1852】、`runtime_safe_node/fastlio`【code:1853-1854】、`base_watchdog_auto_restart`【code:1855】、`follower_control_trace`【code:1857】、`experiment_telemetry/_profile`【code:1860-1861】、`system_snapshot_start/end`【code:1862-1863】、`raw_pointcloud_observer/evidence`【code:1864-1865】等十余字段。
- **狗上真实 manifest 却写** `controller_executable=waypoint_follower_go2_2`(无 `_trace`)、`controller_reference_sha256=d205a596…`、**无 `route_sha256`**、**上面那十余个仓库字段全缺**【狗上 06:15-16 + 全文 grep】。
- `d205a596…` = 狗上 `remote_source/waypoint_follower_go2_2.py`(330 行 `class WaypointFollower`)的实测 sha。**故这条"证据链"在狗上锚定的是那个 330 行跟随器,与仓库 trace 包装器相反**。旧版把仓库模板当狗上真相,方向错了。

### 3. 重定位计划(`route_relocalization_plan`)
两种定位模式 `pcd`(用 `route_relocalizer`【pcd 分支 route_prepare_cmd → ros2 run go2_map_manager route_relocalizer:1989】)或 `manual_anchor`(手动锚点,走 `manual_route_anchor.py`:1918-1932)【code:751-789】;**默认 `manual_anchor`**(`GO2_PATROL_LOCALIZATION_MODE` 默认值:663)。狗 `localization_mode=manual_anchor`【狗上 06:9】——**生产走的是 manual_anchor,即 `route_relocalizer` 未被调用**。

### 4. 严格分级启动(任一失败即清理并退特定码)——**顺序已按源码更正**
旧版把 snapshot/telemetry/perf 错置到 `unitree_safe_cmd_node` 之后;**实测三者在 `sdk2_receiver` 之前(紧跟 ensure_base_ready)**【code:2206/2222/2256 在 2290 之前】。更正后的真实顺序:
```
ensure_base_ready(2203)
  → experiment_snapshot(start)   [exit 48]  (2206)
  → experiment_telemetry         [exit 49]  (2222)
  → performance_monitor          [exit 50]  (2256)
  → go2_sdk2_udp_receiver        [exit 42]  (2290, sleep 4)
  → cmd_vel_udp_sender                      (2312, sleep 1)
  → unitree_safe_cmd_node                   (2317, sleep 1)
  → FAST-LIO fresh-only gate     [exit 44]  (2322)
  → route_prepare(pcd 才 route_relocalizer) [exit 47]  (2327)
  → check_route_start_alignment             (2332)
  → base_health_watchdog                    (2343)
  → rosbag record(等 .db3 就绪)  [exit 41]  (2352)
  → video-loop(touch patrol_video.active)
  → localization_session_guard   [exit 46]
  → patrol-start-gate(CPU/FAST-LIO 稳定) [exit 45]
  → follower(等 FOLLOWER_EXACT_TRACE_READY) [exit 43]
  → echo PATROL_STARTED
```
(退出码全对上:snapshot=48 / telemetry=49 / perf=50 / sdk=42 / fresh=44 / route_frame=47【code:2206-2331】;rosbag=41:2373 / session_guard=46:2427 / gate=45:2436 / follower=43:2449。旧版只提了后四个。)

⚠️ **rosbag 话题:默认 ≠ 生产**——
- **仓库 `PATROL_ROSBAG_TOPICS` = 8 话题**:`/Odometry /livox/imu /lf/sportmodestate /patrol_cmd /cmd_vel /api/sport/request /tf /tf_static`【默认 code:56-65,实际 record:1789,写 manifest:1867】。
- **狗上 `rosbag_profile=control_light` 只录 6 话题**:`/Odometry /patrol_cmd /cmd_vel /api/sport/request /tf /tf_static`(**缺 `/livox/imu` 与 `/lf/sportmodestate`**)【狗上 06:35-36】。
- **同名 profile `control_light` 在仓库=8、在狗=6,再次证明狗跑不同(更老)版本。**

### 5. 收尾
`run_start_patrol` 执行该 bash(`timeout 240s`【code:2577】);失败**且本次拥有当前 run**(`expected_run==current_run`)时才调 `stop_patrol_command` 收尾清理(**绝不误停已在跑的巡检**,注释明写 duplicate-start 拒绝)【code:2586-2590】。

## 四、stop_patrol(`stop_patrol_command:2466`)
- TERM→KILL(先 `kill -TERM`,`sleep 1` 再 `kill -KILL`)掉 follower/safe/cmd_node/cmd_vel_sender/sdk2_receiver/perf/telemetry【code:2466,2480-2483,匹配 `waypoint_follower|unitree_safe_cmd_node|unitree_cmd_node|cmd_vel_udp_sender|go2_sdk2_udp_receiver|patrol_performance_monitor.py|go2_experiment_telemetry.py`】;
- 发 sport API **StopMove**(`ros2 topic pub /api/sport/request api_id:1003`)+ `go2_sdk2_motion_probe --iface stop`(双保险停)【code:2484-2485】;
- 停 rosbag:先 `kill -INT` 再 `kill -TERM`【code:2486-2489】;
- **收尾取证**:manifest 记 `stopped_at`【code:2491】、`ros2 bag info`【2493】、会话结束定位捕获(`manual_route_anchor --capture-only session_end`)【2494-2499】、system_end 快照【2500-2505】、base 日志切片【2506-2519】、**`go2_experiment_audit.py`**【2520-2523】、`evidence_sha256.txt`(route_original + route_runtime + manual_anchor + experiment_telemetry + follower_control_trace 五者的 sha)【2524-2529】、磁盘占用【2530-2531】。（狗 `disk_usage_kib`【狗上 06:38】、`stopped_at`【06:37】佐证收尾确实跑了。）

## 五、视频与上传
- **video-loop**:靠 `patrol_video.active` 文件门控——**只在巡检期间录**(巡检起 `touch`【code:2381-2384】、结束由 stop_patrol【2474/2535】与 video_loop【2941】双路 `unlink`;managed 模式下该文件不存在则 idle【2918】)。每段 `cmd_video_segment` 调 `z1pro_upload_segment.sh`【script=WS/scripts/…:2886】从 Z1Pro 相机(RTSP `rtsp://192.168.144.108`)录 20s(`--seconds` 默认 20【code:3118】+【生产 saas:51】)并上传,超时 `pkill -TERM/-KILL 'gst-launch-1.0.*rtsp://192.168.144.108'`【code:2901,2903】。
- **上传/outbox**:`post_multipart` 传 route/pcd/video/image(`endpoint_for`:398-406);失败 `enqueue_outbox_job('file',…)` 入队【code:1080-1090】。**outbox** = pending/inflight/failed 三态目录【code:166-171】 + 去重 `dedupeKey`【192-202,222】 + 指数退避(`delay=min(300,max(5,5·2^min(n-1,6)))`,即 5·2ⁿ 封顶 300s【code:45,241】) + 陈旧 inflight 回收(`recover_stale_inflight`,max_age=600【262】)—— 专为 4G 断网设计。
- **媒体时间戳校验**:文件名年份 <2024(`MIN_VALID_YEAR`:47)或 mtime 早于 2024-01-01 UTC(`MIN_VALID_EPOCH=1704067200`:46,Orin RTC 没同步)→ 丢弃不传(上传/枚举/outbox 三处都调 `invalid_media_time_reason`)【code:104-120】。

## 六、要点 / mess
1. **命令搭心跳下发**(一个 `/robot/heartbeat` 往返兼拿命令),结果回 `/robot/command/result`。
2. **自由导航未实现**,只有固定路线巡检(v1.5,GOTO 硬拒)。
3. ⚠️ **参数错配 + 硬编码覆盖(核心 mess)**:start_patrol 算了 16 个死参【code:1683-1716,从不入命令】;且 k_yaw 等约 10 个跟随器参数在 `follower_cmd` 是**硬编码字面量**【code:2064-2070】,云端下发不生效——"云端可覆盖"只对 speed/loop 等少数项成立。
4. ⚠️ **证据链方向纠正**:仓库 manifest 会写 `_trace.py`+`route_sha256`+十余字段,但**狗上真跑 330 行 `waypoint_follower_go2_2.py`(sha d205a596)**,manifest 少一大截字段、rosbag 只 6 话题——**repo≠dog**,证据链在狗上锚定的是另一个跟随器。
5. TLS 不校验、后端硬编码 IP —— 安全上可议(非本次重点)。

## 七、留待坐实(中段按需深读 / 狗上无对照)
- `prepare_route_csv` / `download_route_csv` / `resolve_route_map`:路线来源(本地名 vs 云端 URL 下载)与同名 pcd 解析细节。
- `heartbeat_payload`:心跳都上报什么(电量/位姿/状态)。
- `route_relocalization_plan` 的 pcd/manual_anchor 判定与阈值(生产走 manual_anchor)。
- `localization_session_guard.py` / `patrol_performance_monitor.py` / `go2_experiment_*.py` 旁路脚本细节。
- **狗上 `go2_saas_agent.py` 真身**:remote_source 无此文件副本,只能由 manifest 输出反证 repo≠dog;如需逐行 diff 须另取狗上副本。
- **狗上究竟走哪个 launcher**(`start_saas_loops.sh` /tmp 版 vs systemd `install_saas_autostart.sh` $WS/patrol_logs 版)。

---

## 核验台账(claim → 证据 file:line → 判定)

> 依据:对磁盘仓库源码 + 两份狗 manifest 逐条核。`go2_saas_agent.py` 简写 `agent`;`start_saas_loops.sh` 简写 `saas`;`install_saas_autostart.sh` 简写 `install`;manifest 简写 `06/07`。

| # | claim | 证据 | 判定 |
|---|---|---|---|
| 1 | 核心文件 agent 3152 行,纯标准库 | agent:1-27(wc -l=3152) | ✅ CONFIRMED(仓库,狗上≠) |
| 2 | 生产起 command/video/outbox 三循环 | saas:50-52 | ✅ CONFIRMED(狗上是否用此 launcher 无对照) |
| 3 | 后端默认 `https://39.96.37.187/api/v1` | agent:40(覆盖:124) | ✅ CONFIRMED("阿里云"为推断) |
| 4 | 机器人 `go2-tju-01` | agent:41(覆盖:134) | ✅ CONFIRMED(狗上 robotId 未知) |
| 5 | TLS 默认不校验 | agent:152-156 | ✅ CONFIRMED |
| 6 | 密钥从 `~/.config/go2_saas.env` 读,不入库 | saas:5;agent:137-149 | ✅ CONFIRMED |
| 7 | command-loop 每 5s | agent:3083 + saas:46 | ✅ CONFIRMED(两处生产值皆 5s) |
| 8 | POST /robot/heartbeat,响应带 commands | agent:2852-2858,3023 | ✅ CONFIRMED |
| 9 | 按 cmd_id 去重,seen 持久化 | agent:2780-2789,3029;install:13,70 | ⚠️ DEFAULT_VS_PROD:默认/saas=`/tmp/…` / systemd=`$WS/patrol_logs/…`(狗上走哪个未知) |
| 10 | 结果 POST /robot/command/result,失败转 outbox | agent:2737-2747,3024 | ✅ CONFIRMED |
| 11 | 门槛 wall_time_ready / run_file 哨兵消失即退 | agent:2866-2870 | ✅ CONFIRMED |
| 12 | 命令词表与分发 run_safe_command | agent:2690-2712,50-53 | ✅ CONFIRMED |
| 13 | move/walk/go/goto/navigate → 拒绝(v1.5 未实现) | agent:2697-2698 | ✅ CONFIRMED(逐字一致) |
| 14 | execute_safe 门:无 flag 只 dry-run,生产带 | saas:14,47-49;install:6,20-21 | ✅ CONFIRMED |
| 15 | speed 硬上限 0.50,云端可覆盖 | agent:1476-1483;06:6 | ✅ CONFIRMED(但"云端可覆盖"仅对 speed/loop 等成立) |
| 16 | k_yaw 作为可配 bounded_float 传给 go2_2 | agent:1511-1516,1684,2064;06:19 | ⚠️ DEFAULT_VS_PROD:默认 1.20 / 生产硬编码 0.900(k_yaw_arg 死参,云端下发不生效) |
| 17 | 真正传给 go2_2 的只有 14 非路由参数 | agent:2061-2079;06:19-28 | ✅ CONFIRMED + 补注:其中 10 个是硬编码字面量,仅 v_base/max_vx/loop/trace 随云端 |
| 18 | 多算的那批是 line_follow 遗留,go2_2 不消费 | agent:1683-1716 | ⚠️ 死参属实(16 个各只 1 次);"line_follow 遗留"归因 = 推断(全文无 line_follow) |
| 19 | 运行目录 patrol_logs/runs/<时间戳>/ | agent:35,1343-1359;06:5 | ✏️ 小修正:实为 `runs/<YYYYMMDD>/xunjian-<YYYYMMDD>-NN/`(两级) |
| 20 | manifest 写 controller_executable=…_trace.py + source_sha256 + route_sha256 | agent:1818,1830-1831 | ✅ CONFIRMED(仅就仓库模板;与狗上冲突,见 #21) |
| 21 | manifest 第三处证实生产跟随器身份 | 06/07:15-16;remote_source sha d205a596 | ⚠️ DEFAULT_VS_PROD:狗上写 `waypoint_follower_go2_2`+`controller_reference_sha256`+无 route_sha256 → 锚定 330 行版,与仓库相反 |
| 22 | route_relocalization_plan:pcd 或 manual_anchor | agent:751-789,663,1989;06:9 | ✅ CONFIRMED(生产走 manual_anchor,route_relocalizer 未调) |
| 23 | 严格分级启动顺序 | agent:2206,2222,2256,2290,2312,2317,2343,2352 | ✏️ CORRECTED:snapshot/telemetry/perf 在 sdk2_receiver **之前**(紧跟 ensure_base_ready),非 safe_node 之后 |
| 24 | rosbag 录 8 话题 | agent:56-65,1789;06:35-36 | ⚠️ DEFAULT_VS_PROD:仓库 8 话题 / 狗上 control_light 只 6(缺 /livox/imu、/lf/sportmodestate) |
| 25 | run_start_patrol timeout 240s,只清自己的 run | agent:2577,2586-2590 | ✅ CONFIRMED |
| 26 | stop_patrol TERM→KILL 七类进程 | agent:2466,2480-2483 | ✅ CONFIRMED |
| 27 | 发 StopMove(1003)+ motion_probe stop | agent:2484-2485 | ✅ CONFIRMED |
| 28 | 停 rosbag(INT) | agent:2486-2489 | ✅ CONFIRMED(先 INT 再 TERM) |
| 29 | 收尾取证(stopped_at/bag info/audit/evidence_sha256/…) | agent:2491-2531 | ✅ CONFIRMED |
| 30 | video-loop 靠 patrol_video.active 门控 | agent:2381-2384,2918,2941,2535 | ✅ CONFIRMED |
| 31 | cmd_video_segment 录 Z1Pro RTSP 20s 上传 | agent:2886,2901-2903,3118 | ✅ CONFIRMED |
| 32 | post_multipart 传 route/pcd/video/image,失败入队 | agent:398-406,1080-1090 | ✅ CONFIRMED |
| 33 | outbox 三态 + dedupeKey + 指数退避 + 陈旧回收 | agent:45,166-171,241,262 | ✅ CONFIRMED |
| 34 | 媒体时间戳校验(<2024 或 mtime 太早丢弃) | agent:46-47,104-120 | ✅ CONFIRMED |

**dog-side 状态**:agent = repo≠dog(manifest 反证)；`waypoint_follower_go2_2.py` = repo≠dog(sha 009cb25b/1043 行 vs d205a596/330 行)；`waypoint_follower_go2_2_trace.py` = repo≠dog(狗未启,且目标基类只在狗上版存在)；`saas`/`install` = 【无狗上对照】；`06/07 manifest.txt` = 狗端真相参照物(07≡06)。

# 08 · SaaS Agent(云端总编排,把一切串起来)

> 原则同 00。核心文件:`scripts/go2_saas_agent.py`(3152 行,纯标准库)。
> **已逐行读**:1–460(HTTP/outbox/配置)、1475–1835 + 2340–3152(start_patrol 装配头、stop、分发、循环、CLI、main);
> 中段(路线解析/下载/上传/遥测/heartbeat_payload、start_patrol 中段)为 grep 级,细节按需再深读。

## 一、它是什么
一个**子命令 CLI**(argparse),纯 Python 标准库(狗上无需 pip)。生产由 `start_saas_loops.sh` 起三个常驻循环:
- **command-loop**:轮询云端拿命令并执行;
- **video-loop**:巡检时录 20s 视频段上传;
- **outbox-loop**:把上传任务队列带重试地发出去。

后端:`https://39.96.37.187/api/v1`(默认,阿里云 IP),机器人 `go2-tju-01`,**TLS 默认不校验**(`GO2_BACKEND_VERIFY_TLS=0`)。密钥从环境(`~/.config/go2_saas.env`)读,不入库。

## 二、命令怎么来、怎么执行(command-loop)
- `cmd_command_loop`(每 `interval`=5s)→ `cmd_command_poll_once`:**POST `/robot/heartbeat`(心跳)**,**响应体里带 `commands` 数组**——命令搭心跳一次往返下发。
- `handle_commands`:逐条按 `cmd_id` **去重**(seen 集合持久化 `/tmp/go2_saas_seen_commands.json`)→ `run_safe_command(action, params)` → 结果 POST `/robot/command/result`(失败转 outbox)。
- **门槛**:`wall_time_ready`(时钟没同步不动)、`run_file` 哨兵消失即退出(stop 机制)。

### 命令词表与分发(`run_safe_command:2690`)
| 命令 | 行为 |
|---|---|
| `start_patrol/patrol_start/follow_route/auto_patrol/...` | **run_start_patrol**(装配并启动整条巡检栈) |
| `stop_patrol/patrol_stop/stop_route` | **run_stop_patrol** |
| `start_base` / `stop_base` | 起/停 `base_bringup.sh` / `base_stop.sh` |
| `camera_start_loop` / `camera_stop_loop` | 相机循环开关 |
| `ping/noop/status` | 接受 |
| `move/walk/go/goto/navigate` | ⚠️ **"goto/free navigation is not implemented in v1.5" → 拒绝**。只有固定路线巡检,没有自由导航。 |
| 其它 | 拒绝 |
- ⚠️ **execute_safe 门**:不带 `--execute-safe` 时一切只 dry-run(接受不执行);生产带(`EXECUTE_SAFE=1`)。

## 三、start_patrol 到底做什么(`start_patrol_command:1475` 装配一个巨型 bash)
1. **参数**:一大批 `bounded_float`(各带 min/max 夹取),speed **硬上限 0.50**;云端可下发覆盖。
   - ⚠️ **参数错配(mess)**:这里算了 `cornerAngleDeg/headingSlowAngleDeg/lineDeadband/minimumMovingSpeed/maxCorrectionAngleDeg/...` 一大堆,但真正传给 `waypoint_follower_go2_2` 的只有 14 个(v_base/max_vx/k_yaw/max_yaw_rate/lookahead/reach/goal/loop/search_window/turn_in_place/slow_down/stuck/relocalize/trace)。**多算的那批基本是给另一套跟随器(line_follow)的遗留,go2_2 不消费。**
2. **运行目录** `patrol_logs/runs/<时间戳>/`:放 route_runtime.csv、各日志、`manifest.txt`、trace、rosbag。
   - manifest 明写:`controller_executable=waypoint_follower_go2_2_trace.py`、`controller_source_sha256=...`、`localization_mode`、`route_sha256` —— **第三处独立证实生产跟随器身份 + 强证据链**。
3. **重定位计划** `route_relocalization_plan`:两种定位模式 `pcd`(用 route_relocalizer,见 07)或 `manual_anchor`(手动锚点)。
4. **严格分级启动(任一失败即清理并退特定码)**,顺序大致:
   ```
   ensure_base_ready → go2_sdk2_udp_receiver → cmd_vel_udp_sender → unitree_safe_cmd_node
     → performance_monitor / experiment_telemetry / snapshot
     → (pcd 模式: route_relocalizer)
     → base_health_watchdog
     → rosbag record(等 .db3 就绪, 否则 exit 41)
     → video-loop(touch patrol_video.active)
     → localization_session_guard(等就绪, 否则 exit 46)
     → patrol-start-gate(CPU/FAST-LIO 稳定, 否则 exit 45)
     → follower(等日志出现 FOLLOWER_EXACT_TRACE_READY, 否则 exit 43)
     → echo PATROL_STARTED
   ```
   rosbag 录的话题:`/Odometry /livox/imu /lf/sportmodestate /patrol_cmd /cmd_vel /api/sport/request /tf /tf_static`。
5. `run_start_patrol` 执行该 bash(timeout 240s);失败且本次拥有当前 run 时,调 `stop_patrol_command` 收尾清理(**绝不误停已在跑的巡检**)。

## 四、stop_patrol(`stop_patrol_command:2466`)
- TERM→KILL 掉 follower/safe/cmd_node/cmd_vel_sender/sdk2_receiver/perf/telemetry;
- 发 sport API **StopMove**(api_id 1003)+ `go2_sdk2_motion_probe stop`(双保险停);
- 停 rosbag(INT);
- **收尾取证**:manifest 记 stopped_at、`ros2 bag info`、会话结束定位捕获、system_end 快照、base 日志切片、**experiment_audit**、`evidence_sha256.txt`(route/runtime/anchor/telemetry/trace 的 sha)、磁盘占用。

## 五、视频与上传
- **video-loop**:靠 `patrol_video.active` 文件门控——**只在巡检期间录**(巡检起 touch、结束 unlink)。每段 `cmd_video_segment` 调 `z1pro_upload_segment.sh` 从 Z1Pro 相机(RTSP `rtsp://192.168.144.108`)录 20s 并上传(超时会 pkill gst-launch)。
- **上传/outbox**:`post_multipart` 传 route/pcd/video/image;失败 `enqueue_outbox_job` 入队。**outbox** = pending/inflight/failed 三态目录 + 去重 dedupeKey + 指数退避(5·2ⁿ,封顶 300s)+ 陈旧 inflight 回收 —— 专为 4G 断网设计。
- **媒体时间戳校验**:文件名年份 <2024 或 mtime 太早(Orin RTC 没同步)→ 丢弃不传。

## 六、要点 / mess
1. **命令搭心跳下发**(一个 `/robot/heartbeat` 往返兼拿命令),结果回 `/robot/command/result`。
2. **自由导航未实现**,只有固定路线巡检(v1.5)。
3. **参数错配**:start_patrol 算了一堆 go2_2 不用的跟随参数(line_follow 遗留)。
4. **强证据链**:manifest/sha256/audit/rosbag/trace,每次巡检留一整套可回溯证据。
5. TLS 不校验、后端硬编码 IP —— 安全上可议(非本次重点)。

## 七、留待坐实(中段按需深读)
- `prepare_route_csv` / `download_route_csv` / `resolve_route_map`:路线来源(本地名 vs 云端 URL 下载)与同名 pcd 解析细节。
- `heartbeat_payload`:心跳都上报什么(电量/位姿/状态)。
- `route_relocalization_plan` 的 pcd/manual_anchor 判定与阈值。
- `localization_session_guard.py` / `patrol_performance_monitor.py` / `go2_experiment_*.py` 旁路脚本细节。

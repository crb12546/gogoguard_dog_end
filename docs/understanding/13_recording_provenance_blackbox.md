# 13 · 录制证据链:route_recording_blackbox(录制的"专业版")

> 原则同 00。核心文件(已逐行读):`scripts/route_recording_blackbox.py`。
> 与 05(裸 `route_recorder` 节点)的关系:05 是最底层录点;本篇是把录制包成**带完整证据的黑盒**,并产出 saas 巡检要校验的 `.recording.json`。

## 核验状态
- **本轮(2026-07-25)已对磁盘源码逐条核对**:§一~§三 对 `route_recording_blackbox.py` 的逐行描述几乎全部对得上真实代码(阈值 / topic / 就绪信号 / schema / 字段 / errors 全部 CONFIRMED)。
- **两处已更正(原文有过度断言)**:
  - **§四**:原文"没有干净录制证据的路线,巡检可能被拒 …… 信任链的锁"过强。真实逻辑是**"有坏 sidecar 才 raise 拒绝;缺 sidecar 直接放行(`available=False`,不 raise)"**——缺证据不被拦,只锁坏证据。已改写。
  - **§五**:原文把 `go2_experiment_audit.py` 当成"录制停止时的大审计"不成立。录制停止走的是**进程内 `build_route_audit`**;`go2_experiment_audit.py` 只在 **saas 巡检停止路径**被调,属巡检(非录制)证据链。已区分。
- **源标签约定**:
  - 【默认 file:line】= 磁盘源码里的 code path 默认值/逻辑。
  - 【生产·console file:line】= 录制黑盒的**生产启动方**=操作员控制台 `tools/patrol_console/server.py`。
  - 【生产·saas file:line】= sidecar 的**生产消费方**=`go2_saas_agent.py`(巡检启动前校验)。
  - 【无狗上对照】= 该文件不在狗上 4 份副本(`laserMapping.cpp`/`lddc.cpp`/`lds.cpp`/`waypoint_follower_go2_2.py`)之列,repo↔dog 无法 sha 验,**狗上是否一致本轮无法验证**。
- **仍无法验证**:本篇引用的全部脚本(`route_recording_blackbox.py`、`go2_saas_agent.py`、`localization_session_guard.py`、`manual_route_anchor.py`、`go2_experiment_telemetry.py`、`patrol_performance_monitor.py`、`go2_experiment_snapshot.py`、`go2_experiment_audit.py`、`route_recorder.py`、`server.py`)均**【无狗上对照】**——磁盘 repo 逻辑已核实,狗上实际跑的是否同一份代码无从对照。

## 一、为什么要它
裸 `route_recorder` 只写点。但一条路线要能被信任地回放,必须证明:**它是在某一次完整、未中断的 FAST-LIO 会话里录的**,且录制器采样忠实。`route_recording_blackbox.py` 就干这个(docstring 原文一字不差:"Manage a complete, low-observer-effect route-recording experiment.")【默认 route_recording_blackbox.py:2】【无狗上对照】。

## 二、start(一次调用起录、带就绪门控)
1. 校验路线目标(必须在 `src/go2_fastlio_patrol/routes/` 下、`.csv`,越界或非 csv 直接 raise)【默认 :750-761】;查冲突(不能与巡检/其它录制并行,`CONFLICT_PATTERN` 命中 `route_recorder|waypoint_follower|go2map_capture|ros2 bag record|localization_session_guard …--mode recorder` 等即 raise `mode conflict`)【默认 CONFLICT_PATTERN :42-48;find_conflicts :171-193;raise :808-813】;建 run 目录 `patrol_logs/recordings/<日>/record-<ts>-<stem>-blackbox-<pid>`【默认 :726-747】。
2. **捕获起始 FAST-LIO 会话身份**(`manual_route_anchor.py --capture-only`)+ 系统快照(`go2_experiment_snapshot.py`)【默认 start 阶段依次调用 :850-862;capture :270-308(`--capture-only` :281);snapshot :311-339】。会话身份 = `boot_id`/`pid`/`start_ticks`【默认 manual_route_anchor.py:87-89】。
3. **起法(措辞已校正)**:前三个 telemetry/performance/rosbag 是**先批量 spawn**(:876-916)**再按下列顺序逐个等就绪门控**(:918-955);只有 `session_guard`、`route_recorder` 才是严格"spawn→等就绪→再起下一个"。就绪门控的先后顺序与下面列举一致。都作为独立进程组(`start_new_session=True`、记 `pgid`)【默认 spawn_process :196-217(pgid :211)】:
   - `go2_experiment_telemetry.py --profile recording`,等 `"kind":"recorder_ready"`【默认 spawn :876-889;门控 :918-930;marker go2_experiment_telemetry.py:541】
   - `patrol_performance_monitor.py`,等 `PERF_MONITOR_START`【默认 spawn :890-903;门控 :931-943;marker patrol_performance_monitor.py:633】
   - **轻量 rosbag**:只录 `/Odometry /livox/imu /lf/wirelesscontroller /wirelesscontroller /tf /tf_static`(恰好这 6 个),等 `.db3` 出现 —— **故意不录点云**,减少观测负载(`observer_policy.raw_pointcloud_subscriber=False`,点云证据改由"Livox and FAST-LIO in-process timing logs"提供)【默认 topics :34-41;spawn :904-916;门控 :944-955;observer_policy :840-843】
   - `localization_session_guard.py --mode recorder`(带 `--invalidate-output <route_file>`;会话变了就把半成品路线**改名为 `.invalid.YYYYmmdd_HHMMSS`**——注意是带时间戳后缀,非裸 `.invalid`;并写 `abort_reason=localization_session_changed`),等 `SESSION_GUARD_STARTED`【默认 spawn :957-976;门控 :977-991;invalidate_output localization_session_guard.py:50-62(后缀 :54);abort :112-134】
   - `route_recorder` 节点(`-p route_file:=` / `-p min_distance:=`),等日志出现 `route_recorder started` 且文件已开始写(`size>5`)【默认 spawn :994-1009;门控 :1010-1024;日志串 route_recorder.py:49】
4. 状态写 `/tmp/go2_route_recording_blackbox.json`(docstring 明言"跨 SSH 安全")【默认 DEFAULT_STATE_FILE :31;docstring :5-6】。任一步失败 → 清理所有进程组 + 失败快照(`phase='failure'`)+ 删 state【默认 except 分支 :1041-1063;cleanup_started_processes :772-788】。

## 三、stop(收尾 + 出证据)
1. 按序停各进程组:`route_recorder`/`rosbag` 用 SIGINT,`telemetry`/`performance`/`session_guard` 用 SIGTERM;`stop_group` 返回 `not_running`/`graceful`/`terminated`/`killed`【默认 停止顺序 :1128-1152;stop_group :240-251】。
2. **捕获结束会话身份** + 结束快照(`phase='stop'`);拷路线 → `route_recorded.csv`(`shutil.copy2`)【默认 session_end :1156-1161;snapshot :1164-1171;route_copy :1175-1179】。
3. **路线审计 `build_route_audit`(进程内,非外部脚本)**【默认 :594-659】:
   - 几何(`route_geometry`):点距分布、路径长、`>0.60m`/`>1.00m` 大间距计数、最强的 **20** 个转角、body-yaw vs 轨迹方向误差、包围盒。阈值 `0.60`/`1.00` 与 top-20 均**硬编码**,无生产变体【默认 :465-520(spacing_above_0_60m :502-504;above_1_00m :505-507;strongest_turns[:20] :486-490;body_yaw_vs_track :491-495/508-510;bounding_box :512-517)】。
   - **遥测重现校验**(`telemetry_route_comparison`):用遥测里的 odom 按 `min_distance` 重采样,和 CSV 逐点比;判据 `exact_within_2mm_0_002rad` = 起点误差≤0.002 且 逐点 pos 误差 max≤0.002 且 yaw 误差 max≤0.002(即 **2mm=0.002m、0.002rad,硬编码**)【默认 :523-591(重采样 :543-549;逐点 :554-563;exact 判据 :564-570;字段名 :582)】。
4. **产出 `route.csv.recording.json` sidecar**(schema `go2.route_recording_link.v1`)【默认 schema :1274;写 `str(route_file)+'.recording.json'` :1291-1293】,关键字段:
   - `route_sha256`、`route_line_count`【默认 :1278-1279(sha256_file :1236;行数 :1237-1243)】
   - **`same_fastlio_session_at_start_and_stop`**(首尾 `boot_id`+`pid`+`start_ticks` 三者全一致才 True)【默认 比较 :1254-1263;写入 :1283】
   - `status`: `complete` / `complete_with_warnings`(有 errors 即后者)【默认 :1275】
   - `errors[]`:rosbag 空(`rosbag_missing_or_empty`)、遥测未干净收尾(需 `recorder_ready`+`recorder_stop`)、性能监控未干净收尾(需 `PERF_MONITOR_START`+`STOP`)、缺 `livox.log`/`fast_lio.log`、重现不匹配(`route_sampler_reproduction_mismatch`)、**会话变了**(`fastlio_session_changed_during_recording`)、进程被 kill(`unclean_process_stop:<name>`)等【默认 :1208 / :1210-1214 / :1216-1220 / :1225-1229 / :1230-1235 / :1264-1265 / :1266-1269】。

## 四、和巡检启动的闭环(为什么重要 —— 已更正过度断言)
sidecar 的**生产消费方**是 `go2_saas_agent.route_recording_evidence`(08),在命令执行前调用(`prepare_route_csv` 与 `start_patrol_command` 两处)【生产·saas go2_saas_agent.py:1249-1311;调用点 :1327 / :1652】。

它的实际判定分两种情形,**必须分清**:

- **① sidecar 存在但坏 → raise → 阻断巡检**(这才是"锁"锁住的对象):
  - `route_sha256` 与当前 CSV 不符 → `ROUTE_RECORDING_LINK_MISMATCH`【生产·saas :1274-1278】
  - `status != complete` → `ROUTE_RECORDING_EVIDENCE_INCOMPLETE`(文档旧写法 `INCOMPLETE` 是简写)【生产·saas :1280-1292】
  - `same_fastlio_session_at_start_and_stop` **`is False`** → `ROUTE_RECORDING_SESSION_CHANGED`(旧写法 `SESSION_CHANGED` 是简写)。判据是 `is False`——**缺字段/None 不触发,只有显式 `False` 才触发**【生产·saas :1293-1298】。
- **② sidecar 根本不存在 → 直接 `return available=False`,不 raise → 巡检照常放行**【生产·saas :1259-1265】。且 `start_patrol` 侧只把 `recordingEvidence` 存进 `route_info`,**没有针对 `available==False` 的额外拦截**【生产·saas :1646-1656】。

→ **更正结论**:**缺录制证据的路线不会被拦,巡检照跑;被拦的只有"有证据但证据坏"的路线**。所以这条链是"**坏证据锁**",不是"**必须有证据**"的强制门。旧文"没有干净录制证据 → 可能被拒 …… 信任链的锁"会误导为"无证据即拒",与代码不符,已改写。

## 五、录制审计 vs 巡检审计(已区分,勿混)
- **录制停止审计 = 进程内 `build_route_audit`**(见 §三.3),`route_recording_blackbox.py` 全文**从不调用** `go2_experiment_audit.py`【默认 build_route_audit :594-659;grep 全文无 audit 引用】。录制流真正外调的只有 `go2_experiment_snapshot.py`(start/stop/failure 三处快照)【默认 via take_snapshot :311-339,调用点 :315】。
- **巡检停止审计 = `go2_experiment_audit.py`**,它的**唯一生产调用点在 saas 巡检停止路径**(一段大 shell 命令里 `python3 -u …/scripts/go2_experiment_audit.py --run-dir …`),属**巡检**证据链,不属录制【生产·saas go2_saas_agent.py:2520】。详见 14(诊断/遥测)。
- `go2_experiment_telemetry.py`(录制/巡检遥测,产 jsonl)、`patrol_performance_monitor.py` 的完整职责亦见 14。

## 六、min_distance:默认 vs 生产(核对结论:一致,且真被消费)
录制重采样/录点步距 `min_distance`,两端取值**一致均 0.40,无 DEFAULT_VS_PROD 分歧**:
- **默认**:黑盒 CLI `--min-distance default=0.40`【默认 route_recording_blackbox.py:1391】;底层节点 `declare_parameter('min_distance', 0.4)`【默认 route_recorder.py:23】。
- **生产**:录制黑盒由**操作员控制台**驱动,`ROUTE_NORMAL_SPACING = 0.40` → 启动串 `--min-distance 0.40`【生产·console tools/patrol_console/server.py:61 / 569-571;测试断言 `--min-distance 0.40` test_patrol_console_anchor_paths.py:29】。
- **真到达消费端、不被覆盖**:该值经 `route_recorder -p min_distance:=%.6f` 下发给录点节点【默认 :1004-1005】,并进入停止审计的重采样 `state['min_distance_m']` / `audit.min_distance_m`【默认 :1183;:632】。**不存在"云端下发其实不生效 / 算了不用 / 被硬编码覆盖"的情况**。
- **注意生产驱动方**:是操作员控制台 `server.py` 启动录制黑盒;**saas 不启动录制,只消费其 sidecar**(§四)。

## 核验台账(claim → 证据 file:line → 判定)
| # | 断言 | 证据 | 判定 |
|---|---|---|---|
| 1 | docstring "low-observer-effect route-recording experiment" | route_recording_blackbox.py:2 | CONFIRMED |
| 2 | 路线目标须在 `src/go2_fastlio_patrol/routes/` 下且 `.csv` | :750-761 | CONFIRMED |
| 3 | 查冲突,不与巡检/其它录制并行 | :42-48 / :171-193 / :808-813 | CONFIRMED |
| 4 | 建 run 目录 `patrol_logs/recordings/<日>/record-…` | :726-747 | CONFIRMED |
| 5 | 捕获起始会话身份(`--capture-only`)+ 快照 | :270-308 / :311-339 / :850-862;manual_route_anchor.py:87-89 | CONFIRMED |
| 6 | telemetry `--profile recording`,等 `recorder_ready` | :876-889 / :918-930;telemetry.py:541 | CONFIRMED |
| 7 | performance monitor,等 `PERF_MONITOR_START` | :890-903 / :931-943;monitor.py:633 | CONFIRMED |
| 8 | 轻量 rosbag 只录 6 topic,等 `.db3`,不录点云 | :34-41 / :904-916 / :944-955 / :840-843 | CONFIRMED |
| 9 | session_guard `--mode recorder`,会话变改 `.invalid.<ts>`,等 `SESSION_GUARD_STARTED` | :957-991;session_guard.py:50-62(后缀 :54)/112-134 | CONFIRMED(后缀带时间戳,原文措辞略简) |
| 10 | route_recorder 等 `route_recorder started` 且开始写 | :994-1024;route_recorder.py:49 | CONFIRMED |
| 11 | 独立进程组、记 pgid、逐个等就绪 | :196-217;各门控 :918-955/977-991/1010-1024 | CONFIRMED(前三批起+顺序门控,"依次起"已校正) |
| 12 | state 写 `/tmp/go2_route_recording_blackbox.json`(跨 SSH 安全) | :31;docstring :5-6 | CONFIRMED |
| 13 | 失败 → 清理进程组 + 失败快照 | :1041-1063;:772-788 | CONFIRMED |
| 14 | 停止:recorder/rosbag SIGINT,余 SIGTERM;记 graceful/terminated/killed | :1128-1152;:240-251 | CONFIRMED |
| 15 | 捕获结束会话 + 结束快照;拷 `route_recorded.csv` | :1156-1161 / :1164-1171 / :1175-1179 | CONFIRMED |
| 16 | `build_route_audit` 几何(0.60/1.00/top-20/包围盒…) | :465-520 | CONFIRMED(阈值硬编码) |
| 17 | 遥测重现 `exact_within_2mm_0_002rad`(0.002m/rad) | :523-591 | CONFIRMED(硬编码) |
| 18 | sidecar `route.csv.recording.json`,schema `go2.route_recording_link.v1` | :1274 / :1291-1293 | CONFIRMED |
| 19 | 字段 `route_sha256`、`route_line_count` | :1278-1279 | CONFIRMED |
| 20 | `same_fastlio_session…` = 首尾 boot_id+pid+start_ticks 一致 | :1254-1263 / :1283 | CONFIRMED |
| 21 | `status`: complete / complete_with_warnings | :1275 | CONFIRMED |
| 22 | `errors[]` 各项(rosbag空/收尾/缺日志/重现/会话/kill) | :1208/1214/1220/1225-1229/1235/1265/1266-1269 | CONFIRMED |
| 23 | saas 巡检前校验:LINK_MISMATCH / INCOMPLETE / SESSION_CHANGED | go2_saas_agent.py:1274-1278/1280-1292/1293-1298;调用 :1327/1652 | CONFIRMED(全名 `ROUTE_RECORDING_EVIDENCE_INCOMPLETE`/`…SESSION_CHANGED`) |
| 24 | ~~无干净证据 → 巡检可能被拒(信任链的锁)~~ | go2_saas_agent.py:1259-1265(缺 sidecar return `available=False` **不 raise**);:1646-1656(无 available==False 拦截) | **CORRECTED**:缺证据放行,只锁坏证据 |
| 25 | ~~`go2_experiment_audit.py`(录制停止的大审计)~~ | route_recording_blackbox.py 全文无 audit 引用,停止走 build_route_audit :594-659;audit 唯一生产点 go2_saas_agent.py:2520(巡检停止) | **CORRECTED**:录制流不调 audit 脚本 |
| 26 | min_distance 取值(默认 vs 生产) | 默认 :1391 / route_recorder.py:23=0.4;生产 server.py:61/571=0.40;测试 :29;消费 :1004-1005 / :1183/632 | CONFIRMED 一致 0.40,真被消费,无覆盖 |

**狗上状态汇总**:上表所有 file:line 均出自磁盘 repo;本篇引用的 10 个脚本/节点**全部【无狗上对照】**(不在狗上 `laserMapping.cpp`/`lddc.cpp`/`lds.cpp`/`waypoint_follower_go2_2.py` 4 份副本内),**狗上是否运行同一份代码本轮无法验证**。

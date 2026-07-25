# 17 · 巡检总审计器 go2_experiment_audit.py(停止时对每次巡检逐阶段核验)

> 原则同 00。核心文件:`scripts/go2_experiment_audit.py`(2591 行)【默认 audit.py:1(wc -l=2591)】。
> 结构已逐行核实(函数级),它是**证据消费者**——不产生新行为,只把各章那套证据汇总成一份审计报告。
> 由 `stop_patrol`(08)调用【生产 saas.py:2466 def stop_patrol_command;:2520-2522 触发】。
>
> 路径备注:磁盘真实路径带前缀 `orin_go2_fastlio_ws/scripts/go2_experiment_audit.py`;本篇沿用 00/saas 的
> workspace 相对写法(saas 调用串亦为 `%s/scripts/go2_experiment_audit.py`),非笔误。

## 核验状态(本轮)

- 本轮对**磁盘源码逐条核实**(函数级):`orin_go2_fastlio_ws/scripts/go2_experiment_audit.py` 全 2591 行,
  以及调用它的 `go2_saas_agent.py`、被它读取的产物源 `patrol_performance_monitor.py`。约 14–15 个审计子报告与代码映射**基本正确**。
- 本轮**改正 3 处可证伪错误**(详见正文与文末台账):① `classify` 并不给整段跑打 PASS/WARN/FAIL;
  ② `command_chain_audit` 是**两级**(patrol_cmd→cmd_vel)不是三级;③ 输入清单误列 `manifest.txt`、且 `performance_audit` 不汇总内存。
- **默认 vs 生产 参数区分:本篇不适用。** 该文件是纯只读事后分析器,除 `--run-dir` 外**无 launch/`-p` 可配参数**,
  云端不下发任何行为参数,故无"代码默认值≠生产值"之说。可证伪项只区分【默认 audit.py:line=脚本本体逻辑】与【生产 saas.py:line=saas 侧调用/重定向】。
- **狗上一致性:无法验证(no_reference)。** 本次快照的 `remote_source/` 狗上副本仅 4 份
  (laserMapping.cpp / lddc.cpp / lds.cpp / waypoint_follower_go2_2.py)。`go2_experiment_audit.py`、`go2_saas_agent.py`、
  `patrol_performance_monitor.py` **均无狗上副本**,属 ~117 个自研文件之一,狗上是否与仓库一致**无从 sha 验**。本篇为函数级核实,未声称狗上等同。
- 标签缩写:`audit.py` = go2_experiment_audit.py;`saas.py` = go2_saas_agent.py;`perfmon.py` = patrol_performance_monitor.py。

## 一、它吃什么、吐什么

- **输入**(审计**真正读取**的 run 目录证据,`patrol_logs/runs/<日>/xunjian-…/`):
  - `experiment_telemetry.jsonl`(遥测)【默认 audit.py:2490】、`follower_control_trace.jsonl`(控制轨迹)【默认 audit.py:2524】
  - `rosbag/*.db3`【默认 audit.py:1299】+ `rosbag_info.txt`【默认 audit.py:1273】
  - `performance_monitor.log`【默认 audit.py:2540】、`system_start.json`/`system_end.json`(快照)【默认 audit.py:1605-1606】
  - `base_logs/`(livox.log、fast_lio.log 等)【默认 audit.py:1212-1227,1718-1719】
  - `manual_anchor.json`【默认 audit.py:2478】、`route_original.csv`【默认 audit.py:2476】、`route_runtime.csv`【默认 audit.py:2477】
  - 另读(原文漏列):`route_recording.json`【默认 audit.py:2479】、`localization_session_end.json`【默认 audit.py:1764】、
    `*_guard.log`【默认 audit.py:1781】、以及 log_timing 用的 follower/safe/udp 四份日志。
- **`manifest.txt` 不是审计输入(改正)**:`audit.py` 从不读取它(grep manifest = 0 命中)。`manifest.txt` 是 saas `stop_patrol`
  **追加写的产物**【生产 saas.py:2491,2499,2505,2523 `>>"$run_dir/manifest.txt"`】。原文把"saas 侧写的清单"误当成"审计输入"。
- **输出**:
  - `experiment_audit.log` = saas 把审计器 **stdout 重定向**的捕获【生产 saas.py:2521 `>"$run_dir/experiment_audit.log" 2>&1`】;
    内容仅一行 `EXPERIMENT_AUDIT_READY`,**不是**结构化报告。
  - `experiment_audit.json`(结构化报告,schema `go2.patrol_experiment_audit.v1`)【默认 audit.py:2572 写文件;:2502 写 schema 串】
  - `experiment_audit.md`(markdown 渲染)【默认 audit.py:2577】;markdown 本身**不带** schema 字段。
- **`classify()` 不给整段跑打 PASS/WARN/FAIL(改正)**:见第二节 classify 行与第二节末说明。

## 二、约 14–15 项审计(逐个函数)

装配见 `build_report`【默认 audit.py:2501-2553】,report 段:evidence_health、route_recording_link、coordinate_transform、
localization_sessions、route_tracking、follower_control_trace、command_chain、measured_response、robot_state、
sensor_body_alignment、rosbag、timing_logs、performance、system_configuration、external_ground_truth,末尾 `findings=classify`【默认 audit.py:2554】。

| 审计函数 | 查什么 | 源标签 |
|---|---|---|
| `control_trace_audit` | 跟随器控制轨迹:每周期计数、odom→控制延迟、odom 复用、停车计数(措辞见下注) | 【默认 audit.py:340-358】 |
| `verify_manual_transform` | manual_anchor 变换正确性(原始 CSV × runtime CSV × 锚点 metadata 刚性一致,阈值 2e-6) | 【默认 audit.py:518-589】 |
| `tracking_audit` | **跟踪质量**:用 odom 投影到路线算横向偏差(mean/median/p95/max)、progress 回退计数 | 【默认 audit.py:592-675】 |
| `command_chain_audit` | **两级**:`/patrol_cmd`(raw)↔`/cmd_vel`(safe)配对一致性 + 收令时延 + 非零转零 override(**不含 sport,见改正**) | 【默认 audit.py:689-727】 |
| `measured_response_audit` | **指令 vs 实测运动**:`cmd_vel` 命令 vx/vyaw 与 sport 实测速度是否吻合(扫 0~0.5s lag 取最优)——此处才涉及 `/api/sport/request` | 【默认 audit.py:766-835】 |
| `sensor_body_alignment_audit` | 四元数/欧拉:机体姿态(sport imu)vs 传感器(odom orientation)相对 roll/pitch/yaw | 【默认 audit.py:898-1002】 |
| `robot_state_audit` | sport/lowstate:error_code、步态、电量(bms.soc)、电机温度/丢失 | 【默认 audit.py:1005-1124】 |
| `log_timing_audit` | 从各日志解析时序标记(TIMING_FOLLOWER/SAFE/SENDER/RECEIVER、LIVOX_*、FAST_LIO_*_TIMING、SAFE OVERRIDE/stale/ALERT) | 【默认 audit.py:1188-1269】 |
| `rosbag_audit` | rosbag 是否完整、话题齐、db3 **存在**(记 size 但裁决按"存在"非"size>0",见注) | 【默认 audit.py:1272-1320】 |
| `performance_audit` | 运行期 **CPU/温度/网络丢包/CPU 压力**是否饱和(**不含内存,见改正**) | 【默认 audit.py:1323-1448】 |
| `localization_configuration_audit` / `system_configuration_audit` | 从快照核验 FAST-LIO/Livox 参数、以及各源码/可执行的 **sha256** 起止变更(当时到底跑的哪份代码);前者被后者内部调用 | 【默认 audit.py:1520-1601 / 1604-1681(:1676 调用)】 |
| `evidence_health` | 证据完整性(各流是否齐、malformed 行、缺流/缺件) | 【默认 audit.py:1684-1742】 |
| `localization_session_audit` | 首尾 FAST-LIO 会话身份一致(boot_id/pid/start_ticks,没中途重启)+ guard abort | 【默认 audit.py:1745-1808】 |
| `classify` | **逐阶段 findings 列表**(severity ∈ {ok, warning, error, needs_external_measurement})+ 每条 stage:conclusion——**非整段聚合裁决** | 【默认 audit.py:1811-2164】 |

**措辞订正(避免过度断言):**

- **`control_trace_audit`**:odom→控制延迟 = `odom_receive_to_control_ms`【默认 audit.py:358,源 :235】;停车 = `is_stop` 计数
  【默认 audit.py:249-250】→ `active_control_records`(:341)vs `control_records`(:340);odom 复用 = `consecutive_control_cycles_per_odom`(:349)。
  原文的"发指令率"**无显式速率(Hz)输出**、"控制周期"也非以周期时长计、"停车占比"是**可推导计数**非直接比值——功能级成立,措辞偏松。
- **`tracking_audit`**:`cross_track_error_m` = summary(含 mean/median/p95/max)【默认 audit.py:659】;progress 回退 = `progress_regression_over_0_20m_count`【默认 audit.py:664】。
  "是否脱轨"**无显式布尔**,靠 progress 回退计数 + classify 中 `p95>0.30 → "deviated materially"`【默认 audit.py:1891】间接体现。
- **`command_chain_audit`(结构性改正)**:该函数**只比两级** patrol_cmd(raw)↔cmd_vel(safe):配对、`raw_to_safe_receive_age_s`、
  `absolute_difference`、`nonzero_to_zero_override`【默认 audit.py:690-691,713-727】,**返回字段无 sport**。
  `cmd_vel`→`/api/sport/request`(sport 实测)那一级在 **`measured_response_audit`**【默认 audit.py:767-768 commands=cmd_vel + sport_samples】。
  "`/patrol_cmd`→`/cmd_vel`→`/api/sport/request` 三级链"作为**系统概念**成立(rosbag 关键话题含此三者【默认 audit.py:1286-1291】),
  但把"三级一致性"归给 `command_chain_audit` 单个函数**不准确**。
- **`rosbag_audit`**:`available = bool(db3 存在)`【默认 audit.py:1310】,记 `size_bytes`(:1299-1306)但**裁决未按 size>0 判空**,严格说是"db3 存在"。
- **`performance_audit`(内存改正 + 算了不用)**:只聚合 `monitor_wake_late_ms`、`max_temperature_c`、`cpu_pressure_some_avg10`、
  `per_cpu_pct`、`process_cpu_pct`、`network_drops`、`udp_deltas`、`hwmon`、`process_scheduling`【默认 audit.py:1410-1447】——
  **无任何 memory/RSS/内存压力字段**。原始 PERF_SAMPLE 确含内存【perfmon.py:102 read_memory,:710 pressure_memory,:712 memory】,
  但**审计端未取用**。另:performance 段**算了却未进 classify**——classify 只吃 evidence/rosbag/transform/tracking/control_trace/recording/sessions/system/robot/alignment
  【默认 audit.py:1811-1822,无 performance】,属"算了不用";"是否饱和/和控制时序尖峰对齐"亦非代码显式实现,为解释性描述。
- **`classify`(过度断言改正,原文两处 line 11 与表格 line 29)**:`classify` 返回的是**逐阶段 findings 列表**,每条 severity ∈
  {ok, warning, error, needs_external_measurement}【默认 audit.py:1827/1892/1898/1904/2155 等】;全文件**无 'PASS'/'FAIL' 字样、无对整段跑的单一聚合裁决**
  (grep PASS/FAIL/verdict = 0 命中)。`build_markdown` 逐条渲染 `[severity] stage:conclusion`【默认 audit.py:2226-2234】。
  saas 侧调用为 **fire-and-forget,不解析 findings 生成总评**【生产 saas.py:2520-2523】。ok/warning/error 可粗映射 PASS/WARN/FAIL,
  但(1)逐阶段非整段(2)字面量不同(3)另有 `needs_external_measurement` 第 4 类——故"给整段跑打 PASS/WARN/FAIL / 汇总打分"系过度断言。

## 三、为什么它对你重要

你最初的痛点是"巡检逻辑有问题、连定位都费劲"。**这个审计器 + 每次跑留下的 run 目录,就是定位问题的现成金矿**:

- 想知道某次巡检偏没偏、偏多少 → `tracking_audit` 的横偏 p95/max【默认 audit.py:659】+ progress 回退计数(:664)。
- 想知道"命令发了但狗没动"还是"命令就不对" → `measured_response_audit`(cmd_vel 指令 vs sport 实测,:766-835)+ `command_chain_audit`(patrol_cmd↔cmd_vel,:689-727)。
- 想确认"当时跑的是不是仓库这份代码" → 配置审计里的 sha256【默认 audit.py:1678 起止比对 :1623-1627】。
  **注(推断-未验)**:该函数本身只做 **start-vs-end 变更检测**(跑中有没有被改);要真正**印证"仓库≠狗上"**,
  需拿快照 `system_start.json` 里的 sha256 与仓库比对,**由使用者外部完成**,非本函数直接产出。
- 每个 `analysis/xunjian-*` 目录里都有这套证据 + 审计报告,回放即可。

## 四、说明

- 本文件是**函数级**核实(2591 行的每项审计做什么、吃什么、判什么已清楚)【默认 audit.py:1(wc -l=2591)】;
  逐行的数值算法细节(某个百分位/阈值)可在需要定位具体指标时再对着函数看。
- 它纯读证据、纯出报告,不接触 ROS/机器人、**不影响狗的任何实时行为**【默认 audit.py:2-5 docstring "uses only captured files … cannot alter the run"】;
  且只在停止(kill 完成)后才由 saas 触发【生产 saas.py:2520】。
- **狗上状态:【无狗上对照】(no_reference)。** `audit.py`、`saas.py`、`perfmon.py` 均不在本次 `remote_source/`(狗上仅 4 份 C++/py 副本),
  **无从 sha 验狗上是否与仓库一致**——本篇结论仅覆盖磁盘仓库这份源码,不主张狗上等同。

## 核验台账

> claim → 证据 file:line → 判定。逐条对真源码核过;`audit.py`=go2_experiment_audit.py,`saas.py`=go2_saas_agent.py,`perfmon.py`=patrol_performance_monitor.py。

| # | claim(原文) | 证据 | 判定 |
|---|---|---|---|
| 1 | 核心文件 `scripts/go2_experiment_audit.py`(2591 行) | audit.py wc -l=2591;真实路径带 `orin_go2_fastlio_ws/` 前缀,workspace 相对写法与 00/saas 一致 | **CONFIRMED**(路径省前缀非误) |
| 2 | 由 `stop_patrol`(08)调用 | saas.py:2466 def stop_patrol_command;:2520-2522 `python3 -u %s/scripts/go2_experiment_audit.py --run-dir`;:2630 run_stop_patrol | **CONFIRMED** |
| 3 | 证据消费者,不影响狗实时行为 | audit.py:2-5 docstring;停止(kill 完成)后才在 saas:2520 触发 | **CONFIRMED**("汇总各章证据"为跨章推断) |
| 4 | 输入含 `manifest.txt` | grep manifest audit.py = **0 命中**;manifest.txt 是 saas 追加写的产物 saas.py:2491,2499,2505,2523 | **CORRECTED**:manifest.txt 非审计输入 |
| 5 | 输入:telemetry/control_trace/rosbag/perf/start-end/base_logs/anchor/route_original/route_runtime | telemetry:2490;control_trace:2524;rosbag/*.db3:1299+info.txt:1273;perf.log:2540;start/end:1605-1606;base_logs:1212-1227,1718-1719;anchor:2478;route_original:2476;route_runtime:2477 | **CONFIRMED**(另漏列 route_recording.json:2479 / localization_session_end.json:1764 / *_guard.log:1781 / follower·safe·udp 日志) |
| 6 | 输出:experiment_audit.log + 结构化报告(schema v1,JSON+md) | .log=saas:2521 stdout 重定向(仅一行 EXPERIMENT_AUDIT_READY);.json:2572;.md:2577;schema 串在 JSON:2502(md 不带 schema) | **CONFIRMED**(细节点明) |
| 7 | `classify()` 给整段跑打 PASS/WARN/FAIL(line 11) | classify:1811 返回 findings 列表,severity∈{ok,warning,error,needs_external_measurement};grep PASS/FAIL/verdict=0;build_markdown:2226-2234 逐条渲染;saas fire-and-forget:2520-2523 | **CORRECTED**:无整段聚合裁决 |
| 8 | classify 汇总打分→PASS/WARN/FAIL + 原因清单(表格 line 29) | 同上;:1811 返回 findings(list);:2554 report['findings']=classify | **CORRECTED**:原因清单对,汇总打分错 |
| 9 | `control_trace_audit` 控制周期/延迟/发指令率/停车占比 | odom→控制延迟:358(源:235);停车 is_stop:249-250→active:341 vs control:340;odom 复用:349 | **CONFIRMED**(措辞偏松:无 Hz 速率、占比为可推导计数) |
| 10 | `verify_manual_transform` 刚性一致 | audit.py:518-589,施刚性旋转平移,exact_within_csv_precision 阈值 2e-6(:588) | **CONFIRMED** |
| 11 | `tracking_audit` 横向偏差(mean/p95/max)、是否脱轨 | :592-675;cross_track_error_m summary:659;progress_regression:664 | **CONFIRMED**("是否脱轨"无显式布尔,靠 p95>0.30:1891 间接) |
| 12 | `command_chain_audit` `/patrol_cmd`→`/cmd_vel`→`/api/sport/request` **三级**一致性 | :690-691 raw=patrol_cmd,safe=cmd_vel;:713-727 返回字段**无 sport**;sport 级在 measured_response_audit:767-768 | **CORRECTED**:实为两级 |
| 13 | `measured_response_audit` 命令 vs sport 实测 | :766-835;commands=cmd_vel:767;sport velocity/yaw_speed:739-741;扫 0~0.5s lag 取 best_response_lag_s | **CONFIRMED** |
| 14 | `sensor_body_alignment_audit` 机体 vs 传感器对齐 | :898-1002;body=sport imu 四元数:900-906;lidar=odom orientation:915-920;relative=q_body⁻¹·q_lidar→欧拉:952-956 | **CONFIRMED** |
| 15 | `robot_state_audit` error_code/步态/电量/电机温度 | :1005-1124;low→power/bms.soc/motor temp/lost:1018-1052;sport→error_code/mode/gait_type:1063-1065 | **CONFIRMED** |
| 16 | `log_timing_audit` 各日志时序标记 | :1188-1269;解析 TIMING_*/LIVOX_*/FAST_LIO_*_TIMING/SAFE OVERRIDE/stale/ALERT | **CONFIRMED** |
| 17 | `rosbag_audit` 完整/话题齐/db3 非空 | :1272-1320;info.txt 话题计数;critical_topics:1285-1292;列 db3 名+size:1299-1306;available=bool(db3 存在):1310 | **CONFIRMED**(严格是"db3 存在"非 size>0 判空) |
| 18 | `performance_audit` CPU/**内存**/温度/丢包饱和 | :1323-1448 无 mem/ram/rss;聚合 wake_late/max_temp/cpu_pressure/per_cpu/process_cpu/network_drops/udp/hwmon/scheduling:1410-1447;PERF_SAMPLE 含内存 perfmon.py:102,710,712 但未取用;performance 未进 classify:1811-1822 | **CORRECTED**:不汇总内存 + 算了不用 |
| 19 | `localization_/system_configuration_audit` 参数 + sha256 | localization:1520-1601(extrinsic/qos/time_sync/scan_rate/MID360s_config);system:1604-1681,sha256 起止比 changed_files:1623-1627,1678;前者被后者调用:1676 | **CONFIRMED** |
| 20 | `evidence_health` 证据完整/malformed | :1684-1742;malformed_jsonl_lines/record_counts/missing_required_streams:1695/missing_required_artifacts:1721 | **CONFIRMED** |
| 21 | `localization_session_audit` 首尾会话身份一致 | :1745-1808;session_identity_equal 比 boot_id/pid/start_ticks:1748-1751;+ session_guard_abort_detected | **CONFIRMED** |
| 22 | 约 15 项审计 | build_report:2501-2553 装配约 14-15 段 + findings=classify | **CONFIRMED**("约"字准确) |
| 23 | sha256 印证"仓库≠狗上" | system_configuration_audit 输出 source_or_executable_files_changed_during_run:1678(起止 sha256 比对:1623-1627);原值存 system_start.json | **CONFIRMED(推断-未验)**:函数只做 start-vs-end;比仓库需外部完成 |
| 24 | 2591 行每项审计已清楚(line 39) | wc -l=2591 | **CONFIRMED** |

**狗上状态**:`audit.py` / `saas.py` / `perfmon.py` 三者 **均无狗上副本(no_reference)**——本次 `remote_source/` 仅
laserMapping.cpp / lddc.cpp / lds.cpp / waypoint_follower_go2_2.py 4 份;上述三文件属 ~117 个自研文件,狗上一致性**无从 sha 验**。

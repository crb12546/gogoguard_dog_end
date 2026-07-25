# 13 · 录制证据链:route_recording_blackbox(录制的"专业版")

> 原则同 00。核心文件(已逐行读):`scripts/route_recording_blackbox.py`。
> 与 05(裸 `route_recorder` 节点)的关系:05 是最底层录点;本篇是把录制包成**带完整证据的黑盒**,并产出 saas 巡检要校验的 `.recording.json`。

## 一、为什么要它
裸 `route_recorder` 只写点。但一条路线要能被信任地回放,必须证明:**它是在某一次完整、未中断的 FAST-LIO 会话里录的**,且录制器采样忠实。`route_recording_blackbox.py` 就干这个(docstring: "low-observer-effect route-recording experiment")。

## 二、start(一次调用起录、带就绪门控)
1. 校验路线目标(必须在 `src/go2_fastlio_patrol/routes/` 下、`.csv`);查冲突(不能与巡检/其它录制并行);建 run 目录 `patrol_logs/recordings/<日>/record-…`。
2. **捕获起始 FAST-LIO 会话身份**(`manual_route_anchor --capture-only`)+ 系统快照。
3. 依次起(都作为独立进程组、记 pgid、逐个等就绪标志):
   - `go2_experiment_telemetry.py --profile recording`(等 `recorder_ready`)
   - `patrol_performance_monitor.py`(等 `PERF_MONITOR_START`)
   - **轻量 rosbag**:只录 `/Odometry /livox/imu /lf/wirelesscontroller /wirelesscontroller /tf /tf_static`(等 `.db3` 出现)—— 故意不录点云,减少观测负载。
   - `localization_session_guard.py --mode recorder`(会话变了就把半成品路线改 `.invalid`;等 `SESSION_GUARD_STARTED`)
   - `route_recorder` 节点(等日志出现 `route_recorder started` 且文件开始写)。
4. 状态写 `/tmp/go2_route_recording_blackbox.json`(跨 SSH 安全)。任一步失败 → 清理所有进程组 + 失败快照。

## 三、stop(收尾 + 出证据)
1. 按序停各进程组(recorder/rosbag 用 SIGINT,其余 SIGTERM;记录 graceful/terminated/killed)。
2. **捕获结束会话身份** + 结束快照;拷路线 → `route_recorded.csv`。
3. **路线审计 `build_route_audit`**:
   - 几何:点距分布、路径长、>0.6m/>1.0m 的大间距计数、最强的 20 个转角、body-yaw vs 轨迹方向误差、包围盒。
   - **遥测重现校验**:用遥测里的 odom 按 min_distance 重采样,和 CSV 逐点比,`exact_within_2mm_0_002rad` 判录制器是否忠实复现。
4. **产出 `route.csv.recording.json` sidecar**(schema `go2.route_recording_link.v1`),关键字段:
   - `route_sha256`、`route_line_count`
   - **`same_fastlio_session_at_start_and_stop`**(首尾 boot_id+pid+start_ticks 一致)
   - `status`: `complete` / `complete_with_warnings`
   - `errors[]`:rosbag 空、遥测/性能未干净收尾、缺 livox/fast_lio 日志、重现不匹配、**会话变了**、进程被 kill 等。

## 四、和巡检启动的闭环(为什么重要)
`go2_saas_agent.route_recording_evidence`(08)在 `start_patrol` 前会:
- 找 `route.csv.recording.json`,校验 `route_sha256` 与当前 CSV 一致(`ROUTE_RECORDING_LINK_MISMATCH`)、`status=complete`(`INCOMPLETE`)、`same_fastlio_session…!=False`(`SESSION_CHANGED`)。
→ **没有干净录制证据的路线,巡检可能被拒**。这就是整条"录制→回放"信任链的锁。

## 五、留待坐实
- `go2_experiment_telemetry.py`(录制/巡检遥测,产 jsonl)、`patrol_performance_monitor.py`、`go2_experiment_snapshot.py`、`go2_experiment_audit.py`(停止时的大审计)—— 见 14(诊断/遥测)。

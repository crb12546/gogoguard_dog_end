# 14 · 证据与诊断层(每次巡检/录制都产的"飞行记录仪")

> 原则同 00。核心文件(均已逐行读):
> `scripts/go2_experiment_telemetry.py`、`scripts/go2_experiment_snapshot.py`、
> `scripts/patrol_performance_monitor.py`、`scripts/go2_lio_trace_recorder.py`。
> 消费者 `scripts/go2_experiment_audit.py`(停止时的大审计)见 §5,待深读。

## 一、为什么有这一层
巡检/录制在户外一次性发生,出了问题要能事后复盘。所以每次运行都并行拉起一组**低观测负载**的记录器,把"传感器、控制、系统资源、以及当时到底跑的哪份代码"全留证。**关键设计:热路径记录器一律不订阅点云**(在 Python 里反序列化整帧点云会改变它要测的时序)。

## 二、四个记录器
### `go2_experiment_telemetry.py`(跨层遥测 → JSONL)
- 订阅:`/Odometry`、`/livox/imu`、`/lf/sportmodestate`、`/lf/lowstate`、无线手柄(`/lf/wirelesscontroller` 和 `/wirelesscontroller` 两名都订)、`/patrol_cmd`、`/cmd_vel`、`/api/sport/request`。
- 每流限速(KEEP_LAST depth=1 / best_effort),记 pose+twist+协方差、IMU、sport(足力/足位/步态/电机温/range_obstacle)、**LowState 全量**(12 电机 q/dq/tau/温度、BMS soc/电流/单体电压、体 IMU)、手柄摇杆。
- 标志:`recorder_start` / `recorder_ready`(必需流都到齐;recording profile 还需手柄输入)/ `recorder_health`(每秒 received/written/age)/ `recorder_stop`。→ 这些标志正是黑盒/saas 启动时 `wait_for` 的对象。
- profile:`recording` vs `patrol`。

### `go2_experiment_snapshot.py`(边界快照,不连续)
- 在 start/stop/failure 各拍一次;记 platform/boot_id/uptime/loadavg/kernel cmdline、**nvpmodel(Jetson 功耗档)**、CPU 调速器/频率、内存、温度、PSI 压力、中断。
- **代码身份铁证**:对一长串源码/可执行/配置(base_bringup、follower、safe、cmd_vel_bridge、FAST-LIO config、laserMapping.cpp、install 产物…)记 **sha256 + size + mtime**;对关键进程记 pid/executable/**executable_sha256**/start_ticks/cpu 亲和/调度/nice。
- 并发跑 ROS CLI 探针(node/topic list、关键话题 QoS、`/laser_mapping` 和 `/livox` 的 param dump)。
- **故意不 dump 环境变量**(SaaS env 里有 token)。

### `patrol_performance_monitor.py`(每秒系统/进程画像)
- 纯 procfs/sysfs:系统 & 每核 CPU%、CPU 频率、**GPU/EMC**、**hwmon INA3221 功耗轨(整机功耗)**、温度、load、procs_running/blocked、上下文切换、PSI、内存、磁盘 I/O、网络速率+丢包错误、**UDP 计数(对应 cmd_vel UDP 运动路)**,以及各关键进程 CPU%/RSS/线程/末核/亲和/上下文切换。
- 标志:`PERF_MONITOR_START` / `PERF_SAMPLE` / `PERF_MONITOR_STOP`。
- 用途:把控制时序尖峰和系统饱和对齐着看。

### `go2_lio_trace_recorder.py`(FAST-LIO 输入输出 CSV,独立诊断工具)
- 订 odom/cloud/cloud_body/livox raw/imu → 分别写 CSV;检 `TOPIC_GAP`/`STAMP_GAP`/`ODOM_JUMP`/`TOPIC_STALE`/`CLOUD_EMPTY` 事件。非默认巡检链的一部分,按需单跑。

## 三、和运行链的关系
- 巡检(08)与录制黑盒(13)启动时都会拉起 telemetry + performance(+ rosbag),并**等其 ready 标志**才继续;停止时收尾并跑审计。
- 产物落在 `patrol_logs/runs/<日>/xunjian-…/` 或 `patrol_logs/recordings/…`,再被同步进 `analysis/`(仓库里那些海量日志的来源)。

## 四、这层告诉我们的工程取向
整套系统对"**可复现、可取证**"极其执着:每次跑都锁定代码 sha、进程身份、FAST-LIO 会话、系统资源、全量遥测。这也是"乱"的另一面——**大量精力花在诊断/证据基建上**(experiment_*/performance/telemetry/blackbox/snapshot 合计数千行),控制本体反而只是其中一小块。

## 五、留待坐实
- `go2_experiment_audit.py`(80KB,停止时消费上述证据出审计报告)—— 待深读,单列或并入本篇。

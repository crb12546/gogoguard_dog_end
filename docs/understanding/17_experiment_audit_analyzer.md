# 17 · 巡检总审计器 go2_experiment_audit.py(停止时给每次巡检打分)

> 原则同 00。核心文件:`scripts/go2_experiment_audit.py`(2591 行)。结构已逐行核实(函数级),
> 它是**证据消费者**——不产生新行为,只把 13/14 章那套证据汇总成一份评分报告。由 `stop_patrol`(08)调用。

## 一、它吃什么、吐什么
- **输入**:一次巡检的 run 目录 `patrol_logs/runs/<日>/xunjian-…/` 里的全部证据:
  `experiment_telemetry.jsonl`(遥测)、`follower_control_trace.jsonl`(控制轨迹)、`rosbag/`、
  `performance_monitor.log`、`system_start.json`/`system_end.json`(快照)、`manifest.txt`、`base_logs/`、
  `manual_anchor.json`/`route_original.csv`/`route_runtime.csv`。
- **输出**:`experiment_audit.log` + 结构化报告(schema `go2.patrol_experiment_audit.v1`,JSON + markdown),`classify()` 给整段跑打 **PASS/WARN/FAIL**。

## 二、约 15 项审计(逐个函数)
| 审计函数 | 查什么 |
|---|---|
| `control_trace_audit` | 跟随器控制轨迹:控制周期、odom→控制延迟、发指令率、停车占比 |
| `verify_manual_transform` | manual_anchor 变换正确性(原始 CSV × runtime CSV × 锚点 metadata 刚性一致) |
| `tracking_audit` | **跟踪质量**:用 odom 投影到路线算横向偏差(mean/p95/max)、是否脱轨 |
| `command_chain_audit` | `/patrol_cmd`→`/cmd_vel`→`/api/sport/request` 三级命令**一致性 + 延迟** |
| `measured_response_audit` | **指令 vs 实测运动**:命令的 vx/vyaw 和 sport 实测速度是否吻合(狗真按命令动了吗) |
| `sensor_body_alignment_audit` | 四元数/欧拉:机体姿态 vs 传感器对齐(俯仰/横滚偏差) |
| `robot_state_audit` | sport/lowstate:error_code、步态、电量、电机温度异常 |
| `log_timing_audit` | 从各日志解析时序标记(启动阶段耗时等) |
| `rosbag_audit` | rosbag 是否完整、话题齐、db3 非空 |
| `performance_audit` | 运行期 CPU/内存/温度/丢包是否饱和(和控制时序尖峰对齐) |
| `localization_configuration_audit` / `system_configuration_audit` | 从快照核验 FAST-LIO/Livox 参数、以及各源码/可执行的 **sha256**(当时到底跑的哪份代码) |
| `evidence_health` | 证据完整性(各流是否齐、有无 malformed) |
| `localization_session_audit` | 首尾 FAST-LIO 会话身份一致(没中途重启) |
| `classify` | 汇总打分 → PASS/WARN/FAIL + 原因清单 |

## 三、为什么它对你重要
你最初的痛点是"巡检逻辑有问题、连定位都费劲"。**这个审计器 + 每次跑留下的 run 目录,就是定位问题的现成金矿**:
- 想知道某次巡检偏没偏、偏多少 → `tracking_audit` 的横偏 p95/max。
- 想知道"命令发了但狗没动"还是"命令就不对" → `measured_response_audit`(指令vs实测)+ `command_chain_audit`。
- 想确认"当时跑的是不是仓库这份代码" → 配置审计里的 sha256(能直接印证"仓库≠狗上")。
- 每个 `analysis/xunjian-*` 目录里都有这套证据 + 审计报告,回放即可。

## 四、说明
- 本文件是**函数级**核实(2591 行的每项审计做什么、吃什么、判什么已清楚);逐行的数值算法细节(某个百分位/阈值)可在需要定位具体指标时再对着函数看。
- 它纯读证据、纯出报告,不影响狗的任何实时行为。

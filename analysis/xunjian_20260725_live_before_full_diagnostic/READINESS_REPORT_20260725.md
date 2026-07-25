# Go2 全链路诊断就绪记录

记录时间：2026-07-25 14:18 +08:00

## 结论边界

本文件记录正式“新路线录制一次、同路线巡检一次”之前的部署、故障注入、门禁验证和现场只读体检。正式实验尚未由用户执行，因此这里证明的是证据链已经可用，不提前声称路线偏差根因已经确定。

## 已由日志精确证明的问题

旧 FAST-LIO 实现把单次大于 1 秒的正向 Livox 帧间隔永久锁死，第一次完整录制烟测取得：

`[FAST_LIO_HEALTH] rejecting Livox frame 1/3 ... dt=1.09296 ...`

随后进入永久锁定，而 Livox 驱动仍持续约 10.06 Hz 点云和约 200 Hz IMU。该结论来自进程日志与同一时段驱动计数，不是从路线外观反推。

新 FAST-LIO 已改为丢弃跨越间隙的旧组合并从当前帧恢复。静止、无控制器的故障注入用 `SIGSTOP` 制造 1.58976 秒真实间隙，日志明确记录：

`[FAST_LIO_RECOVERY] accepted current Livox frame after positive gap dt=1.58976s; continuing without a base restart`

随后重新取得连续 10 帧约 99.3 ms 的输出。

## 五次录制链路烟测

| 证据目录 | 目的 | 结果 |
|---|---|---|
| `record-20260725-133015-failed` | 复现旧实现 | 精确复现 1.09296 秒后永久锁死 |
| `record-20260725-133836-recovered` | 验证恢复实现 | 同类间隙后恢复，无需重启底座 |
| `record-20260725-135321-warmup-gated` | 验证观察者热身门禁 | 采集器启动峰值不进入正式窗口 |
| `record-20260725-140055-final-gated` | 验证所有参与者后的最终门禁 | 启动期 1.9872 秒间隙被隔离；正式 5 秒 50 帧，最大记录间隔 113.406 ms |
| `record-20260725-140543-writer-gated` | 验证 CSV 写入门禁 | 第 0 点只在门禁释放后写入；正式窗口 51 帧，最大间隔 115.074 ms |

最后一次烟测中，CSV 第 0 点与独立遥测在正式开始后 48.317 ms 配对，位置差约 `4.86e-7 m`，yaw 差约 `2.59e-5°`。启动期日志和正式路线日志分别按精确字节偏移保存。

## 正式录制将保存的证据

- 原始 CSV、0.40 m 抽样逐点复算、路线长度/间距/曲率审计及 SHA-256 sidecar；
- `/Odometry`、Livox IMU、SportModeState、LowState、遥控输入和 TF rosbag；
- FAST-LIO/Livox 内部输入、输出、队列、发布时延、恢复与健康事件；
- 每核利用率、频率、在线状态、线程实际运行核、温度、NVPModel、GPU/EMC、内存 PSI、磁盘和网络；
- 录制首尾定位会话身份、代码/配置/二进制哈希和运行时参数；
- 正式窗口开始/结束边界，以及采集器启动热身的独立日志。

## 正式巡检将额外保存的证据

- 原始 CSV、人工锚定后的运行时 CSV及刚体变换逐点复算；
- 每个控制周期实际采用的里程计回调、帧龄、最近点、目标点、横向/航向误差及原控制器发布命令；
- `/patrol_cmd` 到安全节点 `/cmd_vel` 的改写和原因；
- UDP 序号、间隔、时延、丢序号和 SDK `Move` 调用耗时；
- Unitree 底层报告速度、位置、yaw rate、脚力、电池、功率和电机温度；
- FAST-LIO 姿态与狗身 IMU 姿态的时间配对；
- 成功、失败、定位会话改变或人工停止时的统一收尾报告。

完整证据源未就绪时不释放 CSV 第 0 点；巡检参与者与至少 5 个新鲜里程计回调未就绪时只发零，不进入原控制器状态机，也不释放机器狗运动。

## 部署一致性

本地与狗端以下运行源码 SHA-256 完全一致：

- `route_recording_blackbox.py`: `7a2f7fb1ba584272526126f11c947ece592e02eb9dc1ab0b3106a447102881fe`
- `go2_saas_agent.py`: `a3349ee20b623460f6235486e6cfaa0c8cb7c1c8c034092c800711ae7cb935b8`
- `go2_experiment_audit.py`: `ff907f037136ed324a25af10c3c7801ff77bfb79b967d5f68ee5b2ba5a42c7a6`
- `go2_experiment_telemetry.py`: `5433c70ba6e2a867bcd2cac58f1411c7b05aed5597d46c2577311867ff66e13a`
- `patrol_performance_monitor.py`: `8975d041dcfa1efa7b5bd301b05090156da828dda9c6bcefeed356df1b4495dd`
- `go2_experiment_snapshot.py`: `977c8dc8afee36d421f4edf2ae98bb28b589e561a7d23aff9b0c58f9703dc944`
- `localization_session_guard.py`: `79c4e0001a862b42c336ec60746fe3c8ec6473973c714770531097f71fdb93fe`
- `waypoint_follower_go2_2_trace.py`: `ff7a67fc3bb20fbd0e5bdbe3e404258ab3d8a4806555141585e83a0fcb946e74`
- 实际 Python 导入的 `route_recorder.py`: `6f7547d3c1bf85e815a7a4798a6e12653f14ac2fa56fce400d5fe32d2b2ed5d5`
- 狗端 FAST-LIO 可执行文件：`f326f31dab120753dbe1eeee5a590f591654d69fb736e4e908ccc0361c3d8d90`

狗端 Python 明确报告实际导入路径为：

`/home/unitree/go2_fastlio_ws/src/go2_fastlio_patrol/go2_fastlio_patrol/route_recorder.py`

并确认包含 `startup_enable_file` 写入门禁。

## 2026-07-25 14:18 现场只读体检

- SSH：`go2wired` 正常；
- Livox 与 FAST-LIO：运行中，当前会话 PID 分别为 77123、77512；
- 当前连续 FAST-LIO 输出窗口：每组 10 帧，平均局部间隔约 101–103 ms，最近各组最大值约 103–115 ms；
- 管理台最终重启前后 FAST-LIO 恢复事件计数均为 3，未新增间隙；
- 里程计接收年龄：约 0.2 s；
- 电量：51%；CPU 温度：约 54°C；
- NVPModel：25W；在线 CPU：`0-7`；
- 工作区可用空间：约 414.97 GiB；
- Livox 网络：可达；
- `go2-saas-command`、`go2-saas-outbox`、`go2-saas-video`：均为 `active`；
- 路线录制、跟线、安全、UDP、PCD、证据采集进程：均未运行；
- 录制/巡检/视频互斥标志：均不存在；
- 诊断临时路线：无；
- 录制黑匣子状态：`active=false`。

## 管理台验证

`http://127.0.0.1:8642` 已改为先完成一次真实远端状态扫描。最后一次冷重启：

- 端口启动到首个状态响应：3.520 s；
- 首个响应：`status_ready=true`、`ssh_ok=true`、Livox/FAST-LIO 为运行；
- ROS 遥测随后取得 `/Odometry`，页面不再把初始化空状态显示为“狗没连接”；
- 遥测流年龄改用本机单调接收时间，不再受两台机器约 0.5 秒墙钟差影响；
- 路线录制 HTTP 等待上限从 90 秒增至 320 秒，覆盖完整热身和写入门禁。

管理台“启动完整巡检”和 GoGoGuard `start_patrol` 调用同一套 `go2_saas_agent.py patrol-start` 生产链路。

## 回归结果

- 管理台、SaaS、锚定、会话保护和证据审计：41 项通过；随后新增 2 项管理台启动/遥测年龄测试也通过；
- 控制、路线和质量策略：65 项通过；
- Python 编译、生成 shell 语法、HTML 内联 JavaScript 语法和 `git diff --check`：通过。

## 仍必须由正式实验提供的外部真值

车载日志无法独立证明真实地面轨迹。正式录制和巡检必须使用同一固定机位原视频、地面起点中心、至少 1 m 朝向线以及第一长段的可量测参照点。没有这份外部真值，即使所有车载坐标彼此一致，也无法在“定位整体漂移”和“狗真实执行偏差”之间作最终裁决。

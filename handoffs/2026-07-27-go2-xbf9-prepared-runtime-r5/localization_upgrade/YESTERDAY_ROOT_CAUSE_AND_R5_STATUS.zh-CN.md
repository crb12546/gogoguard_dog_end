# 2026-07-26 现场失败根因与当前 R5 覆盖情况

## 结论先说

昨天并不是“点云算法反复算错导致狗不走”。实际是多项启动工程问题按顺序
暴露，最后一次测试又被 follower 进程组竞态提前终止。全部实机尝试中，
coordinator 从未被观察到进入 `RUNNING`，所有运动输出始终为零。

因此昨天的证据可以确认“启动链有问题”，但不能据此判定实时 PCD 配准成功或
失败。当前 R5 已把已确认的启动根因纳入正式交付，并换成用户重新对齐的
CSV、固定物层和 checkpoint；实时配准能否在这个真实起点收敛，仍必须由下一次
真狗扫描验证。

## 已确认根因

| 昨天现象 | 根因 | 当前 R5 |
|---|---|---|
| 定位器涨到约 14.6 GiB RSS，耗尽 swap 后被 OOM kill | 2.55 MiB descriptor JSON 经 yaml-cpp 展开成大量节点，内存被放大 | 已改为 JsonCpp 严格解析，并设 64 MiB 输入上限；真实 production map 已在 1 GiB 约束下完成加载 |
| 即使空 ROS 节点也出现约 14.7 GiB RSS | 新节点落到该板上的默认 Fast DDS，与稳定基础栈不一致 | 全部 XBF 节点在完整 source 顺序后重新固定为 Cyclone DDS |
| coordinator 收不到路线、地图身份；localizer 的地图路径为空 | ROS 2 Foxy 中 params-file 覆盖了命令行参数，启动脚本假设的优先级不成立 | 启动前物化 localizer/coordinator 两份 runtime YAML，不再依赖覆盖顺序 |
| receiver 明明能启动，却被报告 UDP 5005 未监听 | receiver 在绑定 UDP 前先执行 StandUp、等待、BalanceStand、再等待；原 10 秒门限过短；旧检测还假设 `ss` 固定列 | 等待改为 30 秒，UDP 检测扫描全部字段 |
| 最后一次 follower 已读入 1277 点，却被监督器报告退出并开始清理 | `setsid command &` 后立即用 `$!` 当 PGID；新进程组尚未建立，瞬时检查误判，随后产生残留 follower | 改为 Python 原地 `setsid()+exec()`，只有确认 `PID=PGID=SID` 后才记录；停止按 run-id 和 Linux starttime 精确清理 |

最后一项是昨天最后一次无法继续定位和运动的直接终止原因。

## 昨天没有证明、当前只能在真狗上回答的两项

### 1. 起点是否能进入 RUNNING

昨天最后一次在 coordinator 的 `localizing` 阶段就因 follower 生命周期竞态
结束，没有留下足够长的配准窗口。因此不存在一个有证据支持的
“NDT/GICP 算法失败根因”。

当前 R5 的变化：

- 使用平台重新确认的整体 SE(2) 对齐，旋转约 `-15.74°`；
- 使用 28 个 approved 固定物形成的 `stable_layer.pcd` 验证配准；
- 起点先采集 5 帧，连续确认后才允许 coordinator 进入 `RUNNING`；
- 未收到明确 `RUNNING + localization_ready=true` 时，运动始终为零。

这些改动让配准具备了正确输入和可判断结果，但不能代替现场实时点云。

### 2. 一次 0.918 秒的 odometry 时间戳过旧

昨天只看到一次：

```text
Rejecting odometry timestamp: age=0.918s
```

单个样本可能是启动调度瞬态，也可能是持续的时间戳问题。没有足够日志能把它
定成持续根因。R5 没有通过放宽门限掩盖它，而是在启动定位器前测量 10 秒
`/Odometry` 和 `/cloud_registered_body` 的样本数、P95 年龄、未来时间和
非递增时间戳。若真实输入仍持续超过 `[-0.10, 0.50]` 秒范围，本次启动会明确
失败并留下 `input_timestamps.json`，而不是继续到一个永远无法收敛的状态。

## 本次新增的 GoGuard 固定入口

临时策略按用户要求实现：

```text
GoGuard start_patrol
  -> 忽略 fileName / routeUrl / CSV 内容
  -> 固定启动 xbf9-horizontal-clean-r1
  -> 固定使用已对齐 CSV、PCD、28 个固定物、8 个 checkpoint

GoGuard stop_patrol
  -> 固定停止这套 XBF 任务
  -> 最终调用 SDK2 StopMove
```

原 `go2_saas_agent.py` 文件不被覆盖；systemd drop-in 只把 command 服务入口
换成一个薄桥接程序。心跳、平台认证、命令轮询、commandId 去重、结果上传、
video 和 outbox 仍使用原代码。

`start_patrol` 向平台先返回 `running`，让同一个 command-loop 可以继续接收
紧急的 `stop_patrol`。狗端只有 coordinator 真正进入 `RUNNING` 后才放行
非零 follower 指令。

## 当前能作出的准确判断

- 昨天五项已证实的启动根因均已进入当前代码。
- 平台重新导出的 PCD/CSV/地标/checkpoint 已完成离线哈希与语义绑定。
- GoGuard 原 agent 的全部 6 个 start 别名和 3 个 stop 别名均已接入固定
  handler；平台传入的假 CSV/URL 不会进入下载链。
- 已在 Linux 环境复现并验证“PID 文件尚未生成时立即 stop”的窗口，以及
  `PID=PGID=SID` 生命周期；停止后没有遗留监督器或子进程。
- 当前精确源码已在 ARM64 Ubuntu 20.04 / ROS 2 Foxy 环境从空 build 目录
  重建 4 个包；colcon 汇总为 `39 tests, 0 errors, 0 failures, 0 skipped`。
- 当前代码具备到达真实起点配准测试的条件，不再会因昨天已知的五类问题提前
  退出。
- 尚不能声称真狗一定能够在当前摆放位置配准成功，也不能声称 600 米已跑通。
  现场第一次 `start_patrol` 的 `route_ready.json`、localizer 和 coordinator
  日志才是这个问题的最终证据。

# Go2 xbf9 R5：给现场部署人员

## 只使用这个目录

Git 中的正式候选目录：

```text
handoffs/2026-07-27-go2-xbf9-prepared-runtime-r5/localization_upgrade
```

它基于干净 `main` 提交
`8d861d564b2dc5c6a1fb58074b9a2ea15fa92af9` 制作。不要把昨天现场分支、
patch、release、日志或旧 R3 的地图混入本目录；不要覆盖
`realtime_dog_end_code` 和 `/home/unitree/go2_fastlio_ws` 的原文件。

本目录已经内置用户新确认的地图、对齐 CSV、28 个固定物和 8 个 checkpoint。
不需要在狗上再次运行平台导入器，也不需要另传原始 PCD。

## 1. 复制与构建

把本目录完整复制到狗端一个新的 release 目录，再让
`/home/unitree/localization_upgrade` 指向它。若该路径已经存在，先记录它指向
哪里并保留原目录，方便回退，不要直接删除。

进入新目录：

```bash
cd /home/unitree/localization_upgrade
bash scripts/deploy_localization_overlay.sh
python3 scripts/verify_xbf_bundle_offline.py
```

离线核验必须显示：

```text
map_id: xbf9-horizontal-clean-r1
route_points: 1277
checkpoints: 26,161,274,368,577,737,907,1040
approved_landmarks: 28
no ROS node was started and no motion command was published
```

构建或核验失败时不要现场改源码，保存完整输出后回传。

## 2. 运行前状态

1. 按原方式启动 Livox 和 FAST-LIO。
2. 确认 `/Odometry` 与 `/cloud_registered_body` 持续发布。
3. 停止或暂停旧 SaaS 巡检 command-loop；video/outbox 可以继续。
4. 保持遥控器和急停可用，狗周围留出安全空间。
5. 确认狗端真实 MID-360/IMU/base 外参与当前稳定版本一致。

然后：

```bash
cd /home/unitree/localization_upgrade
export GO2_INPUT_EXTRINSICS_VERIFIED=1
export GO2_SDK_IF=eth0
bash scripts/preflight_xbf_patrol.sh
```

预检中的 `deployment_ready=false` 和
`field_truth_verified=false` 是“尚未真狗验收”的记录提示，不会阻止启动。
哈希、topic、旧控制源、UDP 端口或真实狗源码不一致则会停止，这是防止两套程序
同时控制狗，不是人工审批门。

## 3. 第一次只做静止定位

```bash
export GO2_XBF_CALIBRATION_ONLY=1
bash scripts/start_xbf_patrol.sh
```

该模式速度和角速度都锁为零，但 SDK receiver 初始化会调用 `StandUp()` 和
`BalanceStand()`，狗可能改变站姿。成功标准不是“进程还在”，而是：

```text
/tmp/go2_xbf_patrol/logs/route_ready.json
  ready: true

/checkpoint_localization/route_status
  state: 3
  localization_ready: true
```

停止：

```bash
bash /home/unitree/localization_upgrade/scripts/stop_xbf_patrol.sh
```

若未进入 `RUNNING`，请一次性回传：

```text
/tmp/go2_xbf_patrol/logs/
/tmp/go2_xbf_patrol/patrol.pids
ros2 topic echo --once /checkpoint_localization/route_status
```

不要放宽配准门限或时间戳门限来“试着跑”。

## 4. 5～10 m 低速短测

静止定位通过后：

```bash
unset GO2_XBF_CALIBRATION_ONLY
export GO2_XBF_PATROL_SPEED=0.10
export GO2_XBF_MAX_YAW_RATE=0.30
export GO2_XBF_LOOP_MODE=once
bash scripts/start_xbf_patrol.sh
```

确认三件事：

1. 狗首先进入正确的 CSV 方向，而不是沿错误的平行道路走。
2. 实际落点与可走路面一致。
3. 停止脚本、遥控停止和异常清理都能让 UDP 5005 释放且没有残留 XBF 进程。

短测通过后再恢复默认 `0.20 m/s`；不要第一次就做 600 m 全程。

## 5. 第一个 checkpoint 验证

第一个 checkpoint 是 waypoint 26，约在路线 12 m 处，对应固定物
`AUTO-P07`。到点后应：

```text
follower 命令被门控为零
-> 狗停车
-> localizer 重定位
-> 状态重新变为 RUNNING
-> follower 自动继续
```

先确认这个闭环，再验证后面的 7 个 checkpoint。

## 6. 回退

```bash
bash /home/unitree/localization_upgrade/scripts/stop_xbf_patrol.sh
```

确认没有 XBF 进程、UDP 5005 已释放、最终 `StopMove` 已发送后，把
`/home/unitree/localization_upgrade` 恢复为部署前状态，并恢复旧 SaaS
command-loop。地图、release 和日志可以保留用于取证，不需要覆盖原 FAST-LIO
工作区。

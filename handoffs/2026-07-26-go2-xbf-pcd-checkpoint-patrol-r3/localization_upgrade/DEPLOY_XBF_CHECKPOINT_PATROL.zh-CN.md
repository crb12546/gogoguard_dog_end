# XBF 起点与 Checkpoint 自动校准：部署人员手册

## 一、交付行为

程序保留真狗当前的 `waypoint_follower_go2_2`。PCD 定位只修正 follower 看到
的 odometry，不替换它的 CSV 跟线算法：

```text
/Odometry
  -> PCD map<-odom 校准
  -> /checkpoint_localization/aligned_odometry
  -> waypoint_follower_go2_2
  -> /checkpoint_localization/follower_cmd
  -> checkpoint coordinator
  -> /checkpoint_localization/gated_cmd
  -> unitree_safe_cmd_node
  -> /cmd_vel
  -> UDP sender（127.0.0.1:5005）
  -> SDK2 UDP receiver（GO2_SDK_IF，默认 eth0）
  -> Unitree SDK2
```

启动时在起点停车定位；到 waypoint 373、585、787 再停车定位。每次定位成功后
自动继续，不需要在平台上人工批准。普通路段关闭高算力点云配准。

## 二、复制与一次性构建

将完整目录复制到：

```text
/home/unitree/localization_upgrade
```

执行：

```bash
cd /home/unitree/localization_upgrade
bash scripts/deploy_localization_overlay.sh
```

原工作区 `/home/unitree/go2_fastlio_ws` 必须已经构建这些程序：

```text
go2_fastlio_patrol/waypoint_follower_go2_2
go2_fastlio_patrol/unitree_safe_cmd_node
go2_cmd_vel_bridge/cmd_vel_udp_sender
build/go2_cmd_vel_bridge/go2_sdk2_udp_receiver
build/go2_cmd_vel_bridge/go2_sdk2_motion_probe
```

## 三、启动前

按狗端原方式启动：

- Livox MID-360；
- FAST-LIO，确保存在 `/Odometry` 和 `/cloud_registered_body`。

不要启动 SaaS 的旧“开始巡检”，也不要单独启动旧 follower、safe node 或
UDP bridge。本任务会自行启动完整运动链，不需要预先启动 Unitree graph，也
不依赖 `/api/sport/request`。

默认 SDK 网卡是 SaaS 当前使用的 `eth0`。如现场不同：

```bash
export GO2_SDK_IF=<真实网卡名>
```

确认 MID-360、IMU、`base_link` 外参、轴向与时间基准后设置：

```bash
export GO2_INPUT_EXTRINSICS_VERIFIED=1
```

这是机器配置声明，不是每次定位后的人工审批按钮。

## 四、离线交付包核对

可在不启动 ROS 节点、不发送运动命令的情况下运行：

```bash
cd /home/unitree/localization_upgrade
python3 scripts/verify_xbf_bundle_offline.py
```

它核对：

- R2 map、manifest、tile 与 publication 哈希链；
- 路线、checkpoint、源 PCD/CSV 的绑定；
- waypoint 373、585、787；
- production 定位/协调器参数；
- 原 follower、safe node、UDP sender、receiver、motion probe 源码哈希；
- production 启停脚本确实使用 UDP 5005、20 Hz safe node 和最终 StopMove。

## 五、在线预检

```bash
cd /home/unitree/localization_upgrade
export GO2_INPUT_EXTRINSICS_VERIFIED=1
export GO2_SDK_IF=eth0
bash scripts/preflight_xbf_patrol.sh
```

预检自动确认：

- ROS 包和六个运行程序存在；
- SDK 网卡、receiver 二进制、StopMove probe 可用；
- FAST-LIO 两个输入 topic 类型正确；
- `/patrol_cmd` 没有旧发布者/订阅者，`/cmd_vel`、内部命令和定位 topic
  没有旧发布者；rosbag 等只读订阅不会阻止启动；
- UDP 5005 没有被旧 receiver 占用；
- 没有旧定位服务实例；
- 地图、路线和 checkpoint 精确匹配。

`field_truth_verified=false` 只显示现场复核警告，不阻止运行。其他失败说明当前
确有拿错文件、缺程序、网卡不对或双控制源，脚本不会靠人工按钮绕过。

## 六、启动

正式巡检：

```bash
cd /home/unitree/localization_upgrade
export GO2_INPUT_EXTRINSICS_VERIFIED=1
export GO2_SDK_IF=eth0
bash scripts/start_xbf_patrol.sh
```

脚本固定按以下顺序启动：

1. 地图定位器；
2. checkpoint coordinator；
3. `go2_sdk2_udp_receiver`；
4. `cmd_vel_udp_sender`；
5. 原 `unitree_safe_cmd_node`；
6. 原 `waypoint_follower_go2_2`。

默认巡检参数：

```text
线速度上限：0.20 m/s
角速度上限：0.45 rad/s
safe node：20 Hz
UDP：127.0.0.1:5005
SDK 网卡：eth0
```

safe node 使用原 SaaS 的障碍参数：停车距离 0.80 m、恢复距离 1.00 m、最少
15 点，ROI 为 x=0.35..1.50、y=-0.30..0.30、z=0.30..0.90 m。

需要换速度：

```bash
export GO2_XBF_PATROL_SPEED=0.10
export GO2_XBF_MAX_YAW_RATE=0.30
```

只定位不运动：

```bash
export GO2_XBF_CALIBRATION_ONLY=1
bash scripts/start_xbf_patrol.sh
```

calibration-only 会把 follower、safe node 和 UDP sender 的线速度/角速度上限
同时设为 0，不需要修改代码。

## 七、自动冲突检查

没有人工安全门。coordinator 与监督脚本自动检查真实执行链：

```text
/checkpoint_localization/gated_cmd：1 个发布者、至少 1 个订阅者
/cmd_vel：1 个发布者、至少 1 个订阅者
/patrol_cmd：0 个发布者、0 个订阅者
```

coordinator 的 graph guard 也被运行时改为检查 `/cmd_vel`，不依赖
`/api/sport/request`。额外的 rosbag、监控窗口等只读订阅允许存在；任何进程
退出或出现第二个运动命令发布者，本任务自动结束并按停车顺序清理。

## 八、观察

另开一个已 source 环境的终端：

```bash
ros2 topic echo /checkpoint_localization/route_status
ros2 topic echo /localization/status
ros2 topic info /checkpoint_localization/gated_cmd
ros2 topic info /cmd_vel
```

日志：

```text
/tmp/go2_xbf_patrol/logs/localizer.log
/tmp/go2_xbf_patrol/logs/coordinator.log
/tmp/go2_xbf_patrol/logs/sdk_receiver.log
/tmp/go2_xbf_patrol/logs/cmd_vel_sender.log
/tmp/go2_xbf_patrol/logs/safe_cmd.log
/tmp/go2_xbf_patrol/logs/follower.log
/tmp/go2_xbf_patrol/logs/stopmove.log
```

正常状态：

```text
settling -> activating -> resetting -> localizing
-> capturing -> deactivating -> running
```

进入 `running` 后才透传 follower 命令。到 checkpoint 后重新进入
`settling`，成功后自动继续。真正定位失败才进入 `FAULT_HOLD` 并保持零速度；
排除遮挡、点云或地图问题后可请求重新计算：

```bash
ros2 service call /checkpoint_localization/retry_after_fault \
  std_srvs/srv/Trigger "{}"
```

这不是正常流程中的批准步骤，只是故障后的重试。

## 九、停止

前台按 `Ctrl+C`，或：

```bash
bash /home/unitree/localization_upgrade/scripts/stop_xbf_patrol.sh
```

监督进程最多等待约 35 秒完成顺序停车：

1. 停 follower；
2. coordinator、safe node、sender、receiver 持续把零速度送到 SDK2；
3. 停定位器和 coordinator；
4. 停 safe node；
5. 停 UDP sender；
6. 停 SDK2 receiver；
7. 调用 `go2_sdk2_motion_probe --iface "$GO2_SDK_IF" stop`。

即使监督进程已经异常退出，停止脚本也会按 PID 文件中的六个进程组执行同样
的 fallback 和最终 StopMove，不使用广域 `pkill`。

## 十、首次现场边界

- 狗放在 CSV 起点约 2 m 内，粗略方向误差约 35°内；
- 当前 CSV→PCD 旋转约 `-15.8°`；
- 平移 `(-0.8 m, +0.2 m)` 有约 `1.5 m` 横向歧义；
- `0.3°` 是多次算法解的一致性要求，不是外部仪器证明的绝对误差；
- 程序不负责从校园任意地点规划到路线起点。

第一次现场建议先跑 calibration-only，再以 `0.10 m/s` 走 5–10 m，然后跑到
第一个 checkpoint。确认路线落在真实路面、停车定位和自动继续均正常后，再跑
完整约 600 m 路线。这个是新地图/路线的一次性实地验收，不是每次巡检审批。

# Go2 U2：XBF PCD + CSV 起点与 Checkpoint 自动校准包

## 结论

这是一套保留原 CSV follower、加入 PCD 停车重定位的固定任务部署包。正式入口：

- `DEPLOY_XBF_CHECKPOINT_PATROL.zh-CN.md`
- `scripts/preflight_xbf_patrol.sh`
- `scripts/start_xbf_patrol.sh`
- `scripts/stop_xbf_patrol.sh`

默认任务绑定：

```text
源 PCD：xbf-2 2.pcd
源 PCD SHA-256：3526e4f116586d3594c0afa45efb3fb254e4eca1bf89fa21f18896a558ee5aa2
源 CSV：xbf9_horizontal_clean.csv
源 CSV SHA-256：b4abadd38c30f5904f4cfe10eb529b8c1a4940ba023019847ea3959c48fd53a2
部署地图：xbf-2026-07-26-map-reviewed-r2
部署路线：xbf9_horizontal_clean.map-reviewed-r2.csv（1277 点）
Checkpoint：373、585、787
```

启动时会按原始字节核对地图、路线、checkpoint 与部署元数据，避免拿错任务。

## 狗实际怎么走

```text
/Odometry
  -> PCD 定位得到并冻结 map<-odom
  -> /checkpoint_localization/aligned_odometry
  -> 原 waypoint_follower_go2_2
  -> /checkpoint_localization/follower_cmd
  -> checkpoint coordinator
  -> /checkpoint_localization/gated_cmd
  -> 原 unitree_safe_cmd_node（20 Hz、原障碍参数）
  -> /cmd_vel
  -> cmd_vel_udp_sender（127.0.0.1:5005）
  -> go2_sdk2_udp_receiver（默认 eth0）
  -> Unitree SDK2 Move
```

本包不再依赖 `/api/sport/request` 的 graph 执行端。UDP sender、receiver 和退出
时的 SDK2 `StopMove` 都由启停脚本管理。

运行顺序：

1. 起点保持零速度，采集 5 组静止 MID-360 扫描；
2. 对起点附近的静态地图做有限 SE2 多初值 NDT/GICP；
3. 多次结果一致后冻结 `map<-odom`；
4. 原 follower 按 CSV 跟线；
5. 到 373、585、787 时停车、重新配准、更新冻结变换；
6. 配准成功自动继续，不需要网页批准。

正常路段不连续跑 NDT/GICP，只对 odometry 做低成本二维变换。因此校准是按路线
checkpoint 触发，不是每 5 秒或 10 秒触发。

## 标记物怎么参与

R2 稳定层包含墙面/建筑直角、固定路沿和路灯 H/I。在线不是寻找某个单独的
“PCD 点”，也不是先识别物体类别；它把当前一片雷达扫描与 checkpoint 周围
的静态地图 tile 做几何配准。多方向墙面、路沿和灯杆共同约束 x、y、yaw。

```text
373：路灯 H + K01-W 路沿
585：路灯 I + K-B 路沿
787：C01 双墙直角 + K-A 路沿
```

树、车辆、人群、可移动围挡、弱杆和疑似重影没有作为唯一定位依据。

## 没有人工审批门

定位成功后自动走，checkpoint 成功后自动继续。保留的都是自动故障检查：

- PCD/地图/CSV/checkpoint 身份不一致；
- 定位不收敛或结果不一致；
- Odometry、点云或命令超时；
- 旧 `/patrol_cmd` 链、第二个 `/cmd_vel` 控制源或旧定位实例仍在运行；
- 真狗 follower、safe node、UDP bridge 源码与已核对版本不一致。

这些检查没有“人工点同意”的步骤。通过即运行；失败则保持零速度并给出日志，
避免两套程序同时控制狗。

## 部署

狗端先按原方式启动 Livox 和 FAST-LIO，但不要启动 SaaS 旧巡检。然后：

```bash
cd /home/unitree/localization_upgrade
bash scripts/deploy_localization_overlay.sh

export GO2_INPUT_EXTRINSICS_VERIFIED=1
export GO2_SDK_IF=eth0
bash scripts/preflight_xbf_patrol.sh
bash scripts/start_xbf_patrol.sh
```

停止：

```bash
bash /home/unitree/localization_upgrade/scripts/stop_xbf_patrol.sh
```

停止时先让整条链持续发送零，再停止 safe node、UDP sender、UDP receiver，
最后直接调用：

```text
go2_sdk2_motion_probe --iface eth0 stop
```

脚本只停止 PID 文件中记录的进程组，不使用广域 `pkill`。

## 可选模式

只定位、绝不产生非零命令：

```bash
export GO2_XBF_CALIBRATION_ONLY=1
bash scripts/start_xbf_patrol.sh
```

正式巡检默认 `0.20 m/s`、最大角速度 `0.45 rad/s`。也可显式设置：

```bash
export GO2_XBF_PATROL_SPEED=0.10
export GO2_XBF_MAX_YAW_RATE=0.30
export GO2_XBF_LOOP_MODE=once
```

## 已验证与现场边界

离线已验证哈希链、1277 点路线、3 个 checkpoint、30 项 Python 测试、C++ 核心
测试，以及原 follower、safe node、UDP sender/receiver/motion probe 的源码
哈希。脚本接线与原 SaaS 使用的 SDK2 UDP 链一致。另用本 PCD 在起点和三个
checkpoint 做了 68 组同图配准压力测试：起点 17/17 组通过并达到 10 cm /
0.3°；三个 checkpoint 在 CSV 已把狗带到附近、初始朝向误差 ±1°时均恢复到
不超过 7.9 cm / 0.293°。这属于算法上限测试，不替代真狗实时点云验收。

本机没有连接真狗，因此 MID-360 外参、现场绝对精度、CPU 峰值和整条约 600 m
路线仍需在 U2 上验证。当前 CSV→PCD 旋转约 `-15.8°`；平移
`(-0.8 m, +0.2 m)` 有约 `1.5 m` 的道路横向歧义。这个 `field_truth` 状态只
打印警告，不阻止启动。第一次现场仍建议先用 calibration-only 看定位结果，再
以 `0.10 m/s` 走 5–10 m 核对路线确实落在路面上。

狗应放在 CSV 起点约 2 m 内、粗略朝向误差约 35°内。本版本负责精确对齐和沿
CSV 行走，不包含从校园任意地点绕障规划到起点。

## 历史文件

`CHECKPOINT_FOLLOWER_CANDIDATE.zh-CN.md` 记录方案演进；正式部署只使用本页列出的
production 脚本和 R2 任务文件。

# Go2 XBF PCD 自动校准 R3：Git 交付入口

这个目录是交给机器狗部署人员的独立交付件。它不会覆盖仓库现有的
`orin_go2_fastlio_ws` 或 `realtime_dog_end_code`；部署人员拉取代码后，再手动
把本目录中的 `localization_upgrade` 复制到机器狗。

## 本次固定任务

- PCD：`xbf-2 2.pcd`；
- CSV：`xbf9_horizontal_clean.csv`；
- 路线：1277 点，约 602.4 m；
- 停车重定位点：waypoint 373、585、787；
- 运动控制继续使用狗端原有 `waypoint_follower_go2_2` 和 SDK2 UDP 链。

先阅读：

1. `先看我-交给部署人员.md`；
2. `localization_upgrade/交给部署人员-先看我-R3.md`；
3. `localization_upgrade/DEPLOY_XBF_CHECKPOINT_PATROL.zh-CN.md`。

## 部署到狗

狗端已有工作区：

```text
/home/unitree/go2_fastlio_ws
```

将本目录中的 `localization_upgrade` 完整复制到：

```text
/home/unitree/localization_upgrade
```

然后在狗上执行：

```bash
cd /home/unitree/localization_upgrade
bash scripts/deploy_localization_overlay.sh
python3 scripts/verify_xbf_bundle_offline.py

export GO2_INPUT_EXTRINSICS_VERIFIED=1
export GO2_SDK_IF=eth0
bash scripts/preflight_xbf_patrol.sh
bash scripts/start_xbf_patrol.sh
```

停止：

```bash
bash /home/unitree/localization_upgrade/scripts/stop_xbf_patrol.sh
```

## 状态说明

电脑端构建、单元测试、哈希核对和同图点云压力测试已经通过；本机没有连接真实
机器狗。当前制品仍明确记录 `field_truth_verified=false`，CSV→PCD 的横向平移
存在约 1.5 m 离线歧义。因此第一次部署先静止定位，再以 0.10 m/s 走 5～10 m
并验证第一个 checkpoint；这是一次现场闭合，不是以后每次巡检的人工审批。

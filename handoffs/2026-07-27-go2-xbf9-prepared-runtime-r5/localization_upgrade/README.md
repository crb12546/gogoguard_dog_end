# Go2 U2 xbf9 PCD 定位巡检候选版（R5）

这是基于用户在平台重新确认后的
`xbf9_horizontal_clean.go2-patrol-preparation.zip` 生成的任务专用交付目录。
部署人员不需要再现场拼接 PCD、CSV、标记物或 checkpoint，也不要使用
2026-07-26 的旧 R3 交付目录。

## 本目录已经绑定好的内容

| 项目 | 值 |
|---|---|
| 原始 PCD | `xbf-2 2.pcd`，1,085,459 点 |
| 原始 PCD SHA-256 | `3526e4f116586d3594c0afa45efb3fb254e4eca1bf89fa21f18896a558ee5aa2` |
| 原始 CSV | `xbf9_horizontal_clean.csv`，1,277 点 |
| 原始 CSV SHA-256 | `b4abadd38c30f5904f4cfe10eb529b8c1a4940ba023019847ea3959c48fd53a2` |
| 平台整体对齐 | 水平旋转 `-0.2747470147 rad`（约 `-15.74°`），平移 `(0, 0) m` |
| 运行地图 | `maps/xbf9-horizontal-clean-r1` |
| 执行路线 | `routes/xbf9_horizontal_clean.aligned.csv` |
| 已确认固定物 | 28 个 |
| 中途停车校准点 | 8 个：26、161、274、368、577、737、907、1040 |

`TASK_BUILD_REPORT.json` 记录完整哈希、数量和验证边界；
`task_provenance/` 保存平台 ZIP 及其解包后的原始证据。

## 狗端执行逻辑

1. 起点保持零速度，用 MID-360 实时扫描和完整 PCD 地图做有限范围配准。
2. 28 个已确认固定物生成的 `stable_layer.pcd` 独立验证这次配准是否得到固定结构支持。
3. 配准通过后冻结 `map <- odom`，原
   `waypoint_follower_go2_2` 按已经对齐的 CSV 跟线。
4. 到 8 个 checkpoint 时停车，重新配准并更新冻结变换，成功后自动继续。
5. 普通路段不连续运行重型 NDT/GICP，不改变原 CSV follower 的跟线算法。

运行链为：

```text
MID-360 / FAST-LIO
  -> PCD localizer
  -> aligned_odometry
  -> 原 waypoint follower
  -> checkpoint coordinator
  -> 原 safe cmd
  -> 原 UDP 5005 / SDK2 运动链
```

## 现场问题已经进入正式源码的修正

- 大描述符 JSON 不再通过 yaml-cpp 造成约 22 GiB 内存膨胀。
- 全部新增 ROS 2 节点固定使用 Cyclone DDS。
- Foxy 的地图路径和路线身份写入运行时 YAML，不依赖参数覆盖顺序。
- SDK receiver 最长等待 30 秒，UDP 5005 检查不依赖 `ss` 固定列。
- 组件采用确定的 `setsid()+exec()` 生命周期，不再发生 follower PID/PGID 竞态。
- 停止脚本按本次唯一 run-id 清理精确进程组并最终调用 SDK2 `StopMove`。
- follower 接好后必须看到 coordinator 明确进入 `RUNNING`，否则不放行运动。
- 启动前记录 `/Odometry` 和 `/cloud_registered_body` 时间戳分布。

## 先读

- 部署步骤：`DEPLOYER_README.zh-CN.md`
- 算法与现场验收：`DEPLOY_XBF_CHECKPOINT_PATROL.zh-CN.md`
- 离线核验：`python3 scripts/verify_xbf_bundle_offline.py`

## 真实边界

本目录已经通过 ZIP、PCD、CSV、地图、路线、地标和 checkpoint 的离线交叉校验，
但本机没有连接真实机器狗。当前不能声称实时点云已经定位成功、coordinator
已经进入 `RUNNING`，也不能声称约 600 m 全程已经跑通。第一次现场必须先做
静止定位和 5～10 m 低速短测。

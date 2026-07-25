# 第一批算力/存储优化记录

实施时间：2026-07-21（Asia/Shanghai）

## 已实施

- CSV 录制不再自动启动 Z-1Pro 视频。
- PCD 采集不再自动启动 Z-1Pro 视频。
- CSV、PCD、巡检、手动连续视频增加远端进程级互斥保护。
- `go2-saas-video.service` 保留运行，但无巡检标记时只低频等待，不连接 RTSP、不生成文件。
- 平台启动巡检后创建 `patrol_video.active`，巡检停止时删除；路线正常结束后，当前视频分段收尾并自动回到空闲。
- 摄像头文件列表由“每个文件启动两次 stat”改为一次 `find` 扫描，后台刷新间隔由 8 秒改为 30 秒。

## 明确未改

- FAST-LIO、route_recorder、CSV 质量阈值和运动合理性阈值。
- waypoint_follower 的速度、拐角、yaw、横向纠偏参数。
- nvpmodel、在线 CPU 核数和系统功耗模式。
- systemd unit 文件、相机采集脚本和上传脚本。

## 部署与清理结果

- 狗端部署文件：`/home/unitree/go2_fastlio_ws/scripts/go2_saas_agent.py`
- 部署后 SHA-256：`03b93f17ba899689d9f0c2414e7471a799d78a72234410428b9e735a7a8e042e`
- 删除视频：11,181 个，共 25,890,930,870 bytes（约 24.11 GiB）。
- 删除对应 outbox 元数据：pending 3、inflight 0、failed 41。
- 视频文件已永久删除，不能通过本备份恢复。

## 现场修复

- 2026-07-21 21:13：修复本地控制台停止 CSV 时因缺少 `import os` 导致的 HTTP 500。
- 受影响路线 `xbf0721-13.csv` 已用正常 TERM 信号收尾，质量报告为 `valid=true`；未发生强制终止。

## 备份内容

- `local_before/`：本地控制台与本地代理修改前版本。
- `remote_before/`：狗端代理、录像脚本和 systemd unit 修改前版本。
- `remote_after/`：实际部署到狗端的第一批代理版本。
- `rollback_phase1.sh`：代码回滚脚本；执行后旧版常驻录像行为会恢复。

# 15 · 现场操作台 patrol_console(本地 UI,SaaS 的人工对照)

> 原则同 00。核心文件(已逐行读):`tools/patrol_console/server.py`(FastAPI 本地服务)。
> 前端 `tools/patrol_console/static/`(html/js,未逐行,属展示层)。

## 一、定位:它跑在本机,不在狗上
一个 **FastAPI Web 服务**,监听 `127.0.0.1:8642`(仅本机浏览器)。设计三原则(docstring):
1. **绝不修改狗上文件**——一切通过 **SSH 调狗上已有脚本/命令**;
2. **遥测脚本经 stdin 注入**远端 `python3` 运行,不在狗上落盘;
3. 只监听 localhost。
→ 它是 **SaaS 云端自动路径(08)的"人工现场"对照**:同一批底层脚本(base_bringup / route_recording_blackbox / go2_saas_agent patrol-start / manual_route_anchor …),一个由云端命令驱动,一个由操作员点按钮驱动。

## 二、连接与两个后台线程
- **SSH 主机自动选**:别名 `go2wired`(网线最快)→ `go2`(热点)→ `go2home`;断了就重探(自动处理网线/热点切换)。
- **telemetry_worker**:SSH 注入一段 ROS 遥测脚本(`nice -n 10`),订 `/lf/lowstate` `/lf/sportmodestate` `/Odometry`,每秒吐 JSON → 台上显示电量/电机温/SOC/位姿/sport。
- **status_worker**(每 3s):一条复合 SSH 命令,查关键进程(livox/fastlio/recorder/safe/follower/pcd/camera_loop/saas_loop)、录制行数、tail 各 console 日志、CPU 温度、WiFi 信号、Livox ping、**Z1 相机 ping/RTSP/GCU 探测**、`camera.env` 源、`/run/go2-4g-manager-state.json`。并把 follower/safe 日志**摘要成中文状态**(巡线运行中/卡住/障碍物限停/…)。

## 三、动作(全部白名单 `ACTIONS`)
起停底座、起停录制(→ `route_recording_blackbox`)、起停安全节点、**起停巡检**(→ `go2_saas_agent patrol-start/patrol-stop`,`--localization-mode manual_anchor`)、**急停**(pkill follower,靠 safe 节点 0.5s cmd 超时持续发 Move(0,0,0))、stop_all_control、tail 日志、**起停在线建 pcd**、相机(probe/preset/snapshot/record/loop)、saas(heartbeat/manifest/command-result/video-segment/patrol-loop)。
- ⚠️ **运动类动作(start_safe/start_follower)必须先 `armed` 解锁**(非 dry_run)——UI 层安全联锁。
- **模式互斥守卫**:重模式(巡检/录制/建图/视频)不能并行,远端 ps 检查冲突进程。

## 四、在线建图 `go2map_capture`(解决之前的悬念)
`start_pcd` 动作**注入**一段 Python(`/tmp/go2map_capture.py`):订 `/cloud_registered` → 每 3 帧取一帧、点抽样 2、累积 → 每 20s voxel(0.08)压缩 → 收到 TERM 时存 ASCII pcd 到 **`maps/console/<name>.pcd`**;带 session_guard(pcd 模式)。
- → 这就是遍布各处冲突守卫里的 `go2map_capture`。**现场在线建图**路径,产物落 `maps/console/`,**正是 SaaS `resolve_route_map` 找同名 pcd 的目录**(`PCD_DIR`)。
- 与 06 的离线 `go2_loop_backend`(从 rosbag 重建 + 回环优化)是**两条建图路**:现场快速在线 vs 离线高质量。

## 五、Web API
`/api/status`、`/api/routes`(列表+起点)、`/api/route_points`(2D 预览)、`/api/pcd_list`、`/api/pcd_points`(远端抽样避免传大文件)、`/api/pcd_pack`(base64 打包 xyz 给 3D viewer,重心归零)、`/api/camera_files`、`/api/download`、`/api/file`(视频带 Range 流式)、`/api/action`。

## 六、要点
- 两条"驱动狗"的路:**云端 SaaS(自动、远程)** vs **操作台(人工、现场 SSH)**,底层脚本共用。
- 操作台里的 safe 节点参数是"测试档"(stop_distance 0.40、roi_x 0.35~0.90),与生产 saas(0.80、0.35~1.50)不同 → 又一处"同一节点不同参数,以启动串为准"。
- 现场 pcd(maps/console)与离线 pcd(maps/loop_backend)两套,注意别混。

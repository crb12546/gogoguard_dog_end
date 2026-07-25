# 15 · 现场操作台 patrol_console(本地 UI,SaaS 的人工对照)

> 原则同 00。核心文件(已逐行读):`tools/patrol_console/server.py`(FastAPI 本地服务)。
> ⚠️ 前端 `tools/patrol_console/static/`(html/js,展示层)**在仓库中完全缺失**——无法核验,详见 §五末与核验状态。

---

## 核验状态(本轮 2026-07-25)

- 本轮已对**磁盘仓库源码逐条核**:`tools/patrol_console/server.py`、`orin_go2_fastlio_ws/scripts/go2_saas_agent.py`、`.../go2_fastlio_patrol/unitree_safe_cmd_node.py`、`go2_loop_backend/`、两包 `setup.py`,并交叉了 `analysis/.../runs/xunjian-20260725-06`、`-07` 两份 `manifest.txt`。
- 结论:核心断言(端口/双后台线程/动作白名单/在线建 pcd 参数/Web API/safe "测试档 vs 生产")对源码**基本准确,无颠覆性错误**。但 **4 处需修正/补全**(下文已就地标注):
  1. 文档所述前端 `tools/patrol_console/static/`(index.html / *.js / vendor/)**在仓库中完全缺失**、未被 `.gitignore` 排除——无法核验,且"照仓库直接启动 `server.py` 会 **RuntimeError**"(见 §五末)。
  2. safe 参数实为**三档**——文档漏了 node `declare_parameter` 默认值(stop_distance=0.70 / roi_x_max=1.20);且 console 的 0.40"测试档"**只在独立 `start_safe` 按钮生效**,console 驱动的**巡检**其实走 saas `patrol-start` = 生产 0.80(见 §六)。
  3. "maps/console 是 SaaS `resolve_route_map` 找 pcd 的目录"字面对,但该消费**仅在 pcd 重定位模式触发**;console / manifest 实跑的是 `manual_anchor` 模式(**根本不查 pcd**)(见 §四)。
  4. §二 `status_worker` 关键进程枚举**漏列 `cmd`**(`PROC_KEYS` 实为 **9 项**,含 `unitree_cmd_node`)。
- **狗上对照**:本篇所引源文件**全部无狗上副本对照【无狗上对照】**(狗上远端仅存 `laserMapping.cpp` / `lddc.cpp` / `lds.cpp` / `waypoint_follower_go2_2.py` 四份)。patrol_console 本就跑在**本机、非狗上**,无从与狗上比对;`go2_saas_agent` / `unitree_safe_cmd_node` 的狗上版是否与仓库一致**均无法验证**。仅 manifest 间接佐证**部分生产值确达狗端运行期**(speed=0.5 / loop=pingpong / manual_anchor / k_yaw=0.900 / follower sha `d205a596…` ≠ 仓库版);**manifest 不含 safe 节点 stop_distance/roi_x**,故这两项的"狗上生效值"属**代码路径推断**、非直证。

> **【源标签】约定**:`【code server.py:行】`=console 自身源码 / `【默认 …_node.py:行】`=节点 `declare_parameter` 默认值 / `【生产 saas go2_saas_agent.py:行】`=生产云端 `-p` 启动串 / `【狗上 dog:…】`=manifest 直证 / `【推断-未验】` / `【README对照】`。**冲突时优先信 `-p` 启动串与狗上 manifest。**

---

## 一、定位:它跑在本机,不在狗上
一个 **FastAPI Web 服务**,监听 `127.0.0.1:8642`(仅本机浏览器)`【code server.py:34 PORT=8642 / 1019 FastAPI / 1350 uvicorn.run host=127.0.0.1】`。设计三原则(docstring)`【code server.py:5-7】`:
1. **绝不修改狗上文件**——一切通过 **SSH 调狗上已有脚本/命令**;
2. **遥测脚本经 stdin 注入**远端 `python3` 运行,不在狗上落盘 `【code server.py:250-251 proc.stdin.write(TELEMETRY_SCRIPT)】`;
3. 只监听 localhost `【code server.py:1350 127.0.0.1】`。

→ 它是 **SaaS 云端自动路径(08)的"人工现场"对照**:同一批底层脚本(base_bringup / route_recording_blackbox / go2_saas_agent patrol-start / manual_route_anchor …),一个由云端命令驱动,一个由操作员点按钮驱动。

## 二、连接与两个后台线程
- **SSH 主机自动选**:别名 `go2wired` → `go2` → `go2home`,断了就重探 `【code server.py:33 HOSTS,注释"网线最快优先, 热点次之" / 273-274 断开时 STATE["host"]=None 触发重探 / 123-131 require_host→detect_host】`。
  - ⚠️ 注释只标了"网线/热点"**顺序**;"`go2`=热点、`go2home`=?"这类逐个标签属**文档推断**`【推断-未验】`。仅"`go2wired`=网线最快优先"有注释直证。
- **telemetry_worker**:SSH 注入一段 ROS 遥测脚本(`nice -n 10`),订 `/lf/lowstate` `/lf/sportmodestate` `/Odometry`,**每秒**节流吐 JSON → 台上显示 **电量/电机温/SOC/位姿/sport** `【code server.py:233 nice -n 10 python3 -u - / 162-164 三订阅 / 188 每秒节流 / 196-208 power_v·power_a·motor_temps·soc / 209-219 sport / 220-228 odom】`。
- **status_worker**(每 **3s**):一条复合 SSH 命令 `【code server.py:459 time.sleep(3)】`,查关键进程、录制行数、tail 各 console 日志、CPU 温度、WiFi 信号、Livox ping、**Z1 相机 ping/RTSP/GCU 探测**、`camera.env` 源、`/run/go2-4g-manager-state.json`,并把 follower/safe 日志**摘要成中文状态**(巡线运行中/卡住/障碍物限停/…)。
  - **进程枚举 `PROC_KEYS` 实为 9 项**(原文只写 8 项、**漏了 `cmd`=`unitree_cmd_node`**):`livox / fastlio / recorder / safe / follower / cmd / pcd / camera_loop / saas_loop` `【code server.py:284 PROC_KEYS / 293 $0~/[u]nitree_cmd_node/{cmd=1}】`。
  - 各探测点 `【code server.py:374 wc -l 录制行数 / 377-380 tail console_recorder·pcd·follower·safe.log / 381 thermal_zone0/temp / 382 iw wlan0 signal / 383 livox ping 192.168.1.161 / 389-391 z1 192.168.144.108 ping·554 RTSP·2332 GCU / 384 camera.env GO2_CAMERA_SOURCE / 392 /run/go2-4g-manager-state.json】`。
  - 中文摘要 `【code server.py:319-333 summarize_follower:巡线运行中/stuck 卡住/失败 / 336-348 summarize_safe:限停·前方障碍物/点云超时/等待巡线命令】`。

## 三、动作(全部白名单 `ACTIONS`,25 项)
`【code server.py:987-1013 ACTIONS 字典(25 项)/ 1323-1327 api_action 仅接受 ACTIONS.get(name)】`
起停底座、起停录制、起停安全节点、**起停巡检**、**急停**、stop_all_control、tail 日志、**起停在线建 pcd**、相机(probe/preset/snapshot/record/loop)、saas(heartbeat/manifest/command-result/video-segment/patrol-loop)。25 个动作名与文档枚举一一对应。

- 起停录制 → `route_recording_blackbox` `【code server.py:46 RECORDING_BLACKBOX=route_recording_blackbox.py / 569,584 act_start·stop_recorder】`。
- **起停巡检** → `go2_saas_agent patrol-start/patrol-stop`,`--localization-mode manual_anchor` `【code server.py:646-649 act_start_follower / 628 act_stop_follower】`(动作名 `start_follower/stop_follower` 内部即调 saas patrol-start/stop;`manual_anchor` 与 manifest 的 `localization_mode=manual_anchor` 一致)。
- **急停**:`pkill` follower,靠 safe 节点 **0.5s cmd 超时**持续发 `Move(0,0,0)` `【code server.py:660-663 act_estop pkill TERM+KILL waypoint_follower / 默认 unitree_safe_cmd_node.py:54 cmd_timeout=0.5 / 467-469 command_age>cmd_timeout→(0,0,0) reason=cmd_timeout / 491 publish_move】`。机制成立且**依赖 safe 节点在跑**。
  - 细节补充(两条 safe 启动路径的差异):
    - **cmd_timeout**:console 自身 `act_start_safe` **未设** cmd_timeout → 用节点默认 0.5 `【code server.py:600-607 无该参数;默认 unitree_safe_cmd_node.py:54=0.5】`;生产 saas 显式 `cmd_timeout:=0.5` `【生产 go2_saas_agent.py:2054】`。两者等值。
    - **输出话题**:console-safe 走默认 `''` → unitree_api `Request/Move` `【默认 unitree_safe_cmd_node.py:48,164,258-259】`;生产 safe 走 `Twist → /cmd_vel` `【生产 go2_saas_agent.py:2056 output_cmd_topic:=/cmd_vel】`。**零速等效**。
- ⚠️ **运动类动作(`start_safe`/`start_follower`)必须先 `armed` 解锁**(非 dry_run)`【code server.py:1015 MOTION_ACTIONS={start_safe,start_follower} / 1328-1331 api_action:name∈MOTION_ACTIONS 且 not armed 且 not dry_run → 403】`。
  - **精确化**:该联锁由 console 的 **FastAPI 服务端**(`api_action`)强制、非满足即返回 **403**,**并非纯前端**;`dry_run=True` 可绕过。原文"UI 层"应理解为"在 **console 侧**而非狗上",大意正确。
- **模式互斥守卫**:重模式(巡检/录制/建图/视频)不能并行,远端 `ps` 检查冲突进程 `【code server.py:541-554 _mode_conflict_guard(ps -eo … awk 匹配→exit 4)/ 608,788,875 各动作装配 mode_guard】`。

## 四、在线建图 `go2map_capture`(解决之前的悬念)
`start_pcd` 动作**注入**一段 Python(`/tmp/go2map_capture.py`)`【code server.py:810 cat>/tmp/go2map_capture.py】`:订 `/cloud_registered` → **每 3 帧取 1、点抽样 2**、累积 → **每 20s voxel(0.08)压缩** → 收到 TERM 时存 **ASCII pcd** 到 **`maps/console/<name>.pcd`**;带 session_guard(pcd 模式)。全部数值逐一命中 `【code server.py:697 VOXEL,FRAME_STRIDE,POINT_STRIDE=0.08,3,2 / 738 订 /cloud_registered / 742 每3帧 / 729 P[::POINT_STRIDE] / 763 >20s compact / 753-754,768-777 SIGTERM→写 ASCII / 787 out=PCD_DIR/name.pcd / 796-808 _session_guard mode=pcd】`。

- → 这就是遍布各处冲突守卫里的 `go2map_capture` `【code server.py:294 PROC_STATUS pcd / 512 _base_restart_guard / 610 act_start_safe mode_guard / 790 start_pcd / 877 camera loop;生产 go2_saas_agent.py:2156 patrol 冲突守卫】`。**现场在线建图**路径,产物落 `maps/console/` `【code server.py:37 PCD_DIR=WS/maps/console】`。
- **限定(修正)**:字面上 `maps/console` 确是 SaaS `resolve_route_map` 找同名 pcd 的目录 `【生产 go2_saas_agent.py:32 PCD_DIR 默认=WS/maps/console / 723-748 resolve_route_map 在 PCD_DIR 找 stem+.pcd】`,**但消费有条件**:
  - `resolve_route_map` **仅在 pcd 重定位模式**被 `route_relocalization_plan` 调用 `【生产 go2_saas_agent.py:772】`;**`manual_anchor` 模式在 765-770 提前 `return`、根本不查 pcd** `【生产 go2_saas_agent.py:765-770】`——而 **console 与两份 manifest 实跑都是 `manual_anchor`** `【狗上 dog:xunjian-20260725-06/07 manifest localization_mode=manual_anchor】`。
  - 故 console 建的 `maps/console` pcd **供 pcd 重定位模式用,并非 `manual_anchor` 巡检消费**。
  - 另 saas 侧 `PCD_DIR` 受环境变量 `GO2_PCD_DIR` 覆盖,**默认才** = `maps/console` `【生产 go2_saas_agent.py:32】`。
- 与 06 的离线 `go2_loop_backend`(从 rosbag 重建 + 回环优化)是**两条建图路**:现场快速在线 vs 离线高质量 `【code:go2_loop_backend 包存在;keyframe_saver.py:34 / sliding_window_static_filter.py:115 / dynamic_map_filter.py:126 → maps/loop_backend;含 scan_context_detector 回环 / pose_graph_optimizer 优化;server.py:37 maps/console】`(loop_backend 内部机制属 06 篇范围,此处仅证其存在与输出目录)。

## 五、Web API(10 个端点全对)
`/api/status`、`/api/routes`(列表+起点)、`/api/route_points`(2D 预览)、`/api/pcd_list`、`/api/pcd_points`(远端抽样避免传大文件)、`/api/pcd_pack`(base64 打包 xyz 给 3D viewer)、`/api/camera_files`、`/api/download`、`/api/file`(视频带 Range 流式)、`/api/action` `【code server.py:1034 status / 1056 routes(1082 起点)/ 1086 route_points(1106 [x,y,0,yaw])/ 1112 pcd_list / 1130 pcd_points(1137-1146 远端 awk 抽样)/ 1186 pcd_pack / 1223 camera_files / 1278 download / 1293 file(1303-1318 Range 206)/ 1323 action】`。
- 细节:`pcd_pack` 的 **x,y 按质心(重心)归零、z 按 min 归零** `【code server.py:1179 x-cx/y-cy 归心 + 1181 b64】`——文档"重心归零"对 **x,y 精确**。

> ### ⚠️ 前端展示层:仓库缺失,照仓库直接启动会 RuntimeError
> `tools/patrol_console/static/`(`index.html` / `*.js` / `vendor/`)**不在仓库**(`find tools/patrol_console` 仅见 `server.py` / `README.md` / `requirements.txt` / `test_*.py`),**未被 `.gitignore` 排除,就是没有**。而 `server.py` 在**模块顶层**即:
> - `app.mount("/vendor", StaticFiles(directory=STATIC_DIR/"vendor"))` `【code server.py:1020】`
> - `index()` 返回 `FileResponse(STATIC_DIR/"index.html")` `【code server.py:1031】`
>
> Starlette `StaticFiles` 默认 `check_dir=True`,**目录不存在会在导入/启动时抛 `RuntimeError`**。即**照仓库直接跑 `server.py` 会失败**;此展示层**无法从仓库核验**。判定 `【UNVERIFIABLE / 仓库缺失】`——前端应是**未随仓库提供**(部署时另置),文档不应把 `static/` 当成"已存在的展示层"。

## 六、要点
- 两条"驱动狗"的路:**云端 SaaS(自动、远程)** vs **操作台(人工、现场 SSH)**,底层脚本共用。
- **safe 节点参数实为三档**(原文只呈现"测试档 vs 生产"两档,漏了 node 默认):

  | 档 | stop_distance | roi_x_min~max | 源标签 |
  |---|---|---|---|
  | node `declare_parameter` 默认(被两处启动串覆盖的第三值) | **0.70** | 0.35~**1.20** | `【默认 unitree_safe_cmd_node.py:73 / 59-60】` |
  | console `act_start_safe` 独立按钮(测试档) | **0.40** | 0.35~**0.90** | `【code server.py:604-605】` |
  | 生产 saas `safe_cmd`(生产档) | **0.80** | 0.35~**1.50** | `【生产 go2_saas_agent.py:2057-2058】` |

  - **狗上实际生效哪个**:用 console 跑**巡检**时(`start_follower` → saas `patrol-start`,`execute_safe=True` 启动 `safe_cmd`)走的是**生产 0.80 / 0.35~1.50**,**不是测试档** `【生产 go2_saas_agent.py:2319 detached_command(safe_cmd) / 2650 patrol-start→execute_safe=True】`。测试档 0.40 / 0.35~0.90 **只在 operator 手点独立 `start_safe` 按钮时生效**。
  - → "同一节点不同参数,以启动串为准"方法论正确,但须点明:**console 跑巡检 ≠ 测试档**。
  - **狗上未直证**:两份 manifest **不含** safe 节点 stop_distance/roi_x,故"狗上巡检=0.80/1.50"属**代码路径推断**(patrol-start `execute_safe=True` 启动 safe_cmd),非 manifest 直证 `【推断-未验】`。
- **现场 pcd(maps/console)** 与 **离线 pcd(maps/loop_backend)** 两套,注意别混;且记住 `maps/console` pcd **仅供 pcd 重定位模式**,`manual_anchor` 巡检不消费(见 §四)。
- **生产装配确达狗上**(manifest 直证)`【狗上 dog:runs/xunjian-20260725-06/07 manifest.txt:speed=0.5, loop=pingpong, localization_mode=manual_anchor, go2_2_k_yaw=0.900, controller_reference_sha256=d205a596…(狗上 follower sha ≠ 仓库版)】`——但直证的是 **follower/巡检运行期真相**(k_yaw/speed/模式),**safe 参数不在其中**。

---

## 核验台账(claim → 证据 file:line → 判定)

| # | 断言 | 证据 file:line | 判定 | 源标签 |
|---|---|---|---|---|
| 1 | FastAPI 监听 127.0.0.1:8642 | server.py:34,1019,1350 | ✅ CONFIRMED | code |
| 2 | docstring 三原则(不改狗上文件/stdin 注入不落盘/仅 localhost) | server.py:5-7,250-251,1350 | ✅ CONFIRMED | code |
| 3 | SSH 主机 go2wired→go2→go2home,断了重探 | server.py:33,273-274,123-131 | ✅ 顺序+"网线最快"CONFIRMED;go2=热点/go2home 标签 | code + 推断-未验 |
| 4 | telemetry_worker:nice -n 10 / 三订阅 / 每秒 JSON | server.py:233,162-164,188 | ✅ CONFIRMED | code |
| 5 | 台上显示 电量/电机温/SOC/位姿/sport | server.py:196-228 | ✅ CONFIRMED | code |
| 6 | status_worker 每 3s 查关键进程 | server.py:459,284,293 | ✅ CONFIRMED,**修正:PROC_KEYS 9 项、文档漏 cmd** | code |
| 7 | 录制行数/tail 日志/CPU 温/WiFi/Livox ping/Z1 ping·RTSP·GCU/camera.env/4g-state | server.py:374-392 | ✅ CONFIRMED | code |
| 8 | follower/safe 日志摘要成中文状态 | server.py:319-348 | ✅ CONFIRMED | code |
| 9 | 动作白名单 ACTIONS(25 项) | server.py:987-1013,1323-1327 | ✅ CONFIRMED | code |
| 10 | 起停录制 → route_recording_blackbox | server.py:46,569,584 | ✅ CONFIRMED | code |
| 11 | 起停巡检 → saas patrol-start/stop --localization-mode manual_anchor | server.py:646-649,628 | ✅ CONFIRMED | code |
| 12 | 急停 pkill follower + safe 0.5s cmd 超时发 Move(0,0,0) | server.py:660-663;unitree_safe_cmd_node.py:54,467-469,491 | ✅ CONFIRMED(依赖 safe 在跑) | code + 默认 |
| 13 | 运动类动作先 armed 解锁 | server.py:1015,1328-1331 | ✅ CONFIRMED,**精确化:服务端 403 / dry_run 可绕过** | code |
| 14 | 模式互斥守卫(远端 ps 查冲突) | server.py:541-554,608,788,875 | ✅ CONFIRMED | code |
| 15 | start_pcd go2map_capture 全参数(3帧/抽样2/voxel0.08/20s/ASCII→maps/console/session_guard) | server.py:697,738,742,729,763,753-777,787,796-810 | ✅ CONFIRMED(数值逐一命中) | code |
| 16 | go2map_capture 遍布冲突守卫 | server.py:294,512,610,790,877;go2_saas_agent.py:2156 | ✅ CONFIRMED | code + 生产 |
| 17 | maps/console = resolve_route_map 找同名 pcd 目录 | server.py:37;go2_saas_agent.py:32,723-748 | ✅ 字面 CONFIRMED,**限定:仅 pcd 模式消费;manual_anchor 765-770 提前 return 不查;GO2_PCD_DIR 可覆盖** | code + 生产 |
| 18 | vs 06 go2_loop_backend 两条建图路(maps/console vs maps/loop_backend) | go2_loop_backend 包;keyframe_saver.py:34 等 → maps/loop_backend;server.py:37 | ✅ CONFIRMED | code |
| 19 | Web API 10 端点(pcd_pack 重心归零) | server.py:1034,1056,1086,1112,1130,1186,1223,1278,1293,1323,1179 | ✅ CONFIRMED(x,y 质心归零精确) | code |
| 20 | safe stop_distance:测试 0.40 / 生产 0.80 | 默认 unitree_safe_cmd_node.py:73=0.70;server.py:604=0.40;go2_saas_agent.py:2057=0.80,2319,2650 | ⚠️ **DEFAULT_VS_PROD:实三档,狗上巡检=0.80** | 混合 |
| 21 | safe roi_x:测试 0.35~0.90 / 生产 0.35~1.50 | 默认 unitree_safe_cmd_node.py:59-60(上限 1.20);server.py:605;go2_saas_agent.py:2058 | ⚠️ **DEFAULT_VS_PROD:实三档,狗上巡检=1.50** | 混合 |
| 22 | 前端 static/(html/js 展示层) | find 仅 server.py/README/requirements/test_*;server.py:1020,1031 引用但盘上缺失 | ❌ **UNVERIFIABLE:仓库缺失→启动 RuntimeError** | code |
| 23 | 生产装配参数确达狗上(manifest 佐证) | runs/06·07 manifest:speed=0.5, loop=pingpong, manual_anchor, k_yaw=0.900, sha d205a596… | ✅ CONFIRMED(直证 follower/巡检期;不含 safe) | dog |

### 狗上一致性(逐文件)
- `tools/patrol_console/server.py` —— **【无狗上对照】**:本机运行、非狗上,不在狗上 4 份远端副本之列;是否与狗上任何东西一致【无法验证】,仅能对仓库源码核。
- `orin_go2_fastlio_ws/scripts/go2_saas_agent.py` —— **【无狗上对照】**:狗上版是否与仓库一致【无法验证】;manifest 间接佐证其装配的部分生产值(speed=0.5 / loop=pingpong / manual_anchor / k_yaw=0.900)达狗端运行期,**safe 参数不在 manifest 中**。
- `.../go2_fastlio_patrol/unitree_safe_cmd_node.py` —— **【无狗上对照】**:仅据仓库读 `declare_parameter` 默认值(stop_distance=0.70 / roi_x_max=1.20 / cmd_timeout=0.5)。
- `orin_go2_fastlio_ws/src/go2_loop_backend/` —— **【无狗上对照】**:仅证仓库内该包存在、输出目录为 maps/loop_backend。
- `.../go2_fastlio_patrol/setup.py` —— **【无狗上对照】**:仅确认 entry_point `unitree_safe_cmd_node → unitree_safe_cmd_node:main`(setup.py:27)。

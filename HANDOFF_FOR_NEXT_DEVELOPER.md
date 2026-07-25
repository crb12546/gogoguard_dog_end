# Go2 巡检系统研发交接说明

本文面向接手继续开发的研发人员，说明当前 Go2 + Orin + Livox + Z-1Pro + GoGoGuard 巡检系统的代码结构、运行方式、接口链路、已验证能力、风险点和后续开发建议。

更新时间: 2026-07-24 20:45（Asia/Shanghai）

---

## 0. 2026-07-24 当前权威状态（新对话先读）

> 本节是当前状态。下文第 1～15 节主要是 2026-07-07 的历史背景；如有冲突，以本节为准。

### 0.1 用户当前目标与代码基线

用户要求狗端除 CSV、录制视频等数据文件外，代码、参数和配置严格恢复到
2026-07-24 08:00 前的状态。该严格回退已完成。

回退前的完整本地备份：

```text
/Users/constantine/Project/Go2/dog_xunjian_backup/20260724_codex_08
```

当前本地工作副本：

```text
/Users/constantine/Project/Go2/orin_go2_fastlio_ws
```

当前狗端工作区：

```text
/home/unitree/go2_fastlio_ws
```

重要约束：

```text
1. CSV 路线和录制的视频未随严格回退删除或覆盖。
2. route_recorder.py 本来就是 08:00 时的版本，未额外改动。
3. 2026-07-24 晚间排查只做了读取、探测和启动 Mac 本地管理台；
   尚未实施下面所述的 Livox 队列修复，也没有再次修改巡线控制器。
4. 下一位接手者不要把“建议方案”误认为“已部署方案”。
```

### 0.2 当前运行与安全状态

Mac 本地管理台入口：

```text
http://127.0.0.1:8642
```

启动入口：

```bash
cd /Users/constantine/Project/Go2/tools/patrol_console
nohup python3 -u server.py >/tmp/patrol_console.log 2>&1 </dev/null &
```

最近一次实测时狗端：

```text
Livox:             运行
FAST-LIO:          运行，但输出位置不满足实时性要求
unitree_safe_cmd:  未运行
waypoint_follower: 未运行
SaaS command-loop: 运行，带 --execute-safe
nvpmodel:          25W，mode 3，8 核在线
CPU 单核上限:      1.497 GHz
```

`command-loop --execute-safe` 表示 SaaS 再下发 `start_patrol` 时狗端会真实尝试启动。
当前虽然会被预检拦住，但调试时仍必须按真实运动权限对待；不要无人值守下发。

### 0.3 2026-07-24 SaaS 启动失败记录

日志：

```text
/home/unitree/go2_fastlio_ws/patrol_logs/service/go2-saas-command.log
```

运行目录：

```text
/home/unitree/go2_fastlio_ws/patrol_logs/runs/20260724/
```

三次关键命令：

```text
COMMAND 241 start_patrol:
  下载/选择 xbf2.csv，speed=0.5，loopMode=pingpong
  FAST-LIO 缺失后自动启动底座
  等待 120 秒后仍为 age_ms=1764.28，超过 350 ms
  结果: FASTLIO_STALE，失败

COMMAND 242 start_patrol:
  FAST-LIO 后来恢复到约 114～131 ms
  随后起点检查失败：
    current=(0.004,-0.012,-0.000)
    route_start=(3.389,-9.017,-0.168)
    distance=9.621 m，允许 0.350 m
  结果: ROUTE_START_NOT_ALIGNED，失败

COMMAND 243 start_patrol:
  创建 xunjian-20260724-15
  FAST-LIO 缺失后再次自动启动底座
  等待 120 秒后 age_ms=1840.06，超过 350 ms
  结果: FASTLIO_STALE，失败并成功回传 SaaS
```

三次失败都没有启动 `safe` 或 `waypoint_follower`，没有下发巡线运动。

### 0.4 温度与整机算力结论

已排除温度导致的热降频：

```text
CPU: 约 60°C
GPU: 约 57°C
8 个 CPU 核心在线
无 thermal/throttle/overheat 内核记录
内存和整机总负载均未饱和
```

25W 八核模式的局限是每核最高只有 `1.497 GHz`。Livox 驱动中有一条线程长期接近
`98%～109%` 单核占用；增加核心数不会自动把这条有序流水线拆成多核。

MAXN mode 0 可把单核上限提高到 `1.984 GHz`，约增加 32.5%，但这只能作为验证手段，
不能替代队列实时性修复。

### 0.5 已定位的实时性证据

Livox 接收侧报告正常：

```text
point frame: 约 10 Hz
IMU:         约 200 Hz
每帧点数:   19872
原始包队列: 通常为 0
网卡:       无明显丢包
```

FAST-LIO 侧却持续看到：

```text
FAST_LIO_INPUT_TIMING: 约 1.72～1.81 秒
FAST_LIO_OUTPUT_TIMING: 约 1.75～1.84 秒
lidar_buffer: 0
imu_buffer: 约 350～370 帧
```

因此延迟在进入 FAST-LIO 时已经存在；FAST-LIO 计算本身通常只再增加几十毫秒。
这不是 CSV 跟踪器、转弯控制器或机器狗执行折扣造成的。

### 0.6 高可信根因：Livox 上游仍保留旧帧

FAST-LIO 订阅端虽然已经使用较小队列和 `BEST_EFFORT`，但 Livox 驱动上游仍有两层
不适合实时控制的缓存。

第一层，驱动内部完整点云队列：

```text
CalculatePacketQueueSize(10 Hz) 返回 10
InitQueue 会向上取整到 2 的幂，实际为 16 帧
16 帧 / 10 Hz ≈ 1.6 秒
```

对应代码：

```text
orin_go2_fastlio_ws/src/livox_ros_driver2/src/comm/comm.cpp
orin_go2_fastlio_ws/src/livox_ros_driver2/src/comm/ldq.cpp
orin_go2_fastlio_ws/src/livox_ros_driver2/src/lds.cpp
```

当前队列满时，`Lds::PushLidarData()` 不写入新帧，也就是保留旧帧、丢弃新帧。
观察到的 `1.75～1.84 秒` 与 16 帧内部队列高度吻合。

第二层，ROS2 点云发布端：

```text
/livox/lidar publisher:
  Reliability = RELIABLE
  global publisher history depth = 256
```

对应代码：

```text
orin_go2_fastlio_ws/src/livox_ros_driver2/src/lddc.cpp
```

所以只修改 FAST-LIO 订阅深度，无法消除 Livox 驱动内部和发布端的旧数据。

这是目前的高可信根因，仍需通过修改后实测闭环确认；截至交接时尚未修改或部署。

### 0.7 下一步建议（先修实时性，不改巡线算法）

建议只改 Livox 点云链路，暂不碰 `waypoint_follower` 的路线、转弯或纠偏逻辑：

```text
1. 修改前新增独立备份，命名 YYYYMMDD_codex_NN。
2. 完整点云内部队列缩到 2 帧，或实现“积压时丢最旧、保留最新完整帧”。
3. /livox/lidar 发布改为 BEST_EFFORT + KEEP_LAST(1 或 2)。
4. FAST-LIO 点云订阅继续保持 BEST_EFFORT + depth=2。
5. IMU 必须保持有序和足够队列，不能照点云方式只留最后一帧。
6. Release 模式重新编译 livox_ros_driver2。
7. 先只启动 Livox + FAST-LIO，禁止巡检，持续验证至少 5 分钟。
8. 通过标准：
   - FAST_LIO_INPUT_TIMING 和 OUTPUT_TIMING 稳定低于 350 ms；
   - 不再出现约 350 帧 IMU 长期积压；
   - /Odometry 连续、无跳变；
   - 重启底座后也能重复通过。
9. 只有上述通过后，才允许 SaaS 做低风险 start_patrol 现场测试。
```

不要采用以下“假修复”：

```text
不要把 350 ms 门槛直接放宽到 1～2 秒；
不要把旧点云时间戳伪装成当前时间；
不要先重写巡线纠偏算法；
不要并行无序发布点云帧。
```

如果队列修复后实时帧率仍不足，再做第二阶段多核化：

```text
原始包解码拆成 2～4 个工作线程；
每个包保留序号和传感器时间；
最后由单线程按序组帧和发布；
Livox、FAST-LIO、SaaS/视频进程分配不同核心。
```

### 0.8 另一个独立问题：FAST-LIO 重启与 CSV 坐标系

`xbf2.csv` 保存的是旧 FAST-LIO 会话中的绝对坐标，起点约为：

```text
(3.389, -9.017, -0.168)
```

FAST-LIO 重启后当前坐标回到接近 `(0,0,0)`，即使狗物理上位于原路线起点，
绝对数值也会相差约 9.6 m，从而触发严格的 0.35 m 起点检查。

2026-07-24 已选择并实现 B 的“人工起点锚定”版本：

```text
1. 不覆盖原始 CSV，不重置 FAST-LIO；
2. 狗完成站立且里程计静止后，把当前位姿视为 CSV 第 0 点；
3. 整条路线刚性变换为本次 route_runtime.csv；
4. manual_anchor.json 记录原始起点、当前锚点和 FAST-LIO PID/进程启动时间；
5. 巡检、CSV 录制或 PCD 采集中定位会话变化时立即终止，禁止跨会话续跑。
6. 底座启动/停止入口在上述任务运行时拒绝重启；
7. 进程就绪检查排除 bash/sh，避免把完整启动命令文本误判成真实子进程；
8. `ensure_base_ready.sh` 按 `fastlio_mapping` 的可执行进程名精确计数。
```

该方案依赖人工把狗准确放回真实起点和朝向；没有 PCD 重定位时无法识别摆错位置。
狗端 SaaS agent 与辅助脚本已在 2026-07-24 23:25 部署并重载服务；已完成无运动
锚定、会话守护、完整底座自检和 SaaS 干运行验证，尚未做真实行走验收。

### 0.9 新对话建议开场

可以在新对话中直接说：

```text
请先完整阅读 /Users/constantine/Project/Go2/HANDOFF_FOR_NEXT_DEVELOPER.md
第 0 节。狗端已部署 2026-07-24 人工起点锚定与 FAST-LIO 会话守护版。
先确认当前没有活动任务、SaaS 服务在线且定位稳定；下一步只在人员持急停、
狗位于真实路线起点且朝向一致时，做一条短路线的低速真实验收。不要恢复旧的
跨会话绝对 CSV 起点比较，也不要默认启用 PCD 重定位。Livox 旧数据队列问题
仍是独立后续项。
```

---

## 1. 一句话概览

这套系统把宇树 Go2 作为现场巡检终端：

```text
Go2 / Orin 侧
  Livox MID-360S + FAST-LIO 产出定位和点云
  route_recorder 录制 CSV 路线
  waypoint_follower 读取 CSV 并生成 /patrol_cmd
  unitree_safe_cmd_node 做前方点云安全门控并发 /api/sport/request
  Z-1Pro 录制 20s 视频段
  go2_saas_agent.py 对接 GoGoGuard 心跳 / 视频上传 / 命令轮询

Mac 侧
  tools/patrol_console 本地网页管理台
  通过 SSH 调用 Orin 上的 ROS2 节点和脚本

GoGoGuard 云端
  接收 heartbeat / video
  通过 heartbeat response.commands 下发 start_patrol / stop_patrol
  做视频识别、点位匹配、大屏、报告
```

当前最重要结论：

```text
GoGoGuard v1.5 下发 start_patrol -> 狗端接收 -> 下载/读取 CSV -> 启动 safe + follower -> 狗真实移动，这条链路已经打通。
```

但路线推进和 FAST-LIO 稳定性仍需继续调试。

---

## 2. 工作区结构

根目录：`/Users/constantine/Project/Go2`

关键路径：

```text
orin_go2_fastlio_ws/                         # Orin 上 /home/unitree/go2_fastlio_ws 的本地同步副本
  scripts/
    env_common.sh                            # 远端 ROS/Unitree 环境加载
    base_bringup.sh                          # 启动 Livox + FAST-LIO，不发运动命令
    base_stop.sh                             # 停底座
    go2_saas_agent.py                        # GoGoGuard 对接核心 agent
    go2_network_recover.sh                   # Orin 网络/4G 恢复脚本
    z1pro_capture.sh                         # Z-1Pro RTSP 拍照/录像
    z1pro_gcu_control.py                     # Z-1Pro GCU 云台控制
    z1pro_preset.sh                          # 云台预设
  src/go2_fastlio_patrol/go2_fastlio_patrol/
    route_recorder.py                        # 记录 /Odometry 为 CSV 路线
    waypoint_follower.py                     # CSV 路线跟随，发布 /patrol_cmd
    unitree_safe_cmd_node.py                 # 点云安全门控，发布 /api/sport/request
  maps/console/*.pcd                         # 管理台保存的点云
  patrol_logs/videos/*.mp4                   # Z-1Pro 20s 视频段

tools/patrol_console/
  server.py                                  # Mac 本地 FastAPI 管理台后端
  static/index.html                          # 管理台前端
  test_go2_saas_agent.py                     # GoGoGuard fake server 单元测试

FIELD_GUIDE.md                               # 现场操作手册
MOTION_CONTROL_CHAIN_ANALYSIS.md             # 运动链路分析
HANDOFF_FOR_NEXT_DEVELOPER.md                # 本文件
```

---

## 3. 网络与设备地址

常用 SSH alias：

```text
ssh go2wired    # Orin 有线管理口 192.168.123.18，Mac 静态 192.168.123.222
ssh go2         # iPhone 热点下 Orin，常见 172.20.10.2
ssh go2home     # 家里 WiFi，历史 IP 192.168.0.122
```

设备地址：

```text
Orin eth0:
  192.168.123.18/24     # Go2 本体/有线管理网
  192.168.1.5/24        # Livox 网
  192.168.144.100/24    # Z-1Pro 网

Go2 本体:       192.168.123.161
Livox MID-360S: 192.168.1.161
Z-1Pro:         192.168.144.108
GoGoGuard IP:   39.96.37.187
```

Z-1Pro：

```text
RTSP: rtsp://192.168.144.108/
GCU:  TCP 192.168.144.108:2332
```

---

## 4. Mac 本地管理台

启动：

```bash
cd /Users/constantine/Project/Go2/tools/patrol_console
nohup python3 server.py > /tmp/patrol_console.log 2>&1 < /dev/null &
```

浏览器：

```text
http://127.0.0.1:8642
```

停止：

```bash
pkill -f "patrol_console/server.py"
```

管理台只监听 `127.0.0.1`，通过 SSH 调用 Orin，不直接暴露公网。

关键 API：

```text
GET  /api/status                         # 遥测、进程、Orin 状态、动作日志
GET  /api/routes                         # CSV 路线列表
GET  /api/route_points?path=...          # 路线点
GET  /api/pcd_list                       # PCD 列表
GET  /api/pcd_pack?path=...              # 点云三维渲染包
GET  /api/camera_files                   # Z-1Pro 视频/图片
POST /api/action                         # 白名单动作入口
```

白名单动作包括：

```text
start_base / stop_base
start_recorder / stop_recorder
start_pcd / stop_pcd
start_safe / stop_safe
start_follower / stop_follower / estop / stop_all_control
camera_probe / camera_preset / camera_snapshot / camera_record / camera_start_loop / camera_stop_loop
saas_heartbeat / saas_manifest / saas_command_result / saas_start_loop / saas_stop_loop
```

---

## 5. ROS/运动控制链路

当前主运动链路：

```text
CSV route
  -> waypoint_follower
  -> /patrol_cmd (geometry_msgs/Twist)
  -> unitree_safe_cmd_node
  -> /api/sport/request (unitree_api/msg/Request, Move api_id=1008)
  -> Go2 sport service
```

### 5.1 `route_recorder.py`

职责：订阅 `/Odometry`，按最小距离间隔写 CSV 路线。

输出位置：

```text
/home/unitree/go2_fastlio_ws/src/go2_fastlio_patrol/routes/*.csv
```

CSV 字段包含：

```text
id,x,y,z,yaw,v
```

### 5.2 `waypoint_follower.py`

职责：读取 CSV，订阅 `/Odometry`，发布 `/patrol_cmd`。

关键参数：

```text
route_file
v_base / max_vx
k_yaw / max_yaw_rate
lookahead_distance
reach_distance
goal_distance
loop_mode: once / pingpong
search_window
turn_in_place_angle
slow_down_angle
stuck_time
relocalize_distance
```

当前 GoGoGuard v1.5 启动时使用的参数：

```text
v_base=0.5
max_vx=0.5
k_yaw=0.9
max_yaw_rate=0.45
lookahead_distance=0.6
reach_distance=0.4
goal_distance=0.25
loop_mode=pingpong
search_window=6
turn_in_place_angle=1.0
slow_down_angle=0.5
stuck_time=3.0
relocalize_distance=1.5
```

已知问题：长路线执行时可能卡在某个 `nearest_index`，日志出现：

```text
stuck recovery: nearest_index 2 -> 2
```

这表示 follower 认为没有推进到下一点。可能原因：实际运动、FAST-LIO 里程计、路线局部点距/起点姿态/跟随参数不匹配。

### 5.3 `unitree_safe_cmd_node.py`

职责：

```text
订阅 /patrol_cmd
订阅 /cloud_registered_body
判断前方 ROI 是否有障碍
正常时转发 Move(x=vx, z=yaw_rate)
超时/障碍时发 Move(0,0,0)
```

当前 v1.5 参数：

```text
max_vx=0.5
max_yaw_rate=0.45
publish_rate=20.0
pointcloud_topic=/cloud_registered_body
sport_request_topic=/api/sport/request
stop_distance=0.40
resume_distance=0.50
min_stop_points=15
roi_x_min=0.35
roi_x_max=0.90
roi_y_min=-0.30
roi_y_max=0.30
roi_z_min=0.30
roi_z_max=0.90
```

注意：这是前方窄 ROI 的速度门控，不是完整全身避障。

---

## 6. GoGoGuard 对接

核心脚本：

```text
/home/unitree/go2_fastlio_ws/scripts/go2_saas_agent.py
本地副本：orin_go2_fastlio_ws/scripts/go2_saas_agent.py
```

私有配置：

```text
~/.config/go2_saas.env
```

该文件包含 token，不能提交仓库。典型字段：

```bash
export GO2_ROBOT_ID='bangguard_zh'
export GO2_AUTH_TOKEN='...'
export GO2_AUTH_HEADER='Authorization'
export GO2_BACKEND_BASE='https://39.96.37.187/api/v1'
export GO2_DEVICE_TOKEN='...可选...'
```

### 6.1 已实现接口

上行：

```text
POST /api/v1/robot/heartbeat
POST /api/v1/robot/video/upload
POST /api/v1/robot/command/result
```

下行：

```text
平台命令放在 heartbeat response.commands 中
```

计划接口：

```text
GET /api/v1/devices/plan
Header: X-Device-Token: <GO2_DEVICE_TOKEN>
```

### 6.2 heartbeat payload

`go2_saas_agent.py` 当前 heartbeat 同时兼容旧字段和 v1.5 字段：

```json
{
  "robotId": "bangguard_zh",
  "time": "YYYY-MM-DD HH:MM:SS",
  "timestamp": 1783340000,
  "status": "patrolling|ready|idle|scan|video_recording",
  "pose": {"x": 0, "y": 0, "z": 0, "yaw": 0},
  "position": {"x": 0, "y": 0, "z": 0, "yaw": 0},
  "motion": {
    "position": {"x": 0, "y": 0, "z": 0, "yaw": 0},
    "yaw_rad": 0,
    "velocity": {"vx": 0, "vyaw": 0}
  },
  "patrol": {"running": true, "route_file": "shang8_3.csv"},
  "battery": {"soc": 35},
  "diagnostics": {"processes": {}, "network": {}},
  "telemetry": {},
  "assets": {}
}
```

### 6.3 视频上传

视频来自：

```text
/home/unitree/go2_fastlio_ws/patrol_logs/videos/z1pro_*.mp4
```

上传 endpoint：

```text
POST /api/v1/robot/video/upload
multipart/form-data
```

表单字段包括：

```text
file
robotId
fileName
fileSize
kind / assetType
time
meta
x/y/z/yaw
pose
position
patrolId 可选
```

平台已验证返回：

```json
{"ok": true, "hadPose": true, "framesScanned": 8}
```

### 6.4 v1.5 命令

支持：

```text
ping / noop / status
start_patrol
stop_patrol
goto/go/navigate: rejected，暂不实现任意点导航
```

`start_patrol` 推荐格式：

```json
{
  "id": 12,
  "type": "start_patrol",
  "action": "start_patrol",
  "params": {
    "fileName": "shang8_3.csv",
    "routeUrl": "https://gogoguard.cn/api/v1/robot/route/s404035be",
    "speed": 0.5,
    "loopMode": "pingpong"
  }
}
```

处理逻辑：

```text
1. 解析 fileName / route / routeUrl
2. 若有 routeUrl，下载 CSV 到 routes/<fileName>
3. 最小检查：文件非空、行数 >= 3
4. 若 execute_safe=false，只 dry-run 回执
5. 若 execute_safe=true，启动 safe node + waypoint_follower
6. POST /robot/command/result 回执
```

特别修复：

```text
如果 routeUrl 是 https://gogoguard.cn/...
狗端会自动改写到 GO2_BACKEND_BASE 的 host，例如：
https://39.96.37.187/...
```

原因：4G 到 `gogoguard.cn` 域名链路出现过 reset；IP 版本稳定。

### 6.5 command-loop 与 patrol-loop 分工

必须注意：

```text
command-loop 负责 heartbeat + 拉 commands + 执行/回执命令
patrol-loop 只负责视频上传，不应抢 commands
```

正确运行方式：

```bash
# 视频上传循环，不发 heartbeat，不抢命令
python3 -u /home/unitree/go2_fastlio_ws/scripts/go2_saas_agent.py \
  patrol-loop --no-heartbeat --video latest --upload --interval 20 \
  --run-file /tmp/go2_saas_loop.run --ros-timeout 1.5

# 命令执行循环，真实执行模式
python3 -u /home/unitree/go2_fastlio_ws/scripts/go2_saas_agent.py \
  command-loop --interval 5 --run-file /tmp/go2_saas_command.run \
  --ros-timeout 1.5 --execute-safe
```

如果不希望平台命令真动狗，不要加 `--execute-safe`。

---

## 7. 当前已验证的 GoGoGuard v1.5 结果

2026-07-06 已完成真实联调：

1. GoGoGuard heartbeat/video 上报成功。
2. 平台下发空 route 时，狗端正确拒绝。
3. 平台下发 `shang8_3.csv` + `routeUrl` 后，狗端成功下载/读取。
4. `start_patrol` 成功启动：

```text
COMMAND 12 start_patrol success
SAFE_STARTED
FOLLOWER_STARTED
PATROL_STARTED route=shang8_3.csv speed=0.500 loop=pingpong
```

5. 狗端实际产生运动，safe node 日志有：

```text
Move x=0.500, z=0.05~0.07
```

6. `waypoint_follower` 成功加载路线：

```text
route_file: .../routes/shang8_3.csv
route points: 755
loop_mode: pingpong
init nearest_index=0, distance_to_start=0.057
```

7. 用户手动停止后，远端也执行过补充 StopMove，当前 safe/follower/cmd 已清理。

当前限制：

```text
执行时 follower 只推进到 nearest=2 附近，然后反复 stuck recovery。
这属于本地巡线控制/定位问题，不是 GoGoGuard 命令链路问题。
```

---

## 8. 关键资产

### 8.1 可用路线/点云

`shang8_3` 是当前最新可用大路线：

```text
CSV: /home/unitree/go2_fastlio_ws/src/go2_fastlio_patrol/routes/shang8_3.csv
PCD: /home/unitree/go2_fastlio_ws/maps/console/shang8_3.pcd
```

路线统计：

```text
points=755
length≈334.5m
step_median≈0.444m
max_step≈0.523m
bbox x=-14.0..115.1, y=-60.6..1.0
```

PCD：

```text
points=1,146,536
bbox x=-82.7..151.7, y=-88.8..79.8, z=-45.7..94.5
```

### 8.2 不建议使用的路线

```text
shang8.csv: BAD，FAST-LIO 漂移严重，长度约 13km，bbox 到 x=5533/y=10306
```

### 8.3 其它历史样本

```text
xiaoqu1.csv / xiaoqu1.pcd: 现场长路线样本，路线约 142m，PCD 约 49 万点
roomtest7.csv / roomtest7.pcd: 短路线，验证 safe chain 可动
shang8_2.csv: 正常但较短，约 61.8m
```

---

## 9. 4G 网络现状与修复

4G 卡：A7600C，USB ID：

```text
1e0e:9011
```

已验证正确流程：

```text
1. 不插 4G，先启动 Orin
2. SSH go2wired 通
3. 再热插 4G
4. 等 /Modem/N 出现
5. 手动只执行一次 sudo nmcli con up go2-4g
```

`go2-4g` 当前应保持：

```text
connection.autoconnect: no
connection.interface-name: --
ipv4.never-default: no
ipv4.route-metric: 50
ipv6.method: ignore
```

4G 成功状态：

```text
ttyUSB2:gsm:connected:go2-4g
ppp0: 10.x.x.x/32
default dev ppp0
ping 39.96.37.187: OK
```

### 9.1 已修复的干扰源

之前 `go2-network-recover.timer` 会周期运行：

```text
/home/unitree/go2_fastlio_ws/scripts/go2_network_recover.sh
```

旧脚本问题：

```text
会把 go2-4g 改回 autoconnect yes / never-default yes
会重复执行 nmcli con up go2-4g
会在已连接/连接中时打断 pppd
只按 eth1/192.168.0.1 NAT 模式写默认路由，不适配 ppp0
```

已修复：

```text
不再重写成 autoconnect yes
不再把 never-default 改回 yes
若 gsm 已 connecting/connected/deactivating，不再重复 nmcli con up
支持 ppp0 默认路由 default dev ppp0
```

当前建议保持：

```text
go2-network-recover.timer 停用
```

不要在现场反复 `nmcli con up`，会把 A7600C 的 AT 口打到 timeout/invalid。

---

## 10. 当前运行状态与安全结论

最近一次测试结束时：

```text
safe: false
follower: false
cmd: false
command-loop --execute-safe: 已停止/已解除平台运动权限
patrol-loop --no-heartbeat --video latest --upload: 仍可保留用于视频上传
base/Livox/FAST-LIO: 曾运行，但 FAST-LIO 后续发散
camera_loop: 运行过
```

非常重要：后续不要直接继续跑平台运动，原因：

```text
FAST-LIO 已严重发散，Odometry 到 1e5 米量级，z 到 -4e5 米量级
电量降到约 19%~20%
```

继续测试前必须：

```text
1. 充电到至少 50%，最好 >60%
2. 停 base / FAST-LIO
3. 狗静止摆正到路线起点
4. 重新启动 base_bringup.sh
5. 等 /Odometry 回到正常小范围且稳定
6. 确认起点对齐
7. 再打开 command-loop --execute-safe
```

---

## 11. 常用命令

### 11.1 查看状态

```bash
ssh go2wired 'date; ip -br addr; ip route; nmcli -t -f DEVICE,TYPE,STATE,CONNECTION device status'

ssh go2wired 'ps -eo pid=,comm=,args= | awk '\''$0 ~ /[r]oute_recorder|[g]o2map_capture|[w]aypoint_follower|[u]nitree_safe_cmd_node|[f]astlio_mapping|[l]ivox_ros_driver2_node|[g]o2_saas_agent\.py|[z]1pro_video_loop\.sh/ {print}'\'''
```

### 11.2 启动 Mac 管理台

```bash
cd tools/patrol_console
nohup python3 server.py > /tmp/patrol_console.log 2>&1 < /dev/null &
```

### 11.3 启动底座

```bash
ssh go2wired 'setsid nohup bash /home/unitree/go2_fastlio_ws/scripts/base_bringup.sh </dev/null >/tmp/console_base.log 2>&1 & echo BASE_STARTED'
```

验证：

```bash
ssh go2wired 'bash -lc '\''source /home/unitree/go2_fastlio_ws/scripts/env_common.sh >/dev/null 2>&1 || true; timeout 8 ros2 topic list | grep -E "^/(livox/lidar|livox/imu|Odometry|cloud_registered|cloud_registered_body)$"'\'''
```

### 11.4 启动 Z-1Pro 20 秒分段

```bash
ssh go2wired 'cat > /tmp/z1pro_video_loop.sh <<'\''SH'\''
#!/usr/bin/env bash
touch /tmp/z1pro_video_loop.run
while [ -f /tmp/z1pro_video_loop.run ]; do
  /home/unitree/go2_fastlio_ws/scripts/z1pro_capture.sh record 20 || true
  sleep 1
done
SH
chmod +x /tmp/z1pro_video_loop.sh
setsid nohup bash /tmp/z1pro_video_loop.sh </dev/null >/tmp/console_z1pro_loop.log 2>&1 &'
```

停止：

```bash
ssh go2wired 'rm -f /tmp/z1pro_video_loop.run; pkill -TERM -f "[z]1pro_video_loop.sh"; pkill -TERM -f "[g]st-launch-1.0.*192.168.144.108" || true'
```

### 11.5 启动 GoGoGuard 视频上传循环

推荐不要让它发 heartbeat，避免抢命令：

```bash
ssh go2wired 'bash -lc '\''source /home/unitree/go2_fastlio_ws/scripts/env_common.sh >/dev/null 2>&1 || true; source ~/.config/go2_saas.env; touch /tmp/go2_saas_loop.run; setsid nohup python3 -u /home/unitree/go2_fastlio_ws/scripts/go2_saas_agent.py patrol-loop --no-heartbeat --video latest --upload --interval 20 --run-file /tmp/go2_saas_loop.run --ros-timeout 1.5 >/tmp/go2_saas_loop.log 2>&1 &'\'''
```

停止：

```bash
ssh go2wired 'rm -f /tmp/go2_saas_loop.run; pkill -TERM -f "[g]o2_saas_agent.py patrol-loop"'
```

### 11.6 启动命令循环

干跑，不会动：

```bash
ssh go2wired 'bash -lc '\''source /home/unitree/go2_fastlio_ws/scripts/env_common.sh >/dev/null 2>&1 || true; source ~/.config/go2_saas.env; touch /tmp/go2_saas_command.run; setsid nohup python3 -u /home/unitree/go2_fastlio_ws/scripts/go2_saas_agent.py command-loop --interval 5 --run-file /tmp/go2_saas_command.run --ros-timeout 1.5 >/tmp/go2_saas_command.log 2>&1 &'\'''
```

真实执行，会动狗：

```bash
ssh go2wired 'bash -lc '\''source /home/unitree/go2_fastlio_ws/scripts/env_common.sh >/dev/null 2>&1 || true; source ~/.config/go2_saas.env; touch /tmp/go2_saas_command.run; setsid nohup python3 -u /home/unitree/go2_fastlio_ws/scripts/go2_saas_agent.py command-loop --interval 5 --run-file /tmp/go2_saas_command.run --ros-timeout 1.5 --execute-safe >/tmp/go2_saas_command.log 2>&1 &'\'''
```

停止命令循环：

```bash
ssh go2wired 'rm -f /tmp/go2_saas_command.run; pkill -TERM -f "[g]o2_saas_agent.py command-loop"'
```

### 11.7 急停 / 停控制节点

```bash
ssh go2wired 'p=$(ps -eo pid=,comm=,args= | awk '\''($2 == "ros2" && $0 ~ /[w]aypoint_follower|[u]nitree_safe_cmd_node|[u]nitree_cmd_node/) || $2 ~ /^waypoint_follow/ || $2 ~ /^unitree_safe_cm/ || $2 ~ /^unitree_cmd/ {print $1}'\'' | tr "\n" " " || true); [ -n "$p" ] && kill -TERM $p 2>/dev/null || true; sleep 1; p2=$(ps -eo pid=,comm=,args= | awk '\''($2 == "ros2" && $0 ~ /[w]aypoint_follower|[u]nitree_safe_cmd_node|[u]nitree_cmd_node/) || $2 ~ /^waypoint_follow/ || $2 ~ /^unitree_safe_cm/ || $2 ~ /^unitree_cmd/ {print $1}'\'' | tr "\n" " " || true); [ -n "$p2" ] && kill -KILL $p2 2>/dev/null || true; source /home/unitree/go2_fastlio_ws/scripts/env_common.sh >/dev/null 2>&1 || true; ros2 topic pub -1 /api/sport/request unitree_api/msg/Request "{header: {identity: {id: 9999, api_id: 1003}, lease: {id: 0}, policy: {priority: 0, noreply: false}}, parameter: '\''{}'\'', binary: []}" >/dev/null 2>&1 || true; echo CONTROL_STOPPED'
```

---

## 12. 测试

本地 fake GoGoGuard 测试：

```bash
python3 tools/patrol_console/test_go2_saas_agent.py
```

当前覆盖：

```text
heartbeat v1/v1.5 字段
视频 multipart 带 pose/position
commands 轮询与回执
重复 commandId 去重
plan-fetch 的 X-Device-Token
start_patrol 下载 CSV 并生成 safe/follower 启动命令
goto 默认 rejected
gogoguard.cn routeUrl 自动改写到 GO2_BACKEND_BASE host
```

最近结果：

```text
8 tests OK
```

---

## 13. 后续开发建议

优先级从高到低：

### P0：安全和稳定

1. 增加 `go2_saas_agent.py` 启动前安全门：
   - 电量低于阈值拒绝 `start_patrol`
   - odom 坐标绝对值异常时拒绝
   - FAST-LIO 未稳定时拒绝
   - 起点距离过大时拒绝
2. 平台运动权限和命令轮询分离：
   - 默认 command-loop 不执行运动
   - 现场显式 arm 后才允许 `--execute-safe`
3. 4G 稳定性：
   - 继续保持 `go2-network-recover.timer` 停用或改成只修 eth0，不碰 4G
   - A7600C 建议加独立供电/延迟上电 USB hub，或换更稳定 modem

### P1：巡线能力

1. 调 `waypoint_follower`：
   - 对长路线 `shang8_3.csv` 卡在 `nearest=2` 的问题做专项复现
   - 考虑增大 `lookahead_distance` 到 0.8/1.0
   - 降低 `speed` 到 0.2~0.3
   - 改进 progress 判断，不能只靠 nearest index 变化
2. 增加路线起点对齐检查到 `go2_saas_agent.py` 的 `start_patrol`。
3. 增加平台命令参数映射：允许平台指定速度上限、loopMode，但要限幅。

### P2：GoGoGuard 协议

1. 让平台只由 command-loop 拉 `commands`，不要让普通视频上传 heartbeat 返回命令。
2. 平台可以增加命令状态：`pending/running/success/failed/rejected`。
3. 平台下发 routeUrl 最好直接给 IP 版或签名下载 URL；狗端当前已做域名改写兜底。
4. `stop_patrol` 应该能撤销/清空平台未执行的 `start_patrol` 队列，避免 stop/start 同时返回时顺序不明确。

### P3：产品化

1. systemd service 管理 `go2_saas_agent`。
2. 管理台增加 GoGoGuard v1.5 专用页面：
   - 当前 command-loop 模式：off/dry-run/execute-safe
   - 最近平台命令
   - 最近回执
   - 一键 arm/disarm
3. GoGoGuard route/PCD 上传接口确认后，去掉临时 `/robot/asset/upload` 假设。

---

## 14. 安全红线

1. 自动巡线必须有人在狗旁边，遥控器在手。
2. `command-loop --execute-safe` 一旦开启，平台发 `start_patrol` 狗就会真动。
3. 电量 `<25%` 不再跑真实巡检。
4. FAST-LIO 发散时立刻停止，不要继续执行路线。
5. `patrol-loop` 不要发 heartbeat 抢 commands，必须用 `--no-heartbeat`。
6. 不要带 4G 冷启动；先启动 Orin，SSH 通后再热插。
7. 不要反复执行 `nmcli con up go2-4g`，A7600C AT 口容易被打 invalid。
8. `gogoguard.cn` 域名在 4G 下可能 reset，狗端使用 `39.96.37.187` 作为后端基准。

---

## 15. 当前交接时的最后状态

截至本文件撰写：

```text
GoGoGuard v1.5 指令链路：已打通
routeUrl 域名改写：已实现并部署
4G：可用但不稳定，需按流程热插和单次激活
最新可用路线：shang8_3.csv
最新可用 PCD：shang8_3.pcd
已知禁止继续运动条件：FAST-LIO 已发散、电量约 19%~20%
```

接手后第一件事：不要立刻继续平台下发巡检。先充电、重启 base/FAST-LIO、确认 odom 正常，再考虑打开 `--execute-safe`。

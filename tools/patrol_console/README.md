# Go2 巡检管理台 (patrol_console)

在 Mac 本地运行的网页管理台，用于 Go2 巡检系统的现场调试：路线录制、点云建图、巡线控制、实时遥测、Z-1Pro 摄像头预览/录制/云台预设。

## 设计原则

- 管理台通过 SSH 调用狗端受控脚本；人工起点锚定和定位会话守护脚本随狗端版本统一部署
- 遥测采集脚本通过 stdin 注入远端 python3 运行，不在狗上落盘
- 只监听 `127.0.0.1`，仅本机浏览器可访问
- 运动类操作三重保护：解锁开关 + 人工起点确认 + FAST-LIO 会话守护

## 快速开始

```bash
# 依赖(装一次): python3 -m pip install fastapi uvicorn
cd tools/patrol_console
nohup python3 server.py > /tmp/patrol_console.log 2>&1 < /dev/null &
# 浏览器打开 http://127.0.0.1:8642
```

停止: `pkill -f "patrol_console/server.py"`（或 `pkill -9 -f server.py`）

## 连接方式（自动按序探测）

| 别名 | 地址 | 场景 |
|------|------|------|
| `go2wired` | 192.168.123.18 | 网线直连（最快 0.3ms，优先） |
| `go2` | 172.20.10.2 | iPhone 热点（现场默认） |
| `go2home` | 192.168.0.122 | 家里 WiFi / 现场同网 WiFi（当前可用） |

断线约 10 秒自动发现（SSH 心跳），自动切换到下一条可用链路。

## 界面布局

```text
左栏·遥测              中栏·3D地图工作区        右栏·操作
电池/电压/电流          已有路线载入+下载CSV     ① 底座(雷达+FAST-LIO)
机身/关节温度          路线录制(起名→录→停)     Z-1Pro 摄像头(探测/云台/拍照/录像/预览)
运动状态码/速度/步态    点云建图(起名→采→存)     ② 巡检控制(解锁/干跑/真实/巡线)
定位坐标/足底力        3D点云+路线+实时轨迹      🔴 急停
Orin CPU温度/WiFi信号  人工起点锚定提示          远端日志查看
远端进程指示灯         动作日志
```

## HTTP API

| 接口 | 说明 |
|------|------|
| `GET /api/status` | 遥测+进程+Orin健康+动作日志（前端 1.5s 轮询） |
| `GET /api/routes` | 路线 CSV 列表（含点数、起点坐标） |
| `GET /api/route_points?path=` | 路线坐标点数组 |
| `GET /api/pcd_list` | 已保存的 PCD 文件列表 |
| `GET /api/pcd_points?path=&max_points=` | PCD 抽样点（2D fallback 用） |
| `GET /api/pcd_pack?path=&max_points=` | 云端同款 `{b64, meta}` 点云包（3D 渲染用） |
| `GET /api/camera_files` | Z-1Pro 最近快照/视频列表 |
| `GET /api/file?path=` | Z-1Pro 快照/视频内嵌预览 |
| `GET /api/download?path=` | 下载 CSV/PCD 原始文件 |
| `POST /api/action` | 白名单动作 `{name, params, armed}` |

动作清单: `start_base` `stop_base` `start_recorder` `stop_recorder` `start_safe`(支持 `dry_run`) `stop_safe` `start_follower` `estop` `stop_all_control` `start_pcd` `stop_pcd` `tail_log` `camera_probe` `camera_preset` `camera_snapshot` `camera_record` `camera_start_loop` `camera_stop_loop` `saas_heartbeat` `saas_manifest` `saas_command_result` `saas_start_loop` `saas_stop_loop`

## GoGoGuard / SaaS 联调循环

狗端脚本:

```bash
/home/unitree/go2_fastlio_ws/scripts/go2_saas_agent.py
```

当前 `xiaoqu1` 现场资产已验证存在:

```text
/home/unitree/go2_fastlio_ws/src/go2_fastlio_patrol/routes/xiaoqu1.csv   # 315 点
/home/unitree/go2_fastlio_ws/maps/console/xiaoqu1.pcd                   # 490,313 点
/home/unitree/go2_fastlio_ws/patrol_logs/videos/z1pro_*.mp4             # 跳过 0 字节，轮换有效视频段
```

干跑一轮，不上传:

```bash
ssh go2home 'source /home/unitree/go2_fastlio_ws/scripts/env_common.sh >/dev/null 2>&1 || true; /home/unitree/go2_fastlio_ws/scripts/go2_saas_agent.py upload-once --route xiaoqu1 --pcd xiaoqu1 --video cycle --patrol-id xiaoqu1-field'
```

干跑循环，不上传:

```bash
ssh go2home 'source /home/unitree/go2_fastlio_ws/scripts/env_common.sh >/dev/null 2>&1 || true; /home/unitree/go2_fastlio_ws/scripts/go2_saas_agent.py patrol-loop --route xiaoqu1 --pcd xiaoqu1 --video cycle --patrol-id xiaoqu1-field --interval 20'
```

真实上传前，在狗端创建私有环境文件（不要写进仓库）:

```bash
ssh go2home
mkdir -p ~/.config
umask 077
read -rsp 'GO2_AUTH_TOKEN: ' GO2_AUTH_TOKEN; echo
cat > ~/.config/go2_saas.env <<EOF
export GO2_AUTH_TOKEN='$GO2_AUTH_TOKEN'
export GO2_AUTH_HEADER='Authorization'
export GO2_BACKEND_BASE='https://39.96.37.187/api/v1'
export GO2_DEVICE_TOKEN='<可选: /devices/plan 和路线下载用的设备令牌>'
export GO2_ROUTE_UPLOAD_ENDPOINT='/robot/asset/upload'
export GO2_PCD_UPLOAD_ENDPOINT='/robot/asset/upload'
export GO2_VIDEO_UPLOAD_ENDPOINT='/robot/video/upload'
EOF
chmod 600 ~/.config/go2_saas.env
```

如果平台要求的认证头不是 `Authorization: Bearer <token>`，只改 `GO2_AUTH_HEADER`；如果 CSV/PCD 的接口不是 `/robot/asset/upload`，只改对应 endpoint。`GO2_DEVICE_TOKEN` 只用于 v1.5 计划接口/路线下载的 `X-Device-Token`。管理台右侧「SaaS 联调循环」会自动读取这个私有文件。

### 命令轮询与本地测试

当前狗端会运行两个 SaaS 循环:

```text
patrol-loop   # 心跳 + 最新 Z-1Pro 视频上传
command-loop  # 心跳轮询 response.commands，并回传 /robot/command/result
```

`command-loop` 默认只做安全识别和回执: `ping/noop/status` 会成功，`start_base/camera_*` 等安全动作默认 dry-run 成功；`goto/go/navigate` 等任意点导航命令会拒绝并回传 `rejected`。命令按 `commandId/id/cmdId` 去重，避免平台重复返回同一条命令时重复执行。

v1.5 路线命令已接入第一版:

```text
plan-fetch             # GET /devices/plan，使用 GO2_DEVICE_TOKEN -> X-Device-Token
start_patrol           # 下载/缓存 CSV，把第0点刚性锚定到当前静止位姿，再执行 follower + safe node
stop_patrol            # 停 follower/safe/cmd 并尝试发 StopMove
goto/go/navigate       # 暂不实现任意点导航，回传 rejected
```

真实执行需要启动 `command-loop --execute-safe`；不加 `--execute-safe` 时 `start_patrol/stop_patrol` 只 dry-run 回执，不会运动。推荐平台命令参数:

```json
{
      "commandId": "cmd-001",
      "action": "start_patrol",
      "params": {
            "fileName": "xiaoqu1.csv",
            "routeUrl": "https://gogoguard.cn/api/v1/.../xiaoqu1.csv",
            "speed": 0.5,
            "loopMode": "pingpong"
      }
}
```

若只下发 `fileName`，狗端会执行本地 `routes/` 下同名 CSV；新狗没有本地副本时，平台必须同时提供 `routeUrl` 或提前同步 CSV。默认 `manual_anchor` 模式不会覆盖原始 CSV，而是在本次巡检目录生成 `route_runtime.csv` 和 `manual_anchor.json`。操作员必须把狗放在路线的真实起点并对准录制时的机头方向；系统不会用 PCD 判断是否摆错。巡检中 FAST-LIO 重启或里程计会话中断会立即终止本次任务。

本地模拟平台测试:

```bash
python3 tools/patrol_console/test_go2_saas_agent.py
```

覆盖项: heartbeat 的 `pose/position/motion.position` 与 v1.5 `status/patrol/diagnostics` 字段、视频 multipart 带 pose、`plan-fetch` 设备令牌、heartbeat 返回 `commands` 后回传 command result、`start_patrol` 下载 CSV 并生成启动命令、`goto` 默认拒绝、重复 commandId 去重。

## 可切换巡检摄像头

当前使用宇树内置固定前摄，统一配置位于：

```text
/home/unitree/go2_fastlio_ws/config/camera.env
GO2_CAMERA_SOURCE=unitree_builtin
```

外置相机修好后把该值改为 `z1pro` 即可；拍照、MP4 分段、预览、下载和平台上传流程不变。宇树内置相机固定前向，因此不支持上下左右云台动作；切回 Z-1Pro 后这些预设自动恢复。

页面能力:

- 探测 RTSP + GCU 控制链路
- 云台预设: `front/down/up/left/right/home`
- 拍照并预览 JPEG
- 录制 20 秒 MP4 并预览/下载
- 启动/停止连续 20 秒分段录制
- 勾选“录制时自动追播最新视频”后，路线/点云录制时自动以约 20 秒延迟播放最新片段；“测试实时播放”可在不录制路线时单独启动分段并预览
- 路线录制和 PCD 采集各自锁定启动时的 FAST-LIO 会话；中途重启定位会终止采集并把输出标记为无效

## 3D 点云 / 路线渲染

本地管理台已内置 Three.js，并复用云端 `pcd-render` 的数据契约：

```text
{ b64, meta: { cx, cy, zmin, zmax, radius, count } }
```

坐标变换与云端当前版本一致：

```text
three.x = 原始x - cx
three.y = 原始z - zmin
three.z = 原始y - cy
```

当前能力：

- 载入 PCD 后显示 3D 点云
- 路线 CSV 叠加为 3D 折线（路线本身是 2D 里程计，z 暂按 0）
- 实时轨迹 / 当前狗位置叠加到 3D 场景
- 保留原 2D canvas 作为 fallback，但默认隐藏

统一入口:

```bash
/home/unitree/go2_fastlio_ws/scripts/go2_camera_capture.sh
/home/unitree/go2_fastlio_ws/scripts/go2_camera_preset.sh
/home/unitree/go2_fastlio_ws/scripts/go2_camera_upload_segment.sh
```

Z-1Pro 专用脚本保留为后端驱动。切回后需注意其偏航可控范围为 `±140°`，不是 360°。

## 下楼采集建议流程

```text
1. Mac 与 Orin 连同一个 WiFi/热点，确认 ssh go2home 可用。
2. 页面“巡检摄像头”显示当前来源，并确认“探测”正常。
3. 启动底座(雷达 + FAST-LIO)，狗静止 10 秒。
4. 单独完成 PCD 采集并停止保存；采集中不要重启雷达或 FAST-LIO。
5. 如需按独立会话录路线，重启底座后把狗放回同一真实起点和朝向，等待定位稳定。
6. 开始 CSV 录制，用遥控器带狗走完整路线，然后正常停止录制。
7. 测试巡检前再次把狗放回真实起点和朝向；启动时管理台会生成本次临时锚定路线。
8. 下载 CSV / PCD / 视频，或后续交给云端 agent 上传。
```

## 架构

```text
浏览器 ←1.5s轮询→ FastAPI(server.py)
                    ├─ 遥测线程: 常驻 SSH + stdin 注入 rclpy 脚本, 1Hz JSON 流
                    ├─ 状态线程: 每 3s pgrep 进程 + 录制行数 + pcd 进度 + Orin 健康
                    └─ 动作接口: 白名单命令经 SSH 执行(参数校验/路径白名单)
                          ↓ SSH (go2wired → go2 → go2home 自动探测)
                    狗上现有脚本: base_bringup.sh / route_recorder / 
                    unitree_safe_cmd_node / waypoint_follower / base_stop.sh
```

## 重要实现注意（踩过的坑）

1. **Mac 默认 python3 是 3.7**：不能用 `str | None` 等 3.8+ 语法
2. **后台启动必须 `nohup ... < /dev/null & disown`**：否则进程碰终端输入会被 macOS 挂起（表现为页面冻结）；macOS 没有 `setsid`
3. **狗上后台进程忽略 SIGINT**：所有停止命令用 `pkill -TERM` + 1 秒后 `pkill -KILL` 升级；唯独点云采集只发 TERM（它靠信号触发保存）
4. **进程检查自匹配**：`[b]racket` 只能避免匹配检查命令本身；若外层 bash
   的完整命令行同时包含真实启动文本，仍会误报。就绪/清理检查应从 `ps` 结果中
   排除 `bash`/`sh`，再匹配真实非 shell 进程。
5. **狗的 ROS2 Foxy 没有 `sensor_msgs_py`**：点云解析用 numpy 手动位偏移解析 PointCloud2
6. **`ros2 topic echo --once` 在该 Foxy 版本不存在**：用 `timeout N ros2 topic echo` 截流
7. PCD 大文件不直接回传：远端 awk 抽样后只传 2D 点（上限 4 万点）

## 已知安全设计

- 急停 = 杀 `waypoint_follower`，安全节点 0.5s 内因命令超时持续发 `Move(0,0,0)`
- 巡线启动条件：底座就绪 + 安全节点在跑 + 狗静止 + 人工确认真实起点/朝向 + 解锁
- 原始 CSV 不覆盖；巡线只读取本次刚性变换后的临时 CSV
- 录制、PCD 或巡线运行时，管理端/SaaS 的底座重启入口会拒绝操作；若底层因故障自行重启，会话守护会停止当前任务
- 干跑模式：安全节点输出到 `/debug/sport/request`，狗不动，用于验证逻辑

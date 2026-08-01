# 08 · 组件清单与依赖事实

> **本文不含任何"可删/该删"的结论。** 只给四栏事实:
> **规模 · 运行证据 · 依赖关系 · 移除后的直接影响**。
> 第 2 步的判断请基于这些事实作出。

**运行证据基准**:`patrol_logs/runs/20260726/xunjian-20260726-02`(最后一次巡检,
其 36 个文件的 sha256 与快照**逐字节一致**,见台账第 28 块) + `patrol_logs/recordings/20260725/`(录制)。

---

## A · 巡检链在跑(每条都有运行证据)

| 组件 | 规模 | 运行证据 | 依赖 / 移除影响 |
|---|---:|---|---|
| `scripts/go2_saas_agent.py` | 3630 | 3 个常驻进程 pid 1470/1471/1472 | **唯一的云端入口与编排者**。移除 = 无法接收任何命令 |
| `src/FAST_LIO/src/laserMapping.cpp` | 1414(团队补丁 8 处) | pid 5628 | 产出 `/Odometry` + `/dev/shm` 快照。移除 = 定位链断,6 个读者全失效 |
| `src/livox_ros_driver2/`(`pub_handler.cpp` 6 处补丁 · `lddc.cpp` 1 处) | — | pid 5275 | 产出 `/livox/lidar` `/livox/imu`。移除 = FAST-LIO 无输入 |
| `scripts/waypoint_follower_go2_2_trace.py` | 1259 | `waypoint_follower.log` 45 KB · trace 13 MB | **实际控制器**。依赖包内基类 + `horizontal_frame.py` |
| `.../go2_fastlio_patrol/waypoint_follower_go2_2.py` | 330 | 作为基类被继承(`trace.py:34,43,148`) | 纯追踪算法本体 |
| `.../go2_fastlio_patrol/unitree_safe_cmd_node.py` | 569 | 日志 301 KB | 避障与限幅。移除 = `/cmd_vel` 无来源 |
| `src/go2_cmd_vel_bridge/src/cmd_vel_udp_sender.cpp` | 247 | 日志 83 KB | Twist→G2CM 包。移除 = UDP 无包 |
| `src/go2_cmd_vel_bridge/src/go2_sdk2_udp_receiver.cpp` | 245 | 日志 83 KB | **从 `build/` 直接执行**。移除 = 指令无人执行 |
| `scripts/horizontal_frame.py` | 349 | trace 中 `horizontal_frame_ready` | 34° 修正的核心 |
| `scripts/build_horizontal_route.py` | 449 | `route_horizontal.csv/.json` | 路线水平化 |
| `scripts/manual_route_anchor.py` | — | `manual_anchor.json` | 起点锚定。失败 → exit 47 |
| `scripts/check_route_start_alignment.py` | — | saas `:2772` | 起点对齐核验 |
| `scripts/check_fastlio_freshness.py` | — | saas `:2763` | 新鲜度门禁。失败 → exit 44 |
| `scripts/localization_session_guard.py` | — | 日志 150 B · pgid 文件 | **熔断器**。失败 → exit 46 |
| `scripts/go2_experiment_snapshot.py` | 552 | `system_start/end.json` 172/92 KB | 失败 → exit 48 |
| `scripts/go2_experiment_telemetry.py` | 637 | `experiment_telemetry.jsonl` 15 MB | 失败 → exit 49 |
| `scripts/patrol_performance_monitor.py` | 764 | 日志 1.9 MB | 失败 → exit 50 |
| `scripts/go2_experiment_audit.py` | 2807 | `experiment_audit.json` 60 KB + `.md` | **每次停止时执行** |
| `scripts/go2_base_health_watchdog.py` | 268 | `ros2 node list` 中活跃 | ⚠️ 日志 4.8 KB,巡检期 run 内为 **0 字节** |
| `scripts/go2_wired_ssh_rescue.sh` | 87 | systemd enable | 网络故障时保住 SSH |
| `scripts/check_livox_stream.py` | 251 | 启动检查 | 6 字段与驱动 `LIVOX_STREAM_HEALTH` 严格对应 |
| `scripts/{base_bringup,ensure_base,ensure_base_ready,env_common,wait_valid_time}.sh` | — | 全链路引用 | 基础层拉起与环境 |

---

## B · 录制链在跑

| 组件 | 规模 | 运行证据 | 备注 |
|---|---:|---|---|
| `scripts/route_recording_blackbox.py` | 1761 | 7 次录制目录(2 次真实:xbf8/xbf9) | 三阶段编排 + 14 项错误检查 |
| `.../go2_fastlio_patrol/route_recorder.py` | 133 | `route_recorder.log` 124 KB | 经 `ros2 run` 启动,`min_distance=0.400` |

---

## C · 相机链在跑

| 组件 | 规模 | 运行证据 | 备注 |
|---|---:|---|---|
| `config/camera.env` | 13 | `GO2_CAMERA_SOURCE=unitree_builtin` | **唯一分叉开关** |
| `scripts/go2_camera_capture.sh` | 216 | **461 段** `unitree_builtin_*.mp4`(7/23–7/26) | 主编排 |
| `bin/go2_builtin_camera_capture` | 二进制 | 同上 | **在运行路径上** |
| `cpp_tools/go2_builtin_camera_capture.cpp` | 源码 | — | 上者的源 |
| `scripts/build_go2_builtin_camera_capture.sh` | — | — | 构建脚本 |
| `scripts/go2_camera_upload_segment.sh` | 68 | outbox 有上传记录 | 分段上传 |
| `scripts/z1pro_capture.sh` | 119 | **218 段** `z1pro_*.mp4`(**7/21–7/22**)+ `z1pro_tests/` 2 组(2026-07-05) | ⚠️ **z1pro 曾是主力相机**,用到 7/22;当前不在货,用户明确要求保留 |
| `scripts/go2_camera_preset.sh` | 45 | 同上 | 云台/变焦,仅 z1pro 适用 |

---

## D · 代码存在,但在最后一次巡检中未执行

> **"未执行"是运行事实,不等于"无用"。** 判断留给第 2 步。

| 组件 | 规模 | 未执行的原因(以证据为凭) |
|---|---:|---|
| `src/go2_map_manager/src/route_relocalizer.cpp` | 1004 | 仅 `localizationMode=pcd` 触发;**132 次巡检记录中命中 0 次**。<br>⚠️ **但它是 `11_prepared_not_deployed.md` 那套方案所需的基础能力**(PCD/ICP 配准)——“未执行”不等于“不需要” |
| `.../go2_fastlio_patrol/waypoint_follower.py` | 1425 | 不在任何启动命令中;实际执行体是 `trace.py` |
| `.../go2_fastlio_patrol/patrol_control.py` | 471 | 无入口;仅被上一行的 `waypoint_follower.py` import 11 个函数 |
| `.../go2_fastlio_patrol/route_quality.py` | 1108 | 无 main / 无 argparse / 未注册 console_script / **全库零引用** |
| `.../go2_fastlio_patrol/waypoint_follower_old.py` | 209 | 排除自身/pycache/.bak 后**全库零引用** |
| `.../go2_fastlio_patrol/unitree_cmd_node.py` | 152 | saas 从不启动;⚠️ **但出现在 stop 的 pkill 模式里**(`:2950,:2952`) |
| `.../go2_fastlio_patrol/unitree_go_safe_cmd_node.py` | 312 | saas 从不启动 |
| `src/go2_loop_backend/` 全 17 模块 | 3094 | 离线建图工具,手动命令行调用,**无运行记录**;产物是 `maps/` 下 7000+ PCD |
| `src/go2_map_manager/src/submap_builder.cpp` | 188 | 仅被 CMakeLists 引用;订阅 `/cloud_registered` 但进程未运行 |
| `cpp_tools/go2_oa_cpp_test/src/oa_cpp_test.cpp` | 76 | 宇树内置避障的试验程序,未采用 |
| `cpp_tools/legacy_iox_stub/` | — | ⚠️ **未核实**,见 `10_open_questions.md` |

---

## D2 · 文档此前未列、且**全库零引用**的脚本(14 个)

> 经第二轮反向核查(源码→文档)补入。判断留给第 2 步。

| 脚本 | 行数 | 归属 |
|---|---:|---|
| `patrol_cli.disabled_before_cmd_rework.py` | **377** | 文件名自带 `disabled`,命令改造前的旧 CLI |
| `go2_a7600c_usb_monitor.sh` | 147 | 4G 遗产 |
| `run_roomtest7_readme_safe_patrol.sh` | 110 | 房间测试脚本 |
| `run_roomtest7_cli_cmd_patrol.sh` | 99 | 同上 |
| `install_a7600c_ecm_only.sh` | 94 | 4G 遗产(ECM 模式,与 PPP 版并存) |
| `probe_legacy_go2_cmd_bridge.sh` | 79 | **legacy 命令桥**组 |
| `body_yaw_alignment.py` | **70** | ⚠️ 机身 yaw 对齐;manifest `body_yaw_alignment_enabled=false`,当前不启用 |
| `install_autostart.sh` | 66 | 与 `install_saas_autostart.sh` 并存的旧版 |
| `start_legacy_go2_cmd_bridge.sh` | 48 | **legacy 命令桥**组 |
| `check_legacy_send_cmd_start.sh` | 45 | **legacy 命令桥**组 |
| `enable_oa_only.py` | 42 | 宇树内置避障开关(与 `oa_cpp_test.cpp` 呼应,同属未采用的探索) |
| `stop_legacy_go2_cmd_bridge.sh` | 18 | **legacy 命令桥**组 |
| `go2_motion_probe.sh` | 16 | 运动探测小工具 |
| `build_legacy_iox_stub.sh` | 11 | **legacy 命令桥**组,`gcc -shared -fPIC` 编译桩库 |

### legacy 命令桥这一组的真相

`start_legacy_go2_cmd_bridge.sh` 头部:
```bash
IOX_STUB="$WS/cpp_tools/legacy_iox_stub/liblegacy_iox_stub.so"
CYCLONE_NO_SHM="$WS/config/cyclonedds_no_shm_eth0.xml"
if [[ ! -f "$IOX_STUB" ]]; then "$WS/scripts/build_legacy_iox_stub.sh"; fi
```
`cpp_tools/legacy_iox_stub/free_iox_chunk_stub.c` 全文只有两个空实现:
```c
void  free_iox_chunk(void *iox_sub, void **iox_chunk) { if (iox_chunk) *iox_chunk = NULL; }
void *iceoryx_header_from_chunk(const void *iox_chunk) { return (void *)iox_chunk; }
```
→ **iceoryx(共享内存传输)的空桩**,配合 `cyclonedds_no_shm` 配置,
  让旧版 go2 命令桥在没有 iceoryx 的环境下也能链接运行。
→ **该组 5 个脚本全部零引用**,是被 `cmd_vel_udp_sender` + `go2_sdk2_udp_receiver` 取代的旧通路。

---

## E · 4G 治理:整套现已停用

| 组件 | 规模 | 状态 |
|---|---:|---|
| `scripts/go2_4g_manager.py` | 1594 | 曾运行(写了 96 条故障记录,7/21–7/23) |
| `scripts/go2_connectivity_watchdog.sh` | 1053 | + **10 个 `.bak`**(781~969 行不等) |
| `scripts/go2_network_recover.sh` | 248 | + 1 `.bak` |
| `scripts/install_a7600c_ppp_only.sh` | 159 | + 4 `.bak` |
| `scripts/install_go2_4g_manager.sh` | 255 | 声明管理 **8 个 systemd 单元** |
| `scripts/install_connectivity_watchdog.sh` | 78 | — |
| `scripts/install_network_recover.sh` | 50 | — |

**关键事实**:
1. 该安装脚本声明的 8 个单元(`go2-4g-manager` · `go2-xhci-keep-awake` · `go2-4g-rndis-block` ·
   `go2-4g-serial-isolation` · `go2-4g-cfun-test` · `go2-connectivity-watchdog` · `go2-network-recover`)
   在 `system_config/etc/systemd/system/` 中**一个都不存在**
2. 该脚本 `:82-86` 的动作本身就是 `systemctl disable --now`
3. 故障记录时间窗 **2026-07-21 22:07 → 07-23 14:07**,此后无新记录
4. **当前上网方式**:`default via 192.168.0.1 dev usb0 proto dhcp` —— 标准 USB 网卡 + DHCP,零自定义代码

---

## F · 目录级事实(见 `05_dependencies.md` 详证)

| 目录 | 是否在运行路径上 | 证据 |
|---|---|---|
| `build/` | **是** | ① receiver 直接执行 `build/go2_cmd_vel_bridge/` 下二进制;② livox 的 install 入口是指向 `build/` 的软链;③ 录制会话记录的 FAST-LIO 可执行路径也是 `build/` |
| `bin/` | **是** | `go2_camera_capture.sh:31` `BUILTIN_BIN=${WS}/bin/go2_builtin_camera_capture` |
| `install/` | **是** | `ros2 run` 的解析目标;多数文件是指向 `src/`/`build/` 的软链 |
| `src/build/` `src/install/` | **否** | 72 / 36 个文件,带 `COLCON_IGNORE`;是"有人在 src 目录里跑过 colcon build"的产物 |

---

## F2 · `bags/` 离线数据集(第五轮核查补入,此前 12 份文档零提及)

`/home/unitree/go2_fastlio_ws/bags/` —— **6 个 bag,共 2.0 GB**,与 `maps/` 下的实验目录同名。

| bag | 时长 | 大小 | 话题构成 | 可重放性 |
|---|---:|---:|---|---|
| **`bag_002`** | 48.3 s | 193 MB | `/livox/imu`×9659 · **`/livox/lidar`×471**(CustomMsg 原始) | ✅ **可从头跑 FAST-LIO** |
| `campus_building_loop_001` | 382.4 s | 824 MB | `/Odometry`×3726 · `/cloud_registered`×3726 · `/cloud_registered_body`×3726 | 仅可重放已配准点云 |
| `lab_outdoor_param_001` | 137.3 s | 388 MB | 同上 ×1321 | 同上 |
| `loop_backend_test_002` | 108.8 s | 261 MB | `/cloud_registered_body`×1060 · `/Odometry`×1060 | 同上 |
| `extrinsic_est_false_check_001` | 85.5 s | 250 MB | 同上 ×837 | 同上 |
| `lab_small_001` | 76.3 s | 220 MB | 同上 ×720 | 同上 |

`bag_002` 的 471 帧 / 48.3 秒 = **9.75 Hz**,接近标称 10 Hz。

### 全狗可重放资源的精确账(`inventory/rosbag-replay-candidates.txt`)

```
rosbag metadata 总数            49
含 /cloud_registered_body        6
含 /cloud_registered             6
含【原始 /livox/lidar】           2      ← bags/bag_002 与 runs/20260723/xunjian-20260723-01
含 /livox/imu                   20
```

**当前 xbf9 黑盒录制的结论(原文)**:
`can_replay_odometry_and_body_state_but_cannot_replay_map_localization_scan_matching`

→ **能从头验证 FAST-LIO 或点云配准的数据源,全狗只有 2 个。** 这是 `11_prepared_not_deployed.md`
所要求的“本地回放测试”的全部家底。

---

## F3 · 快照时刻(2026-07-27 21:41)的完整进程盘点

**本项目只有 4 个进程**:
```
722    root      bash scripts/go2_wired_ssh_rescue.sh
1444   unitree   go2_saas_agent.py command-loop
1449   unitree   go2_saas_agent.py outbox-loop
1450   unitree   go2_saas_agent.py video-loop
```

⚠️ **pid 随开机变化**:上表为 7/27 快照时;7/26 巡检时同样三个 loop 的 pid 是 **1470/1471/1472**。
本交付其余各处出现的 pid 均为**当次运行的实测值,不是固定值**。

**其余全部为系统与宇树自带**,其中三项值得留意:
| 进程 | 说明 |
|---|---|
| `667 root /usr/bin/python3 /upgradePythonServer/server.py` | 宇树升级服务(应即 `tcp 0.0.0.0:80` 的监听者) |
| `1410 containerd` + `1453 dockerd` | **Docker 在跑**(`docker.service` enabled;`docker0` 为 linkdown) |
| `1561 gdm3` + `gdm-x-session` + `pulseaudio` + `tracker-miner-fs` | **完整 GNOME 桌面在跑** |

**监听端口**(`inventory/listening_ports.txt`):
```
tcp 0.0.0.0:22    SSH          tcp 0.0.0.0:80    (宇树升级服务)
udp 0.0.0.0:7400/7401  CycloneDDS discovery(saas 持有)
udp 5005          【空闲】—— 快照时运动链未运行,与 final-live-state.txt 的 udp_5005=free 一致
```

**systemd enabled 服务共 88 个**,其中本项目 4 个(见 `02_flow_timeline.md` 阶段 0)。

---

## G · 历史文件统计

| 类别 | 数量 | 位置 |
|---|---:|---|
| `.bak*` 文件 | **104** | `scripts/` 47 · `src/` 32 · **`install/` 25** |
| `backups/` 目录 | 多套 | 含 `unified_horizontal_20260725_1809/` · `body_yaw_alignment_before_20260725_155151/` · `route_recorder*` 等 |
| 同一 trace 文件的副本 | **3 份** | 现役 1 + backups 2 |
| `routes/` 下 CSV | **80 个 / 78 种内容** | 含 1 组内容错配(见 `06_invariants.md` III-2) |
| `outbox/failed/` 上传失败任务 | **225** | 与 4G 时期的 414 次 POST_ERROR 呼应 |
| 残留进程标记 | 3 | `heartbeat-safe.pid`(7/19 后即失效)· `outbox.run.paused_20260716_2150` · `video.run.paused_20260716_2150` |

---

## H · 曾经运行、现已停止的服务

| 名称 | 日志 | 事实 |
|---|---:|---|
| `go2-saas-heartbeat-safe` | 240 KB | 仅存活 **2026-07-19 16:43:48 → 18:40:37**(约 2 小时)。**830 次请求**中 **416 成功 / 414 失败**(成功率 **50.1%**;日志另有 831 行 CycloneDDS 弃用警告,每次请求前一条),末条为 `Network is unreachable`。**无对应 systemd 单元**;saas 里有 `heartbeat-once`(`:3516`)与 `cmd_heartbeat_loop`(`:3275`)代码,**无任何调用方** |
| `go2-lio-trace-recorder` | 70 KB | 产物 `diagnostics/lio_trace_20260716_*`(每组 5 个 csv)。`go2_lio_trace_recorder.py` **全库只有自己提到自己**,无启动方 —— 手动诊断工具 |

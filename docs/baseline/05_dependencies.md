# 05 · 组件依赖图

> ⚠️ **基准:2026-07-27 整机快照(删除阶段之前)。**
> 本篇描述的是**删代码之前**的狗端状态,内容在该基准下成立。
> 读时注意两点:
> 1. **有些组件现在已从仓库删除**(4G 治理整套、legacy 命令桥、22 个脚本等)——
>    见 `13_file_accounting.md` 与 `14_dog_verification.md`。文档没写错,是那个时间点确实有。
> 2. **`go2_saas_agent.py` 的行号已漂移**(3630 → 3633 行,删除阶段改过 4 处)。
>    换算:引用 `1–52` 不用调 · `53–480` **+3** · **`481–3184` +5**(绝大多数引用在此段) · `3186` 之后 +3。
>    其他文件行号不受影响。
> 已用已知答案自检:本文若写 `commands` 列表在 `:2590`,当前仓库实测在 `:2595`,差 5 ✓

> **本文只陈述依赖事实**:X 依赖 Y,移除 Y 会导致 Z 断。
> **不含任何"可删/该删"的判断** —— 那是第 2 步的工作,本文是它的输入。

---

## 一、目录级依赖 ★ 最容易踩坑的一层

```
/home/unitree/go2_fastlio_ws/
├── src/          源码。colcon --symlink-install 的源,install/ 里多数是指向它的软链
├── build/        ⚠️ 在运行路径上,见下
├── install/      ROS 运行入口(ros2 run 解析到这里)
├── bin/          ⚠️ 在运行路径上,见下
├── scripts/      40+ 脚本,含最大的 go2_saas_agent.py(3630 行)
├── config/       camera.env 等
├── cpp_tools/    相机取流源码 + legacy_iox_stub
├── patrol_logs/  全部运行记录(证据源)
└── maps/         7000+ PCD 产物
```

### `build/` 处在运行路径上 —— 两条独立证据

**证据 1**:`go2_sdk2_udp_receiver` **直接执行 build 下的二进制**,不走 `install/`、不走 `ros2 run`
```
saas:2419-2421   "GO2_SDK_MAX_VY=0.020 %s/build/go2_cmd_vel_bridge/go2_sdk2_udp_receiver %s 5005"
```

**证据 2**:livox 驱动的 install 入口**是指向 build 的软链**
```
install/livox_ros_driver2/lib/livox_ros_driver2/livox_ros_driver2_node
    → build/livox_ros_driver2/livox_ros_driver2_node      sha cc31e33470b1b9dc ✅
```

**证据 3**:录制的会话身份记录里,FAST-LIO 可执行文件路径也指向 build
```
route_link.json.localization_session_start.executable
    = /home/unitree/go2_fastlio_ws/build/fast_lio/fastlio_mapping
```

→ **移除 `build/`:UDP 接收端直接消失(狗无法接收运动指令),livox 驱动入口变断链。**

### `bin/` 处在运行路径上

```
go2_camera_capture.sh:31   BUILTIN_BIN=${GO2_BUILTIN_CAMERA_BIN:-${WS}/bin/go2_builtin_camera_capture}
```
→ **移除 `bin/`:内置相机取流失败**(源码在 `cpp_tools/go2_builtin_camera_capture.cpp`,需重新编译)。

### `install/` 中的软链特性

`colcon build --symlink-install` 使 `install/` 里多数文件是指向 `src/` 或 `build/` 的软链。
已验证的 4 条:
```
install/fast_lio/share/fast_lio/config/go2_mid360s.yaml   → src/FAST_LIO/config/go2_mid360s.yaml
install/fast_lio/share/fast_lio/launch/mapping.launch.py  → src/FAST_LIO/launch/mapping.launch.py
install/livox_ros_driver2/share/.../MID360s_config.json   → src/livox_ros_driver2/config/MID360s_config.json
install/livox_ros_driver2/lib/.../livox_ros_driver2_node  → build/livox_ros_driver2/livox_ros_driver2_node
```
→ **后果**:Python 侧改 `src/` 立即生效(无需重编);C++ 侧改 `src/` 需重新 `colcon build`。
→ **副作用**:在非狗环境(如 Mac)检查文件存在性时,这些软链是**断链**,
   `os.path.isfile()` / `[ -f ]` 会返回 False。(我为此栽过三次。)

---

## 二、进程启动依赖(谁拉起谁)

```
systemd
 ├─ go2-saas-command.service
 │    ├─ ExecStartPre ──► scripts/wait_valid_time.sh   ← 等系统时间有效后才继续
 │    └─ ExecStart ────► go2_saas_agent.py command-loop
 │                                  │
 │                                  ├─ base bringup ──► livox_ros_driver2_node
 │                                  │                 └► fastlio_mapping
 │                                  ├─ setsid nohup ──► go2_base_health_watchdog.py
 │                                  │
 │                                  └─ start_patrol(57 步)──► 巡检 10 进程,见下
 ├─ go2-saas-outbox.service  ──► go2_saas_agent.py outbox-loop
 ├─ go2-saas-video.service   ──► go2_saas_agent.py video-loop ──(门控)──► go2_camera_capture.sh
 └─ go2-wired-ssh-rescue.service ──► go2_wired_ssh_rescue.sh
```

**巡检期间由 saas 启动的进程(启动顺序即依赖顺序)**:
```
1. go2_experiment_snapshot.py --phase start
2. go2_experiment_telemetry.py --profile patrol
3. patrol_performance_monitor.py
4. go2_sdk2_udp_receiver          (build/ 下二进制)
5. cmd_vel_udp_sender             (ros2 run)
6. unitree_safe_cmd_node          (ros2 run)
7. manual_route_anchor.py [+ build_horizontal_route.py]
8. check_route_start_alignment.py
9. ros2 bag record
10. localization_session_guard.py
11. waypoint_follower_go2_2_trace.py   (python3 裸脚本)
停止时另加:go2_experiment_snapshot.py --phase end · go2_experiment_audit.py
```

**录制期间(5 进程)**:telemetry(profile=recording) · performance · rosbag(9 话题) · session_guard · **route_recorder(ros2 run)**

---

## 三、运行时依赖(谁需要谁活着)

```
fastlio_mapping
   ├─► /Odometry ──────────┬─► follower          (无它:follower 收不到定位,不放行)
   │                       ├─► safe node
   │                       ├─► telemetry / rosbag
   │                       └─► submap_builder(未运行)
   ├─► /cloud_registered_body ─► safe node       (无它:避障失效,cloud_timeout 触发)
   └─► /dev/shm 快照 ──────┬─► base_bringup.sh / ensure_base_ready.sh
                           ├─► check_fastlio_freshness.py   (无它:exit 44)
                           ├─► manual_route_anchor.py       (无它:exit 47)
                           ├─► localization_session_guard.py
                           └─► check_route_start_alignment.py

livox_ros_driver2_node ─► /livox/lidar + /livox/imu ─► fastlio_mapping
                                        └─► follower(horizontal_imu_topic,水平化标定)

follower ─► /patrol_cmd ─► safe node ─► /cmd_vel ─► cmd_vel_udp_sender
                                                      ─UDP 5005─► go2_sdk2_udp_receiver ─► 宇树 SDK

宇树自带进程 ─► /lf/sportmodestate ─► follower(body_yaw,当前未启用)+ telemetry + saas
             ─► /lf/lowstate、/wirelesscontroller ─► telemetry
```

**链路断点的表现**:
| 断掉的环节 | 表现 |
|---|---|
| livox 驱动 | FAST-LIO 无输入 → 快照不刷新 → 启动时 exit 44 |
| fastlio | 同上;运行中则 guard 检测到会话消失 → 熔断 |
| safe node | `/cmd_vel` 无来源 → sender 收不到 → 狗停 |
| sender | UDP 无包 → receiver 的 `cmd_timeout` 触发 → 狗停 |
| receiver | 指令无人执行 → 狗不动(启动时 exit 42) |
| session_guard | 启动时 exit 46;运行中失去熔断保护 |

---

## 四、代码级依赖(import / 继承)

```
waypoint_follower_go2_2_trace.py   (scripts/,不在 ROS 包内)
   │ :34  from go2_fastlio_patrol import waypoint_follower_go2_2 as base_module
   │ :43  BaseFollower = getattr(base_module, "WaypointFollower")
   │ :148 class TracedWaypointFollower(BaseFollower)
   │ :977 super().control_loop()
   └──依赖──► src/go2_fastlio_patrol/go2_fastlio_patrol/waypoint_follower_go2_2.py  (基类,330 行)
   └──依赖──► scripts/horizontal_frame.py                                          (水平化)

go2_saas_agent.py ──调用(subprocess)──► 约 15 个 scripts/ 下的脚本
   manual_route_anchor.py · build_horizontal_route.py · check_route_start_alignment.py
   check_fastlio_freshness.py · localization_session_guard.py · go2_experiment_snapshot.py
   go2_experiment_telemetry.py · go2_experiment_audit.py · patrol_performance_monitor.py
   go2_base_health_watchdog.py · ensure_base.sh · base_bringup.sh
   **base_stop.sh**(saas `:3184`,云端 `stop_base` 安全命令)
   **z1pro_upload_segment.sh**(saas `:3364`)

**相机预设的三级链**(仅 z1pro 源时走):
```
go2_camera_preset.sh:21  exec ${WS}/scripts/z1pro_preset.sh <preset>
        └──► z1pro_preset.sh (39 行)
                 └──► z1pro_gcu_control.py (183 行)   ← 云台/变焦实际控制
```

route_recording_blackbox.py ──► route_recorder(ros2 run) + 上述观察类脚本
```

**ROS 包内 10 个 py,`setup.py` 注册 6 个 console_scripts**:
```
注册:route_recorder · waypoint_follower · waypoint_follower_go2_2 ·
      unitree_cmd_node · unitree_safe_cmd_node · unitree_go_safe_cmd_node
未注册(纯库/无入口):patrol_control.py · route_quality.py · waypoint_follower_old.py · __init__.py
```
⚠️ 注意:被注册的 `waypoint_follower_go2_2` **不是实际执行体**;
实际执行体 `waypoint_follower_go2_2_trace.py` 在包外,**未注册**,由 saas 用 `python3 <路径>` 直接跑。

---

## 五、配置依赖

| 配置 | 被谁读 | 缺失后果 |
|---|---|---|
| `install/fast_lio/share/fast_lio/config/go2_mid360s.yaml` | fastlio 进程(`--params-file`) | 回落到代码默认值:`extrinsic_est_en` 变 true、`path_en` 变 true 等 |
| `config/camera.env` | `go2_camera_capture.sh` / `preset.sh` / `upload_segment.sh` | **三者默认值均为 `z1pro`**,会静默切到当前不在货的设备 |
| `MID360s_config.json` | livox 驱动 | 雷达无法连接 |
| `scripts/env_common.sh` | 各 systemd 单元的 `bash -lc` 里 source | ROS 环境缺失 |

---

## 六、第三方边界(已 grep 穷尽,零团队标记 = 未改动)

| 目标 | 团队标记 | 状态 |
|---|---:|---|
| `src/FAST_LIO/src/laserMapping.cpp` | **8 处** | 已改 |
| `src/livox_ros_driver2/src/comm/pub_handler.cpp` | **6 处** | 已改 |
| `src/livox_ros_driver2/src/lddc.cpp` | **1 处** | 已改 |
| `src/Livox-SDK2/`(202 源文件) | **0** | 原版 |
| `src/FAST_LIO/include/`(IKFoM_toolkit · ikd-Tree · so3_math.h · common_lib.h · use-ikfom.hpp · Exp_mat.h · matplotlibcpp.h) | **0** | 原版 |

**编译依赖**:`src/go2_cmd_vel_bridge/CMakeLists.txt:10` 需要
`third_party/unitree_sdk2_install/lib/libunitree_sdk2.a`(预编译静态库)。

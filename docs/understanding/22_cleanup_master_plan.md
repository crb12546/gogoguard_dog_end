# 22 · 屎山整理终极方案(分类 → 重组 → 改造)

> 原则同 00:只认代码。锚在**狗真身 `realtime_dog_end_code`**(= 背板实跑,已 sha 验)。
> 方法:从 **7 个 systemd 运行时根**(doc 21+系统单元)做**传递可达性**得"真在用"集,其余按证据归类。
> ✅ **本篇分类已由穷尽核验工作流(10 agent 逐文件对全量狗代码 grep)+ 补批核验,100 个自研文件全核完,14 个被纠正**。§1 为核验版。我手搓可达性初稿翻过 3 次车、且分类初稿错 14/100——**所以下面的分类是核过的,不是我拍的**。

## ★ 核验揪出的 3 个系统级纠错(不只是分桶,改了对系统的认知)
1. **相机:狗当前跑内置相机 `unitree_builtin`,非 Z1Pro**。`config/camera.env` `GO2_CAMERA_SOURCE=unitree_builtin`(注释"Z-1Pro kept ready for switching back")→ 整条 z1pro 链(`z1pro_capture/gcu_control/preset` `go2_camera_preset`)是**配置门控的非激活后端**(E)。运行时相机链:saas video → `z1pro_upload_segment.sh`(兼容入口)→ `go2_camera_upload_segment.sh` → `go2_camera_capture.sh` →(builtin)`go2_builtin_camera_capture` 二进制。**doc 08/10 说的"z1pro 相机"对当前狗是错的,待修**。
2. **4G:狗跑 shell `go2_connectivity_watchdog.sh`(canonical),非 Python `go2_4g_manager.py`**。后者 installer `Conflicts=`+`disable` 前者、opt-in(只 README)→ 未采用。**两套 4G 悬案定案:shell 版**。
3. **录制 `route_recorder`/`route_recording_blackbox` 是离线/手动路线制作,不在自动巡检运行时**(巡检用 `ros2 bag record`)→ E,非 A。

## 〇、这系统由什么组成(6 大子系统 + 支撑)
1. **感知/定位**:Livox 驱动 → FAST-LIO → `/Odometry`+`/cloud_registered_body`(三方 SLAM/驱动,整体保留)。
2. **巡检控制**:follower(纯跟踪)→ 安全节点(限速+点云急停)→ cmd_vel 桥 → SDK2 驱腿。
3. **云端 SaaS**:`go2_saas_agent.py` 三循环(command/video/outbox)+ start_patrol 现场装配。
4. **定位守卫**:manual_anchor / session_guard / health_watchdog / 起点对齐 / 新鲜度门。
5. **取证/诊断**:telemetry / snapshot / performance / experiment_audit / 录制黑盒。
6. **网络生存**:4G(connectivity_watchdog + network_recover)+ 相机(z1pro)。
支撑:部署脚本(install_*)、离线建图(loop_backend)、测试、遗留。

## 一、分类总表(每类:是什么 → 处置)
> 处置 5 档:**A 保留-运行时**(部署到狗)/ **B 保留-部署工具**(单独放,非运行时)/ **C 丢弃-死代码** / **D 隔离-测试**(择机看)/ **E 隔离-未部署/废弃算法**(先放着)/ **F 分析数据**(重组)。

### A · 保留-运行时(狗上真在用,可达性+doc 08/11/12/13/14 佐证)
| 子系统 | 文件 |
|---|---|
| 基础层 | `base_bringup.sh` `base_stop.sh` `ensure_base_ready.sh` `env_common.sh` `wait_valid_time.sh` `check_livox_stream.py` `check_fastlio_freshness.py` + `FAST_LIO/launch/mapping.launch.py` + 配置 `go2_mid360s.yaml` `MID360s_config.json` `cyclonedds_no_shm_eth0.xml` + 三方包 `FAST_LIO`/`livox_ros_driver2`/`unitree_api`(整体) |
| SaaS | `go2_saas_agent.py` `start_saas_loops.sh` |
| 巡检控制 | `waypoint_follower_go2_2_trace.py`+`waypoint_follower_go2_2.py`、`unitree_safe_cmd_node.py`、`cmd_vel_udp_sender.cpp`、`go2_sdk2_udp_receiver.cpp`、`route_relocalizer.cpp` |
| 定位守卫 | `manual_route_anchor.py` `localization_session_guard.py` `go2_base_health_watchdog.py` `check_route_start_alignment.py` `body_yaw_alignment.py` `horizontal_frame.py` |
| 取证诊断 | `go2_experiment_telemetry.py` `go2_experiment_snapshot.py` `patrol_performance_monitor.py` `go2_experiment_audit.py` |
| 录制 | `route_recorder.py` `route_recording_blackbox.py` |
| 网络生存 | `go2_connectivity_watchdog.sh` `go2_network_recover.sh` |
| 相机 | `z1pro_capture.sh` `z1pro_upload_segment.sh` `go2_camera_capture.sh` `go2_camera_upload_segment.sh` + `camera.env` |
| 停车保险 | `go2_sdk2_motion_probe.cpp`(stop 时 saas 调) |

### B · 保留-部署工具(单独文件夹,装机时跑一次,非运行时)
`install_autostart.sh` `install_saas_autostart.sh` `install_connectivity_watchdog.sh` `install_network_recover.sh` `install_go2_4g_manager.sh` `install_a7600c_ecm_only.sh` `install_a7600c_ppp_only.sh` `go2_wired_ssh_rescue.sh` `build_go2_builtin_camera_capture.sh` `build_legacy_iox_stub.sh`

### C · 丢弃-死代码(7,无人引用 + legacy/disabled 自述,可直接进 attic/dead)
`waypoint_follower_old.py`、`start_legacy_go2_cmd_bridge.sh` `stop_legacy_go2_cmd_bridge.sh` `probe_legacy_go2_cmd_bridge.sh`、`check_legacy_send_cmd_start.sh`、`build_legacy_iox_stub.sh`、`patrol_cli.disabled_before_cmd_rework.py`

### D · 隔离-测试(12,择机看)
`src/*/test/test_*.py`(两包共 8)、`fake_odom_line_corner_dryrun.py`、`run_roomtest7_{cli_cmd,readme_safe}_patrol.sh`、`cpp_tools/go2_oa_cpp_test/`

### E · 隔离-未部署/废弃(38,先放着)★核验后大幅扩容(A 类误判多在此纠回)
- **离线建图整包** `go2_loop_backend/*`(17:keyframe_saver/build_raw_map/scan_context_detector/pose_graph_optimizer/dynamic_map_filter/level_*/pcd_to_nav2_map/odom_to_tf_* …)——手动离线,不在实时环。
- **z1pro 相机链(非激活后端,`camera.env=unitree_builtin`)** `z1pro_capture.sh` `z1pro_gcu_control.py` `z1pro_preset.sh` `go2_camera_preset.sh`。
- **离线路线制作(非巡检运行时)** `route_recorder.py` `route_recording_blackbox.py` `route_quality.py`。
- **命令节点变体** `unitree_cmd_node.py` `unitree_go_safe_cmd_node.py`;**测试版 follower** `waypoint_follower.py`(只被 run_roomtest 跑)。
- **未接线手动/诊断/救援** `go2_lio_trace_recorder.py` `enable_oa_only.py` `go2_a7600c_usb_monitor.sh` `go2_start_level_scan.sh` `build_horizontal_route.py` `submap_builder.cpp` `go2_motion_probe.sh` `go2_wired_ssh_rescue.sh` `start_saas_loops.sh`(0 引用的备用启动器)。
- **nav2/amcl/slam_toolbox 旁支配置** `config/go2_amcl*.yaml` `go2_slam_toolbox*.yaml` `nav2_params.yaml`;**仅仓库未上狗(doc 21)** `go2_course_control.py`/`go2_pcd_capture.py`/`horizontal_frame_calibration.json`。

### F · 分析数据(重组,不是代码)
`analysis/`(8963)、`maps/`、`patrol_logs/`、`backups/`、`routes/` 的 `xbf*` 质量/录制 json。→ 独立 `evidence/` 或 `data/`,和代码分家。

## 二、目标结构(重组成什么样)
```
gogoguard/
├─ dog/                         ★部署到狗的唯一 canonical 源(= A 类,按子系统分包)
│  ├─ base/           (bringup + livox/fastlio 配置 + 健康门)
│  ├─ perception/     (FAST_LIO, livox_ros_driver2, unitree_api — 三方,pin 版本)
│  ├─ patrol/         (follower + safe + cmd_vel 桥)
│  ├─ localization/   (anchor/session_guard/health/alignment)
│  ├─ saas/           (go2_saas_agent;start_saas_loops 已废→attic)
│  ├─ evidence/       (telemetry/snapshot/performance/audit;录制黑盒是离线→attic)
│  ├─ network/        (4g:go2_connectivity_watchdog[canonical]+network_recover;Python manager→attic)
│  └─ camera/         (builtin 激活:go2_camera_capture+builtin二进制;z1pro 整链→attic/unshipped)
├─ deploy/                      ★B 类:装机脚本 + systemd 单元定义(一处集中)
├─ evidence_data/               ★F 类:analysis/maps/logs(和代码分家,可 gitignore 大件)
├─ attic/                       ★"临时_用不上"
│  ├─ dead/           (C 类,确认死)
│  ├─ tests_old/      (D 类,择机看)
│  └─ unshipped/      (E 类,未部署/废弃算法)
└─ tools/                       (笔记本上跑的离线工具:build_route/loop_backend,如仍要用)
```

## 三、剩下的"也不合理",应改成什么
1. **`scripts/` 50 文件扁平杂物抽屉** → 按子系统拆进 `dog/*`(见上),不再一个平目录塞所有。
2. **3 个 follower 同 node 名 `waypoint_follower`** → 只留狗上跑的 `waypoint_follower_go2_2`(330),node 名改唯一;`waypoint_follower.py`(测试)进 attic 或明确标测试。
3. **同一参数散在 default/saas/README 三处** → 收敛:安全/follower 参数只留一处权威(saas 启动串或独立 config),README 只引用不重复。
4. **仓库↔狗无部署纪律** → `deploy/` 里放一个 `deploy.sh`:显式 rsync + sha 校验 + 记录 commit(doc 21 的双向回捞先做:狗的 PPP/eth1 补丁回仓,仓库的 safety/4G 改进按评审下发)。
5. **两套 4G / 双相机后端** → ✅**已核定 canonical**:4G=`go2_connectivity_watchdog.sh`(shell)、相机=`unitree_builtin`;另一套(Python 4G / z1pro 链)进 attic/unshipped。
6. **systemd 单元散在 install 脚本里** → `deploy/systemd/` 集中放 7 个单元定义,一眼可见开机拉起什么。

## 四、执行顺序(先分类不动码,再动)
1. **先出核验版分类**(本篇 + 工作流核验,§6 存疑清零)。
2. **定基线**:仓库↔狗 6 分叉逐个定 canonical(doc 21),双向回捞。
3. **建 `dog/` canonical 源**(把 A 类按子系统归位,不改逻辑)。
4. **挪 attic**(C/D/E),`git mv` 保留历史,不删。
5. **补 `deploy/`**(systemd 集中 + deploy.sh)。
6. **收敛配置/命名**(§三)。
7. 冻结话题契约(doc 20)后再谈改逻辑。

## 五、风险
- **误杀**:把"看着死实则运行时用"的挪进 attic → 狗炸。故 A/C/E 边界必须核验(§6),尤其可达性假阴性。
- **丢现场补丁**:狗领先的文件(PPP)没回捞就以仓库覆盖 → 丢。
- **快照时效**:`realtime_dog_end_code` 是昨晚快照,狗"明儿还改",分类要能重跑。

## 六、存疑项(已由核验工作流逐条决案)
1. `waypoint_follower.py`+`patrol_control.py` → **follower 是测试链(E)**;但 `patrol_control.py` 被运行时 `unitree_safe_cmd_node` import(限幅/侧向ROI/流龄)→ **patrol_control 留 A**。
2. `go2_4g_manager.py`(Python 4G) → **B/未采用**:其 installer `Conflicts=`+`disable` 掉 `go2_connectivity_watchdog.service`、opt-in;**狗跑 shell watchdog(canonical)**。
3. `route_quality.py` → **E**:唯一消费者是单测,无运行时节点 import。
4. `z1pro_gcu_control/z1pro_preset/go2_camera_preset` → **E**:`camera.env=unitree_builtin`,z1pro 后端未激活。
5. 相机 canonical → **`unitree_builtin`**(`go2_camera_capture.sh`→builtin 二进制);z1pro 整链 attic。
6. `go2_sdk2_motion_probe` → **A**:运行时确在 stop 用(`go2_saas_agent:2955`、`session_guard:80`、`health_watchdog:50` 直调 build 二进制)。
7. `body_yaw_alignment.py` `horizontal_frame.py` → **A**(运行时可达)。
8. A 类每个已反向核("从 5 systemd 根追到");核验共纠正 14/100(A 类误判最多,多为 config 门控非激活/离线工具)。

## 核验状态
✅ **已核验**:100 自研文件全部逐个对全量狗代码 grep 核过(工作流 90 + 补批 10),分类见 §1(核验版),存疑 §6 已决,3 处系统级纠错见头部。**可作重组依据;唯一残留:快照时效(狗会再改,分类需可重跑)+ 三方包/配置未纳入本 100(整体 KEEP)。**

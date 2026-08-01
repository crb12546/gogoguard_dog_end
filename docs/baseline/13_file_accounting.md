# 全量归位表 —— 213 个团队文件逐个对账

> **这篇存在的理由**:如果有代码存在,而 `12_one_line_walkthrough.md` 那条线没描述到,
> 那要么是那条线漏讲了,要么是这段代码本来就不该在。**两种都必须点名,不能含糊过去。**
>
> **基准**:`gogoguard_dog_go2_real_code` @ `8b3d789`(2026-08-01,已完成删除阶段)
> **方法**:`git ls-files` 全量枚举 → 逐个 grep 反向引用 → 不抽查、不凭文件名判断

---

## 一、总账(命令核准,非手数)

```
git ls-files 总计                                    591
├ 上游第三方(不算团队代码)                          378
│   Livox-SDK2 221 · livox_ros_driver2 104 · FAST_LIO 43 · unitree_api 10
└ 团队自己的                                         213
    ├ 历史路线数据 routes/                           103   ← 数据,不是代码
    ├ 构建元数据 package.xml/setup/CMakeLists/resource 12
    ├ 测试 test/                                       9
    ├ .bak 残留(仍在盘上)                             6
    ├ 说明文档 + 预编译二进制                           4
    ├ 配置 yaml/xml/env/service                       15
    └ 真正的代码 py/cpp/sh                            64
```

**校验**:`103 + 12 + 9 + 6 + 4 + 15 + 64 = 213` ✅

> ⚠️ 注意 `.bak` 6 个仍被 git 跟踪,盘上另有 24 个未跟踪的
> (`src/go2_fastlio_patrol/go2_fastlio_patrol/` 下 17 个 `waypoint_follower.py.bak_*`)。
> 前者可删,后者是本地残留。

---

## 二、64 个代码文件 —— 逐个归位

### ⓪ 开机 / 底座(7)
| 文件 | 位置 |
|---|---|
| `scripts/base_bringup.sh` | 起雷达 + FAST-LIO |
| `scripts/base_stop.sh` | 停底座 |
| `scripts/ensure_base_ready.sh` | 就绪探针(saas :2767 调) |
| `scripts/env_common.sh` | 公共环境;`:33` **运行时现生成** cyclonedds 配置到 `/tmp` |
| `scripts/wait_valid_time.sh` | 等系统时间有效 |
| `scripts/start_saas_loops.sh` | 拉起四条 loop |
| `scripts/install_saas_autostart.sh` | 安装 systemd |

### ① 录制(2)
`src/go2_fastlio_patrol/go2_fastlio_patrol/route_recorder.py` · `scripts/route_recording_blackbox.py`

### ② 定位探针(2)
`scripts/check_livox_stream.py` · `scripts/check_fastlio_freshness.py`

### ③ 扶正(2)
`scripts/horizontal_frame.py` · `scripts/body_yaw_alignment.py`

### ④ 路线对齐(4)
| 文件 | 说明 |
|---|---|
| `scripts/manual_route_anchor.py` | **默认路径**(saas :2321) |
| `scripts/build_horizontal_route.py` | 产出 `route_horizontal.csv` |
| `scripts/check_route_start_alignment.py` | 验收闸 —— ⚠️ **结构上只可能通过**,见 12 篇 ④ |
| `src/go2_map_manager/src/route_relocalizer.cpp` | PCD/ICP 路径(saas :2391),**默认不走** |

### ⑤ 算法(1)
`src/go2_fastlio_patrol/go2_fastlio_patrol/waypoint_follower_go2_2.py` —— 331 行

### ⑥ 外壳(1)
`scripts/waypoint_follower_go2_2_trace.py` —— 1259 行,**实际被启动的就是它**(saas :2065)

### ⑦ 安全(2)
`unitree_safe_cmd_node.py` · `patrol_control.py`
⚠️ 后者被前者 `:13` import 三个函数,**是运动链核心,不是工具库**

### ⑧ 搬运 + 刹车(3)
`cmd_vel_udp_sender.cpp` · `go2_sdk2_udp_receiver.cpp` · `go2_sdk2_motion_probe.cpp`
第三个是**绕过 ROS 的独立刹车**,3 处调用(saas :2960 / guard :80 / watchdog :50)

### ⑨⑩ 编排 + 云端(1)
`scripts/go2_saas_agent.py` —— 3,634 行,**一个文件六件事**

### ⑪ 观测(3)
`go2_experiment_audit.py` · `go2_experiment_telemetry.py` · `patrol_performance_monitor.py`

### ⑫ 监督(2)
`localization_session_guard.py`(巡检期临时) · `go2_base_health_watchdog.py`(常驻)

### ⑬ 证据(1)
`go2_experiment_snapshot.py`

### 分支2 · 相机(9)
`go2_camera_capture.sh` · `go2_camera_preset.sh` · `go2_camera_upload_segment.sh` ·
`build_go2_builtin_camera_capture.sh` · `cpp_tools/go2_builtin_camera_capture.cpp` ·
`z1pro_capture.sh` · `z1pro_gcu_control.py` · `z1pro_preset.sh` · `z1pro_upload_segment.sh`

⚠️ 狗上实跑 `unitree_builtin`,不是 z1pro(`config/camera.env`)。
**z1pro 四件按用户要求保留**(相机待修)。

### 分支4 · 救援(1)
`scripts/go2_wired_ssh_rescue.sh`

### 分支5 · 离线建图(12 可达)
`go2_loop_backend/` 中 **11 个注册为 ROS 可执行**(setup.py :23-33)+ `__init__.py`:
`keyframe_saver` · `offline_keyframe_extractor` · `build_raw_map` · `scan_context_detector` ·
`pose_graph_optimizer` · `dynamic_map_filter` · `sliding_window_static_filter` ·
`export_registered_cloud_map` · `level_pcd` · `pcd_to_nav2_map` · `pcd_to_nav2_map_fast`

**巡检期间完全不跑**,是离线手动工作流。

### 包声明(2)
`src/go2_fastlio_patrol/go2_fastlio_patrol/__init__.py` · `src/go2_loop_backend/go2_loop_backend/__init__.py`

### 本次整理新增(1)
`tools/check_integrity.py` —— 删文件后的完整性自检(6 项)

**小计**:7+2+2+2+4+1+1+2+3+1+3+2+1+9+1+12+2+1 = **56**

---

## 三、剩下的 8 个代码文件 + 10 个配置 —— **线上没有**

### A. 零引用(全仓库无人提及,含 systemd/deploy)

| 文件 | 证据 |
|---|---|
| `scripts/go2_lio_trace_recorder.py` | 全仓库 grep 无命中 |
| `go2_start_level_scan.sh` | 仅 `PROVENANCE.md` 提到 |
| `go2_loop_backend/filter_keyframes_front_fov.py` | 未注册可执行,零引用 |
| `go2_loop_backend/odom_to_tf_odom_level_2d.py` | 未注册可执行,零引用 |
| `config/nav2_params.yaml` | 零引用 |
| `config/go2_amcl.yaml` | 零引用 |
| `config/go2_amcl_base_footprint.yaml` | 零引用 |
| `config/go2_slam_toolbox.yaml` | 零引用 |
| `config/go2_slam_toolbox_1km.yaml` | 零引用 |
| `config/cyclonedds_go2_wired.xml` | 零引用;运行时由 `env_common.sh:33` 现生成到 `/tmp` |
| `config/cyclonedds_no_shm_eth0.xml` | 同上 |
| 顶层 `laser_mapping.yaml` | 零引用;`FAST_LIO/config/` 内有自己那份 |
| 顶层 `livox_lidar_publisher.yaml` | 零引用;驱动包内有自己那份 |

### B. 悬空链(只被"零引用者"引用 → 整串够不着)

```
go2_start_level_scan.sh        ← 自己零引用
   ├→ level_cloud_node.py
   ├→ odom_to_tf_map.py
   ├→ odom_to_tf_map_2d.py
   └→ odom_to_tf_map_level_2d.py
```
**5 个文件连成一串,一起吊在半空。**

### C. 编译了但无人运行
`src/go2_map_manager/src/submap_builder.cpp` —— 仅被 `CMakeLists.txt` 引用。

**小计**:代码 8 个(`go2_lio_trace_recorder` + `go2_start_level_scan` + loop_backend 6 + `submap_builder`)
+ 配置 10 个 = **18 个够不着**

**校验**:56(在线上)+ 8(代码够不着)= 64 ✅

---

## 四、这次对账挖出的三件事

### ① 那道"起点对齐检查"是自证的
`go2_saas_agent.py` 执行顺序:
```
:2772  manual_anchor 把路线转到狗身上  → 产出 route_runtime.csv (:2326 --output-route)
:2777  check_route_start_alignment --route-file route_runtime.csv  (:1956 route_arg)
```
**两处指向同一个文件。** 它量的是"manual_anchor 干成了没有",不是"狗摆得正不正"。
用户关心的起点 1° 误差,**这道闸在结构上挡不住**。

### ② nav2 / AMCL / slam_toolbox 五份配置零引用
前人把配置放好了,**没有一行代码读它们**。
这对第 3 步(想上 PCD/地图定位)是个重要事实:**不是"改改就能用",是"从来没接上过"**。

### ③ 全系统只有"停"这件事有冗余
- 两条看门狗:`localization_session_guard`(盯 FAST-LIO 会话)+ `go2_base_health_watchdog`(盯底座)
- 两把刹车:ROS 链发零 + `motion_probe stop` 绕过 ROS 直连 SDK

**其余全是单点**:定位只有 FAST-LIO;路线只有 manual_anchor;朝向锁完就冻结。

---

## 五、怎么用这份表

1. **删任何文件前**,先在本表查它的归位;不在表上说明本表过期,先更新表。
2. **删完跑** `tools/check_integrity.py`(6 项:Python import / setup.py 入口 /
   saas 脚本引用 / shell 引用 / CMakeLists 源文件 / 测试 import)。
3. **重构时**按本表的节点归组:哪一环的代码放哪一环,不跨环。
   `12_one_line_walkthrough.md` 末尾的代码量表标了四处该动的。

---

## 六、本表未覆盖

- **上游第三方 378 个文件**未逐个核(Livox-SDK2 / livox_ros_driver2 / FAST_LIO / unitree_api)。
  它们是外部依赖,不在重构范围。
- **103 个历史路线 CSV** 未逐份核哪些还需要。它们是数据不是代码,
  但 `routes/invalid/`(7)、`routes/quality/`(17)、`routes/raw/`(12)明显是过程产物。
- **盘上 24 个未跟踪 `.bak`** 未清理(git 已忽略,但占空间)。

# 16 · 离线路线工具、4G 双实现、以及已解的松散点

> 原则同 00。本篇收口:离线路线处理工具、两套 4G 恢复系统的重复、以及深读中解决的若干"留待坐实"。

## 核验状态(2026-07-25 本轮逐条核过磁盘源码)

- **可靠性**:整体高度可靠,绝大多数断言逐条对上真实源码。本轮已对下述 files_checked 全部仓库源码逐行核过(见文末台账)。
- **源标签约定**:【默认 code path:line】= 节点 declare/argparse 里的代码默认;【生产 saas path:line】= go2_saas_agent 实际 `-p` 启动串;【狗上 dog:证据】= 狗上真机抓包版直接核;【README对照】= 第三方/说明文档对照;【推断-未验】= 合理推断但无直接证据。
- **狗上对照状态**:本篇涉及的文件里 **只有 1 份有狗上真机对照**——`laserMapping.cpp`(第三节第 1 点,repo≠dog,sha 验)。**其余全部【无狗上对照】**(离线笔记本工具 + 仓库脚本/节点;狗上副本仅 laserMapping/lddc/lds/waypoint_follower_go2_2 四份),即"仓库源码 = 狗上运行版"这一点无法证实,下文逐处标注,**不默认等同狗上**。
- **两处硬修正**:
  1. `go2_4g_manager.py` 实为 **1594 行**(原文两处写 1595,off-by-one)【默认 orin_go2_fastlio_ws/scripts/go2_4g_manager.py:1594 末行 `raise SystemExit(main())`】。
  2. `level_cloud_node.py` 的 **12.3° 从不生效**:它只是节点 `declare_parameter` 代码默认,被其唯一启动器 `go2_start_level_scan.sh` 硬覆盖成 **13.0°**,且该节点**根本不在巡检运行链**里。原文以 12.3 立论、并暗示它是在用校正——已改写(详见第三节第 2 点)。
- **过时账目已更新**:`go2_experiment_audit.py` 已被 `docs/understanding/17_experiment_audit_analyzer.md` 覆盖;docs 总账由"00–16 + 90(18 篇)"更新为 **00–19 + 90(22 篇)**(17/18/19 为本篇写作后 git 6ec2382 起新增)。
- **若干加注的精度问题**(不影响结论):rescue_basement 裁 pcd 含 25 m 外扩;"同一套几何"仅对 build_bounded 成立;nvv4l2 硬编落点在 shell 而非 cpp;camera.env 当前生效后端是 unitree_builtin;linux-4g 型号 SIM7100 vs SIM7500/7600 需分清。

---

## 一、离线路线处理工具(`tools/`,笔记本上跑,非狗端)

> 狗上状态:本节四个工具**全部【无狗上对照】**——狗上副本不含这些离线脚本,仓库版 = 狗上版无法证实;下列行号均为仓库源码。

把原始录制 CSV 处理成干净巡检路线 + `.quality.json`(routes/quality/ 里那些):

- **`build_piecewise_route.py`**:动态规划分段(正交回归误差 + 段惩罚)→ 合并近平行段 → 线交点求顶点 → 转弯感知重采样 → 输出路线 + 质量报告【默认 tools/build_piecewise_route.py:109-138(segment_route DP + orthogonal_squared_error + penalty)/161-177(merge_nearly_parallel)/180-216(line_intersection + build_vertices)/230-266(resample_polyline 转弯感知)/296-331,407-446(write_route + write_json 报告)】。**只用 XY 几何,不用 body yaw**(四足会蟹行)【默认 :4-6 docstring "uses only x/y geometry … does not use body yaw … crab angle" + :419 `body_yaw_used_for_geometry: False`】。
- **`build_bounded_route.py`**:`importlib` 加载狗端 `route_quality.py`【默认 tools/build_bounded_route.py:17-34(spec_from_file_location 指向 route_quality.py 并 exec_module)/206(load_route_quality)】,用 `build_clean_route` + 严格偏差上限(RDP tol = max_deviation,默认 0.03)【默认 :222-231(build_clean_route,simplify_tolerance=args.max_deviation)/186(--max-deviation default 0.03)→ route_quality.py:1016(simplify_rdp tolerance=simplify_tolerance)/878-901(simplify_rdp 即 RDP)】,校验最大偏差 / 无反向尖峰,可出 matplotlib 预览图【默认 :238-243(violates deviation bound)/258-262(reverse spike)/136-177(write_preview,matplotlib Agg)】。
- **`build_original_go2_route.py`**(未逐行,同类离线路线构建器,重建 Go2_2 recorder 距离采样)【默认 tools/build_original_go2_route.py:2 / 43-54 / 229-242】。
- **`rescue_basement.py`**:一次性硬编码修复——在首个 >5 m 跳变处截断 basement 路线【默认 tools/rescue_basement.py:7-10(硬编码 IN/OUT 路径)/12(MAX_ROUTE_STEP_M=5.0)/33-40(step>5 即 break 截断)】+ 把 pcd 裁到**路线包围盒再外扩 25 m**(非紧贴 bbox)、z 限 [-50, 50]【默认 :123-128(bounds = route bbox ± PCD_MARGIN_M)/13(PCD_MARGIN_M=25.0)/14-15(PCD_MIN_Z=-50 / PCD_MAX_Z=50)/68-118(rescue_pcd 过滤)】。⚠️ 原文"裁到路线包围盒"漏了 25 m 余量,已补。
- **几何复用范围要限定**:只有 `build_bounded_route.py` 真正 `import` 狗端 `route_quality.py`(12/route 章那套),是它的离线入口【默认 tools/build_bounded_route.py:17-34,206-231】;而 `build_piecewise_route.py` **自带一套独立重写的几何**(SegmentCosts / fit_line / line_intersection / 自有 resample_polyline),**不 import route_quality**【默认 tools/build_piecewise_route.py:73-266,import 段无 route_quality】。⚠️ 原文"tools 与狗端 route_quality 是同一套几何"对 piecewise 属过度概括,已收窄为仅限 bounded。

---

## 二、⚠️ 4G 有两套完整恢复系统(重复)

> 狗上状态:两套实现均【无狗上对照】,行数 / 行为按仓库源码核。

| 实现 | 语言 | 安装脚本 | 模式 | 状态文件 |
|---|---|---|---|---|
| `go2_4g_manager.py` | Python **1594 行**(原文 1595,off-by-one)【默认 go2_4g_manager.py:1594】 | `install_go2_4g_manager.sh` | ECM(单一所有者)【默认 :2 docstring "Single-owner A7600C ECM connection manager"】 | **写 `/run/go2-4g-manager-state.json`**【默认 :181-182(GO2_4G_STATE_FILE 默认此路径)/834-874(set_state→tmp+os.replace 原子写);saas 读的硬编码路径与此默认一致,无生产覆盖】 |
| `go2_connectivity_watchdog.sh` | Shell 1053 行【默认 wc -l=1053】 | `install_connectivity_watchdog.sh`(被 ecm/ppp install 调用)【默认 install_a7600c_ppp_only.sh:133 + install_a7600c_ecm_only.sh:56-60】 | auto/ppp/ecm/nmcli/passive(默认 auto)【默认 go2_connectivity_watchdog.sh:13 `MODE=${GO2_4G_MODE:-auto}`】 | **不写**上面那个 json【默认 全文无 state.json 写入】 |

- 两者都干"保 4G 活 + 抗 USB/XHCI 崩溃",功能大面积重叠。**saas 心跳只读 Python 那套的 state**;若现场装的是 shell 那套,则 shell 不产该文件 → state=None、summary 无 `cellular` 键,saas 蜂窝遥测为空【生产 go2_saas_agent.py:938(读 /run/go2-4g-manager-state.json)/939-941(except→state=None)/944(仅 dict 才置 summary['cellular'])】。
- shell 版额外亮点:**从 `https://39.96.37.187/` 抓 HTTP `Date:` 响应头引导系统时钟**(走 HTTPS 请求、取其 Date 响应头,解决 Orin 无 RTC 的冷启动时间问题;仅当年份 < TIME_BOOTSTRAP_MIN_YEAR 才 `date -s`)【默认 go2_connectivity_watchdog.sh:9(TARGET_HOST 默认 39.96.37.187)/364-376(curl -skI https://.../ 取 date 头 → date -s)】;netdev-unregister 卡住检测 + 清理 dhclient/pppd/nmcli + **可选重启(默认关闭)**【默认 :154-169(pattern "unregister_netdevice: waiting for … to become free")/201,203,204-206(pkill dhclient/pppd + nmcli disconnect)/62,212-216(NETDEV_STUCK_REBOOT_AFTER 默认 0=禁用,>0 才 reboot)】。
- → 现场到底跑哪套,取决于装了哪个 install。**这是"乱"的又一处:同一功能两套实现、状态不互通。**

---

## 三、深读中已解决的"留待坐实"

> 狗上状态:第 1 点有狗上对照(repo≠dog,sha 验);第 2–6 点全部【无狗上对照】,按仓库源码 + 生产 `-p` 串核。

1. **`/dev/shm/go2_fastlio_latest_odom.txt` 谁写**(⭐本篇唯一有狗上真机对照项):被改过的 FAST-LIO `laserMapping.cpp`(**狗上 1395 行**),每帧 `pubOdomAftMapped->publish` 后原子写(`.tmp`→`std::rename`),内容为 `stamp x y z qx qy qz qw` 的 key=value 文本【狗上 dog: analysis/xunjian_20260725_shutdown_capture/previous_boot/remote_source/laserMapping.cpp:736(publish)/738-740(.tmp)/754(rename tmp→.txt)/744-751(键值);全文 1395 行,直接核狗上真跑版】。所有定位守卫(锚点 / 会话 / 新鲜度 / 对齐)都低开销读它【默认 grep 命中 scripts/manual_route_anchor.py(锚点)/localization_session_guard.py(会话)/check_fastlio_freshness.py(新鲜度)/check_route_start_alignment.py(对齐);route_relocalizer.cpp 不在读取列表】。laserMapping 还被改为打 `output_age_ms` 时序日志(= (host_epoch − lidar_end)×1000,累加 sum/max/count)【狗上 dog: 同文件 :759-765】。
   > ⚠️ **repo≠dog(sha 验)**:仓库 `orin_go2_fastlio_ws/src/FAST_LIO/src/laserMapping.cpp` 为 **1414 行**(sha 5fec8282…),狗上抓包版 **1395 行**(sha e4cd05cb…),排序后 173 行不同。上述断言核的是**狗上 1395 行版**,全部成立;仓库版与狗上版有差异,勿混。

2. **雷达装歪的两个"校正"值,以及巡检链其实用哪个**(⚠️ 原文此点误导已改写):
   - `level_pcd.py`:pitch 默认 **13°**【默认 level_pcd.py:99 `--pitch_deg default=13.0`】。
   - `level_cloud_node.py`:绕 Y 轴校正 `/cloud_registered_body`→`_level`,**节点代码默认 `pitch_deg=12.3`**【默认 level_cloud_node.py:19(`declare_parameter("pitch_deg", 12.3)`)/28-37(绕 Y 轴 R)/16-17(topics)】——**但 12.3 从不生效**:该节点唯一启动器 `go2_start_level_scan.sh` 总以 `-p pitch_deg:=13.0` 覆盖它【生产/覆盖 go2_start_level_scan.sh:5(`PITCH=${1:-13.0}`)/53(`-p pitch_deg:=${PITCH}`)】,与 level_pcd 的 13° 一致,故**实际用 13.0**。
   - 更关键:`level_cloud_node.py` **根本不在巡检运行链**——saas/launch 都不启动它、`setup.py` 也未注册其 console_script,它只被 `go2_start_level_scan.sh`(即第四节自列为"未读 dev/遗留工具"的 level_scan 建图脚本)拉起,输出 `_level` 也只被同脚本里的 `pointcloud_to_laserscan` 消费【默认 go2_start_level_scan.sh:53(拉起 level_cloud_node)/58(pointcloud_to_laserscan 消费 _level)】。所以 12.3 既非在用值、又非在用节点,原文两处误导已改。
   - 巡检**安全节点**用的是**未校正**的 `/cloud_registered_body`,ROI 的 z 范围已把装歪考虑进去:默认 topic `/cloud_registered_body`、roi_z 0.25–0.90【默认 unitree_safe_cmd_node.py:46(pointcloud_topic default '/cloud_registered_body')/63-64(roi_z 0.25/0.90)】;**生产 `-p` 覆盖**为 topic 仍 `/cloud_registered_body`、roi_z **0.30–0.90**、x 0.35–1.50、y −0.30..0.30【生产 go2_saas_agent.py:2055(-p pointcloud_topic:=/cloud_registered_body)/2059(-p roi_z_min:=0.30 -p roi_z_max:=0.90)】。**狗上实际生效以生产 `-p` 为准**(默认 x_max 1.20 / y −0.45..0.45 均被生产覆盖)。

3. **Go2 原生避障**:`enable_oa_only.py` 开 `ObstaclesAvoidClient().Init()` → `SwitchSet(True)`、并 `UseRemoteCommandFromApi(False)`【默认 enable_oa_only.py:20-25 / 34-35】——狗自带一套避障(Go2 原生 OA),**独立于**安全节点的点云 ROI 急停,两套并存【推断-未验:`SwitchSet(True)` 属实,"两套避障独立并存"为合理推断——Go2 原生 OA 与安全节点 ROI 急停确为不同机制】。

4. **相机双后端**:`camera.env` 的 `GO2_CAMERA_SOURCE` 可选 `z1pro`(云台,RTSP `192.168.144.108` + GCU 私有控制口 TCP **2332**)或 `unitree_builtin`(Go2 内置相机)【默认 config/camera.env(Supported values: unitree_builtin, z1pro;Z1PRO_RTSP_URL=rtsp://192.168.144.108/)+ z1pro_gcu_control.py:158(--port default 2332)+ CAMERA_Z1PRO_NOTES.md:100/132】。**当前实际置为 `unitree_builtin`(非 z1pro)**【默认 camera.env:3 `GO2_CAMERA_SOURCE=unitree_builtin`;⚠️【无狗上对照】狗上真实取值无法确认,仓库为 unitree_builtin】。unitree_builtin 后端:`go2_builtin_camera_capture` 只负责抓 MJPEG/JPEG【默认 cpp_tools/go2_builtin_camera_capture.cpp:25-93,152-163(fetch_jpeg/is_jpeg/stream_mjpeg,encoding=jpeg)】,而 **`nvv4l2` 硬编码其实在 shell 封装 `go2_camera_capture.sh` 的 gstreamer 管线里,不在 cpp**【默认 scripts/go2_camera_capture.sh:31(BUILTIN_BIN=go2_builtin_camera_capture)/129(nvv4l2decoder mjpeg=1)/132(nvv4l2h264enc)】。⚠️ 原文"→ nvv4l2 硬编"方向对,但落点是 shell 封装、不是 cpp,已点明。

5. **有线 SSH 保命**:`go2_wired_ssh_rescue.sh` 给有线网卡挂 3 个子网 IP——`192.168.123.x`(Go2)/ `192.168.1.5`(Livox 雷达)/ `192.168.144.100`(相机),4G/WiFi 全挂也能有线进狗【默认 go2_wired_ssh_rescue.sh:5(WIRED_IPS 默认三 CIDR '192.168.123.18/24 192.168.1.5/24 192.168.144.100/24')/51-75(逐 CIDR ip addr add);子网映射与 go2_network_recover 的 LIVOX_ADDR 192.168.1.5、Z1PRO 192.168.144.x 一致】。

6. **第三条网络路径**:`go2_network_recover.sh`(systemd timer,OnBootSec=20s / OnUnitActiveSec=30s,走 mmcli/nmcli 的 ModemManager 路径,偏 NetworkManager 托管 GSM 连接 go2-4g)——与上面两套 4G 又不同【默认 install_network_recover.sh:29-44(写 go2-network-recover.timer 并 enable)+ go2_network_recover.sh:142-150(mmcli -L/--enable ModemManager)/126-128,160-164(nmcli GSM 连接 go2-4g)】。**且与两套 4G 互斥**:装 go2_4g_manager 时会主动 disable/stop go2-network-recover.timer/service【默认 install_go2_4g_manager.sh:49-50/83-84】——佐证"同一功能多套实现、状态不互通"。

---

## 四、覆盖总账(本篇写作时点,现已部分过时)

> ⚠️ **时点说明**:本节是"本篇写作时"的覆盖账;git 6ec2382 之后仓库又加了 doc 17/18/19,下面的总数与"尚未逐行"清单已相应过时,阅读时以本注为准。

**已逐行读完**:整个巡检功能链、三套 follower / 三套 cmd 节点、course/patrol_control、安全节点、运动桥(cmd_vel_sender/sdk2_receiver/motion_probe)、录制(recorder + blackbox + route_quality)、建图(go2_loop_backend 全部 + submap + 现场 go2map_capture)、重定位(relocalizer + manual_anchor + session_guard)、SaaS agent(3152 行全)、4G(**Python 1594** + Shell 1053 两套全;~~原文 1595~~ 系 off-by-one)、诊断(telemetry/snapshot/performance/lio_trace)、操作台 server、相机链、开机自启动、狗上 laserMapping 的 shm 写入、离线路线工具。~~共 docs 00–16 + 90(18 篇)~~ → **现已为 docs 00–19 + 90(22 篇)**(17/18/19 为后加)。

**尚未逐行(本篇写作时的尾巴,均为 dev 工具 / 参考 / 测试,对"狗怎么跑"边际 ≈ 0)**:
- ~~`go2_experiment_audit.py`(80KB,停止时消费证据出审计报告)~~ → **已被 `docs/understanding/17_experiment_audit_analyzer.md` 覆盖**,不再算"未读"(体积核对:82737 字节 ≈ 80.8 KiB、2591 行)【默认 scripts/go2_experiment_audit.py】。
- 小探针 / 遗留:`go2_builtin_camera_capture.cpp` / `_probe.cpp`、`go2_front_video_stream_probe.cpp`、`oa_cpp_test.cpp`、`livox_timing_probe.cpp`、`free_iox_chunk_stub.c`、legacy `go2_cmd_bridge` 系列、`go2_a7600c_usb_monitor.sh`、`go2_start_level_scan.sh`(即上文 level_cloud_node 的唯一启动器)、`patrol_cli.disabled_before_cmd_rework.py`、`pcd_to_nav2_map_fast.py`、`odom_to_tf` 2D/level 变体、相机 / 预设小脚本。
- `test_*.py`(各包单测)。
- `linux-4g/` 厂商 NDIS 驱动源码(**第三方参考,非狗端自研**):cdc-wdm.c / qmi_wwan.c 属 **SIM7100 系列**,另有 **SIM7500/7600 系列**的 `sim7500_sim7600_wwan.c`(原文笼统写作 "SIM7100/7500 … wwan.c",型号应分清)【README对照 linux-4g/SIM7100 系列…/cdc-wdm.c、qmi_wwan.c;linux-4g/SIM7500_SIM7600…/sim7500_sim7600_wwan.c】。

---

## 核验台账(claim → 证据 file:line → 判定)

> 本轮已对磁盘仓库源码逐条核。判定:✅默认/生产/狗上/README = 核实成立(标源);❗更正 = 台账改了数值;⚠️收窄/过时 = 表述需限定或已 stale。狗上对照仅 laserMapping 一份,余皆【无狗上对照】。

| # | claim | 证据 file:line | 判定 |
|---|---|---|---|
| 1 | build_piecewise DP 分段→合并→交点→重采样→报告 | tools/build_piecewise_route.py:109-138,161-177,180-216,230-266,407-446 | ✅默认 |
| 2 | build_piecewise 只用 XY、不用 body yaw | 同上 :4-6,419 | ✅默认 |
| 3 | build_bounded importlib 加载 route_quality | tools/build_bounded_route.py:17-34,206 | ✅默认 |
| 4 | build_bounded build_clean_route + RDP tol=max_deviation(0.03) | 同上 :222-231,186 → route_quality.py:1016,878-901 | ✅默认 |
| 5 | build_bounded 校验偏差 / 反向尖峰 + matplotlib 预览 | 同上 :238-243,258-262,136-177 | ✅默认 |
| 6 | build_original_go2_route 同类离线构建器 | tools/build_original_go2_route.py:2,43-54,229-242 | ✅默认 |
| 7 | rescue_basement >5m 跳变截断 | tools/rescue_basement.py:7-10,12,33-40 | ✅默认 |
| 8 | rescue_basement 裁 pcd = 路线 bbox **+ 25m 余量** + z∈[-50,50] | 同上 :123-128,13(PCD_MARGIN_M=25.0),14-15 | ✅默认(原文漏 25m) |
| 9 | "同一套几何"**仅对 build_bounded**;piecewise 自带独立几何、不 import | bounded:17-34,206-231;piecewise:73-266 无 route_quality | ⚠️收窄 |
| 10 | go2_4g_manager Python 行数 | go2_4g_manager.py:1594(末行) | ❗更正 1595→1594 |
| 11 | go2_4g_manager = ECM 单一所有者 | 同上 :2 | ✅默认 |
| 12 | go2_4g_manager 写 /run/go2-4g-manager-state.json(原子) | 同上 :181-182,834-874 | ✅默认 |
| 13 | watchdog Shell 1053 行 | go2_connectivity_watchdog.sh(wc -l=1053) | ✅默认 |
| 14 | install_connectivity_watchdog 被 ecm/ppp install 调用 | install_a7600c_ppp_only.sh:133;_ecm_only.sh:56-60 | ✅默认 |
| 15 | watchdog 模式 auto/ppp/ecm/nmcli/passive(默认 auto) | go2_connectivity_watchdog.sh:13 | ✅默认 |
| 16 | watchdog 不写 state json | 全文无写入 | ✅默认 |
| 17 | saas 心跳只读 Python state;装 shell 则遥测空 | go2_saas_agent.py:938,939-941,944 | ✅生产 |
| 18 | shell 版 HTTP Date 头引导时钟(走 HTTPS 抓取) | go2_connectivity_watchdog.sh:9,364-376 | ✅默认 |
| 19 | shell 版 netdev 卡住检测 + 清理 + 可选重启(默认关) | 同上 :154-169,201-206,62,212-216 | ✅默认 |
| 20 | shm 由狗上 laserMapping(1395 行)每帧原子写 | 狗上 laserMapping.cpp:736,738-740,754;全文 1395 | ✅狗上 |
| 21 | shm 含 stamp x y z qx qy qz qw(key=value) | 狗上 同文件 :744-751 | ✅狗上 |
| 22 | 四类定位守卫都读该 shm | grep 命中 manual_route_anchor / session_guard / freshness / alignment | ✅默认 |
| 23 | laserMapping 打 output_age_ms 日志 | 狗上 同文件 :759-765 | ✅狗上 |
| 24 | level_cloud_node 绕 Y 轴校正 pitch:**默认 12.3 不生效 / 实际 13.0** | level_cloud_node.py:19(默认 12.3),28-37 / go2_start_level_scan.sh:5,53(-p 13.0) | ❗默认12.3 / 实际13.0 |
| 25 | level_pcd 默认 13° | level_pcd.py:99 | ✅默认 |
| 26 | 安全节点用未校正 /cloud_registered_body(默认 vs 生产 ROI) | unitree_safe_cmd_node.py:46,63-64 / go2_saas_agent.py:2055,2059 | ✅默认→生产 |
| 27 | enable_oa SwitchSet(True);两套避障并存 | enable_oa_only.py:20-25,34-35 | ✅默认 + 推断-未验 |
| 28 | camera.env 选 z1pro / unitree_builtin(**当前 = unitree_builtin**) | camera.env:2-3 | ✅默认 |
| 29 | z1pro GCU 私有口 TCP 2332 | z1pro_gcu_control.py:158;CAMERA_Z1PRO_NOTES.md:100/132 | ✅默认 |
| 30 | unitree_builtin cpp 抓 MJPEG;**nvv4l2 硬编在 shell** | go2_builtin_camera_capture.cpp:25-93,152-163 / go2_camera_capture.sh:31,129,132 | ✅默认(落点 shell) |
| 31 | go2_wired_ssh_rescue 挂 3 子网 IP(Go2/雷达/相机) | go2_wired_ssh_rescue.sh:5,51-75 | ✅默认 |
| 32 | go2_network_recover 第三条网络路径(与 4G 互斥) | install_network_recover.sh:29-44 / go2_network_recover.sh:142-150,126-128,160-164 / install_go2_4g_manager.sh:49-50,83-84 | ✅默认 |
| 33 | go2_experiment_audit 80KB(**现已被 doc17 覆盖**) | go2_experiment_audit.py(82737B,2591 行) | ✅默认(账目更新) |
| 34 | 行数总账 Py 1594 / Sh 1053 / saas 3152 / laserMapping 1395 | 各文件 wc | ❗Py 更正 1594 |
| 35 | docs 00–16 + 90(18 篇) | docs/understanding/ 现含 00–19 + 90(22 篇) | ⚠️过时→22 篇 |
| 36 | linux-4g 第三方 NDIS 驱动(**型号收窄**) | SIM7100:cdc-wdm.c/qmi_wwan.c;SIM7500/7600:sim7500_sim7600_wwan.c | ✅README(型号收窄) |

**狗上对照汇总**:第 20/21/23 三条 = ✅狗上(laserMapping 1395 行真机版,sha e4cd05cb…;仓库 src/FAST_LIO 版 1414 行 sha 5fec8282…,173 行不同,repo≠dog)。其余全部条目 = 仓库源码核实、**【无狗上对照】**(狗上运行版是否一致无法证实)。

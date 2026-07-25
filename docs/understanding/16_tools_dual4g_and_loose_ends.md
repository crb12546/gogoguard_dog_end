# 16 · 离线路线工具、4G 双实现、以及已解的松散点

> 原则同 00。本篇收口:离线路线处理工具、两套 4G 恢复系统的重复、以及深读中解决的若干"留待坐实"。

## 一、离线路线处理工具(`tools/`,笔记本上跑,非狗端)
把原始录制 CSV 处理成干净巡检路线 + `.quality.json`(routes/quality/ 里那些):
- **`build_piecewise_route.py`**:动态规划分段(正交回归误差 + 段惩罚)→ 合并近平行段 → 线交点求顶点 → 转弯感知重采样 → 输出路线 + 质量报告。**只用 XY 几何,不用 body yaw**(四足会蟹行)。
- **`build_bounded_route.py`**:`importlib` 加载狗端 `route_quality.py`,用 `build_clean_route` + 严格偏差上限(RDP tol=max_deviation),校验最大偏差/无反向尖峰,可出 matplotlib 预览图。
- **`build_original_go2_route.py`**(未逐行,同类)、**`rescue_basement.py`**:一次性硬编码修复(在首个 >5m 跳变处截断 basement 路线 + 把 pcd 裁到路线包围盒)。
- 与狗端 `route_quality.py`(12/route章)是同一套几何,tools 是离线入口。

## 二、⚠️ 4G 有两套完整恢复系统(重复)
| 实现 | 语言 | 安装脚本 | 模式 | 状态文件 |
|---|---|---|---|---|
| `go2_4g_manager.py` | Python 1595行 | `install_go2_4g_manager.sh` | ECM(单一所有者) | **写 `/run/go2-4g-manager-state.json`** |
| `go2_connectivity_watchdog.sh` | Shell 1053行 | `install_connectivity_watchdog.sh`(被 ecm/ppp install 调用) | auto/ppp/ecm/nmcli/passive | 不写上面那个 json |
- 两者都干"保 4G 活 + 抗 USB/XHCI 崩溃",功能大面积重叠。**saas 心跳只读 Python 那套的 state**;若现场装的是 shell 那套,则 saas 蜂窝遥测为空。
- shell 版额外亮点:**从 `https://39.96.37.187/` 的 HTTP `Date:` 头引导系统时钟**(解决 Orin 无 RTC 的冷启动时间问题);netdev-unregister 卡住检测 + 清理 dhclient/pppd/nmcli + 可选重启。
- → 现场到底跑哪套,取决于装了哪个 install。**这是"乱"的又一处:同一功能两套实现、状态不互通。**

## 三、深读中已解决的"留待坐实"
1. **`/dev/shm/go2_fastlio_latest_odom.txt` 谁写**:被改过的 FAST-LIO `laserMapping.cpp`(狗上 1395 行),在每帧 `pubOdomAftMapped->publish` 后原子写(`.tmp`→rename),含 `stamp x y z qx qy qz qw`。所有定位守卫(锚点/会话/新鲜度/对齐)都低开销读它。laserMapping 还被改为打 `output_age_ms` 时序日志。
2. **雷达装歪 ~12.3°**:`level_cloud_node.py`(绕 Y 轴 12.3° 校正 `/cloud_registered_body`→`_level`)、`level_pcd.py`(默认 13°)。安全节点用的是**未校正**的 `/cloud_registered_body`(ROI 的 z 范围已考虑)。
3. **Go2 原生避障**:`enable_oa_only.py` 开 `ObstaclesAvoidClient.SwitchSet(True)`——狗自带一套避障,**独立于**安全节点的点云 ROI 急停(两套避障并存)。
4. **相机双后端**:`camera.env` 的 `GO2_CAMERA_SOURCE` 选 `z1pro`(云台,RTSP 192.168.144.108 + GCU 2332)或 `unitree_builtin`(Go2 内置相机,`go2_builtin_camera_capture` 抓 MJPEG → nvv4l2 硬编)。
5. **有线 SSH 保命**:`go2_wired_ssh_rescue.sh` 给有线网卡挂 3 个子网 IP(Go2/雷达/相机),4G/WiFi 全挂也能有线进狗。
6. **第三条网络路径**:`go2_network_recover.sh`(systemd timer,mmcli/nmcli 的 ModemManager 路径)——与上面两套 4G 又不同,偏向 NetworkManager 托管。

## 四、覆盖总账(截至本篇)
**已逐行读完**:整个巡检功能链、三套 follower/三套 cmd 节点、course/patrol_control、安全节点、运动桥(cmd_vel_sender/sdk2_receiver/motion_probe)、录制(recorder + blackbox + route_quality)、建图(go2_loop_backend 全部 + submap + 现场 go2map_capture)、重定位(relocalizer + manual_anchor + session_guard)、SaaS agent(3152行全)、4G(Python 1595 + Shell 1053 两套全)、诊断(telemetry/snapshot/performance/lio_trace)、操作台 server、相机链、开机自启动、狗上 laserMapping 的 shm 写入、离线路线工具。共 **docs 00–16 + 90(18 篇)**。

**尚未逐行(最后的尾巴,均为 dev 工具/参考/测试,对"狗怎么跑"边际≈0)**:
- `go2_experiment_audit.py`(80KB,停止时消费证据出审计报告)
- 小探针/遗留:`go2_builtin_camera_capture.cpp`/`_probe.cpp`、`go2_front_video_stream_probe.cpp`、`oa_cpp_test.cpp`、`livox_timing_probe.cpp`、`free_iox_chunk_stub.c`、legacy `go2_cmd_bridge` 系列、`go2_a7600c_usb_monitor.sh`、`go2_start_level_scan.sh`、`patrol_cli.disabled_before_cmd_rework.py`、`pcd_to_nav2_map_fast.py`、`odom_to_tf` 2D/level 变体、相机/預設小脚本
- `test_*.py`(各包单测)
- `linux-4g/` 厂商 NDIS 驱动源码(SIM7100/7500 的 cdc-wdm.c/qmi_wwan.c/wwan.c —— **第三方参考**,非狗端自研)

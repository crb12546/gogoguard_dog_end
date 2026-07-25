# Go2 巡检全链路自动体检

- 生成时间：2026-07-25T14:49:20.940+08:00
- 运行目录：`/home/unitree/go2_fastlio_ws/patrol_logs/runs/20260725/xunjian-20260725-08`
- 证据流是否完整：是
- 完整体检材料是否齐全：否

## 关键结论

- [error] evidence_artifacts：full experiment artifacts are incomplete
- [ok] odometry_continuity：the canonical bag has no odometry gap above 350 ms
- [ok] fastlio_liveness：FAST-LIO logged neither a recovery event nor a permanent odometry lock during the patrol
- [ok] coordinate_transform：original-to-runtime CSV conversion is exactly reproducible within CSV precision
- [warning] route_tracking：FAST-LIO trajectory had localized large deviation from the runtime CSV
- [warning] follower_frame_usage：the follower had elevated odometry age or prolonged reuse of one odometry update
- [warning] recording_link：this patrol cannot be cryptographically linked to a route-recording blackbox
- [ok] code_identity：captured source/executable identities stayed unchanged during the run
- [error] robot_low_level_state：Unitree state reported a motor-lost count or a nonzero sport error code
- [error] sensor_body_alignment：FAST-LIO and body IMU relative orientation changed materially during the run
- [needs_external_measurement] physical_ground_truth：onboard logs cannot prove whether FAST-LIO's reported XY equals the dog's true floor position; use the fixed camera/floor-marker record for that final distinction

## 核心数值

- 坐标转换可精确复算：是
- 起步位置误差：0.018 m；起步朝向误差：0.111°
- 全程横向误差：中位 0.16 m，P95 0.276 m，最大 0.328 m
- 前 5m 横向误差：P95 0.271 m，最大 0.284 m
- 跟线器逐周期证据：14772 条控制决策、使用 7078 个里程计回调；同一里程计连续复用最多 7 个控制周期；里程计时间戳年龄 P95 392.4 ms
- 全速 rosbag 关键话题：齐全；各话题消息数={"/Odometry": 7308, "/api/sport/request": 15183, "/cmd_vel": 15163, "/lf/sportmodestate": 15260, "/livox/imu": 152471, "/patrol_cmd": 15077, "/tf": 7308}
- rosbag 里程计最大记录间隔：0.337 s
- FAST-LIO 活性事件：自动恢复 0 次，健康失败 0 次，永久锁死 0 次
- 安全层把非零命令改成零：0 次（配对命令 9297 条）
- 性能采样：797 条；最高温度 69.1°C；监控唤醒延迟 P95 4.4 ms
- Orin 功耗模式：`NVPM VERB: Config file: /etc/nvpmodel.conf / NVPM VERB: parsing done for /etc/nvpmodel.conf / NVPM VERB: Current mode: NV Power Mode: 25W / 3 / NVPM VERB: PARAM CPU_ONLINE: ARG CORE_0: PATH /sys/devices/system/cpu/cpu0/online: REAL_VAL: 1 CONF_VAL: 1 / NVPM VERB: PARAM CPU_ONLINE: ARG CORE_1: PATH /sys/devices/system/cpu/cpu1/online: REAL_VAL: 1 CONF_VAL: 1 / NVPM VERB: PARAM CPU_ONLINE: ARG CORE_2: PATH /sys/devices/system/cpu/cpu2/online: REAL_VAL: 1 CONF_VAL: 1 / NVPM VERB: PARAM CPU_ONLINE: ARG CORE_3: PATH /sys/devices/system/cpu/cpu3/online: REAL_VAL: 1 CONF_VAL: 1 / NVPM VERB: PARAM CPU_ONLINE: ARG CORE_4: PATH /sys/devices/system/cpu/cpu4/online: REAL_VAL: 1 CONF_VAL: 1 / NVPM VERB: PARAM CPU_ONLINE: ARG CORE_5: PATH /sys/devices/system/cpu/cpu5/online: REAL_VAL: 1 CONF_VAL: 1 / NVPM VERB: PARAM CPU_ONLINE: ARG CORE_6: PATH /sys/devices/system/cpu/cpu6/online: REAL_VAL: 1 CONF_VAL: 1 / NVPM VERB: PARAM CPU_ONLINE: ARG CORE_7: PATH /sys/devices/system/cpu/cpu7/online: REAL_VAL: 1 CONF_VAL: 1 / NVPM VERB: PARAM FBP_POWER_GATING: ARG FBP_PG_MASK: PATH /sys/devices/gpu.0/fbp_pg_mask: REAL_VAL: 2 CONF_VAL: 2 / NVPM VERB: PARAM TPC_POWER_GATING: ARG TPC_PG_MASK: PATH /sys/devices/gpu.0/tpc_pg_mask: REAL_VAL: 240 CONF_VAL: 240 / NVPM VERB: PARAM GPU_POWER_CONTROL_ENABLE: ARG GPU_PWR_CNTL_EN: PATH /sys/devices/gpu.0/power/control: REAL_VAL: auto CONF_VAL: on / NVPM VERB: PARAM CPU_A78_0: ARG MIN_FREQ: PATH /sys/devices/system/cpu/cpu0/cpufreq/scaling_min_freq: REAL_VAL: 729600 CONF_VAL: 729600 / NVPM VERB: PARAM CPU_A78_0: ARG MAX_FREQ: PATH /sys/devices/system/cpu/cpu0/cpufreq/scaling_max_freq: REAL_VAL: 1497600 CONF_VAL: 1497600 / NVPM VERB: PARAM CPU_A78_1: ARG MIN_FREQ: PATH /sys/devices/system/cpu/cpu4/cpufreq/scaling_min_freq: REAL_VAL: 729600 CONF_VAL: 729600 / NVPM VERB: PARAM CPU_A78_1: ARG MAX_FREQ: PATH /sys/devices/system/cpu/cpu4/cpufreq/scaling_max_freq: REAL_VAL: 1497600 CONF_VAL: 1497600 / NVPM VERB: PARAM GPU: ARG MIN_FREQ: PATH /sys/devices/17000000.ga10b/devfreq_dev/min_freq: REAL_VAL: 306000000 CONF_VAL: 0 / NVPM VERB: PARAM GPU: ARG MAX_FREQ: PATH /sys/devices/17000000.ga10b/devfreq_dev/max_freq: REAL_VAL: 408000000 CONF_VAL: 408000000 / NVPM VERB: PARAM GPU_POWER_CONTROL_DISABLE: ARG GPU_PWR_CNTL_DIS: PATH /sys/devices/gpu.0/power/control: REAL_VAL: auto CONF_VAL: auto / NVPM ERROR: Error opening /sys/kernel/nvpmodel_emc_cap/emc_iso_cap: 13 / NVPM ERROR: failed to read PARAM EMC: ARG MAX_FREQ: PATH /sys/kernel/nvpmodel_emc_cap/emc_iso_cap / NVPM VERB: PARAM DLA0_CORE: ARG MAX_FREQ: PATH /sys/devices/platform/13e40000.host1x/15880000.nvdla0/acm/clk_cap/dla0_core: REAL_VAL: 614400000 CONF_VAL: 614400000 / NVPM VERB: PARAM DLA1_CORE: ARG MAX_FREQ: PATH /sys/devices/platform/13e40000.host1x/158c0000.nvdla1/acm/clk_cap/dla1_core: REAL_VAL: 614400000 CONF_VAL: 614400000 / NVPM VERB: PARAM DLA0_FALCON: ARG MAX_FREQ: PATH /sys/devices/platform/13e40000.host1x/15880000.nvdla0/acm/clk_cap/dla0_falcon: REAL_VAL: 294400000 CONF_VAL: 294400000 / NVPM VERB: PARAM DLA1_FALCON: ARG MAX_FREQ: PATH /sys/devices/platform/13e40000.host1x/158c0000.nvdla1/acm/clk_cap/dla1_falcon: REAL_VAL: 294400000 CONF_VAL: 294400000 / NVPM VERB: PARAM PVA0_VPS: ARG MAX_FREQ: PATH /sys/devices/platform/13e40000.host1x/16000000.pva0/acm/clk_cap/pva0_vps: REAL_VAL: 512000000 CONF_VAL: 512000000 / NVPM VERB: PARAM PVA0_AXI: ARG MAX_FREQ: PATH /sys/devices/platform/13e40000.host1x/16000000.pva0/acm/clk_cap/pva0_cpu_axi: REAL_VAL: 358400000 CONF_VAL: 358400000`；在线 CPU：`0-7`；识别到 8 个 CPU 核配置
- FAST-LIO 多核证据：进程 CPU 峰值 104%，线程数最多 11，线程实际落到的 CPU=[0, 1, 2, 3, 4, 5, 6, 7]，允许 CPU=["0-7"]
- FAST-LIO 实际配置：外参在线估计=无，LiDAR→IMU 平移=[]，旋转矩阵=[]，LiDAR/IMU 队列深度=无/无
- Livox 配置外参：`{"pitch": 0.0, "roll": 0.0, "x": 0, "y": 0, "yaw": 0.0, "z": 0}`（注意：这是雷达/雷达内置 IMU 的配置，不能替代雷达相对狗身安装角的实测）
- FAST-LIO 与狗身 IMU 相对姿态：可比较；相对 R/P/Y 中位数=-0.89°/26.68°/6.93°；相对姿态变化 P95=32.17°/47.29°/20.29°
- 狗底层：电机最高温度 56°C；电池功率 P95 185.1W；Sport 非零错误 9960 条；motor lost 7020 条

## 解释边界

横向误差使用 FAST-LIO 自己报告的轨迹与运行时 CSV 比较。如果这个误差很小、但现场视频里狗明显偏离地面路线，问题就在定位真值而不是跟线器；这一步必须用固定机位视频或地面标记作为外部真值，任何纯车载日志都无法独立证明。

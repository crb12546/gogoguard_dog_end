# 09 · 实测基线数值

> ⚠️ **基准:2026-07-27 整机快照(删除阶段之前)。**
> 本篇描述的是**删代码之前**的狗端状态,内容在该基准下成立。
> 读时注意两点:
> 1. **有些组件现在已从仓库删除**(4G 治理整套、legacy 命令桥、22 个脚本等)——
>    见 `13_file_accounting.md` 与 `14_dog_verification.md`。文档没写错,是那个时间点确实有。
> 2. **`go2_saas_agent.py` 的行号已漂移**(3630 → 3633 行,删除阶段改过 4 处)。
>    换算:引用 `1–52` 不用调 · `53–480` **+3** · **`481–3184` +5**(绝大多数引用在此段) · `3186` 之后 +3。
>    其他文件行号不受影响。
> 已用已知答案自检:本文若写 `commands` 列表在 `:2590`,当前仓库实测在 `:2595`,差 5 ✓

> **第 4 步验证的标尺。** 改造后重跑一次,与本表逐项对比,就能知道有没有变坏。
> 全部数字来自狗自己算出并落盘的产物,**不是我的估算**。

**基准运行**:`patrol_logs/runs/20260726/xunjian-20260726-02`
2026-07-26 19:30:18 → 19:37:18(**420 秒**),路线 `xbf2.csv`,speed=0.5,loop=pingpong
该次所用 **36 个文件的 sha256 与快照逐字节一致**(台账第 28 块),故可作为当前代码的基线。

---

## 一、雷达安装与水平化 ★ 最根本的一组

| 量 | 实测值 | 出处 |
|---|---:|---|
| **`tilt_removed_deg`**(要旋掉的倾角) | **34.3240°** | `maps/console/xbf.leveling.json.proof` |
| 独立复算 `arccos(map_up.z)` | 34.3240° ✅ | `arccos(0.8258617)` |
| **机身↔传感器夹角** | **33.1010°** | 由 `q_sensor_from_body=[0.02284,-0.28391,0.00429,0.95857]` 复算 |
| 双 IMU 重力夹角 | 31.8734° / 31.9773° | `build_horizontal_route` 的 `measured_body_lidar_gravity_angle_deg`(xbf2/xbf9) |
| 录制轨迹平面拟合倾角 | 23.0120° / 24.3669° | `tilt_deg` —— **受路面起伏影响,不是安装角** |
| `map_up_before` | `[-0.5602, -0.0639, 0.8259]` | 旋正前的重力方向 |
| `map_up_after` | `[-4.16e-17, -1.73e-18, 1.0]` | 旋正后 |
| `residual_gravity_tilt_deg` | **0.0** | — |

**水平化门禁的实测余量**(门限 vs 实测):

| 门禁 | 门限 | 本次 | 另 3 次独立运行 |
|---|---:|---:|---|
| `source_disagreement_deg` | ≤ 3.0 | **0.9127** | 1.0021 / 0.5158 / 0.7027 |
| `spread_deg` | ≤ 1.5 | **0.7577** | 0.8281 / 0.7479 / 0.5952 |
| `sample_count` | ≥ 15 | 15 | 15 / 15 / 15 |
| `rejected_invalid/motion/source` | — | **0 / 0 / 0** | — |
| `quiet_calibration_samples` | ≥ 20 | 88 | — |

**数学自证**:
```
rotation_determinant             = 1.0
rotation_orthogonality_max_error = 4.85e-17
maximum_pair_distance_error_m    = 1.14e-13   (512 点对)
```

**倾斜危害的量化**(同一片 1,085,459 点):
| | z 最小 | z 最大 | z 跨度 |
|---|---:|---:|---:|
| 未旋正 | -5.99 | 181.16 | **187.15 m** |
| 旋正后 | -4.19 | 32.18 | **36.37 m** |

---

## 二、控制回路

| 量 | 实测 | 说明 |
|---|---:|---|
| 控制周期总数(**含启动互锁期**) | **6828** | trace `kind=control` = audit 的 `total_control_records_including_startup` |
| 控制周期数(**仅 active 期**) | **6588** | trace 中 `motion_enabled=true` = audit 的 `control_records` / `active_control_records` |
| ↑ 差值 = 启动互锁期周期数 | **240** | 该期发零速 Twist |
| odom 回调总数 | **3093** | trace 中不同的 `odom_callback_sequence` |
| odom 回调数(**active 期用到**) | **3025** | audit 的 `unique_odom_callback_sequences_used` |
| ↑ 差值 = 互锁期的 odom | **68** | 未被用于实际控制 |
| **每个定位被复用** | **2.21**(全程)/ **2.18**(active) | 6828/3093 · 6588/3025 |

> ⚠️ **对比时必须统一口径**。audit 报告正文显示的是 **active 期**(6588 / 3025),
> 而直接解析 trace 得到的是**全程**(6828 / 3093)。两者都对,差的是启动互锁那 240 个周期。
**注意口径**:audit 报告正文用 **active 期**(6588 条);直接解析 trace 全程则含互锁期(6828 条)。
我已用 active 口径独立重算,与 audit 逐项一致(见下表"复算"列),证明两侧算法等价。

| 量 | **active 口径**(audit) | 全程口径(解析 trace) | 复算 |
|---|---:|---:|---|
| `odom_source_stamp_age_ms` 中位 | **203.17 ms** | 204.71 ms | ✅ 203.168 |
| 同上 P95 | **306.13 ms** | 335.93 ms | ✅ 306.177 |
| 同上 最大 | **871.34 ms** | 921.29 ms | ✅ 871.343 |
| **`internal_cross_track_m`** 中位 | **0.0295 m** | — | ✅ 0.0295 |
| 同上 P95 / 最大 | **0.0610 / 0.1608 m** | — | ✅ 一致 |
| 同一 odom 最长连续复用 | **12 个周期**(中位 2,P95 3) | — | audit 报 `[warning] follower_frame_usage` |

> audit 阈值:`odom_stamp_age` P95 >500 ms = error,>250 ms = warning → 本次 306.13 落在 **warning**。

**audit 另记的 6 项(本基线补充,改造后同样应对比)**:

| 字段 | 实测 | 意义 |
|---|---:|---|
| **`odom_callback_sequence_step`** | 中位 1.0 / P95 1.0 / **最大 1.0** | **步长恒为 1 ⇒ 全程没有丢失任何 odom 回调** |
| **`odom_receive_to_control_ms`** | 中位 **53.65** / P95 **112.17** / 最大 **575.52** | 从收到 odom 到发出控制的端到端延迟 |
| `consumed_odom_source_stamp_gap_s` | 中位 0.0994 / P95 0.1987 / 最大 0.6932 | 被消费的 odom 时间戳间隔(≈10 Hz) |
| `control_compute_to_trace_ms` | 中位 0.5719 / P95 1.8595 / 最大 15.63 | 控制计算到落盘的耗时 |
| `selected_alpha_rad` | 中位 -0.0127 / P95 0.0594 / 最大 0.2246 | 航向角误差 |
| `startup_interlock_control_records` | **240** | 互锁期周期数(audit 自己就记了) |
| `control_records_reusing_an_odom_callback` | **3563** | 复用了上一个 odom 的控制周期数 |
| `trace_to_patrol_cmd_matched_records` | **6583** | 与 `/patrol_cmd` 对上的记录(比 6588 少 **5** 条) |
| `malformed_lines` / `clean_stop` | 0 / True | trace 文件无损坏,正常收尾 |

> **audit 自带的解读**(`interpretation` 字段原文):
> *Reusing one odometry sample for two 20 Hz control cycles is normal when FAST-LIO publishes at 10 Hz.
> Risk is indicated by large odom age or long consecutive reuse, not by dropping intermediate sensor frames by itself.*
> → 即:**复用本身不是问题,风险看的是 odom 年龄和连续复用长度。**
| `route_rotation_deg` | **-15.6700°** | 运行时路线对齐旋转 |
| 启动到放行 | **12.0 秒** | 标定 4.3s → 锚定 → odom 门禁 7.7s |
| 巡检总时长 | 341.4 秒 | trace monotonic 9657.67 → 9999.11 |
| 停止原因 | `signal_15`(SIGTERM) | 正常收尾 |

---

## 三、audit 的口径(与上表 cross_track **不同源**,见 `10_open_questions.md`)

| 量 | 实测 |
|---|---:|
| 起步位置误差 | **0.008 m** |
| 起步朝向误差 | **0.029°** |
| 前 5 m 横向误差 P95 / 最大 | **0.265 / 0.281 m** |
| **全程横向误差** 中位 / P95 / 最大 | **5.110 / 6.080 / 6.576 m** |
| 路线总长 | **577.0 m** |

> ⚠️ follower 自报 0.029 m 与 audit 报 5.110 m 相差 **176 倍**。两者评估对象不同,
> **不能混用**。详见 `10_open_questions.md` 第 1 条。

---

## 四、话题吞吐(rosbag 实录,420 秒)

> ⚠️ **重要订正**:此前用巡检总时长 **420 秒**(19:30:18→19:37:18)作分母,**是错的**。
> rosbag 实际起止为 **19:31:25.837 → 19:37:15.917 = 350.079 秒**
> (`rosbag/metadata.yaml` 的 `duration.nanoseconds`)——bag 比巡检**晚 67 秒启动**,
> 因为前面在跑 57 步启动流程。用 350.079 秒重算后,**各话题频率其实正常**:

| 话题 | 消息数 | **正确频率**(÷350.079) | 标称 | 旧错值(÷420) |
|---|---:|---:|---|---:|
| `/livox/imu` | 69926 | **199.74 Hz** | 200 Hz | ~~166.5~~ |
| `/lf/sportmodestate` | 7002 | **20.001 Hz** | 20 Hz | ~~16.7~~ |
| `/api/sport/request` | 6924 | **19.778 Hz** | 20 Hz | ~~16.5~~ |
| `/cmd_vel` | 6910 | **19.738 Hz** | 20 Hz | ~~16.5~~ |
| `/patrol_cmd` | 6830 | **19.510 Hz** | 20 Hz | ~~16.3~~ |
| `/Odometry` | 3157 | **9.018 Hz** | 10 Hz | ~~7.5~~ |
| `/tf` | 3157 | 9.018 Hz | — | ~~7.5~~ |

→ **此前"全链路掉频严重"的判断作废**。控制链稳定在 19.5–20.0 Hz,
  FAST-LIO 9.018 Hz(标称 10)略低但不构成严重问题。

| 其他 | 实测 |
|---|---:|
| rosbag 里程计**最大记录间隔** | **0.592 s**(audit 阈值 >0.50 = error) |
| 安全层把非零命令改成零 | **69 次**(配对命令 4148 条) |

---

## 五、平台与资源

| 量 | 实测 |
|---|---:|
| 性能采样条数 | 356(1 Hz) |
| **最高温度** | **64.6 °C** |
| 监控唤醒延迟 P95 | 5.4 ms |
| Orin 功耗模式 | **25 W / 模式 3**,CPU 在线 0-7 |
| CPU 频率范围 | 729.6 MHz – 1497.6 MHz |
| **FAST-LIO 进程 CPU 峰值** | **78 %** |
| FAST-LIO 最大线程数 | 11(落在全部 8 个核上) |
| 狗底层:电机最高温 | 45 °C |
| 狗底层:电池功率 P95 | 180.2 W |
| **Sport 非零错误** | **4504 条** |
| **motor lost** | **4272 条** |

> 后两项在 7002 条 `/lf/sportmodestate` 中占比超 60%,audit 判 `[error] robot_low_level_state`。
> **成因未确定**,见 `10_open_questions.md`。

---

## 六、FAST-LIO 与狗身 IMU 的相对姿态

| 量 | 实测 |
|---|---:|
| 相对 R/P/Y 中位数 | **25.93° / 8.96° / 2.94°** |
| 相对姿态**变化** P95 | **38.47° / 31.39° / 25.81°** |

> 刚体安装下该相对姿态应为常数。audit 判 `[error] sensor_body_alignment`
> (阈值:P95 >8.0° = error,>3.0° = warning)。

---

## 七、生产 FAST-LIO 参数(改造时的对照基准)

`install/fast_lio/share/fast_lio/config/go2_mid360s.yaml`,与 `src/` 版 sha 相同 `6b4eed4745b8d7cb266b`

| 参数 | 生产值 | 代码默认 |
|---|---|---|
| `extrinsic_est_en` | **false** | true |
| `path_en` | **false** | true |
| `map_en` / `effect_map_en` | false / false | — |
| `scan_publish_en` / `scan_bodyframe_pub_en` | true / true | — |
| `dense_publish_en` | false | — |
| `pcd_save_en` | **false** | false |
| `cube_side_length` | **1000.0** | (上游 200) |
| `det_range` | **100.0** | (上游 300) |
| `max_iteration` | **3** | (上游 4) |
| `point_filter_num` | 3 | — |
| `blind` | 0.5 m | — |
| `filter_size_surf` / `filter_size_map` | 0.5 / 0.5 | — |
| `lidar_qos_depth` / `imu_qos_depth` | **2 / 400** | — |
| `acc_cov` / `gyr_cov` | 0.1 / 0.1 | — |
| `b_acc_cov` / `b_gyr_cov` | 0.0001 / 0.0001 | — |
| `extrinsic_T` | `[-0.011, -0.02329, 0.04412]` | — |
| `extrinsic_R` | **单位阵** | — |

---

## 八、控制器参数(manifest 实录)

```
go2_2_k_yaw               = 0.900      go2_2_max_yaw_rate     = 0.450
go2_2_lookahead_distance  = 0.600      go2_2_search_window    = 6
go2_2_turn_in_place_angle = 1.000      go2_2_slow_down_angle  = 0.500
go2_2_stuck_time          = 3.000      go2_2_relocalize_distance = 1.500
go2_2_goal_distance       = 0.250      go2_2_reach_distance   = 0.400  ← 仅打印,不参与计算
command_vy                = 0.000      udp_sdk_vx_limit       = 0.500
```

---

## 九、录制基线(xbf9,2026-07-25 18:59:29 → 19:12:18)

| 量 | 实测 |
|---|---:|
| 录制时长 | 12 分 49 秒 |
| 路线点数 | 1278 |
| `min_distance` | 0.400 m |
| rosbag 大小 | **90 MB**(9 话题) |
| telemetry | 32 MB |
| performance 日志 | 3.1 MB |
| `status` | `complete_with_warnings` |
| `errors` | `["odom_record_gap_during_recording:0.399279s"]` |
| `fastlio_health` 三项 | 0 / 0 / 0 |

---

## 十、视频基线

| 量 | 实测 |
|---|---:|
| 7/26 视频段数 | 24(**100% 落在两次巡检窗口内**) |
| PCD 总数 | **7256**(keyframes 下 7173 + 聚合产物 83) |
| 单段长度 | 20 秒 |
| 相机源 | `unitree_builtin` |
| FPS / 码率 | 15 / 4,000,000 |
| 编码路径 | `nvv4l2h264enc`(NVIDIA 硬编码) |
| **历史总段数** | **682**(其中 `unitree_*` 461 · `z1pro_*` 218 · 隐藏临时残留 3) |
| **相机切换分界** | **7/22 → 7/23**:z1pro(7/21 29 段 + 7/22 189 段)→ unitree_builtin(7/23 起) |
| 未清理的 `.tmp.mp4` 残留 | **3 个,均 0 字节** —— `trap 'rm -f' EXIT INT TERM` 未执行 ⇒ 那 3 次被 SIGKILL 强杀 |

---

## 使用建议

改造后重跑一次巡检,**优先对比这 8 项**(最敏感、最能反映链路健康):

1. `tilt_removed_deg` —— 应仍在 33–35° 区间
2. `source_disagreement_deg` / `spread_deg` —— 应仍远小于 3.0 / 1.5
3. follower 自报 `cross_track` 中位 —— 基线 0.029 m
4. `odom_stamp_age` 中位与 P95 —— 基线 204.7 / 335.9 ms
5. 控制周期 : odom 回调之比 —— 基线 2.2
6. `/Odometry` 实测频率 —— 基线 7.5 Hz
7. 安全层触发次数 —— 基线 69 次 / 4148
8. 启动到放行耗时 —— 基线 12.0 秒

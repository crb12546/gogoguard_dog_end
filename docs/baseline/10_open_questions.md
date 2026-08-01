# 10 · 未解清单

> ⚠️ **基准:2026-07-27 整机快照(删除阶段之前)。**
> 本篇描述的是**删代码之前**的狗端状态,内容在该基准下成立。
> 读时注意两点:
> 1. **有些组件现在已从仓库删除**(4G 治理整套、legacy 命令桥、22 个脚本等)——
>    见 `13_file_accounting.md` 与 `14_dog_verification.md`。文档没写错,是那个时间点确实有。
> 2. **`go2_saas_agent.py` 的行号已漂移**(3630 → 3633 行,删除阶段改过 4 处)。
>    换算:引用 `1–52` 不用调 · `53–480` **+3** · **`481–3184` +5**(绝大多数引用在此段) · `3186` 之后 +3。
>    其他文件行号不受影响。
> 已用已知答案自检:本文若写 `commands` 列表在 `:2590`,当前仓库实测在 `:2595`,差 5 ✓

> **诚实划出边界。** 每条给:不确定什么 · 为什么无法确定 · 需要什么才能确定。
>
> 存在理由:第 3 步做方案时,如果把下面任何一条当成"已知",方案就会建在错误前提上。

---

## 一、无法用车载数据回答的(需要外部手段)

### ★ 1. 狗在地面上到底偏没偏 —— 两个口径差 176 倍

| 口径 | 数值 | 怎么算的 |
|---|---:|---|
| follower 自报 | **0.029 m**(中位) | 水平化后的 FAST-LIO 位姿 vs `route_horizontal.csv` |
| audit 报告 | **5.110 m**(中位) | FAST-LIO 原始轨迹 vs `route_runtime.csv` |

**为什么无法判定**:两者评估的是**不同坐标系里的不同路线**。
`route_runtime.csv` 与 `route_horizontal.csv` 逐点距离**中位 67.0 m**(34° 旋转的必然结果),
再叠加运行时 `route_rotation_deg = -15.67°` 的对齐旋转。

**狗自己也这么说**(`experiment_audit.md` 的"解释边界",原文):
> 横向误差使用 FAST-LIO 自己报告的轨迹与运行时 CSV 比较。
> 如果这个误差很小、但现场视频里狗明显偏离地面路线,问题就在定位真值而不是跟线器;
> **这一步必须用固定机位视频或地面标记作为外部真值,任何纯车载日志都无法独立证明。**

**要确定需要**:固定机位视频 / 地面标记 / 外部定位设备。

⚠️ **这条不解决,"跟线精度"相关的任何改造方案都缺少验收标准。**

---

### 2. 是不是同一条链上有两套评估对象(设计还是缺陷?)

manifest 明确记录:
```
follower_route = .../route_horizontal.csv     ← follower 实际跟的
runtime_route  = route_runtime.csv            ← audit 拿去算误差的
```
`check_route_start_alignment.py` 也评 `route_runtime.csv`。

**不确定**:这是有意的(比如 runtime 系被认为更"绝对"),还是水平化功能加入后
audit 侧未同步更新。**代码里没有任何注释说明。**

**要确定需要**:开发者本人确认,或找到相关提交记录。

---

## 二、有现象、无成因的

### ★ 3. Sport 非零错误 4504 条 / motor lost 4272 条

7 分钟内 7002 条 `/lf/sportmodestate` 中,**超过 60% 报错**。audit 判 `[error] robot_low_level_state`。

**不确定**:是宇树固件的常态噪声、是本项目指令方式引起、还是硬件问题。

**要确定需要**:①在**不跑巡检**的情况下采集同样时长的 `/lf/lowstate` 做对照;
②宇树对这两个字段的定义文档。

---

### ★ 4. FAST-LIO 与狗身 IMU 的相对姿态变化 P95 达 38.47°

刚体安装下,这个相对姿态**应该是常数**。实测中位 25.93°/8.96°/2.94°,但变化 P95 高达 38°/31°/26°。

**不确定**:是 FAST-LIO 姿态估计在漂、是狗身 IMU 数据质量问题、还是两者时间戳配对方式导致的假象。

**要确定需要**:静止状态下长时间采集两者姿态,看相对量是否稳定。

---

### 5. `/Odometry` 实测只有 7.5 Hz(标称 10 Hz)

420 秒内 3157 条。同时 `/cmd_vel` 与 `/patrol_cmd` 也只有 16.5 Hz(标称 20 Hz)。

**不确定**:是驱动丢帧门禁(`points<5000` / `span` 越界)按预期丢掉了部分帧,
还是 CPU 不足导致处理不过来(FAST-LIO 峰值 78%)。

**要确定需要**:比对同一时段的 `LIVOX_STREAM_HEALTH` 的 `frame_hz` 与
`FAST_LIO_INPUT/OUTPUT_TIMING`,看丢在哪一层。**数据都在日志里,只是尚未做这项交叉分析。**


**第五轮新增线索**:快照显示狗上同时跑着**完整 GNOME 桌面**(gdm3 + gdm-x-session + pulseaudio +
tracker-miner-fs)、**Docker**(dockerd + containerd)与宇树升级服务(`/upgradePythonServer/server.py`,
监听 tcp:80)。对一台巡检机器人这些都非必需。结合 FAST-LIO 进程 CPU 峰值 **78%**,
**“后台资源占用导致掉频”成为一个新的候选解释** —— 但**尚未验证**。
要确定需要:停掉桌面与 Docker 后重跑一次,对比 `/Odometry` 频率与 CPU 峰值。

---

### 5b. ROI 斜切片:现有避障是"误停"还是"漏停"?(方案审查时新增)

`/cloud_registered_body` 实为雷达系(见 `06_invariants.md` II-6),ROI 盒因此是斜切片。

**不确定**:倾斜方向导致的是把地面误判为障碍(误停),还是让真实障碍逃出 ROI(漏停)。
实测 341 秒触发 69 次(1.66%),未见持续误报,但**这不能排除漏停**。

**要确定需要**:用 `runs/20260723/xunjian-20260723-01` 的 bag 离线回放,
对比"扶正后套 ROI"与"当前直接套 ROI"的 `would_stop` 帧数与触发位置。

⚠️ **这条影响避障改造的方向**:若是漏停,扶正后会**停得更多**,与"丝滑绕障"目标冲突,
需同步调整 `stop_distance` / `min_stop_points`。

---

### 6. `go2_base_health_watchdog` 日志为 0 字节

该节点在 `ros2 node list` 中活跃,但 run 目录内 `base_logs/go2-base-health-watchdog.log`
与 `base_logs/go2_saas_base.log` **都是 0 字节**(录制目录里也一样)。

**不确定**:是它本就只在异常时才输出、还是日志重定向配置有误。

---

### 7. 录制时 `session_guard` 的 `stop_results` 为 `not_running`

xbf9 录制的 manifest:
```json
{"performance":"graceful","rosbag":"graceful","route_recorder":"graceful",
 "session_guard":"not_running","telemetry":"graceful"}
```
它有 pid(531239)说明起来过,但停止时已不在,日志仅 4 KB。

**不确定**:是正常提前退出(任务完成)、还是异常中止。**无告警。**

---

## 三、代码存在但行为未被观测到的

### 8. 六处日志字符串契约不匹配 —— 断裂已确证,后果未观测

| 期待方 | 期待的串 | 实际产出 |
|---|---|---|
| audit / blackbox | `FAST_LIO_HEALTH.*FAILED` | `[FAST_LIO_HEALTH] rejecting Livox frame`(**无 FAILED**) |
| audit / blackbox | `odometry output is (?:now )?locked` | **全 src 无此串**;实际是 `FAST_LIO_RECOVERY: waiting for a scan with ...` |
| saas `:2739` | `StandUp ret=<非零>` | receiver `:91,93` **调用后不打印返回值** |
| audit | `TIMING_FOLLOWER` | 生产方在 `waypoint_follower.py`,**该文件未部署** |
| saas `:3379` | `gst-launch.*rtsp://192.168.144.108` | 内置相机管线是 `fdsrc fd=0`,**不含该地址** |

**第二轮核查新增第 6 处 —— 云端相机命令与当前视频机制完全脱节**:
```python
saas:3185  "camera_start_loop": test -x /tmp/z1pro_video_loop.sh && echo CAMERA_LOOP_ALREADY_CONFIGURED
                                                                  || echo CAMERA_LOOP_SCRIPT_MISSING
saas:3186  "camera_stop_loop":  rm -f /tmp/z1pro_video_loop.run;
                                pkill -TERM -f '[z]1pro_video_loop.sh'; echo CAMERA_LOOP_STOPPED
```
- `/tmp/z1pro_video_loop.sh` **全库无任何脚本生成它**(仅 saas `:481`/`:3185`/`:3186` 三处提及)
- 当前视频由 `go2-saas-video.service` + `patrol_video.active` 门控驱动,**与这两个命令无关**
→ **`camera_start_loop` 恒返回 `CAMERA_LOOP_SCRIPT_MISSING`;`camera_stop_loop` 的 pkill 恒无目标。**
→ 二者均在 `SAFE_COMMANDS` 白名单内,云端可随时下发,但**实际是空操作**。

**断裂本身是确证的**(字符串层面)。**但后果未被观测到**:
最后一次巡检的 `fast_lio.log` 里 `FAST_LIO_HEALTH` 与 `FAST_LIO_RECOVERY` 出现次数**均为 0** ——
也就是说这次**本来就没有故障发生**,报告写"健康失败 0 次"是**正确**的。

→ **不能用这次运行证明"它掩盖了真实故障"。**

**要确定需要**:构造一次真实的 Livox 异常帧场景,看 audit 报告是否仍显示 0。

---

### 9. 完整版控制器/录制为何未接线 —— 现象已知,日志中未定位

`waypoint_follower.py`(1425)+`patrol_control.py`(471,含横向纠偏)、
`route_quality.py`(1108,完整录制流水线)均写好且带单元测试,但未部署。

用户口述原因是"**跑起来非常不稳,不知道为什么**"。

**不确定**:不稳的具体形态(画龙?原地打转?某个弯道必崩?)在现有日志中**未能定位**。
该批文件的调参期(`.bak` 时间戳集中在 7/15–7/16)**早于**水平化统一(7/25),
但这只是时间相关性,**不构成因果证据**。

**要确定需要**:①用户回忆具体现象;②或在当前(已水平化)条件下原样接回跑一次实测。

⚠️ 这条直接影响第 3 步:**"把完整控制器接回来"是否可行,目前没有数据支撑,只有假设。**

---

## 四、尚未核实的代码

| 对象 | 规模 | 状态 |
|---|---:|---|
| ~~`cpp_tools/legacy_iox_stub/`~~ | — | ✅ **已于第二轮核查解答**:是 iceoryx 的两个空实现桩(`free_iox_chunk` / `iceoryx_header_from_chunk`),由 `build_legacy_iox_stub.sh` 用 `gcc -shared -fPIC` 编译,仅供 `start_legacy_go2_cmd_bridge.sh` 配合 `cyclonedds_no_shm_eth0.xml` 使用。**该 legacy 组 5 个脚本全部零引用**,详见 `08_component_inventory.md` D2 节 |
| `patrol_logs/runs/` 下 **89 次老格式记录**(`20260717_*`–`20260722_*`) | — | **未看**。那时 manifest 尚无 sha 字段,与当前程序差十余版本,对"现在跑什么"无证据力 |
| `laserMapping.cpp` 上游算法主体 | 约 1300 行 | **未逐行读**(已读团队补丁 8 处 + 结构骨架)。行为由生产 yaml 参数控制,参数值已全部取到 |
| `src/Livox-SDK2/` | 202 源文件 | **未读**。已 grep 穷尽验证:**零团队标记**,为上游原版 |
| `src/FAST_LIO/include/`(IKFoM_toolkit · ikd-Tree · so3_math.h · common_lib.h · use-ikfom.hpp · Exp_mat.h · matplotlibcpp.h) | — | 同上,**零团队标记** |

---

## 四之二、尚未读的快照目录(第五轮核查发现)

| 目录 | 文件数 | 状态 |
|---|---:|---|
| `current_task/` | 9 | 已于第五轮读完 → 产出 `11_prepared_not_deployed.md` |
| `inventory/` | **26** | ✅ **已于第五轮读完关键 12 份**(`identity` `final-live-state` `critical-sha256` `remote-local-critical-verification` `listening_ports` `services` `processes` `network` `rosbag-replay-candidates` `route-files` `video-files` `map-files`)。剩余 14 份为软件包/硬件清单(`dpkg-packages` 3342 行 · `apt-manual` 419 · `pip3-freeze` 193 · `hardware` `devices` `ros` `software_versions` `critical_binaries` `journal-current-boot` 等),对本交付结论无影响 |
| `development/` | 1 | 未读(`本地开发规则.md`) |
| `archives/` | 0 | 空 |

⚠️ **方法层面的教训**:此前把“通读整个仓库”等同于只读 `mirror/`,
漏掉了快照根目录下四个平级目录。其中 `current_task/` 装着一整套已评审的方案 ——
**若不补,第 3 步会在“没有下一代方案”的错误前提下从零设计。**

## 五、方法层面的已知局限

1. **本交付的运行证据主要来自 1 次巡检 + 1 次录制**
   (7/26 那次因 36 文件 sha 与快照一致而被选为基准)。
   其余 42 次新格式巡检记录**未逐一分析**,故"典型值 vs 偶发值"的区分不充分。

2. **快照是 2026-07-27 21:41 的静态镜像**,此后狗上的任何变动不在其中。
   用户已说明 7/26 之后曾改动程序并回滚 —— 回滚后的版本与本基线一致(sha 全量比对通过),
   但**若之后又有改动,本交付需重新校准**。

3. **我在分析与三轮回源核查中共犯过 21 个核验错误**(全部记录在 `tmp/linewise/LEDGER.md`),
   其中 5 个发生在横向通道扫描阶段、4 个发生在交付物回源核查阶段,全部为工具/正则层面,且均在下结论前被拦截。
   最危险的一个:用 basename 建索引选中了 `install/` 下的同名副本,**把 3 处正确的文档误报为错**。
   拦截手段:①先用已知答案自检表本身 ②逐个核实可疑项。
   **但这不能证明没有漏网的错误** —— 阅读本交付时,凡关键决策仍建议回到 file:line 复核。

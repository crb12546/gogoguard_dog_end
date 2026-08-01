# 07 · 设计意图记录 ★

> ⚠️ **基准:2026-07-27 整机快照(删除阶段之前)。**
> 本篇描述的是**删代码之前**的狗端状态,内容在该基准下成立。
> 读时注意两点:
> 1. **有些组件现在已从仓库删除**(4G 治理整套、legacy 命令桥、22 个脚本等)——
>    见 `13_file_accounting.md` 与 `14_dog_verification.md`。文档没写错,是那个时间点确实有。
> 2. **`go2_saas_agent.py` 的行号已漂移**(3630 → 3633 行,删除阶段改过 4 处)。
>    换算:引用 `1–52` 不用调 · `53–480` **+3** · **`481–3184` +5**(绝大多数引用在此段) · `3186` 之后 +3。
>    其他文件行号不受影响。
> 已用已知答案自检:本文若写 `commands` 列表在 `:2590`,当前仓库实测在 `:2595`,差 5 ✓

> **哪些"看起来像缺陷"其实是有意设计。**
>
> 这份文档的存在理由:第 3 步要列"现在的不合理之处"。
> 如果下面这些被误列进去改掉,会**把好东西改坏,而且改完不会立刻报错**。
>
> 我自己就差点犯过其中一条 —— 把横向速度的三道限幅当成了"参数悬空导致上限放大 5 倍"。

**分级**:
- **【确证】** 有代码注释、配套证据或运行数据直接支撑
- **【推断】** 逻辑上自洽且有间接证据,但无直接表述 —— 改动前建议先确认

---

## 1.【确证】`output_cmd_topic` 默认空串 —— 是模式开关,不是漏配

**看起来像**:一个输出话题名的默认值居然是空字符串,像是忘了填。

**实际是**:双模选择器。
```python
unitree_safe_cmd_node.py:46   declare_parameter('output_cmd_topic', '')
                        :151  if self.output_cmd_topic:          # 空串为假
                        :152      self.twist_pub = create_publisher(Twist, ...)

publish_move():
    if self.twist_pub is not None:  发 Twist → /cmd_vel → UDP 桥 → 宇树 SDK
    else:                           发 /api/sport/request(宇树原生,不经桥)
```
saas `:2454` 传 `-p output_cmd_topic:=/cmd_vel` → 走 UDP 桥。
**留空则退回宇树原生通道** —— 这是一条完整的备用链路。

---

## 2.【确证】录制期间不录视频 —— 用"默认关"代替"主动停"

**看起来像**:`route_recording_blackbox.py` 里 grep `video` = **0 处**,
像是忘了在录制时暂停视频。

**实际是**:视频默认就不录。`go2-saas-video.service` 常驻,但盯着门控文件:
```python
go2_saas_agent.py:3396  if managed_service_loop and not PATROL_VIDEO_ACTIVE_FILE.exists():
                  :3398      print("VIDEO_LOOP_IDLE waiting_for_patrol ...")   # 空转
```
录路线不是巡检 → 门控文件不存在 → 自然不录。**录制脚本无需做任何事。**

**运行证据**:7/26 共 24 段视频 100% 落在巡检窗口内;
xbf9 录制窗口(18:59:29–19:12:18)内 **0 段**;xbf8(14:23:27–14:29:21)结束后 6 分 45 秒才恢复。

> 用"默认关 + 巡检才开"替代"到处主动停",少写代码也少一类出错可能。
> PCD 生成同理(手动离线工具,不是巡检)。

---

## 3.【确证】`FOLLOWER_BODY_YAW_READY` 从未出现 —— 互斥分支没走那条

**看起来像**:saas 在 `grep -q 'FOLLOWER_BODY_YAW_READY'`,而 follower 日志里 **0 次**,
像是一个永远等不到的信号。

**实际是**:三选一分支(`:2505-2516`)
```python
follower_frame_ready_probe = (
    grep FOLLOWER_HORIZONTAL_FRAME_READY   if use_horizontal_frame
    else grep FOLLOWER_BODY_YAW_READY      if use_body_yaw_alignment
    else "true"
)
```
本次 `horizontal_frame_enabled=true` → 只走第一分支(实测打印 1 次 ✅)。
manifest 也记 `body_yaw_alignment_enabled=false`。**0 次是正确行为。**

---

## 4.【确证】`/Laser_map` `/cloud_effected` `/path` 有发布无订阅

**看起来像**:三个话题建了 publisher 却没人订阅,像是死代码。

**实际是**:生产 yaml 里三个开关全是 `false`,**运行时压根不发布**。
```yaml
publish:
  path_en: false        # 注释:Patrol does not consume /path; avoid growing message serialization.
  effect_map_en: false
  map_en: false         # 注释:Patrol does not consume the accumulated /Laser_map cloud.
```
> 代码保留发布口(上游 FAST-LIO 原有),配置关掉 —— **代码与配置是一致的**。

---

## 5.【确证】`current_patrol_run` 身份核对 —— 防止误停

**看起来像**:启动失败的收尾流程里多了一次文件读取和字符串比较,显得啰嗦。

**实际是**(`:3059-3070` 注释原文):
> *Only finalize the directory allocated by this exact attempt.
> A duplicate-start rejection must never stop an already running patrol.*

**场景**:巡检正在跑,有人又点了一次"开始巡检" → 被 `PATROL_ALREADY_RUNNING` 拒绝(exit 4)
→ 若无这道核对,收尾逻辑会把**正在跑的那次**给停掉。

---

## 6.【确证】run 目录用 `mkdir` 而非 `mkdir -p`

**看起来像**:没加 `-p`,像是疏忽。

**实际是**:利用"目录已存在则失败"的原子性防重入 → `PATROL_LOG_DIR_EXISTS` **exit 5**。

---

## 7.【确证】rosbag 用 `kill -INT`,其余用 `TERM`

**看起来像**:信号用得不统一。

**实际是**:`ros2 bag` 需要 SIGINT 才会正常收尾并 flush sqlite;
用 TERM 会留下损坏的 `.db3`。**证据链的完整性取决于此。**

---

## 8.【确证】失败回滚先删门控文件,再杀进程

**看起来像**:清理顺序随意。

**实际是**:先删 `.motion_enabled`,follower 在下一个 50 ms 周期自己发零速;
若先杀进程,可能留下"进程已死但最后一条非零指令仍在 UDP 路上"。

---

## 9.【确证】`origin` / `direct` / `none` 被强制当作 `manual_anchor`

**看起来像**:参数别名映射得莫名其妙,把三个不同的模式合并了。

**实际是**(`:609-611` 注释原文):
> *Historical `origin`/`direct` requests are intentionally treated as manual anchors.
> Comparing their old absolute CSV coordinates with a new FAST-LIO origin after a reboot
> **is not meaningful**.*

FAST-LIO 原点 = 进程启动位置,重启即变 → 录制时的绝对坐标失效。

**运行证据**:7/23–7/25 的 16 次 `mode=origin` **全部无 `manual_anchor.json`**;
7/25 之后 19 次 `manual_anchor` 有 17 次带该文件。**切换真实发生过。**

---

## 10.【确证】`extrinsic_R` 是单位阵 —— 正确,不是漏标定

**看起来像**:雷达明明歪装 34°,外参却是单位阵,像是没标定。

**实际是**:该外参描述**雷达 ↔ 雷达内置 IMU**(同一设备内),单位阵正确。
34° 是**雷达 ↔ 狗身**的安装角,由 `horizontal_frame` 在 FAST-LIO 输出之后处理。

audit `:1695-1699` 专门写了 `mounting_note` 提醒这个区别。

**这是本系统最好的一处架构决策** —— 第三方零改动,修正外挂。详见 `06_invariants.md` I-1。

---

## 11.【确证】`pcd_save_en: false` —— PCD 不由 FAST-LIO 产生

**看起来像**:关掉了 PCD 保存,那狗上 7000+ 个 PCD 哪来的?

**实际是**:PCD 全部由 `go2_loop_backend` 产生 —— 其中 **7173 个**来自关键帧法
(`keyframe_saver.py` 在线 / `offline_keyframe_extractor.py` 离线),另 **83 个**来自聚合与后处理模块
(`build_raw_map` / `export_registered_cloud_map` / `dynamic_map_filter` / `level_pcd` / `pcd_to_nav2_map` 等),
FAST-LIO 的 `save_to_pcd()`(`:1334-1336`)**永不执行**。

**运行证据**:`maps/` 下共 **7256** 个 `.pcd`,其中 **7173 个**位于 `*/keyframes*/`(关键帧法产物)。
另 **83 个**是聚合/后处理产物,分布为:
`console/` 43 + `console/raw/` 1(操作台水平化 PCD,路径 B 导出)· `extrinsic_est_false_check/level_sweep/` 9(手工 pitch 扫描)·
`loop_backend/` 9 · `campus_loop_backend/` 5 · `lab_small_001/` 4 · `extrinsic_est_false_check/` 4 ·
`campus_loop_backend/nav2_level_y12p3{,_fast}/` 3+3 · `lab_outdoor_param_001/` 2
→ **无论哪一类,都不是 FAST-LIO 的 `save_to_pcd()` 产出的。**

---

## 12.【确证】横向速度全链路为 0 —— 三道关卡一致设死

**看起来像**:`GO2_SDK_MAX_VY=0.020` 设了没人读,而 C++ 里硬编码 `±0.10f`,
像是"配置失效导致上限被放大 5 倍"。

**实际是**:上游两道都是 `max_vy:=0.000`,在 sender 就截成 0 了。
```
saas:2450  safe node  -p max_vy:=0.000
saas:2446  sender     -p max_vy:=0.000
cmd_vel_udp_sender.cpp:146  pkt.vy = sign × limit(linear.y, max_vy_)   ← 归零
manifest:  command_vy=0.000
```
`GO2_SDK_MAX_VY` 悬空属实,但**没有任何后果**。

> ⚠️ **我在初次分析时把这条误判成了缺陷**,并写进过给用户的总结。
> 教训:分析任何物理量必须走完整条关卡链,不能只看末端。见 `06_invariants.md` II-4。

---

## 13.【确证】观察者先于被观察者启动

**看起来像**:57 步里先起了 telemetry / performance_monitor 这些"不干活的",顺序奇怪。

**实际是**:保证运动链最初几秒有完整记录。
观察者(步 22/25,`:2661`/`:2695`)排在运动末端(步 28/31/32,`:2729`/`:2751`/`:2756`)之前。

---

## 14.【确证】录制的 rosbag 话题集与巡检不同

**看起来像**:两处话题列表不一致,像是维护时漏改了一处。

**实际是**:两种场景需求不同。
| | 独有话题 |
|---|---|
| 录制(9) | `/lf/lowstate` · `/lf/wirelesscontroller` · `/wirelesscontroller` · `/wirelesscontroller_unprocessed` |
| 巡检(8) | `/patrol_cmd` · `/cmd_vel` · `/api/sport/request` |

录制是**人用手柄遥控**,所以记手柄;巡检是自动,所以记控制指令。

---

## 15.【推断】`route_relocalizer` 的 `anchor_route_start` 强制起点对齐

**看起来像**:好不容易用 ICP 在地图里定位出来了,又把平移强行改掉,白算了。

**推断的意图**:把 ICP 的作用限定为"校朝向 + 验证狗确实摆在起点附近",
而不是"全局重定位"。`:945-955` 有阈值门,差太多直接 `RELOCALIZE_FAILED` 退出码 4。

```cpp
:956-957  t_old_from_current(0,3) += (route_start.x - mapped_current.x());
          t_old_from_current(1,3) += (route_start.y - mapped_current.y());
```
旋转保留,平移被拉回起点。

**为何标【推断】**:代码本身没有注释说明这个取舍。
**旁证**:该模式在 132 次巡检中**从未被使用过**,收益相对 manual_anchor 有限
(多一张 PCD 地图 + 多一组失败模式,只换来朝向校正)。

---

## 16.【推断】rosbag 不录点云

**看起来像**:最关键的传感器数据没进包,像是漏了。

**推断的意图**:性能取舍。`observer_policy` 明确写了
`raw_pointcloud_subscriber: false`,并给出替代方案
`pointcloud_evidence: "Livox and FAST-LIO in-process timing logs"`。

**代价是确实存在的**:黑盒无法离线重放点云匹配过程。
**为何标【推断】**:是有意配置(有 `observer_policy` 字段为证),
但"值不值得"这个权衡没有文字记录。

---

## 使用建议

第 3 步列"不合理之处"时,先对照本文件。
若某条改造建议触及以上任何一条,需要额外说明**为什么原设计不再适用**,
而不能仅以"看起来不合理"为由改动。

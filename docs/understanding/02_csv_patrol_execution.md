# 02 · 狗执行 CSV 巡检的控制逻辑(仓库版逻辑 · 狗上实跑为简版)

> 原则同 00。本篇涉及文件与**狗上状态**(源标签见下节缩写):
> - `src/go2_fastlio_patrol/go2_fastlio_patrol/waypoint_follower_go2_2.py`(仓库 1043 行 `WaypointFollowerGo22`,含课程反馈 + trace 埋点)——**仓库 ≠ 狗上(sha 验证)**:狗上实跑是 330 行简版 `WaypointFollower`(见「核验状态」)。
> - `scripts/waypoint_follower_go2_2_trace.py`(诊断外壳)——**【无狗上对照】**;狗 07-25 未走 trace。
> - `src/.../go2_course_control.py`(直线课程反馈几何)——**【无狗上对照】,且狗上不 import(未加载/未运行)**。
> - `scripts/go2_saas_agent.py`(拉起 follower 的 saas)——**【无狗上对照】**;其 manifest 模板与狗 manifest 字段集不一致(两版 saas)。
> - 控制频率 **20Hz**:仓库 `control_period=0.05`【默认 code go2_2.py:243 + create_timer:274】;狗版 `create_timer(0.05)`【狗上 dog:98】→ **20Hz 对狗成立**(但行号 243/274 仅属仓库)。

## 核验状态(本轮 2026-07-25 已对磁盘源码逐条核过)

**源标签缩写**:`code`/`go2_2.py`=仓库版 follower;`dog`=`analysis/xunjian_20260725_shutdown_capture/previous_boot/remote_source/waypoint_follower_go2_2.py`(狗上安装版副本);`saas`=`go2_saas_agent.py`;`course.py`=`go2_course_control.py`;`trace.py`=trace 外壳;`manifest`=`analysis/.../runs/xunjian-20260725-06/manifest.txt`。源标签格式:【默认 code path:line】/【生产 saas path:line】/【狗上 dog:证据】/【README对照】/【推断-未验】。

**最关键结论:仓库 ≠ 狗上(sha 验证)。**
- 仓库 `go2_2.py` sha `009cb25b`(1043 行 `class WaypointFollowerGo22`【go2_2.py:67】)
- 狗上副本 `dog` sha `d205a596`(330 行 `class WaypointFollower`【dog:26】)= manifest `controller_reference_sha256`【狗上 manifest:16】
- manifest 另写 `controller_executable=waypoint_follower_go2_2`(**非** trace 外壳)、`controller=go2_2_enhanced_nearest_lookahead`【狗上 manifest:14-15】。
- ⟹ 本篇原以仓库版为「控制真身」陈述,但**狗上真正跑的是简版**。下文凡标【仅仓库】者**未作用于狗**。

**狗上实跑 = 简版 · 差异清单(简版相对仓库缺什么)**:
1. **无 trace 外壳**:直接跑 `waypoint_follower_go2_2`【狗上 manifest:15】。
2. **无 QoSProfile**:订阅 `/Odometry` 用队列深度 10、**默认 reliable**【狗上 dog:90-95】;仓库才是 best_effort/keep_last/depth=1【go2_2.py:262-272】。
3. **无课程反馈整套**:不 import `go2_course_control`(grep `course/MotionCourse/closest_route` 0 命中)——无 `MotionCourseEstimator`、无 `closest_route_projection`、无 `course_alpha`/`control_source`,运动为**单一 body-yaw**【狗上 dog:277】。⟹ 第五节整章、第四节第 6 步、第 7 步直线子限幅**均不作用于狗**。
4. **odom 埋点少**:`odom_callback` 只取 x/y/yaw【狗上 dog:124-137】——无 `current_z`、无 `measured_speed`、无 odom 时间戳年龄、无 `course_estimator`。
5. **无转向来源选择**:恒 `alpha = body-yaw`,无 `course_straight/body_corner/body_fallback` 分支。
6. **无直线段更严限幅** `course_feedback_max_yaw_rate=0.35`。
7. **`yaw`/`v` 两列纯死码**:解析后【狗上 dog:113-116】全程不用,连 trace 都不写(狗不 trace)。
8. **主循环名不同**:狗为 `control_loop`【狗上 dog:255-313】,无独立 `control_cycle`;核心控制数学一致。
- 附:trace 外壳在**仓库**组合下会崩(`BASE_FOLLOWER_CLASS_MISSING`,见第一节),但狗上安装版恰有 `class WaypointFollower`【狗上 dog:26】,故若狗真跑 trace 也不会崩——但狗 07-25 本就没跑 trace。

**已核可信 / 仍无法验证**:
- ✅ 已核:所有 file:line、类名、控制数学、第七节生产参数字面量,在仓库源码逐条对上,且与狗 manifest 生产值逐条吻合;狗版行为已用 `dog`(330 行副本)逐条比对。
- ❓ 无狗上对照:`trace.py`、`course.py`、`saas` 三者**无狗上副本**——文档对其行级描述**仅对仓库源码成立**;狗上是否存在/一致未知(但 manifest 反证狗未走 trace、未加载 course、跑的是另一版 saas)。

---

## 一、入口与「真身」

- **【仅仓库 saas】** 仓库 saas 用 `python3 -u waypoint_follower_go2_2_trace.py` 拉起(执行档 = `follower_trace_script`)【生产 saas:2061-2073】。
- **⚠️ 狗上不同**:狗 07-25 `controller_executable=waypoint_follower_go2_2`(**直接跑简版,无 trace**)【狗上 manifest:15】。仓库 saas 的 manifest 模板写的是 `..._trace.py` + `trace_wrapper` 字段【saas:1813-1867】,**与狗 manifest 字段集不符 ⟹ 两版 saas 不同**。
- trace 外壳**只做观测**【仅仓库】:继承基类、原样调 `super().odom_callback()/control_loop()`【trace.py:130,216,228】,`RecordingPublisher` 原样转发 `publish`【trace.py:122-124】,把每次下发的 Twist + 位姿/最近点/目标点写 JSONL 轨迹。**不改任何控制数学**。
- **⚠️ 仓库一致性问题**:外壳取 `waypoint_follower_go2_2.WaypointFollower`【trace.py:25】,但仓库模块类名是 `WaypointFollowerGo22`【go2_2.py:67,全文件仅此一处、无 `WaypointFollower` 别名】→ `getattr(...,None) → raise RuntimeError`【trace.py:26-30】→ **按当前仓库代码启动即崩**(`BASE_FOLLOWER_CLASS_MISSING`)。注:狗上安装版有 `class WaypointFollower`【狗上 dog:26】,故此崩**仅对「仓库 trace + 仓库 module」组合成立**,「按当前仓库代码」限定语是准确的。
- 下面第二~五节以仓库 `go2_2.py` 为准描述**仓库版控制逻辑**;**狗上实跑简版**的差异见上「核验状态 · 差异清单」。

## 二、路线 CSV 格式与加载(仓库 `load_route`【go2_2.py:501-518】;狗 `load_route`【狗上 dog:107-122】,逻辑一致)

- CSV 表头必须含 **`x,y,yaw,v`** 四列(`DictReader`,逐行 `float(row['x'/'y'/'yaw'/'v'])`);缺列会 `KeyError` 崩【go2_2.py:505-511 / 狗上 dog:110-116】。
- 至少 2 个点,否则 `raise`【go2_2.py:514-517 / 狗上 dog:119-120】。
- ⚠️ **`yaw`/`v` 读入但控制里基本不用**:
  - 速度来自 `v_base/max_vx`,不是每点 `v`【go2_2.py:858-863】;
  - 朝向用**相邻点几何** `route_heading_at`,不是每点 `yaw`【go2_2.py:800】。
  - **更正(台账)**:原文「它们只被写进 trace 日志」对 `v` 不准确——
    - `v`:仅在解析处出现一次【go2_2.py:511 / 狗上 dog:116】,此后全代码零引用,**连 trace 都不写**(彻底死码)。
    - `yaw`:仓库里被写进 trace(`target_yaw`)【go2_2.py:438-439】,但**不进控制**;狗上连 trace 都没有 ⟹ **狗上 `yaw`/`v` 均纯死码**【狗上 dog:113-116 解析后不用】。

## 三、坐标与位姿(仓库 `process_odometry`【go2_2.py:542-590】)

- **⚠️ 本节大半为【仅仓库】埋点,未作用于狗**(狗 `odom_callback` 只取 x/y/yaw【狗上 dog:124-137】)。
- 订阅 `/Odometry`:
  - **【仅仓库】** QoS **best_effort / keep_last / depth=1**【go2_2.py:262-272】——只要最新一帧、丢帧无所谓、追求实时。
  - **狗上不同**:队列深度 **10、无 QoSProfile(默认 reliable)**【狗上 dog:90-95】,**不是** best_effort/keep_last/depth=1。
- 取 `current_x/y/z`、`current_yaw = yaw_from_quaternion(...)`【go2_2.py:544-547】—— **狗版无 `current_z`**【狗上 dog:125-127 仅 x/y/yaw】。
- **【仅仓库】** 记录 odom 时间戳年龄(新鲜度)【go2_2.py:549-559】、位置差分算 `measured_speed`【go2_2.py:561-567】、喂 `MotionCourseEstimator`(估实际行进方向,见五)【go2_2.py:572】—— **狗版三者皆无**。
- **首帧初始化**:`find_nearest_global()` 全局找最近路线点 → `nearest_index`,`target_index=nearest`【go2_2.py:579-584 / 狗一致 dog:129-134】。

## 四、控制主循环(仓库 `control_cycle`【go2_2.py:772-896,实际含日志延至 :922】;狗 `control_loop`【狗上 dog:255-313】)—— 逐步

每 50ms 一次:

1. **无位姿 / 已完成** → 发停(0,0)【go2_2.py:773-778 / 狗上 dog:256-262】。
2. **`update_nearest_index()`**【go2_2.py:633 / 狗一致 dog:171-193】更新「走到路线第几个点」:
   - 在 `nearest_index ± search_window` 窗口里找最近点【go2_2.py:612-631】;
   - **每周期最多前进 1 个索引**(按 `direction`),防抄近道/跳点(单调推进)【go2_2.py:636-646 / 狗上 dog:175-180】;
   - 窗口最近距离 > `relocalize_distance` → 全局重找最近点(脱轨重定位)【go2_2.py:648-655 / 狗上 dog:183-188】。
   - **`relocalize_distance`:默认 1.0**【默认 code go2_2.py:97 / 狗上 dog:50】**→ 生产 1.5**【生产 saas:2070 / 狗上 manifest:28】,**狗上生效 1.5**。
3. **`handle_goal()`**【go2_2.py:713-749 / 狗一致 dog:219-253】判到终点:到末端且距终点 ≤ `goal_distance`:`loop_mode=pingpong` → **掉头**(`direction` 翻转、`nearest` 置到另一端)继续;否则 `finished=True`、发停、结束。
   - **`goal_distance`:默认 0.5**【默认 code go2_2.py:88 / 狗上 dog:41】**→ 生产 0.25**【生产 saas:2066 / 狗上 manifest:23】,**狗上生效 0.25**。
4. **`compute_lookahead_index()`**【go2_2.py:664-682 / 狗一致 dog:195-213】纯跟踪前视点:从 `nearest_index` 沿 `direction` 累加段长,累计 ≥ `lookahead_distance` 的那个点作 `target_index`。
   - **`lookahead_distance`:默认 0.8**【默认 code go2_2.py:86 / 狗上 dog:39】**→ 生产 0.6**【生产 saas:2065 / 狗上 manifest:21】,**狗上生效 0.6**。
5. **朝向误差**:`target_angle = atan2(target_dy, target_dx)`;`body_alpha = normalize(target_angle − current_yaw)`(机身指向前视点的角误差)【go2_2.py:789-790】(狗版同式,命名为 `alpha`,因狗上只有这一个 alpha 源【狗上 dog:276-277】)。
6. **【仅仓库】选转向来源(直线课程反馈 vs 机身角,见五)**【go2_2.py:818-840】:
   - 直线段 → `alpha = course_alpha`(纠实际行进方向 + 横向漂移),`control_source='course_straight'`;
   - 转弯/课程未就绪 → `alpha = body_alpha`,`control_source='body_corner'/'body_fallback'`。
   - **⚠️ 狗上无此步**:无 `course_alpha`、无 `control_source`,**恒为单一 body-yaw**【狗上 dog:277】。此步是课程反馈子系统的一部分,狗上整体缺失。
7. **转向速度**:`yaw_rate = k_yaw · alpha`,限幅 `±max_yaw_rate`【go2_2.py:842 / 狗上 dog:279-280】。
   - **`k_yaw`:默认 0.6**【默认 code go2_2.py:83 / 狗上 dog:36】**→ 生产硬编码 0.900**【生产 saas:2064 / 狗上 manifest:19】,**狗上生效 0.9**。
   - **`max_yaw_rate`:默认 0.30**【默认 code go2_2.py:84 / 狗上 dog:37】**→ 生产 0.450**【生产 saas:2064 / 狗上 manifest:20】,**狗上生效 0.45**。
   - **【仅仓库】直线段更严**:取 `course_feedback_max_yaw_rate=0.35`【默认 code go2_2.py:105(未进 -p 串,故生产 = 默认 0.35),用于 :844-848】——**狗上不存在**。
   - **⚠️ 云端算了不用 / 被硬编码盖掉(专抓)**:saas 用 `bounded_float` 算了 `kYaw`(默认 **1.20**)【saas:1512】、`maxYawRate`(默认 **0.60**)【saas:1497】、`tracking_lookahead_distance`【saas:1517】及约 13 个 corner/course 参数,但 **follower `-p` 串把它们硬编码为 `k_yaw:=0.900` / `max_yaw_rate:=0.450` / `lookahead_distance:=0.600` 盖掉**【生产 saas:2064-2065】。其中 `k_yaw_arg`【saas:1684 定义后从不消费】、`tracking_lookahead_distance_arg`【saas:1685-1686】及那约 13 个 `*_arg` **全是死参**(grep 确认全文件仅出现一次);`max_yaw_rate_arg` 只流向 safe 节点与 cmd_vel 桥【saas:2049,2060】,**不流向 follower**。⟹ **这些云端参数对 follower 不可调、是死码**(勿以为 `kYaw` 能下发)。
8. **前进速度分三档**【go2_2.py:858-863】:
   - `|body_alpha| > turn_in_place_angle`(**1.0rad ≈ 57.3°**)→ **原地转** `vx=0`;
   - `speed_alpha > slow_down_angle`(**0.5rad ≈ 28.6°**)→ **减速** `vx=min(v_base·0.5, max_vx)`;
   - 否则 → **全速** `vx=min(v_base, max_vx)`。
   - `turn_in_place_angle=1.0`【默认 code go2_2.py:93 / 狗上 dog:46 / manifest:25,**生产 = 默认,非分歧**】、`slow_down_angle=0.5`【默认 code go2_2.py:94 / 狗上 dog:47 / manifest:26,**生产 = 默认**】。
   - 狗版两档均用 `abs(alpha)`【狗上 dog:283-288】,无 `body_alpha/speed_alpha` 之分,但课程恒不激活时数值等价。
9. **卡住恢复**【go2_2.py:865-874,`now=time.time()` 在 :865、判断在 :866;狗一致 dog:291-299】:`nearest_index` 超过 `stuck_time` 没推进 → 全局重定位。`stuck_time=3.0`【默认 code go2_2.py:96 / 狗上 dog:49 / manifest:27,**生产 = 默认**】。
10. **`publish_command(vx, yaw_rate)`** → `Twist(linear.x=vx, linear.y=0, angular.z=yaw_rate)` 发到 **`/patrol_cmd`**【go2_2.py:684-688;cmd_topic 默认 `/patrol_cmd` :74 / 狗上 dog:301-305,cmd_topic :31】。
    ⚠️ **`linear.y` 恒为 0** —— 跟随器从不主动横移,只有「前进 + 转」(manifest `command_vy=0.000` 印证【狗上 manifest:18】)。

## 五、直线课程反馈:为什么要它(`go2_course_control.py`)—— **【仅仓库,未作用于狗】**

> **⚠️ 全章仅仓库版**:`course.py` 仅被仓库 `go2_2.py` import【go2_2.py:20-26】;**狗上简版完全不 import**(grep `course/MotionCourse/closest_route` 0 命中)。狗上无 `MotionCourseEstimator`、无 `closest_route_projection`、无 `course_alpha`,运动为**单一 body-yaw**【狗上 dog:277】。**故「狗靠课程反馈治蟹行」对狗不成立**——这是本篇最关键的过度断言。以下描述与仓库 `course.py` 源码逐条相符,但**未运行于狗 07-25**;`course.py` 亦**【无狗上对照】副本**。

**问题**(docstring):四足狗转弯后会**「蟹行(crab / 侧滑)」**——机身 yaw 已「对准」,但实际路径仍与 CSV **平行偏移**。只看机身 yaw 纠不回来。

**做法**(仓库):
- `MotionCourseEstimator`【course.py:163】:锚点法,狗每移动 ≥0.12m 就用 `atan2(dy,dx)` 量一次**实际行进方向**【:168,190-192】,圆周平滑 0.6【:196-199】,0.8s 内有更新才 `valid`【:205-210】(参数来自 go2_2.py:100/102/101)。
- `closest_route_projection`【course.py:38】:把当前位姿投影到附近路段,给**带符号横向偏差** `signed_distance`(左正右负)【:48-49,78-81】和该段航向 `route_heading`【:75】。
- `use_straight_course_feedback`【course.py:146-160】:**仅当**开启 + 课程有效 + 前方路线转角小(≤ `max_route_turn 20°`,go2_2.py:103)+ 机身角误差小(≤ `turn_in_place_angle`)→ 判「直线段」。
- `straight_target_course`【course.py:126-143】:直线段上——横偏 ≤死区 0.03m(go2_2.py:99)就沿 CSV 切线走【:137-138】;超死区就朝前视点走但**限制切入角**(≤ `max_course 22°`,go2_2.py:104)【:139-143】,平滑纠偏。
- 直线段用「期望课程 − 实测课程」当 `alpha`,从而**纠正侧滑漂移**;转弯段回退机身 yaw 对准前视点。

## 六、一句话总结(狗到底怎么跑 CSV)

> **(仓库版)** 以 20Hz 循环:先在路线上定位「我到第几个点」(窗口最近点、单调前进、脱轨/卡住则全局重定位)→ 沿路线向前取一个「前视点」→ 算机身指向前视点的角误差 → **直线段**用「实际行进方向 vs 路线方向 + 横向偏差」精细纠偏(治蟹行),**转弯段**直接对准前视点 → 输出「前进速度 + 转向角速度」(角误差大就减速甚至原地转)→ 发 `/patrol_cmd`,交给安全节点限速/急停后驱动狗。全程只用 `/Odometry` 定位,**不主动横移**。
>
> **(狗上简版实跑)** 同样 20Hz、同样「窗口定位 / 单调前进 / 脱轨/卡住重定位 / 前视点 / 三档速度 / pingpong 掉头」,但**没有课程反馈那一层**:全程只算「机身指向前视点的角误差」这**单一 body-yaw**,不治蟹行、不主动横移【狗上 dog:277,279-280】。

## 七、生产实际参数(仓库 saas 启动串【生产 saas:2061-2079】;狗 manifest 逐条吻合【狗上 manifest:19-28】)

**⚠️ `v_base=max_vx=speed`**,`speed` 被 saas 夹到 ≤0.50【saas:1477-1483】,狗那次 `speed=0.5`【狗上 manifest:6】。`loop=<下发>`【saas:1641 取自 params,默认 once;狗那次 = pingpong,manifest:7】。

| 参数 | 代码默认【code】 | 生产【saas / manifest】 | 狗上生效 |
|---|---|---|---|
| v_base | 0.25(go2_2.py:81) | =speed(saas:2063) | **0.5** |
| max_vx | 0.25(go2_2.py:82) | =speed | **0.5** |
| k_yaw | 0.6(go2_2.py:83) | 0.900(saas:2064 / manifest:19) | **0.9** |
| max_yaw_rate | 0.30(go2_2.py:84) | 0.450(manifest:20) | **0.45** |
| lookahead | 0.8(go2_2.py:86) | 0.600(manifest:21) | **0.6** |
| goal | 0.5(go2_2.py:88) | 0.250(manifest:23) | **0.25** |
| relocalize | 1.0(go2_2.py:97) | 1.500(manifest:28) | **1.5** |
| search_window | 6(go2_2.py:91) | 6(manifest:24) | 6(= 默认) |
| turn_in_place | 1.0(go2_2.py:93) | 1.000(manifest:25) | 1.0(= 默认) |
| slow_down | 0.5(go2_2.py:94) | 0.500(manifest:26) | 0.5(= 默认) |
| stuck | 3.0(go2_2.py:96) | 3.000(manifest:27) | 3.0(= 默认) |
| loop_mode | once(go2_2.py:90) | <下发>(saas:1641) | pingpong |
| **reach** | 0.4(go2_2.py:87) | 0.400(manifest:22) | **死参(见下)** |

- **⚠️ `reach_distance` 生产设 0.400,但两版 follower 都是死参**——仅 declare/读/log,**从不进控制决策**【go2_2.py:87,125-126,292 / 狗上 dog:40,62,105】。
- **⚠️ 云端可配但对 follower 不生效**:`kYaw`(默认 1.20)/`maxYawRate`(默认 0.60)/`tracking_lookahead` 等云端 `bounded_float` 值被 `-p` 串硬编码盖掉(详见第四节第 7 步),对应 `*_arg` 是死参,**勿以为可下发**。
- 代码默认值更保守(如 `v_base=0.25`);**以启动串 / manifest 为准**。

## 八、留待坐实

- ✅(本轮已坐实)狗上部署版类名/逻辑与仓库不一致 —— 已拉 `dog`(remote_source)副本 sha 比对:狗 = 330 行 `WaypointFollower`,无课程反馈 / 无 trace / 无 QoSProfile(见「核验状态」)。差异远不止类名。
- `waypoint_follower.py`(测试版)与仓库 `waypoint_follower_go2_2.py`(生产版)控制差异逐条对比 → 见 03 或专章。
- `/patrol_cmd` 之后:安全节点如何限速/急停/双路输出 → 见 03、04。
- `trace.py` / `course.py` / `saas` 三者**无狗上对照副本**:狗上是否存在/与仓库是否一致,仍待拉狗上实体确认(现有 manifest 反证狗未走 trace、未加载 course、跑的是另一版 saas)。

## 核验台账(claim → 证据 file:line → 判定)

> 判定图例:**CONFIRMED**=对仓库准确(必要时注明是否对狗成立);**CORRECTED**=已更正;**DEFAULT_VS_PROD**=补生产值。狗侧文件状态:`go2_2.py` **仓库≠狗(sha 验)**;`dog`=狗上 330 行副本(=manifest sha);`trace.py`/`course.py`/`saas` 均**【无狗上对照】**。

- **20Hz**(control_period=0.05)→ go2_2.py:243,274 / 狗上 dog:98 → **CONFIRMED**(狗成立;行号 243/274 仅仓库)
- **saas 用 trace 外壳拉起** → saas:2061-2073 / 狗上 manifest:15 → **CORRECTED**(仓库走 trace;狗 `controller_executable=waypoint_follower_go2_2` 非 trace;两版 saas 不同)
- **trace 外壳只观测、不改数学** → trace.py:130,216,228,122-124 → **CONFIRMED**(仅仓库;狗未跑)
- **外壳取 `WaypointFollower` 而仓库模块是 `WaypointFollowerGo22` → 仓库启动崩** → trace.py:25-30 / go2_2.py:67 → **CONFIRMED**(仅「仓库 trace + 仓库 module」组合;狗上有 `class WaypointFollower` dog:26 不崩)
- **狗上部署版 ≠ 仓库** → sha 009cb25b(1043 行)≠ d205a596(330 行)= manifest:16 → **CONFIRMED**(差异远不止类名)
- **CSV 需 x,y,yaw,v 四列 / <2 点报错** → go2_2.py:501-518 / 狗上 dog:107-122 → **CONFIRMED**(逻辑一致)
- **yaw/v 不进控制、「只写 trace」** → go2_2.py:511,438-439,858-863,800 → **CORRECTED**(`v` 连 trace 都不写、纯死;仅 `yaw` 写 trace;狗上两者皆纯死码 dog:113-116)
- **odom QoS best_effort/keep_last/depth=1** → go2_2.py:262-272 / 狗上 dog:90-95 → **CORRECTED**(仅仓库;狗 depth=10、默认 reliable)
- **process_odometry 取 xyz / measured_speed / stamp-age / course_estimator / 首帧 nearest** → go2_2.py:542-590 / 狗上 dog:124-137 → **CONFIRMED**(仅仓库埋点;狗只取 x/y/yaw)
- **control_cycle 无位姿/完成发停** → go2_2.py:772-896(延至 :922)/ 狗上 dog:255-313 → **CONFIRMED**(行号近似,含日志到 922)
- **update_nearest_index 窗口 ±6 / 单调前进** → go2_2.py:633 / 狗上 dog:171-193 → **CONFIRMED**(search_window 生产 = 默认 6,非分歧)
- **relocalize > 1.5m 全局重找** → go2_2.py:97,648-655 / saas:2070 / manifest:28 → **DEFAULT_VS_PROD**(默认 1.0 → 生产 1.5,狗生效 1.5)
- **handle_goal ≤0.25m / pingpong 掉头** → go2_2.py:713-749 / 狗上 dog:219-253 → **DEFAULT_VS_PROD**(goal 默认 0.5 → 生产 0.25)
- **compute_lookahead ≥0.6m** → go2_2.py:664-682 / 狗上 dog:195-213 → **DEFAULT_VS_PROD**(lookahead 默认 0.8 → 生产 0.6)
- **body_alpha = normalize(target_angle − current_yaw)** → go2_2.py:789-790 / 狗上 dog:276-277 → **CONFIRMED**(狗命名为 alpha)
- **选转向来源 course_straight / body_corner** → go2_2.py:818-840 / 狗上 dog:277 → **CORRECTED**(仅仓库;狗恒 body-yaw 单源,无 course_alpha/control_source)
- **yaw_rate = k_yaw(0.9)·alpha 限幅 ±0.45** → go2_2.py:83-84,842 / saas:2064 / manifest:19-20 → **DEFAULT_VS_PROD**(默认 0.6/0.30 → 生产 0.9/0.45;云端 kYaw 1.20/maxYawRate 0.60 被 -p 盖掉、`*_arg` 死参 saas:1684-1686)
- **直线段更严 0.35** → go2_2.py:105,844-848 → **CONFIRMED**(节点默认,生产 = 默认;仅仓库)
- **三档速度 57°/29°** → go2_2.py:858-863,93,94 → **CONFIRMED**(1.0rad=57.3°/0.5rad=28.6°;turn_in_place/slow_down 生产 = 默认)
- **卡住恢复 stuck 3s** → go2_2.py:865-874(判断在 :866)/ 狗上 dog:291-299 → **CONFIRMED**(生产 = 默认 3.0)
- **publish → Twist(linear.y=0)→ /patrol_cmd** → go2_2.py:684-688,74 / 狗上 dog:301-305 / manifest:18 → **CONFIRMED**
- **第五节课程反馈是「生产控制」的一部分** → go2_2.py:20-26 import / 狗上 dog grep 0 命中 → **CORRECTED**(course.py 内容属实,但狗不 import、不运行 ⟹ 狗不靠它治蟹行)
- **MotionCourseEstimator 0.12m / 平滑 0.6 / 0.8s valid** → course.py:163,168,190-192,196-199,205-210 → **CONFIRMED**(仅仓库加载)
- **closest_route_projection 左正右负 signed_distance + route_heading** → course.py:38,48-49,78-81,75 → **CONFIRMED**(仅仓库)
- **use_straight_course_feedback 前方转角 ≤20°** → course.py:146-160 / go2_2.py:103 → **CONFIRMED**(仅仓库)
- **straight_target_course 死区 0.03m / 切入 ≤22°** → course.py:126-143 / go2_2.py:99,104 → **CONFIRMED**(仅仓库)
- **第七节生产参数逐条(-p 串 = manifest 生产值)** → saas:2061-2079 / manifest:19-28 / v_base 默认 0.25 :81 → **CONFIRMED**(speed 被夹 ≤0.50 saas:1477-1483;loop 取自 params saas:1641;reach 死参)

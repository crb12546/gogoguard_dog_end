# 02 · 狗执行 CSV 巡检的控制逻辑(生产跟随器)

> 原则同 00。核心文件:
> `scripts/waypoint_follower_go2_2_trace.py`(诊断外壳)、
> `src/go2_fastlio_patrol/go2_fastlio_patrol/waypoint_follower_go2_2.py`(**真正的控制逻辑**)、
> `src/.../go2_course_control.py`(直线课程反馈几何)。
> 控制频率 **20Hz**(`control_period=0.05`,`waypoint_follower_go2_2.py:243,274`)。

## 一、入口与"真身"
- 生产由 saas 用 `python3 -u waypoint_follower_go2_2_trace.py` 拉起(`go2_saas_agent.py:2061`)。
- trace 外壳**只做观测**:继承基类、原样调 `super().odom_callback()/control_loop()`,把每次下发的 Twist + 位姿/最近点/目标点写成 JSONL 轨迹(`trace_file`)。**不改任何控制数学**。
- ⚠️ **仓库一致性问题**:外壳取 `waypoint_follower_go2_2.WaypointFollower`(`trace.py:25`),但该模块类名是 `WaypointFollowerGo22`(`go2_2.py:67`),仓库里**无此别名** → 按当前仓库代码启动即崩(`BASE_FOLLOWER_CLASS_MISSING`)。→ 说明狗上部署版与仓库不一致,待用 `analysis/.../remote_source` 副本比对(见 90)。
- 下面逻辑以 `waypoint_follower_go2_2.py` 为准(它是控制真身,trace 只是壳)。

## 二、路线 CSV 格式与加载(`load_route`,`:501-518`)
- CSV 表头必须含 **`x,y,yaw,v`** 四列(`DictReader`,逐行 `float(row['x'/'y'/'yaw'/'v'])`);缺列会 `KeyError` 崩。
- 至少 2 个点,否则报错。
- ⚠️ **`yaw` 和 `v` 两列被读入但控制里基本不用**:
  - 速度来自参数 `v_base/max_vx`,不是每点的 `v`;
  - 朝向用**相邻点几何**算(`route_heading_at`),不是每点的 `yaw`。
  - (它们只被写进 trace 日志,不参与控制。)

## 三、坐标与位姿(`process_odometry`,`:542-590`)
- 订阅 `/Odometry`(QoS **best_effort / keep_last / depth=1**,`:262-272`)——只要**最新一帧**,丢帧无所谓,追求实时。
- 从消息取 `current_x/y/z`,`current_yaw = yaw_from_quaternion(...)`。
- 记录 odom 时间戳年龄(新鲜度)、用位置差分算**实测速度** `measured_speed`。
- 喂 `MotionCourseEstimator`(估计**实际行进方向**,见五)。
- **首帧初始化**:`find_nearest_global()` 全局找最近路线点 → `nearest_index`,`target_index=nearest`。

## 四、控制主循环(`control_cycle`,`:772-896`) —— 逐步

每 50ms 一次:

1. **无位姿 / 已完成** → 发停(0,0)。
2. **`update_nearest_index()`(`:633`)** 更新"我走到路线第几个点":
   - 在 `nearest_index ± search_window`(默认6)窗口里找最近点;
   - **每周期最多前进 1 个索引**(按 `direction`),防止抄近道/跳点(单调推进);
   - 若窗口最近距离 > `relocalize_distance`(生产1.5m)→ 全局重找最近点(脱轨重定位)。
3. **`handle_goal()`(`:713`)** 判到终点:
   - 到达末端且距终点 ≤ `goal_distance`(生产0.25m):
     - `loop_mode=pingpong` → **掉头**(`direction` 翻转、`nearest` 置到另一端),继续;
     - 否则 → `finished=True`,发停,结束。
4. **`compute_lookahead_index()`(`:664`)** 纯跟踪前视点:从 `nearest_index` 沿 `direction` 累加段长,累计 ≥ `lookahead_distance`(生产0.6m)的那个点作为 `target_index`。
5. **朝向误差**:`target_angle = atan2(target_dy, target_dx)`;`body_alpha = normalize(target_angle - current_yaw)`(机身指向前视点的角误差)。
6. **选转向来源(直线课程反馈 vs 机身角,见五)**:
   - 直线段 → `alpha = course_alpha`(纠正实际行进方向 + 横向漂移),`control_source='course_straight'`;
   - 转弯/课程未就绪 → `alpha = body_alpha`,`control_source='body_corner'/'body_fallback'`。
7. **转向速度**:`yaw_rate = k_yaw(0.9)·alpha`,限幅 `±max_yaw_rate`(生产0.45;直线段更严,取 `course_feedback_max_yaw_rate=0.35`)。
8. **前进速度分三档(`:858-863`)**:
   - `|body_alpha| > turn_in_place_angle`(1.0rad≈57°)→ **原地转**,`vx=0`;
   - `speed_alpha > slow_down_angle`(0.5rad≈29°)→ **减速** `vx=min(v_base·0.5, max_vx)`;
   - 否则 → **全速** `vx=min(v_base, max_vx)`。
9. **卡住恢复(`:865`)**:`nearest_index` 超过 `stuck_time`(3s)没推进 → 全局重定位。
10. **`publish_command(vx, yaw_rate)`** → `Twist(linear.x=vx, linear.y=0, angular.z=yaw_rate)` 发到 **`/patrol_cmd`**。
    ⚠️ **`linear.y` 恒为 0** —— 跟随器从不主动横移,只有"前进 + 转"。

## 五、直线课程反馈:为什么要它(`go2_course_control.py`)
**问题**(docstring `:2-11`):四足狗转弯后会**"蟹行"(crab / 侧滑)**——机身 yaw 已经"对准",但实际走的路径仍与 CSV **平行偏移**。只看机身 yaw 纠不回来。

**做法**:
- `MotionCourseEstimator`(`:163`):锚点法,狗每移动 ≥0.12m 就用 `atan2(dy,dx)` 量一次**实际行进方向**,圆周平滑(0.6),0.8s 内有更新才 `valid`。
- `closest_route_projection`(`:38`):把当前位姿投影到附近路段,给出**带符号横向偏差** `signed_distance`(左正右负)和该段航向 `route_heading`。
- `use_straight_course_feedback`(`:146`):**仅当**开启 + 课程有效 + 前方路线转角小(`≤max_route_turn 20°`)+ 机身角误差小(`≤turn_in_place_angle`)→ 判为"直线段"。
- `straight_target_course`(`:126`):直线段上——横偏在死区(0.03m)内就沿 CSV 切线走;超出死区就朝前视点走但**限制切入角**(`≤max_course 22°`),平滑纠偏。
- 直线段用"期望课程 − 实测课程"当 `alpha`,从而**纠正侧滑漂移**;转弯段回退到机身 yaw 对准前视点。

## 六、一句话总结(狗到底怎么跑 CSV)
> 以 20Hz 循环:先在路线上定位"我到第几个点"(窗口最近点、单调前进、脱轨/卡住则全局重定位)→ 沿路线向前取一个"前视点"→ 算机身指向前视点的角误差 → **直线段**用"实际行进方向 vs 路线方向 + 横向偏差"精细纠偏(治蟹行),**转弯段**直接对准前视点 → 输出"前进速度 + 转向角速度"(角误差大就减速甚至原地转)→ 发 `/patrol_cmd`,交给安全节点限速/急停后驱动狗。全程只用 `/Odometry` 定位,**不主动横移**。

## 七、生产实际参数(saas 启动串 `go2_saas_agent.py:2061-2078`)
`v_base=speed`、`max_vx=speed`、`k_yaw=0.9`、`max_yaw_rate=0.45`、`lookahead=0.6`、`reach=0.4`、`goal=0.25`、`loop_mode=<下发>`、`search_window=6`、`turn_in_place=1.0`、`slow_down=0.5`、`stuck=3.0`、`relocalize=1.5`。(代码默认值更保守,如 `v_base=0.25`;**以启动串为准**。)

## 八、留待坐实
- `waypoint_follower.py`(测试版)与 `waypoint_follower_go2_2.py`(生产版)控制差异逐条对比 → 见 03 或专章。
- 狗上部署版类名/逻辑是否与仓库一致(拉 `remote_source` 副本)。
- `/patrol_cmd` 之后:安全节点如何限速/急停/双路输出 → 见 03、04。

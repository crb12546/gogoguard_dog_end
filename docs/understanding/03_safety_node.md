# 03 · 安全节点:限速 + 雷达点云急停(`unitree_safe_cmd_node`)

> 原则同 00。核心文件:
> `src/go2_fastlio_patrol/go2_fastlio_patrol/unitree_safe_cmd_node.py`、
> 依赖 `patrol_control.py`(`limit_planar_command` / `point_in_lateral_motion_roi` / `stream_receive_age`)。
> 定时发布频率:生产 20Hz(`publish_rate`,默认 40Hz)。

## 一、它在链路里的位置
跟随器只管"往哪走",**安全节点是狗和运动执行之间的最后一道闸**:接住跟随器的速度,限幅、按雷达点云判急停、按流超时兜底,再输出给运动层。

- **输入**:`/patrol_cmd`(Twist,来自跟随器)+ `/cloud_registered_body`(PointCloud2,来自 FAST-LIO,QoS best_effort/depth=2)。
- **输出(二选一,`publish_move:250-259`)**:
  - `output_cmd_topic` 非空(**生产=`/cmd_vel`**)→ 发 **Twist** 到该话题(→ UDP 运动桥)。
  - `output_cmd_topic` 为空(**测试**)→ 发 **unitree_api `Request`**(Move,`api_id=1008`,parameter=JSON`{x,y,z}`)到 `/api/sport/request`。
  - ✅ **结论:生产只发 `/cmd_vel`,不发 sport API**;sport API 仅测试链用。总览"双发悬念"就此关闭。

## 二、接指令:限幅(`cmd_callback:208`)
每收到一条 `/patrol_cmd`,用 `limit_planar_command`(`patrol_control.py:420`)把 `vx/vy/yaw_rate` 各自 clamp 到 `±max_vx / ±max_vy / ±max_yaw_rate`,存为 `last_*` 并记 `last_cmd_time`。**注意:安全节点自己不算控制,只限幅+兜底透传。**

## 三、雷达点云判障(`process_cloud_message:302`)——回答"雷达怎么做急停"
- 处理限速到 `max_cloud_process_rate`(20Hz)。
- 直接解析 `PointCloud2` 原始字节(`struct.unpack` float32 x/y/z),**每 `point_skip`(生产2)个点取一个**降负载。
- **前方 ROI(盒子)**:`in_roi` 判点是否落在 `x∈[roi_x_min,roi_x_max], y∈[roi_y_min,roi_y_max], z∈[roi_z_min,roi_z_max]`(生产:x[0.35,1.50] y[-0.30,0.30] z[0.30,0.90],机体系,x 向前)。落框计 `roi_count`;其中 `x ≤ stop_distance`(生产0.80m)的计 `stop_count`;记 `nearest_x`。
- **侧向 ROI(`point_in_lateral_motion_roi`)**:⚠️ **正常巡检时是死的**——它开头 `if abs(vy)≤deadband(0.02): return False`,而跟随器永远发 `vy=0`,故侧向计数恒 0、从不触发。只有当有横移指令时才按 vy 符号选左/右侧检测。
- **判危险**:`unsafe = stop_count ≥ min_stop_points(生产15) 或 lateral_count ≥ lateral_min_stop_points(12)`。
- **去抖(关键)**:连续 `stop_frames`(生产1)帧危险 → 置 `obstacle_stop=True`(**1 帧就停,快**);置位后需连续 `clear_frames`(5)帧无危险 → 才解除(**恢复保守**)。

## 四、定时输出与兜底(`publish_safe_cycle:431`,每 1/publish_rate 秒)
按优先级,任一命中就输出 `Move(0,0,0)`:
1. **startup_interlock**:设了 `startup_enable_file` 且文件还没出现 → 停(开机联锁,等使能文件)。
2. **cmd_timeout**:距上次 `/patrol_cmd` > `cmd_timeout`(生产0.5s)→ 停(**跟随器挂了/卡了,狗自动停**)。
3. **cloud_timeout**:距上次点云 > `cloud_timeout`(生产1.0s)→ 停(**雷达/FAST-LIO 断流,狗自动停**)。
4. **cloud_recovery**:点云超时恢复后,需连续 `cloud_recovery_frames`(5)帧新鲜才放行,其间 → 停。
5. **obstacle**:`obstacle_stop=True` → 停。
否则透传限幅后的 `last_vx/vy/yaw_rate`。
> 流新鲜度用**本地接收时间**判(`stream_receive_age`),不信消息头 stamp(`:186-191` 明说 header stamp 仅诊断)。

## 五、安全哲学小结
- **正向**:只有前方盒子里近处点够多才停(点数阈值抗噪),停得快、恢复慢。
- **兜底**:指令断流、点云断流、开机未使能,一律零速——**故障默认停**。
- **盲区**:侧向避障事实上不生效(vy 恒 0);高度 ROI 下限 0.30m 会忽略很矮的障碍;盲区 0.5m(FAST-LIO blind)内无点。

## 六、`patrol_control.py` 的现状(顺带审计)
该文件是一大堆平面控制 helper(`line_follow_command`/`heading_drive_command`/`ordered_route_heading`/`corner_turn_angle`/`feedback_motion_scale`…),但:
- **生产跟随器**(`waypoint_follower_go2_2`)只 import `go2_course_control`,**完全不用** `patrol_control`。
- **安全节点**只用其中 3 个:`limit_planar_command`、`point_in_lateral_motion_roi`、`stream_receive_age`。
- 其余大量函数(整条 `line_follow_command` 直线跟踪方案)在生产路径上**无人调用** → 属于另一套/更早的跟随设计遗留,待 90 审计确认使用方(疑似 `waypoint_follower.py` 测试版或 `unitree_cmd_node`)。

## 七、留待坐实
- `/cmd_vel` 之后:`cmd_vel_udp_sender` → UDP → `go2_sdk2_udp_receiver` → SDK2 如何变成狗腿动作(见 04)。
- `patrol_control.line_follow_command` 等到底谁在用(见 90 / follower 对比)。

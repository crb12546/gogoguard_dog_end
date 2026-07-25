# 04 · 运动输出:Twist 怎么变成狗腿动作

## 核验状态
> 本轮(2026-07-25)已对**仓库磁盘源码逐条核对**,下文每个可证伪断言均落到真源码 `file:line`(见文末「核验台账」)。原文的架构/调用链/文件位置/子系统职责基本全部属实,本次只在台账标出问题处做保守修订,不重写、不删正确内容。
>
> **狗上对照缺口(必须明写,不许默认仓库==狗上)**:本文涉及的**全部运动输出源文件都不在狗上 4 份快照对照内 → 一律【无狗上对照】**。只能对仓库核,生产运行期真相靠 saas 装配串(`-p` 参数)+ 狗上 `manifest` **交叉印证**。
> - `cmd_vel_udp_sender.cpp` / `go2_sdk2_udp_receiver.cpp` / `go2_sdk2_motion_probe.cpp` / `go2_saas_agent.py` / `unitree_safe_cmd_node.py` / `patrol_control.py` / `unitree_cmd_node.py`:**【无狗上对照】**。
> - `waypoint_follower_go2_2.py`:**repo≠dog(sha 验)** —— 仓库 1043 行 `WaypointFollowerGo22`(sha `009cb25b…`)≠ 狗上实跑 330 行 `WaypointFollower`(sha `d205a596…` = manifest `controller_reference_sha256`)。两版控制周期均 20Hz、`linear.y=0`、发 `/patrol_cmd`,故本文数据流成立;但算法细节(尤其"课程纠偏")以狗版为准,归 03。
> - **真实 vy cap 无法确定**:生产给接收端注入 `GO2_SDK_MAX_VY=0.020`,而仓库 `go2_sdk2_udp_receiver.cpp` 全文无 `getenv`、从不读它(硬限幅写死 ±0.10)。狗跑仓库码则 cap=±0.10、该 env 是死的;狗跑读该 env 的更新二进制则 cap=±0.020、仓库已过期。无狗上对照,二者无法裁决(详见四·4)。
>
> 源标签约定:【默认 code file:line】=代码声明默认值 / 【生产 saas file:line】=生产装配串 `-p` 实际下发 / 【狗上 dog:证据】=狗上快照或 manifest / 【推断-未验】。**默认≠生产时两值都列,狗上生效以生产 `-p` / manifest 为准。**

> 原则同 00。核心文件(仓库根为 `orin_go2_fastlio_ws/`,下列为其内相对路径;原文写的 `src/…` 是 ROS 工作区相对写法,非错误,补根以免误导):
> `src/go2_cmd_vel_bridge/src/cmd_vel_udp_sender.cpp`(ROS2 节点)、
> `src/go2_cmd_vel_bridge/src/go2_sdk2_udp_receiver.cpp`(裸 C++ + 宇树 SDK2)。

## 一、两种运动输出后端(再确认)
| 后端 | 谁用 | 路径 | 落点 |
|---|---|---|---|
| **SDK2 SportClient**(生产) | 生产链 | 安全节点→`/cmd_vel`→`cmd_vel_udp_sender`→UDP→`go2_sdk2_udp_receiver`→`SportClient.Move()` | 直接调宇树 SDK2 C++ API |
| **sport API 话题**(测试) | 测试链 | 安全节点→`Request(api_id=1008)`→`/api/sport/request` | Go2 板载 ROS 桥接的运动服务 |

- **生产链**【CONFIRMED】:安全节点 `output_cmd_topic` 生产设为 `/cmd_vel`【生产 saas:2056】→走 Twist 分支→sender【生产 saas:2045-2049】→receiver【生产 saas:2022-2025】→`Move()`【receiver.cpp:219】。
- **测试链**【CONFIRMED,但生产不走】:**同一个 safe 节点**,`output_cmd_topic` 为空(默认 `''`)时才走 `Request(api_id=1008)`→`/api/sport/request`【默认 safe_node:48(默认空)、22(`SPORT_API_ID_MOVE=1008`)、164/258-259(空则发 Request)、238】。**生产设了 `/cmd_vel`,故这条测试路径在生产不产出**(算了不用:代码在,生产链走另一支)。
- 两者语义相同(都是 `Move(vx, vy, yaw_rate)`)【CONFIRMED:safe_node:242-246 JSON x/y/z api_id=1008 ↔ receiver.cpp:219 `Move(vx,vy,vyaw)`】,只是**传输方式不同**。生产选了 SDK2 直连。

## 二、发送端 `cmd_vel_udp_sender`(ROS2 → UDP)　【无狗上对照,仅对仓库核】
- 节点名 `cmd_vel_udp_sender`,订阅 **`/cmd_vel`**(Twist,来自安全节点)【CONFIRMED:sender.cpp:77(节点名)、103-104(sub `/cmd_vel` Twist)】。
- 每帧打成一个**二进制包 `CmdPacket`**(`#pragma pack(1)`)【CONFIRMED:sender.cpp:23-35】:
  - `magic="G2CM"(0x4732434d)`【sender.cpp:20,逐字节 = G2CM】、`version=2`【sender.cpp:21】、`packet_size`、自增 `sequence`【sender.cpp:143 `++sequence_`】、`send_steady_ns`/`send_system_ns`【sender.cpp:148-149】、`vx/vy/vyaw`(float32)【sender.cpp:32-34】。
  - ⚠️ 两个时间戳**并非并列"都用来测延迟"**(原文措辞不准):接收端只消费 `send_steady_ns` 算 transit / 端到端延迟【receiver.cpp:204-206,223-225】;`send_system_ns` 被打进包但**接收端从不读**(算了不用),仅是发送侧系统墙钟戳。
- 发送前限幅(**默认≠生产,狗上生效以生产 `-p` 为准**):
  - `vx≤max_vx`:【默认 sender.cpp:81 = 0.3】/【生产 saas:2048 `max_vx:=speed`】;生产 speed 实际 = **0.5**【生产 saas:1476-1483/1679:speed 默认 0.50 且 `min(speed,0.50)`;狗上 dog:manifest `speed=0.5` 佐证】。
  - `vy≤max_vy`:【默认 sender.cpp:82 = 0.10】/**【生产 saas:2048 `max_vy:=0.000` 字面硬编码,不随云端变】**。→ **生产发送端 vy 恒被限到 0**;叠加 safe 节点也设 `max_vy:=0.000`【生产 saas:2052】、上游 follower `linear.y=0` → **生产整条链 vy 全程 = 0**。原文只写默认 0.10、漏标生产覆盖,已补。
  - `vyaw≤max_vyaw`:【默认 sender.cpp:83 = 0.5】/【生产 saas:2048-2049 `max_vyaw:=max_yaw_rate`,云端不给则 = **0.600**】【生产 saas:1496-1501:`max_yaw_rate` 默认 0.60】。→ 生产此层阈值是 **0.600,不是代码默认 0.5**;不过上游 follower 已把 yaw 限到 0.45(见四·1),此层实战非绑定,但值须写对。
  - `vy` 乘 `unitree_vy_sign`(默认 +1,+Y=左)【CONFIRMED:sender.cpp:86 = 1.0、84-85 注释、146 应用】;生产未设该参→保持 +1,但因生产 vy=0 **实际无效**。
- `sendto` 到 **`target_ip:target_port`**【默认 sender.cpp:79-80 = `127.0.0.1:5005`】=【生产 saas:2047 `target_ip:=127.0.0.1 target_port:=5005`,与默认一致】—— **本机 localhost UDP**(进程间通信,不出网)。
- 每秒打 `TIMING_SENDER` 诊断(gap/callback/sendto 延迟、错误、序号)【CONFIRMED:sender.cpp:106-108(1s wall_timer)、181-195】。

## 三、接收端 `go2_sdk2_udp_receiver`(UDP → SDK2 → 腿)　【无狗上对照,仅对仓库核】
- **不是 ROS 节点**,是裸 `main`,直接用宇树 SDK2【CONFIRMED:receiver.cpp:70(`int main`)、16-17(SDK2 头)、80(SportClient)】:
  - `ChannelFactory::Instance()->Init(0, net_interface)`(原文写 `ChannelFactory::Init` 略简,真实为 `Instance()->Init`,语义同)【receiver.cpp:78】;net_interface【默认 receiver.cpp:72 = `eth0`、75 argv[1] 可覆盖】/【生产 saas:2024 传 `sdk_if_arg`;狗上 dog:manifest `sdk_if=eth0` 佐证】→ 连 Go2 内部 DDS。
  - `go2::SportClient`(运动控制客户端),`SetTimeout(10)`,`Init()`【CONFIRMED:receiver.cpp:80-82 `SetTimeout(10.0f)`】。
- **启动即让狗站起来**:`StandUp()` → sleep2 → `BalanceStand()` → sleep1【CONFIRMED:receiver.cpp:91-94】。⚠️ **狗的"起立/平衡"就发生在这个进程**,不是别处——全仓 grep `StandUp/BalanceStand` 仅 receiver(主链)与 motion_probe(诊断)命中;生产还监控其 ret 码,非零则 `exit 42`【生产 saas:2298-2300】,佐证这段载荷真实执行。
- 绑 UDP 端口 5005(`SO_RCVTIMEO=1s`,超时也醒一下好打印 cmd_age)【CONFIRMED:receiver.cpp:73(默认 5005,argv[2] 可覆盖)、105(bind)、115-120(`SO_RCVTIMEO tv_sec=1`);生产 saas:2024 显式传 5005】。
- 主循环:`recv` 一个包 → 校验 magic/version/size【receiver.cpp:193-198】→ **硬限幅** → 调 **`sport_client.Move(vx, vy, vyaw)`**【receiver.cpp:219,**这就是真正驱动狗腿的那一下**】。
  - 硬限幅**仓库源码**:`vx∈[-0.5,0.5]`、`vy∈[-0.10,0.10]`、`vyaw∈[-0.5,0.5]`【receiver.cpp:214-216】。
  - ⚠️ **但 vy cap 真值无法确定**【DOG_UNKNOWN】:生产给本进程注入 `GO2_SDK_MAX_VY=0.020`【生产 saas:2023】,**意图**把 vy 收紧到 ±0.020,而**仓库源码全文无 `getenv`、从不读这个 env**【receiver.cpp:215 `±0.10` 硬编码】。这是反向"算了不用/被覆盖":要么狗跑仓库码 → env 是死的、cap = ±0.10;要么狗跑读该 env 的更新二进制 → 仓库已过期、cap = ±0.020。**无狗上对照,不能把 ±0.10 当确定值**(详见四·4)。
- 统计序号丢包 `seq_gaps`、UDP 传输延迟、Move 调用耗时、端到端延迟(`sender_to_move_done_ms`)。
- ~~退出 `StopMove()`~~ **【CORRECTED — 原文误】**:`StopMove()` 位于 `while(true)` 主循环**之后**【receiver.cpp:241】,而该循环内**无 break/return**【receiver.cpp:191】→ **`StopMove()` 与其后的 `return 0`【receiver.cpp:243】都是不可达死代码**。进程实际靠**外部信号(SIGINT/SIGTERM/SIGKILL)**被终止,**永不执行 `StopMove`**。原文写成"正常退出动作"是误导。(生产清理走的是 motion_probe 的 stop,见六。)

## 四、限速是"层层设防"(defense in depth)
同一个速度被反复限幅,任何一层都能兜住。**逐层标默认/生产,狗上生效以生产为准**:
1. **跟随器** `max_vx/max_yaw_rate`【生产 saas:2063 `max_vx:=speed=0.5` / saas:2064 `max_yaw_rate:=0.450` 硬编码;狗上 dog:manifest `go2_2_max_yaw_rate=0.450` 佐证】。注:follower 的 `k_yaw/max_yaw_rate` 在 `-p` 里被硬编码 `0.900/0.450`,云端 bounded_float 算出的默认(`k_yaw 1.20` 等)被盖掉——典型"算了不用"(细节属 03)。
2. **安全节点** `limit_planar_command`(对称 clamp `max_vx/vy/yaw`)【CONFIRMED:safe_node:15-16(import)、208-227(cmd_callback 调用);patrol_control:420-435(定义)】。生产在此层 `max_vy:=0.000`【生产 saas:2052】。
3. **发送端** `max_vx/vy/vyaw`(见二):默认 0.3/0.10/0.5;生产 0.5/**0.000**/**0.600**。
4. **接收端硬编码最后一道**(写死在 C++)——**真值存疑,不是干净的 ±0.10**【DOG_UNKNOWN】:
   - 仓库源码 = `±0.5 / ±0.10 / ±0.5`【receiver.cpp:214-216】。
   - 生产却注入 `GO2_SDK_MAX_VY=0.020`【生产 saas:2023】而仓库从不读它【receiver.cpp 全文无 `getenv`】→ vy cap 实际是 ±0.10(跑仓库码)还是 ±0.020(跑读 env 的更新二进制)**无法裁决**【无狗上对照】。
   - 另:生产上游 vy 恒 0(见二),故这道 vy cap **实战根本不触发**。

## 五、端到端闭环(整条生产巡检驱动链,已全程逐行核实)
```
Livox雷达 ─► FAST-LIO ─┬─► /Odometry ───────────────► waypoint_follower_go2_2(纯跟踪, 20Hz)
                       │                                        │ Twist(vx,0,yaw) → /patrol_cmd
                       └─► /cloud_registered_body ─► unitree_safe_cmd_node(限幅+前方ROI急停+断流兜底, 生产20Hz/默认40Hz)
                                                              │ Twist(vx,vy,yaw) → /cmd_vel
                                                              ▼
                                                   cmd_vel_udp_sender ─UDP:127.0.0.1:5005─► go2_sdk2_udp_receiver
                                                                                                  │ SportClient.Move(vx,vy,vyaw)
                                                                                                  ▼
                                                                                             Go2 腿(先 StandUp+BalanceStand)
```
图注(修正原图两处标注):
- **follower 框**【repo≠dog,sha 验】:`20Hz / vy=0 / →/patrol_cmd` 均属实【CONFIRMED,狗上 dog:`remote_source/waypoint_follower_go2_2.py:98`(`create_timer(0.05)`=20Hz)、303(`linear.y=0.0`)、31(`/patrol_cmd`);manifest `command_vy=0.000`、`controller=go2_2_enhanced_nearest_lookahead`】。但原图**"课程纠偏"是仓库 1043 行 `WaypointFollowerGo22` 的特征**(`course_feedback_*`),**狗上实跑 330 行 `WaypointFollower`**(sha `d205a596…` = manifest `controller_reference_sha256`)是 nearest+lookahead 纯跟踪,**未见明显课程纠偏** → 已从图中删去"课程纠偏"字样,算法细节以狗版为准(归 03)。
- **safe 节点框**:`publish_rate`【默认 safe_node:53 = 40.0】/【生产 saas:2053 = 20.0】→ 图中 20Hz 是**生产值**,代码默认 40Hz(原图未标默认)。三机制(限幅 + 前方 ROI 急停 + 断流兜底)均在源码坐实【safe_node:281-286+358-362+398(ROI 急停)、467/471(cmd/cloud timeout 兜底);输入 `/cloud_registered_body`、输出 `/cmd_vel` 见 生产 saas:2055-2056】。

## 六、留待坐实
- `SportClient.Move` 的坐标/单位约定、Go2 sport 模式细节(宇树 SDK,第三方闭源,按需)【推断-未验】。
- `go2_sdk2_motion_probe.cpp`(诊断探针,**非主链**)何时用:【CONFIRMED,已坐实】独立 `main`【probe.cpp:39】、argv 驱动 `StandUp/Move/StopMove`【probe.cpp:117-140】;**生产主链不跑 probe,仅退出清理时调其 stop**【生产 saas:2485】,主链 receiver 用 `go2_sdk2_udp_receiver`【生产 saas:2024】。
- ~~生产是谁、在哪启动 `go2_sdk2_udp_receiver`(net_interface 传什么)~~ **【本轮已坐实】**:生产在 saas `start_patrol` 组 `sdk_receiver_cmd` 启动【生产 saas:2022-2025】,net_interface = `eth0`【生产 saas:1718(`sdk_if` 默认 eth0)、1721(`sdk_if_arg`);狗上 dog:manifest `sdk_if=eth0`】。(原文指向 08 SaaS agent 装配串,现直接坐实。)

## 核验台账
> 本轮对仓库磁盘源码逐条核对(claim → 证据 `file:line` → 判定)。判定:CONFIRMED=属实 / DEFAULT_VS_PROD=默认≠生产(两值都列)/ CORRECTED=原文错已更正 / DOG_UNKNOWN=无狗上对照无法裁决。**全部运动输出源文件均【无狗上对照】,follower 为 repo≠dog(sha 验)。** 文件简写:sender=`cmd_vel_udp_sender.cpp`,receiver=`go2_sdk2_udp_receiver.cpp`,probe=`go2_sdk2_motion_probe.cpp`,saas=`go2_saas_agent.py`,safe_node=`unitree_safe_cmd_node.py`,patrol_control=`patrol_control.py`,follower=`waypoint_follower_go2_2.py`。

| # | claim | 证据 file:line | 判定 |
|---|---|---|---|
| 1 | 生产链 safe→/cmd_vel→sender→UDP→receiver→Move() | saas:2056,2045-2049,2022-2025;receiver:219 | CONFIRMED |
| 2 | 测试链 safe→Request(api_id=1008)→/api/sport/request | safe_node:22,48,164/258-259,238 | CONFIRMED(默认路径,生产不走) |
| 3 | 两后端语义同 Move(vx,vy,yaw) | safe_node:242-246;receiver:219 | CONFIRMED |
| 4 | 节点名 cmd_vel_udp_sender,订阅 /cmd_vel(Twist) | sender:77,103-104 | CONFIRMED |
| 5 | CmdPacket pack(1)/magic G2CM 0x4732434d/version2/… float32 | sender:20,21,23-35,143,148-149,32-34 | CONFIRMED |
| 6 | send_steady_ns/send_system_ns 测延迟 | sender:148-149;receiver:204-206,223-225 | CONFIRMED*(仅 steady 被消费,system 从不读) |
| 7 | vx≤max_vx | sender:81(默认0.3);saas:2048,1476-1483/1679;manifest speed=0.5 | CONFIRMED(生产=0.5) |
| 8 | vy≤max_vy | sender:82(默认0.10);saas:2048(生产0.000),2052 | **DEFAULT_VS_PROD 默认0.10/生产0.000(全链vy=0)** |
| 9 | vyaw≤max_vyaw | sender:83(默认0.5);saas:2048-2049,1496-1501(生产0.600) | **DEFAULT_VS_PROD 默认0.5/生产0.600** |
| 10 | vy×unitree_vy_sign 默认+1,+Y=左 | sender:86,84-85,146 | CONFIRMED(生产vy=0故无效) |
| 11 | sendto 默认=生产 127.0.0.1:5005 | sender:79-80;saas:2047 | CONFIRMED |
| 12 | 每秒 TIMING_SENDER | sender:106-108,181-195 | CONFIRMED |
| 13 | 接收端裸 main + SDK2 | receiver:70,16-17,80 | CONFIRMED |
| 14 | ChannelFactory::Instance()->Init(0,eth0) | receiver:78,72,75;saas:2024;manifest sdk_if=eth0 | CONFIRMED(原文写法略简) |
| 15 | SportClient SetTimeout(10) Init() | receiver:80-82 | CONFIRMED |
| 16 | StandUp→BalanceStand 在本进程 | receiver:91-94;saas:2298-2300 | CONFIRMED |
| 17 | 绑 UDP 5005 SO_RCVTIMEO=1s | receiver:73,105,115-120;saas:2024 | CONFIRMED |
| 18 | 主循环 校验→硬限幅→Move | receiver:193-198,214-216,219 | CONFIRMED(仓库 vy cap=0.10) |
| 19 | 退出 StopMove() | receiver:191(while(true)无break),241(不可达),243(不可达) | **CORRECTED→死代码,靠信号退出,永不执行** |
| 20 | 限速层1 follower max_vx/max_yaw_rate 生产0.5/0.45 | saas:2063,2064;manifest go2_2_max_yaw_rate=0.450 | CONFIRMED |
| 21 | 限速层2 safe limit_planar_command | safe_node:15-16,208-227;patrol_control:420-435 | CONFIRMED |
| 22 | 限速层4 接收端硬编码 ±0.5/±0.10/±0.5 | receiver:215(±0.10,全文无getenv);saas:2023(GO2_SDK_MAX_VY=0.020) | **DOG_UNKNOWN→仓库0.10 vs 意图0.020,无法裁决** |
| 23 | 图 follower 20Hz/vy=0/→/patrol_cmd(+"课程纠偏") | 狗版 follower:98,303,31;manifest | CONFIRMED 数据流;**"课程纠偏"repo≠dog,狗版无,已删** |
| 24 | 图 safe 限幅+ROI急停+断流兜底 20Hz →/cmd_vel | safe_node:53(默认40),281-286/358-362/398,467/471;saas:2053,2055-2056 | **DEFAULT_VS_PROD 默认40Hz/生产20Hz** |
| 25 | motion_probe 诊断探针非主链 | probe:39,117-140;saas:2485(仅清理);主链 saas:2024 | CONFIRMED |
| 26 | 生产 saas start_patrol 启 receiver,net_interface=eth0 | saas:2022-2025,1718,1721;manifest sdk_if=eth0 | CONFIRMED(原「留待坐实」已坐实) |

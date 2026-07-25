# 04 · 运动输出:Twist 怎么变成狗腿动作

> 原则同 00。核心文件:
> `src/go2_cmd_vel_bridge/src/cmd_vel_udp_sender.cpp`(ROS2 节点)、
> `src/go2_cmd_vel_bridge/src/go2_sdk2_udp_receiver.cpp`(裸 C++ + 宇树 SDK2)。

## 一、两种运动输出后端(再确认)
| 后端 | 谁用 | 路径 | 落点 |
|---|---|---|---|
| **SDK2 SportClient**(生产) | 生产链 | 安全节点→`/cmd_vel`→`cmd_vel_udp_sender`→UDP→`go2_sdk2_udp_receiver`→`SportClient.Move()` | 直接调宇树 SDK2 C++ API |
| **sport API 话题**(测试) | 测试链 | 安全节点→`Request(api_id=1008)`→`/api/sport/request` | Go2 板载 ROS 桥接的运动服务 |

两者语义相同(都是 `Move(vx, vy, yaw_rate)`),只是**传输方式不同**。生产选了 SDK2 直连。

## 二、发送端 `cmd_vel_udp_sender`(ROS2 → UDP)
- 节点名 `cmd_vel_udp_sender`,订阅 **`/cmd_vel`**(Twist,来自安全节点)。
- 每帧打成一个**二进制包 `CmdPacket`**(`#pragma pack(1)`):
  - `magic="G2CM"(0x4732434d)`、`version=2`、`packet_size`、自增 `sequence`、`send_steady_ns`/`send_system_ns`(发送时刻,用于测延迟)、`vx/vy/vyaw`(float32)。
- 发送前限幅:`vx≤max_vx`(默认0.3,生产传 speed)、`vy≤max_vy`(0.10)、`vyaw≤max_vyaw`(0.5);`vy` 乘 `unitree_vy_sign`(默认+1,+Y=左)。
- `sendto` 到 **`target_ip:target_port`(默认 `127.0.0.1:5005`)** —— **本机 localhost UDP**(进程间通信,不出网)。
- 每秒打 `TIMING_SENDER` 诊断(gap/callback/sendto 延迟、错误、序号)。

## 三、接收端 `go2_sdk2_udp_receiver`(UDP → SDK2 → 腿)
- **不是 ROS 节点**,是裸 `main`,直接用宇树 SDK2:
  - `ChannelFactory::Init(0, net_interface)`(默认 `eth0`,参数可传)→ 连 Go2 内部 DDS。
  - `go2::SportClient`(运动控制客户端),`SetTimeout(10)`,`Init()`。
- **启动即让狗站起来**:`StandUp()` → sleep2 → `BalanceStand()` → sleep1。⚠️ **狗的"起立/平衡"就发生在这个进程**,不是别处。
- 绑 UDP 端口 5005(`SO_RCVTIMEO=1s`,超时也醒一下好打印 cmd_age)。
- 主循环:`recv` 一个包 → 校验 magic/version/size → **硬限幅** `vx∈[-0.5,0.5] vy∈[-0.10,0.10] vyaw∈[-0.5,0.5]` → 调 **`sport_client.Move(vx, vy, vyaw)`**(**这就是真正驱动狗腿的那一下**)。
- 统计序号丢包 `seq_gaps`、UDP 传输延迟、Move 调用耗时、端到端延迟(`sender_to_move_done_ms`)。
- 退出 `StopMove()`。

## 四、限速是"层层设防"(defense in depth)
同一个速度被反复限幅,任何一层都能兜住:
1. 跟随器 `max_vx/max_yaw_rate`(生产 0.x/0.45);
2. 安全节点 `limit_planar_command`(`max_vx/vy/yaw`);
3. 发送端 `max_vx/vy/vyaw`;
4. 接收端**硬编码** `±0.5 / ±0.10 / ±0.5`(最后一道,写死在 C++ 里)。

## 五、端到端闭环(整条生产巡检驱动链,已全程逐行核实)
```
Livox雷达 ─► FAST-LIO ─┬─► /Odometry ───────────────► waypoint_follower_go2_2(纯跟踪+课程纠偏, 20Hz)
                       │                                        │ Twist(vx,0,yaw) → /patrol_cmd
                       └─► /cloud_registered_body ─► unitree_safe_cmd_node(限幅+前方ROI急停+断流兜底, 20Hz)
                                                              │ Twist(vx,vy,yaw) → /cmd_vel
                                                              ▼
                                                   cmd_vel_udp_sender ─UDP:127.0.0.1:5005─► go2_sdk2_udp_receiver
                                                                                                  │ SportClient.Move(vx,vy,vyaw)
                                                                                                  ▼
                                                                                             Go2 腿(先 StandUp+BalanceStand)
```

## 六、留待坐实
- `SportClient.Move` 的坐标/单位约定、Go2 sport 模式细节(宇树 SDK,第三方,按需)。
- `go2_sdk2_motion_probe.cpp`(诊断探针,非主链)何时用。
- 生产是谁、在哪启动 `go2_sdk2_udp_receiver`(net_interface 传什么)→ 见 08 SaaS agent 装配串(`sdk_receiver_cmd`)。

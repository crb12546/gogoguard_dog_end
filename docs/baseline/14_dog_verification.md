# 真机核验记录 —— 2026-08-01

> **这篇回答一个问题:前面 13 篇是靠读代码写的,拿到真狗上对,对得上吗?**
>
> **核验对象**:Go2 U2,`192.168.123.18`,有线直连,开机约 3 分钟,狗趴着,无巡检运行。
> **核验方式**:SSH 只读观察 + 在独立目录做编译测试。**原目录 `/home/unitree/go2_fastlio_ws` 全程未改动。**
> **被验代码**:`gogoguard_dog_go2_real_code` @ `8b3d789`(删除阶段完成后的版本)

---

# 一、结论速览

| 项目 | 结果 |
|---|---|
| 删完的代码能否编译 | ✅ **9/9 包成功,0 失败,5 分 38 秒,0 编译错误** |
| 完整性自检 | ✅ **6 项全过,0 处断裂** |
| 删掉的 22 个脚本是否真的没用 | ✅ **全部未运行、无 cron、无 service、无活引用** |
| 文档结论是否对得上 | ✅ 4 条对上,❌ **1 条错**(见第三节) |
| 删除决策是否有误 | ❌ **有 1 处**:避障两个文件应归档而非删除(见第四节) |

---

# 二、编译验证

## 做法

```
rsync 仓库 → /home/unitree/go2_dog_clean_test   (700 文件 / 17 MB)
ln -s /home/unitree/go2_fastlio_ws/third_party  (SDK 静态库不在仓库里,按 PROVENANCE 说明)
colcon build --symlink-install --parallel-workers 6 --cmake-args -DCMAKE_BUILD_TYPE=Release
```

**原目录未动**:核验前后 `/home/unitree/go2_fastlio_ws` 顶层 28 项不变。

## 结果

```
Summary: 9 packages finished [5min 38s]
  4 packages had stderr output: fast_lio go2_map_manager livox_ros_driver2 livox_sdk2
编译错误 0 个,警告 14 个
```

各包耗时:
```
go2_fastlio_patrol   5.54s      livox_ros_driver2   2min 23s
go2_loop_backend     6.09s      fast_lio            3min  9s
go2_cmd_vel_bridge  49.9s       livox_sdk2          3min 53s
unitree_api         1min  7s    unitree_sdk2        4min 28s
go2_map_manager     1min 45s
```

## 关键产物(saas 启动链直接执行的)

| 可执行文件 | 大小 | 由谁调用 |
|---|---|---|
| `go2_sdk2_udp_receiver` | 3.6M | saas :2426 |
| `cmd_vel_udp_sender` | 404K | saas :2449 |
| `go2_sdk2_motion_probe` | 3.6M | saas :2960 / guard :80 / watchdog :50 |
| `fastlio_mapping` | 1.5M | `base_bringup.sh` |
| `livox_ros_driver2_node` | 60K | `base_bringup.sh` |
| `route_relocalizer` | 760K | saas :2391(PCD 模式,默认不走) |

**6 个全部产出。**

## 包识别对照

| | 包数 | 差异 |
|---|---|---|
| 原工作区 | 10 | 多一个 `go2_oa_cpp_test` |
| 测试目录 | 9 | 少的就是它(见第四节) |

## 完整性自检

```
python3 tools/check_integrity.py

  ✅ Python 相对/包内 import      0 处断裂
  ✅ setup.py console_scripts     0 处断裂
  ✅ saas 引用的 scripts/          0 处断裂
  ✅ shell 脚本间引用              0 处断裂
  ✅ CMakeLists 源文件             0 处断裂
  ✅ 单元测试 import               0 处断裂
  合计断裂: 0
```

---

# 三、文档结论逐条对账

## ✅ 对上的

### 1. 相机跑 `unitree_builtin` 不是 z1pro
```
/home/unitree/go2_fastlio_ws/config/camera.env:
  GO2_CAMERA_SOURCE=unitree_builtin
  GO2_BUILTIN_CAMERA_MAC=4c:bb:47:ab:e4:c2
  GO2_BUILTIN_CAMERA_FPS=15
  Z1PRO_RTSP_URL=rtsp://192.168.144.108/     ← 保留但未启用
```

### 2. 巡检实跑的是 trace 版跟线器
最后一次巡检 `runs/20260726/xunjian-20260726-02/manifest.txt`:
```
controller_executable=waypoint_follower_go2_2_trace.py
controller=deployed_go2_2_nearest_lookahead_unified_horizontal_frame
controller_heading_feedback=full_fast_lio_quaternion_in_frozen_gravity_level_frame
localization_mode=manual_anchor
horizontal_frame_enabled=true
speed=0.5   loop=pingpong
```

### 3. nav2 / AMCL / slam_toolbox 配置是死的 —— 而且比文档写的更彻底
```
5 份 yaml 最后修改时间:2026-05-17 13:04,之后再没动过
nav2_bringup   未安装
slam_toolbox   未安装        ← 那两份 slam_toolbox 配置就算被引用也跑不起来
nav2_amcl      已安装(但缺整个 nav2 栈和 map_server,单独无法用于导航)
```

### 4. 起点对齐检查是自证的 —— **真实数据实证**

最后一次巡检的三份路线,各自的起点坐标:

| 文件 | 起点 x | 起点 y | 含义 |
|---|---:|---:|---|
| `route_original.csv` | **6.736304** | **−6.468605** | 当初录制时的真实位置 |
| `route_runtime.csv` | **−0.002130** | **−0.037761** | manual_anchor 转完 —— **搬到狗脚下** |
| `route_horizontal.csv` | **0.000000000** | **0.000000000** | 精确原点 |

`manual_anchor.json` 显示这次搬运了 **1277 个路点**。

**整条路线被搬到狗的位置上。** 那道 `check_route_start_alignment.py`
再去量"狗和路线起点差多少",量到的是 **3.8 厘米** —— 结构上只可能通过。

**从代码顺序推出的结论,被真实运行数据证实。**

## ❌ 错了的:开机自启是 4 条,不是 5 条

文档 `12` 早先版本写"五条常驻循环",把 `deploy/systemd_user/go2-fastlio-base.service` 算作第五条。

```
systemctl --user status go2-fastlio-base   →  Unit could not be found
~/.config/systemd/user/                     →  无 go2 单元
find / -name "go2-fastlio-base.service"     →  只有仓库里那一份
```

**它从来没被安装到狗上。**

真机实际 enabled + running 的团队服务:
```
go2-saas-command.service        active running
go2-saas-outbox.service         active running
go2-saas-video.service          active running
go2-wired-ssh-rescue.service    active running
```
（另有宇树自带的 `unitree-upgrade.service`,不是团队代码)

开机 3 分钟时的进程表:**没有任何 FAST-LIO / 雷达 / 相机进程**,只有上面 4 个。

**教训**:仓库里有 `.service` 文件 ≠ 它在狗上生效。
判断自启必须看 `systemctl list-unit-files --state=enabled`。

已在 `12_one_line_walkthrough.md` ⓪ 节更正。

---

# 四、删除决策复核

## ✅ 22 个被删脚本:确认无用

狗上还在、我删掉的 22 个,逐个核验(用完整进程表,不用 `pgrep -f` —— 后者会匹配到查询命令自身):

```
build_legacy_iox_stub.sh          install_a7600c_ecm_only.sh
check_legacy_send_cmd_start.sh    install_a7600c_ppp_only.sh
enable_oa_only.py            ←    install_autostart.sh
go2_4g_manager.py                 install_connectivity_watchdog.sh
go2_a7600c_usb_monitor.sh         install_go2_4g_manager.sh
go2_connectivity_watchdog.sh      install_network_recover.sh
go2_motion_probe.sh               patrol_cli.disabled_before_cmd_rework.py
go2_network_recover.sh            probe_legacy_go2_cmd_bridge.sh
go2_saas_agent.py.before_×2       run_roomtest7_×2
                                  start/stop_legacy_go2_cmd_bridge.sh
```

- **运行中**:0 个
- **crontab**:`no crontab for unitree`
- **systemd 引用**:0 处
- **互相引用**:全部闭合在这 22 个内部(4G 簇引 4G 簇,legacy 簇引 legacy 簇)——
  **保留的 33 个活脚本,没有一个引用它们**

## ✅ 4G 那套(3685 行)删得最有把握

真机查到了 4G 的真实管理者:

```
ip -br addr:
  usb0    UP    192.168.0.100/24          ← 4G 走 ECM 模式,不是 PPP
  eth0    UP    192.168.123.18/24  192.168.1.5/24  192.168.144.100/24

ip route show default:
  default via 192.168.0.1 dev usb0        ← 默认路由确实走 4G

4G 模块:/dev/ttyUSB0  /dev/ttyUSB1  /dev/ttyUSB2       在位

谁在管:
  NetworkManager   active
  ModemManager     active
  pppd             未运行
```

**4G 正在工作,由操作系统的 NetworkManager + ModemManager 管理。**
被删的 `go2_connectivity_watchdog.sh` / `go2_4g_manager.py` 没跑、没 cron、没 service。

> 这条修正了 `CLAUDE.md` 里的旧记录("4G 跑 shell `go2_connectivity_watchdog.sh`
> 非 Python `go2_4g_manager.py`")。**真相是两个都没跑,操作系统在管。**

## ❌ 删错 1 处:厂商避障 API 参考

`scripts/enable_oa_only.py` + `cpp_tools/go2_oa_cpp_test/` 按"零引用"被删(commit `1a1cf67`)。

**零引用这一点是对的,但删除是错的** —— 它们记录了宇树 SDK 的避障接口:

```cpp
#include <unitree/robot/go2/obstacles_avoid/obstacles_avoid_client.hpp>
client.SwitchSet(true);                 // 开/关厂商内置避障
client.SwitchGet();                     // 读开关状态(纯读,安全)
client.UseRemoteCommandFromApi(true);   // 避障开着的同时接受 API 速度指令
client.Move(vx, vy, vyaw);              // 带避障地走
```

**这正是第 2 步"保持 50-70cm 平滑绕行"需要的能力,而且是厂商提供的。**

已恢复到 `attic/厂商避障API参考/`(含说明 README)。

### 但要说清楚:这两个文件在狗上跑不了

```
enable_oa_only.py    写死 SDK_PATH="/home/unitree/unitree_sdk2_python"  → 该目录不存在
                     unitree_sdk2py 模块在整台狗上未安装(pip 环境也没有)
oa_cpp_test.cpp      cpp_tools/go2_oa_cpp_test/build/ 只有 CMakeCache.txt,无任何产物
                     → 从没编译成功过
```

**它们是没做完的实验,价值在于记录了接口,不在于能跑。**

⚠️ `oa_cpp_test.cpp` 里有 `client.Move(0.5f, 0, 0)` 持续 1 秒 —— **编译运行会让狗往前走**。

---

# 五、用户提供的关键背景(2026-08-01)

> 遥控器**长按 Y** = 解除避障;**短按 A** = 开启避障;**开机自动开启避障**。
> 确实是狗本身(厂商)的避障。

## 这引出一个必须查清的问题

**巡检的时候,到底有几套避障在跑?**

已知:
1. 厂商避障**开机默认开启**
2. 巡检链自己还有一层 ROS 安全层(`unitree_safe_cmd_node.py`:ROI 盒子 + 数点 + 二值停)
3. 巡检发指令走 `SportClient.Move()`(`go2_sdk2_udp_receiver.cpp:219`),
   **不是** `ObstaclesAvoidClient.Move()`

未知:
- 厂商避障开着时,会不会拦截/修改 `SportClient.Move()` 的速度?
- **如果会** → 巡检期间是两套避障叠加,现场看到"狗停下来"可能根本不是 ROI 触发的,
  过去所有关于避障行为的观察都需要重新归因
- **如果不会** → 巡检期间厂商避障形同虚设,只有 ROI 那层在起作用

## 怎么查(安全,不让狗动)

写一个**只调 `SwitchGet()`** 的 C++ 探针,链接已编译好的 `libunitree_sdk2.a`,
读出巡检前/巡检中的开关状态。`SwitchGet` 是纯读接口,不发运动指令。

**为什么必须查清楚**:这决定第 2 步避障方案的起点 ——
"在厂商避障之上做路径规划" vs "从零做避障",工作量差一个数量级。

---

# 六、顺带核实:Docker 与桌面环境

用户此前问"GNOME 桌面和 Docker 能不能去了"。真机数据:

| | 数据 | 结论 |
|---|---|---|
| Docker | enabled + active,但 **0 容器、0 镜像、`/var/lib/docker` 占 4.0K** | 完全是空壳 |
| 桌面栈 | 7 个进程,**CPU 0.9%,内存 261 MB** | 开销很小 |
| 整机内存 | 15 Gi 总量,**已用 2.1 Gi,可用 12 Gi** | 内存完全不紧张 |

**结论**:两个都可以关(减少攻击面、少几个进程),
但**不要指望关掉它们能解决性能问题** —— 它们本来就几乎不占资源。

---

# 七、CPU 排查(2026-08-01 实测)

## 做法

用 `tools/diag/cpu_forensics.py`(187 行,纯读 `/proc`,零外部依赖 ——
因为狗上**没装 sysstat**,`pidstat` 不存在;**没装 linux-tools**,`perf` 不存在)。

分三档测,其中"底座"这一档是关键设计:
**`base_bringup.sh` 只启动雷达驱动 + FAST-LIO 两个进程,不含任何运动调用**,
所以可以在**狗完全不动**的前提下复现感知与 SLAM 的算力负载。

启动前的安全预检(11 个运动关键词在 `base_bringup.sh` 中全部 0 命中):
```
Move  StandUp  BalanceStand  sport_client  motion_enabled  patrol_cmd
cmd_vel  safe_cmd  udp_receiver  udp_sender  waypoint_follower
```
运行期间复核:`unitree_safe_cmd_node` / `cmd_vel_udp_sender` /
`go2_sdk2_udp_receiver` / `waypoint_follower` 均为 **0 个进程**。狗全程未动。

## 三档负载对照

| 档位 | 整机忙 | 上下文切换/s |
|---|---:|---:|
| **空闲**(狗趴着,仅 4 条常驻) | **0.11 核** | 408 |
| **底座**(雷达 + FAST-LIO,狗不动) | **1.45 核** | 25,000 |
| **巡检**(此前遥测记录) | **7.98 核** | 3,607,648 |

## 底座档的逐进程归因 —— 干净

30 秒 `/proc/<pid>/stat` 差分:

```
整机忙                      1.45 核 / 8 核
  livox_ros_driver2_node    1.128 核    ← 雷达驱动
  fastlio_mapping           0.210 核    ← SLAM 算法本体
  go2_saas_agent command    0.018 核
可归因合计                  1.36 核
不明去向                    0.10 核  (7%)
```

### ⚠️ 第一个发现:雷达驱动比 SLAM 算法贵 5.4 倍

`livox_ros_driver2_node` **1.128 核** vs `fastlio_mapping` **0.210 核**。

一个"把网络包解成点云再发布"的驱动,开销是 IESKF + ikd-Tree 整套算法的 5 倍以上。

**这是稳态第一大 CPU 开销,而且它的配置在我们手里**
(`livox_lidar_publisher.yaml` / `msg_MID360s_launch.py`:发布频率、点云格式、
是否同时发 PointCloud2 和 CustomMsg、DDS 序列化路径)。
**第 3 步重构应把它列为优化对象。**

### 第二个发现:底座档归因干净 → 6.2 核不在底座

不明去向仅 0.10 核(7%)。**说明"6.2 核不明"不可能来自雷达或 FAST-LIO。**
它只可能来自巡检时额外启动的东西:运动链(安全层 + 两个 UDP 搬运 + 跟线器)
与观测层(rosbag + telemetry + performance_monitor + trace 写盘)。

## 上下文切换归因 —— 我在这里犯了两个测量错误

### 错误一:只读主线程,得出"98% 归因不上"

第一次测,读 `/proc/<pid>/status` 的 `voluntary_ctxt_switches` /
`nonvoluntary_ctxt_switches`,结果:
```
整机 25,135 次/秒,可归因仅 474 次/秒 (2%)
```

**差点把这当成系统级谜团。** 实际原因:
`/proc/<pid>/status` **只报主线程**,而 ROS2 节点是多线程的。

改为遍历 `/proc/<pid>/task/<tid>/status` 汇总全部线程后:
```
整机 25,180 次/秒
  fastlio_mapping          7,235 次/秒   (11 线程)
  livox_ros_driver2_node   4,859 次/秒   (17 线程)
  [rcu_preempt]              135 次/秒
  _ros2_daemon                72 次/秒   (23 线程)
  containerd                  44 次/秒   (14 线程)
可归因 12,345 次/秒 (49%)
```
**归因从 2% 升到 50%。**

> 顺带印证了 `09_measured_baseline.md` 的记录:FAST-LIO 最大线程数 11 —— 实测正是 11。

### 错误二:剩下的 50% 也差点当成"不明"

完整统计(所有进程、所有线程、无阈值):
```
整机切换率           25,001 次/秒
存活进程可归因       12,399 次/秒 (50%)
期间新建进程贡献          0 次/秒
仍不明               12,602 次/秒 (50%)
```

这 50% 是**空闲任务(swapper,PID 0)的切换** —— 每个核进入/退出空闲都计入
`/proc/stat` 的 `ctxt`,但 PID 0 没有 `/proc` 条目。
8 个核 × 高频唤醒(雷达 10 Hz + DDS 线程)→ 这个量级属**正常内核开销,不是问题**。

## ⚠️ 对"6.2 核不明"这个旧结论本身的存疑

该数字来自此前对巡检遥测的分析("整机 P95 7.98 核、项目进程 2.87 核、
6.20 核不明")。

**今天的两个测量错误提示:那次分析很可能犯了同一类错** ——
按固定进程名 pattern 匹配、且未汇总线程,导致项目进程的真实占用被低估。

**结论:`6.2 核不明` 这个数不可信,需要在真跑巡检时用今天修正后的方法重新测量。**
(汇总 `task/<tid>`、不用进程名 pattern 而用全量 `/proc` 差分)

## 本档测量的边界

- 只测了**底座**,没测运动链和观测层 —— 那两摊只在巡检时存在
- 巡检档的数字是**引用此前遥测**,不是今天测的
- 要闭合这个问题,必须跑一次真实巡检并全程采样(见第九节 C 项)

---

# 八、本次核验中我自己犯的操作错误

**用 `pgrep -fc "文件名"` 查进程,结果每个都返回 2 —— 全是假阳性。**
原因:`pgrep -f` 匹配完整命令行,而我的 SSH 命令行里就含着这些文件名字符串,
它匹配到了查询命令自身。**改为先抓完整 `ps` 输出到文件再本地 grep,才得到正确结果(全部 0)。**

**SSH 内嵌套引号转义漏了,`|` 被当成管道执行。**
本意是 `grep -icE "gnome|gdm|Xorg|nautilus"`,实际把 `gdm`、`Xorg`、`nautilus`、`pulseaudio`
当命令执行了 —— **在狗上意外启动了 Xorg 和 Nautilus**(都立刻失败:X 已在跑、无显示)。
已核实无残留进程。**改用 `ssh dog 'bash -s' <<'EOF'` 传脚本,不再内嵌引号。**

---

# 九、下一步

| | 事项 | 状态 |
|---|---|---|
| A | CPU 排查 | ✅ 本次完成(第七节)。结论:底座 1.45 核归因干净;雷达驱动是最大头;`6.2 核不明` 存疑需重测 |
| B | 归档避障文件 + 更正文档 | ✅ 本次完成 |
| C | 真机跑一次完整巡检 | **待做,需用户在场、场地安全、有急停**。一次跑完可同时闭合两件事:①验证删除后的版本能实跑 ②用修正后的方法重测 CPU |
| D | 写 `SwitchGet` 探针,查清巡检时厂商避障是否生效 | 待做(第 2 步前置) |
| E | 查 `livox_ros_driver2_node` 为何吃 1.128 核 | 新增。稳态第一大开销,配置在我们手里 |

## 狗的收尾状态(本次核验结束时)

```
底座已用 base_stop.sh 停止:livox / fastlio / ros2 launch 均 0 个进程
4 条常驻服务完好:command-loop / outbox-loop / video-loop / wired-ssh-rescue
负载恢复 1.17,与核验开始时一致
原目录 /home/unitree/go2_fastlio_ws 全程未改动
测试目录 /home/unitree/go2_dog_clean_test 按用户要求保留(含编译产物,下次免重编)
```

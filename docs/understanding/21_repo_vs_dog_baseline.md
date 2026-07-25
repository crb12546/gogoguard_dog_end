# 21 · 仓库 ↔ 狗上 基线对齐(规范的前置决策)

> 原则同 00:只认代码。**本篇基于 2026-07-26 对 git 树 blob sha 的逐文件比对 + 6 个分叉文件的内容 diff**。
> 目的:给"规范仓库"提供**可拍板的基线**——哪些一致、哪些分叉、每个该以谁为准、哪些必须回捞。
> 两份代码:`orin_go2_fastlio_ws/`=仓库开发版;`realtime_dog_end_code/`=**狗上背板真实程序**(2223文件,带狗自己的 `SHA256SUMS`;其 follower sha256 `d205a596`=狗运行 manifest `controller_reference_sha256`,已坐实为权威狗版)。

## 〇、核验状态
- 【今日实测】= 对 `/tmp/gg_tree2.json`(GitHub 最新递归树,commit `da068cc`)逐文件比 git blob sha + 对 6 分叉文件 `diff` 实读。
- **口径**:git blob sha 内容寻址,两文件 sha 相同 ⟺ 内容 byte 级相同。仅比源码类(py/cpp/hpp/h/sh/yaml/yml/json/xml/service)。
- ⚠️ 权威(canonical)归属是**团队决策**,本篇只给"谁新/改了啥/建议",不替你定。

## 一、总体分叉(源码逐文件)【今日实测】
| | 数 | 说明 |
|---|---|---|
| 两边共有 | 1961 | 其中 **byte 级相同 1955(99.7%)**、**内容不同 6** |
| 仅仓库有 | 40 | 多为噪音:`backups/`×15 + `maps/` nav2 yaml×18;真源码仅 ~6(见三) |
| 仅狗上有 | 21 | 2 配置(`laser_mapping.yaml`/`livox_lidar_publisher.yaml`)+ 其余 `xbf*.{quality,recording,horizontal}.json` 路线数据 |

> **结论:99.7% 已一致。规范聚焦 ≈14 个文件(6 分叉 + 2 狗独有配置 + ~6 仓库独有源码),不是全库重整。**

## 二、6 个内容分叉文件(逐个:方向 / 实质 / 建议)
> 方向=谁的版本更新;建议里 canonical 归属**待团队拍板**。

| # | 文件 | 行数 仓/狗 | 谁领先 | 实质改动【diff 实读】 | 建议 |
|---|---|---|---|---|---|
| ⑥ | `src/go2_fastlio_patrol/.../waypoint_follower_go2_2.py` | 1043 / 330 | **仓库(大重写)** | 仓库=`WaypointFollowerGo22`,import `go2_course_control`+`QoSProfile`+`json/os`,含课程反馈(治蟹行)+trace 埋点;狗=`WaypointFollower` 简版,仅 csv/math/time/rclpy,单一 body-yaw,无课程反馈 | 🔴**核心决策**:1043 重写是"要部署的未来"还是"已废弃分支"?决定了狗要不要升级、doc 02 描述哪版 |
| ① | `src/go2_fastlio_patrol/.../unitree_safe_cmd_node.py` | 636 / 569 | **仓库(+70)** | 仓库加:`cloud_recovery_frames`(点云超时后连续5帧新鲜才放行)、`startup_enable_file`(开机运动联锁)、点云订阅显式 `QoS depth=2/BEST_EFFORT`;**狗上这些都没有**(裸 `depth=10` 默认订阅) | 仓库领先=安全增强;评审后下发狗。⚠️ **doc 03 把 cloud_recovery/startup_interlock 当生产在跑=描述的是仓库版,狗上无** |
| ② | `scripts/go2_network_recover.sh` | 216 / 248 | **🟠 狗(+34)** | 狗加:PPP 接口探测(`ppp[0-9]+`)、`gsm_should_not_interrupt`(GSM 忙时不打断)、PPP 感知默认路由(PPP 用 `route ... dev` 无网关) | **狗→仓库回捞**(PPP 现场适配,对应 doc 09 的 PPP 4G 模式;以仓库为准会丢) |
| ③ | `src/go2_cmd_vel_bridge/src/go2_sdk2_motion_probe.cpp` | 150 / 151 | 狗 | 狗:默认 iface `eth0→eth1` + 读 `GO2_SDK_IF` 环境变量 | 狗→仓库回捞(狗 SDK 走 eth1);低风险(诊断探针) |
| ④ | `scripts/install_go2_4g_manager.sh` | 261 / 255 | 仓库(+8) | 仓库:重启退避更细(`FATAL_REBOOT_MAX_BURST/BACKOFF/STABLE_ONLINE`、`USB_UNSTABLE_SETTLE`、`ABSENT_RESCAN`),`REBOOT_DELAY 90→30` | 仓库领先=4G 重启策略改进;下发狗 |
| ⑤ | `scripts/z1pro_capture.sh` | 119 / 119 | 仓库(对) | RTSP 探测串:仓库 `\r\n`(正确 CRLF)/ 狗 `\\r\\n`(字面反斜杠=探测 bug) | 仓库为准(狗有小 bug) |

**双向欠债规律**:仓库攒了开发(follower 重写 / safety 增强 / 4G 重启策略)**未部署**;狗有现场补丁(PPP 路由 / eth1)**未回仓**。→ **不能一刀切"以仓库为准",逐文件定,两方向都要回捞。**

## 三、单边源码(非噪音部分)
**仅仓库有的真源码 ~6**(狗上没有):
- `src/go2_fastlio_patrol/.../go2_course_control.py` —— **文件级坐实:狗上根本没有课程反馈模块**(呼应 ⑥,狗 follower 不 import 它)。
- `scripts/go2_pcd_capture.py`、`config/horizontal_frame_calibration.json`、`test/{fake_odom_route_publisher,test_go2_course_control}.py`、`src/livox_ros_driver2/test/no_sync_timestamp_mapper_test.cpp`。
- (另 `backups/`×15、`maps/` yaml×18 是备份/地图,非部署代码,归档即可。)

**仅狗上有 21**:`laser_mapping.yaml`、`livox_lidar_publisher.yaml`(2 个狗上配置)——✅**已核:非活配置**。狗 `base_bringup.sh`(与仓库 sha 相同 `19cedc6a`)第 220 行 `ros2 launch fast_lio mapping.launch.py config_file:=go2_mid360s.yaml`,且 `go2_mid360s.yaml` **repo↔狗 sha 相同(`3ecfccf4`)**;这 2 个 yaml 无任何脚本引用=遗留/备用。**⟹ doc 01/18/19 的 FAST-LIO 生产值(cube1000/det100/iter3/extrinsic false 等)对狗成立、已坐实。** 其余全 `xbf*` 路线质量/录制/水平校正 json(运营数据)。

## 四、对已有文档的影响(须随基线决策更新)
- **doc 02**(CSV 巡检):已标 repo≠dog;⑥定案后明确"描述哪版"。
- **doc 03**(安全节点):⚠️ **cloud_recovery(5帧)/startup_interlock 是仓库独有,狗上无**——doc 03 现按仓库 636 行写,若狗为准须删这两节。
- **doc 09**(4G):`go2_network_recover.sh` 的 PPP 段是**狗现场补丁**;`install_go2_4g_manager.sh` 重启策略仓库更新——两处需注方向。
- **全部 22 篇**:现在有了狗全量代码,那些 "无狗上对照" 的 hedge 大多可升级为 "repo==dog(sha验)"(1955 文件适用),仅 6 分叉单独标——**建议做一次"文档转狗权威版"**(待讨论)。

## 五、规范基线的建议流程(拍板后执行)
1. **逐文件定 canonical**(本篇二/三表)——尤其⑥(follower)是主决策。
2. **双向回捞**:狗→仓库(②③ PPP/eth1 现场补丁,勿丢);仓库→狗(①④ 安全/4G 改进,评审后部署)。
3. ~~核对狗独有配置~~ ✅已核:狗加载 `go2_mid360s.yaml`(与仓库相同),2 个狗独有 yaml 未使用 → doc 01/18/19 生产值对狗成立。
4. **归档**:`backups/`、死代码(doc 20 清单)。
5. **冻结契约后**再动结构(doc 20 第六节)。

## 核验台账
| # | claim | 证据【今日实测】 | 判定 |
|---|---|---|---|
| 1 | 源码 1961 共有,相同 1955 / 不同 6 | git blob sha 逐文件比(tree `da068cc`) | CONFIRMED |
| 2 | 狗 follower=330行 WaypointFollower sha d205a596=manifest | `realtime_dog_end_code/SHA256SUMS` + wc/grep | CONFIRMED |
| 3 | 6 分叉方向/实质 | 各文件 `diff` 实读(见二) | CONFIRMED |
| 4 | go2_course_control 狗上无 | 仅 `orin_go2_fastlio_ws/` 有该路径 | CONFIRMED |
| 5 | 狗独有 laser_mapping.yaml/livox_lidar_publisher.yaml | 仅 `realtime_dog_end_code/` 有 | CONFIRMED |
| 6 | doc 03 的 cloud_recovery/interlock 狗上无 | safety node diff:该逻辑仅 orin 版有 | CONFIRMED(doc 03 待修) |

> **狗上对照**:本篇是**两版之间的实测比对**,repo/dog 双侧均有据(git 树含两份)。canonical 归属未定=团队决策,非代码可导出。

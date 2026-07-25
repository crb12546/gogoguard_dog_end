# 06 · PCD 地图怎么生成(打点 → 拼合 → 去动态)

> 原则同 00。核心包:`src/go2_loop_backend`(**11 个 ROS 可执行**,离线建图/回环工具集)。
> **已逐行读**:`keyframe_saver.py`、`export_registered_cloud_map.py`、`build_raw_map.py`、`dynamic_map_filter.py`。
> **按名/头部推断角色、待按需深读**:`sliding_window_static_filter`、`scan_context_detector`、`pose_graph_optimizer`、`level_pcd`/`level_cloud_node`、`pcd_to_nav2_map(_fast)`、`filter_keyframes_front_fov`、`odom_to_tf_*`。

## 核验状态
- **本轮已对磁盘源码逐条核**:setup.py console_scripts、keyframe_saver / export_registered_cloud_map / build_raw_map / dynamic_map_filter / sliding_window_static_filter / scan_context_detector / pose_graph_optimizer / level_pcd / level_cloud_node / pcd_to_nav2_map(_fast) / filter_keyframes_front_fov / odom_to_tf_map(_2d/_level_2d/_odom_level_2d) 全部读过;所有**数值/阈值/topic/PCD 格式/算法**均与源码 argparse/declare 默认值逐条吻合。
- **本轮修掉 3 处台账实质问题**:①`odom_to_tf_map*.py` 的 TF 帧向原文写反(实为 `map→base_link`,不是 `odom→map`);②"没有任何脚本编排/grep 全空"过度断言(`go2_start_level_scan.sh` 确实编排了包内两个节点);③路线 B 管线箭头把三个各自独立读关键帧的工具画成串行数据流。另订正 `pcd_to_nav2_map` 产物、`level_cloud_node` 校平角度、"11 个可执行 vs 6 个未注册模块"的口径。
- **狗上对照:几乎全空**。doc06 引用的 `go2_loop_backend` **全部为自研文件,不在狗上 4 份副本内**,狗上是否跑同一份代码**无法验证**(标 `【无狗上对照】`)。唯一有狗上副本的是 `FAST_LIO/src/laserMapping.cpp`,且 **repo≠dog**(repo sha256 `5fec8282…` ≠ 狗上 `e4cd05cb…`,行号也不同),但**本篇唯一相关断言 `pcd_save_en 默认=false` 在 repo(:77)与狗上(:77)两侧一致**。
- **口径约定(源标签)**:`【默认 code:文件:行】`=节点内 declare/argparse 默认值;`【生产 saas:文件:行】`=`go2_saas_agent.py`/`base_bringup.sh` 巡检链实际下发值;`【生产 helper:文件:行】`=手动脚本 `go2_start_level_scan.sh` 的 `-p` 覆盖串(非 saas 巡检链、无狗上副本,狗上是否用它无从确认);`【狗上 dog:证据】`;`【推断-未验】`。

## 关键前提
- FAST-LIO 巡检配置 `pcd_save_en:false`(见 01)→ **地图不是 FAST-LIO 自动存的**。
  - 【默认 code:FAST_LIO/config/go2_mid360s.yaml:52 = false;laserMapping.cpp:77 代码默认亦 false】
  - 【生产 saas:base_bringup.sh:220 明确以 `config_file:=go2_mid360s.yaml` 起 fast_lio】(其余 ouster64/velodyne/mid360/horizon 配置为 true,但**非** go2 巡检配置)
  - 【狗上 dog:laserMapping.cpp 狗上副本 :77 亦 false,与 repo 一致;但 repo≠dog(sha 不同)】→ 结论稳。
- ⚠️ **订正原"没有任何脚本/SaaS 编排这些工具(grep 全空)"** —— 该断言**过度**,应拆成两层:
  - **(准)11 个 map-building 可执行 + `keyframe_saver` 确实无任何 saas/launch/base_bringup 编排** → 这部分"**pcd 建图不在自动巡检链里、是离线手动跑命令**"的**结论成立**。【生产 saas:go2_saas_agent.py 全篇无 build_raw_map / dynamic_map_filter / keyframe_saver 等引用】
  - **(错)"grep 全空/没有任何脚本"字面为假** —— `go2_start_level_scan.sh`(独立手动 tmux 脚本,`PITCH` 默认 13.0)**确实编排了包内两个节点**:`odom_to_tf_map_level_2d.py`【生产 helper:go2_start_level_scan.sh:50,`-p level_pitch_deg:=13.0`】与 `level_cloud_node.py`【生产 helper:go2_start_level_scan.sh:53,`-p pitch_deg:=13.0`】。但**此脚本本身也非 saas 巡检链、无任何调用者,属手动 helper**,且 `【无狗上对照】`。
  - `odom_to_tf_map.py` / `odom_to_tf_map_2d.py` / `odom_to_tf_odom_level_2d.py` **无任何脚本启动**(仅在 go2_start_level_scan.sh:9-11 被 `pkill` 清理)→ **实为死代码**。【默认 code:go2_start_level_scan.sh:9-12 只 pkill 不 start】
- 产物 pcd 供巡检前重定位用(见 07),或转 nav2 栅格。
  - "**转 nav2 栅格**"成立(见下 `pcd_to_nav2_map`,CONFIRMED)。
  - "**与路线同名 / route_relocalizer**"属 doc07 范畴,本篇不展开【推断-未验】。saas 侧确有按名 pcd 重定位入口【生产 saas:go2_saas_agent.py:629/678 `relocalize=pcd` 模式、:570 `resolve_named_path` 按名解析 pcd】;**但狗上 manifest `localization_mode=manual_anchor`**【狗上 dog:runs/xunjian-20260725-06、-07/manifest.txt】,与 "route_relocalizer" 措辞不完全对应,**留待 doc07 核**。

## "打点拼合"到底怎么做(回答你的问题:是的,就是不停打点然后拼)

### 造原始地图有两条路

**路线 A:直接从 rosbag 累积**(`export_registered_cloud_map.py`,自研 `【无狗上对照】`)
- 录一个含 **`/cloud_registered`** 的 rosbag(FAST-LIO 输出的**世界系**已配准点云)。【默认 code:export_registered_cloud_map.py:42 `--topic` 默认 `/cloud_registered`】
- 从 bag 的 sqlite 里逐帧读(每 `frame_stride=3` 帧、每 `point_stride=2` 点)。【默认 code:export_...:44 `frame_stride=3`、:45 `point_stride=2`、:57 `sqlite3.connect`、:82/:90 步幅逻辑】
- **因为 `/cloud_registered` 已经是世界系**,直接 `np.vstack` **拼**起来即可(**无需再变换**),再 `voxel_downsample(0.08)` 合并重复点 → 写 pcd。【默认 code:export_...:108 `np.vstack` 无坐标变换、:46 `voxel=0.08`、:111 下采样】
- 特点:最简单,**依赖 FAST-LIO 里程计,不纠漂移**。

**路线 B:关键帧 + 位姿变换拼合**(`keyframe_saver.py` → `build_raw_map.py`,自研 `【无狗上对照】`)
- `keyframe_saver`(在线节点):订阅 `/Odometry` + **`/cloud_registered_body`**(机体系),**每移动 ≥1.0m 或转 ≥10°** 存一个关键帧 = 一个机体系 pcd + 一行位姿。
  - 【默认 code:keyframe_saver.py:32-33 `odom_topic=/Odometry`、`cloud_topic=/cloud_registered_body`(节点内 declare 默认,**该节点无任何脚本/-p 启动,无生产覆盖值**);:35-36 `distance_thresh=1.0`、`yaw_thresh_deg=10.0`;:85 `if dist<thresh and dyaw<thresh: return` → 保存条件 = `dist≥1.0` **或** `dyaw≥10°`;首帧 `last_key_pose=None` 无条件保存】
  - 一行位姿写入 `poses_raw.txt`:`# idx stamp x y z qx qy qz qw yaw pcd_file`。【默认 code:keyframe_saver.py:49 表头、:94-99 逐字段写入】
- `build_raw_map`(离线):逐关键帧 `pts_world = pts_body @ R(q).T + t` **把每帧点云按其位姿变换到世界系再 vstack 拼**,`voxel_downsample(0.10)` → `raw_map.pcd`。【默认 code:build_raw_map.py:92-94 `R=quat_to_rot; pts_w = pts @ R.T + t`、:104 `np.vstack`、:69 `--voxel` 默认 `0.10`、:68 `--out` 默认 `.../raw_map.pcd`、:107】(argparse 默认;无脚本调用故默认即生效)
- 特点:关键帧+位姿可先做**回环/位姿图优化**再拼,**能纠漂移**。

### 去动态(拼完的清洗,`dynamic_map_filter.py`,自研 `【无狗上对照】`)
把"人、临时障碍"等动点从地图里删掉:
- 逐关键帧变换到世界系,裁剪 `z∈[-0.3,2.5]`、去掉离机身水平(XY)距离 <0.5m 的点(狗自己的腿)。【默认 code:dynamic_map_filter.py:130-131 `z_min=-0.30/z_max=2.50`、:173 `pts_world=pts_local@R.T+t`、:178-179 z mask;:132 `ego_radius=0.50`、:176 `local_range=norm(pts_local[:,:2])`、:180 保留 `local_range>=0.5`(即删 <0.5m)】
- 体素化,统计每个体素:被多少**帧**命中(`hit_frame_count`,记为 hf,**是帧数不是点数**)、质心;**可选**光线投射(默认关)把"传感器到命中点"沿途体素标记为 free。【默认 code:dynamic_map_filter.py:139 `--raycast` `action=store_true`(**默认关闭**)】
- 判定:
  - 命中 **≥3 帧** → 高置信**静态**,保留;【默认 code:dynamic_map_filter.py:134 `static_hit_frames=3`、:237 `if hf>=3`】
  - 命中 **≥2 帧且周围 ≥4 个稳定邻居** → 静态,保留;【默认 code:dynamic_map_filter.py:135 `candidate_hit_frames=2`、:136 `min_neighbor_support=4`、:137 `neighbor_min_hit_frames=2`(稳定邻居=邻域体素命中帧≥2)、:234/:243】
  - 光线 free 反证强(`free ≥ 5×命中帧`)→ **删**(动态/穿透);**仅 `--raycast` 开启时生效,默认关**。【默认 code:dynamic_map_filter.py:143 `free_conflict_ratio=5.0`、:249 `if args.raycast and fc>=5*max(1,hf)`】
  - 单帧/弱支持 → **删**。
- 输出 `static_map_filtered.pcd`(干净静态图)+ `dynamic_removed.pcd`(被删的动点)。【默认 code:dynamic_map_filter.py:126-127 `--out_static/--out_removed`、:261-262】

### 另一条静态过滤(`sliding_window_static_filter.py`,自研 `【无狗上对照】`)
- 与 `dynamic_map_filter` **并列的替代方案**,同样**直接读关键帧目录**(`--keyframes` → `poses_raw.txt` + 各关键帧 pcd),**不消费 `dynamic_map_filter` 或 `build_raw_map` 的输出**。【默认 code:sliding_window_static_filter.py:114/:132 `--keyframes`,不读 dynamic 输出】
- 产出另一条静态图(滑窗静态过滤),与 `raw_map` / `static_map_filtered` **三条并列产物**、按需三选一。

## 完整离线管线(手动,**并列可选**,不是一条串行数据流)

> ⚠️ 订正:原箭头图把 `build_raw_map / dynamic_map_filter / sliding_window_static_filter` 画成串行,**实际三者各自独立读关键帧目录,互不消费彼此输出**,是**替代/并列**关系;`filter_keyframes_front_fov` 也读**关键帧目录**(不是合并 pcd),逻辑上应在 build **之前**。

```
【路线 A · 快】录 bag(/cloud_registered,世界系)──► export_registered_cloud_map ──► raw pcd

【路线 B · 可纠漂移】
在线 keyframe_saver ──► 关键帧目录(poses_raw.txt + 每帧机体系 pcd)
   │  (可选·拼前)filter_keyframes_front_fov  : 读关键帧目录 → 按前视 FOV 筛出关键帧目录'
   │  (可选·拼前)scan_context_detector 回环检测 + pose_graph_optimizer 位姿图优化 → 校正位姿
   │
   ├─── 三个消费者各自独立读【关键帧目录】,互不消费彼此输出,三选一 ───┐
   │        build_raw_map ─────────────► raw_map.pcd                    (只拼,不去动态)
   │        dynamic_map_filter ────────► static_map_filtered.pcd + dynamic_removed.pcd
   │        sliding_window_static_filter ► static_map_sliding.pcd       (滑窗静态过滤)
   │                                                                     ▼
   │                                                          得到【单个合并 pcd】
   │        level_pcd(校平·读单个合并 pcd)──────────────────► 校平后的 pcd
   │        pcd_to_nav2_map(_fast)(读单个合并 pcd)──► nav2 占用栅格(见下)
```

- **`odom_to_tf_map*.py` 发布的是 `map → base_link` 的 TF(parent=`map`,child=`base_link`),不是原文写的 `odom→map`**;供 RViz/nav2 可视化对齐(`/Odometry` 是**输入 topic**,不是 TF 帧)。
  - 【默认 code:odom_to_tf_map.py:14-15/:36-37 parent=`map`/child=`base_link`;odom_to_tf_map_2d.py:28-29;odom_to_tf_map_level_2d.py:47-48/:90-91;level_2d 注释 :84 "Nav2 使用 2D 平面 TF"】
  - 【生产 helper:go2_start_level_scan.sh:50 `-p parent_frame:=map -p child_frame:=base_link`(仅 `_level_2d` 变体被此 helper 启动)】
  - 例外:未匹配通配的 `odom_to_tf_odom_level_2d.py` 的 parent 才是 `odom`(child 仍是 `base_link`,即 `odom→base_link`),且**无脚本启动**。
- **`level_cloud_node`(在线校平节点)校平角度**:代码默认 `pitch_deg=12.3`【默认 code:level_cloud_node.py:19】,但在生产 helper 里被覆盖为 `13.0`【生产 helper:go2_start_level_scan.sh:53 `-p pitch_deg:=13.0`】,与离线 `level_pcd`(默认 `pitch=13.0`)对齐 →**若跑 helper,狗上生效 13.0;但 helper 无狗上副本、非 saas 链,狗上是否用它无从确认** `【无狗上对照】`。
- nav2 相关配置在 `config/nav2_params.yaml`、`go2_amcl*.yaml`、`go2_slam_toolbox*.yaml`(说明也做过 nav2/amcl/slam_toolbox 路线,**非当前巡检主链**)。【默认 code:上述文件均存在;生产 saas:go2_saas_agent.py 无 amcl/slam_toolbox 编排,巡检走 fastlio】

### pcd → nav2 栅格(`pcd_to_nav2_map.py` / `_fast.py`,自研 `【无狗上对照】`)
- 栅格数值 `UNKNOWN=205 / FREE=254 / OCCUPIED=0`。【默认 code:pcd_to_nav2_map.py:10-12;pcd_to_nav2_map_fast.py:9-11 相同】
- **订正产物**:并非"一个 pcd",而是 **`.pgm`(P5)+ `.yaml`(`mode: trinary`,`occupied_thresh 0.65`/`free_thresh 0.25`)+ 3 个 debug pcd**(`_debug_free/_debug_occupied/_debug_unknown_observed`)。原文"占用栅格"表述正确,但"──► nav2 占用栅格(一个产物)"的措辞需按此细化。【默认 code:pcd_to_nav2_map.py:57/62 `write_pgm`(P5)、:309-321 `.pgm`+`.yaml`(trinary/0.65/0.25)、:323-325 debug pcd】

## pcd 文件格式
全是 **ASCII PCD v0.7**(`FIELDS x y z [intensity]`),关键帧保留 intensity,合并图只留 xyz。【默认 code:keyframe_saver.py:130 有 intensity 时 `FIELDS x y z intensity`、:144 否则 `x y z`;build_raw_map.py:52 / dynamic_map_filter.py:70 / sliding_window_static_filter.py:64 / export_registered_cloud_map.py:26 / level_pcd.py:36 / pcd_to_nav2_map.py:43 均 `FIELDS x y z` + `VERSION 0.7` + `DATA ascii`;filter_keyframes_front_fov.py:18-20 显式**拒绝非 ascii** PCD,佐证"全是 ASCII"】

## 工具口径:11 个 console_script vs 6 个未注册模块
- **`ros2 run` 能跑的正好 11 个**(`setup.py` console_scripts):`keyframe_saver / offline_keyframe_extractor / build_raw_map / scan_context_detector / pose_graph_optimizer / dynamic_map_filter / sliding_window_static_filter / export_registered_cloud_map / level_pcd / pcd_to_nav2_map / pcd_to_nav2_map_fast`。【默认 code:setup.py:22-34,恰 11 个】
- **另有 6 个未注册为 console_script 的模块**,只能 `python3 直跑`、**不能 `ros2 run`**:`level_cloud_node`、`filter_keyframes_front_fov`、`odom_to_tf_map`、`odom_to_tf_map_2d`、`odom_to_tf_map_level_2d`、`odom_to_tf_odom_level_2d`。原文在管线/角色处把这些也当"工具"列,口径上需注明它们不是 `ros2 run` 目标;其中 `odom_to_tf_map(.py/_2d/_odom_level_2d)` **无任何脚本启动 = 死代码**,仅 `odom_to_tf_map_level_2d` + `level_cloud_node` 被手动 helper 启动。

## 留待坐实
- `scan_context_detector`(回环检测,Scan Context 描述子:rings×sectors,bin 取最大高度)【默认 code:scan_context_detector.py:49-64】/ `pose_graph_optimizer`(scipy `least_squares` 的 SE(2) 位姿图 + 回环边)【默认 code:pose_graph_optimizer.py:6/:17-35/:61】—— 角色已核对无误,回环闭合**具体算法**按需再深读。
- `pcd_to_nav2_map` 的栅格化细节(已知产物 .pgm+.yaml,阈值 0.65/0.25)、`level_pcd` 的校平方法(绕 Y 轴 `pitch=13.0°`,`R=rot_z@rot_y@rot_x`,`pts_out=pts@R.T`)【默认 code:level_pcd.py:98-100/:118-119】。
- 实际现场用的是路线 A 还是 B(仓库无 saas 编排脚本,需问运维/看 `analysis` 里的建图记录)。
- **狗上一致性**:`go2_loop_backend` 整包 `【无狗上对照】`,狗上是否跑同一份自研代码**无从验证**;若要坐实,需在狗上抓这些文件的副本做 sha 对照。

## 核验台账
> claim → 证据 file:line → 判定。源标签见"核验状态"。自研文件均 `【无狗上对照】`,除注明外不再重复。

| # | claim | 证据(file:line) | 判定 |
|---|-------|------------------|------|
| 1 | 核心包 `go2_loop_backend` 有 11 个 ROS 可执行 | setup.py:22-34(console_scripts 恰 11) | **CONFIRMED**(另有 6 个未注册模块只能 python3 直跑) |
| 2 | 巡检 `pcd_save_en:false` → 地图非 FAST-LIO 自动存 | go2_mid360s.yaml:52=false;base_bringup.sh:220 起该配置;laserMapping.cpp:77 默认 false | **CONFIRMED**(狗上副本 :77 亦 false;但 laserMapping.cpp repo≠dog) |
| 3 | ~~没有任何脚本/SaaS 编排(grep 全空)~~ | go2_start_level_scan.sh:50/:53 编排 odom_to_tf_map_level_2d + level_cloud_node | **CORRECTED**:11 个建图可执行确无编排属实;但该 helper 编排了包内两节点,"grep 全空"字面为假;odom_to_tf_map(.py/_2d/_odom_level_2d)无启动=死代码 |
| 4 | keyframe_saver 订阅 /Odometry + /cloud_registered_body(机体系) | keyframe_saver.py:32-33/:56-57 | **CONFIRMED**(declare 默认,无 -p 覆盖) |
| 5 | 每移动 ≥1.0m 或转 ≥10° 存一关键帧 | keyframe_saver.py:35-36/:85 | **CONFIRMED**(and-return 实现"或";首帧无条件存) |
| 6 | 关键帧 = 机体系 pcd + 一行位姿(poses_raw.txt) | keyframe_saver.py:49/:94-99 | **CONFIRMED** |
| 7 | build_raw_map `pts_world=pts_body@R.T+t` 再 vstack | build_raw_map.py:92-94/:104 | **CONFIRMED** |
| 8 | build_raw_map voxel 0.10 → raw_map.pcd | build_raw_map.py:69/:68/:107 | **CONFIRMED** |
| 9 | 路线A:export 读 /cloud_registered(世界系)直接 vstack 无需变换 | export_...:42/:57/:108 | **CONFIRMED** |
| 10 | export 每 frame_stride=3 帧、point_stride=2 点,voxel 0.08 | export_...:44/:45/:46/:82/:90/:111 | **CONFIRMED** |
| 11 | dynamic_map_filter 逐帧变换,裁剪 z∈[-0.3,2.5] | dynamic_map_filter.py:130-131/:173/:178-179 | **CONFIRMED** |
| 12 | 去掉离机身 <0.5m 的点(用 XY 水平距离) | dynamic_map_filter.py:132/:176/:180 | **CONFIRMED** |
| 13 | 命中 ≥3 帧 → 静态保留 | dynamic_map_filter.py:134/:237 | **CONFIRMED**(hf=命中帧数) |
| 14 | 命中 ≥2 帧且 ≥4 稳定邻居 → 保留 | dynamic_map_filter.py:135/:136/:137/:234/:243 | **CONFIRMED** |
| 15 | free ≥ 5×命中 → 删 | dynamic_map_filter.py:143/:249 | **CONFIRMED**(仅 --raycast 开时生效,:139 默认关) |
| 16 | 输出 static_map_filtered.pcd + dynamic_removed.pcd | dynamic_map_filter.py:126-127/:261-262 | **CONFIRMED** |
| 17 | pcd_to_nav2_map → 占用栅格 UNKNOWN205/FREE254/OCCUPIED0 | pcd_to_nav2_map.py:10-12;_fast.py:9-11 | **CONFIRMED**(产物实为 .pgm+.yaml+debug pcd,非单个 pcd:pcd_to_nav2_map.py:309-325) |
| 18 | ~~odom_to_tf_map*.py 发 odom→map 的 TF~~ | odom_to_tf_map.py:14-15/:36-37;_2d.py:28-29;_level_2d.py:47-48/:90-91;helper go2_start_level_scan.sh:50 | **CORRECTED**:发布 `map→base_link`(parent=map/child=base_link),无 odom 帧;原文帧向写反且略去 child |
| 19 | nav2 配置在 nav2_params/go2_amcl*/go2_slam_toolbox*(非主链) | config/ 下 5 文件均在;go2_saas_agent.py 无 amcl/slam_toolbox 编排 | **CONFIRMED** |
| 20 | 全 ASCII PCD v0.7,关键帧留 intensity、合并图只 xyz | keyframe_saver.py:130/:144;build_raw_map.py:52 等;filter_keyframes_front_fov.py:18-20 拒非 ascii | **CONFIRMED** |
| 21 | ~~路线B:build→dynamic→sliding→filter/level 串行~~ | dynamic_map_filter.py:125/:147;sliding_window_static_filter.py:114/:132;filter_keyframes_front_fov.py:115-116;level_pcd.py:94-95 | **CORRECTED**:build_raw_map/dynamic_map_filter/sliding_window_static_filter 各自独立读关键帧目录、互不消费彼此输出(并列三选一);filter_keyframes_front_fov 读关键帧目录应在 build 之前;level_pcd 读单个合并 pcd |
| 22 | scan_context_detector = 回环检测 | scan_context_detector.py:49-64 | **CONFIRMED**(角色核对无误) |
| 23 | pose_graph_optimizer = 位姿图优化 | pose_graph_optimizer.py:6/:17-35/:61 | **CONFIRMED** |
| 24 | level_pcd = 校平 | level_pcd.py:98-100(pitch 13.0)/:118-119 | **CONFIRMED**(在线版 level_cloud_node 代码默认 pitch 12.3,helper 覆盖 13.0) |
| 25 | 产物 pcd 供 route_relocalizer 重定位(见07)/转 nav2 栅格 | "转栅格"已证;relocalize:go2_saas_agent.py:629/678/:570;狗上 manifest localization_mode=manual_anchor | **UNVERIFIABLE**:"转栅格"成立;"route_relocalizer/与路线同名"属 doc07,狗上 manifest 为 manual_anchor,留待 doc07 核 |

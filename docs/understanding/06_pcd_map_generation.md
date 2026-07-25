# 06 · PCD 地图怎么生成(打点 → 拼合 → 去动态)

> 原则同 00。核心包:`src/go2_loop_backend`(11 个可执行,离线建图/回环工具集)。
> **已逐行读**:`keyframe_saver.py`、`export_registered_cloud_map.py`、`build_raw_map.py`、`dynamic_map_filter.py`。
> **按名/头部推断角色、待按需深读**:`sliding_window_static_filter`、`scan_context_detector`、`pose_graph_optimizer`、`level_pcd`/`level_cloud_node`、`pcd_to_nav2_map(_fast)`、`filter_keyframes_front_fov`、`odom_to_tf_*`。

## 关键前提
- FAST-LIO 巡检配置 `pcd_save_en:false`(见 01)→ **地图不是 FAST-LIO 自动存的**。
- ⚠️ **没有任何脚本/SaaS 编排这些工具**(grep 全空)→ pcd 建图是**离线、手动**跑命令的流程,不在自动巡检链里。
- 产物 pcd(与路线同名)供巡检前 `route_relocalizer` 重定位用(见 07),或转 nav2 栅格。

## "打点拼合"到底怎么做(回答你的问题:是的,就是不停打点然后拼)

### 造原始地图有两条路

**路线 A:直接从 rosbag 累积**(`export_registered_cloud_map.py`)
- 录一个含 **`/cloud_registered`** 的 rosbag(FAST-LIO 输出的**世界系**已配准点云)。
- 从 bag 的 sqlite 里逐帧读(每 `frame_stride=3` 帧、每 `point_stride=2` 点)。
- **因为 `/cloud_registered` 已经是世界系**,直接 `np.vstack` **拼**起来即可(无需再变换),再 `voxel_downsample(0.08)` 合并重复点 → 写 pcd。
- 特点:最简单,**依赖 FAST-LIO 里程计,不纠漂移**。

**路线 B:关键帧 + 位姿变换拼合**(`keyframe_saver.py` → `build_raw_map.py`)
- `keyframe_saver`(在线节点):订阅 `/Odometry` + **`/cloud_registered_body`**(机体系),**每移动 ≥1.0m 或转 ≥10°** 存一个关键帧 = 一个机体系 pcd + 一行位姿(`poses_raw.txt`:idx stamp x y z qx qy qz qw yaw 文件名)。
- `build_raw_map`(离线):逐关键帧 `pts_world = pts_body @ R(q).T + t` **把每帧点云按其位姿变换到世界系再 vstack 拼**,`voxel_downsample(0.10)` → `raw_map.pcd`。
- 特点:关键帧+位姿可先做**回环/位姿图优化**再拼,**能纠漂移**。

### 去动态(拼完的清洗,`dynamic_map_filter.py`)
把"人、临时障碍"等动点从地图里删掉:
- 逐关键帧变换到世界系,裁剪 `z∈[-0.3,2.5]`、去掉离机身 <0.5m 的点(狗自己的腿)。
- 体素化,统计每个体素:被多少帧命中(`hit_frames`)、质心;可选**光线投射**把"传感器到命中点"沿途体素标记为 free。
- 判定:
  - 命中 **≥3 帧** → 高置信**静态**,保留;
  - 命中 **≥2 帧且周围 ≥4 个稳定邻居** → 静态,保留;
  - 光线 free 反证强(free ≥ 5×命中)→ **删**(动态/穿透);
  - 单帧/弱支持 → **删**。
- 输出 `static_map_filtered.pcd`(干净静态图)+ `dynamic_removed.pcd`(被删的动点)。

## 完整离线管线(手动,按需组合)
```
录 bag(/cloud_registered)──► export_registered_cloud_map ──► raw pcd        (路线A, 快)
        或
在线 keyframe_saver(存机体系关键帧+位姿) ──► [scan_context 回环检测 + pose_graph 位姿优化]
        ──► build_raw_map(按位姿拼) ──► dynamic_map_filter(去动态) ──► sliding_window_static_filter
        ──► filter_keyframes_front_fov / level_pcd(校平) ──► 干净 pcd     (路线B, 可纠漂移)
                                                        └► pcd_to_nav2_map ──► nav2 占用栅格(UNKNOWN205/FREE254/OCCUPIED0)
```
- `odom_to_tf_map*.py`:发 `odom→map` 的 TF,供 RViz/nav2 可视化对齐。
- nav2 相关配置在 `config/nav2_params.yaml`、`go2_amcl*.yaml`、`go2_slam_toolbox*.yaml`(说明也做过 nav2/amcl/slam_toolbox 路线,非当前巡检主链)。

## pcd 文件格式
全是 **ASCII PCD v0.7**(`FIELDS x y z [intensity]`),关键帧保留 intensity,合并图只留 xyz。

## 留待坐实
- `scan_context_detector` / `pose_graph_optimizer` 的回环闭合具体算法(按需深读)。
- `pcd_to_nav2_map` 的栅格化细节、`level_pcd` 的校平方法。
- 实际现场用的是路线 A 还是 B(仓库无编排脚本,需问运维/看 `analysis` 里的建图记录)。

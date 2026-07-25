# 05 · 路线录制:一条巡检路线怎么录出来

> 原则同 00。核心文件:`src/go2_fastlio_patrol/go2_fastlio_patrol/route_recorder.py`(节点 `route_recorder`)。

## 一句话
**人工遥控**狗沿目标路线走一遍,`route_recorder` 订阅 FAST-LIO 的 `/Odometry`,**每走过 `min_distance` 就落一个点**,写成 `id,x,y,yaw,v` 的 CSV。

## 逐行逻辑(`odom_callback:54`)
1. 订阅 `/Odometry`;每帧取 `x, y`,`yaw = yaw_from_quaternion(...)`。
2. **按距离采样**:第一帧必存;之后只有距上一个存点位移 `≥ min_distance`(默认 **0.4m**)才存。→ 天然按空间均匀打点,和走多快无关。
3. 写一行 CSV:`[id, x(.6f), y(.6f), yaw(.6f), v]`,其中 **`v` 恒为 `default_speed`(默认 0.20)** 的常数占位;`flush` 落盘。
4. `Ctrl+C` 停止 → 关文件。

## 几个要点(和跟随器对上)
- CSV 表头:`id, x, y, yaw, v`。跟随器 `load_route` 按列名读 `x/y/yaw/v`(`id` 不读)。
- ⚠️ **`yaw` 和 `v` 录了但生产跟随器基本不用**:`yaw` 来自里程计四元数、`v` 是常数 0.20;而 `waypoint_follower_go2_2` 用相邻点几何算朝向、用参数定速度(见 02)。→ 这两列是历史/其它跟随器的遗留字段。
- 坐标系 = **FAST-LIO 局部系**(以开机静止初始化点为原点)。所以 README 死磕"固定起点固定朝向":录制与回放必须同一起点,否则坐标系对不上会偏移/反向(除非走生产链的 route_relocalizer 重定位,见 07)。
- README §9 建议:走道路中间、0.2~0.5m/s、转弯放慢、事后手工删抖动点。

## 相关(非本节,后续)
- `tools/build_bounded_route.py` / `build_piecewise_route.py` / `route_quality.py`:录后对路线做质量评估/裁剪/分段处理(见后续"路线处理"或审计章)。
- 生产里路线可由云端下发下载(`go2_saas_agent.py download_route_csv`,见 08)。

# 07 · 巡检前重定位:把录制路线对齐到当前坐标系

> 原则同 00。核心文件:`src/go2_map_manager/src/route_relocalizer.cpp`(C++ 一次性节点,PCL + ICP)。
> 生产在巡检前由 SaaS 调用(`go2_saas_agent.py:1989`),输出 `relocalized.csv` 给跟随器用。

## 一、它解决什么问题
录路线时的 FAST-LIO 坐标系(以录制那次开机点为原点)和**这次开机**的坐标系不一样。`route_relocalizer` 用**预建的 pcd 地图**做桥梁,把录制路线**刚性变换**到当前坐标系。
> ⚠️ **结论**:有了它,生产链**不严格要求"固定起点固定朝向"**(README 那套是无重定位的测试链要求)。但前提是**存在同名 pcd 地图**;没有地图就退回固定起点。

## 二、流程(`main`,逐段)
1. 参数:`route_file`、`map_file`(.pcd)、`out_route_file`(默认 `/tmp/go2_route_runtime/relocalized.csv`)、`cloud_topic`(`/cloud_registered`,`cloud_in_body_frame` 可选)、`odom_topic`。
2. 读路线**首点** `(x,y,yaw)`;加载地图 pcd。
3. **采一段当前扫描**:订阅点云 + `/Odometry` 累积 `collect_seconds`(3s);若是机体系则用当前位姿转到 odom 系。要求 ≥`min_points`(200)、≥`min_cloud_frames`(8)。
   - ⚠️ **采集期间狗必须静止**:位移 ≤`max_capture_translation`(0.10m)、转角 ≤`max_capture_yaw_deg`(5°),否则 `RELOCALIZE_FAILED robot moved`。
4. 过滤:地图/扫描各做 **fine**(体素 0.35/0.25)和 **coarse**(体素 0.60、≤2500 点)两版,裁 `z∈[-1.5,2.5]`,扫描裁 35m 内。
5. **粗搜索(多假设 ICP)**:`base_yaw = 路线首点yaw - 当前yaw`;在 **yaw(±180°步30°)× xy(±5m步2m)** 网格上,每个初值跑一次 coarse ICP(scan→map),收集所有收敛候选,取 fitness 最低者为 `best`。
6. **歧义候选**:在候选里找一个与 best "实质不同"(≥1m 或 ≥20°)的 `alternate`。
7. **精修**:从 best 初值跑 fine ICP;alternate 同样精修;更优则替换。
8. **质量闸(任一不过就失败)**:
   - `best.fitness > fitness_threshold`(0.12)→ 失败(对不齐)。
   - **歧义**:alternate 与 best 实质不同,且 `alternate.fitness ≤ best.fitness×1.20`(两个差很远的位姿拟合得几乎一样好)→ 失败(不敢选)。
9. 可选 `anchor_route_start`:把"当前位姿映射到地图后"强制钉到路线首点(带残差/yaw 阈值校验)。
10. **变换整条路线**:`t_current_from_old = inverse(t_old_from_current)`,对每个路点 `p_new = T·p_old`、`yaw += yaw_delta`,写 `relocalized.csv`;同时写 `.relocalize.json`(fitness、变换矩阵、scan/map 点数等)。
11. 打印 `RELOCALIZE_OK ...` 或 `RELOCALIZE_FAILED ...`(返回码 2 参数/文件错、3 数据不足/动了、4 ICP 不收敛/fitness 差/歧义)。

## 三、和整链的衔接
```
SaaS start_patrol ─► route_relocalizer(route.csv × map.pcd × 当前扫描)
     ├─ 成功 ─► /tmp/go2_route_runtime/relocalized.csv ─► waypoint_follower_go2_2 跟随
     └─ 失败(码2/3/4)─► saas 据此决定是否中止/回退(见 08)
```

## 四、要点
- 用的是 **PCL 的 IterativeClosestPoint**;粗搜索本质是"多起点全局配准",避免 ICP 陷入局部最优。
- **歧义拒绝**很关键:走廊/对称环境里两个位姿都能对上时,它宁可失败也不乱选(防止路线整体镜像/平移到错误分支)。
- `submap_builder.cpp`(同包,未深读)推测用于建/切子图,按需再看。

## 五、留待坐实
- 生产实际传的 `cloud_in_body_frame` 和各阈值(看 08 的 `route_relocalization_plan` / 启动串)。
- `submap_builder.cpp` 的职责。

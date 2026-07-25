# 07 · 巡检前重定位:把录制路线对齐到当前坐标系

> 原则同 00。核心文件:`orin_go2_fastlio_ws/src/go2_map_manager/src/route_relocalizer.cpp`(C++ 一次性节点,PCL + ICP)。
>
> ⛔ **前提更正(读全文前先看):`route_relocalizer`(pcd/ICP 重定位)不是生产默认路径,而是显式可选模式。** 默认模式是 `manual_anchor`(操作员把狗摆到路线物理起点,临时路线刚性变换进当前 FAST-LIO 会话)。【生产 go2_saas_agent.py:663】环境变量 `GO2_PATROL_LOCALIZATION_MODE` 缺省 `manual_anchor`、【go2_saas_agent.py:3100】CLI default 亦 `manual_anchor`、【docstring go2_saas_agent.py:603-607】明言 "PCD/ICP remains an explicit mode"。**两次实测狗跑(manifest 06/07)`localization_mode=manual_anchor`,`route_relocalizer` 根本没执行。** 本文以下对该节点的描述属"代码可达路径 + 显式 pcd 模式启动串"分析,而非观测到的实况。

---

## 〇、核验状态

- **本轮已核**:对磁盘源码 `route_relocalizer.cpp`(:462~1002)、`go2_saas_agent.py`(重定位相关段 :600~2590)、`submap_builder.cpp` 逐条比对;并读取两份实测狗跑 manifest(06/07)作为运行期真相。原文的**流程/质量闸/歧义逻辑/返回码/数据流**描述与源码大体吻合,保留。
- **系统性问题(已修)**:原文把 `route_relocalizer.cpp` 的 `declare_parameter` **代码默认值当成生产权威值**陈述,但 pcd 模式生产靠 `-p` 启动串【go2_saas_agent.py:1990-1998】覆盖了约 12 个参数(见第二节表)。本轮对每个可证伪数值补了【源标签】,默认≠生产的两个都写,并注明狗上实际走哪个。
- **仍无法验证**:三个源文件**均无狗上对照副本**(`previous_boot/remote_source/` 只有 4 份:laserMapping / lddc / lds / waypoint_follower_go2_2),repo↔狗一致性**未验**。sha256 见文末台账。
- **源标签图例**:
  - 【默认 cpp:行】= `route_relocalizer.cpp` 的 `declare_parameter` 默认(**纯代码值,不等于狗上生效**)。
  - 【生产 saas:行】= `go2_saas_agent.py` **pcd 模式** `-p` 启动串实际下发(**仅当显式选 pcd 模式才生效**)。
  - 【狗上 manifest:证据】= 实测狗跑运行期真相文件(该文件 repo==dog)。
  - 【README对照】/【推断-未验】。

---

## 一、它解决什么问题
录路线时的 FAST-LIO 坐标系(以录制那次开机点为原点)和**这次开机**的坐标系不一样。`route_relocalizer` 用**预建的 pcd 地图**做桥梁,把录制路线**刚性变换**到当前坐标系。

> ⚠️ **更正原"结论"**:
> - 原文说"有了它,生产链不严格要求固定起点固定朝向"——**只在显式选 pcd 模式时成立,且非观测实况**。生产**默认** `manual_anchor` 本身就是"把狗摆到物理起点"的固定起点式方案【生产 go2_saas_agent.py:663】,并非被 pcd 重定位取代。
> - 原文说"没有地图就退回固定起点"——**与代码相反**。pcd 模式若无同名 pcd 地图 → 抛 `RuntimeError`【go2_saas_agent.py:774】→ `run_start_patrol` 直接返回 `rejected`【go2_saas_agent.py:2565-2576,尤其 :2575】,**硬拒绝,无任何 try/except 把缺图自动降级为 manual_anchor**。"退回固定起点"是原文臆测,不存在该兜底。

---

## 二、流程(`main`,逐段)

> ⚠️ 下表参数,凡"默认≠生产"的,**pcd 模式生效的是"生产"列**;`manual_anchor` 模式(实测两次狗跑)该节点不运行,故任一列都不上狗。

| 参数 | 默认(cpp) | 生产 pcd 模式(saas `-p`) | 备注 |
|---|---|---|---|
| `out_route_file` | `/tmp/go2_route_runtime/relocalized.csv`【cpp:470】 | `<patrolRunDir>/route_runtime.csv`【saas:1659,1990】 | 见第三节;**文件名生产为 route_runtime.csv** |
| `cloud_topic` | `/cloud_registered`【cpp:471】 | `/cloud_registered_body`【saas:1991,硬编码】 | 生产订阅 **body 系**话题 |
| `cloud_in_body_frame` | `false`【cpp:472】 | `true`【saas:1991,硬编码】 | 生产恒 true→必走 body→odom 变换 |
| `odom_topic` | `/Odometry`【cpp:473】 | `/Odometry`【saas:1991】 | 一致 |
| `collect_seconds` | `3.0`【cpp:474】 | `3.0`【saas:1934-1939,云端可下发,范围[1,10]】 | 值一致 |
| `min_points` | `200`【cpp:475】 | **`800`**【saas:1992,硬编码】 | 两处检查【cpp:694 原始 / :772-775 过滤后】都用它 |
| `min_cloud_frames` | `8`【cpp:492】 | `8`【saas:1992】 | 一致 |
| `min_z`/`max_z` | `-1.5`/`2.5`【cpp:476-477】 | 未覆盖→同默认 | 地图与扫描两侧都裁【cpp:213-216】 |
| `max_scan_range` | `35.0`【cpp:478】 | 未覆盖→同默认 | **仅扫描侧**按半径裁【cpp:747/767 use_center=true】,地图侧不裁 |
| `map_voxel_leaf`/`scan_voxel_leaf` | `0.35`/`0.25`【cpp:479-480】 | 未覆盖→同默认 | fine 两版 |
| `coarse_voxel_leaf` | `0.60`【cpp:481】 | 未覆盖→同默认 | |
| `coarse_max_points` | `2500`【cpp:482】 | 未覆盖→同默认 | **仅作用于 scan_coarse**【cpp:770】;map_coarse 传 0=**不限点**【cpp:760】 |
| `yaw_search_deg` | `180.0`【cpp:483】 | **`30.0`**【saas:1995,硬编码】 | 粗搜索网格大幅收窄 |
| `yaw_step_deg` | `30.0`【cpp:484】 | **`10.0`**【saas:1952-1957,云端可下发,范围[5,20]】 | |
| `xy_search_radius` | `5.0`【cpp:485】 | **`0.75`**【saas:1940-1945,云端可下发,范围[0.3,1.0]】 | |
| `xy_search_step` | `2.0`【cpp:486】 | **`0.5`**【saas:1946-1951,云端可下发,范围[0.2,1.0]】 | |
| `fitness_threshold` | `0.12`【cpp:491】 | **`0.23`**【saas:1958-1963,云端可下发,范围[0.02,0.30]】 | best.fitness>阈值→码4 |
| `ambiguity_fitness_ratio` | `1.20`【cpp:495】 | `1.20`【saas:1996】 | 一致 |
| `ambiguity_min_translation` | `1.00`【cpp:496】 | `1.00`【saas:1996】 | 一致 |
| `ambiguity_min_yaw_deg` | `20.0`【cpp:497】 | `20.0`【saas:1997】 | 一致 |
| `max_capture_translation` | `0.10`【cpp:493】 | `0.10`【saas:1993】 | 一致 |
| `max_capture_yaw_deg` | `5.0`【cpp:494】 | `5.0`【saas:1993】 | 一致 |
| `anchor_route_start` | **`false`**【cpp:498】 | **`true`**【saas:1964-1968,云端可下发】 | pcd 生产**默认开启** |
| `max_start_anchor_residual` | **`0.0`**【cpp:499】 | **`0.75`**【saas:1969-1977,范围[0.10,0.80]】 | ⚠️ 见第九步自锁陷阱 |
| `max_start_anchor_yaw_deg` | **`0.0`**【cpp:500】 | **`20.0`**【saas:1978-1986,范围[5,30]】 | 同上 |

> 🔧 **硬编码 vs 云端可下发**(标准#3):`cloud_topic`/`cloud_in_body_frame`/`odom_topic`/`min_points=800`/`min_cloud_frames=8`/`max_capture_*`/`yaw_search_deg=30`/`ambiguity_*` 是 `-p` 串里**写死的字面量**,云端下发同名参数**不生效**;`collect_seconds`/`xy_search_radius`/`xy_search_step`/`yaw_step_deg`/`fitness_threshold`/`anchor_route_start`/`start_anchor_max_*` 才是 `bounded_float`/`bool_param` **云端可覆盖**(上表给的是其缺省)。

流程本体(逻辑与原文一致,补源标签):

1. 参数:`route_file`、`map_file`(.pcd)、`out_route_file`、`cloud_topic`(`cloud_in_body_frame`)、`odom_topic`(取值见上表)。
2. 读路线**首点** `(x,y,yaw)`;加载地图 pcd。
3. **采一段当前扫描**:订阅点云 + `/Odometry` 累积 `collect_seconds`;若是机体系则用当前位姿转到 odom 系【cpp:552-560,`current_orientation*point_body + current_pos`】(**生产 `cloud_in_body_frame=true` 故此路径必生效**)。要求 ≥`min_points`(默认200/**生产800**)、≥`min_cloud_frames`(8)。
   - ⚠️ **采集期间狗必须静止**:位移 ≤`max_capture_translation`(0.10m)、转角 ≤`max_capture_yaw_deg`(5°),否则失败。判据是 **>(严格大于)** 才算动【cpp:719-728】,即成功需 ≤;实际错误串为 `RELOCALIZE_FAILED robot moved during capture`【cpp:722】(原文截断为 "robot moved")。
4. 过滤:地图/扫描各做 **fine**(体素 0.35/0.25)和 **coarse**(体素 0.60)两版,裁 `z∈[-1.5,2.5]`,**扫描裁 35m 内(地图侧不裁)**。⚠️ 原文"coarse ≤2500 点"**只对扫描侧成立**;地图侧 `map_coarse` 传 0=**不限点**【cpp:760】。
5. **粗搜索(多假设 ICP)**:`base_yaw = 路线首点yaw - 当前yaw`【cpp:787】;在 **yaw × xy** 网格【cpp:794-810】上每个初值跑一次 coarse ICP(scan→map)【cpp:815-830,`run_icp(scan_coarse, map_coarse)`】,收集所有收敛候选,取 fitness 最低者为 `best`。
   - 网格尺寸**默认 yaw ±180°步30° × xy ±5m步2m**【cpp:483-486】,**生产 pcd 收窄为 yaw ±30°步10° × xy ±0.75m步0.5m**【saas:1940-1957,1994-1995】。搜索假设数从(约 13×6×6≈468)缩到(约 7×4×4≈112)。
6. **歧义候选**:在候选里找一个与 best "实质不同"(≥1m 或 ≥20°,OR 逻辑)的 `alternate`【cpp:65-73,849-860】。
7. **精修**:从 best 初值跑 fine ICP,更优则替换;alternate 同样精修,更优则 swap【cpp:862-893】。
8. **质量闸(任一不过就失败,返回码 4)**:
   - `best.fitness > fitness_threshold`(默认**0.12**/生产**0.23**)→ 失败【cpp:895-905,判据 :904】。
   - **歧义**:alternate 与 best 实质不同,且 `alternate.fitness ≤ best.fitness×1.20`(两个差很远的位姿拟合得几乎一样好)→ 失败【cpp:906-924,判据 :923】。
9. 可选 `anchor_route_start`(默认 **false**/**生产 pcd 默认 true**):把"当前位姿映射到地图后"算残差/yaw 误差,过阈值校验后**平移钉到路线首点**【cpp:930-959】。
   - ⚠️ **自锁陷阱(原文未提)**:纯代码默认 `max_start_anchor_residual/yaw = 0.0`,会命中【cpp:944-945】使 anchor **永远失败**;该特性**只在生产阈值(0.75 / 20.0°)下才可用**。
   - ⚠️ **返回码 4 还含 start-anchor 残差/yaw 校验失败**【cpp:954】,原文第 11 步漏列;因**生产 anchor 默认开启**,此失因实际相关。
10. **变换整条路线**:`t_current_from_old = inverse(t_old_from_current)`【cpp:960】,对每个路点 `p_new = T·p_old`【cpp:338】、`yaw += yaw_delta`【cpp:301,346】,写出 CSV;同时写 `.relocalize.json`(fitness【cpp:398】、变换矩阵 4×4【cpp:412-424】、scan/map 点数【cpp:403-404】等)。
    - ⚠️ **输出 CSV 文件名**:默认 `relocalized.csv`【cpp:470】,**生产实际为 `route_runtime.csv`**【saas:1659,1990;manifest `runtime_route=route_runtime.csv`】(见第三节)。
11. 打印 `RELOCALIZE_OK ...` / `RELOCALIZE_FAILED ...`【cpp:984】。返回码:
    - **2**(参数/文件错):缺 route/map【cpp:647】、读路线失败【:656】、载图失败【:664】、写出失败【:965】。
    - **3**(数据不足/动了):无里程【:692】、点太少【:699】、帧太少【:706】、缺采集位姿【:712】、动了【:728】。
    - **4**(ICP 不收敛/fitness 差/歧义/**start-anchor 校验失败**):不收敛【:840】、fitness 超阈【:904】、歧义【:923】、start-anchor 残差/yaw 失败【:954】。

---

## 三、和整链的衔接

```
SaaS start_patrol_command(:1475)
  └─(仅当显式 pcd 模式,elif pcdRelocalization :1933)
     ─► route_relocalizer(route.csv × map.pcd × 当前扫描)
        ├─ 成功 ─► <patrolRunDir>/route_runtime.csv ─► waypoint_follower_go2_2 跟随
        └─ 失败(码2/3/4)─► rc!=0→status failed + 目录 finalization(:2577-2590),saas 据此中止(无自动降级/回退)
  └─(默认 manual_anchor 模式,:1918-1932)
     ─► manual_anchor_script(不含 route_relocalizer)─► 同样写 route_runtime.csv ─► 跟随器
```

更正原衔接图:
- 函数名是 **`start_patrol_command`**【saas:1475】(原文写 `start_patrol`)。
- 成功输出路径生产为 **`<patrolRunDir>/route_runtime.csv`**【saas:1659 `effective_route_path`,经 route_arg 传入 :1990 第 6 个 `%s`;manifest `runtime_route=route_runtime.csv`】,**不是** `/tmp/go2_route_runtime/relocalized.csv`。该 `/tmp` 路径只在 plan 里算过【saas:781】随即被 :1660 覆盖。
- **数据流"给跟随器用"成立**:`follower_route_arg = route_arg = 同一路径`【saas:1668,2074】,跟随器确读该 CSV。
- ⚠️ **注意**:实测两次狗跑的 `route_runtime.csv` 是 **manual_anchor 分支**产出的(`effective_route_path` 在 :1659 与模式无关地设好),**并非 route_relocalizer 写的**——两条分支恰好共用同一运行期文件名。
- 失败无"回退":`rc!=0` → `status failed` + 目录 finalization【saas:2577-2590】,"saas 据此中止"合理;**不存在自动降级到 manual_anchor 的机制**。

---

## 四、要点
- 用的是 **PCL 的 `IterativeClosestPoint`**【cpp:445】;粗搜索本质是"多起点全局配准"(三重循环 yaw×dx×dy 对每个初值跑 ICP【cpp:794-834】),避免陷入局部最优。
- **歧义拒绝**很关键:走廊/对称环境里两个位姿都能对上时,它宁可失败也不乱选(防止路线整体镜像/平移到错误分支)。
- 该节点是**一次性节点**:`RelocalizerNode`【cpp:462】在 `main`【cpp:638】中跑一次即 `rclcpp::shutdown`【cpp:1002】。
- `submap_builder.cpp`(同包)推测用于建/切子图:订阅 `/cloud_registered`+`/Odometry` 累积 `PointXYZI` 子图,每移动 `save_distance_threshold_=10.0m` 存一块到 `~/maps` 并写 `trajectory.txt`,析构时再存当前块【submap_builder.cpp:18-62】。"建子图"基本正确("切"为按距离分块)。**是否在生产链/狗上运行,未确认**【推断-未验】。

---

## 五、留待坐实 → 已坐实(本轮)
- ✅ 生产实际传的 `cloud_in_body_frame`(=true)与各阈值:已由 pcd 模式 `-p` 启动串坐实,见第二节表【saas:1990-1998】。
- ⬜ `submap_builder.cpp` 是否上狗运行、repo↔狗是否一致:**仍未验**(无狗上副本)。

---

## 六、狗上对照状态(标准#4)

| 文件 | 狗上状态 | 说明 |
|---|---|---|
| `route_relocalizer.cpp` | 【无狗上对照】 | `remote_source/` 无此副本,repo↔狗一致性**未验**。sha256=`17f6dc51…`。**运行期佐证:manifest 06/07 均 manual_anchor,该 pcd 节点两次实测狗跑均未执行。** |
| `go2_saas_agent.py` | 【无狗上对照】 | 无狗上副本,无法逐字核。sha256=`79158fb3…`。**manifest 侧面佐证:狗上确用默认 manual_anchor、输出 route_runtime.csv,与仓库 saas 默认逻辑吻合。** |
| `submap_builder.cpp` | 【无狗上对照】 | 无狗上副本,repo↔狗一致或是否运行均**未验**。sha256=`9856eabd…`。 |
| `runs/xunjian-20260725-06/manifest.txt` | **repo==dog(运行期真相)** | `localization_mode=manual_anchor`、`runtime_route=route_runtime.csv`。**直接证明 route_relocalizer(pcd)未跑、输出名非 relocalized.csv。** |
| `runs/xunjian-20260725-07/manifest.txt` | **repo==dog(运行期真相)** | 同 06:`localization_mode=manual_anchor`。 |

---

## 七、核验台账(claim → 证据 → 判定)

> 台账已对真源码逐条核过。C=CONFIRMED,COR=CORRECTED,D/P=DEFAULT_VS_PROD。

| # | 断言(原文) | 证据 file:line | 判定 |
|---|---|---|---|
| 1 | 核心文件 route_relocalizer.cpp,C++ 一次性节点,PCL+ICP | cpp:462,638,445,1002 | **C** |
| 2 | 生产在巡检前由 SaaS 调用(:1989) | saas:1933,1989,663,3100;docstring:603-607;manifest | **COR** 仅 pcd 分支;默认 manual_anchor,实测未跑 |
| 3 | 输出 relocalized.csv 给跟随器 | cpp:470;saas:1659,1668,1990,2074 | **D/P** 生产=route_runtime.csv;数据流成立 |
| 4 | 不严格要求固定起点;没地图退回固定起点 | saas:772-778,2565-2576 | **COR** 缺图→RuntimeError→rejected,无降级;manual_anchor 本就是默认 |
| 5 | out_route_file 默认 /tmp/…/relocalized.csv | cpp:470 | **C**(但生产 -p 覆盖) |
| 6 | cloud_topic(/cloud_registered)、cloud_in_body_frame 可选 | cpp:471-472;saas:1991 | **D/P** 生产=/cloud_registered_body、true |
| 7 | odom_topic(/Odometry) | cpp:473;saas:1991 | **C** |
| 8 | 累积 collect_seconds(3s) | cpp:474;saas:1934-1939 | **C** |
| 9 | 要求 ≥min_points(200) | cpp:475,694,772-775;saas:1992 | **D/P** 生产=800 |
| 10 | 要求 ≥min_cloud_frames(8) | cpp:492,701-707;saas:1992 | **C** |
| 11 | 机体系用当前位姿转 odom | cpp:552-560 | **C**(生产 true 故生效) |
| 12 | 位移≤0.10/转角≤5°,否则 robot moved | cpp:493-494,719-728;saas:1993 | **C**(错误串截断) |
| 13 | fine 体素 0.35/0.25 | cpp:479-480,731-750 | **C** |
| 14 | coarse 体素 0.60、≤2500 点 | cpp:481-482,751-770 | **C** 但 2500 仅 scan;map 侧不限点(:760) |
| 15 | 裁 z∈[-1.5,2.5] | cpp:476-477,213-216 | **C** |
| 16 | 扫描裁 35m 内 | cpp:478,741-770,217-224 | **C** 仅扫描侧 |
| 17 | base_yaw=首点yaw-当前yaw | cpp:787 | **C** |
| 18 | 粗搜索网格 yaw±180步30 × xy±5m步2m | cpp:483-486,794-810;saas:1994-1995,1940-1957 | **D/P** 生产=±30步10 × ±0.75步0.5 |
| 19 | 每初值 coarse ICP(scan→map),取最小 fitness | cpp:815-830 | **C** |
| 20 | 歧义候选 ≥1m 或 ≥20° | cpp:496-497,65-73,849-860;saas:1996-1997 | **C** |
| 21 | 精修 best/alternate,更优则替换 | cpp:862-893 | **C** |
| 22 | best.fitness>fitness_threshold(0.12)→失败 | cpp:491,895-905;saas:1958-1963,1995 | **D/P** 生产=0.23 |
| 23 | 歧义 alternate.fitness ≤ best×1.20 → 失败 | cpp:495,906-924;saas:1996 | **C** |
| 24 | 可选 anchor_route_start(带残差/yaw 校验) | cpp:498-500,930-959;saas:1964-1986,1997-1998 | **D/P** 生产默认 true(0.75/20.0);默认 0.0 自锁 |
| 25 | 变换整条路线;写 CSV + .relocalize.json | cpp:960,301,338,346,386-424 | **C**(CSV 名生产 route_runtime.csv) |
| 26 | 打印 OK/FAILED;码 2/3/4 | cpp:984,647,656,664,965,692,699,706,712,728,840,904,923,954 | **C** 码4 另含 start-anchor 失败(:954) |
| 27 | 衔接图 start_patrol→…→relocalized.csv→跟随;失败中止/回退 | saas:1475,1659-1660,1668,2074,2577-2590,1990,2008-2009 | **COR** 函数=start_patrol_command;输出=route_runtime.csv;无自动回退 |
| 28 | PCL ICP;粗搜索=多起点全局配准 | cpp:445,794-834 | **C** |
| 29 | submap_builder 建/切子图 | submap_builder.cpp:18-57,62 | **C**(是否上狗运行未验) |
| 30 | 留待坐实:cloud_in_body_frame 与各阈值 | saas:1990-1998 | **C** 已坐实(见第二节) |

# 05 · 路线录制:一条巡检路线怎么录出来

> 原则同 00。核心文件:`orin_go2_fastlio_ws/src/go2_fastlio_patrol/go2_fastlio_patrol/route_recorder.py`(节点 `route_recorder`,`setup.py:23` entry_point `route_recorder = go2_fastlio_patrol.route_recorder:main`)。
> ⚠️ 原文路径写成 `src/go2_fastlio_patrol/...`,漏了工作区前缀 `orin_go2_fastlio_ws/`,从仓库根按字面不可解析——已补全(与本套文档统一惯例)。

## 核验状态
本轮已对**磁盘仓库源码逐行核对**:`route_recorder.py` 的逐行逻辑、CSV 格式、参数默认值、跟随器忽略 CSV 的 `yaw/v`,均与真实源码一致;生产录制链的 `min_distance` 也已顺链核到。判定见文末「核验台账」。

**仍无法验证的两处(必须记住,别默认狗上=仓库):**
- **核心文件 `route_recorder.py` 无狗上 sha 对照**——它不在 4 份做过 sha 校验的狗上副本(`laserMapping.cpp` / `lddc.cpp` / `lds.cpp` / `waypoint_follower_go2_2.py`)之内。**整篇描述的是仓库代码,狗上是否跑同一份录制器【无法验证】。** `HANDOFF` 里「未额外改动」只是文档口径,不算证据。
- **跟随器仓库版 ≠ 狗上版(sha 已证)**:仓库 `waypoint_follower_go2_2.py`(1043 行,`class WaypointFollowerGo22`,sha256 `009cb2…42e75`)≠ 狗上版(330 行,`class WaypointFollower`,sha256 `d205a5…1120`)。但「几何算朝向 / 参数定速度 / 忽略 CSV `yaw`+`v`」这几条**已在狗上版(`remote_source`)逐条复核为真**,故结论对狗端仍成立。

**源标签约定**:`【默认 code f:line】`=仓库代码默认值;`【生产链 console/blackbox f:line】`=真实录制触发链下发值;`【生产 saas f:line】`=云端 agent 侧;`【狗上 dog:证据】`=狗上副本已验;`【README对照 f:line】`;`【推断-未验】`=逻辑推断,源码未直接断言。

## 一句话
**人工遥控**狗沿目标路线走一遍【README对照 README.md:336】,`route_recorder` 订阅 FAST-LIO 的 `/Odometry`【默认 code route_recorder.py:21,42-47】,**每走过 `min_distance` 就落一个点**【默认 code route_recorder.py:64-66】,写成 `id,x,y,yaw,v` 的 CSV【默认 code route_recorder.py:35】。

## 逐行逻辑(`odom_callback:54`)
1. 订阅 `/Odometry`;每帧取 `x, y`,`yaw = yaw_from_quaternion(...)`【默认 code route_recorder.py:55-57】。
2. **按距离采样**:第一帧必存【默认 code route_recorder.py:61-62】;之后只有距**上一个存点**位移 `≥ min_distance` 才存(`last_x/last_y` 只在存点后更新,所以基准是「上一个存点」而非「上一帧」)【默认 code route_recorder.py:64-66,83-84】。→ 天然按空间均匀打点、和走多快无关(距离采样的逻辑推论)【推断-未验】。
   - `min_distance` **默认 0.4m**【默认 code route_recorder.py:23】。生产链下发值 **0.40**——`console` 的 `ROUTE_NORMAL_SPACING=0.40`【生产链 console tools/patrol_console/server.py:61】→ 拼成 `--min-distance 0.40`【生产链 console server.py:571】→ `blackbox` spawn 时 `min_distance:=0.400000`【生产链 blackbox route_recording_blackbox.py:1005】(其 argparse 默认亦 0.40【生产链 blackbox route_recording_blackbox.py:1391】)。**默认==生产,无背离,狗上实际生效 0.40。**
3. 写一行 CSV:`[id, x(.6f), y(.6f), yaw(.6f), v(.3f)]`(`v` 实际以 `.3f` 输出,原文未标格式,已补)【默认 code route_recorder.py:69-75】,其中 **`v` 恒为 `default_speed`(默认 0.20)的常数占位**【默认 code route_recorder.py:24,74】;`flush` 落盘【默认 code route_recorder.py:76】。
   - ⚠️ **`default_speed` 生产链从不下发**:`blackbox` spawn 只传 `route_file` + `min_distance` 两个 `-p`【生产链 blackbox route_recording_blackbox.py:994-1006】,README 手动示例也不传【README对照 README.md:362-364】。→ 狗上录出的 `v` **恒为 `0.200` 的死值占位**,不是真实速度。
4. `Ctrl+C` 停止 → `finally` 里 `destroy_node()`【默认 code route_recorder.py:99,102】,`destroy_node` 内 `file.close()` 关文件【默认 code route_recorder.py:88】。

## 几个要点(和跟随器对上)
- CSV 表头:`id, x, y, yaw, v`【默认 code route_recorder.py:35】。跟随器 `load_route` 用 `csv.DictReader` 按列名读 `x/y/yaw/v`、`id` 不读【狗上 dog:remote_source waypoint_follower_go2_2.py:110-117】(仓库版同 :505-511)。
- ⚠️ **`yaw` 和 `v` 录了但生产跟随器基本不用**:
  - `yaw` 来自里程计四元数【默认 code route_recorder.py:57】、`v` 是常数 `0.20`【默认 code route_recorder.py:74】;
  - 而 `waypoint_follower_go2_2` 用**相邻点几何**算朝向 `target_angle = atan2(dy, dx)`【狗上 dog:remote_source waypoint_follower_go2_2.py:276-277】、`current_yaw` 取 odom 四元数(非 CSV `yaw`)【狗上 dog:remote_source waypoint_follower_go2_2.py:127】、速度 `vx = min(v_base, max_vx)` 用**参数**定【狗上 dog:remote_source waypoint_follower_go2_2.py:288】(见 02);
  - CSV 的 `row['yaw']/['v']` 虽被读入 route 字典【狗上 dog:remote_source waypoint_follower_go2_2.py:113-116】,但**控制回路从不引用**。→ 这两列是历史/其它跟随器的遗留字段。
  - (仓库版一致:速度同用 `v_base/max_vx`【仓库 waypoint_follower_go2_2.py:861-863】;CSV `yaw` 仅在 `write_control_trace` **日志字段**露一次 `target.get('yaw')`【仓库 :439】、不参与控制;CSV `v`【仓库 :511】全程不再引用——故「基本不用」措辞恰当。**注:仓库版 ≠ 狗上版(sha 已证),此处结论已在狗上版复核。**)
- 坐标系 = **FAST-LIO 局部系**(以开机静止初始化点为原点)。`route_recorder` 采样的是 `/Odometry`(FAST-LIO 输出)的 pose【默认 code route_recorder.py:55-57】;「开机静止初始化点为原点」是 FAST-LIO 特性,录制器本身不断言,由 README「固定起点」【README对照 README.md:239-253】+「启动后静止 5~10 秒初始化」【README对照 README.md:308】佐证【推断-未验】。所以 README 死磕"固定起点固定朝向":录制与回放必须同一起点,否则坐标系对不上会偏移/反向【README对照 README.md:253】(除非走生产链的 `route_relocalizer` 重定位【生产 saas go2_saas_agent.py 引用】,组件源在 `orin_go2_fastlio_ws/src/go2_map_manager/src/route_relocalizer.cpp`,见 07)。
- README §9 建议:走道路中间【README对照 README.md:341】、0.2~0.5m/s(上限不超 1.0)【README对照 README.md:343,342】、转弯放慢【README对照 README.md:344】、事后手工删抖动/跳变/重复点【README对照 README.md:392】。

## 生产「录制」是谁触发的(补:原文未点明入口)
`route_recorder` **不由** `go2_saas_agent.py` 的 `start_patrol` 或任何 launch 脚本启动——saas 只在进程匹配处把 `route_recorder` 当**进程名**用【生产 saas go2_saas_agent.py:477】。真正的生产录制入口是:`patrol_console/server.py` 的 `act_start_recorder`【生产链 console server.py:557】→ `route_recording_blackbox.py` spawn 录制器【生产链 blackbox route_recording_blackbox.py:994-1006】。上文「生产值 0.40」即出自此 console/blackbox 链,而非 saas `-p` 串。

## 相关(非本节,后续)
- `tools/build_bounded_route.py`(重采样+严格约束偏离原始 XY)【tools/build_bounded_route.py:2】 与 `tools/build_piecewise_route.py`(分段线性路线)【tools/build_piecewise_route.py:2】:确在 `tools/`,录后对路线做裁剪/分段处理。
- ⚠️ **`route_quality.py` 不在 `tools/`**(原文把三者并列易被误读为同在 `tools/`)——实际在包目录 `orin_go2_fastlio_ws/src/go2_fastlio_patrol/go2_fastlio_patrol/route_quality.py`(1108 行,质量统计模块)【orin_go2_fastlio_ws/src/go2_fastlio_patrol/go2_fastlio_patrol/route_quality.py:1】。功能(质量评估)大体成立,仅**位置**须更正。
- 生产里路线可由云端下发下载:`go2_saas_agent.py` 的 `download_route_csv(route_url, route_path, timeout=45, attempts=3)`【生产 saas go2_saas_agent.py:1170】,在路线拉取时调用【生产 saas go2_saas_agent.py:1322】(见 08)。

## 狗上状态一览(sha 校验口径)
| 文件 / 子系统 | 角色 | 狗上状态 |
|---|---|---|
| `route_recorder.py` | **本文核心** | **【无狗上对照】**——不在 4 份 sha 副本内,狗上是否同一份无法验证(最重要的可靠性缺口) |
| `waypoint_follower_go2_2.py` | 跟随器(对照用) | **repo≠dog(sha 已证)**;结论已在狗上版(`remote_source`)逐条复核为真 |
| `go2_saas_agent.py` | 云端 agent / 下载路线 | 【无狗上对照】(仓库源码已核 `download_route_csv`) |
| `tools/patrol_console/server.py` | 生产录制**触发**链 | 【无狗上对照】(`ROUTE_NORMAL_SPACING=0.40`、`act_start_recorder` 已核) |
| `route_recording_blackbox.py` | 生产录制**spawn**处 | 【无狗上对照】(只传 `route_file`+`min_distance`、默认 0.40 已核) |
| `route_quality.py`(包目录) | 路线质量统计 | 【无狗上对照】(存在,1108 行,位置已更正) |
| `tools/build_bounded_route.py` / `build_piecewise_route.py` | 重采样/分段 | 【无狗上对照】(docstring 已核) |
| `route_relocalizer.cpp` | 重定位(见 07) | 【无狗上对照】(C++ 源存在,生产链引用) |
| `README.md` | 仓库文档 | 【无狗上对照】(非狗端运行期代码;§6/§8/§9 内容按仓库 README 核实) |

## 核验台账(claim → 证据 → 判定)
- 核心文件/节点名 `route_recorder`(entry_point 已证)→ `route_recorder.py:19`,`setup.py:23` → **CONFIRMED**(路径须带 `orin_go2_fastlio_ws/` 前缀,低严重度)
- 订阅 `/Odometry`、每 `min_distance` 落点、写 `id,x,y,yaw,v` → `route_recorder.py:21,42-47,35,64-66` → **CONFIRMED**(「人工遥控」由 README §9:336 支持,属使用假设)
- 逐行①:取 `x/y`、`yaw=yaw_from_quaternion` → `route_recorder.py:55-57,11-14` → **CONFIRMED**（`odom_callback` 确在 :54)
- 逐行②:第一帧必存 / 距**上一个存点** `≥ min_distance`(默认 0.4)/ 空间均匀 → `route_recorder.py:61-66,23,83-84` → **CONFIRMED**;生产链 0.40 = 代码默认,**无背离**(`server.py:61` → `:571` → `blackbox:1005/1391`)
- 逐行③:写 `.6f` 三列 + `v` 恒 `default_speed`(默认 0.20,实为 `.3f`)+ `flush` → `route_recorder.py:69-76,24` → **CONFIRMED**;生产链**从不下发** `default_speed`(`blackbox:994-1006` 只传两参)→ `v` 恒 `0.200`
- 逐行④:`Ctrl+C` → `finally destroy_node` → `file.close()` → `route_recorder.py:99-103,86-91` → **CONFIRMED**
- 表头 + 跟随器按列名读、`id` 不读 → `route_recorder.py:35`;狗上 `remote_source:110-117`;仓库 `:505-511` → **CONFIRMED**（两版一致）
- `yaw`/`v` 录了不用(几何算朝向、参数定速度)→ 狗上 `remote_source:272-277,288,113-116,127`;仓库 `:786-790,861-863,439,508-511`;源头 `route_recorder.py:57,74` → **CONFIRMED**（狗上版已复核,仓库版仅日志用一次）
- 坐标系=FAST-LIO 局部系、须同起点否则偏移/反向、否则走 `route_relocalizer` → `route_recorder.py:21,55-57`;`README.md:239-253,308,253`;`route_relocalizer.cpp` → **CONFIRMED**（原点特性属合理推断）
- README §9 建议四条(道路中间/0.2~0.5/转弯放慢/删抖动点)→ `README.md:341,343,344,392`(§9=334-395)→ **CONFIRMED**（归属正确)
- `tools/…build_bounded / build_piecewise / route_quality`:质量评估/裁剪/分段 → `tools/build_bounded_route.py:2`;`tools/build_piecewise_route.py:2`;**`route_quality.py:1`(在包目录,非 `tools/`)** → **CORRECTED**（`route_quality.py` 位置更正为 `orin_go2_fastlio_ws/src/go2_fastlio_patrol/go2_fastlio_patrol/route_quality.py`）
- 云端下发下载路线 → `go2_saas_agent.py:1170`(`:1322` 调用)→ **CONFIRMED**（见 08）

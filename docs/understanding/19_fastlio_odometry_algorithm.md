# 19 · FAST-LIO 里程计算法(点 + IMU → /Odometry)

> 原则同 00。逐行读:`FAST_LIO/src/laserMapping.cpp`(仓库版 wc -l≈1414、Read 计 1415,主循环+观测模型+发布)、
> `FAST_LIO/src/IMU_Processing.hpp`(IMU 初始化+前向传播+反向去畸变)、
> `FAST_LIO/include/{use-ikfom.hpp, so3_math.h, common_lib.h, ikd-Tree/ikd_Tree.h}`。
> `esekfom.hpp`(66KB/68556B 的 IKFoM 迭代卡尔曼库)**未逐行**,按其接口/调用理解其职责(第三方库,非本项目改)。
> 承接 doc 18(点已进到 `/livox/lidar` CustomMsg + `/livox/imu`)。

## 〇、核验状态(本轮已对磁盘源码逐条核)

**本轮对工作树源码逐条核对,并首次引入"狗上真跑版"对照。两条系统性结论必须先摆在前面:**

1. **整篇原文的 `file:line` 标注全部对应"仓库版" `orin_go2_fastlio_ws/.../laserMapping.cpp`(sha `5fec8282…`,1414 行);但狗上真跑的是另一个 build:** `analysis/xunjian_20260725_shutdown_capture/previous_boot/remote_source/laserMapping.cpp`(sha `e4cd05cb…`,1395 行)。两版**行号不对应**(`sync_packages` 仓库:494 / 狗:485;`h_share_model` 仓库:849 / 狗:821;`timer_callback` 仓库:1146 / 狗:1118),且**第五节健康门的核心行为完全相反**(见下)。**本文凡引仓库行号一律标【默认 code:行】,凡有狗上对照的另标【狗 dog:行】。**
2. **生产启动串覆盖了大量代码默认值。** 生产经 `scripts/base_bringup.sh:220`:
   `ros2 launch fast_lio mapping.launch.py config_file:=go2_mid360s.yaml rviz:=false`
   (saas 侧 `go2_saas_agent.py:2560` 经 nohup 调 `base_bringup.sh`)加载 `config/go2_mid360s.yaml`。原文几乎全按 `declare_parameter` 默认值写,**多处与狗上生效值不符**(cube 200→1000、det_range 300→100、max_iteration 4→3、extrinsic_est_en true→false、path_en true→false、dense true→false、lidar_qos 深度→2、imu_qos 深度→400)。

**源标签约定:**
- 【默认 code:行】= 仓库 `laserMapping.cpp`(或注明的头文件)的常量/`declare_parameter` 默认。
- 【生产 cfg:行】= `config/go2_mid360s.yaml` 的实际加载值(经 `base_bringup.sh:220`)。**狗上生效以此为准。**
- 【狗 dog:证据】= 有狗上副本/落盘佐证(`remote_source/laserMapping.cpp` 行号,或 `runs/*/manifest.txt`)。
- 【无狗上对照】= 仅能对仓库核,狗上是否一致**无法验证**(本项目多数头文件属此类)。

**狗上文件状态一览:**
- `laserMapping.cpp`:**repo≠dog(sha 验)**——有狗上对照 `remote_source`;仓库 1414 行 / 狗 1395 行;核心差异见第五节。
- `IMU_Processing.hpp`、`use-ikfom.hpp`、`ikd_Tree.h`、`common_lib.h`、`preprocess.{h,cpp}`、`esekfom.hpp`:**均【无狗上对照】**——第三方 FAST-LIO 头/源,行号命中仓库,狗上是否一致未验。
- `go2_mid360s.yaml`:文件本身【无狗上对照】,但其 `lidar_qos_depth:2 / imu_qos_depth:400` 被狗 `manifest.txt`(`fast_lio_input_qos=lidar_2_best_effort_imu_400_reliable`)**落盘佐证=狗生产配置**;其余值(cube/det_range/max_iter/extrinsic/path/dense)manifest 未记,狗上与仓库是否一致未验。
- `mapping.launch.py`、`base_bringup.sh`、`go2_saas_agent.py`:【无狗上对照】(launch 默认 config 是 `mid360.yaml`,被 `base_bringup.sh:220` 传参覆盖为 `go2_mid360s.yaml`)。

---

## 一、状态是什么(`use-ikfom.hpp` MTK 流形)
FAST-LIO 估计一个 **23维误差状态 / 24维名义状态**(`state_ikfom`),在流形上:
`pos(3)`、`rot(SO3)`、**`offset_R_L_I(SO3)`、`offset_T_L_I(3)`=雷达↔IMU外参**、`vel(3)`、`bg(3)陀螺零偏`、`ba(3)加计零偏`、`grav(S2)重力方向`。【默认 code:use-ikfom.hpp:12-21 manifold、:47 `get_f` 返回 `Matrix<24,1>`、:61 `df_dx` 为 `Matrix<24,23>`】【无狗上对照】
- **外参在状态里,但生产不在线估。** `extrinsic_est_en=true` 时**才**在线自估雷达-IMU 外参,config 的 `extrinsic_T/R` 只是初值。⚠️ **默认 true 但生产 false**:【默认 code:laserMapping.cpp:77,1005,1047=true】↔【生产 cfg:go2_mid360s.yaml:37 `extrinsic_est_en:false`】→ **狗上不做在线外参估计**,外参锁在 config 初值 `extrinsic_T:[-0.011,-0.02329,0.04412]`/`extrinsic_R:单位阵`(初值注入 `IMU_Processing.hpp:197-198`【无狗上对照】)。
- 过程模型 `get_f`(`:47`):`ṗ=v`,`Ṙ=ω−bg`,`v̇=R(a−ba)+g`;`df_dx/df_dw` 给雅可比。过程噪声 `ng/na/nbg/nba`。【默认 code:use-ikfom.hpp:47-59,:61 df_dx,:80 df_dw,:28-33 process_noise_ikfom】【无狗上对照】

## 二、主循环(`laserMapping.cpp` timer_callback,100Hz)【默认 code:1146 / 狗 dog:1118】
> 周期 `period_ms=1000.0/100.0=10ms→100Hz`【默认 code:1127-1128】。每拍走这条流水线:

1. **配对** `sync_packages`【默认 code:494 / 狗 dog:485】:取一帧雷达 + 落在 `[帧头, 帧尾]` 内的所有 IMU,打包成 `MeasureGroup`。帧尾时刻 = 帧头 + 最后一点 curvature(ms→s,`lidar_end_time=lidar_beg_time+points.back().curvature/1000`【默认 code:517】,来自 doc 18 的逐点时间)。
2. **IMU 处理** `p_imu->Process`【默认 code:IMU_Processing.hpp:339】【无狗上对照】:
   - **未初始化**(前 `MAX_INI_COUNT=10` 帧【:26】):静止收 IMU,估**重力方向**(`grav=S2(−mean_acc/|mean_acc|·G)`【:193】)、**陀螺零偏**(`bg=mean_gyr`【:196】)、加计/陀螺协方差【:185-186】、设初始 P【:201-208】(触发 `init_iter_num>MAX_INI_COUNT`【:357】)。
   - **已初始化** → `UndistortPcl`【:213】去运动畸变:
     a. **前向传播**:把上一帧尾 IMU 接到本帧头,逐个 IMU 区间 `kf.predict(dt,Q,in)`【:278】(用中值 `angvel_avr/acc_avr`【:251-256】),推进状态+协方差,并在每个 IMU 时刻存 `IMUpose`【:289】;
     b. **反向传播**:点按时间排序【:225】,从帧尾往前【:303-304】,对每个点用它所处 IMU 区间的位姿把它**补偿到"帧尾"坐标系**:
        `P_comp = R_L_I^{-1}·(R_e^{-1}·(R_i·(R_L_I·P_i+T_L_I)+T_ei)−T_L_I)`【:327】,
        其中 `R_i=R_head·Exp(ω,dt)`【:323】、`T_ei=pos_head+v·dt+½a·dt²−pos_e`【:326】。**这就是逐点去畸变**——狗在动,一帧内 2 万点不是同一时刻打的,必须各自搬到同一时刻。→ 输出 `feats_undistort`。
3. **滑窗地图** `lasermap_fov_segment`【默认 code:248】:维护一个以雷达为心的局部立方体;⚠️ **边长 `cube_len`:【默认 code:990,1032=200.】↔【生产 cfg:go2_mid360s.yaml:8 `cube_side_length:1000.0`】→ 狗上是 1000m 不是 200m**。当雷达走到离边界 `<1.5×DET_RANGE`(`MOV_THRESHOLD=1.5f`【:82】,判据 :268)时,`ikdtree.Delete_Point_Boxes`【:292】删掉另一侧远点。⚠️ **`DET_RANGE`:【默认 code:81,991,1033=300】↔【生产 cfg:go2_mid360s.yaml:36 `det_range:100.0`】→ 触删阈值狗上是 1.5×100=150m 不是 1.5×300=450m**。**这让它是"里程计"(局部图)不是全局 SLAM**——地图有界、不闭环。
4. **降采样** `downSizeFilterSurf`(体素 `filter_size_surf=0.5m`【默认 code:988,1030=0.5,`setLeafSize` :1070】,**生产同值**【生产 cfg:go2_mid360s.yaml:6 `filter_size_surf:0.5`】)→ `feats_down_body`(几百~几千点)。
5. **建/用 ikd-Tree**:树空 → 用首帧 `Build`;否则进更新。
6. ★ **迭代卡尔曼更新** `kf.update_iterated_dyn_share_modified`【默认 code:1240 / 狗 dog:1216】:⚠️ **迭代 ≤`max_iteration` 次:【默认 code:979,1017=4】↔【生产 cfg:go2_mid360s.yaml:5 `max_iteration:3`】→ 狗上 ≤3 次**。每次回调 **`h_share_model`(观测模型/本项目核心逻辑)**【默认 code:849 / 狗 dog:821】:
   - 逐降采样点:用**当前状态**变到世界系 → `ikdtree.Nearest_Search` 找 **5 个最近地图点**(`NUM_MATCH_POINTS=5`【common_lib.h:24】【无狗上对照】,`:881`);`esti_plane` 拟合平面 `(n, d)`【:889】;
   - **点到面残差** `pd2 = n·p_world + d`【:891】;权重 `s = 1−0.9·|pd2|/√range`【:892】;`s>0.9`【:894】才算**有效点**;
   - 组测量雅可比 `H`(每行 = 法向量 + 叉乘旋转项 + 外参项)【:955】,测量值 `= −pd2`【:963,`h(i)=−norm_p.intensity`】。⚠️ **"外参项"(B 块 :954-955)只有 `extrinsic_est_en=true` 才组入;生产=false → 走 else 分支【默认 code:959】,雅可比无外参 B 项**。
   - IESKF(esekfom)据此解线性化增量、更新 23 维误差态+协方差,再线性化,收敛为止。**本质 = 点到面 ICP,但先验是 IMU 推的位姿,融进一个迭代卡尔曼**。
7. **发布** `publish_odometry`【默认 code:739 / 狗 dog:738】:`/Odometry`(frame `odom`→`base_link`【:741-742】)+ tf 广播【:819-830】 + ⚠️**patch:原子写 `/dev/shm/go2_fastlio_latest_odom.txt`**(写 `.tmp` 后 `rename`,内容 `stamp+xyz+四元数`【默认 code:747-763 / 狗 dog:738-758,**两版一致**】)。
8. **地图增量** `map_incremental`【默认 code:549】:把本帧点(世界系)加进 ikd-Tree,带体素去重决策【:566-583】(只加能提升覆盖的点,`Add_Points` :592-593)。地图边走边长。
9. **发布点云(⚠️ 生产与默认有别):**
   - `/path`:【默认 code:973,977 `path_en=true`】↔【生产 cfg:go2_mid360s.yaml:44 `path_en:false`】→ **狗上不发 `/path`**(声明 :1123)。
   - `/cloud_registered`(世界系):稠密与否由 `dense_publish_en` 决定【三元 :604】,【默认 code:true】↔【生产 cfg:go2_mid360s.yaml:48 `dense_publish_en:false`】→ **狗上用 `feats_down_body`(降采样),非"稠密"**(声明 :1118)。
   - `/cloud_registered_body`(base_link 系,frame :671,给下游 2D/避障):`scan_bodyframe_pub_en:true`【生产 cfg:go2_mid360s.yaml:49】→ **照发,属实**(声明 :1119)。

## 三、ikd-Tree(增量 kd 树地图,`ikd_Tree.h`)【无狗上对照】
- FAST-LIO 的地图 = 一棵**支持增量增删 + 后台重建平衡**的 kd 树(非 PCL 静态树)。
- 关键操作:`Build/Add_Points/Delete_Point_Boxes/Nearest_Search`【ikd_Tree.h:327/331/334/328】;失衡超阈值时**开子线程重建**(`Multi_Thread_Rebuild_Point_Num=1500`【:15】)。
- ⚠️ **降采样格 `downsample_size`:成员默认 `0.2`【ikd_Tree.h:279】,但运行时被 `set_downsample_param(filter_size_map_min)` 覆盖**【赋值 :321,调用 laserMapping.cpp:1193】,而 `filter_size_map` 生产=`0.5`【生产 cfg:go2_mid360s.yaml:7】→ **实际生效 `downsample_size=0.5` 不是 0.2**(此值"云端/config 下发后被这条覆盖生效",非成员默认)。
- 作用:第 6 步的最近邻搜索(找面)和第 8 步的插入都在它上面 O(logN);滑窗删点也是删它的 box。

## 四、和配置/其他文档的钉死关系(`declare_parameter` 段【默认 code:973】)
- `lidar_type=1(AVIA)` → 订 **CustomMsg** `/livox/lidar` + `livox_pcl_cbk` + `avia_handler`(坐实 doc 18)。**默认与生产一致**【默认 code:998,1040=1】=【生产 cfg:go2_mid360s.yaml:24 `lidar_type:1`】;枚举 `AVIA=1`【preprocess.h:16】、订阅 `:1101-1105`、调用链 `preprocess.cpp:49→:95 avia_handler`【均无狗上对照】。
- `pcd_save_en=false` + `publish_frame_world` 里 PCD 累积段**整段注释**(`:627-654`)→ **FAST-LIO 本身不存图**;PCD 地图是另一条离线链(doc 06),不是这里。**默认与生产一致=false**【默认 code:77,1006,1048】=【生产 cfg:go2_mid360s.yaml:52 `pcd_save_en:false`】。(注:`avia.yaml`/`mid360.yaml` 里 `pcd_save_en:true`,但生产只用 `go2_mid360s.yaml`,结论对狗成立。)
- ⚠️ `extrinsic_est_en`:**在线精修雷达-IMU 外参仅在 true 时发生;生产=false → 狗上不精修外参**(同第一节,别再当"在线自估"陈述)。【默认 code:true】↔【生产 cfg:go2_mid360s.yaml:37 false】。
- QoS:雷达 `best_effort`【默认 code:1104】、IMU `reliable`【:1112】→ 偏向"最新帧"(配 doc 18 的驱动跳帧)。⚠️ **深度是生产值,不是默认**:
  - 雷达深度:代码 clamp 上限 20【:1025】,**生产=2**【生产 cfg:go2_mid360s.yaml:17 `lidar_qos_depth:2`】;
  - IMU 深度:**生产=400**(默认 10)【生产 cfg:go2_mid360s.yaml:18 `imu_qos_depth:400`】。
  - 【狗 dog:manifest】`runs/xunjian-20260725-06、-07/manifest.txt` 落盘 `fast_lio_input_qos=lidar_2_best_effort_imu_400_reliable` → **狗上确为 lidar 深度 2 / imu 深度 400**(此串由 `go2_saas_agent.py:1847` 生成)。

## 五、这里的 patch(超出原版 FAST-LIO,团队改的)——⚠️ 仓库版 ≠ 狗版,方向相反
> **头号 repo≠dog、且与巡检"跑着跑着瘫死/需重启底盘"直接相关:健康门的后果两版正相反。原文按仓库版写成"暂停但不永久锁死",而狗上真跑版恰恰会永久闩死到 base restart。** 阈值两版一致,**后果两版相反**。

1. **`livox_pcl_cbk` 健康门**【默认 code:378-426 / 狗 dog:377-411】:帧点数`<5000`、跨度不在`[0.08,0.13]s`、时间戳/timebase 非法、`dt≤0` → 计数(阈值常量两版同:`kMinLivoxFramePoints=5000`【code:107 / dog:107】、`kMinLivoxFrameSpan=0.08`【code:108 / dog:108】、`kMaxLivoxFrameSpan=0.13`【code:109 / dog:109】、`kMaxConsecutiveBadLidarFrames=3`【code:105 / dog:105】)。
   - ⚠️ **正 gap `>1.0s` 的后果——两版相反:**
     - **仓库版(默认):** `frame_dt>kRecoverableLivoxFrameGap(1.00)`【code:111,391】视为**可恢复**(小 best-effort 队列丢了旧帧),**不永久锁死**,打 `[FAST_LIO_RECOVERY]`【code:412,423】接受当前帧继续。
     - **狗上版(真跑):** 有 `constexpr double kHardLivoxFrameDt=1.00`【dog:110】+ `bool lio_health_failed`【dog:111】;`invalid_frame_dt=(frame_dt<=0 || frame_dt>kHardLivoxFrameDt)`【dog:377-378】属**硬失败** `hard_frame_failure`【dog:379-380】 → `lio_health_failed=true`【dog:398】、清缓冲、日志 **`odometry output is now locked until base restart`**【dog:404-407】。**狗上一旦触发就永久闩死,直到重启底盘。**
     - **`[FAST_LIO_RECOVERY]` 标签只存在于仓库版**(:412,423,1261);**狗版 `grep` 计数=0**,狗上只有 `[FAST_LIO_HEALTH]`。原文"打 `[FAST_LIO_HEALTH]/[FAST_LIO_RECOVERY]`"**对狗不成立**。
2. **`consecutive_no_effective_scans`**【默认 code:1243 / 狗 dog:1219-1240】:连续 `5` 帧无有效点(`kMaxConsecutiveNoEffectiveScans=5`【code:106 / dog:106】)→ 清缓冲。⚠️ **后果两版相反:**
   - **仓库版:** 清缓冲等新帧,含注释 `no permanent health lock was set`【code:1262】,**不永久锁**。
   - **狗上版:** 连续 5 帧无有效点 → `lio_health_failed=true`【dog:1229】、清缓冲、`RCLCPP_FATAL` **`odometry output is locked until base restart`**【dog:1237】。**同样永久闩死。**
3. **`FAST_LIO_INPUT_TIMING/OUTPUT_TIMING`** stderr 统计(时间戳偏移、本地间隔、缓冲深度)——运维埋点。【默认 code:360,791 / 狗 dog:输出侧 760 起 `output_age_ms` 等,标签保留】**两版都有**。
4. ⚠️ **`/dev/shm/go2_fastlio_latest_odom.txt` 快照**:每次出里程计原子写一份纯文本位姿 → **下游不订 ROS 也能读最新位姿**(这正是 doc 13/16 里那个 `/dev/shm` 读取方的**生产者**,至此对上了)。生产者【code:747-763 / 狗 dog:738-758,**两版一致**】;消费者读同一路径:`manual_route_anchor.py:23`、`localization_session_guard.py:157`、`check_fastlio_freshness.py:33`、`check_route_start_alignment.py:89`。

> ⚠️ **设计动机——原文引的注释与狗上注释文本相反:**
> - **仓库版注释**【code:103-105】:`a short positive frame gap must not permanently latch FAST-LIO off during patrol`——意即"**不能**把 FAST-LIO 永久闩死"。
> - **狗上版注释**【dog:103-104】:`Once latched, FAST-LIO stops publishing odometry instead of letting IMU-only propagation contaminate a route or PCD map`——意即"**一旦闩死就停发里程计**,宁可停也不让 IMU-only 推算污染路线/地图"。
> **→ 狗上的设计就是要永久闩死到 base restart。** 原文把"狗永不闩死"当结论恰好讲反了;这与巡检 bug 排查直接相关,是本轮最要紧的更正。

## 六、一句话回答"雷达打点/工作机制"
雷达固件测 ToF+角度算出笛卡尔点(doc 18)→ 驱动 mm→m+逐点时间戳 → FAST-LIO 用 **IMU 前向传播定先验、反向传播把一帧内每个点去畸变到同一时刻**,再用**点到面 ICP 嵌迭代卡尔曼**对齐到 ikd-Tree 局部地图,输出机体位姿 `/Odometry` 和 `/dev/shm` 快照。**没有"雷达发一束回一束单独测距"的主机计算**——测距在雷达内部;主机干的是"多点拼帧 + 去畸变 + 和地图配准求位姿"。【推断-未验:架构性总结,与已核对的 `sync_packages`/`UndistortPcl`/`h_share_model`/`update_iterated` 一致】

---

## 核验台账(claim → 证据 file:line → 判定)

> 判定口径:CONFIRMED=与仓库源码一致;DEFAULT_VS_PROD=代码默认与生产 `go2_mid360s.yaml` 不同,狗上以生产为准;CORRECTED=原文讲错,已用更正值。所有 `laserMapping.cpp` 行号=仓库版(狗版另注 dog 行)。

| # | claim(原文) | 证据 | 判定 |
|---|---|---|---|
| 1 | laserMapping.cpp 1415 行 | 仓库 wc -l=1414;**狗版 1395 行**(remote_source sha e4cd05cb) | CONFIRMED(但仅仓库;**repo≠dog**) |
| 2 | esekfom.hpp 66KB 未逐行 | =68556B≈66.9KiB【无狗上对照】 | CONFIRMED |
| 3 | 23/24 维状态布局 | use-ikfom.hpp:12-21,:47,:61【无狗上对照】 | CONFIRMED |
| 4 | `extrinsic_est_en=true` 在线自估外参 | 默认 code:77,1005,1047=true;生产 cfg:37=**false**;else 分支 code:952-960 | **DEFAULT_VS_PROD**(狗=false,不估) |
| 5 | get_f 过程模型/噪声 | use-ikfom.hpp:47-59,61,80,28-33【无狗上对照】 | CONFIRMED |
| 6 | timer_callback 100Hz | code:1146(dog:1118);周期 code:1127-1128 | CONFIRMED |
| 7 | sync_packages 拼帧/帧尾时刻 | code:494(dog:485),:517,:534-540 | CONFIRMED |
| 8 | IMU Process 初始化(10 帧/grav/bg/P) | IMU_Processing.hpp:339,26,193,196,185-186,201-208,357【无狗上对照】 | CONFIRMED |
| 9 | UndistortPcl 前向/反向去畸变 | IMU_Processing.hpp:213,251-256,278,289,225,303-304,327,323,326【无狗上对照】 | CONFIRMED |
| 10 | cube_len=200m 局部立方体 | code:990,1032=200;生产 cfg:8=**1000** | **DEFAULT_VS_PROD**(狗=1000m) |
| 11 | 离边界 <1.5×DET_RANGE(300) 删远点 | MOV_THRESHOLD=1.5f code:82,:268,:292;DET_RANGE 默认 300;生产 cfg:36=**100** | **DEFAULT_VS_PROD**(狗阈值=150m;1.5 系数属实) |
| 12 | filter_size_surf=0.5m 降采样 | code:988,1030=0.5,:1070;生产 cfg:6=0.5 | CONFIRMED(默认=生产) |
| 13 | max_iteration=4 | code:979,1017=4;生产 cfg:5=**3** | **DEFAULT_VS_PROD**(狗≤3) |
| 14 | h_share_model 点到面 ICP(5点/esti_plane/pd2/s>0.9/H) | code:849(dog:821),NUM_MATCH_POINTS=5 common_lib.h:24,:881,889,891,892,894,963,955 | CONFIRMED(**外参 B 块仅 extrinsic_est_en=true**,生产走 :959 无 B) |
| 15 | publish_odometry + /dev/shm 快照 | code:739(dog:738),:741-742,:747-763(dog:738-758),:819-830 | CONFIRMED(狗版一致) |
| 16 | map_incremental 体素去重加点 | code:549,566-583,592-593 | CONFIRMED |
| 17 | 发 /path、/cloud_registered(稠密)、/cloud_registered_body | 声明 :1123/:1118/:1119(:671);dense 三元 :604;path/dense 默认 true;生产 cfg:44,48=**false** | **DEFAULT_VS_PROD**(狗不发 /path;/cloud_registered 非稠密;body 帧照发) |
| 18 | ikd-Tree downsample_size=0.2 / 1500 重建 | ikd_Tree.h:279=0.2 被 :321/laserMapping.cpp:1193 `set_downsample_param(0.5)` 覆盖;1500 ikd_Tree.h:15【无狗上对照】 | **DEFAULT_VS_PROD**(实际生效 0.5;1500 及四操作属实) |
| 19 | declare_parameter 段 :973 | code:973 首个 declare | CONFIRMED |
| 20 | lidar_type=1(AVIA)→CustomMsg 链 | AVIA=1 preprocess.h:16;code:998,1040=1;生产 cfg:24=1;:1101-1105;preprocess.cpp:49→95【无狗上对照】 | CONFIRMED(默认=生产) |
| 21 | pcd_save_en=false + PCD 段注释 → 不存图 | code:77,1006,1048=false;生产 cfg:52=false;:627-654 注释块 | CONFIRMED(avia/mid360=true 但生产不用) |
| 22 | QoS:雷达 best_effort ≤20 / IMU reliable | code:1104,1112,1025(clamp≤20);生产 cfg:17,18=**2/400**;dog manifest `lidar_2..imu_400` | **DEFAULT_VS_PROD**(狗:lidar 深度 2、imu 深度 400) |
| 23 | 健康门阈值(<5000/[0.08,0.13]/dt≤0/连续3坏) | code:378-418,108,109,110,388-389,106,408-411;dog 常量:107-109 | CONFIRMED(阈值两版同) |
| 24 | 正 gap>1.0s 可恢复、不永久锁、打 RECOVERY | **仓库**:kRecoverableLivoxFrameGap=1.00 code:111,391,421-426,标签 412/423/1261;**狗**:kHardLivoxFrameDt=1.00 dog:110、dog:377-378、dog:395-407 `lio_health_failed=true`+`locked until base restart`;狗 grep RECOVERY=0 | **CORRECTED**(狗永久闩死,无 RECOVERY 标签) |
| 25 | 连续5帧无有效点 → 清缓冲(不永久锁) | 仓库 code:1243-1266(:1262 no permanent lock);**狗** dog:1219-1239(:1229 lio_health_failed=true、:1237 locked until base restart) | **CORRECTED**(狗永久闩死) |
| 26 | INPUT/OUTPUT_TIMING stderr 统计 | code:360,791;dog:760 起 | CONFIRMED(两版都有) |
| 27 | /dev/shm 快照是 doc13/16 读取方的生产者 | 生产者 code:747-763(dog:738-758);消费者 manual_route_anchor.py:23、localization_session_guard.py:157、check_fastlio_freshness.py:33、check_route_start_alignment.py:89 | CONFIRMED |
| 28 | `:103` 注释:一次坏帧不能永久闩死 | **仓库** code:103-105 `must not permanently latch`;**狗** dog:103-104 `Once latched, FAST-LIO stops publishing odometry`（设计就是永久闩死） | **CORRECTED**(注释文本狗上相反) |
| 29 | 第六节:无"逐束单独测距"主机计算,主机干拼帧+去畸变+配准 | 与 :494/IMU_Processing.hpp:213/:849/:1240 一致 | CONFIRMED(推断,架构性总结) |

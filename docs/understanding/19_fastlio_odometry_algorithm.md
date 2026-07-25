# 19 · FAST-LIO 里程计算法(点 + IMU → /Odometry)

> 原则同 00。逐行读:`FAST_LIO/src/laserMapping.cpp`(1415行,主循环+观测模型+发布)、
> `FAST_LIO/src/IMU_Processing.hpp`(IMU 初始化+前向传播+反向去畸变)、
> `FAST_LIO/include/{use-ikfom.hpp, so3_math.h, common_lib.h, ikd-Tree/ikd_Tree.h}`。
> `esekfom.hpp`(66KB 的 IKFoM 迭代卡尔曼库)**未逐行**,按其接口/调用理解其职责(第三方库,非本项目改)。
> 承接 doc 18(点已进到 `/livox/lidar` CustomMsg + `/livox/imu`)。

## 一、状态是什么(`use-ikfom.hpp` MTK 流形)
FAST-LIO 估计一个 **23维误差状态 / 24维名义状态**(`state_ikfom`),在流形上:
`pos(3)`、`rot(SO3)`、**`offset_R_L_I(SO3)`、`offset_T_L_I(3)`=雷达↔IMU外参**、`vel(3)`、`bg(3)陀螺零偏`、`ba(3)加计零偏`、`grav(S2)重力方向`。
- **外参在状态里** → `extrinsic_est_en=true` 时**在线自估**雷达-IMU 外参;config 的 `extrinsic_T/R` 只是初值。
- 过程模型 `get_f`(`:47`):`ṗ=v`,`Ṙ=ω−bg`,`v̇=R(a−ba)+g`;`df_dx/df_dw` 给雅可比。过程噪声 `ng/na/nbg/nba`。

## 二、主循环(`laserMapping.cpp` timer_callback,100Hz,`:1146`)
每拍走这条流水线:

1. **配对** `sync_packages`(`:494`):取一帧雷达 + 落在 `[帧头, 帧尾]` 内的所有 IMU,打包成 `MeasureGroup`。帧尾时刻 = 帧头 + 最后一点 curvature(ms,来自 doc 18 的逐点时间)。
2. **IMU 处理** `p_imu->Process`(`IMU_Processing.hpp:339`):
   - **未初始化**(前 `MAX_INI_COUNT=10` 帧):静止收 IMU,估**重力方向**(`grav=−mean_acc/|mean_acc|·G`)、**陀螺零偏**(`bg=mean_gyr`)、加计/陀螺协方差。设初始 P。
   - **已初始化** → `UndistortPcl`(`:213`)去运动畸变:
     a. **前向传播**:把上一帧尾 IMU 接到本帧头,逐个 IMU 区间 `kf.predict(dt,Q,in)`(用中值 `angvel_avr/acc_avr`),推进状态+协方差,并在每个 IMU 时刻存 `IMUpose`;
     b. **反向传播**:点按时间排序,从帧尾往前,对每个点用它所处 IMU 区间的位姿把它**补偿到"帧尾"坐标系**:
        `P_comp = R_L_I^{-1}·(R_e^{-1}·(R_i·(R_L_I·P_i+T_L_I)+T_ei)−T_L_I)`(`:327`),
        其中 `R_i=R_head·Exp(ω,dt)`、`T_ei=pos_head+v·dt+½a·dt²−pos_e`。**这就是逐点去畸变**——狗在动,一帧内 2 万点不是同一时刻打的,必须各自搬到同一时刻。→ 输出 `feats_undistort`。
3. **滑窗地图** `lasermap_fov_segment`(`:248`):维护一个以雷达为心、边长 `cube_len=200m` 的局部立方体;当雷达走到离边界 `<1.5×DET_RANGE(300)` 时,`ikdtree.Delete_Point_Boxes` 删掉另一侧远点。**这让它是"里程计"(局部图)不是全局 SLAM**——地图有界、不闭环。
4. **降采样** `downSizeFilterSurf`(体素 `filter_size_surf=0.5m`)→ `feats_down_body`(几百~几千点)。
5. **建/用 ikd-Tree**:树空 → 用首帧 `Build`;否则进更新。
6. ★ **迭代卡尔曼更新** `kf.update_iterated_dyn_share_modified`(`:1240`):迭代 ≤`max_iteration=4` 次,每次回调 **`h_share_model`(`:849`,观测模型/本项目核心逻辑)**:
   - 逐降采样点:用**当前状态**变到世界系 → `ikdtree.Nearest_Search` 找 **5 个最近地图点**;`esti_plane` 拟合平面 `(n, d)`;
   - **点到面残差** `pd2 = n·p_world + d`;权重 `s = 1−0.9·|pd2|/√range`;`s>0.9` 才算**有效点**;
   - 组测量雅可比 `H`(每行 = 法向量 + 叉乘旋转项 + 外参项),测量值 `= −pd2`。
   - IESKF(esekfom)据此解线性化增量、更新 23 维误差态+协方差,再线性化,收敛为止。**本质 = 点到面 ICP,但先验是 IMU 推的位姿,融进一个迭代卡尔曼**。
7. **发布** `publish_odometry`(`:739`):`/Odometry`(frame `odom`→`base_link`)+ tf 广播 + ⚠️**patch:原子写 `/dev/shm/go2_fastlio_latest_odom.txt`**(stamp+xyz+四元数)。
8. **地图增量** `map_incremental`(`:549`):把本帧点(世界系)加进 ikd-Tree,带体素去重决策(只加能提升覆盖的点)。地图边走边长。
9. 发 `/path`、`/cloud_registered`(世界系稠密)、`/cloud_registered_body`(base_link 系,给下游 2D/避障)。

## 三、ikd-Tree(增量 kd 树地图,`ikd_Tree.h`)
- FAST-LIO 的地图 = 一棵**支持增量增删 + 后台重建平衡**的 kd 树(非 PCL 静态树)。
- 关键操作:`Build/Add_Points/Delete_Point_Boxes/Nearest_Search`;`downsample_size=0.2`,失衡超 `balance/delete` 阈值时**开子线程重建**(`Multi_Thread_Rebuild_Point_Num=1500`)。
- 作用:第 6 步的最近邻搜索(找面)和第 8 步的插入都在它上面 O(logN);滑窗删点也是删它的 box。

## 四、和配置/其他文档的钉死关系(`declare_parameter` 段 `:973`)
- `lidar_type=1(AVIA)` → 订 **CustomMsg** `/livox/lidar` + `livox_pcl_cbk` + `avia_handler`(坐实 doc 18)。
- `pcd_save_en=false` + `publish_frame_world` 里 PCD 累积段**整段注释**(`:627-654`)→ **FAST-LIO 本身不存图**;PCD 地图是另一条离线链(doc 06),不是这里。
- `extrinsic_est_en=true` → 在线精修雷达-IMU 外参。
- QoS:雷达 `best_effort` 深度≤20,IMU `reliable` → 偏向"最新帧"(配 doc 18 的驱动跳帧)。

## 五、这里的 patch(超出原版 FAST-LIO,团队改的)
1. **`livox_pcl_cbk` 健康门**(`:378-426`):帧点数`<5000`、跨度不在`[0.08,0.13]s`、时间戳/timebase 非法、dt≤0 → 计数,连续 `3` 帧坏 → 清缓冲**暂停里程计**;但正 gap`>1.0s` 视为**可恢复**(小 best-effort 队列丢了旧帧),不永久锁死。打 `[FAST_LIO_HEALTH]/[FAST_LIO_RECOVERY]`。
2. **`consecutive_no_effective_scans`**(`:1243`):连续 `5` 帧无有效点 → 清缓冲等新帧(同样不永久锁)。
3. **`FAST_LIO_INPUT_TIMING/OUTPUT_TIMING`** stderr 统计(时间戳偏移、本地间隔、缓冲深度)——运维埋点。
4. ⚠️ **`/dev/shm/go2_fastlio_latest_odom.txt` 快照**:每次出里程计原子写一份纯文本位姿 → **下游不订 ROS 也能读最新位姿**(这正是 doc 13/16 里那个 /dev/shm 读取方的**生产者**,至此对上了)。

> 这些"暂停但不永久锁死"的设计动机(`:103` 注释):巡检中一次短暂坏帧/队列抖动**不能把 FAST-LIO 永久闩死**,否则狗就瘫在原地。→ 和巡检 bug 排查相关,值得记。

## 六、一句话回答"雷达打点/工作机制"
雷达固件测 ToF+角度算出笛卡尔点(doc 18)→ 驱动 mm→m+逐点时间戳 → FAST-LIO 用 **IMU 前向传播定先验、反向传播把一帧内每个点去畸变到同一时刻**,再用**点到面 ICP 嵌迭代卡尔曼**对齐到 ikd-Tree 局部地图,输出机体位姿 `/Odometry` 和 `/dev/shm` 快照。**没有"雷达发一束回一束单独测距"的主机计算**——测距在雷达内部;主机干的是"多点拼帧 + 去畸变 + 和地图配准求位姿"。

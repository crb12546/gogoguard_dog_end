# 18 · 雷达打点数据全链(激光 → 点 → /livox/lidar → FAST-LIO 入口)

> 原则同 00。这是对 doc 01"雷达怎么用"的**底层补全**,逐行读了感知栈第三方源码。
> 已逐行:`Livox-SDK2/include/livox_lidar_def.h`、`sdk_core/data_handler/data_handler.cpp`、
> `livox_ros_driver2/src/{comm/comm.h, call_back/livox_lidar_callback.cpp, comm/pub_handler.cpp, lddc.cpp}`、
> `FAST_LIO/src/preprocess.{h,cpp}`。配置:`MID360s_config.json`、`go2_mid360s.yaml`。

## 一、一个点到底怎么"算"出来的
**关键认知:算 xyz 的是雷达固件,不是主机代码。**
- MID360 是**非重复扫描**(`pattern_mode=0`),4 条激光线(`kLineNumberMid360=4`)靠棱镜扫玫瑰花瓣。打激光 → 测**飞行时间(ToF)→ 距离**;棱镜编码器给**光束方向**;在**雷达内部**算成三维点 + 反射率 + tag。
- 配置 `pcl_data_type=1` = `kLivoxLidarCartesianCoordinateHighData` → 雷达**直接输出笛卡尔坐标**。每点 = `LivoxLidarCartesianHighRawPoint`(`livox_lidar_def.h:167`):**x/y/z int32 单位毫米 + reflectivity(u8) + tag(u8) = 14 字节**。
- (球坐标模式 `pcl_data_type=3` 才由**驱动**用 `r·sinθ·cosφ` 算 xyz,`pub_handler.cpp:496-501`;此部署未用。)

## 二、从雷达到 `/livox/lidar` 的 7 步
1. **发包**:雷达 UDP 发 `LivoxLidarEthernetPacket`(头含 `dot_num` 点数、`time_interval` 每包时长[0.1µs]、`timestamp[8]`、`data_type`)到 `192.168.1.5:56301`,每包 ≤100 点。IMU 走 56401,`LivoxLidarImuRawPoint{gyro xyz, acc xyz}`(float)。
2. **SDK 收**(`data_handler.cpp:69`):按 `data_type` 分流到点回调 / IMU 回调 + 观察者。
3. **驱动回调**(`pub_handler.cpp:101`):组 `RawPacket`,算 `point_interval = time_interval×100/dot_num`(ns/点),打包时间戳,入 `raw_packet_queue_`(积压 >1024 打印 `Livox point packet backlog`)。
4. **时间戳映射**(`GetEthPacketTimestamp:331`):无 PTP/GPS 同步(此部署 `time_sync_en=false`)时,用 `no_sync_timestamp_mapper` 把雷达内部时钟映射到**主机墙钟**。
5. **点处理**(`ProcessCartesianHighPoint:435`):逐点
   - **mm→m**:`x = raw.x/1000`(带装配外参旋转+平移,配置外参全 0 → 单位阵,等于只 ÷1000);
   - `intensity=反射率`,`line=i%4`,`tag=tag`;
   - **逐点时刻** `offset_time = 帧基准 + i×point_interval`。
6. **组帧**(`CheckTimer:221`):攒够 ~100ms(10Hz)发一帧,**正常一帧 ≈2 万点**;
   ⚠️ patch:`帧跨度>2×周期 或 <5000 点` → 打印 `Dropping malformed Livox frame` 丢弃(防坏帧毁掉逐点去畸变);每秒打 `LIVOX_STREAM_HEALTH frame_hz/imu_hz/span_ms/points/lidar_imu_offset_ms/queue_depth`。
7. **发布**(`lddc.cpp`):FAST-LIO 用 **CustomMsg**(`lidar_type=1`);`FillPointsToCustomMsg` 每点 `offset_time=该点−帧基准`(ns);⚠️ patch:发布前**跳到最新帧**丢旧帧(`:225`)。→ `/livox/lidar` + `/livox/imu`。

> **base_bringup 的 Livox 校验(doc 01)校的就是第 3/6 步这些打印字符串**——都是团队**给驱动打补丁**加的埋点。

## 三、FAST-LIO 入口怎么筛这帧(`preprocess.cpp: avia_handler`)
`lidar_type=1`(AVIA)+ `feature_enabled=false`,逐点:
- **回波过滤**:`tag & 0x30 ∈ {0x00,0x10}` 才要(丢多回波/噪声 tag);
- **抽稀**:每 `point_filter_num=3` 留 1(2万→约6~7千);
- **盲区**:`x²+y²+z² > blind²`(`blind=0.5m`)才留;
- **去重**:与前点同坐标丢;
- **逐点时间**:`curvature = offset_time/1e6`(ns→**ms**),存进 PCL `curvature` 字段 → 给 IMU 去畸变用;
- 输出 `pl_surf`(面点),交卡尔曼更新(见 19,待读:laserMapping + IMU_Processing + iKF + ikd-Tree)。
- (`feature_enabled=true` 才跑 `give_feature` 提取平面/边缘特征;此部署关闭,用全部筛后点做 scan-to-map。)

## 四、坐标系与外参(和定位相关)
- 雷达装配外参(`MID360s_config.json` 全 0)→ 点在**雷达自身系**。
- FAST-LIO `extrinsic_T=[-0.011,-0.02329,0.04412]`、`extrinsic_R=I`(`go2_mid360s.yaml`)是 **IMU↔雷达** 外参,在 laserMapping/IMU_Processing 里用于把点变换到 IMU 系再做 SLAM。
- 雷达装歪 ~12.3° 的校正(`level_cloud_node`,doc 16)是**下游**给建图/2D 用的,**不改** FAST-LIO 的输入(FAST-LIO 直接吃机体系原始点 + IMU)。

## 五、留待坐实(下一单元)
- `laserMapping.cpp`(主 SLAM 循环)+ `IMU_Processing.hpp`(前向传播 + 反向去畸变)+ `use-ikfom.hpp`/`esekfom.hpp`(迭代误差状态卡尔曼 IESKF)+ `ikd_Tree.cpp`(增量 kd 树地图):**点 + IMU 怎么算出 /Odometry**。见 doc 19。

# 18 · 雷达打点数据全链(激光 → 点 → /livox/lidar → FAST-LIO 入口)

> 原则同 00。这是对 doc 01"雷达怎么用"的**底层补全**,逐行读了感知栈第三方源码。
> 已逐行:`Livox-SDK2/include/livox_lidar_def.h`、`sdk_core/data_handler/data_handler.cpp`、
> `livox_ros_driver2/src/{comm/comm.h, call_back/livox_lidar_callback.cpp, comm/pub_handler.cpp, lddc.cpp}`、
> `FAST_LIO/src/preprocess.{h,cpp}`。配置:`MID360s_config.json`、`go2_mid360s.yaml`。

## 核验状态

**本轮已对磁盘源码(仓库侧)逐条核实**:行号 / 类名 / 字节数 / 阈值 / 打印串 / 调用链全部对得上,26 条可证伪断言里 22 条 CONFIRMED、2 条 DEFAULT_VS_PROD(`blind`、`point_filter_num`)、其余为硬件约定 / 代码行为反推的语义(标【推断-未验】)。**结论高可靠**,只订正三处非硬错(见下)。

**源标签约定**(下文行内用):
- 【默认 code:file:line】= 节点 `declare_parameter` / 构造函数里的**代码默认值**;
- 【生产 saas:file:line】= 生产装配配置(`go2_mid360s.yaml` / `MID360s_config.json` / launch),**狗上实用**;
- 【狗上 dog:证据】= 有狗上副本且 SHA 已验;
- 【推断-未验】= 由代码行为反推或 Livox 硬件约定,非源码显式断言;
- 【无狗上对照】= 只对仓库核过,狗上副本缺失、是否一致无法验证。

**狗上对照范围(重要,不许默认等同狗上)**:
- `lddc.cpp` → **repo==dog**(SHA `b5811eaf…` 与 `analysis/…/remote_source/lddc.cpp` 字节级相同)。第7步得狗端坐实。
- `laserMapping.cpp` → **repo≠dog**(repo `5fec8282…` ≠ 狗上 `e4cd05cb…`);**非文档直接引用文件**,仅借它取 `blind`/`point_filter_num` 的代码默认,所引 `declare_parameter` 默认行两版**逐字一致**(狗上 969-975 / 1011-1017)。
- 其余全部【无狗上对照】:`pub_handler.cpp`、`comm.h`、`data_handler.cpp`、`livox_lidar_def.h`、`livox_lidar_callback.cpp`、`lds_lidar.cpp`、`preprocess.{h,cpp}`、`MID360s_config.json`、`go2_mid360s.yaml`、`mid360.yaml`、`msg_MID360s_launch.py`、`base_bringup.sh`、`level_cloud_node.py`。**尤其第 2–6 步全在 `pub_handler.cpp` 的团队 patch 上,狗上是否装同版补丁无法验证。**
- **"狗上确用 `go2_mid360s.yaml"** 的证据链根(`base_bringup.sh:220` 的 `config_file:=go2_mid360s.yaml` + `msg_MID360s_launch.py` 的 `xfer_format=1`/`publish_freq=10`/`MID360s_config.json`)**全部仓库侧、无狗上副本**;靠 manifest 的 `fast_lio_input_qos=lidar_2…imu_400`(深度 2/400 **仅** `go2_mid360s.yaml` 有,`mid360.yaml` 无)**间接坐实** → 非直接确认。

**本轮订正三处(均非硬错,原结论方向对)**:
1. **第4步时戳映射的因果**:走 no_sync 分支是**驱动按每包 `timestamp_type` 判定**,不是 FAST-LIO 的 `time_sync_en=false`(层级混淆)。
2. **"帧基准"一词拆成两个量**:第5步 = `pkt.time_stamp`(绝对包时戳);第7步 = `pkg.base_time`(帧首点偏移)。
3. **`blind`/`point_filter_num` 标 DEFAULT_VS_PROD**:文档取的是生产 yaml 值(狗上实用、正确),补注节点代码默认不同。

## 一、一个点到底怎么"算"出来的
**关键认知:算 xyz 的是雷达固件,不是主机代码。**(此结论【推断-未验】:由 `pub_handler.cpp:435-458` 只做 ÷1000+外参、不含 ToF/棱镜解算,加 `pcl_data_type=1` 反推得出,非源码明写。)
- MID360 是**非重复扫描**(`pattern_mode=0`【生产 saas:MID360s_config.json:28,经 `msg_MID360s_launch.py:19` 装为 `user_config_path` 加载】;但"0=非重复扫描"是 **Livox 硬件约定**——SDK 里 `pattern_mode` 仅是结构体字段 `livox_lidar_def.h:344/368`、无枚举定义此语义 →【推断-未验】),4 条激光线(`kLineNumberMid360=4`【默认 code:comm.h:82】)靠棱镜扫玫瑰花瓣。打激光 → 测**飞行时间(ToF)→ 距离**;棱镜编码器给**光束方向**;在**雷达内部**算成三维点 + 反射率 + tag。
- 配置 `pcl_data_type=1`【生产 saas:MID360s_config.json:27】= `kLivoxLidarCartesianCoordinateHighData`(=0x01【默认 code:livox_lidar_def.h:193】)→ 雷达**直接输出笛卡尔坐标**,分流到 `ProcessCartesianHighPoint`(pub_handler.cpp:389)。每点 = `LivoxLidarCartesianHighRawPoint`:**x/y/z int32 单位毫米 + reflectivity(u8) + tag(u8) = 14 字节**【默认 code:livox_lidar_def.h:167-173;14 字节成立前提=该结构体处于 `#pragma pack(1)`(:32)…`#pragma pack()`(:491)之间,`sizeof=14`,`raw[i]` 步进才对】。
- (球坐标模式 `pcl_data_type=3` 才由**驱动**用 `r·sinθ·cosφ` 算 xyz——`src_x=radius*sin(theta)*cos(phi)` 在 `ProcessSphericalPoint`(data_type=0x03)【默认 code:pub_handler.cpp:496-501】;config 用 1 故此部署不走此路。)

## 二、从雷达到 `/livox/lidar` 的 7 步
1. **发包**:雷达 UDP 发 `LivoxLidarEthernetPacket`(头含 `dot_num` 点数、`time_interval` 每包时长[注释 `unit:0.1 us`]、`timestamp[8]`、`data_type`)【默认 code:livox_lidar_def.h:129-142】到 `192.168.1.5:56301`【生产 saas:MID360s_config.json:15 host_ip / :18 point_data_port】,每包 ≤100 点(`kMaxPointPerEthPacket=100`【默认 code:comm.h:48】)。IMU 走 56401【生产 saas:MID360s_config.json:19】,`LivoxLidarImuRawPoint{gyro xyz, acc xyz}`(6×float)【默认 code:livox_lidar_def.h:158-165】。
2. **SDK 收**(`Handle()`@`data_handler.cpp:69`)【默认 code:data_handler.cpp:69-95】:按 `data_type` 分流——`==kLivoxLidarImuData`→IMU 回调(:75),else→点回调(:80),再走观察者 `observers_`(:87-92)。
3. **驱动回调**(`OnLivoxLidarPointCloudCallback`@`pub_handler.cpp:101`)【默认 code:pub_handler.cpp:101,147,159-162】:组 `RawPacket`,算 `point_interval = time_interval×100/dot_num`(ns/点,:147),打包时间戳,入 `raw_packet_queue_`;积压 `size()>1024` 且 1s 节流打印 `Livox point packet backlog`(:159,**团队埋点**)。
4. **时间戳映射**(`GetEthPacketTimestamp`@`pub_handler.cpp:331`)【默认 code:pub_handler.cpp:331,335-343】:无外部同步时,用 `no_sync_timestamp_mapper_.Map(雷达内部 stamp, system_clock 墙钟)` 映射到**主机墙钟**。
   - ⚠️ **因果订正**:是否走 no_sync 分支由**驱动按每包 `timestamp_type` 判定**(`pub_handler.cpp:108,335`:非 `kTimestampTypeGptpOrPtp`/`kTimestampTypeGps` 才走 Map),取决于**雷达硬件 / `MID360s_config` 是否配 PTP/GPS**——这是**驱动层**开关。文档旧版归因于 FAST-LIO 的 `time_sync_en=false`(`go2_mid360s.yaml:19`)是**层级混淆**:那是 **FAST-LIO 层**的独立开关,**不驱动**这段 C++ 路径。二者都指向"无外部同步",结论对,但机制是**两层两个开关**。
5. **点处理**(`ProcessCartesianHighPoint:435`)【默认 code:pub_handler.cpp:435-458】:逐点
   - **mm→m**:`x = raw.x/1000`。注意 `pkt.extrinsic_enable=false`(:137)时**恒走旋转矩阵分支**(:445-453),不是那个"简单 ÷1000"的 `if` 分支(:441-443);但 `MID360s_config.json` 外参全 0 经 `lds_lidar.cpp:165-179`→`AddLidarsExtParam`→`SetLidarsExtParam` 算得**单位阵 + 零平移**,故数值上**等价 raw/1000**。
   - `intensity=反射率`(:455),`line = i % pkt.line_num`(line_num=4,:456),`tag=tag`(:457);
   - **逐点时刻** `offset_time = 包时戳基准 + i×point_interval`(:458)。⚠️ **此处"帧基准"= `pkt.time_stamp`**(第4步映射出的**绝对包时戳**),与第7步不是同一个量。
6. **组帧**(`CheckTimer:221`)【默认 code:pub_handler.cpp:221,271,274-277,207-213,191,69】:攒够 ~100ms(10Hz)发一帧,**正常一帧 ≈2 万点**(注释 "roughly 20k raw points" :271);
   - ⚠️ **团队 patch**:`帧跨度 > publish_interval_×2 || size < 5000` → 打印 `Dropping malformed Livox frame` 丢弃(:274-275,防坏帧毁掉逐点去畸变);每秒(`elapsed≥1.0` 节流 :191)打 `LIVOX_STREAM_HEALTH frame_hz/imu_hz/span_ms/points/lidar_imu_offset_ms/queue_depth`(六字段 :207-213)。
   - ~100ms 由 `SetPointCloudConfig`(:69)在 `publish_freq=10`【生产 saas:msg_MID360s_launch.py:11】下得 `publish_interval_=1e8ns`。
7. **发布**(`lddc.cpp`)【狗上 dog:SHA `b5811eaf…` 与狗上副本字节级相同 → **repo==dog**】:FAST-LIO 用 **CustomMsg**(`xfer_format=1` customized【生产 saas:msg_MID360s_launch.py:8】,`lidar_type=1`=AVIA【默认 code:preprocess.h:16】);`FillPointsToCustomMsg` 每点 `offset_time = points[i].offset_time − pkg.base_time`(ns,:468);⚠️ patch:发布前**跳到最新帧**丢旧帧(`skipped_before_publish`/`QueuePopUpdate` :225-233)。→ `/livox/lidar` + `/livox/imu`(【生产 saas:go2_mid360s.yaml:13-14】)。⚠️ **此处"帧基准"= `pkg.base_time`**(帧首点偏移),与第5步的 `pkt.time_stamp` **非同一量**;两步算式各自忠实、差值确为相对 ns,但**同名"帧基准"指代两层含义,勿混**。

> **base_bringup 的 Livox 校验(doc 01)校的就是第 3/6 步这些打印字符串**——都是团队**给驱动打补丁**加的埋点。⚠️ 第 2–6 步的 `pub_handler.cpp` 全部【无狗上对照】,狗上是否装同版 patch 无法验证;仅第7步 `lddc.cpp` 有狗上字节级坐实。

## 三、FAST-LIO 入口怎么筛这帧(`preprocess.cpp: avia_handler`)
`lidar_type=1`(AVIA)【生产 saas:go2_mid360s.yaml:24】+【默认 code:preprocess.h:16 / laserMapping.cpp:998,两者一致 → 无 DEFAULT_VS_PROD】、`feature_enabled=false`,逐点:
> **命名注**:文档"`feature_enabled`"是 `Preprocess` 成员名,yaml/param 键实为 `feature_extract_enable`【生产 saas:go2_mid360s.yaml:3】/【默认 code:laserMapping.cpp:1003,两版一致】——命名差异,非错。
- **回波过滤**:`tag & 0x30 ∈ {0x00,0x10}` 才要(feature 关闭走 else 分支 :168 起,丢多回波/噪声 tag)【默认 code:preprocess.cpp:171】;
- **抽稀**:每 `point_filter_num` 留 1(条件 `valid_num % point_filter_num == 0` :173-174)。⚠️ **DEFAULT_VS_PROD**:节点 `declare`/`get_parameter_or` 代码默认 **2**【默认 code:laserMapping.cpp:1002,1044】、`Preprocess` 构造函数硬编码 **1**【默认 code:preprocess.cpp:8】、**生产 yaml=3**【生产 saas:go2_mid360s.yaml:4】。**狗上实际生效 3**(yaml 值经 laserMapping 覆盖构造函数默认;base_bringup.sh:220 加载该 yaml、manifest qos 佐证)→ 2万 → 约 6~7 千。
- **盲区**:`x²+y²+z² > blind²` 才留(:186)。⚠️ **DEFAULT_VS_PROD**:节点 `declare`/`get` 默认 **0.01**【默认 code:laserMapping.cpp:997,1039】、`Preprocess` ctor **0.01**【默认 code:preprocess.cpp:8】、**生产 yaml=0.5m**【生产 saas:go2_mid360s.yaml:26】。**狗上实际生效 0.5**。
- **去重**:与前点同坐标丢(`|dx|>1e-7 || |dy|>1e-7 || |dz|>1e-7` 才留,与盲区条件 **AND** 合并在同一 `if` :183-186)【默认 code:preprocess.cpp:183-185】;
- **逐点时间**:`curvature = offset_time/1e6`(ns→**ms**,注释 "curvature unit: ms"),存进 PCL `curvature` 字段 → 给 IMU 去畸变用【默认 code:preprocess.cpp:180-181】;
- 输出 `pl_surf`(面点,:188;`process()` 里 `*pcl_out=pl_surf` :50),交卡尔曼更新【默认 code:preprocess.cpp:188,50】(见 19,待读:laserMapping + IMU_Processing + iKF + ikd-Tree)。
- (`feature_enabled=true` 才跑 `give_feature`(:160)提取平面/边缘特征;此部署关闭走 else 分支(:166 起),用全部筛后点做 scan-to-map【默认 code:preprocess.cpp:115,160,166】+【生产 saas:go2_mid360s.yaml:3】。)

## 四、坐标系与外参(和定位相关)
- 雷达装配外参(`MID360s_config.json` roll/pitch/yaw=0、x/y/z=0)→ 经 `lds_lidar` 装入 pub_handler 得单位阵,点未被外参旋转 → 留**雷达自身系**【生产 saas:MID360s_config.json:29-36 + lds_lidar.cpp:165-179】。
- FAST-LIO `extrinsic_T=[-0.011,-0.02329,0.04412]`、`extrinsic_R=I`(`extrinsic_est_en=false` :37)是 **IMU↔雷达** 外参,在 laserMapping/IMU_Processing 里用于把点变换到 IMU 系再做 SLAM【生产 saas:go2_mid360s.yaml:37-41】。
- 雷达装歪 ~12.3° 的校正(`level_cloud_node`,doc 16)是**下游**给建图/2D 用的,**不改** FAST-LIO 的输入(订阅 `/cloud_registered_body`=FAST-LIO 输出、不订阅 `/livox/lidar`)【默认 code:level_cloud_node.py:16 input / :17 output / :19 pitch_deg 默认 12.3】。⚠️ 12.3 是**代码默认**,狗上是否被 `-p` 覆盖【无狗上对照】。

## 五、留待坐实(下一单元)
- `laserMapping.cpp`(主 SLAM 循环)+ `IMU_Processing.hpp`(前向传播 + 反向去畸变)+ `use-ikfom.hpp`/`esekfom.hpp`(迭代误差状态卡尔曼 IESKF)+ `ikd_Tree.cpp`(增量 kd 树地图):**点 + IMU 怎么算出 /Odometry**。见 doc 19。

## 核验台账
> claim → 证据 file:line → 判定 / 源标签。路径见文首 files 列表;除标注外均**仓库侧、无狗上对照**。

| # | 断言 | 证据 file:line | 判定 |
|---|------|----------------|------|
| 1 | MID360 非重复扫描 `pattern_mode=0` | MID360s_config.json:28(值)+ livox_lidar_def.h:344/368(无枚举) | CONFIRMED【生产 saas】;"0=非重复扫描"语义【推断-未验】 |
| 2 | 4 线 `kLineNumberMid360=4` | comm.h:82 | CONFIRMED【默认】 |
| 3 | `pcl_data_type=1`=CartesianHighData → 雷达出笛卡尔 | MID360s_config.json:27 + livox_lidar_def.h:193 | CONFIRMED【生产+默认】 |
| 4 | 每点=14 字节(x/y/z int32 + refl u8 + tag u8) | livox_lidar_def.h:167-173(pack :32/:491) | CONFIRMED【默认】 |
| 5 | 算 xyz 在雷达固件、非主机 | pub_handler.cpp:435-458 + config:27 | CONFIRMED【推断-未验】 |
| 6 | 球坐标 `pcl_data_type=3` 才驱动算 xyz | pub_handler.cpp:496-501 | CONFIRMED【默认】;此部署未用 |
| 7 | UDP→`192.168.1.5:56301`、≤100 点 | livox_lidar_def.h:129-142 + comm.h:48 + config:15,18 | CONFIRMED【默认+生产】 |
| 8 | IMU→56401、`LivoxLidarImuRawPoint` 6×float | config:19 + livox_lidar_def.h:158-165 | CONFIRMED【生产+默认】 |
| 9 | SDK `Handle()` 按 data_type 分流 | data_handler.cpp:69-95 | CONFIRMED【默认】 |
| 10 | 驱动回调组包 + backlog>1024 打印 | pub_handler.cpp:101,147,159-162 | CONFIRMED【默认,团队 patch】 |
| 11 | 时戳映射 no_sync→墙钟 | pub_handler.cpp:331,335-343(+:108) / yaml:19 | CONFIRMED【默认】;**因果订正**(time_type ≠ FAST-LIO time_sync_en) |
| 12 | `ProcessCartesianHighPoint` 逐点(÷1000+外参/line/tag/offset) | pub_handler.cpp:435-458 + lds_lidar.cpp:165-179 + :137 | CONFIRMED【默认】;帧基准=`pkt.time_stamp` |
| 13 | `CheckTimer` 组帧/丢坏帧/health 打印 | pub_handler.cpp:221,271,274-277,207-213,191,69 + msg_MID360s_launch.py:11 | CONFIRMED【默认+生产 publish_freq】 |
| 14 | `lddc` 发 CustomMsg、offset−base_time、跳最新帧 | lddc.cpp:219-233,457-472 + launch:8 + preprocess.h:16 + yaml:13-14 | CONFIRMED【**狗上 repo==dog** SHA b5811eaf…】;帧基准=`pkg.base_time` |
| 15 | `avia_handler` lidar_type=1(AVIA)+feature off | yaml:24,3 + preprocess.h:16 + laserMapping.cpp:998,1003 | CONFIRMED【生产+默认一致】;键名实为 `feature_extract_enable` |
| 16 | 回波 `tag&0x30∈{0x00,0x10}` 才要 | preprocess.cpp:171 | CONFIRMED【默认】 |
| 17 | 抽稀 `point_filter_num` 留 1 | preprocess.cpp:173-174 / laserMapping.cpp:1002,1044 / yaml:4 | **DEFAULT_VS_PROD**:默认 2 / ctor 1 / 生产 3 → **狗上 3** |
| 18 | 盲区 `x²+y²+z²>blind²` | preprocess.cpp:186 / laserMapping.cpp:997,1039 / yaml:26 | **DEFAULT_VS_PROD**:默认 0.01 / 生产 0.5 → **狗上 0.5** |
| 19 | 去重(与前点同坐标丢) | preprocess.cpp:183-185 | CONFIRMED【默认】 |
| 20 | `curvature=offset_time/1e6`(ns→ms) | preprocess.cpp:180-181 | CONFIRMED【默认】 |
| 21 | 输出 `pl_surf` 面点交卡尔曼 | preprocess.cpp:188,50 | CONFIRMED【默认】 |
| 22 | feature on 才 `give_feature`;此关闭 | preprocess.cpp:115,160,166 + yaml:3 | CONFIRMED【默认+生产】 |
| 23 | 雷达外参全 0 → 点留雷达系 | MID360s_config.json:29-36 + lds_lidar.cpp:165-179 | CONFIRMED【生产 saas】 |
| 24 | FAST-LIO `extrinsic_T/R` 为 IMU↔雷达外参 | go2_mid360s.yaml:37-41 | CONFIRMED【生产 saas】;`extrinsic_est_en=false` |
| 25 | 12.3° level_cloud 不改 FAST-LIO 输入 | level_cloud_node.py:16,17,19 | CONFIRMED【默认】;12.3 狗上是否 `-p` 覆盖【无狗上对照】 |
| 26 | 文档头所列文件均已逐行读 | 均在盘、路径正确 | CONFIRMED【推断-未验】 |

# Go2 地图定位器

这是一套正式应用基线，用于根据预先构建的校园 PCD 地图，对搭载 Livox
MID-360 的 Unitree Go2 EDU 进行定位。目标运行环境是 Ubuntu 20.04、
ROS 2 Foxy、C++17 和 PCL 1.10。

## 数据流

该节点按时间将 `/cloud_registered_body`（`base_link`）与 `/Odometry`
（`odom -> base_link`）配对，并在当前机身坐标系中累积多组已配对扫描。
洁净室实现的极坐标描述子先检索候选地图瓦片，然后依次使用粗 NDT、精细
NDT 和 GICP 估算 `map -> base_link`。节点据此推导并持续广播：

```text
map -> odom = (map -> base_link) * inverse(odom -> base_link)
```

现有跟随器只要通过 TF 变换地图坐标系中的路线，就可以继续使用 FAST-LIO
里程计。新的路线控制器应使用 `/localization/pose`，并要求
`/localization/status.safe_to_move` 成立。

该节点必须是 `map -> odom` 的**唯一**广播者。启动前应禁用旧版
`odom_to_tf_map*` 辅助程序；两个发布所有者会导致 TF 结果不确定。

## 地图契约

地图目录必须包含：

```text
manifest.json
manifest.sha256
descriptor_index.json
descriptor_index.json.sha256
review_assets/stable_layer.pcd
tiles/<tile-id>.pcd
```

正式加载器要求使用 `go2.map_tiles/v2`。其清单包含
`descriptor_index: {path, sha256}`，因此用作路线/地图身份的清单哈希也会
绑定确切的描述子索引。加载器会严格验证
`go2.polar_descriptor_index/v1`、地图 ID、每个瓦片恰好一个描述子的完整
覆盖、描述子中心、向量长度、瓦片源哈希、点数、安全相对路径，以及在启用
验证时的每个 SHA-256。描述子索引还必须严格绑定
`source_layer: {path, sha256, point_count, role}`；生产角色为
`global_retrieval`。启用地标验证时，加载器会再次核对该 PCD 在地图根目录
以内、哈希和有限点数一致，然后建立三维 KdTree。旧版
`go2.map_tiles/v1` 只能在禁用验证的情况下
用于诊断，因此绝不能授权运动。加载过程具有事务性：有问题的替换地图不会
破坏旧地图，但地图加载请求一开始，当前定位就会失效；即使替换地图加载
失败，也必须重新执行一次全局重定位。
允许加载禁用哈希验证的地图用于诊断，但这种地图绝不能产生
`safe_to_move=true`。

正式 v2 制品包目录属于不可变发布版本：新地图必须编译到新目录，绝不能
替换活动地图目录下的文件。加载器会先验证所有瓦片和描述子哈希，再原子
切换地图对象并清空瓦片缓存。每个已验证瓦片在第一次延迟读取前后还会再次
进行哈希检查，随后才从进程内缓存提供。这种进程完整性并不是物理运动
联锁：操作员开始地图维护前，真实机器人的网关必须已经处于 `DISARMED`。

极坐标描述子是本项目采用 Apache-2.0 许可证的原创实现。它只使用径向/
角度分箱的一般数学思想，没有复制 Scan Context 代码。

## 构建与运行

```bash
cd ~/orin_go2_fastlio_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install \
  --packages-select go2_nav_interfaces go2_map_localizer
source install/setup.bash
ros2 launch go2_map_localizer localizer.launch.py \
  map_manifest:=/data/campus-map/manifest.json \
  params_file:=/data/robot-config/localizer.yaml
```

随附配置有意采用失效闭锁策略：
`input_extrinsics_verified: false`。系统预期 `/cloud_registered_body` 已经
由 FAST-LIO 变换到真实机器人的 `base_link`；本节点不会假定
LiDAR/IMU/机身之间是恒等变换。把该标志改为 `true` 之前，必须核对当前
FAST-LIO YAML 中 MID-360 到 IMU 的外参旋转和平移、真实 `base_link`
约定、运动状态下的时间戳，以及实测的直行/转弯 bag。该标志为 false 时，
节点保持 `LOST`，拒绝全局重定位，并发布 `safe_to_move=false`。
系统拒绝在运行时修改参数，因为配准阈值和安全阈值是作为一套完整配置统一
审查的；应改为更新 YAML 并重启。

服务：

```bash
ros2 service call /localization/global_relocalize \
  go2_nav_interfaces/srv/GlobalRelocalize \
  "{use_initial_guess: false, search_radius: 0.0}"

ros2 service call /localization/reset \
  go2_nav_interfaces/srv/ResetLocalization "{clear_map: false}"

ros2 service call /localization/load_map \
  go2_nav_interfaces/srv/LoadMap \
  "{manifest_path: /data/campus-map-v2/manifest.json, verify_hashes: true}"

ros2 service call /localization/set_active \
  std_srvs/srv/SetBool "{data: true}"
```

`/localization/set_active` 只在 `manual_activation.enabled=true` 时可用。它服务于
“起点 + 显式 checkpoint 停车重定位”：inactive 时点云回调会在转换、
降采样、描述子、NDT 和 GICP 之前返回；activate 会清除旧扫描和旧位姿，因此
下游必须继续保持停车，随后 reset、采集新静止扫描并调用全局重定位。完成
修正抓取后再 deactivate。默认生产/Shadow 配置保持该功能关闭，旧行为不变。

在 `global.auto_relocalize=false` 的手动模式中，协调器需要对每批新扫描重复
调用 `/localization/global_relocalize`。第一项通过的结果只建立 provisional
锚点；后续调用进入独立确认，直到状态报告
`startup_precision_verified=true`。一次服务 `accepted=true` 仍不能单独
授权运动。

协调器在起点和 checkpoint 使用 `use_initial_guess=true` 的锚定模式。这个
模式根据 CSV/当前冻结变换提供的有界先验，在完整 `cleaned_static_map` 局部
区域运行 NDT/GICP；它不依赖全局描述子排名。配准质量、修正幅度和置信度先
通过后，`stable_layer.pcd` 再独立验证固定物证据：1.50 m 地标邻域中，
0.60 m 内至少 40 个匹配、16 个不同的 0.25 m 三维体素、10% 邻域匹配率，
且三维最大跨度至少 1.5 m。这样水平墙/路牙和竖直灯杆都可形成证据，而少量
密集背景点不能冒充完整对象。任一条件失败都会拒绝锚定修正。

运行时替换地图有意设计为需要调用两次服务的维护流程。首先调用
`/localization/reset`；它会立即把状态变为不安全，并根据单调时间启动
隔离计时。保持运动网关处于 `DISARMED`，等待至少
`services.map_load_quarantine_sec`（配置值绝不能低于 2 秒），然后调用
`/localization/load_map`。如果正在跟踪、仍有临时全局位姿等待确认、
此前没有执行 reset，或等待时间尚未结束，加载请求都会被拒绝。手动调用
全局重定位会取消维护窗口，并恢复使用当前地图。

全局服务使用最新的同步扫描数据包。在积累足够 LiDAR 数据之前，服务会
持续返回拒绝。没有初始位姿的全局搜索不能绕过地点描述子或歧义检查；带
初始位姿的锚定模式按上一段的完整地图配准与审核地标证据门执行。一个通过的
无先验全局匹配也只是一项临时假设：在五次由描述子支撑的
独立全局搜索均与固定锚点一致、且时间跨度至少达到两秒之前，节点不会发布
位姿或 `map -> odom`，并始终保持 `safe_to_move=false`。局部跟踪绝不会
推进该确认计数器。

针对当前巡检跟随器，启动确认还增加了一个更严格的**重复解一致性门**：
五次全局搜索得到的 `map -> odom` 都必须相对第一次结果保持在 0.10 m 和
0.00523598776 rad（0.3°）以内，才会设置
`startup_precision_verified=true`。任何一次超限都会清空整轮临时结果，
从新的全局初始解重新开始；它不会用连续相邻结果“慢慢漂过去”。这项检查
只证明多次算法结果彼此一致，不能证明它们在真实校园中的绝对位置和朝向
正确。绝对 0.3° 要求必须用全站仪、测量控制点或另一套独立真值系统验证；
在 100 m 无反馈直行中，单是 0.3° 初始偏航就约对应 0.52 m 横向误差。

## 质量与失效安全行为

配准必须通过收敛性、适应度、内点比例、横滚/俯仰，以及跟踪期间的修正跳变
门槛。全局检索还要求达到配置的占用单元重叠率和描述子总覆盖率（默认值为
0.05）。缺失或无效的描述子证据对置信度贡献为零；绝不会把它当作完美
匹配，也不会通过重新归一化将其影响消除。反复拒绝会使状态依次经过
`DEGRADED` 并进入 `LOST`；丢失的位姿不再发布，同时恢复自动全局重定位。
适应度数值和阈值均采用以米为单位的 GICP 对应点 RMSE（评估前会对 PCL
给出的均方结果开平方）。`safe_to_move` 还要求：同步位姿新鲜、修正新鲜、
置信度充足、地图完整性已经验证、RMSE 不大于 0.40 m、内点比例至少为
0.35，并且最近的修正不超过 0.25 m 或 7 度。跟踪接受门槛和跳变门槛的
默认值分别为 0.40 m RMSE、0.50 m 平移和 15 度旋转。相差超过 0.25 m
或 5 度的全局结果会视为不同竞争候选，并且必须同时满足 0.10 的绝对
置信度差值和相对最优结果 20% 的置信度差值。各次独立确认都必须保持在
固定锚点的 0.25 m/7 度范围内。比当前时间超前 0.10 s 以上或陈旧超过
0.15 s 的输入时间戳会被拒绝；任何时间位于未来的位姿/修正都会使状态
变为不安全，ROS 时钟纪元重置则会清空配对缓冲区并使定位失效。这些默认值
只是保守起点；在校园自主运行前，必须使用录制的 bag 完成标定。

当前离线编译器会在每个 20 m 瓦片中心创建一个描述子，并设定
`center.z=0`。该描述子可以容忍偏航角变化，但不具备平移不变性，而且这一
高度原点不适用于任意坡地或多层空间。因此，在地图流水线生成由轨迹/测量
数据支撑、稠密且高度一致的地点记录之前，这套基线不能通过真实落地状态的
`ARM`；另一种放行路径是由真实现场测试覆盖每个瓦片边缘、角点、高程和
重复立面，并取得零错误接受以及经过验证的 top-k 召回率。

同步位姿的安全时效默认值为 0.15 s，状态由墙上时钟定时器以 20 Hz 发布。
因此，在计入 DDS 和调度开销之前，定位器从里程计陈旧到发布不安全状态的
配置最坏间隔约为 0.20 s。这为下游 0.30 s 的陈旧状态制动门槛留下了名义
预算，但只有端到端 HIL 延迟测量（包括调度器、中间件、网关和执行器的
p99）才能证明满足 300 ms 要求。

默认情况下，全局搜索会对保留的四个地点候选逐一评估完整的 3x3 平移种子
格点（中心、四个正方向和四个对角方向种子）。这样可以减小以瓦片中心为
基准的捕获范围盲区，但不能证明能从校园中任意位置启动。在瓦片边缘/角点、
特征稀疏区域或重复立面上的回放成功，是阻断发布的验收项目；如果这些 bag
未通过，应提高描述子/关键帧密度，或引入粗粒度全局阶段，而不能放宽歧义
门槛或安全门槛。

构建离线描述子时，应使用接近 MID-360 在真实校园内可重复探测范围的半径
（通常可从约 40 m 开始），不能假定传感器宣传的远距离回波始终可用。

定位器不会向机器人发送控制命令。安全监督器仍是唯一有权把定位健康状态
转换为运动许可的组件。

## 非 ROS 算法检查

描述子与变换约定测试有意不依赖 ROS、PCL 或 Eigen：

```bash
./scripts/run_standalone_tests.sh
```

部署验收时，应回放具有代表性的 rosbag2 录制数据，其中包括正常路线、
初始朝向反转、绑架式异地启动、行人、停放车辆、特征稀疏路段，以及人为
制造的时间戳丢失。

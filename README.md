# GO2 EDU + MID-360S + FAST-LIO 固定路线巡检部署 README

> 继续现场调试前，先阅读
> [HANDOFF_FOR_NEXT_DEVELOPER.md](HANDOFF_FOR_NEXT_DEVELOPER.md) 第 0 节。
> 其中记录了 2026-07-24 严格回退后的真实狗端状态、SaaS 启动失败证据，以及尚未实施的
> Livox 实时队列修复方案。本 README 的部分“当前状态”描述早于该交接记录。

## 1. 交付前提

### 1.1 硬件

- Unitree GO2 EDU
- Orin （16G内存版本）拓展坞
- Livox MID-360S 雷达
- MID-360S 与 Orin 网络互通

### 1.2 软件环境

推荐环境：

```text
Orin: Ubuntu 20.04 + ROS2 Foxy
本地调试电脑: Ubuntu 22.04 + ROS2 Humble，可用于 RViz2
```

工程默认路径：

```bash
/home/unitree/go2_fastlio_ws
```

如果实际路径不同，请把本文档中的路径同步替换。

---

## 2. 网络配置：Orin 与 MID-360S 通信

### 2.1 查看 Orin 网卡

在 Orin 上执行：

```bash
ip addr
```

确认连接 MID-360S 的网卡名称，例如：

```text
eth0
enP8p1s0
eno1
```

下文用 `<NET_IF>` 表示该网卡名。

### 2.2 确认 MID-360S IP

MID-360S 的 IP 以实际配置为准。常见情况包括：

```text
192.168.123.20
192.168.1.161
```

请以客户设备实际雷达 IP 为准。

### 2.3 给 Orin 配置同网段 IP

例如 MID-360S 是：

```text
192.168.123.20
```

则 Orin 可配置为：

```bash
sudo ip addr add 192.168.123.18/24 dev <NET_IF>
```

例如网卡是 `eth0`：

```bash
sudo ip addr add 192.168.123.18/24 dev eth0
```

如果 MID-360S 是：

```text
192.168.1.161
```

则 Orin 可配置为：

```bash
sudo ip addr add 192.168.1.50/24 dev <NET_IF>
```

### 2.4 测试网络连通

```bash
ping <MID360_IP>
```

例如：

```bash
ping 192.168.123.20
```

正常应能持续收到响应。

如果 ping 不通，优先检查：

```text
1. 网线/交换机连接
2. Orin 网卡名是否正确
3. Orin IP 是否和雷达同网段
4. 雷达 IP 是否与驱动配置一致
5. 防火墙是否阻断
```

可临时关闭防火墙测试：

```bash
sudo ufw disable
```

---

## 3. 安装依赖

在 Orin 上执行：

```bash
sudo apt update && sudo apt install -y \
  git build-essential cmake make gcc g++ pkg-config curl wget lsb-release software-properties-common \
  python3-pip python3-dev python3-setuptools python3-colcon-common-extensions python3-rosdep python3-vcstool python3-argcomplete \
  libeigen3-dev libpcl-dev libboost-all-dev libapr1-dev libyaml-cpp-dev libomp-dev \
  ros-foxy-desktop ros-foxy-rclcpp ros-foxy-rclcpp-components \
  ros-foxy-ament-cmake ros-foxy-ament-cmake-auto \
  ros-foxy-rosidl-default-generators ros-foxy-rosidl-default-runtime \
  ros-foxy-std-msgs ros-foxy-sensor-msgs ros-foxy-geometry-msgs ros-foxy-nav-msgs ros-foxy-visualization-msgs \
  ros-foxy-pcl-ros ros-foxy-pcl-conversions \
  ros-foxy-tf2 ros-foxy-tf2-ros ros-foxy-tf2-eigen ros-foxy-tf2-geometry-msgs \
  ros-foxy-rosbag2 ros-foxy-rosbag2-storage-default-plugins ros-foxy-rviz2
```

初始化 rosdep：

```bash
sudo rosdep init 2>/dev/null || true
rosdep update
```

---

## 4. 工程部署与编译

### 4.1 放置工程

将交付的工程文件夹放到：

```bash
/home/unitree/go2_fastlio_ws
```

确认目录存在：

```bash
ls -lh ~/go2_fastlio_ws
ls -lh ~/go2_fastlio_ws/src
```

### 4.2 编译

```bash
cd ~/go2_fastlio_ws

source /opt/ros/foxy/setup.bash

rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install
```

编译完成后加载环境：

```bash
source ~/go2_fastlio_ws/install/setup.bash
```

### 4.3 检查关键可执行程序

```bash
source /opt/ros/foxy/setup.bash
source ~/go2_fastlio_ws/install/setup.bash

ros2 pkg executables go2_fastlio_patrol
```

应能看到类似：

```text
go2_fastlio_patrol route_recorder
go2_fastlio_patrol waypoint_follower
go2_fastlio_patrol unitree_safe_cmd_node
```

如果看不到，请重新检查包是否编译成功。

---

## 5. 每个终端的通用环境

后续每个终端启动 ROS2 节点前，都建议执行：

```bash
cd ~/go2_fastlio_ws

source /opt/ros/foxy/setup.bash
source /unitree/module/graph_pid_ws/install/setup.bash
source install/setup.bash

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
```

说明：

- `source /opt/ros/foxy/setup.bash`：加载 ROS2 Foxy。
- `source /unitree/module/graph_pid_ws/install/setup.bash`：加载 GO2 运动控制相关环境。如果客户机器没有该目录，请替换为实际 GO2 控制工作空间路径。
- `source install/setup.bash`：加载本工程编译结果。
- `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`：与当前工程调试环境保持一致。

---

## 6. 固定起点要求

本方案依赖 FAST-LIO 的局部坐标系，因此必须采用固定起点策略。

现场请执行：

```text
1. 在地面贴出 GO2 起点框。
2. 在地面贴出 GO2 机头方向箭头。
3. 每次录制和回放前，把 GO2 放在同一个起点框内。
4. GO2 机头方向对准同一个箭头方向。
5. 启动 FAST-LIO 后保持静止 5~10 秒，等待 LiDAR-IMU 初始化稳定。
```

录制路线和自动巡线必须使用相同的起点和朝向。否则 `route.csv` 坐标系会和当前 FAST-LIO 坐标系不一致，导致路线偏移或反向。

---

## 7. 启动 MID-360S 雷达驱动

终端 1：

```bash
cd ~/go2_fastlio_ws

source /opt/ros/foxy/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

ros2 launch livox_ros_driver2 msg_MID360s_launch.py
```

验证：

```bash
ros2 topic list | grep -E "/livox/lidar|/livox/imu"
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
```

正常应看到：

```text
/livox/lidar
/livox/imu
```

如果没有数据，请回到第 2 节检查网络和雷达 IP 配置。

---

## 8. 启动 FAST-LIO

终端 2：

```bash
cd ~/go2_fastlio_ws

source /opt/ros/foxy/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

ros2 launch fast_lio mapping.launch.py config_file:=go2_mid360s.yaml rviz:=false
```

启动后让 GO2 静止 5~10 秒。

验证：

```bash
ros2 topic list | grep -E "/Odometry|/cloud_registered|/cloud_registered_body|/path"
ros2 topic hz /Odometry
```

正常应看到：

```text
/Odometry
/cloud_registered
/cloud_registered_body
```

其中：

```text
/Odometry 是 FAST-LIO 输出的位姿话题
/cloud_registered_body 是车体/传感器机体系下的点云，用于前方急停检测
```

---

## 9. 录制巡检路线

录制路线时，人工遥控 GO2 沿目标路线走一遍。

建议：

```text
1. GO2 尽量走在道路中间。
2. 录制速度不宜超过 1.0 m/s。
3. 推荐录制速度 0.2~0.5 m/s。
4. 路线转弯处放慢速度。
5. 起点和终点处不要反复来回移动。
```

终端 3：启动路线录制

```bash
cd ~/go2_fastlio_ws

source /opt/ros/foxy/setup.bash
source /unitree/module/graph_pid_ws/install/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

mkdir -p ~/go2_fastlio_ws/src/go2_fastlio_patrol/routes

ros2 run go2_fastlio_patrol route_recorder --ros-args \
  -p min_distance:=0.4 \
  -p route_file:=/home/unitree/go2_fastlio_ws/src/go2_fastlio_patrol/routes/route_test_0517.csv
```

参数说明：

```text
min_distance:
  路线点记录间隔，单位 m。
  不宜过短。当前测试推荐 0.4。

route_file:
  路线文件保存路径。
```

录制完成后，在录制终端按：

```text
Ctrl+C
```

检查路线文件：

```bash
ls -lh /home/unitree/go2_fastlio_ws/src/go2_fastlio_patrol/routes/route_test_0517.csv
head /home/unitree/go2_fastlio_ws/src/go2_fastlio_patrol/routes/route_test_0517.csv
tail /home/unitree/go2_fastlio_ws/src/go2_fastlio_patrol/routes/route_test_0517.csv
```

路线文件应包含连续的 `x, y, yaw` 数据。若出现明显跳变、重复点或启动前几秒的抖动点，应人工删除异常行。

---

## 10. 自动巡线流程

自动巡线前请重新把 GO2 放回固定起点，机头对准固定方向箭头。  
然后**重新启动**雷达和 FAST-LIO，静止等待 5~10 秒。

### 10.1 启动雷达与 FAST-LIO

如已启动并稳定，可跳过。否则按第 7、8 节重新启动。

### 10.2 启动安全控制节点

终端 3：

```bash
cd ~/go2_fastlio_ws

source /opt/ros/foxy/setup.bash
source /unitree/module/graph_pid_ws/install/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

ros2 run go2_fastlio_patrol unitree_safe_cmd_node --ros-args \
  -p max_vx:=0.5 \
  -p max_yaw_rate:=0.45 \
  -p publish_rate:=20.0 \
  -p pointcloud_topic:=/cloud_registered_body \
  -p stop_distance:=0.40 \
  -p resume_distance:=0.50 \
  -p min_stop_points:=15 \
  -p roi_x_min:=0.35 \
  -p roi_x_max:=0.90 \
  -p roi_y_min:=-0.30 \
  -p roi_y_max:=0.30 \
  -p roi_z_min:=0.30 \
  -p roi_z_max:=0.90
```

参数说明：

```text
max_vx:
  最大前进速度限幅，最终速度不会超过该值。

max_yaw_rate:
  最大偏航角速度限幅，最终转向速度不会超过该值。

publish_rate:
  控制指令发布频率。

pointcloud_topic:
  用于急停检测的点云话题。

stop_distance / resume_distance:
  前方障碍物急停与恢复距离。

min_stop_points:
  前方 ROI 内超过该点数才触发急停，可减少误检。

roi_x_min / roi_x_max:
  前方检测区域的前后范围。

roi_y_min / roi_y_max:
  前方检测区域的左右范围。

roi_z_min / roi_z_max:
  前方检测区域的高度范围。
```

注意：`unitree_safe_cmd_node` 是实际安全控制节点。它应在 `waypoint_follower` 之前启动。

### 10.3 启动 waypoint_follower 巡线

终端 4：

```bash
cd ~/go2_fastlio_ws

source /opt/ros/foxy/setup.bash
source /unitree/module/graph_pid_ws/install/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

ros2 run go2_fastlio_patrol waypoint_follower --ros-args \
  -p route_file:=/home/unitree/go2_fastlio_ws/src/go2_fastlio_patrol/routes/route_test_0517.csv \
  -p v_base:=0.5 \
  -p max_vx:=0.5 \
  -p k_yaw:=0.9 \
  -p max_yaw_rate:=0.45 \
  -p lookahead_distance:=0.6 \
  -p reach_distance:=0.4 \
  -p goal_distance:=0.25 \
  -p loop_mode:=pingpong \
  -p search_window:=6 \
  -p turn_in_place_angle:=1.0 \
  -p slow_down_angle:=0.5 \
  -p stuck_time:=3.0 \
  -p relocalize_distance:=1.5
```

参数说明：

```text
route_file:
  巡线路线文件。

v_base:
  waypoint_follower 期望巡线速度。

max_vx:
  waypoint_follower 内部速度上限。
  实际最终速度仍受 unitree_safe_cmd_node 的 max_vx 限制。

k_yaw:
  偏航角误差增益。过大容易左右摆，过小转弯跟不上。

max_yaw_rate:
  waypoint_follower 内部偏航角速度上限。
  实际最终转向速度仍受 unitree_safe_cmd_node 限制。

lookahead_distance:
  前视距离。过小容易抖动，过大容易切弯。

reach_distance:
  到达中间路线点的判定距离。

goal_distance:
  到达终点的判定距离。

loop_mode:
  noop：到终点后停止。
  pingpong：到终点后自动反向返回起点。

search_window:
  当前路线点附近搜索窗口，用于减少跳点。

turn_in_place_angle:
  角度误差大于该值时优先原地转向。

slow_down_angle:
  角度误差大于该值时降低前进速度。

stuck_time:
  判断卡住/长时间无进展的时间阈值。

relocalize_distance:
  允许在路线附近重新匹配目标点的距离阈值。
```

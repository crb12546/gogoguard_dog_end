# Realtime Dog-End Code Snapshot

本目录是从机器狗 Orin 背板当前工作区只读抓取的独立代码快照，用于让没有现场设备访问权限的同事准确查看狗端实际存在的源码、脚本、配置和第三方依赖。

## 快照来源

- 设备：`go2wired`（远端主机名 `ubuntu`，用户 `unitree`）
- 狗端路径：`/home/unitree/go2_fastlio_ws`
- 抓取开始：`2026-07-25T22:37:07+08:00`
- 抓取结束：`2026-07-25T22:37:17+08:00`
- 抓取方式：只读 `rsync`
- 抓取后验证：全部纳入文件与狗端逐文件 SHA-256/rsync checksum 一致
- 当时本地 Git 基线：`b5b87fc15c67afcc0633c0a635afb90b36b222ca`

## 包含内容

- `scripts/`：狗端启动、巡检、SaaS、相机、4G、诊断和安全守护脚本
- `config/`：狗端工作区配置
- `deploy/`：部署文件
- `cpp_tools/`：狗端 C++ 辅助工具
- `src/`：ROS 2、FAST-LIO、Livox 和巡检相关源码
- `third_party/`：狗端实际使用的 Unitree 等第三方依赖
- 工作区根目录下当前使用的启动脚本和 YAML

`src/go2_fastlio_patrol/routes/` 中的 CSV 和质量/录制元数据也被保留，因为它们是狗端当前巡检程序的直接任务输入。

## 明确未包含

本目录不是整机磁盘镜像。以下内容不是可维护源码，已从快照中排除：

- `build/`、`install/`、`log/` 和 FAST-LIO `Log/`
- `maps/`、`bags/`、`patrol_logs/`
- `backups/`、`.staging/`、`.bak*`、`.before_*`
- `bin/` 编译产物
- `.git`、Python/pytest 缓存
- PCD、rosbag、posegraph、运行日志等大体积运行数据
- 工作区外的私有环境文件，例如 `~/.config/go2_saas.env`

因此，本目录回答的是“狗端当时实际有哪些可维护代码和配置”，不代表完整运行时状态，也不包含密钥或平台凭据。

## 与本地主工作副本的关系

抓取时，本目录与 `../orin_go2_fastlio_ws/` 的大部分核心源码一致。以下活动代码文件内容不同，必须以本目录为狗端当时的真实版本：

- `scripts/go2_network_recover.sh`
- `scripts/install_go2_4g_manager.sh`
- `scripts/z1pro_capture.sh`
- `src/go2_cmd_vel_bridge/src/go2_sdk2_motion_probe.cpp`
- `src/go2_fastlio_patrol/go2_fastlio_patrol/unitree_safe_cmd_node.py`
- `src/go2_fastlio_patrol/go2_fastlio_patrol/waypoint_follower_go2_2.py`

狗端还存在根级 `laser_mapping.yaml`、`livox_lidar_publisher.yaml` 以及一批路线/质量元数据，本地主工作副本当时没有对应文件或内容不同。

本地主工作副本另有尚未部署到狗端的水平坐标、PCD 抓取、course control 和测试代码。GitHub 提交不会自动部署到狗端；任何后续部署都应显式比较、测试并记录对应提交号。

## 完整性校验

`SHA256SUMS` 覆盖本目录内除清单自身外的全部文件。校验方法：

```bash
cd realtime_dog_end_code
shasum -a 256 -c SHA256SUMS
```

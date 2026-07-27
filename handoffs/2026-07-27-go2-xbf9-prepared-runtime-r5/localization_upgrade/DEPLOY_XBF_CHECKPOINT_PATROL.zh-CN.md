# xbf9 R5 算法与现场验收说明

## 这次和旧交付的根本差别

旧 R3 使用的是算法推测的 CSV→PCD 平移和 3 个示例 checkpoint，现场真值没有
闭合。R5 直接使用用户在平台确认并导出的 preparation：

```text
水平旋转：-0.27474701469097695 rad（约 -15.74°）
水平平移：(0, 0) m
路线：1277 点，未删减
固定物：28 approved / 1 candidate / 7 rejected
checkpoint：8 个
```

只有 28 个 `approved` 对象进入 `stable_layer.pcd`；candidate 和 rejected
仍留在审核记录中，但不会参与在线固定结构验证。

## 定位不是逐点寻找某个地标

每次起点或 checkpoint 校准分两层：

1. 实时 MID-360 点云与 `cleaned_static_map.pcd` 做 NDT/GICP，计算完整候选位姿。
2. 候选位姿下的实时点云再与 28 个审核对象形成的稳定层核对，确认这个结果确实
   得到墙体、固定杆体等静态结构支持。

稳定层验证默认要求：

- 审核地标附近 `1.50 m` 内形成支持邻域；
- 距稳定层点 `0.60 m` 内至少 40 个匹配点；
- 至少 16 个不同的 `0.25 m` 三维体素；
- 邻域匹配率至少 `10%`；
- 匹配结构最大三维跨度至少 `1.5 m`。

任一条件失败，本次修正不写入，运动命令保持为零。地标不会单独替代完整地图
计算位置，也不会在普通跟线阶段持续占用大量算力。

## 起点怎么工作

狗放在 CSV 起点附近、朝向大致合理的位置后：

1. coordinator 让运动链保持零；
2. localizer 收集 5 组静止点云；
3. 在地图候选和有限 SE(2) 初值内配准；
4. 多次结果一致且稳定层验证通过后，冻结 `map <- odom`；
5. coordinator 发布 `RUNNING`；
6. 原 follower 根据对齐后的 CSV 与 aligned odometry 开始走。

R5 不包含“从校园任意位置自主规划到起点”。如果狗远离路线或周围缺少足够静态
结构，正确行为是保持停止并报告定位失败。

## 中途怎么校准

| waypoint | 路线进度 | 平台关联固定物 | 固定物距路线 |
|---:|---:|---|---:|
| 26 | 11.99 m | AUTO-P07 | 1.75 m |
| 161 | 75.15 m | AUTO-W04 | 10.60 m |
| 274 | 125.54 m | AUTO-P59 | 6.80 m |
| 368 | 169.52 m | AUTO-P90 | 2.10 m |
| 577 | 271.52 m | AUTO-P117 | 2.31 m |
| 737 | 346.04 m | AUTO-P143 | 8.97 m |
| 907 | 428.64 m | AUTO-P156 | 1.44 m |
| 1040 | 491.98 m | AUTO-P163 | 4.97 m |

到点时 coordinator 先把 follower 输出门控为零，确认停车后再触发重定位。成功
则更新冻结变换并自动继续；失败则保持零速度。正常路段只做轻量二维坐标变换，
不按 5 秒或 10 秒周期持续运行重型配准。

## 昨天现场暴露问题的处理

| 现场问题 | R5 处理 |
|---|---|
| descriptor JSON 导致 OOM | JsonCpp 严格解析，输入上限 64 MiB |
| 默认 Fast DDS 内存异常 | 新节点固定 Cyclone DDS |
| Foxy 参数覆盖地图/路线 | 启动前生成绝对路径运行时 YAML |
| UDP 监听检测误报 | 扫描 `ss` 所有字段，不依赖列号 |
| SDK receiver 10 秒内未监听 | 等待上限 30 秒 |
| `setsid command &` 记录错进程 | Python 原地 `setsid()+exec()`，确认 PID=PGID=SID |
| follower 已启动却被 supervisor 判死 | 以真实进程组和唯一 run-id 监督 |
| 只看进程存在就认为成功 | 必须收到 `RouteStatus.RUNNING` |
| 一次 odometry 约 0.918 s 延迟 | 启动前记录 p50/p95/p99，不盲目放宽阈值 |

更完整的证据边界见
`YESTERDAY_ROOT_CAUSE_AND_R5_STATUS.zh-CN.md`。昨天没有观察到
`RUNNING`，所以不能把最后一次提前清理误写成“点云算法已经失败”。

## 临时 GoGuard 固定入口

当前平台协议仍只给松散的 CSV URL，没有准备任务 ID。本版按用户要求采用临时
固定策略：

```text
任何 start_patrol 别名
  -> 不下载、不解析平台 CSV URL
  -> 固定启动 xbf9-horizontal-clean-r1

任何 stop_patrol 别名
  -> 固定停止该任务并请求最终 StopMove
```

桥接层复用原 `go2_saas_agent.py` 的命令轮询、commandId 去重和结果回传，只
替换 start/stop handler。它先向平台返回 `running`，不阻塞后续
`stop_patrol`；真正的非零运动仍必须等待 coordinator 发布
`RUNNING + localization_ready=true`。

## CPU 与实时性

高算力配准只在起点和 8 个显式 checkpoint 停车时运行。原 CSV follower、
safe cmd 和 UDP/SDK2 链仍按原节奏运行。这样把“定位修正”和“已经表现稳定的
CSV 跟线”解耦，避免巡检过程持续抢占算力。

## 当前能证明和不能证明的事

已经离线证明：

- 正式 ZIP 无损且只含 7 个约定文件；
- ZIP 与原始 PCD、原始 CSV 的 SHA-256 一致；
- 对齐 CSV、SE(2)、28 个固定物、8 个 checkpoint 互相绑定；
- 地图 146 个 tile、146 个描述符和 34,313 点稳定层哈希完整；
- 修正后的启动、停止、进程组和数据导入代码通过离线检查；
- GoGuard 任一 start 别名都会进入固定 handler，平台 CSV/URL 不会进入旧链；
- 原 follower、safe cmd、UDP sender/receiver、motion probe 源码哈希未改变。

尚未证明：

- 真狗实时点云能在起点进入 `RUNNING`；
- 8 个 checkpoint 在当天环境中都能看到足够稳定结构；
- MID-360/IMU/base 外参与离线地图完全一致；
- 600 m 全程和当前算力温度长期稳定。

因此第一次现场验收顺序必须是：静止定位 → 5～10 m 低速 → waypoint 26
停车校准 → 再逐步扩大距离。

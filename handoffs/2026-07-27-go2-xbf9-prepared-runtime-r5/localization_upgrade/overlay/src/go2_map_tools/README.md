# go2_map_tools

`go2_map_tools` 将一份只含有限数值的 XYZ PCD 地图转换成可版本化、可验证的
地图资产，供机器人端定位器和路线设计平台共同使用。它没有 ROS 运行时依赖，
支持 Python 3.8 及更高版本。

## 支持的 PCD 输入

- PCD v0.7 的 `DATA ascii` 或未压缩 `DATA binary`。
- 字段顺序不受限制，但编译地图时必须存在名为 `x`、`y`、`z` 的标量字段。
- 支持的 `SIZE`/`TYPE` 组合：4/8 字节浮点 `F`，以及 1/2/4/8 字节有符号或
  无符号整数。支持解析 `COUNT > 1`。
- `WIDTH * HEIGHT` 必须等于 `POINTS`；数据载荷大小或行数必须相符。
- 默认拒绝 NaN 和无穷大。
- 明确拒绝 `binary_compressed`；使用前请通过 PCL 转换。

## 命令

直接安装到 Python 环境时使用下列 `go2-map` 命令；通过 ROS 2
`colcon` overlay 安装时，把每条命令写成
`ros2 run go2_map_tools go2-map <子命令> ...`。

```bash
# 面向生产的一步式操作：PCD -> 图块 + 哈希 + 描述子索引。
go2-map compile campus.pcd campus_map \
  --map-id campus-2026-07-25 \
  --tile-size 20 --voxel-size 0.20 \
  --rings 20 --sectors 60 --max-radius 80

# 分阶段操作与诊断。
go2-map pcd-info campus.pcd
go2-map downsample campus.pcd campus_020.pcd --voxel-size 0.20
go2-map tile campus.pcd campus_map --tile-size 20 --voxel-size 0.20
go2-map build-index campus_map/manifest.json
go2-map verify campus_map/manifest.json
# 对于尚无描述子索引的中间 `tile` 输出：
go2-map verify campus_map/manifest.json --tiles-only
go2-map query-index campus_map/descriptor_index.json current_scan.pcd

# 生成供网页审核平台使用的确定性点云质检包。
go2-map review-bundle registered_map.pcd campus_review \
  --keyframes keyframes.jsonl \
  --session session.json \
  --max-preview-points 500000

# 应用网页审核导出的显式排除掩膜，生成一份新的 PCD 和过滤报告。
go2-map filter-annotations \
  registered_map.pcd \
  registered-map.0123456789ab.annotations.json \
  campus_filtered
```

默认的 `verify` 命令会检查两个校验和文件、每个图块的哈希及点数/边界、描述子
与图块的源哈希、地图 ID，以及描述子覆盖范围/中心。编译结果如下：

```text
campus_map/
  manifest.json
  manifest.sha256
  descriptor_index.json
  descriptor_index.json.sha256
  tiles/
    x+000000_y-000001.pcd
    ...
```

`compile` 在暂存目录中构建并验证完整地图包，然后通过一次原子目录重命名完成
发布。已发布的 v2 目录不可变，即使提供 `--overwrite` 也不会被覆盖；重新构建
必须使用新的版本/哈希目录。`--overwrite` 仅适用于能够被识别、且不能授权运动
的中间 v1 图块构建。底层 `tile` 命令生成该 v1 清单，`build-index` 再将其升级
为完整 v2 地图包。

## PCD 质检包

`review-bundle` 严格复用本包的 PCD 解析器，在 Mac 或 Orin 上离线生成网页
工作台可以安全分块上传的质检资产。它不修改或复制权威源 PCD，也不会把网页
审核结论变成机器人运行授权。

输出目录必须尚不存在。命令会先在同一父目录的暂存目录中完成全部解析、统计、
哈希和文件同步，成功后再通过一次目录重命名原子发布：

```text
campus_review/
  preview.xyz.bin
  preview.xyz.bin.sha256
  review.json
  review.sha256
```

`preview.xyz.bin` 是确定性系统抽样得到的绝对地图坐标，每点固定为 12 字节：
三个 little-endian IEEE-754 float32，顺序为 `x y z`。默认且最大只包含
500,000 点；完整点数、抽样方式、字节数和 SHA-256 都绑定在 `review.json` 中。

`review.json` 的 schema 是 `go2.map_review_bundle/v1`，其中包含：

- 源 PCD 的文件名、SHA-256、原始字节数、编码、字段、点数、三维边界和高度跨度；
- 1 m XY 网格中每格的点数、覆盖比例、密度分位数，以及四邻接连通分量；
- 预览文件的编码、点数、大小和 SHA-256；
- 可选录制证据的文件 Hash 和严格校验结果；
- 机器可读的质量警告。

传入 `--keyframes` 后，JSONL 中的 `index`、点偏移和点数必须连续且总数必须
精确等于源 PCD 点数；云与里程计时间戳、同步误差、位姿、输入/无效点数和
截断标志也会被逐条校验。纳秒时间戳始终作为 JSON 整数保存，不会先转换成
浮点秒。报告包含轨迹距离、关键帧间隔、同步误差 p95/最大值、截断帧、闭环
间隙和逐帧轨迹，网页可以据此定位从哪一帧开始出现错层或发散。

传入 `--session` 后，只接受完成状态的
`go2.registered_pcd_session/v1`，并核对 PCD 点数、关键帧数以及 session
中绑定的 PCD/关键帧 Hash。若 session 明确声明地图只是未经回环优化的
FAST-LIO 注册点云，报告会保留为警告。只有 PCD 而没有关键帧或 session 时
仍可检查外观、密度和连通性，但不能追溯时间同步、外参或具体发散帧。

## 应用地图审核标注

`filter-annotations` 消费网页审核平台导出的
`go2.map_review_annotations/v1` 或 `go2.map_review_annotations/v2` JSON。
它是离线、失败关闭的地图资产操作，不会向机器人发送运动命令，也不会覆盖
源 PCD。v1 圆形标注仍可读；v2 是推荐格式，可描述带高度范围的圆柱、旋转框
和多边形柱体，从而把整段墙面、建筑转角或停车区域作为对象级 ROI 审核。

命令会在读点云前校验标注文件，并在发布前再次校验输入没有变化：

- `map.sha256` 必须与源 PCD 完整文件的 SHA-256 精确一致；
- 坐标系必须是绝对 `map` 坐标、单位必须是 `metre`；v1 几何必须是
  `xy_circle_all_z`，v2 必须是 `object_roi_v2`；
- schema、类别语义、安全字段、标注 ID、半径和字段集合必须完整有效；
- 拒绝重复 JSON 键、重复标注 ID、NaN、无穷大、未知类别和未知字段；
- 输出目录必须尚不存在，不提供覆盖选项；若掩膜会删除全部点，整次操作失败且
  不发布输出。

发布规则有意保持简单且明确：

- 只有 `dynamic_exclude`、`vegetation_exclude` 和 `parking_exclude`
  会从 cleaned static map 删除点；v2 还要求点的 Z 落入对象高度范围；
- `stable_include` 会把 cleaned static map 中落入该 ROI 的点提取到独立
  stable layer。它不会变成反向过滤器，未标注区仍完整保留在 cleaned map；
- exclusion 优先于 stable：重叠点不会进入 stable layer；
- `low_confidence`、`ghosting`、`drift_suspect` 等类别只是审核发现，同样只
  进入报告；
- 单根旗杆或单面墙不能独自约束 `x`、`y`、`yaw` 三自由度。
  `stable_include` 必须与多个方向、多个位置的固定结构组合使用，不能被当成
  自动定位锚点。

成功后输出目录为不可变的新资产：

```text
campus_filtered/
  cleaned_static_map.pcd
  cleaned_static_map.pcd.sha256
  stable_layer.pcd
  stable_layer.pcd.sha256
  filter_report.json
  filter_report.json.sha256
```

报告 schema 为 `go2.map_annotation_filter_report/v2`，同时绑定原始 PCD
SHA-256、标注 schema/revision/SHA-256、cleaned map SHA-256 和 stable
layer SHA-256。

### v2 对象 ROI 示例

```json
{
  "schema": "go2.map_review_annotations/v2",
  "revision": "R2",
  "annotations": [
    {
      "id": "wall-east-01",
      "category": "stable_include",
      "roi": {
        "shape": "oriented_box",
        "center_xy_m": {"x": 180.0, "y": -156.0},
        "size_xy_m": {"x": 18.0, "y": 1.2},
        "yaw_rad": 0.12,
        "z_range_m": {"min": 0.3, "max": 8.0}
      }
    }
  ]
}
```

`coordinate_system.region_geometry` 需声明：

```json
{
  "type": "object_roi_v2",
  "supported_shapes": ["cylinder", "oriented_box", "polygon_prism"],
  "z_range_fields": ["roi.z_range_m.min", "roi.z_range_m.max"]
}
```

## 一条命令发布审核地图

正式发布不要手工拼接过滤和编译结果，使用：

```bash
go2-map publish-reviewed \
  registered_map.pcd \
  registered-map.annotations.json \
  /data/maps/campus-r2 \
  --map-id campus-r2 \
  --minimum-stable-points 1000

go2-map verify-reviewed /data/maps/campus-r2
```

该命令原子地执行：

```text
源 PCD + 标注
  -> cleaned static map + stable layer
  -> cleaned map 编译 tracking tiles
  -> stable layer 构建全局地点描述子
  -> reviewed_map_publication.json 绑定全部哈希
  -> 全量复验后发布目录
```

输出中的 `reviewed_map_publication.json` 仍保持
`deployment_ready=false`；哈希验证通过不等于真狗已经获准运动。

两个 PCD 都保留源 PCD 的字段、点顺序和 ASCII/Binary 编码。cleaned map
仅移除命中 exclusion 的行；stable layer 是 cleaned map 的子集。报告记录
各 ROI 命中点数、总删除/保留点数、stable 点数和其他审核发现。多个重叠 ROI
的逐 ROI 命中数可能重复计数，但总删除点数和 stable layer 不会重复写点。

应用掩膜前仍应在审核平台预览。过滤后必须使用新 PCD 的 SHA-256 重新生成
质检包、地图图块、描述子和路线绑定；过滤报告本身不是定位质量证明或实机放行。

## 地图清单 v2

生产环境的顶层 schema 字符串是 `go2.map_tiles/v2`。它同时绑定每个图块和描述子
索引的精确字节内容。路线会保存该清单的 SHA-256，因此替换索引及其附属文件
不能悄悄改变地点识别行为。

```json
{
  "schema": "go2.map_tiles/v2",
  "map_id": "campus-2026-07-25",
  "created_utc": "2026-07-25T08:00:00Z",
  "frame_id": "map",
  "tile_size_m": 20.0,
  "voxel_size_m": 0.2,
  "source": {
    "filename": "campus.pcd",
    "sha256": "<64 lowercase hex>",
    "point_count": 123456
  },
  "bounds": {
    "min": [-40.0, -20.0, -2.0],
    "max": [80.0, 60.0, 12.0]
  },
  "descriptor_index": {
    "path": "descriptor_index.json",
    "sha256": "<64 lowercase hex>"
  },
  "tiles": [
    {
      "id": "x+000000_y-000001",
      "ix": 0,
      "iy": -1,
      "path": "tiles/x+000000_y-000001.pcd",
      "sha256": "<64 lowercase hex>",
      "point_count": 1234,
      "bounds": {"min": [0.1, -19.9, -1.0], "max": [19.8, -0.1, 8.0]},
      "grid_bounds": {"min": [0.0, -20.0], "max": [20.0, 0.0]}
    }
  ]
}
```

图块文件是确定性生成的 PCD v0.7 ASCII，且只包含 `x y z` 浮点字段。
`manifest.sha256` 用来验证清单的精确字节；每个图块也有自己的 SHA-256，以及
声明的点数/边界。生产验证要求使用 v2。中间 v1 schema 不绑定描述子，只能被
仅图块诊断模式接受。

## 极坐标描述子索引 v1

schema 字符串是 `go2.polar_descriptor_index/v1`。顶层包含 `map_id`、
`created_utc`、`parameters` 和 `entries`。参数包括：
`rings`, `sectors`, `max_radius_m`, `min_z_m`, `max_z_m`，以及
`value="max_height_normalized"`.

每个条目包含：

- `id`、`tile_id`、`center: [x,y,z]`，以及中心图块的 `source_sha256`；
- `values`：长度为 `rings * sectors`、只含有限数值、环优先/扇区次优先的数组；
- 长度为 `rings` 的 `ring_key`，以及长度为 `sectors` 的 `sector_key`。

该实现是独立开发的径向/角向直方图。循环平移扇区可以给出航向角假设；它只是
粗粒度地点候选，必须经过几何配准确认后才能允许运动。

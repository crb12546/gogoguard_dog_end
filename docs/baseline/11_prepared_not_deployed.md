# 11 · 已准备好、但未部署的下一代方案

> ⚠️ **基准:2026-07-27 整机快照(删除阶段之前)。**
> 本篇描述的是**删代码之前**的狗端状态,内容在该基准下成立。
> 读时注意两点:
> 1. **有些组件现在已从仓库删除**(4G 治理整套、legacy 命令桥、22 个脚本等)——
>    见 `13_file_accounting.md` 与 `14_dog_verification.md`。文档没写错,是那个时间点确实有。
> 2. **`go2_saas_agent.py` 的行号已漂移**(3630 → 3633 行,删除阶段改过 4 处)。
>    换算:引用 `1–52` 不用调 · `53–480` **+3** · **`481–3184` +5**(绝大多数引用在此段) · `3186` 之后 +3。
>    其他文件行号不受影响。
> 已用已知答案自检:本文若写 `commands` 列表在 `:2590`,当前仓库实测在 `:2595`,差 5 ✓

> **这份文档补的是一个此前完全遗漏的东西。**
> 快照根目录下的 `current_task/` 装着一整套**已经做好、通过评审、但狗上没有**的定位方案。
> 它不属于"狗上现在跑什么"(那是 02–09 的范围),但它**直接决定第 3 步的起点** ——
> 因为改造方案不应该从零设计,而应该从这套已有的东西接续。

**遗漏原因**:我此前只读了快照的 `mirror/` 与 `system_config/`,
未读根目录下的 `current_task/` `development/` `inventory/` `archives/` 四个目录。

---

## 一、它是什么

一套**巡检准备包**,由平台侧产出,schema `go2.patrol_preparation/v1`。

| 组件 | 文件 | 状态 |
|---|---|---|
| 基准 PCD | `maps/console/xbf.pcd`(1,085,459 点) | ✅ **狗上已有,同一份** |
| 源 CSV | `routes/xbf9.csv.horizontal.csv`(1278 行) | ✅ **狗上已有,同一份** |
| **对齐后 CSV** | `xbf9_horizontal_clean.aligned.csv` | ❌ **狗上没有** |
| 地图对象标注 | `xbf-2-2.3526e4f11658.annotations.json` | ❌ 狗上没有 |
| 停车重定位点 | `xbf9_horizontal_clean.checkpoints.json` | ❌ 狗上没有 |
| 总绑定 | `preparation.json` | ❌ 狗上没有 |

**全部靠 SHA-256 相互绑定**,`当前任务说明.md` 原文:
> `preparation.json` 是总绑定文件,所有组件都通过 SHA-256 关联,**不能只按名字配对**。

> ZIP 本身没有重复放入 27 MB 的 PCD,而是要求部署时**找到哈希完全一致的 PCD**。

哈希:
```
pcd_sha256 = 3526e4f116586d3594c0afa45efb3fb254e4eca1bf89fa21f18896a558ee5aa2
csv_sha256 = b4abadd38c30f5904f4cfe10eb529b8c1a4940ba023019847ea3959c48fd53a2   ← 源 CSV
aligned    = 973c906c89a753f1eee6ab21052f92f9195015df3252a443498e14a6f4564f55   ← 对齐后
```

---

## 二、对齐变换 —— 与运行时实测互为交叉验证 ★

`alignment.json`:
```json
"method": "operator-planar-drag-rotate/v1",
"status": "reviewed",
"transform": { "type": "SE2", "theta_rad": -0.27474701469097695, "translation_m": [0, 0] }
```
`theta_rad` → **-15.741844°**;仅 SE(2),**Z / roll / pitch / scale 均未改动**。
`evidence.note_zh`:*操作员在真实 PCD 全点局部视图中完成整体 XY 平移与 yaw 旋转。*

**交叉验证**:
| 来源 | 角度 | 方式 |
|---|---:|---|
| 准备包 `alignment.json` | **-15.741844°** | **操作员在 PCD 视图里手工拖拽** |
| 7/26 运行时 trace `route_rotation_deg` | **-15.669984°** | **follower 自动计算**(`align_route_to_pose`) |
| 差值 | **0.0719°** | 两条路径完全独立 |

→ **人工对齐与自动对齐相差 0.072°** —— 这是对"当前自动锚定精度"的一个独立佐证。

`trim`:`source_start_index=0` · `source_end_index=1276` · **1277 个 waypoint 全部保留,未裁剪首尾**。

---

## 三、地标层:28 批准 / 1 候选 / 7 拒绝

`annotations.json` 共 **36 个对象**,字段含 `category`(`stable_include`)· `object_type`(`wall`/`pole`)·
`source`(`seed_fit`)· `review_status` · `geometry` · `coordinate_system` · `category_semantics` · `safety`。

| 状态 | 数量 | ID |
|---|---:|---|
| **批准** | **28** | 杆状物 12:`AUTO-P07 P09 P58 P59 P90 P117 P143 P152 P156 P163 P170 P172`<br>墙体 16:`AUTO-W01~W10 W13 W14 W16 W17 W19 W20` |
| 候选 | 1 | `AUTO-P74` |
| 拒绝 | 7 | `AUTO-C01` `AUTO-P20` `AUTO-P81` `AUTO-P157` `AUTO-P162` `AUTO-P203` `AUTO-W11` |

**它们是对象级 ROI(墙体、杆状物),不是孤立的点。**

---

## 四、8 个停车重定位检查点

`checkpoints.json`,全部结构一致:
```json
{"waypoint_index": N, "checkpoint": {"mode": "relocalize", "stop_timeout_s": 60, "required": true}}
```

| # | waypoint_index |
|---:|---:|
| 1–8 | **26 · 161 · 274 · 368 · 577 · 737 · 907 · 1040** |

**全部 `required: true`** —— 重定位失败**不允许继续**;`stop_timeout_s: 60` 为单点停车上限。

---

## 五、运行逻辑(设计意图,原文摘录)

> 运行时**不应**要求算法先识别"这是 AUTO-P90 这根灯杆",再用单个物体算位置。正确逻辑是:
>
> 1. 离线用批准的固定结构 ROI 从 PCD 编译出**稳定地图层**,同时排除动态物体和不可靠区域。
> 2. 实时把 MID360 当前扫描与稳定地图层做**点云配准**,得到狗在 PCD map 坐标系中的 `x, y, yaw` 和**置信度**。
> 3. 已对齐 CSV 本来就在同一个 PCD map 坐标系中,因此定位成功后即可算出狗到路线起点或下一 waypoint 的距离与方向。
> 4. 起点先**静止定位**;若置信度不足,可做受控的**小范围初始化运动**后重算,但**不能盲目直接进入 600 m 路线**。
> 5. 运行到 8 个 checkpoint 时,**现有 follower 停车并保持零速度**;定位器重新计算 map 位姿。
> 6. 成功后更新"地图坐标→follower 局部坐标"的变换,再让原 follower 继续;
>    **失败则保持停车并报告,不能拿低置信结果硬改路线。**
>
> 固定结构的作用是**提高整片点云匹配的稳定性**,而不是让某一根杆或某一面墙独自决定 `x, y, yaw`。

### 接入约定(原文)

> 收到 GoGoGuard 的 `start patrol` 后,可以**先忽略命令中下发的 CSV URL**,固定选择这套准备包。但程序仍必须:
> 1. 只响应明确的 `start patrol`,**不能开机自动走**。
> 2. 校验 PCD、源 CSV、对齐 CSV、标注和 checkpoint **哈希**。
> 3. 启动雷达/FAST-LIO 和定位器,**先完成起点定位**。
> 4. 定位成功后才启动现有 Waypoint Follower,并传入**对齐后的 CSV**。
> 5. checkpoint 停车期间**保持运动命令为零**。
> 6. `stop patrol`、遥控器/急停和异常退出**必须仍能终止运动**。
>
> "临时忽略 URL"只解决当前任务文件选择,**不等于可以取消哈希校验、停止命令和异常清理**。

---

## 六、与当前系统的关系 ★

| 维度 | 当前(manual_anchor) | 准备包方案 |
|---|---|---|
| 参考系 | FAST-LIO 会话局部系,**原点=进程启动位置** | **PCD map 绝对坐标系** |
| 起点确定 | 人把狗摆到起点,取当前位姿作锚 | 点云配准算出绝对位姿 + 置信度 |
| 中途校正 | **无** | 8 个 checkpoint 停车重定位 |
| 路线来源 | 现场生成 `route_horizontal.csv` | 预先对齐好的 `aligned.csv` |
| 漂移应对 | 无(577 m 纯里程计) | checkpoint 周期性拉回 |

**这直接指向 `10_open_questions.md` 第 1 条那个 176 倍矛盾的根源**:
当前系统只有"相对起点"的参考,**没有任何绝对真值**,所以车载日志无法自证偏没偏。
准备包方案给出的正是绝对参考。

**同时,它也重新定位了 `route_relocalizer.cpp`**(1004 行,132 次巡检从未运行):
它做的正是 PCD/ICP 配准 —— **不是"没用的代码",而是这套方案所需的基础能力**。
(注意其现状:saas 传的 `anchor_route_start=true` 会把 ICP 平移强制拉回起点,
 见 `07_design_intent.md` 第 15 条 —— 若要用于 checkpoint 重定位,这个行为需要重新审视。)

---

## 七、当前部署状态(原文,明确)

> 本快照采集时,**这套准备包尚未部署**:
> - 狗上的 PCD 与本包引用的 PCD 相同 ✅
> - 狗上的 `xbf9.csv.horizontal.csv` 与本包的源 CSV 相同 ✅
> - **狗上没有本包对齐后的 CSV** ❌
> - **狗上没有 `/home/unitree/localization_upgrade`** ❌
> - 因此现在直接从 GoGoGuard 下发 `start patrol`,**不会自动得到这里描述的起点校准和 8 次停车重定位**。

**与 `SNAPSHOT.json` 一致**:`localization_upgrade_path: "absent"`。

### 最后一句是给第 3 步的直接约束(原文)

> 下一次交付必须把"真实狗端代码基线"和"本准备包"**整合成一个经过本地回放测试的不可变 release**,
> **不能只把 ZIP 扔到狗上。**

---

## 八、快照根目录其余三个目录

| 目录 | 文件数 | 内容 |
|---|---:|---|
| `development/` | 1 | `本地开发规则.md` |
| `inventory/` | **26** | 硬件/系统/ROS/软件包/进程/服务/端口/地图/路线/视频/rosbag 清单,含 `services.txt` `final-live-state.txt` `route-files.tsv` `rosbag-files.tsv` `remote-local-critical-verification.txt` 等 |
| `archives/` | 0 | 空 |

⚠️ `inventory/` 的 26 份清单**尚未逐份读过**,其中多份可与 02–09 的结论交叉验证
(例如 `services.txt` 对服务清单、`route-files.tsv` 对 80 个 CSV、`rosbag-files.tsv` 对 bag 覆盖面)。
已记入 `10_open_questions.md`。

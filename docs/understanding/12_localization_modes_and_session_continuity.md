# 12 · 定位模式 & FAST-LIO 会话连续性(之前漏讲的一整层)

> 原则同 00。核心文件(均已逐行读):
> `scripts/manual_route_anchor.py`、`scripts/localization_session_guard.py`、
> `scripts/go2_base_health_watchdog.py`、`scripts/check_route_start_alignment.py`、
> `scripts/ensure_base_ready.sh`、`go2_saas_agent.py: localization_mode_from_params/route_relocalization_plan`。

## 一、⚠️ 重要修正:定位有两种模式,`manual_anchor` 是默认(不是 pcd)
`go2_saas_agent.localization_mode_from_params`(`:602`)决定巡检起始坐标策略:
- **`manual_anchor`(默认)**:把狗**当前静止位姿**当作路线首点,把整条路线**刚性搬**到当前 FAST-LIO 会话坐标系。**不需要地图 pcd。** 别名 `manual/origin/direct/none` 都归它。
- **`pcd`**:用 `route_relocalizer`(见 07)+ 同名地图 ICP 对齐。别名 `relocalize/map`。只有显式请求或 `requireRelocalize=true` 才走。
- 环境变量 `GO2_PATROL_LOCALIZATION_MODE` 可改默认;`requireRelocalize=true` 但模式非 pcd 会报错。
> 我在 07/08 里把 pcd 讲成了主路径,**错了**:现场默认是 manual_anchor(把狗摆到起点、当前位姿=路线起点)。

## 二、manual_anchor 怎么做(`manual_route_anchor.py`)
1. **等静止**:读 `/dev/shm/go2_fastlio_latest_odom.txt` 快照,要求 1s 内 ≥8 次更新、位移 <0.08m、yaw <0.08rad(狗必须站稳)。
2. **刚性变换**:源 CSV 首点 = 锚点,`delta_yaw = 当前yaw - 源首点yaw`,每个点绕首点旋转 delta_yaw 再平移到当前位姿 → 输出 runtime CSV(源 CSV 不改)。
3. **记录 FAST-LIO 会话身份**:`current_fastlio_session()` 从 `/proc` 取 `fastlio_mapping` 的 **boot_id + pid + start_ticks**,写进 metadata。
- `--capture-only`:只抓会话身份(不变换路线),给 pcd 模式和录制/停止收尾用。

## 三、会话连续性:整套系统的"定位正确性"支柱
路线 CSV 只在**某一次 FAST-LIO 进程会话**的坐标系里有意义。FAST-LIO 一旦重启,坐标系原点变了,路线就全错。因此有多重守卫:

- **`localization_session_guard.py`**(巡检/录制期间常驻):每 0.2s 比对当前 FAST-LIO 会话身份与启动时记录的是否相同;若 **pid/boot_id/start_ticks 变了** 或 **odom 快照静默 >2s** 或 **odom 时间戳倒退** → 立即**中止**任务(patrol: 杀 follower/safe/桥/sdk + 发 StopMove;recorder: 停录 + 把半成品路线改名 `.invalid`),写事件日志。返回码 10。
- **`go2_base_health_watchdog.py`**(巡检期间):订阅 `/Odometry`,校验位姿有限、不越界(|xy|≤100m)、不跳变(动态限:`max_vx·dt·2.5+0.3`)、不静默(>6s);连续 2 次坏 → **停巡检**(但**不重启 base**,`auto_restart=false`,为的是保住坐标连续性)。
- **`check_route_start_alignment.py`**(巡检启动 gate):读快照,要求当前位姿距路线首点 <max_distance、yaw 误差 <max_yaw(生产 0.35m/0.35rad),否则拒绝开跑。
- **`ensure_base_ready.sh`**:base 就绪门 + `--patrol-start-gate`(CPU<85% 且 FAST-LIO 连续新鲜 3 次)+ `--fresh-only`;base 不在就拉起 `base_bringup.sh`。

## 四、`/dev/shm/go2_fastlio_latest_odom.txt` 快照(01 的待坐实,这里定位)
- 格式:一行 `stamp=… x=… y=… z=… qx=… qy=… qz=… qw=…`(`manual_route_anchor.read_odom_snapshot` 解析)。
- 由 FAST-LIO(被改过的 `laserMapping.cpp`,狗上副本 58.9KB,待读)高频写入,供上述所有守卫**低开销**读位姿 + 判新鲜度(不必订阅 ROS)。
- `check_fastlio_freshness.py`(01)也读它。

## 五、一句话
巡检的"定位对不对",不只靠 FAST-LIO 出位姿,而是靠一整套**会话连续性守卫**:起始把路线锚到当前会话(manual_anchor)或对齐到地图(pcd)→ 全程盯着"还是不是同一个 FAST-LIO 会话/位姿正不正常"→ 一旦坐标系可能变了就立即停,宁可中止也不让狗按错坐标乱跑。

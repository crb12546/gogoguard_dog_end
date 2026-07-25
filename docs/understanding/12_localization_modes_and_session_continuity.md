# 12 · 定位模式 & FAST-LIO 会话连续性(之前漏讲的一整层)

> 原则同 00。核心文件(仓库版均已逐行读):
> `scripts/manual_route_anchor.py`、`scripts/localization_session_guard.py`、
> `scripts/go2_base_health_watchdog.py`、`scripts/check_route_start_alignment.py`、
> `scripts/ensure_base_ready.sh`、`go2_saas_agent.py: localization_mode_from_params/route_relocalization_plan`。
>
> **源标签约定**:【默认 code file:line】=脚本 argparse/常量默认值;【生产 saas file:line】=`go2_saas_agent.py` 实际装配的 `-p`/CLI/env;【狗上 dog:证据】=狗上文件或 dog manifest 佐证;【推断-未验】=无源直证。默认≠生产时两个都写,并点明"狗上实际生效哪个"。

## 核验状态(本轮 2026-07-25 逐条对磁盘源码核过)

- **已核范围(仓库源码逐条 CONFIRMED)**:上列 6 个守卫脚本 + `go2_saas_agent.py` + `route_recording_blackbox.py` + `unitree_cmd_node.py` 的相关行,全部对着真实源码核过,行号见文末台账。28 条断言里 25 条 CONFIRMED,3 条本轮更正(见下)。
- **runtime 真相(狗上实际生效)**:靠两份 dog manifest 交叉印证 —— `runs/xunjian-20260725-06`、`runs/xunjian-20260725-07`。它们记录:`localization_mode=manual_anchor`【dog manifest.txt:9】、`start_alignment=runtime_anchor_point0_0.35m_0.35rad`【dog manifest.txt:17】、`localization_session_guard=required`【:12】、`localization_restart_policy=abort_current_patrol`【:13】、`base_watchdog_auto_restart=false`、`startup_cpu_max_pct=85.0`/`startup_stable_samples=3`。这些和仓库 saas 的装配一致。
- **⚠️ 可验证性边界 —— 别默认"仓库==狗上"**:除 `laserMapping.cpp` 外,**所有 Python 守卫脚本与 `go2_saas_agent.py` 均无狗上源码副本**(【无狗上对照】),狗端是否跑同一版本 **UNVERIFIABLE**。唯一有狗版对照的是 `laserMapping.cpp`:仓库版 `sha256=5fec8282…`(61919B≈60.5KB)**≠** 狗版 remote_source `sha256=e4cd05cb…`(60358B=58.9KB),diff 216 行(健康门/输入时序逻辑不同)—— **repo≠dog(sha 验)**。好在 §4 只引用其"快照写入块",该块已在狗版 line 738-754 直接核实存在,故 §4 结论据狗版成立。
- **本轮更正(旧版错处)**:①§三 跳变公式 `max_vx·dt·2.5+0.3` → 真实 `0.80+0.60·dt`(与 max_vx 无关);②§三 "odom 静默>6s 也停巡检" → 静默被特判**不停巡检**(保零速等恢复);③§三 对齐门只写生产 0.35/0.35,未注明脚本默认 0.8/0.9(default≠prod);另 §一 pcd 触发条件措辞收紧(`requireRelocalize=true` 是断言不是选择器)。

## 一、⚠️ 重要修正:定位有两种模式,`manual_anchor` 是默认(不是 pcd)

`go2_saas_agent.localization_mode_from_params`(:602)决定巡检起始坐标策略【生产 saas go2_saas_agent.py:602;被 `route_relocalization_plan`(:752)、`start_patrol_command`(:1645)消费】。〔go2_saas_agent.py【无狗上对照】,但 runtime 值由 dog manifest 佐证〕

- **`manual_anchor`(默认)**:把狗**当前静止位姿**当作路线首点,把整条路线**刚性搬**到当前 FAST-LIO 会话坐标系。**不需要地图 pcd**(manual 分支直接 return,不解析 pcd)。别名 `manual/origin/direct/none`(还含 `manual_anchor/start_anchor/origin_direct`)都归它。【生产 saas go2_saas_agent.py:620-627 别名表、662-668 env 默认、765-770 manual 分支 reason="current stationary pose is waypoint 0"+line 770 return;刚性变换 code manual_route_anchor.py:260-299】【狗上 dog:两份 manifest 均 `localization_mode=manual_anchor`,manifest.txt:9】
- **`pcd`**:用 `route_relocalizer`(见 07)+ **同名地图** ICP 对齐。别名 `relocalize/map/relocalization`【生产 saas go2_saas_agent.py:629-631 别名、723-748 `resolve_route_map` 要求同名 `.pcd`、1989-1998 `ros2 run go2_map_manager route_relocalizer -p map_file:=`】。**只有显式请求才走 pcd**:显式 `localizationMode=pcd` / legacy 参数 / `GO2_PATROL_LOCALIZATION_MODE=pcd`【生产 saas go2_saas_agent.py:639-685】。
- **措辞修正**:`requireRelocalize=true` **本身不选 pcd** —— 它只是个**断言**(模式非 pcd 就 raise),不是选择器【生产 saas go2_saas_agent.py:700-701】。旧版把它写成"才走 pcd 的触发条件"是误导。
- 环境变量 `GO2_PATROL_LOCALIZATION_MODE` 可改默认,**但仅在无 per-request `localizationMode`/legacy 参数时兜底**【默认 code / 生产 saas go2_saas_agent.py:662-664】。狗上该 env 未设 → 解析为 `manual_anchor`【狗上 dog manifest.txt:9】。

> 我在 07/08 里把 pcd 讲成了主路径,**错了**:现场默认是 manual_anchor(把狗摆到起点、当前位姿=路线起点)。dog manifest 双证此结论。

## 二、manual_anchor 怎么做(`manual_route_anchor.py`)〔【无狗上对照】,dog manifest 侧证流程被用:`manual_anchor_metadata=manual_anchor.json`〕

1. **等静止**:读 `/dev/shm/go2_fastlio_latest_odom.txt` 快照,要求 1s 内 **≥8 次更新、位移 <0.08m、yaw <0.08rad**(狗必须站稳)。此处 **default==prod**:脚本默认 `stable_seconds=1.0/min_updates=8/max_translation=0.08/max_yaw=0.08`【默认 code manual_route_anchor.py:23,178-205,365-369】,生产 `route_prepare_cmd` 显式 `-p` 传相同值【生产 saas go2_saas_agent.py:1919-1924】→ 狗上生效即这组值。
2. **刚性变换**:源 CSV 首点 = 锚点,`delta_yaw = normalize(当前yaw - 源首点yaw)`,每个点绕首点旋转 delta_yaw 再平移到当前位姿 → 输出 **runtime CSV**(源 CSV 不改)【默认 code manual_route_anchor.py:263-284 变换、422-430 读源写 output_route】。
3. **记录 FAST-LIO 会话身份**:`current_fastlio_session()` 从 `/proc` 按 comm/exe 匹配 `fastlio_mapping`,取 **boot_id + pid + start_ticks**(外加 executable 字段),写进 metadata【默认 code manual_route_anchor.py:65,79,86-91,402】。
- `--capture-only`:只抓会话身份(不变换路线),给 **pcd 装配、录制黑盒、stop 收尾**三处消费【生产 saas go2_saas_agent.py:2000(pcd)、2494(stop→`localization_session_end.json`);录制 code route_recording_blackbox.py:279;定义 manual_route_anchor.py:362,421】。

## 三、会话连续性:整套系统的"定位正确性"支柱

路线 CSV 只在**某一次 FAST-LIO 进程会话**的坐标系里有意义。FAST-LIO 一旦重启,坐标系原点变了,路线就全错。因此有多重守卫:

- **`localization_session_guard.py`**(巡检/录制期间常驻)〔【无狗上对照】;dog manifest 侧证 `localization_session_guard=required`、`restart_policy=abort_current_patrol`〕:每 **0.2s** 比对当前 FAST-LIO 会话身份与启动时记录的是否相同(**default==prod**:脚本默认 `interval=0.20`,生产 `session_guard_cmd` 未传 `--interval` → 用默认)【默认 code localization_session_guard.py:160,234;生产 saas go2_saas_agent.py:2080-2090 不覆盖】。触发中止的三种情形:
  - **pid/boot_id/start_ticks 变了**【code localization_session_guard.py:94-100 `sessions_equal`、196-200】;
  - **odom 快照静默 >2s**(default==prod=2.0s,生产未覆盖)【默认 code localization_session_guard.py:159,223-229;生产 saas go2_saas_agent.py:2080-2090】;
  - **odom 时间戳倒退**【code localization_session_guard.py:209-216】。
  → 立即**中止**:patrol 分支 `pkill waypoint_follower/unitree_safe_cmd_node/unitree_cmd_node/cmd_vel_udp_sender/go2_sdk2_udp_receiver` + 发 **StopMove**(api_id **1003**,常量 `SPORT_API_ID_STOPMOVE=1003` 印证)+ motion_probe stop【code localization_session_guard.py:74,77,78,80;unitree_cmd_node.py:13】;recorder 分支停录 + 把半成品路线**改名 `.invalid.YYYYmmdd_HHMMSS`**(带时间戳;且仅在传了 `--invalidate-output` 时生效 —— patrol 不传、recorder 传)【code localization_session_guard.py:50-62,86-102,122】。**返回码 10**【code localization_session_guard.py:233】。
- **`go2_base_health_watchdog.py`**(巡检期间)〔【无狗上对照】;dog manifest 侧证 `base_watchdog_auto_restart=false`〕:订阅 `/Odometry`(生产以 `setsid nohup python3 -u <script>` **零 CLI 参**启动 → **全用脚本默认**,default==prod)【默认 code go2_base_health_watchdog.py:227;生产 saas go2_saas_agent.py:2344 无参启动】。逐帧校验:
  - **位姿有限、不越界**(`|x|,|y| ≤ 100m`)【默认 code go2_base_health_watchdog.py:65,67,235=100.0;watchdog 用 argparse 默认、不读 env → default==prod】;
  - **不跳变** —— ⚠️**公式更正**:真实动态上限 `allowed = max_jump_distance + max_jump_speed·dt = 0.80 + 0.60·dt`,**与 max_vx 无关,无 2.5 系数、无 0.3 常数**(旧版写的 `max_vx·dt·2.5+0.3` 全错)【默认 code go2_base_health_watchdog.py:81 公式、237-238 默认 0.80/0.60;无参启动 saas go2_saas_agent.py:2344 → 就是默认】;
  - **不静默**(`no_odom > 6s`)【默认 code go2_base_health_watchdog.py:159-163,243=6.0】。
  - **坏事累计 `bad_count=2` 才动作**【默认 code go2_base_health_watchdog.py:241=2】,且 ⚠️**行为分叉(旧版混为一谈,更正)**:
    - **越界/跳变** 累计 2 次 → 真正 `stop_patrol_processes` **停巡检**(杀 follower 那套),**但不重启 base**【code go2_base_health_watchdog.py:212-219,218】;
    - **静默(`no_odom_for_…`)被特判 → 不停巡检**:重置计数、**保持巡检安全零速、等待同一 FAST-LIO 恢复、明确不重启 base**,直接 return【code go2_base_health_watchdog.py:205-211】。
  - **停巡检但不重启 base**(`auto_restart=false`),为的是保住坐标连续性 —— 代码里**根本没有 base 重启路径**【code go2_base_health_watchdog.py:134 日志 `auto_restart=false`、221 "base was not restarted"】【狗上 dog manifest `base_watchdog_auto_restart=false`;saas 硬编码字面量 go2_saas_agent.py:1855】。
- **`check_route_start_alignment.py`**(巡检启动 gate)〔【无狗上对照】;生产值由 dog manifest 佐证〕:读快照,要求当前位姿距路线首点 `<max_distance`、yaw 误差 `<max_yaw`,否则**拒绝开跑**【gate 拒绝 code check_route_start_alignment.py:195-197】。⚠️ **默认≠生产**:
  - **脚本代码默认 = 0.8m / 0.9rad**【默认 code check_route_start_alignment.py:92-93】;
  - **生产 = 0.35m / 0.35rad**:saas `start_max_distance/start_max_yaw_error` 默认 0.35 且以 `-p` 传入【生产 saas go2_saas_agent.py:1485-1492,2333-2339】;
  - **狗上实际生效 = 0.35m / 0.35rad**(以 saas `-p` 覆盖为准)【狗上 dog manifest.txt:17 `start_alignment=…0.35m_0.35rad`】。旧版只写 0.35/0.35(=生产/狗上值,对)但没提脚本默认 0.8/0.9,补上。
- **`ensure_base_ready.sh`**〔【无狗上对照】;门限由 dog manifest 佐证〕:base 就绪门 + `--patrol-start-gate`(**CPU<85% 且 FAST-LIO 连续新鲜 3 次**)+ `--fresh-only`;base 不在就拉起 `base_bringup.sh`。85.0/3 为 **env 可覆盖默认,狗上用默认**【默认 code ensure_base_ready.sh:18=85.0,19=3,116-131 门、139-147 两 gate、5/176-186 拉起 base_bringup.sh;生产 saas go2_saas_agent.py:2323(--fresh-only)、2434(--patrol-start-gate)】【狗上 dog manifest `startup_cpu_max_pct=85.0`、`startup_stable_samples=3`】。

## 四、`/dev/shm/go2_fastlio_latest_odom.txt` 快照(01 的待坐实,这里定位)

- 格式:一行 `stamp=… x=… y=… z=… qx=… qy=… qz=… qw=…`,由 `manual_route_anchor.read_odom_snapshot` 解析【默认 code manual_route_anchor.py:24 `REQUIRED_ODOM_KEYS`、103-134 解析】。**写端(狗版)与读端字段完全对齐**【狗上 dog laserMapping.cpp:744-752 同序写出 stamp/x/y/z/qx/qy/qz/qw】。
- 由 FAST-LIO(**被改过的** `laserMapping.cpp`)高频写入,供上述所有守卫**低开销**读位姿 + 判新鲜度(不必订阅 ROS)。⚠️ **文件对照**:狗版副本 = **60358B = 58.9KB**,`publish_odometry` 内以 `.tmp→rename` **原子写** `/dev/shm/go2_fastlio_latest_odom.txt`,写入频率随每帧里程计触发,已在狗上副本直接核实【狗上 dog analysis/xunjian_20260725_shutdown_capture/previous_boot/remote_source/laserMapping.cpp:730-754】。**"被改过的"成立**(含快照写入 + 输入/输出健康门,非 stock FAST-LIO)。注意:**仓库版是另一个文件** = 61919B(≈60.5KB)、`sha256=5fec8282…`,与狗版 `e4cd05cb…` diff 216 行,**别把二者当同一文件** —— **repo≠dog(sha 验)**。
- `check_fastlio_freshness.py`(01)也读它【默认 code check_fastlio_freshness.py:10-21 `read_snapshot_stamp`、33 默认路径同上】〔【无狗上对照】〕。

## 五、一句话

巡检的"定位对不对",不只靠 FAST-LIO 出位姿,而是靠一整套**会话连续性守卫**:起始把路线锚到当前会话(manual_anchor,狗上默认)或对齐到地图(pcd,仅显式请求)→ 全程盯着"还是不是同一个 FAST-LIO 会话/位姿正不正常"→ 一旦坐标系可能变了(会话身份变/时间戳倒退/越界/跳变)就立即停,宁可中止也不让狗按错坐标乱跑;唯独 **odom 短暂静默是特例** —— 保零速、等同一会话回来、不停不重启,以免误杀了坐标连续性本身。

---

## 核验台账(claim → 证据 file:line → 判定)

> saas=`go2_saas_agent.py`;watchdog=`go2_base_health_watchdog.py`;guard=`localization_session_guard.py`;anchor=`manual_route_anchor.py`;align=`check_route_start_alignment.py`。**除 laserMapping.cpp(repo≠dog,sha 验)外全部【无狗上对照】,runtime 值靠两份 dog manifest 佐证。**

- 定位策略入口 `localization_mode_from_params` → saas:602(消费 :752/:1645) → **CONFIRMED**〔生产〕
- `manual_anchor` 是默认、当前位姿=首点、刚性搬、免 pcd → saas:662-668/765-770(+770 return)、anchor:260-299、dog manifest.txt:9 → **CONFIRMED**〔混合/双证〕
- 别名 manual/origin/direct/none(+manual_anchor/start_anchor/origin_direct)→ saas:620-627 → **CONFIRMED**〔生产〕
- pcd=route_relocalizer+同名地图 ICP,别名 relocalize/map/relocalization → saas:629-631/723-748/1989-1998 → **CONFIRMED**〔生产〕
- pcd 仅显式(localizationMode/legacy/env)选;`requireRelocalize=true` 是断言非选择器 → saas:639-685、700-701 → **CONFIRMED**〔生产;旧版措辞已收紧〕
- env `GO2_PATROL_LOCALIZATION_MODE` 仅兜底默认 → saas:662-664 → **CONFIRMED**〔默认/生产〕
- `requireRelocalize=true` 且模式非 pcd → raise → saas:700-701 → **CONFIRMED**〔生产〕
- 等静止:≥8 次/1s、位移<0.08m、yaw<0.08rad(default==prod) → anchor:23/178-205/365-369、saas:1919-1924 → **CONFIRMED**〔混合〕
- 刚性变换(delta_yaw 绕首点旋转+平移,源不改) → anchor:263-284/422-430 → **CONFIRMED**〔默认〕
- `current_fastlio_session()` 取 boot_id+pid+start_ticks 入 metadata → anchor:65/79/86-91/402 → **CONFIRMED**〔默认〕
- `--capture-only` 三消费端(pcd/录制/stop 收尾) → anchor:362/421、saas:2000/2494、route_recording_blackbox.py:279 → **CONFIRMED**〔生产〕
- guard 每 0.2s 比对会话(default==prod) → guard:160/234、saas:2080-2090 → **CONFIRMED**〔混合〕
- pid/boot_id/start_ticks 变 → 中止 → guard:94-100/196-200 → **CONFIRMED**〔默认〕
- odom 快照静默>2s → 中止(default==prod=2.0) → guard:159/223-229、saas:2080-2090 → **CONFIRMED**〔混合〕
- odom 时间戳倒退 → 中止 → guard:209-216 → **CONFIRMED**〔默认〕
- patrol 中止:pkill follower/safe/桥/sdk + StopMove(1003) → guard:74/77/78/80、unitree_cmd_node.py:13 → **CONFIRMED**〔默认〕
- recorder 中止:停录 + 改名 `.invalid.<ts>`(仅 --invalidate-output) → guard:50-62/86-102/122 → **CONFIRMED**〔默认〕
- 返回码 10 → guard:233 → **CONFIRMED**〔默认〕
- watchdog 订阅 `/Odometry`(无参启动=默认) → watchdog:227、saas:2344 → **CONFIRMED**〔混合〕
- 位姿有限、`|xy|≤100m`(default==prod) → watchdog:65/67/235=100.0 → **CONFIRMED**〔混合〕
- **跳变动态限 = `0.80 + 0.60·dt`(与 max_vx 无关)** → watchdog:81/237-238、saas:2344 → **CORRECTED**(旧:`max_vx·dt·2.5+0.3`)〔默认〕
- **静默(no_odom>6s)不停巡检:保零速/等同一会话恢复/不重启 base;仅越界·跳变累计 2 次才停** → watchdog:159-163/205-211/212-219/241/243 → **CORRECTED**(旧:静默也归"2 次坏→停巡检")〔默认〕
- 停巡检不重启 base(`auto_restart=false`)保坐标连续 → watchdog:134/218/221、saas:1855、dog manifest → **CONFIRMED**〔混合〕
- 启动对齐门 max_distance/max_yaw → **默认 0.8/0.9 vs 生产·狗上 0.35/0.35** → align:92-93(默认)/195-197(拒绝)、saas:1485-1492/2333-2339、dog manifest.txt:17 → **DEFAULT_VS_PROD**(狗上生效 0.35/0.35)〔混合〕
- `ensure_base_ready.sh`:就绪门 + 两 gate(CPU<85%·连续新鲜 3)+ 缺失拉 base_bringup → ensure_base_ready.sh:18/19/116-131/139-147/5/176-186、saas:2323/2434、dog manifest → **CONFIRMED**〔混合〕
- 快照格式 stamp/x/y/z/qx..qw,写读对齐 → anchor:24/103-134、dog laserMapping.cpp:744-752 → **CONFIRMED**〔混合〕
- 被改过的 laserMapping.cpp(狗版 58.9KB=60358B)原子写快照 → dog remote_source laserMapping.cpp:730-754 → **CONFIRMED**〔狗上;仓库版 61919B 且 sha 不同 = repo≠dog〕
- `check_fastlio_freshness.py`(01)也读该快照 → check_fastlio_freshness.py:10-21/33 → **CONFIRMED**〔默认〕

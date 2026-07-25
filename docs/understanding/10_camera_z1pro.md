# 10 · 相机(当前生产 = Go2 内置相机;Z1Pro 云台相机 = 停用/备用后端)

> 原则同 00。核心文件(路径均为 `orin_go2_fastlio_ws/` 下的简写,磁盘上带该前缀,无顶层
> `scripts/`、`config/` 目录——与 `00_overview.md:27,62` 同一约定,不算错误【README对照 00_overview.md:27,62】):
> - **共享分发器**:`scripts/go2_camera_capture.sh`(按后端分发)、`scripts/go2_camera_upload_segment.sh`(录+传)、`scripts/go2_camera_preset.sh`。
> - **Z1Pro 备用后端驱动**:`scripts/z1pro_gcu_control.py`、`scripts/z1pro_capture.sh`、`scripts/z1pro_preset.sh`;`scripts/z1pro_upload_segment.sh` 仅是兼容外壳(`exec` 到共享后端)。
> - **内置相机专属**:`scripts/build_go2_builtin_camera_capture.sh`(编译内置采集二进制,链 Unitree SDK2)。
> - **后端选择**:`config/camera.env` + `CAMERA_BACKEND_SWITCH.md`。可切换:Go2 内置相机 或 Z1Pro。

## 核验状态

本轮已对磁盘仓库源码**逐条**核对上述文件(协议/opcode/管线/调用链)。结论:**细节基本可信、主线错位需更正**。

- **【主线更正·最重要】** 原文全篇把 **Z1Pro 当作系统当前在用相机**——这在“当前用哪台相机”上判断相反。
  仓库实配 `config/camera.env:3` 为 `GO2_CAMERA_SOURCE=unitree_builtin`【仓库实配 config/camera.env:3】,
  即**当前激活后端是 Go2 内置相机**;`CAMERA_BACKEND_SWITCH.md:18` 明写 “The active value is unitree_builtin”、
  `:69` “Switching back to Z-1Pro / After the external camera is repaired”,表明 **Z1Pro 处于停用待修状态**
  【仓库文档 CAMERA_BACKEND_SWITCH.md:18,69-80】。原文仅第 5 行轻描“可切换”,未点明现状,故须以
  **“内置相机=当前生产、Z1Pro=停用/备用驱动”** 为主线重读。
- **细节可信**:Z1Pro 的名字(Z-1Pro/Xianfei)、GCU 协议、opcode、RTSP 管线等——就“z1pro 两个驱动文件论其自身行为”而言基本准确且可复核;这些内容保留,只在“是否当前在用”上打休眠标注。
- **【无狗上对照·必须明写】** 本篇涉及的**全部相机源文件**(`z1pro_*`、`go2_camera_*`、`camera.env`、`go2_saas_agent.py`)**在狗上均无副本**(狗上 `remote_source` 仅 `laserMapping / lddc / lds / waypoint_follower_go2_2` 四份);两份运行期 manifest(`xunjian-20260725-06/07`)`grep camera/z1pro/builtin/rtsp/video` **全无命中**→ manifest 不记录相机后端。因此**只能确认“仓库配置=unitree_builtin”,狗端实际生效哪个后端不可从证据核实**,不得默认仓库==狗上。

> **源标签约定**:【默认 file:line】= 脚本兜底默认/驱动自身行为;【生产 saas file:line】= SaaS 装配/巡检链路;
> 【仓库实配 file:line】= 仓库配置文件取值(camera.env);【仓库文档 file:line】= 仓库内 md 文档(佐证,非代码真相);
> 【README对照】= 路径/约定对照;【无狗上对照】= 狗上无副本、不可比对;【推断-未验】= 方向合理但仅“未见反证”。

## 〇、当前用哪台相机(默认 ≠ 生产,先说清)

- **代码兜底默认 = z1pro**:`go2_camera_capture.sh:17` `CAMERA_SOURCE=${GO2_CAMERA_SOURCE:-z1pro}`
  【默认 go2_camera_capture.sh:17】;`go2_camera_upload_segment.sh:44`、`go2_camera_preset.sh:17` 同为 `:-z1pro`
  【默认 go2_camera_upload_segment.sh:44】【默认 go2_camera_preset.sh:17】。**若什么都不配,脚本会走 Z1Pro。**
- **仓库实配覆盖之 → 生产 = unitree_builtin**:`go2_camera_capture.sh:7-15` 先 `source camera.env`(`set -a`),
  `camera.env:3` 把 `GO2_CAMERA_SOURCE` 显式设为 `unitree_builtin`,盖过了 `:17` 的 `:-z1pro` 兜底
  【仓库实配 config/camera.env:3】【默认 go2_camera_capture.sh:7-15】。`camera.env:2` 注释支持值 `unitree_builtin, z1pro`。
- **狗上实际生效哪个**:**不可核实**——`camera.env` 无狗上副本、manifest 无 camera 字段【无狗上对照】。
  仓库真相为 `unitree_builtin`;狗端是否一致须以狗上 `camera.env` 或运行日志坐实,当前无证据。
- **结论**:下文第一、二节里 **Z1Pro 专属路径(RTSP 采集、`192.168.144.x`、GCU 2332)当前处于休眠**,
  仅当 `GO2_CAMERA_SOURCE=z1pro` 才被 `exec`(`go2_camera_capture.sh:191`)【默认 go2_camera_capture.sh:191】。

## 一、硬件与两个接口(Z1Pro 备用后端;当前休眠)

> 注意:本节描述的是 **Z1Pro 备用后端自身的两个接口**,仅在 `GO2_CAMERA_SOURCE=z1pro` 下有效;当前生产为内置相机(见第 〇 节)。

相机(备用)是 **Z-1Pro / 先飞(Xianfei)云台相机**【默认 z1pro_gcu_control.py:155,argparse 描述 “Z-1Pro/Xianfei GCU private protocol client”】,IP `192.168.144.108`:
1. **RTSP 视频流** `rtsp://192.168.144.108/`(`554` 端口,H.264)——取流录像/截图
   【默认 z1pro_capture.sh:6(RTSP_URL)、:109(554 探活)、:82-83(rtph264depay/h264parse)】。**仅 z1pro 后端有效。**
2. **GCU 私有控制协议**(TCP `2332`,`z1pro_gcu_control.py`)——云台+相机控制
   【默认 z1pro_gcu_control.py:157(--host 192.168.144.108)、:158(--port 2332)】:
   - 二进制包,CRC16-CCITT(**半字节查表变体** `crc16_ccitt_nibble`),头 `0xA8E5`(发,字节序 A8 E5)/`0x8A5E`(收,8A 5E)
     【默认 z1pro_gcu_control.py:9-10(headers)、:14(crc)】。
   - 命令与 opcode【默认 z1pro_gcu_control.py:106-151;choices 见 :156】:
     - 带**角度**(roll/pitch/yaw,±180°,由 `s16_scaled_deg` 钳到 ±18000→±180.00°,:30-32):`angle`(0x10)、`euler`(0x14)。
     - 带**速率**(roll/pitch/yaw):`rate`(0x00,与 probe 同 opcode 但带控制位)。
     - **不吃角度**(原文把它们并入“云台 roll/pitch/yaw”是分组不精确):`probe`(0x00)、`home`(0x03)。
     - 相机动作:`photo`(0x20)、`record-toggle`(0x21)、`focus`(0x26)、`zoom-in`(0x22)、`zoom-out`(0x23)、
       **`zoom-stop`(0x24,原文漏列)**、`zoom-set`(0x25,1–60x:钳 10..600 再 /10,:146-147)、`track-exit`(0x17)。

## 二、采集(共享分发器 + 两后端;当前实际跑内置)

**入口是分发器 `go2_camera_capture.sh`(两后端共用,非“内置相机路径”),按 `GO2_CAMERA_SOURCE` 分派**
【默认 go2_camera_capture.sh:185-216】:

- **`unitree_builtin`(当前生产)→ `record_builtin` / `snapshot_builtin` / `probe`**【默认 go2_camera_capture.sh:193-211】:
  - `record_builtin`:`BUILTIN_BIN stream <secs> <fps> <iface>`(内置采集二进制 ← **Unitree SDK2 `VideoClient::GetImageSample`**
    【仓库文档 CAMERA_BACKEND_SWITCH.md:55】)喂管线
    `fdsrc ! image/jpeg ! jpegparse ! nvv4l2decoder ! nvvidconv ! nvv4l2h264enc ! h264parse ! mp4mux ! filesink`
    (Jetson 硬解/硬编),输出 **`patrol_logs/videos/unitree_builtin_<时间戳>_<秒>s.mp4`**
    【默认 go2_camera_capture.sh:104-165,118,124-136】;录前 `wait_valid_time`(:114)。
  - 采集二进制由 `build_go2_builtin_camera_capture.sh` 编译(`g++` 链 Unitree SDK2)【默认 build_go2_builtin_camera_capture.sh:6-7,23-34】。
- **`z1pro`(当前休眠)→ `exec z1pro_capture.sh`**【默认 go2_camera_capture.sh:187-191】,其自身行为:
  - `record`(默认 20s):`gst-launch-1.0 rtspsrc(tcp) ! rtph264depay ! h264parse ! mp4mux ! filesink`
    → `patrol_logs/videos/z1pro_<时间戳>_20s.mp4`;先等有效墙钟
    【默认 z1pro_capture.sh:5(20s),:7(OUT_DIR),:75(wait_valid_time),:77(文件名),:75-83(管线)】。
  - `snapshot`:解一帧 → jpeg(`avdec_h264 ! videoconvert ! jpegenc ! multifilesink`)【默认 z1pro_capture.sh:94-102】。
  - `probe`:发 RTSP `OPTIONS` 探活(554)【默认 z1pro_capture.sh:104-113】。
  - 停止:`INT`→`TERM`→`KILL` 逐级收 gstreamer【默认 z1pro_capture.sh:17-38】。

> 即:第二节原文那条 `rtspsrc→mp4` 采集管线是 **Z1Pro 后端自身行为、当前休眠**;当前真正执行的是 `record_builtin`(`fdsrc`←SDK2 + 硬编码,前缀 `unitree_builtin_`)。

## 三、上传与巡检集成

- `z1pro_upload_segment.sh` 是**兼容外壳**,`exec` 到共享后端 `go2_camera_upload_segment.sh`
  【默认 z1pro_upload_segment.sh:7】;后者 `source camera.env`(:7-12)、`CAPTURE_SCRIPT` 默认
  `go2_camera_capture.sh`(:21,按 `GO2_CAMERA_SOURCE` 分发)、录制(:35)、端点
  `/robot/video/upload`(:48)、上传须 `GO2_CAMERA_UPLOAD/Z1PRO_UPLOAD=1`(:20,51),`meta.source` 字段回填
  `${GO2_CAMERA_SOURCE:-z1pro}`(:44,49——**此处 source 名也带 `:-z1pro` 兜底**)【默认 go2_camera_upload_segment.sh:7-12,20-21,44,48】。
- **生产链路(见 08)**:SaaS `start_patrol` 装配
  `python3 -u go2_saas_agent.py video-loop --seconds 20 --upload --run-file ...`(先 `touch patrol_video.active`)
  【生产 saas go2_saas_agent.py:2381-2392】;`cmd_video_loop`(:2928)→ `cmd_video_segment`(:2886 调 `scripts/z1pro_upload_segment.sh`);
  **门控**:仅 `managed_service_loop`(`run-file==SERVICE_VIDEO_RUN_FILE`)时,按 `patrol_video.active` 存在与否 idle
  → **仅巡检期间录**【生产 saas go2_saas_agent.py:2886,2894,2918,2928】;传到 `/robot/video/upload`【默认 go2_camera_upload_segment.sh:48】。
- **超时 `pkill`——【硬编码只匹配 Z1Pro,内置管线杀不到:实际缺陷】**:`cmd_video_segment` 超时(`seconds+60`,:2895)会
  `pkill -TERM` 后 `-KILL`(:2901,2903),但模式**硬编码** `[g]st-launch-1.0.*rtsp://192.168.144.108`,只匹配 Z1Pro 的
  rtsp 管线;**当前激活的内置相机 `record_builtin` 用 `fdsrc`、命令行无该 rtsp URL**(`go2_camera_capture.sh:124-136`),
  故**超时兜底杀不到内置采集管线**——这是“覆盖不全”的实际缺陷,须知悉
  【生产 saas go2_saas_agent.py:2895,2901,2903】【默认 go2_camera_capture.sh:124-136】。

## 四、要点

- 相机与雷达/运动是**独立子系统**,不参与巡检控制环、只做证据采集(视频/图片)回传——
  **方向合理但属推断**(相机脚本不写 `/patrol_cmd` 等控制话题,证据为“未见反证”)【推断-未验】。
- **接入网段:默认写的 `192.168.144.x` 只对 Z1Pro 成立(CORRECTED)**。当前 `unitree_builtin` 后端**不走该网段**:
  `record_builtin` 经 Unitree SDK2 取图,网口按 `GO2_BUILTIN_CAMERA_MAC`(`camera.env:8` = `4c:bb:47:ab:e4:c2`)解析,
  回退子网 `192.168.123.x`【默认 go2_camera_capture.sh:79-98,92】【仓库实配 config/camera.env:8】。
- **后端可插拔**(Go2 内置 / Z1Pro)属实;但 **`go2_camera_capture.sh` 是两后端共用分发器,不是“内置相机路径”(CORRECTED)**——
  `z1pro` 时它 `exec z1pro_capture.sh`(:191);**内置专属的是 `build_go2_builtin_camera_capture.sh`**(编内置二进制)
  【默认 go2_camera_capture.sh:185-216,191;build_go2_builtin_camera_capture.sh:6-7,23-34】。
- **狗上状态**:本篇全部相机源文件 **【无狗上对照】**——狗端实际后端/是否 `unitree_builtin` 均不可核实,仅确认仓库配置。

## 五、留待坐实

- **狗端实际相机后端**:最重要的待坐实项——需狗上 `camera.env`、`go2_saas_agent.py` 副本或运行日志证实是否 `unitree_builtin`(现无狗上对照)。
- `go2_camera_upload_segment.sh` / `camera.env` 的后端选择与上传字段细节(按需)。
- GCU 协议各命令的完整字段语义(以 `z1pro_gcu_control.py` 为准)。
- 内置相机分辨率/帧率(README 称 1920×1080、~15FPS,`camera.env:9` FPS=15)——以运行时产物坐实。

## 核验台账(claim → 证据 file:line → 判定)

| # | 断言(原文) | 证据 file:line | 判定 |
|---|---|---|---|
| 1 | **相机是 Z1Pro/先飞,当前在用**(标题+第8行,全文主线) | config/camera.env:3;CAMERA_BACKEND_SWITCH.md:18,69-71;go2_camera_capture.sh:17 | **DEFAULT_VS_PROD** — 兜底默认 z1pro / **生产实配 unitree_builtin**;狗上不可核实 |
| 2 | 后端可切换(camera.env + SWITCH.md) | go2_camera_capture.sh:17-25,185-216;camera.env:2-3 | CONFIRMED(但原文漏点明“当前=unitree_builtin、Z1Pro 停用”) |
| 3 | z1pro_upload_segment 是兼容外壳,exec 共享后端 | z1pro_upload_segment.sh:7;go2_camera_upload_segment.sh:5-12,21 | CONFIRMED |
| 4 | GCU IP 192.168.144.108,TCP 2332 | z1pro_gcu_control.py:157,158 | CONFIRMED(Z1Pro 专属) |
| 5 | RTSP rtsp://192.168.144.108/(554,H.264) | z1pro_capture.sh:6,82-83,109 | CONFIRMED(仅 z1pro 后端) |
| 6 | CRC16-CCITT,头 0xA8E5/0x8A5E | z1pro_gcu_control.py:9-10,14-27 | CONFIRMED(半字节变体) |
| 7 | 命令 probe/home/angle/euler/rate/photo/record-toggle/focus/zoom-*/track-exit | z1pro_gcu_control.py:106-151,156 | CONFIRMED(**漏 zoom-stop 0x24**;probe/home 不吃角度,分组已更正) |
| 8 | record 20s:rtspsrc→mp4 → z1pro_<ts>_20s.mp4 | z1pro_capture.sh:5,7,75-83,77 | CONFIRMED(**z1pro 后端自身行为,当前休眠**;实跑 record_builtin) |
| 9 | snapshot/probe/停止 INT→TERM→KILL | z1pro_capture.sh:94-102,104-113,17-38 | CONFIRMED(仅 z1pro 后端) |
| 10 | 生产链 video-loop→cmd_video_segment→z1pro_upload_segment(patrol_video.active 门控) | go2_saas_agent.py:2381-2392,2886,2918,2928;go2_camera_upload_segment.sh:48 | CONFIRMED |
| 11 | 超时 pkill gst-launch | go2_saas_agent.py:2895,2901,2903;go2_camera_capture.sh:124-136 | CONFIRMED(**pkill 硬编码 rtsp://192.168.144.108,杀不到内置 fdsrc 管线:实际缺陷**) |
| 12 | 通过 IP 网络 192.168.144.x 接入 | go2_camera_capture.sh:79-98,92;camera.env:8 | **CORRECTED** — 仅 Z1Pro;内置按 MAC/192.168.123.x 解析、走 SDK2 |
| 13 | go2_camera_capture.sh / build_*.sh 为“内置相机路径” | go2_camera_capture.sh:185-216,191;build_go2_builtin_camera_capture.sh:6-7,23-34 | **CORRECTED** — 前者是两后端共用分发器;后者才内置专属 |
| 14 | 核心文件路径写作 scripts/、config/ | 00_overview.md:27,62 | CONFIRMED(简写约定,磁盘带 orin_go2_fastlio_ws/ 前缀) |
| — | “不参与控制环、只做证据采集” | 相机脚本未写 /patrol_cmd 等控制话题 | 【推断-未验】方向合理,证据为“未见反证” |

**狗上对照总览**:`z1pro_gcu_control.py / z1pro_capture.sh / z1pro_upload_segment.sh / go2_camera_upload_segment.sh / go2_camera_capture.sh / z1pro_preset.sh / go2_camera_preset.sh / build_go2_builtin_camera_capture.sh / config/camera.env / go2_saas_agent.py / CAMERA_BACKEND_SWITCH.md` —— **全部【无狗上对照】**(狗上 remote_source 仅 laserMapping/lddc/lds/waypoint_follower_go2_2 四份;manifest 无 camera 字段)。仅确认仓库源码;**不得默认仓库==狗上**。

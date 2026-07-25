# 10 · 相机(Z1Pro 云台相机)

> 原则同 00。核心文件:`scripts/z1pro_gcu_control.py`、`scripts/z1pro_capture.sh`、
> `scripts/z1pro_upload_segment.sh`(→ `go2_camera_upload_segment.sh` 共享后端)、`scripts/z1pro_preset.sh`。
> 相机后端可切换(`config/camera.env` + `CAMERA_BACKEND_SWITCH.md`):Z1Pro 或 Go2 内置相机。

## 一、硬件与两个接口
相机是 **Z-1Pro / 先飞(Xianfei)云台相机**,IP `192.168.144.108`:
1. **RTSP 视频流** `rtsp://192.168.144.108/`(554 端口,H.264)——取流录像/截图。
2. **GCU 私有控制协议**(TCP `2332`,`z1pro_gcu_control.py`)——云台+相机控制:
   - 二进制包,CRC16-CCITT,头 `0xA8E5`(发)/`0x8A5E`(收)。
   - 命令:`probe/home/angle/euler`(云台 roll/pitch/yaw,±180°)、`rate`、`photo`、`record-toggle`、`focus`、`zoom-in/out/set`(1–60x)、`track-exit`。

## 二、采集(`z1pro_capture.sh`)
- `record`(默认 20s):`gst-launch-1.0 rtspsrc(tcp) ! rtph264depay ! h264parse ! mp4mux ! filesink` → `patrol_logs/videos/z1pro_<时间戳>_20s.mp4`;先等有效墙钟(文件名带时间戳)。
- `snapshot`:解一帧 → jpeg。
- `probe`:发 RTSP OPTIONS 探活。
- 停止用 INT→TERM→KILL 逐级收 gstreamer。

## 三、上传与巡检集成
- `z1pro_upload_segment.sh` 是**兼容外壳**,`exec` 到共享后端 `go2_camera_upload_segment.sh`(按 `config/camera.env` 选相机)。
- 生产链路(见 08):SaaS **video-loop** → `cmd_video_segment` → `z1pro_upload_segment.sh`(录 20s + 传),**仅巡检期间**(`patrol_video.active` 门控)录;超时会 `pkill gst-launch`;传到 `/robot/video/upload`。

## 四、要点
- 相机与雷达/运动是**独立子系统**,通过 IP 网络(192.168.144.x)接入,不参与巡检控制环,只做证据采集(视频/图片)回传。
- **后端可插拔**(Z1Pro / Go2 内置),`go2_camera_capture.sh` / `build_go2_builtin_camera_capture.sh` 为内置相机路径。

## 五、留待坐实
- `go2_camera_upload_segment.sh` / `camera.env` 的后端选择与上传字段细节(按需)。
- GCU 协议各命令的完整字段语义(以 `z1pro_gcu_control.py` 为准)。

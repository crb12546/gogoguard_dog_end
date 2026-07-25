# Z-1Pro Camera Notes

> Camera business logic now uses the shared backend described in
> `CAMERA_BACKEND_SWITCH.md`. Keep this document for Z-1Pro-specific RTSP and
> gimbal details; do not call these backend scripts from new patrol logic.

Verified on 2026-07-05.

## Network

- Camera IP: `192.168.144.108`
- Orin wired interface profile includes: `192.168.144.100/24`, `192.168.123.18/24`, `192.168.1.5/24`
- Mac reaches Orin over home WiFi with `ssh go2home`

## Verified Video

- RTSP URL: `rtsp://192.168.144.108/`
- RTSP server: `lal0.37.4`
- Codec: H.264
- Open ports: `80`, `554`, `22`, `23`
- HTTP root and common API paths return `404`; the HTTP service does not expose a normal web UI on tested paths.

Orin has working GStreamer plugins for RTSP/H.264 capture:

- `rtspsrc`
- `rtph264depay`
- `h264parse`
- `mp4mux`
- `avdec_h264`
- `jpegenc`

Helper script on Orin:

```bash
/home/unitree/go2_fastlio_ws/scripts/z1pro_capture.sh probe
/home/unitree/go2_fastlio_ws/scripts/z1pro_capture.sh snapshot
/home/unitree/go2_fastlio_ws/scripts/z1pro_capture.sh record 20
```

GoGoGuard upload helper on Orin:

```bash
# record only / dry-run upload metadata
Z1PRO_UPLOAD=0 GO2_ROBOT_ID=go2-tju-01 /home/unitree/go2_fastlio_ws/scripts/z1pro_upload_segment.sh 20

# record and POST to /api/v1/robot/video/upload
Z1PRO_UPLOAD=1 GO2_ROBOT_ID=go2-tju-01 /home/unitree/go2_fastlio_ws/scripts/z1pro_upload_segment.sh 20
```

Default upload endpoint:

```text
https://39.96.37.187/api/v1/robot/video/upload
```

`meta.recordedAt` is generated after segment recording and therefore represents the segment end time, as required by the GoGoGuard adapter API.

Outputs are stored under:

```text
/home/unitree/go2_fastlio_ws/patrol_logs/videos/
```

## Verified Samples

- Local MP4 sample: `z1pro_test_20260705_121724.mp4`
- Local JPEG sample: `z1pro_snapshot_latest.jpg`

## Control Status

Video is solved. Gimbal control is now partially verified.

HTTP/ONVIF/PTZ-style paths all returned `404`, including:

- `/api`
- `/api/v1`
- `/control`
- `/ptz`
- `/onvif/device_service`
- `/cgi-bin/ptz.cgi`
- `/cgi-bin/magicBox.cgi?action=getSystemInfo`
- `/status`
- `/debug`
- `/config`

The public Unitree SDK2 tree exposes Go2 video/image client APIs but no obvious Z-1Pro gimbal/PTZ client in local headers or examples.

Do not attempt SSH/Telnet login to the camera without explicit credentials or vendor documentation.

The vendor page at `https://www.allxianfei.com/z-1Pro.html` provides Z-1Pro documents. The useful control document is:

```text
GCU私有通信协议-XF(A5)V2.0.6
```

Important protocol facts extracted from the document:

- Serial: TTL, 8N1, full duplex, baud rates `115200`, `250000`, `500000`, `1000000`
- UDP mode: source port `2337`, destination port `2338`
- TCP Server mode: camera/GCU listens on TCP `2332`
- Send packet header: `A8 E5`
- Receive packet header: `8A 5E`
- Protocol version: `0x02`
- Packet structure: header, length, version, 32-byte main frame, 32-byte sub frame, command, optional params, CRC16 high byte, CRC16 low byte
- CRC covers bytes `0..S-3`, CRC bytes are high then low; other numeric fields are little-endian
- Command byte offset is `69`

Useful commands:

- `0x00`: empty command, used to separate repeated same commands
- `0x03`: home/center
- `0x10`: angle control
- `0x14`: Euler angle control
- `0x17`: tracking enter/exit
- `0x1A`: point translation
- `0x20`: photo
- `0x21`: record start/stop
- `0x22` / `0x23` / `0x24` / `0x25`: zoom in/out/stop/specified zoom
- `0x26`: focus

Helper script on Orin:

```bash
/home/unitree/go2_fastlio_ws/scripts/z1pro_gcu_control.py probe
/home/unitree/go2_fastlio_ws/scripts/z1pro_gcu_control.py home
/home/unitree/go2_fastlio_ws/scripts/z1pro_gcu_control.py angle --roll 0 --pitch -10 --yaw 0
/home/unitree/go2_fastlio_ws/scripts/z1pro_gcu_control.py angle --roll 0 --pitch 0 --yaw 0
```

Verified:

- TCP `2332` is open
- `probe` receives GCU state packets
- `home` returns success status
- Small `angle` pitch command changes absolute pitch as expected, then can return to `0` degrees
- `focus` command returns failure status `0x01`; the user manual describes the visible-light lens as a fixed-focus camera (`定焦相机`)
- `zoom-set --zoom 1.0` returns success and is useful before clarity tests to avoid digital zoom blur

Local visual test packages:

```text
z1pro_visual_test/index.html      # angle stills/videos + motion video
z1pro_focus_test/index.html       # before/focus/zoom-1x clarity comparison
```

## Next Steps

1. Wrap safe gimbal commands in the dog agent: `probe`, `home`, bounded `angle`, `photo`, `record-toggle`.
2. Add software angle limits before exposing gimbal commands through GoGoGuard.
3. Keep RTSP segment upload as the first platform integration path.
4. Add target tracking/point translation only after UI and safety limits are defined.

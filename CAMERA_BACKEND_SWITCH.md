# Go2 Camera Backend Switch

The patrol camera now has one shared entry point. Snapshot, segmented recording,
management-console preview, media discovery, and GoGoGuard upload no longer call
Z-1Pro directly.

## Current setting

On the Orin, edit:

```text
/home/unitree/go2_fastlio_ws/config/camera.env
```

The active value is:

```bash
GO2_CAMERA_SOURCE=unitree_builtin
```

Supported values:

```bash
GO2_CAMERA_SOURCE=unitree_builtin  # fixed Go2 front camera
GO2_CAMERA_SOURCE=z1pro            # external Z-1Pro RTSP + gimbal
```

The file is read at the start of every snapshot/video segment, so a change takes
effect on the next operation. If a segment is already recording, let it finish
or stop that recording first. The persistent SaaS services do not normally need
to be restarted just for this setting.

## Shared interfaces

```bash
/home/unitree/go2_fastlio_ws/scripts/go2_camera_capture.sh probe
/home/unitree/go2_fastlio_ws/scripts/go2_camera_capture.sh snapshot
/home/unitree/go2_fastlio_ws/scripts/go2_camera_capture.sh record 20
/home/unitree/go2_fastlio_ws/scripts/go2_camera_preset.sh front
/home/unitree/go2_fastlio_ws/scripts/go2_camera_upload_segment.sh 20
```

Both sources still produce JPEG snapshots and H.264 MP4 video files under:

```text
/home/unitree/go2_fastlio_ws/patrol_logs/videos/
```

The existing GoGoGuard upload endpoint, patrol-triggered 20-second loop, media
manifest, local preview, range playback, and retry/outbox behavior remain above
this shared interface.

## Built-in camera implementation

- Input: Unitree SDK2 `VideoClient::GetImageSample`
- Network interface: resolved on every operation from the stable wired-port MAC;
  it remains valid if Linux renames the port between `eth0` and `eth1`
- Image: 1920x1080 JPEG, approximately 15 FPS
- Encoding: Jetson NVIDIA JPEG decoder and H.264 hardware encoder
- Output prefix: `unitree_builtin_`
- Direction: fixed forward; pan/tilt presets are unsupported

If the helper binary ever needs rebuilding:

```bash
/home/unitree/go2_fastlio_ws/scripts/build_go2_builtin_camera_capture.sh
```

## Switching back to Z-1Pro

After the external camera is repaired:

1. Change only `GO2_CAMERA_SOURCE=z1pro` in `config/camera.env`.
2. Run `go2_camera_capture.sh probe` and confirm an RTSP 200 response.
3. Run `go2_camera_preset.sh probe` and confirm GCU communication.
4. Record a short segment with `go2_camera_capture.sh record 5`.

Z-1Pro-specific drivers remain in `z1pro_capture.sh`,
`z1pro_gcu_control.py`, and `z1pro_preset.sh`; they are now backend
implementations rather than direct business-logic entry points.

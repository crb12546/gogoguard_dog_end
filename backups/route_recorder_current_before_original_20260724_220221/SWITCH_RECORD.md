# Switch record

## Repository state

At switch time, none of these locations was a Git repository:

- `/Users/constantine/Project/Go2`
- `/Users/constantine/Project/Go2/orin_go2_fastlio_ws`
- `/Users/constantine/Desktop/go2_original_linux_code_20260708`
- `/home/unitree/go2_fastlio_ws` on the robot

The `.git` directories found on the robot belong only to third-party projects
such as FAST_LIO, Livox-SDK2, livox_ros_driver2, and unitree_sdk2.

## Active replacement

The active recorder was replaced with the exact desktop original:

`go2_original_linux_code_20260708/source_original_candidate/orin_go2_fastlio_ws/src/go2_fastlio_patrol/go2_fastlio_patrol/route_recorder.py`

SHA256 after the switch:

```text
e7354b66f18cab944511e6fdb4c3cf1415bef227c3f553ecae03f55718db0a82  route_recorder.py
b72a64fb62722202554934acc0e0f0df163b2c5fab2a6bf3036c113b7644e2fd  patrol_console/server.py
3f95b44ffde483f921ac396bd9af0702e8221f4a20a9c10a1c905c1bcbf2ab70  patrol_console/static/index.html
```

Robot verification:

- ROS 2 package rebuilt with
  `colcon build --packages-select go2_fastlio_patrol --symlink-install`.
- Robot source and build copies both had recorder SHA256
  `e7354b66f18cab944511e6fdb4c3cf1415bef227c3f553ecae03f55718db0a82`.
- Runtime import resolved to
  `/home/unitree/go2_fastlio_ws/build/go2_fastlio_patrol/go2_fastlio_patrol/route_recorder.py`.
- A stationary smoke recording wrote a valid header and one odometry point to
  `/tmp/route_recorder_original_switch_smoke_20260724_220221.csv`.
- A full patrol-console start/stop test wrote a valid CSV and returned
  `STOPPED` plus `LINES=2`; no recorder process remained afterward.

## Caller adaptation

The local patrol console was changed to:

- pass only `route_file` and `min_distance=0.40`;
- describe the recorder as original fixed-distance sampling;
- stop the exact recorder process group with `SIGINT` first, then bounded
  `SIGTERM`/`SIGKILL` fallbacks;
- stop reading the quality-gated recorder's JSON report;
- avoid treating stale `valid=false` JSON as the result of an original-recorder
  run.

`route_quality.py` and its tests remain in the package but are no longer
imported by the active recorder. They are also captured in this backup.

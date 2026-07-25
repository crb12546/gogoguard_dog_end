# Go2 Motion Control Chain Analysis

Date: 2026-07-05

## Goal

The immediate target is not to rewrite patrol. The target is to make an existing recorded route, such as `roomtest7.csv`, execute through the correct existing motion-control chain.

## Known Good Project Pieces

- `route_recorder` records FAST-LIO `/Odometry` into CSV route points.
- `waypoint_follower` reads a CSV route and publishes `/patrol_cmd` as `geometry_msgs/Twist`.
- Today logs showed `waypoint_follower` can produce nonzero commands such as `vx=0.10`.
- 2026-06-28 ROS logs on the Orin show multiple real route recordings and executions; the robot clock is currently NTP-synchronized, so these file times should be treated as meaningful unless older boot logs prove otherwise.

Therefore, route recording and waypoint command generation are not the primary suspects.

## Chain A: README Safe Chain

Documented in `README.md` section 10:

```text
waypoint_follower
-> /patrol_cmd
-> unitree_safe_cmd_node
-> /api/sport/request
-> Go2 sport service
```

`unitree_safe_cmd_node` adds point-cloud safety logic using `/cloud_registered_body`. If no timeout or obstacle is active, it constructs a `unitree_api/msg/Request`:

```text
api_id = 1008  # Move
parameter = {"x": vx, "y": 0.0, "z": yaw_rate}
```

This matches the official SDK2 `Move` API id, but it still depends on the robot's `/api/sport/request` service accepting and executing the request.

### 2026-06-28 Field Evidence

This is the strongest known-good evidence so far. The Orin currently reports synchronized time (`Asia/Shanghai`, NTP active), and the following 2026-06-28 files exist on the robot:

```text
/home/unitree/go2_fastlio_ws/src/go2_fastlio_patrol/routes/route_test_0519.csv  # 12:25, 6760 points, FAST-LIO drifted badly
/home/unitree/go2_fastlio_ws/src/go2_fastlio_patrol/routes/route_test_0520.csv  # 12:52, 23 points, ~9.5 m
/home/unitree/go2_fastlio_ws/src/go2_fastlio_patrol/routes/route_test_0517.csv  # 16:14, 1329 points, ~587 m
```

The execution logs were not in `patrol_logs`; they were preserved under `/home/unitree/.ros/log/python3_*.log`. For `route_test_0517.csv`, logs show the README safe chain:

```text
waypoint_follower -> /patrol_cmd -> unitree_safe_cmd_node -> /api/sport/request
```

Important runs:

```text
16:30  waypoint_follower route_test_0517.csv, points=1329, loop_mode=pingpong, vx=0.50, nearest 0..283
16:31  unitree_safe_cmd_node ACTIVE, nonzero Move x=0.500 count=505
16:44  waypoint_follower route_test_0517.csv, points=1329, loop_mode=pingpong, vx=0.50, nearest 0..1328
16:45  unitree_safe_cmd_node ACTIVE, nonzero Move x=0.500 count=803
18:16  waypoint_follower route_test_0517.csv, points=1329, loop_mode=pingpong, vx=0.50, nearest 0..1328, reached start and switched forward
18:16  unitree_safe_cmd_node ACTIVE, nonzero Move x=0.500 count=4712
```

This corrects the earlier emphasis on Chain B. Chain B has old CLI evidence, but 2026-06-28 field evidence points to Chain A as the chain that was actually used for the handed-over successful run.

### Invocation Comparison With `roomtest7.csv`

`roomtest7.csv` does not conflict with the 2026-06-28 evidence. The mismatch was the invocation:

```text
2026-06-28 field call:
	waypoint_follower route_file:=route_test_0517.csv or route_test_0519.csv
	v_base:=0.5 max_vx:=0.5 max_yaw_rate:=0.45
	lookahead_distance:=0.6 reach_distance:=0.4 goal_distance:=0.25
	loop_mode:=pingpong search_window:=6 relocalize_distance:=1.5
	unitree_safe_cmd_node max_vx:=0.5 publish_rate:=20.0
	pointcloud_topic:=/cloud_registered_body
	stop_distance:=0.40 resume_distance:=0.50 min_stop_points:=15
	roi_x=[0.35,0.90], roi_y=[-0.30,0.30], roi_z=[0.30,0.90]

Current failed roomtest7 CLI test:
	waypoint_follower route_file:=roomtest7.csv
	v_base:=0.10 max_vx:=0.10
	loop_mode:=once
	unitree_cmd_node, not unitree_safe_cmd_node
```

So the fair `roomtest7.csv` test is to keep the route file but reproduce the 2026-06-28 safe-chain invocation. The management console defaults and `run_roomtest7_readme_safe_patrol.sh` have been aligned to `SPEED=0.50` and `LOOP_MODE=pingpong` for that purpose.

### 2026-07-05 `roomtest7.csv` Safe-Chain Result

The fair test succeeded: `roomtest7.csv` moved the robot through the safe chain.

```text
Run 1:
	route_file:=roomtest7.csv
	route points: 3
	loop_mode: pingpong
	vx=0.50
	nearest 0 -> 1
	reach end, switch to backward

Run 2:
	route_file:=roomtest7.csv
	route points: 3
	loop_mode: pingpong
	vx=0.20
	nearest 0 -> 1
	reach end, switch to backward
	reach start, switch to forward
```

This confirms the main movement chain is working:

```text
waypoint_follower -> /patrol_cmd -> unitree_safe_cmd_node -> /api/sport/request
```

The apparent early turn is explained by the test route being only ~0.9 m and `goal_distance:=0.25`. On a three-point route, reaching within 25 cm of the endpoint can feel like turning around halfway. This is a route/control-threshold issue, not a bridge failure.

Earlier `once` evidence should not be treated as proof that single-pass mode is broken: the logged `once` run used `vx=0.10` and the old `unitree_cmd_node` chain, while the successful runs used `unitree_safe_cmd_node` and `vx=0.50/0.20`. A fair once-mode test should use the safe chain, `vx>=0.20`, and preferably a smaller `goal_distance` such as `0.10` for this short route.

Obstacle avoidance was active but not fully protective in the room test. Logs show ROI points and obstacle events, but also many frames where objects were in ROI with `nearest_x` around `0.41-0.50` while `stop_count=0`, because the current stop threshold is `stop_distance:=0.40` and `min_stop_points:=15`. The safety node only gates the commanded motion; it is a narrow forward ROI filter, not full-body collision avoidance.

### 2026-07-05 `xiaoqu1.csv` Field Result

The larger field test produced good route/map artifacts and confirmed that the safe chain can follow a long recorded route, but also exposed a follower tuning issue.

Recorded artifacts:

```text
/home/unitree/go2_fastlio_ws/src/go2_fastlio_patrol/routes/xiaoqu1.csv
	315 route points, ~142.45 m
	bbox x=[0.005, 65.478], y=[-5.985, 6.663]
	step median=0.446 m, max=0.557 m
	no coordinate explosion or obvious jump

/home/unitree/go2_fastlio_ws/maps/console/xiaoqu1.pcd
	~490,313 points, ~11.4 MB
```

Patrol attempts:

```text
Run 1: xiaoqu1.csv, safe chain, nearest 0..90 / 315
Run 2: xiaoqu1.csv, safe chain, nearest 0..258 / 315
```

This is a successful recording/build-map/long-route-following validation. The remaining problem is follower tuning on long routes, not motion-bridge failure. In the second run the follower spent long periods in turn-in-place behavior:

```text
alpha ~= 2.7 rad
vx=0.00
yaw_rate=0.45
stuck recovery around nearest=65 and nearest=258
```

The next tuning pass should reduce long-route instability: start at `v_base=0.20-0.25`, use a larger `lookahead_distance` such as `0.8-1.0`, and tune turn-in-place behavior so the robot does not get trapped rotating when the route heading changes sharply.

## Chain B: Old CLI Cmd Chain

Implemented in `scripts/patrol_cli.disabled_before_cmd_rework.py`:

```text
waypoint_follower
-> /patrol_cmd
-> unitree_cmd_node
-> /api/sport/request
-> Go2 sport service
```

`unitree_cmd_node` has no point-cloud safety layer. It also constructs `api_id=1008` Move requests. This is the exact chain used by the old CLI `start_patrol()` function, so it should be tested before inventing any new bridge.

## Chain C: graph_pid_ws /go2_cmd Chain

Seen in `/unitree/module/graph_pid_ws/src/QT_Server/shell/*.sh`:

```text
/go2_cmd
-> go2_control_by_sdk send_cmd
-> old unitree::robot::SportClient
-> Go2
```

This is a separate existing motion bridge. It is not the same as `/api/sport/request`.

Current finding: `go2_control_by_sdk send_cmd` fails at startup on this environment because its old SDK/DDS runtime expects CycloneDDS shared-memory/iceoryx symbols and config that are no longer present:

```text
undefined symbol: free_iox_chunk
undefined symbol: iceoryx_header_from_chunk
SharedMemory: unknown element
```

So Chain C may be historically important, but it is not currently healthy until its runtime environment is fixed.

## Official Unitree SDK2 Facts

Official SDK2 Go2 movement uses:

```cpp
ChannelFactory::Instance()->Init(0, networkInterface);
unitree::robot::go2::SportClient sport_client;
sport_client.Init();
sport_client.BalanceStand();
sport_client.Move(vx, vy, vyaw);
sport_client.StopMove();
```

The official `sport_api.hpp` defines:

```text
BalanceStand = 1002
StopMove     = 1003
Move         = 1008
service name = "sport"
```

SDK channel naming uses:

```text
rt/api/<service>/request
rt/api/<service>/response
```

So `/api/sport/request` is a reasonable ROS/DDS mapping of the official sport request channel. But correctness of message construction is not enough; the robot service state and response must be checked.

## Missing Diagnosis

Before declaring Chain A or B wrong, inspect:

```text
/api/sport/response
robot_state ServiceList
sport_mode status/protect
```

The official `RobotStateClient` exposes:

```cpp
ServiceList()
ServiceSwitch("sport_mode", 0/1, status)
```

This suggests Go2 may reject or ignore `Move` if the sport service mode is disabled, protected, or superseded by another mode.

## Test Scripts Added

Two scripts reproduce existing project chains for `roomtest7.csv` without rewriting patrol logic:

```bash
/home/unitree/go2_fastlio_ws/scripts/run_roomtest7_readme_safe_patrol.sh
/home/unitree/go2_fastlio_ws/scripts/run_roomtest7_cli_cmd_patrol.sh
```

They are interactive and require Enter before starting. They stop on Ctrl+C and send `StopMove` on exit.

Use them only with the robot stood up, operator nearby, and remote controller in hand.

## Current Recommendation

1. Reproduce the 2026-06-28 field chain first: `waypoint_follower -> unitree_safe_cmd_node -> /api/sport/request`.
2. Do not use `roomtest7.csv` as the main proof route. It is only 3 points/~0.9 m and was tested at `vx=0.10`; the field run used long routes and `vx=0.50`.
3. Prefer a safer scaled reproduction of the known-good conditions: `route_test_0520.csv` or a freshly recorded 5-10 m route, `loop_mode=pingpong`, and a moderate speed closer to the field run than `0.10`.
4. If the safe chain publishes Move but Go2 still does not move, inspect `/api/sport/response` and `robot_state` service status before trying another bridge.
5. Treat Chain C (`go2_control_by_sdk`) as a separate graph_pid_ws bridge that needs runtime repair before it can be trusted.
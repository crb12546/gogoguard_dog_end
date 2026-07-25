#!/usr/bin/env python3
import importlib.util
import json
import os
import tempfile
import threading
import unittest
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from types import SimpleNamespace


ROOT = Path(__file__).resolve().parents[2]
AGENT_PATH = ROOT / "orin_go2_fastlio_ws/scripts/go2_saas_agent.py"


def load_agent():
    spec = importlib.util.spec_from_file_location("go2_saas_agent_tested", str(AGENT_PATH))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class FakeGoGoGuard:
    def __init__(self):
        self.commands = []
        self.heartbeats = []
        self.results = []
        self.uploads = []
        self.gets = []
        self.get_requests = []
        self.plan = {"device_id": "robot-a", "site_id": "site-a", "points": [], "routes": [{"name": "xiaoqu1", "points": []}]}
        self.route_csv = "id,x,y,yaw,v\n0,0,0,0,0.2\n1,1,0,0,0.2\n"
        self.server = None
        self.thread = None
        self.base = None

    def __enter__(self):
        owner = self

        class Handler(BaseHTTPRequestHandler):
            def _read(self):
                length = int(self.headers.get("Content-Length", "0"))
                return self.rfile.read(length)

            def _json(self, status, payload):
                body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
                self.send_response(status)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self):
                owner.gets.append(self.path)
                owner.get_requests.append({"path": self.path, "headers": dict(self.headers)})
                if self.path.endswith("/health"):
                    self._json(200, {"ok": True})
                elif self.path.endswith("/devices/plan"):
                    self._json(200, owner.plan)
                elif self.path.endswith("/routes/xiaoqu1.csv"):
                    body = owner.route_csv.encode("utf-8")
                    self.send_response(200)
                    self.send_header("Content-Type", "text/csv")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                else:
                    self._json(404, {"detail": "Not Found"})

            def do_POST(self):
                body = self._read()
                if self.path.endswith("/robot/heartbeat"):
                    payload = json.loads(body.decode("utf-8"))
                    owner.heartbeats.append({"headers": dict(self.headers), "payload": payload})
                    self._json(200, {"ok": True, "registered": True, "robotId": payload.get("robotId"), "status": "巡检中", "commands": owner.commands})
                elif self.path.endswith("/robot/command/result"):
                    owner.results.append(json.loads(body.decode("utf-8")))
                    self._json(200, {"ok": True})
                elif self.path.endswith("/robot/video/upload"):
                    owner.uploads.append({"headers": dict(self.headers), "body": body})
                    self._json(200, {"ok": True, "hadPose": b'name="position"' in body, "framesScanned": 8})
                elif self.path.endswith("/robot/asset/upload"):
                    owner.uploads.append({"headers": dict(self.headers), "body": body})
                    self._json(200, {"ok": True, "kind": "asset"})
                else:
                    self._json(404, {"detail": "Not Found"})

            def log_message(self, *_):
                return

        self.server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        port = self.server.server_address[1]
        self.base = "http://127.0.0.1:%d/api/v1" % port
        self.thread = threading.Thread(target=self.server.serve_forever)
        self.thread.daemon = True
        self.thread.start()
        return self

    def __exit__(self, *_):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=5)


class EnvPatch:
    def __init__(self, **updates):
        self.updates = updates
        self.old = {}

    def __enter__(self):
        for key, value in self.updates.items():
            self.old[key] = os.environ.get(key)
            if value is None:
                os.environ.pop(key, None)
            else:
                os.environ[key] = value

    def __exit__(self, *_):
        for key, value in self.old.items():
            if value is None:
                os.environ.pop(key, None)
            else:
                os.environ[key] = value


class Go2SaasAgentTest(unittest.TestCase):
    def setUp(self):
        self.agent = load_agent()
        self.pose = {"frame": "fast_lio_map", "source": "/Odometry", "x": 1.2, "y": 3.4, "z": 0.5, "yaw": 0.6}
        self.agent.collect_ros = lambda timeout_sec=0: {"ok": True, "pose": dict(self.pose), "sport": {"vx": 0.1, "vyaw": 0.2}, "battery": {"soc": 80}, "errors": []}
        self.agent.process_status = lambda: {"fastlio": True, "livox": True, "camera_loop": True}
        self.agent.network_summary = lambda: {"hostname": "test", "addresses": ["127.0.0.1"], "defaultRoute": "default"}
        self.agent.asset_manifest = lambda robot_id, limit=3: {"robotId": robot_id, "routes": [], "pcds": [], "media": []}

    def test_heartbeat_payload_position_aliases(self):
        args = SimpleNamespace(robot_id="robot-a", ros_timeout=0)
        payload = self.agent.heartbeat_payload(args)
        self.assertEqual(payload["robotId"], "robot-a")
        self.assertIn("time", payload)
        self.assertIn("timestamp", payload)
        self.assertEqual(payload["status"], "video_recording")
        self.assertEqual(payload["pose"], self.pose)
        self.assertEqual(payload["position"], self.pose)
        self.assertEqual(payload["motion"]["position"], self.pose)
        self.assertEqual(payload["motion"]["yaw_rad"], 0.6)
        self.assertEqual(payload["motion"]["velocity"], {"vx": 0.1, "vyaw": 0.2})
        self.assertEqual(payload["battery"], {"soc": 80})
        self.assertIn("diagnostics", payload)
        self.assertFalse(payload["patrol"]["running"])

    def test_command_poll_posts_success_and_rejection(self):
        with FakeGoGoGuard() as fake, EnvPatch(GO2_BACKEND_BASE=fake.base, GO2_ROBOT_ID="robot-a", GO2_AUTH_TOKEN="test-token"):
            fake.commands = [
                {"commandId": "cmd-ping", "action": "ping"},
                {"commandId": "cmd-move", "action": "move", "params": {"x": 1}},
            ]
            args = SimpleNamespace(
                robot_id=None,
                heartbeat_endpoint="/robot/heartbeat",
                result_endpoint="/robot/command/result",
                ros_timeout=0,
                post_timeout=3,
                execute_safe=False,
                dry_run_results=False,
            )
            rc = self.agent.cmd_command_poll_once(args)
            self.assertEqual(rc, 0)
            self.assertEqual(len(fake.heartbeats), 1)
            self.assertEqual(len(fake.results), 2)
            self.assertEqual(fake.results[0]["commandId"], "cmd-ping")
            self.assertEqual(fake.results[0]["status"], "success")
            self.assertEqual(fake.results[0]["result"]["command_id"], "cmd-ping")
            self.assertTrue(fake.results[0]["result"]["ok"])
            self.assertEqual(fake.results[1]["commandId"], "cmd-move")
            self.assertEqual(fake.results[1]["status"], "rejected")
            self.assertFalse(fake.results[1]["result"]["ok"])

    def test_plan_fetch_uses_device_token(self):
        with FakeGoGoGuard() as fake, EnvPatch(GO2_BACKEND_BASE=fake.base, GO2_DEVICE_TOKEN="device-token"):
            args = SimpleNamespace(endpoint="/devices/plan", dry_run=False, post_timeout=3)
            rc = self.agent.cmd_plan_fetch(args)
            self.assertEqual(rc, 0)
            self.assertEqual(fake.get_requests[-1]["path"], "/api/v1/devices/plan")
            self.assertEqual(fake.get_requests[-1]["headers"].get("X-Device-Token"), "device-token")

    def test_start_patrol_downloads_route_and_posts_success(self):
        with tempfile.TemporaryDirectory() as tmp, FakeGoGoGuard() as fake, EnvPatch(GO2_BACKEND_BASE=fake.base, GO2_ROBOT_ID="robot-a", GO2_AUTH_TOKEN="test-token", GO2_DEVICE_TOKEN="device-token"):
            routes_dir = Path(tmp) / "routes"
            self.agent.ROUTES_DIR = routes_dir
            shell_calls = []

            def fake_shell(cmd, timeout=3):
                shell_calls.append({"cmd": cmd, "timeout": timeout})
                return "PATROL_STARTED route=xiaoqu1.csv", "", 0

            self.agent.shell_out = fake_shell
            fake.commands = [{
                "commandId": "cmd-start",
                "action": "start_patrol",
                "params": {"fileName": "xiaoqu1.csv", "routeUrl": fake.base + "/routes/xiaoqu1.csv"},
            }]
            args = SimpleNamespace(
                robot_id=None,
                heartbeat_endpoint="/robot/heartbeat",
                result_endpoint="/robot/command/result",
                ros_timeout=0,
                post_timeout=3,
                execute_safe=True,
                dry_run_results=False,
            )
            rc = self.agent.cmd_command_poll_once(args)
            self.assertEqual(rc, 0)
            self.assertTrue((routes_dir / "xiaoqu1.csv").is_file())
            self.assertIn("waypoint_follower", shell_calls[-1]["cmd"][-1])
            self.assertEqual(fake.results[0]["commandId"], "cmd-start")
            self.assertEqual(fake.results[0]["status"], "success")
            self.assertEqual(fake.results[0]["detail"]["route"]["downloaded"], True)
            self.assertTrue(fake.results[0]["result"]["ok"])

    def test_start_patrol_waits_for_rosbag_database_and_tracks_process_groups(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            route = root / "route.csv"
            route.write_text(
                "id,x,y,yaw,v\n0,0,0,0,0.2\n1,1,0,0,0.2\n",
                encoding="utf-8",
            )
            self.agent.PATROL_RUNS_DIR = root / "runs"

            command = self.agent.start_patrol_command(
                route,
                {"speed": 0.5, "loopMode": "once"},
                route_info={},
            )

            self.assertIn("ROSBAG_LAUNCHED", command)
            self.assertIn("rosbag.pgid", command)
            self.assertIn("performance_monitor.pgid", command)
            self.assertIn("experiment_telemetry.pgid", command)
            self.assertIn("go2_experiment_snapshot.py", command)
            self.assertIn("go2_experiment_telemetry.py", command)
            self.assertGreaterEqual(
                command.count("nice -n 10"),
                3,
            )
            self.assertIn(
                "runtime_fastlio="
                "recover_positive_gap_without_permanent_latch",
                command,
            )
            self.assertIn(
                "observer_warmup_gate="
                "cpu_and_consecutive_fastlio_frames",
                command,
            )
            self.assertIn("follower_control_trace.jsonl", command)
            self.assertIn("trace_file:=", command)
            self.assertIn(
                "waypoint_follower_go2_2_trace.py",
                command,
            )
            self.assertIn(
                "FOLLOWER_EXACT_TRACE_READY",
                command,
            )
            self.assertIn("FOLLOWER_ODOM_READY", command)
            self.assertIn("FOLLOWER_BODY_YAW_READY", command)
            self.assertIn(
                "motion_enable_file:=",
                command,
            )
            self.assertIn(
                "use_body_yaw_alignment:=true",
                command,
            )
            self.assertIn(
                "body_yaw_topic:=/lf/sportmodestate",
                command,
            )
            self.assertIn(
                "body_yaw_alignment_samples:=10",
                command,
            )
            self.assertIn(
                "body_yaw_alignment_enabled=true",
                command,
            )
            self.assertIn(
                "controller_heading_feedback="
                "unitree_body_yaw_plus_startup_lio_yaw_offset",
                command,
            )
            self.assertIn(
                "body_yaw_stale_policy="
                "zero_velocity_hold_until_fresh",
                command,
            )
            self.assertIn(
                "FOLLOWER_MOTION_INTERLOCK_RELEASED",
                command,
            )
            self.assertLess(
                command.index("FOLLOWER_ODOM_READY"),
                command.index("--patrol-start-gate"),
            )
            self.assertLess(
                command.index("FOLLOWER_BODY_YAW_READY"),
                command.index("--patrol-start-gate"),
            )
            self.assertLess(
                command.index("--patrol-start-gate"),
                command.index(
                    "FOLLOWER_MOTION_INTERLOCK_RELEASED"
                ),
            )
            self.assertNotIn("course_feedback_enabled", command)
            self.assertIn('"kind":"recorder_ready"', command)
            self.assertIn("EXPERIMENT_TELEMETRY_READY", command)
            self.assertIn("PERFORMANCE_MONITOR_READY", command)
            self.assertIn("/livox/imu", command)
            self.assertIn("/lf/sportmodestate", command)
            self.assertIn(
                "for startup_attempt in $(seq 1 50)",
                command,
            )
            self.assertIn("sleep 0.200", command)
            self.assertIn("-name '*.db3'", command)
            self.assertIn('kill -INT -- "-$pgid"', command)
            self.assertIn("ROSBAG_READY attempt=$startup_attempt", command)
            self.assertNotIn(
                "ROSBAG_STARTED log=",
                command,
            )
            self.assertIn(
                "LOCALIZATION_SESSION_GUARD_LAUNCHED",
                command,
            )
            self.assertIn("localization_session_guard.pgid", command)
            self.assertIn("guard_ready=0", command)
            self.assertIn(
                "grep -q SESSION_GUARD_STARTED",
                command,
            )
            self.assertIn(
                "LOCALIZATION_SESSION_GUARD_READY "
                "attempt=$startup_attempt",
                command,
            )
            self.assertNotIn(
                "LOCALIZATION_SESSION_GUARD_STARTED log=",
                command,
            )

            stop_command = self.agent.stop_patrol_command()
            self.assertIn("go2_experiment_audit.py", stop_command)
            self.assertIn("system_end.json", stop_command)
            self.assertIn("localization_session_end.json", stop_command)
            self.assertIn("experiment_audit.log", stop_command)

    def test_body_yaw_alignment_has_explicit_rollback_switch(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            route = root / "route.csv"
            route.write_text(
                "id,x,y,yaw,v\n0,0,0,0,0.2\n1,1,0,0,0.2\n",
                encoding="utf-8",
            )
            self.agent.PATROL_RUNS_DIR = root / "runs"

            command = self.agent.start_patrol_command(
                route,
                {
                    "useBodyYawAlignment": False,
                    "loopMode": "once",
                },
                route_info={},
            )

            self.assertIn(
                "use_body_yaw_alignment:=false",
                command,
            )
            self.assertIn(
                "body_yaw_alignment_enabled=false",
                command,
            )
            self.assertIn(
                "controller_heading_feedback="
                "raw_fast_lio_euler_yaw",
                command,
            )
            self.assertNotIn("FOLLOWER_BODY_YAW_READY", command)

    def test_hash_locked_horizontal_route_replaces_body_yaw_adapter(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            route = root / "route.csv"
            route.write_text(
                "id,x,y,yaw,v\n0,4,7,0.1,0.2\n1,5,7,0.1,0.2\n",
                encoding="utf-8",
            )
            horizontal = Path(str(route) + ".horizontal.csv")
            horizontal.write_text(
                "id,x,y,yaw,v\n0,0,0,0.2,0.2\n1,1,0,0.2,0.2\n",
                encoding="utf-8",
            )
            metadata = Path(str(route) + ".horizontal.json")
            metadata.write_text(
                json.dumps(
                    {
                        "schema": "go2.horizontal_route.v1",
                        "source_route_sha256": (
                            self.agent.sha256_file(route)
                        ),
                        "horizontal_route_sha256": (
                            self.agent.sha256_file(horizontal)
                        ),
                        "route_points": 2,
                    }
                ),
                encoding="utf-8",
            )
            evidence = self.agent.horizontal_route_evidence(route)
            self.assertTrue(evidence["available"])
            alias = root / "platform-alias.csv"
            alias.write_bytes(route.read_bytes())
            stale_horizontal = Path(
                str(alias) + ".horizontal.csv"
            )
            stale_horizontal.write_text(
                "id,x,y,yaw,v\n0,0,0,0,0.2\n1,2,0,0,0.2\n",
                encoding="utf-8",
            )
            Path(str(alias) + ".horizontal.json").write_text(
                json.dumps(
                    {
                        "schema": "go2.horizontal_route.v1",
                        "source_route_sha256": "old-route-hash",
                        "horizontal_route_sha256": (
                            self.agent.sha256_file(stale_horizontal)
                        ),
                        "route_points": 2,
                    }
                ),
                encoding="utf-8",
            )
            alias_evidence = self.agent.horizontal_route_evidence(alias)
            self.assertTrue(alias_evidence["available"])
            self.assertTrue(alias_evidence["contentAlias"])
            self.assertEqual(
                alias_evidence["horizontalRoutePath"],
                str(horizontal),
            )
            self.agent.PATROL_RUNS_DIR = root / "runs"

            command = self.agent.start_patrol_command(
                route,
                {"loopMode": "once"},
                route_info={"horizontalRouteEvidence": evidence},
            )

            self.assertIn("use_horizontal_frame:=true", command)
            self.assertIn("use_body_yaw_alignment:=false", command)
            self.assertIn("FOLLOWER_HORIZONTAL_FRAME_READY", command)
            self.assertNotIn("FOLLOWER_BODY_YAW_READY", command)
            self.assertIn(
                "controller_heading_feedback="
                "full_fast_lio_quaternion_in_frozen_gravity_level_frame",
                command,
            )
            self.assertIn("horizontal_runtime_body_yaw_replacement=false", command)

    def test_start_failure_finalizes_only_its_own_evidence_directory(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            current_file = root / "current_patrol_run"
            allocated_run = root / "runs" / "attempt-1"
            self.agent.CURRENT_PATROL_RUN_FILE = current_file
            self.agent.prepare_route_csv = lambda params, dry_run=False: (
                root / "route.csv",
                {},
            )

            def fake_start_command(_route, _params, route_info=None):
                route_info["patrolRunDir"] = str(allocated_run)
                current_file.parent.mkdir(parents=True, exist_ok=True)
                current_file.write_text(str(allocated_run))
                return "exit 47"

            calls = []

            def fake_shell(command, timeout=3):
                calls.append((command, timeout))
                if len(calls) == 1:
                    return "", "ROUTE_FRAME_PREPARATION_FAILED", 47
                return "PATROL_STOPPED", "", 0

            self.agent.start_patrol_command = fake_start_command
            self.agent.shell_out = fake_shell

            status, message, detail = self.agent.run_start_patrol(
                {"fileName": "route.csv"},
                execute_safe=True,
            )

            self.assertEqual(status, "failed")
            self.assertEqual(len(calls), 2)
            self.assertEqual(calls[1][1], 120)
            self.assertIn(
                "START_FAILURE_EVIDENCE_FINALIZATION",
                message,
            )
            self.assertEqual(
                detail["failureFinalization"]["runDir"],
                str(allocated_run),
            )

    def test_duplicate_start_failure_does_not_stop_existing_patrol(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            current_file = root / "current_patrol_run"
            current_file.write_text(str(root / "existing-run"))
            self.agent.CURRENT_PATROL_RUN_FILE = current_file
            self.agent.prepare_route_csv = lambda params, dry_run=False: (
                root / "route.csv",
                {},
            )

            def fake_start_command(_route, _params, route_info=None):
                route_info["patrolRunDir"] = str(root / "new-attempt")
                return "exit 4"

            calls = []

            def fake_shell(command, timeout=3):
                calls.append((command, timeout))
                return "", "PATROL_ALREADY_RUNNING", 4

            self.agent.start_patrol_command = fake_start_command
            self.agent.shell_out = fake_shell

            status, _message, detail = self.agent.run_start_patrol(
                {"fileName": "route.csv"},
                execute_safe=True,
            )

            self.assertEqual(status, "failed")
            self.assertEqual(len(calls), 1)
            self.assertIsNone(detail["failureFinalization"])

    def test_recording_sidecar_must_match_downloaded_route_hash(self):
        with tempfile.TemporaryDirectory() as tmp:
            route = Path(tmp) / "route.csv"
            route.write_text(
                "id,x,y,yaw,v\n0,0,0,0,0.2\n1,1,0,0,0.2\n"
            )
            sidecar = Path(str(route) + ".recording.json")
            sidecar.write_text(
                json.dumps(
                    {
                        "route_sha256": self.agent.sha256_file(route),
                        "recording_run_dir": "/recordings/test",
                        "status": "complete",
                    }
                )
            )
            evidence = self.agent.route_recording_evidence(route)
            self.assertTrue(evidence["available"])

            route.write_text(
                "id,x,y,yaw,v\n0,0,0,0,0.2\n1,2,0,0,0.2\n"
            )
            with self.assertRaisesRegex(
                RuntimeError,
                "ROUTE_RECORDING_LINK_MISMATCH",
            ):
                self.agent.route_recording_evidence(route)

    def test_recording_sidecar_follows_byte_identical_platform_alias(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            recorded = root / "xbf9.csv"
            recorded.write_text(
                "id,x,y,yaw,v\n0,0,0,0,0.2\n1,1,0,0,0.2\n"
            )
            recorded_sha = self.agent.sha256_file(recorded)
            recorded_sidecar = Path(
                str(recorded) + ".recording.json"
            )
            recorded_sidecar.write_text(
                json.dumps(
                    {
                        "schema": "go2.route_recording_link.v1",
                        "route_sha256": recorded_sha,
                        "recording_run_dir": "/recordings/xbf9",
                        "status": "complete",
                        "same_fastlio_session_at_start_and_stop": True,
                    }
                )
            )

            alias = root / "xbf2.csv"
            alias.write_bytes(recorded.read_bytes())
            Path(str(alias) + ".recording.json").write_text(
                json.dumps(
                    {
                        "schema": "go2.route_recording_link.v1",
                        "route_sha256": "old-route-hash",
                        "recording_run_dir": "/recordings/old",
                        "status": "complete",
                    }
                )
            )

            evidence = self.agent.route_recording_evidence(alias)
            self.assertTrue(evidence["available"])
            self.assertTrue(evidence["contentAlias"])
            self.assertEqual(
                evidence["sidecarPath"],
                str(recorded_sidecar),
            )
            self.assertEqual(
                evidence["recordingRunDir"],
                "/recordings/xbf9",
            )

    def test_recording_sidecar_with_warnings_is_not_executable(self):
        with tempfile.TemporaryDirectory() as tmp:
            route = Path(tmp) / "route.csv"
            route.write_text(
                "id,x,y,yaw,v\n0,0,0,0,0.2\n1,1,0,0,0.2\n"
            )
            Path(str(route) + ".recording.json").write_text(
                json.dumps(
                    {
                        "route_sha256": self.agent.sha256_file(route),
                        "recording_run_dir": "/recordings/test",
                        "status": "complete_with_warnings",
                        "errors": ["end_snapshot:test"],
                    }
                )
            )

            with self.assertRaisesRegex(
                RuntimeError,
                "ROUTE_RECORDING_EVIDENCE_INCOMPLETE",
            ):
                self.agent.route_recording_evidence(route)

    def test_detached_ready_rejects_unsafe_shell_variable_name(self):
        with self.assertRaises(ValueError):
            self.agent.wait_for_detached_ready(
                "true",
                "/tmp/test.pgid",
                ready_var="bad; touch /tmp/not-allowed",
            )

    def test_video_upload_multipart_contains_pose(self):
        with tempfile.TemporaryDirectory() as tmp, FakeGoGoGuard() as fake, EnvPatch(GO2_BACKEND_BASE=fake.base, GO2_ROBOT_ID="robot-a", GO2_AUTH_TOKEN="test-token"):
            video_dir = Path(tmp)
            video = video_dir / "z1pro_20260706_010101_20s.mp4"
            video.write_bytes(b"fake-video")
            self.agent.VIDEO_DIR = video_dir
            self.agent.current_pose = lambda timeout_sec=0.8: dict(self.pose)
            args = SimpleNamespace(
                robot_id=None,
                heartbeat=False,
                heartbeat_endpoint="/robot/heartbeat",
                route="",
                pcd="",
                video="latest",
                route_endpoint="/robot/asset/upload",
                pcd_endpoint="/robot/asset/upload",
                video_endpoint="/robot/video/upload",
                patrol_id="patrol-test",
                upload=True,
                post_timeout=3,
                file_timeout=3,
                ros_timeout=0,
            )
            rc = self.agent.cmd_upload_once(args)
            self.assertEqual(rc, 0)
            self.assertEqual(len(fake.uploads), 1)
            body = fake.uploads[0]["body"]
            self.assertIn(b'name="robotId"', body)
            self.assertIn(b"robot-a", body)
            self.assertIn(b'name="position"', body)
            self.assertIn(b'name="pose"', body)
            self.assertIn(b'name="file"', body)

    def test_builtin_camera_media_is_discovered_like_z1pro_media(self):
        with tempfile.TemporaryDirectory() as tmp:
            video_dir = Path(tmp)
            z1pro = video_dir / "z1pro_20260723_010101_20s.mp4"
            builtin = video_dir / "unitree_builtin_20260723_010102_20s.mp4"
            z1pro.write_bytes(b"z1pro-video")
            builtin.write_bytes(b"builtin-video")
            self.agent.VIDEO_DIR = video_dir

            media = self.agent.valid_media_files(limit=10)

            self.assertEqual(set(media), {z1pro, builtin})

    def test_patrol_loop_runs_requested_cycles(self):
        calls = []
        self.agent.upload_patrol_assets = lambda args, cycle_index=0: calls.append(cycle_index) or 0
        self.agent.drain_outbox_once = lambda max_jobs=5: 0
        args = SimpleNamespace(cycles=3, run_file="", interval=0, upload=True, route="", pcd="", video="latest")
        rc = self.agent.cmd_patrol_loop(args)
        self.assertEqual(rc, 0)
        self.assertEqual(calls, [0, 1, 2])

    def test_command_loop_dedupes_seen_command_ids(self):
        with tempfile.TemporaryDirectory() as tmp, FakeGoGoGuard() as fake, EnvPatch(GO2_BACKEND_BASE=fake.base, GO2_ROBOT_ID="robot-a", GO2_AUTH_TOKEN="test-token"):
            fake.commands = [{"commandId": "cmd-repeat", "action": "ping"}]
            args = SimpleNamespace(
                robot_id=None,
                heartbeat_endpoint="/robot/heartbeat",
                result_endpoint="/robot/command/result",
                ros_timeout=0,
                post_timeout=3,
                execute_safe=False,
                dry_run_results=False,
                cycles=2,
                run_file="",
                interval=0,
                seen_file=str(Path(tmp) / "seen.json"),
            )
            rc = self.agent.cmd_command_loop(args)
            self.assertEqual(rc, 0)
            self.assertEqual(len(fake.heartbeats), 2)
            self.assertEqual(len(fake.results), 1)
            self.assertEqual(fake.results[0]["commandId"], "cmd-repeat")


if __name__ == "__main__":
    unittest.main(verbosity=2)

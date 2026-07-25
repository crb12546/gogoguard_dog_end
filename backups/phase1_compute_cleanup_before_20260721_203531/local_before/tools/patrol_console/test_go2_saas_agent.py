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
        self.route_csv = "id,x,y,z,yaw\n0,0,0,0,0\n1,1,0,0,0\n"
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

    def test_patrol_loop_runs_requested_cycles(self):
        calls = []
        self.agent.upload_patrol_assets = lambda args, cycle_index=0: calls.append(cycle_index) or 0
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

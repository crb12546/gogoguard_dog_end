import importlib.util
import subprocess
import time
import unittest
from pathlib import Path


MODULE_PATH = Path(__file__).resolve().parent / "server.py"
INDEX_PATH = Path(__file__).resolve().parent / "static/index.html"
SPEC = importlib.util.spec_from_file_location("patrol_console_server", MODULE_PATH)
SERVER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(SERVER)


class PatrolConsoleAnchorPathsTest(unittest.TestCase):
    def assert_shell_syntax(self, command):
        result = subprocess.run(
            ["bash", "-n", "-c", command],
            capture_output=True,
            text=True,
        )
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_route_recording_locks_localization_session(self):
        command = SERVER.act_start_recorder(
            {"route_name": "anchor_test", "overwrite": True}
        )
        self.assertIn("route_recording_blackbox.py start", command)
        self.assertIn("--route-file", command)
        self.assertIn("--min-distance 0.40", command)
        self.assertIn("--overwrite", command)
        self.assert_shell_syntax(command)

    def test_pcd_recording_locks_localization_session(self):
        command = SERVER.act_start_pcd({"name": "anchor_test"})
        self.assertIn("manual_route_anchor.py --capture-only", command)
        self.assertIn("localization_session_guard.py", command)
        self.assertIn("--mode pcd", command)
        self.assertIn("--invalidate-output", command)
        self.assertIn("go2_horizontal_frame_calibration.json", command)
        self.assertIn("--raw-output", command)
        self.assertIn("/maps/console/raw/anchor_test.pcd", command)
        self.assertIn(
            "/maps/console/anchor_test.leveling.json", command
        )
        self.assertIn("HorizontalFrameEstimator", command)
        self.assertIn("SESSION_GUARD_NOT_READY", command)
        self.assertLess(
            command.index("SESSION_GUARD_STARTED"),
            command.index("PCD_CAPTURE_STARTED"),
        )
        self.assert_shell_syntax(command)

    def test_console_patrol_uses_full_saas_patrol_chain(self):
        route = SERVER.ROUTES_DIR + "/anchor_test.csv"
        command = SERVER.act_start_follower(
            {
                "route_path": route,
                "v_base": 0.2,
                "loop_mode": "once",
            }
        )
        self.assertIn("go2_saas_agent.py patrol-start", command)
        self.assertIn("--route-name anchor_test.csv", command)
        self.assertIn("--speed 0.200", command)
        self.assertIn("--loop-mode once", command)
        self.assertIn("--localization-mode manual_anchor", command)
        self.assertIn("/tmp/go2_saas_follower.log", command)
        self.assertIn("/tmp/go2_saas_safe.log", command)
        self.assert_shell_syntax(command)

    def test_console_requires_unambiguous_patrol_result_markers(self):
        html = INDEX_PATH.read_text()
        self.assertIn(
            "PATROL_CLI_STARTED\\b|PATROL_STARTED route=",
            html,
        )
        self.assertIn(
            "PATROL_CLI_STOPPED\\b|PATROL_STOPPED\\b",
            html,
        )
        self.assertIn(
            "ROUTE_RECORDING_EVIDENCE_INCOMPLETE",
            html,
        )

    def test_console_startup_status_is_neutral_and_recording_timeout_covers_gates(self):
        html = INDEX_PATH.read_text()
        source = MODULE_PATH.read_text()
        self.assertIn("const statusProbing = !s.status_ready", html)
        self.assertIn("正在检测机器狗", html)
        self.assertIn('"start_recorder": 320', source)

    def test_telemetry_stream_age_uses_local_monotonic_receipt_time(self):
        with SERVER.LOCK:
            old_received_at = SERVER.STATE["telemetry_received_at"]
            old_age = SERVER.STATE["telemetry_age"]
            try:
                SERVER.STATE["telemetry_received_at"] = time.monotonic() - 0.5
                SERVER._refresh_telemetry_age_locked()
                self.assertGreaterEqual(SERVER.STATE["telemetry_age"], 0.4)
                self.assertLessEqual(SERVER.STATE["telemetry_age"], 0.7)
            finally:
                SERVER.STATE["telemetry_received_at"] = old_received_at
                SERVER.STATE["telemetry_age"] = old_age

    def test_base_restart_is_blocked_by_active_operations(self):
        for command in (SERVER.act_start_base({}), SERVER.act_stop_base({})):
            self.assertIn("BASE_RESTART_BLOCKED_ACTIVE_MODE", command)
            self.assertIn("[l]ocalization_session_guard.py", command)
            self.assertIn("[r]oute_recorder", command)
            self.assertIn("[w]aypoint_follower", command)
            self.assert_shell_syntax(command)

    def test_session_guard_probe_does_not_match_enclosing_shell(self):
        token = "go2_console_probe_shell_only_token"
        pattern = "[g]" + token[1:]
        command = (
            ": %s; if %s; then exit 9; else exit 0; fi"
            % (token, SERVER._non_shell_process_probe(pattern))
        )
        result = subprocess.run(["bash", "-c", command])
        self.assertEqual(result.returncode, 0)


if __name__ == "__main__":
    unittest.main()

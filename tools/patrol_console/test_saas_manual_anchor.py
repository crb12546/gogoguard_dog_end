import importlib.util
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
MODULE_PATH = ROOT / "orin_go2_fastlio_ws/scripts/go2_saas_agent.py"
SPEC = importlib.util.spec_from_file_location("go2_saas_anchor_test", MODULE_PATH)
AGENT = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(AGENT)


class SaasManualAnchorTest(unittest.TestCase):
    def test_process_probe_ignores_shell_wrapper_but_finds_python(self):
        shell_token = "go2_probe_shell_only_token"
        shell_pattern = "[g]" + shell_token[1:]
        shell_command = (
            ": %s; if %s; then exit 9; else exit 0; fi"
            % (shell_token, AGENT.non_shell_process_probe(shell_pattern))
        )
        shell_result = subprocess.run(["bash", "-c", shell_command])
        self.assertEqual(shell_result.returncode, 0)
        shell_ids = AGENT.non_shell_process_ids(shell_pattern)
        ids_result = subprocess.run(
            [
                "bash",
                "-c",
                ": %s; ids=$(%s || true); test -z \"$ids\""
                % (shell_token, shell_ids),
            ]
        )
        self.assertEqual(ids_result.returncode, 0)

        process_token = "go2_probe_python_process_token"
        process_pattern = "[g]" + process_token[1:]
        child = subprocess.Popen(
            [
                sys.executable,
                "-c",
                "import time; time.sleep(5)",
                process_token,
            ]
        )
        try:
            time.sleep(0.1)
            process_result = subprocess.run(
                ["bash", "-c", AGENT.non_shell_process_probe(process_pattern)]
            )
            self.assertEqual(process_result.returncode, 0)
            matching_ids = subprocess.run(
                ["bash", "-c", AGENT.non_shell_process_ids(process_pattern)],
                capture_output=True,
                text=True,
            )
            self.assertEqual(matching_ids.returncode, 0)
            self.assertIn(str(child.pid), matching_ids.stdout.split())
        finally:
            child.terminate()
            child.wait(timeout=2)

    def test_origin_compatibility_aliases_use_manual_anchor(self):
        for params in (
            {},
            {"localizationMode": "manual_anchor"},
            {"localizationMode": "origin"},
            {"localizationMode": "direct"},
            {"relocalize": False},
        ):
            self.assertEqual(
                AGENT.localization_mode_from_params(params),
                "manual_anchor",
            )
        self.assertEqual(
            AGENT.localization_mode_from_params({"localizationMode": "pcd"}),
            "pcd",
        )

    def test_start_patrol_prepares_anchor_after_posture_and_before_follower(self):
        with tempfile.TemporaryDirectory() as temporary:
            old_runs = AGENT.PATROL_RUNS_DIR
            old_video = AGENT.VIDEO_RUN_DIR
            old_current = AGENT.CURRENT_PATROL_RUN_FILE
            old_service_video = AGENT.SERVICE_VIDEO_RUN_FILE
            old_patrol_video = AGENT.PATROL_VIDEO_ACTIVE_FILE
            try:
                AGENT.PATROL_RUNS_DIR = Path(temporary) / "runs"
                AGENT.VIDEO_RUN_DIR = Path(temporary) / "run"
                AGENT.CURRENT_PATROL_RUN_FILE = (
                    AGENT.VIDEO_RUN_DIR / "current_patrol_run"
                )
                AGENT.SERVICE_VIDEO_RUN_FILE = AGENT.VIDEO_RUN_DIR / "video.run"
                AGENT.PATROL_VIDEO_ACTIVE_FILE = (
                    AGENT.VIDEO_RUN_DIR / "patrol_video.active"
                )
                route_info = {}
                command = AGENT.start_patrol_command(
                    Path("/tmp/source.csv"),
                    {"localizationMode": "origin"},
                    route_info,
                )
            finally:
                AGENT.PATROL_RUNS_DIR = old_runs
                AGENT.VIDEO_RUN_DIR = old_video
                AGENT.CURRENT_PATROL_RUN_FILE = old_current
                AGENT.SERVICE_VIDEO_RUN_FILE = old_service_video
                AGENT.PATROL_VIDEO_ACTIVE_FILE = old_patrol_video

        self.assertEqual(route_info["localizationMode"], "manual_anchor")
        self.assertIn("route_runtime.csv", command)
        self.assertIn("manual_anchor.json", command)
        self.assertIn("manual_route_anchor.py", command)
        self.assertIn("localization_session_guard.py", command)
        self.assertIn("FOLLOWER_EXACT_TRACE_READY", command)
        self.assertLess(
            command.index("SDK_RECEIVER_STARTED"),
            command.index("manual_route_anchor.py"),
        )
        self.assertLess(
            command.index("manual_route_anchor.py"),
            command.index("localization_session_guard.py"),
        )
        self.assertLess(
            command.index("localization_session_guard.py"),
            command.index("FOLLOWER_EXACT_TRACE_READY"),
        )
        self.assertLess(
            command.index("FOLLOWER_ODOM_READY"),
            command.index("--patrol-start-gate"),
        )
        self.assertLess(
            command.index("--patrol-start-gate"),
            command.index("FOLLOWER_MOTION_INTERLOCK_RELEASED"),
        )
        syntax = subprocess.run(
            ["bash", "-n", "-c", command],
            capture_output=True,
            text=True,
        )
        self.assertEqual(syntax.returncode, 0, syntax.stderr)

    def test_saas_base_restart_has_active_mode_interlock(self):
        command = AGENT.start_base_command()
        self.assertIn("BASE_RESTART_BLOCKED_ACTIVE_MODE", command)
        self.assertIn("[l]ocalization_session_guard.py", command)
        self.assertIn("[r]oute_recorder", command)
        self.assertIn("[w]aypoint_follower", command)


if __name__ == "__main__":
    unittest.main()

from __future__ import annotations

import importlib.util
import json
import os
import sys
import tempfile
import time
import types
import unittest
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parents[1] / "scripts"
SPEC = importlib.util.spec_from_file_location(
    "gogoguard_xbf9_r5_agent",
    SCRIPT_DIR / "gogoguard_xbf9_r5_agent.py",
)
assert SPEC is not None and SPEC.loader is not None
bridge = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = bridge
SPEC.loader.exec_module(bridge)


class FakeProcess:
    def __init__(self, pid: int = 424242, return_code=None) -> None:
        self.pid = pid
        self.return_code = return_code

    def poll(self):
        return self.return_code


class FakeCompleted:
    def __init__(self, return_code: int = 0) -> None:
        self.returncode = return_code
        self.stdout = "stopped"
        self.stderr = ""


class FixedBridgeTests(unittest.TestCase):
    def make_root(self, temporary: str):
        root = Path(temporary) / "localization_upgrade"
        scripts = root / "scripts"
        scripts.mkdir(parents=True)
        for name in ("start_xbf_patrol.sh", "stop_xbf_patrol.sh"):
            path = scripts / name
            path.write_text("#!/usr/bin/env bash\nexit 0\n", encoding="utf-8")
            path.chmod(0o755)
        (root / "TASK_BUILD_REPORT.json").write_text(
            json.dumps(
                {
                    "task_id": bridge.TASK_ID,
                    "compiled_runtime": {
                        "map_id": bridge.TASK_ID,
                        "aligned_route_sha256": "a" * 64,
                        "checkpoint_sha256": "b" * 64,
                    },
                }
            ),
            encoding="utf-8",
        )
        return root

    def test_dry_run_ignores_platform_route_without_launch(self):
        with tempfile.TemporaryDirectory() as temporary:
            launched = []
            manager = bridge.FixedTaskManager(
                bundle_root=self.make_root(temporary),
                runtime_dir=Path(temporary) / "runtime",
                popen_factory=lambda *args, **kwargs: launched.append(
                    (args, kwargs)
                ),
                start_reaper=False,
            )
            status, _, detail = manager.start(
                {
                    "fileName": "wrong.csv",
                    "routeUrl": "https://example.invalid/wrong.csv",
                    "speed": 0.8,
                },
                execute_safe=False,
            )
            self.assertEqual(status, "success")
            self.assertEqual(launched, [])
            self.assertTrue(detail["platformRouteIgnored"])
            self.assertEqual(
                detail["ignoredPlatformRouteKeys"], ["fileName", "routeUrl"]
            )
            self.assertEqual(detail["fixedSpeedMps"], 0.20)

    def test_real_start_uses_only_fixed_bundle_and_returns_running(self):
        with tempfile.TemporaryDirectory() as temporary:
            calls = []

            def fake_popen(command, **kwargs):
                calls.append((command, kwargs))
                return FakeProcess()

            root = self.make_root(temporary).resolve()
            manager = bridge.FixedTaskManager(
                bundle_root=root,
                runtime_dir=Path(temporary) / "runtime",
                popen_factory=fake_popen,
                sleep=lambda _: None,
                start_reaper=False,
            )
            status, _, detail = manager.start(
                {
                    "routeUrl": "https://example.invalid/never-downloaded.csv",
                    "fileName": "never-used.csv",
                    "speed": 0.8,
                },
                execute_safe=True,
            )
            self.assertEqual(status, "running")
            self.assertEqual(len(calls), 1)
            command, kwargs = calls[0]
            self.assertEqual(
                command, ["/bin/bash", str(root / "scripts/start_xbf_patrol.sh")]
            )
            self.assertNotIn("example.invalid", " ".join(command))
            self.assertEqual(
                kwargs["env"]["GO2_XBF_PATROL_SPEED"],
                bridge.FIXED_SPEED_MPS,
            )
            self.assertEqual(
                kwargs["env"]["GO2_XBF_LOOP_MODE"], bridge.FIXED_LOOP_MODE
            )
            self.assertTrue(detail["platformRouteIgnored"])

    def test_immediate_start_failure_is_reported(self):
        with tempfile.TemporaryDirectory() as temporary:
            manager = bridge.FixedTaskManager(
                bundle_root=self.make_root(temporary),
                runtime_dir=Path(temporary) / "runtime",
                popen_factory=lambda *args, **kwargs: FakeProcess(
                    return_code=17
                ),
                sleep=lambda _: None,
                start_reaper=False,
            )
            status, _, detail = manager.start({}, execute_safe=True)
            self.assertEqual(status, "failed")
            self.assertEqual(detail["rc"], 17)

    def test_stop_uses_fixed_stop_script(self):
        with tempfile.TemporaryDirectory() as temporary:
            calls = []

            def fake_run(command, **kwargs):
                calls.append((command, kwargs))
                return FakeCompleted()

            root = self.make_root(temporary).resolve()
            manager = bridge.FixedTaskManager(
                bundle_root=root,
                runtime_dir=Path(temporary) / "runtime",
                run_factory=fake_run,
                start_reaper=False,
            )
            status, _, detail = manager.stop(
                {"routeUrl": "https://example.invalid/ignored.csv"},
                execute_safe=True,
            )
            self.assertEqual(status, "success")
            self.assertEqual(
                calls[0][0],
                ["/bin/bash", str(root / "scripts/stop_xbf_patrol.sh")],
            )
            self.assertTrue(detail["platformRouteIgnored"])

    def test_hooks_replace_only_start_and_stop_handlers(self):
        calls = []

        class FakeManager:
            def start(self, params, execute_safe=False):
                calls.append(("start", params, execute_safe))
                return "running", "start", {"action": "start_patrol"}

            def stop(self, params, execute_safe=False):
                calls.append(("stop", params, execute_safe))
                return "success", "stop", {"action": "stop_patrol"}

        module = types.SimpleNamespace(
            run_start_patrol=lambda *_args, **_kwargs: None,
            run_stop_patrol=lambda *_args, **_kwargs: None,
            run_safe_command=lambda *_args, **_kwargs: None,
            main=lambda _argv: 0,
        )
        original_safe_command = module.run_safe_command
        bridge.install_fixed_handlers(module, FakeManager())
        self.assertIs(module.run_safe_command, original_safe_command)
        self.assertEqual(
            module.run_start_patrol({"routeUrl": "ignored"}, True)[0],
            "running",
        )
        self.assertEqual(module.run_stop_patrol({}, True)[0], "success")
        self.assertEqual(calls[0][0], "start")
        self.assertEqual(calls[1][0], "stop")

    def test_real_base_agent_dispatches_every_start_alias_to_fixed_handler(self):
        configured_path = os.environ.get("GO2_SAAS_TEST_BASE_AGENT")
        if configured_path:
            base_agent_path = Path(configured_path)
        else:
            base_agent_path = None
            for parent in Path(__file__).resolve().parents:
                candidate = (
                    parent
                    / "realtime_dog_end_code/scripts/go2_saas_agent.py"
                )
                if candidate.is_file():
                    base_agent_path = candidate
                    break
            if base_agent_path is None:
                self.skipTest("real go2_saas_agent.py is not mounted")
        base_agent = bridge.load_base_agent(base_agent_path)
        calls = []

        class FakeManager:
            bundle_root = Path("/fixed-xbf9-r5")

            def start(self, params, execute_safe=False):
                calls.append(("start", params, execute_safe))
                return (
                    "running" if execute_safe else "success",
                    "fixed",
                    {
                        "action": "start_patrol",
                        "taskId": bridge.TASK_ID,
                        "platformRouteIgnored": True,
                    },
                )

            def stop(self, params, execute_safe=False):
                calls.append(("stop", params, execute_safe))
                return (
                    "success",
                    "fixed",
                    {
                        "action": "stop_patrol",
                        "taskId": bridge.TASK_ID,
                        "platformRouteIgnored": True,
                    },
                )

        bridge.install_fixed_handlers(base_agent, FakeManager())
        for action in sorted(base_agent.START_PATROL_COMMANDS):
            with self.subTest(action=action):
                status, _, detail = base_agent.run_safe_command(
                    action,
                    {
                        "routeUrl": (
                            "https://example.invalid/must-not-download.csv"
                        ),
                        "fileName": "must-not-use.csv",
                    },
                    execute_safe=True,
                )
                self.assertEqual(status, "running")
                self.assertEqual(detail["action"], "start_patrol")
                self.assertEqual(calls[-1][0], "start")
                self.assertEqual(calls[-1][1]["fileName"], "must-not-use.csv")
        for action in sorted(base_agent.STOP_PATROL_COMMANDS):
            with self.subTest(action=action):
                status, _, detail = base_agent.run_safe_command(
                    action,
                    {"routeUrl": "https://example.invalid/ignored.csv"},
                    execute_safe=True,
                )
                self.assertEqual(status, "success")
                self.assertEqual(detail["action"], "stop_patrol")
                self.assertEqual(calls[-1][0], "stop")
        ping_status, _, _ = base_agent.run_safe_command(
            "ping", {}, execute_safe=True
        )
        self.assertEqual(ping_status, "success")
        self.assertEqual(
            bridge.bridge_self_check(base_agent, FakeManager())["task_id"],
            bridge.TASK_ID,
        )

    @unittest.skipUnless(Path("/proc/self/stat").is_file(), "Linux /proc required")
    def test_stop_covers_real_pre_pidfile_startup_window(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = self.make_root(temporary).resolve()
            start_script = root / "scripts/start_xbf_patrol.sh"
            start_script.write_text(
                "#!/usr/bin/env bash\n"
                "trap 'exit 0' INT TERM\n"
                "while true; do sleep 1; done\n",
                encoding="utf-8",
            )
            start_script.chmod(0o755)

            manager = bridge.FixedTaskManager(
                bundle_root=root,
                runtime_dir=Path(temporary) / "runtime",
                run_factory=lambda *args, **kwargs: FakeCompleted(),
                start_reaper=True,
            )
            status, _, detail = manager.start({}, execute_safe=True)
            self.assertEqual(status, "running")
            pid = int(detail["supervisorPid"])
            self.assertTrue(bridge._pid_alive(pid))
            self.assertFalse(manager.pid_file.exists())

            stop_status, _, stop_detail = manager.stop({}, execute_safe=True)
            self.assertEqual(stop_status, "success")
            self.assertTrue(stop_detail["stoppedPrePidfileLaunch"])
            deadline = time.monotonic() + 3.0
            while bridge._pid_alive(pid) and time.monotonic() < deadline:
                time.sleep(0.05)
            self.assertFalse(bridge._pid_alive(pid))


if __name__ == "__main__":
    unittest.main()

#!/usr/bin/env python3
import importlib.util
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace


ROOT = Path(__file__).resolve().parents[2]
SCRIPTS = ROOT / "orin_go2_fastlio_ws/scripts"


def load_guard():
    sys.path.insert(0, str(SCRIPTS))
    try:
        path = SCRIPTS / "localization_session_guard.py"
        spec = importlib.util.spec_from_file_location(
            "localization_session_guard_tested",
            str(path),
        )
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module
    finally:
        sys.path.remove(str(SCRIPTS))


class LocalizationSessionGuardTest(unittest.TestCase):
    def test_abort_keeps_run_pointer_for_evidence_finalization(self):
        guard = load_guard()
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            run_dir = root / "run"
            run_dir.mkdir()
            pointer = root / "patrol_logs/run/current_patrol_run"
            pointer.parent.mkdir(parents=True)
            pointer.write_text(str(run_dir))
            active = run_dir / ".patrol_active"
            active.touch()
            manifest = run_dir / "manifest.txt"
            event_log = run_dir / "session_guard.log"

            guard.WS = str(root)
            guard.stop_patrol = lambda enable_file="": SimpleNamespace(
                returncode=0
            )
            guard.invalidate_output = lambda path, reason: ""
            args = SimpleNamespace(
                active_file=str(active),
                enable_file="",
                mode="patrol",
                invalidate_output="",
                event_log=str(event_log),
                manifest=str(manifest),
            )

            guard.abort_operation(args, "test-session-change")

            self.assertFalse(active.exists())
            self.assertTrue(pointer.exists())
            self.assertEqual(pointer.read_text(), str(run_dir))
            self.assertIn(
                "abort_reason=localization_session_changed",
                manifest.read_text(),
            )


if __name__ == "__main__":
    unittest.main(verbosity=2)

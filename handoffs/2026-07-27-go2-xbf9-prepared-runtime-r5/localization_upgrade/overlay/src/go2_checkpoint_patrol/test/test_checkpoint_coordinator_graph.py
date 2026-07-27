import ast
import unittest
from pathlib import Path
from typing import Optional


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
COORDINATOR_PATH = (
    PACKAGE_ROOT
    / "go2_checkpoint_patrol"
    / "checkpoint_coordinator.py"
)
PRODUCTION_CONFIG_PATH = (
    PACKAGE_ROOT
    / "config"
    / "checkpoint-coordinator.production.yaml"
)


def load_graph_mismatch_function():
    source = COORDINATOR_PATH.read_text(encoding="utf-8")
    tree = ast.parse(source, filename=str(COORDINATOR_PATH))
    function = next(
        node
        for node in tree.body
        if isinstance(node, ast.FunctionDef)
        and node.name == "_control_graph_mismatch_reason"
    )
    module = ast.Module(body=[function], type_ignores=[])
    namespace = {"Optional": Optional}
    exec(compile(module, str(COORDINATOR_PATH), "exec"), namespace)
    return namespace["_control_graph_mismatch_reason"]


class ControlGraphContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.reason = staticmethod(load_graph_mismatch_function())

    def counts(self, **overrides):
        values = {
            "gated_cmd_topic": "/checkpoint_localization/gated_cmd",
            "gated_publishers": 1,
            "actuator_command_topic": "/cmd_vel",
            "actuator_publishers": 1,
            "actuator_subscribers": 1,
            "legacy_cmd_topic": "/patrol_cmd",
            "legacy_publishers": 0,
        }
        values.update(overrides)
        return values

    def test_exact_one_to_one_command_graph_is_accepted(self):
        self.assertIsNone(self.reason(**self.counts()))

    def test_gated_command_requires_exactly_one_publisher(self):
        reason = self.reason(**self.counts(gated_publishers=2))
        self.assertIn("/checkpoint_localization/gated_cmd", reason)
        self.assertIn("2 publishers, expected exactly 1", reason)

    def test_cmd_vel_requires_exactly_one_safe_publisher(self):
        reason = self.reason(**self.counts(actuator_publishers=0))
        self.assertIn("/cmd_vel", reason)
        self.assertIn("0 publishers, expected exactly 1", reason)

    def test_cmd_vel_requires_a_consumer_but_allows_observers(self):
        reason = self.reason(**self.counts(actuator_subscribers=0))
        self.assertIn("/cmd_vel", reason)
        self.assertIn("0 subscribers, expected at least 1", reason)
        self.assertIsNone(
            self.reason(**self.counts(actuator_subscribers=2))
        )

    def test_legacy_patrol_command_must_have_no_publishers(self):
        reason = self.reason(**self.counts(legacy_publishers=1))
        self.assertIn("legacy /patrol_cmd", reason)
        self.assertIn("1 publishers, expected exactly 0", reason)

    def test_production_parameter_uses_cmd_vel_not_sport_request(self):
        coordinator_source = COORDINATOR_PATH.read_text(encoding="utf-8")
        production_config = PRODUCTION_CONFIG_PATH.read_text(encoding="utf-8")
        self.assertIn(
            'declare_parameter("actuator_command_topic", "/cmd_vel")',
            coordinator_source,
        )
        self.assertIn("actuator_command_topic: /cmd_vel", production_config)
        self.assertNotIn("actuator_request_topic", coordinator_source)
        self.assertNotIn("/api/sport/request", production_config)

    def test_runtime_guard_counts_cmd_vel_subscribers(self):
        coordinator_source = COORDINATOR_PATH.read_text(encoding="utf-8")
        tree = ast.parse(coordinator_source, filename=str(COORDINATOR_PATH))
        coordinator_class = next(
            node
            for node in tree.body
            if isinstance(node, ast.ClassDef)
            and node.name == "CheckpointLocalizationCoordinator"
        )
        guard = next(
            node
            for node in coordinator_class.body
            if isinstance(node, ast.FunctionDef)
            and node.name == "_graph_guard_reason"
        )
        calls = [
            node
            for node in ast.walk(guard)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
        ]
        self.assertTrue(
            any(call.func.attr == "count_subscribers" for call in calls)
        )


if __name__ == "__main__":
    unittest.main()

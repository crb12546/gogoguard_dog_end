import importlib.util
import math
import sys
import types
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock


ROOT = Path(__file__).resolve().parents[2]
MODULE_PATH = (
    ROOT
    / "orin_go2_fastlio_ws"
    / "scripts"
    / "go2_base_health_watchdog.py"
)


def load_watchdog_module():
    fake_rclpy = types.ModuleType("rclpy")
    fake_rclpy_node = types.ModuleType("rclpy.node")
    fake_nav_msgs = types.ModuleType("nav_msgs")
    fake_nav_msgs_msg = types.ModuleType("nav_msgs.msg")

    class DummyNode:
        pass

    class DummyOdometry:
        pass

    fake_rclpy_node.Node = DummyNode
    fake_nav_msgs_msg.Odometry = DummyOdometry
    fake_nav_msgs.msg = fake_nav_msgs_msg

    spec = importlib.util.spec_from_file_location(
        "go2_base_health_watchdog_tested",
        MODULE_PATH,
    )
    module = importlib.util.module_from_spec(spec)
    fake_modules = {
        "rclpy": fake_rclpy,
        "rclpy.node": fake_rclpy_node,
        "nav_msgs": fake_nav_msgs,
        "nav_msgs.msg": fake_nav_msgs_msg,
    }
    with mock.patch.dict(sys.modules, fake_modules):
        spec.loader.exec_module(module)
    return module


WATCHDOG = load_watchdog_module()


def policy_args(monitor_z=False):
    return SimpleNamespace(
        monitor_z=monitor_z,
        max_abs_xy=100.0,
        max_abs_z=5.0,
        max_jump_distance=0.80,
        max_jump_speed=0.60,
        max_jump_z=0.50,
        max_jump_dt=2.0,
    )


class BaseHealthWatchdogPolicyTests(unittest.TestCase):
    def validate(
        self,
        *,
        monitor_z=False,
        last_pos=(0.0, 0.0, 0.0),
        last_time=10.0,
        now=10.1,
        position=(0.05, 0.02, 10.0),
    ):
        return WATCHDOG.validate_odom_sample(
            policy_args(monitor_z),
            last_pos,
            last_time,
            now,
            *position,
        )

    def test_planar_mode_ignores_finite_z_absolute_value_and_jump(self):
        self.assertEqual(self.validate(), "")

    def test_planar_mode_still_rejects_horizontal_jump(self):
        reason = self.validate(position=(2.0, 0.0, 10.0))
        self.assertTrue(reason.startswith("odom_horizontal_jump:"))

    def test_planar_mode_still_rejects_non_finite_z(self):
        reason = self.validate(position=(0.05, 0.02, math.nan))
        self.assertTrue(reason.startswith("non_finite_odom:"))

    def test_five_kilometre_default_accepts_long_route_coordinates(self):
        args = policy_args()
        args.max_abs_xy = WATCHDOG.DEFAULT_MAX_ABS_XY_M
        reason = WATCHDOG.validate_odom_sample(
            args,
            (339.0, -300.0, 150.0),
            10.0,
            10.1,
            339.05,
            -300.02,
            150.1,
        )
        self.assertEqual(WATCHDOG.DEFAULT_MAX_ABS_XY_M, 5000.0)
        self.assertEqual(reason, "")

    def test_3d_mode_rejects_z_absolute_value(self):
        reason = self.validate(monitor_z=True)
        self.assertTrue(reason.startswith("odom_z_abs_out_of_range:"))

    def test_3d_mode_rejects_z_jump(self):
        reason = self.validate(
            monitor_z=True,
            position=(0.05, 0.02, 1.0),
        )
        self.assertTrue(reason.startswith("odom_z_jump:"))


if __name__ == "__main__":
    unittest.main()

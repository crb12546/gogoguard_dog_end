#!/usr/bin/env python3
import math
import unittest

from go2_fastlio_patrol.patrol_control import (
    lateral_velocity_command,
    limit_planar_command,
    point_in_lateral_motion_roi,
)


class PatrolControlTest(unittest.TestCase):
    def test_left_route_error_commands_right_body_velocity(self):
        vy = lateral_velocity_command(
            lateral_error=0.20,
            route_yaw=0.0,
            current_yaw=0.0,
        )

        self.assertLess(vy, 0.0)
        self.assertAlmostEqual(vy, -0.085, places=6)

    def test_lateral_velocity_respects_deadband_and_limit(self):
        self.assertEqual(
            lateral_velocity_command(0.02, 0.0, 0.0),
            0.0,
        )
        self.assertAlmostEqual(
            lateral_velocity_command(1.0, 0.0, 0.0),
            -0.12,
        )

    def test_lateral_velocity_is_disabled_during_large_heading_error(self):
        vy = lateral_velocity_command(
            0.20,
            route_yaw=math.radians(90.0),
            current_yaw=0.0,
        )

        self.assertEqual(vy, 0.0)

    def test_safety_override_zeros_all_three_axes(self):
        command = limit_planar_command(
            0.5,
            -0.2,
            0.4,
            max_vx=0.4,
            max_vy=0.12,
            max_yaw_rate=0.3,
            enabled=False,
        )

        self.assertEqual(command, (0.0, 0.0, 0.0))

    def test_planar_command_clamps_vy_independently(self):
        command = limit_planar_command(
            0.5,
            -0.2,
            0.4,
            max_vx=0.4,
            max_vy=0.12,
            max_yaw_rate=0.3,
        )

        self.assertEqual(command, (0.4, -0.12, 0.3))

    def test_lateral_obstacle_roi_follows_commanded_side(self):
        self.assertTrue(point_in_lateral_motion_roi(0.5, 0.4, 0.5, vy=0.1))
        self.assertFalse(point_in_lateral_motion_roi(0.5, -0.4, 0.5, vy=0.1))
        self.assertTrue(point_in_lateral_motion_roi(0.5, -0.4, 0.5, vy=-0.1))
        self.assertFalse(point_in_lateral_motion_roi(0.5, 0.4, 0.5, vy=-0.1))

    def test_lateral_obstacle_roi_is_off_without_lateral_motion(self):
        self.assertFalse(point_in_lateral_motion_roi(0.5, 0.4, 0.5, vy=0.0))
        self.assertFalse(point_in_lateral_motion_roi(0.5, 0.4, 0.5, vy=0.02))

    def test_lateral_obstacle_roi_excludes_body_and_out_of_height(self):
        self.assertFalse(point_in_lateral_motion_roi(0.5, 0.2, 0.5, vy=0.1))
        self.assertFalse(point_in_lateral_motion_roi(0.5, 0.4, 1.0, vy=0.1))


if __name__ == '__main__':
    unittest.main()

#!/usr/bin/env python3
import math
import unittest

from go2_fastlio_patrol.patrol_control import (
    corner_heading_command,
    cumulative_turn_candidate,
    lateral_pid_command,
    lateral_velocity_command,
    lateral_velocity_limit,
    limit_planar_command,
    point_in_lateral_motion_roi,
)


class PatrolControlTest(unittest.TestCase):
    def test_corner_heading_keeps_turning_during_target_conflict(self):
        command = corner_heading_command(
            target_alpha=math.radians(2.9),
            outgoing_heading_error=math.radians(-57.3),
            normal_blend=0.30,
            conflict_blend=0.65,
        )

        self.assertLess(math.degrees(command), -30.0)

    def test_cumulative_curve_detects_same_direction_arc(self):
        candidate = cumulative_turn_candidate(
            [
                (142.2, 1.2, math.radians(3.5)),
                (143.6, 2.6, math.radians(9.1)),
                (144.4, 3.4, math.radians(13.9)),
            ],
            threshold=math.radians(20.0),
        )

        self.assertTrue(candidate[3])
        self.assertAlmostEqual(math.degrees(candidate[0]), 26.5, places=3)
        self.assertAlmostEqual(candidate[1], 142.2, places=3)

    def test_cumulative_curve_rejects_opposing_wiggle(self):
        candidate = cumulative_turn_candidate(
            [
                (10.0, 0.5, math.radians(14.0)),
                (10.8, 1.3, math.radians(-14.0)),
                (11.5, 2.0, math.radians(8.0)),
            ],
            threshold=math.radians(20.0),
        )

        self.assertFalse(candidate[3])

    def test_cumulative_curve_rejects_single_sharp_corner(self):
        candidate = cumulative_turn_candidate(
            [(54.0, 0.5, math.radians(-81.0))],
            threshold=math.radians(20.0),
        )

        self.assertFalse(candidate[3])

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

    def test_lateral_pid_preserves_existing_proportional_command(self):
        old_command = lateral_velocity_command(
            lateral_error=-0.12,
            route_yaw=0.0,
            current_yaw=0.0,
            gain=0.75,
            max_vy=0.15,
            deadband=0.03,
            heading_limit_deg=25.0,
        )
        result = lateral_pid_command(
            lateral_error=-0.12,
            route_yaw=0.0,
            current_yaw=0.0,
            dt=0.05,
            kp=0.75,
            ki=0.0,
            kd=0.0,
            max_vy=0.15,
            deadband=0.03,
            heading_limit_deg=25.0,
        )

        self.assertAlmostEqual(result['command'], old_command)
        self.assertAlmostEqual(result['p_term'], old_command)

    def test_lateral_pid_integral_removes_persistent_small_bias(self):
        state = {
            'integral_error': 0.0,
            'previous_error': None,
            'filtered_derivative': 0.0,
        }
        first_command = None
        result = None
        for _ in range(200):
            result = lateral_pid_command(
                lateral_error=-0.10,
                route_yaw=0.0,
                current_yaw=0.0,
                dt=0.05,
                integral_error=state['integral_error'],
                previous_error=state['previous_error'],
                filtered_derivative=state['filtered_derivative'],
            )
            if first_command is None:
                first_command = result['command']
            state = {
                'integral_error': result['integral_error'],
                'previous_error': result['previous_error'],
                'filtered_derivative': result['filtered_derivative'],
            }

        self.assertGreater(result['command'], first_command)
        self.assertGreater(result['i_term'], 0.0)
        self.assertLessEqual(result['i_term'], 0.04)

    def test_lateral_pid_does_not_wind_up_when_p_is_saturated(self):
        state = {
            'integral_error': 0.0,
            'previous_error': None,
            'filtered_derivative': 0.0,
        }
        result = None
        for _ in range(200):
            result = lateral_pid_command(
                lateral_error=-0.50,
                route_yaw=0.0,
                current_yaw=0.0,
                dt=0.05,
                integral_error=state['integral_error'],
                previous_error=state['previous_error'],
                filtered_derivative=state['filtered_derivative'],
            )
            state = {
                'integral_error': result['integral_error'],
                'previous_error': result['previous_error'],
                'filtered_derivative': result['filtered_derivative'],
            }

        self.assertAlmostEqual(result['command'], 0.15)
        self.assertAlmostEqual(result['integral_error'], 0.0)
        self.assertAlmostEqual(result['i_term'], 0.0)

    def test_lateral_pid_resets_accumulation_when_error_changes_side(self):
        accumulated = lateral_pid_command(
            lateral_error=-0.10,
            route_yaw=0.0,
            current_yaw=0.0,
            dt=0.5,
            integral_error=0.3,
            previous_error=0.07,
            filtered_derivative=0.01,
        )
        changed_side = lateral_pid_command(
            lateral_error=0.10,
            route_yaw=0.0,
            current_yaw=0.0,
            dt=0.05,
            integral_error=accumulated['integral_error'],
            previous_error=accumulated['previous_error'],
            filtered_derivative=accumulated['filtered_derivative'],
        )

        self.assertLess(changed_side['command'], 0.0)
        self.assertLessEqual(changed_side['integral_error'], 0.0)
        self.assertAlmostEqual(changed_side['d_term'], 0.0)

    def test_lateral_pid_clears_state_outside_heading_limit(self):
        result = lateral_pid_command(
            lateral_error=-0.20,
            route_yaw=math.radians(30.0),
            current_yaw=0.0,
            dt=0.05,
            integral_error=0.4,
            previous_error=0.1,
            filtered_derivative=0.1,
            heading_limit_deg=25.0,
        )

        self.assertFalse(result['active'])
        self.assertEqual(result['command'], 0.0)
        self.assertEqual(result['integral_error'], 0.0)
        self.assertIsNone(result['previous_error'])

    def test_lateral_pid_derivative_is_filtered_and_bounded(self):
        result = lateral_pid_command(
            lateral_error=-0.20,
            route_yaw=0.0,
            current_yaw=0.0,
            dt=0.02,
            previous_error=-0.10,
            filtered_derivative=0.0,
            kd=1.0,
            derivative_vy_limit=0.02,
        )

        self.assertLessEqual(abs(result['d_term']), 0.02)

    def test_first_post_corner_large_error_keeps_same_saturated_command(self):
        old_command = lateral_velocity_command(
            lateral_error=-0.47,
            route_yaw=0.0,
            current_yaw=0.0,
            gain=0.75,
            max_vy=0.15,
            deadband=0.03,
            heading_limit_deg=25.0,
        )
        result = lateral_pid_command(
            lateral_error=-0.47,
            route_yaw=0.0,
            current_yaw=0.0,
            dt=0.05,
            kp=0.75,
            ki=0.08,
            kd=0.04,
            max_vy=0.15,
            deadband=0.03,
            heading_limit_deg=25.0,
        )

        self.assertAlmostEqual(old_command, 0.15)
        self.assertAlmostEqual(result['command'], old_command)
        self.assertAlmostEqual(result['i_term'], 0.0)
        self.assertAlmostEqual(result['d_term'], 0.0)

    def test_pid_warmup_keeps_old_lateral_velocity_limit(self):
        self.assertAlmostEqual(
            lateral_velocity_limit(0.20, True, False, 0.15),
            0.15,
        )
        self.assertAlmostEqual(
            lateral_velocity_limit(0.20, True, True, 0.15),
            0.20,
        )
        self.assertAlmostEqual(
            lateral_velocity_limit(0.20, False, False, 0.15),
            0.20,
        )

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

#!/usr/bin/env python3
import math
import unittest

from go2_fastlio_patrol.patrol_control import (
    corner_heading_command,
    cumulative_turn_candidate,
    lateral_drift_compensation_target,
    lateral_velocity_alignment_allowed,
    lateral_velocity_command,
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

    def test_lateral_velocity_requires_aligned_follow_state(self):
        self.assertTrue(lateral_velocity_alignment_allowed(
            'FOLLOW',
            steering_error=math.radians(4.0),
            route_heading_error=math.radians(6.0),
            steering_limit_deg=8.0,
            route_heading_limit_deg=10.0,
        ))
        self.assertFalse(lateral_velocity_alignment_allowed(
            'FOLLOW',
            steering_error=math.radians(9.0),
            route_heading_error=math.radians(6.0),
            steering_limit_deg=8.0,
            route_heading_limit_deg=10.0,
        ))
        self.assertFalse(lateral_velocity_alignment_allowed(
            'FOLLOW',
            steering_error=math.radians(4.0),
            route_heading_error=math.radians(11.0),
            steering_limit_deg=8.0,
            route_heading_limit_deg=10.0,
        ))

    def test_production_heading_limit_allows_straight_side_slip_correction(self):
        self.assertTrue(lateral_velocity_alignment_allowed(
            'FOLLOW',
            steering_error=math.radians(6.0),
            route_heading_error=math.radians(20.0),
        ))
        self.assertFalse(lateral_velocity_alignment_allowed(
            'FOLLOW',
            steering_error=math.radians(6.0),
            route_heading_error=math.radians(26.0),
        ))
        self.assertFalse(lateral_velocity_alignment_allowed(
            'CURVE',
            steering_error=0.0,
            route_heading_error=math.radians(20.0),
        ))

    def test_lateral_velocity_is_always_disabled_outside_follow(self):
        for mode in ('CURVE', 'CORNER', 'APPROACH_CORNER', 'RECOVER'):
            self.assertFalse(lateral_velocity_alignment_allowed(
                mode,
                steering_error=0.0,
                route_heading_error=0.0,
            ))

    def test_drift_compensation_adds_missing_return_velocity(self):
        correction = lateral_drift_compensation_target(
            lateral_error=-0.19,
            base_vy=0.11,
            actual_cross_velocity=0.0,
            stable_elapsed=2.0,
        )

        self.assertTrue(correction['active'])
        self.assertAlmostEqual(correction['desired_closing_speed'], 0.035)
        self.assertAlmostEqual(correction['extra_vy'], 0.035)

    def test_drift_compensation_preserves_existing_correction_sign(self):
        correction = lateral_drift_compensation_target(
            lateral_error=0.20,
            base_vy=-0.12,
            actual_cross_velocity=0.0,
            stable_elapsed=2.0,
        )

        self.assertLess(correction['extra_vy'], 0.0)

    def test_drift_compensation_stays_off_before_stable_straight(self):
        for eligible, elapsed in ((False, 2.0), (True, 0.8)):
            correction = lateral_drift_compensation_target(
                lateral_error=-0.20,
                base_vy=0.12,
                actual_cross_velocity=0.0,
                stable_elapsed=elapsed,
                eligible=eligible,
            )
            self.assertFalse(correction['active'])
            self.assertEqual(correction['extra_vy'], 0.0)

    def test_drift_compensation_stays_off_when_motion_already_returns(self):
        correction = lateral_drift_compensation_target(
            lateral_error=-0.20,
            base_vy=0.12,
            actual_cross_velocity=0.04,
            stable_elapsed=2.0,
        )

        self.assertFalse(correction['active'])
        self.assertEqual(correction['extra_vy'], 0.0)

    def test_drift_compensation_is_bounded_during_wrong_way_motion(self):
        correction = lateral_drift_compensation_target(
            lateral_error=-0.20,
            base_vy=0.12,
            actual_cross_velocity=-0.20,
            stable_elapsed=2.0,
        )

        self.assertAlmostEqual(correction['extra_vy'], 0.04)

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

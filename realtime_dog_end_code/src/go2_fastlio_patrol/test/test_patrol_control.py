#!/usr/bin/env python3
import math
import unittest

from go2_fastlio_patrol.patrol_control import (
    corner_turn_angle,
    displacement_course_heading,
    effective_forward_speed,
    feedback_motion_scale,
    heading_drive_command,
    line_follow_command,
    limit_planar_command,
    normalize_angle,
    ordered_route_heading,
    ordered_upcoming_corner,
    point_target_heading,
    point_in_lateral_motion_roi,
    segment_metrics,
    stream_receive_age,
    waypoint_reached,
)


class PatrolControlTest(unittest.TestCase):
    def test_course_heading_requires_real_displacement(self):
        self.assertIsNone(
            displacement_course_heading(
                0.0,
                0.0,
                0.02,
                0.01,
                minimum_distance=0.10,
            )
        )
        heading = displacement_course_heading(
            0.0,
            0.0,
            0.10,
            0.10,
            minimum_distance=0.10,
        )
        self.assertAlmostEqual(math.degrees(heading), 45.0)

    def test_segment_metrics_preserves_selected_csv_line(self):
        metrics = segment_metrics(
            x=2.0,
            y=0.4,
            start_x=1.0,
            start_y=0.0,
            end_x=3.0,
            end_y=0.0,
        )

        self.assertAlmostEqual(metrics['along'], 1.0)
        self.assertAlmostEqual(metrics['lateral_error'], 0.4)
        self.assertAlmostEqual(metrics['projected_x'], 2.0)
        self.assertAlmostEqual(metrics['projected_y'], 0.0)

    def test_waypoint_gate_advances_in_order_when_crossed(self):
        metrics = segment_metrics(1.1, 0.1, 0.0, 0.0, 1.0, 0.0)

        self.assertTrue(waypoint_reached(metrics, 0.05))

    def test_waypoint_gate_advances_after_endpoint_despite_offset(self):
        metrics = segment_metrics(1.1, 0.7, 0.0, 0.0, 1.0, 0.0)

        self.assertTrue(waypoint_reached(metrics, 0.05))

    def test_waypoint_gate_does_not_advance_before_endpoint(self):
        metrics = segment_metrics(0.9, 0.7, 0.0, 0.0, 1.0, 0.0)

        self.assertFalse(waypoint_reached(metrics, 0.05))

    def test_failed_353_to_354_pose_can_no_longer_lock_gate(self):
        metrics = segment_metrics(
            55.983,
            -83.919,
            56.514987,
            -82.898716,
            56.514987,
            -83.297340,
        )

        self.assertGreater(abs(metrics['lateral_error']), 0.50)
        self.assertGreater(metrics['along'], metrics['length'])
        self.assertTrue(waypoint_reached(metrics, 0.20))

    def test_corner_gate_advances_after_crossing_with_offset(self):
        metrics = segment_metrics(1.1, 0.40, 0.0, 0.0, 1.0, 0.0)

        self.assertTrue(waypoint_reached(metrics, 0.20))

    def test_corner_angle_uses_ordered_csv_segments(self):
        angle = corner_turn_angle(
            (0.0, 0.0),
            (1.0, 0.0),
            (1.0, 1.0),
        )

        self.assertAlmostEqual(math.degrees(angle), 90.0)

    def test_corner_is_detected_before_current_target_reaches_it(self):
        route = [
            {'x': 0.0, 'y': 0.0},
            {'x': 0.2, 'y': 0.0},
            {'x': 0.4, 'y': 0.0},
            {'x': 0.6, 'y': 0.0},
            {'x': 0.8, 'y': 0.0},
            {'x': 1.0, 'y': 0.0},
            {'x': 1.0, 'y': 0.2},
        ]
        corner = ordered_upcoming_corner(
            route=route,
            anchor_index=0,
            target_index=1,
            direction=1,
            current_segment_remaining=0.05,
            corner_angle=math.radians(30.0),
            search_distance=0.90,
        )

        self.assertEqual(corner['index'], 5)
        self.assertAlmostEqual(corner['distance'], 0.85)
        self.assertAlmostEqual(
            math.degrees(corner['turn_angle']),
            90.0,
        )

    def test_corner_outside_preparation_horizon_is_ignored(self):
        route = [
            {'x': 0.0, 'y': 0.0},
            {'x': 0.5, 'y': 0.0},
            {'x': 1.0, 'y': 0.0},
            {'x': 1.0, 'y': 0.5},
        ]
        corner = ordered_upcoming_corner(
            route=route,
            anchor_index=0,
            target_index=1,
            direction=1,
            current_segment_remaining=0.50,
            corner_angle=math.radians(30.0),
            search_distance=0.40,
        )

        self.assertIsNone(corner)

    def test_ordered_route_heading_smoothly_sees_a_csv_curve(self):
        route = [
            {'x': 0.0, 'y': 0.0},
            {'x': 0.4, 'y': 0.0},
            {'x': 0.8, 'y': 0.1},
            {'x': 1.1, 'y': 0.4},
        ]
        arc_s = [0.0]
        for previous, current in zip(route, route[1:]):
            arc_s.append(
                arc_s[-1]
                + math.hypot(
                    current['x'] - previous['x'],
                    current['y'] - previous['y'],
                )
            )

        heading = ordered_route_heading(
            route,
            arc_s,
            progress_s=0.30,
            direction=1,
            lookahead_distance=0.60,
            fallback_yaw=0.0,
        )

        self.assertGreater(math.degrees(heading), 5.0)
        self.assertLess(math.degrees(heading), 30.0)

    def test_ordered_route_heading_respects_reverse_direction(self):
        route = [
            {'x': 0.0, 'y': 0.0},
            {'x': 1.0, 'y': 0.0},
            {'x': 2.0, 'y': 0.0},
        ]

        heading = ordered_route_heading(
            route,
            [0.0, 1.0, 2.0],
            progress_s=1.5,
            direction=-1,
            lookahead_distance=0.5,
            fallback_yaw=0.0,
        )

        self.assertAlmostEqual(abs(math.degrees(heading)), 180.0)

    def test_left_of_line_turns_body_right_without_crab(self):
        command = line_follow_command(
            lateral_error=0.40,
            lateral_velocity=0.0,
            route_yaw=0.0,
            current_yaw=0.0,
            forward_speed=0.50,
        )

        self.assertAlmostEqual(command['vx'], 0.50)
        self.assertEqual(command['vy'], 0.0)
        self.assertLess(command['correction_angle'], 0.0)
        self.assertLess(command['yaw_rate'], 0.0)

    def test_larger_line_error_creates_bounded_stronger_intercept(self):
        near = line_follow_command(
            lateral_error=0.05,
            lateral_velocity=0.0,
            route_yaw=0.0,
            current_yaw=0.0,
            forward_speed=0.50,
        )
        far = line_follow_command(
            lateral_error=0.80,
            lateral_velocity=0.0,
            route_yaw=0.0,
            current_yaw=0.0,
            forward_speed=0.50,
        )

        self.assertLess(near['correction_angle'], 0.0)
        self.assertLess(
            far['correction_angle'],
            near['correction_angle'],
        )
        self.assertGreaterEqual(
            far['correction_angle'],
            -math.radians(12.0),
        )

    def test_twenty_five_degree_intercept_is_capped_at_twelve(self):
        lookahead = 0.50
        error_for_twenty_five_degrees = (
            lookahead * math.tan(math.radians(25.0))
        )
        command = line_follow_command(
            lateral_error=error_for_twenty_five_degrees,
            lateral_velocity=0.0,
            route_yaw=0.0,
            current_yaw=0.0,
            forward_speed=0.50,
            correction_lookahead_distance=lookahead,
            max_correction_angle=math.radians(12.0),
        )

        self.assertAlmostEqual(
            math.degrees(command['correction_angle']),
            -12.0,
        )

    def test_cross_velocity_prediction_straightens_before_crossing(self):
        stationary = line_follow_command(
            lateral_error=0.08,
            lateral_velocity=0.0,
            route_yaw=0.0,
            current_yaw=0.0,
            forward_speed=0.50,
        )
        already_closing = line_follow_command(
            lateral_error=0.08,
            lateral_velocity=-0.10,
            route_yaw=0.0,
            current_yaw=0.0,
            forward_speed=0.50,
        )

        self.assertLess(
            abs(already_closing['correction_angle']),
            abs(stationary['correction_angle']),
        )

    def test_line_deadband_stops_position_correction(self):
        command = line_follow_command(
            lateral_error=0.01,
            lateral_velocity=0.0,
            route_yaw=0.0,
            current_yaw=0.0,
            forward_speed=0.50,
        )

        self.assertEqual(command['correction_angle'], 0.0)
        self.assertEqual(command['vy'], 0.0)

    def test_three_centimeter_corridor_ignores_smaller_error(self):
        command = line_follow_command(
            lateral_error=0.027,
            lateral_velocity=0.0,
            route_yaw=0.0,
            current_yaw=0.0,
            forward_speed=0.50,
            line_deadband=0.030,
        )

        self.assertEqual(command['correction_angle'], 0.0)
        self.assertEqual(command['yaw_rate'], 0.0)

    def test_right_of_line_turns_body_left_without_crab(self):
        command = line_follow_command(
            lateral_error=-0.05,
            lateral_velocity=0.0,
            route_yaw=0.0,
            current_yaw=0.0,
            forward_speed=0.50,
        )

        self.assertGreater(command['correction_angle'], 0.0)
        self.assertGreater(command['yaw_rate'], 0.0)
        self.assertEqual(command['vy'], 0.0)

    def test_course_feedback_keeps_turning_when_body_only_looks_aligned(self):
        target_intercept = math.radians(12.0)
        command = line_follow_command(
            lateral_error=-0.20,
            lateral_velocity=0.0,
            route_yaw=0.0,
            current_yaw=target_intercept,
            motion_yaw=0.0,
            forward_speed=0.50,
            correction_lookahead_distance=0.50,
            max_correction_angle=target_intercept,
        )

        self.assertEqual(command['heading_feedback_source'], 'course')
        self.assertAlmostEqual(
            math.degrees(command['body_heading_error']),
            0.0,
        )
        self.assertGreater(command['yaw_rate'], 0.0)
        self.assertEqual(command['vy'], 0.0)

    def test_body_feedback_is_used_until_course_is_measurable(self):
        command = line_follow_command(
            lateral_error=-0.20,
            lateral_velocity=0.0,
            route_yaw=0.0,
            current_yaw=math.radians(12.0),
            motion_yaw=None,
            forward_speed=0.50,
            correction_lookahead_distance=0.50,
            max_correction_angle=math.radians(12.0),
        )

        self.assertEqual(command['heading_feedback_source'], 'body')
        self.assertEqual(command['yaw_rate'], 0.0)

    def test_heading_error_only_aligns_body_to_route(self):
        command = line_follow_command(
            lateral_error=0.0,
            lateral_velocity=0.0,
            route_yaw=math.radians(10.0),
            current_yaw=0.0,
            forward_speed=0.50,
        )

        self.assertGreater(command['yaw_rate'], 0.0)
        self.assertLessEqual(command['yaw_rate'], 0.30)

    def test_heading_drive_has_minimum_effective_yaw_and_zero_vy(self):
        command = heading_drive_command(
            desired_yaw=math.radians(4.0),
            current_yaw=0.0,
            forward_speed=0.50,
            yaw_deadband=math.radians(2.0),
            min_yaw_rate=0.15,
        )

        self.assertEqual(command['vy'], 0.0)
        self.assertAlmostEqual(command['yaw_rate'], 0.15)

    def test_corner_approach_aims_at_corner_not_incoming_line(self):
        heading = point_target_heading(
            x=0.8,
            y=0.2,
            target_x=1.0,
            target_y=0.0,
            fallback_yaw=0.0,
        )

        self.assertAlmostEqual(math.degrees(heading), -45.0)

    def test_forward_steering_reaches_three_centimeter_corridor(self):
        x = 0.0
        lateral_error = 0.12
        yaw = 0.0
        cross_velocity = 0.0
        first_corridor_x = None
        corridor_errors = []

        for _ in range(160):
            command = line_follow_command(
                lateral_error=lateral_error,
                lateral_velocity=cross_velocity,
                route_yaw=0.0,
                current_yaw=yaw,
                forward_speed=0.50,
                line_deadband=0.030,
                correction_lookahead_distance=0.50,
                correction_prediction_time=0.20,
                max_correction_angle=math.radians(12.0),
            )
            self.assertEqual(command['vy'], 0.0)

            dt = 0.05
            yaw = normalize_angle(yaw + command['yaw_rate'] * dt)
            x += command['vx'] * math.cos(yaw) * dt
            cross_velocity = command['vx'] * math.sin(yaw)
            lateral_error += cross_velocity * dt
            if abs(lateral_error) <= 0.03:
                if first_corridor_x is None:
                    first_corridor_x = x
                corridor_errors.append(lateral_error)

        self.assertIsNotNone(first_corridor_x)
        self.assertLess(first_corridor_x, 1.0)
        self.assertLessEqual(
            max(abs(value) for value in corridor_errors),
            0.031,
        )

    def test_continuous_correction_resists_persistent_right_yaw_bias(self):
        lateral_error = 0.0
        yaw = 0.0
        cross_velocity = 0.0
        peak_error = 0.0

        for _ in range(800):
            command = line_follow_command(
                lateral_error=lateral_error,
                lateral_velocity=cross_velocity,
                route_yaw=0.0,
                current_yaw=yaw,
                forward_speed=0.50,
                line_deadband=0.030,
                correction_lookahead_distance=0.50,
                correction_prediction_time=0.20,
                max_correction_angle=math.radians(12.0),
                yaw_deadband=0.03,
                min_yaw_rate=0.20,
            )

            dt = 0.05
            effective_yaw_rate = 0.40 * command['yaw_rate'] - 0.03
            yaw = normalize_angle(yaw + effective_yaw_rate * dt)
            cross_velocity = command['vx'] * math.sin(yaw)
            lateral_error += cross_velocity * dt
            peak_error = max(peak_error, abs(lateral_error))

        self.assertLess(peak_error, 0.05)

    def test_receive_age_ignores_remote_header_stamp(self):
        self.assertAlmostEqual(
            stream_receive_age(100.0, 99.9),
            0.1,
        )
        self.assertTrue(
            math.isinf(stream_receive_age(100.0, 0.0))
        )

    def test_feedback_age_uses_effective_floor_before_hold(self):
        scales = [
            feedback_motion_scale(age)
            for age in (0.0, 0.60, 0.90, 1.20, 1.60, 2.00, 3.0)
        ]

        self.assertEqual(scales[0], 1.0)
        self.assertEqual(scales[1], 1.0)
        self.assertAlmostEqual(scales[3], 0.50)
        self.assertAlmostEqual(scales[4], 0.50)
        self.assertEqual(scales[5], 0.0)
        self.assertEqual(scales[6], 0.0)
        self.assertTrue(
            all(
                left >= right
                for left, right in zip(scales, scales[1:])
            )
        )

    def test_invalid_feedback_age_holds_motion(self):
        self.assertEqual(feedback_motion_scale(float('inf')), 0.0)
        self.assertEqual(feedback_motion_scale(float('nan')), 0.0)

    def test_forward_speed_never_stays_below_effective_floor(self):
        self.assertAlmostEqual(
            effective_forward_speed(0.50, 0.50, 0.50, 0.22),
            0.25,
        )
        self.assertAlmostEqual(
            effective_forward_speed(0.20, 0.50, 0.35, 0.22),
            0.22,
        )
        self.assertAlmostEqual(
            effective_forward_speed(0.20, 0.15, 0.35, 0.22),
            0.15,
        )
        self.assertEqual(
            effective_forward_speed(0.50, 0.50, 0.0, 0.22),
            0.0,
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
        self.assertTrue(
            point_in_lateral_motion_roi(0.5, 0.4, 0.5, vy=0.1)
        )
        self.assertFalse(
            point_in_lateral_motion_roi(0.5, -0.4, 0.5, vy=0.1)
        )
        self.assertTrue(
            point_in_lateral_motion_roi(0.5, -0.4, 0.5, vy=-0.1)
        )
        self.assertFalse(
            point_in_lateral_motion_roi(0.5, 0.4, 0.5, vy=-0.1)
        )

    def test_lateral_obstacle_roi_is_off_without_lateral_motion(self):
        self.assertFalse(
            point_in_lateral_motion_roi(0.5, 0.4, 0.5, vy=0.0)
        )
        self.assertFalse(
            point_in_lateral_motion_roi(0.5, 0.4, 0.5, vy=0.02)
        )

    def test_lateral_obstacle_roi_excludes_body_and_height(self):
        self.assertFalse(
            point_in_lateral_motion_roi(0.5, 0.2, 0.5, vy=0.1)
        )
        self.assertFalse(
            point_in_lateral_motion_roi(0.5, 0.4, 1.0, vy=0.1)
        )


if __name__ == '__main__':
    unittest.main()

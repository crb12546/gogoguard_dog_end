#!/usr/bin/env python3
import math
import unittest

from go2_fastlio_patrol.route_quality import (
    AdaptiveRouteSampler,
    ConsecutiveLargeGapGate,
    OdometryQualityGate,
    RouteSample,
    analyze_source_gaps,
    assess_source_gap,
    build_clean_route,
    sample_distance,
    straighten_stable_yaw_sections,
    validate_route_report,
)


def line_samples(start, end, spacing, yaw):
    length = math.hypot(end[0] - start[0], end[1] - start[1])
    count = max(1, int(round(length / spacing)))
    return [
        RouteSample(
            start[0] + (end[0] - start[0]) * index / count,
            start[1] + (end[1] - start[1]) * index / count,
            yaw,
        )
        for index in range(count + 1)
    ]


class RouteQualityTest(unittest.TestCase):
    def assert_max_gap(self, samples, limit):
        gaps = [
            sample_distance(samples[index - 1], samples[index])
            for index in range(1, len(samples))
        ]
        self.assertLessEqual(max(gaps, default=0.0), limit)

    def test_straight_line_is_straight_and_isolated_spike_is_removed(self):
        raw = []
        for index in range(501):
            x = index * 0.02
            y = 0.012 * math.sin(index * 0.31)
            if index == 250:
                y += 0.22
            raw.append(RouteSample(x, y, 0.0))

        clean, report = build_clean_route(
            raw,
            straighten_stable_sections=False,
        )

        self.assertGreaterEqual(report['isolated_spikes_removed'], 1)
        self.assertLess(max(abs(sample.y) for sample in clean), 0.02)
        self.assert_max_gap(clean, 0.205)

    def test_right_angle_keeps_exact_corner_and_dense_turn_zone(self):
        raw = line_samples((0.0, 0.0), (1.0, 0.0), 0.02, 0.0)
        raw += line_samples((1.0, 0.0), (1.0, 1.0), 0.02, math.pi / 2)[1:]

        clean, _ = build_clean_route(raw)

        corner_index = min(
            range(len(clean)),
            key=lambda index: math.hypot(clean[index].x - 1.0, clean[index].y),
        )
        self.assertLess(math.hypot(clean[corner_index].x - 1.0, clean[corner_index].y), 0.005)
        self.assertGreater(corner_index, 0)
        self.assertLess(corner_index, len(clean) - 1)
        self.assertLessEqual(sample_distance(clean[corner_index - 1], clean[corner_index]), 0.105)
        self.assertLessEqual(sample_distance(clean[corner_index], clean[corner_index + 1]), 0.105)
        self.assert_max_gap(clean, 0.205)

    def test_ten_degree_course_changes_are_preserved(self):
        yaw = math.radians(10.0)
        corner = (10.0 * math.cos(yaw), 10.0 * math.sin(yaw))
        raw = line_samples((0.0, 0.0), corner, 0.02, yaw)
        raw += line_samples(corner, (corner[0] + 5.0, corner[1]), 0.02, 0.0)[1:]

        clean, _ = build_clean_route(raw)

        nearest = min(math.hypot(sample.x - corner[0], sample.y - corner[1]) for sample in clean)
        self.assertLess(nearest, 0.005)
        self.assert_max_gap(clean, 0.205)

    def test_in_place_oversteer_does_not_create_xy_detour(self):
        raw = line_samples((0.0, 0.0), (1.0, 0.0), 0.02, 0.0)
        raw.extend([
            RouteSample(1.0, 0.0, math.radians(30.0)),
            RouteSample(1.0, 0.0, math.radians(95.0)),
            RouteSample(1.0, 0.0, math.radians(90.0)),
        ])
        raw += line_samples((1.0, 0.0), (1.0, 1.0), 0.02, math.pi / 2)[1:]

        clean, _ = build_clean_route(raw)

        self.assertLessEqual(max(sample.x for sample in clean), 1.000001)
        corner_count = sum(
            1 for sample in clean
            if math.hypot(sample.x - 1.0, sample.y) < 0.005
        )
        self.assertEqual(corner_count, 1)

    def test_live_sampler_enters_turn_after_two_confirmations(self):
        sampler = AdaptiveRouteSampler()
        emitted = []
        for index in range(51):
            emitted.extend(sampler.add(RouteSample(index * 0.02, 0.0, 0.0)))
        emitted.extend(sampler.add(RouteSample(1.0, 0.0, math.radians(6.0))))
        self.assertFalse(sampler.turning)
        emitted.extend(sampler.add(RouteSample(1.0, 0.0, math.radians(6.0))))
        self.assertTrue(sampler.turning)
        before = len(emitted)
        emitted.extend(sampler.add(RouteSample(1.10, 0.01, math.radians(12.0))))
        self.assertGreater(len(emitted), before)
        self.assertLessEqual(sample_distance(emitted[-2], emitted[-1]), 0.205)

    def test_time_aware_gate_accepts_35cm_at_plausible_speed(self):
        gate = OdometryQualityGate()
        self.assertTrue(gate.evaluate(0.0, 0.0, 0.0, 0.0).accepted)

        decision = gate.evaluate(0.35, 0.0, 0.0, 0.112)

        self.assertTrue(decision.accepted)
        self.assertLess(decision.speed_mps, 4.0)
        self.assertGreater(decision.allowed_step_m, 0.35)

    def test_gate_recovers_after_isolated_outlier_without_stale_baseline(self):
        gate = OdometryQualityGate(recovery_samples=3, failure_samples=3)
        self.assertTrue(gate.evaluate(0.0, 0.0, 0.0, 0.0).accepted)
        first_bad = gate.evaluate(0.8, 0.0, 0.0, 0.1)
        self.assertFalse(first_bad.accepted)

        # The next samples are locally plausible around the new coordinate.
        # A stale-baseline gate would keep comparing them with x=0 forever;
        # this gate advances its observation baseline and recovers deliberately.
        self.assertFalse(gate.evaluate(0.82, 0.0, 0.0, 0.2).accepted)
        self.assertFalse(gate.evaluate(0.84, 0.0, 0.0, 0.3).accepted)
        recovered = gate.evaluate(0.86, 0.0, 0.0, 0.4)

        self.assertTrue(recovered.accepted)
        self.assertTrue(recovered.recovered)

    def test_gate_fails_after_sustained_impossible_motion(self):
        gate = OdometryQualityGate(failure_samples=3)
        self.assertTrue(gate.evaluate(0.0, 0.0, 0.0, 0.0).accepted)
        self.assertFalse(gate.evaluate(1.0, 0.0, 0.0, 0.1).should_fail)
        self.assertFalse(gate.evaluate(2.0, 0.0, 0.0, 0.2).should_fail)
        failed = gate.evaluate(3.0, 0.0, 0.0, 0.3)

        self.assertFalse(failed.accepted)
        self.assertTrue(failed.should_fail)

    def test_large_straight_source_gap_is_safe_to_interpolate(self):
        assessment = assess_source_gap(
            RouteSample(0.0, 0.0, 0.0),
            RouteSample(1.15, 0.02, math.radians(1.0)),
        )

        self.assertTrue(assessment.exceeds_soft_limit)
        self.assertFalse(assessment.exceeds_hard_limit)
        self.assertTrue(assessment.safe_to_interpolate)

    def test_large_missing_turn_source_gap_is_not_safe_to_interpolate(self):
        angle = math.radians(30.0)
        assessment = assess_source_gap(
            RouteSample(0.0, 0.0, 0.0),
            RouteSample(math.sin(angle) * 2.0, (1.0 - math.cos(angle)) * 2.0, angle),
        )

        self.assertTrue(assessment.exceeds_soft_limit)
        self.assertFalse(assessment.exceeds_hard_limit)
        self.assertFalse(assessment.safe_to_interpolate)

    def test_missing_turn_is_rejected_by_neighboring_xy_headings(self):
        angle = math.radians(30.0)
        before = RouteSample(-0.20, 0.0, 0.0)
        start = RouteSample(0.0, 0.0, 0.0)
        end = RouteSample(
            math.sin(angle) * 2.0,
            (1.0 - math.cos(angle)) * 2.0,
            angle,
        )
        after = RouteSample(
            end.x + 0.20 * math.cos(angle),
            end.y + 0.20 * math.sin(angle),
            angle,
        )

        report = analyze_source_gaps([before, start, end, after])

        self.assertEqual(report['large_source_gap_count'], 1)
        self.assertEqual(report['unsafe_source_gap_count'], 1)

    def test_xbf0720_2_sampled_gap_does_not_trigger_source_gap_limit(self):
        last_emitted = RouteSample(54.246370, -21.136912, -1.055475)
        previous_raw = RouteSample(54.317391, -21.304084, -1.054218)
        current_raw = RouteSample(54.256905, -22.000695, -1.028391)

        self.assertGreater(sample_distance(last_emitted, current_raw), 0.80)
        assessment = assess_source_gap(previous_raw, current_raw)
        self.assertLess(assessment.gap_m, 0.80)
        self.assertTrue(assessment.safe_to_interpolate)

    def test_xbf0720_3_crab_motion_follows_previous_xy_track(self):
        raw = [
            RouteSample(29.061439, 0.403362, -0.008479),
            RouteSample(29.395049, 0.375822, -0.068940),
            RouteSample(29.977459, 0.660104, -0.081758),
            RouteSample(30.993659, 1.198199, -0.106163),
        ]

        report = analyze_source_gaps(raw)

        self.assertEqual(report['large_source_gap_count'], 1)
        self.assertEqual(report['unsafe_source_gap_count'], 0)

    def test_large_side_jump_and_snapback_is_unsafe(self):
        raw = [
            RouteSample(0.0, 0.0, 0.0),
            RouteSample(0.2, 0.0, 0.0),
            RouteSample(1.2, 0.8, 0.0),
            RouteSample(1.4, 0.0, 0.0),
            RouteSample(1.6, 0.0, 0.0),
        ]

        report = analyze_source_gaps(raw)

        self.assertGreaterEqual(report['unsafe_source_gap_count'], 1)

    def test_stable_yaw_crab_drift_is_projected_to_straight_line(self):
        raw = [
            RouteSample(index * 0.05, index * 0.001, 0.0)
            for index in range(201)
        ]

        straight, report = straighten_stable_yaw_sections(raw)

        self.assertGreaterEqual(report['straightened_sections'], 1)
        self.assertGreater(report['max_lateral_correction_m'], 0.19)
        self.assertLess(max(abs(sample.y) for sample in straight), 1e-9)

    def test_short_heading_correction_is_merged_into_same_straight(self):
        raw = []
        for index in range(201):
            x = index * 0.05
            y = 0.20 * math.sin(math.pi * x / 10.0)
            yaw = 0.0
            if 98 <= index <= 102:
                yaw = math.radians(10.0)
            raw.append(RouteSample(x, y, yaw))

        straight, report = straighten_stable_yaw_sections(raw)

        self.assertGreaterEqual(report['straightened_sections'], 1)
        self.assertLess(max(abs(sample.y) for sample in straight), 1e-9)

    def test_smooth_arc_is_not_flattened(self):
        radius = 2.0
        raw = []
        for index in range(181):
            angle = math.radians(90.0 * index / 180.0)
            raw.append(RouteSample(
                radius * math.sin(angle),
                radius * (1.0 - math.cos(angle)),
                angle,
            ))

        clean, _ = build_clean_route(raw)
        radial_errors = [
            abs(math.hypot(sample.x, sample.y - radius) - radius)
            for sample in clean
        ]

        self.assertLess(max(radial_errors), 0.04)

    def test_large_gap_streak_fails_on_fifth_consecutive_transition(self):
        gate = ConsecutiveLargeGapGate(threshold_m=1.50, failure_count=5)

        for expected_streak in range(1, 5):
            decision = gate.evaluate(1.60)
            self.assertEqual(decision.current_streak, expected_streak)
            self.assertFalse(decision.should_fail)
        decision = gate.evaluate(1.60)

        self.assertEqual(decision.current_streak, 5)
        self.assertTrue(decision.should_fail)

    def test_large_gap_streak_resets_after_normal_transition(self):
        gate = ConsecutiveLargeGapGate(threshold_m=1.50, failure_count=5)
        self.assertEqual(gate.evaluate(1.60).current_streak, 1)
        self.assertEqual(gate.evaluate(1.70).current_streak, 2)

        reset = gate.evaluate(0.20)

        self.assertEqual(reset.current_streak, 0)
        for expected_streak in range(1, 5):
            decision = gate.evaluate(1.80)
            self.assertEqual(decision.current_streak, expected_streak)
            self.assertFalse(decision.should_fail)

    def test_single_gap_over_three_metres_is_catastrophic(self):
        assessment = assess_source_gap(
            RouteSample(0.0, 0.0, 0.0),
            RouteSample(3.01, 0.0, 0.0),
        )

        self.assertTrue(assessment.exceeds_hard_limit)
        self.assertFalse(assessment.safe_to_interpolate)

    def test_single_safe_gap_below_three_metres_can_be_interpolated(self):
        assessment = assess_source_gap(
            RouteSample(0.0, 0.0, 0.0),
            RouteSample(2.90, 0.0, 0.0),
        )

        self.assertFalse(assessment.exceeds_hard_limit)
        self.assertTrue(assessment.safe_to_interpolate)

    def test_final_quality_gate_rejects_three_point_short_route(self):
        report = {
            'route_points': 3,
            'route_length_m': 0.35,
            'rejection_ratio': 0.96,
            'max_raw_gap_m': 0.20,
        }

        reasons = validate_route_report(report)

        self.assertGreaterEqual(len(reasons), 3)

    def test_final_quality_gate_accepts_normal_route(self):
        report = {
            'route_points': 101,
            'route_length_m': 20.0,
            'rejection_ratio': 0.01,
            'max_raw_gap_m': 0.25,
        }

        self.assertEqual(validate_route_report(report), [])

    def test_final_quality_gate_rejects_unsafe_interpolation(self):
        report = {
            'route_points': 101,
            'route_length_m': 20.0,
            'rejection_ratio': 0.01,
            'max_raw_gap_m': 1.15,
            'unsafe_source_gap_count': 1,
            'max_unsafe_source_gap_m': 1.15,
        }

        reasons = validate_route_report(report, max_raw_gap_m=3.00)

        self.assertEqual(len(reasons), 1)
        self.assertIn('cannot be safely interpolated', reasons[0])


if __name__ == '__main__':
    unittest.main()

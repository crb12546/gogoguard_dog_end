#!/usr/bin/env python3
import math
import unittest

from go2_fastlio_patrol.route_quality import (
    AdaptiveRouteSampler,
    RouteSample,
    build_clean_route,
    sample_distance,
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

        clean, report = build_clean_route(raw)

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


if __name__ == '__main__':
    unittest.main()

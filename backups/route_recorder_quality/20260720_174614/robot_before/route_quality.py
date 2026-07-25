#!/usr/bin/env python3
"""Pure-Python route sampling and cleanup helpers.

This module deliberately has no ROS imports so the geometry can be exercised with
repeatable simulations on the development machine before it is deployed to Go2.
"""

from collections import deque
from dataclasses import dataclass
import math


@dataclass
class RouteSample:
    x: float
    y: float
    yaw: float


def angle_difference(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


def sample_distance(a, b):
    return math.hypot(b.x - a.x, b.y - a.y)


def _interpolate(a, b, ratio):
    yaw = a.yaw + angle_difference(b.yaw, a.yaw) * ratio
    return RouteSample(
        a.x + (b.x - a.x) * ratio,
        a.y + (b.y - a.y) * ratio,
        yaw,
    )


class AdaptiveRouteSampler:
    """Select useful live-preview points without losing turn anchors.

    Straight sections use ``normal_spacing``.  A sustained yaw departure enters
    turn mode, backfills the beginning of the turn from a short history, and then
    uses ``turn_spacing`` until the heading has remained stable over distance.
    """

    def __init__(
        self,
        normal_spacing=0.20,
        turn_spacing=0.10,
        turn_enter_deg=5.0,
        turn_confirm_samples=2,
        turn_anchor_deg=2.0,
        turn_exit_deg=2.0,
        turn_exit_distance=0.25,
    ):
        self.normal_spacing = max(0.01, float(normal_spacing))
        self.turn_spacing = max(0.01, float(turn_spacing))
        self.turn_enter = math.radians(max(0.1, float(turn_enter_deg)))
        self.turn_confirm_samples = max(1, int(turn_confirm_samples))
        self.turn_anchor = math.radians(max(0.1, float(turn_anchor_deg)))
        self.turn_exit = math.radians(max(0.1, float(turn_exit_deg)))
        self.turn_exit_distance = max(0.01, float(turn_exit_distance))

        self.turning = False
        self.last_saved = None
        self.last_input = None
        self.heading_reference = None
        self.turn_candidate_count = 0
        self.turn_candidate_sign = 0
        self.stable_yaw = None
        self.stable_distance = 0.0
        self.history = deque(maxlen=200)

    @property
    def mode(self):
        return 'turn' if self.turning else 'straight'

    def _emit(self, sample, emitted, force=False):
        if self.last_saved is None or force or sample_distance(self.last_saved, sample) >= 0.005:
            if self.last_saved is None or sample_distance(self.last_saved, sample) >= 0.005:
                emitted.append(sample)
                self.last_saved = sample

    def _find_turn_anchor(self):
        previous = self.last_saved
        for sample in self.history:
            delta = abs(angle_difference(sample.yaw, self.heading_reference))
            if delta >= self.turn_anchor:
                return previous if previous is not None else sample
            previous = sample
        return self.last_saved

    def add(self, sample):
        emitted = []
        if self.last_saved is None:
            self._emit(sample, emitted)
            self.heading_reference = sample.yaw
            self.last_input = sample
            self.history.append(sample)
            return emitted

        step = sample_distance(self.last_input, sample) if self.last_input else 0.0
        self.last_input = sample
        self.history.append(sample)

        if not self.turning:
            delta = angle_difference(sample.yaw, self.heading_reference)
            sign = 1 if delta > 0.0 else -1
            if abs(delta) >= self.turn_enter:
                if sign == self.turn_candidate_sign:
                    self.turn_candidate_count += 1
                else:
                    self.turn_candidate_sign = sign
                    self.turn_candidate_count = 1
            else:
                self.turn_candidate_count = 0
                self.turn_candidate_sign = 0

            if self.turn_candidate_count >= self.turn_confirm_samples:
                anchor = self._find_turn_anchor()
                if anchor is not None:
                    self._emit(anchor, emitted, force=True)
                self.turning = True
                self.stable_yaw = sample.yaw
                self.stable_distance = 0.0
                self.turn_candidate_count = 0
                self.turn_candidate_sign = 0

        if self.turning:
            if abs(angle_difference(sample.yaw, self.stable_yaw)) <= self.turn_exit:
                self.stable_distance += step
            else:
                self.stable_yaw = sample.yaw
                self.stable_distance = 0.0

            if sample_distance(self.last_saved, sample) >= self.turn_spacing:
                self._emit(sample, emitted)

            if self.stable_distance >= self.turn_exit_distance:
                self.turning = False
                self.heading_reference = sample.yaw
                self.stable_distance = 0.0
                self.history.clear()
                self.history.append(sample)
        elif sample_distance(self.last_saved, sample) >= self.normal_spacing:
            self._emit(sample, emitted)
            self.heading_reference = sample.yaw
            self.history.clear()
            self.history.append(sample)

        return emitted


def _deduplicate(samples, minimum_distance=0.005):
    output = []
    for sample in samples:
        if not all(math.isfinite(value) for value in (sample.x, sample.y, sample.yaw)):
            continue
        if output and sample_distance(output[-1], sample) < minimum_distance:
            # Retain the stable position but keep the most recent orientation. This
            # collapses in-place steering corrections into one geometric point.
            output[-1] = RouteSample(output[-1].x, output[-1].y, sample.yaw)
        else:
            output.append(sample)
    return output


def _point_segment_distance(point, start, end):
    dx = end.x - start.x
    dy = end.y - start.y
    length_squared = dx * dx + dy * dy
    if length_squared <= 1e-12:
        return sample_distance(point, start)
    ratio = (
        (point.x - start.x) * dx + (point.y - start.y) * dy
    ) / length_squared
    ratio = min(1.0, max(0.0, ratio))
    projected_x = start.x + ratio * dx
    projected_y = start.y + ratio * dy
    return math.hypot(point.x - projected_x, point.y - projected_y)


def remove_isolated_spikes(samples, lateral_threshold=0.06, turn_threshold_deg=120.0):
    """Remove short out-and-back position glitches, never ordinary corners."""
    points = list(samples)
    removed = 0
    turn_threshold = math.radians(turn_threshold_deg)
    for _ in range(3):
        if len(points) < 3:
            break
        drop = set()
        for index in range(1, len(points) - 1):
            previous = points[index - 1]
            current = points[index]
            following = points[index + 1]
            first_length = sample_distance(previous, current)
            second_length = sample_distance(current, following)
            direct_length = sample_distance(previous, following)
            if min(first_length, second_length) < 0.02:
                continue
            first_heading = math.atan2(
                current.y - previous.y,
                current.x - previous.x,
            )
            second_heading = math.atan2(
                following.y - current.y,
                following.x - current.x,
            )
            turn = abs(angle_difference(second_heading, first_heading))
            lateral = _point_segment_distance(current, previous, following)
            rejoins_quickly = direct_length <= max(
                0.12,
                0.45 * (first_length + second_length),
            )
            if turn >= turn_threshold and lateral >= lateral_threshold and rejoins_quickly:
                drop.add(index)
        if not drop:
            break
        points = [point for index, point in enumerate(points) if index not in drop]
        removed += len(drop)
    return points, removed


def simplify_rdp(samples, tolerance=0.03):
    """Ramer-Douglas-Peucker polyline simplification with bounded XY error."""
    if len(samples) <= 2 or tolerance <= 0.0:
        return list(samples)
    keep = {0, len(samples) - 1}
    stack = [(0, len(samples) - 1)]
    while stack:
        start_index, end_index = stack.pop()
        largest_distance = -1.0
        largest_index = None
        for index in range(start_index + 1, end_index):
            distance = _point_segment_distance(
                samples[index],
                samples[start_index],
                samples[end_index],
            )
            if distance > largest_distance:
                largest_distance = distance
                largest_index = index
        if largest_index is not None and largest_distance > tolerance:
            keep.add(largest_index)
            stack.append((start_index, largest_index))
            stack.append((largest_index, end_index))
    return [samples[index] for index in sorted(keep)]


def _turn_vertices(samples, threshold_deg):
    flags = [False] * len(samples)
    threshold = math.radians(threshold_deg)
    for index in range(1, len(samples) - 1):
        incoming = math.atan2(
            samples[index].y - samples[index - 1].y,
            samples[index].x - samples[index - 1].x,
        )
        outgoing = math.atan2(
            samples[index + 1].y - samples[index].y,
            samples[index + 1].x - samples[index].x,
        )
        flags[index] = abs(angle_difference(outgoing, incoming)) >= threshold
    return flags


def resample_polyline(
    samples,
    normal_spacing=0.20,
    turn_spacing=0.10,
    turn_angle_deg=5.0,
    turn_zone_distance=0.30,
):
    if len(samples) <= 1:
        return list(samples)
    normal_spacing = max(0.01, float(normal_spacing))
    turn_spacing = max(0.01, float(turn_spacing))
    turn_zone_distance = max(0.0, float(turn_zone_distance))
    turns = _turn_vertices(samples, turn_angle_deg)
    output = [samples[0]]

    for index in range(len(samples) - 1):
        start = samples[index]
        end = samples[index + 1]
        length = sample_distance(start, end)
        if length < 0.005:
            continue
        travelled = 0.0
        while travelled < length - 1e-9:
            near_start_turn = turns[index] and travelled < turn_zone_distance
            near_end_turn = turns[index + 1] and length - travelled <= turn_zone_distance
            spacing = turn_spacing if near_start_turn or near_end_turn else normal_spacing
            next_distance = min(length, travelled + spacing)
            point = _interpolate(start, end, next_distance / length)
            if sample_distance(output[-1], point) >= 0.005:
                output.append(point)
            travelled = next_distance

    return output


def recompute_route_yaws(samples):
    if not samples:
        return []
    if len(samples) == 1:
        return list(samples)
    output = []
    for index, sample in enumerate(samples):
        if index < len(samples) - 1:
            other = samples[index + 1]
            yaw = math.atan2(other.y - sample.y, other.x - sample.x)
        else:
            other = samples[index - 1]
            yaw = math.atan2(sample.y - other.y, sample.x - other.x)
        output.append(RouteSample(sample.x, sample.y, yaw))
    return output


def build_clean_route(
    raw_samples,
    normal_spacing=0.20,
    turn_spacing=0.10,
    turn_angle_deg=5.0,
    turn_zone_distance=0.30,
    simplify_tolerance=0.03,
    spike_lateral_threshold=0.06,
):
    """Build a patrol-ready route and return it with quality statistics."""
    finite_count = sum(
        1
        for sample in raw_samples
        if all(math.isfinite(value) for value in (sample.x, sample.y, sample.yaw))
    )
    deduplicated = _deduplicate(raw_samples)
    despiked, removed_spikes = remove_isolated_spikes(
        deduplicated,
        lateral_threshold=spike_lateral_threshold,
    )
    simplified = simplify_rdp(despiked, tolerance=simplify_tolerance)
    resampled = resample_polyline(
        simplified,
        normal_spacing=normal_spacing,
        turn_spacing=turn_spacing,
        turn_angle_deg=turn_angle_deg,
        turn_zone_distance=turn_zone_distance,
    )
    cleaned = recompute_route_yaws(resampled)
    gaps = [
        sample_distance(cleaned[index - 1], cleaned[index])
        for index in range(1, len(cleaned))
    ]
    report = {
        'raw_samples': len(raw_samples),
        'finite_samples': finite_count,
        'deduplicated_samples': len(deduplicated),
        'isolated_spikes_removed': removed_spikes,
        'simplified_vertices': len(simplified),
        'route_points': len(cleaned),
        'max_route_gap_m': max(gaps) if gaps else 0.0,
        'mean_route_gap_m': sum(gaps) / len(gaps) if gaps else 0.0,
        'simplify_tolerance_m': float(simplify_tolerance),
        'straight_spacing_m': float(normal_spacing),
        'turn_spacing_m': float(turn_spacing),
        'turn_angle_deg': float(turn_angle_deg),
    }
    return cleaned, report

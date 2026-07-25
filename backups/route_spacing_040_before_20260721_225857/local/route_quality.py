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


@dataclass
class OdometryGateDecision:
    accepted: bool
    should_fail: bool
    recovered: bool
    reason: str
    step_m: float
    dt_s: float
    speed_mps: float
    allowed_step_m: float
    consecutive_anomalies: int


@dataclass
class SourceGapAssessment:
    gap_m: float
    exceeds_soft_limit: bool
    exceeds_hard_limit: bool
    safe_to_interpolate: bool
    yaw_change_deg: float
    heading_mismatch_deg: float
    reason: str
    incoming_track_mismatch_deg: float = 0.0
    outgoing_track_mismatch_deg: float = 0.0
    neighboring_track_turn_deg: float = 0.0


@dataclass
class LargeGapStreakDecision:
    is_large: bool
    should_fail: bool
    current_streak: int
    max_streak: int
    reason: str


class ConsecutiveLargeGapGate:
    """Fail only after several consecutive large source transitions."""

    def __init__(self, threshold_m=1.50, failure_count=5):
        self.threshold_m = max(0.0, float(threshold_m))
        self.failure_count = max(1, int(failure_count))
        self.reset()

    def reset(self):
        self.current_streak = 0
        self.max_streak = 0

    def evaluate(self, gap_m):
        gap_m = max(0.0, float(gap_m))
        is_large = self.threshold_m > 0.0 and gap_m > self.threshold_m
        if is_large:
            self.current_streak += 1
            self.max_streak = max(self.max_streak, self.current_streak)
        else:
            self.current_streak = 0

        should_fail = is_large and self.current_streak >= self.failure_count
        if should_fail:
            reason = (
                'source gap %.3fm exceeds %.3fm for %d consecutive transitions'
                % (gap_m, self.threshold_m, self.current_streak)
            )
        elif is_large:
            reason = (
                'large source gap streak %d/%d: gap=%.3fm exceeds %.3fm'
                % (
                    self.current_streak,
                    self.failure_count,
                    gap_m,
                    self.threshold_m,
                )
            )
        else:
            reason = 'large source gap streak reset'
        return LargeGapStreakDecision(
            is_large,
            should_fail,
            self.current_streak,
            self.max_streak,
            reason,
        )


class OdometryQualityGate:
    """Time-aware odometry gate with bounded automatic recovery.

    Every finite observation becomes the comparison baseline, including a
    rejected one.  That prevents a single borderline update from pinning the
    recorder forever to an old position.  Recording resumes only after several
    plausible transitions, while sustained anomalies request a hard failure.
    """

    def __init__(
        self,
        base_step_m=0.30,
        max_speed_mps=4.0,
        step_margin_m=0.10,
        hard_speed_mps=6.0,
        hard_step_m=3.0,
        max_gap_s=1.0,
        recovery_samples=3,
        failure_samples=3,
        failure_duration_s=0.50,
    ):
        self.base_step_m = max(0.0, float(base_step_m))
        self.max_speed_mps = max(0.01, float(max_speed_mps))
        self.step_margin_m = max(0.0, float(step_margin_m))
        self.hard_speed_mps = max(self.max_speed_mps, float(hard_speed_mps))
        self.hard_step_m = max(self.base_step_m, float(hard_step_m))
        self.max_gap_s = max(0.01, float(max_gap_s))
        self.recovery_samples = max(1, int(recovery_samples))
        self.failure_samples = max(1, int(failure_samples))
        self.failure_duration_s = max(0.0, float(failure_duration_s))
        self.reset()

    def reset(self):
        self.last_position = None
        self.last_sample_time = None
        self.consecutive_anomalies = 0
        self.anomaly_started_at = None
        self.recovery_count = 0

    def _decision(
        self,
        accepted,
        should_fail,
        recovered,
        reason,
        step_m=0.0,
        dt_s=0.0,
        speed_mps=0.0,
        allowed_step_m=0.0,
    ):
        return OdometryGateDecision(
            accepted=accepted,
            should_fail=should_fail,
            recovered=recovered,
            reason=reason,
            step_m=step_m,
            dt_s=dt_s,
            speed_mps=speed_mps,
            allowed_step_m=allowed_step_m,
            consecutive_anomalies=self.consecutive_anomalies,
        )

    def _reject(
        self,
        sample_time,
        reason,
        step_m,
        dt_s,
        speed_mps,
        allowed_step_m,
    ):
        if self.consecutive_anomalies == 0:
            self.anomaly_started_at = sample_time
        self.consecutive_anomalies += 1
        self.recovery_count = 0
        elapsed = 0.0
        if self.anomaly_started_at is not None:
            elapsed = max(0.0, sample_time - self.anomaly_started_at)
        should_fail = (
            self.consecutive_anomalies >= self.failure_samples
            or (
                self.failure_duration_s > 0.0
                and elapsed >= self.failure_duration_s
            )
        )
        return self._decision(
            False,
            should_fail,
            False,
            reason,
            step_m,
            dt_s,
            speed_mps,
            allowed_step_m,
        )

    def evaluate(self, x, y, z, sample_time):
        values = (x, y, z, sample_time)
        if not all(math.isfinite(value) for value in values):
            safe_time = (
                self.last_sample_time
                if self.last_sample_time is not None
                else 0.0
            )
            return self._reject(
                safe_time,
                'non-finite odometry sample',
                float('inf'),
                0.0,
                float('inf'),
                self.base_step_m,
            )

        position = (float(x), float(y), float(z))
        sample_time = float(sample_time)
        if self.last_position is None or self.last_sample_time is None:
            self.last_position = position
            self.last_sample_time = sample_time
            return self._decision(True, False, False, 'first sample')

        dx = position[0] - self.last_position[0]
        dy = position[1] - self.last_position[1]
        dz = position[2] - self.last_position[2]
        step_m = math.sqrt(dx * dx + dy * dy + dz * dz)
        dt_s = sample_time - self.last_sample_time

        # Always advance the comparison baseline for a finite observation.  A
        # rejected point starts a recovery sequence instead of permanently
        # comparing all future points with the same stale coordinate.
        self.last_position = position
        self.last_sample_time = sample_time

        if dt_s <= 0.0 or dt_s > self.max_gap_s:
            return self._reject(
                sample_time,
                'odometry timestamp gap %.3fs outside (0, %.3f]s'
                % (dt_s, self.max_gap_s),
                step_m,
                dt_s,
                float('inf'),
                self.base_step_m,
            )

        speed_mps = step_m / dt_s
        allowed_step_m = max(
            self.base_step_m,
            self.max_speed_mps * dt_s + self.step_margin_m,
        )
        if (
            step_m > allowed_step_m
            or step_m > self.hard_step_m
            or speed_mps > self.hard_speed_mps
        ):
            return self._reject(
                sample_time,
                (
                    'odometry motion implausible: step=%.3fm, dt=%.3fs, '
                    'speed=%.3fm/s, allowed_step=%.3fm'
                ) % (step_m, dt_s, speed_mps, allowed_step_m),
                step_m,
                dt_s,
                speed_mps,
                allowed_step_m,
            )

        if self.consecutive_anomalies > 0:
            self.recovery_count += 1
            if self.recovery_count < self.recovery_samples:
                return self._decision(
                    False,
                    False,
                    False,
                    'odometry recovering: stable transition %d/%d'
                    % (self.recovery_count, self.recovery_samples),
                    step_m,
                    dt_s,
                    speed_mps,
                    allowed_step_m,
                )
            self.consecutive_anomalies = 0
            self.anomaly_started_at = None
            self.recovery_count = 0
            return self._decision(
                True,
                False,
                True,
                'odometry recovered after stable transitions',
                step_m,
                dt_s,
                speed_mps,
                allowed_step_m,
            )

        return self._decision(
            True,
            False,
            False,
            'odometry accepted',
            step_m,
            dt_s,
            speed_mps,
            allowed_step_m,
        )


def angle_difference(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


def sample_distance(a, b):
    return math.hypot(b.x - a.x, b.y - a.y)


def assess_source_gap(
    previous,
    current,
    soft_gap_m=0.80,
    hard_gap_m=3.00,
    max_yaw_change_deg=15.0,
    max_heading_mismatch_deg=25.0,
    incoming_heading=None,
    outgoing_heading=None,
    max_track_heading_mismatch_deg=25.0,
):
    """Decide whether a missing section can safely be replaced by a chord.

    The limits apply to consecutive accepted source observations, never to the
    distance accumulated between two down-sampled CSV points. A gap above the
    soft limit is usable only when its chord follows available neighboring XY
    motion. Body yaw is diagnostic when that XY context exists. The hard limit
    always rejects the gap.
    """
    gap_m = sample_distance(previous, current)
    soft_gap_m = max(0.0, float(soft_gap_m))
    hard_gap_m = float(hard_gap_m)
    yaw_change_deg = math.degrees(
        abs(angle_difference(current.yaw, previous.yaw))
    )
    segment_heading = math.atan2(
        current.y - previous.y,
        current.x - previous.x,
    )
    average_yaw = previous.yaw + 0.5 * angle_difference(
        current.yaw,
        previous.yaw,
    )
    heading_mismatch_deg = math.degrees(
        abs(angle_difference(segment_heading, average_yaw))
    )
    incoming_track_mismatch_deg = 0.0
    outgoing_track_mismatch_deg = 0.0
    track_checks = []
    if incoming_heading is not None:
        incoming_track_mismatch_deg = math.degrees(
            abs(angle_difference(segment_heading, incoming_heading))
        )
        track_checks.append(('incoming', incoming_track_mismatch_deg))
    if outgoing_heading is not None:
        outgoing_track_mismatch_deg = math.degrees(
            abs(angle_difference(segment_heading, outgoing_heading))
        )
        track_checks.append(('outgoing', outgoing_track_mismatch_deg))
    neighboring_track_turn_deg = 0.0
    if incoming_heading is not None and outgoing_heading is not None:
        neighboring_track_turn_deg = math.degrees(
            abs(angle_difference(outgoing_heading, incoming_heading))
        )
    exceeds_soft_limit = soft_gap_m > 0.0 and gap_m > soft_gap_m
    exceeds_hard_limit = hard_gap_m > 0.0 and gap_m > hard_gap_m

    if exceeds_hard_limit:
        return SourceGapAssessment(
            gap_m,
            exceeds_soft_limit,
            True,
            False,
            yaw_change_deg,
            heading_mismatch_deg,
            'source gap %.3fm exceeds hard limit %.3fm'
            % (gap_m, hard_gap_m),
            incoming_track_mismatch_deg,
            outgoing_track_mismatch_deg,
            neighboring_track_turn_deg,
        )

    if not exceeds_soft_limit:
        return SourceGapAssessment(
            gap_m,
            False,
            False,
            True,
            yaw_change_deg,
            heading_mismatch_deg,
            'source gap is within the soft interpolation limit',
            incoming_track_mismatch_deg,
            outgoing_track_mismatch_deg,
            neighboring_track_turn_deg,
        )

    yaw_ok = yaw_change_deg <= float(max_yaw_change_deg)
    if track_checks:
        heading_ok = all(
            mismatch <= float(max_track_heading_mismatch_deg)
            for _, mismatch in track_checks
        )
        if incoming_heading is not None and outgoing_heading is not None:
            heading_ok = (
                heading_ok
                and neighboring_track_turn_deg
                <= float(max_track_heading_mismatch_deg)
            )
    else:
        heading_ok = heading_mismatch_deg <= float(max_heading_mismatch_deg)
    # With neighboring XY context, body yaw is diagnostic only: the Go2 can
    # translate sideways while its nose remains fixed. Without XY context,
    # yaw remains a conservative fallback for an otherwise unknowable gap.
    safe_to_interpolate = heading_ok and (yaw_ok or bool(track_checks))
    if safe_to_interpolate:
        reason = (
            'large source gap follows the local trajectory: '
            'yaw_change=%.1fdeg, body_heading_mismatch=%.1fdeg'
            % (yaw_change_deg, heading_mismatch_deg)
        )
    else:
        failed_checks = []
        if not yaw_ok:
            if not track_checks:
                failed_checks.append(
                    'yaw_change %.1fdeg exceeds %.1fdeg'
                    % (yaw_change_deg, float(max_yaw_change_deg))
                )
        if not heading_ok and track_checks:
            for label, mismatch in track_checks:
                if mismatch > float(max_track_heading_mismatch_deg):
                    failed_checks.append(
                        '%s_track_mismatch %.1fdeg exceeds %.1fdeg'
                        % (
                            label,
                            mismatch,
                            float(max_track_heading_mismatch_deg),
                        )
                    )
        elif not heading_ok:
            failed_checks.append(
                'body_heading_mismatch %.1fdeg exceeds %.1fdeg'
                % (heading_mismatch_deg, float(max_heading_mismatch_deg))
            )
        if (
            not heading_ok
            and incoming_heading is not None
            and outgoing_heading is not None
            and neighboring_track_turn_deg
            > float(max_track_heading_mismatch_deg)
        ):
            failed_checks.append(
                'neighboring_track_turn %.1fdeg exceeds %.1fdeg'
                % (
                    neighboring_track_turn_deg,
                    float(max_track_heading_mismatch_deg),
                )
            )
        reason = 'large source gap cannot be safely interpolated: ' + ', '.join(
            failed_checks
        )
    return SourceGapAssessment(
        gap_m,
        True,
        False,
        safe_to_interpolate,
        yaw_change_deg,
        heading_mismatch_deg,
        reason,
        incoming_track_mismatch_deg,
        outgoing_track_mismatch_deg,
        neighboring_track_turn_deg,
    )


def neighbor_track_heading(samples, anchor_index, direction, minimum_distance=0.05):
    anchor = samples[anchor_index]
    index = anchor_index + direction
    while 0 <= index < len(samples):
        other = samples[index]
        if sample_distance(anchor, other) >= minimum_distance:
            if direction < 0:
                return math.atan2(anchor.y - other.y, anchor.x - other.x)
            return math.atan2(other.y - anchor.y, other.x - anchor.x)
        index += direction
    return None


def analyze_source_gaps(
    samples,
    soft_gap_m=0.80,
    hard_gap_m=3.00,
    max_yaw_change_deg=15.0,
    max_track_heading_mismatch_deg=25.0,
):
    """Audit source gaps using neighboring XY motion, not body yaw alone."""
    large_count = 0
    unsafe = []
    for index in range(1, len(samples)):
        incoming_heading = neighbor_track_heading(samples, index - 1, -1)
        outgoing_heading = neighbor_track_heading(samples, index, 1)
        assessment = assess_source_gap(
            samples[index - 1],
            samples[index],
            soft_gap_m=soft_gap_m,
            hard_gap_m=hard_gap_m,
            max_yaw_change_deg=max_yaw_change_deg,
            # A holonomic Go2 may crab sideways while body yaw stays fixed.
            # Body-vs-track mismatch remains diagnostic only when XY context
            # exists, while neighboring trajectory headings decide continuity.
            max_heading_mismatch_deg=180.0,
            incoming_heading=incoming_heading,
            outgoing_heading=outgoing_heading,
            max_track_heading_mismatch_deg=max_track_heading_mismatch_deg,
        )
        if assessment.exceeds_soft_limit:
            large_count += 1
        if not assessment.safe_to_interpolate:
            unsafe.append((index - 1, index, assessment))
    return {
        'large_source_gap_count': large_count,
        'unsafe_source_gap_count': len(unsafe),
        'max_unsafe_source_gap_m': max(
            (item[2].gap_m for item in unsafe),
            default=0.0,
        ),
        'unsafe_source_gaps': unsafe,
    }


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


def _circular_median(angles):
    if not angles:
        return 0.0
    anchor = angles[0]
    unwrapped = sorted(anchor + angle_difference(angle, anchor) for angle in angles)
    middle = len(unwrapped) // 2
    if len(unwrapped) % 2:
        return unwrapped[middle]
    return 0.5 * (unwrapped[middle - 1] + unwrapped[middle])


def _run_path_length(samples, start, end):
    return sum(
        sample_distance(samples[index - 1], samples[index])
        for index in range(start + 1, end + 1)
    )


def _stable_yaw_runs(samples, tolerance_deg, correction_merge_distance):
    if not samples:
        return []
    tolerance = math.radians(max(0.1, float(tolerance_deg)))
    runs = []
    start = 0
    sum_cos = math.cos(samples[0].yaw)
    sum_sin = math.sin(samples[0].yaw)
    reference = samples[0].yaw
    for index in range(1, len(samples)):
        yaw = samples[index].yaw
        if abs(angle_difference(yaw, reference)) <= tolerance:
            sum_cos += math.cos(yaw)
            sum_sin += math.sin(yaw)
            reference = math.atan2(sum_sin, sum_cos)
            continue
        runs.append([start, index - 1])
        start = index
        sum_cos = math.cos(yaw)
        sum_sin = math.sin(yaw)
        reference = yaw
    runs.append([start, len(samples) - 1])

    # A short yaw excursion that returns to the same heading is a steering
    # correction, not a new route corner. Merge the two straight runs around it.
    changed = True
    while changed and len(runs) >= 3:
        changed = False
        merged = []
        index = 0
        while index < len(runs):
            if index + 2 < len(runs):
                first = runs[index]
                third = runs[index + 2]
                first_yaw = _circular_median(
                    [sample.yaw for sample in samples[first[0]:first[1] + 1]]
                )
                third_yaw = _circular_median(
                    [sample.yaw for sample in samples[third[0]:third[1] + 1]]
                )
                middle_length = _run_path_length(
                    samples,
                    first[1],
                    third[0],
                )
                if (
                    abs(angle_difference(first_yaw, third_yaw)) <= tolerance
                    and middle_length <= float(correction_merge_distance)
                ):
                    merged.append([first[0], third[1]])
                    index += 3
                    changed = True
                    continue
            merged.append(runs[index])
            index += 1
        runs = merged
    return runs


def straighten_stable_yaw_sections(
    samples,
    yaw_tolerance_deg=4.0,
    min_section_length=1.0,
    max_lateral_correction=0.35,
    correction_merge_distance=0.60,
):
    """Project stable body-heading sections onto their intended straight line.

    A Go2 can crab sideways while its body yaw remains fixed.  For a sufficiently
    long, stable-yaw section, the body heading is treated as the operator's
    intended straight direction and bounded lateral drift is removed. Sustained
    heading changes and curves remain separate sections.
    """
    points = list(samples)
    output = list(points)
    report = {
        'straightened_sections': 0,
        'straightened_samples': 0,
        'straightening_skipped_sections': 0,
        'max_lateral_correction_m': 0.0,
    }
    if len(points) < 2:
        return output, report

    runs = _stable_yaw_runs(
        points,
        yaw_tolerance_deg,
        correction_merge_distance,
    )
    for start, end in runs:
        section_length = _run_path_length(points, start, end)
        if section_length < float(min_section_length):
            continue
        reference_yaw = _circular_median(
            [sample.yaw for sample in points[start:end + 1]]
        )
        ux = math.cos(reference_yaw)
        uy = math.sin(reference_yaw)
        anchor = points[start]
        corrections = []
        projected = []
        for index in range(start, end + 1):
            sample = points[index]
            dx = sample.x - anchor.x
            dy = sample.y - anchor.y
            longitudinal = dx * ux + dy * uy
            lateral = -dx * uy + dy * ux
            corrections.append(abs(lateral))
            projected.append(RouteSample(
                anchor.x + longitudinal * ux,
                anchor.y + longitudinal * uy,
                sample.yaw,
            ))

        maximum_correction = max(corrections, default=0.0)
        if maximum_correction > float(max_lateral_correction):
            report['straightening_skipped_sections'] += 1
            continue
        output[start:end + 1] = projected
        report['straightened_sections'] += 1
        report['straightened_samples'] += end - start + 1
        report['max_lateral_correction_m'] = max(
            report['max_lateral_correction_m'],
            maximum_correction,
        )
    return output, report


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
    straighten_stable_sections=True,
    straight_yaw_tolerance_deg=4.0,
    straight_min_length=1.0,
    straight_max_lateral_correction=0.35,
    straight_correction_merge_distance=0.60,
):
    """Build a patrol-ready route and return it with quality statistics."""
    finite_count = sum(
        1
        for sample in raw_samples
        if all(math.isfinite(value) for value in (sample.x, sample.y, sample.yaw))
    )
    finite_samples = [
        sample
        for sample in raw_samples
        if all(math.isfinite(value) for value in (sample.x, sample.y, sample.yaw))
    ]
    straighten_report = {
        'straightened_sections': 0,
        'straightened_samples': 0,
        'straightening_skipped_sections': 0,
        'max_lateral_correction_m': 0.0,
    }
    if straighten_stable_sections:
        finite_samples, straighten_report = straighten_stable_yaw_sections(
            finite_samples,
            yaw_tolerance_deg=straight_yaw_tolerance_deg,
            min_section_length=straight_min_length,
            max_lateral_correction=straight_max_lateral_correction,
            correction_merge_distance=straight_correction_merge_distance,
        )
    deduplicated = _deduplicate(finite_samples)
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
    raw_gaps = [
        sample_distance(deduplicated[index - 1], deduplicated[index])
        for index in range(1, len(deduplicated))
    ]
    gaps = [
        sample_distance(cleaned[index - 1], cleaned[index])
        for index in range(1, len(cleaned))
    ]
    route_length = sum(gaps)
    report = {
        'raw_samples': len(raw_samples),
        'finite_samples': finite_count,
        'deduplicated_samples': len(deduplicated),
        'isolated_spikes_removed': removed_spikes,
        'simplified_vertices': len(simplified),
        'route_points': len(cleaned),
        'route_length_m': route_length,
        'max_raw_gap_m': max(raw_gaps) if raw_gaps else 0.0,
        'max_route_gap_m': max(gaps) if gaps else 0.0,
        'mean_route_gap_m': sum(gaps) / len(gaps) if gaps else 0.0,
        'simplify_tolerance_m': float(simplify_tolerance),
        'straight_spacing_m': float(normal_spacing),
        'turn_spacing_m': float(turn_spacing),
        'turn_angle_deg': float(turn_angle_deg),
        'straightening_enabled': bool(straighten_stable_sections),
        'straight_yaw_tolerance_deg': float(straight_yaw_tolerance_deg),
        'straight_min_length_m': float(straight_min_length),
        'straight_max_lateral_correction_limit_m': float(
            straight_max_lateral_correction
        ),
    }
    report.update(straighten_report)
    return cleaned, report


def validate_route_report(
    report,
    min_route_points=5,
    min_route_length_m=1.0,
    max_rejection_ratio=0.25,
    max_raw_gap_m=3.00,
    runtime_failure='',
):
    """Return human-readable reasons that a recorded route is not usable."""
    reasons = []
    if runtime_failure:
        reasons.append(str(runtime_failure))
    route_points = int(report.get('route_points', 0))
    route_length = float(report.get('route_length_m', 0.0))
    rejection_ratio = float(report.get('rejection_ratio', 0.0))
    source_gap = float(report.get('max_raw_gap_m', 0.0))
    unsafe_source_gaps = int(report.get('unsafe_source_gap_count', 0))
    max_unsafe_source_gap = float(
        report.get('max_unsafe_source_gap_m', 0.0)
    )
    if route_points < int(min_route_points):
        reasons.append(
            'route has %d points; minimum is %d'
            % (route_points, int(min_route_points))
        )
    if route_length < float(min_route_length_m):
        reasons.append(
            'route length %.3fm is below minimum %.3fm'
            % (route_length, float(min_route_length_m))
        )
    if rejection_ratio > float(max_rejection_ratio):
        reasons.append(
            'odometry rejection ratio %.1f%% exceeds %.1f%%'
            % (
                rejection_ratio * 100.0,
                float(max_rejection_ratio) * 100.0,
            )
        )
    if source_gap > float(max_raw_gap_m):
        reasons.append(
            'accepted source gap %.3fm exceeds %.3fm'
            % (source_gap, float(max_raw_gap_m))
        )
    if unsafe_source_gaps > 0:
        reasons.append(
            '%d large source gap(s) cannot be safely interpolated; max=%.3fm'
            % (unsafe_source_gaps, max_unsafe_source_gap)
        )
    return reasons

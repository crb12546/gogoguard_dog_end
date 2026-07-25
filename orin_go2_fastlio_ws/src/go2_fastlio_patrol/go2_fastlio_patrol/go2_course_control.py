#!/usr/bin/env python3
"""Small geometry helpers for Go2_2 motion-course feedback.

The original Go2_2 follower assumes that commanded forward velocity follows
the robot's instantaneous body yaw.  A quadruped can temporarily crab after a
turn, so body yaw alone can say "aligned" while the travelled path remains
parallel to the CSV.  This module measures the travelled course and projects
the robot onto the nearby route segment.  Straight travel uses that one
feedback source continuously; body-yaw feedback is reserved for corners and
the short period before a travelled course is measurable.
"""

from dataclasses import dataclass
import math


def normalize_angle(value):
    return math.atan2(math.sin(value), math.cos(value))


@dataclass
class RouteProjection:
    segment_index: int
    ratio: float
    x: float
    y: float
    distance: float
    signed_distance: float
    route_heading: float


def _xy(point):
    if isinstance(point, dict):
        return float(point['x']), float(point['y'])
    return float(point[0]), float(point[1])


def closest_route_projection(
    route,
    x,
    y,
    center_index,
    search_window,
    direction=1,
):
    """Project a pose onto nearby route segments.

    ``signed_distance`` is positive on the left side of the direction of
    travel and negative on the right side.
    """
    if len(route) < 2:
        raise ValueError('route must contain at least two points')

    first_segment = max(0, int(center_index) - int(search_window) - 1)
    last_segment = min(
        len(route) - 2,
        int(center_index) + int(search_window),
    )
    best = None
    for segment_index in range(first_segment, last_segment + 1):
        ax, ay = _xy(route[segment_index])
        bx, by = _xy(route[segment_index + 1])
        dx = bx - ax
        dy = by - ay
        length_squared = dx * dx + dy * dy
        if length_squared <= 1e-12:
            continue
        ratio = ((x - ax) * dx + (y - ay) * dy) / length_squared
        ratio = min(1.0, max(0.0, ratio))
        projection_x = ax + ratio * dx
        projection_y = ay + ratio * dy
        offset_x = float(x) - projection_x
        offset_y = float(y) - projection_y
        distance = math.hypot(offset_x, offset_y)
        route_heading = math.atan2(dy, dx)
        if int(direction) < 0:
            route_heading = normalize_angle(route_heading + math.pi)
        signed_distance = (
            -offset_x * math.sin(route_heading)
            + offset_y * math.cos(route_heading)
        )
        candidate = RouteProjection(
            segment_index=segment_index,
            ratio=ratio,
            x=projection_x,
            y=projection_y,
            distance=distance,
            signed_distance=signed_distance,
            route_heading=route_heading,
        )
        if best is None or candidate.distance < best.distance:
            best = candidate

    if best is None:
        raise ValueError('route contains no non-zero segments')
    return best


def route_heading_at(route, index, direction=1):
    if len(route) < 2:
        raise ValueError('route must contain at least two points')
    index = min(len(route) - 1, max(0, int(index)))
    if int(direction) >= 0:
        start = min(index, len(route) - 2)
        end = start + 1
    else:
        end = max(index, 1)
        start = end - 1
    ax, ay = _xy(route[start])
    bx, by = _xy(route[end])
    heading = math.atan2(by - ay, bx - ax)
    if int(direction) < 0:
        heading = normalize_angle(heading + math.pi)
    return heading


def bounded_target_course(route_heading, target_angle, maximum_angle):
    correction = normalize_angle(target_angle - route_heading)
    correction = max(
        -abs(float(maximum_angle)),
        min(abs(float(maximum_angle)), correction),
    )
    return normalize_angle(route_heading + correction)


def straight_target_course(
    route_heading,
    target_angle,
    lateral_distance,
    deadband,
    maximum_angle,
):
    """Return one continuous straight-line target course."""
    # Inside the deadband, follow the CSV tangent without lateral correction.
    # Outside it, use the lookahead target for a bounded intercept angle.  The
    # deadband changes only that term and never changes the feedback source.
    if abs(float(lateral_distance)) <= max(0.0, float(deadband)):
        return normalize_angle(route_heading)
    return bounded_target_course(
        route_heading,
        target_angle,
        maximum_angle,
    )


def use_straight_course_feedback(
    enabled,
    course_valid,
    route_turn,
    maximum_route_turn,
    body_alpha,
    turn_in_place_angle,
):
    """Choose course feedback for a straight, never from lateral error."""
    return (
        bool(enabled)
        and bool(course_valid)
        and abs(float(route_turn)) <= abs(float(maximum_route_turn))
        and abs(float(body_alpha)) <= abs(float(turn_in_place_angle))
    )


class MotionCourseEstimator:
    """Distance-windowed and circularly smoothed travelled direction."""

    def __init__(
        self,
        minimum_distance=0.12,
        smoothing=0.60,
        maximum_age=0.80,
    ):
        self.minimum_distance = max(0.01, float(minimum_distance))
        self.smoothing = min(1.0, max(0.01, float(smoothing)))
        self.maximum_age = max(0.05, float(maximum_age))
        self.anchor_x = None
        self.anchor_y = None
        self.course = None
        self.last_update_time = None

    def update(self, x, y, now):
        x = float(x)
        y = float(y)
        now = float(now)
        if self.anchor_x is None:
            self.anchor_x = x
            self.anchor_y = y
            return False
        dx = x - self.anchor_x
        dy = y - self.anchor_y
        if math.hypot(dx, dy) < self.minimum_distance:
            return False
        measured = math.atan2(dy, dx)
        if self.course is None:
            self.course = measured
        else:
            delta = normalize_angle(measured - self.course)
            self.course = normalize_angle(
                self.course + self.smoothing * delta
            )
        self.anchor_x = x
        self.anchor_y = y
        self.last_update_time = now
        return True

    def valid(self, now):
        return (
            self.course is not None
            and self.last_update_time is not None
            and float(now) - self.last_update_time <= self.maximum_age
        )

#!/usr/bin/env python3
"""Pure planar-motion helpers shared by follower and safety nodes."""

import math


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def lateral_velocity_command(
    lateral_error,
    route_yaw,
    current_yaw,
    gain=0.50,
    max_vy=0.12,
    deadband=0.03,
    heading_limit_deg=35.0,
):
    """Return body-frame ``vy`` that removes route cross-track error.

    Positive route lateral error means the robot is left of the route, so the
    correction points right.  The correction is disabled when body heading is
    far from route heading, such as during an in-place turn or recovery.
    """
    lateral_error = float(lateral_error)
    deadband = max(0.0, float(deadband))
    max_vy = max(0.0, float(max_vy))
    if max_vy <= 0.0 or abs(lateral_error) <= deadband:
        return 0.0

    heading_error = normalize_angle(float(route_yaw) - float(current_yaw))
    if abs(heading_error) > math.radians(max(0.0, float(heading_limit_deg))):
        return 0.0

    effective_error = math.copysign(
        abs(lateral_error) - deadband,
        lateral_error,
    )
    route_lateral_velocity = -max(0.0, float(gain)) * effective_error
    # Rotate the route-normal correction into the robot body frame. This is
    # exactly route_lateral_velocity when the body is aligned with the route.
    body_vy = route_lateral_velocity * math.cos(heading_error)
    return max(-max_vy, min(max_vy, body_vy))


def limit_planar_command(
    vx,
    vy,
    yaw_rate,
    max_vx,
    max_vy,
    max_yaw_rate,
    enabled=True,
):
    """Clamp a command, or return a complete zero command for a safety stop."""
    if not enabled:
        return 0.0, 0.0, 0.0

    def clamp(value, limit):
        limit = max(0.0, float(limit))
        return max(-limit, min(limit, float(value)))

    return (
        clamp(vx, max_vx),
        clamp(vy, max_vy),
        clamp(yaw_rate, max_yaw_rate),
    )


def point_in_lateral_motion_roi(
    x,
    y,
    z,
    vy,
    cmd_deadband=0.02,
    x_min=0.10,
    x_max=1.00,
    inner_y=0.30,
    outer_y=0.65,
    z_min=0.25,
    z_max=0.90,
):
    """Return whether a point blocks the side selected by body-frame ``vy``.

    The regular obstacle ROI protects forward motion.  This second ROI is
    active only while a lateral command is present and covers the selected
    side/front swept area.  The inner Y boundary keeps the robot body out of
    the test; positive body Y and positive ``vy`` are the left side.
    """
    vy = float(vy)
    if abs(vy) <= max(0.0, float(cmd_deadband)):
        return False
    if not (
        float(x_min) <= float(x) <= float(x_max)
        and float(z_min) <= float(z) <= float(z_max)
    ):
        return False

    inner_y = max(0.0, float(inner_y))
    outer_y = max(inner_y, float(outer_y))
    y = float(y)
    if vy > 0.0:
        return inner_y <= y <= outer_y
    return -outer_y <= y <= -inner_y

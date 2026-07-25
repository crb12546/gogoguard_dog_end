#!/usr/bin/env python3
"""Pure planar-motion helpers shared by follower and safety nodes."""

import math


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def blend_angles(primary, auxiliary, auxiliary_weight):
    """Blend two wrapped angles without crossing the +/-pi discontinuity."""
    weight = max(0.0, min(1.0, float(auxiliary_weight)))
    x = (1.0 - weight) * math.cos(primary) + weight * math.cos(auxiliary)
    y = (1.0 - weight) * math.sin(primary) + weight * math.sin(auxiliary)
    if abs(x) < 1e-9 and abs(y) < 1e-9:
        return normalize_angle(primary)
    return math.atan2(y, x)


def corner_heading_command(
    target_alpha,
    outgoing_heading_error,
    normal_blend=0.30,
    conflict_blend=0.65,
):
    """Keep a sharp turn committed when pure pursuit briefly points backward.

    Near a vertex, the lookahead target can cross to the other side of the
    robot before its body has reached the outgoing heading.  In that conflict
    case the outgoing CSV tangent must remain the stronger signal; otherwise
    the robot counter-steers in the middle of the corner.
    """
    target_alpha = normalize_angle(float(target_alpha))
    outgoing_heading_error = normalize_angle(float(outgoing_heading_error))
    weight = max(0.0, min(1.0, float(normal_blend)))
    if target_alpha * outgoing_heading_error <= 0.0 and abs(outgoing_heading_error) > 1e-6:
        weight = max(weight, max(0.0, min(1.0, float(conflict_blend))))
    return blend_angles(target_alpha, outgoing_heading_error, weight)


def cumulative_turn_candidate(turn_samples, threshold, min_samples=2):
    """Find a same-direction cumulative turn in ordered route boundaries.

    Each sample is ``(boundary_s, distance, signed_angle)`` in traversal
    order.  This detects a genuine recorded arc made of several modest angle
    changes while naturally rejecting an opposing left/right wiggle.
    """
    threshold = max(0.0, float(threshold))
    min_samples = max(1, int(min_samples))
    best_angle = 0.0
    run_sum = 0.0
    run_sign = 0.0
    run_start_s = -1.0
    run_start_distance = -1.0
    run_count = 0

    for boundary_s, distance, signed_angle in turn_samples:
        signed_angle = normalize_angle(float(signed_angle))
        if abs(signed_angle) <= 1e-6:
            continue
        sign = 1.0 if signed_angle > 0.0 else -1.0
        if run_sign == 0.0 or sign != run_sign:
            run_sum = signed_angle
            run_sign = sign
            run_start_s = float(boundary_s)
            run_start_distance = max(0.0, float(distance))
            run_count = 1
        else:
            run_sum += signed_angle
            run_count += 1

        angle = abs(run_sum)
        best_angle = max(best_angle, angle)
        if run_count >= min_samples and angle >= threshold:
            return angle, run_start_s, run_start_distance, True

    return best_angle, -1.0, -1.0, False


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


def lateral_velocity_limit(max_vy, pid_enabled, pid_warm, warmup_max_vy):
    """Keep the proven lateral limit during PID startup and corner exit."""
    max_vy = max(0.0, float(max_vy))
    if bool(pid_enabled) and not bool(pid_warm):
        return min(max_vy, max(0.0, float(warmup_max_vy)))
    return max_vy


def lateral_pid_warmup_ready(
    current_s,
    start_s,
    direction,
    warmup_distance,
    elapsed,
    warmup_seconds,
    progress_recent=True,
):
    """Enable full lateral PID only after real route progress.

    A positive ``warmup_distance`` takes precedence over the legacy wall-clock
    delay.  This prevents an obstacle stop from consuming the complete
    post-corner protection period while the chassis is stationary.
    """
    if start_s is None or not bool(progress_recent):
        return False

    warmup_distance = max(0.0, float(warmup_distance))
    if warmup_distance > 0.0:
        direction = 1.0 if float(direction) >= 0.0 else -1.0
        directed_progress = direction * (float(current_s) - float(start_s))
        return directed_progress >= warmup_distance

    return max(0.0, float(elapsed)) >= max(0.0, float(warmup_seconds))


def lateral_velocity_yaw_limit(max_vy, yaw_rate, slow_yaw_rate, stop_yaw_rate):
    """Reduce lateral motion while a strong heading correction is active."""
    max_vy = max(0.0, float(max_vy))
    yaw_rate = abs(float(yaw_rate))
    slow_yaw_rate = max(0.0, float(slow_yaw_rate))
    stop_yaw_rate = max(slow_yaw_rate, float(stop_yaw_rate))

    if max_vy <= 0.0 or yaw_rate >= stop_yaw_rate:
        return 0.0
    if yaw_rate <= slow_yaw_rate or stop_yaw_rate <= slow_yaw_rate:
        return max_vy

    scale = (stop_yaw_rate - yaw_rate) / (stop_yaw_rate - slow_yaw_rate)
    return max_vy * max(0.0, min(1.0, scale))


def curve_lateral_taper_scale(curve_distance, taper_start_distance, zero_distance):
    """Return a 0..1 scale that removes lateral motion before a curve.

    ``curve_distance`` is the remaining route distance to the detected curve
    boundary.  A negative distance means the boundary is no longer ahead, so
    lateral motion must already be zero.
    """
    curve_distance = float(curve_distance)
    zero_distance = max(0.0, float(zero_distance))
    taper_start_distance = max(zero_distance, float(taper_start_distance))

    if curve_distance < 0.0 or curve_distance <= zero_distance:
        return 0.0
    if curve_distance >= taper_start_distance:
        return 1.0
    if taper_start_distance <= zero_distance:
        return 0.0

    scale = (
        (curve_distance - zero_distance)
        / (taper_start_distance - zero_distance)
    )
    return max(0.0, min(1.0, scale))


def smooth_lateral_velocity(desired, previous, accel_rate, decel_rate, dt):
    """Rate-limit ``vy`` with faster braking and no direct sign reversal."""
    desired = float(desired)
    previous = float(previous)
    dt = max(0.0, float(dt))
    accel_rate = max(0.0, float(accel_rate))
    decel_rate = max(accel_rate, float(decel_rate))

    if previous * desired < 0.0:
        desired = 0.0

    increasing_magnitude = (
        abs(desired) > abs(previous)
        and (previous == 0.0 or previous * desired > 0.0)
    )
    rate = accel_rate if increasing_magnitude else decel_rate
    if rate <= 0.0:
        return desired

    max_step = rate * dt
    delta = max(-max_step, min(max_step, desired - previous))
    result = previous + delta
    if desired == 0.0 and previous * result < 0.0:
        return 0.0
    return result


def lateral_pid_command(
    lateral_error,
    route_yaw,
    current_yaw,
    dt,
    integral_error=0.0,
    previous_error=None,
    filtered_derivative=0.0,
    kp=0.75,
    ki=0.08,
    kd=0.04,
    max_vy=0.15,
    deadband=0.03,
    heading_limit_deg=25.0,
    integral_vy_limit=0.04,
    derivative_vy_limit=0.02,
    derivative_filter_tau=0.35,
    integral_reset_error=0.06,
    integrate=True,
):
    """Return a bounded body-frame lateral PID command and updated state.

    The proportional term is exactly the existing lateral controller.  The
    integral term only removes a persistent straight-line bias and is bounded
    independently from the final ``max_vy`` clamp.  The derivative term is
    low-pass filtered and separately bounded so odometry noise cannot create a
    lateral kick.

    The caller owns the state and decides when PID is allowed.  Heading-limit
    violations, deadband entry, and error sign changes clear state here so a
    correction from one side of the route cannot carry over to the other side.
    """
    lateral_error = float(lateral_error)
    max_vy = max(0.0, float(max_vy))
    deadband = max(0.0, float(deadband))
    dt = max(1e-3, min(0.5, float(dt)))

    def inactive_result():
        return {
            'command': 0.0,
            'integral_error': 0.0,
            'previous_error': None,
            'filtered_derivative': 0.0,
            'p_term': 0.0,
            'i_term': 0.0,
            'd_term': 0.0,
            'active': False,
        }

    if max_vy <= 0.0 or abs(lateral_error) <= deadband:
        return inactive_result()

    heading_error = normalize_angle(float(route_yaw) - float(current_yaw))
    if abs(heading_error) > math.radians(max(0.0, float(heading_limit_deg))):
        return inactive_result()

    effective_error = math.copysign(
        abs(lateral_error) - deadband,
        lateral_error,
    )
    # Positive control error means body-left velocity is required.  Rotating
    # the route-normal correction into the body frame preserves the previous
    # proportional controller exactly.
    control_error = -effective_error * math.cos(heading_error)

    sign_changed = (
        previous_error is not None
        and control_error * float(previous_error) < 0.0
    )
    if previous_error is None or sign_changed:
        filtered_derivative = 0.0
    else:
        raw_derivative = (control_error - float(previous_error)) / dt
        filter_tau = max(0.0, float(derivative_filter_tau))
        alpha = 1.0 if filter_tau <= 0.0 else dt / (filter_tau + dt)
        filtered_derivative += alpha * (
            raw_derivative - float(filtered_derivative)
        )

    kp = max(0.0, float(kp))
    ki = max(0.0, float(ki))
    kd = max(0.0, float(kd))
    integral_vy_limit = max(0.0, float(integral_vy_limit))
    derivative_vy_limit = max(0.0, float(derivative_vy_limit))

    p_term = kp * control_error
    d_term = kd * float(filtered_derivative)
    if derivative_vy_limit > 0.0:
        d_term = max(-derivative_vy_limit, min(derivative_vy_limit, d_term))
    else:
        d_term = 0.0

    integral_reset_error = max(deadband, float(integral_reset_error))
    if sign_changed or abs(lateral_error) <= integral_reset_error:
        integral_error = 0.0

    if integrate and ki > 0.0 and abs(lateral_error) > integral_reset_error:
        candidate_integral = float(integral_error) + control_error * dt
        if integral_vy_limit > 0.0:
            max_integral_error = integral_vy_limit / ki
            candidate_integral = max(
                -max_integral_error,
                min(max_integral_error, candidate_integral),
            )
        else:
            candidate_integral = 0.0

        candidate_i_term = ki * candidate_integral
        candidate_unclamped = p_term + candidate_i_term + d_term
        pushes_positive_saturation = (
            candidate_unclamped > max_vy and control_error > 0.0
        )
        pushes_negative_saturation = (
            candidate_unclamped < -max_vy and control_error < 0.0
        )
        if not (pushes_positive_saturation or pushes_negative_saturation):
            integral_error = candidate_integral

    i_term = ki * float(integral_error)
    if integral_vy_limit > 0.0:
        i_term = max(-integral_vy_limit, min(integral_vy_limit, i_term))
    else:
        i_term = 0.0

    command = max(-max_vy, min(max_vy, p_term + i_term + d_term))
    return {
        'command': command,
        'integral_error': float(integral_error),
        'previous_error': control_error,
        'filtered_derivative': float(filtered_derivative),
        'p_term': p_term,
        'i_term': i_term,
        'd_term': d_term,
        'active': True,
    }


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

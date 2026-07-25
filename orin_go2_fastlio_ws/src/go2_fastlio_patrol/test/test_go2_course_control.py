import math

from go2_fastlio_patrol.go2_course_control import (
    MotionCourseEstimator,
    bounded_target_course,
    closest_route_projection,
    straight_target_course,
    use_straight_course_feedback,
)


def test_projection_sign_follows_travel_direction():
    route = [
        {'x': 0.0, 'y': 0.0},
        {'x': 2.0, 'y': 0.0},
    ]
    forward = closest_route_projection(route, 1.0, -0.2, 0, 2, 1)
    backward = closest_route_projection(route, 1.0, -0.2, 1, 2, -1)
    assert math.isclose(forward.distance, 0.2)
    assert math.isclose(forward.signed_distance, -0.2)
    assert math.isclose(backward.signed_distance, 0.2)


def test_straight_course_selection_does_not_depend_on_lateral_error():
    assert use_straight_course_feedback(
        enabled=True,
        course_valid=True,
        route_turn=0.0,
        maximum_route_turn=math.radians(20.0),
        body_alpha=0.0,
        turn_in_place_angle=1.0,
    )


def test_corner_and_unmeasured_course_use_body_feedback():
    common = {
        'enabled': True,
        'maximum_route_turn': math.radians(20.0),
        'body_alpha': 0.0,
        'turn_in_place_angle': 1.0,
    }
    assert not use_straight_course_feedback(
        course_valid=False,
        route_turn=0.0,
        **common,
    )
    assert not use_straight_course_feedback(
        course_valid=True,
        route_turn=math.radians(30.0),
        **common,
    )


def test_three_centimeter_deadband_changes_term_not_feedback_source():
    route_heading = math.radians(5.0)
    inside = straight_target_course(
        route_heading=route_heading,
        target_angle=math.radians(15.0),
        lateral_distance=0.02,
        deadband=0.03,
        maximum_angle=math.radians(22.0),
    )
    outside = straight_target_course(
        route_heading=route_heading,
        target_angle=math.radians(15.0),
        lateral_distance=0.04,
        deadband=0.03,
        maximum_angle=math.radians(22.0),
    )
    assert math.isclose(inside, route_heading)
    assert math.isclose(outside, math.radians(15.0))


def test_motion_course_uses_distance_window_and_age():
    estimator = MotionCourseEstimator(
        minimum_distance=0.10,
        smoothing=1.0,
        maximum_age=0.50,
    )
    assert not estimator.update(0.0, 0.0, 1.0)
    assert not estimator.update(0.05, 0.0, 1.1)
    assert estimator.update(0.11, 0.0, 1.2)
    assert math.isclose(estimator.course, 0.0)
    assert estimator.valid(1.6)
    assert not estimator.valid(1.8)


def test_target_course_is_limited_around_route_heading():
    result = bounded_target_course(
        route_heading=0.0,
        target_angle=math.radians(45.0),
        maximum_angle=math.radians(22.0),
    )
    assert math.isclose(
        result,
        math.radians(22.0),
        abs_tol=1e-9,
    )

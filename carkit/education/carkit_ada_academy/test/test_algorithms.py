import math

from carkit_ada_academy.math_utils import keep_detection
from carkit_ada_academy.math_utils import pure_pursuit_command
from carkit_ada_academy.math_utils import straight_line_points


def test_straight_line_reaches_goal():
    points = straight_line_points((0.0, 0.0), (1.0, 0.0), spacing=0.2)
    assert points[0] == (0.0, 0.0)
    assert points[-1] == (1.0, 0.0)


def test_pure_pursuit_stops_without_a_path():
    assert pure_pursuit_command((0.0, 0.0), 0.0, []) == (0.0, 0.0)


def test_pure_pursuit_steers_toward_path():
    speed, steering = pure_pursuit_command(
        (0.0, 0.0),
        0.0,
        [(1.0, 0.5), (2.0, 0.5)],
    )
    assert speed > 0.0
    assert 0.0 < steering <= 0.34
    assert math.isfinite(steering)


def test_pure_pursuit_does_not_target_a_passed_waypoint():
    speed, steering = pure_pursuit_command(
        (1.2, 0.0),
        0.0,
        [(0.0, 0.4), (0.5, 0.2), (1.0, 0.0), (1.8, 0.0), (2.5, 0.0)],
        lookahead=0.55,
    )
    assert speed > 0.0
    assert abs(steering) < 1.0e-9


def test_pure_pursuit_slows_smoothly_near_goal():
    far_speed, _ = pure_pursuit_command(
        (0.0, 0.0), 0.0, [(0.0, 0.0), (1.0, 0.0)], speed=0.45
    )
    near_speed, _ = pure_pursuit_command(
        (0.7, 0.0), 0.0, [(0.0, 0.0), (1.0, 0.0)], speed=0.45
    )
    stopped_speed, _ = pure_pursuit_command(
        (0.9, 0.0), 0.0, [(0.0, 0.0), (1.0, 0.0)], speed=0.45
    )
    assert 0.0 < near_speed < far_speed
    assert stopped_speed == 0.0


def test_detection_threshold_is_student_tunable():
    assert keep_detection("person", 0.6, 0.5)
    assert not keep_detection("person", 0.4, 0.5)

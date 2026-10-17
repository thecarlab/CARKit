import math
import pytest
from carkit_bringup.speed_feedback import SpeedFeedback


def test_under_speed_correction_and_convergence():
    controller = SpeedFeedback(4, 2)
    speed = 0.0
    for _ in range(1000):
        output = controller.update(1.0, speed, .02)
        speed += .08 * (.8 * output - speed)
    assert speed == pytest.approx(1.0, abs=.01)
    assert output == pytest.approx(1.25, abs=.02)


def test_reverse_and_overspeed():
    controller = SpeedFeedback(4, 2)
    assert controller.update(-1, -.5, .02) < -1
    controller.reset()
    assert 0 < controller.update(1, 1.3, .02) < 1


def test_stall_bounded_stop_and_direction_reset():
    controller = SpeedFeedback(4, 2)
    for _ in range(2000):
        assert 0 <= controller.update(1, 0, .02) <= 1.5
    assert controller.update(0, 0, .02) == 0
    assert controller.integral == 0
    assert -1.5 <= controller.update(-1, 0, .02) < -1


def test_limits_and_invalid_values():
    controller = SpeedFeedback(.8, .4)
    for _ in range(500):
        assert controller.update(1, 0, .02) <= .8
    assert controller.integral == 0
    assert controller.update(-1, 0, .02) >= -.4
    for value in (math.nan, math.inf):
        assert controller.update(value, 0, .02) == 0
    assert controller.update(1, 0, .5) == 0

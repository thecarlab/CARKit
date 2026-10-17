import math

import pytest

from carkit_bringup.measured_odometry import MeasuredOdometry, rotate_vector


def integrate(speed, rate, duration=4.0):
    motion = MeasuredOdometry()
    for i in range(round(duration / .02) + 1):
        motion.update(100 + i * .02, speed, rate)
    return motion


@pytest.mark.parametrize('speed,rate', [(1.0, .3), (-.4, -.2), (.5, 0.0)])
def test_measured_motion_matches_exact_arc(speed, rate):
    motion = integrate(speed, rate)
    expected_x = speed * math.sin(rate * 4) / rate if rate else speed * 4
    expected_y = speed * (1 - math.cos(rate * 4)) / rate if rate else 0.0
    assert motion.x == pytest.approx(expected_x, abs=1e-9)
    assert motion.y == pytest.approx(expected_y, abs=1e-9)
    assert motion.yaw == pytest.approx(rate * 4, abs=1e-9)


def test_gyro_observes_rotation_without_wheel_rotation():
    motion = integrate(0.0, .2)
    assert motion.x == motion.y == 0.0
    assert motion.yaw == pytest.approx(.8)


def test_sensor_gap_does_not_invent_motion_and_increases_uncertainty():
    motion = integrate(.5, .2)
    pose = (motion.x, motion.y, motion.yaw)
    covariance = [motion.covariance[i][i] for i in range(3)]
    motion.update(110., .5, .2)
    assert (motion.x, motion.y, motion.yaw) == pose
    assert all(motion.covariance[i][i] > covariance[i] for i in range(3))
    motion.pause()
    motion.update(120., 0., 0.)
    assert (motion.x, motion.y, motion.yaw) == pose


def test_old_or_invalid_measurements_cannot_change_pose():
    motion = integrate(.5, .2)
    pose = (motion.x, motion.y, motion.yaw)
    assert not motion.update(100., 3., 2.)
    with pytest.raises(ValueError):
        motion.update(104.02, float('nan'), 0.)
    assert (motion.x, motion.y, motion.yaw) == pose


def test_optical_gyro_axes_are_rotated_before_yaw_integration():
    # Sensor +Y is body +Z; reading raw gyro.z would miss all this turn.
    q = (math.sin(math.pi / 4), 0., 0., math.cos(math.pi / 4))
    assert rotate_vector(q, (0., .3, 0.)) == pytest.approx((0., 0., .3))
    with pytest.raises(ValueError):
        rotate_vector((0., 0., 0., 0.), (0., .3, 0.))


def test_pose_covariance_remains_symmetric_and_nonnegative():
    p = integrate(.5, .3).covariance
    for i in range(3):
        assert p[i][i] > 0
        for j in range(3):
            assert p[i][j] == pytest.approx(p[j][i])
    for v in ((1., -1., .5), (0., 1., -1.), (1., 0., 0.)):
        assert sum(v[i] * p[i][j] * v[j] for i in range(3) for j in range(3)) >= 0

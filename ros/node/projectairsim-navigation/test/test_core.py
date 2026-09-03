import math

import pytest

from projectairsim_navigation.core import (
    Controller,
    GroundTruthFilter,
    LineTrajectory,
    Reference,
    enu_to_nwu_vector,
    norm,
    nwu_to_enu_quaternion,
    nwu_to_enu_vector,
    wrap_angle,
    yaw_from_quaternion,
)


def test_nwu_enu_vector_round_trip():
    source = (1.0, 2.0, 3.0)
    assert nwu_to_enu_vector(source) == (-2.0, 1.0, 3.0)
    assert enu_to_nwu_vector(nwu_to_enu_vector(source)) == source


def test_nwu_forward_orientation_points_enu_north():
    converted = nwu_to_enu_quaternion((0.0, 0.0, 0.0, 1.0))
    assert yaw_from_quaternion(converted) == pytest.approx(math.pi / 2.0)


def test_velocity_filter_uses_timestamp_and_alpha():
    estimator = GroundTruthFilter(alpha=0.2)
    assert estimator.update(1.0, (0.0, 0.0, 0.0), 0.0)[0] == (0.0, 0.0, 0.0)
    velocity, yaw_rate = estimator.update(2.0, (10.0, 0.0, 0.0), 1.0)
    assert velocity == pytest.approx((2.0, 0.0, 0.0))
    assert yaw_rate == pytest.approx(0.2)


def test_yaw_difference_wraps_at_pi():
    assert wrap_angle(math.radians(-179.0) - math.radians(179.0)) == pytest.approx(math.radians(2.0))


def test_line_trajectory_endpoints_and_speed_limit():
    trajectory = LineTrajectory((0.0, 0.0, 0.0), (10.0, 0.0, 0.0), 0.0, 1.0, 2.0)
    assert trajectory.sample(0.0).position == (0.0, 0.0, 0.0)
    assert trajectory.sample(trajectory.duration).position == (10.0, 0.0, 0.0)
    assert trajectory.sample(trajectory.duration).velocity == (0.0, 0.0, 0.0)
    assert norm(trajectory.sample(trajectory.duration / 2.0).velocity) <= 2.0


def test_controller_feedback_saturation_and_yaw_wrap():
    controller = Controller(position_gain=1.5, yaw_gain=5.0, max_speed=4.0, max_yaw_rate=1.0)
    reference = Reference((100.0, 0.0, 0.0), (1.0, 0.0, 0.0), math.radians(-179.0))
    velocity, yaw_rate = controller.calculate((0.0, 0.0, 0.0), math.radians(179.0), reference)
    assert norm(velocity) == pytest.approx(4.0)
    assert yaw_rate == pytest.approx(math.radians(10.0))


@pytest.mark.parametrize("alpha", [0.0, -1.0, 1.1])
def test_filter_rejects_invalid_alpha(alpha):
    with pytest.raises(ValueError):
        GroundTruthFilter(alpha)

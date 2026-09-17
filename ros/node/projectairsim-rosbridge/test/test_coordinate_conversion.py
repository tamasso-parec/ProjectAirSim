"""Coordinate conversion between Project AirSim and ROS frame conventions.

Project AirSim uses world NED and body FRD.  The bridge's historical output is
North-West-Up, which is self-consistent but is not the ENU that REP 103
assumes, so an ENU convention is selectable.  These tests pin down both.
"""
import math

import pytest

from projectairsim_rosbridge import utils


def rotate(quat_wxyz, vector):
    """Rotate a vector by a (w, x, y, z) quaternion, independently of utils."""
    quat_conjugate = (
        quat_wxyz[0],
        -quat_wxyz[1],
        -quat_wxyz[2],
        -quat_wxyz[3],
    )
    pure = (0.0,) + tuple(vector)
    result = utils._quat_multiply(
        utils._quat_multiply(quat_wxyz, pure), quat_conjugate
    )
    return result[1:]


def approx(values):
    return pytest.approx(values, abs=1e-12)


@pytest.fixture
def nwu():
    return utils.CoordinateConverter(utils.CONVENTION_NWU)


@pytest.fixture
def enu():
    return utils.CoordinateConverter(utils.CONVENTION_ENU)


def test_unknown_convention_is_rejected():
    with pytest.raises(ValueError, match="ned"):
        utils.CoordinateConverter("ned")


def test_nwu_reproduces_the_historical_negate_y_and_z_behaviour(nwu):
    assert nwu.world_vector((1.0, 2.0, 3.0)) == approx((1.0, -2.0, -3.0))
    # The legacy quaternion conversion was (x, -y, -z, w).
    assert nwu.world_quaternion((0.9, 0.1, 0.2, 0.3)) == approx(
        (0.9, 0.1, -0.2, -0.3)
    )


def test_module_level_helpers_default_to_the_historical_convention():
    assert (
        utils.get_default_coordinate_converter().convention == utils.CONVENTION_NWU
    )
    assert utils.to_ros_position_list({"x": 1.0, "y": 2.0, "z": 3.0}) == approx(
        (1.0, -2.0, -3.0)
    )
    assert utils.to_ros_quaternion_list(
        {"w": 0.9, "x": 0.1, "y": 0.2, "z": 0.3}
    ) == approx((0.1, -0.2, -0.3, 0.9))


def test_setting_the_default_converter_changes_the_module_helpers():
    original = utils.get_default_coordinate_converter()
    try:
        utils.set_default_coordinate_converter(
            utils.CoordinateConverter(utils.CONVENTION_ENU)
        )
        # 10 m north, 5 m up in NED becomes 10 m along ENU north (+Y).
        assert utils.to_ros_position_list(
            {"x": 10.0, "y": 0.0, "z": -5.0}
        ) == approx((0.0, 10.0, 5.0))
    finally:
        utils.set_default_coordinate_converter(original)

    assert utils.to_ros_position_list({"x": 10.0, "y": 0.0, "z": -5.0}) == approx(
        (10.0, -0.0, 5.0)
    )


def test_set_default_converter_rejects_other_types():
    with pytest.raises(TypeError):
        utils.set_default_coordinate_converter("enu")


def test_enu_maps_ned_axes_onto_east_north_up(enu):
    assert enu.world_vector((10.0, 0.0, -5.0)) == approx((0.0, 10.0, 5.0))
    assert enu.world_vector((0.0, 3.0, 0.0)) == approx((3.0, 0.0, -0.0))


def test_enu_orientation_of_a_level_vehicle_facing_north_is_90_degrees_of_yaw(enu):
    # Identity in NED means facing north; in ENU that is +90 degrees of yaw.
    assert enu.world_quaternion((1.0, 0.0, 0.0, 0.0)) == approx(
        (math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5))
    )


def test_enu_orientation_of_a_level_vehicle_facing_east_is_the_identity(enu):
    ned_yaw_90 = (math.cos(math.pi / 4), 0.0, 0.0, math.sin(math.pi / 4))
    assert enu.world_quaternion(ned_yaw_90) == approx((1.0, 0.0, 0.0, 0.0))


@pytest.mark.parametrize("convention", utils.CONVENTIONS)
def test_body_vectors_are_independent_of_the_world_convention(convention):
    converter = utils.CoordinateConverter(convention)
    assert converter.body_vector((1.0, 2.0, 3.0)) == approx((1.0, -2.0, -3.0))


@pytest.mark.parametrize("convention", utils.CONVENTIONS)
def test_conversions_are_their_own_inverse(convention):
    converter = utils.CoordinateConverter(convention)
    vector = (1.5, -2.5, 3.5)
    quat = (0.5, 0.5, 0.5, 0.5)

    assert converter.world_vector(converter.world_vector(vector)) == approx(vector)
    assert converter.world_quaternion(
        converter.world_quaternion(quat)
    ) == approx(quat)


@pytest.mark.parametrize("convention", utils.CONVENTIONS)
def test_world_quaternion_preserves_unit_norm(convention):
    converter = utils.CoordinateConverter(convention)
    quat = (0.18257418583505536, 0.3651483716701107, 0.5477225575051661, 0.7302967433402214)
    converted = converter.world_quaternion(quat)

    assert math.sqrt(sum(value * value for value in converted)) == pytest.approx(
        1.0, abs=1e-12
    )


@pytest.mark.parametrize("convention", utils.CONVENTIONS)
@pytest.mark.parametrize(
    "ned_quat",
    [
        (1.0, 0.0, 0.0, 0.0),
        # 30 degrees nose-down pitch about the body Y axis.
        (math.cos(math.radians(15)), 0.0, math.sin(math.radians(15)), 0.0),
        # 40 degrees of roll about the body X axis.
        (math.cos(math.radians(20)), math.sin(math.radians(20)), 0.0, 0.0),
        # A general orientation.
        (0.5, 0.5, 0.5, 0.5),
    ],
)
def test_converted_orientation_agrees_with_converting_the_body_axes(
    convention, ned_quat
):
    """
    The orientation and vector conversions must describe the same rotation.

    Rotating a body axis in Project AirSim and then converting the resulting
    world vector must give the same answer as converting the orientation and
    rotating the corresponding ROS body axis.  This is what catches a
    conversion that is self-consistent but mixes up the world and body parts.
    """
    converter = utils.CoordinateConverter(convention)
    ros_quat = converter.world_quaternion(ned_quat)

    # Project AirSim body FRD axes and the ROS FLU axes that correspond.
    axis_pairs = [
        ((1.0, 0.0, 0.0), (1.0, 0.0, 0.0)),  # forward
        ((0.0, -1.0, 0.0), (0.0, 1.0, 0.0)),  # left
        ((0.0, 0.0, -1.0), (0.0, 0.0, 1.0)),  # up
    ]
    for frd_axis, flu_axis in axis_pairs:
        expected = converter.world_vector(rotate(ned_quat, frd_axis))
        assert rotate(ros_quat, flu_axis) == approx(expected)

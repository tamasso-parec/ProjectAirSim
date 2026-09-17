"""Reprojection of Project AirSim depth images into ROS point clouds.

The geometry here is the part that fails silently: a wrong sign or a
transposed axis still produces a plausible-looking cloud that maps the world
inside out.  These tests pin the pinhole model and both axis conventions
against values worked out independently.
"""
import math
import struct

import numpy as np
import pytest

from builtin_interfaces.msg import Time
from sensor_msgs.msg import PointCloud2, PointField

from projectairsim_rosbridge.interface_profile import (
    DepthSettings,
    PointCloudSettings,
)
from projectairsim_rosbridge.msg_converter import MsgConverter
from projectairsim_rosbridge.sim_time import SimTimeSource


class FakeROSNode:
    @staticmethod
    def get_time_now_msg():
        return Time(sec=12, nanosec=34)

    @staticmethod
    def get_time_from_nanos(nanos):
        return int(nanos)

    @staticmethod
    def get_time_to_msg(timestamp):
        nanos = int(timestamp)
        return Time(sec=nanos // 1000000000, nanosec=nanos % 1000000000)


# Deliberately small and asymmetric so a transposed axis cannot pass.
WIDTH = 5
HEIGHT = 3
FX = 100.0
FY = 50.0
CX = 2.0
CY = 1.0

# Row-major CameraInfo.k for the intrinsics above.
K = [FX, 0.0, CX, 0.0, FY, CY, 0.0, 0.0, 1.0]
K_UNCALIBRATED = [0.0] * 9


def depth_message(depths_mm, width=WIDTH, height=HEIGHT, big_endian=False, **extra):
    prefix = ">" if big_endian else "<"
    flat = list(depths_mm)
    assert len(flat) == width * height
    message = {
        "encoding": "16UC1",
        "height": height,
        "width": width,
        "big_endian": big_endian,
        "data": struct.pack(f"{prefix}{len(flat)}H", *flat),
    }
    message.update(extra)
    return message


def uniform_depth(millimetres):
    return depth_message([millimetres] * (WIDTH * HEIGHT))


def converter(**kwargs):
    return MsgConverter(FakeROSNode(), **kwargs)


def read_points(point_cloud):
    """Return the cloud as an (height, width, 3) float array."""
    values = np.frombuffer(bytes(point_cloud.data), dtype="<f4")
    return values.reshape(point_cloud.height, point_cloud.width, 3)


def convert(result=None, message=None, settings=None, k=None):
    return (result or converter()).convert_depth_image_to_point_cloud(
        message if message is not None else uniform_depth(2000),
        K if k is None else k,
        "camera_optical",
        settings or PointCloudSettings(),
    )


def test_point_cloud_message_layout_is_organised_xyz():
    point_cloud = convert()

    assert isinstance(point_cloud, PointCloud2)
    assert (point_cloud.height, point_cloud.width) == (HEIGHT, WIDTH)
    assert [field.name for field in point_cloud.fields] == ["x", "y", "z"]
    assert [field.offset for field in point_cloud.fields] == [0, 4, 8]
    assert all(
        field.datatype == PointField.FLOAT32 for field in point_cloud.fields
    )
    assert point_cloud.point_step == 12
    assert point_cloud.row_step == 12 * WIDTH
    assert point_cloud.is_bigendian is False
    # Invalid pixels stay in place as NaN, so the cloud is organised, not dense.
    assert point_cloud.is_dense is False
    assert len(point_cloud.data) == 12 * WIDTH * HEIGHT


def test_optical_convention_follows_the_pinhole_model():
    """X right, Y down, Z forward, with Z the measured depth."""
    depth_m = 2.0
    points = read_points(convert(message=uniform_depth(2000)))

    for row in range(HEIGHT):
        for column in range(WIDTH):
            x, y, z = points[row, column]
            assert z == pytest.approx(depth_m)
            assert x == pytest.approx((column - CX) * depth_m / FX)
            assert y == pytest.approx((row - CY) * depth_m / FY)


def test_the_principal_point_projects_onto_the_optical_axis():
    """The pixel at (cx, cy) must have zero lateral offset."""
    points = read_points(convert(message=uniform_depth(3000)))
    x, y, z = points[int(CY), int(CX)]

    assert (x, y) == (pytest.approx(0.0), pytest.approx(0.0))
    assert z == pytest.approx(3.0)


def test_ros_convention_is_the_optical_frame_rotated_into_a_body_frame():
    """X forward, Y left, Z up."""
    optical = read_points(convert(settings=PointCloudSettings("optical")))
    body = read_points(convert(settings=PointCloudSettings("ros")))

    assert body[..., 0] == pytest.approx(optical[..., 2])  # forward
    assert body[..., 1] == pytest.approx(-optical[..., 0])  # left
    assert body[..., 2] == pytest.approx(-optical[..., 1])  # up


def test_separate_focal_lengths_are_not_confused():
    """A single fx used for both axes would make these agree; they must not."""
    points = read_points(convert(message=uniform_depth(1000)))
    x_at_one_pixel_right = points[int(CY), int(CX) + 1][0]
    y_at_one_pixel_down = points[int(CY) + 1, int(CX)][1]

    assert x_at_one_pixel_right == pytest.approx(1.0 / FX)
    assert y_at_one_pixel_down == pytest.approx(1.0 / FY)
    assert x_at_one_pixel_right != pytest.approx(y_at_one_pixel_down)


def test_invalid_samples_become_nan_points():
    """Zero is "no reading" and saturation is "too far": neither is geometry."""
    depths = [2000] * (WIDTH * HEIGHT)
    depths[0] = 0  # no reading
    depths[1] = 65535  # saturated
    points = read_points(convert(message=depth_message(depths)))

    assert all(math.isnan(value) for value in points[0, 0])
    assert all(math.isnan(value) for value in points[0, 1])
    assert not any(math.isnan(value) for value in points[0, 2])


def test_samples_beyond_the_configured_range_become_nan_points():
    depths = [2000] * (WIDTH * HEIGHT)
    depths[3] = 12000
    result = converter(depth=DepthSettings(max_range_m=10.0))
    points = read_points(convert(result, message=depth_message(depths)))

    assert all(math.isnan(value) for value in points[0, 3])
    assert points[0, 0][2] == pytest.approx(2.0)


def test_no_points_are_produced_before_the_camera_intrinsics_arrive():
    """
    Camera info is a separate Project AirSim topic. Reprojecting with a zero
    focal length would silently collapse every point onto the optical axis.
    """
    assert convert(k=K_UNCALIBRATED) is None


def test_decimation_subsamples_the_grid_and_keeps_the_geometry():
    points_full = read_points(convert(message=uniform_depth(2000)))
    decimated = convert(
        message=uniform_depth(2000), settings=PointCloudSettings(decimation=2)
    )
    points = read_points(decimated)

    assert (decimated.height, decimated.width) == (2, 3)
    assert decimated.row_step == 12 * 3
    # Decimated samples must keep their original pixel geometry, not be
    # renumbered as if the image had shrunk.
    for row in range(2):
        for column in range(3):
            assert points[row, column] == pytest.approx(
                points_full[row * 2, column * 2]
            )


def test_the_transmitted_byte_order_is_respected():
    little = read_points(convert(message=depth_message([1500] * 15)))
    big = read_points(
        convert(message=depth_message([1500] * 15, big_endian=True))
    )

    assert big == pytest.approx(little)
    assert little[int(CY), int(CX)][2] == pytest.approx(1.5)


def test_the_cloud_carries_the_frame_and_the_images_simulation_timestamp():
    sim_time = SimTimeSource(FakeROSNode(), enabled=True)
    result = converter(sim_time=sim_time)
    point_cloud = convert(
        result, message=uniform_depth(2000) | {"time_stamp": 4500000000}
    )

    assert point_cloud.header.frame_id == "camera_optical"
    assert point_cloud.header.stamp == Time(sec=4, nanosec=500000000)


def test_depth_image_and_point_cloud_agree_on_the_same_frame():
    """
    They come from one message, so a consumer fusing them must not see two
    different depths for one pixel.
    """
    result = converter()
    message = depth_message([1234] * (WIDTH * HEIGHT))

    image = result.convert_image_to_ros("/robot/sensors/D/depth_planar_camera", message)
    points = read_points(convert(result, message=message))
    image_values = np.frombuffer(bytes(image.data), dtype="<f4").reshape(
        HEIGHT, WIDTH
    )

    assert image.encoding == "32FC1"
    assert points[..., 2] == pytest.approx(image_values)

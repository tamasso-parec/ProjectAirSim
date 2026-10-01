"""Colouring a reprojected point cloud from its camera's colour image."""

# A Gazebo RGB-D sensor publishes a coloured cloud, so a stack standing in for
# one inherits consumers that expect the field: RViz's point cloud display
# defaults to colouring by RGB8 and draws nothing at all without it, which
# reads as "no data" rather than "no colour".
#
# Project AirSim splits one capture across two topics, so the colour arrives
# on a different handler than the depth. These tests pin the pairing, the
# packing convention, and that a cloud is still published when colour is
# missing or mismatched rather than being dropped.
import struct

import numpy as np
import pytest

from projectairsim_rosbridge.interface_profile import PointCloudSettings
from projectairsim_rosbridge.msg_converter import MsgConverter


DEPTH_TOPIC = "/Sim/Scene/robots/Drone1/sensors/RGBD/depth_planar_camera"
COLOR_TOPIC = "/Sim/Scene/robots/Drone1/sensors/RGBD/scene_camera"

K = [100.0, 0.0, 2.0, 0.0, 100.0, 2.0, 0.0, 0.0, 1.0]


def converter():
    return MsgConverter.__new__(MsgConverter)


def prepared(rows=4, columns=4):
    """A converter holding one colour frame, with a known BGR pattern."""
    c = converter()
    c._color_frames = {}
    c._color_size_warnings = set()
    import builtin_interfaces.msg as rostimemsg

    c.sim_time = type(
        "T", (), {"stamp": staticmethod(lambda _m: rostimemsg.Time())}
    )()
    c.depth = type("D", (), {"max_range_m": 0.0})()

    bgr = np.zeros((rows, columns, 3), dtype=np.uint8)
    bgr[..., 0] = 10   # blue
    bgr[..., 1] = 20   # green
    bgr[..., 2] = 30   # red
    c._color_frames[COLOR_TOPIC[: -len(MsgConverter.COLOR_IMAGE_SUFFIX)]] = {
        "height": rows,
        "width": columns,
        "data": bgr.tobytes(),
    }
    return c, bgr


def depth_message(rows=4, columns=4, millimetres=1000):
    return {
        "height": rows,
        "width": columns,
        "big_endian": 0,
        "data": np.full((rows, columns), millimetres, dtype="<u2").tobytes(),
    }


def test_a_colour_frame_is_matched_to_its_own_cameras_depth_image():
    c, _ = prepared()

    samples = c._color_for_depth(DEPTH_TOPIC, rows=4, columns=4, step=1)

    assert samples is not None
    assert samples.shape == (4, 4, 3)


def test_another_cameras_depth_image_is_not_coloured_from_this_one():
    # Two RGB-D sensors on one vehicle would otherwise paint each other's
    # clouds, and the result looks plausible rather than obviously wrong.
    c, _ = prepared()

    other = "/Sim/Scene/robots/Drone1/sensors/Down/depth_planar_camera"
    assert c._color_for_depth(other, rows=4, columns=4, step=1) is None


def test_colour_is_absent_until_a_frame_has_arrived():
    c = converter()
    c._color_frames = {}
    c._color_size_warnings = set()

    assert c._color_for_depth(DEPTH_TOPIC, rows=4, columns=4, step=1) is None


def test_the_rgb_field_carries_the_cameras_colours():
    c, _ = prepared()
    settings = PointCloudSettings(color=True)

    cloud = c.convert_depth_image_to_point_cloud(
        depth_message(), K, "cam", settings, DEPTH_TOPIC
    )

    names = [field.name for field in cloud.fields]
    assert names == ["x", "y", "z", "rgb"]
    assert cloud.point_step == 16
    assert cloud.row_step == 16 * cloud.width

    # Unpack the first point's colour the way a consumer would.
    packed = struct.unpack_from("<f", bytes(cloud.data), 12)[0]
    value = struct.unpack("<I", struct.pack("<f", packed))[0]
    assert ((value >> 16) & 0xFF, (value >> 8) & 0xFF, value & 0xFF) == (30, 20, 10)


def test_without_colour_the_cloud_keeps_its_original_layout():
    # Existing consumers of an xyz-only cloud must be unaffected by the
    # feature existing, which is why it is off by default.
    c, _ = prepared()

    cloud = c.convert_depth_image_to_point_cloud(
        depth_message(), K, "cam", PointCloudSettings(), DEPTH_TOPIC
    )

    assert [f.name for f in cloud.fields] == ["x", "y", "z"]
    assert cloud.point_step == 12


def test_a_mismatched_colour_resolution_yields_an_uncoloured_cloud():
    # Resampling would be a guess about registration; an uncoloured cloud is
    # honest, and the warning says what to change.
    c, _ = prepared(rows=8, columns=8)
    warnings = []
    import projectairsim_rosbridge.msg_converter as module

    original = module.projectairsim_log
    module.projectairsim_log = lambda: type(
        "L", (), {"warning": staticmethod(warnings.append)}
    )()
    try:
        cloud = c.convert_depth_image_to_point_cloud(
            depth_message(4, 4), K, "cam", PointCloudSettings(color=True), DEPTH_TOPIC
        )
    finally:
        module.projectairsim_log = original

    assert [f.name for f in cloud.fields] == ["x", "y", "z"]
    assert warnings and "without colour" in warnings[0]


def test_decimation_samples_the_colour_the_same_way_as_the_depth():
    c, _ = prepared(rows=8, columns=8)
    settings = PointCloudSettings(color=True, decimation=2)

    cloud = c.convert_depth_image_to_point_cloud(
        depth_message(8, 8), K, "cam", settings, DEPTH_TOPIC
    )

    assert (cloud.height, cloud.width) == (4, 4)
    assert [f.name for f in cloud.fields] == ["x", "y", "z", "rgb"]

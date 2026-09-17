import math
import struct

import numpy as np
import pytest

from builtin_interfaces.msg import Time
from radar_msgs.msg import RadarScan, RadarTracks
from sensor_msgs.msg import Image, Imu, NavSatFix, PointCloud2
from sensor_msgs_py import point_cloud2

from projectairsim_rosbridge import utils
from projectairsim_rosbridge.interface_profile import DepthSettings
from projectairsim_rosbridge.msg_converter import MsgConverter
from projectairsim_rosbridge.sim_time import SimTimeSource


class FakeROSNode:
    PointCloud2 = point_cloud2

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

    def create_publisher(self, topic, msg_type, **kwargs):
        raise AssertionError("the converter must not create publishers")


ROBOT_TOPIC = "/airsim_node/robots/Drone1/sensors/TestSensor/data"


def converter(**kwargs):
    result = MsgConverter(FakeROSNode(), **kwargs)
    result.set_robot_base_frame_ids({ROBOT_TOPIC: "airsim_node/robots/Drone1"})
    return result


def depth_message(depths_mm, big_endian=False):
    prefix = ">" if big_endian else "<"
    return {
        "encoding": "16UC1",
        "height": 1,
        "width": len(depths_mm),
        "big_endian": big_endian,
        "data": struct.pack(f"{prefix}{len(depths_mm)}H", *depths_mm),
    }


def test_pose_and_bgr_image_conversion():
    result = converter()
    pose = result.convert_actual_pose_to_ros(
        ROBOT_TOPIC,
        {
            "position": {"x": 1.0, "y": 2.0, "z": 3.0},
            "orientation": {"x": 0.1, "y": 0.2, "z": 0.3, "w": 0.9},
        },
    )
    image = result.convert_image_to_ros(
        ROBOT_TOPIC,
        {
            "encoding": "BGR",
            "height": 1,
            "width": 2,
            "big_endian": False,
            "data": bytes([1, 2, 3, 4, 5, 6]),
        },
    )

    assert pose.pose.position.y == -2.0
    assert pose.pose.position.z == -3.0
    assert isinstance(image, Image)
    assert image.encoding == "bgr8"
    assert image.step == 6
    assert bytes(image.data) == bytes([1, 2, 3, 4, 5, 6])


def test_pose_conversion_accepts_numpy_scalars_from_live_topics():
    pose = converter().convert_actual_pose_to_ros(
        ROBOT_TOPIC,
        {
            "position": {
                "x": np.float32(1.0),
                "y": np.float64(2.0),
                "z": np.float32(3.0),
            },
            "orientation": {
                "x": np.float32(0.0),
                "y": np.float64(0.0),
                "z": np.float32(0.0),
                "w": np.float64(1.0),
            },
        },
    )

    assert pose.pose.position.x == 1.0
    assert pose.pose.position.y == -2.0
    assert pose.pose.orientation.w == 1.0


def test_depth_image_defaults_to_metric_float_metres():
    """The default must be directly usable by the ROS depth ecosystem."""
    image = converter().convert_image_to_ros(
        ROBOT_TOPIC, depth_message([1500, 3250, 10000])
    )
    values = np.frombuffer(bytes(image.data), dtype="<f4")

    assert image.encoding == "32FC1"
    assert image.step == 4 * 3
    assert image.is_bigendian == 0
    assert values == pytest.approx([1.5, 3.25, 10.0], abs=1e-6)


def test_depth_zero_becomes_nan_and_saturation_becomes_infinity():
    image = converter().convert_image_to_ros(
        ROBOT_TOPIC, depth_message([0, 65535, 2000])
    )
    values = np.frombuffer(bytes(image.data), dtype="<f4")

    assert math.isnan(values[0])
    assert math.isinf(values[1]) and values[1] > 0
    assert values[2] == pytest.approx(2.0)


def test_depth_beyond_the_configured_range_becomes_infinity():
    result = converter(depth=DepthSettings(max_range_m=10.0))
    image = result.convert_image_to_ros(
        ROBOT_TOPIC, depth_message([9999, 10000, 10001])
    )
    values = np.frombuffer(bytes(image.data), dtype="<f4")

    assert values[0] == pytest.approx(9.999)
    assert values[1] == pytest.approx(10.0)
    assert math.isinf(values[2])


def test_depth_can_be_published_as_millimetres():
    result = converter(
        depth=DepthSettings(
            encoding=DepthSettings.ENCODING_16UC1, max_range_m=10.0
        )
    )
    image = result.convert_image_to_ros(
        ROBOT_TOPIC, depth_message([0, 1500, 10001, 65535])
    )
    values = np.frombuffer(bytes(image.data), dtype="<u2")

    assert image.encoding == "16UC1"
    assert image.step == 2 * 4
    # 0 is the 16UC1 "no reading" value, so out-of-range samples collapse to it.
    assert list(values) == [0, 1500, 0, 0]


def test_depth_mono8_reproduces_the_legacy_scaling():
    result = converter(
        depth=DepthSettings(encoding=DepthSettings.ENCODING_MONO8)
    )
    image = result.convert_image_to_ros(
        ROBOT_TOPIC, depth_message([0, result.max_depth_mm])
    )

    assert result.max_depth_mm == 6000
    assert image.encoding == "mono8"
    assert image.step == 2
    assert list(image.data) == [0, 255]


def test_depth_mono8_does_not_wrap_beyond_its_range():
    """The legacy conversion overflowed uint8 for samples past its range."""
    result = converter(
        depth=DepthSettings(encoding=DepthSettings.ENCODING_MONO8)
    )
    image = result.convert_image_to_ros(ROBOT_TOPIC, depth_message([65535]))

    assert list(image.data) == [255]


def test_depth_respects_the_transmitted_byte_order():
    image = converter().convert_image_to_ros(
        ROBOT_TOPIC, depth_message([1500, 3250], big_endian=True)
    )
    values = np.frombuffer(bytes(image.data), dtype="<f4")

    assert values == pytest.approx([1.5, 3.25], abs=1e-6)


def test_unsupported_image_encoding_is_rejected():
    with pytest.raises(ValueError, match="BGR or 16UC1"):
        converter().convert_image_to_ros(
            ROBOT_TOPIC, {"encoding": "PNG", "height": 1, "width": 1}
        )


def test_messages_are_stamped_from_the_wall_clock_by_default():
    result = converter()
    imu = result.convert_imu_to_ros(
        ROBOT_TOPIC,
        {
            "time_stamp": 5000000000,
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            "angular_velocity": {"x": 0.0, "y": 0.0, "z": 0.0},
            "linear_acceleration": {"x": 0.0, "y": 0.0, "z": 0.0},
        },
    )

    assert imu.header.stamp == Time(sec=12, nanosec=34)


def test_messages_are_stamped_from_simulation_time_when_enabled():
    node = FakeROSNode()
    sim_time = SimTimeSource(node, enabled=True)
    result = MsgConverter(node, sim_time=sim_time)
    result.set_robot_base_frame_ids({ROBOT_TOPIC: "airsim_node/robots/Drone1"})

    imu = result.convert_imu_to_ros(
        ROBOT_TOPIC,
        {
            "time_stamp": 5250000000,
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            "angular_velocity": {"x": 0.0, "y": 0.0, "z": 0.0},
            "linear_acceleration": {"x": 0.0, "y": 0.0, "z": 0.0},
        },
    )
    image = result.convert_image_to_ros(
        ROBOT_TOPIC, dict(depth_message([1000]), time_stamp=5500000000)
    )

    assert imu.header.stamp == Time(sec=5, nanosec=250000000)
    assert image.header.stamp == Time(sec=5, nanosec=500000000)
    assert sim_time.sim_time_nanos == 5500000000


def test_the_configured_frame_convention_is_applied():
    result = converter(
        coords=utils.CoordinateConverter(utils.CONVENTION_ENU)
    )
    pose = result.convert_actual_pose_to_ros(
        ROBOT_TOPIC,
        {
            "position": {"x": 10.0, "y": 0.0, "z": -5.0},
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        },
    )

    # 10 m north and 5 m up in NED is 10 m along ENU north, which is +Y.
    assert (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z) == (
        pytest.approx(0.0),
        pytest.approx(10.0),
        pytest.approx(5.0),
    )
    # Facing north is +90 degrees of yaw in ENU.
    assert pose.pose.orientation.z == pytest.approx(math.sqrt(0.5))
    assert pose.pose.orientation.w == pytest.approx(math.sqrt(0.5))


def test_body_relative_values_ignore_the_world_convention():
    """IMU rates are body-relative, so ENU must not rotate them."""
    result = converter(coords=utils.CoordinateConverter(utils.CONVENTION_ENU))
    imu = result.convert_imu_to_ros(
        ROBOT_TOPIC,
        {
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            "angular_velocity": {"x": 1.0, "y": 2.0, "z": 3.0},
            "linear_acceleration": {"x": 4.0, "y": 5.0, "z": 6.0},
        },
    )

    assert (imu.angular_velocity.x, imu.angular_velocity.y) == (1.0, -2.0)
    assert imu.linear_acceleration.z == -6.0


def test_imu_gps_and_lidar_conversion():
    result = converter()
    imu = result.convert_imu_to_ros(
        ROBOT_TOPIC,
        {
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            "angular_velocity": {"x": 1.0, "y": 2.0, "z": 3.0},
            "linear_acceleration": {"x": 4.0, "y": 5.0, "z": 6.0},
        },
    )
    gps = result.convert_gps_to_ros(
        ROBOT_TOPIC,
        {"fix_type": 3, "latitude": 47.0, "longitude": 8.0, "altitude": 500.0},
    )
    lidar = result.convert_lidar_to_ros(
        ROBOT_TOPIC,
        {"frame_id": "lidar", "point_cloud": [1.0, 2.0, 3.0]},
    )

    assert isinstance(imu, Imu)
    assert imu.angular_velocity.y == -2.0
    assert isinstance(gps, NavSatFix)
    assert gps.latitude == 47.0
    assert isinstance(lidar, PointCloud2)
    point = list(point_cloud2.read_points(lidar, field_names=("x", "y", "z")))[0]
    assert tuple(point) == (1.0, -2.0, -3.0)


def test_radar_messages_keep_the_legacy_types():
    result = converter()
    scan = result.convert_radar_detection_to_ros(
        ROBOT_TOPIC,
        {
            "radar_detections": [
                {
                    "range": 10.0,
                    "azimuth": 0.2,
                    "elevation": 0.1,
                    "velocity": 3.0,
                    "rcs_sqm": 1.0,
                }
            ]
        },
    )
    tracks = result.convert_radar_track_to_ros(
        ROBOT_TOPIC,
        {
            "radar_tracks": [
                {
                    "position_est": {"x": 1.0, "y": 2.0, "z": 3.0},
                    "velocity_est": {"x": 4.0, "y": 5.0, "z": 6.0},
                    "accel_est": {"x": 7.0, "y": 8.0, "z": 9.0},
                }
            ]
        },
    )

    assert isinstance(scan, RadarScan)
    assert scan.returns[0].range == 10.0
    assert isinstance(tracks, RadarTracks)
    assert tracks.tracks[0].position.y == -2.0

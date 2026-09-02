import struct

import numpy as np

from builtin_interfaces.msg import Time
from radar_msgs.msg import RadarScan, RadarTracks
from sensor_msgs.msg import Image, Imu, NavSatFix, PointCloud2
from sensor_msgs_py import point_cloud2

from projectairsim_rosbridge.msg_converter import MsgConverter


class FakeROSNode:
    PointCloud2 = point_cloud2

    @staticmethod
    def get_time_now_msg():
        return Time(sec=12, nanosec=34)


ROBOT_TOPIC = "/airsim_node/robots/Drone1/sensors/TestSensor/data"


def converter():
    result = MsgConverter(FakeROSNode())
    result.set_robot_base_frame_ids({ROBOT_TOPIC: "airsim_node/robots/Drone1"})
    return result


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


def test_depth_image_is_scaled_to_mono8():
    result = converter()
    depth_bytes = struct.pack("=HH", 0, result.max_depth_mm)
    image = result.convert_image_to_ros(
        ROBOT_TOPIC,
        {
            "encoding": "16UC1",
            "height": 1,
            "width": 2,
            "big_endian": False,
            "data": depth_bytes,
        },
    )

    assert image.encoding == "mono8"
    assert list(image.data) == [0, 255]


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

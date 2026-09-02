import os

import numpy as np
import rclpy
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image

from projectairsim_ros2 import ROS2Node


os.environ.setdefault("ROS_LOG_DIR", "/tmp/projectairsim_ros_test_logs")


def test_runtime_node_name_honors_ros_remapping():
    rclpy.init(args=["--ros-args", "-r", "__node:=remapped_bridge"])
    ros_node = ROS2Node("projectairsim")
    try:
        assert ros_node.name == "remapped_bridge"
    finally:
        ros_node.destroy()
        if rclpy.ok():
            rclpy.shutdown()


def test_camera_info_accepts_numpy_calibration_values():
    camera_info = CameraInfo()
    values = [np.float32(1.0)]
    ROS2Node.SensorHelper().set_camera_info(
        camera_info,
        values * 5,
        values * 9,
        values * 9,
        values * 12,
    )

    assert list(camera_info.d) == [1.0] * 5
    assert list(camera_info.k) == [1.0] * 9
    assert list(camera_info.p) == [1.0] * 12


def test_sensor_and_latched_qos_profiles():
    rclpy.init()
    ros_node = ROS2Node("projectairsim_qos_test")
    try:
        sensor_publisher = ros_node.create_publisher(
            "/sensor", Image, latch=False, qos_profile="sensor_data"
        )
        latched_publisher = ros_node.create_publisher(
            "/latched", Image, latch=True, queue_size=1
        )

        sensor_info = ros_node.native_node.get_publishers_info_by_topic("/sensor")[0]
        latched_info = ros_node.native_node.get_publishers_info_by_topic("/latched")[0]
        assert sensor_info.qos_profile.reliability == ReliabilityPolicy.BEST_EFFORT
        assert sensor_info.qos_profile.durability == DurabilityPolicy.VOLATILE
        assert latched_info.qos_profile.reliability == ReliabilityPolicy.RELIABLE
        assert latched_info.qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL

        sensor_publisher.destroy()
        sensor_publisher.destroy()
        latched_publisher.destroy()
    finally:
        ros_node.destroy()
        if rclpy.ok():
            rclpy.shutdown()

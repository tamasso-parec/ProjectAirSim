import os

import numpy as np
import rclpy
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
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


def test_simulation_timestamps_convert_to_ros_time():
    """Project AirSim timestamps are integer nanoseconds."""
    rclpy.init()
    ros_node = ROS2Node("projectairsim_time_test")
    try:
        stamp = ros_node.get_time_to_msg(
            ros_node.get_time_from_nanos(5250000000)
        )

        assert (stamp.sec, stamp.nanosec) == (5, 250000000)
        # Nanosecond precision must survive, not be rounded through a float.
        precise = ros_node.get_time_to_msg(
            ros_node.get_time_from_nanos(1234567890123456789)
        )
        assert (precise.sec, precise.nanosec) == (1234567890, 123456789)
    finally:
        ros_node.destroy()
        if rclpy.ok():
            rclpy.shutdown()


def test_clock_publisher_is_reliable_volatile_and_shallow():
    """
    rclpy and rclcpp both subscribe to /clock best-effort, and a reliable
    publisher is compatible with best-effort and reliable subscribers alike.
    """
    rclpy.init()
    ros_node = ROS2Node("projectairsim_clock_qos_test")
    try:
        publisher = ros_node.create_publisher(
            "/clock", Clock, latch=False, queue_size=1, qos_profile="default"
        )
        info = ros_node.native_node.get_publishers_info_by_topic("/clock")[0]

        # History depth is not carried in DDS discovery, so it is not
        # observable here; reliability and durability are what decide whether
        # a use_sim_time subscriber matches at all.
        assert info.qos_profile.reliability == ReliabilityPolicy.RELIABLE
        assert info.qos_profile.durability == DurabilityPolicy.VOLATILE

        publisher.destroy()
    finally:
        ros_node.destroy()
        if rclpy.ok():
            rclpy.shutdown()


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

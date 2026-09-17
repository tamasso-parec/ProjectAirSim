"""End-to-end check that the bridge's /clock actually drives simulated time.

The unit tests for SimTimeSource use a fake node.  This exercises the real
rclpy path instead: a real publisher, real DDS, and a real second node running
with use_sim_time, which is the consumer the feature exists for.  It is the
test that would catch a QoS mismatch or a nanosecond-precision loss that the
fakes cannot see.
"""
import os
import time

import rclpy
import rclpy.node

from projectairsim_rosbridge.sim_time import SimTimeSource
from projectairsim_ros2 import ROS2Node


os.environ.setdefault("ROS_LOG_DIR", "/tmp/projectairsim_ros_test_logs")

# Project AirSim's steppable clock starts at zero, so these are small values a
# long way from wall-clock time; a consumer reading wall time instead would be
# obvious.
FIRST_SIM_NANOS = 1_500_000_000
SECOND_SIM_NANOS = 4_250_000_000


def spin_until(node, predicate, timeout_sec=5.0):
    """Spin a node until a predicate holds, returning whether it did."""
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if predicate():
            return True
    return predicate()


def test_published_clock_drives_a_use_sim_time_consumer():
    rclpy.init()
    bridge_node = None
    consumer = None
    sim_time = None
    try:
        bridge_node = ROS2Node("projectairsim_clock_publisher")
        sim_time = SimTimeSource(bridge_node, enabled=True)
        sim_time.start()

        consumer = rclpy.node.Node(
            "projectairsim_clock_consumer",
            parameter_overrides=[
                rclpy.parameter.Parameter(
                    "use_sim_time", rclpy.Parameter.Type.BOOL, True
                )
            ],
        )

        # Wait for discovery before the first clock message, otherwise the
        # volatile publisher can send it before anyone is listening.
        assert spin_until(
            consumer,
            lambda: consumer.count_publishers("/clock") > 0,
        ), "the consumer never discovered the /clock publisher"

        sim_time.update(FIRST_SIM_NANOS)
        assert spin_until(
            consumer,
            lambda: consumer.get_clock().now().nanoseconds == FIRST_SIM_NANOS,
        ), (
            "the consumer's clock did not follow /clock; it read "
            f"{consumer.get_clock().now().nanoseconds}"
        )

        # And it must keep following, not latch on the first value.
        sim_time.update(SECOND_SIM_NANOS)
        assert spin_until(
            consumer,
            lambda: consumer.get_clock().now().nanoseconds == SECOND_SIM_NANOS,
        ), (
            "the consumer's clock did not advance; it read "
            f"{consumer.get_clock().now().nanoseconds}"
        )
    finally:
        if sim_time is not None:
            sim_time.stop()
        if consumer is not None:
            consumer.destroy_node()
        if bridge_node is not None:
            bridge_node.destroy()
        if rclpy.ok():
            rclpy.shutdown()


def test_no_clock_is_published_when_simulated_time_is_disabled():
    rclpy.init()
    bridge_node = None
    observer = None
    try:
        # Use a dedicated topic name: within one process, DDS discovery of
        # the /clock publisher from the test above can still be visible here.
        clock_topic = "/projectairsim_disabled_clock"
        bridge_node = ROS2Node("projectairsim_clock_disabled")
        sim_time = SimTimeSource(
            bridge_node, enabled=False, clock_topic=clock_topic
        )
        sim_time.start()
        sim_time.update(FIRST_SIM_NANOS)

        observer = rclpy.node.Node("projectairsim_clock_observer")
        spin_until(observer, lambda: False, timeout_sec=0.5)

        assert observer.count_publishers(clock_topic) == 0
    finally:
        if observer is not None:
            observer.destroy_node()
        if bridge_node is not None:
            bridge_node.destroy()
        if rclpy.ok():
            rclpy.shutdown()

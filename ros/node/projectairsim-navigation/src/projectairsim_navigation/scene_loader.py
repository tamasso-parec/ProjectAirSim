"""Load one Project AirSim scene through the ROS bridge."""

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def main():
    rclpy.init()
    node = Node("projectairsim_scene_loader")
    node.declare_parameter("scene_config", "scene_basic_drone.jsonc")
    node.declare_parameter("load_scene_topic", "/ProjectAirSim/node/projectairsim/load_scene")
    node.declare_parameter("wait_timeout_sec", 15.0)
    scene_config = node.get_parameter("scene_config").value
    topic = node.get_parameter("load_scene_topic").value
    timeout = node.get_parameter("wait_timeout_sec").value
    publisher = node.create_publisher(String, topic, 1)
    node.get_logger().info(f"Waiting for the bridge scene loader on {topic}")
    deadline = time.monotonic() + timeout
    while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if publisher.get_subscription_count() == 0:
        node.get_logger().error(
            f"Bridge did not subscribe to {topic} within {timeout:.1f} seconds"
        )
    else:
        publisher.publish(String(data=scene_config))
        node.get_logger().info(f"Requested Project AirSim scene: {scene_config}")
        rclpy.spin_once(node, timeout_sec=0.5)
    node.destroy_node()
    rclpy.shutdown()

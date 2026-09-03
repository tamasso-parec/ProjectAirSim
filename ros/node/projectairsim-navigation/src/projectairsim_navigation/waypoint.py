"""Publish one ENU waypoint from the command line."""

import argparse
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


def main():
    parser = argparse.ArgumentParser(description="Send one Project AirSim navigation waypoint")
    parser.add_argument("x", type=float)
    parser.add_argument("y", type=float)
    parser.add_argument("z", type=float)
    parser.add_argument("--yaw", type=float, default=0.0, help="ENU yaw in degrees")
    parser.add_argument("--topic", default="/navigation/goal")
    args = parser.parse_args()
    rclpy.init()
    node = Node("waypoint_input")
    publisher = node.create_publisher(PoseStamped, args.topic, 1)
    deadline = time.monotonic() + 2.0
    while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    message = PoseStamped()
    message.header.stamp = node.get_clock().now().to_msg()
    message.header.frame_id = "enu"
    message.pose.position.x, message.pose.position.y, message.pose.position.z = args.x, args.y, args.z
    yaw = math.radians(args.yaw)
    message.pose.orientation.z, message.pose.orientation.w = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
    publisher.publish(message)
    rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.shutdown()

"""Launch the minimal navigation stack against an already-running bridge."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("projectairsim_navigation"), "config", "navigation.yaml"
    )
    robot_path = LaunchConfiguration("robot_path")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_path",
                description="Discovered Project AirSim robot topic path, beginning with /ProjectAirSim",
            ),
            Node(
                package="projectairsim_navigation",
                executable="ground_truth_adapter",
                output="screen",
                parameters=[config],
                remappings=[("actual_pose", [robot_path, "/actual_pose"])],
            ),
            Node(
                package="projectairsim_navigation",
                executable="simple_navigator",
                output="screen",
                parameters=[config],
                remappings=[("cmd_vel", [robot_path, "/cmd_vel"])],
            ),
        ]
    )

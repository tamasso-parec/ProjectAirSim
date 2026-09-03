"""Launch the Project AirSim bridge and minimal navigation stack together."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bridge_launch = os.path.join(
        get_package_share_directory("projectairsim_ros2"),
        "launch",
        "projectairsim_bridge_ros2.launch.py",
    )
    navigation_launch = os.path.join(
        get_package_share_directory("projectairsim_navigation"),
        "launch",
        "navigation.launch.py",
    )
    arguments = [
        DeclareLaunchArgument(
            "robot_path",
            default_value="/Sim/SceneBasicDrone/robots/Drone1",
        ),
        DeclareLaunchArgument("scene_config", default_value="scene_basic_drone.jsonc"),
        DeclareLaunchArgument("sim_config_path", default_value="sim_config/"),
        DeclareLaunchArgument("address", default_value="127.0.0.1"),
        DeclareLaunchArgument("topics_port", default_value="8989"),
        DeclareLaunchArgument("services_port", default_value="8990"),
        DeclareLaunchArgument("takeoff_timeout_sec", default_value="30.0"),
        DeclareLaunchArgument("land_timeout_sec", default_value="60.0"),
    ]
    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bridge_launch),
        launch_arguments={
            "sim_config_path": LaunchConfiguration("sim_config_path"),
            "address": LaunchConfiguration("address"),
            "topics_port": LaunchConfiguration("topics_port"),
            "services_port": LaunchConfiguration("services_port"),
            "cmd_vel_timeout_sec": "0.2",
            "takeoff_timeout_sec": LaunchConfiguration("takeoff_timeout_sec"),
            "land_timeout_sec": LaunchConfiguration("land_timeout_sec"),
        }.items(),
    )
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_launch),
        launch_arguments={"robot_path": LaunchConfiguration("robot_path")}.items(),
    )
    scene_loader = Node(
        package="projectairsim_navigation",
        executable="scene_loader",
        output="screen",
        parameters=[{"scene_config": LaunchConfiguration("scene_config")}],
    )
    return LaunchDescription(arguments + [bridge, scene_loader, navigation])

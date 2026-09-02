import importlib.util
import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


os.environ.setdefault("ROS_LOG_DIR", "/tmp/projectairsim_ros_test_logs")


def test_bridge_launch_description_exposes_all_parameters():
    launch_path = (
        Path(__file__).parents[1]
        / "launch"
        / "projectairsim_bridge_ros2.launch.py"
    )
    spec = importlib.util.spec_from_file_location("projectairsim_launch", launch_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    description = module.generate_launch_description()
    arguments = {
        entity.name
        for entity in description.entities
        if isinstance(entity, DeclareLaunchArgument)
    }
    nodes = [entity for entity in description.entities if isinstance(entity, Node)]

    assert isinstance(description, LaunchDescription)
    assert arguments == {
        "node_name",
        "address",
        "topics_port",
        "services_port",
        "sim_config_path",
        "cmd_vel_timeout_sec",
        "takeoff_timeout_sec",
        "land_timeout_sec",
    }
    assert len(nodes) == 1

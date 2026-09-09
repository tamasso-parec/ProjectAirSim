import importlib.util
import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node


os.environ.setdefault("ROS_LOG_DIR", "/tmp/projectairsim_ros_test_logs")


def test_px4_bridge_launch_description_exposes_all_parameters():
    launch_path = (
        Path(__file__).parents[1]
        / "launch"
        / "projectairsim_px4_bridge.launch.py"
    )
    spec = importlib.util.spec_from_file_location("projectairsim_px4_launch", launch_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    description = module.generate_launch_description()
    arguments = {
        entity.name
        for entity in description.entities
        if isinstance(entity, DeclareLaunchArgument)
    }
    includes = [
        entity for entity in description.entities if isinstance(entity, IncludeLaunchDescription)
    ]
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
        "fcu_url",
        "gcs_url",
        "tgt_system",
        "tgt_component",
        "mavros_namespace",
    }
    # Project AirSim ROS 2 bridge is included, MAVROS is launched as its own node.
    assert len(includes) == 1
    assert len(nodes) == 1
    assert nodes[0].node_package == "mavros"
    assert nodes[0].node_executable == "mavros_node"

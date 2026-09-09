"""Launch the Project AirSim ROS 2 bridge together with MAVROS2 for PX4 offboard control.

MAVROS attaches to PX4 SITL's independent GCS-facing MAVLink endpoint (default
UDP port 14550), leaving Project AirSim's own PX4 control link (UDP port 14540,
the robot config's `controller.px4-settings.control-port`) untouched. See
docs/controllers/px4/px4_sitl.md for the PX4 SITL side of this setup.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bridge_launch_path = os.path.join(
        os.path.dirname(__file__), "projectairsim_bridge_ros2.launch.py"
    )

    arguments = [
        # Project AirSim ROS 2 bridge args (forwarded to projectairsim_bridge_ros2.launch.py)
        DeclareLaunchArgument("node_name", default_value="projectairsim"),
        DeclareLaunchArgument("address", default_value="127.0.0.1"),
        DeclareLaunchArgument("topics_port", default_value="8989"),
        DeclareLaunchArgument("services_port", default_value="8990"),
        DeclareLaunchArgument("sim_config_path", default_value="sim_config/"),
        DeclareLaunchArgument("cmd_vel_timeout_sec", default_value="1.0"),
        DeclareLaunchArgument("takeoff_timeout_sec", default_value="20.0"),
        DeclareLaunchArgument("land_timeout_sec", default_value="60.0"),
        # MAVROS2 args
        DeclareLaunchArgument(
            "fcu_url",
            default_value="udp://:14550@127.0.0.1:14550",
            description="PX4 SITL's GCS-facing MAVLink endpoint (not Project AirSim's control-port)",
        ),
        DeclareLaunchArgument("gcs_url", default_value=""),
        DeclareLaunchArgument("tgt_system", default_value="1"),
        DeclareLaunchArgument("tgt_component", default_value="1"),
        DeclareLaunchArgument("mavros_namespace", default_value="mavros"),
    ]

    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bridge_launch_path),
        launch_arguments=[
            ("node_name", LaunchConfiguration("node_name")),
            ("address", LaunchConfiguration("address")),
            ("topics_port", LaunchConfiguration("topics_port")),
            ("services_port", LaunchConfiguration("services_port")),
            ("sim_config_path", LaunchConfiguration("sim_config_path")),
            ("cmd_vel_timeout_sec", LaunchConfiguration("cmd_vel_timeout_sec")),
            ("takeoff_timeout_sec", LaunchConfiguration("takeoff_timeout_sec")),
            ("land_timeout_sec", LaunchConfiguration("land_timeout_sec")),
        ],
    )

    mavros_node = Node(
        package="mavros",
        executable="mavros_node",
        namespace=LaunchConfiguration("mavros_namespace"),
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "fcu_url": LaunchConfiguration("fcu_url"),
                "gcs_url": LaunchConfiguration("gcs_url"),
                "tgt_system": ParameterValue(
                    LaunchConfiguration("tgt_system"), value_type=int
                ),
                "tgt_component": ParameterValue(
                    LaunchConfiguration("tgt_component"), value_type=int
                ),
            }
        ],
    )

    return LaunchDescription(arguments + [bridge, mavros_node])

"""Launch the Project AirSim ROS 2 bridge with configurable connection values."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument("node_name", default_value="projectairsim"),
        DeclareLaunchArgument("address", default_value="127.0.0.1"),
        DeclareLaunchArgument("topics_port", default_value="8989"),
        DeclareLaunchArgument("services_port", default_value="8990"),
        DeclareLaunchArgument("sim_config_path", default_value="sim_config/"),
        DeclareLaunchArgument("cmd_vel_timeout_sec", default_value="1.0"),
        DeclareLaunchArgument("takeoff_timeout_sec", default_value="20.0"),
        DeclareLaunchArgument("land_timeout_sec", default_value="60.0"),
        DeclareLaunchArgument(
            "interface_profile",
            default_value="",
            description=(
                "Path to an interface profile YAML file mapping Project "
                "AirSim topics and frames onto fixed ROS names. Empty uses "
                "the bridge's default names and conventions."
            ),
        ),
        DeclareLaunchArgument(
            "connect_timeout_sec",
            default_value="60.0",
            description=(
                "How long to keep retrying a connection to Project AirSim "
                "that is refused because the simulator is still starting. "
                "Zero makes a single attempt."
            ),
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description=(
                "Publish /clock from Project AirSim simulation timestamps and "
                "stamp bridged messages with simulation time."
            ),
        ),
    ]

    bridge = Node(
        package="projectairsim_ros2",
        executable="projectairsim_bridge_ros2",
        name=LaunchConfiguration("node_name"),
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "address": LaunchConfiguration("address"),
                "topics_port": ParameterValue(
                    LaunchConfiguration("topics_port"), value_type=int
                ),
                "services_port": ParameterValue(
                    LaunchConfiguration("services_port"), value_type=int
                ),
                "sim_config_path": LaunchConfiguration("sim_config_path"),
                "cmd_vel_timeout_sec": ParameterValue(
                    LaunchConfiguration("cmd_vel_timeout_sec"), value_type=float
                ),
                "takeoff_timeout_sec": ParameterValue(
                    LaunchConfiguration("takeoff_timeout_sec"), value_type=float
                ),
                "land_timeout_sec": ParameterValue(
                    LaunchConfiguration("land_timeout_sec"), value_type=float
                ),
                "interface_profile": LaunchConfiguration("interface_profile"),
                "connect_timeout_sec": ParameterValue(
                    LaunchConfiguration("connect_timeout_sec"), value_type=float
                ),
                "use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"), value_type=bool
                ),
            }
        ],
    )
    return LaunchDescription(arguments + [bridge])

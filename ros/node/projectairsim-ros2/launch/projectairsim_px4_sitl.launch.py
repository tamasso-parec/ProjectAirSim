"""Launch PX4 SITL, the uXRCE-DDS agent and the Project AirSim ROS 2 bridge.

This is the px4_msgs path: a navigation stack talks to PX4 over uXRCE-DDS
(/fmu/in/..., /fmu/out/...), while Project AirSim supplies the physics and
rendering over PX4's MAVLink simulator link on TCP 4560. MAVROS is not
involved; use projectairsim_px4_bridge.launch.py for that instead.

Start-up order matters and is handled here:

  1. PX4 SITL starts and waits for a simulator on TCP 4560, doing nothing else.
  2. The bridge connects to an already-running Project AirSim.
  3. Loading a scene with a px4-api robot is what makes Project AirSim dial
     PX4. Only then does PX4 start its estimator and set a home position.

So Project AirSim itself must already be running before this launch, and the
scene has to be loaded afterwards, either through the bridge's load_scene topic
or by passing `scene:=` here to have it published automatically once the bridge
is up.

Example:

    ros2 launch projectairsim_ros2 projectairsim_px4_sitl.launch.py \\
      px4_dir:=$HOME/PX4-Autopilot \\
      sim_config_path:=$PWD/client/python/example_user_scripts/sim_config \\
      scene:=scene_x500_realsense_ground.jsonc \\
      interface_profile:=$PWD/ros/node/projectairsim-ros2/config/interface_profile_example.yaml
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)


def generate_launch_description():
    launch_dir = os.path.dirname(__file__)

    arguments = [
        # ------------------------------------------------------------------
        # Project AirSim bridge
        # ------------------------------------------------------------------
        DeclareLaunchArgument("node_name", default_value="projectairsim"),
        DeclareLaunchArgument("address", default_value="127.0.0.1"),
        DeclareLaunchArgument("topics_port", default_value="8989"),
        DeclareLaunchArgument("services_port", default_value="8990"),
        DeclareLaunchArgument("sim_config_path", default_value="sim_config/"),
        DeclareLaunchArgument(
            "interface_profile",
            default_value="",
            description=(
                "Interface profile mapping Project AirSim topics and frames "
                "onto the names the navigation stack expects."
            ),
        ),
        DeclareLaunchArgument(
            "connect_timeout_sec",
            default_value="60.0",
            description=(
                "How long the bridge keeps retrying a refused connection. "
                "Unreal is started alongside it here and only opens its "
                "ports once its map has loaded, so the bridge has to outwait "
                "the simulator's cold start."
            ),
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description=(
                "PX4 lock-step decouples simulation time from wall time, so "
                "simulated time is the right default for this path."
            ),
        ),
        DeclareLaunchArgument(
            "scene",
            default_value="",
            description=(
                "Scene config to load once the bridge is up. Loading a scene "
                "with a px4-api robot is what connects Project AirSim to PX4. "
                "Empty leaves it to you to publish load_scene yourself."
            ),
        ),
        DeclareLaunchArgument(
            "scene_load_delay_sec",
            default_value="5.0",
            description=(
                "Head start given to Unreal and the bridge before the scene "
                "is published. The bridge waits for the simulator on its own "
                "and the publication waits for the bridge to subscribe, so "
                "this only avoids a burst of retries on a cold start."
            ),
        ),
        # ------------------------------------------------------------------
        # PX4 SITL
        # ------------------------------------------------------------------
        DeclareLaunchArgument(
            "px4",
            default_value="true",
            description="Start PX4 SITL. Set false to run it yourself.",
        ),
        DeclareLaunchArgument(
            "px4_dir",
            default_value=os.environ.get("PX4_AUTOPILOT_DIR", ""),
            description=(
                "PX4-Autopilot checkout. Defaults to $PX4_AUTOPILOT_DIR."
            ),
        ),
        DeclareLaunchArgument(
            "px4_build",
            default_value="px4_sitl_default",
            description="PX4 build directory name under <px4_dir>/build.",
        ),
        DeclareLaunchArgument(
            "sys_autostart",
            default_value="10020",
            description=(
                "PX4 airframe id. 10020 is the X500 airframe shipped in "
                "config/px4_airframes; install it and rebuild PX4 first."
            ),
        ),
        DeclareLaunchArgument(
            "px4_sim_model",
            default_value="x500",
            description="PX4 model name reported by the airframe.",
        ),
        DeclareLaunchArgument(
            "instance",
            default_value="0",
            description=(
                "PX4 SITL instance id. The simulator TCP port is 4560 plus "
                "this, matching the robot config's tcp-port."
            ),
        ),
        # ------------------------------------------------------------------
        # PX4 estimator
        # ------------------------------------------------------------------
        DeclareLaunchArgument(
            "ekf2_ev_ctrl",
            default_value="0",
            description=(
                "EKF2 external-vision fusion bitmask. 11 fuses SLAM "
                "horizontal and vertical position and yaw."
            ),
        ),
        DeclareLaunchArgument(
            "ekf2_hgt_ref",
            default_value="1",
            description="EKF2 height reference; 3 selects vision.",
        ),
        DeclareLaunchArgument(
            "ekf2_gps_ctrl",
            default_value="7",
            description=(
                "EKF2 GNSS fusion bitmask. Zero disables GNSS, which a "
                "SLAM-local planning frame needs."
            ),
        ),
        DeclareLaunchArgument(
            "ekf2_multi_imu",
            default_value="1",
            description=(
                "PX4 defaults this to 3 for simulation, but the MAVLink "
                "simulator link carries a single IMU."
            ),
        ),
        # ------------------------------------------------------------------
        # uXRCE-DDS agent and vehicle frames
        # ------------------------------------------------------------------
        DeclareLaunchArgument(
            "micro_xrce_agent",
            default_value="true",
            description="Start the Micro XRCE-DDS Agent for PX4's ROS 2 topics.",
        ),
        DeclareLaunchArgument(
            "micro_xrce_agent_cmd",
            default_value="MicroXRCEAgent",
            description="Micro XRCE-DDS Agent executable.",
        ),
        DeclareLaunchArgument("uxrce_port", default_value="8888"),
        DeclareLaunchArgument(
            "static_frames",
            default_value="true",
            description="Publish the vehicle's fixed sensor extrinsics.",
        ),
        DeclareLaunchArgument("vehicle_frame", default_value="x500_realsense"),
    ]

    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "projectairsim_bridge_ros2.launch.py")
        ),
        launch_arguments=[
            ("node_name", LaunchConfiguration("node_name")),
            ("address", LaunchConfiguration("address")),
            ("topics_port", LaunchConfiguration("topics_port")),
            ("services_port", LaunchConfiguration("services_port")),
            ("sim_config_path", LaunchConfiguration("sim_config_path")),
            ("interface_profile", LaunchConfiguration("interface_profile")),
            ("connect_timeout_sec", LaunchConfiguration("connect_timeout_sec")),
            ("use_sim_time", LaunchConfiguration("use_sim_time")),
        ],
    )

    vehicle_frames = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                launch_dir, "projectairsim_x500_realsense_frames.launch.py"
            )
        ),
        condition=IfCondition(LaunchConfiguration("static_frames")),
        launch_arguments=[
            ("vehicle_frame", LaunchConfiguration("vehicle_frame")),
            ("use_sim_time", LaunchConfiguration("use_sim_time")),
        ],
    )

    # PX4 must run from its build directory so that it finds its ROMFS.
    px4_build_dir = PathJoinSubstitution(
        [LaunchConfiguration("px4_dir"), "build", LaunchConfiguration("px4_build")]
    )

    px4 = ExecuteProcess(
        cmd=[
            PathJoinSubstitution([px4_build_dir, "bin", "px4"]),
            "-i",
            LaunchConfiguration("instance"),
        ],
        cwd=px4_build_dir,
        name="px4_sitl",
        output="screen",
        condition=IfCondition(LaunchConfiguration("px4")),
        additional_env={
            "PX4_SYS_AUTOSTART": LaunchConfiguration("sys_autostart"),
            "PX4_SIM_MODEL": LaunchConfiguration("px4_sim_model"),
            "PX4_PARAM_EKF2_EV_CTRL": LaunchConfiguration("ekf2_ev_ctrl"),
            "PX4_PARAM_EKF2_HGT_REF": LaunchConfiguration("ekf2_hgt_ref"),
            "PX4_PARAM_EKF2_GPS_CTRL": LaunchConfiguration("ekf2_gps_ctrl"),
            "PX4_PARAM_EKF2_MULTI_IMU": LaunchConfiguration("ekf2_multi_imu"),
        },
    )

    micro_xrce_agent = ExecuteProcess(
        cmd=[
            LaunchConfiguration("micro_xrce_agent_cmd"),
            "udp4",
            "-p",
            LaunchConfiguration("uxrce_port"),
        ],
        name="micro_xrce_agent",
        output="screen",
        condition=IfCondition(LaunchConfiguration("micro_xrce_agent")),
    )

    # Loading the scene is what makes Project AirSim dial PX4, so it has to
    # wait until the bridge has connected and advertised load_scene.
    load_scene = TimerAction(
        period=LaunchConfiguration("scene_load_delay_sec"),
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "topic",
                    "pub",
                    "--once",
                    [
                        "/ProjectAirSim/node/",
                        LaunchConfiguration("node_name"),
                        "/load_scene",
                    ],
                    "std_msgs/msg/String",
                    ["{data: ", LaunchConfiguration("scene"), "}"],
                ],
                name="load_scene",
                output="screen",
            )
        ],
        condition=IfCondition(
            # An empty scene argument means "do not load one".
            PythonExpression(["'", LaunchConfiguration("scene"), "' != ''"])
        ),
    )

    return LaunchDescription(
        arguments + [px4, micro_xrce_agent, bridge, vehicle_frames, load_scene]
    )

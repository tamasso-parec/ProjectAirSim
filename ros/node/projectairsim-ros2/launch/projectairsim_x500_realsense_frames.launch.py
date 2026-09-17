"""Fixed sensor extrinsics for the X500 + RealSense D435 vehicle.

The Project AirSim bridge can broadcast a sensor's transform itself, but a
stack with its own state estimator usually turns that off (tf.publish_sensor_tf
in an interface profile), because a transform frame can have only one parent.
These static transforms then supply the same fixed geometry, matching the
Gazebo x500_realsense model so that a stack developed against it sees an
identical transform tree.

The geometry comes from the Gazebo model: the camera sits 0.14 m forward and
0.21 m above the body origin, and the optical frame is the body frame rotated
by roll -90 degrees then yaw -90 degrees, which is the standard REP 103 optical
convention of X right, Y down, Z forward.

Frames are published relative to `vehicle_frame`, which is whatever the
estimator publishes as the vehicle's body frame.
"""

import math

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


# Rotation from the body frame (X forward, Y left, Z up) to the camera's
# optical frame (X right, Y down, Z forward), as the roll/pitch/yaw that
# tf2_ros static_transform_publisher applies as Rz(yaw) @ Ry(pitch) @ Rx(roll).
#
# Named rather than inlined so that the value under test is the value used.
BODY_TO_OPTICAL_RPY = (-math.pi / 2.0, 0.0, -math.pi / 2.0)


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument(
            "vehicle_frame",
            default_value="x500_realsense",
            description="Body frame the camera is mounted on.",
        ),
        DeclareLaunchArgument(
            "camera_optical_frame",
            default_value="x500_realsense/realsense_d435/color_optical_frame",
            description="Optical frame the camera's images and points use.",
        ),
        DeclareLaunchArgument(
            "camera_x",
            default_value="0.14",
            description="Camera offset forward of the body origin, metres.",
        ),
        DeclareLaunchArgument(
            "camera_y",
            default_value="0.0",
            description="Camera offset left of the body origin, metres.",
        ),
        DeclareLaunchArgument(
            "camera_z",
            default_value="0.21",
            description="Camera offset above the body origin, metres.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Static transforms are latched, but keep the clock consistent.",
        ),
    ]

    camera_optical_transform = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="projectairsim_camera_optical_frame",
        output="screen",
        arguments=[
            "--x", LaunchConfiguration("camera_x"),
            "--y", LaunchConfiguration("camera_y"),
            "--z", LaunchConfiguration("camera_z"),
            "--roll", str(BODY_TO_OPTICAL_RPY[0]),
            "--pitch", str(BODY_TO_OPTICAL_RPY[1]),
            "--yaw", str(BODY_TO_OPTICAL_RPY[2]),
            "--frame-id", LaunchConfiguration("vehicle_frame"),
            "--child-frame-id", LaunchConfiguration("camera_optical_frame"),
        ],
        parameters=[
            {
                "use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"), value_type=bool
                )
            }
        ],
    )

    return LaunchDescription(arguments + [camera_optical_transform])

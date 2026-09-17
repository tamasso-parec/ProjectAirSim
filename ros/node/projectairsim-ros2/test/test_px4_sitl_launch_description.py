"""Structure of the PX4 SITL launch files.

These load the launch descriptions and inspect them; nothing is executed, so
no simulator, PX4 instance or ROS node is started.
"""
import importlib.util
import math
import os
from pathlib import Path

import pytest

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch_ros.actions import Node


os.environ.setdefault("ROS_LOG_DIR", "/tmp/projectairsim_ros_test_logs")

LAUNCH_DIR = Path(__file__).parents[1] / "launch"


def load_module(name):
    spec = importlib.util.spec_from_file_location(
        name, LAUNCH_DIR / f"{name}.launch.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def load(name):
    return load_module(name).generate_launch_description()


def arguments(description):
    return {
        entity.name
        for entity in description.entities
        if isinstance(entity, DeclareLaunchArgument)
    }


def defaults(description):
    return {
        entity.name: entity.default_value
        for entity in description.entities
        if isinstance(entity, DeclareLaunchArgument)
    }


def entities_of(description, kind):
    return [
        entity for entity in description.entities if isinstance(entity, kind)
    ]


def text_of(substitutions):
    """Flatten a launch default value into a plain string."""
    if substitutions is None:
        return ""
    return "".join(
        getattr(part, "text", getattr(part, "variable_name", ""))
        for part in substitutions
    )


# ---------------------------------------------------------------------------
# PX4 SITL bring-up
# ---------------------------------------------------------------------------


@pytest.fixture(scope="module")
def px4_sitl():
    return load("projectairsim_px4_sitl")


def test_px4_sitl_launch_loads(px4_sitl):
    assert isinstance(px4_sitl, LaunchDescription)


def test_px4_sitl_starts_px4_the_agent_and_the_bridge(px4_sitl):
    processes = entities_of(px4_sitl, ExecuteProcess)
    includes = entities_of(px4_sitl, IncludeLaunchDescription)

    # PX4 and the uXRCE-DDS agent are external processes.
    assert len(processes) == 2
    # The bridge and the vehicle's fixed frames are included launch files.
    assert len(includes) == 2


def test_px4_sitl_exposes_the_arguments_needed_to_place_px4(px4_sitl):
    assert {
        "px4",
        "px4_dir",
        "px4_build",
        "sys_autostart",
        "px4_sim_model",
        "instance",
    } <= arguments(px4_sitl)


def test_px4_sitl_defaults_to_the_shipped_x500_airframe(px4_sitl):
    values = defaults(px4_sitl)

    assert text_of(values["sys_autostart"]) == "10020"
    assert text_of(values["px4_sim_model"]) == "x500"


def test_px4_sitl_defaults_to_simulated_time(px4_sitl):
    """
    Lock-step decouples simulation time from wall time, so wall-clock stamps
    would be wrong for every consumer on this path.
    """
    assert text_of(defaults(px4_sitl)["use_sim_time"]) == "true"


def test_px4_sitl_exposes_the_estimator_parameters_the_stack_retunes(px4_sitl):
    """
    A SLAM-driven stack turns GNSS off and external vision on; these have to
    be reachable without editing the launch file.
    """
    assert {
        "ekf2_ev_ctrl",
        "ekf2_hgt_ref",
        "ekf2_gps_ctrl",
        "ekf2_multi_imu",
    } <= arguments(px4_sitl)


def test_px4_sitl_overrides_the_multi_imu_default(px4_sitl):
    """
    PX4 defaults EKF2_MULTI_IMU to 3 for simulation, but the MAVLink simulator
    link carries a single IMU.
    """
    assert text_of(defaults(px4_sitl)["ekf2_multi_imu"]) == "1"


def test_px4_sitl_loads_the_scene_on_a_delay(px4_sitl):
    """
    Loading a scene is what makes Project AirSim dial PX4, and it cannot
    happen before the bridge has advertised load_scene.
    """
    timers = entities_of(px4_sitl, TimerAction)

    assert len(timers) == 1
    assert "scene_load_delay_sec" in arguments(px4_sitl)


def test_px4_sitl_loads_no_scene_by_default(px4_sitl):
    """Defaulting to a scene would surprise anyone driving it themselves."""
    assert text_of(defaults(px4_sitl)["scene"]) == ""


def test_px4_sitl_can_leave_px4_to_the_caller(px4_sitl):
    """Running PX4 in its own terminal is the normal debugging workflow."""
    assert "px4" in arguments(px4_sitl)
    assert text_of(defaults(px4_sitl)["px4"]) == "true"


# ---------------------------------------------------------------------------
# Vehicle frames
# ---------------------------------------------------------------------------


@pytest.fixture(scope="module")
def frames():
    return load("projectairsim_x500_realsense_frames")


def test_frames_launch_publishes_one_static_transform(frames):
    nodes = entities_of(frames, Node)

    assert len(nodes) == 1
    assert nodes[0].node_package == "tf2_ros"
    assert nodes[0].node_executable == "static_transform_publisher"


def test_frames_launch_defaults_match_the_gazebo_camera_mount(frames):
    values = defaults(frames)

    assert float(text_of(values["camera_x"])) == pytest.approx(0.14)
    assert float(text_of(values["camera_y"])) == pytest.approx(0.0)
    assert float(text_of(values["camera_z"])) == pytest.approx(0.21)


def test_frames_launch_names_the_frames_the_stack_expects(frames):
    values = defaults(frames)

    assert text_of(values["vehicle_frame"]) == "x500_realsense"
    assert (
        text_of(values["camera_optical_frame"])
        == "x500_realsense/realsense_d435/color_optical_frame"
    )


def test_the_optical_rotation_maps_body_axes_onto_optical_axes(frames):
    """
    The static transform's roll/pitch/yaw must turn the body frame (X forward,
    Y left, Z up) into the optical frame (X right, Y down, Z forward). A sign
    error here tilts every point cloud by 90 degrees, which is easy to miss
    and ruins mapping.
    """
    roll, pitch, yaw = load_module(
        "projectairsim_x500_realsense_frames"
    ).BODY_TO_OPTICAL_RPY

    assert roll == pytest.approx(-math.pi / 2)
    assert pitch == pytest.approx(0.0)
    assert yaw == pytest.approx(-math.pi / 2)

    # Rebuild R = Rz(yaw) @ Ry(pitch) @ Rx(roll) and check it maps the optical
    # axes onto the body axes they are meant to represent.
    def rotation_x(angle):
        c, s = math.cos(angle), math.sin(angle)
        return [[1, 0, 0], [0, c, -s], [0, s, c]]

    def rotation_z(angle):
        c, s = math.cos(angle), math.sin(angle)
        return [[c, -s, 0], [s, c, 0], [0, 0, 1]]

    def multiply(a, b):
        return [
            [sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)]
            for i in range(3)
        ]

    def apply(matrix, vector):
        return [
            sum(matrix[i][j] * vector[j] for j in range(3)) for i in range(3)
        ]

    rotation = multiply(rotation_z(yaw), rotation_x(roll))

    # Optical Z is forward, optical X is right, optical Y is down.
    assert apply(rotation, [0, 0, 1]) == pytest.approx([1, 0, 0], abs=1e-12)
    assert apply(rotation, [1, 0, 0]) == pytest.approx([0, -1, 0], abs=1e-12)
    assert apply(rotation, [0, 1, 0]) == pytest.approx([0, 0, -1], abs=1e-12)

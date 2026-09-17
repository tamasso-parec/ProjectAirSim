"""The X500 + RealSense vehicle configuration, against the Gazebo model it stands in for.

These are the numbers that decide whether a navigation stack tuned on the
Gazebo x500 behaves the same here. A mistyped coefficient would still fly, just
differently, and the difference would be blamed on the planner rather than on
the config -- so the derivation is re-done from the Gazebo source values and
compared, rather than the config's numbers simply being restated.

Everything here is offline: it reads configuration files and never starts a
simulator.
"""
import math
import os
import re
from pathlib import Path

import pytest


REPO_ROOT = Path(__file__).resolve().parents[4]
SIM_CONFIG_DIR = REPO_ROOT / "client" / "python" / "example_user_scripts" / "sim_config"
AIRFRAME_DIR = (
    REPO_ROOT / "ros" / "node" / "projectairsim-ros2" / "config" / "px4_airframes"
)

ROBOT_CONFIG = "robot_x500_realsense_px4_sitl.jsonc"
GROUND_SCENE = "scene_x500_realsense_ground.jsonc"
WALL_SCENE = "scene_x500_realsense_wall.jsonc"
AIRFRAME = "10020_none_x500"

os.environ.setdefault("ROS_LOG_DIR", "/tmp/projectairsim_ros_test_logs")


# ---------------------------------------------------------------------------
# The Gazebo x500 this configuration stands in for.
#
# From PX4's Tools/simulation/gz/models: x500/model.sdf (the motor plugins) and
# x500_base/model.sdf (masses and inertia).
# ---------------------------------------------------------------------------

GZ_MOTOR_CONSTANT = 8.54858e-06  # thrust = motorConstant * w^2, w in rad/s
GZ_MOMENT_CONSTANT = 0.016  # torque = momentConstant * thrust
GZ_MAX_ROT_VELOCITY = 1000.0  # rad/s
GZ_BASE_MASS = 2.0
GZ_ROTOR_MASS = 0.016076923076923075
GZ_IXX = GZ_IYY = 0.02166666666666667
GZ_IZZ = 0.04000000000000001
GZ_ARM = 0.174  # CA_ROTOR*_P{X,Y} magnitude

AIR_DENSITY = 1.225  # Project AirSim's kBaseAirDensity
GRAVITY = 9.80665

# The Gazebo RealSense D435 sensor, from the unseen x500_realsense model.
GZ_CAMERA_WIDTH = 640
GZ_CAMERA_HEIGHT = 480
GZ_CAMERA_FX = 554.25469
GZ_CAMERA_RATE_HZ = 30.0
GZ_CAMERA_OFFSET_FORWARD = 0.14
GZ_CAMERA_OFFSET_UP = 0.21


def gz_max_thrust():
    """Peak thrust of one Gazebo x500 rotor, in newtons."""
    return GZ_MOTOR_CONSTANT * GZ_MAX_ROT_VELOCITY**2


def gz_max_torque():
    """Peak reaction torque of one Gazebo x500 rotor, in newton-metres."""
    return GZ_MOMENT_CONSTANT * gz_max_thrust()


def projectairsim_max_thrust(rotor):
    """
    Peak thrust from Project AirSim's rotor model.

    Mirrors CalcMaxThrustAndTorque in core_sim/include/core_sim/actuators/
    rotor.hpp: max_thrust = Ct * rho * n^2 * D^4, with n in revolutions per
    second.
    """
    revs_per_second = rotor["max-rpm"] / 60.0
    return (
        rotor["coeff-of-thrust"]
        * AIR_DENSITY
        * revs_per_second**2
        * rotor["propeller-diameter"] ** 4
    )


def projectairsim_max_torque(rotor):
    """max_torque = Cq * rho * n^2 * D^5 / (2*pi), from the same source."""
    revs_per_second = rotor["max-rpm"] / 60.0
    return (
        rotor["coeff-of-torque"]
        * AIR_DENSITY
        * revs_per_second**2
        * rotor["propeller-diameter"] ** 5
        / (2.0 * math.pi)
    )


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------


def load_scene(name):
    """Load and schema-validate a scene the same way the simulator does."""
    from projectairsim.utils import load_scene_config_as_dict

    scene, _ = load_scene_config_as_dict(name, str(SIM_CONFIG_DIR))
    return scene


@pytest.fixture(scope="module")
def ground_scene():
    return load_scene(GROUND_SCENE)


@pytest.fixture(scope="module")
def wall_scene():
    return load_scene(WALL_SCENE)


@pytest.fixture(scope="module")
def robot(ground_scene):
    return ground_scene["actors"][0]["robot-config"]


@pytest.fixture(scope="module")
def rotors(robot):
    return {
        actuator["name"]: actuator
        for actuator in robot["actuators"]
        if actuator["type"] == "rotor"
    }


@pytest.fixture(scope="module")
def airframe_params():
    """Parse `param set-default NAME VALUE` out of the shipped PX4 airframe."""
    text = (AIRFRAME_DIR / AIRFRAME).read_text()
    return {
        match.group(1): match.group(2)
        for match in re.finditer(
            r"^param set-default (\S+)\s+(\S+)\s*$", text, re.MULTILINE
        )
    }


@pytest.fixture(scope="module")
def links(robot):
    return {link["name"]: link for link in robot["links"]}


# ---------------------------------------------------------------------------
# Schema validity
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("scene_name", [GROUND_SCENE, WALL_SCENE])
def test_scenes_validate_against_the_shipped_schemas(scene_name):
    """load_scene_config_as_dict validates both scene and robot schemas."""
    scene = load_scene(scene_name)

    assert scene["actors"][0]["robot-config"]["physics-type"] == "fast-physics"


def test_both_scenes_use_the_same_vehicle(ground_scene, wall_scene):
    for scene in (ground_scene, wall_scene):
        assert len(scene["actors"]) == 1
        assert scene["actors"][0]["name"] == "Drone1"


def test_scene_ids_are_distinct(ground_scene, wall_scene):
    """Topic paths are built from the scene id, so a clash would be confusing."""
    assert ground_scene["id"] != wall_scene["id"]


# ---------------------------------------------------------------------------
# Rotor physics
# ---------------------------------------------------------------------------


def test_every_rotor_reproduces_the_gazebo_x500_peak_thrust(rotors):
    assert len(rotors) == 4
    for name, rotor in rotors.items():
        thrust = projectairsim_max_thrust(rotor["rotor-settings"])
        assert thrust == pytest.approx(gz_max_thrust(), rel=1e-6), name


def test_every_rotor_reproduces_the_gazebo_x500_peak_torque(rotors):
    for name, rotor in rotors.items():
        torque = projectairsim_max_torque(rotor["rotor-settings"])
        assert torque == pytest.approx(gz_max_torque(), rel=1e-6), name


def test_max_rpm_matches_the_gazebo_rotor_speed_limit(rotors):
    """1000 rad/s expressed as revolutions per minute."""
    expected_rpm = GZ_MAX_ROT_VELOCITY * 60.0 / (2.0 * math.pi)
    for rotor in rotors.values():
        assert rotor["rotor-settings"]["max-rpm"] == pytest.approx(
            expected_rpm, rel=1e-6
        )


def test_total_mass_matches_the_gazebo_airframe(links):
    total = sum(link["inertial"]["mass"] for link in links.values())
    expected = GZ_BASE_MASS + 4 * GZ_ROTOR_MASS

    assert total == pytest.approx(expected, rel=1e-9)


def test_body_inertia_matches_the_gazebo_base_link(links):
    inertia = links["Frame"]["inertial"]["inertia"]

    assert inertia["type"] == "matrix"
    assert inertia["ixx"] == pytest.approx(GZ_IXX)
    assert inertia["iyy"] == pytest.approx(GZ_IYY)
    assert inertia["izz"] == pytest.approx(GZ_IZZ)


def test_thrust_to_weight_ratio_leaves_usable_control_authority(links, rotors):
    total_thrust = 4 * gz_max_thrust()
    weight = sum(link["inertial"]["mass"] for link in links.values()) * GRAVITY

    ratio = total_thrust / weight

    # The real X500 sits near 1.7; anything at or below 1 cannot hover.
    assert ratio == pytest.approx(1.689, abs=0.01)


def test_hover_throttle_agrees_with_the_px4_airframe(links, airframe_params):
    """
    Project AirSim's rotor model is linear in the normalized actuator command,
    so hover sits at weight / total max thrust. PX4 needs to be told the same
    number or it fights the vehicle on take-off.
    """
    weight = sum(link["inertial"]["mass"] for link in links.values()) * GRAVITY
    hover_fraction = weight / (4 * gz_max_thrust())

    assert hover_fraction == pytest.approx(0.592, abs=0.01)
    assert float(airframe_params["MPC_THR_HOVER"]) == pytest.approx(
        hover_fraction, abs=0.01
    )


# ---------------------------------------------------------------------------
# Rotor geometry and control allocation
# ---------------------------------------------------------------------------


def rotor_position(actuator):
    return [float(value) for value in actuator["origin"]["xyz"].split()]


def test_rotor_arms_match_the_px4_airframe_geometry(rotors, airframe_params):
    """
    PX4 allocates control from CA_ROTOR*_P{X,Y}. If the simulated arms differ,
    PX4's torque predictions are wrong and the attitude loop mistunes.
    """
    expected = {
        "Prop_FR_actuator": (0, GZ_ARM, GZ_ARM),
        "Prop_RL_actuator": (1, -GZ_ARM, -GZ_ARM),
        "Prop_FL_actuator": (2, GZ_ARM, -GZ_ARM),
        "Prop_RR_actuator": (3, -GZ_ARM, GZ_ARM),
    }
    for name, (index, px, py) in expected.items():
        x, y, _ = rotor_position(rotors[name])
        assert x == pytest.approx(px), name
        assert y == pytest.approx(py), name
        assert float(airframe_params[f"CA_ROTOR{index}_PX"]) == pytest.approx(px)
        assert float(airframe_params[f"CA_ROTOR{index}_PY"]) == pytest.approx(py)


def test_turning_directions_match_the_airframe_moment_signs(
    rotors, airframe_params
):
    """
    CA_ROTOR*_KM's sign is the rotor's spin direction. Getting one wrong makes
    yaw control push the wrong way, which reads as an unflyable vehicle.
    """
    index_by_actuator = {
        "Prop_FR_actuator": 0,
        "Prop_RL_actuator": 1,
        "Prop_FL_actuator": 2,
        "Prop_RR_actuator": 3,
    }
    directions = {}
    for name, index in index_by_actuator.items():
        km = float(airframe_params[f"CA_ROTOR{index}_KM"])
        directions[rotors[name]["rotor-settings"]["turning-direction"]] = (
            directions.get(
                rotors[name]["rotor-settings"]["turning-direction"], set()
            )
            | {math.copysign(1.0, km)}
        )

    # Each turning direction must map onto exactly one sign of KM, and the two
    # directions must map onto opposite signs.
    assert set(directions) == {"clock-wise", "counter-clock-wise"}
    assert all(len(signs) == 1 for signs in directions.values())
    assert directions["clock-wise"] != directions["counter-clock-wise"]


def test_diagonal_rotors_spin_the_same_way(rotors):
    """A quad X cancels yaw torque by pairing opposite arms."""
    settings = {
        name: rotor["rotor-settings"]["turning-direction"]
        for name, rotor in rotors.items()
    }

    assert settings["Prop_FR_actuator"] == settings["Prop_RL_actuator"]
    assert settings["Prop_FL_actuator"] == settings["Prop_RR_actuator"]
    assert settings["Prop_FR_actuator"] != settings["Prop_FL_actuator"]


def test_actuator_order_follows_the_px4_rotor_indices(robot, airframe_params):
    """
    The PX4 controller maps actuator outputs positionally, so this list has to
    be in CA_ROTOR index order or the vehicle flips on take-off.
    """
    order = [
        entry["id"]
        for entry in robot["controller"]["px4-settings"]["actuator-order"]
    ]

    assert order == [
        "Prop_FR_actuator",
        "Prop_RL_actuator",
        "Prop_FL_actuator",
        "Prop_RR_actuator",
    ]
    assert int(airframe_params["CA_ROTOR_COUNT"]) == len(order)


def test_rotors_push_upward(rotors):
    """Z is down in this frame, so a lifting rotor's normal is negative."""
    for name, rotor in rotors.items():
        normal = [
            float(value)
            for value in rotor["rotor-settings"]["normal-vector"].split()
        ]
        assert normal == pytest.approx([0.0, 0.0, -1.0]), name


# ---------------------------------------------------------------------------
# PX4 wiring
# ---------------------------------------------------------------------------


def test_the_airframe_does_not_select_the_gazebo_bridge(airframe_params):
    """
    This is the whole point of a separate airframe: PX4's own x500 airframe
    sets SIM_GZ_EN and PX4_SIMULATOR=gz, which routes it to Gazebo instead of
    the MAVLink simulator link Project AirSim connects on.
    """
    # Only the directives matter; the comments above them name these very
    # variables while explaining why they are absent.
    directives = "\n".join(
        line
        for line in (AIRFRAME_DIR / AIRFRAME).read_text().splitlines()
        if line.strip() and not line.lstrip().startswith("#")
    )

    assert "SIM_GZ_EN" not in directives
    assert "PX4_SIMULATOR" not in directives
    assert "PX4_GZ" not in directives
    # It must still be a multicopter airframe.
    assert "rc.mc_defaults" in directives


def test_the_airframe_routes_actuators_through_pwm_main(airframe_params):
    """The MAVLink simulator link carries actuators as PWM main channels."""
    for channel in range(1, 5):
        assert int(airframe_params[f"PWM_MAIN_FUNC{channel}"]) == 100 + channel


def test_the_robot_uses_the_px4_controller_in_lock_step(robot):
    controller = robot["controller"]
    settings = controller["px4-settings"]

    assert controller["type"] == "px4-api"
    # Lock-step is what lets simulation time stretch under a slow render
    # instead of starving PX4 of sensor data.
    assert settings["lock-step"] is True
    assert settings["use-tcp"] is True
    assert settings["tcp-port"] == 4560


def test_the_scene_home_position_matches_the_px4_local_origin(
    ground_scene, robot
):
    """
    PX4 places its home at LPE_LAT/LPE_LON. If the scene's home-geo-point
    disagrees, PX4's home ends up somewhere other than the vehicle.
    """
    home = ground_scene["home-geo-point"]
    parameters = robot["controller"]["px4-settings"]["parameters"]

    assert home["latitude"] == pytest.approx(parameters["LPE_LAT"])
    assert home["longitude"] == pytest.approx(parameters["LPE_LON"])


# ---------------------------------------------------------------------------
# Simulation clock
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("scene_name", [GROUND_SCENE, WALL_SCENE])
def test_scenes_use_a_steppable_clock_fast_enough_for_px4(scene_name):
    """
    Lock-step needs a steppable clock, and PX4 integrates its IMU at 250 Hz
    (IMU_INTEG_RATE), so the step has to be at least that fast.
    """
    clock = load_scene(scene_name)["clock"]

    assert clock["type"] == "steppable"
    step_hz = 1e9 / clock["step-ns"]
    assert step_hz >= 250.0
    assert clock["pause-on-start"] is False


# ---------------------------------------------------------------------------
# Camera
# ---------------------------------------------------------------------------


def camera(robot):
    cameras = [s for s in robot["sensors"] if s["type"] == "camera"]
    assert len(cameras) == 1
    return cameras[0]


def capture(robot, image_type):
    for setting in camera(robot)["capture-settings"]:
        if setting["image-type"] == image_type:
            return setting
    raise AssertionError(f"no capture setting for image type {image_type}")


def test_the_camera_publishes_colour_and_planar_depth(robot):
    """
    Image type 1 is planar (Z) depth, the quantity a Gazebo depth camera and a
    RealSense report. Type 2 is perspective distance and would read as a
    curved wall.
    """
    enabled = {
        setting["image-type"]
        for setting in camera(robot)["capture-settings"]
        if setting["capture-enabled"]
    }

    assert enabled == {0, 1}


def test_camera_intrinsics_match_the_gazebo_realsense(robot):
    """
    Project AirSim derives fx from the field of view as
    (width / 2) / tan(fov / 2); see core_sim/src/sensors/camera.cpp. A stack
    calibrated against the Gazebo model must see the same focal length.
    """
    for image_type in (0, 1):
        setting = capture(robot, image_type)
        assert setting["width"] == GZ_CAMERA_WIDTH
        assert setting["height"] == GZ_CAMERA_HEIGHT

        fx = (setting["width"] / 2) / math.tan(
            math.radians(setting["fov-degrees"]) / 2
        )
        # Gazebo's SDF rounds the horizontal FOV to 1.0472 rad rather than
        # exactly 60 degrees, which moves fx by under 0.002 pixels.
        assert fx == pytest.approx(GZ_CAMERA_FX, abs=0.01)


def test_the_principal_point_differs_from_gazebo_by_half_a_pixel(robot):
    """
    A known, deliberate difference: Project AirSim puts the principal point at
    width/2, Gazebo at (width + 1)/2. Recorded here so it is not mistaken for
    a calibration error later.
    """
    setting = capture(robot, 0)
    projectairsim_cx = setting["width"] / 2
    gazebo_cx = (setting["width"] + 1) / 2

    assert gazebo_cx - projectairsim_cx == pytest.approx(0.5)


def test_the_camera_runs_at_the_gazebo_frame_rate(robot):
    interval = camera(robot)["capture-interval"]

    assert 1.0 / interval == pytest.approx(GZ_CAMERA_RATE_HZ, rel=1e-3)


def test_the_camera_is_mounted_where_the_gazebo_model_puts_it(robot):
    """
    The Gazebo model mounts the D435 0.14 m forward and 0.21 m above the body
    origin. Z is down here, so the height is negative.
    """
    x, y, z = (float(value) for value in camera(robot)["origin"]["xyz"].split())

    assert x == pytest.approx(GZ_CAMERA_OFFSET_FORWARD)
    assert y == pytest.approx(0.0)
    assert z == pytest.approx(-GZ_CAMERA_OFFSET_UP)


def test_depth_is_not_requested_as_float_pixels(robot):
    """
    Project AirSim ignores pixels-as-float for depth and always sends 16-bit
    millimetres, so asking for floats would only mislead a reader.
    """
    assert capture(robot, 1)["pixels-as-float"] is False
    assert capture(robot, 1)["compress"] is False


# ---------------------------------------------------------------------------
# Sensors PX4 needs
# ---------------------------------------------------------------------------


def test_the_vehicle_carries_the_sensors_px4_requires(robot):
    """
    PX4 fuses these over the MAVLink simulator link; a missing one leaves the
    estimator unable to initialise, which surfaces as a vehicle that never
    sets a home position.
    """
    types = {sensor["type"] for sensor in robot["sensors"] if sensor["enabled"]}

    assert {"imu", "gps", "barometer", "magnetometer"} <= types


# ---------------------------------------------------------------------------
# Wall scene
# ---------------------------------------------------------------------------


def test_the_wall_scene_spawns_one_obstacle(wall_scene):
    objects = wall_scene["spawn-objects"]["sim-packaged"]

    assert len(objects) == 1
    assert objects[0]["name"] == "Wall"
    assert objects[0]["physics-enabled"] is False


def test_the_wall_stands_between_the_vehicle_and_the_goal(wall_scene):
    """
    The planner scenarios fly to roughly 12 m ahead, so an obstacle that is
    not in the way makes the scenario meaningless.
    """
    wall = wall_scene["spawn-objects"]["sim-packaged"][0]
    wall_x, wall_y, wall_z = (
        float(value) for value in wall["origin"]["xyz"].split()
    )
    thickness, width, height = (
        float(value) for value in wall["scale"].split()
    )
    vehicle_x = float(
        wall_scene["actors"][0]["origin"]["xyz"].split()[0]
    )

    assert vehicle_x < wall_x < 12.0
    # Wide and tall enough that the goal cannot be reached by skirting it at
    # the altitudes the planner is allowed to use.
    assert width >= 6.0
    assert height >= 4.0
    assert thickness > 0.0
    # Centred on its own half-height so the slab rests on the ground.
    assert wall_z == pytest.approx(-height / 2)
    assert wall_y == pytest.approx(0.0)

"""
Copyright (C) Microsoft Corporation. 
Copyright (C) 2025 IAMAI CONSULTING CORP
MIT License.
ROS bridge for Project AirSim: Callback, topic path, and type conversion utilities
"""

import math
import re

import geometry_msgs.msg as rosgeommsg


# --------------------------------------------------------------------------
class Callbacks:
    """
    Manages a list of callback functions.
    """

    def __init__(self):
        """
        Constructor.
        """
        self.callbacks = []

    def __bool__(self):
        """
        Return true if there are callbacks registered, false otherwise.
        """
        return bool(self.callbacks)

    def __call__(self, *args, **kwargs):
        """
        Invoke all registered callbacks.

        Arguments:
            *args - Position arguments to callback function
            **kwargs - Named arguments to callback function
        """
        for callback in self.callbacks:
            callback(*args, **kwargs)

    def __len__(self):
        """
        Return the number of registered callback functions.
        """
        return len(self.callbacks)

    def add(self, callback) -> bool:
        """
        Add a callback function.  If the function is already on the list it is not added again.

        Arguments:
            callback - The function to add

        Return:
            (Return) - True if callback was added, False if callback was already on the list.
        """
        if not callable(callback):
            raise TypeError(f"specified callback object is not callable: {callback}")

        if not self.callbacks:
            self.callbacks = [callback]
            subscriber_added = True
        else:
            if callback in self.callbacks:
                subscriber_added = False
            else:
                self.callbacks.append(callback)
                subscriber_added = True

        return subscriber_added

    def remove(self, callback):
        """
        Remove a callback function.

        Arguments:
            callback - The function to remove
        """
        self.callbacks.remove(callback)


# --------------------------------------------------------------------------
# Topic Path Extraction Functions
# --------------------------------------------------------------------------

# Regular expression to extract robot's base transform frame ID, everything
# past the initial forward-slash ("/") up to the forward-slash terminating
# the name
_re_get_robot_base_frame_id = re.compile("/(.*/robots/[^/]*)")

# Regular expression to extract robot name, everything up to forward-slash
# ("/") or end-of-string terminating the name
_re_get_robot_name = re.compile(".*/robots/([^/]*)")

# Regular expression to extract robot path, everything up to forward-slash
# ("/") terminating the name
_re_get_robot_path = re.compile("(.*/robots/[^/]*)")

# Regular expression to extract robot's sensor path, everything up to
# forward-slash ("/") terminating the name
_re_get_sensor_path = re.compile("(.*/sensors/[^/]*)")

# Regular expression to extract robot's sensor transform frame ID, everything
# past the initial forward-slash ("/") up to the forward-slash terminating
# the name
_re_get_sensor_frame_id = re.compile("/(.*/sensors/[^/]*)")


def get_robot_path(projectairsim_topic_name: str) -> str:
    """
    Given an Project AirSim topic name, return the robot path which contains the
    initial portion of the topic name ending with the robot name minus the
    terminating forward-slash.

    Arguments:
        projectairsim_topic_name - Project AirSim topic name

    Returns:
        (return)  The robot name path or None if the projectairsim_topic_name
            doesn't contain a robot name path
    """
    match = _re_get_robot_path.match(projectairsim_topic_name)
    return None if match is None else match.group(1)


def get_robot_frame_id(projectairsim_topic_name: str) -> str:
    """
    Given an Project AirSim topic name, return the transform frame ID for the
    robot's base.

    Arguments:
        projectairsim_topic_name - Project AirSim topic name

    Returns:
        (return)  The transform frame ID for the robot's base, or None
            if projectairsim_topic_name does not contain a robot name
    """
    match = _re_get_robot_base_frame_id.match(projectairsim_topic_name)
    return None if match is None else match.group(1)


def get_robot_name(projectairsim_topic_name: str) -> str:
    """
    Given an Project AirSim topic name, return the robot name minus the
    leading and terminating forward-slashes.

    Arguments:
        projectairsim_topic_name - Project AirSim topic name

    Returns:
        (return)  The robot name  or None if the projectairsim_topic_name
            doesn't contain a robot name
    """
    match = _re_get_robot_name.match(projectairsim_topic_name)
    return None if match is None else match.group(1)


def get_sensor_path(projectairsim_topic_name: str) -> str:
    """
    Given an Project AirSim topic name, return the sensor path which contains
    the initial portion of the topic name ending with the sensor name minus
    the terminating forward-slash.

    Arguments:
        projectairsim_topic_name - Project AirSim topic name

    Returns:
        (return)  The sensor name path or None if the projectairsim_topic_name
            doesn't contain a sensor name path
    """
    match = _re_get_sensor_path.match(projectairsim_topic_name)
    return None if match is None else match.group(1)


def get_sensor_frame_id(projectairsim_topic_name: str) -> str:
    """
    Given an Project AirSim topic name, return the transform frame ID for a
    robot's sensor.

    Arguments:
        projectairsim_topic_name - Project AirSim topic name

    Returns:
        (return)  The transform frame ID for the robot's base, or None
            if projectairsim_topic_name does not contain a sensor name
    """
    match = _re_get_sensor_frame_id.match(projectairsim_topic_name)
    return None if match is None else match.group(1)



# --------------------------------------------------------------------------
# Project AirSim / ROS Coordinate Conversion
# --------------------------------------------------------------------------

# Project AirSim expresses world poses in NED (X north, Y east, Z down) and
# body-relative quantities in FRD (X forward, Y right, Z down).  ROS uses a
# Z-up world and an FLU (X forward, Y left, Z up) body frame.
#
# Two separate conversions are therefore needed:
#
#   * Body-relative vectors (IMU rates and accelerations, magnetometer body
#     field, sensor-frame point clouds) always convert FRD -> FLU, which is a
#     180-degree rotation about the body X axis: (x, -y, -z).  This does not
#     depend on the world convention.
#
#   * World quantities (robot and sensor poses, velocity requests) convert NED
#     into whichever Z-up world convention the bridge is configured for.
#
# CONVENTION_NWU is the bridge's long-standing behaviour and remains the
# default: NED -> North-West-Up, a 180-degree rotation about the world X axis.
# It is cheap and self-consistent, but it is *not* standard ENU, so a
# world-aligned "map" frame has X pointing north rather than east.
#
# CONVENTION_ENU converts NED -> East-North-Up, the ROS standard (REP 103).
# It is a 180-degree rotation about the world (1, 1, 0)/sqrt(2) axis.
#
# Both world conversions are involutions (applying one twice is the identity),
# so a single implementation serves both the to-ROS and from-ROS directions.

CONVENTION_NWU = "nwu"
CONVENTION_ENU = "enu"

CONVENTIONS = (CONVENTION_NWU, CONVENTION_ENU)

# Body FRD <-> FLU as a quaternion (180 degrees about X), in (w, x, y, z).
_QUAT_BODY = (0.0, 1.0, 0.0, 0.0)

# World NED <-> NWU as a quaternion (180 degrees about X), in (w, x, y, z).
_QUAT_WORLD_NWU = (0.0, 1.0, 0.0, 0.0)

# World NED <-> ENU as a quaternion (180 degrees about (1, 1, 0)/sqrt(2)),
# in (w, x, y, z).
_SQRT_HALF = math.sqrt(0.5)
_QUAT_WORLD_ENU = (0.0, _SQRT_HALF, _SQRT_HALF, 0.0)


def _quat_multiply(lhs, rhs):
    """
    Hamilton product of two (w, x, y, z) quaternion tuples.
    """
    lw, lx, ly, lz = lhs
    rw, rx, ry, rz = rhs
    return (
        lw * rw - lx * rx - ly * ry - lz * rz,
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
    )


def _quat_inverse(quat):
    """
    Conjugate of a *unit* (w, x, y, z) quaternion, which is also its inverse.
    """
    w, x, y, z = quat
    return (w, -x, -y, -z)


class CoordinateConverter:
    """
    Converts positions, vectors and orientations between Project AirSim and
    ROS conventions.

    A converter is immutable and cheap to construct.  Instances are held by
    the bridge's topic managers so that every message handler shares one
    configured convention.
    """

    def __init__(self, convention: str = CONVENTION_NWU):
        """
        Constructor.

        Arguments:
            convention - World frame convention, "nwu" (default, the bridge's
                historical behaviour) or "enu" (ROS standard, REP 103)
        """
        convention = str(convention).lower()
        if convention not in CONVENTIONS:
            raise ValueError(
                f'unknown frame convention "{convention}", '
                f"expected one of {', '.join(CONVENTIONS)}"
            )

        self.convention = convention

        if convention == CONVENTION_ENU:
            self._world_left = _QUAT_WORLD_ENU
        else:
            self._world_left = _QUAT_WORLD_NWU

        self._world_right = _quat_inverse(_QUAT_BODY)

    @property
    def is_enu(self) -> bool:
        """
        Return whether this converter produces standard ENU world values.
        """
        return self.convention == CONVENTION_ENU

    # ----------------------------------------------------------------------
    # Body-relative quantities (convention-independent)
    # ----------------------------------------------------------------------

    def body_vector(self, xyz):
        """
        Convert a body-relative vector between FRD and FLU.

        Arguments:
            xyz - Iterable of three components

        Returns:
            (return) - Converted (x, y, z) tuple
        """
        x, y, z = (float(value) for value in xyz)
        return (x, -y, -z)

    # ----------------------------------------------------------------------
    # World quantities (convention-dependent)
    # ----------------------------------------------------------------------

    def world_vector(self, xyz):
        """
        Convert a world-frame vector between NED and the configured ROS world
        convention.

        Arguments:
            xyz - Iterable of three components

        Returns:
            (return) - Converted (x, y, z) tuple
        """
        x, y, z = (float(value) for value in xyz)
        if self.convention == CONVENTION_ENU:
            return (y, x, -z)
        return (x, -y, -z)

    def world_quaternion(self, wxyz):
        """
        Convert an orientation between Project AirSim NED/FRD and the
        configured ROS world convention with an FLU body frame.

        Arguments:
            wxyz - Iterable of four components in (w, x, y, z) order

        Returns:
            (return) - Converted (w, x, y, z) tuple
        """
        quat = tuple(float(value) for value in wxyz)
        return _quat_multiply(
            _quat_multiply(self._world_left, quat), self._world_right
        )


# Process-wide default converter.  The bridge replaces this during start-up so
# that the module-level helper functions below, which are part of the bridge's
# published surface, follow the configured convention.
_default_converter = CoordinateConverter(CONVENTION_NWU)


def get_default_coordinate_converter() -> CoordinateConverter:
    """
    Return the process-wide default coordinate converter.
    """
    return _default_converter


def set_default_coordinate_converter(converter: CoordinateConverter):
    """
    Set the process-wide default coordinate converter.

    Arguments:
        converter - The CoordinateConverter to use
    """
    global _default_converter
    if not isinstance(converter, CoordinateConverter):
        raise TypeError("converter must be a CoordinateConverter instance")
    _default_converter = converter


# --------------------------------------------------------------------------
# Project AirSim / ROS Coordinate Conversion Functions
#
# These free functions operate on the process-wide default converter and are
# retained as the bridge's stable conversion API.
# --------------------------------------------------------------------------


def to_projectairsim_position(ros_vector):
    """
    Convert a world position from the ROS world convention to Project AirSim
    NED.
    """
    x, y, z = _default_converter.world_vector(
        (ros_vector.x, ros_vector.y, ros_vector.z)
    )
    return {"x": x, "y": y, "z": z}


def to_projectairsim_angular_rotation(ros_vector):
    """
    Convert a world angular rotation vector from the ROS world convention to
    Project AirSim NED.
    """
    x, y, z = _default_converter.world_vector(
        (ros_vector.x, ros_vector.y, ros_vector.z)
    )
    return {"x": x, "y": y, "z": z}


def to_projectairsim_quaternion(ros_quaternion: rosgeommsg.Quaternion):
    """
    Convert an orientation from the ROS world convention to Project AirSim
    NED/FRD.
    """
    w, x, y, z = _default_converter.world_quaternion(
        (
            ros_quaternion.w,
            ros_quaternion.x,
            ros_quaternion.y,
            ros_quaternion.z,
        )
    )
    return {"x": x, "y": y, "z": z, "w": w}


def to_ros_point(projectairsim_vector):
    """
    Convert a world position from Project AirSim NED to the ROS world
    convention.
    """
    x, y, z = _default_converter.world_vector(
        (
            projectairsim_vector["x"],
            projectairsim_vector["y"],
            projectairsim_vector["z"],
        )
    )
    return rosgeommsg.Point(x=x, y=y, z=z)


def to_ros_position_list(projectairsim_vector):
    """
    Convert a world position from Project AirSim NED to the ROS world
    convention.
    """
    return _default_converter.world_vector(
        (
            projectairsim_vector["x"],
            projectairsim_vector["y"],
            projectairsim_vector["z"],
        )
    )


def to_ros_position_list2list(projectairsim_list):
    """
    Convert a world position from Project AirSim NED to the ROS world
    convention.
    """
    return _default_converter.world_vector(projectairsim_list[0:3])


def to_ros_position_vector3(projectairsim_vector):
    """
    Convert a body-relative vector from Project AirSim FRD to ROS FLU.
    """
    x, y, z = _default_converter.body_vector(
        (
            projectairsim_vector["x"],
            projectairsim_vector["y"],
            projectairsim_vector["z"],
        )
    )
    return rosgeommsg.Vector3(x=x, y=y, z=z)


def to_ros_body_vector_list(projectairsim_vector):
    """
    Convert a body-relative vector from Project AirSim FRD to ROS FLU.
    """
    return _default_converter.body_vector(
        (
            projectairsim_vector["x"],
            projectairsim_vector["y"],
            projectairsim_vector["z"],
        )
    )


def to_ros_quaternion(projectairsim_quaternion):
    """
    Convert an orientation from Project AirSim NED/FRD to the ROS world
    convention with an FLU body frame.
    """
    w, x, y, z = _default_converter.world_quaternion(
        (
            projectairsim_quaternion["w"],
            projectairsim_quaternion["x"],
            projectairsim_quaternion["y"],
            projectairsim_quaternion["z"],
        )
    )
    return rosgeommsg.Quaternion(x=x, y=y, z=z, w=w)


def to_ros_quaternion_list(projectairsim_quaternion):
    """
    Convert an orientation from Project AirSim NED/FRD to the ROS world
    convention, returned in (x, y, z, w) order.
    """
    w, x, y, z = _default_converter.world_quaternion(
        (
            projectairsim_quaternion["w"],
            projectairsim_quaternion["x"],
            projectairsim_quaternion["y"],
            projectairsim_quaternion["z"],
        )
    )
    return (x, y, z, w)


def to_ros_quaternion_list2list(projectairsim_list):
    """
    Convert an orientation in (x, y, z, w) order from Project AirSim NED/FRD
    to the ROS world convention, returned in (x, y, z, w) order.
    """
    w, x, y, z = _default_converter.world_quaternion(
        (
            projectairsim_list[3],
            projectairsim_list[0],
            projectairsim_list[1],
            projectairsim_list[2],
        )
    )
    return (x, y, z, w)

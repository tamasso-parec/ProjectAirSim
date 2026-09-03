"""ROS-independent coordinate, trajectory, and control utilities."""

from dataclasses import dataclass
import math
from typing import Iterable, Tuple

Vector3 = Tuple[float, float, float]
Quaternion = Tuple[float, float, float, float]


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def wrap_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def nwu_to_enu_vector(value: Iterable[float]) -> Vector3:
    x, y, z = value
    return (-y, x, z)


def enu_to_nwu_vector(value: Iterable[float]) -> Vector3:
    x, y, z = value
    return (y, -x, z)


def quaternion_multiply(a: Quaternion, b: Quaternion) -> Quaternion:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def nwu_to_enu_quaternion(value: Quaternion) -> Quaternion:
    # Changing the world basis from NWU to ENU is a +90 degree Z rotation.
    half = math.pi / 4.0
    return quaternion_multiply((0.0, 0.0, math.sin(half), math.cos(half)), value)


def yaw_from_quaternion(value: Quaternion) -> float:
    x, y, z, w = value
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def rotate_vector_inverse(value: Vector3, quaternion: Quaternion) -> Vector3:
    """Rotate a world-frame vector into the quaternion's local frame."""
    x, y, z, w = quaternion
    conjugate = (-x, -y, -z, w)
    vector_q = (value[0], value[1], value[2], 0.0)
    rotated = quaternion_multiply(quaternion_multiply(conjugate, vector_q), quaternion)
    return rotated[:3]


def norm(value: Iterable[float]) -> float:
    return math.sqrt(sum(component * component for component in value))


def limit_norm(value: Vector3, maximum: float) -> Vector3:
    magnitude = norm(value)
    if magnitude <= maximum or magnitude == 0.0:
        return value
    scale = maximum / magnitude
    return tuple(component * scale for component in value)  # type: ignore[return-value]


@dataclass(frozen=True)
class Reference:
    position: Vector3
    velocity: Vector3
    yaw: float
    yaw_rate: float = 0.0


class GroundTruthFilter:
    """Timestamped finite-difference velocity estimator with exponential filtering."""

    def __init__(self, alpha: float = 0.2):
        if not 0.0 < alpha <= 1.0:
            raise ValueError("alpha must be in (0, 1]")
        self.alpha = alpha
        self._previous = None
        self.velocity: Vector3 = (0.0, 0.0, 0.0)
        self.yaw_rate = 0.0

    def update(self, stamp: float, position: Vector3, yaw: float):
        if self._previous is not None:
            old_stamp, old_position, old_yaw = self._previous
            dt = stamp - old_stamp
            if dt > 1e-6:
                raw = tuple((position[i] - old_position[i]) / dt for i in range(3))
                self.velocity = tuple(
                    self.alpha * raw[i] + (1.0 - self.alpha) * self.velocity[i]
                    for i in range(3)
                )  # type: ignore[assignment]
                raw_yaw_rate = wrap_angle(yaw - old_yaw) / dt
                self.yaw_rate = self.alpha * raw_yaw_rate + (1.0 - self.alpha) * self.yaw_rate
        self._previous = (stamp, position, yaw)
        return self.velocity, self.yaw_rate


class LineTrajectory:
    """Time-scaled straight line with zero velocity at both ends."""

    def __init__(self, start: Vector3, goal: Vector3, start_yaw: float, goal_yaw: float, max_speed: float):
        if max_speed <= 0.0:
            raise ValueError("max_speed must be positive")
        self.start, self.goal = start, goal
        self.start_yaw, self.yaw_delta = start_yaw, wrap_angle(goal_yaw - start_yaw)
        self.delta = tuple(goal[i] - start[i] for i in range(3))
        # smoothstep has peak derivative 1.5, so this duration respects max_speed.
        self.duration = max(0.1, 1.5 * norm(self.delta) / max_speed)

    def sample(self, elapsed: float) -> Reference:
        u = clamp(elapsed / self.duration, 0.0, 1.0)
        blend = u * u * (3.0 - 2.0 * u)
        blend_rate = 0.0 if u >= 1.0 else 6.0 * u * (1.0 - u) / self.duration
        position = tuple(self.start[i] + blend * self.delta[i] for i in range(3))
        velocity = tuple(blend_rate * component for component in self.delta)
        return Reference(
            position=position,  # type: ignore[arg-type]
            velocity=velocity,  # type: ignore[arg-type]
            yaw=wrap_angle(self.start_yaw + blend * self.yaw_delta),
            yaw_rate=blend_rate * self.yaw_delta,
        )


class Controller:
    """Feed-forward velocity plus position/yaw feedback."""

    def __init__(self, position_gain=1.5, yaw_gain=5.0, max_speed=4.0, max_yaw_rate=math.radians(60.0)):
        if min(position_gain, yaw_gain, max_speed, max_yaw_rate) <= 0.0:
            raise ValueError("controller gains and limits must be positive")
        self.position_gain = position_gain
        self.yaw_gain = yaw_gain
        self.max_speed = max_speed
        self.max_yaw_rate = max_yaw_rate

    def calculate(self, position: Vector3, yaw: float, reference: Reference):
        velocity = tuple(
            reference.velocity[i] + self.position_gain * (reference.position[i] - position[i])
            for i in range(3)
        )
        yaw_rate = reference.yaw_rate + self.yaw_gain * wrap_angle(reference.yaw - yaw)
        return limit_norm(velocity, self.max_speed), clamp(yaw_rate, -self.max_yaw_rate, self.max_yaw_rate)

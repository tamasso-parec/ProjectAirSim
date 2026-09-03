"""ROS 2 nodes for ground-truth adaptation and minimal waypoint navigation."""

import math

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformBroadcaster

from .core import (
    Controller,
    GroundTruthFilter,
    LineTrajectory,
    enu_to_nwu_vector,
    norm,
    nwu_to_enu_quaternion,
    nwu_to_enu_vector,
    rotate_vector_inverse,
    yaw_from_quaternion,
)


def _stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _quaternion_tuple(quaternion):
    return quaternion.x, quaternion.y, quaternion.z, quaternion.w


class GroundTruthAdapter(Node):
    """Converts the bridge's NWU ground-truth pose into a standard ENU odometry stream."""

    def __init__(self):
        super().__init__("ground_truth_adapter")
        self.declare_parameter("actual_pose_topic", "actual_pose")
        self.declare_parameter("odom_topic", "/navigation/odom")
        self.declare_parameter("velocity_filter_alpha", 0.2)
        self.declare_parameter("enu_frame", "enu")
        self.declare_parameter("base_frame", "base_link_enu")

        self.filter = GroundTruthFilter(self.get_parameter("velocity_filter_alpha").value)
        self.publisher = self.create_publisher(Odometry, self.get_parameter("odom_topic").value, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.subscription = self.create_subscription(
            PoseStamped,
            self.get_parameter("actual_pose_topic").value,
            self._pose_callback,
            qos_profile_sensor_data,
        )
        self.received_pose = False
        self.get_logger().info(
            f"Waiting for Project AirSim ground truth on {self.subscription.topic_name}"
        )

    def _pose_callback(self, message):
        if not self.received_pose:
            self.received_pose = True
            self.get_logger().info("Ground-truth pose received; publishing ENU odometry")
        source_position = message.pose.position
        position = nwu_to_enu_vector((source_position.x, source_position.y, source_position.z))
        orientation = nwu_to_enu_quaternion(_quaternion_tuple(message.pose.orientation))
        yaw = yaw_from_quaternion(orientation)
        stamp_seconds = _stamp_seconds(message.header.stamp)
        if stamp_seconds == 0.0:
            stamp = self.get_clock().now().to_msg()
            stamp_seconds = _stamp_seconds(stamp)
        else:
            stamp = message.header.stamp
        world_velocity, yaw_rate = self.filter.update(stamp_seconds, position, yaw)
        body_velocity = rotate_vector_inverse(world_velocity, orientation)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.get_parameter("enu_frame").value
        odom.child_frame_id = self.get_parameter("base_frame").value
        odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = position
        (
            odom.pose.pose.orientation.x,
            odom.pose.pose.orientation.y,
            odom.pose.pose.orientation.z,
            odom.pose.pose.orientation.w,
        ) = orientation
        odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z = body_velocity
        odom.twist.twist.angular.z = yaw_rate
        self.publisher.publish(odom)

        transform = TransformStamped()
        transform.header = odom.header
        transform.child_frame_id = odom.child_frame_id
        transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z = position
        transform.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)


class SimpleNavigator(Node):
    """Plans a smooth straight line and tracks it with a velocity controller."""

    def __init__(self):
        super().__init__("simple_navigator")
        defaults = {
            "odom_topic": "/navigation/odom",
            "goal_topic": "/navigation/goal",
            "trajectory_topic": "/navigation/trajectory",
            "cmd_vel_topic": "cmd_vel",
            "control_rate_hz": 50.0,
            "position_gain": 1.5,
            "yaw_gain": 5.0,
            "max_speed": 4.0,
            "trajectory_speed": 1.5,
            "max_yaw_rate": math.radians(60.0),
            "state_timeout_sec": 0.2,
            "goal_tolerance": 0.25,
            "yaw_tolerance": math.radians(10.0),
            "path_samples": 50,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.controller = Controller(
            self.get_parameter("position_gain").value,
            self.get_parameter("yaw_gain").value,
            self.get_parameter("max_speed").value,
            self.get_parameter("max_yaw_rate").value,
        )
        self.position = None
        self.yaw = 0.0
        self.last_state_time = None
        self.trajectory = None
        self.trajectory_start = None
        self.goal = None
        self.received_odom = False
        self.last_wait_warning = self.get_clock().now()
        self.cmd_publisher = self.create_publisher(Twist, self.get_parameter("cmd_vel_topic").value, 10)
        self.path_publisher = self.create_publisher(Path, self.get_parameter("trajectory_topic").value, 1)
        self.odom_subscription = self.create_subscription(
            Odometry, self.get_parameter("odom_topic").value, self._odom_callback, 10
        )
        self.goal_subscription = self.create_subscription(
            PoseStamped, self.get_parameter("goal_topic").value, self._goal_callback, 10
        )
        self.timer = self.create_timer(1.0 / self.get_parameter("control_rate_hz").value, self._control)
        self.get_logger().info(
            "Navigator ready: waiting for ENU odometry on "
            f"{self.odom_subscription.topic_name}; velocity output is "
            f"{self.cmd_publisher.topic_name}"
        )

    def _odom_callback(self, message):
        if not self.received_odom:
            self.received_odom = True
            self.get_logger().info("ENU odometry received; navigator is ready for waypoints")
        point = message.pose.pose.position
        self.position = (point.x, point.y, point.z)
        self.yaw = yaw_from_quaternion(_quaternion_tuple(message.pose.pose.orientation))
        self.last_state_time = self.get_clock().now()

    def _goal_callback(self, message):
        if self.position is None:
            self.get_logger().warning("Ignoring waypoint until ground-truth odometry is available")
            return
        point = message.pose.position
        goal_yaw = yaw_from_quaternion(_quaternion_tuple(message.pose.orientation))
        self.goal = ((point.x, point.y, point.z), goal_yaw)
        self.trajectory = LineTrajectory(
            self.position,
            self.goal[0],
            self.yaw,
            goal_yaw,
            self.get_parameter("trajectory_speed").value,
        )
        self.trajectory_start = self.get_clock().now()
        self._publish_path(message.header.frame_id or "enu")
        self.get_logger().info("Accepted waypoint in ENU frame")

    def _publish_path(self, frame):
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = frame
        count = self.get_parameter("path_samples").value
        for index in range(count + 1):
            reference = self.trajectory.sample(self.trajectory.duration * index / count)
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = reference.position
            pose.pose.orientation.z = math.sin(reference.yaw / 2.0)
            pose.pose.orientation.w = math.cos(reference.yaw / 2.0)
            path.poses.append(pose)
        self.path_publisher.publish(path)

    def _publish_command(self, velocity=(0.0, 0.0, 0.0), yaw_rate=0.0):
        # cmd_vel is interpreted in the bridge's NWU world frame.
        velocity_nwu = enu_to_nwu_vector(velocity)
        command = Twist()
        command.linear.x, command.linear.y, command.linear.z = velocity_nwu
        command.angular.z = yaw_rate
        self.cmd_publisher.publish(command)

    def _control(self):
        # Stay passive outside an active waypoint. Publishing zero velocity at
        # 50 Hz here would repeatedly invoke MoveByVelocity and override the
        # bridge's takeoff or landing operation.
        if self.trajectory is None:
            return

        now = self.get_clock().now()
        state_stale = self.last_state_time is None or (
            now - self.last_state_time
        ).nanoseconds * 1e-9 > self.get_parameter("state_timeout_sec").value
        if state_stale:
            if (now - self.last_wait_warning).nanoseconds * 1e-9 >= 5.0:
                self.get_logger().warning(
                    "Ground-truth odometry became stale during navigation; "
                    "holding zero velocity. "
                    "Check that a Project AirSim scene is loaded and robot_path is exact."
                )
                self.last_wait_warning = now
            self._publish_command()
            return
        elapsed = (now - self.trajectory_start).nanoseconds * 1e-9
        reference = self.trajectory.sample(elapsed)
        velocity, yaw_rate = self.controller.calculate(self.position, self.yaw, reference)
        position_error = norm(tuple(self.goal[0][i] - self.position[i] for i in range(3)))
        yaw_error = abs(math.atan2(math.sin(self.goal[1] - self.yaw), math.cos(self.goal[1] - self.yaw)))
        if elapsed >= self.trajectory.duration and position_error <= self.get_parameter("goal_tolerance").value and yaw_error <= self.get_parameter("yaw_tolerance").value:
            self.trajectory = None
            # One explicit stop is sufficient. The bridge's command timeout is
            # the secondary failsafe after this message.
            self._publish_command()
            self.get_logger().info("Waypoint reached")
            return
        self._publish_command(velocity, yaw_rate)


def _spin(node_type):
    rclpy.init()
    node = node_type()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def ground_truth_main():
    _spin(GroundTruthAdapter)


def navigator_main():
    _spin(SimpleNavigator)

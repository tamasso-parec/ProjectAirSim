"""
Copyright (C) Microsoft Corporation.
Copyright (C) 2025 IAMAI CONSULTING CORP
MIT License.
ROS bridge for Project AirSim: simulation clock source
"""
import logging
import threading

import rosgraph_msgs.msg as rosgraphmsg

from .node import ROSNode


class SimTimeSource:
    """
    Authoritative timestamp source for messages the bridge publishes, and
    publisher of the ROS ``/clock`` topic.

    Every Project AirSim sensor message carries a ``time_stamp`` field holding
    the simulation clock in nanoseconds.  When simulated time is enabled this
    class stamps bridged messages from that value instead of the wall clock,
    and republishes the simulation clock on ``/clock`` so that ROS nodes
    running with ``use_sim_time`` share the simulator's notion of time.

    Simulated time only advances when the simulator produces data, so
    ``/clock`` starts once a scene containing a robot has been loaded.  This
    mirrors Gazebo, which publishes no clock until its server is running.

    When simulated time is disabled this class delegates to the ROS node's
    wall clock, preserving the bridge's historical behaviour exactly.

    All methods are thread-safe: Project AirSim delivers topic messages on its
    own client threads.
    """

    # Default minimum simulation-time advance between two ``/clock`` messages.
    # One millisecond bounds the clock rate at 1 kHz, which is ample for
    # ``use_sim_time`` consumers even when the simulator steps faster.
    DEFAULT_MIN_STEP_NANOS = 1000000

    # A simulation-time jump backwards larger than this is treated as the
    # simulator having restarted rather than as an out-of-order message.
    DEFAULT_RESET_THRESHOLD_NANOS = 1000000000

    def __init__(
        self,
        ros_node: ROSNode,
        enabled: bool = False,
        clock_topic: str = "/clock",
        min_step_nanos: int = DEFAULT_MIN_STEP_NANOS,
        reset_threshold_nanos: int = DEFAULT_RESET_THRESHOLD_NANOS,
        logger: logging.Logger = None,
    ):
        """
        Constructor.

        Arguments:
            ros_node - Project AirSim ROS node object
            enabled - If true, stamp messages from simulation time and publish
                the ``/clock`` topic; if false, use the ROS wall clock
            clock_topic - Name of the ROS clock topic
            min_step_nanos - Minimum simulation-time advance between published
                clock messages
            reset_threshold_nanos - Backwards jump that is interpreted as a
                simulator restart
            logger - Log message handler
        """
        if min_step_nanos < 0:
            raise ValueError("min_step_nanos cannot be negative")
        if reset_threshold_nanos <= 0:
            raise ValueError("reset_threshold_nanos must be greater than zero")

        self.ros_node = ros_node
        self.clock_topic = clock_topic
        self.min_step_nanos = int(min_step_nanos)
        self.reset_threshold_nanos = int(reset_threshold_nanos)
        self.logger = logger

        self._enabled = bool(enabled)
        self._lock = threading.Lock()
        self._sim_time_nanos = None  # Latest known simulation time
        self._published_nanos = None  # Simulation time of the last /clock message
        self._clock_publisher = None

    @property
    def enabled(self) -> bool:
        """
        Return whether simulated time is in use.
        """
        return self._enabled

    @property
    def sim_time_nanos(self):
        """
        Return the latest known simulation time in nanoseconds, or None if the
        simulator has not produced any timestamped data yet.
        """
        with self._lock:
            return self._sim_time_nanos

    def start(self):
        """
        Advertise the ROS clock topic.  Does nothing when simulated time is
        disabled.
        """
        if not self._enabled or self._clock_publisher is not None:
            return

        # A reliable, volatile, depth-one publisher.  Both rclpy and rclcpp
        # subscribe to /clock best-effort, and a reliable publisher is
        # compatible with best-effort and reliable subscribers alike.
        self._clock_publisher = self.ros_node.create_publisher(
            topic=self.clock_topic,
            msg_type=rosgraphmsg.Clock,
            latch=False,
            queue_size=1,
            qos_profile="default",
        )

    def stop(self):
        """
        Stop publishing the ROS clock topic and free resources.
        """
        publisher = self._clock_publisher
        self._clock_publisher = None
        if publisher is not None:
            publisher.destroy()

    def reset(self):
        """
        Forget the current simulation time.

        The bridge calls this when it loads a new scene, because the
        simulator's clock restarts from zero.
        """
        with self._lock:
            self._sim_time_nanos = None
            self._published_nanos = None

    def update(self, time_stamp_nanos) -> bool:
        """
        Advance the simulation clock from a Project AirSim message timestamp
        and publish ``/clock`` if the clock moved far enough.

        Timestamps arrive from several topics on several threads, so the
        latest value wins and out-of-order messages are ignored.  A large
        backwards jump is taken to mean the simulator restarted.

        Arguments:
            time_stamp_nanos - Simulation time in nanoseconds

        Returns:
            (return) - True if the simulation clock advanced
        """
        if not self._enabled or time_stamp_nanos is None:
            return False

        try:
            nanos = int(time_stamp_nanos)
        except (TypeError, ValueError):
            return False

        # A simulator that has not started stamps messages with zero. Treat
        # that as "no time known yet" so /clock does not latch at zero.
        if nanos <= 0:
            return False

        restarted_from = None
        with self._lock:
            current = self._sim_time_nanos
            if current is not None:
                if nanos <= current:
                    if nanos >= current - self.reset_threshold_nanos:
                        # Out-of-order message from a slower topic.
                        return False
                    # Simulator restarted; follow it backwards.
                    self._published_nanos = None
                    restarted_from = current

            self._sim_time_nanos = nanos

            should_publish = (self._published_nanos is None) or (
                nanos - self._published_nanos >= self.min_step_nanos
            )
            if should_publish:
                self._published_nanos = nanos

        if restarted_from is not None and self.logger is not None:
            # A clock that jumps backwards is worth saying out loud: it makes
            # consumers on simulated time discard their history.
            self.logger.info(
                "Simulation clock jumped backwards from "
                f"{restarted_from / 1e9:.3f} s to {nanos / 1e9:.3f} s; "
                "treating it as a simulator restart"
            )

        if should_publish and self._clock_publisher is not None:
            # A fresh message per publication: timestamps arrive on several
            # client threads, and a shared message object could be mutated by
            # one thread while another is publishing it.
            clock_message = rosgraphmsg.Clock()
            clock_message.clock = self.ros_node.get_time_to_msg(
                self.ros_node.get_time_from_nanos(nanos)
            )
            self._clock_publisher.publish(clock_message)

        return True

    def now_msg(self):
        """
        Return a message header timestamp for the current time.

        With simulated time enabled this is the latest simulation time seen,
        falling back to zero before the simulator has produced any data.
        Otherwise it is the ROS node's wall clock.

        Returns:
            (return) - Message header timestamp
        """
        if not self._enabled:
            return self.ros_node.get_time_now_msg()

        nanos = self.sim_time_nanos
        if nanos is None:
            nanos = 0
        return self.ros_node.get_time_to_msg(
            self.ros_node.get_time_from_nanos(nanos)
        )

    def stamp(self, projectairsim_message_data=None):
        """
        Return a message header timestamp for a Project AirSim message,
        advancing the simulation clock from the message's own timestamp.

        Messages that carry no ``time_stamp`` field, such as camera info, are
        stamped with the current time instead.

        Arguments:
            projectairsim_message_data - Project AirSim topic message, or None

        Returns:
            (return) - Message header timestamp
        """
        if not self._enabled:
            return self.ros_node.get_time_now_msg()

        time_stamp_nanos = None
        if isinstance(projectairsim_message_data, dict):
            time_stamp_nanos = projectairsim_message_data.get("time_stamp")

        if time_stamp_nanos is not None:
            self.update(time_stamp_nanos)
            try:
                nanos = int(time_stamp_nanos)
            except (TypeError, ValueError):
                nanos = None
            if nanos is not None and nanos > 0:
                return self.ros_node.get_time_to_msg(
                    self.ros_node.get_time_from_nanos(nanos)
                )

        return self.now_msg()

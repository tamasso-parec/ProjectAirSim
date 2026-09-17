"""
Copyright (C) Microsoft Corporation. 
Copyright (C) 2025 IAMAI CONSULTING CORP
MIT License.
ROS bridge for Project AirSim: Main bridge module
"""

import logging
import re

import geometry_msgs.msg as rosgeommsg
import radar_msgs.msg as rosradarmsg
import sensor_msgs.msg as rossensmsg
import std_msgs.msg as rosstdmsg

try:
    import projectairsim
except ModuleNotFoundError as exc:
    if exc.name != "projectairsim":
        raise
    raise ModuleNotFoundError(
        "The Project AirSim Python client is not available to the interpreter "
        "running the ROS bridge. Install it with that interpreter (for a "
        "system-Python colcon build, run `/usr/bin/python3 -m pip install "
        "--user -e client/python/projectairsim` from the repository root), "
        "or add client/python/projectairsim/src and its dependencies to "
        "PYTHONPATH."
    ) from exc
from projectairsim import ProjectAirSimClient
from projectairsim.utils import projectairsim_log

from . import utils
from .interface_profile import InterfaceProfile
from .msg_converter import MsgConverter
from .node import ROSNode
from .sim_time import SimTimeSource
from .topic_helpers import (
    BasicBridgeToROS,
    BasicROSSubscriber,
    CameraBridgeToROS,
    SensorBridgeToROS,
    RobotControlHandler,
    RobotPoseBridgeToROS,
    TopicsManagers,
)


class ProjectAirSimROSBridge:
    """
    This class bridges Project AirSim and ROS by "wrapping" Project AirSim topics
    and presenting them as ROS topics and vice-versa, and publishes
    ROS transform updates for suitable AirSim topics such as robot vehicle
    poses.

    The topic message processing is done by "topic handlers" that act upon
    messages received from a topic (from Project AirSim or ROS.)  Typically
    topic handlers simply convert the message to the data type appropriate for
    the corresponding topic on the "other side" of the bridge and publishes
    the message to the "other side" (i.e., Project AirSim to ROS and ROS to
    Project AirSim.)

    Some messages require more processing such as a robot's actual pose.
    For that, the topic handler converts and publishes the pose
    to a ROS topic and also publishes a ROS transform that is used to
    map robot-relative data (like sensor data) to "map" coordinates.

    Topic managers for the Project AirSim and ROS topics abstract the details
    of topic management away from topic handlers.  Topic handlers need only
    indicate the topics they wish to subscribe to and publish; the topic
    managers handle the details of actually subscribing and publishing
    topics and routing topic messages to subscribing topic handlers.
    """

    # -------------------------------------------------------------------------
    # ProjectAirSimROSBridge Types
    # -------------------------------------------------------------------------

    # --------------------------------------------------------------------------
    class TopicEntry:
        """
        Contains info for a Project AirSim topic name pattern and how we want to
        handle the topic.
        """

        class MatchType:
            """
            How to match a topic name to our name pattern.
            """

            EXACT = 0  # Name matches exactly
            ENDS_WITH = 1  # Name ends with this pattern
            REGEX = 2  # Name matches this regular expression
            REGEX_ANY = 3  # Name matches this regular expression anywhere in the name

        def __init__(
            self,
            match_type: MatchType,
            name_pattern: str,
            ros_message_type,
            topic_handler_type,
            **kwargs,
        ):
            """
            Constructor.

            topic_handler_type is the class that handles the Project AirSim
            topic.
            Typically the topic handler receives the data from a ROS/Project
            AirSim topic, converts the data type and publishes the converted
            data to the corresponding Project AirSim/ROS topic.  A handler is
            created in response to one Project AirSim topic but can publish
            or subscribe to multiple Project AirSim or ROS topics and is not
            otherwise restricted.

            The topic handler class may have additional class-specific
            constructor parameters (see the class documentation.)  Those
            parameters are specified as additional keyword arguments to this
            constructor and store in self.topic_handler_params.  The
            following named arguments are always passed to the topic handler
            class constructor :
                projectairsim_topic_name - The name of the Project Airsim topic
                ros_message_type - The class object for the ROS messages published to or received from the ROS topic
                topics_managers - Project AirSim and ROS topic managers and ROS Transform broadcaster

            Note that the topic handler class must derive the ROS topic name
            from the Project AirSim topic name (usually the same.)

            Arguments:
                match_type - How to match a topic name to our name_pattern
                name_pattern - The string recognition pattern for topics to which this entry applies
                topic_handler_type - The class from which to construct the topic handler
                ros_message_type - The class object for the ROS messages published to or received from the ROS topic
            """
            # ros_message_type must be a class
            if not isinstance(ros_message_type, type):
                raise TypeError(
                    "ros_message_type must be a type or class object, not an instance"
                )

            self.name_pattern = name_pattern
            self.match_type = match_type
            self.topic_handler_type = topic_handler_type
            self.topic_handler_params = kwargs
            self.ros_message_type = ros_message_type
            if (match_type == self.MatchType.REGEX) or (
                match_type == self.MatchType.REGEX_ANY
            ):
                self.regex = re.compile(name_pattern)
            else:
                self.regex = None

        def is_match(self, topic_name):
            """
            Returns whether the specified string matches our topic name pattern.

            Argument:
                topic_name - The string to compare to our topic name pattern

            Returns:
                (Return) - True if the string matches, False otherwise
            """
            if self.match_type == self.MatchType.ENDS_WITH:
                return topic_name.endswith(self.name_pattern)
            elif self.match_type == self.MatchType.REGEX:
                return self.regex.match(topic_name)
            elif self.match_type == self.MatchType.REGEX_ANY:
                return self.regex.search(topic_name)

            return topic_name == self.name_pattern

    # -------------------------------------------------------------------------
    # ProjectAirSimROSBridge Constants
    # -------------------------------------------------------------------------

    # Topic name prefix for persistent (non-scene based) topic handlers
    HANDLER_PREFIX_PERSISTENT = "//./"

    # Project AirSim camera image types that carry depth, and so are the only
    # ones a point cloud can be reprojected from.
    DEPTH_IMAGE_TYPE_SUFFIXES = ("/depth_planar_camera", "/depth_camera")

    # -------------------------------------------------------------------------
    # ProjectAirSimROSBridge Properties
    # -------------------------------------------------------------------------
    @property
    def is_connected(self):
        """
        Returns whether we're connected to Project AirSim as a client
        """
        return (self.projectairsim_client is not None) and self.is_connected_to_client

    # -------------------------------------------------------------------------
    # ProjectAirSimROSBridge Methods
    # -------------------------------------------------------------------------
    def __init__(
        self,
        ros_node: ROSNode,
        address: str = "127.0.0.1",
        port_topics: int = 8989,
        port_services: int = 8990,
        sim_config_path: str = "sim_config/",
        start_ros: bool = True,
        client: ProjectAirSimClient = None,
        logger: logging.Logger = None,
        cmd_vel_timeout_sec: float = 1.0,
        takeoff_timeout_sec: float = 20.0,
        land_timeout_sec: float = 60.0,
        interface_profile=None,
        use_sim_time: bool = False,
    ):
        """
        Constructor.

        If start_ros is False, start_ros() must be called to start processing
        topics.

        An Project AirSim client object that's already connected to Project AirSim
        can be provided via the client argument.  Usually it is left as None
        and a client object is automatically created and connected to Project
        AirSim at the IP address and TCP/IP ports specified by the address,
        port_topics, and port_services arguments.  This client can be retrieved
        as the projectairsim_client attribute.

        When ROS is shutdown, an automatically-created client is also disconnected
        and closed.  If an external client object is passed-in, then the client
        is NOT closed and the caller must disconnect and close the client
        themselves.

        Arguments:
            ros_node - Project AirSim ROS node object
            address - The IP address where Project AirSim is found if client is None
            port_topics - The TCP port where Project AirSim is handling the pub-sub Client API if client is None
            port_services - The TCP port where Project AirSim is handling the services Client API if client is None
            sim_config_path - THe directory containing the simulation config files
            start_ros - If true, ROS topic processing is started immediately
            client - Project AirSim client object (already connected to Project AirSim)
            logger - Logger object; if None, the default Project AirSim logger is used
            cmd_vel_timeout_sec - Velocity-command failsafe duration
            takeoff_timeout_sec - Maximum takeoff service duration
            land_timeout_sec - Maximum landing service duration
            interface_profile - An InterfaceProfile, a path to a profile YAML
                file, or None for the bridge's default names and conventions
            use_sim_time - If true, enable simulated time even when the
                profile does not.  ROS 2 callers pass their node's standard
                use_sim_time parameter here.
        """
        # TODO: make dynamic limits class or rosparam?

        # Resolve the interface profile first: it decides the frame
        # convention, simulated-time behaviour and depth encoding that every
        # other component below is constructed with.
        if isinstance(interface_profile, InterfaceProfile):
            self.interface_profile = interface_profile
        else:
            self.interface_profile = InterfaceProfile.from_file(
                interface_profile if interface_profile else ""
            )

        self.coords = self.interface_profile.create_coordinate_converter()
        # The module-level conversion helpers are part of the bridge's public
        # surface, so point them at the configured convention too.
        utils.set_default_coordinate_converter(self.coords)

        self.sim_time = SimTimeSource(
            ros_node,
            enabled=self.interface_profile.sim_time.enabled or bool(use_sim_time),
            clock_topic=self.interface_profile.sim_time.clock_topic,
            min_step_nanos=self.interface_profile.sim_time.min_step_nanos,
            logger=logger if logger is not None else projectairsim_log(),
        )

        self.msg_converter = MsgConverter(
            ros_node,
            sim_time=self.sim_time,
            coords=self.coords,
            depth=self.interface_profile.depth,
        )  # Topic message converter

        # List of Project AirSim topics we'll bridge to ROS.  Each Project AirSim
        # topic name is matched against this list in this order and the first
        # match is used.
        self.topic_entries = [
            self.TopicEntry(
                self.TopicEntry.MatchType.ENDS_WITH,
                "/gps",
                rossensmsg.NavSatFix,
                topic_handler_type=BasicBridgeToROS,
                message_callback=self.msg_converter.convert_gps_to_ros,
                ros_topic_is_latching=False,
            ),
            self.TopicEntry(
                self.TopicEntry.MatchType.ENDS_WITH,
                "/actual_pose",
                rosgeommsg.PoseStamped,
                topic_handler_type=RobotPoseBridgeToROS,
                message_callback=self.msg_converter.convert_actual_pose_to_ros,
                frame_id_parent="map",
            ),
            self.TopicEntry(
                self.TopicEntry.MatchType.ENDS_WITH,
                "/barometer",
                rossensmsg.FluidPressure,
                topic_handler_type=BasicBridgeToROS,
                message_callback=self.msg_converter.convert_barometer_to_ros,
                ros_topic_is_latching=False,
            ),
            self.TopicEntry(
                self.TopicEntry.MatchType.ENDS_WITH,
                "/depth_camera",
                rossensmsg.Image,
                topic_handler_type=CameraBridgeToROS,
                image_message_callback=self.msg_converter.convert_image_to_ros,
                desired_pose_message_callback=self.msg_converter.convert_desired_pose_from_ros,
                points_message_callback=self.msg_converter.convert_depth_image_to_point_cloud,
            ),
            self.TopicEntry(
                self.TopicEntry.MatchType.ENDS_WITH,
                "/depth_planar_camera",
                rossensmsg.Image,
                topic_handler_type=CameraBridgeToROS,
                image_message_callback=self.msg_converter.convert_image_to_ros,
                desired_pose_message_callback=self.msg_converter.convert_desired_pose_from_ros,
                points_message_callback=self.msg_converter.convert_depth_image_to_point_cloud,
            ),
            self.TopicEntry(
                self.TopicEntry.MatchType.ENDS_WITH,
                "/depth_vis_camera",
                rossensmsg.Image,
                topic_handler_type=CameraBridgeToROS,
                image_message_callback=self.msg_converter.convert_image_to_ros,
                desired_pose_message_callback=self.msg_converter.convert_desired_pose_from_ros,
                points_message_callback=self.msg_converter.convert_depth_image_to_point_cloud,
            ),
            self.TopicEntry(
                self.TopicEntry.MatchType.ENDS_WITH,
                "/disparity_normalized_camera",
                rossensmsg.Image,
                topic_handler_type=CameraBridgeToROS,
                image_message_callback=self.msg_converter.convert_image_to_ros,
                desired_pose_message_callback=self.msg_converter.convert_desired_pose_from_ros,
                points_message_callback=self.msg_converter.convert_depth_image_to_point_cloud,
            ),
            self.TopicEntry(
                self.TopicEntry.MatchType.ENDS_WITH,
                "/imu_kinematics",
                rossensmsg.Imu,
                topic_handler_type=BasicBridgeToROS,
                message_callback=self.msg_converter.convert_imu_to_ros,
                ros_topic_is_latching=False,
            ),
            self.TopicEntry(
                self.TopicEntry.MatchType.ENDS_WITH,
                "/lidar",
                rossensmsg.PointCloud2,
                topic_handler_type=SensorBridgeToROS,
                message_callback=self.msg_converter.convert_lidar_to_ros,
                transform_message_callback=self.msg_converter.convert_lidar_to_ros_transform,
                ros_topic_is_latching=False,
            ),
            self.TopicEntry(
                self.TopicEntry.MatchType.ENDS_WITH,
                "/magnetometer",
                rossensmsg.MagneticField,
                topic_handler_type=BasicBridgeToROS,
                message_callback=self.msg_converter.convert_magnetometer_to_ros,
                ros_topic_is_latching=False,
            ),
            self.TopicEntry(
                self.TopicEntry.MatchType.ENDS_WITH,
                "/radar_detections",
                rosradarmsg.RadarScan,
                topic_handler_type=BasicBridgeToROS,
                message_callback=self.msg_converter.convert_radar_detection_to_ros,
                ros_topic_is_latching=False,
            ),
            self.TopicEntry(
                self.TopicEntry.MatchType.ENDS_WITH,
                "/radar_tracks",
                rosradarmsg.RadarTracks,
                topic_handler_type=BasicBridgeToROS,
                message_callback=self.msg_converter.convert_radar_track_to_ros,
                ros_topic_is_latching=False,
            ),
            self.TopicEntry(
                self.TopicEntry.MatchType.ENDS_WITH,
                "/scene_camera",
                rossensmsg.Image,
                topic_handler_type=CameraBridgeToROS,
                image_message_callback=self.msg_converter.convert_image_to_ros,
                desired_pose_message_callback=self.msg_converter.convert_desired_pose_from_ros,
                points_message_callback=self.msg_converter.convert_depth_image_to_point_cloud,
            ),
            self.TopicEntry(
                self.TopicEntry.MatchType.ENDS_WITH,
                "/segmentation_camera",
                rossensmsg.Image,
                topic_handler_type=CameraBridgeToROS,
                image_message_callback=self.msg_converter.convert_image_to_ros,
                desired_pose_message_callback=self.msg_converter.convert_desired_pose_from_ros,
                points_message_callback=self.msg_converter.convert_depth_image_to_point_cloud,
            ),
            self.TopicEntry(
                self.TopicEntry.MatchType.ENDS_WITH,
                "/surface_normals_camera",
                rossensmsg.Image,
                topic_handler_type=CameraBridgeToROS,
                image_message_callback=self.msg_converter.convert_image_to_ros,
                desired_pose_message_callback=self.msg_converter.convert_desired_pose_from_ros,
                points_message_callback=self.msg_converter.convert_depth_image_to_point_cloud,
            ),
        ]

        if self.interface_profile.collision.enabled:
            self.topic_entries.append(
                self.TopicEntry(
                    self.TopicEntry.MatchType.ENDS_WITH,
                    "/collision_info",
                    # Importing here rather than at module scope keeps
                    # ros_gz_interfaces optional for everyone who does not ask
                    # for Gazebo-shaped collision reports.
                    MsgConverter._gz_interfaces_msgs().Contacts,
                    topic_handler_type=BasicBridgeToROS,
                    message_callback=(
                        self.msg_converter.convert_collision_info_to_gz_contacts
                    ),
                    ros_topic_is_latching=False,
                )
            )

        # Initialize data members
        self.projectairsim_client = client  # Connection to Project AirSim
        self.sim_config_path = (
            sim_config_path  # Directory containing the simulation configuration files
        )
        self.is_client_ours = (
            False  # Whether we created self.projectairsim_client or it was passed to us
        )
        self.is_connected_to_client = (
            False  # Whether self.projectairsim_client is connected to Project AirSim
        )
        self.topic_handlers = {}  # Handlers for each topic
        self.robot_control_handlers = {}  # Per-robot command and service handlers
        self.robot_paths = {}  # Mapping from Project AirSim topic name to robot path
        self.robot_base_frame_ids = (
            {}
        )  # Mapping from Project AirSim topic name to robot's base transform frame ID
        self.ros_node = ros_node  # ROS node
        self.cmd_vel_timeout_sec = float(cmd_vel_timeout_sec)
        self.takeoff_timeout_sec = float(takeoff_timeout_sec)
        self.land_timeout_sec = float(land_timeout_sec)
        self.ros_is_started = False
        self.topics_managers = None  # Topic and transform managers

        if self.cmd_vel_timeout_sec <= 0:
            raise ValueError("cmd_vel_timeout_sec must be greater than zero")
        if self.takeoff_timeout_sec <= 0 or self.land_timeout_sec <= 0:
            raise ValueError("takeoff and land timeouts must be greater than zero")

        if logger is None:
            self.logger = projectairsim_log()
        else:
            self.logger = logger

        # Report the effective profile before connecting, so the settings are
        # on record even when the connection to Project AirSim then fails.
        self._log_interface_profile()

        # Create and connect to Project AirSim client if one wasn't given
        if self.projectairsim_client is not None:
            self.is_client_ours = False
        else:
            self.projectairsim_client = ProjectAirSimClient(
                address, port_topics=port_topics, port_services=port_services
            )
            self.is_client_ours = True
            self.projectairsim_client.connect()
            self.projectairsim_client.get_topic_info()
        self.is_connected_to_client = True

        # Create topic managers
        self.topics_managers = TopicsManagers(
            self.projectairsim_client,
            self.ros_node,
            self.logger,
            sim_time=self.sim_time,
            coords=self.coords,
        )

        # Register ROS shutdown hook
        self.ros_node.on_shutdown(self._on_ros_shutdown)

        # Add persistent topics not dictated by the scene
        self.topic_handlers[
            self.HANDLER_PREFIX_PERSISTENT + "load_scene"
        ] = BasicROSSubscriber(
            ros_topic_name=f"/ProjectAirSim/node/{self.ros_node.name}/load_scene",
            ros_message_type=rosstdmsg.String,
            topics_managers=self.topics_managers,
            message_callback=self._load_scene_message_cb,
        )

        # Start ROS processing, if so directed
        if start_ros:
            self.start_ros()

    def __del__(self):
        """
        Destructor.
        """
        self.clear()

    def clear(self):
        """
        Stop processing and free resources.

        Construction can fail part-way through, for instance on an invalid
        interface profile or a refused connection to Project AirSim, and
        Python still calls the destructor on the half-built object.  Attributes
        are therefore read defensively so that tearing down cannot raise an
        AttributeError that masks the original failure.
        """
        self.stop_ros()
        topics_managers = getattr(self, "topics_managers", None)
        if topics_managers is not None:
            topics_managers.close()
        projectairsim_client = getattr(self, "projectairsim_client", None)
        if projectairsim_client is not None and getattr(
            self, "is_connected_to_client", False
        ):
            self.is_connected_to_client = False
            if getattr(self, "is_client_ours", False):
                projectairsim_client.disconnect()
            self.projectairsim_client = None
        if (topics_managers is not None) and (
            topics_managers.tf_broadcaster is not None
        ):
            topics_managers.tf_broadcaster.clear()
            self.topics_managers = None

        self.ros_node = None

    def start_ros(self):
        """
        Start ROS topic processing.
        """
        self.ros_is_started = True
        self.sim_time.start()
        self.update_topics()
        self.topics_managers.tf_broadcaster.start()

    def stop_ros(self):
        """
        Shutdown ROS topics processing
        """
        topics_managers = getattr(self, "topics_managers", None)
        if topics_managers is not None:
            topics_managers.tf_broadcaster.stop()
        if hasattr(self, "topic_handlers"):
            self._drop_handlers()
        sim_time = getattr(self, "sim_time", None)
        if sim_time is not None:
            sim_time.stop()
        self.ros_is_started = False

    def update_topics(self):
        """
        Update ROS topics by scanning the available topics from Project AirSim and setting up
        handlers to advertise and subscribe to the corresponding ROS topics.

        Existing handlers are reused if possible.  Existing handlers for topics that no longer
        exist are removed.
        """
        topic_handlers_new = {}
        robot_control_handlers_new = {}
        robot_paths_new = {}
        robot_base_frame_ids_new = {}

        # Save persistent topic handlers
        for pair in self.topic_handlers.items():
            if pair[0].startswith(self.HANDLER_PREFIX_PERSISTENT):
                topic_handlers_new[pair[0]] = pair[1]
        for topic in topic_handlers_new:
            del self.topic_handlers[topic]

        # Construct new scene-dependant topic handlers
        if self.projectairsim_client and self.projectairsim_client.topics:
            for topic_name in self.projectairsim_client.topics:
                for topic_entry in self.topic_entries:
                    if topic_entry.is_match(topic_name):
                        # Get the robot path and transform frame ID for this topic
                        robot_path = utils.get_robot_path(topic_name)
                        if robot_path is not None:
                            robot_paths_new[topic_name] = robot_path
                        robot_base_frame_id = self.interface_profile.resolve_frame(
                            robot_path, utils.get_robot_frame_id(topic_name)
                        )
                        if robot_base_frame_id is not None:
                            robot_base_frame_ids_new[topic_name] = robot_base_frame_id

                        # Get topic handler object
                        if topic_name in self.topic_handlers:
                            # Have an existing handler for the topic--reuse it
                            topic_handler = self.topic_handlers.pop(topic_name)
                        else:
                            # Create a new handler for the topic, applying the
                            # interface profile's topic and frame aliases
                            handler_params = dict(topic_entry.topic_handler_params)
                            handler_params.update(
                                self._resolve_handler_params(topic_name, topic_entry)
                            )
                            topic_handler = topic_entry.topic_handler_type(
                                projectairsim_topic_name=topic_name,
                                ros_message_type=topic_entry.ros_message_type,
                                topics_managers=self.topics_managers,
                                **handler_params,
                            )

                        topic_handlers_new[topic_name] = topic_handler
                        break

            # Controls are per robot, not per sensor/topic. This ensures cmd_vel
            # and lifecycle services exist even if desired_pose is unavailable.
            projectairsim_topic_names = set(self.projectairsim_client.topics)
            for robot_path in sorted(set(robot_paths_new.values())):
                desired_pose_topic_name = robot_path + "/desired_pose"
                if desired_pose_topic_name not in projectairsim_topic_names:
                    desired_pose_topic_name = None

                if robot_path in self.robot_control_handlers:
                    handler = self.robot_control_handlers.pop(robot_path)
                else:
                    handler = RobotControlHandler(
                        robot_path=robot_path,
                        topics_managers=self.topics_managers,
                        desired_pose_topic_name=desired_pose_topic_name,
                        cmd_vel_timeout_sec=self.cmd_vel_timeout_sec,
                        takeoff_timeout_sec=self.takeoff_timeout_sec,
                        land_timeout_sec=self.land_timeout_sec,
                        create_lifecycle_services=self.ros_node.supports_lifecycle_services,
                    )
                robot_control_handlers_new[robot_path] = handler

        # Clear handlers that are no longer needed and save new handlers
        self._clear_handlers()  # Skip setting empty dictionaries
        self.topic_handlers = topic_handlers_new
        self.robot_control_handlers = robot_control_handlers_new
        self.robot_paths = robot_paths_new
        self.msg_converter.set_robot_base_frame_ids(robot_base_frame_ids_new)

        # Refresh topic handlers

    def _resolve_handler_params(self, topic_name: str, topic_entry) -> dict:
        """
        Return the topic handler constructor arguments implied by the
        interface profile for one Project AirSim topic.

        Handlers fall into four groups with different naming and transform
        needs, so each is resolved separately.  The returned values override
        the defaults declared on the topic entry.

        Arguments:
            topic_name - Project AirSim topic name
            topic_entry - The TopicEntry matching the topic

        Returns:
            (return) - Constructor keyword arguments
        """
        profile = self.interface_profile
        handler_type = topic_entry.topic_handler_type
        params = {}

        # Cameras publish an image and a camera info topic, so a single alias
        # is not enough to name them.
        if issubclass(handler_type, CameraBridgeToROS):
            camera_topics = profile.resolve_camera_topics(topic_name)
            params["ros_topic_name_image"] = camera_topics.image
            params["ros_topic_name_camera_info"] = camera_topics.camera_info
            params["points_settings"] = profile.points
            if camera_topics.points:
                if topic_name.endswith(self.DEPTH_IMAGE_TYPE_SUFFIXES):
                    params["ros_topic_name_points"] = camera_topics.points
                else:
                    self.logger.warning(
                        f'Ignoring the point cloud topic "{camera_topics.points}" '
                        f'requested for "{topic_name}": a point cloud can only '
                        "be reprojected from a depth image type "
                        f"({', '.join(self.DEPTH_IMAGE_TYPE_SUFFIXES)})"
                    )
            params["publish_tf"] = profile.tf.publish_sensor_tf
            params["frame_id_parent"] = profile.world_frame
            frame_id = profile.resolve_frame(utils.get_sensor_path(topic_name))
            if frame_id is not None:
                params["frame_id"] = frame_id
            return params

        # Other sensors publish one topic and broadcast their own frame.
        if issubclass(handler_type, SensorBridgeToROS):
            ros_topic_name = profile.resolve_topic(topic_name)
            if ros_topic_name is not None:
                params["ros_topic_name"] = ros_topic_name
            params["publish_tf"] = profile.tf.publish_sensor_tf
            params["frame_id_parent"] = profile.world_frame
            frame_id = profile.resolve_frame(utils.get_sensor_path(topic_name))
            if frame_id is not None:
                params["frame_id"] = frame_id
            return params

        # The robot pose names the vehicle's own frame.
        if issubclass(handler_type, RobotPoseBridgeToROS):
            ros_topic_name = profile.resolve_topic(topic_name)
            if ros_topic_name is not None:
                params["ros_topic_name"] = ros_topic_name
            if profile.tf.ground_truth_topic:
                params["ground_truth_tf_topic"] = profile.tf.ground_truth_topic
            params["publish_tf"] = profile.tf.publish_robot_tf
            params["frame_id_parent"] = profile.world_frame
            frame_id = profile.resolve_frame(utils.get_robot_path(topic_name))
            if frame_id is not None:
                params["frame_id"] = frame_id
            return params

        # Everything else just publishes one topic.
        if issubclass(handler_type, BasicBridgeToROS):
            ros_topic_name = profile.resolve_topic(topic_name)
            if ros_topic_name is not None:
                params["ros_topic_name"] = ros_topic_name

        return params

    def _log_interface_profile(self):
        """
        Log the effective interface profile.

        Topic names, frame conventions and simulated time are the settings
        that most often explain "the topic is there but nothing arrives", so
        record what is actually in force.
        """
        profile = self.interface_profile
        self.logger.info(
            f"Interface profile: {profile.source}; "
            f"frame convention {profile.frame_convention}; "
            f"world frame {profile.world_frame}; "
            f"depth {profile.depth.encoding}"
            + (
                f" clamped to {profile.depth.max_range_m} m"
                if profile.depth.max_range_m > 0.0
                else ""
            )
        )
        if profile.sim_time.enabled:
            self.logger.info(
                "Simulated time enabled: publishing "
                f"{profile.sim_time.clock_topic} from Project AirSim "
                "message timestamps. Run ROS nodes with use_sim_time set."
            )
        else:
            self.logger.info(
                "Simulated time disabled: messages are stamped with the ROS "
                "wall clock"
            )
        if not profile.tf.publish_robot_tf:
            self.logger.info("Robot transform broadcast disabled by profile")
        if not profile.tf.publish_sensor_tf:
            self.logger.info("Sensor transform broadcast disabled by profile")
        if profile.tf.ground_truth_topic:
            self.logger.info(
                "Publishing ground truth robot transforms on "
                f"{profile.tf.ground_truth_topic}"
            )
        if profile.collision.enabled:
            self.logger.info(
                f"Collision reporting enabled as {profile.collision.message}"
            )

    def _clear_handlers(self):
        """
        Clear all topic handlers of their resources.
        """
        for pair in self.topic_handlers.items():
            pair[1].clear()
        for handler in self.robot_control_handlers.values():
            handler.clear()
        self.robot_control_handlers = {}

    def _drop_handlers(self):
        """
        Clear all topic handlers of their resources and drop them.
        """
        if self.topics_managers is not None:
            self.topics_managers.cancel_pending_requests()
        self._clear_handlers()
        self.topic_handlers = {}

    def _drop_handlers_for_scene(self):
        """
        Clear scene-related topic handlers of their resources and drop
        them.  Persistent handlers are unaffected.
        """
        self.topics_managers.cancel_pending_requests()
        topic_handlers_new = {}
        for pair in self.topic_handlers.items():
            if pair[0].startswith(self.HANDLER_PREFIX_PERSISTENT):
                topic_handlers_new[pair[0]] = pair[1]
            else:
                pair[1].clear()
                self.topic_handlers[pair[0]] = None

        self.topic_handlers = topic_handlers_new
        for handler in self.robot_control_handlers.values():
            handler.clear()
        self.robot_control_handlers = {}

    def _load_scene_message_cb(self, ros_topic_name, ros_message):
        """
        Handle a message received from the ROS load_scene topic which loads
        the simulation server with a new scene.  This causes us to stop
        publishing ROS topics that aren't in the new scene and start
        publishing ROS topics that are.

        Arguments:
            ros_topic_name - ROS topic name
            ros_message - Message received from the ROS topic
        """
        scene_config = ros_message.data
        self.logger.info(f"Got request to load scene config file: {scene_config}")
        if self.projectairsim_client is not None:
            try:
                # Clear existing scene-based handlers
                self._drop_handlers_for_scene()

                # The simulator restarts its clock with the new scene.
                self.sim_time.reset()

                # Load scene and update ROS topics to match the new scene
                projectairsim.World(
                    client=self.projectairsim_client,
                    scene_config_name=scene_config,
                    sim_config_path=self.sim_config_path,
                )
                if self.ros_is_started:
                    self.update_topics()

                self.logger.info(f"Successfully load scene config file: {scene_config}")
            except Exception as e:
                self.logger.error(
                    f'Failed to load scene config file "{scene_config}": {e}'
                )

    def _on_ros_shutdown(self):
        """
        This function is called when ROS is shutdown so we can shutdown our node.
        """
        self.logger.info("ROS is shutting down--closing down node...")
        self.clear()

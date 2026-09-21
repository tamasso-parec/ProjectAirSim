"""
Copyright (C) Microsoft Corporation.
Copyright (C) 2025 IAMAI CONSULTING CORP
MIT License.
ROS bridge for Project AirSim: interface profile (topic and frame aliases)
"""
import fnmatch
import re

import yaml

from . import utils


class ProfileError(ValueError):
    """
    Raised when an interface profile cannot be understood.
    """


class SimTimeSettings:
    """
    Simulated-time settings from an interface profile.
    """

    def __init__(self, enabled=False, clock_topic="/clock", min_step_ms=1.0):
        self.enabled = bool(enabled)
        self.clock_topic = str(clock_topic)
        min_step_ms = float(min_step_ms)
        if min_step_ms < 0.0:
            raise ProfileError("sim_time.min_step_ms cannot be negative")
        self.min_step_nanos = int(min_step_ms * 1e6)


class DepthSettings:
    """
    Depth-image conversion settings from an interface profile.
    """

    # Metric 32-bit float metres.  Directly consumable by depth_image_proc,
    # RTAB-Map, ORB-SLAM3 and the rest of the ROS depth ecosystem.
    ENCODING_32FC1 = "32FC1"

    # Metric 16-bit unsigned millimetres, as Project AirSim transmits it.
    ENCODING_16UC1 = "16UC1"

    # The bridge's historical output: 8-bit greyscale scaled so that
    # mono8_max_range_m maps to 255.  Useful only for eyeballing an image and
    # lossy enough to be unusable for mapping or SLAM.
    ENCODING_MONO8 = "mono8"

    ENCODINGS = (ENCODING_32FC1, ENCODING_16UC1, ENCODING_MONO8)

    def __init__(
        self,
        encoding=ENCODING_32FC1,
        max_range_m=0.0,
        mono8_max_range_m=6.0,
    ):
        encoding = str(encoding)
        # Accept any capitalisation of the OpenCV-style names.
        for candidate in self.ENCODINGS:
            if encoding.lower() == candidate.lower():
                encoding = candidate
                break
        else:
            raise ProfileError(
                f'unknown depth encoding "{encoding}", '
                f"expected one of {', '.join(self.ENCODINGS)}"
            )

        max_range_m = float(max_range_m)
        if max_range_m < 0.0:
            raise ProfileError("depth.max_range_m cannot be negative")
        mono8_max_range_m = float(mono8_max_range_m)
        if mono8_max_range_m <= 0.0:
            raise ProfileError("depth.mono8_max_range_m must be greater than zero")

        self.encoding = encoding
        self.max_range_m = max_range_m
        self.mono8_max_range_m = mono8_max_range_m


class TFSettings:
    """
    Transform-broadcast settings from an interface profile.

    A ROS transform frame can have only one parent, so a stack that already
    has its own localisation source, such as a PX4 estimator or a SLAM
    front end, must be able to stop the bridge from claiming those frames.
    """

    def __init__(
        self,
        publish_robot_tf=True,
        publish_sensor_tf=True,
        ground_truth_topic="",
    ):
        self.publish_robot_tf = bool(publish_robot_tf)
        self.publish_sensor_tf = bool(publish_sensor_tf)
        self.ground_truth_topic = str(ground_truth_topic or "")


class PointCloudSettings:
    """
    Depth-image-to-point-cloud settings from an interface profile.
    """

    # X right, Y down, Z forward: the optical convention of REP 103, and what
    # depth_image_proc and Gazebo's depth cameras produce.
    CONVENTION_OPTICAL = "optical"

    # X forward, Y left, Z up: the body convention, for a stack whose camera
    # frame is a body frame rather than an optical one.
    CONVENTION_ROS = "ros"

    CONVENTIONS = (CONVENTION_OPTICAL, CONVENTION_ROS)

    def __init__(self, frame_convention=CONVENTION_OPTICAL, decimation=1):
        frame_convention = str(frame_convention).lower()
        if frame_convention not in self.CONVENTIONS:
            raise ProfileError(
                f'unknown points.frame_convention "{frame_convention}", '
                f"expected one of {', '.join(self.CONVENTIONS)}"
            )
        decimation = int(decimation)
        if decimation < 1:
            raise ProfileError("points.decimation must be at least 1")

        self.frame_convention = frame_convention
        self.decimation = decimation

    @property
    def is_optical(self) -> bool:
        return self.frame_convention == self.CONVENTION_OPTICAL


class CollisionSettings:
    """
    Collision-reporting settings from an interface profile.

    Project AirSim reports collisions on each robot's ``collision_info``
    topic.  ROS has no canonical collision message, so the shape is chosen
    explicitly and defaults to not publishing at all.
    """

    # Do not bridge collision info.  The default, and the bridge's behaviour
    # before collision reporting existed.
    MESSAGE_NONE = "none"

    # ros_gz_interfaces/msg/Contacts, what a Gazebo-based stack subscribes to.
    # Requires the ros_gz_interfaces package, which is imported only when this
    # is selected so that it stays an optional dependency.
    MESSAGE_GZ_CONTACTS = "gz_contacts"

    MESSAGES = (MESSAGE_NONE, MESSAGE_GZ_CONTACTS)

    def __init__(self, message=MESSAGE_NONE):
        message = str(message).lower()
        if message not in self.MESSAGES:
            raise ProfileError(
                f'unknown collision.message "{message}", '
                f"expected one of {', '.join(self.MESSAGES)}"
            )
        self.message = message

    @property
    def enabled(self) -> bool:
        return self.message != self.MESSAGE_NONE


class SceneSettings:
    """
    Changes made to the loaded scene so that it matches the world another
    simulator would have presented.

    A photorealistic environment is a level somebody built, and it arrives
    with everything that level contains.  The world an existing stack was
    evaluated against is usually barer, because a Gazebo world holds exactly
    the models the experiment asked for and nothing else.  Comparing results
    across the two therefore means being able to take the level's own
    furniture out of the way, and leaving it in is not a neutral choice: it
    is scenery the other simulator's runs never had to fly through.

    ``remove_objects`` is a list of regular expressions matched against scene
    object names.  Matching objects are destroyed after the scene loads,
    including actors placed in the level rather than spawned by the scene
    config.
    """

    def __init__(self, remove_objects=None):
        if remove_objects is None:
            remove_objects = []
        if isinstance(remove_objects, str) or not isinstance(
            remove_objects, (list, tuple)
        ):
            raise ProfileError(
                "scene.remove_objects must be a list of regular expressions"
            )

        patterns = []
        for pattern in remove_objects:
            if not isinstance(pattern, str) or not pattern:
                raise ProfileError(
                    "scene.remove_objects entries must be non-empty strings"
                )
            try:
                re.compile(pattern)
            except re.error as exc:
                # Caught here rather than at the simulator, which would
                # otherwise just return no matches and quietly leave the
                # objects in place.
                raise ProfileError(
                    f'scene.remove_objects entry "{pattern}" is not a valid '
                    f"regular expression: {exc}"
                ) from exc
            patterns.append(pattern)

        self.remove_objects = patterns

    @property
    def enabled(self) -> bool:
        return bool(self.remove_objects)


class CameraTopicNames:
    """
    Resolved ROS topic names for one Project AirSim camera image type.

    ``points`` is None unless the profile asked this camera for a point
    cloud, which only a depth image type can produce.
    """

    def __init__(self, image: str, camera_info: str, points: str = None):
        self.image = image
        self.camera_info = camera_info
        self.points = points


class InterfaceProfile:
    """
    Maps the scene-dependent Project AirSim topic tree onto fixed ROS names.

    Project AirSim names its topics after the loaded scene and robot, for
    example ``/Sim/SceneBasicDrone/robots/Drone1/sensors/Cam/scene_camera``.
    An existing ROS stack usually expects its own fixed names and transform
    frames instead.  A profile expresses that mapping declaratively, in the
    spirit of ``ros_gz_bridge``'s configuration file, so that Project AirSim
    can stand in for another simulator without modifying the stack.

    Patterns are shell-style globs matched against the full Project AirSim
    topic or object path, and ``*`` also matches ``/``.  Entries are tried in
    file order and the first match wins, so put specific patterns first.

    Every section is optional.  An empty profile reproduces the bridge's
    default behaviour.
    """

    DEFAULT_WORLD_FRAME = "map"

    _KNOWN_KEYS = frozenset(
        {
            "frame_convention",
            "world_frame",
            "sim_time",
            "depth",
            "points",
            "collision",
            "tf",
            "frames",
            "topics",
            "scene",
        }
    )

    def __init__(self, config: dict = None, source: str = "<defaults>"):
        """
        Constructor.

        Arguments:
            config - Parsed profile mapping, or None for defaults
            source - Description of where the profile came from, for messages
        """
        config = {} if config is None else config
        if not isinstance(config, dict):
            raise ProfileError(
                f"{source}: profile must be a mapping, not {type(config).__name__}"
            )

        unknown = sorted(set(config) - self._KNOWN_KEYS)
        if unknown:
            raise ProfileError(
                f"{source}: unknown profile section(s): {', '.join(unknown)}; "
                f"expected any of {', '.join(sorted(self._KNOWN_KEYS))}"
            )

        self.source = source

        self.frame_convention = str(
            config.get("frame_convention", utils.CONVENTION_NWU)
        ).lower()
        if self.frame_convention not in utils.CONVENTIONS:
            raise ProfileError(
                f'{source}: unknown frame_convention "{self.frame_convention}", '
                f"expected one of {', '.join(utils.CONVENTIONS)}"
            )

        self.world_frame = str(config.get("world_frame", self.DEFAULT_WORLD_FRAME))
        if not self.world_frame:
            raise ProfileError(f"{source}: world_frame cannot be empty")

        try:
            self.sim_time = SimTimeSettings(**self._section(config, "sim_time"))
            self.depth = DepthSettings(**self._section(config, "depth"))
            self.points = PointCloudSettings(**self._section(config, "points"))
            self.collision = CollisionSettings(
                **self._section(config, "collision")
            )
            self.tf = TFSettings(**self._section(config, "tf"))
            self.scene = SceneSettings(**self._section(config, "scene"))
        except TypeError as exc:
            # Raised when a section carries an unexpected key.
            raise ProfileError(f"{source}: {exc}") from exc
        except ProfileError as exc:
            raise ProfileError(f"{source}: {exc}") from exc

        self._frames = self._parse_frames(config.get("frames"), source)
        self._topics = self._parse_topics(config.get("topics"), source)

    @staticmethod
    def _section(config: dict, name: str) -> dict:
        section = config.get(name)
        if section is None:
            return {}
        if not isinstance(section, dict):
            raise ProfileError(
                f"{name} must be a mapping, not {type(section).__name__}"
            )
        return dict(section)

    @staticmethod
    def _parse_frames(frames, source: str):
        if frames is None:
            return []
        if not isinstance(frames, dict):
            raise ProfileError(
                f"{source}: frames must be a mapping of pattern to frame ID"
            )

        parsed = []
        for pattern, frame_id in frames.items():
            if not isinstance(frame_id, str) or not frame_id:
                raise ProfileError(
                    f'{source}: frames["{pattern}"] must be a non-empty string'
                )
            parsed.append((str(pattern), frame_id))
        return parsed

    @staticmethod
    def _parse_topics(topics, source: str):
        if topics is None:
            return []
        if not isinstance(topics, dict):
            raise ProfileError(
                f"{source}: topics must be a mapping of pattern to ROS topic name"
            )

        parsed = []
        for pattern, value in topics.items():
            pattern = str(pattern)
            if isinstance(value, str):
                if not value:
                    raise ProfileError(
                        f'{source}: topics["{pattern}"] cannot be empty'
                    )
                parsed.append((pattern, value))
            elif isinstance(value, dict):
                unknown = sorted(set(value) - {"image", "camera_info", "points"})
                if unknown:
                    raise ProfileError(
                        f'{source}: topics["{pattern}"] has unknown key(s): '
                        f"{', '.join(unknown)}; expected image, camera_info "
                        "and points"
                    )
                image = value.get("image")
                if not isinstance(image, str) or not image:
                    raise ProfileError(
                        f'{source}: topics["{pattern}"].image must be a '
                        "non-empty string"
                    )
                camera_info = value.get("camera_info")
                if camera_info is None:
                    camera_info = image + "/camera_info"
                elif not isinstance(camera_info, str) or not camera_info:
                    raise ProfileError(
                        f'{source}: topics["{pattern}"].camera_info must be a '
                        "non-empty string"
                    )
                points = value.get("points")
                if points is not None and (
                    not isinstance(points, str) or not points
                ):
                    raise ProfileError(
                        f'{source}: topics["{pattern}"].points must be a '
                        "non-empty string"
                    )
                parsed.append(
                    (pattern, CameraTopicNames(image, camera_info, points))
                )
            else:
                raise ProfileError(
                    f'{source}: topics["{pattern}"] must be a string or a '
                    "mapping with image and camera_info"
                )
        return parsed

    @classmethod
    def from_file(cls, path: str):
        """
        Load a profile from a YAML file.

        An empty path returns the default profile so that callers can pass a
        launch argument straight through.

        Arguments:
            path - Path to the profile file, or an empty string

        Returns:
            (return) - New InterfaceProfile
        """
        if not path:
            return cls()

        try:
            with open(path, "r", encoding="utf-8") as profile_file:
                config = yaml.safe_load(profile_file)
        except OSError as exc:
            raise ProfileError(f"cannot read interface profile {path}: {exc}") from exc
        except yaml.YAMLError as exc:
            raise ProfileError(f"cannot parse interface profile {path}: {exc}") from exc

        return cls(config, source=path)

    def create_coordinate_converter(self) -> utils.CoordinateConverter:
        """
        Return a coordinate converter for this profile's frame convention.
        """
        return utils.CoordinateConverter(self.frame_convention)

    def resolve_frame(self, projectairsim_path: str, default: str = None):
        """
        Return the ROS transform frame ID for a Project AirSim robot or sensor
        path.

        Arguments:
            projectairsim_path - Project AirSim robot or sensor path
            default - Value to return when no pattern matches

        Returns:
            (return) - Frame ID, or default if no pattern matches
        """
        if projectairsim_path is None:
            return default
        for pattern, frame_id in self._frames:
            if fnmatch.fnmatchcase(projectairsim_path, pattern):
                return frame_id
        return default

    def resolve_topic(self, projectairsim_topic_name: str, default: str = None):
        """
        Return the ROS topic name for a Project AirSim topic.

        Arguments:
            projectairsim_topic_name - Project AirSim topic name
            default - Value to return when no pattern matches

        Returns:
            (return) - ROS topic name, or default if no pattern matches

        Raises:
            ProfileError - The matching entry names camera topics, which this
                Project AirSim topic cannot use
        """
        alias = self._match_topic(projectairsim_topic_name)
        if alias is None:
            return default
        if isinstance(alias, CameraTopicNames):
            raise ProfileError(
                f"{self.source}: the entry for "
                f'"{projectairsim_topic_name}" names image and camera_info '
                "topics, but that Project AirSim topic is not a camera"
            )
        return alias

    def resolve_camera_topics(
        self, projectairsim_topic_name: str, default_base_name: str = None
    ) -> CameraTopicNames:
        """
        Return the ROS image and camera-info topic names for a Project AirSim
        camera topic.

        A profile entry may give either an explicit ``image``/``camera_info``
        pair or a single base name, in which case ``/image`` and
        ``/camera_info`` are appended as the bridge does by default.

        A point cloud is published only when the entry names a ``points``
        topic, because only a depth image type can produce one.

        Arguments:
            projectairsim_topic_name - Project AirSim camera topic name
            default_base_name - Base name to use when no pattern matches

        Returns:
            (return) - CameraTopicNames for the camera
        """
        alias = self._match_topic(projectairsim_topic_name)
        if isinstance(alias, CameraTopicNames):
            return alias

        base_name = alias if alias is not None else default_base_name
        if base_name is None:
            base_name = projectairsim_topic_name
        return CameraTopicNames(
            base_name + "/image", base_name + "/camera_info"
        )

    def _match_topic(self, projectairsim_topic_name: str):
        if projectairsim_topic_name is None:
            return None
        for pattern, alias in self._topics:
            if fnmatch.fnmatchcase(projectairsim_topic_name, pattern):
                return alias
        return None

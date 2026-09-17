"""Interface-profile aliases reaching the topic handlers.

These cover ProjectAirSimROSBridge._resolve_handler_params, which translates a
profile into the constructor arguments for each kind of topic handler.  It is
where a mistake would silently leave a stack listening on the wrong topic.
"""
import geometry_msgs.msg as rosgeommsg
import sensor_msgs.msg as rossensmsg

import pytest

from projectairsim_rosbridge.interface_profile import InterfaceProfile
from projectairsim_rosbridge.ros_bridge import ProjectAirSimROSBridge
from projectairsim_rosbridge.topic_helpers import (
    BasicBridgeToROS,
    CameraBridgeToROS,
    RobotPoseBridgeToROS,
    SensorBridgeToROS,
)

SCENE = "/Sim/SceneBasicDrone/robots/Drone1"
POSE_TOPIC = SCENE + "/actual_pose"
CAMERA_TOPIC = SCENE + "/sensors/RGBD/scene_camera"
DEPTH_TOPIC = SCENE + "/sensors/RGBD/depth_planar_camera"
LIDAR_TOPIC = SCENE + "/sensors/Lidar1/lidar"
IMU_TOPIC = SCENE + "/sensors/IMU1/imu_kinematics"

MatchType = ProjectAirSimROSBridge.TopicEntry.MatchType


def entry(name_pattern, ros_message_type, handler_type, **kwargs):
    return ProjectAirSimROSBridge.TopicEntry(
        MatchType.ENDS_WITH, name_pattern, ros_message_type, handler_type, **kwargs
    )


class RecordingLogger:
    def __init__(self):
        self.warnings = []

    def warning(self, message):
        self.warnings.append(message)


def bridge(profile):
    instance = ProjectAirSimROSBridge.__new__(ProjectAirSimROSBridge)
    instance.interface_profile = profile
    instance.logger = RecordingLogger()
    return instance


UNSEEN_PROFILE = {
    "frame_convention": "enu",
    "world_frame": "map",
    "tf": {"publish_robot_tf": False, "publish_sensor_tf": False},
    "frames": {
        "*/robots/Drone1": "x500_realsense",
        "*/robots/Drone1/sensors/RGBD": "x500_realsense/realsense_d435/color_optical_frame",
    },
    "topics": {
        "*/robots/Drone1/actual_pose": "/ground_truth/pose",
        "*/robots/Drone1/sensors/RGBD/scene_camera": "/rgbd_camera",
        "*/robots/Drone1/sensors/RGBD/depth_planar_camera": {
            "image": "/rgbd_camera/depth_image",
            "camera_info": "/rgbd_camera/depth_camera_info",
            "points": "/rgbd_camera/points",
        },
        "*/robots/Drone1/sensors/Lidar1/lidar": "/points",
    },
}


@pytest.fixture
def aliased():
    return bridge(InterfaceProfile(UNSEEN_PROFILE))


@pytest.fixture
def default():
    return bridge(InterfaceProfile())


def test_default_profile_keeps_the_bridges_own_naming(default):
    camera = default._resolve_handler_params(
        CAMERA_TOPIC,
        entry("/scene_camera", rossensmsg.Image, CameraBridgeToROS),
    )

    assert camera["ros_topic_name_image"] == CAMERA_TOPIC + "/image"
    assert camera["ros_topic_name_camera_info"] == CAMERA_TOPIC + "/camera_info"
    assert camera["publish_tf"] is True
    assert camera["frame_id_parent"] == "map"
    # Without a frames entry the handler keeps deriving its own frame ID.
    assert "frame_id" not in camera


def test_default_profile_leaves_simple_topics_alone(default):
    params = default._resolve_handler_params(
        IMU_TOPIC, entry("/imu_kinematics", rossensmsg.Imu, BasicBridgeToROS)
    )

    assert params == {}


def test_camera_base_name_alias_expands_to_image_and_camera_info(aliased):
    params = aliased._resolve_handler_params(
        CAMERA_TOPIC,
        entry("/scene_camera", rossensmsg.Image, CameraBridgeToROS),
    )

    assert params["ros_topic_name_image"] == "/rgbd_camera/image"
    assert params["ros_topic_name_camera_info"] == "/rgbd_camera/camera_info"
    assert (
        params["frame_id"]
        == "x500_realsense/realsense_d435/color_optical_frame"
    )
    assert params["publish_tf"] is False


def test_explicit_camera_topics_are_used_verbatim(aliased):
    params = aliased._resolve_handler_params(
        DEPTH_TOPIC,
        entry("/depth_planar_camera", rossensmsg.Image, CameraBridgeToROS),
    )

    assert params["ros_topic_name_image"] == "/rgbd_camera/depth_image"
    assert (
        params["ros_topic_name_camera_info"] == "/rgbd_camera/depth_camera_info"
    )


def test_colour_and_depth_of_one_camera_share_its_frame(aliased):
    colour = aliased._resolve_handler_params(
        CAMERA_TOPIC, entry("/scene_camera", rossensmsg.Image, CameraBridgeToROS)
    )
    depth = aliased._resolve_handler_params(
        DEPTH_TOPIC,
        entry("/depth_planar_camera", rossensmsg.Image, CameraBridgeToROS),
    )

    assert colour["frame_id"] == depth["frame_id"]


def test_sensor_handlers_receive_the_topic_alias_and_tf_setting(aliased):
    params = aliased._resolve_handler_params(
        LIDAR_TOPIC,
        entry(
            "/lidar",
            rossensmsg.PointCloud2,
            SensorBridgeToROS,
            message_callback=lambda *args: None,
            transform_message_callback=lambda *args: None,
        ),
    )

    assert params["ros_topic_name"] == "/points"
    assert params["publish_tf"] is False
    assert params["frame_id_parent"] == "map"
    # No frames entry for this sensor, so its own derived frame ID stands.
    assert "frame_id" not in params


def test_robot_pose_handler_receives_the_robot_frame_and_tf_setting(aliased):
    params = aliased._resolve_handler_params(
        POSE_TOPIC,
        entry(
            "/actual_pose",
            rosgeommsg.PoseStamped,
            RobotPoseBridgeToROS,
            frame_id_parent="map",
        ),
    )

    assert params["ros_topic_name"] == "/ground_truth/pose"
    assert params["frame_id"] == "x500_realsense"
    assert params["publish_tf"] is False


def test_resolved_params_override_the_topic_entry_defaults(aliased):
    """
    update_topics() merges these over the entry's declared parameters, so the
    profile must win where the two overlap.
    """
    topic_entry = entry(
        "/actual_pose",
        rosgeommsg.PoseStamped,
        RobotPoseBridgeToROS,
        frame_id_parent="map",
    )
    merged = dict(topic_entry.topic_handler_params)
    merged.update(aliased._resolve_handler_params(POSE_TOPIC, topic_entry))

    assert merged["frame_id_parent"] == "map"
    assert merged["publish_tf"] is False
    # The entry's own callbacks survive the merge.
    assert "frame_id_parent" in topic_entry.topic_handler_params


def test_world_frame_becomes_the_transform_parent():
    instance = bridge(
        InterfaceProfile({"world_frame": "odom", "frames": {}, "topics": {}})
    )
    params = instance._resolve_handler_params(
        CAMERA_TOPIC, entry("/scene_camera", rossensmsg.Image, CameraBridgeToROS)
    )

    assert params["frame_id_parent"] == "odom"


def test_an_unrecognised_handler_type_gets_no_overrides(default):
    class ForeignHandler:
        pass

    params = default._resolve_handler_params(
        POSE_TOPIC,
        entry("/actual_pose", rosgeommsg.PoseStamped, ForeignHandler),
    )

    assert params == {}


def test_robot_frame_alias_reaches_the_sensor_message_headers():
    """
    Basic sensors take their frame ID from the robot_base_frame_ids map rather
    than a constructor argument, so the alias has to apply there too.
    """
    profile = InterfaceProfile(UNSEEN_PROFILE)

    assert profile.resolve_frame(SCENE, "fallback") == "x500_realsense"


# ---------------------------------------------------------------------------
# Point clouds
# ---------------------------------------------------------------------------


def test_a_depth_camera_receives_its_point_cloud_topic(aliased):
    params = aliased._resolve_handler_params(
        DEPTH_TOPIC,
        entry("/depth_planar_camera", rossensmsg.Image, CameraBridgeToROS),
    )

    assert params["ros_topic_name_points"] == "/rgbd_camera/points"
    assert params["points_settings"] is aliased.interface_profile.points


def test_no_point_cloud_topic_is_passed_without_one_in_the_profile(default):
    params = default._resolve_handler_params(
        DEPTH_TOPIC,
        entry("/depth_planar_camera", rossensmsg.Image, CameraBridgeToROS),
    )

    assert "ros_topic_name_points" not in params
    # The settings still travel, so enabling points needs only a topic name.
    assert params["points_settings"] is default.interface_profile.points


def test_a_point_cloud_requested_from_a_colour_camera_is_refused():
    """
    Only a depth image type carries depth. Reprojecting a colour image would
    read its BGR bytes as depth and emit a plausible-looking but meaningless
    cloud, so this is caught and reported rather than obeyed.
    """
    instance = bridge(
        InterfaceProfile(
            {
                "topics": {
                    "*/scene_camera": {
                        "image": "/rgbd_camera/image",
                        "points": "/rgbd_camera/points",
                    }
                }
            }
        )
    )
    params = instance._resolve_handler_params(
        CAMERA_TOPIC, entry("/scene_camera", rossensmsg.Image, CameraBridgeToROS)
    )

    assert "ros_topic_name_points" not in params
    assert len(instance.logger.warnings) == 1
    warning = instance.logger.warnings[0]
    assert "/rgbd_camera/points" in warning
    assert "depth image type" in warning


def test_a_perspective_depth_camera_may_also_produce_points():
    instance = bridge(
        InterfaceProfile(
            {
                "topics": {
                    "*/depth_camera": {
                        "image": "/depth/image",
                        "points": "/depth/points",
                    }
                }
            }
        )
    )
    topic = SCENE + "/sensors/RGBD/depth_camera"
    params = instance._resolve_handler_params(
        topic, entry("/depth_camera", rossensmsg.Image, CameraBridgeToROS)
    )

    assert params["ros_topic_name_points"] == "/depth/points"
    assert instance.logger.warnings == []


# ---------------------------------------------------------------------------
# Ground truth transforms
# ---------------------------------------------------------------------------


def test_no_ground_truth_topic_reaches_the_pose_handler_by_default(default):
    params = default._resolve_handler_params(
        POSE_TOPIC,
        entry(
            "/actual_pose",
            rosgeommsg.PoseStamped,
            RobotPoseBridgeToROS,
            frame_id_parent="map",
        ),
    )

    assert "ground_truth_tf_topic" not in params


def test_the_ground_truth_topic_reaches_the_pose_handler():
    instance = bridge(
        InterfaceProfile(
            {
                "tf": {
                    "publish_robot_tf": False,
                    "ground_truth_topic": "/ground_truth/tf",
                }
            }
        )
    )
    params = instance._resolve_handler_params(
        POSE_TOPIC,
        entry(
            "/actual_pose",
            rosgeommsg.PoseStamped,
            RobotPoseBridgeToROS,
            frame_id_parent="map",
        ),
    )

    assert params["ground_truth_tf_topic"] == "/ground_truth/tf"
    assert params["publish_tf"] is False

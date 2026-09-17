"""Interface profile parsing and topic/frame alias resolution."""
import textwrap

import pytest

from projectairsim_rosbridge import utils
from projectairsim_rosbridge.interface_profile import (
    CollisionSettings,
    DepthSettings,
    InterfaceProfile,
    PointCloudSettings,
    ProfileError,
)

CAMERA_TOPIC = "/Sim/SceneBasicDrone/robots/Drone1/sensors/RGBD/scene_camera"
DEPTH_TOPIC = "/Sim/SceneBasicDrone/robots/Drone1/sensors/RGBD/depth_planar_camera"
POSE_TOPIC = "/Sim/SceneBasicDrone/robots/Drone1/actual_pose"
ROBOT_PATH = "/Sim/SceneBasicDrone/robots/Drone1"
SENSOR_PATH = "/Sim/SceneBasicDrone/robots/Drone1/sensors/RGBD"


def write_profile(tmp_path, text):
    path = tmp_path / "profile.yaml"
    path.write_text(textwrap.dedent(text))
    return str(path)


def test_default_profile_preserves_the_bridges_historical_behaviour():
    profile = InterfaceProfile()

    assert profile.frame_convention == utils.CONVENTION_NWU
    assert profile.world_frame == "map"
    assert profile.sim_time.enabled is False
    assert profile.tf.publish_robot_tf is True
    assert profile.tf.publish_sensor_tf is True
    assert profile.resolve_topic(POSE_TOPIC) is None
    assert profile.resolve_frame(ROBOT_PATH) is None


def test_default_depth_encoding_is_metric_float():
    profile = InterfaceProfile()

    assert profile.depth.encoding == DepthSettings.ENCODING_32FC1
    assert profile.depth.max_range_m == 0.0


def test_missing_profile_path_returns_defaults():
    assert InterfaceProfile.from_file("").frame_convention == utils.CONVENTION_NWU


def test_unreadable_profile_reports_the_path():
    with pytest.raises(ProfileError, match="cannot read interface profile"):
        InterfaceProfile.from_file("/nonexistent/profile.yaml")


def test_malformed_yaml_is_reported(tmp_path):
    path = write_profile(tmp_path, "frames: [unclosed\n")
    with pytest.raises(ProfileError, match="cannot parse interface profile"):
        InterfaceProfile.from_file(path)


def test_unknown_sections_are_rejected(tmp_path):
    path = write_profile(tmp_path, "frame_conventions: enu\n")
    with pytest.raises(ProfileError, match="unknown profile section"):
        InterfaceProfile.from_file(path)


def test_unknown_frame_convention_is_rejected(tmp_path):
    path = write_profile(tmp_path, "frame_convention: ned\n")
    with pytest.raises(ProfileError, match="unknown frame_convention"):
        InterfaceProfile.from_file(path)


def test_unknown_depth_encoding_is_rejected(tmp_path):
    path = write_profile(tmp_path, "depth:\n  encoding: 8UC3\n")
    with pytest.raises(ProfileError, match="unknown depth encoding"):
        InterfaceProfile.from_file(path)


def test_depth_encoding_is_case_insensitive(tmp_path):
    path = write_profile(tmp_path, "depth:\n  encoding: 32fc1\n")

    assert (
        InterfaceProfile.from_file(path).depth.encoding
        == DepthSettings.ENCODING_32FC1
    )


def test_negative_depth_range_is_rejected(tmp_path):
    path = write_profile(tmp_path, "depth:\n  max_range_m: -1.0\n")
    with pytest.raises(ProfileError, match="max_range_m"):
        InterfaceProfile.from_file(path)


def test_unknown_section_key_is_reported(tmp_path):
    path = write_profile(tmp_path, "sim_time:\n  enable: true\n")
    with pytest.raises(ProfileError, match="sim_time|enable"):
        InterfaceProfile.from_file(path)


def test_frames_must_be_non_empty_strings(tmp_path):
    path = write_profile(tmp_path, 'frames:\n  "*/robots/Drone1":\n')
    with pytest.raises(ProfileError, match="non-empty string"):
        InterfaceProfile.from_file(path)


def test_full_profile_is_loaded(tmp_path):
    path = write_profile(
        tmp_path,
        """
        frame_convention: enu
        world_frame: odom
        sim_time:
          enabled: true
          clock_topic: /sim_clock
          min_step_ms: 2.0
        depth:
          encoding: 16UC1
          max_range_m: 10.0
        tf:
          publish_robot_tf: false
          publish_sensor_tf: false
        frames:
          "*/robots/Drone1": x500
          "*/robots/Drone1/sensors/RGBD": x500/camera_optical
        topics:
          "*/robots/Drone1/actual_pose": /ground_truth/pose
        """,
    )
    profile = InterfaceProfile.from_file(path)

    assert profile.frame_convention == utils.CONVENTION_ENU
    assert profile.world_frame == "odom"
    assert profile.sim_time.enabled is True
    assert profile.sim_time.clock_topic == "/sim_clock"
    assert profile.sim_time.min_step_nanos == 2000000
    assert profile.depth.encoding == DepthSettings.ENCODING_16UC1
    assert profile.depth.max_range_m == 10.0
    assert profile.tf.publish_robot_tf is False
    assert profile.tf.publish_sensor_tf is False
    assert profile.resolve_frame(ROBOT_PATH) == "x500"
    assert profile.resolve_frame(SENSOR_PATH) == "x500/camera_optical"
    assert profile.resolve_topic(POSE_TOPIC) == "/ground_truth/pose"
    assert profile.create_coordinate_converter().is_enu


def test_profile_converter_matches_the_declared_convention():
    assert not InterfaceProfile().create_coordinate_converter().is_enu


def test_unmatched_paths_fall_back_to_the_supplied_default():
    profile = InterfaceProfile({"frames": {"*/robots/Other": "other"}})

    assert profile.resolve_frame(ROBOT_PATH, "fallback") == "fallback"
    assert profile.resolve_frame(None, "fallback") == "fallback"
    assert profile.resolve_topic(POSE_TOPIC, "/default") == "/default"


def test_the_first_matching_pattern_wins():
    profile = InterfaceProfile(
        {
            "frames": {
                "*/robots/Drone1": "specific",
                "*/robots/*": "general",
            }
        }
    )

    assert profile.resolve_frame(ROBOT_PATH) == "specific"
    assert profile.resolve_frame("/Sim/S/robots/Drone9") == "general"


def test_camera_topics_default_to_the_bridges_naming():
    camera = InterfaceProfile().resolve_camera_topics(CAMERA_TOPIC)

    assert camera.image == CAMERA_TOPIC + "/image"
    assert camera.camera_info == CAMERA_TOPIC + "/camera_info"


def test_a_camera_alias_string_is_used_as_a_base_name():
    profile = InterfaceProfile({"topics": {"*/scene_camera": "/rgbd_camera"}})
    camera = profile.resolve_camera_topics(CAMERA_TOPIC)

    assert camera.image == "/rgbd_camera/image"
    assert camera.camera_info == "/rgbd_camera/camera_info"


def test_a_camera_alias_mapping_names_both_topics_explicitly():
    profile = InterfaceProfile(
        {
            "topics": {
                "*/depth_planar_camera": {
                    "image": "/rgbd_camera/depth_image",
                    "camera_info": "/rgbd_camera/depth_camera_info",
                }
            }
        }
    )
    camera = profile.resolve_camera_topics(DEPTH_TOPIC)

    assert camera.image == "/rgbd_camera/depth_image"
    assert camera.camera_info == "/rgbd_camera/depth_camera_info"


def test_a_camera_alias_mapping_may_omit_camera_info():
    profile = InterfaceProfile(
        {"topics": {"*/scene_camera": {"image": "/camera/rgb"}}}
    )
    camera = profile.resolve_camera_topics(CAMERA_TOPIC)

    assert camera.image == "/camera/rgb"
    assert camera.camera_info == "/camera/rgb/camera_info"


def test_a_camera_alias_mapping_requires_an_image_topic():
    with pytest.raises(ProfileError, match="image must be a non-empty string"):
        InterfaceProfile({"topics": {"*/scene_camera": {"camera_info": "/info"}}})


def test_unknown_camera_alias_keys_are_rejected():
    with pytest.raises(ProfileError, match="unknown key"):
        InterfaceProfile(
            {"topics": {"*/scene_camera": {"image": "/i", "depth": "/d"}}}
        )


def test_a_camera_alias_on_a_non_camera_topic_is_an_error():
    profile = InterfaceProfile(
        {"topics": {"*/actual_pose": {"image": "/oops"}}}
    )
    with pytest.raises(ProfileError, match="not a camera"):
        profile.resolve_topic(POSE_TOPIC)


def test_a_topic_alias_must_be_a_string_or_mapping():
    with pytest.raises(ProfileError, match="must be a string or a mapping"):
        InterfaceProfile({"topics": {"*/actual_pose": 42}})


def test_sections_must_be_mappings():
    with pytest.raises(ProfileError, match="sim_time must be a mapping"):
        InterfaceProfile({"sim_time": True})
    with pytest.raises(ProfileError, match="topics must be a mapping"):
        InterfaceProfile({"topics": ["/a"]})
    with pytest.raises(ProfileError, match="frames must be a mapping"):
        InterfaceProfile({"frames": ["/a"]})


def test_profile_must_be_a_mapping():
    with pytest.raises(ProfileError, match="must be a mapping"):
        InterfaceProfile([1, 2, 3])


def test_empty_world_frame_is_rejected():
    with pytest.raises(ProfileError, match="world_frame cannot be empty"):
        InterfaceProfile({"world_frame": ""})


def test_the_shipped_example_profile_is_valid():
    """The documented example must stay loadable as the schema evolves."""
    import os

    example = os.path.join(
        os.path.dirname(__file__),
        "..",
        "..",
        "projectairsim-ros2",
        "config",
        "interface_profile_example.yaml",
    )
    if not os.path.exists(example):
        pytest.skip("example profile not present in this layout")

    profile = InterfaceProfile.from_file(example)

    assert profile.frame_convention == utils.CONVENTION_ENU
    assert profile.sim_time.enabled is True
    assert profile.depth.encoding == DepthSettings.ENCODING_32FC1
    assert profile.depth.max_range_m == 10.0
    assert profile.tf.publish_robot_tf is False
    assert profile.tf.publish_sensor_tf is False
    assert profile.tf.ground_truth_topic == "/ground_truth/tf"
    assert profile.points.is_optical
    assert (
        profile.resolve_camera_topics(DEPTH_TOPIC).points == "/rgbd_camera/points"
    )
    # The shipped example must run without optional packages installed, so it
    # may not select a collision message that needs ros_gz_interfaces.
    assert profile.collision.message == CollisionSettings.MESSAGE_NONE


# ---------------------------------------------------------------------------
# Point clouds
# ---------------------------------------------------------------------------


def test_point_cloud_defaults_to_the_optical_convention_without_decimation():
    profile = InterfaceProfile()

    assert profile.points.frame_convention == PointCloudSettings.CONVENTION_OPTICAL
    assert profile.points.is_optical
    assert profile.points.decimation == 1


def test_point_cloud_settings_are_loaded(tmp_path):
    path = write_profile(
        tmp_path,
        """
        points:
          frame_convention: ros
          decimation: 4
        """,
    )
    profile = InterfaceProfile.from_file(path)

    assert profile.points.frame_convention == PointCloudSettings.CONVENTION_ROS
    assert not profile.points.is_optical
    assert profile.points.decimation == 4


def test_unknown_point_cloud_convention_is_rejected():
    with pytest.raises(ProfileError, match="points.frame_convention"):
        InterfaceProfile({"points": {"frame_convention": "ned"}})


@pytest.mark.parametrize("decimation", [0, -1])
def test_invalid_decimation_is_rejected(decimation):
    with pytest.raises(ProfileError, match="decimation"):
        InterfaceProfile({"points": {"decimation": decimation}})


def test_no_point_cloud_topic_is_resolved_by_default():
    """A point cloud is opt-in: only a depth camera can produce one."""
    assert InterfaceProfile().resolve_camera_topics(DEPTH_TOPIC).points is None


def test_a_camera_alias_may_name_a_point_cloud_topic():
    profile = InterfaceProfile(
        {
            "topics": {
                "*/depth_planar_camera": {
                    "image": "/rgbd_camera/depth_image",
                    "points": "/rgbd_camera/points",
                }
            }
        }
    )
    camera = profile.resolve_camera_topics(DEPTH_TOPIC)

    assert camera.image == "/rgbd_camera/depth_image"
    assert camera.camera_info == "/rgbd_camera/depth_image/camera_info"
    assert camera.points == "/rgbd_camera/points"


def test_an_empty_point_cloud_topic_is_rejected():
    with pytest.raises(ProfileError, match="points must be a non-empty string"):
        InterfaceProfile(
            {"topics": {"*/depth_planar_camera": {"image": "/i", "points": ""}}}
        )


# ---------------------------------------------------------------------------
# Collision reporting
# ---------------------------------------------------------------------------


def test_collision_reporting_is_off_by_default():
    """
    ros_gz_interfaces is an optional dependency, so nothing may require it
    unless the profile asks for it.
    """
    profile = InterfaceProfile()

    assert profile.collision.message == CollisionSettings.MESSAGE_NONE
    assert not profile.collision.enabled


def test_collision_reporting_can_be_requested(tmp_path):
    path = write_profile(tmp_path, "collision:\n  message: gz_contacts\n")
    profile = InterfaceProfile.from_file(path)

    assert profile.collision.message == CollisionSettings.MESSAGE_GZ_CONTACTS
    assert profile.collision.enabled


def test_unknown_collision_message_is_rejected():
    with pytest.raises(ProfileError, match="collision.message"):
        InterfaceProfile({"collision": {"message": "contacts"}})


# ---------------------------------------------------------------------------
# Ground truth transforms
# ---------------------------------------------------------------------------


def test_no_ground_truth_transform_topic_by_default():
    assert InterfaceProfile().tf.ground_truth_topic == ""


def test_ground_truth_transform_topic_is_loaded(tmp_path):
    path = write_profile(
        tmp_path,
        """
        tf:
          publish_robot_tf: false
          ground_truth_topic: /ground_truth/tf
        """,
    )
    profile = InterfaceProfile.from_file(path)

    assert profile.tf.ground_truth_topic == "/ground_truth/tf"
    assert profile.tf.publish_robot_tf is False

from types import SimpleNamespace

from geometry_msgs.msg import PoseStamped

import pytest

import projectairsim_rosbridge.ros_bridge as bridge_module
from projectairsim_rosbridge.interface_profile import InterfaceProfile, ProfileError
from projectairsim_rosbridge.ros_bridge import ProjectAirSimROSBridge


class FakeTopicHandler:
    def __init__(self, **kwargs):
        self.cleared = False

    def clear(self):
        self.cleared = True


class FakeRobotControlHandler:
    instances = []

    def __init__(self, **kwargs):
        self.robot_path = kwargs["robot_path"]
        self.desired_pose_topic_name = kwargs["desired_pose_topic_name"]
        self.cleared = False
        self.instances.append(self)

    def clear(self):
        self.cleared = True


def make_bridge(topics):
    bridge = ProjectAirSimROSBridge.__new__(ProjectAirSimROSBridge)
    bridge.topic_entries = [
        ProjectAirSimROSBridge.TopicEntry(
            ProjectAirSimROSBridge.TopicEntry.MatchType.ENDS_WITH,
            "/actual_pose",
            PoseStamped,
            FakeTopicHandler,
        )
    ]
    bridge.projectairsim_client = SimpleNamespace(topics={topic: None for topic in topics})
    bridge.topic_handlers = {}
    bridge.robot_control_handlers = {}
    bridge.robot_paths = {}
    bridge.topics_managers = object()
    bridge.interface_profile = InterfaceProfile()
    bridge.ros_node = SimpleNamespace(supports_lifecycle_services=True)
    bridge.msg_converter = SimpleNamespace(set_robot_base_frame_ids=lambda value: None)
    bridge.cmd_vel_timeout_sec = 1.0
    bridge.takeoff_timeout_sec = 20.0
    bridge.land_timeout_sec = 60.0
    return bridge


def test_scene_discovery_creates_one_control_handler_per_robot(monkeypatch):
    monkeypatch.setattr(bridge_module, "RobotControlHandler", FakeRobotControlHandler)
    FakeRobotControlHandler.instances = []
    bridge = make_bridge(
        [
            "/scene/robots/Drone1/actual_pose",
            "/scene/robots/Drone1/desired_pose",
            "/scene/robots/Drone2/actual_pose",
        ]
    )

    bridge.update_topics()

    assert set(bridge.robot_control_handlers) == {
        "/scene/robots/Drone1",
        "/scene/robots/Drone2",
    }
    assert bridge.robot_control_handlers[
        "/scene/robots/Drone1"
    ].desired_pose_topic_name.endswith("/desired_pose")
    assert (
        bridge.robot_control_handlers[
            "/scene/robots/Drone2"
        ].desired_pose_topic_name
        is None
    )
    bridge.clear = lambda: None


def test_scene_replacement_destroys_removed_robot_controls(monkeypatch):
    monkeypatch.setattr(bridge_module, "RobotControlHandler", FakeRobotControlHandler)
    bridge = make_bridge(["/scene/robots/Drone1/actual_pose"])
    bridge.update_topics()
    removed_handler = bridge.robot_control_handlers["/scene/robots/Drone1"]

    bridge.projectairsim_client.topics = {
        "/scene/robots/Drone2/actual_pose": None
    }
    bridge.update_topics()

    assert removed_handler.cleared
    assert set(bridge.robot_control_handlers) == {"/scene/robots/Drone2"}
    bridge.clear = lambda: None


def test_a_half_built_bridge_can_be_torn_down_without_raising():
    """
    A failure inside __init__ still leaves Python to run the destructor.  If
    teardown raised, the AttributeError would replace the real error in the
    traceback.
    """
    bridge = ProjectAirSimROSBridge.__new__(ProjectAirSimROSBridge)

    bridge.clear()
    bridge.clear()


def test_an_invalid_interface_profile_surfaces_as_a_profile_error():
    with pytest.raises(ProfileError, match="cannot read interface profile"):
        ProjectAirSimROSBridge(
            ros_node=None, interface_profile="/nonexistent/profile.yaml"
        )

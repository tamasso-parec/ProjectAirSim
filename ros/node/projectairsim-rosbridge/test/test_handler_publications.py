"""Topic-handler wiring for Phase 2 publications.

These exercise the handlers rather than the conversions: which ROS topics get
advertised, when the upstream Project AirSim subscription is held, and that
ground truth stays off the standard /tf topic.
"""
import struct

import pytest

from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, PointCloud2
from tf2_msgs.msg import TFMessage

from projectairsim_rosbridge import utils
from projectairsim_rosbridge.interface_profile import PointCloudSettings
from projectairsim_rosbridge.topic_helpers import (
    AutoSubscriber,
    CameraBridgeToROS,
    RobotPoseBridgeToROS,
)

POSE_TOPIC = "/Sim/Scene/robots/Drone1/actual_pose"
DEPTH_TOPIC = "/Sim/Scene/robots/Drone1/sensors/RGBD/depth_planar_camera"


class FakeTopic:
    def __init__(self, path):
        self.path = path


class FakeProjectAirSimTopicsManager:
    def __init__(self):
        self.subscriptions = {}

    def add_subscriber(self, topic_name, callback):
        self.subscriptions[topic_name] = callback

    def remove_subscriber(self, topic_name, callback):
        self.subscriptions.pop(topic_name, None)


class FakeROSTopicsManager:
    def __init__(self):
        self.publishers = {}
        self.subscribers = {}
        self.published = []

    def add_publisher(
        self,
        topic_name,
        ros_message_type,
        peer_change_callback=None,
        is_latching=True,
        ros_queue_size=1,
        qos_profile="default",
    ):
        self.publishers[topic_name] = {
            "type": ros_message_type,
            "callback": peer_change_callback,
            "qos_profile": qos_profile,
        }

    def remove_publisher(self, topic_name, peer_change_callback):
        self.publishers.pop(topic_name, None)

    def add_subscriber(self, topic_name, ros_message_type, callback, **kwargs):
        self.subscribers[topic_name] = callback

    def remove_subscriber(self, topic_name, callback):
        self.subscribers.pop(topic_name, None)

    def publish(self, topic_name, message):
        self.published.append((topic_name, message))

    def messages_on(self, topic_name):
        return [message for topic, message in self.published if topic == topic_name]


class FakeTFBroadcaster:
    def __init__(self):
        self.frames = {}
        self.updates = []

    def add_frame(self, frame_id, frame_id_parent, transform=None):
        self.frames[frame_id] = frame_id_parent

    def set_frame(self, frame_id, transform, timevalue=None):
        self.updates.append(frame_id)

    def remove_frame(self, frame_id):
        self.frames.pop(frame_id, None)


class FakeSensorHelper:
    def set_camera_info(self, *args, **kwargs):
        pass


class FakeROSNode:
    @staticmethod
    def create_sensor_helper():
        return FakeSensorHelper()

    @staticmethod
    def get_time_from_msg(stamp):
        return stamp

    @staticmethod
    def get_time_now_msg():
        return Time(sec=12, nanosec=34)


class FakeTopicsManagers:
    def __init__(self):
        self.ros_node = FakeROSNode()
        self.ros_topics_manager = FakeROSTopicsManager()
        self.projectairsim_topics_manager = FakeProjectAirSimTopicsManager()
        self.tf_broadcaster = FakeTFBroadcaster()
        self.sim_time = None
        self.coords = utils.CoordinateConverter()


def pose_message(*args):
    posestamped = PoseStamped()
    posestamped.header.stamp = Time(sec=5, nanosec=0)
    posestamped.pose.position.x = 1.0
    posestamped.pose.position.y = 2.0
    posestamped.pose.position.z = 3.0
    posestamped.pose.orientation.w = 1.0
    return posestamped


def make_pose_handler(**kwargs):
    managers = FakeTopicsManagers()
    handler = RobotPoseBridgeToROS(
        projectairsim_topic_name=POSE_TOPIC,
        ros_message_type=PoseStamped,
        topics_managers=managers,
        message_callback=pose_message,
        frame_id_parent="map",
        frame_id="x500_realsense",
        **kwargs,
    )
    return managers, handler


def feed_pose(handler):
    handler._projectairsim_topic_update_cb(FakeTopic(POSE_TOPIC), {})


# ---------------------------------------------------------------------------
# Ground truth transforms
# ---------------------------------------------------------------------------


def test_no_ground_truth_topic_is_advertised_by_default():
    managers, handler = make_pose_handler()
    feed_pose(handler)

    assert "/ground_truth/tf" not in managers.ros_topics_manager.publishers
    assert not managers.ros_topics_manager.messages_on("/ground_truth/tf")


def test_ground_truth_transforms_are_published_on_their_own_topic():
    managers, handler = make_pose_handler(ground_truth_tf_topic="/ground_truth/tf")
    feed_pose(handler)

    advertised = managers.ros_topics_manager.publishers["/ground_truth/tf"]
    assert advertised["type"] is TFMessage

    messages = managers.ros_topics_manager.messages_on("/ground_truth/tf")
    assert len(messages) == 1
    transform = messages[0].transforms[0]
    assert transform.header.frame_id == "map"
    assert transform.child_frame_id == "x500_realsense"
    assert transform.header.stamp == Time(sec=5, nanosec=0)
    assert transform.transform.translation.x == 1.0
    assert transform.transform.translation.z == 3.0
    assert transform.transform.rotation.w == 1.0


def test_ground_truth_transforms_work_with_standard_tf_disabled():
    """
    This is the configuration that matters: a stack whose own estimator owns
    /tf still wants simulator ground truth for evaluation.
    """
    managers, handler = make_pose_handler(
        ground_truth_tf_topic="/ground_truth/tf", publish_tf=False
    )
    feed_pose(handler)

    assert managers.ros_topics_manager.messages_on("/ground_truth/tf")
    # Nothing claimed the vehicle frame on the standard transform tree.
    assert managers.tf_broadcaster.frames == {}
    assert managers.tf_broadcaster.updates == []


def test_standard_tf_is_still_broadcast_when_enabled():
    managers, handler = make_pose_handler(ground_truth_tf_topic="/ground_truth/tf")
    feed_pose(handler)

    assert managers.tf_broadcaster.frames == {"x500_realsense": "map"}
    assert managers.tf_broadcaster.updates == ["x500_realsense"]


def test_clearing_removes_the_ground_truth_publisher():
    managers, handler = make_pose_handler(ground_truth_tf_topic="/ground_truth/tf")
    handler.clear()

    assert "/ground_truth/tf" not in managers.ros_topics_manager.publishers


# ---------------------------------------------------------------------------
# Point cloud publication
# ---------------------------------------------------------------------------


@pytest.fixture(autouse=True)
def reset_camera_singleton():
    """CameraBridgeToROS shares one desired-pose helper across instances."""
    CameraBridgeToROS._desired_pose_bridge_from_ros = (
        CameraBridgeToROS.DesiredPoseBridgeFromROS()
    )


def depth_message():
    return {
        "encoding": "16UC1",
        "height": 1,
        "width": 2,
        "big_endian": False,
        "data": struct.pack("<2H", 1000, 2000),
        "pos_x": 0.0,
        "pos_y": 0.0,
        "pos_z": 0.0,
        "rot_w": 1.0,
        "rot_x": 0.0,
        "rot_y": 0.0,
        "rot_z": 0.0,
    }


def make_camera_handler(points_topic=None, points_callback=None):
    managers = FakeTopicsManagers()
    calls = []

    def record_points(message, intrinsics, frame_id, settings):
        calls.append((message, tuple(intrinsics), frame_id, settings))
        return PointCloud2()

    handler = CameraBridgeToROS(
        projectairsim_topic_name=DEPTH_TOPIC,
        ros_message_type=Image,
        topics_managers=managers,
        image_message_callback=lambda *args: Image(),
        desired_pose_message_callback=lambda *args: None,
        frame_id="camera_optical",
        ros_topic_name_image="/rgbd_camera/depth_image",
        ros_topic_name_camera_info="/rgbd_camera/depth_camera_info",
        ros_topic_name_points=points_topic,
        points_settings=PointCloudSettings(),
        points_message_callback=(
            points_callback if points_callback is not None else record_points
        ),
    )
    return managers, handler, calls


def test_no_point_cloud_topic_is_advertised_by_default():
    managers, handler, calls = make_camera_handler()
    handler._projectairsim_topic_update_cb(FakeTopic(DEPTH_TOPIC), depth_message())

    assert "/rgbd_camera/points" not in managers.ros_topics_manager.publishers
    assert calls == []


def test_a_requested_point_cloud_topic_is_advertised_as_sensor_data():
    managers, handler, calls = make_camera_handler("/rgbd_camera/points")
    advertised = managers.ros_topics_manager.publishers["/rgbd_camera/points"]

    assert advertised["type"] is PointCloud2
    assert advertised["qos_profile"] == "sensor_data"


def test_camera_info_is_subscribed_eagerly_when_points_are_requested():
    """Reprojection needs the intrinsics, which arrive on their own topic."""
    managers, handler, calls = make_camera_handler("/rgbd_camera/points")

    assert handler.auto_subscriber_camera_info.is_subscribed


def test_reprojection_is_skipped_while_nobody_subscribes():
    """Reprojecting every frame for no one would be pure waste."""
    managers, handler, calls = make_camera_handler("/rgbd_camera/points")
    handler._projectairsim_topic_update_cb(FakeTopic(DEPTH_TOPIC), depth_message())

    assert calls == []
    assert not managers.ros_topics_manager.messages_on("/rgbd_camera/points")


def test_points_are_published_once_someone_subscribes():
    managers, handler, calls = make_camera_handler("/rgbd_camera/points")
    handler._auto_subscriber.peer_change_cb("/rgbd_camera/points", 1)

    handler._projectairsim_topic_update_cb(FakeTopic(DEPTH_TOPIC), depth_message())

    assert len(calls) == 1
    _, intrinsics, frame_id, settings = calls[0]
    assert frame_id == "camera_optical"
    assert intrinsics == tuple(handler.camera_info.k)
    assert isinstance(settings, PointCloudSettings)
    assert len(managers.ros_topics_manager.messages_on("/rgbd_camera/points")) == 1


def test_nothing_is_published_before_the_intrinsics_arrive():
    """The converter returns None until camera info is known."""
    managers, handler, calls = make_camera_handler(
        "/rgbd_camera/points", points_callback=lambda *args: None
    )
    handler._auto_subscriber.peer_change_cb("/rgbd_camera/points", 1)

    handler._projectairsim_topic_update_cb(FakeTopic(DEPTH_TOPIC), depth_message())

    assert not managers.ros_topics_manager.messages_on("/rgbd_camera/points")


def test_a_points_topic_requires_a_conversion_callback():
    managers = FakeTopicsManagers()
    with pytest.raises(TypeError, match="points_message_callback"):
        CameraBridgeToROS(
            projectairsim_topic_name=DEPTH_TOPIC,
            ros_message_type=Image,
            topics_managers=managers,
            image_message_callback=lambda *args: Image(),
            desired_pose_message_callback=lambda *args: None,
            ros_topic_name_points="/rgbd_camera/points",
        )


def test_clearing_removes_the_point_cloud_publisher():
    managers, handler, calls = make_camera_handler("/rgbd_camera/points")
    handler.clear()

    assert "/rgbd_camera/points" not in managers.ros_topics_manager.publishers


# ---------------------------------------------------------------------------
# Shared upstream subscription
# ---------------------------------------------------------------------------


class RecordingAutoSubscriber(AutoSubscriber):
    def __init__(self):
        self.subscribe_calls = 0
        self.unsubscribe_calls = 0
        super().__init__("/some/topic", lambda *args: None, None)

    def subscribe(self):
        self.subscribe_calls += 1
        self.is_subscribed = True

    def unsubscribe(self):
        self.unsubscribe_calls += 1
        self.is_subscribed = False


def test_one_subscriber_behaves_as_before():
    subscriber = RecordingAutoSubscriber()

    subscriber.peer_change_cb("/image", 1)
    assert subscriber.is_subscribed
    subscriber.peer_change_cb("/image", 0)
    assert not subscriber.is_subscribed
    assert (subscriber.subscribe_calls, subscriber.unsubscribe_calls) == (1, 1)


def test_the_upstream_subscription_is_held_while_any_ros_topic_has_peers():
    """
    A depth message feeds both an image topic and a point cloud topic.  Losing
    the last image subscriber must not cut off the point cloud.
    """
    subscriber = RecordingAutoSubscriber()
    subscriber.peer_change_cb("/rgbd_camera/depth_image", 1)
    subscriber.peer_change_cb("/rgbd_camera/points", 1)

    subscriber.peer_change_cb("/rgbd_camera/depth_image", 0)

    assert subscriber.is_subscribed
    assert subscriber.unsubscribe_calls == 0

    subscriber.peer_change_cb("/rgbd_camera/points", 0)

    assert not subscriber.is_subscribed
    assert subscriber.unsubscribe_calls == 1


def test_repeated_peer_changes_do_not_resubscribe():
    subscriber = RecordingAutoSubscriber()
    subscriber.peer_change_cb("/image", 1)
    subscriber.peer_change_cb("/image", 2)
    subscriber.peer_change_cb("/points", 1)

    assert subscriber.subscribe_calls == 1

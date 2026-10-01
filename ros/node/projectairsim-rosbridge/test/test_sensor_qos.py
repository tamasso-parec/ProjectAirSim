"""The reliability sensor topics are published with."""

# A best-effort publisher cannot satisfy a reliable subscriber: DDS refuses
# the match and not one message is delivered. The bridge says so --
#
#   New subscription discovered on topic '/rgbd_camera/points', requesting
#   incompatible QoS. No messages will be sent to it.
#
# -- but the topic still lists, still counts a publisher, and `ros2 topic hz`
# run from a shell that happens to subscribe best effort still shows a rate,
# so it reads as a rendering problem rather than a delivery one. ros_gz_bridge
# publishes reliably, so a stack built against Gazebo subscribes reliably and
# gets silence from the bridge's default. Pinned here because the symptom
# points nowhere near the cause.
import pytest

from projectairsim_rosbridge.interface_profile import InterfaceProfile, ProfileError


def test_the_default_is_best_effort_as_a_sensor_driver_would_be():
    profile = InterfaceProfile()

    assert profile.qos.sensor_reliability == "best_effort"
    assert profile.qos.sensor_profile_name == "sensor_data"


def test_reliable_maps_onto_the_nodes_default_profile():
    # "default" is what the ROS 2 node layer turns into RELIABLE + VOLATILE,
    # which is what ros_gz_bridge publishes and what the Gazebo-era RViz
    # configuration and octomap subscription ask for.
    profile = InterfaceProfile({"qos": {"sensor_reliability": "reliable"}})

    assert profile.qos.sensor_profile_name == "default"


@pytest.mark.parametrize("value", ["RELIABLE", "Reliable"])
def test_the_value_is_case_insensitive(value):
    assert InterfaceProfile({"qos": {"sensor_reliability": value}}).qos.sensor_profile_name == "default"


def test_an_unknown_reliability_is_refused():
    # Left to the simulator this would silently fall back to a default and the
    # subscriber would go quiet again, which is the failure being avoided.
    with pytest.raises(ProfileError, match="sensor_reliability"):
        InterfaceProfile({"qos": {"sensor_reliability": "at_most_once"}})


def test_an_unknown_key_in_the_section_is_refused():
    with pytest.raises(ProfileError):
        InterfaceProfile({"qos": {"reliability": "reliable"}})

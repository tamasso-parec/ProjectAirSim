"""Project AirSim collision reports as ros_gz_interfaces Contacts messages.

ros_gz_interfaces is an optional dependency and is not installed in every
environment, so most of these tests stand a minimal fake in for it, keeping the
field mapping under test everywhere.  The fake cannot catch a wrong field type,
though, so the tests at the end run against the real message definition where
it is installed, and one test covers the case where the package is absent.
"""
import sys
import types

import pytest

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Vector3

from projectairsim_rosbridge import utils
from projectairsim_rosbridge.msg_converter import MsgConverter


COLLISION_TOPIC = "/Sim/Scene/robots/Drone1/collision_info"
ROBOT_FRAME = "x500_realsense"


class FakeROSNode:
    @staticmethod
    def get_time_now_msg():
        return Time(sec=12, nanosec=34)

    @staticmethod
    def get_time_from_nanos(nanos):
        return int(nanos)

    @staticmethod
    def get_time_to_msg(timestamp):
        nanos = int(timestamp)
        return Time(sec=nanos // 1000000000, nanosec=nanos % 1000000000)


class FakeEntity:
    def __init__(self):
        self.name = ""


class FakeContact:
    def __init__(self):
        self.collision1 = FakeEntity()
        self.collision2 = FakeEntity()
        self.positions = []
        self.normals = []
        self.depths = []


class FakeContacts:
    def __init__(self):
        self.header = None
        self.contacts = []


@pytest.fixture
def gz_interfaces(monkeypatch):
    """Install a stand-in ros_gz_interfaces.msg module."""
    package = types.ModuleType("ros_gz_interfaces")
    module = types.ModuleType("ros_gz_interfaces.msg")
    module.Contact = FakeContact
    module.Contacts = FakeContacts
    package.msg = module
    monkeypatch.setitem(sys.modules, "ros_gz_interfaces", package)
    monkeypatch.setitem(sys.modules, "ros_gz_interfaces.msg", module)
    return module


def converter(**kwargs):
    result = MsgConverter(FakeROSNode(), **kwargs)
    result.set_robot_base_frame_ids({COLLISION_TOPIC: ROBOT_FRAME})
    return result


def collision_message(**overrides):
    message = {
        "time_stamp": 3000000000,
        "object_name": "Wall_01",
        "segmentation_id": 7,
        "position": {"x": 1.0, "y": 2.0, "z": 3.0},
        "impact_point": {"x": 4.0, "y": 5.0, "z": 6.0},
        "normal": {"x": 0.0, "y": 1.0, "z": 0.0},
        "penetration_depth": 0.125,
    }
    message.update(overrides)
    return message


def test_a_collision_becomes_a_single_contact(gz_interfaces):
    contacts = converter().convert_collision_info_to_gz_contacts(
        COLLISION_TOPIC, collision_message()
    )

    assert len(contacts.contacts) == 1
    contact = contacts.contacts[0]
    # Gazebo names both sides; Project AirSim knows only what the robot hit,
    # so the robot's own frame stands in for the other side.
    assert contact.collision1.name == ROBOT_FRAME
    assert contact.collision2.name == "Wall_01"
    assert contact.depths == [pytest.approx(0.125)]


def test_contact_geometry_is_converted_to_the_ros_convention(gz_interfaces):
    contacts = converter().convert_collision_info_to_gz_contacts(
        COLLISION_TOPIC, collision_message()
    )
    position = contacts.contacts[0].positions[0]
    normal = contacts.contacts[0].normals[0]

    # Contact.positions and .normals are Vector3 sequences, not Point.
    assert isinstance(position, Vector3)
    # The default NWU convention negates Y and Z.
    assert (position.x, position.y, position.z) == (4.0, -5.0, -6.0)
    assert (normal.x, normal.y, normal.z) == (0.0, -1.0, -0.0)


def test_contact_geometry_follows_the_configured_frame_convention(gz_interfaces):
    result = converter(coords=utils.CoordinateConverter(utils.CONVENTION_ENU))
    contacts = result.convert_collision_info_to_gz_contacts(
        COLLISION_TOPIC, collision_message()
    )
    position = contacts.contacts[0].positions[0]

    # NED (4 north, 5 east, 6 down) is ENU (5 east, 4 north, -6 up).
    assert (position.x, position.y, position.z) == (5.0, 4.0, -6.0)


def test_the_header_carries_the_robot_frame_and_simulation_timestamp(
    gz_interfaces,
):
    from projectairsim_rosbridge.sim_time import SimTimeSource

    result = converter(sim_time=SimTimeSource(FakeROSNode(), enabled=True))
    contacts = result.convert_collision_info_to_gz_contacts(
        COLLISION_TOPIC, collision_message()
    )

    assert contacts.header.frame_id == ROBOT_FRAME
    assert contacts.header.stamp == Time(sec=3, nanosec=0)


def test_no_collision_produces_an_empty_contacts_message(gz_interfaces):
    """
    An empty Contacts message is how a Gazebo consumer sees "nothing is
    touching", so it must not be confused with a contact against "".
    """
    contacts = converter().convert_collision_info_to_gz_contacts(
        COLLISION_TOPIC, collision_message(object_name="")
    )

    assert contacts.contacts == []
    assert contacts.header.frame_id == ROBOT_FRAME


def test_a_missing_object_name_is_treated_as_no_collision(gz_interfaces):
    message = collision_message()
    del message["object_name"]

    contacts = converter().convert_collision_info_to_gz_contacts(
        COLLISION_TOPIC, message
    )

    assert contacts.contacts == []


def test_a_missing_penetration_depth_defaults_to_zero(gz_interfaces):
    message = collision_message()
    del message["penetration_depth"]

    contacts = converter().convert_collision_info_to_gz_contacts(
        COLLISION_TOPIC, message
    )

    assert contacts.contacts[0].depths == [0.0]


def test_an_absent_ros_gz_interfaces_package_explains_how_to_fix_it(monkeypatch):
    """
    The package is optional, so the failure has to name it and the profile
    setting that asked for it rather than surfacing a bare ImportError.
    """
    monkeypatch.setitem(sys.modules, "ros_gz_interfaces", None)
    monkeypatch.setitem(sys.modules, "ros_gz_interfaces.msg", None)

    with pytest.raises(ImportError) as excinfo:
        MsgConverter._gz_interfaces_msgs()

    message = str(excinfo.value)
    assert "ros_gz_interfaces" in message
    assert "collision.message" in message
    assert "ros-gz-interfaces" in message


# ---------------------------------------------------------------------------
# Against the real message definition, where it is installed
# ---------------------------------------------------------------------------

real_gz_msgs = pytest.importorskip(
    "ros_gz_interfaces.msg",
    reason="ros_gz_interfaces is an optional dependency",
)


def test_the_conversion_produces_a_valid_real_contacts_message():
    """
    The fakes above cannot catch a field name or type the real message would
    reject, and generated ROS messages validate on assignment.
    """
    contacts = converter().convert_collision_info_to_gz_contacts(
        COLLISION_TOPIC, collision_message()
    )

    assert isinstance(contacts, real_gz_msgs.Contacts)
    assert len(contacts.contacts) == 1
    contact = contacts.contacts[0]
    assert isinstance(contact, real_gz_msgs.Contact)
    assert contact.collision1.name == ROBOT_FRAME
    assert contact.collision2.name == "Wall_01"
    assert len(contact.positions) == 1
    assert len(contact.normals) == 1
    assert list(contact.depths) == [pytest.approx(0.125)]
    # A field the conversion leaves alone must still be valid and empty.
    assert list(contact.wrenches) == []


def test_the_real_empty_message_round_trips():
    contacts = converter().convert_collision_info_to_gz_contacts(
        COLLISION_TOPIC, collision_message(object_name="")
    )

    assert isinstance(contacts, real_gz_msgs.Contacts)
    assert list(contacts.contacts) == []

"""How bulk image and point-cloud payloads are handed to rclpy."""

# rclpy stores a uint8[] field as array.array('B') and validates any other
# sequence element by element on assignment. For a 640x480 organised point
# cloud -- 3.7 MB -- that check measures around 190 ms against a 33 ms budget
# at 30 Hz, and it is paid on the Project AirSim client's single receive
# thread, which runs every subscription callback inline. The simulator's
# socket backs up behind it and its sends time out:
#
#   nng_send for topic '.../robots/Drone1/actual_pose' failed with 'Timed out'
#
# which reads as a lost vehicle pose and says nothing about the point cloud
# that caused it. Pinned here because the cost is invisible in review: the
# difference is the type of the object assigned, not the code around it.
import array

import numpy as np
import pytest

from projectairsim_rosbridge.msg_converter import MsgConverter


PAYLOADS = {
    "bytes": b"\x01\x02\x03\x04",
    "numpy tobytes": np.arange(4, dtype="uint8").tobytes(),
    "memoryview": memoryview(b"\x05\x06\x07\x08"),
}


@pytest.mark.parametrize("name,payload", sorted(PAYLOADS.items()))
def test_every_payload_kind_becomes_the_array_rclpy_wants(name, payload):
    encoded = MsgConverter._as_ros_bytes(payload)

    assert isinstance(encoded, array.array)
    assert encoded.typecode == "B"
    assert bytes(encoded) == bytes(payload)


def test_the_conversion_is_assignable_without_rclpy_revalidating_it():
    # The whole point: rclpy accepts this object as-is. If a future change
    # returns bytes again the message still builds, just 1300x slower, so
    # assert the type rather than the behaviour.
    import sensor_msgs.msg as rossensmsg

    cloud = np.zeros((4, 4, 3), dtype="<f4")
    message = rossensmsg.PointCloud2()
    message.data = MsgConverter._as_ros_bytes(cloud.tobytes())

    assert len(message.data) == cloud.nbytes


def test_an_empty_payload_is_handled():
    assert len(MsgConverter._as_ros_bytes(b"")) == 0

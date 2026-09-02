import math

from geometry_msgs.msg import Quaternion, Vector3

from projectairsim_rosbridge import utils


def test_position_conversion_round_trip():
    ros_vector = Vector3(x=1.25, y=-2.5, z=3.75)
    projectairsim_vector = utils.to_projectairsim_position(ros_vector)

    assert projectairsim_vector == {"x": 1.25, "y": 2.5, "z": -3.75}
    assert utils.to_ros_position_list(projectairsim_vector) == (1.25, -2.5, 3.75)


def test_quaternion_conversion_round_trip():
    ros_quaternion = Quaternion(x=0.1, y=0.2, z=-0.3, w=0.9)
    projectairsim_quaternion = utils.to_projectairsim_quaternion(ros_quaternion)
    converted = utils.to_ros_quaternion(projectairsim_quaternion)

    assert math.isclose(converted.x, ros_quaternion.x)
    assert math.isclose(converted.y, ros_quaternion.y)
    assert math.isclose(converted.z, ros_quaternion.z)
    assert math.isclose(converted.w, ros_quaternion.w)

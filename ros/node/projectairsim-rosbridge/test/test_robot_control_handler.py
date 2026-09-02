from types import SimpleNamespace

from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool, Trigger

from projectairsim_rosbridge.topic_helpers import RobotControlHandler


class FakeService:
    def __init__(self, name, callback):
        self.name = name
        self.callback = callback
        self.destroyed = False

    def destroy(self):
        self.destroyed = True


class FakeROSNode:
    def __init__(self):
        self.services = []

    def create_service(self, name, service_type, callback):
        service = FakeService(name, callback)
        self.services.append(service)
        return service


class FakeROSTopicsManager:
    def __init__(self):
        self.subscribers = {}

    def add_subscriber(self, name, message_type, callback, qos_profile="default"):
        self.subscribers[name] = callback

    def remove_subscriber(self, name, callback):
        self.subscribers.pop(name, None)


class FakeTopicsManagers:
    def __init__(self):
        self.ros_node = FakeROSNode()
        self.ros_topics_manager = FakeROSTopicsManager()
        self.logger = SimpleNamespace(
            info=lambda *args, **kwargs: None,
            exception=lambda *args, **kwargs: None,
        )
        self.requests = []

    def request(self, request):
        self.requests.append(request)
        return True

    def request_async_blocking(self, request, timeout_sec):
        self.requests.append(request)
        return None


def make_handler():
    managers = FakeTopicsManagers()
    handler = RobotControlHandler(
        robot_path="/airsim_node/robots/Drone1",
        topics_managers=managers,
        desired_pose_topic_name=None,
        cmd_vel_timeout_sec=1.5,
        takeoff_timeout_sec=20.0,
        land_timeout_sec=60.0,
        create_lifecycle_services=True,
    )
    return managers, handler


def test_cmd_vel_exists_without_desired_pose_and_converts_coordinates():
    managers, handler = make_handler()
    command = Twist()
    command.linear.x = 1.0
    command.linear.y = 2.0
    command.linear.z = 3.0
    command.angular.z = 0.4

    handler.handle_ros_cmd_vel("unused", command)

    request = managers.requests[-1]
    assert request["method"].endswith("/MoveByVelocity")
    assert request["params"]["vx"] == 1.0
    assert request["params"]["vy"] == -2.0
    assert request["params"]["vz"] == -3.0
    assert request["params"]["yaw"] == -0.4
    assert request["params"]["duration"] == 1.5


def test_lifecycle_services_route_to_the_same_robot_path():
    managers, handler = make_handler()

    arm_response = handler.handle_arm(
        SetBool.Request(data=True), SetBool.Response()
    )
    takeoff_response = handler.handle_takeoff(
        Trigger.Request(), Trigger.Response()
    )

    assert arm_response.success
    assert takeoff_response.success
    assert managers.requests[0]["method"].endswith("/Arm")
    assert managers.requests[1]["method"].endswith("/Takeoff")
    assert managers.requests[1]["params"] == {"timeout_sec": 20.0}


def test_lifecycle_service_errors_are_returned_to_ros_clients():
    managers, handler = make_handler()

    def fail(request, timeout_sec):
        raise TimeoutError("simulator did not respond")

    managers.request_async_blocking = fail
    response = handler.handle_land(Trigger.Request(), Trigger.Response())

    assert not response.success
    assert "simulator did not respond" in response.message


def test_timeouts_must_be_positive():
    managers = FakeTopicsManagers()

    try:
        RobotControlHandler(
            robot_path="/airsim_node/robots/Drone1",
            topics_managers=managers,
            takeoff_timeout_sec=0.0,
        )
    except ValueError as exc:
        assert "takeoff_timeout_sec" in str(exc)
    else:
        raise AssertionError("zero takeoff timeout should be rejected")


def test_clear_destroys_topics_and_services_idempotently():
    managers, handler = make_handler()
    services = list(managers.ros_node.services)

    handler.clear()
    handler.clear()

    assert not managers.ros_topics_manager.subscribers
    assert all(service.destroyed for service in services)

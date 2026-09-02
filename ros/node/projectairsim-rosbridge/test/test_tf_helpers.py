import threading

from projectairsim_rosbridge.tf_helpers import TFBroadcaster


class BlockingRate:
    def __init__(self):
        self.sleep_started = threading.Event()
        self.destroyed = threading.Event()

    def sleep(self):
        self.sleep_started.set()
        self.destroyed.wait(timeout=5.0)

    def destroy(self):
        self.destroyed.set()


class FakeTransformBroadcaster:
    def sendTransform(self, transform):
        pass


class FakeROSNode:
    ROSInterruptException = RuntimeError

    def __init__(self):
        self.rate = BlockingRate()

    def create_rate(self, frequency):
        return self.rate

    def create_static_transform_broadcaster(self):
        return FakeTransformBroadcaster()

    def create_transform_broadcaster(self):
        return FakeTransformBroadcaster()

    @staticmethod
    def get_time_now():
        return 0

    @staticmethod
    def get_time_now_msg():
        return None

    @staticmethod
    def get_time_to_msg(timestamp):
        return None


def test_clear_wakes_and_joins_transform_thread():
    ros_node = FakeROSNode()
    broadcaster = TFBroadcaster(ros_node)
    thread = broadcaster.broadcast_thread
    broadcaster.add_frame("robot", "map")
    broadcaster.start()

    assert ros_node.rate.sleep_started.wait(timeout=1.0)
    broadcaster.clear()
    broadcaster.clear()

    assert ros_node.rate.destroyed.is_set()
    assert not thread.is_alive()

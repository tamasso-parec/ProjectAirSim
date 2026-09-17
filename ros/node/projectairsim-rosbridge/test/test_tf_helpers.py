import threading

from builtin_interfaces.msg import Time

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
    def __init__(self):
        self.sent = []

    def sendTransform(self, transform):
        self.sent.append(transform)


class FakeROSNode:
    ROSInterruptException = RuntimeError

    def __init__(self):
        self.rate = BlockingRate()
        self.transform_broadcaster = FakeTransformBroadcaster()
        self.static_transform_broadcaster = FakeTransformBroadcaster()

    def create_rate(self, frequency):
        return self.rate

    def create_static_transform_broadcaster(self):
        return self.static_transform_broadcaster

    def create_transform_broadcaster(self):
        return self.transform_broadcaster

    @staticmethod
    def get_time_now():
        return 7

    @staticmethod
    def get_time_now_msg():
        return Time(sec=7, nanosec=0)

    @staticmethod
    def get_time_from_nanos(nanos):
        return int(nanos)

    @staticmethod
    def get_time_to_msg(timestamp):
        nanos = int(timestamp)
        return Time(sec=nanos // 1000000000, nanosec=nanos % 1000000000)


class FakeSimTimeSource:
    """Stands in for SimTimeSource with a fixed simulation time."""

    def __init__(self, nanos):
        self.nanos = nanos

    def now_msg(self):
        return Time(
            sec=self.nanos // 1000000000, nanosec=self.nanos % 1000000000
        )


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


def test_frames_are_stamped_from_the_wall_clock_without_a_sim_time_source():
    ros_node = FakeROSNode()
    broadcaster = TFBroadcaster(ros_node)
    try:
        broadcaster.add_frame("robot", "map")
        stamped = broadcaster.frames["robot"].transform_stamped

        assert stamped.header.stamp == Time(sec=7, nanosec=0)
        assert stamped.header.frame_id == "map"
        assert stamped.child_frame_id == "robot"
    finally:
        broadcaster.clear()


def test_frames_are_stamped_from_simulation_time_when_configured():
    ros_node = FakeROSNode()
    sim_time = FakeSimTimeSource(1500000000)
    broadcaster = TFBroadcaster(ros_node, sim_time=sim_time)
    try:
        broadcaster.add_frame("robot", "map")

        assert broadcaster.frames["robot"].transform_stamped.header.stamp == Time(
            sec=1, nanosec=500000000
        )

        # set_frame() without an explicit time also follows simulation time.
        sim_time.nanos = 2250000000
        broadcaster.set_frame("robot", broadcaster.frames["robot"].transform_stamped.transform)

        assert broadcaster.frames["robot"].transform_stamped.header.stamp == Time(
            sec=2, nanosec=250000000
        )
    finally:
        broadcaster.clear()


def test_set_frame_honours_an_explicit_time_value():
    ros_node = FakeROSNode()
    broadcaster = TFBroadcaster(ros_node, sim_time=FakeSimTimeSource(1000000000))
    try:
        broadcaster.add_frame("robot", "map")
        transform = broadcaster.frames["robot"].transform_stamped.transform
        broadcaster.set_frame("robot", transform, timevalue=4000000000)

        assert broadcaster.frames["robot"].transform_stamped.header.stamp == Time(
            sec=4, nanosec=0
        )
    finally:
        broadcaster.clear()

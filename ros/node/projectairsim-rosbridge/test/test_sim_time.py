"""Simulation clock source and /clock publication."""
from builtin_interfaces.msg import Time

import pytest

from projectairsim_rosbridge.sim_time import SimTimeSource


class FakePublisher:
    def __init__(self, topic):
        self.topic = topic
        self.published = []
        self.destroyed = False

    def publish(self, msg):
        self.published.append(msg.clock)

    def destroy(self):
        self.destroyed = True


class FakeROSNode:
    WALL_CLOCK = Time(sec=1700000000, nanosec=0)

    def __init__(self):
        self.publishers = []

    def create_publisher(self, topic, msg_type, **kwargs):
        publisher = FakePublisher(topic)
        publisher.kwargs = kwargs
        self.publishers.append(publisher)
        return publisher

    @staticmethod
    def get_time_now_msg():
        return FakeROSNode.WALL_CLOCK

    @staticmethod
    def get_time_from_nanos(nanos):
        return int(nanos)

    @staticmethod
    def get_time_to_msg(timestamp):
        nanos = int(timestamp)
        return Time(sec=nanos // 1000000000, nanosec=nanos % 1000000000)


def source(enabled=True, **kwargs):
    node = FakeROSNode()
    sim_time = SimTimeSource(node, enabled=enabled, **kwargs)
    sim_time.start()
    return node, sim_time


def test_disabled_source_uses_the_wall_clock_and_publishes_nothing():
    node, sim_time = source(enabled=False)

    assert not node.publishers
    assert sim_time.now_msg() == FakeROSNode.WALL_CLOCK
    assert sim_time.stamp({"time_stamp": 5000000000}) == FakeROSNode.WALL_CLOCK
    assert sim_time.update(5000000000) is False
    assert sim_time.sim_time_nanos is None


def test_enabled_source_publishes_the_clock_topic():
    node, sim_time = source(clock_topic="/clock")

    assert [publisher.topic for publisher in node.publishers] == ["/clock"]
    assert sim_time.update(1500000000) is True
    assert node.publishers[0].published == [Time(sec=1, nanosec=500000000)]
    assert sim_time.sim_time_nanos == 1500000000


def test_messages_are_stamped_from_their_own_simulation_timestamp():
    _, sim_time = source()

    assert sim_time.stamp({"time_stamp": 2250000000}) == Time(
        sec=2, nanosec=250000000
    )
    assert sim_time.sim_time_nanos == 2250000000


def test_messages_without_a_timestamp_use_the_latest_simulation_time():
    _, sim_time = source()
    sim_time.update(3000000000)

    assert sim_time.stamp({"width": 640}) == Time(sec=3, nanosec=0)
    assert sim_time.stamp(None) == Time(sec=3, nanosec=0)
    assert sim_time.now_msg() == Time(sec=3, nanosec=0)


def test_time_starts_at_zero_before_the_simulator_produces_data():
    _, sim_time = source()

    assert sim_time.sim_time_nanos is None
    assert sim_time.now_msg() == Time(sec=0, nanosec=0)


def test_out_of_order_messages_do_not_move_the_clock_backwards():
    node, sim_time = source()
    sim_time.update(5000000000)

    # A slower topic reporting an older timestamp is ignored.
    assert sim_time.update(4999000000) is False
    assert sim_time.sim_time_nanos == 5000000000
    assert node.publishers[0].published == [Time(sec=5, nanosec=0)]


def test_a_large_backwards_jump_is_treated_as_a_simulator_restart():
    node, sim_time = source(reset_threshold_nanos=1000000000)
    sim_time.update(60000000000)

    assert sim_time.update(500000000) is True
    assert sim_time.sim_time_nanos == 500000000
    assert node.publishers[0].published == [
        Time(sec=60, nanosec=0),
        Time(sec=0, nanosec=500000000),
    ]


def test_reset_forgets_the_clock_so_a_new_scene_starts_over():
    node, sim_time = source()
    sim_time.update(9000000000)
    sim_time.reset()

    assert sim_time.sim_time_nanos is None
    # A fresh low timestamp is accepted rather than rejected as out of order.
    assert sim_time.update(1000000) is True
    assert sim_time.sim_time_nanos == 1000000


def test_zero_and_invalid_timestamps_are_ignored():
    node, sim_time = source()

    assert sim_time.update(0) is False
    assert sim_time.update(-5) is False
    assert sim_time.update(None) is False
    assert sim_time.update("not a number") is False
    assert sim_time.sim_time_nanos is None
    assert not node.publishers[0].published


def test_min_step_bounds_the_clock_publication_rate():
    node, sim_time = source(min_step_nanos=10000000)  # 10 ms
    sim_time.update(1000000000)
    # Advancing by 1 ms each time must not publish until 10 ms have passed.
    for step in range(1, 10):
        sim_time.update(1000000000 + step * 1000000)

    assert node.publishers[0].published == [Time(sec=1, nanosec=0)]

    sim_time.update(1010000000)

    assert node.publishers[0].published == [
        Time(sec=1, nanosec=0),
        Time(sec=1, nanosec=10000000),
    ]
    # The clock itself still tracks every update, only publication is bounded.
    assert sim_time.sim_time_nanos == 1010000000


def test_stop_destroys_the_clock_publisher():
    node, sim_time = source()
    sim_time.stop()

    assert node.publishers[0].destroyed
    # Updates after stopping must not raise.
    assert sim_time.update(1000000000) is True


def test_start_is_idempotent():
    node, sim_time = source()
    sim_time.start()

    assert len(node.publishers) == 1


@pytest.mark.parametrize(
    "kwargs", [{"min_step_nanos": -1}, {"reset_threshold_nanos": 0}]
)
def test_invalid_settings_are_rejected(kwargs):
    with pytest.raises(ValueError):
        SimTimeSource(FakeROSNode(), enabled=True, **kwargs)


class RecordingLogger:
    def __init__(self):
        self.messages = []

    def info(self, message):
        self.messages.append(message)


def test_a_detected_restart_is_logged():
    logger = RecordingLogger()
    sim_time = SimTimeSource(
        FakeROSNode(), enabled=True, reset_threshold_nanos=1000000000, logger=logger
    )
    sim_time.start()
    sim_time.update(60000000000)
    assert not logger.messages

    sim_time.update(500000000)

    assert len(logger.messages) == 1
    assert "jumped backwards" in logger.messages[0]


def test_ordinary_advances_are_not_logged():
    logger = RecordingLogger()
    sim_time = SimTimeSource(FakeROSNode(), enabled=True, logger=logger)
    sim_time.start()
    sim_time.update(1000000000)
    sim_time.update(2000000000)
    # An out-of-order message is routine and must stay quiet.
    sim_time.update(1999000000)

    assert logger.messages == []

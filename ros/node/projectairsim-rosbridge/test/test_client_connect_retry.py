"""Tests for outwaiting Project AirSim's start-up on the client connection."""

# Unreal opens its topic and service ports only once its map has loaded, well
# after the bridge process starts, so the first connection attempt is normally
# refused. Taking that refusal at face value kills the bridge before the
# load_scene subscriber is ever created, and the scene publication then waits
# for a subscriber that cannot appear. Pinned here because the failure is
# silent: every process stays up and the launch simply never progresses.
from types import SimpleNamespace

import pynng
import pytest

from projectairsim_rosbridge.ros_bridge import ProjectAirSimROSBridge


class FakeSocket:
    def __init__(self):
        self.closed = False

    def close(self):
        self.closed = True


class FakeClient:
    """A client that refuses a given number of times, then connects."""

    def __init__(self, refusals, address="127.0.0.1"):
        self.refusals = refusals
        self.address = address
        self.attempts = 0
        self.sockets = []
        self.socket_topics = None
        self.socket_services = None

    def connect(self):
        self.attempts += 1
        # The real client builds both sockets before dialling either, so a
        # refusal leaves them behind for the caller to clean up.
        self.socket_topics = FakeSocket()
        self.socket_services = FakeSocket()
        self.sockets.extend([self.socket_topics, self.socket_services])
        if self.attempts <= self.refusals:
            raise pynng.exceptions.ConnectionRefused("Connection refused", 0)


def make_bridge(client, monkeypatch):
    """A bare bridge with only what _connect_client touches."""
    bridge = ProjectAirSimROSBridge.__new__(ProjectAirSimROSBridge)
    bridge.projectairsim_client = client
    bridge.logger = SimpleNamespace(info=lambda message: None)
    # Retry immediately: the interval is a politeness to the simulator, not
    # part of what is under test, and waiting for it would only slow the suite.
    monkeypatch.setattr(ProjectAirSimROSBridge, "CONNECT_RETRY_INTERVAL_SEC", 0)
    return bridge


def test_connect_retries_until_the_simulator_accepts(monkeypatch):
    client = FakeClient(refusals=3)
    bridge = make_bridge(client, monkeypatch)

    bridge._connect_client(timeout_sec=30.0)

    assert client.attempts == 4


def test_connect_gives_up_once_the_budget_is_spent(monkeypatch):
    client = FakeClient(refusals=100)
    bridge = make_bridge(client, monkeypatch)

    now = [0.0]
    monkeypatch.setattr(
        "projectairsim_rosbridge.ros_bridge.time.monotonic", lambda: now[0]
    )
    monkeypatch.setattr(
        "projectairsim_rosbridge.ros_bridge.time.sleep",
        lambda _: now.__setitem__(0, now[0] + 0.5),
    )

    with pytest.raises(pynng.exceptions.ConnectionRefused):
        bridge._connect_client(timeout_sec=2.0)

    # The refusal is reported as itself rather than as a timeout, so the log
    # still names what actually went wrong when the simulator is truly absent.
    assert client.attempts >= 2


def test_a_zero_budget_makes_one_attempt(monkeypatch):
    client = FakeClient(refusals=1)
    bridge = make_bridge(client, monkeypatch)

    with pytest.raises(pynng.exceptions.ConnectionRefused):
        bridge._connect_client(timeout_sec=0.0)

    assert client.attempts == 1


def test_refused_attempts_do_not_leak_sockets(monkeypatch):
    client = FakeClient(refusals=3)
    bridge = make_bridge(client, monkeypatch)

    bridge._connect_client(timeout_sec=30.0)

    # Every socket from a refused attempt is closed; the pair from the
    # successful attempt is the connection itself and stays open.
    refused, connected = client.sockets[:-2], client.sockets[-2:]
    assert refused and all(socket.closed for socket in refused)
    assert not any(socket.closed for socket in connected)

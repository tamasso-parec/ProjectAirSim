"""Removing the level's own props so the scene matches the world being stood in for."""

# A photorealistic environment is a level somebody built, and it arrives with
# everything that level contains. Blocks, for instance, is Unreal's template
# map: around 160 cubes scattered over the ground. A Gazebo world holds only
# the models the experiment asked for, so leaving the level's props in place
# silently changes the obstacle course the results describe. These tests pin
# that the removal happens, that it is scoped to what the profile named, and
# that failing to remove something does not cost the scene.
from types import SimpleNamespace

import pytest

from projectairsim_rosbridge.interface_profile import InterfaceProfile, ProfileError
from projectairsim_rosbridge.ros_bridge import ProjectAirSimROSBridge


class FakeWorld:
    """Stands in for projectairsim.World's object listing and destruction."""

    def __init__(self, objects, undestroyable=(), unlistable=(), paused=False):
        self.objects = list(objects)
        self.undestroyable = set(undestroyable)
        self.unlistable = set(unlistable)
        self.listed = []
        self.paused = paused
        # Ordering is the whole point: anything destroyed after the clock
        # starts happened in a world the vehicle was already living in.
        self.events = []

    def is_paused(self):
        return self.paused

    def resume(self):
        self.paused = False
        self.events.append("resume")

    def list_objects(self, name_regex):
        import re

        self.listed.append(name_regex)
        if name_regex in self.unlistable:
            raise RuntimeError("service call failed")
        pattern = re.compile(name_regex)
        return [name for name in self.objects if pattern.fullmatch(name)]

    def destroy_object(self, object_name):
        if object_name in self.undestroyable:
            raise RuntimeError("actor is not destructible")
        self.objects.remove(object_name)
        self.events.append(f"destroy:{object_name}")


def make_bridge(profile_config):
    bridge = ProjectAirSimROSBridge.__new__(ProjectAirSimROSBridge)
    bridge.interface_profile = InterfaceProfile(profile_config)
    bridge.logger = SimpleNamespace(
        info=lambda message: None,
        warning=lambda message: None,
    )
    return bridge


BLOCKS = [f"TemplateCube_Rounded_{index}" for index in range(5)]


def test_the_levels_props_are_removed_and_the_scenes_own_are_not():
    world = FakeWorld(BLOCKS + ["Wall", "TemplateFloor"])
    bridge = make_bridge({"scene": {"remove_objects": ["TemplateCube_Rounded.*"]}})

    bridge._remove_scene_objects(world)

    # The wall the scene spawned is the obstacle under test, and the floor is
    # the ground plane. Neither is the level's clutter.
    assert sorted(world.objects) == ["TemplateFloor", "Wall"]


def test_nothing_is_touched_without_the_profile_asking():
    world = FakeWorld(BLOCKS + ["Wall"])
    bridge = make_bridge({})

    bridge._remove_scene_objects(world)

    assert world.objects == BLOCKS + ["Wall"]
    # An empty profile must not even ask, so that a bridge against a scene
    # with no removals costs no service round trips.
    assert world.listed == []


def test_an_object_that_will_not_die_does_not_cost_the_scene():
    # The scene is loaded by this point, and loading it is what connects
    # Project AirSim to the flight controller. Raising here would trade a
    # slightly wrong world for no world at all.
    world = FakeWorld(BLOCKS, undestroyable={"TemplateCube_Rounded_2"})
    bridge = make_bridge({"scene": {"remove_objects": ["TemplateCube_Rounded.*"]}})

    bridge._remove_scene_objects(world)

    assert world.objects == ["TemplateCube_Rounded_2"]


def test_a_failed_listing_does_not_stop_the_other_patterns():
    world = FakeWorld(BLOCKS + ["Debris_1"], unlistable={"Debris.*"})
    bridge = make_bridge(
        {"scene": {"remove_objects": ["Debris.*", "TemplateCube_Rounded.*"]}}
    )

    bridge._remove_scene_objects(world)

    assert world.objects == ["Debris_1"]


# ---------------------------------------------------------------------------
# Profile parsing
# ---------------------------------------------------------------------------


def test_an_unparseable_pattern_is_refused_by_the_profile():
    # The simulator would just match nothing and leave the objects in place,
    # which looks exactly like a world that was cleaned successfully.
    with pytest.raises(ProfileError, match="valid regular expression"):
        InterfaceProfile({"scene": {"remove_objects": ["TemplateCube_Rounded(["]}})


def test_a_bare_string_is_refused_rather_than_read_as_characters():
    with pytest.raises(ProfileError, match="must be a list"):
        InterfaceProfile({"scene": {"remove_objects": "TemplateCube_Rounded.*"}})


def test_an_empty_profile_removes_nothing():
    assert InterfaceProfile().scene.remove_objects == []
    assert InterfaceProfile().scene.enabled is False


# ---------------------------------------------------------------------------
# Ordering against the simulation clock
# ---------------------------------------------------------------------------


def test_the_props_are_gone_before_the_clock_starts():
    """
    A robot exists the moment the scene loads, in whatever the level holds at
    that instant. Blocks puts a cube across the world origin, so a vehicle
    spawned there starts embedded in it, reporting a horizontal contact
    normal; fast physics does not treat that as a landing, so the vehicle is
    never clamped to a surface and falls indefinitely. Removing the cube
    afterwards does not undo the half second it already fell.
    """
    world = FakeWorld(BLOCKS, paused=True)
    bridge = make_bridge({"scene": {"remove_objects": ["TemplateCube_Rounded.*"]}})

    bridge._remove_scene_objects(world)
    bridge._start_scene_clock(world)

    assert world.events[-1] == "resume"
    assert all(event.startswith("destroy:") for event in world.events[:-1])
    assert world.paused is False


def test_a_paused_scene_is_started_even_with_nothing_to_change():
    # Otherwise a scene that pauses on start is simply left stopped, and every
    # node waiting on simulated time waits forever.
    world = FakeWorld(BLOCKS, paused=True)
    bridge = make_bridge({})

    bridge._remove_scene_objects(world)
    bridge._start_scene_clock(world)

    assert world.paused is False


def test_a_running_scene_is_left_alone():
    world = FakeWorld(BLOCKS, paused=False)
    bridge = make_bridge({})

    bridge._start_scene_clock(world)

    assert world.events == []


def test_a_clock_that_will_not_start_is_reported():
    class StuckWorld(FakeWorld):
        def resume(self):
            raise RuntimeError("service call failed")

    world = StuckWorld(BLOCKS, paused=True)
    reported = []
    bridge = make_bridge({})
    bridge.logger = SimpleNamespace(
        info=lambda message: None,
        warning=lambda message: None,
        error=reported.append,
    )

    bridge._start_scene_clock(world)

    assert reported and "will not advance" in reported[0]

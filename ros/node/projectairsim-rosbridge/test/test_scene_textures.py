"""Dressing the loaded scene with photographs, after removal and before the clock."""

# A scene rebuilt from primitives inside a template level shows the camera grey
# shapes. These tests pin that the profile's texture rules reach exactly the
# objects they name, that the choice of image is reproducible, that it happens
# before simulated time starts, and that a failure costs neither the scene nor
# the other objects.
from types import SimpleNamespace

import pytest

from projectairsim_rosbridge.interface_profile import InterfaceProfile, ProfileError
from projectairsim_rosbridge.ros_bridge import ProjectAirSimROSBridge


class FakeWorld:
    def __init__(self, objects, untexturable=(), paused=True):
        self.objects = list(objects)
        self.untexturable = set(untexturable)
        self.paused = paused
        self.textures = {}
        self.events = []

    def is_paused(self):
        return self.paused

    def resume(self):
        self.paused = False
        self.events.append("resume")

    def list_objects(self, name_regex):
        import re

        pattern = re.compile(name_regex)
        return [name for name in self.objects if pattern.fullmatch(name)]

    def destroy_object(self, object_name):
        self.objects.remove(object_name)
        self.events.append(f"destroy:{object_name}")

    def set_object_texture_from_file(self, object_name, path):
        if object_name in self.untexturable:
            raise RuntimeError("no static mesh")
        self.textures[object_name] = path
        self.events.append(f"texture:{object_name}")
        return True


def make_bridge(profile_config, warnings=None):
    bridge = ProjectAirSimROSBridge.__new__(ProjectAirSimROSBridge)
    bridge.interface_profile = InterfaceProfile(profile_config)
    bridge.logger = SimpleNamespace(
        info=lambda message: None,
        warning=(warnings.append if warnings is not None else lambda message: None),
    )
    return bridge


@pytest.fixture
def texture_dir(tmp_path):
    for role, names in {"bark": ["a", "b", "c"], "ground": ["g"]}.items():
        (tmp_path / role).mkdir()
        for name in names:
            (tmp_path / role / f"{name}_Color.png").write_bytes(b"png")
    return tmp_path


def textures(texture_dir, seed=0):
    return {"scene": {"textures": {
        "directory": str(texture_dir),
        "seed": seed,
        "rules": [
            {"objects": "[0-9]+_link_visual", "files": "bark/*_Color.png"},
            {"objects": "ground_tile_.*|Ground", "files": "ground/*_Color.png"},
        ],
    }}}


def test_each_rule_reaches_only_the_objects_it_names(texture_dir):
    world = FakeWorld(["0_link_visual", "1_link_visual", "ground_tile_0_0", "Ground", "SunSky"])
    make_bridge(textures(texture_dir))._texture_scene_objects(world)

    assert set(world.textures) == {"0_link_visual", "1_link_visual", "ground_tile_0_0", "Ground"}
    assert all("/bark/" in world.textures[n] for n in ("0_link_visual", "1_link_visual"))
    assert world.textures["Ground"].endswith("ground/g_Color.png")


def test_the_choice_is_reproducible_and_follows_the_seed(texture_dir):
    names = [f"{i}_link_visual" for i in range(40)]
    first, second, other = FakeWorld(names), FakeWorld(list(reversed(names))), FakeWorld(names)
    make_bridge(textures(texture_dir))._texture_scene_objects(first)
    make_bridge(textures(texture_dir))._texture_scene_objects(second)
    make_bridge(textures(texture_dir, seed=7))._texture_scene_objects(other)

    assert first.textures == second.textures
    assert first.textures != other.textures
    assert len(set(first.textures.values())) == 3


def test_textures_are_applied_before_the_clock_starts(texture_dir):
    world = FakeWorld(["0_link_visual", "TemplateCube_Rounded_1"])
    config = textures(texture_dir)
    config["scene"]["remove_objects"] = ["TemplateCube_Rounded.*"]
    bridge = make_bridge(config)

    bridge._remove_scene_objects(world)
    bridge._texture_scene_objects(world)
    world.resume()

    assert world.events == ["destroy:TemplateCube_Rounded_1", "texture:0_link_visual", "resume"]


def test_a_failure_is_reported_and_the_rest_still_textured(texture_dir):
    warnings = []
    world = FakeWorld(["0_link_visual", "1_link_visual"], untexturable={"0_link_visual"})
    make_bridge(textures(texture_dir), warnings)._texture_scene_objects(world)

    assert set(world.textures) == {"1_link_visual"}
    assert warnings and "0_link_visual" in warnings[0]


def test_missing_image_files_are_reported_not_fatal(tmp_path):
    warnings = []
    world = FakeWorld(["0_link_visual"])
    make_bridge(textures(tmp_path), warnings)._texture_scene_objects(world)

    assert world.textures == {}
    assert warnings and "no files match" in warnings[0]


def test_nothing_is_textured_without_the_profile_asking():
    world = FakeWorld(["0_link_visual"])
    make_bridge({})._texture_scene_objects(world)
    assert world.textures == {}


@pytest.mark.parametrize("textures_config, message", [
    ({"rules": [{"objects": "x", "files": "*.png"}]}, "directory"),
    ({"directory": "/t", "rules": []}, "rules"),
    ({"directory": "/t", "rules": [{"objects": "(", "files": "*.png"}]}, "regular expression"),
    ({"directory": "/t", "rules": [{"objects": "x"}]}, "files"),
    ({"directory": "/t", "seed": "0", "rules": [{"objects": "x", "files": "*.png"}]}, "seed"),
])
def test_malformed_texture_settings_are_refused(textures_config, message):
    with pytest.raises(ProfileError, match=message):
        InterfaceProfile({"scene": {"textures": textures_config}})


def test_the_directory_expands_environment_variables(monkeypatch, texture_dir):
    monkeypatch.setenv("UNSEEN_TEXTURES", str(texture_dir))
    profile = InterfaceProfile({"scene": {"textures": {
        "directory": "$UNSEEN_TEXTURES",
        "rules": [{"objects": "x", "files": "*.png"}],
    }}})
    assert profile.scene.textures.directory == str(texture_dir)

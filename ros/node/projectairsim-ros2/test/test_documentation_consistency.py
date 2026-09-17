"""The documentation against the code it documents.

Documentation drifting from the code is the most common kind of rot: a renamed
parameter or a changed default leaves instructions that look authoritative and
quietly do not work. These tests read the docs and check the claims that can be
checked mechanically -- links, paths, parameter names, defaults, and the
launch arguments used in the copy-pasteable examples.

Everything here is offline: no simulator, no ROS node, no network.
"""
import importlib.util
import os
import re
from pathlib import Path

import pytest

from launch.actions import DeclareLaunchArgument


os.environ.setdefault("ROS_LOG_DIR", "/tmp/projectairsim_ros_test_logs")

REPO_ROOT = Path(__file__).resolve().parents[4]
LAUNCH_DIR = REPO_ROOT / "ros" / "node" / "projectairsim-ros2" / "launch"
ROS_DOC = REPO_ROOT / "docs" / "ros" / "ros.md"
QUICKSTART = REPO_ROOT / "QUICKSTART.md"
DOCS = (ROS_DOC, QUICKSTART)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def heading_anchors(text):
    """GitHub-style anchors for every Markdown heading in a document."""
    found = set()
    for line in text.splitlines():
        match = re.match(r"^#{1,6}\s+(.*?)\s*$", line)
        if not match:
            continue
        title = re.sub(r"[`*_]", "", match.group(1))
        title = re.sub(r"[^\w\s-]", "", title).strip().lower()
        found.add(re.sub(r"\s+", "-", title))
    return found


def links(text):
    """Every Markdown link as (label, target)."""
    return re.findall(r"\[([^\]]+)\]\(([^)]+)\)", text)


def bash_blocks(text):
    return re.findall(r"```bash\n(.*?)```", text, re.S)


def table_rows(text, heading):
    """Map the first two columns of the table under a heading."""
    section = text.split(heading)[1].split("\n## ")[0]
    rows = {}
    for line in section.splitlines():
        match = re.match(r"^\|\s*`([^`]+)`\s*\|\s*(.*?)\s*\|", line)
        if match:
            rows[match.group(1)] = match.group(2).strip().strip("`").strip()
    return rows


def launch_arguments(stem):
    spec = importlib.util.spec_from_file_location(
        stem, LAUNCH_DIR / f"{stem}.launch.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    description = module.generate_launch_description()
    return {
        entity.name
        for entity in description.entities
        if isinstance(entity, DeclareLaunchArgument)
    }


LAUNCH_FILES = {
    "projectairsim_bridge_ros2.launch.py": "projectairsim_bridge_ros2",
    "projectairsim_px4_bridge.launch.py": "projectairsim_px4_bridge",
    "projectairsim_px4_sitl.launch.py": "projectairsim_px4_sitl",
    "projectairsim_x500_realsense_frames.launch.py": (
        "projectairsim_x500_realsense_frames"
    ),
}


# ---------------------------------------------------------------------------
# Links and paths
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("doc", DOCS, ids=lambda p: p.name)
def test_internal_anchors_resolve(doc):
    text = doc.read_text()
    anchors = heading_anchors(text)
    dead = [
        target
        for _, target in links(text)
        if target.startswith("#") and target[1:] not in anchors
    ]

    assert dead == []


@pytest.mark.parametrize("doc", DOCS, ids=lambda p: p.name)
def test_relative_paths_exist(doc):
    missing = []
    for _, target in links(doc.read_text()):
        if target.startswith(("#", "http://", "https://")):
            continue
        path_part = target.split("#", 1)[0]
        if not (doc.parent / path_part).resolve().exists():
            missing.append(path_part)

    assert missing == []


@pytest.mark.parametrize("doc", DOCS, ids=lambda p: p.name)
def test_anchors_into_other_documents_resolve(doc):
    dead = []
    for _, target in links(doc.read_text()):
        if target.startswith(("#", "http://", "https://")) or "#" not in target:
            continue
        path_part, fragment = target.split("#", 1)
        resolved = (doc.parent / path_part).resolve()
        if resolved.suffix == ".md" and resolved.exists():
            if fragment not in heading_anchors(resolved.read_text()):
                dead.append(target)

    assert dead == []


# ---------------------------------------------------------------------------
# Commands in the documentation
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("doc", DOCS, ids=lambda p: p.name)
def test_launch_arguments_in_examples_exist(doc):
    """
    A `foo:=bar` in an example that names a launch file must be a real
    argument of that file, or the copy-pasted command fails.
    """
    unknown = set()
    for block in bash_blocks(doc.read_text()):
        for launch_file, stem in LAUNCH_FILES.items():
            if launch_file not in block:
                continue
            available = launch_arguments(stem)
            for argument in re.findall(r"(\w+):=", block):
                if argument not in available:
                    unknown.add(f"{launch_file}: {argument}")

    assert unknown == set()


@pytest.mark.parametrize("doc", DOCS, ids=lambda p: p.name)
def test_referenced_sim_configs_exist(doc):
    """Scene names passed to load_scene or `scene:=` must be real files."""
    config_dir = REPO_ROOT / "client" / "python" / "example_user_scripts" / "sim_config"
    text = doc.read_text()
    referenced = set(re.findall(r"(scene_[\w]+\.jsonc)", text))
    referenced |= set(re.findall(r"(robot_[\w]+\.jsonc)", text))

    assert referenced, "expected the docs to name at least one scene config"
    missing = [name for name in referenced if not (config_dir / name).exists()]

    assert missing == []


def test_the_documented_airframe_install_script_exists_and_is_executable():
    script = (
        REPO_ROOT
        / "ros"
        / "node"
        / "projectairsim-ros2"
        / "config"
        / "px4_airframes"
        / "install_px4_airframes.sh"
    )

    assert script.exists()
    assert os.access(script, os.X_OK), "documented as directly runnable"


# ---------------------------------------------------------------------------
# Parameter tables
# ---------------------------------------------------------------------------


def test_the_bridge_parameter_table_matches_the_launch_file():
    """
    Every bridge parameter the docs list must be a real launch argument, and
    every launch argument except the node name must be listed.
    """
    documented = set(table_rows(ROS_DOC.read_text(), "### Bridge parameters"))
    available = launch_arguments("projectairsim_bridge_ros2")

    assert documented - available == set(), "documented but not a launch argument"
    # node_name is described in prose rather than the table.
    assert available - documented - {"node_name"} == set(), "undocumented argument"


def test_the_profile_options_table_matches_the_code_defaults():
    """
    The defaults in the table are the ones a reader will rely on without
    testing, so they have to be the values the code actually uses.
    """
    from projectairsim_rosbridge.interface_profile import InterfaceProfile

    profile = InterfaceProfile()
    expected = {
        "frame_convention": profile.frame_convention,
        "world_frame": profile.world_frame,
        "sim_time.enabled": profile.sim_time.enabled,
        "sim_time.clock_topic": profile.sim_time.clock_topic,
        "sim_time.min_step_ms": profile.sim_time.min_step_nanos / 1e6,
        "depth.encoding": profile.depth.encoding,
        "depth.max_range_m": profile.depth.max_range_m,
        "depth.mono8_max_range_m": profile.depth.mono8_max_range_m,
        "points.frame_convention": profile.points.frame_convention,
        "points.decimation": profile.points.decimation,
        "collision.message": profile.collision.message,
        "tf.publish_robot_tf": profile.tf.publish_robot_tf,
        "tf.publish_sensor_tf": profile.tf.publish_sensor_tf,
        "tf.ground_truth_topic": profile.tf.ground_truth_topic,
    }

    rows = table_rows(ROS_DOC.read_text(), "### Profile options")

    def rendered(value):
        if isinstance(value, bool):
            return str(value).lower()
        if isinstance(value, float):
            return f"{value:.1f}" if value % 1 == 0 else repr(value)
        if value == "":
            return '""'
        return str(value)

    mismatches = []
    for key, value in expected.items():
        if key not in rows:
            mismatches.append(f"{key} is not documented")
        elif rows[key] != rendered(value):
            mismatches.append(f"{key}: doc {rows[key]!r} != code {rendered(value)!r}")

    assert mismatches == []
    # The glob sections are documented in the same table.
    assert {"frames", "topics"} <= set(rows)


def test_every_profile_section_is_documented():
    """A new profile section must not reach users undocumented."""
    from projectairsim_rosbridge.interface_profile import InterfaceProfile

    text = ROS_DOC.read_text()
    for section in InterfaceProfile._KNOWN_KEYS:
        assert f"`{section}" in text or f"{section}:" in text, section


# ---------------------------------------------------------------------------
# Claims that must stay true
# ---------------------------------------------------------------------------


def test_the_docs_warn_about_isolating_test_runs():
    """
    Running the suite on a shared DDS domain publishes /clock into whatever
    else is running. Both entry points must say so.
    """
    for doc in DOCS:
        text = doc.read_text()
        assert "ROS_DOMAIN_ID" in text, doc.name


def test_the_default_depth_encoding_is_documented_as_the_behaviour_change():
    """
    Depth changed from mono8 to metric 32FC1. Anyone upgrading needs to find
    that, along with how to get the old behaviour back.
    """
    text = ROS_DOC.read_text()

    assert "Behaviour change" in text
    assert "mono8" in text
    assert "32FC1" in text


def test_the_px4_ordering_requirement_is_documented():
    """
    The scene load is what makes Project AirSim connect to PX4, and home_set
    gates arming. Both are the usual reasons PX4 bring-up fails.
    """
    for doc in DOCS:
        text = doc.read_text()
        assert "home_set" in text, doc.name
        assert "4560" in text, doc.name

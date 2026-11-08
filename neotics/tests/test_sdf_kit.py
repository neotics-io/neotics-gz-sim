import sys
from pathlib import Path
from xml.etree import ElementTree as ET

import pytest

KIT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(KIT))

from neotics_scene import Scene  # noqa: E402
from scene_to_sdf import to_sdf  # noqa: E402


@pytest.fixture
def scene():
    return Scene.load(KIT / "samples" / "depot.scene.json")


@pytest.fixture
def world(scene):
    return ET.fromstring(to_sdf(scene)).find("world")


def test_every_part_is_a_model(scene, world):
    models = {m.get("name") for m in world.findall("model")}
    assert models == {"floor", "crate", scene.robot_name} | {b.name for b in scene.boxes}


def test_only_the_crate_and_arm_can_move(scene, world):
    moving = {m.get("name") for m in world.findall("model") if m.findtext("static") != "true"}
    assert moving == {"crate", scene.robot_name}


def test_each_joint_has_a_position_controller(scene, world):
    arm = next(m for m in world.findall("model") if m.get("name") == scene.robot_name)
    controlled = [p.findtext("joint_name") for p in arm.findall("plugin")]
    assert controlled == scene.joint_names == [j.get("name") for j in arm.findall("joint")]


def test_links_stack_up_from_the_base(scene, world):
    arm = next(m for m in world.findall("model") if m.get("name") == scene.robot_name)
    heights = [float(l.findtext("pose").split()[2]) for l in arm.findall("link")]
    assert heights == pytest.approx([0.0, 0.12, 0.42, 0.67])

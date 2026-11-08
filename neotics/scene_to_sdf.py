"""Turn a Neotics Scene into a Gazebo world.    python scene_to_sdf.py samples/depot.scene.json depot.sdf

The arm holds its home pose under position control. Send a target with:
    gz topic -t /neotics_arm/shoulder/target -m gz.msgs.Double -p "data: 0.7"
"""
import sys
from xml.etree import ElementTree as ET

from neotics_scene import Scene


def vec(values) -> str:
    return " ".join(f"{v:.6g}" for v in values)


def text(parent, tag, value, **attrs):
    node = ET.SubElement(parent, tag, **attrs)
    node.text = str(value)
    return node


def inertial(parent, mass, ixx, iyy, izz, z=0.0):
    node = ET.SubElement(parent, "inertial")
    text(node, "pose", vec((0, 0, z, 0, 0, 0)))
    text(node, "mass", f"{mass:.6g}")
    inertia = ET.SubElement(node, "inertia")
    for name, value in (("ixx", ixx), ("ixy", 0), ("ixz", 0), ("iyy", iyy), ("iyz", 0), ("izz", izz)):
        text(inertia, name, f"{value:.6g}")


def shape(link, label, build, pose=None):
    for tag in ("collision", "visual"):
        node = ET.SubElement(link, tag, name=f"{label}_{tag}")
        if pose:
            text(node, "pose", vec(pose))
        build(ET.SubElement(node, "geometry"))


def box_shape(size):
    return lambda geometry: text(ET.SubElement(geometry, "box"), "size", vec(size))


def static_box(world, box):
    model = ET.SubElement(world, "model", name=box.name)
    text(model, "static", "true")
    text(model, "pose", vec((*box.center, 0, 0, 0)))
    shape(ET.SubElement(model, "link", name="body"), box.name, box_shape(box.size))


def to_sdf(scene: Scene) -> str:
    root = ET.Element("sdf", version="1.10")
    world = ET.SubElement(root, "world", name=scene.name)
    physics = ET.SubElement(world, "physics", name="fast", type="ignored")
    text(physics, "max_step_size", "0.002")
    text(physics, "real_time_factor", "0")
    for library, name in (("physics", "Physics"), ("user-commands", "UserCommands"), ("scene-broadcaster", "SceneBroadcaster")):
        ET.SubElement(world, "plugin", filename=f"gz-sim-{library}-system", name=f"gz::sim::systems::{name}")
    sun = ET.SubElement(world, "light", type="directional", name="sun")
    text(sun, "pose", "0 0 8 0 0 0")
    text(sun, "direction", "-0.3 0.2 -1")

    floor = ET.SubElement(world, "model", name="floor")
    text(floor, "static", "true")
    def plane(geometry):
        node = ET.SubElement(geometry, "plane")
        text(node, "normal", "0 0 1")
        text(node, "size", vec(scene.floor))
    shape(ET.SubElement(floor, "link", name="body"), "floor", plane)

    for box in scene.boxes:
        static_box(world, box)
    for item in scene.items:
        model = ET.SubElement(world, "model", name=item.name)
        text(model, "pose", vec((*item.center, 0, 0, 0)))
        link = ET.SubElement(model, "link", name="body")
        sx, sy, sz = item.size
        inertial(link, item.mass, item.mass * (sy**2 + sz**2) / 12, item.mass * (sx**2 + sz**2) / 12, item.mass * (sx**2 + sy**2) / 12)
        shape(link, item.name, box_shape(item.size))

    arm = ET.SubElement(world, "model", name=scene.robot_name)
    text(arm, "pose", vec((*scene.base, 0, 0, 0)))
    parent, height = "world", 0.0
    for spec in scene.links:
        name = f"{spec.joint}_link"
        link = ET.SubElement(arm, "link", name=name)
        text(link, "pose", vec((0, 0, height, 0, 0, 0)))
        inertial(link, spec.mass, *spec.inertia, z=spec.length / 2)
        def cylinder(geometry, spec=spec):
            node = ET.SubElement(geometry, "cylinder")
            text(node, "radius", f"{spec.radius:.6g}")
            text(node, "length", f"{spec.length:.6g}")
        shape(link, name, cylinder, pose=(0, 0, spec.length / 2, 0, 0, 0))
        joint = ET.SubElement(arm, "joint", name=spec.joint, type="revolute")
        text(joint, "parent", parent)
        text(joint, "child", name)
        axis = ET.SubElement(joint, "axis")
        text(axis, "xyz", vec(spec.axis))
        limit = ET.SubElement(axis, "limit")
        text(limit, "lower", f"{spec.lower:.6g}")
        text(limit, "upper", f"{spec.upper:.6g}")
        dynamics = ET.SubElement(axis, "dynamics")
        text(dynamics, "damping", "0.6")
        parent, height = name, height + spec.length
    for spec in scene.links:
        plugin = ET.SubElement(arm, "plugin", filename="gz-sim-joint-position-controller-system", name="gz::sim::systems::JointPositionController")
        text(plugin, "joint_name", spec.joint)
        text(plugin, "topic", f"/{scene.robot_name}/{spec.joint}/target")
        text(plugin, "p_gain", "120")
        text(plugin, "d_gain", "12")
        text(plugin, "cmd_max", "60")
        text(plugin, "cmd_min", "-60")
    ET.indent(root)
    return '<?xml version="1.0"?>\n' + ET.tostring(root, encoding="unicode") + "\n"


if __name__ == "__main__":
    with open(sys.argv[2], "w", encoding="utf-8") as out:
        out.write(to_sdf(Scene.load(sys.argv[1])))

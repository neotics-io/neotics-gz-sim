"""Full check with real Gazebo: the world loads, runs, the crate stays on the table and the arm follows a command."""
import re
import signal
import subprocess
import sys
import tempfile
import time
from pathlib import Path

KIT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(KIT))

from neotics_scene import Scene  # noqa: E402
from scene_to_sdf import to_sdf  # noqa: E402


def poses(world: str) -> dict:
    """Every moving thing's position from one Gazebo pose message: name -> (x, y, z)."""
    raw = subprocess.run(["gz", "topic", "-e", "-n", "1", "-t", f"/world/{world}/dynamic_pose/info"], capture_output=True, text=True, timeout=60).stdout
    found = {}
    for block in raw.split("pose {")[1:]:
        name = re.search(r'name: "([^"]+)"', block)
        position = re.search(r"position \{(.*?)\}", block, re.S)
        if name and position:
            found[name.group(1)] = tuple(float((re.search(rf"{axis}: (\S+)", position.group(1)) or [0, 0])[1]) for axis in "xyz")
    return found


def main() -> int:
    scene = Scene.load(KIT / "samples" / "depot.scene.json")
    path = Path(tempfile.mkdtemp()) / "depot.sdf"
    path.write_text(to_sdf(scene), encoding="utf-8")
    subprocess.run(["gz", "sdf", "-k", str(path)], check=True)
    server = subprocess.Popen(["gz", "sim", "-s", "-r", "-v", "2", str(path)])
    try:
        time.sleep(15)
        before = poses(scene.name)
        print("crate at", before.get("crate"))
        crate = scene.items[0].center
        if "crate" not in before or abs(before["crate"][2] - crate[2]) > 0.02:
            print("FAIL: the crate is not resting on the table")
            return 1
        subprocess.run(["gz", "topic", "-t", f"/{scene.robot_name}/shoulder/target", "-m", "gz.msgs.Double", "-p", "data: 0.7"], check=True, timeout=30)
        time.sleep(8)
        after = poses(scene.name)
        tilted = {name: p for name, p in after.items() if name.endswith("elbow_link")}
        print("elbow link moved to", tilted)
        moved = tilted and abs(next(iter(tilted.values()))[0] - scene.base[0]) > 0.1
        return 0 if moved else 1
    finally:
        server.send_signal(signal.SIGINT)
        time.sleep(1)
        server.kill()


if __name__ == "__main__":
    sys.exit(main())

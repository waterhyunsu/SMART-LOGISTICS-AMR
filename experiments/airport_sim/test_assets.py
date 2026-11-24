"""Check the committed assets against their shared floor plan and Nav2 clearance."""

import math
from collections import deque

import pytest
import yaml
from PIL import Image

from make_airport import ASSETS, BOUNDARY, OBSTACLES, RESOLUTION, build, inside


def test_assets_are_reproducible(tmp_path):
    build(tmp_path)
    for filename in ("airport.sdf.xacro", "airport.pgm", "airport.yaml", "preview.png"):
        assert (tmp_path / filename).read_bytes() == (ASSETS / filename).read_bytes()


def test_all_waypoints_connected_with_costmap_clearance():
    settings = yaml.safe_load((ASSETS / "airport.yaml").read_text())
    waypoints = yaml.safe_load((ASSETS / "waypoints.yaml").read_text())
    assert waypoints["frame_id"] == "map"
    poses = waypoints["waypoints"]
    assert set(poses) == {"HOME", *(f"S{i}" for i in range(1, 5)),
                          *(f"G{i}" for i in range(1, 9))}
    assert all(set(pose) == {"x", "y", "yaw"} and all(map(math.isfinite, pose.values()))
               for pose in poses.values())
    image = Image.open(ASSETS / settings["image"])
    assert settings["resolution"] == RESOLUTION
    ox, oy, _ = settings["origin"]
    # 0.70 m is the current sandbox inflation radius. Use an independent
    # coarse sampling of the original geometry to check a connected clear route.
    spacing = 0.20
    radius = 0.70

    def clearance(x, y):
        for index in range(16):
            angle = math.tau * index / 16
            px, py = x + radius * math.cos(angle), y + radius * math.sin(angle)
            if not inside(px, py, BOUNDARY) or any(
                    abs(px - cx) <= width / 2 and abs(py - cy) <= depth / 2
                    for cx, cy, width, depth in OBSTACLES):
                return False
            column = math.floor((px - ox) / RESOLUTION)
            row = image.height - 1 - math.floor((py - oy) / RESOLUTION)
            if image.getpixel((column, row)) < 250:
                return False
        return True

    cells = set()
    for i in range(math.ceil(image.width * RESOLUTION / spacing)):
        for j in range(math.ceil(image.height * RESOLUTION / spacing)):
            if clearance(ox + i * spacing, oy + j * spacing):
                cells.add((i, j))
    targets = {}
    for name, pose in poses.items():
        x, y = pose["x"], pose["y"]
        assert clearance(x, y), name
        targets[name] = (round((x - ox) / spacing), round((y - oy) / spacing))
    home = targets["HOME"]
    assert home in cells
    queue = deque([home])
    cells.remove(home)
    reachable = {home}
    while queue:
        i, j = queue.popleft()
        for neighbor in ((i+1, j), (i-1, j), (i, j+1), (i, j-1)):
            if neighbor in cells:
                cells.remove(neighbor)
                reachable.add(neighbor)
                queue.append(neighbor)
    assert all(cell in reachable for cell in targets.values())


def test_left_and_right_arms_require_a_detour():
    poses = yaml.safe_load((ASSETS / "waypoints.yaml").read_text())["waypoints"]
    left, right = poses["G1"], poses["G5"]
    assert not inside((left["x"] + right["x"]) / 2,
                      (left["y"] + right["y"]) / 2, BOUNDARY)

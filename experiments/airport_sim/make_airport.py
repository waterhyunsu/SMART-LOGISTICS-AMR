"""Generate a Gazebo world, its occupancy grid, and a labeled preview."""

import argparse
import math
from pathlib import Path
from xml.etree import ElementTree as ET

import yaml
from PIL import Image, ImageDraw, ImageFont

ASSETS = Path(__file__).resolve().parent / "assets"
ORIGIN = (-18.0, -7.0)
EXTENT = (36.0, 24.0)
RESOLUTION = 0.05
WALL = 0.16
# Counterpart boundaries of each concourse share the same scale and coordinates
# in Gazebo and the occupancy map. Meters, +Y points up in the preview.
BOUNDARY = (
    (-16.0, -6.0), (-17.0, -3.1), (-12.4, 1.0), (-11.7, 2.3),
    (-15.3, 14.5), (-10.1, 16.0), (-5.7, 3.5), (-3.0, 4.0),
    (3.0, 4.0), (6.0, 3.3), (9.2, 16.0), (14.1, 14.8),
    (11.7, 2.2), (12.6, 0.65), (17.0, -2.4), (16.2, -5.8),
    (12.3, -5.0), (7.0, -2.8), (3.2, -2.4), (-3.1, -2.4),
    (-7.0, -2.9), (-12.1, -5.3),
)
# Service desks and kiosks provide asymmetry and require local detours.
OBSTACLES = ((0.0, 2.35, 2.8, 0.8), (3.5, 0.8, 0.9, 1.2), (-4.0, -0.85, 1.2, 0.6))
XACRO = "http://www.ros.org/wiki/xacro"


def inside(x, y, outline):
    crossings = sum(
        (y1 > y) != (y2 > y) and x < x1 + (x2 - x1) * (y - y1) / (y2 - y1)
        for (x1, y1), (x2, y2) in zip(outline, outline[1:] + outline[:1])
    )
    return crossings % 2 == 1


def element(parent, tag, content=None, **attributes):
    node = ET.SubElement(parent, tag, attributes)
    node.text = content
    return node


def geometry(parent, shape, dimensions):
    element(element(element(parent, "geometry"), shape), "size", dimensions)


def box(world, name, x, y, width, depth, height, yaw, color):
    model = element(world, "model", name=name)
    element(model, "static", "true")
    element(model, "pose", f"{x:.6f} {y:.6f} {height / 2:.6f} 0 0 {yaw:.8f}")
    link = element(model, "link", name="link")
    size = f"{width:.6f} {depth:.6f} {height:.6f}"
    geometry(element(link, "collision", name="collision"), "box", size)
    visual = element(link, "visual", name="visual")
    geometry(visual, "box", size)
    element(element(visual, "material"), "diffuse", color)


def write_world(path):
    ET.register_namespace("xacro", XACRO)
    root = ET.Element("sdf", version="1.6")
    element(root, f"{{{XACRO}}}arg", name="headless", default="true")
    # The packaged TurtleBot3 spawn/bridge uses /world/default.
    world = element(root, "world", name="default")
    element(world, "gravity", "0 0 -9.81")
    physics = element(world, "physics", name="airport", type="ode")
    element(physics, "max_step_size", "0.003")
    element(physics, "real_time_factor", "1")

    def plugin(parent, filename, name):
        return element(parent, "plugin", filename=f"gz-sim-{filename}-system",
                       name=f"gz::sim::systems::{name}")

    plugin(world, "physics", "Physics")
    plugin(world, "user-commands", "UserCommands")
    plugin(element(world, f"{{{XACRO}}}unless", value="$(arg headless)"),
           "scene-broadcaster", "SceneBroadcaster")
    element(plugin(world, "sensors", "Sensors"), "render_engine", "ogre2")
    plugin(world, "imu", "Imu")
    element(element(world, "scene"), "shadows", "false")
    sun = element(world, "light", name="sun", type="directional")
    element(sun, "direction", "-0.5 0.2 -0.9")
    element(sun, "diffuse", "0.9 0.9 0.9 1")
    element(sun, "cast_shadows", "false")
    floor = element(world, "model", name="floor")
    element(floor, "static", "true")
    link = element(floor, "link", name="link")
    for tag in ("collision", "visual"):
        plane = element(element(element(link, tag, name=tag), "geometry"), "plane")
        element(plane, "normal", "0 0 1")
        element(plane, "size", "40 36")
    for index, ((x1, y1), (x2, y2)) in enumerate(zip(BOUNDARY, BOUNDARY[1:] + BOUNDARY[:1])):
        box(world, f"wall_{index}", (x1+x2)/2, (y1+y2)/2,
            math.hypot(x2-x1, y2-y1) + WALL, WALL, 1.4,
            math.atan2(y2-y1, x2-x1), "0.25 0.48 0.60 1")
    for index, (x, y, width, depth) in enumerate(OBSTACLES):
        box(world, f"service_desk_{index}", x, y, width, depth, 0.7, 0,
            "0.78 0.44 0.20 1")
    ET.indent(root, space="  ")
    ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)


def build(output):
    output.mkdir(parents=True, exist_ok=True)
    image = Image.new("L", tuple(round(size / RESOLUTION) for size in EXTENT), 0)
    draw = ImageDraw.Draw(image)

    def pixel(x, y):
        return (round((x - ORIGIN[0]) / RESOLUTION - 0.5),
                round(image.height - 0.5 - (y - ORIGIN[1]) / RESOLUTION))

    boundary = [pixel(x, y) for x, y in BOUNDARY]
    draw.polygon(boundary, fill=254)
    width = math.ceil(WALL / RESOLUTION) + 1
    draw.line(boundary + boundary[:1], fill=0, width=width, joint="curve")
    for x, y, w, d in OBSTACLES:
        draw.rectangle((*pixel(x - w/2, y + d/2), *pixel(x + w/2, y - d/2)), fill=0)
    image.save(output / "airport.pgm")
    (output / "airport.yaml").write_text(
        f"image: airport.pgm\nmode: trinary\nresolution: {RESOLUTION}\n"
        f"origin: [{ORIGIN[0]}, {ORIGIN[1]}, 0.0]\n"
        "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n"
    )
    write_world(output / "airport.sdf.xacro")

    mask = image.resize((1080, 720), Image.Resampling.NEAREST)
    mask = mask.point(lambda level: 255 if level else 0)
    preview = Image.new("RGB", mask.size, "#122235")
    preview.paste("#d8e9ed", (0, 0, *mask.size), mask)
    labels = ImageDraw.Draw(preview)
    font = ImageFont.load_default(size=20)
    labels.text((540, 20), "MOVEIT  /  AIRPORT SIM", anchor="mt", font=font, fill="#f4fafb")
    labels.text((540, 49), "13 logistics waypoints | schematic floor plan",
                anchor="mt", font=font, fill="#b9d8e0")
    poses = yaml.safe_load((ASSETS / "waypoints.yaml").read_text())["waypoints"]
    for name, pose in poses.items():
        px, py = pixel(pose["x"], pose["y"])
        x, y = 1.5 * px, 1.5 * py
        color = "#16616a" if name == "HOME" else "#94491c" if name.startswith("S") else "#304894"
        labels.ellipse((x - 10, y - 10, x + 10, y + 10), fill=color, outline="white", width=2)
        labels.text((x + 13, y), name, anchor="lm", fill=color, font=font, stroke_width=1,
                    stroke_fill="#d8e9ed")
    labels.text((25, 690), "HOME  standby      S  pickup      G  delivery", font=font, fill="#f4fafb")
    preview.save(output / "preview.png")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ASSETS)
    build(parser.parse_args().output)

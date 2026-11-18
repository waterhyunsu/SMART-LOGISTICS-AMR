"""Make an occupancy map (pgm + yaml) straight from a Gazebo world file: the 'ground-truth' map of what the LiDAR sees.

    python3 tools/world_to_map.py worlds/lecture_hall_real.world -o experiments/r1/maps/lecture_hall_real
           [--seed 0 0] [--z 0.17] [--res 0.05] [--margin 0.5]

- Every <model> collision box/cylinder whose height range contains the scan height --z (default 0.17 m, about the
  LDS height of worlds/models/turtlebot3_waffle_pi_real) is drawn as an obstacle: a cell is occupied when its centre
  lies within half a cell of the shape (thin chair legs still give 1-4 cells, like a SLAM map).
- Free = cells reachable from --seed (robot start pose) without crossing an obstacle; everything else is unknown.
- World coordinates = map coordinates. The yaml records '# world: <name>' so scripts/run.sh starts that world when only
  --map is given. Reverse of tools/map_to_world.py.
- Difference from a SLAM map: no sensor noise, no unexplored stripes (an idealised map). Used for research 1
  (experiments/r1) so that planning is compared on a map that matches the Gazebo world exactly.
Only yaw rotations are handled (roll/pitch of models, links and collisions must be 0, as in this repo's worlds).
"""
import argparse
import math
import os
import xml.etree.ElementTree as ET
from collections import deque

import numpy as np

FREE, OCC, UNKNOWN = 254, 0, 205


def parse_pose(elem):
    """<pose>x y z r p yaw</pose> of a child of elem -> (x, y, z, yaw). Missing -> zeros."""
    p = elem.find('pose') if elem is not None else None
    if p is None or not (p.text or '').strip():
        return 0.0, 0.0, 0.0, 0.0
    v = [float(t) for t in p.text.split()] + [0.0] * 6
    if abs(v[3]) > 1e-6 or abs(v[4]) > 1e-6:
        raise ValueError(f'roll/pitch are not supported: {p.text.strip()}')
    return v[0], v[1], v[2], v[5]


def compose(a, b):
    """Pose b expressed in frame a -> pose in a's parent frame (x, y, z, yaw)."""
    ax, ay, az, ayaw = a
    bx, by, bz, byaw = b
    c, s = math.cos(ayaw), math.sin(ayaw)
    return ax + c * bx - s * by, ay + s * bx + c * by, az + bz, ayaw + byaw


def world_shapes(path):
    """Collision shapes of all models: [(kind, x, y, yaw, size_x_or_radius, size_y, z_min, z_max, model_name)]."""
    root = ET.parse(path).getroot()
    world = root.find('world')
    shapes = []
    for model in world.findall('model'):
        mpose = parse_pose(model)
        for link in model.findall('link'):
            lpose = compose(mpose, parse_pose(link))
            for col in link.findall('collision'):
                x, y, z, yaw = compose(lpose, parse_pose(col))
                geom = col.find('geometry')
                box, cyl = geom.find('box'), geom.find('cylinder')
                if box is not None:
                    sx, sy, sz = (float(t) for t in box.find('size').text.split())
                    shapes.append(('box', x, y, yaw, sx, sy, z - sz / 2, z + sz / 2, model.get('name')))
                elif cyl is not None:
                    r = float(cyl.find('radius').text)
                    ln = float(cyl.find('length').text)
                    shapes.append(('cylinder', x, y, 0.0, r, r, z - ln / 2, z + ln / 2, model.get('name')))
    return world.get('name'), shapes


def rasterize(shapes, res, z, margin):
    """Occupied mask (row 0 = bottom) and origin for the shapes that cross height z."""
    cut = [s for s in shapes if s[6] <= z <= s[7]]
    if not cut:
        raise SystemExit(f'높이 z={z} 에 걸치는 충돌 모양이 없습니다')
    def half_extent(s):
        """Axis-aligned half size (hx, hy) of a shape."""
        if s[0] == 'cylinder':
            return s[4], s[4]
        c, si = abs(math.cos(s[3])), abs(math.sin(s[3]))
        return c * s[4] / 2 + si * s[5] / 2, si * s[4] / 2 + c * s[5] / 2

    ext = [half_extent(s) for s in cut]
    x0 = min(s[1] - e[0] for s, e in zip(cut, ext)) - margin
    y0 = min(s[2] - e[1] for s, e in zip(cut, ext)) - margin
    x1 = max(s[1] + e[0] for s, e in zip(cut, ext)) + margin
    y1 = max(s[2] + e[1] for s, e in zip(cut, ext)) + margin
    ox, oy = math.floor(x0 / res) * res, math.floor(y0 / res) * res
    w, h = int(math.ceil((x1 - ox) / res)), int(math.ceil((y1 - oy) / res))
    occ = np.zeros((h, w), dtype=bool)
    half = res / 2
    for (kind, x, y, yaw, a, b, _, _, _), (ex, ey) in zip(cut, ext):
        c0 = max(0, int((x - ex - half - ox) / res) - 1)
        c1 = min(w, int((x + ex + half - ox) / res) + 2)
        r0 = max(0, int((y - ey - half - oy) / res) - 1)
        r1 = min(h, int((y + ey + half - oy) / res) + 2)
        cx = ox + (np.arange(c0, c1) + 0.5) * res
        cy = oy + (np.arange(r0, r1) + 0.5) * res
        gx, gy = np.meshgrid(cx - x, cy - y)
        if kind == 'cylinder':
            hit = np.hypot(gx, gy) <= a + half
        else:
            c, s = math.cos(-yaw), math.sin(-yaw)
            lx, ly = c * gx - s * gy, s * gx + c * gy     # cell centre in the box frame
            dx = np.maximum(np.abs(lx) - a / 2, 0.0)
            dy = np.maximum(np.abs(ly) - b / 2, 0.0)
            hit = np.hypot(dx, dy) <= half
        occ[r0:r1, c0:c1] |= hit
    return occ, ox, oy, len(cut)


def flood_free(occ, seed_rc):
    """Cells 4-connected to seed_rc that are not occupied."""
    h, w = occ.shape
    free = np.zeros_like(occ)
    r, c = seed_rc
    if not (0 <= r < h and 0 <= c < w) or occ[r, c]:
        raise SystemExit(f'시작점 칸 {seed_rc} 이 맵 밖이거나 장애물입니다 (--seed 확인)')
    q = deque([(r, c)])
    free[r, c] = True
    while q:
        r, c = q.popleft()
        for nr, nc in ((r + 1, c), (r - 1, c), (r, c + 1), (r, c - 1)):
            if 0 <= nr < h and 0 <= nc < w and not free[nr, nc] and not occ[nr, nc]:
                free[nr, nc] = True
                q.append((nr, nc))
    return free


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('world')
    ap.add_argument('-o', '--out', required=True, help='출력 경로 (확장자 없이). <out>.pgm, <out>.yaml')
    ap.add_argument('--seed', type=float, nargs=2, default=(0.0, 0.0), metavar=('X', 'Y'))
    ap.add_argument('--z', type=float, default=0.17, help='LiDAR 스캔 높이 (m)')
    ap.add_argument('--res', type=float, default=0.05)
    ap.add_argument('--margin', type=float, default=0.5)
    args = ap.parse_args()

    name, shapes = world_shapes(args.world)
    occ, ox, oy, n = rasterize(shapes, args.res, args.z, args.margin)
    seed = (int(math.floor((args.seed[1] - oy) / args.res)), int(math.floor((args.seed[0] - ox) / args.res)))
    free = flood_free(occ, seed)
    grid = np.full(occ.shape, UNKNOWN, dtype=np.uint8)
    grid[free] = FREE
    grid[occ] = OCC
    img = np.flipud(grid)    # pgm row 0 = top of the map

    out_dir = os.path.dirname(os.path.abspath(args.out))
    os.makedirs(out_dir, exist_ok=True)
    base = os.path.basename(args.out)
    h, w = img.shape
    with open(args.out + '.pgm', 'wb') as f:
        f.write(f'P5\n# made by tools/world_to_map.py from {os.path.basename(args.world)}\n{w} {h}\n255\n'.encode())
        f.write(img.tobytes())
    with open(args.out + '.yaml', 'w', encoding='utf-8') as f:
        f.write(f'# world: {name}\n'
                f'# made by tools/world_to_map.py from {args.world} (scan height z={args.z} m, seed {args.seed[0]} {args.seed[1]})\n'
                f'image: {base}.pgm\n'
                f'mode: trinary\n'
                f'resolution: {args.res}\n'
                f'origin: [{ox:.3f}, {oy:.3f}, 0.0]\n'
                f'negate: 0\n'
                f'occupied_thresh: 0.65\n'
                f'free_thresh: 0.196\n')
    print(f'[world_to_map] 월드 {name}: 충돌 모양 {len(shapes)}개 중 z={args.z} m 에 걸친 것 {n}개 → '
          f'{w}x{h} 칸 ({w * args.res:.2f} x {h * args.res:.2f} m), 원점 ({ox:.2f}, {oy:.2f}), '
          f'빈 칸 {int(free.sum())}, 장애물 칸 {int(occ.sum())}')
    print(f'[world_to_map] 저장: {args.out}.pgm, {args.out}.yaml')


if __name__ == '__main__':
    main()

"""ROS-free core of the B global planner comparison (NavFn A* vs Dijkstra).

Loads scenarios and planner configs, versions them, runs trials through an injected ``plan_fn`` and writes
ExperimentResult-style CSV/JSON. ``run_compare.py`` binds ``plan_fn`` to Nav2 ComputePathToPose; the tests bind
it to a fake planner (MOCK).

Path blocking (R1-D): a pair may list ``obstacles`` (boxes). Its runs are then measured twice, on the scenario map
and on a copy of the map with the boxes drawn in as occupied cells (a "map variant" named after the pair).
"""

import csv
import hashlib
import json
import math
import os
import statistics
from dataclasses import dataclass, field
from datetime import datetime, timezone

import yaml

RUN_MODES = ('MOCK', 'SIM', 'REAL')
EXPECTS = ('success', 'failure')

# docs/INTERFACES.md §4.7 ExperimentResult fields first (units: m, ms), then planner-bench extras.
# Driving fields (actual_distance, execution_time, goal_success, mission_success, controller_id) stay empty:
# this bench only plans, it never drives.
EXPERIMENT_RESULT_FIELDS = [
    'experiment_id', 'scenario_id', 'run_mode', 'algorithm', 'config_version', 'map_version', 'planner_id',
    'controller_id', 'planned_distance', 'actual_distance', 'planning_time', 'execution_time', 'goal_success',
    'mission_success', 'failure_reason']
# Columns added later go at the end so existing column positions never move (C/E may read these files).
# map_variant: '' on the scenario map, the pair id on the map with that pair's obstacles drawn in.
# obstacle_hits: path poses inside one of the pair's obstacles (only for pairs with obstacles and a returned path).
# min_clearance / mean_clearance: m from the path poses (robot centre) to the nearest occupied map cell.
# turn_sum_deg: sum of |heading change| along the path resampled every 0.10 m (0 = straight, ~90 per corner).
EXTRA_FIELDS = [
    'scenario_set', 'scenario_version', 'pair_id', 'direction', 'repetition', 'order', 'warmup', 'plan_success',
    'expected', 'expectation_met', 'error_code', 'error_name', 'error_msg', 'roundtrip_time', 'num_poses',
    'path_end_error', 'start_x', 'start_y', 'start_yaw_deg', 'goal_x', 'goal_y', 'goal_yaw_deg', 'stamp',
    'map_variant', 'obstacle_hits', 'min_clearance', 'mean_clearance', 'turn_sum_deg']
TRIAL_FIELDS = EXPERIMENT_RESULT_FIELDS + EXTRA_FIELDS
SUMMARY_FIELDS = [
    'scenario_id', 'pair_id', 'direction', 'expected', 'planner_id', 'algorithm', 'trials', 'plan_successes',
    'success_rate', 'expectation_met', 'planned_distance_mean', 'planned_distance_std', 'planned_distance_min',
    'planned_distance_max', 'planning_time_median', 'planning_time_max', 'roundtrip_time_median',
    'failure_reasons', 'map_variant', 'obstacle_crossings', 'min_clearance_min', 'mean_clearance_mean',
    'turn_sum_deg_mean']


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw_deg: float


@dataclass(frozen=True)
class Box:
    """Obstacle box: centre (m), size along its own x/y axes (m), rotation (deg) in the scenario frame."""
    cx: float
    cy: float
    size_x: float
    size_y: float
    yaw_deg: float = 0.0

    def contains(self, x, y, margin=0.0):
        yaw = math.radians(self.yaw_deg)
        dx, dy = x - self.cx, y - self.cy
        lx = math.cos(yaw) * dx + math.sin(yaw) * dy
        ly = -math.sin(yaw) * dx + math.cos(yaw) * dy
        return abs(lx) <= self.size_x / 2 + margin and abs(ly) <= self.size_y / 2 + margin

    def as_list(self):
        return [self.cx, self.cy, self.size_x, self.size_y, self.yaw_deg]


@dataclass(frozen=True)
class PlanRun:
    """One directed start -> goal request of a scenario pair.

    ``obstacles`` are the pair's boxes (kept on both map runs so the plain-map path can be checked against them);
    ``map_variant`` is '' on the scenario map and the pair id on the map with the boxes drawn in.
    """
    run_id: str
    pair_id: str
    name: str
    direction: str
    start: Pose2D
    goal: Pose2D
    expect: str
    obstacles: tuple = ()
    map_variant: str = ''


@dataclass(frozen=True)
class ScenarioSet:
    name: str
    path: str
    map_name: str
    map_yaml: str
    frame_id: str
    runs: tuple

    def map_variants(self):
        """{variant (pair id): obstacles} for every pair that is measured again with its obstacles drawn in."""
        return {r.map_variant: r.obstacles for r in self.runs if r.map_variant}


@dataclass(frozen=True)
class PlannerConfig:
    planner_id: str
    algorithm: str
    params: dict


@dataclass
class PlanOutcome:
    """What a planner call returned. ``poses`` are (x, y) in the scenario frame."""
    success: bool
    poses: list = field(default_factory=list)
    planning_time_ms: float | None = None
    roundtrip_ms: float | None = None
    error_code: int | None = None
    error_name: str = ''
    failure_reason: str = ''
    error_msg: str = ''


def _load_yaml(path):
    with open(path, encoding='utf-8') as f:
        return yaml.safe_load(f)


def _pose(name, value):
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError(f'point {name!r}: expected [x, y, yaw_deg]')
    try:
        x, y, yaw = (float(v) for v in value)
    except (TypeError, ValueError):
        raise ValueError(f'point {name!r}: non-numeric value {value!r}') from None
    if not all(math.isfinite(v) for v in (x, y, yaw)):
        raise ValueError(f'point {name!r}: non-finite value {value!r}')
    return Pose2D(x, y, yaw)


def _box(pid, value):
    if not isinstance(value, (list, tuple)) or len(value) not in (4, 5):
        raise ValueError(f'{pid}: obstacle must be [cx, cy, size_x, size_y] or [cx, cy, size_x, size_y, yaw_deg]')
    try:
        nums = [float(v) for v in value]
    except (TypeError, ValueError):
        raise ValueError(f'{pid}: non-numeric obstacle {value!r}') from None
    if not all(math.isfinite(v) for v in nums) or nums[2] <= 0 or nums[3] <= 0:
        raise ValueError(f'{pid}: obstacle {value!r} needs finite values and a positive size')
    return Box(*nums)


def load_scenarios(path):
    """Read a scenario file into fixed, directed start/goal runs (A->B and B->A are separate runs).

    A pair with ``obstacles`` also gets ``<id>:<direction>:blocked`` runs, planned on the map with the boxes drawn
    in and checked against ``expect_blocked``.
    """
    doc = _load_yaml(path)
    try:
        name, map_doc, points, pairs = doc['scenario_set'], doc['map'], doc['points'], doc['pairs']
        map_name, map_rel = map_doc['name'], map_doc['yaml']
    except (TypeError, KeyError) as e:
        raise ValueError(f'{path}: missing key {e}') from None
    poses = {k: _pose(k, v) for k, v in points.items()}
    runs, seen = [], set()
    for pair in pairs:
        pid = str(pair.get('id', '')).strip()
        if not pid or pid in seen:
            raise ValueError(f'missing or duplicate pair id {pid!r}')
        seen.add(pid)
        if pair.get('expect') not in EXPECTS:
            raise ValueError(f'{pid}: expect must be one of {EXPECTS}')
        a, b = pair.get('from'), pair.get('to')
        for point in (a, b):
            if point not in poses:
                raise ValueError(f'{pid}: unknown point {point!r}')
        if a == b:
            raise ValueError(f'{pid}: start and goal are the same point')
        label = str(pair.get('name', ''))
        obstacles = tuple(_box(pid, v) for v in pair.get('obstacles') or ())
        expect_blocked = pair.get('expect_blocked')
        if obstacles and expect_blocked not in EXPECTS:
            raise ValueError(f'{pid}: a pair with obstacles needs expect_blocked, one of {EXPECTS}')
        if expect_blocked is not None and not obstacles:
            raise ValueError(f'{pid}: expect_blocked is only valid with obstacles')
        for point in (a, b):
            if any(box.contains(poses[point].x, poses[point].y) for box in obstacles):
                raise ValueError(f'{pid}: an obstacle covers point {point!r}')
        directions = [('fwd', poses[a], poses[b])]
        if pair.get('reverse', False):
            directions.append(('rev', poses[b], poses[a]))
        for d, start, goal in directions:
            runs.append(PlanRun(f'{pid}:{d}', pid, label, d, start, goal, pair['expect'], obstacles))
        if obstacles:
            for d, start, goal in directions:
                runs.append(PlanRun(f'{pid}:{d}:blocked', pid, label, d, start, goal, expect_blocked, obstacles, pid))
    map_yaml = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(path)), map_rel))
    return ScenarioSet(str(name), os.path.abspath(path), str(map_name), map_yaml,
                       str(map_doc.get('frame_id', 'map')), tuple(runs))


def load_planner_configs(path):
    """NavFn planner plugins from a planner_server params file, in planner_plugins order."""
    doc = _load_yaml(path)
    try:
        params = doc['planner_server']['ros__parameters']
        ids = params['planner_plugins']
    except (TypeError, KeyError) as e:
        raise ValueError(f'{path}: missing planner_server parameter {e}') from None
    configs = []
    for pid in ids:
        plugin = params.get(pid)
        if not isinstance(plugin, dict) or not str(plugin.get('plugin', '')).startswith('nav2_navfn_planner'):
            raise ValueError(f'{pid}: not a NavFn planner plugin')
        if not isinstance(plugin.get('use_astar'), bool):
            raise ValueError(f'{pid}: use_astar must be true or false')
        configs.append(PlannerConfig(pid, 'astar' if plugin['use_astar'] else 'dijkstra', dict(plugin)))
    return configs


def config_differences(configs):
    keys = set().union(*(c.params for c in configs))
    return sorted(k for k in keys if len({repr(c.params.get(k)) for c in configs}) > 1)


def check_comparison_pair(configs):
    """The comparison is valid only for one A* and one Dijkstra NavFn that differ in nothing but use_astar."""
    if sorted(c.algorithm for c in configs) != ['astar', 'dijkstra']:
        raise ValueError('need exactly one NavFn with use_astar: true and one with use_astar: false')
    diff = config_differences(configs)
    if diff != ['use_astar']:
        raise ValueError(f'planner configs must differ only in use_astar, they differ in {diff}')


def _digest(paths_and_kinds):
    h = hashlib.sha256()
    for path, text in paths_and_kinds:
        with open(path, 'rb') as f:
            data = f.read()
        h.update(data.replace(b'\r\n', b'\n') if text else data)    # same hash on CRLF and LF checkouts
    return h.hexdigest()[:12]


def map_image_path(map_yaml):
    image = _load_yaml(map_yaml)['image']
    return image if os.path.isabs(image) else os.path.join(os.path.dirname(map_yaml), image)


def config_version(params_path):
    stem = os.path.splitext(os.path.basename(params_path))[0]
    return f'{stem}@{_digest([(params_path, True)])}'


def map_version(map_name, map_yaml):
    return f'{map_name}@{_digest([(map_yaml, True), (map_image_path(map_yaml), False)])}'


def scenario_version(scenario_path):
    stem = os.path.splitext(os.path.basename(scenario_path))[0]
    return f'{stem}@{_digest([(scenario_path, True)])}'


def _pgm_header(data, count):
    """First ``count`` header tokens of a PGM file and the offset just after the last one."""
    tokens, i = [], 0
    while len(tokens) < count and i < len(data):
        if data[i:i + 1].isspace():
            i += 1
        elif data[i:i + 1] == b'#':
            while i < len(data) and data[i:i + 1] != b'\n':
                i += 1
        else:
            j = i
            while j < len(data) and not data[j:j + 1].isspace():
                j += 1
            tokens.append(data[i:j])
            i = j
    return tokens, i


def read_pgm_size(path):
    with open(path, 'rb') as f:
        tokens, _ = _pgm_header(f.read(1024), 3)
    if len(tokens) < 3 or tokens[0] not in (b'P2', b'P5'):
        raise ValueError(f'{path}: not a PGM image')
    return int(tokens[1]), int(tokens[2])


def read_pgm_p5(path):
    """(width, height, maxval, pixels) of a binary 8-bit PGM. Pixel rows run from the top of the image."""
    with open(path, 'rb') as f:
        data = f.read()
    tokens, end = _pgm_header(data, 4)
    if len(tokens) < 4 or tokens[0] != b'P5' or not 0 < int(tokens[3]) < 256:
        raise ValueError(f'{path}: not a binary 8-bit PGM (P5) image')
    width, height, maxval = int(tokens[1]), int(tokens[2]), int(tokens[3])
    pixels = data[end + 1:end + 1 + width * height]     # exactly one whitespace byte follows maxval
    if len(pixels) != width * height:
        raise ValueError(f'{path}: truncated PGM raster')
    return width, height, maxval, bytearray(pixels)


def box_cells(box, width, height, resolution, origin):
    """Grid cells (col, row counted from the bottom, as in OccupancyGrid) whose centre lies in the box.

    Half a cell of tolerance so thin boxes always cover at least one cell row (same rule as legacy_r1).
    """
    ox, oy = float(origin[0]), float(origin[1])
    reach = math.hypot(box.size_x, box.size_y) / 2 + resolution
    i0, i1 = max(0, int((box.cx - reach - ox) / resolution)), min(width - 1, int((box.cx + reach - ox) / resolution))
    j0, j1 = max(0, int((box.cy - reach - oy) / resolution)), min(height - 1, int((box.cy + reach - oy) / resolution))
    return [(i, j) for j in range(j0, j1 + 1) for i in range(i0, i1 + 1)
            if box.contains(ox + (i + 0.5) * resolution, oy + (j + 0.5) * resolution, resolution / 2)]


def write_blocked_map(map_yaml, boxes, out_dir, name):
    """Copy of the scenario map with ``boxes`` drawn in as occupied cells: ``<out_dir>/<name>.yaml`` + ``.pgm``.

    The output depends only on the inputs (no time stamps), so its map_version is reproducible.
    """
    doc = _load_yaml(map_yaml)
    image = map_image_path(map_yaml)
    width, height, maxval, pixels = read_pgm_p5(image)
    occupied = maxval if int(doc.get('negate', 0)) else 0
    for box in boxes:
        cells = box_cells(box, width, height, float(doc['resolution']), doc['origin'][:2])
        if not cells:
            raise ValueError(f'obstacle {box.as_list()} does not cover any cell of {map_yaml}')
        for i, j in cells:
            pixels[(height - 1 - j) * width + i] = occupied
    os.makedirs(out_dir, exist_ok=True)
    header = (f'P5\n# bench_core.write_blocked_map: {os.path.basename(image)} + {len(boxes)} obstacles\n'
              f'{width} {height}\n{maxval}\n')
    with open(os.path.join(out_dir, f'{name}.pgm'), 'wb') as f:
        f.write(header.encode('ascii') + bytes(pixels))
    out_doc = dict(doc, image=f'{name}.pgm')
    out_yaml = os.path.join(out_dir, f'{name}.yaml')
    with open(out_yaml, 'w', encoding='utf-8', newline='\n') as f:
        yaml.safe_dump(out_doc, f, sort_keys=True, default_flow_style=None)
    return out_yaml


def obstacle_hits(points, boxes):
    """Number of path points (robot centre) inside one of the boxes."""
    return sum(1 for x, y in points if any(b.contains(x, y) for b in boxes))


_FAR = 1e20


def _edt_1d(f):
    """Squared distance transform of one row/column (Felzenszwalb & Huttenlocher), in cells squared."""
    n = len(f)
    d, v, z = [0.0] * n, [0] * n, [0.0] * (n + 1)
    k = 0
    z[0], z[1] = -_FAR, _FAR
    for q in range(1, n):
        while True:
            p = v[k]
            s = ((f[q] + q * q) - (f[p] + p * p)) / (2.0 * (q - p))
            if s > z[k]:
                break
            k -= 1
        k += 1
        v[k], z[k], z[k + 1] = q, s, _FAR
    k = 0
    for q in range(n):
        while z[k + 1] < q:
            k += 1
        d[q] = (q - v[k]) ** 2 + f[v[k]]
    return d


@dataclass(frozen=True)
class ClearanceField:
    """Distance (m) from every map cell centre to the nearest occupied cell centre. Rows count from the bottom."""
    width: int
    height: int
    resolution: float
    origin: tuple
    dist: tuple

    def at(self, x, y):
        i = math.floor((x - self.origin[0]) / self.resolution)
        j = math.floor((y - self.origin[1]) / self.resolution)
        if not (0 <= i < self.width and 0 <= j < self.height):
            return None
        return self.dist[j * self.width + i]


def clearance_field(map_yaml):
    """ClearanceField of a map file. Occupied = cells map_server marks occupied (occupancy > occupied_thresh).

    Unknown cells are not obstacles (same rule as legacy_r1). Returns None for a map without occupied cells.
    The value is cell-based, so it is accurate to about one cell (the map resolution).
    """
    doc = _load_yaml(map_yaml)
    width, height, maxval, pixels = read_pgm_p5(map_image_path(map_yaml))
    negate, thresh = int(doc.get('negate', 0)), float(doc.get('occupied_thresh', 0.65))
    grid = []
    for j in range(height):                       # j = 0 is the bottom row = last image row
        row = pixels[(height - 1 - j) * width:(height - j) * width]
        grid.append([0.0 if (v if negate else maxval - v) / maxval > thresh else _FAR for v in row])
    if not any(v == 0.0 for row in grid for v in row):
        return None
    cols = [_edt_1d([grid[j][i] for j in range(height)]) for i in range(width)]
    dist = []
    for j in range(height):
        dist.extend(math.sqrt(d) * float(doc['resolution']) for d in _edt_1d([cols[i][j] for i in range(width)]))
    return ClearanceField(width, height, float(doc['resolution']), (float(doc['origin'][0]), float(doc['origin'][1])),
                          tuple(dist))


def path_clearance(field, points):
    """(min, mean) clearance in m over the path points inside the map; (None, None) without a field or points."""
    values = [v for v in (field.at(x, y) for x, y in points) if v is not None] if field is not None else []
    return (min(values), sum(values) / len(values)) if values else (None, None)


def turn_total(points, step=0.10):
    """Sum of |heading change| in degrees along the path resampled every ``step`` m (same rule as legacy_r1)."""
    if len(points) < 3:
        return 0.0
    keep = [points[0]]
    for p in points[1:]:
        if math.dist(p, keep[-1]) >= step:
            keep.append(p)
    headings = [math.atan2(b[1] - a[1], b[0] - a[0]) for a, b in zip(keep, keep[1:])]
    total = sum(abs((b - a + math.pi) % (2 * math.pi) - math.pi) for a, b in zip(headings, headings[1:]))
    return math.degrees(total)


class PathStore:
    """Keeps the path of the first measured successful trial of every (run, planner) and counts identical ones."""

    def __init__(self):
        self.items = {}

    def add(self, row, poses):
        if row['warmup'] or not row['plan_success']:
            return
        rounded = [[round(x, 4), round(y, 4)] for x, y in poses]
        digest = hashlib.sha256(json.dumps(rounded).encode()).hexdigest()[:12]
        key = (row['scenario_id'], row['planner_id'])
        item = self.items.get(key)
        if item is None:
            self.items[key] = {
                'scenario_id': row['scenario_id'], 'planner_id': row['planner_id'], 'algorithm': row['algorithm'],
                'map_variant': row['map_variant'], 'repetition': row['repetition'],
                'planned_distance': row['planned_distance'], 'path_digest': digest, 'successful_trials': 1,
                'identical_trials': 1, 'poses': rounded}
        else:
            item['successful_trials'] += 1
            item['identical_trials'] += int(digest == item['path_digest'])

    def as_list(self):
        return list(self.items.values())


def map_info(map_yaml):
    """Expected map metadata, compared with what map_server actually publishes."""
    doc = _load_yaml(map_yaml)
    width, height = read_pgm_size(map_image_path(map_yaml))
    return {'width': width, 'height': height, 'resolution': float(doc['resolution']),
            'origin': [float(v) for v in doc['origin'][:2]]}


def path_length(points):
    return sum(math.dist(a, b) for a, b in zip(points, points[1:]))


def end_error(points, goal):
    return math.dist(points[-1], (goal.x, goal.y)) if points else None


def yaw_quaternion(yaw_deg):
    half = math.radians(yaw_deg) / 2.0
    return 0.0, 0.0, math.sin(half), math.cos(half)


def error_table(result_cls):
    """{code: NAME} from the integer constants of a ComputePathToPose.Result class."""
    return {v: k for k, v in vars(result_cls).items()
            if k.isupper() and isinstance(v, int) and not isinstance(v, bool)}


def utc_stamp(now=None):
    return (now or datetime.now(timezone.utc)).isoformat(timespec='milliseconds').replace('+00:00', 'Z')


def experiment_id(started, run_mode, scenario_set):
    return f'{started:%Y%m%dT%H%M%SZ}_{run_mode}_{scenario_set}'


def _round(value, digits):
    return '' if value is None else round(value, digits)


def _call(plan_fn, planner_id, start, goal):
    try:
        return plan_fn(planner_id, start, goal)
    except Exception as e:   # keep the run going: a client-side error is a recorded failure, not a crash
        return PlanOutcome(False, failure_reason=f'CLIENT_ERROR:{type(e).__name__}', error_msg=str(e))


def run_trials(scenarios, planners, plan_fn, *, repeat, warmup, run_mode, experiment_id, config_version,
               map_version, scenario_version='', clock=utc_stamp, set_map=None, variant_map_versions=None,
               clearance_fields=None, path_store=None):
    """Run every scenario run with every planner, ``warmup`` + ``repeat`` times.

    Both planners get the identical start/goal back to back; their order alternates per repetition so neither
    always runs first. Warmup rows are kept (warmup=True) and excluded from summaries.

    With map variants (pairs with obstacles), the runs on the scenario map come first, then the runs of each
    variant. ``set_map(variant)`` ('' = scenario map) is called before each of these groups and must raise when it
    cannot confirm the switch, so no trial is recorded on the wrong map. ``variant_map_versions`` gives the
    map_version of every variant.

    ``clearance_fields`` ({variant: ClearanceField}, '' = scenario map) fills the clearance columns; without it
    they stay empty. ``path_store`` (PathStore) collects one path per (run, planner) for plotting.
    """
    if run_mode not in RUN_MODES:
        raise ValueError(f'run_mode must be one of {RUN_MODES}')
    if repeat < 1 or warmup < 0:
        raise ValueError('repeat must be >= 1 and warmup >= 0')
    variants = scenarios.map_variants()
    versions = {'': map_version, **(variant_map_versions or {})}
    if variants and set_map is None:
        raise ValueError('the scenario has map variants (obstacles) but no set_map was given')
    missing = [v for v in variants if v not in versions]
    if missing:
        raise ValueError(f'no map_version for map variants {missing}')
    groups = [('', [r for r in scenarios.runs if not r.map_variant])]
    groups += [(v, [r for r in scenarios.runs if r.map_variant == v]) for v in variants]
    rows = []
    for variant, runs in groups:
        if not runs:
            continue
        if variants:
            set_map(variant)
        for k in range(warmup + repeat):
            ordered = list(planners) if k % 2 == 0 else list(reversed(planners))
            for run in runs:
                for order, cfg in enumerate(ordered, 1):
                    out = _call(plan_fn, cfg.planner_id, run.start, run.goal)
                    rows.append(_trial_row(
                        run, cfg, order, k, warmup, out, experiment_id=experiment_id, run_mode=run_mode,
                        config_version=config_version, map_version=versions[variant], scenario_set=scenarios.name,
                        scenario_version=scenario_version, stamp=clock(),
                        field=(clearance_fields or {}).get(variant)))
                    if path_store is not None:
                        path_store.add(rows[-1], out.poses)
    return rows


def _trial_row(run, cfg, order, k, warmup, out, *, experiment_id, run_mode, config_version, map_version,
               scenario_set, scenario_version, stamp, field=None):
    ok = bool(out.success and out.poses)
    min_clear, mean_clear = path_clearance(field, out.poses) if ok else (None, None)
    if ok:
        reason = ''
    elif out.success:
        reason = 'EMPTY_PATH'
    else:
        reason = out.failure_reason or out.error_name or 'UNKNOWN'
    return {
        'experiment_id': experiment_id, 'scenario_id': run.run_id, 'run_mode': run_mode,
        'algorithm': cfg.algorithm, 'config_version': config_version, 'map_version': map_version,
        'planner_id': cfg.planner_id, 'controller_id': '',
        'planned_distance': _round(path_length(out.poses), 4) if ok else '',
        'actual_distance': '', 'planning_time': _round(out.planning_time_ms, 3),
        'execution_time': '', 'goal_success': '', 'mission_success': '', 'failure_reason': reason,
        'scenario_set': scenario_set, 'scenario_version': scenario_version, 'pair_id': run.pair_id,
        'direction': run.direction, 'map_variant': run.map_variant, 'repetition': k - warmup + 1, 'order': order,
        'warmup': k < warmup, 'plan_success': ok, 'expected': run.expect,
        'expectation_met': ok == (run.expect == 'success'),
        'error_code': '' if out.error_code is None else out.error_code, 'error_name': out.error_name,
        'error_msg': out.error_msg, 'roundtrip_time': _round(out.roundtrip_ms, 3),
        'num_poses': len(out.poses), 'path_end_error': _round(end_error(out.poses, run.goal), 4),
        'obstacle_hits': obstacle_hits(out.poses, run.obstacles) if ok and run.obstacles else '',
        'min_clearance': _round(min_clear, 4), 'mean_clearance': _round(mean_clear, 4),
        'turn_sum_deg': round(turn_total(out.poses), 1) if ok else '',
        'start_x': run.start.x, 'start_y': run.start.y, 'start_yaw_deg': run.start.yaw_deg,
        'goal_x': run.goal.x, 'goal_y': run.goal.y, 'goal_yaw_deg': run.goal.yaw_deg, 'stamp': stamp}


def _stats(values, fn):
    return round(fn(values), 4) if values else ''


def summarize(rows):
    """Per (scenario run, planner) statistics over non-warmup trials, in first-seen order."""
    groups = {}
    for r in rows:
        if not r['warmup']:
            groups.setdefault((r['scenario_id'], r['planner_id']), []).append(r)
    summary = []
    for (scenario_id, planner_id), rs in groups.items():
        dist = [r['planned_distance'] for r in rs if r['plan_success']]
        ptime = [r['planning_time'] for r in rs if r['planning_time'] != '']
        rtime = [r['roundtrip_time'] for r in rs if r['roundtrip_time'] != '']
        ok = sum(1 for r in rs if r['plan_success'])
        hits = [r['obstacle_hits'] for r in rs if r['plan_success'] and r['obstacle_hits'] != '']
        summary.append({
            'scenario_id': scenario_id, 'pair_id': rs[0]['pair_id'], 'direction': rs[0]['direction'],
            'map_variant': rs[0]['map_variant'], 'expected': rs[0]['expected'], 'planner_id': planner_id,
            'algorithm': rs[0]['algorithm'], 'trials': len(rs), 'plan_successes': ok,
            'success_rate': round(100.0 * ok / len(rs), 1),
            'expectation_met': sum(1 for r in rs if r['expectation_met']),
            'obstacle_crossings': sum(1 for h in hits if h > 0) if hits else '',
            'min_clearance_min': _stats([r['min_clearance'] for r in rs if r['min_clearance'] != ''], min),
            'mean_clearance_mean': _stats([r['mean_clearance'] for r in rs if r['mean_clearance'] != ''],
                                          statistics.fmean),
            'turn_sum_deg_mean': _stats([r['turn_sum_deg'] for r in rs if r['turn_sum_deg'] != ''],
                                        statistics.fmean),
            'planned_distance_mean': _stats(dist, statistics.fmean),
            'planned_distance_std': _stats(dist, statistics.pstdev),
            'planned_distance_min': _stats(dist, min), 'planned_distance_max': _stats(dist, max),
            'planning_time_median': _stats(ptime, statistics.median), 'planning_time_max': _stats(ptime, max),
            'roundtrip_time_median': _stats(rtime, statistics.median),
            'failure_reasons': ';'.join(sorted({r['failure_reason'] for r in rs if r['failure_reason']}))})
    return summary


def _cell(value):
    return '-' if value == '' else str(value)


def render_summary_md(summary, meta):
    lines = [
        f"# {meta['experiment_id']}", '',
        f"- run_mode: **{meta['run_mode']}** ({meta.get('note') or '-'})",
        f"- config_version: `{meta['config_version']}` / map_version: `{meta['map_version']}` / "
        f"scenario_version: `{meta['scenario_version']}`",
        f"- repeat {meta['repeat']} (+ warmup {meta['warmup']}, excluded) / ROS {meta['env'].get('ros_distro')}"
        f" / nav2_planner {meta['env'].get('nav2_planner')}",
        '- planned_distance 는 계획 경로 길이(m)이며 실제 주행 거리가 아니다. planning_time 은 planner_server 가 보고한 값(ms).',
        '- 장애물 거리는 경로 점(로봇 중심)에서 가장 가까운 점유 칸까지의 거리(m, 칸 단위 정확도)다. '
        '꺾인 각도 합은 경로를 0.10 m 간격으로 다시 찍어 더한 방향 변화(도)다.']
    for variant, info in (meta.get('map_variants') or {}).items():
        lines.append(f"- 차단 지도 `{variant}` (`:blocked` 실행): `{info['map_version']}`, 상자 {info['obstacles']}")
    lines += [
        '',
        '| 실행 | 기대 | 플래너 | 성공 | 기대 일치 | 상자 자리 통과 | 경로 길이 평균 (m) | 표준편차 | 최소 장애물 거리 (m) '
        '| 평균 장애물 거리 (m) | 꺾인 각도 합 (도) | 계획 시간 중앙값 (ms) | 최대 | 왕복 중앙값 (ms) | 실패 사유 |',
        '| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |']
    for s in summary:
        crossings = '-' if s['obstacle_crossings'] == '' else f"{s['obstacle_crossings']}/{s['plan_successes']}"
        lines.append(
            f"| {s['scenario_id']} | {s['expected']} | {s['planner_id']} ({s['algorithm']}) "
            f"| {s['plan_successes']}/{s['trials']} | {s['expectation_met']}/{s['trials']} | {crossings} "
            f"| {_cell(s['planned_distance_mean'])} | {_cell(s['planned_distance_std'])} "
            f"| {_cell(s['min_clearance_min'])} | {_cell(s['mean_clearance_mean'])} | {_cell(s['turn_sum_deg_mean'])} "
            f"| {_cell(s['planning_time_median'])} | {_cell(s['planning_time_max'])} "
            f"| {_cell(s['roundtrip_time_median'])} | {s['failure_reasons'] or '-'} |")
    return '\n'.join(lines) + '\n'


def _write_csv(path, fields, rows):
    with open(path, 'w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def write_results(out_dir, rows, summary, meta, paths=None):
    """Write trials.csv, summary.csv, summary.md, meta.json (and paths.json) into a new directory (no overwrite)."""
    os.makedirs(out_dir, exist_ok=False)
    _write_csv(os.path.join(out_dir, 'trials.csv'), TRIAL_FIELDS, rows)
    _write_csv(os.path.join(out_dir, 'summary.csv'), SUMMARY_FIELDS, summary)
    with open(os.path.join(out_dir, 'summary.md'), 'w', encoding='utf-8') as f:
        f.write(render_summary_md(summary, meta))
    with open(os.path.join(out_dir, 'meta.json'), 'w', encoding='utf-8') as f:
        json.dump(meta, f, ensure_ascii=False, indent=2)
        f.write('\n')
    if paths is not None:
        with open(os.path.join(out_dir, 'paths.json'), 'w', encoding='utf-8') as f:
            json.dump({'experiment_id': meta['experiment_id'], 'paths': paths}, f, ensure_ascii=False,
                      separators=(',', ':'))
            f.write('\n')
    return out_dir

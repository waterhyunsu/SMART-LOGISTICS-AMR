"""Research 1: A* vs Dijkstra - repeated comparison on the same map (planning only) and in Gazebo (driving).

The two algorithms are the same search in warehouse_robot/astar.py (GridPlanner) with the heuristic on/off, on the
same planning grid (map + plan_map.py processing + robot radius/cost), so only the heuristic differs.
Scenarios and maps: experiments/r1/scenarios.yaml. Results: experiments/r1/results/<time>_<mode>_<world>/.
Protocol and how to run: experiments/r1/README.md.

    # 1) planning only (no ROS needed, in the container: docker exec -it wsim bash, cd /app)
    python3 tools/r1_bench.py offline --world lecture_hall_real [--repeat 30] [--random 200] [--only R1-A,R1-B]

    # 2) Gazebo driving (after scripts/run.sh --robot sim --mode map --map <the world's map> is running)
    source scripts/env.sh
    python3 tools/r1_bench.py sim --world lecture_hall_real [--repeat 5] [--only R1-A]

    # 3) summary table + charts of one or more result folders (also written automatically at the end of 1/2)
    python3 tools/r1_bench.py report experiments/r1/results/<folder> [...]

Per trial one CSV row (trials.csv) with the team's common fields (experiment_id, scenario_id, algorithm,
config_version, planned_distance, actual_distance, planning_time, execution_time, goal_success, mission_success,
failure_reason) plus search statistics. Units: m, ms (planning_time, search_time), s (execution_time).
Order is counterbalanced: even repetitions run A* first, odd ones Dijkstra first.
"""
import argparse
import csv
import gc
import hashlib
import json
import math
import os
import platform
import subprocess
import sys
import threading
import time
from datetime import datetime

import numpy as np
import yaml

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, 'ros2_ws', 'src', 'warehouse_robot'))   # repo source first (same as symlink-install)

from clean_map import read_pgm, read_yaml  # noqa: E402  (tools/)
from warehouse_robot.astar import ALGORITHMS, GridPlanner, PlanningError  # noqa: E402
from warehouse_robot.plan_map import near_mask, prepare, with_added  # noqa: E402

SCENARIOS = os.path.join(REPO, 'experiments', 'r1', 'scenarios.yaml')
RESULTS = os.path.join(REPO, 'experiments', 'r1', 'results')
PARAMS = os.path.join(REPO, 'ros2_ws', 'src', 'warehouse_robot', 'config', 'params.yaml')
LABEL = {'astar': 'A*', 'dijkstra': 'Dijkstra'}
COLUMNS = [
    'experiment_id', 'mode', 'world', 'scenario_id', 'scenario_name', 'direction', 'pair', 'rep', 'order', 'algorithm',
    'config_version', 'start_x', 'start_y', 'goal_x', 'goal_y', 'straight_distance',
    'planning_time', 'search_time', 'expanded', 'pushed', 'planned_distance', 'raw_distance', 'path_cost', 'plan_success',
    'actual_distance', 'execution_time', 'execution_time_sim', 'n_plans', 'replan_search_time', 'final_error',
    'goal_success', 'mission_success', 'failure_reason', 'wall_time',
    'roundtrip_time', 'path_points', 'recoveries',   # Nav2 modes
    'battery_v',                                     # real robot: battery voltage at the start of the trial
    'planner_id',                                    # Nav2 planner plugin used for the trial
    'min_clearance', 'mean_clearance',               # obstacle distance along the plan (nav2plan) / driven track (nav2sim), m
    'turn_total',                                    # sum of heading changes along the plan, deg (smoothness)
    'inflation',                                     # global costmap inflation used: 'r=<radius>,s=<cost_scaling_factor>'
]


# ---------------------------------------------------------------------------------------------------- common
def load_scenarios(path, world):
    with open(path, encoding='utf-8') as f:
        data = yaml.safe_load(f)
    worlds = data.get('worlds', {})
    if world not in worlds:
        raise SystemExit(f'{path} 에 월드 "{world}" 가 없습니다 (있는 것: {", ".join(worlds)})')
    cfg = worlds[world]
    points = {k: tuple(float(v) for v in p) for k, p in cfg.get('points', {}).items()}
    for sc in cfg.get('scenarios', []):
        for key in ('from', 'to'):
            if sc[key] not in points:
                raise SystemExit(f'시나리오 {sc["id"]}: 지점 "{sc[key]}" 가 points 에 없습니다')
    return cfg, points


def resolve_map(spec):
    """'turtlebot3_navigation2' = the package's default map (turtlebot3_world), otherwise a path relative to the repo."""
    if spec == 'turtlebot3_navigation2':
        prefixes = os.environ.get('AMENT_PREFIX_PATH', '').split(os.pathsep) + ['/opt/ros/humble']
        for p in prefixes:
            cand = os.path.join(p, 'share', 'turtlebot3_navigation2', 'map', 'map.yaml')
            if p and os.path.isfile(cand):
                return cand
        raise SystemExit('turtlebot3_navigation2 의 map.yaml 을 찾지 못했습니다 (컨테이너 안에서 실행하세요)')
    path = spec if os.path.isabs(spec) else os.path.join(REPO, spec)
    if not os.path.isfile(path):
        raise SystemExit(f'맵 파일 없음: {path}')
    return path


def load_map(path):
    """map_server (trinary) equivalent: returns (grid int16 row 0 = bottom, -1/0/100, resolution, origin_x, origin_y)."""
    conf = read_yaml(path)
    image = conf['image']
    if not os.path.isabs(image):
        image = os.path.join(os.path.dirname(path), image)
    img = read_pgm(image).astype(float)
    res = float(conf['resolution'])
    origin = [float(v) for v in conf['origin'].strip('[] ').split(',')]
    p = img / 255.0 if int(conf.get('negate', 0)) else (255.0 - img) / 255.0
    grid = np.full(img.shape, -1, dtype=np.int16)
    grid[p > float(conf.get('occupied_thresh', 0.65))] = 100
    grid[p < float(conf.get('free_thresh', 0.196))] = 0
    return np.flipud(grid).copy(), res, origin[0], origin[1], image


def load_params():
    with open(PARAMS, encoding='utf-8') as f:
        return yaml.safe_load(f)['astar_planner']['ros__parameters']


def sha1_file(path, n=6):
    with open(path, 'rb') as f:
        return hashlib.sha1(f.read()).hexdigest()[:n]


def git_version():
    """Short commit (+dirty if the code/scenarios/maps used by the experiment differ from it).
    safe.directory: in the container the bind-mounted repo belongs to another user. Only LF/binary paths are checked
    (on a Windows checkout other files are CRLF on disk and would always look modified to Linux git)."""
    git = ['git', '-c', 'safe.directory=*', '-c', 'core.fileMode=false', '-C', REPO]
    try:
        rev = subprocess.run(git + ['rev-parse', '--short', 'HEAD'], capture_output=True, text=True,
                             timeout=10).stdout.strip()
        dirty = subprocess.run(git + ['status', '--porcelain', '--untracked-files=no', '--', 'ros2_ws/src/*.py',
                                      'ros2_ws/src/*.yaml', 'tools/*.py', 'experiments/r1/*.yaml', 'experiments/r1/*.pgm'],
                               capture_output=True, text=True, timeout=30).stdout.strip()
        return (rev or 'nogit') + ('+dirty' if dirty else '')
    except (OSError, subprocess.SubprocessError):
        return 'nogit'


def config_version(prm, map_image):
    p = hashlib.sha1(json.dumps({k: v for k, v in sorted(prm.items()) if k != 'algorithm'}).encode()).hexdigest()[:6]
    return f'{git_version()}-p{p}-m{sha1_file(map_image)}'


def box_cells(shape, res, ox, oy, box):
    """Cells covered by box [cx, cy, size_x, size_y, yaw_deg] (half a cell of tolerance)."""
    cx, cy, sx, sy = box[:4]
    yaw = math.radians(box[4]) if len(box) > 4 else 0.0
    h, w = shape
    xs = ox + (np.arange(w) + 0.5) * res
    ys = oy + (np.arange(h) + 0.5) * res
    gx, gy = np.meshgrid(xs - cx, ys - cy)
    c, s = math.cos(-yaw), math.sin(-yaw)
    lx, ly = c * gx - s * gy, s * gx + c * gy
    return (np.abs(lx) <= sx / 2 + res / 2) & (np.abs(ly) <= sy / 2 + res / 2)


def planning_planner(grid, res, ox, oy, prm, start, boxes=()):
    """GridPlanner on the same grid astar_planner_node uses in map mode (plan_map.prepare + start clearing),
    with extra obstacle boxes marked like remembered LiDAR obstacles (value 100)."""
    base, cluster_added, _ = prepare(grid, res, prm['stripe_cells'], prm['cluster_inflate'], prm['cluster_max_size'],
                                     prm['occupied_threshold'])
    added = cluster_added if cluster_added is not None else np.zeros(grid.shape, dtype=bool)
    clear = prm['start_clear_radius']
    start_inside = bool(added.any() and (added & near_mask(grid.shape, res, ox, oy, start, clear)).any())
    data = with_added(base, added, res, ox, oy, start if start_inside else None, clear).copy()
    for b in boxes:
        data[box_cells(data.shape, res, ox, oy, b)] = 100
    h, w = data.shape
    return GridPlanner(data.ravel(), w, h, res, ox, oy, robot_radius=prm['robot_radius'],
                       inflation_radius=prm['inflation_radius'], cost_weight=prm['cost_weight'],
                       occupied_threshold=prm['occupied_threshold'], allow_unknown=False,
                       unknown_cost=prm.get('unknown_cost', 2.0))


def directions(sc, points):
    a, b = points[sc['from']], points[sc['to']]
    out = [('fwd', a, b)]
    if sc.get('reverse', True):
        out.append(('rev', b, a))
    return out


def new_out_dir(mode, world, out=None):
    d = out or os.path.join(RESULTS, f'{datetime.now():%Y%m%d-%H%M%S}_{mode}_{world}')
    os.makedirs(d, exist_ok=True)
    return d


class CsvLog:
    """trials.csv, flushed after every row (partial results survive Ctrl-C)."""

    def __init__(self, path):
        self.f = open(path, 'w', newline='', encoding='utf-8')
        self.w = csv.DictWriter(self.f, fieldnames=COLUMNS, extrasaction='ignore')
        self.w.writeheader()
        self.n = 0

    def write(self, row):
        self.w.writerow({k: ('' if row.get(k) is None else row.get(k)) for k in COLUMNS})
        self.f.flush()
        self.n += 1

    def close(self):
        self.f.close()


def r3(v):
    return None if v is None else round(float(v), 4)


# ---------------------------------------------------------------------------------------------------- offline
def timed_plan(planner, a, b, tol, algorithm):
    """One plan with gc paused (less timing jitter). Returns (points or None, stats, error message)."""
    gc.collect()
    gc.disable()
    try:
        pts = planner.plan(a, b, goal_tolerance=tol, algorithm=algorithm)
        err = None
    except PlanningError as e:
        pts, err = None, str(e)
    finally:
        gc.enable()
    return pts, dict(planner.last_stats or {}), err


def offline_row(base, alg, pts, s, err):
    return dict(base, algorithm=alg, planning_time=r3(s.get('plan_ms')), search_time=r3(s.get('search_ms')),
                expanded=s.get('expanded'), pushed=s.get('pushed'), planned_distance=r3(s.get('length')),
                raw_distance=r3(s.get('raw_length')), path_cost=r3(s.get('cost')), plan_success=int(pts is not None),
                failure_reason=err, wall_time=round(time.time(), 3))


def run_offline(args):
    cfg, points = load_scenarios(args.scenarios, args.world)
    map_path = resolve_map(cfg['map'])
    grid, res, ox, oy, image = load_map(map_path)
    prm = load_params()
    tol = prm['goal_tolerance']
    out = new_out_dir('offline', args.world, args.out)
    exp_id = f'R1-offline-{os.path.basename(out)}'
    cver = config_version(prm, image)
    log = CsvLog(os.path.join(out, 'trials.csv'))
    only = set(args.only.split(',')) if args.only else None
    vis = {}   # (scenario, direction) -> {alg: (closed, points, stats)}, start, goal, planner
    print(f'[r1] offline: 월드 {args.world}, 맵 {map_path} ({grid.shape[1]}x{grid.shape[0]}, {res} m), '
          f'반복 {args.repeat}, 설정 {cver}', flush=True)

    for sc in cfg.get('scenarios', []):
        if only and sc['id'] not in only:
            continue
        for dname, a, b in directions(sc, points):
            t0 = time.perf_counter()
            planner = planning_planner(grid, res, ox, oy, prm, a[:2], sc.get('obstacles') or ())
            grid_ms = (time.perf_counter() - t0) * 1000.0
            for alg in ALGORITHMS:   # warm-up (not recorded)
                timed_plan(planner, a[:2], b[:2], tol, alg)
            base = dict(experiment_id=exp_id, mode='offline', world=args.world, scenario_id=sc['id'],
                        scenario_name=sc.get('name', ''), direction=dname, config_version=cver,
                        start_x=a[0], start_y=a[1], goal_x=b[0], goal_y=b[1],
                        straight_distance=r3(math.hypot(b[0] - a[0], b[1] - a[1])))
            res_alg = {}
            for rep in range(args.repeat):
                order = ALGORITHMS if rep % 2 == 0 else ALGORITHMS[::-1]
                for i, alg in enumerate(order):
                    pts, s, err = timed_plan(planner, a[:2], b[:2], tol, alg)
                    log.write(offline_row(dict(base, pair=f'{sc["id"]}-{dname}-{rep}', rep=rep, order=i), alg, pts, s, err))
                    res_alg[alg] = (bytes(planner.last_closed) if planner.last_closed is not None else None, pts, s, err)
            vis[(sc['id'], dname)] = dict(res=res_alg, start=a, goal=b, planner=planner, name=sc.get('name', ''))
            sa, sd = res_alg['astar'][2], res_alg['dijkstra'][2]
            print(f'[r1] {sc["id"]} {dname}: 격자 준비 {grid_ms:.0f} ms | '
                  + ' | '.join(f'{LABEL[k]} {"성공" if v[1] is not None else "실패"} {v[2].get("search_ms", 0):.1f} ms '
                               f'확장 {v[2].get("expanded", 0)} 길이 {v[2].get("length") or 0:.2f} m'
                               for k, v in (('astar', res_alg['astar']), ('dijkstra', res_alg['dijkstra'])))
                  + (f' | 비용 차 {abs((sa.get("cost") or 0) - (sd.get("cost") or 0)):.4f}' if sa.get('cost') else ''),
                  flush=True)

    if args.random:
        run_random(args, grid, res, ox, oy, prm, tol, exp_id, cver, log)
    log.close()
    meta = dict(mode='offline', world=args.world, map=map_path, map_image=image, config_version=cver,
                repeat=args.repeat, random=args.random, seed=args.seed, params=prm, host=platform.node(),
                python=platform.python_version(), started=os.path.basename(out), rows=log.n)
    with open(os.path.join(out, 'meta.json'), 'w', encoding='utf-8') as f:
        json.dump(meta, f, ensure_ascii=False, indent=2)
    if not args.no_plots:
        for (sid, dname), v in vis.items():
            plot_paths(os.path.join(out, f'paths_{sid}_{dname}.png'), v, grid, res, ox, oy,
                       f'{args.world} {sid} ({dname})')   # ASCII only: the container has no Korean font
    print(f'[r1] 저장: {out} (trials.csv {log.n}행)', flush=True)
    report([out], no_plots=args.no_plots)


def run_random(args, grid, res, ox, oy, prm, tol, exp_id, cver, log):
    """Random start/goal pairs on free cells (seeded): a larger sample of the same comparison on this map."""
    rng = np.random.default_rng(args.seed)
    planner = planning_planner(grid, res, ox, oy, prm, (ox, oy))
    clear = prm['robot_radius'] + 0.1
    rr, cc = np.nonzero(~planner.lethal & (planner.dist >= clear))
    if len(rr) < 2:
        print('[r1] 무작위 쌍: 빈 칸이 부족합니다', flush=True)
        return
    done = tries = 0
    while done < args.random and tries < args.random * 50:
        tries += 1
        i, j = rng.integers(0, len(rr), 2)
        a = planner.cell_to_world(rr[i], cc[i])
        b = planner.cell_to_world(rr[j], cc[j])
        d = math.hypot(b[0] - a[0], b[1] - a[1])
        if d < args.min_dist:
            continue
        base = dict(experiment_id=exp_id, mode='offline', world=args.world, scenario_id='RND',
                    scenario_name='무작위 쌍', direction='fwd', config_version=cver, start_x=r3(a[0]), start_y=r3(a[1]),
                    goal_x=r3(b[0]), goal_y=r3(b[1]), straight_distance=r3(d))
        for rep in range(args.random_repeat):
            order = ALGORITHMS if (done + rep) % 2 == 0 else ALGORITHMS[::-1]
            for k, alg in enumerate(order):
                pts, s, err = timed_plan(planner, a, b, tol, alg)
                log.write(offline_row(dict(base, pair=f'RND-{done}-{rep}', rep=rep, order=k), alg, pts, s, err))
        done += 1
    print(f'[r1] 무작위 쌍 {done}개 (직선거리 >= {args.min_dist} m, 쌍마다 {args.random_repeat}회)', flush=True)


# ---------------------------------------------------------------------------------------------------- Gazebo
class SimRig:
    """ROS side of a Gazebo trial: planner parameter, obstacle memory, Gazebo models, ground-truth pose, plan stats."""

    def __init__(self, robot_entity, sim_time=True):
        import rclpy
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.parameter import Parameter
        from std_msgs.msg import String
        self.rclpy = rclpy
        rclpy.init()
        self.node = rclpy.create_node('r1_bench', parameter_overrides=[Parameter('use_sim_time', value=sim_time)])
        self.entity = robot_entity
        self.stats = []                 # (wall time received, dict)
        self.gt = []                    # (wall time, x, y) while tracking
        self.tracking = False
        self.lock = threading.Lock()
        self.node.create_subscription(String, 'astar/plan_stats', self.on_stats, 50)
        from gazebo_msgs.srv import DeleteEntity, GetEntityState, SpawnEntity
        from rcl_interfaces.srv import SetParameters
        from std_srvs.srv import Empty
        self.srv = {
            'set_param': self.node.create_client(SetParameters, '/astar_planner/set_parameters'),
            'clear': self.node.create_client(Empty, '/astar/clear_obstacles'),
            'spawn': self.node.create_client(SpawnEntity, '/spawn_entity'),
            'delete': self.node.create_client(DeleteEntity, '/delete_entity'),
            'state': self.node.create_client(GetEntityState, '/get_entity_state'),
        }
        self.types = dict(SetParameters=SetParameters, Empty=Empty, SpawnEntity=SpawnEntity, DeleteEntity=DeleteEntity,
                          GetEntityState=GetEntityState)
        self.executor = MultiThreadedExecutor(num_threads=3)
        self.executor.add_node(self.node)
        self.running = True
        self.spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.spin_thread.start()
        self.poll_thread = threading.Thread(target=self.poll_gt, daemon=True)
        self.poll_thread.start()

    def on_stats(self, msg):
        try:
            d = json.loads(msg.data)
        except ValueError:
            return
        with self.lock:
            self.stats.append((time.time(), d))

    def call(self, name, req, timeout=5.0):
        cli = self.srv[name]
        if not cli.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f'서비스 {cli.srv_name} 가 없습니다')
        ev = threading.Event()
        fut = cli.call_async(req)
        fut.add_done_callback(lambda _: ev.set())
        if not ev.wait(timeout):
            raise RuntimeError(f'서비스 {cli.srv_name} 응답 없음')
        return fut.result()

    def set_algorithm(self, alg):
        from rcl_interfaces.msg import Parameter as PMsg
        from rcl_interfaces.msg import ParameterType, ParameterValue
        req = self.types['SetParameters'].Request()
        req.parameters = [PMsg(name='algorithm', value=ParameterValue(type=ParameterType.PARAMETER_STRING,
                                                                       string_value=alg))]
        r = self.call('set_param', req)
        if not r.results or not r.results[0].successful:
            raise RuntimeError(f'algorithm={alg} 설정 실패: {r.results[0].reason if r.results else "?"}')

    def clear_memory(self):
        self.call('clear', self.types['Empty'].Request())

    def spawn_box(self, name, box):
        cx, cy, sx, sy = box[:4]
        yaw = math.radians(box[4]) if len(box) > 4 else 0.0
        sdf = (f'<?xml version="1.0"?><sdf version="1.6"><model name="{name}"><static>true</static><link name="l">'
               f'<collision name="c"><pose>0 0 0.25 0 0 0</pose><geometry><box><size>{sx} {sy} 0.5</size></box>'
               f'</geometry></collision><visual name="v"><pose>0 0 0.25 0 0 0</pose><geometry><box>'
               f'<size>{sx} {sy} 0.5</size></box></geometry><material><ambient>0.85 0.35 0.1 1</ambient>'
               f'<diffuse>0.85 0.35 0.1 1</diffuse></material></visual></link></model></sdf>')
        req = self.types['SpawnEntity'].Request()
        req.name, req.xml = name, sdf
        req.initial_pose.position.x, req.initial_pose.position.y = float(cx), float(cy)
        req.initial_pose.orientation.z, req.initial_pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        r = self.call('spawn', req, timeout=10.0)
        if not r.success:
            raise RuntimeError(f'장애물 {name} 생성 실패: {r.status_message}')

    def delete(self, name):
        req = self.types['DeleteEntity'].Request()
        req.name = name
        try:
            self.call('delete', req, timeout=10.0)
        except RuntimeError as e:
            print(f'[r1] 경고: {e}', flush=True)

    def gt_pose(self):
        req = self.types['GetEntityState'].Request()
        req.name, req.reference_frame = self.entity, 'world'
        r = self.call('state', req, timeout=2.0)
        if not r.success:
            return None
        p, q = r.state.pose.position, r.state.pose.orientation
        return p.x, p.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    def poll_gt(self):
        while self.running:
            time.sleep(0.2)
            if not self.tracking or not self.running:
                continue
            try:
                p = self.gt_pose()
            except RuntimeError:
                continue
            if p is not None:
                with self.lock:
                    self.gt.append((time.time(), p[0], p[1]))

    def sim_now(self):
        return self.node.get_clock().now().nanoseconds * 1e-9

    def close(self):
        """Stop the pose poller before tearing down the node (a service call on a destroyed node segfaults at exit)."""
        self.running = False
        self.tracking = False
        self.poll_thread.join(timeout=3.0)
        self.executor.shutdown(timeout_sec=2.0)
        self.spin_thread.join(timeout=3.0)
        self.node.destroy_node()
        self.rclpy.try_shutdown()


class UiClient:
    """Sends goals the way the web UI does (/ws/ui/) and follows state/log/pose."""

    def __init__(self, url):
        import websocket
        self.ws_mod = websocket
        self.ws = websocket.create_connection(url, timeout=10.0)
        self.snapshot = json.loads(self.ws.recv())   # first message = snapshot (may take a moment)
        self.ws.settimeout(0.2)
        self.pose = self.snapshot.get('pose')
        robot = (self.snapshot.get('robot') or {})
        if robot.get('state') == 'OFFLINE':
            print('[r1] 경고: 로봇(브리지)이 서버에 연결되어 있지 않습니다 (OFFLINE)', flush=True)

    def send(self, **msg):
        self.ws.send(json.dumps(msg))

    def recv(self):
        try:
            m = json.loads(self.ws.recv())
        except self.ws_mod.WebSocketTimeoutException:
            return None
        if m.get('type') == 'pose' and 'x' in m:
            self.pose = m
        return m

    def drain(self, seconds=0.3):
        t = time.time()
        while time.time() - t < seconds:
            self.recv()


def clearance_field(grid, res, ox, oy, boxes=()):
    """Distance (m) from every cell to the nearest obstacle the robot can hit: map cells >= 65 plus the scenario boxes.
    Unknown cells are not counted (they are outside the walls). Returns (dist, res, ox, oy) for path_clearance."""
    from scipy.ndimage import distance_transform_edt
    occ = grid >= 65
    for b in boxes:
        occ = occ | box_cells(grid.shape, res, ox, oy, b)
    return distance_transform_edt(~occ) * res, res, ox, oy


def path_clearance(field, pts):
    """(min, mean) distance from the points (robot centre) to the nearest obstacle, m. (None, None) without points.
    Subtract the robot radius (0.22 m) for the free margin."""
    if field is None or not pts:
        return None, None
    dist, res, ox, oy = field
    h, w = dist.shape
    v = []
    for x, y in pts:
        r, c = int(math.floor((y - oy) / res)), int(math.floor((x - ox) / res))
        if 0 <= r < h and 0 <= c < w:
            v.append(float(dist[r, c]))
    return (min(v), sum(v) / len(v)) if v else (None, None)


def turn_total(pts, step=0.10):
    """Sum of |heading change| along a path resampled every `step` m, in degrees: 0 for a straight line, about 90
    per right-angle corner. Wiggles and zig-zags add up, so it compares how smooth planners' paths are."""
    if not pts or len(pts) < 3:
        return 0.0
    keep = [pts[0]]
    for x, y in pts[1:]:
        if math.hypot(x - keep[-1][0], y - keep[-1][1]) >= step:
            keep.append((x, y))
    hd = [math.atan2(y1 - y0, x1 - x0) for (x0, y0), (x1, y1) in zip(keep, keep[1:])]
    tot = 0.0
    for a, b in zip(hd, hd[1:]):
        tot += abs((b - a + math.pi) % (2 * math.pi) - math.pi)
    return math.degrees(tot)


def polyline_length(pts, min_step=0.01):
    """Path length of sampled positions, ignoring jitter below min_step."""
    total, last = 0.0, None
    for x, y in pts:
        if last is None:
            last = (x, y)
            continue
        d = math.hypot(x - last[0], y - last[1])
        if d >= min_step:
            total += d
            last = (x, y)
    return total


def drive(ui, rig, goal, timeout):
    """One goto_point. Returns dict with success, failure_reason, times, distances, plan stats during the drive."""
    ui.drain()
    with rig.lock:
        rig.gt.clear()
        mark = len(rig.stats)
    rig.tracking = True
    t0, s0 = time.time(), rig.sim_now()
    bridge_pts = []
    ui.send(action='goto_point', x=goal[0], y=goal[1], yaw=math.radians(goal[2]) if len(goal) > 2 else 0.0)
    state, moving, fail, done = None, False, None, False
    while time.time() - t0 < timeout:
        m = ui.recv()
        if m is None:
            continue
        typ = m.get('type')
        if typ == 'pose' and 'x' in m:
            bridge_pts.append((m['x'], m['y']))
        elif typ == 'error':
            fail, done = f'요청 거부: {m.get("message")}', True
            break
        elif typ == 'log':
            text = m.get('log', {}).get('message', '')
            if '이동 실패' in text:
                fail = text
        elif typ == 'state':
            state = m['robot']['state']
            msg = m['robot'].get('message') or ''
            if state == 'MOVING':
                moving = True
            if msg.startswith('직전 이동 실패'):
                fail = fail or msg
            if moving and state in ('IDLE', 'ERROR', 'OFFLINE'):
                done = True
                if state != 'IDLE':
                    fail = fail or f'상태 {state}: {msg}'
                break
    if not done:
        ui.send(action='estop')
        time.sleep(1.0)
        ui.send(action='reset_error')
        fail = f'시간 초과 ({timeout:.0f}초) → 비상정지'
    rig.tracking = False
    t1, s1 = time.time(), rig.sim_now()
    time.sleep(0.6)   # last plan_stats / pose
    with rig.lock:
        gt = [(x, y) for _, x, y in rig.gt]
        stats = [d for _, d in rig.stats[mark:]]
    try:
        final = rig.gt_pose()
    except RuntimeError:
        final = None
    if final is None and ui.pose:
        final = (ui.pose['x'], ui.pose['y'], ui.pose.get('yaw', 0.0))
    err = math.hypot(final[0] - goal[0], final[1] - goal[1]) if final else None
    return dict(success=fail is None, failure_reason=fail, execution_time=t1 - t0, execution_time_sim=s1 - s0,
                actual_distance=polyline_length(gt) if len(gt) > 2 else polyline_length(bridge_pts),
                final_error=err, stats=stats, final=final)


def reposition(ui, rig, point, timeout):
    """Bring the robot to the trial start (A*, not recorded) unless it is already there."""
    try:
        p = rig.gt_pose()
    except RuntimeError:
        p = None
    if p is not None:
        dyaw = abs((p[2] - math.radians(point[2] if len(point) > 2 else 0.0) + math.pi) % (2 * math.pi) - math.pi)
        if math.hypot(p[0] - point[0], p[1] - point[1]) < 0.25 and dyaw < math.radians(25):
            return True
    rig.set_algorithm('astar')
    rig.clear_memory()
    for attempt in range(2):
        r = drive(ui, rig, point, timeout)
        if r['success']:
            return True
        print(f'[r1]   출발점 이동 실패 ({r["failure_reason"]}), 재시도 {attempt + 1}', flush=True)
        time.sleep(3.0)
    return False


def run_sim(args):
    cfg, points = load_scenarios(args.scenarios, args.world)
    map_path = resolve_map(cfg['map'])
    _, _, _, _, image = load_map(map_path)
    mode_env = os.path.join(os.environ.get('WAREHOUSE_HOME', REPO), 'logs', 'run_mode.env')
    running_map = ''
    if os.path.isfile(mode_env):
        running_map = next((ln.split('=', 1)[1].strip() for ln in open(mode_env, encoding='utf-8')
                            if ln.startswith('WAREHOUSE_MAP=')), '')
    if os.path.realpath(running_map) != os.path.realpath(map_path):
        print(f'[r1] 경고: 실행 중인 맵 ({running_map or "모름"}) 이 시나리오 맵 ({map_path}) 과 다릅니다. '
              f'scripts/run.sh --robot sim --mode map --map {map_path} 로 실행했는지 확인하세요', flush=True)
        if not args.force:
            raise SystemExit('중단 (--force 로 무시)')
    prm = load_params()
    out = new_out_dir('sim', args.world, args.out)
    exp_id = f'R1-sim-{os.path.basename(out)}'
    cver = config_version(prm, image)
    rig = SimRig(os.environ.get('TURTLEBOT3_MODEL', 'waffle_pi'))
    ui = UiClient(args.server)
    log = CsvLog(os.path.join(out, 'trials.csv'))
    only = set(args.only.split(',')) if args.only else None
    spawned = []
    print(f'[r1] sim: 월드 {args.world}, 반복 {args.repeat}, 설정 {cver}', flush=True)
    scenarios = [sc for sc in cfg.get('scenarios', []) if sc.get('sim', True) and not (only and sc['id'] not in only)]
    try:
        # Repetition-major: if a long run is stopped early, every scenario still has the same number of repetitions
        for rep in range(args.repeat):
            for sc in scenarios:
                dirs = directions(sc, points)
                order = ALGORITHMS if rep % 2 == 0 else ALGORITHMS[::-1]
                for k, alg in enumerate(order):
                    for dname, a, b in dirs:
                        if not reposition(ui, rig, a, args.timeout):
                            raise RuntimeError(f'{sc["id"]}: 출발점 {a} 로 가지 못했습니다')
                        rig.set_algorithm(alg)
                        rig.clear_memory()
                        for i, box in enumerate(sc.get('obstacles') or ()):
                            name = f'r1_obs_{i}'
                            rig.spawn_box(name, box)
                            spawned.append(name)
                        time.sleep(args.settle)
                        r = drive(ui, rig, b, args.timeout)
                        for name in spawned:
                            rig.delete(name)
                        spawned.clear()
                        first = r['stats'][0] if r['stats'] else {}
                        row = dict(experiment_id=exp_id, mode='sim', world=args.world, scenario_id=sc['id'],
                                   scenario_name=sc.get('name', ''), direction=dname, pair=f'{sc["id"]}-{dname}-{rep}',
                                   rep=rep, order=k, algorithm=alg, config_version=cver, start_x=a[0], start_y=a[1],
                                   goal_x=b[0], goal_y=b[1], straight_distance=r3(math.hypot(b[0] - a[0], b[1] - a[1])),
                                   planning_time=r3(first.get('service_ms')), search_time=r3(first.get('search_ms')),
                                   expanded=first.get('expanded'), pushed=first.get('pushed'),
                                   planned_distance=r3(first.get('length')), raw_distance=r3(first.get('raw_length')),
                                   path_cost=r3(first.get('cost')), plan_success=int(bool(first.get('success'))),
                                   actual_distance=r3(r['actual_distance']), execution_time=r3(r['execution_time']),
                                   execution_time_sim=r3(r['execution_time_sim']), n_plans=len(r['stats']),
                                   replan_search_time=r3(sum(d.get('search_ms') or 0 for d in r['stats'][1:])),
                                   final_error=r3(r['final_error']), goal_success=int(r['success']),
                                   mission_success=int(r['success']), failure_reason=r['failure_reason'],
                                   wall_time=round(time.time(), 3))
                        wrong = [d['algorithm'] for d in r['stats'] if d.get('algorithm') != alg]
                        if wrong:
                            row['failure_reason'] = (row['failure_reason'] or '') + f' [경고: 다른 알고리즘 계획 {wrong}]'
                        log.write(row)
                        print(f'[r1] {sc["id"]} {dname} #{rep} {LABEL[alg]:8s}: {"도착" if r["success"] else "실패"} '
                              f'{r["execution_time"]:.1f} s (sim {r["execution_time_sim"]:.1f} s), '
                              f'{r["actual_distance"]:.2f} m, 계획 {len(r["stats"])}회 '
                              f'(첫 탐색 {first.get("search_ms") or 0:.1f} ms, 확장 {first.get("expanded") or 0}), '
                              f'오차 {r["final_error"] or 0:.2f} m'
                              + (f' - {r["failure_reason"]}' if r['failure_reason'] else ''), flush=True)
    except KeyboardInterrupt:
        print('[r1] 중단 (Ctrl-C): 비상정지, 장애물 정리', flush=True)
        ui.send(action='estop')
    finally:
        for name in spawned:
            rig.delete(name)
        try:
            rig.set_algorithm(prm.get('algorithm', 'astar'))
        except RuntimeError:
            pass
        log.close()
        meta = dict(mode='sim', world=args.world, map=map_path, map_image=image, config_version=cver,
                    repeat=args.repeat, settle=args.settle, timeout=args.timeout, params=prm, host=platform.node(),
                    started=os.path.basename(out), rows=log.n)
        with open(os.path.join(out, 'meta.json'), 'w', encoding='utf-8') as f:
            json.dump(meta, f, ensure_ascii=False, indent=2)
        rig.close()
    print(f'[r1] 저장: {out} (trials.csv {log.n}행)', flush=True)
    if log.n:
        report([out], no_plots=args.no_plots)


# ---------------------------------------------------------------------------------------------------- Nav2
NAV2_PLANNERS = {'astar': 'AStar', 'dijkstra': 'Dijkstra'}   # planner_id in experiments/r1/nav2/nav2_r1.yaml
NAV2_BT_FILE = {'AStar': 'bt_astar.xml', 'Dijkstra': 'bt_dijkstra.xml', 'Smac2D': 'bt_smac2d.xml',
                'Smac2DEq': 'bt_smac2deq.xml',
                'ThetaStar': 'bt_thetastar.xml', 'SmacLattice': 'bt_smaclattice.xml'}
NAV2_BT = {alg: os.path.join(REPO, 'experiments', 'r1', 'nav2', NAV2_BT_FILE[pid]) for alg, pid in NAV2_PLANNERS.items()}
ASTAR_LABEL = {'AStar': 'A*', 'Smac2D': 'A* (Smac 2D)', 'Smac2DEq': 'A* (Smac 2D, NavFn-equal cost)',
               'ThetaStar': 'Theta*', 'SmacLattice': 'Smac Lattice'}
CANDIDATES = list(ASTAR_LABEL)   # planners that can take the 'astar' (candidate) side; the baseline is always NavFn Dijkstra


def set_astar_planner(pid):
    """Which Nav2 plugin fills the A* side: 'AStar' (NavFn use_astar true, main study) or 'Smac2D' (re-test with
    Nav2's Smac Planner 2D). The Dijkstra side is always NavFn use_astar false."""
    NAV2_PLANNERS['astar'] = pid
    NAV2_BT['astar'] = os.path.join(REPO, 'experiments', 'r1', 'nav2', NAV2_BT_FILE[pid])
    LABEL['astar'] = ASTAR_LABEL[pid]
    return '' if pid == 'AStar' else '-' + pid.lower()


class Nav2Rig(SimRig):
    """SimRig (Gazebo models, ground-truth pose) + Nav2 actions/services: ComputePathToPose, NavigateToPose,
    costmap clearing, map_server/load_map, /initialpose, and the planner_server's /plan topic."""

    def __init__(self, robot_entity, real=False):
        super().__init__(robot_entity, sim_time=not real)
        self.real = real
        self.active_goal = None   # goal handle of the running NavigateToPose (cancelled on Ctrl-C)
        self.battery_v = None
        if real:   # no Gazebo: the robot pose is AMCL's (TF map -> base_footprint)
            from sensor_msgs.msg import BatteryState
            from tf2_ros import Buffer, TransformListener
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self.node)
            self.node.create_subscription(BatteryState, 'battery_state',
                                          lambda m: setattr(self, 'battery_v', round(m.voltage, 2)), 1)
        from geometry_msgs.msg import PoseWithCovarianceStamped
        from nav2_msgs.action import ComputePathToPose, NavigateToPose
        from nav2_msgs.srv import ClearEntireCostmap, LoadMap
        from nav_msgs.msg import Path
        from rclpy.action import ActionClient
        self.nav_types = dict(ComputePathToPose=ComputePathToPose, NavigateToPose=NavigateToPose,
                              ClearEntireCostmap=ClearEntireCostmap, LoadMap=LoadMap,
                              PoseWithCovarianceStamped=PoseWithCovarianceStamped)
        self.acts = {'plan': ActionClient(self.node, ComputePathToPose, 'compute_path_to_pose'),
                     'nav': ActionClient(self.node, NavigateToPose, 'navigate_to_pose')}
        self.srv['clear_global'] = self.node.create_client(ClearEntireCostmap,
                                                           '/global_costmap/clear_entirely_global_costmap')
        self.srv['clear_local'] = self.node.create_client(ClearEntireCostmap,
                                                          '/local_costmap/clear_entirely_local_costmap')
        self.srv['load_map'] = self.node.create_client(LoadMap, '/map_server/load_map')
        self.init_pub = self.node.create_publisher(PoseWithCovarianceStamped, 'initialpose', 1)
        self.plans = []        # (wall time, length m, points) of every /plan message
        self.feedback = None
        self.node.create_subscription(Path, 'plan', self.on_plan, 10)
        from nav2_msgs.msg import Costmap
        self.costmap = None    # (wall time received, nav2_msgs/Costmap) latest raw global costmap
        self.node.create_subscription(Costmap, '/global_costmap/costmap_raw',
                                      lambda m: setattr(self, 'costmap', (time.time(), m)), 1)

    def capture_costmap(self, path, newer_than, timeout=10.0):
        """Save the raw global costmap (uint8, row 0 = bottom) received after newer_than. Returns its metadata."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            c = self.costmap
            if c is not None and c[0] > newer_than:
                m = c[1]
                md = m.metadata
                with open(path, 'wb') as f:
                    f.write(bytes(m.data))
                return dict(nx=md.size_x, ny=md.size_y, res=md.resolution, ox=md.origin.position.x,
                            oy=md.origin.position.y, file=path)
            time.sleep(0.1)
        raise RuntimeError('global_costmap/costmap_raw 를 받지 못했습니다')

    def gt_pose(self):
        if not self.real:
            return super().gt_pose()
        from rclpy.time import Time
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_footprint', Time())
        except Exception as e:  # tf2 exception classes differ by version
            raise RuntimeError(f'TF map->base_footprint 없음: {e}')
        q = t.transform.rotation
        return (t.transform.translation.x, t.transform.translation.y,
                math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))

    def on_plan(self, msg):
        pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        with self.lock:
            self.plans.append((time.time(), polyline_length(pts, 0.0), len(pts)))

    def pose_stamped(self, p):
        from geometry_msgs.msg import PoseStamped
        m = PoseStamped()
        m.header.frame_id = 'map'
        m.pose.position.x, m.pose.position.y = float(p[0]), float(p[1])
        yaw = math.radians(p[2]) if len(p) > 2 else 0.0
        m.pose.orientation.z, m.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        return m

    def wait_future(self, fut, timeout):
        ev = threading.Event()
        fut.add_done_callback(lambda _: ev.set())
        return ev.wait(timeout)

    def restart_navigation(self, timeout=60.0):
        """RESET + STARTUP of the Nav2 navigation nodes (planner_server, controller, bt_navigator, ...) through
        lifecycle_manager_navigation. Humble's action servers keep every finished goal for 15 min (rcl_action default
        result_timeout, no Nav2 parameter in 1.1.20) and each new goal gets slower with the retained count (measured:
        NavFn Dijkstra R1-A response 25 -> 123 -> 228 ms over three back-to-back runs), which adds the same delay to
        both planners and squeezes their ratio toward 1. nav2plan therefore starts every run from fresh servers.
        AMCL / map_server (lifecycle_manager_localization) are not touched."""
        from nav2_msgs.srv import ManageLifecycleNodes
        if 'manage_nav' not in self.srv:
            self.srv['manage_nav'] = self.node.create_client(ManageLifecycleNodes,
                                                             '/lifecycle_manager_navigation/manage_nodes')
        for cmd, name in ((ManageLifecycleNodes.Request.RESET, 'RESET'),
                          (ManageLifecycleNodes.Request.STARTUP, 'STARTUP')):
            req = ManageLifecycleNodes.Request()
            req.command = cmd
            r = self.call('manage_nav', req, timeout=timeout)
            if not r.success:
                raise RuntimeError(f'Nav2 navigation {name} 실패')
        self.wait_servers()

    def wait_servers(self, timeout=30.0):
        for name, a in self.acts.items():
            if not a.wait_for_server(timeout_sec=timeout):
                raise RuntimeError(f'Nav2 액션 서버가 없습니다: {a._action_name} (run_nav2_sim.sh 실행 확인)')

    def compute_path(self, a, b, alg, timeout=15.0):
        """ComputePathToPose from a to b with planner AStar/Dijkstra. Returns (points, planning_time ms, round trip ms, error)."""
        goal = self.nav_types['ComputePathToPose'].Goal()
        goal.start, goal.goal = self.pose_stamped(a), self.pose_stamped(b)
        goal.planner_id, goal.use_start = NAV2_PLANNERS[alg], True
        t0 = time.perf_counter()
        fut = self.acts['plan'].send_goal_async(goal)
        if not self.wait_future(fut, timeout) or not fut.result().accepted:
            return None, None, None, '목표 거부/응답 없음'
        rfut = fut.result().get_result_async()
        if not self.wait_future(rfut, timeout):
            return None, None, None, '결과 응답 없음'
        rt = (time.perf_counter() - t0) * 1000.0
        res = rfut.result()
        pt = res.result.planning_time
        # Humble (nav2_planner 1.1.x) leaves planning_time empty -> None; the NavFn core benchmark measures it instead
        planning_ms = (pt.sec * 1000.0 + pt.nanosec / 1e6) or None
        if res.status != 4:   # action_msgs/GoalStatus.STATUS_SUCCEEDED
            return None, planning_ms, rt, f'계획 실패 (상태 {res.status})'
        pts = [(p.pose.position.x, p.pose.position.y) for p in res.result.path.poses]
        if not pts:
            return None, planning_ms, rt, '빈 경로'
        return pts, planning_ms, rt, None

    def on_feedback(self, msg):
        self.feedback = msg.feedback

    def navigate(self, goal_pt, alg, timeout):
        """NavigateToPose with the BT that uses planner alg. Returns dict like drive()."""
        g = self.nav_types['NavigateToPose'].Goal()
        g.pose, g.behavior_tree = self.pose_stamped(goal_pt), NAV2_BT[alg]
        with self.lock:
            self.gt.clear()
            mark = len(self.plans)
        self.feedback = None
        self.tracking = True
        t0, s0 = time.time(), self.sim_now()
        fut = self.acts['nav'].send_goal_async(g, feedback_callback=self.on_feedback)
        fail = None
        status = None
        if not self.wait_future(fut, 15.0) or not fut.result().accepted:
            fail = 'NavigateToPose 목표 거부'
        else:
            gh = fut.result()
            self.active_goal = gh
            rfut = gh.get_result_async()
            if self.wait_future(rfut, timeout):
                status = rfut.result().status
                if status != 4:
                    fail = {6: 'ABORTED (Nav2 가 목표 포기: 경로 없음/복구 실패)', 5: 'CANCELED'}.get(status, f'상태 {status}')
            else:
                cfut = gh.cancel_goal_async()
                self.wait_future(cfut, 10.0)
                fail = f'시간 초과 ({timeout:.0f}초) → 목표 취소'
            self.active_goal = None
        self.tracking = False
        t1, s1 = time.time(), self.sim_now()
        time.sleep(0.5)
        with self.lock:
            gt = [(x, y) for _, x, y in self.gt]
            plans = self.plans[mark:]
        try:
            final = self.gt_pose()
        except RuntimeError:
            final = None
        err = math.hypot(final[0] - goal_pt[0], final[1] - goal_pt[1]) if final else None
        fb = self.feedback
        return dict(success=fail is None, failure_reason=fail, execution_time=t1 - t0, execution_time_sim=s1 - s0,
                    actual_distance=polyline_length(gt), final_error=err, plans=plans, track=gt,
                    recoveries=fb.number_of_recoveries if fb is not None else 0)

    def cancel_active(self):
        """Cancel the running NavigateToPose (Ctrl-C): Nav2 stops the controller, so the robot stops."""
        gh, self.active_goal = self.active_goal, None
        if gh is not None:
            self.wait_future(gh.cancel_goal_async(), 5.0)
            print('[r1] 진행 중이던 Nav2 목표를 취소했습니다 (로봇 정지)', flush=True)

    def set_global_inflation(self, radius, scaling):
        from rcl_interfaces.msg import Parameter as PMsg
        from rcl_interfaces.msg import ParameterType, ParameterValue
        from rcl_interfaces.srv import SetParameters
        if 'gc_param' not in self.srv:
            self.srv['gc_param'] = self.node.create_client(SetParameters, '/global_costmap/global_costmap/set_parameters')
        req = SetParameters.Request()
        dbl = ParameterType.PARAMETER_DOUBLE
        req.parameters = [PMsg(name='inflation_layer.inflation_radius', value=ParameterValue(type=dbl, double_value=radius)),
                          PMsg(name='inflation_layer.cost_scaling_factor',
                               value=ParameterValue(type=dbl, double_value=scaling))]
        r = self.call('gc_param', req, timeout=10.0)
        bad = [x.reason for x in r.results if not x.successful]
        if bad:
            raise RuntimeError(f'global costmap 인플레이션 설정 실패: {bad}')

    def clear_costmaps(self):
        for k in ('clear_global', 'clear_local'):
            self.call(k, self.nav_types['ClearEntireCostmap'].Request())

    def load_map(self, path):
        req = self.nav_types['LoadMap'].Request()
        req.map_url = path
        r = self.call('load_map', req, timeout=15.0)
        if r.result != 0:
            raise RuntimeError(f'맵 바꾸기 실패 ({path}): 결과 {r.result}')

    def reset_pose(self):
        """Tell AMCL the ground-truth pose (after a map switch AMCL may re-initialise)."""
        if self.real:
            return
        p = self.gt_pose()
        if p is None:
            return
        m = self.nav_types['PoseWithCovarianceStamped']()
        m.header.frame_id = 'map'
        m.pose.pose.position.x, m.pose.pose.position.y = p[0], p[1]
        m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(p[2] / 2), math.cos(p[2] / 2)
        m.pose.covariance[0] = m.pose.covariance[7] = 0.02 ** 2
        m.pose.covariance[35] = math.radians(3) ** 2
        for _ in range(3):
            self.init_pub.publish(m)
            time.sleep(0.2)


def obstacle_map(map_path, boxes, out_dir, name):
    """Copy of the map with the boxes drawn as occupied (for planning-only runs: map_server/load_map)."""
    conf = read_yaml(map_path)
    grid, res, ox, oy, image = load_map(map_path)
    mask = np.flipud(np.zeros(grid.shape, dtype=bool))
    for b in boxes:
        mask |= box_cells(grid.shape, res, ox, oy, b)
    img = read_pgm(image)
    img[np.flipud(mask)] = 0 if not int(conf.get('negate', 0)) else 255
    os.makedirs(out_dir, exist_ok=True)
    pgm, yml = os.path.join(out_dir, f'{name}.pgm'), os.path.join(out_dir, f'{name}.yaml')
    h, w = img.shape
    with open(pgm, 'wb') as f:
        f.write(f'P5\n# tools/r1_bench.py: {os.path.basename(map_path)} + {len(boxes)} boxes\n{w} {h}\n255\n'.encode())
        f.write(img.astype(np.uint8).tobytes())
    with open(yml, 'w', encoding='utf-8') as f:
        f.write(f'image: {name}.pgm\nmode: trinary\nresolution: {res}\norigin: [{ox}, {oy}, 0.0]\n'
                f'negate: {int(conf.get("negate", 0))}\noccupied_thresh: {conf.get("occupied_thresh", 0.65)}\n'
                f'free_thresh: {conf.get("free_thresh", 0.196)}\n')
    return yml


def nav2_preflight(args):
    cfg, points = load_scenarios(args.scenarios, args.world)
    map_path = resolve_map(cfg['map'])
    mode_env = os.path.join(os.environ.get('WAREHOUSE_HOME', REPO), 'logs', 'run_mode.env')
    env = {}
    if os.path.isfile(mode_env):
        for ln in open(mode_env, encoding='utf-8'):
            if '=' in ln and not ln.startswith('#'):
                k, v = ln.strip().split('=', 1)
                env[k] = v
    real = getattr(args, 'real', False)
    script = 'run_nav2_real.sh' if real else 'run_nav2_sim.sh'
    if (env.get('WAREHOUSE_NAV') != 'nav2' or env.get('WAREHOUSE_ROBOT') != ('real' if real else 'sim')
            or os.path.realpath(env.get('WAREHOUSE_MAP', '')) != os.path.realpath(map_path)):
        print(f'[r1] 경고: Nav2 {"실차" if real else "시뮬레이션"} 실행({script} {args.world})이 아닌 것 같습니다 '
              f'(ROBOT={env.get("WAREHOUSE_ROBOT")}, NAV={env.get("WAREHOUSE_NAV")}, MAP={env.get("WAREHOUSE_MAP")})',
              flush=True)
        if not args.force:
            raise SystemExit(f'중단: experiments/r1/nav2/{script} 로 먼저 실행하세요 (--force 로 무시)')
    return cfg, points, map_path


def nav2_config_version(map_image, inflation=None):
    p = sha1_file(os.path.join(REPO, 'experiments', 'r1', 'nav2', 'nav2_r1.yaml'))
    return f'{git_version()}-n{p}-m{sha1_file(map_image)}' + (f'-i{inflation[0]:g}_{inflation[1]:g}' if inflation else '')


def apply_inflation(rig, args):
    """--inflation R,S: set the global costmap inflation (radius m, cost_scaling_factor) for this run, wait for the
    costmap to be rebuilt. Returns (R, S) or None (nav2_r1.yaml values: 0.55, 3.0)."""
    if not getattr(args, 'inflation', None):
        return None
    r, sc = (float(v) for v in args.inflation.split(','))
    rig.set_global_inflation(r, sc)
    rig.clear_costmaps()
    time.sleep(3.0)   # global costmap update_frequency 1 Hz: let it re-inflate
    print(f'[r1] global costmap 인플레이션: inflation_radius {r:g} m, cost_scaling_factor {sc:g}', flush=True)
    return r, sc


def inflation_tag(infl):
    return '' if not infl else f'-inf{infl[0]:g}s{infl[1]:g}'


def run_nav2plan(args):
    cfg, points, map_path = nav2_preflight(args)
    grid, res, ox, oy, image = load_map(map_path)
    suffix = set_astar_planner(args.astar_planner)
    if suffix:
        args.no_core = True   # the C++ core benchmark is NavFn only
    out = new_out_dir('nav2plan' + suffix, args.world, args.out)
    exp_id = f'R1-nav2plan-{os.path.basename(out)}'
    rig = Nav2Rig(os.environ.get('TURTLEBOT3_MODEL', 'waffle_pi'))
    rig.wait_servers()
    if not args.no_restart:
        print('[r1] Nav2 navigation 노드 재시작 (액션 서버에 쌓인 목표 비우기)', flush=True)
        rig.restart_navigation()
        time.sleep(max(args.settle, 3.0))
    infl = apply_inflation(rig, args)
    cver = nav2_config_version(image, infl)
    if infl:   # the folder name gets the inflation too
        old_out = out
        out = new_out_dir('nav2plan' + suffix + inflation_tag(infl), args.world,
                          args.out or old_out.replace('_nav2plan', '_nav2plan' + inflation_tag(infl), 1))
        if old_out != out and not os.listdir(old_out):
            os.rmdir(old_out)
        exp_id = f'R1-nav2plan-{os.path.basename(out)}'
    log = CsvLog(os.path.join(out, 'trials.csv'))
    only = set(args.only.split(',')) if args.only else None
    vis = {}
    print(f'[r1] nav2plan: 월드 {args.world}, 반복 {args.repeat}, 설정 {cver}', flush=True)

    costmaps, jobs = {}, {}    # map name -> captured raw costmap metadata / NavFn core jobs on it
    cm_dir = os.path.join(out, 'costmaps')
    os.makedirs(cm_dir, exist_ok=True)
    current = ['base']

    def capture(name):
        costmaps[name] = rig.capture_costmap(os.path.join(cm_dir, f'{name}.bin'), time.time())
        current[0] = name

    def trial_rows(sid, name, dname, a, b, pair_prefix, reps, keep=None, field=None):
        base = dict(experiment_id=exp_id, mode='nav2plan', world=args.world, scenario_id=sid, scenario_name=name,
                    inflation=f'r={infl[0]:g},s={infl[1]:g}' if infl else 'r=0.55,s=3',
                    direction=dname, config_version=cver, start_x=r3(a[0]), start_y=r3(a[1]), goal_x=r3(b[0]),
                    goal_y=r3(b[1]), straight_distance=r3(math.hypot(b[0] - a[0], b[1] - a[1])))
        jobs.setdefault(current[0], []).append((dict(base, mode='navfncore'), pair_prefix, a, b,
                                               args.core_repeat if sid != 'RND' else args.random_repeat))
        for alg in ALGORITHMS:   # warm-up (not recorded)
            rig.compute_path(a, b, alg)
        last = {}
        for rep in range(reps):
            order = ALGORITHMS if rep % 2 == 0 else ALGORITHMS[::-1]
            for k, alg in enumerate(order):
                pts, pms, rt, err = rig.compute_path(a, b, alg)
                cmin, cmean = path_clearance(field, pts)
                log.write(dict(base, pair=f'{pair_prefix}-{rep}', rep=rep, order=k, algorithm=alg,
                               min_clearance=r3(cmin), mean_clearance=r3(cmean),
                               turn_total=round(turn_total(pts), 1) if pts else None,
                               planner_id=NAV2_PLANNERS[alg], planning_time=r3(pms), roundtrip_time=r3(rt),
                               planned_distance=r3(polyline_length(pts, 0.0)) if pts else None,
                               path_points=len(pts) if pts else 0, plan_success=int(pts is not None),
                               failure_reason=err, wall_time=round(time.time(), 3)))
                last[alg] = (pts, pms, err)
        if keep is not None:
            keep[(sid, dname)] = dict(res=last, start=a, goal=b)
        return last

    try:
        time.sleep(1.0)
        capture('base')
        for sc in cfg.get('scenarios', []):
            if only and sc['id'] not in only:
                continue
            boxes = sc.get('obstacles') or ()
            if boxes:
                yml = obstacle_map(map_path, boxes, os.path.join(out, 'maps'), f'map_{sc["id"]}')
                rig.load_map(yml)
                rig.reset_pose()
                time.sleep(args.settle)
                capture(f'map_{sc["id"]}')
            field = clearance_field(grid, res, ox, oy, boxes)
            for dname, a, b in directions(sc, points):
                last = trial_rows(sc['id'], sc.get('name', ''), dname, a, b, f'{sc["id"]}-{dname}', args.repeat, vis,
                                  field)
                vis[(sc['id'], dname)]['boxes'] = boxes
                print(f'[r1] {sc["id"]} {dname}: ' + ' | '.join(
                    f'{LABEL[k]} {"성공" if v[0] else "실패"} {polyline_length(v[0], 0.0) if v[0] else 0:.2f} m'
                    for k, v in last.items()), flush=True)
            if boxes:
                rig.load_map(map_path)
                rig.reset_pose()
                time.sleep(args.settle)
                current[0] = 'base'
        if args.random:
            base_field = clearance_field(grid, res, ox, oy)
            rng = np.random.default_rng(args.seed)
            pl = planning_planner(grid, res, ox, oy, load_params(), (ox, oy))
            rr, cc = np.nonzero(~pl.lethal & (pl.dist >= 0.35))
            done = 0
            while done < args.random:
                i, j = rng.integers(0, len(rr), 2)
                a, b = pl.cell_to_world(rr[i], cc[i]), pl.cell_to_world(rr[j], cc[j])
                if math.hypot(b[0] - a[0], b[1] - a[1]) < args.min_dist:
                    continue
                trial_rows('RND', '무작위 쌍', 'fwd', a, b, f'RND-{done}', args.random_repeat, field=base_field)
                done += 1
            print(f'[r1] 무작위 쌍 {done}개 (쌍마다 {args.random_repeat}회)', flush=True)
    finally:
        if infl:   # leave Nav2 as configured for the next run
            rig.set_global_inflation(0.55, 3.0)
        log.close()
        meta = dict(mode='nav2plan', world=args.world, map=map_path, map_image=image, config_version=cver,
                    planners=dict(NAV2_PLANNERS),
                    nav2_params=os.path.join('experiments', 'r1', 'nav2', 'nav2_r1.yaml'), repeat=args.repeat,
                    random=args.random, seed=args.seed, host=platform.node(), started=os.path.basename(out), rows=log.n)
        with open(os.path.join(out, 'meta.json'), 'w', encoding='utf-8') as f:
            json.dump(meta, f, ensure_ascii=False, indent=2)
        rig.close()
    if not args.no_plots:
        for (sid, dname), v in vis.items():
            plot_overlay(os.path.join(out, f'paths_{sid}_{dname}.png'), v, grid, res, ox, oy,
                         f'{args.world} {sid} ({dname}) - Nav2 NavFn')
    print(f'[r1] 저장: {out} (trials.csv {log.n}행)', flush=True)
    report([out], no_plots=args.no_plots)
    if not args.no_core:
        run_navfn_core(out, args, costmaps, jobs, cver)


def ensure_navfn_bench():
    """Compile experiments/r1/nav2/navfn_bench.cpp against the installed Nav2 NavFn (cached by source hash)."""
    src = os.path.join(REPO, 'experiments', 'r1', 'nav2', 'navfn_bench.cpp')
    exe = f'/tmp/navfn_bench_{sha1_file(src, 8)}'
    if not os.path.isfile(exe):
        cmd = ['g++', '-O2', '-std=c++17', '-I/opt/ros/humble/include', src, '-L/opt/ros/humble/lib',
               '-lnav2_navfn_planner', '-Wl,-rpath,/opt/ros/humble/lib', '-o', exe]
        r = subprocess.run(cmd, capture_output=True, text=True)
        if r.returncode:
            raise RuntimeError('navfn_bench 빌드 실패:\n' + r.stderr[-2000:])
    return exe


def run_navfn_core(plan_out, args, costmaps, jobs, cver):
    """NavFn core benchmark (C++, no ROS messaging) on the costmaps captured during nav2plan: exact computation time
    and the number of cells NavFn gave a potential to. Results go to a sibling folder <time>_navfncore_<world>."""
    exe = ensure_navfn_bench()
    core_dir = plan_out.replace('_nav2plan_', '_navfncore_')
    out = new_out_dir('navfncore', args.world, core_dir if core_dir != plan_out else plan_out.rstrip('/') + '_navfncore')
    log = CsvLog(os.path.join(out, 'trials.csv'))
    exp_id = f'R1-navfncore-{os.path.basename(out)}'
    for name, meta in costmaps.items():
        js = jobs.get(name) or []
        if not js:
            continue
        jf, of = os.path.join(out, f'jobs_{name}.txt'), os.path.join(out, f'core_{name}.csv')
        info = {}
        with open(jf, 'w', encoding='utf-8') as f:
            for n, (base, prefix, a, b, reps) in enumerate(js):
                cell = lambda p: (int(math.floor((p[0] - meta['ox']) / meta['res'])),  # noqa: E731
                                  int(math.floor((p[1] - meta['oy']) / meta['res'])))
                (sx, sy), (gx, gy) = cell(a), cell(b)
                f.write(f'J{n} {sx} {sy} {gx} {gy} {reps}\n')
                info[f'J{n}'] = (base, prefix)
        subprocess.run([exe, meta['file'], str(meta['nx']), str(meta['ny']), str(meta['res']), jf, of, '1', '0.5'],
                       check=True)
        with open(of, encoding='utf-8') as f:
            for row in csv.DictReader(f):
                base, prefix = info[row['id']]
                ok = int(row['ok'])
                log.write(dict(base, experiment_id=exp_id, config_version=cver, pair=f'{prefix}-{row["rep"]}',
                               rep=int(row['rep']), order=int(row['order']), algorithm=row['algorithm'],
                               planning_time=r3(row['time_ms']), expanded=int(row['reached_cells']),
                               planned_distance=r3(row['path_length_m']) if ok else None,
                               path_points=int(row['path_cells']), plan_success=ok,
                               failure_reason=None if ok else '경로 없음 (포텐셜/경사 따라가기 실패)'))
    log.close()
    with open(os.path.join(out, 'meta.json'), 'w', encoding='utf-8') as f:
        json.dump(dict(mode='navfncore', world=args.world, from_nav2plan=os.path.basename(plan_out),
                       costmaps={k: os.path.relpath(v['file'], REPO) for k, v in costmaps.items()},
                       bench=os.path.join('experiments', 'r1', 'nav2', 'navfn_bench.cpp'), allow_unknown=True,
                       tolerance=0.5, config_version=cver, rows=log.n), f, ensure_ascii=False, indent=2)
    print(f'[r1] NavFn 핵심 계산 저장: {out} (trials.csv {log.n}행)', flush=True)
    report([out], no_plots=args.no_plots)


def operator(msg, args):
    """Real robot: wait for the operator (obstacles placed/removed, area clear). --yes skips."""
    if not getattr(args, 'real', False) or args.yes:
        return
    input(f'[r1] {msg} → 준비되면 Enter (중단: Ctrl-C) ')


def reposition_nav(rig, point, timeout):
    try:
        p = rig.gt_pose()
    except RuntimeError:
        p = None
    if p is not None:
        dyaw = abs((p[2] - math.radians(point[2] if len(point) > 2 else 0.0) + math.pi) % (2 * math.pi) - math.pi)
        if math.hypot(p[0] - point[0], p[1] - point[1]) < 0.3 and dyaw < math.radians(30):
            return True
    for attempt in range(2):
        rig.clear_costmaps()
        r = rig.navigate(point, 'astar', timeout)
        if r['success']:
            return True
        print(f'[r1]   출발점 이동 실패 ({r["failure_reason"]}), 재시도 {attempt + 1}', flush=True)
        time.sleep(2.0)
    return False


def run_nav2sim(args):
    """Driving comparison. --real: on the real robot (run_nav2_real.sh): pose/distance from AMCL instead of Gazebo,
    scenario obstacles are placed by hand when asked, and every trial waits for the operator (unless --yes)."""
    cfg, points, map_path = nav2_preflight(args)
    grid, res, ox, oy, image = load_map(map_path)
    mode = 'nav2real' if args.real else 'nav2sim'
    suffix = set_astar_planner(args.astar_planner)
    out = new_out_dir(mode + suffix, args.world, args.out)
    exp_id = f'R1-{mode}-{os.path.basename(out)}'
    rig = Nav2Rig(os.environ.get('TURTLEBOT3_MODEL', 'waffle_pi'), real=args.real)
    rig.wait_servers()
    infl = apply_inflation(rig, args)
    cver = nav2_config_version(image, infl)
    if infl:
        old_out = out
        out = new_out_dir(mode + suffix + inflation_tag(infl), args.world,
                          args.out or old_out.replace('_' + mode, '_' + mode + inflation_tag(infl), 1))
        if old_out != out and not os.listdir(old_out):
            os.rmdir(old_out)
        exp_id = f'R1-{mode}-{os.path.basename(out)}'
    log = CsvLog(os.path.join(out, 'trials.csv'))
    only = set(args.only.split(',')) if args.only else None
    scenarios = [sc for sc in cfg.get('scenarios', []) if sc.get('sim', True) and not (only and sc['id'] not in only)]
    spawned = []
    print(f'[r1] {mode}: 월드 {args.world}, 반복 {args.repeat}, 설정 {cver}', flush=True)
    try:
        for rep in range(args.repeat):
            for sc in scenarios:
                order = ALGORITHMS if rep % 2 == 0 else ALGORITHMS[::-1]
                for k, alg in enumerate(order):
                    for dname, a, b in directions(sc, points):
                        operator(f'{sc["id"]} {dname} #{rep} {LABEL[alg]}: 출발점 ({a[0]:.2f}, {a[1]:.2f}) 로 이동합니다. '
                                 '주변을 비우세요', args)
                        if not reposition_nav(rig, a, args.timeout):
                            raise RuntimeError(f'{sc["id"]}: 출발점 {a} 로 가지 못했습니다')
                        boxes = sc.get('obstacles') or ()
                        if args.real and boxes:
                            operator('장애물 놓기: ' + ', '.join(f'중심 ({o[0]:.2f}, {o[1]:.2f}) 크기 {o[2]:.2f}x{o[3]:.2f} m'
                                                              for o in boxes), args)
                        rig.clear_costmaps()
                        if not args.real:
                            for i, box in enumerate(boxes):
                                name = f'r1_obs_{i}'
                                rig.spawn_box(name, box)
                                spawned.append(name)
                        time.sleep(args.settle)
                        battery = rig.battery_v
                        r = rig.navigate(b, alg, args.timeout)
                        for name in spawned:
                            rig.delete(name)
                        spawned.clear()
                        if args.real and boxes:
                            operator('장애물 치우기', args)
                        first = r['plans'][0] if r['plans'] else None
                        cmin, cmean = path_clearance(clearance_field(grid, res, ox, oy, boxes), r['track'])
                        log.write(dict(
                            min_clearance=r3(cmin), mean_clearance=r3(cmean),
                            inflation=f'r={infl[0]:g},s={infl[1]:g}' if infl else 'r=0.55,s=3',
                            experiment_id=exp_id, mode=mode, world=args.world, scenario_id=sc['id'],
                            scenario_name=sc.get('name', ''), direction=dname, pair=f'{sc["id"]}-{dname}-{rep}',
                            rep=rep, order=k, algorithm=alg, planner_id=NAV2_PLANNERS[alg], config_version=cver,
                            start_x=a[0], start_y=a[1],
                            goal_x=b[0], goal_y=b[1], straight_distance=r3(math.hypot(b[0] - a[0], b[1] - a[1])),
                            planned_distance=r3(first[1]) if first else None, path_points=first[2] if first else 0,
                            plan_success=int(first is not None and first[2] > 0),
                            actual_distance=r3(r['actual_distance']), execution_time=r3(r['execution_time']),
                            execution_time_sim=r3(r['execution_time_sim']), n_plans=len(r['plans']),
                            recoveries=r['recoveries'], final_error=r3(r['final_error']),
                            goal_success=int(r['success']), mission_success=int(r['success']),
                            failure_reason=r['failure_reason'], wall_time=round(time.time(), 3), battery_v=battery))
                        print(f'[r1] {sc["id"]} {dname} #{rep} {LABEL[alg]:8s}: {"도착" if r["success"] else "실패"} '
                              f'{r["execution_time"]:.1f} s (sim {r["execution_time_sim"]:.1f} s), '
                              f'{r["actual_distance"]:.2f} m, 계획 {len(r["plans"])}회 '
                              f'(첫 경로 {first[1] if first else 0:.2f} m), 복구 {r["recoveries"]}회, '
                              f'오차 {r["final_error"] or 0:.2f} m'
                              + (f' - {r["failure_reason"]}' if r['failure_reason'] else ''), flush=True)
    except KeyboardInterrupt:
        print('[r1] 중단 (Ctrl-C)', flush=True)
        rig.cancel_active()
    finally:
        rig.cancel_active()
        if infl:   # leave Nav2 as configured for the next run
            rig.set_global_inflation(0.55, 3.0)
        for name in spawned:
            rig.delete(name)
        log.close()
        meta = dict(mode=mode, world=args.world, map=map_path, map_image=image, config_version=cver, real=args.real,
                    nav2_params=os.path.join('experiments', 'r1', 'nav2', 'nav2_r1.yaml'),
                    planners=dict(NAV2_PLANNERS),
                    bt={k: os.path.relpath(v, REPO) for k, v in NAV2_BT.items()}, repeat=args.repeat,
                    settle=args.settle, timeout=args.timeout, host=platform.node(), started=os.path.basename(out),
                    rows=log.n)
        with open(os.path.join(out, 'meta.json'), 'w', encoding='utf-8') as f:
            json.dump(meta, f, ensure_ascii=False, indent=2)
        rig.close()
    print(f'[r1] 저장: {out} (trials.csv {log.n}행)', flush=True)
    if log.n:
        report([out], no_plots=args.no_plots)


# ---------------------------------------------------------------------------------------------------- report
def fnum(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def mean_ci(vals):
    """mean, sd, 95% CI half width (t distribution)."""
    v = np.asarray([x for x in vals if x is not None], dtype=float)
    if len(v) == 0:
        return None, None, None
    if len(v) == 1:
        return float(v[0]), 0.0, None
    m, sd = float(v.mean()), float(v.std(ddof=1))
    try:
        from scipy.stats import t
        q = float(t.ppf(0.975, len(v) - 1))
    except ImportError:
        q = 1.96
    return m, sd, q * sd / math.sqrt(len(v))


def fmt(m, ci=None, nd=2):
    if m is None:
        return '-'
    return f'{m:.{nd}f}' + (f' ± {ci:.{nd}f}' if ci is not None else '')


def read_rows(dirs):
    rows = []
    for d in dirs:
        with open(os.path.join(d, 'trials.csv'), encoding='utf-8') as f:
            rows += list(csv.DictReader(f))
    return rows


def scenario_order(rows):
    """Set r['group'] = scenario id (+ ' fwd'/' rev' when both directions were measured; they are different conditions,
    e.g. the heuristic can fit one direction better) and return the groups in first-seen order."""
    dirs = {}
    for r in rows:
        dirs.setdefault(r['scenario_id'], set()).add(r['direction'])
    seen = []
    for r in rows:
        r['group'] = r['scenario_id'] + (f' {r["direction"]}' if len(dirs[r['scenario_id']]) > 1 else '')
        if r['group'] not in seen:
            seen.append(r['group'])
    return seen


def paired(rows, metric):
    """Dijkstra - A* per pair (same scenario, direction, repetition). Returns list of differences and both values."""
    by = {}
    for r in rows:
        by.setdefault(r['pair'], {})[r['algorithm']] = fnum(r.get(metric))
    out = []
    for p in by.values():
        a, d = p.get('astar'), p.get('dijkstra')
        if a is not None and d is not None:
            out.append((d - a, a, d))
    return out


_M = {   # column: (label in summary.md, decimals)
    'search_time': ('탐색 시간 (ms)', 2), 'planning_time': ('계획 시간 (ms)', 2), 'expanded': ('확장 칸 수', 0),
    'planned_distance': ('계획 경로 길이 (m)', 3), 'path_cost': ('경로 비용', 3),
    'execution_time': ('주행 시간 (s)', 1), 'execution_time_sim': ('주행 시간, sim (s)', 1),
    'actual_distance': ('실제 주행 거리 (m)', 2), 'n_plans': ('계획 횟수', 1), 'final_error': ('도착 오차 (m)', 3),
    'roundtrip_time': ('요청~응답 (ms)', 2), 'path_points': ('경로 점 수', 0), 'recoveries': ('복구 횟수', 2),
    'min_clearance': ('최소 장애물 거리 (m)', 3), 'mean_clearance': ('평균 장애물 거리 (m)', 3),
    'turn_total': ('꺾인 각도 합 (도)', 0),
}
MODE_SPEC = {
    'offline': dict(
        title='계획만 — 자체 격자 탐색 (warehouse_robot/astar.py, 휴리스틱 켜기/끄기)', goal=False,
        metrics=['search_time', 'planning_time', 'expanded', 'planned_distance', 'path_cost'],
        compare=['search_time', 'planning_time', 'expanded', 'planned_distance', 'path_cost'],
        ratios=['search_time', 'expanded'],
        panels=[('search_time', 'Search time (ms)', 'ratio'), ('expanded', 'Expanded cells', 'ratio'),
                ('planned_distance', 'Planned path length (m)', 'diff')]),
    'sim': dict(
        title='Gazebo 주행 — 자체 격자 탐색 + Frenet (프로젝트 브리지)', goal=True,
        metrics=['search_time', 'planning_time', 'expanded', 'planned_distance', 'path_cost', 'execution_time',
                 'actual_distance', 'n_plans', 'final_error'],
        compare=['search_time', 'planning_time', 'expanded', 'planned_distance', 'path_cost', 'execution_time',
                 'actual_distance'],
        ratios=['search_time', 'expanded'],
        panels=[('search_time', 'First-plan search time (ms)', 'ratio'), ('execution_time', 'Drive time (s)', 'diff'),
                ('actual_distance', 'Driven distance (m)', 'diff')]),
    'nav2plan': dict(
        title='계획만 — Nav2 NavFn (ComputePathToPose, planner_id AStar / Dijkstra)', goal=False,
        metrics=['roundtrip_time', 'planned_distance', 'path_points', 'min_clearance', 'mean_clearance', 'turn_total'],
        compare=['roundtrip_time', 'planned_distance', 'min_clearance', 'mean_clearance', 'turn_total'],
        ratios=['roundtrip_time'],
        panels=[('roundtrip_time', 'ComputePathToPose round trip (ms)', 'ratio'),
                ('plan_success', 'Plan found (%)', 'pct'),
                ('planned_distance', 'Planned path length (m)', 'diff')]),
    'navfncore': dict(
        title='계획만 — Nav2 NavFn 핵심 계산 (C++, Nav2 global costmap 그대로, 메시지 통신 없음)', goal=False,
        metrics=['planning_time', 'expanded', 'planned_distance', 'path_points'],
        compare=['planning_time', 'expanded', 'planned_distance'],
        ratios=['planning_time', 'expanded'],
        panels=[('planning_time', 'NavFn computation time (ms)', 'ratio'),
                ('expanded', 'Cells given a potential', 'ratio'),
                ('planned_distance', 'Path length (m)', 'diff')]),
    'nav2real': dict(
        title='실차 주행 — Nav2 (NavigateToPose, NavFn AStar / Dijkstra), 위치·거리는 AMCL 기준', goal=True,
        metrics=['execution_time', 'actual_distance', 'planned_distance', 'n_plans', 'recoveries', 'final_error',
                 'min_clearance'],
        compare=['execution_time', 'actual_distance', 'planned_distance'],
        ratios=[],
        panels=[('execution_time', 'Drive time (s)', 'diff'), ('actual_distance', 'Driven distance, AMCL (m)', 'diff'),
                ('goal_success', 'Goal reached (%)', 'pct')]),
    'nav2sim': dict(
        title='Gazebo 주행 — Nav2 (NavigateToPose, NavFn AStar / Dijkstra + DWB)', goal=True,
        metrics=['execution_time', 'execution_time_sim', 'actual_distance', 'planned_distance', 'n_plans',
                 'recoveries', 'final_error', 'min_clearance'],
        compare=['execution_time', 'execution_time_sim', 'actual_distance', 'planned_distance'],
        ratios=[],
        panels=[('execution_time_sim', 'Drive time, sim clock (s)', 'diff'),
                ('actual_distance', 'Driven distance (m)', 'diff'), ('goal_success', 'Goal reached (%)', 'pct')]),
}


def report(dirs, no_plots=False, out=None):
    rows = read_rows(dirs)
    if not rows:
        print('[r1] 기록 없음')
        return
    out = out or dirs[0]
    pid = next((r.get('planner_id') for r in rows if r['algorithm'] == 'astar' and r.get('planner_id')), 'AStar')
    LABEL['astar'] = ASTAR_LABEL.get(pid, 'A*')
    mode = rows[0]['mode']
    world = rows[0]['world']
    spec = MODE_SPEC[mode]
    sim = spec['goal']
    lines = ['﻿# 연구 1 결과 요약: A* vs Dijkstra', '',
             f'- 모드: **{spec["title"]}**, 월드 `{world}`, 기록 {len(rows)}행',
             f'- 원자료: {", ".join(os.path.relpath(os.path.join(d, "trials.csv"), REPO) for d in dirs)}',
             f'- 설정 버전: {", ".join(sorted({r["config_version"] for r in rows}))}',
             '- 값: 평균 ± 95% 신뢰구간 반폭. 차이 = Dijkstra − A* (같은 시나리오·방향·반복끼리 짝지음)', '']
    metrics = [(k, *_M[k]) for k in spec['metrics']]
    sids = scenario_order(rows)
    names = {r['group']: r['scenario_name'] for r in rows}

    lines += ['## 1. 시나리오별 결과', '']
    head = '| 시나리오 | 알고리즘 | n | 계획 성공 |' + (' 도착 성공 |' if sim else '') + ''.join(f' {m[1]} |' for m in metrics)
    lines += [head, '|' + '---|' * (head.count('|') - 1)]
    summary = {}
    for sid in sids:
        for alg in ALGORITHMS:
            rs = [r for r in rows if r['group'] == sid and r['algorithm'] == alg]
            if not rs:
                continue
            ok = sum(int(r['plan_success'] or 0) for r in rs)
            cells = [f'{sid} {names[sid]}', LABEL[alg], str(len(rs)), f'{ok}/{len(rs)}']
            if sim:
                g = sum(int(r['goal_success'] or 0) for r in rs)
                cells.append(f'{g}/{len(rs)}')
            for key, _, nd in metrics:
                m, _, ci = mean_ci([fnum(r.get(key)) for r in rs])
                summary[(sid, alg, key)] = (m, ci)
                cells.append(fmt(m, ci, nd))
            lines.append('| ' + ' | '.join(cells) + ' |')
    lines += ['', '## 2. 짝 비교 (Dijkstra − A*)', '']
    comp = [(k, *_M[k]) for k in spec['compare']]
    head = ('| 시나리오 | 짝 수 |' + ''.join(f' {m[1]} 차이 [95% CI] |' for m in comp)
            + ''.join(f' {_M[k][0].split(" (")[0]} 배율 (D/A) |' for k in spec['ratios']))
    lines += [head, '|' + '---|' * (head.count('|') - 1)]
    for sid in sids:
        rs = [r for r in rows if r['group'] == sid]
        cells = [f'{sid} {names[sid]}', str(len({r['pair'] for r in rs}))]
        for key, _, nd in comp:
            diffs = [d for d, _, _ in paired(rs, key)]
            m, _, ci = mean_ci(diffs)
            cells.append('-' if m is None else (f'{m:+.{nd}f}' + (f' [{m - ci:+.{nd}f}, {m + ci:+.{nd}f}]' if ci else '')))
        for key in spec['ratios']:
            p = paired(rs, key)
            a = np.mean([x for _, x, _ in p]) if p else None
            d = np.mean([y for _, _, y in p]) if p else None
            cells.append('-' if not a else f'×{d / a:.2f}')
        lines.append('| ' + ' | '.join(cells) + ' |')
    if 'planned_distance' in spec['compare']:
        diffs = [abs(d) for d, _, _ in paired(rows, 'planned_distance')]
        if diffs:
            lines += ['', f'- 계획 경로 길이 차이 |D − A|: 평균 {np.mean(diffs):.3f} m, 최대 {max(diffs):.3f} m, '
                          f'1 cm 넘는 짝 {sum(1 for x in diffs if x > 0.01)}/{len(diffs)}']

    same = [abs(d) for d, _, _ in paired(rows, 'path_cost')] if 'path_cost' in spec['compare'] else []
    if same:
        lines += ['', f'- 경로 비용이 같은 짝: {sum(1 for x in same if x < 1e-6)}/{len(same)} '
                      f'(최대 차이 {max(same):.6f}) — 두 알고리즘 모두 최소 비용 경로를 찾는지 확인']
    fails = [r for r in rows if r['failure_reason']]
    if fails:
        lines += ['', '## 3. 실패 기록', '', '| 시나리오 | 방향 | 반복 | 알고리즘 | 이유 |', '|---|---|---|---|---|']
        for r in fails[:60]:
            lines.append(f'| {r["scenario_id"]} | {r["direction"]} | {r["rep"]} | {LABEL[r["algorithm"]]} | '
                         f'{r["failure_reason"].replace("|", "/")} |')
    charts = []
    if not no_plots:
        charts = plot_summary(out, rows, sids, spec, mode)
        if charts:
            lines += ['', '## 그림', ''] + [f'![{c}]({c})' for c in charts]
    path = os.path.join(out, 'summary.md')
    with open(path, 'w', encoding='utf-8') as f:
        f.write('\n'.join(lines) + '\n')
    print(f'[r1] 요약: {path}' + (f', 그림 {", ".join(charts)}' if charts else ''), flush=True)
    start = lines.index('## 2. 짝 비교 (Dijkstra − A*)')
    for ln in lines[start:]:
        if ln.startswith('## 3'):
            break
        if ln.startswith('| '):
            print('   ' + ln, flush=True)


# ---------------------------------------------------------------------------------------------------- charts
# Two series in fixed categorical order (validated palette: slot 1 blue, slot 2 orange), light surface
COLOR = {'astar': '#2a78d6', 'dijkstra': '#eb6834'}
SURFACE, INK, INK2, GRID = '#fcfcfb', '#0b0b0b', '#52514e', '#e4e3df'


def _style(ax):
    ax.set_facecolor(SURFACE)
    for s in ('top', 'right'):
        ax.spines[s].set_visible(False)
    for s in ('left', 'bottom'):
        ax.spines[s].set_color(GRID)
    ax.tick_params(colors=INK2, labelsize=8, length=0)
    ax.yaxis.grid(True, color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)


def plot_summary(out, rows, sids, spec, mode):
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        print('[r1] matplotlib 없음: 그림 생략', flush=True)
        return []
    panels = spec['panels']
    fig, axes = plt.subplots(1, len(panels), figsize=(4.2 * len(panels), 3.6), dpi=150)
    fig.patch.set_facecolor(SURFACE)
    x = np.arange(len(sids))
    bw = 0.36
    for ax, (key, title, lab) in zip(axes, panels):
        _style(ax)
        tops = []
        scale = 100.0 if lab == 'pct' else 1.0
        for j, alg in enumerate(ALGORITHMS):
            ms, cis = [], []
            for sid in sids:
                vals = [fnum(r.get(key)) for r in rows if r['group'] == sid and r['algorithm'] == alg]
                m, _, ci = mean_ci([None if v is None else v * scale for v in vals])
                ms.append(m or 0.0)
                cis.append(0.0 if lab == 'pct' else (ci or 0.0))
            pos = x + (j - 0.5) * (bw + 0.02)
            ax.bar(pos, ms, bw, color=COLOR[alg], label=LABEL[alg], edgecolor=SURFACE, linewidth=1)
            ax.errorbar(pos, ms, yerr=cis, fmt='none', ecolor=INK2, elinewidth=1, capsize=2)
            tops.append(np.asarray(ms) + np.asarray(cis))
        top = np.maximum(*tops)
        for i, sid in enumerate(sids):
            p = paired([r for r in rows if r['group'] == sid], key)
            if not p:
                continue
            a = np.mean([v for _, v, _ in p]) * scale
            d = np.mean([v for _, _, v in p]) * scale
            if lab == 'ratio':
                text = f'x{d / a:.1f}' if a else ''
            elif lab == 'pct':
                text = f'{d - a:+.0f}pp'
            else:
                text = f'{d - a:+.2f}'
            ax.text(i, top[i] + 0.02 * max(top.max(), 1e-9), text, ha='center', va='bottom', fontsize=7.5, color=INK)
        ax.set_xticks(x)
        ax.set_xticklabels([s.replace(' ', '\n') for s in sids], fontsize=7, color=INK2)
        ax.set_title(title, fontsize=9.5, color=INK, loc='left')
        ax.set_ylim(0, max(top.max() * 1.18, 1e-6))
    axes[0].legend(frameon=False, fontsize=8, labelcolor=INK2, loc='upper left')
    note = 'labels: Dijkstra / A* ratio of means (xN) or Dijkstra - A* (+/-) | whiskers: 95% CI of the mean'
    fig.text(0.01, 0.01, note, fontsize=7, color=INK2)
    fig.tight_layout(rect=(0, 0.04, 1, 1))
    name = f'fig_{mode}_{rows[0]["world"]}.png'
    fig.savefig(os.path.join(out, name), facecolor=SURFACE)
    plt.close(fig)
    return [name]


def plot_paths(path, v, grid, res, ox, oy, title):
    """Side by side: expanded cells and path of A* and of Dijkstra on the planning map (same scale)."""
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        from matplotlib.colors import to_rgb
    except ImportError:
        return
    planner = v['planner']
    h, w = planner.height, planner.width
    base = np.ones((h, w, 3)) * np.array(to_rgb(SURFACE))
    base[grid < 0] = to_rgb('#d9d8d3')
    base[planner.lethal & ~planner.obstacle] = to_rgb('#eeede9')
    base[planner.obstacle] = to_rgb(INK2)
    ext = (ox, ox + w * res, oy, oy + h * res)
    fig, axes = plt.subplots(1, 2, figsize=(9.6, 5.0), dpi=150)
    fig.patch.set_facecolor(SURFACE)
    for ax, alg in zip(axes, ALGORITHMS):
        closed, pts, s, err = v['res'][alg]
        img = base.copy()
        if closed is not None:
            mask = np.frombuffer(closed, dtype=np.uint8).reshape(h, w).astype(bool)
            col = np.array(to_rgb(COLOR[alg]))
            img[mask] = img[mask] * 0.45 + col * 0.55
        ax.imshow(img, origin='lower', extent=ext, interpolation='nearest')
        if pts:
            xs, ys = zip(*pts)
            ax.plot(xs, ys, color=INK, linewidth=2)
        a, b = v['start'], v['goal']
        ax.plot(a[0], a[1], 'o', color=INK, markersize=7, markeredgecolor=SURFACE, markeredgewidth=1.5)
        ax.plot(b[0], b[1], 's', color=INK, markersize=7, markeredgecolor=SURFACE, markeredgewidth=1.5)
        info = (f'{s.get("expanded", 0):,} cells expanded, {s.get("search_ms", 0):.1f} ms'
                + (f', {s.get("length"):.2f} m' if pts else ' - no path'))
        ax.set_title(f'{LABEL[alg]}: {info}', fontsize=9, color=INK, loc='left')
        ax.tick_params(colors=INK2, labelsize=7, length=0)
        for sp in ax.spines.values():
            sp.set_color(GRID)
    fig.suptitle(title + '   (o start, square goal; tinted = expanded cells)', fontsize=9, color=INK2, x=0.01,
                 ha='left')
    fig.tight_layout()
    fig.savefig(path, facecolor=SURFACE)
    plt.close(fig)


def plot_overlay(path, v, grid, res, ox, oy, title):
    """Nav2 planning: both planners' paths over the map (NavFn does not report expanded cells)."""
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        from matplotlib.colors import to_rgb
    except ImportError:
        return
    h, w = grid.shape
    img = np.ones((h, w, 3)) * np.array(to_rgb(SURFACE))
    img[grid < 0] = to_rgb('#d9d8d3')
    img[grid >= 65] = to_rgb(INK2)
    for b in v.get('boxes') or ():
        img[box_cells(grid.shape, res, ox, oy, b)] = to_rgb('#a8a49c')
    fig, ax = plt.subplots(figsize=(6.4, 6.0), dpi=150)
    fig.patch.set_facecolor(SURFACE)
    ax.imshow(img, origin='lower', extent=(ox, ox + w * res, oy, oy + h * res), interpolation='nearest')
    for alg in ALGORITHMS:
        pts, pms, err = v['res'].get(alg, (None, None, None))
        if pts:
            xs, ys = zip(*pts)
            ax.plot(xs, ys, color=COLOR[alg], linewidth=2, solid_capstyle='round',
                    label=f'{LABEL[alg]}: {polyline_length(pts, 0.0):.2f} m'
                          + (f', {pms:.1f} ms' if pms is not None else ''))
        else:
            ax.plot([], [], color=COLOR[alg], linewidth=2, label=f'{LABEL[alg]}: no path ({err})')
    a, b = v['start'], v['goal']
    ax.plot(a[0], a[1], 'o', color=INK, markersize=7, markeredgecolor=SURFACE, markeredgewidth=1.5)
    ax.plot(b[0], b[1], 's', color=INK, markersize=7, markeredgecolor=SURFACE, markeredgewidth=1.5)
    ax.legend(frameon=False, fontsize=8, labelcolor=INK2, loc='upper left', bbox_to_anchor=(0, -0.06), ncol=1)
    ax.set_title(title + '  (o start, square goal)', fontsize=9, color=INK, loc='left')
    ax.tick_params(colors=INK2, labelsize=7, length=0)
    for sp in ax.spines.values():
        sp.set_color(GRID)
    fig.tight_layout()
    fig.savefig(path, facecolor=SURFACE)
    plt.close(fig)


# ---------------------------------------------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    for name in ('offline', 'sim', 'nav2plan', 'nav2sim'):
        p = sub.add_parser(name)
        p.add_argument('--world', default='lecture_hall_real')
        p.add_argument('--scenarios', default=SCENARIOS)
        p.add_argument('--only', help='시나리오 id 쉼표 목록 (예: R1-A,R1-D)')
        p.add_argument('--out', help='결과 폴더 (기본 experiments/r1/results/<시각>_<모드>_<월드>)')
        p.add_argument('--no-plots', action='store_true')
    off = sub.choices['offline']
    off.add_argument('--repeat', type=int, default=30)
    off.add_argument('--random', type=int, default=0, help='무작위 출발/목표 쌍 수 (0 = 안 함)')
    off.add_argument('--random-repeat', type=int, default=3)
    off.add_argument('--min-dist', type=float, default=1.0)
    off.add_argument('--seed', type=int, default=1)
    sim = sub.choices['sim']
    sim.add_argument('--repeat', type=int, default=5)
    sim.add_argument('--timeout', type=float, default=150.0, help='시도 하나의 최대 시간 (s)')
    sim.add_argument('--settle', type=float, default=1.0, help='장애물 생성/설정 후 출발 전 대기 (s)')
    sim.add_argument('--server', default='ws://localhost:8000/ws/ui/')
    sim.add_argument('--force', action='store_true', help='실행 중인 맵이 시나리오 맵과 달라도 진행')
    np_ = sub.choices['nav2plan']
    np_.add_argument('--repeat', type=int, default=30)
    np_.add_argument('--random', type=int, default=0, help='무작위 출발/목표 쌍 수 (0 = 안 함)')
    np_.add_argument('--random-repeat', type=int, default=3)
    np_.add_argument('--min-dist', type=float, default=1.0)
    np_.add_argument('--seed', type=int, default=1)
    np_.add_argument('--settle', type=float, default=3.0, help='맵을 바꾼 뒤 costmap 이 갱신되기를 기다림 (s)')
    np_.add_argument('--core-repeat', type=int, default=100, help='NavFn 핵심 계산(C++) 반복 수 (시나리오·방향마다)')
    np_.add_argument('--inflation', help='global costmap 인플레이션 "반경,cost_scaling_factor" (예: 0.8,2.0). 기본은 설정 파일값')
    np_.add_argument('--no-core', action='store_true', help='NavFn 핵심 계산(C++) 생략')
    np_.add_argument('--no-restart', action='store_true',
                     help='시작할 때 Nav2 navigation 노드를 재시작하지 않음 (기본은 재시작: Humble 액션 서버는 끝난 목표를 '
                          '15분 보관해 요청이 쌓일수록 응답이 느려짐)')
    np_.add_argument('--astar-planner', choices=CANDIDATES, default='AStar',
                     help='비교 후보 (기준은 항상 NavFn Dijkstra): AStar = NavFn use_astar true (기본), Smac2D, ThetaStar, '
                          'SmacLattice')
    np_.add_argument('--force', action='store_true', help='Nav2 실행/맵 확인 없이 진행')
    ns = sub.choices['nav2sim']
    ns.add_argument('--repeat', type=int, default=5)
    ns.add_argument('--timeout', type=float, default=180.0, help='시도 하나의 최대 시간 (s)')
    ns.add_argument('--settle', type=float, default=1.0, help='장애물 생성/costmap 지운 뒤 출발 전 대기 (s)')
    ns.add_argument('--force', action='store_true', help='Nav2 실행/맵 확인 없이 진행')
    ns.add_argument('--inflation', help='global costmap 인플레이션 "반경,cost_scaling_factor" (예: 0.8,2.0). 기본은 설정 파일값')
    ns.add_argument('--real', action='store_true',
                    help='실차 (run_nav2_real.sh): 위치·거리는 AMCL, 장애물은 사람이 놓음, 시도마다 Enter 로 진행')
    ns.add_argument('--yes', action='store_true', help='--real 에서 시도마다 묻지 않음 (장애물 없는 시나리오만 권장)')
    ns.add_argument('--astar-planner', choices=CANDIDATES, default='AStar',
                    help='비교 후보 (기준은 항상 NavFn Dijkstra): AStar = NavFn use_astar true (기본), Smac2D, ThetaStar, '
                         'SmacLattice')
    rep = sub.add_parser('report')
    rep.add_argument('dirs', nargs='+')
    rep.add_argument('--out')
    rep.add_argument('--no-plots', action='store_true')
    args = ap.parse_args()
    if args.cmd == 'offline':
        run_offline(args)
    elif args.cmd == 'sim':
        run_sim(args)
    elif args.cmd == 'nav2plan':
        run_nav2plan(args)
    elif args.cmd == 'nav2sim':
        run_nav2sim(args)
    else:
        report(args.dirs, no_plots=args.no_plots, out=args.out)


if __name__ == '__main__':
    main()

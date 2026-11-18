#!/usr/bin/env python3
"""Compare NavFn A* and Dijkstra on fixed start/goal pairs through a live Nav2 planner_server.

Every request is ComputePathToPose with an explicit start pose (use_start: true) and a planner_id taken from the
planner params file. Before measuring, the script checks that map_server serves the scenario map and that
planner_server runs exactly the NavFn parameters of the params file; it refuses to measure otherwise.

    ros2 launch experiments/path_planner/launch/planner_bench.launch.py map:=<map.yaml>
    python3 experiments/path_planner/scripts/run_compare.py \
        --scenarios experiments/path_planner/scenarios/lecture_hall_v1.yaml --run-mode SIM --repeat 10
"""

import argparse
import math
import os
import platform
import subprocess
import sys
import time
from datetime import datetime, timezone

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import bench_core as core  # noqa: E402

BENCH_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_PARAMS = os.path.join(BENCH_DIR, 'config', 'navfn_compare.yaml')
DEFAULT_OUT = os.path.join(BENCH_DIR, 'results')
CHECKED_KEYS = ('plugin', 'use_astar', 'tolerance', 'allow_unknown')


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--scenarios', required=True, help='scenario yaml (experiments/path_planner/scenarios/)')
    p.add_argument('--params', default=DEFAULT_PARAMS, help='planner_server params given to the launch file')
    p.add_argument('--run-mode', required=True, choices=core.RUN_MODES, help='MOCK / SIM / REAL label of this run')
    p.add_argument('--repeat', type=int, default=10, help='measured repetitions per run and planner')
    p.add_argument('--warmup', type=int, default=1, help='unsummarized repetitions before measuring')
    p.add_argument('--out', default=DEFAULT_OUT, help='results root; a new <experiment_id>/ is created inside')
    p.add_argument('--action', default='compute_path_to_pose')
    p.add_argument('--planner-node', default='planner_server')
    p.add_argument('--map-topic', default='map')
    p.add_argument('--server-timeout', type=float, default=30.0, help='s to wait for action server, map, params')
    p.add_argument('--plan-timeout', type=float, default=30.0, help='s to wait for one planning result')
    p.add_argument('--git-commit', default='', help='repo commit to record (auto-detected when git works)')
    p.add_argument('--note', default='', help='free text recorded in meta.json, e.g. the environment profile')
    return p.parse_args(argv)


def spin_until(node, future, timeout):
    import rclpy
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    return future.done()


class Nav2Planner:
    """plan_fn for bench_core.run_trials backed by the ComputePathToPose action."""

    def __init__(self, node, action_name, frame_id, plan_timeout):
        from nav2_msgs.action import ComputePathToPose
        from rclpy.action import ActionClient
        self.node, self.frame_id, self.timeout = node, frame_id, plan_timeout
        self.action = ComputePathToPose
        self.client = ActionClient(node, ComputePathToPose, action_name)
        self.error_names = core.error_table(ComputePathToPose.Result)

    def pose(self, p):
        from geometry_msgs.msg import PoseStamped
        msg = PoseStamped()
        msg.header.frame_id = self.frame_id
        msg.pose.position.x, msg.pose.position.y = p.x, p.y
        q = msg.pose.orientation
        q.x, q.y, q.z, q.w = core.yaw_quaternion(p.yaw_deg)
        return msg

    def __call__(self, planner_id, start, goal):
        from action_msgs.msg import GoalStatus
        req = self.action.Goal()
        req.goal, req.start = self.pose(goal), self.pose(start)
        req.planner_id, req.use_start = planner_id, True
        t0 = time.perf_counter()

        def elapsed():
            return (time.perf_counter() - t0) * 1000.0

        sent = self.client.send_goal_async(req)
        if not spin_until(self.node, sent, self.timeout):
            return core.PlanOutcome(False, roundtrip_ms=elapsed(), failure_reason='CLIENT_TIMEOUT_GOAL_RESPONSE')
        handle = sent.result()
        if not handle.accepted:
            return core.PlanOutcome(False, roundtrip_ms=elapsed(), failure_reason='GOAL_REJECTED')
        done = handle.get_result_async()
        if not spin_until(self.node, done, self.timeout):
            handle.cancel_goal_async()
            return core.PlanOutcome(False, roundtrip_ms=elapsed(), failure_reason='CLIENT_TIMEOUT_RESULT')
        roundtrip = elapsed()
        status, res = done.result().status, done.result().result
        code = int(res.error_code)
        planning_ms = res.planning_time.sec * 1e3 + res.planning_time.nanosec / 1e6
        ok = status == GoalStatus.STATUS_SUCCEEDED and code == 0
        name = self.error_names.get(code, f'CODE_{code}')
        reason = '' if ok else (name if code else f'STATUS_{status}')
        return core.PlanOutcome(
            ok, [(ps.pose.position.x, ps.pose.position.y) for ps in res.path.poses],
            planning_ms if planning_ms > 0 else None, roundtrip, code, name, reason, getattr(res, 'error_msg', ''))


def check_map(node, scenarios, topic, timeout):
    """Wait for the latched map and compare its size, resolution and origin with the scenario map file."""
    from nav_msgs.msg import OccupancyGrid
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    got = []
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
    sub = node.create_subscription(OccupancyGrid, topic, got.append, qos)
    end = time.monotonic() + timeout
    import rclpy
    while not got and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    if not got:
        raise RuntimeError(f'no map on /{topic} within {timeout} s')
    info, want = got[-1].info, core.map_info(scenarios.map_yaml)
    loaded = {'width': info.width, 'height': info.height, 'resolution': round(info.resolution, 6),
              'origin': [round(info.origin.position.x, 6), round(info.origin.position.y, 6)],
              'frame_id': got[-1].header.frame_id}
    same = (loaded['width'], loaded['height']) == (want['width'], want['height']) \
        and math.isclose(loaded['resolution'], want['resolution'], abs_tol=1e-6) \
        and all(math.isclose(a, b, abs_tol=1e-6) for a, b in zip(loaded['origin'], want['origin']))
    if not same or loaded['frame_id'] != scenarios.frame_id:
        raise RuntimeError(f'map_server serves {loaded}, scenario map is {want} in frame {scenarios.frame_id}')
    return loaded


def check_planner_params(node, planner_node, configs, timeout):
    """Read the NavFn parameters planner_server actually runs and require them to match the params file."""
    from rclpy.parameter import parameter_value_to_python
    from rclpy.parameter_client import AsyncParameterClient
    client = AsyncParameterClient(node, planner_node)
    if not client.wait_for_services(timeout_sec=timeout):
        raise RuntimeError(f'parameter services of /{planner_node} not available')
    names = ['planner_plugins'] + [f'{c.planner_id}.{k}' for c in configs for k in CHECKED_KEYS]
    future = client.get_parameters(names)
    if not spin_until(node, future, timeout):
        raise RuntimeError(f'get_parameters on /{planner_node} timed out')
    actual = {n: parameter_value_to_python(v) for n, v in zip(names, future.result().values)}
    expected = {'planner_plugins': [c.planner_id for c in configs]}
    expected.update({f'{c.planner_id}.{k}': c.params.get(k) for c in configs for k in CHECKED_KEYS})
    wrong = {}
    for n, want in expected.items():
        have = actual.get(n)
        if have is None and want is None:
            continue
        if isinstance(want, float) or isinstance(have, float):
            ok = have is not None and want is not None and math.isclose(float(have), float(want))
        else:
            ok = (list(have) if isinstance(have, (list, tuple)) else have) == want
        if not ok:
            wrong[n] = {'expected': want, 'actual': have}
    if wrong:
        raise RuntimeError(f'planner_server parameters differ from the params file: {wrong}')
    return actual


def package_version(name):
    try:
        import xml.etree.ElementTree as ET
        from ament_index_python.packages import get_package_share_directory
        return ET.parse(os.path.join(get_package_share_directory(name), 'package.xml')).findtext('version')
    except Exception:
        return None


def git_commit(explicit):
    if explicit:
        return explicit
    try:
        return subprocess.run(['git', 'rev-parse', 'HEAD'], cwd=BENCH_DIR, capture_output=True, text=True,
                              check=True, timeout=10).stdout.strip()
    except Exception:
        return None


def main(argv=None):
    args = parse_args(argv)
    scenarios = core.load_scenarios(args.scenarios)
    configs = core.load_planner_configs(args.params)
    core.check_comparison_pair(configs)
    versions = {'config_version': core.config_version(args.params),
                'map_version': core.map_version(scenarios.map_name, scenarios.map_yaml),
                'scenario_version': core.scenario_version(scenarios.path)}
    started = datetime.now(timezone.utc)
    exp_id = core.experiment_id(started, args.run_mode, scenarios.name)

    import rclpy
    rclpy.init()
    node = rclpy.create_node('path_planner_bench')
    try:
        planner = Nav2Planner(node, args.action, scenarios.frame_id, args.plan_timeout)
        if not planner.client.wait_for_server(timeout_sec=args.server_timeout):
            raise SystemExit(f'ComputePathToPose server /{args.action} not available; start the launch file first')
        loaded_map = check_map(node, scenarios, args.map_topic, args.server_timeout)
        server_params = check_planner_params(node, args.planner_node, configs, args.server_timeout)
        print(f'{exp_id}: {len(scenarios.runs)} runs x {len(configs)} planners x ({args.warmup} warmup + '
              f'{args.repeat}) on {versions["map_version"]}, {versions["config_version"]}', flush=True)
        rows = core.run_trials(scenarios, configs, planner, repeat=args.repeat, warmup=args.warmup,
                               run_mode=args.run_mode, experiment_id=exp_id, **versions)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    summary = core.summarize(rows)
    meta = {
        'experiment_id': exp_id, 'run_mode': args.run_mode, 'note': args.note, **versions,
        'started': core.utc_stamp(started), 'finished': core.utc_stamp(),
        'repeat': args.repeat, 'warmup': args.warmup, 'trials': len(rows),
        'scenario_file': os.path.relpath(scenarios.path, BENCH_DIR).replace(os.sep, '/'),
        'params_file': os.path.relpath(os.path.abspath(args.params), BENCH_DIR).replace(os.sep, '/'),
        'map_file': os.path.relpath(scenarios.map_yaml, BENCH_DIR).replace(os.sep, '/'),
        'frame_id': scenarios.frame_id, 'loaded_map': loaded_map, 'planner_server_params': server_params,
        'planners': [{'planner_id': c.planner_id, 'algorithm': c.algorithm, 'params': c.params} for c in configs],
        'request': {'action': args.action, 'use_start': True},
        'env': {'ros_distro': os.environ.get('ROS_DISTRO'), 'nav2_planner': package_version('nav2_planner'),
                'nav2_navfn_planner': package_version('nav2_navfn_planner'),
                'bench_env': os.environ.get('BENCH_ENV'), 'python': platform.python_version(),
                'platform': platform.platform(), 'cpu_count': os.cpu_count(), 'git_commit': git_commit(args.git_commit)}}
    out = core.write_results(os.path.join(args.out, exp_id), rows, summary, meta)
    print(core.render_summary_md(summary, meta))
    missed = [s['scenario_id'] + '/' + s['planner_id'] for s in summary if s['expectation_met'] != s['trials']]
    if missed:
        print('WARNING: result differs from expectation for ' + ', '.join(missed), flush=True)
    print(f'results: {out}')


if __name__ == '__main__':
    main()

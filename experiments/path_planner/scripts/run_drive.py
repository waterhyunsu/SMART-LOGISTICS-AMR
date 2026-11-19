#!/usr/bin/env python3
"""Drive the fixed start/goal pairs in Gazebo with NavFn A* and Dijkstra and record time, distance and success.

One Nav2 stack runs for the whole experiment (launch/drive_bench.launch.py). Before every trial the script sets
``<planner>.use_astar`` on planner_server (and reads it back), puts the robot on the start pose, re-initialises
AMCL there and clears both costmaps, so the two planners start from the same state. Pairs with obstacles are also
driven with the boxes spawned in the world but not in the map: the robot meets them on the way and has to replan.

    ros2 launch experiments/path_planner/launch/drive_bench.launch.py world:=<world.sdf> map:=<map.yaml>
    python3 experiments/path_planner/scripts/run_drive.py \
        --scenarios experiments/path_planner/scenarios/lecture_hall_v1.yaml --run-mode SIM --repeat 5 --only R1-
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
import drive_core as drive  # noqa: E402
from run_compare import check_map, git_commit, package_version, spin_until  # noqa: E402

BENCH_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_PARAMS = os.path.join(BENCH_DIR, 'config', 'nav2_drive.yaml')
DEFAULT_OUT = os.path.join(BENCH_DIR, 'results')
BOX_HEIGHT = 0.5


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--scenarios', required=True)
    p.add_argument('--params', default=DEFAULT_PARAMS, help='Nav2 params given to the launch file (for versioning)')
    p.add_argument('--run-mode', required=True, choices=('SIM',), help='this script only drives in simulation')
    p.add_argument('--repeat', type=int, default=5, help='trials per run and planner (ADR-012: 5 or more)')
    p.add_argument('--only', default='', help='comma separated run id prefixes to drive, e.g. R1-A,R1-B')
    p.add_argument('--out', default=DEFAULT_OUT)
    p.add_argument('--world', default='lecture_hall_real', help='Gazebo world name')
    p.add_argument('--robot', default='waffle_pi', help='Gazebo model name of the robot')
    p.add_argument('--planner', default='GridBased', help='planner plugin id whose use_astar is switched')
    p.add_argument('--timeout', type=float, default=180.0, help='simulation s allowed for one drive')
    p.add_argument('--server-timeout', type=float, default=120.0, help='wall s to wait for Nav2 to come up')
    p.add_argument('--git-commit', default='')
    p.add_argument('--note', default='')
    return p.parse_args(argv)


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def gz_service(world, name, reqtype, req):
    out = subprocess.run(['gz', 'service', '-s', f'/world/{world}/{name}', '--reqtype', reqtype, '--reptype',
                          'gz.msgs.Boolean', '--timeout', '5000', '--req', req],
                         capture_output=True, text=True, timeout=20)
    if 'data: true' not in out.stdout:
        raise RuntimeError(f'gz service {name} failed: {out.stdout.strip()} {out.stderr.strip()}')


class Driver:
    def __init__(self, node, args, frame_id):
        from geometry_msgs.msg import PoseWithCovarianceStamped
        from nav2_msgs.action import NavigateToPose
        from nav2_msgs.srv import ClearEntireCostmap
        from nav_msgs.msg import Odometry, Path
        from rcl_interfaces.srv import GetParameters, SetParameters
        from rclpy.action import ActionClient
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        self.node, self.args, self.frame_id = node, args, frame_id
        self.NavigateToPose, self.PoseCov = NavigateToPose, PoseWithCovarianceStamped
        self.client = ActionClient(node, NavigateToPose, 'navigate_to_pose')
        self.set_params = node.create_client(SetParameters, 'planner_server/set_parameters')
        self.get_params = {n: node.create_client(GetParameters, f'{n}/get_parameters')
                           for n in ('planner_server', 'controller_server')}
        self.clear = [node.create_client(ClearEntireCostmap, f'{c}_costmap/clear_entirely_{c}_costmap')
                      for c in ('global', 'local')]
        self.ClearEntireCostmap = ClearEntireCostmap
        self.initial_pub = node.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        node.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self.on_amcl, latched)
        node.create_subscription(Odometry, 'odom', self.on_odom, 20)
        node.create_subscription(Path, 'plan', self.on_plan, 10)
        self.amcl = self.odom = None
        self.tracking, self.distance, self.plans, self.recoveries = False, 0.0, [], 0

    # --- callbacks
    def on_amcl(self, msg):
        p = msg.pose.pose
        self.amcl = (p.position.x, p.position.y, yaw_of(p.orientation))

    def on_odom(self, msg):
        p = msg.pose.pose
        pose = (p.position.x, p.position.y, yaw_of(p.orientation))
        if self.tracking and self.odom is not None:
            self.distance += math.dist(pose[:2], self.odom[:2])
        self.odom = pose

    def on_plan(self, msg):
        if self.tracking:
            self.plans.append(core.path_length([(ps.pose.position.x, ps.pose.position.y) for ps in msg.poses]))

    def on_feedback(self, msg):
        self.recoveries = max(self.recoveries, int(msg.feedback.number_of_recoveries))

    # --- helpers
    def sim_now(self):
        return self.node.get_clock().now().nanoseconds / 1e9

    def spin(self, seconds):
        import rclpy
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def call(self, client, request, what):
        if not client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError(f'{what}: service not available')
        future = client.call_async(request)
        if not spin_until(self.node, future, 15.0):
            raise RuntimeError(f'{what}: no answer')
        return future.result()

    def get_param(self, node_name, name):
        from rcl_interfaces.srv import GetParameters
        from rclpy.parameter import parameter_value_to_python
        res = self.call(self.get_params[node_name], GetParameters.Request(names=[name]), f'get {name}')
        return parameter_value_to_python(res.values[0])

    def set_algorithm(self, use_astar):
        from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
        from rcl_interfaces.srv import SetParameters
        name = f'{self.args.planner}.use_astar'
        value = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=use_astar)
        res = self.call(self.set_params, SetParameters.Request(parameters=[Parameter(name=name, value=value)]), name)
        if not res.results[0].successful or self.get_param('planner_server', name) is not use_astar:
            raise RuntimeError(f'planner_server did not take {name}={use_astar}: {res.results[0].reason}')

    def place_robot(self, pose):
        yaw = math.radians(pose.yaw_deg)
        gz_service(self.args.world, 'set_pose', 'gz.msgs.Pose',
                   f'name: "{self.args.robot}", position: {{x: {pose.x}, y: {pose.y}, z: 0.01}}, '
                   f'orientation: {{z: {math.sin(yaw / 2)}, w: {math.cos(yaw / 2)}}}')
        self.spin(1.0)

    def localize(self, pose):
        """Give AMCL the start pose and wait until its estimate is there."""
        yaw = math.radians(pose.yaw_deg)
        msg = self.PoseCov()
        msg.header.frame_id = self.frame_id
        msg.pose.pose.position.x, msg.pose.pose.position.y = pose.x, pose.y
        q = msg.pose.pose.orientation
        q.x, q.y, q.z, q.w = core.yaw_quaternion(pose.yaw_deg)
        msg.pose.covariance[0] = msg.pose.covariance[7] = 0.02
        msg.pose.covariance[35] = 0.02
        end = time.monotonic() + 30.0
        while time.monotonic() < end:
            self.amcl = None
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self.initial_pub.publish(msg)
            self.spin(1.5)
            if self.amcl is not None and math.dist(self.amcl[:2], (pose.x, pose.y)) < 0.20 \
                    and abs((self.amcl[2] - yaw + math.pi) % (2 * math.pi) - math.pi) < 0.30:
                return
        raise RuntimeError(f'AMCL did not settle on the start pose {pose}, last estimate {self.amcl}')

    def clear_costmaps(self):
        for client in self.clear:
            self.call(client, self.ClearEntireCostmap.Request(), 'clear costmap')
        self.spin(3.0)       # let the static and obstacle layers fill again

    def spawn_boxes(self, boxes):
        names = []
        for i, b in enumerate(boxes):
            name = f'r1d_box_{i}'
            size = f'{b.size_x} {b.size_y} {BOX_HEIGHT}'
            sdf = (f'<sdf version="1.8"><model name="{name}"><static>true</static><link name="link">'
                   f'<collision name="c"><geometry><box><size>{size}</size></box></geometry></collision>'
                   f'<visual name="v"><geometry><box><size>{size}</size></box></geometry><material>'
                   f'<ambient>0.85 0.35 0.1 1</ambient><diffuse>0.85 0.35 0.1 1</diffuse></material></visual>'
                   f'</link></model></sdf>')
            out = subprocess.run(['ros2', 'run', 'ros_gz_sim', 'create', '-world', self.args.world, '-name', name,
                                  '-string', sdf, '-x', str(b.cx), '-y', str(b.cy), '-z', str(BOX_HEIGHT / 2),
                                  '-Y', str(math.radians(b.yaw_deg))], capture_output=True, text=True, timeout=30)
            if out.returncode != 0:
                raise RuntimeError(f'could not spawn {name}: {out.stderr.strip()[-200:]}')
            names.append(name)
        return names

    def remove_boxes(self, names):
        for name in names:
            gz_service(self.args.world, 'remove', 'gz.msgs.Entity', f'name: "{name}", type: MODEL')

    def drive(self, run):
        """One NavigateToPose from the run's start (the robot is already there) to its goal."""
        from action_msgs.msg import GoalStatus
        import rclpy
        goal = self.NavigateToPose.Goal()
        goal.pose.header.frame_id = self.frame_id
        goal.pose.pose.position.x, goal.pose.pose.position.y = run.goal.x, run.goal.y
        q = goal.pose.pose.orientation
        q.x, q.y, q.z, q.w = core.yaw_quaternion(run.goal.yaw_deg)
        self.distance, self.plans, self.recoveries = 0.0, [], 0
        odom_start, t0 = self.odom, self.sim_now()
        self.tracking = True
        try:
            goal.pose.header.stamp = self.node.get_clock().now().to_msg()
            sent = self.client.send_goal_async(goal, feedback_callback=self.on_feedback)
            if not spin_until(self.node, sent, 20.0) or not sent.result().accepted:
                return {'goal_success': False, 'failure_reason': 'GOAL_REJECTED', 'execution_time': ''}
            handle, done = sent.result(), sent.result().get_result_async()
            timed_out = False
            while not done.done():
                rclpy.spin_once(self.node, timeout_sec=0.05)
                if not timed_out and self.sim_now() - t0 > self.args.timeout:
                    timed_out = True
                    handle.cancel_goal_async()
            elapsed = self.sim_now() - t0
        finally:
            self.tracking = False
        status, result = done.result().status, done.result().result
        ok = status == GoalStatus.STATUS_SUCCEEDED and not timed_out
        code = int(getattr(result, 'error_code', 0))
        start = (run.start.x, run.start.y, math.radians(run.start.yaw_deg))
        return {
            'goal_success': ok, 'timeout': timed_out, 'nav_status': status, 'error_code': code,
            'failure_reason': '' if ok else ('TIMEOUT' if timed_out else f'NAV_STATUS_{status}_CODE_{code}'),
            'execution_time': round(elapsed, 2), 'actual_distance': round(self.distance, 3),
            'planned_distance': round(self.plans[0], 4) if self.plans else '',
            'last_plan_distance': round(self.plans[-1], 4) if self.plans else '', 'plan_count': len(self.plans),
            'recoveries': self.recoveries,
            'final_error': round(drive.relative_pose_error(start, odom_start, self.odom, (run.goal.x, run.goal.y)), 3)}


def main(argv=None):
    args = parse_args(argv)
    scenarios = core.load_scenarios(args.scenarios)
    prefixes = [p for p in args.only.split(',') if p]
    runs = [r for r in scenarios.runs if not prefixes or any(r.run_id.startswith(p) for p in prefixes)]
    if not runs:
        raise SystemExit(f'--only {args.only!r} selects no run of {args.scenarios}')
    versions = {'config_version': core.config_version(args.params),
                'map_version': core.map_version(scenarios.map_name, scenarios.map_yaml),
                'scenario_version': core.scenario_version(scenarios.path)}
    started = datetime.now(timezone.utc)
    exp_id = core.experiment_id(started, args.run_mode, scenarios.name + '_drive')

    import rclpy
    from rclpy.parameter import Parameter
    rclpy.init()
    node = rclpy.create_node('path_planner_drive', parameter_overrides=[Parameter('use_sim_time', value=True)])
    rows, aborted = [], ''
    try:
        d = Driver(node, args, scenarios.frame_id)
        if not d.client.wait_for_server(timeout_sec=args.server_timeout):
            raise SystemExit('navigate_to_pose not available; start launch/drive_bench.launch.py first')
        loaded_map = check_map(node, scenarios, 'map', args.server_timeout)
        end = time.monotonic() + args.server_timeout
        while (d.odom is None or d.sim_now() <= 0.0) and time.monotonic() < end:
            d.spin(0.5)
        if d.odom is None:
            raise SystemExit('no /odom from the simulation')
        planner_plugin = d.get_param('planner_server', f'{args.planner}.plugin')
        controller_id = d.get_param('controller_server', 'FollowPath.plugin')
        if not str(planner_plugin).startswith('nav2_navfn_planner'):
            raise SystemExit(f'{args.planner} is {planner_plugin}, not NavFn')
        algorithms = [('astar', True), ('dijkstra', False)]
        print(f'{exp_id}: {len(runs)} runs x 2 planners x {args.repeat} on {versions["map_version"]}', flush=True)
        for k in range(args.repeat):
            ordered = algorithms if k % 2 == 0 else algorithms[::-1]
            for run in runs:
                for order, (algorithm, use_astar) in enumerate(ordered, 1):
                    boxes = []
                    try:
                        d.set_algorithm(use_astar)
                        d.place_robot(run.start)
                        if run.map_variant:
                            boxes = d.spawn_boxes(run.obstacles)
                        d.localize(run.start)
                        d.clear_costmaps()
                        out = d.drive(run)
                    finally:
                        d.remove_boxes(boxes)
                    row = {f: '' for f in drive.DRIVE_FIELDS}
                    row.update(out)
                    row.update({
                        'experiment_id': exp_id, 'scenario_id': run.run_id, 'run_mode': args.run_mode,
                        'algorithm': algorithm, 'planner_id': args.planner, 'controller_id': controller_id,
                        'map_version': versions['map_version'], 'config_version': versions['config_version'],
                        'scenario_set': scenarios.name, 'scenario_version': versions['scenario_version'],
                        'pair_id': run.pair_id, 'direction': run.direction, 'map_variant': run.map_variant,
                        'repetition': k + 1, 'order': order, 'expected': run.expect,
                        'expectation_met': out['goal_success'] == (run.expect == 'success'),
                        'start_x': run.start.x, 'start_y': run.start.y, 'start_yaw_deg': run.start.yaw_deg,
                        'goal_x': run.goal.x, 'goal_y': run.goal.y, 'goal_yaw_deg': run.goal.yaw_deg,
                        'stamp': core.utc_stamp()})
                    rows.append(row)
                    print(f"[{len(rows)}/{len(runs) * 2 * args.repeat}] rep {k + 1} {run.run_id} {algorithm}: "
                          f"{'arrived' if out['goal_success'] else out['failure_reason']} "
                          f"{out.get('execution_time')} s, {out.get('actual_distance')} m, "
                          f"first plan {out.get('planned_distance')} m, recoveries {out.get('recoveries')}", flush=True)
    except (RuntimeError, KeyboardInterrupt) as e:      # keep what was measured and say why it stopped
        aborted = f'{type(e).__name__}: {e}'
        print(f'ABORTED: {aborted}', flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    if not rows:
        raise SystemExit(f'no trial finished ({aborted})')

    gz = subprocess.run(['gz', 'sim', '--versions'], capture_output=True, text=True).stdout.split()
    summary = drive.summarize(rows)
    meta = {
        'experiment_id': exp_id, 'run_mode': args.run_mode, 'note': args.note, **versions,
        'started': core.utc_stamp(started), 'finished': core.utc_stamp(), 'repeat': args.repeat,
        'trials': len(rows), 'planned_trials': len(runs) * 2 * args.repeat, 'aborted': aborted,
        'runs': [r.run_id for r in runs], 'timeout_s': args.timeout, 'world': args.world,
        'scenario_file': os.path.relpath(scenarios.path, BENCH_DIR).replace(os.sep, '/'),
        'params_file': os.path.relpath(os.path.abspath(args.params), BENCH_DIR).replace(os.sep, '/'),
        'map_file': os.path.relpath(scenarios.map_yaml, BENCH_DIR).replace(os.sep, '/'),
        'loaded_map': loaded_map, 'planner_id': args.planner, 'planner_plugin': planner_plugin,
        'controller_id': controller_id,
        'procedure': 'per trial: set use_astar and read it back, set_pose to the start, spawn boxes (blocked runs), '
                     'initialpose until AMCL is on the start, clear both costmaps, NavigateToPose',
        'env': {'ros_distro': os.environ.get('ROS_DISTRO'), 'nav2_bringup': package_version('nav2_bringup'),
                'nav2_navfn_planner': package_version('nav2_navfn_planner'),
                'turtlebot3_gazebo': package_version('turtlebot3_gazebo'), 'gz_sim': gz[0] if gz else None,
                'bench_env': os.environ.get('BENCH_ENV'), 'python': platform.python_version(),
                'platform': platform.platform(), 'cpu_count': os.cpu_count(), 'git_commit': git_commit(args.git_commit)}}
    out_dir = drive.write_results(os.path.join(args.out, exp_id), rows, summary, meta)
    print(drive.render_summary_md(summary, meta))
    print(f'results: {out_dir}')


if __name__ == '__main__':
    main()

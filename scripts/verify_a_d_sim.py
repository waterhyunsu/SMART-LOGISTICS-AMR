#!/usr/bin/env python3
"""Run a single PR #14 navigation goal in an isolated Gazebo SIM."""
import argparse
from contextlib import ExitStack, contextmanager
from dataclasses import dataclass
import json
from math import cos, hypot, isclose, isfinite, sin
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
from tempfile import gettempdir, mkdtemp
from time import monotonic
from uuid import uuid4


ROOT = Path(__file__).resolve().parents[1]
PR14_COMMIT = '8158b65678a9f8baf895ccd0ef8b43a6fdb34a7f'
SEND_LINE = re.compile(r'\[SEND\] ([0-9a-f]{32}) → waypoint (\S+) \(([^()]*)\)\s*$')
NAV2_SUCCEEDED = 4  # action_msgs/msg/GoalStatus.STATUS_SUCCEEDED


@dataclass(frozen=True)
class Config:
    sim_command: list[str]
    d_command: list[str] | None
    map_yaml: Path
    ros_domain_id: int
    initial_pose: tuple[float, float, float]
    waypoint_id: str
    waypoint_pose: tuple[float, float, float]
    base_frame: str
    tolerance_m: float
    startup_timeout_sec: float
    goal_timeout_sec: float


@dataclass(frozen=True)
class GoalEvidence:
    a_uuid: str
    d_log: str
    nav2_statuses: dict[str, int]
    waiting_confirmation: bool
    final_position: tuple[float, float]


def number(value, label):
    if type(value) not in (int, float) or not isfinite(value):
        raise ValueError(f'{label} must be a finite number')
    return float(value)


def positive(value, label):
    value = number(value, label)
    if value <= 0:
        raise ValueError(f'{label} must be positive')
    return value


def pose(value, label):
    if not isinstance(value, list) or len(value) != 3:
        raise ValueError(f'{label} must be [x, y, yaw]')
    return tuple(number(v, label) for v in value)


def command(value, label, replacements):
    if not isinstance(value, list) or not value or any(
            not isinstance(arg, str) or not arg.strip() for arg in value):
        raise ValueError(f'{label} must be a nonempty array of command arguments')
    for placeholder, replacement in replacements.items():
        value = [arg.replace(placeholder, replacement) for arg in value]
    return value


def load_config(path):
    path = path.resolve()
    settings = json.loads(path.read_text())
    if not isinstance(settings, dict):
        raise ValueError('config must be a JSON object')
    domain = settings['ros_domain_id']
    if type(domain) is not int or not 1 <= domain <= 101:
        raise ValueError('ros_domain_id must be an integer from 1 to 101')
    map_yaml = (path.parent / Path(settings['map_yaml']).expanduser()).resolve()
    if not map_yaml.is_file():
        raise ValueError(f'map YAML not found: {map_yaml}')
    waypoint = settings['waypoint']
    waypoint_id = waypoint['id']
    if not isinstance(waypoint_id, str) or not re.fullmatch(r'[A-Za-z0-9_-]+', waypoint_id):
        raise ValueError('waypoint.id must contain only letters, digits, underscores or hyphens')
    if waypoint_id == 'HOME':
        raise ValueError('choose a non-HOME Waypoint for the pickup goal')
    replacements = {'{map_yaml}': str(map_yaml), '{config_dir}': str(path.parent)}
    sim_command = command(settings['sim_command'], 'sim_command', replacements)
    if not any('{map_yaml}' in arg for arg in settings['sim_command']):
        raise ValueError('sim_command must pass {map_yaml} to the SIM launch')
    d_command = settings.get('d_command')
    if d_command is not None:
        d_command = command(d_command, 'd_command', replacements)
    base_frame = settings.get('base_frame', 'base_link')
    if not isinstance(base_frame, str) or not base_frame or base_frame.startswith('/'):
        raise ValueError('base_frame must be a nonempty TF frame without a leading slash')
    start = pose(settings['initial_pose'], 'initial_pose')
    target = pose(waypoint['pose'], 'waypoint.pose')
    tolerance = positive(settings.get('tolerance_m', 0.25), 'tolerance_m')
    if hypot(start[0] - target[0], start[1] - target[1]) <= tolerance:
        raise ValueError('initial_pose must be outside the Waypoint goal tolerance')
    return Config(sim_command, d_command, map_yaml, domain, start, waypoint_id, target,
                  base_frame, tolerance,
                  positive(settings.get('startup_timeout_sec', 90), 'startup_timeout_sec'),
                  positive(settings.get('goal_timeout_sec', 120), 'goal_timeout_sec'))


def verify_evidence(evidence, waypoint_id, expected_pose, tolerance):
    sends = [line for line in evidence.d_log.splitlines() if '[SEND]' in line]
    if len(sends) != 1:
        raise ValueError(f'expected exactly one D [SEND], observed {len(sends)}')
    match = SEND_LINE.search(sends[0])
    if not match or match[1] != evidence.a_uuid:
        raise ValueError('D [SEND] UUID is missing, invalid or different from A')
    sent_pose = tuple(float(part) for part in match[3].split(','))
    if match[2] != waypoint_id or len(sent_pose) != 3 or any(
            not isclose(a, b, rel_tol=0, abs_tol=1e-6)
            for a, b in zip(sent_pose, expected_pose)):
        raise ValueError('D sent a different Waypoint ID or pose')
    if evidence.nav2_statuses != {evidence.a_uuid: NAV2_SUCCEEDED}:
        raise ValueError(f'Nav2 expected only A UUID with SUCCEEDED, got {evidence.nav2_statuses}')
    if not evidence.waiting_confirmation:
        raise ValueError('A did not keep the dispatched TaskStep in waiting_confirmation')
    error = hypot(*(a - b for a, b in zip(evidence.final_position, expected_pose[:2])))
    if not isfinite(error) or error > tolerance:
        raise ValueError(f'final map position outside goal tolerance ({error:.3f} m)')
    return error


def isolate_ros(config):
    for key in ('ROS_LOCALHOST_ONLY', 'ROS_DISCOVERY_SERVER', 'ROS_SUPER_CLIENT',
                'CYCLONEDDS_URI', 'FASTDDS_DEFAULT_PROFILES_FILE',
                'FASTRTPS_DEFAULT_PROFILES_FILE', 'RMW_FASTRTPS_USE_QOS_FROM_XML'):
        os.environ.pop(key, None)
    os.environ.update(
        ROS_DOMAIN_ID=str(config.ros_domain_id), ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
        ROS_STATIC_PEERS='', GZ_PARTITION=f'amr_ad_sim_{uuid4().hex}', PYTHONUNBUFFERED='1')


def stop_process(process):
    if process.poll() is None:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            pass
    # The launch parent can exit before all its children do.
    for sig in (signal.SIGTERM, signal.SIGKILL):
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            break
        try:
            process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            continue


def launch(stack, argv, log_path, cwd):
    with log_path.open('w') as stream:
        process = subprocess.Popen(argv, cwd=cwd, stdout=stream,
                                   stderr=subprocess.STDOUT, start_new_session=True)
    stack.callback(stop_process, process)
    return process


@contextmanager
def ros_context(rclpy, args):
    # Jazzy's init() returns None.
    rclpy.init(args=args)
    try:
        yield
    finally:
        rclpy.try_shutdown()


def run_pr14_d(config):
    import rclpy
    from amr_navigation import gate1_mock
    from rclpy.executors import ExternalShutdownException

    gate1_mock.MOCK_WAYPOINTS = {config.waypoint_id: config.waypoint_pose}
    with ros_context(rclpy, [
        '--ros-args', '-p', 'use_sim_time:=true',
        '-p', 'wait_for_nav2_before_dispatch:=true',
    ]):
        node = gate1_mock.DNavigationNode()
        try:
            rclpy.spin(node)
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        finally:
            node.destroy_node()


def run_sim(config, config_path, log_dir, summary):
    # Import after setting DDS and Gazebo isolation for this process and children.
    import rclpy
    from action_msgs.msg import GoalStatusArray
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from lifecycle_msgs.srv import GetState
    from logistics_msgs.msg import TaskStep
    from logistics_msgs.srv import OptimizeRoute
    from mission_manager import node as mission_module
    from mission_manager.navigation_services import NavigationServices
    from nav2_msgs.action import NavigateToPose
    from nav_msgs.msg import Odometry
    from rclpy.action import ActionClient
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_action_status_default, qos_profile_sensor_data
    from rclpy.time import Time
    from sensor_msgs.msg import LaserScan
    from tf2_ros import Buffer, TransformListener

    if not Path(mission_module.__file__).resolve().is_relative_to(ROOT / 'ros2_ws'):
        raise ValueError('source this PR #14 workspace install/setup.bash before running')

    with ros_context(rclpy, []), ExitStack() as stack:
        manager = mission_module.MissionManager(parameter_overrides=[
            Parameter('use_sim_time', value=True),
            Parameter('database_path', value=str(log_dir / 'mission.sqlite'))])
        stack.callback(manager.destroy_node)
        bridge = NavigationServices(manager)
        tf = Buffer(node=manager)
        listener = TransformListener(tf, manager)
        stack.callback(listener.unregister)
        sensor_samples = set()
        statuses = {}
        processes = {}

        def observe_status(message):
            for status in message.status_list:
                statuses[bytes(status.goal_info.goal_id.uuid).hex()] = status.status

        manager.create_subscription(
            GoalStatusArray, '/navigate_to_pose/_action/status', observe_status,
            qos_profile_action_status_default)
        manager.create_subscription(LaserScan, '/scan', lambda _: sensor_samples.add('scan'),
                                    qos_profile_sensor_data)
        manager.create_subscription(Odometry, '/odom', lambda _: sensor_samples.add('odom'),
                                    qos_profile_sensor_data)
        manager.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
                                    lambda _: sensor_samples.add('amcl'), 10)
        initialpose = manager.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        navigator = ActionClient(manager, NavigateToPose, '/navigate_to_pose')
        states = {name: manager.create_client(GetState, f'/{name}/get_state')
                  for name in ('amcl', 'planner_server', 'controller_server', 'bt_navigator')}

        def spin(timeout=0.1):
            for name, process in processes.items():
                if process.poll() is not None:
                    raise RuntimeError(f'{name} exited with code {process.returncode}')
            rclpy.spin_once(manager, timeout_sec=timeout)

        def wait(predicate, deadline, reason):
            while not predicate():
                if monotonic() >= deadline:
                    raise TimeoutError(reason)
                spin()

        def active(names, deadline):
            remaining = set(names)
            while remaining:
                for name in tuple(remaining):
                    client = states[name]
                    if client.service_is_ready():
                        future = client.call_async(GetState.Request())
                        wait(future.done, deadline, f'{name} lifecycle response timed out')
                        if future.result().current_state.label == 'active':
                            remaining.remove(name)
                if remaining:
                    if monotonic() >= deadline:
                        raise TimeoutError(f'Nav2 lifecycle not active: {sorted(remaining)}')
                    spin()

        boot_deadline = monotonic() + config.startup_timeout_sec
        processes['SIM'] = launch(stack, config.sim_command, log_dir / 'sim.log', config_path.parent)
        wait(lambda: manager.get_clock().now().nanoseconds > 0
             and {'scan', 'odom'} <= sensor_samples
             and initialpose.get_subscription_count() > 0, boot_deadline,
             'waiting for advancing SIM clock, scan, odometry or AMCL subscriber')
        clock_at_start = manager.get_clock().now().nanoseconds
        wait(lambda: manager.get_clock().now().nanoseconds > clock_at_start, boot_deadline,
             'SIM clock stopped advancing')
        active(('amcl',), boot_deadline)
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = manager.get_clock().now().to_msg()
        x, y, yaw = config.initial_pose
        msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
        msg.pose.pose.orientation.z = sin(yaw / 2)
        msg.pose.pose.orientation.w = cos(yaw / 2)
        msg.pose.covariance[0] = msg.pose.covariance[7] = 0.05
        msg.pose.covariance[35] = 0.03
        sensor_samples.discard('amcl')
        initialpose.publish(msg)
        wait(lambda: 'amcl' in sensor_samples
             and tf.can_transform('map', config.base_frame, Time()), boot_deadline,
             'AMCL pose or map-to-base TF not ready')
        active(states, boot_deadline)
        wait(navigator.server_is_ready, boot_deadline, 'NavigateToPose action not ready')
        if statuses:
            raise ValueError(f'Nav2 already has goal statuses in the chosen domain: {statuses}')
        summary['nav2_ready'] = True

        # Keep the real A mission state machine. Supply one plan locally because C is out of scope.
        pickup = TaskStep(job_id='sim-job', step_id='pickup', kind=TaskStep.PICKUP,
                          waypoint_id=config.waypoint_id)
        delivery = TaskStep(job_id='sim-job', step_id='delivery', kind=TaskStep.DROPOFF,
                            waypoint_id='HOME', prerequisite_ids=['pickup'])
        manager.state.add_steps([pickup, delivery])
        request = manager.state.request('HOME', 'a-d-sim')
        response = OptimizeRoute.Response(
            success=True, mission_id=request.mission_id, request_id=request.request_id,
            base_version=request.base_version, cost_revision=request.cost_revision,
            method='fixed-a-d-sim-plan', estimated_distance_m=0.0,
            ordered_step_ids=['pickup', 'delivery'])
        if not manager.state.apply(response):
            raise RuntimeError('A rejected the SIM fixture plan')
        goal = manager.claim_next_goal()
        if goal is None or goal.step_id != 'pickup':
            raise RuntimeError('A did not claim the pickup goal')

        d_log = log_dir / 'd.log'
        d_command = config.d_command or [
            sys.executable, str(Path(__file__).resolve()), str(config_path), '--pr14-d']
        processes['D'] = launch(stack, d_command, d_log, config_path.parent)
        wait(lambda: bridge.dispatch_client.service_is_ready()
             and bridge.lookup_client.service_is_ready(),
             monotonic() + config.startup_timeout_sec, 'D navigation services not ready')
        if statuses:
            raise ValueError(f'another Nav2 goal appeared before dispatch: {statuses}')
        if not bridge.dispatch(goal):
            raise RuntimeError('A did not submit its pickup goal')
        a_uuid = manager.state.nav_goal_id
        summary.update(mission_id=manager.state.mission_id, step_id=goal.step_id, goal_uuid=a_uuid)
        deadline = monotonic() + config.goal_timeout_sec

        def finished():
            if manager.state.recovery_required:
                raise RuntimeError('A entered recovery; automatic retry is disabled for this run')
            if set(statuses) - {a_uuid}:
                raise RuntimeError(f'unexpected goal UUID on Nav2: {statuses}')
            if statuses.get(a_uuid) in (5, 6):  # CANCELED or ABORTED
                raise RuntimeError(f'Nav2 failed to reach the goal: {statuses}')
            return (statuses.get(a_uuid) == NAV2_SUCCEEDED
                    and manager.step_status(goal.step_id) == 'waiting_confirmation'
                    and tf.can_transform('map', config.base_frame, Time()))

        wait(finished, deadline, 'missing Nav2 SUCCEEDED, A waiting_confirmation, or goal TF')
        # Drain final callbacks and observe late duplicate sends before taking the verdict.
        settle_until = monotonic() + 1.0
        while monotonic() < settle_until:
            spin()
        final = tf.lookup_transform('map', config.base_frame, Time()).transform.translation
        waiting = (manager.step_status(goal.step_id) == 'waiting_confirmation'
                   and manager.current_goal() == goal and not manager.state.recovery_required
                   and not manager.state.completed and manager.claim_next_goal() is None)
        evidence = GoalEvidence(a_uuid, d_log.read_text(errors='replace'), statuses,
                                waiting, (final.x, final.y))
        summary.update(d_send_count=evidence.d_log.count('[SEND]'), nav2_statuses=statuses,
                       a_step_state=manager.step_status(goal.step_id),
                       final_position_m=evidence.final_position)
        summary['position_error_m'] = verify_evidence(
            evidence, config.waypoint_id, config.waypoint_pose, config.tolerance_m)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('config', type=Path)
    parser.add_argument('--run', action='store_true', help='start the configured SIM and send one goal')
    parser.add_argument('--log-dir', type=Path, default=Path(gettempdir()) / 'a-d-sim-logs',
                        help='parent directory for each separate run (default: /tmp/a-d-sim-logs)')
    parser.add_argument('--pr14-d', action='store_true', help=argparse.SUPPRESS)
    args = parser.parse_args()
    config_path = args.config.resolve()
    try:
        config = load_config(config_path)
    except (OSError, ValueError, KeyError, TypeError) as exc:
        parser.error(str(exc))
    if args.pr14_d:
        run_pr14_d(config)
        return 0
    if not args.run:
        print('configuration valid. Use --run to start the SIM.')
        return 0
    if os.environ.get('ROS_DISTRO') != 'jazzy':
        parser.error('source ROS 2 Jazzy and the workspace install/setup.bash first')
    ancestry = subprocess.run(['git', 'merge-base', '--is-ancestor', PR14_COMMIT, 'HEAD'],
                              cwd=ROOT, check=False)
    if ancestry.returncode:
        parser.error(f'this checkout must include PR #14 commit {PR14_COMMIT}')

    isolate_ros(config)
    args.log_dir.mkdir(parents=True, exist_ok=True)
    log_dir = Path(mkdtemp(prefix='a-d-sim-', dir=args.log_dir.resolve()))
    summary = {'verdict': 'FAIL', 'mode': 'SIM', 'ros_domain_id': config.ros_domain_id,
               'config': str(config_path), 'map_yaml': str(config.map_yaml),
               'waypoint': config.waypoint_id, 'waypoint_pose': config.waypoint_pose,
               'd_adapter': 'PR14' if config.d_command is None else 'configured command',
               'commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip()}
    (log_dir / 'config.json').write_text(config_path.read_text())
    print(f'Logs: {log_dir}', flush=True)
    try:
        run_sim(config, config_path, log_dir, summary)
        summary['verdict'] = 'PASS'
    except (Exception, KeyboardInterrupt) as exc:
        summary['error'] = f'{type(exc).__name__}: {exc}'
        print(summary['error'], file=sys.stderr)
    finally:
        (log_dir / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(f"{summary['verdict']}: {log_dir / 'summary.json'}")
    return 0 if summary['verdict'] == 'PASS' else 1


if __name__ == '__main__':
    sys.exit(main())

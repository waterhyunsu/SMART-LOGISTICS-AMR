"""A·C·D 공용 waypoints.yaml: D 가 같은 파일의 좌표로 Nav2 목표를 보내는지 확인."""
from contextlib import ExitStack, contextmanager
from pathlib import Path
import subprocess
import sys
from time import monotonic
from uuid import uuid4

from ament_index_python.packages import get_package_share_directory
import pytest
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from amr_navigation.gate1_mock import load_waypoints
from logistics_msgs.srv import DispatchGoal

D_SCRIPT = Path(__file__).resolve().parents[1] / 'amr_navigation' / 'gate1_mock.py'
SRC_WAYPOINTS = Path(__file__).resolve().parents[1] / 'config' / 'waypoints.yaml'


@contextmanager
def client_node(namespace):
    with Context() as context:
        node = Node('waypoints_file_test', context=context, namespace=namespace)
        with SingleThreadedExecutor(context=context) as executor:
            executor.add_node(node)
            try:
                yield node, executor
            finally:
                executor.remove_node(node)
                node.destroy_node()


def until(executor, predicate, timeout=12):
    deadline = monotonic() + timeout
    while not predicate() and monotonic() < deadline:
        executor.spin_once(timeout_sec=0.05)
    assert predicate()


def run_process(stack, tmp_path, namespace, name, params):
    command = [sys.executable, str(D_SCRIPT), name, '--ros-args', '-r', f'__ns:={namespace}']
    for param in params:
        command.extend(['-p', param])
    log_path = tmp_path / f'{name}.log'
    with log_path.open('wb') as log:
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)

    def stop():
        if process.poll() is None:
            process.terminate()
            try:
                process.wait(2)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
    stack.callback(stop)
    return log_path, process


def call(executor, client, request):
    until(executor, client.service_is_ready)
    future = client.call_async(request)
    until(executor, future.done)
    return future.result()


def context(waypoint):
    return dict(mission_id='wp-mission', step_id='step-1', plan_version=1,
                waypoint_id=waypoint, attempt_id=f'attempt-{uuid4().hex[:8]}',
                nav_goal_id=uuid4().hex)


def test_installed_waypoints_file_matches_source():
    installed = Path(get_package_share_directory('amr_navigation')) / 'config' / 'waypoints.yaml'
    assert installed.is_file()
    waypoints = load_waypoints(installed)
    assert {'HOME', 'PICKUP_1', 'DROP_1'} <= set(waypoints)
    assert waypoints == load_waypoints(SRC_WAYPOINTS)


@pytest.mark.parametrize('text, message', [
    ('waypoints: {A: {x: 1, y: 2, yaw: 0}}', 'HOME'),
    ('frame_id: odom\nwaypoints: {HOME: {x: 0, y: 0, yaw: 0}}', 'frame_id'),
    ('waypoints: {HOME: {x: 0, y: 0}}', 'x, y, yaw'),
    ('waypoints: {HOME: {x: .nan, y: 0, yaw: 0}}', 'finite'),
])
def test_bad_waypoints_file_is_rejected(tmp_path, text, message):
    path = tmp_path / 'bad.yaml'
    path.write_text(text)
    with pytest.raises(ValueError, match=message):
        load_waypoints(path)


def test_d_sends_coordinates_from_waypoints_file(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    namespace = '/wpfile_' + uuid4().hex
    with ExitStack() as stack, client_node(namespace) as (node, executor):
        nav_log, _ = run_process(stack, tmp_path, namespace, 'server', ['delay_sec:=0.2'])
        d_log, _ = run_process(stack, tmp_path, namespace, 'd',
                               [f'waypoints_file:={SRC_WAYPOINTS}'])
        dispatch = node.create_client(DispatchGoal, 'dispatch_goal')
        until(executor, lambda: 'lifecycle: active' in d_log.read_text())

        # MOCK 임시 ID 는 파일에 없으므로 송신 없이 INVALID
        old = context('A')
        response = call(executor, dispatch, DispatchGoal.Request(**old))
        assert response.result == DispatchGoal.Response.RESULT_INVALID
        assert 'unknown waypoint' in response.reason

        ctx = context('PICKUP_1')
        response = call(executor, dispatch, DispatchGoal.Request(**ctx))
        assert response.result == DispatchGoal.Response.RESULT_ACCEPTED
        until(executor, lambda: f'[RESULT] {ctx["nav_goal_id"]} → SUCCEEDED' in d_log.read_text())
        text = d_log.read_text()
        assert f'[SEND] {ctx["nav_goal_id"]} → waypoint PICKUP_1 (-0.55, -0.5, 0.0)' in text
        assert text.count('[SEND]') == 1
        assert nav_log.read_text().count('[GOAL]') == 1


def test_d_refuses_to_start_with_bad_waypoints_file(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    bad = tmp_path / 'bad.yaml'
    bad.write_text('waypoints: {PICKUP_1: {x: 0, y: 0, yaw: 0}}')
    with ExitStack() as stack:
        d_log, process = run_process(stack, tmp_path, '/wpbad_' + uuid4().hex, 'd',
                                     [f'waypoints_file:={bad}'])
        assert process.wait(15) != 0
        assert 'HOME waypoint required' in d_log.read_text()

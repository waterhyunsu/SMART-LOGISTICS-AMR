"""Gate 2: D sends to Nav2 only while bt_navigator is active (fake Nav2 lifecycle)."""
from contextlib import ExitStack, contextmanager
from pathlib import Path
import subprocess
import sys
from time import monotonic
from uuid import uuid4

from rcl_interfaces.srv import SetParameters
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter

from logistics_msgs.srv import DispatchGoal, LookupGoal

D_SCRIPT = Path(__file__).resolve().parents[1] / 'amr_navigation' / 'gate1_mock.py'


@contextmanager
def client_node(namespace):
    with Context() as context:
        node = Node('nav2_lifecycle_gate_test', context=context, namespace=namespace)
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


def spin_for(executor, seconds):
    deadline = monotonic() + seconds
    while monotonic() < deadline:
        executor.spin_once(timeout_sec=0.05)


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
    return log_path


def call(executor, client, request):
    until(executor, client.service_is_ready)
    future = client.call_async(request)
    until(executor, future.done)
    return future.result()


def context(waypoint='A'):
    return dict(mission_id='gate2-mission', step_id='step-1', plan_version=1,
                waypoint_id=waypoint, attempt_id=f'attempt-{uuid4().hex[:8]}',
                nav_goal_id=uuid4().hex)


def set_lifecycle(node, executor, label):
    client = node.create_client(SetParameters, 'mock_nav2_server/set_parameters')
    request = SetParameters.Request()
    request.parameters = [Parameter('lifecycle_state', value=label).to_parameter_msg()]
    result = call(executor, client, request)
    assert all(r.successful for r in result.results)


def lookup(node, executor, ctx):
    client = node.create_client(LookupGoal, 'lookup_goal')
    request = LookupGoal.Request(check_id=uuid4().hex, **ctx)
    response = call(executor, client, request)
    assert response.check_id == request.check_id
    return response


def test_dispatch_service_waits_for_active_bt_navigator(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    namespace = '/gate2_' + uuid4().hex
    with ExitStack() as stack, client_node(namespace) as (node, executor):
        nav_log = run_process(stack, tmp_path, namespace, 'server',
                              ['delay_sec:=0.2', 'lifecycle_state:=inactive'])
        d_log = run_process(stack, tmp_path, namespace, 'd',
                            ['wait_for_nav2_before_dispatch:=true'])
        dispatch = node.create_client(DispatchGoal, 'dispatch_goal')
        until(executor, lambda: 'lifecycle: inactive' in d_log.read_text())
        spin_for(executor, 1.5)
        assert not dispatch.service_is_ready()

        set_lifecycle(node, executor, 'active')
        until(executor, dispatch.service_is_ready)
        ctx = context()
        response = call(executor, dispatch, DispatchGoal.Request(**ctx))
        assert response.result == DispatchGoal.Response.RESULT_ACCEPTED
        until(executor, lambda: f'받은 UUID={ctx["nav_goal_id"]}' in nav_log.read_text())
        assert nav_log.read_text().count('[GOAL]') == 1


def test_refusal_while_inactive_then_explicit_new_attempt(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    namespace = '/gate2_' + uuid4().hex
    with ExitStack() as stack, client_node(namespace) as (node, executor):
        nav_log = run_process(stack, tmp_path, namespace, 'server',
                              ['delay_sec:=0.2', 'lifecycle_state:=inactive'])
        d_log = run_process(stack, tmp_path, namespace, 'd', [])
        dispatch = node.create_client(DispatchGoal, 'dispatch_goal')
        until(executor, lambda: 'lifecycle: inactive' in d_log.read_text())

        refused = context()
        response = call(executor, dispatch, DispatchGoal.Request(**refused))
        assert response.result == DispatchGoal.Response.RESULT_NOT_READY
        assert 'bt_navigator not active' in response.reason
        observed = lookup(node, executor, refused)
        assert observed.observation == LookupGoal.Response.OBS_UNKNOWN
        assert not observed.no_pending_dispatch

        set_lifecycle(node, executor, 'active')
        deadline = monotonic() + 12
        while True:
            observed = lookup(node, executor, refused)
            if observed.observation == LookupGoal.Response.OBS_REFUSED_BEFORE_SEND:
                break
            assert observed.observation == LookupGoal.Response.OBS_UNKNOWN
            assert monotonic() < deadline, 'D did not observe bt_navigator active'
            spin_for(executor, 0.3)
        assert observed.no_pending_dispatch

        replay = call(executor, dispatch, DispatchGoal.Request(**refused))
        assert replay.result == DispatchGoal.Response.RESULT_DUPLICATE
        assert '[GOAL]' not in nav_log.read_text()

        retry = context()
        response = call(executor, dispatch, DispatchGoal.Request(**retry))
        assert response.result == DispatchGoal.Response.RESULT_ACCEPTED
        until(executor, lambda: f'[RESULT] {retry["nav_goal_id"]} → SUCCEEDED' in d_log.read_text())
        assert nav_log.read_text().count('[GOAL]') == 1

        # Going inactive again must stop new sends once the cached active state expires.
        set_lifecycle(node, executor, 'inactive')
        until(executor, lambda: d_log.read_text().count('lifecycle: inactive') >= 2)
        late = context()
        response = call(executor, dispatch, DispatchGoal.Request(**late))
        assert response.result == DispatchGoal.Response.RESULT_NOT_READY
        spin_for(executor, 0.5)
        assert nav_log.read_text().count('[GOAL]') == 1
        assert d_log.read_text().count('[SEND]') == 1


def test_late_active_lifecycle_response_is_ignored(tmp_path, monkeypatch):
    """get_state 요청 후 2초를 넘겨 도착한 ACTIVE 는 준비로 보지 않는다."""
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    namespace = '/gate2_' + uuid4().hex
    with ExitStack() as stack, client_node(namespace) as (node, executor):
        nav_log = run_process(stack, tmp_path, namespace, 'server',
                              ['delay_sec:=0.2', 'lifecycle_state:=active',
                               'get_state_delay_sec:=2.2'])
        d_log = run_process(stack, tmp_path, namespace, 'd', [])
        dispatch = node.create_client(DispatchGoal, 'dispatch_goal')
        until(executor, lambda: ('get_state late response' in d_log.read_text()
                                 or 'get_state timeout' in d_log.read_text()))

        late = context()
        response = call(executor, dispatch, DispatchGoal.Request(**late))
        assert response.result == DispatchGoal.Response.RESULT_NOT_READY
        assert 'bt_navigator not active' in response.reason
        for _ in range(2):
            observed = lookup(node, executor, late)
            assert observed.observation == LookupGoal.Response.OBS_UNKNOWN
            assert not observed.no_pending_dispatch
            spin_for(executor, 1.5)

        other = context()
        response = call(executor, dispatch, DispatchGoal.Request(**other))
        assert response.result == DispatchGoal.Response.RESULT_NOT_READY
        assert 'lifecycle: active' not in d_log.read_text()
        assert '[GOAL]' not in nav_log.read_text()
        assert '[SEND]' not in d_log.read_text()

        # 응답이 다시 제때 오면 정상 복구: 거절했던 요청은 REFUSED_BEFORE_SEND 로 확인된다.
        client = node.create_client(SetParameters, 'mock_nav2_server/set_parameters')
        request = SetParameters.Request()
        request.parameters = [Parameter('get_state_delay_sec', value=0.0).to_parameter_msg()]
        assert all(r.successful for r in call(executor, client, request).results)
        deadline = monotonic() + 12
        while True:
            observed = lookup(node, executor, late)
            if observed.observation == LookupGoal.Response.OBS_REFUSED_BEFORE_SEND:
                break
            assert observed.observation == LookupGoal.Response.OBS_UNKNOWN
            assert monotonic() < deadline, 'D did not recover after timely get_state responses'
            spin_for(executor, 0.3)
        assert observed.no_pending_dispatch
        assert '[GOAL]' not in nav_log.read_text()

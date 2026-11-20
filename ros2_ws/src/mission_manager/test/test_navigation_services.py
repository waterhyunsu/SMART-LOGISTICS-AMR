"""Gate 1 with the real A node, D services and a fake Nav2 Action server."""
from contextlib import ExitStack, contextmanager
import os
from pathlib import Path
import subprocess
import sys
from time import monotonic
from uuid import uuid4

import pytest
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.action.graph import get_action_names_and_types
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter

from logistics_msgs.srv import DispatchGoal, ReportGoalEvent

from mission_manager.navigation_services import NavigationServices
from mission_manager.node import MissionManager, NavigationObservation
from test_contract import mock_optimizer
from test_dds import job


D_SCRIPT = Path(__file__).resolve().parents[2] / 'amr_navigation/amr_navigation/gate1_mock.py'


@contextmanager
def manager(path, namespace):
    with Context() as context:
        node = MissionManager(
            context=context, namespace=namespace,
            parameter_overrides=[Parameter('database_path', value=str(path))])
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
    command = [sys.executable, str(D_SCRIPT), name, '--ros-args',
               '-r', f'__ns:={namespace}']
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
    return stop, log_path


@pytest.mark.parametrize('mode,result_error,d_accept_timeout,expected', [
    ('succeed', False, 0.6, NavigationObservation.ARRIVED),
    ('abort', False, 0.6, NavigationObservation.NO_ACTIVE_GOAL),
    ('no_response', False, 0.6, NavigationObservation.UNKNOWN),
    ('succeed', True, 0.6, NavigationObservation.UNKNOWN),
    ('no_response', False, 10.0, NavigationObservation.UNKNOWN),
])
def test_gate1_separate_processes(tmp_path, monkeypatch, mode, result_error, d_accept_timeout, expected):
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    namespace = '/gate1_' + uuid4().hex
    path = tmp_path / 'mission.sqlite'
    with ExitStack() as stack:
        _, nav_log = run_process(
            stack, tmp_path, namespace, 'server', [f'mode:={mode}', 'delay_sec:=0.7'])
        with manager(path, namespace=namespace) as (node, executor):
            action_name = f'{namespace}/navigate_to_pose'
            until(executor, lambda: any(
                name == action_name for name, _ in get_action_names_and_types(node)))
            stop_d, d_log = run_process(stack, tmp_path, namespace, 'd', [
                f'accept_timeout_sec:={d_accept_timeout}',
                f'fault_result_error:={str(result_error).lower()}'])
            adapter = NavigationServices(node, timeout_sec=3.0)
            until(executor, adapter.dispatch_client.service_is_ready)
            node.state.add_steps(job('one'))
            assert node.state.apply(mock_optimizer(node.state.request('HOME', 'r1')))
            goal = node.claim_next_goal()
            assert adapter.dispatch(goal)
            goal_id = node.state.nav_goal_id
            assert goal_id and not adapter.dispatch(goal)
            context = dict(mission_id=goal.mission_id, step_id=goal.step_id,
                           plan_version=goal.plan_version, waypoint_id=goal.waypoint_id,
                           attempt_id=goal.attempt_id, nav_goal_id=goal_id)
            if expected is NavigationObservation.ARRIVED:
                # Replay on the wire after D has received the first request.
                until(executor, lambda: '→ ACCEPTED' in d_log.read_text())
                request = DispatchGoal.Request(**context)
                future = adapter.dispatch_client.call_async(request)
                until(executor, future.done)
                assert future.result().result == DispatchGoal.Response.RESULT_DUPLICATE
                request.plan_version += 1
                future = adapter.dispatch_client.call_async(request)
                until(executor, future.done)
                assert future.result().result == DispatchGoal.Response.RESULT_CONFLICT

            until(executor, lambda: node.state.waiting or node.state.recovery_required)
            assert node.state.waiting == (expected is NavigationObservation.ARRIVED)
            assert node.claim_next_goal() is None
            if expected is NavigationObservation.ARRIVED:
                assert node.handoff('handoff', node.state.mission_id, goal.step_id) == 'accepted'
                assert node.handoff('handoff', node.state.mission_id, goal.step_id) == 'accepted'
                assert node.state.completed == {goal.step_id}
            else:
                for _ in range(2):
                    lookup = adapter.lookup()
                    assert lookup is not None
                    until(executor, lambda: adapter.last_report is not None)
                    assert adapter.last_report.lookup == lookup
                    assert adapter.last_report.observation is expected
                    assert adapter.last_report.no_pending_dispatch == (
                        expected is NavigationObservation.NO_ACTIVE_GOAL)
                    assert adapter.approve_retry(lookup) is None
                    assert node.state.attempt_id == goal.attempt_id
                if expected is NavigationObservation.NO_ACTIVE_GOAL:
                    retry = adapter.approve_retry(lookup, explicitly_approved=True)
                    assert retry and retry.attempt_id != goal.attempt_id
                    assert adapter.approve_retry(lookup, explicitly_approved=True) is None
                    client = node.create_client(ReportGoalEvent, 'report_goal_event')
                    until(executor, client.service_is_ready)
                    stale = ReportGoalEvent.Request(
                        **context, event=ReportGoalEvent.Request.EVENT_SUCCEEDED, event_seq=100)
                    future = client.call_async(stale)
                    until(executor, future.done)
                    assert not future.result().handled
                    assert not node.state.waiting
                    assert node.current_goal() == retry
                else:
                    assert adapter.approve_retry(lookup, explicitly_approved=True) is None

        assert goal_id in d_log.read_text()
        assert '누적 Nav2 송신 1회' in d_log.read_text()
        assert '누적 Nav2 송신 2회' not in d_log.read_text()
        if mode != 'no_response':
            assert f'받은 UUID={goal_id}' in nav_log.read_text()
        if expected is NavigationObservation.UNKNOWN:
            stop_d()
            run_process(stack, tmp_path, namespace, 'd', [])
            with manager(path, namespace=namespace) as (node, executor):
                assert node.state.recovery_required
                assert node.state.nav_goal_id == goal_id
                adapter = NavigationServices(node, timeout_sec=3.0)
                until(executor, lambda: adapter.lookup_client.service_is_ready())
                lookup = adapter.lookup()
                until(executor, lambda: adapter.last_report is not None)
                assert adapter.last_report.observation is NavigationObservation.UNKNOWN
                assert adapter.approve_retry(lookup, explicitly_approved=True) is None
                assert not adapter.dispatch(node.current_goal())


def test_d_exits_after_acceptance(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    namespace = '/gate1_' + uuid4().hex
    with ExitStack() as stack, manager(tmp_path / 'mission.sqlite', namespace) as (node, executor):
        run_process(stack, tmp_path, namespace, 'server', ['delay_sec:=15.0'])
        until(executor, lambda: any(
            name == f'{namespace}/navigate_to_pose' for name, _ in get_action_names_and_types(node)))
        stop_d, d_log = run_process(stack, tmp_path, namespace, 'd', [])
        adapter = NavigationServices(node, timeout_sec=3.0)
        until(executor, adapter.dispatch_client.service_is_ready)

        node.state.add_steps(job('one'))
        assert node.state.apply(mock_optimizer(node.state.request('HOME', 'r1')))
        goal = node.claim_next_goal()
        assert adapter.dispatch(goal)
        until(executor, lambda: 'ACCEPTED → A handled=True' in d_log.read_text())
        goal_id = node.state.nav_goal_id
        stop_d()
        until(executor, lambda: node.state.recovery_required)
        assert not node.state.waiting
        assert node.state.nav_goal_id == goal_id
        assert node.claim_next_goal() is None
        lookup = adapter.lookup()
        assert lookup and adapter.last_report.observation is NavigationObservation.DISCONNECTED
        assert adapter.approve_retry(lookup, explicitly_approved=True) is None


@pytest.mark.parametrize('drop_receipt', [False, True], ids=['confirmed-refusal', 'lost-receipt'])
def test_retry_after_not_ready_then_nav2_start(tmp_path, monkeypatch, drop_receipt):
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    namespace = '/gate1_' + uuid4().hex
    with ExitStack() as stack, manager(tmp_path / 'mission.sqlite', namespace) as (node, executor):
        _, d_log = run_process(stack, tmp_path, namespace, 'd', [])
        adapter = NavigationServices(node, timeout_sec=3.0)
        until(executor, adapter.dispatch_client.service_is_ready)
        node.state.add_steps(job('one'))
        assert node.state.apply(mock_optimizer(node.state.request('HOME', 'r1')))
        goal = node.claim_next_goal()
        sent_requests = []
        call_async = adapter.dispatch_client.call_async

        def send_request(request):
            sent_requests.append(request)
            future = call_async(request)
            if drop_receipt:
                # Send on DDS but discard its response, without affecting D's handler.
                adapter.dispatch_client.remove_pending_request(future)
            return future

        with monkeypatch.context() as patch:
            patch.setattr(adapter.dispatch_client, 'call_async', send_request)
            assert adapter.dispatch(goal)
        until(executor, lambda: node.state.recovery_required)
        original_id = node.state.nav_goal_id
        assert original_id
        d_dispatch = next(line for line in d_log.read_text().splitlines()
                          if '[dispatch_goal]' in line and original_id in line)
        assert '→ NOT_READY' in d_dispatch and '누적 Nav2 송신 0회' in d_dispatch

        early_lookup = adapter.lookup()
        assert early_lookup is not None
        until(executor, lambda: adapter.last_report is not None)
        assert adapter.last_report.observation is NavigationObservation.UNKNOWN
        assert adapter.approve_retry(early_lookup, explicitly_approved=True) is None

        _, nav_log = run_process(stack, tmp_path, namespace, 'server', ['delay_sec:=0.2'])
        action = ActionClient(node, NavigateToPose, 'navigate_to_pose')
        until(executor, action.server_is_ready)
        until(executor, adapter.lookup_client.service_is_ready)
        lookup = adapter.lookup()
        assert lookup is not None
        until(executor, lambda: adapter.last_report is not None)
        assert adapter.last_report.lookup == lookup
        assert node.current_goal() == goal and node.state.nav_goal_id == original_id
        assert adapter.approve_retry(lookup) is None
        assert not adapter.dispatch(goal)
        assert '[GOAL]' not in nav_log.read_text()

        # D must remember the refusal and block replay even after Nav2 starts.
        duplicate = call_async(sent_requests[0])
        until(executor, duplicate.done)
        assert duplicate.result().result == DispatchGoal.Response.RESULT_DUPLICATE
        assert '[GOAL]' not in nav_log.read_text()

        retry = adapter.approve_retry(lookup, explicitly_approved=True)
        if drop_receipt:
            assert adapter.last_report.observation is NavigationObservation.UNKNOWN
            assert not adapter.last_report.no_pending_dispatch
            assert retry is None
            assert node.state.recovery_required and node.state.nav_goal_id == original_id
            return

        assert retry is not None, 'confirmed refusal should allow an explicit retry after Nav2 starts'
        assert retry.attempt_id != goal.attempt_id
        assert adapter.approve_retry(lookup, explicitly_approved=True) is None
        assert '[GOAL]' not in nav_log.read_text()
        assert adapter.dispatch(retry)
        assert node.state.nav_goal_id != original_id
        until(executor, lambda: node.state.waiting)
        assert nav_log.read_text().count('[GOAL]') == 1

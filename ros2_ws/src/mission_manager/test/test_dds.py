"""Exercise real ROS service traffic; spin endpoints separately to control reply timing."""

from time import monotonic
from uuid import uuid4

import pytest
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor

from logistics_msgs.msg import TaskStep
from mission_manager.node import MissionManager
from mock_optimizer import MockOptimizer


def until(condition, *executors):
    deadline = monotonic() + 5
    while not condition():
        assert monotonic() < deadline, 'timed out waiting for executor'
        for executor in executors:
            executor.spin_once(timeout_sec=0.01)


@pytest.fixture
def endpoints():
    context = Context()
    context.init(args=[])
    namespace = '/test_' + uuid4().hex
    manager = MissionManager(context=context, namespace=namespace)
    server = MockOptimizer(context=context, namespace=namespace)
    client_exec = SingleThreadedExecutor(context=context)
    server_exec = SingleThreadedExecutor(context=context)
    client_exec.add_node(manager)
    server_exec.add_node(server)
    try:
        assert manager.optimizer.wait_for_service(timeout_sec=5)
        yield manager, server, client_exec, server_exec
    finally:
        client_exec.shutdown()
        server_exec.shutdown()
        server.destroy_node()
        manager.destroy_node()
        context.shutdown()


def job(key):
    pickup = TaskStep(job_id=key, step_id=key+'-p', kind=TaskStep.PICKUP, waypoint_id='A')
    delivery = TaskStep(job_id=key, step_id=key+'-d', kind=TaskStep.DROPOFF,
                        waypoint_id='HOME', prerequisite_ids=[pickup.step_id])
    return [pickup, delivery]


def test_new_order_waits_for_arrival_and_handoff(endpoints):
    m, server, client, remote = endpoints
    initial = m.register_jobs(job('one'), 'HOME', 'r1')
    until(lambda: m.plan_status == 'applied', client, remote)
    assert server.requests[0].request_id == initial
    assert m.state.start_next() == 'one-p'
    latest = m.register_jobs(job('two'), 'HOME', 'r1')
    until(lambda: m.plan_status == 'deferred', client, remote)
    req = server.requests[-1]
    assert (req.request_id, req.base_version, req.start_waypoint_id) == (latest, 1, 'A')
    assert req.current_step_id == 'one-p'
    assert m.handoff('early', m.state.mission_id, 'one-p') == 'rejected'
    assert m.arrive('one-p')
    assert m.state.active_version == 1
    assert m.state.current == 'one-p'
    assert m.handoff('done', m.state.mission_id, 'one-p') == 'accepted'
    assert m.state.active_version == 2
    assert m.state.order == ['one-d', 'two-p', 'two-d']
    assert m.handoff('done', m.state.mission_id, 'one-p') == 'accepted'
    assert m.state.active_version == 2


@pytest.mark.parametrize('overrides', [
    {'request_id': 99}, {'base_version': 99}, {'cost_revision': 'old'},
    {'ordered_step_ids': ['one-d', 'one-p']},
    {'success': False, 'failure_reason': 'unreachable'},
])
def test_invalid_service_response(endpoints, overrides):
    m, server, client, remote = endpoints
    server.overrides = overrides
    m.register_jobs(job('one'), 'HOME', 'r1')
    until(lambda: m.plan_status == 'rejected', client, remote)
    assert len(server.requests) == 1
    assert m.state.order == [] and m.state.active_version == 0


def test_superseded_dds_response(endpoints):
    m, server, client, remote = endpoints
    first = m.register_jobs(job('one'), 'HOME', 'r1')
    until(lambda: m.future is not None, client)
    latest = m.request_plan('B', 'r1')
    # Only the first request has reached the server when its obsolete reply is sent.
    until(lambda: len(server.requests) == 1, remote)
    until(lambda: m.future is not None, client)
    assert m.state.active_version == 0
    until(lambda: m.plan_status == 'applied', client, remote)
    assert [r.request_id for r in server.requests] == [first, latest]
    assert m.state.active_version == 1


def test_timeout_then_late_response_and_retry(endpoints):
    m, server, client, remote = endpoints
    m.timeout = 0.2
    m.register_jobs(job('one'), 'HOME', 'r1')
    until(lambda: m.future is not None, client)
    until(lambda: m.plan_status == 'timeout', client)
    until(lambda: len(server.requests) == 1, remote)
    client.spin_once(timeout_sec=0.05)
    assert m.state.active_version == 0 and m.state.pending_request_id is None
    m.timeout = 5
    m.request_plan('HOME', 'r1')
    until(lambda: m.plan_status == 'applied', client, remote)


def test_deferred_candidate_rechecked_at_handoff(endpoints):
    m, _, client, remote = endpoints
    m.register_jobs(job('one'), 'HOME', 'r1')
    until(lambda: m.plan_status == 'applied', client, remote)
    m.state.start_next()
    m.request_plan('A', 'r1')
    until(lambda: m.plan_status == 'deferred', client, remote)
    m.state.update_cost_revision('r2')
    assert m.arrive('one-p')
    assert m.handoff('done', m.state.mission_id, 'one-p') == 'accepted'
    assert m.plan_status == 'rejected' and m.state.active_version == 1
    assert m.state.order == ['one-d']


def test_timeout_preserves_running_mission(endpoints):
    m, server, client, remote = endpoints
    m.register_jobs(job('one'), 'HOME', 'r1')
    until(lambda: m.plan_status == 'applied', client, remote)
    assert m.state.start_next() == 'one-p'
    assert m.arrive('one-p')
    m.timeout = 0.2
    m.request_plan('A', 'r1')
    until(lambda: m.future is not None, client)
    until(lambda: m.plan_status == 'timeout', client)
    until(lambda: len(server.requests) == 2, remote)
    for _ in range(5):
        client.spin_once(timeout_sec=0.02)
    assert m.state.current == 'one-p'
    assert m.state.waiting
    assert m.state.active_version == 1
    assert m.state.order == ['one-d']
    assert not m.state.completed
    assert m.handoff('done', m.state.mission_id, 'one-p') == 'accepted'
    assert m.state.start_next() == 'one-d'

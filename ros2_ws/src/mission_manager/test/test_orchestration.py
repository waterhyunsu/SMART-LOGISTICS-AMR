import json
import sqlite3

import pytest

from mission_manager.node import NavigationObservation
from test_dds import endpoints, job, until
from test_recovery_integration import FakeWeb, manager
from test_contract import mock_optimizer


def orders():
    steps = job('JOB-001') + job('JOB-002') + job('JOB-003')
    for step, waypoint in zip(steps, ('HOME', 'A', 'B', 'C', 'D', 'HOME')):
        step.waypoint_id = waypoint
    return steps


def complete_goal(node, web, goal):
    assert web.send('early:' + goal.step_id, goal.step_id) == 'rejected'
    assert node.handle_navigation_observation(goal, NavigationObservation.ARRIVED)
    assert not node.handle_navigation_observation(goal, NavigationObservation.ARRIVED)
    assert node.step_status(goal.step_id) == 'waiting_confirmation'
    assert node.mission_status == 'active'
    assert node.claim_next_goal() is None
    assert web.send('done:' + goal.step_id, goal.step_id) == 'accepted'


def test_basic_mission_with_home_unloading(endpoints):
    node, _, client, optimizer = endpoints
    node.register_jobs(orders(), 'HOME', 'mock-table')
    assert node.claim_next_goal() is None
    until(lambda: node.plan_status == 'applied', client, optimizer)
    web = FakeWeb(node)
    goals = []
    for waypoint in ('HOME', 'A', 'B', 'C', 'D', 'HOME'):
        goal = node.claim_next_goal()
        goals.append(goal)
        assert goal.waypoint_id == waypoint and goal.plan_version == 1
        assert goal.mission_id == node.state.mission_id
        assert node.claim_next_goal() is None
        assert node.mission_status == 'active'
        complete_goal(node, web, goal)
    assert len(set(goals)) == 6
    assert node.job_status('JOB-003') == 'completed'
    assert node.mission_status == 'completed' and node.claim_next_goal() is None
    assert web.send('done:' + goals[-1].step_id, goals[-1].step_id) == 'accepted'
    assert node.mission_status == 'completed'


@pytest.mark.parametrize('invalid', [None, 'base_version', 'cost_revision'])
def test_new_order_replans_only_after_handoff(endpoints, invalid):
    node, server, client, optimizer = endpoints
    node.register_jobs(orders(), 'HOME', 'r1')
    until(lambda: node.plan_status == 'applied', client, optimizer)
    goal = node.claim_next_goal()
    if invalid:
        server.overrides = {invalid: 0 if invalid == 'base_version' else 'old'}
    node.register_jobs(job('new'), 'HOME', 'r1')
    until(lambda: node.plan_status == ('rejected' if invalid else 'deferred'), client, optimizer)
    assert len(server.requests) == 2
    assert server.requests[-1].current_step_id == goal.step_id
    assert goal.step_id not in {step.step_id for step in server.requests[-1].pending_steps}
    assert node.state.active_version == 1 and node.claim_next_goal() is None
    web = FakeWeb(node)
    complete_goal(node, web, goal)
    version = 1 if invalid else 2
    assert node.state.active_version == version
    assert web.send('done:' + goal.step_id, goal.step_id) == 'accepted'
    assert node.state.active_version == version
    nxt = node.claim_next_goal()
    assert nxt.plan_version == version and nxt.step_id != goal.step_id
    assert not node.handle_navigation_observation(goal, NavigationObservation.ARRIVED)


@pytest.mark.parametrize('arrived', [False, True])
def test_recovery_blocks_dispatch_and_replays_ack(tmp_path, arrived):
    path = tmp_path / 'mission.sqlite'
    with manager(path) as node:
        state = node.state
        state.add_steps(job('one'))
        assert state.apply(mock_optimizer(state.request('HOME', 'r1')))
        goal = node.claim_next_goal()
        if arrived:
            assert node.handle_navigation_observation(goal, NavigationObservation.ARRIVED)
        state.request('A', 'r1')  # in-flight candidate is never restored
    with manager(path) as node:
        assert node.state.waiting is arrived and node.state.pending_request_id is None
        assert node.state.active_version == 1 and node.candidate is None
        for observation in (NavigationObservation.UNKNOWN, NavigationObservation.TIMEOUT,
                            NavigationObservation.DISCONNECTED):
            check = node.begin_recovery_check()
            assert not node.handle_recovery_observation(check, observation)
            assert node.claim_next_goal() is None
            assert not node.handle_navigation_observation(goal, NavigationObservation.ARRIVED)
            assert not node.handle_recovery_observation(check, NavigationObservation.ARRIVED)
        assert node.handle_recovery_observation(node.begin_recovery_check(), NavigationObservation.ARRIVED)
        assert node.claim_next_goal() is None and not node.state.completed
        assert FakeWeb(node).send('done', drop_ack=True) == 'accepted'
    for _ in range(2):
        with manager(path) as node:
            assert FakeWeb(node).send('done') == 'accepted'
            assert node.state.completed == {'one-p'}
    with manager(path) as node:
        complete_goal(node, FakeWeb(node), node.claim_next_goal())
        assert node.mission_status == 'completed'
    with manager(path) as node:
        assert node.mission_status == 'completed' and node.claim_next_goal() is None


def test_separate_home_return_and_failed_claim(tmp_path):
    path = tmp_path / 'mission.sqlite'
    with manager(path) as node:
        steps = job('one')
        steps[-1].waypoint_id = 'B'
        node.state.add_steps(steps)
        assert node.state.apply(mock_optimizer(node.state.request('HOME', 'r1')))
        db = node.state.db
        db.execute("CREATE TRIGGER fail BEFORE UPDATE ON mission BEGIN SELECT RAISE(ABORT, 'failure'); END")
        with pytest.raises(sqlite3.DatabaseError):
            node.claim_next_goal()
        assert node.state.current is None and node.state.order == ['one-p', 'one-d']
        db.execute('DROP TRIGGER fail')
        for _ in steps:
            complete_goal(node, FakeWeb(node), node.claim_next_goal())
        assert node.mission_status == 'returning_home'
        home = node.claim_next_goal()
        assert home.step_id is None and home.waypoint_id == 'HOME'
        assert node.claim_next_goal() is None
    with manager(path) as node:
        assert node.claim_next_goal() is None
        assert not node.handle_navigation_observation(home, NavigationObservation.ARRIVED)
        assert node.handle_recovery_observation(node.begin_recovery_check(), NavigationObservation.ARRIVED)
        assert node.mission_status == 'completed'


def test_legacy_snapshot_recovers_without_assuming_home(tmp_path):
    path = tmp_path / 'legacy.sqlite'
    with manager(path) as node:
        node.state.add_steps(job('one'))
        assert node.state.apply(mock_optimizer(node.state.request('HOME', 'r1')))
        for _ in range(2):
            complete_goal(node, FakeWeb(node), node.claim_next_goal())
        assert node.mission_status == 'completed'

    with sqlite3.connect(path) as db:
        snapshot = json.loads(db.execute('SELECT snapshot FROM mission').fetchone()[0])
        del snapshot['home_status']
        db.execute('UPDATE mission SET snapshot=?', (json.dumps(snapshot),))

    with manager(path) as node:
        assert node.state.completed == {'one-p', 'one-d'}
        assert node.mission_status == 'returning_home'
        goal = node.claim_next_goal()
        assert goal.step_id is None and goal.waypoint_id == 'HOME'
        assert node.handle_navigation_observation(goal, NavigationObservation.ARRIVED)
        assert node.mission_status == 'completed'

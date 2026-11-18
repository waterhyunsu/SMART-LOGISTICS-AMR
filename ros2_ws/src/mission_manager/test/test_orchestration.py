from dataclasses import replace
import json
import os
from pathlib import Path
import sqlite3
import subprocess
import sys

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


@pytest.mark.parametrize('target', ['step', 'home'])
def test_restart_requires_fresh_absence_and_explicit_retry(tmp_path, target):
    setup = """
import os
import sys
from test_contract import mock_optimizer
from test_dds import job
from test_orchestration import complete_goal
from test_recovery_integration import FakeWeb, manager
from mission_manager.node import NavigationObservation

with manager(sys.argv[1]) as node:
    if sys.argv[3] == 'claim':
        steps = job('one')
        steps[-1].waypoint_id = 'B'
        node.state.add_steps(steps)
        assert node.state.apply(mock_optimizer(node.state.request('HOME', 'r1')))
        if sys.argv[2] == 'home':
            for _ in steps:
                complete_goal(node, FakeWeb(node), node.claim_next_goal())
        assert node.claim_next_goal()
    else:
        check = node.begin_recovery_check()
        assert node.handle_recovery_observation(check, NavigationObservation.NO_ACTIVE_GOAL)
        assert node.approve_goal_retry(check)
    os._exit(0)
"""
    path = tmp_path / 'mission.sqlite'
    environment = {**os.environ, 'PYTHONPATH': os.pathsep.join(
        (str(Path(__file__).parent), os.environ.get('PYTHONPATH', '')))}
    subprocess.run([sys.executable, '-c', setup, str(path), target, 'claim'],
                   env=environment, check=True, timeout=15)
    with manager(path) as node:
        first = node.current_goal()
        assert first.attempt_id and node.state.recovery_required
        assert (first.step_id is None) == (target == 'home')
    subprocess.run([sys.executable, '-c', setup, str(path), target, 'retry'],
                   env=environment, check=True, timeout=15)
    with manager(path) as node:
        prior = node.current_goal()
        assert prior.attempt_id != first.attempt_id and node.state.recovery_required
        assert node.claim_next_goal() is None
        for result in (NavigationObservation.NOT_ARRIVED, NavigationObservation.DISCONNECTED,
                       NavigationObservation.TIMEOUT, NavigationObservation.UNKNOWN):
            old = node.begin_recovery_check()
            node.handle_recovery_observation(old, result)
            assert node.state.recovery_required and node.claim_next_goal() is None
            assert node.approve_goal_retry(old) is None
            assert not node.handle_recovery_observation(old, NavigationObservation.ARRIVED)
        check = node.begin_recovery_check()
        for mismatch in (replace(check, mission_id='other'), replace(check, step_id='old'),
                         replace(check, plan_version=0), replace(check, waypoint_id='other'),
                         replace(check, attempt_id=first.attempt_id)):
            assert not node.handle_recovery_observation(mismatch, NavigationObservation.NO_ACTIVE_GOAL)
            assert node.approve_goal_retry(mismatch) is None
        assert node.handle_recovery_observation(check, NavigationObservation.NO_ACTIVE_GOAL)
        assert node.current_goal() == prior and node.claim_next_goal() is None
        db = node.state.db
        db.execute("CREATE TRIGGER fail BEFORE UPDATE ON mission BEGIN SELECT RAISE(ABORT, 'failure'); END")
        with pytest.raises(sqlite3.DatabaseError):
            node.approve_goal_retry(check)
        assert node.current_goal() == prior and node.state.recovery_required
        db.execute('DROP TRIGGER fail')
        retry = node.approve_goal_retry(check)
        assert retry.attempt_id not in (first.attempt_id, prior.attempt_id)
        assert retry.step_id == prior.step_id and retry.plan_version == prior.plan_version
        assert node.approve_goal_retry(check) is None and node.claim_next_goal() is None
        for result in (NavigationObservation.ARRIVED, NavigationObservation.NOT_ARRIVED,
                       NavigationObservation.TIMEOUT):
            assert not node.handle_navigation_observation(prior, result)
        if target == 'home':
            assert node.handle_navigation_observation(retry, NavigationObservation.ARRIVED)
            assert not node.handle_navigation_observation(retry, NavigationObservation.ARRIVED)
            assert node.mission_status == 'completed'
        else:
            node.state.add_steps(job('new'))
            candidate = mock_optimizer(node.state.request(retry.waypoint_id, 'r1'))
            node.candidate = candidate
            assert not node.handle_navigation_observation(prior, NavigationObservation.ARRIVED)
            assert node.candidate is candidate and node.state.active_version == prior.plan_version
            complete_goal(node, FakeWeb(node), retry)
            assert FakeWeb(node).send('done:' + retry.step_id, retry.step_id) == 'accepted'
            assert node.state.active_version == prior.plan_version + 1
            assert node.claim_next_goal().step_id == 'one-d'


def test_legacy_execution_requires_observation_and_waiting_step_cannot_retry(tmp_path):
    path = tmp_path / 'legacy-active.sqlite'
    with manager(path) as node:
        node.state.add_steps(job('one'))
        assert node.state.apply(mock_optimizer(node.state.request('HOME', 'r1')))
        goal = node.claim_next_goal()
        assert node.handle_navigation_observation(goal, NavigationObservation.ARRIVED)
    with sqlite3.connect(path) as db:
        snapshot = json.loads(db.execute('SELECT snapshot FROM mission').fetchone()[0])
        del snapshot['attempt_id']
        db.execute('UPDATE mission SET snapshot=?', (json.dumps(snapshot),))
    with manager(path) as node:
        check = node.begin_recovery_check()
        assert check.attempt_id is None and node.approve_goal_retry(check) is None
        assert node.handle_recovery_observation(check, NavigationObservation.NO_ACTIVE_GOAL)
        assert node.approve_goal_retry(check) is None
        assert node.state.waiting and node.state.recovery_required
        check = node.begin_recovery_check()
        assert node.handle_recovery_observation(check, NavigationObservation.ARRIVED)
        assert FakeWeb(node).send('done') == 'accepted'

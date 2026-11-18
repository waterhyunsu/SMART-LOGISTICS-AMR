from contextlib import contextmanager
from dataclasses import replace
import sqlite3

import pytest
from rclpy.context import Context
from rclpy.parameter import Parameter

from mission_manager.node import MissionManager, NavigationObservation
from mission_manager.persistence import RecoveryPending
from test_contract import mock_optimizer
from test_dds import job


@contextmanager
def manager(path):
    context = Context()
    context.init(args=[])
    node = MissionManager(
        context=context, parameter_overrides=[Parameter('database_path', value=str(path))])
    try:
        yield node
    finally:
        node.destroy_node()
        context.shutdown()


@pytest.fixture
def database(tmp_path):
    path = tmp_path / 'mission.sqlite'
    with manager(path) as node:
        state = node.state
        state.add_steps(job('one'))
        assert state.apply(mock_optimizer(state.request('HOME', 'r1')))
        assert state.start_next() == 'one-p'
        assert node.handoff('early', state.mission_id, 'one-p') == 'rejected'
    return path


class FakeNavigation:
    def __init__(self, node):
        self.targets = []
        self.node = node

    def observe(self, outcome):
        context = self.node.begin_recovery_check()
        self.targets.append(context)
        return self.node.handle_recovery_observation(context, outcome)


class FakeWeb:
    def __init__(self, node):
        self.node = node
        self.acks = []

    def send(self, event, step='one-p', *, drop_ack=False):
        result = self.node.handoff(event, self.node.state.mission_id, step)
        if not drop_ack:
            self.acks.append(result)
        return result


@pytest.mark.parametrize('outcome,waiting', [
    (NavigationObservation.ARRIVED, True),
    (NavigationObservation.NOT_ARRIVED, False),
])
def test_verified_observation_only_reconciles(database, outcome, waiting):
    with manager(database) as node:
        d = FakeNavigation(node)
        e = FakeWeb(node)
        with pytest.raises(RecoveryPending):
            e.send('new')
        assert e.acks == []
        assert e.send('early') == 'rejected'
        assert d.observe(outcome)
        assert d.targets[0].waypoint_id == 'A'
        assert not node.state.recovery_required and node.state.waiting is waiting
        assert not node.state.completed and node.state.current == 'one-p'
        assert node.request is None and node.state.active_version == 1
        assert e.send('new') == ('accepted' if waiting else 'rejected')


@pytest.mark.parametrize('outcome', [
    NavigationObservation.UNKNOWN, NavigationObservation.DISCONNECTED,
    NavigationObservation.TIMEOUT,
])
def test_uncertain_and_late_observations_leave_recovery_blocked(database, outcome):
    with manager(database) as node:
        d = FakeNavigation(node)
        assert not d.observe(outcome)
        old = d.targets[0]
        active = node.begin_recovery_check()
        assert not node.handle_recovery_observation(old, NavigationObservation.ARRIVED)
        for mismatch in [replace(active, mission_id='old'), replace(active, step_id='old'),
                         replace(active, plan_version=0), replace(active, waypoint_id='B')]:
            assert not node.handle_recovery_observation(mismatch, NavigationObservation.ARRIVED)
        assert node.state.recovery_required and node.request is None
        with pytest.raises(RecoveryPending):
            node.state.start_next()
        assert node.handle_recovery_observation(active, NavigationObservation.ARRIVED)


def test_commit_failure_and_lost_ack_replay_across_restarts(database):
    with manager(database) as node:
        assert FakeNavigation(node).observe(NavigationObservation.ARRIVED)
        e = FakeWeb(node)
        db = node.state.db
        db.execute("CREATE TRIGGER fail BEFORE UPDATE ON mission BEGIN SELECT RAISE(ABORT, 'failure'); END")
        with pytest.raises(sqlite3.DatabaseError):
            e.send('done')
        assert e.acks == [] and not node.state.completed
        db.execute('DROP TRIGGER fail')
        assert e.send('done', drop_ack=True) == 'accepted'
        assert e.acks == []
    for _ in range(2):
        with manager(database) as node:
            e = FakeWeb(node)
            revision = node.state.revision
            assert e.send('done') == 'accepted'
            assert e.send('done', 'one-d') == 'conflict'
            assert e.send('early') == 'rejected'
            assert node.state.revision == revision
            assert node.state.completed == {'one-p'} and node.state.order == ['one-d']
            assert node.state.active_version == 1 and node.request is None


@pytest.mark.parametrize('ids', [('', 'm', 's'), ('e', '', 's'), ('e', 'm', ' '),
                                 ('e', None, 's')])
def test_invalid_event_ids_do_not_change_ledger(database, ids):
    with manager(database) as node:
        with pytest.raises(ValueError):
            node.handoff(*ids)
        assert set(node.state.handoff_results) == {'early'}

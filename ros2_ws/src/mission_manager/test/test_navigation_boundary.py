"""MOCK A/D integration: no real Nav2 goal or robot execution."""
from dataclasses import replace
import json
import sqlite3

import pytest

from mission_manager.navigation_boundary import (
    DispatchTicket, NavigationDispatchBoundary, RecoveryReport,
)
from mission_manager.node import NavigationObservation
from test_contract import mock_optimizer
from test_dds import job
from test_orchestration import complete_goal
from test_recovery_integration import FakeWeb, manager


class FakeD:
    """Tracks both Nav2-active UUIDs and delayed outbound submissions."""

    def __init__(self):
        self.pending = set()
        self.active = set()
        self.sent = []

    def send_pending(self, ticket):
        self.sent.append(ticket)
        self.pending.add(ticket.nav_goal_id)

    def send_active(self, ticket):
        self.sent.append(ticket)
        self.active.add(ticket.nav_goal_id)

    def inspection(self, lookup):
        if lookup.nav_goal_id in self.pending or lookup.nav_goal_id in self.active:
            return RecoveryReport(lookup, NavigationObservation.UNKNOWN)
        return RecoveryReport(
            lookup, NavigationObservation.NO_ACTIVE_GOAL, no_pending_dispatch=True)

    def rule_out_all_late_sends(self, goal_id):
        self.pending.discard(goal_id)
        self.active.discard(goal_id)


def setup_goal(path, *, home=False):
    with manager(path) as node:
        steps = job('one')
        if home:
            steps[-1].waypoint_id = 'B'
        state = node.state
        state.add_steps(steps)
        assert state.apply(mock_optimizer(state.request('HOME', 'r1')))
        if home:
            for _ in steps:
                complete_goal(node, FakeWeb(node), node.claim_next_goal())
        goal = node.claim_next_goal()
        boundary = NavigationDispatchBoundary(node)
        ticket = boundary.prepare_dispatch(goal)
        assert ticket is not None
        assert ticket.goal.attempt_id == goal.attempt_id
        assert node.state.nav_goal_id == ticket.nav_goal_id
        return ticket


def test_persist_before_submit_and_at_most_once(tmp_path):
    path = tmp_path / 'first.sqlite'
    with manager(path) as node:
        state = node.state
        state.add_steps(job('one'))
        assert state.apply(mock_optimizer(state.request('HOME', 'r1')))
        goal = node.claim_next_goal()
        d = FakeD()
        boundary = NavigationDispatchBoundary(node)
        ticket = boundary.prepare_dispatch(goal)
        assert ticket and len(ticket.nav_goal_id) == 32
        assert boundary.prepare_dispatch(goal) is None
        assert state.nav_goal_id == ticket.nav_goal_id and d.sent == []
        assert boundary.submit_once(ticket, d.send_active)
        assert not boundary.submit_once(ticket, d.send_active)
        assert len(d.sent) == 1
        assert not boundary.observe(
            DispatchTicket(goal, '0' * 32), NavigationObservation.ARRIVED)
        assert boundary.observe(ticket, NavigationObservation.ARRIVED)
        assert not boundary.observe(ticket, NavigationObservation.ARRIVED)
        assert state.waiting
    with manager(path) as node:
        assert node.state.recovery_required
        assert node.state.nav_goal_id == ticket.nav_goal_id
        assert NavigationDispatchBoundary(node).prepare_dispatch(node.current_goal()) is None


@pytest.mark.parametrize('home', [False, True])
def test_restart_delayed_send_requires_absence_proof_and_approval(tmp_path, home):
    path = tmp_path / 'goal.sqlite'
    original = setup_goal(path, home=home)
    d = FakeD()
    d.send_pending(original)  # A died with a D-side delayed outbound request.
    with manager(path) as node:
        assert node.state.recovery_required
        bridge = NavigationDispatchBoundary(node)
        old_lookup = bridge.begin_recovery_lookup()
        assert old_lookup.nav_goal_id == original.nav_goal_id
        assert old_lookup.check.attempt_id == original.goal.attempt_id
        assert not bridge.handle_recovery_report(d.inspection(old_lookup))
        assert node.state.recovery_required

        lookup = bridge.begin_recovery_lookup()
        assert not bridge.handle_recovery_report(
            RecoveryReport(lookup, NavigationObservation.NO_ACTIVE_GOAL))
        assert bridge.approve_retry(lookup, explicitly_approved=True) is None
        assert bridge.prepare_dispatch(node.current_goal()) is None

        # Trusted D explicitly rules out active Nav2 execution and late delivery.
        d.rule_out_all_late_sends(original.nav_goal_id)
        assert bridge.handle_recovery_report(d.inspection(lookup))
        assert not bridge.handle_recovery_report(
            RecoveryReport(old_lookup, NavigationObservation.ARRIVED))
        assert bridge.approve_retry(lookup) is None
        assert node.state.nav_goal_id == original.nav_goal_id
        new_goal = bridge.approve_retry(lookup, explicitly_approved=True)
        assert new_goal is not None
        assert new_goal.attempt_id != original.goal.attempt_id
        assert node.state.nav_goal_id is None
        assert bridge.approve_retry(lookup, explicitly_approved=True) is None
        assert d.sent == [original]

        new_ticket = bridge.prepare_dispatch(new_goal)
        assert new_ticket and new_ticket.nav_goal_id != original.nav_goal_id
        assert not bridge.observe(original, NavigationObservation.ARRIVED)
        assert bridge.submit_once(new_ticket, d.send_active)
        assert len(d.sent) == 2
        assert bridge.observe(new_ticket, NavigationObservation.ARRIVED)
        if home:
            assert node.mission_status == 'completed'
        else:
            assert node.state.waiting
            assert node.handoff('done', node.state.mission_id, new_ticket.goal.step_id) == 'accepted'


def test_claim_crash_before_uuid_binding_requires_fresh_check(tmp_path):
    path = tmp_path / 'unbound.sqlite'
    with manager(path) as node:
        s = node.state
        s.add_steps(job('one'))
        assert s.apply(mock_optimizer(s.request('HOME', 'r1')))
        first = node.claim_next_goal()
        assert first.attempt_id and s.nav_goal_id is None
    with manager(path) as node:
        bridge = NavigationDispatchBoundary(node)
        lookup = bridge.begin_recovery_lookup()
        assert lookup.nav_goal_id is None
        assert not bridge.handle_recovery_report(
            RecoveryReport(lookup, NavigationObservation.NO_ACTIVE_GOAL))
        assert bridge.handle_recovery_report(
            RecoveryReport(lookup, NavigationObservation.NO_ACTIVE_GOAL,
                           no_pending_dispatch=True))
        assert bridge.approve_retry(lookup, explicitly_approved=True)
        new_goal = node.current_goal()
        assert new_goal.attempt_id != first.attempt_id
        assert bridge.prepare_dispatch(new_goal)


def test_mapping_commit_failure_never_exposes_dispatch_ticket(tmp_path):
    path = tmp_path / 'commit.sqlite'
    with manager(path) as node:
        s = node.state
        s.add_steps(job('one'))
        assert s.apply(mock_optimizer(s.request('HOME', 'r1')))
        goal = node.claim_next_goal()
        revision = s.revision
        bridge = NavigationDispatchBoundary(node)
        d = FakeD()
        s.db.execute(
            "CREATE TRIGGER fail BEFORE UPDATE ON mission BEGIN SELECT RAISE(ABORT, 'disk failure'); END")
        with pytest.raises(sqlite3.DatabaseError):
            bridge.prepare_dispatch(goal)
        assert s.nav_goal_id is None and s.revision == revision and not d.sent
        s.db.execute('DROP TRIGGER fail')
        ticket = bridge.prepare_dispatch(goal)
        assert ticket and s.nav_goal_id == ticket.nav_goal_id
        assert bridge.submit_once(ticket, d.send_active)
        assert len(d.sent) == 1


def test_sender_exception_is_not_automatically_retried(tmp_path):
    path = tmp_path / 'send.sqlite'
    ticket = setup_goal(path)
    with manager(path) as node:
        assert node.state.recovery_required
        # The first process prepared the UUID; only D inspection can recover.
        assert not NavigationDispatchBoundary(node).submit_once(
            ticket, lambda _: None)
    with manager(tmp_path / 'fresh.sqlite') as node:
        s = node.state
        s.add_steps(job('fresh'))
        assert s.apply(mock_optimizer(s.request('HOME', 'r1')))
        bridge = NavigationDispatchBoundary(node)
        ticket = bridge.prepare_dispatch(node.claim_next_goal())
        def broken_sender(_):
            raise ConnectionError('ambiguous send')
        with pytest.raises(ConnectionError):
            bridge.submit_once(ticket, broken_sender)
        assert not bridge.submit_once(ticket, broken_sender)
        assert s.nav_goal_id == ticket.nav_goal_id


def test_retry_commit_failure_and_expired_absence(tmp_path):
    path = tmp_path / 'retry.sqlite'
    original = setup_goal(path)
    with manager(path) as node:
        bridge = NavigationDispatchBoundary(node)
        lookup = bridge.begin_recovery_lookup()
        assert not bridge.handle_recovery_report(
            RecoveryReport(replace(lookup, nav_goal_id='f' * 32),
                           NavigationObservation.NO_ACTIVE_GOAL, True))
        assert bridge.handle_recovery_report(
            RecoveryReport(lookup, NavigationObservation.NO_ACTIVE_GOAL,
                           no_pending_dispatch=True))
        before = node.current_goal()
        node.state.db.execute(
            "CREATE TRIGGER fail BEFORE UPDATE ON mission BEGIN SELECT RAISE(ABORT, 'disk failure'); END")
        with pytest.raises(sqlite3.DatabaseError):
            bridge.approve_retry(lookup, explicitly_approved=True)
        assert node.current_goal() == before
        assert node.state.nav_goal_id == original.nav_goal_id
        assert node.state.recovery_required
        node.state.db.execute('DROP TRIGGER fail')
        # Starting a new check makes older absence evidence invalid.
        newer = bridge.begin_recovery_lookup()
        assert bridge.approve_retry(lookup, explicitly_approved=True) is None
        assert not bridge.handle_recovery_report(
            RecoveryReport(lookup, NavigationObservation.NO_ACTIVE_GOAL, True))
        assert bridge.handle_recovery_report(
            RecoveryReport(newer, NavigationObservation.NO_ACTIVE_GOAL, True))
        assert bridge.approve_retry(newer, explicitly_approved=True)
        assert node.state.nav_goal_id is None


def test_legacy_snapshot_without_mapping_still_requires_observation(tmp_path):
    path = tmp_path / 'legacy.sqlite'
    original = setup_goal(path)
    with sqlite3.connect(path) as db:
        record = json.loads(db.execute('SELECT snapshot FROM mission').fetchone()[0])
        del record['nav_goal_id']
        db.execute('UPDATE mission SET snapshot=?', (json.dumps(record),))
    with manager(path) as node:
        assert node.state.nav_goal_id is None
        lookup = NavigationDispatchBoundary(node).begin_recovery_lookup()
        assert lookup.nav_goal_id is None
        assert lookup.check.attempt_id == original.goal.attempt_id
        assert node.state.recovery_required

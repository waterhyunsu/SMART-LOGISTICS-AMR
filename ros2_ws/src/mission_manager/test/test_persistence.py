import os
import sqlite3
import subprocess
import sys

import pytest

from mission_manager.persistence import PersistentMission
from test_dds import job
from test_contract import mock_optimizer


def waiting(path):
    state = PersistentMission(str(path), 'mission')
    state.add_steps(job('one'))
    assert state.apply(mock_optimizer(state.request('HOME', 'r1')))
    state.start_next()
    state.arrive('one-p')
    return state


def test_restore_and_event_replay(tmp_path):
    path = tmp_path / 'mission.sqlite'
    state = waiting(path)
    issued = state.latest_request_id
    state.close()
    state = PersistentMission(str(path), 'unused-new-id')
    assert state.mission_id == 'mission'
    assert state.waiting and state.current == 'one-p' and state.active_version == 1
    assert state.order == ['one-d'] and state.recovery_required
    with pytest.raises(RuntimeError):
        state.handoff('done', 'mission', 'one-p')
    state.reconcile('one-p', arrived=True)
    assert state.handoff('bad', 'mission', 'one-d') == 'rejected'
    assert state.handoff('done', 'mission', 'one-p') == 'accepted'
    state.close()
    state = PersistentMission(str(path), 'ignored')
    assert state.handoff('done', 'mission', 'one-p') == 'accepted'
    assert state.handoff('done', 'mission', 'one-d') == 'conflict'
    assert state.handoff('bad', 'mission', 'one-d') == 'rejected'
    assert state.completed == {'one-p'}
    assert state.request('A', 'r1').request_id > issued
    state.close()


def test_failed_commit_has_no_memory_or_disk_transition(tmp_path):
    path = tmp_path / 'mission.sqlite'
    state = waiting(path)
    state.db.execute("CREATE TRIGGER fail BEFORE UPDATE ON mission BEGIN SELECT RAISE(ABORT, 'disk failure'); END")
    state.db.commit()
    with pytest.raises(sqlite3.DatabaseError):
        state.handoff('done', 'mission', 'one-p')
    assert not state.completed and not state.handoff_results and state.waiting
    state.db.execute('DROP TRIGGER fail')
    state.db.commit()
    state.close()
    state = PersistentMission(str(path), 'ignored')
    assert not state.completed and not state.handoff_results
    state.close()


def test_stale_writer_cannot_overwrite(tmp_path):
    path = str(tmp_path / 'mission.sqlite')
    first = PersistentMission(path, 'mission')
    second = PersistentMission(path, 'mission')
    with pytest.raises(RuntimeError, match='another writer'):
        first.add_steps(job('one'))
    assert not first.steps
    first.close()
    second.close()


def test_node_process_restart_after_commit_without_ack(tmp_path):
    path = str(tmp_path / 'mission.sqlite')
    code = """
import os, sys
import rclpy
from rclpy.parameter import Parameter
from mission_manager.node import MissionManager
from test_dds import job
from test_contract import mock_optimizer
rclpy.init()
node = MissionManager(parameter_overrides=[Parameter('database_path', value=sys.argv[1])])
s = node.state
if sys.argv[2] == 'write':
    s.add_steps(job('one'))
    s.apply(mock_optimizer(s.request('HOME', 'r1')))
    s.start_next()
    node.arrive('one-p')
    assert node.handoff('done', s.mission_id, 'one-p') == 'accepted'
    os._exit(0)  # crash before an external caller receives an ACK
assert s.completed == {'one-p'}
assert node.handoff('done', s.mission_id, 'one-p') == 'accepted'
assert node.handoff('done', s.mission_id, 'one-d') == 'conflict'
assert s.active_version == 1 and s.request('A','r1').request_id == 2
assert s.start_next() == 'one-d'
node.destroy_node()
rclpy.shutdown()
"""
    environment = {**os.environ, 'PYTHONPATH': os.path.dirname(__file__) + os.pathsep + os.environ.get('PYTHONPATH', '')}
    for mode in ('write', 'read'):
        subprocess.run([sys.executable, '-c', code, path, mode], env=environment, check=True, timeout=15)

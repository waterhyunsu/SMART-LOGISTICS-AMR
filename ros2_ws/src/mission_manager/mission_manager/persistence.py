"""A-owned SQLite snapshot and atomic copy-on-write state transitions."""

from copy import deepcopy
import json
import sqlite3

from logistics_msgs.msg import TaskStep
from mission_manager.contract import MissionState


class PersistentMission:
    transitions = frozenset({
        'add_steps', 'update_cost_revision', 'request', 'apply',
        'start_next', 'arrive', 'confirm', 'handoff',
    })

    def __init__(self, path, mission_id):
        self.db = sqlite3.connect(path)
        self.db.execute('PRAGMA synchronous=FULL')
        self.db.execute(
            'CREATE TABLE IF NOT EXISTS mission '
            '(id INTEGER PRIMARY KEY CHECK(id=1), revision INTEGER NOT NULL, snapshot TEXT NOT NULL)')
        row = self.db.execute('SELECT revision, snapshot FROM mission WHERE id=1').fetchone()
        self.revision = row[0] if row else 0
        self.recovered = row is not None
        if row:
            values = json.loads(row[1])
            if values.pop('schema') != 1:
                raise ValueError('unsupported mission snapshot schema')
            steps = [TaskStep(**step) for step in values.pop('steps')]
            self._state = MissionState(values['mission_id'], steps)
            values['completed'] = set(values['completed'])
            self._state.__dict__.update(values)
        else:
            self._state = MissionState(mission_id, [])
        self.recovery_required = bool(self.recovered and self._state.current)
        # Requests and deferred candidates never survive a process restart.
        candidate = deepcopy(self._state)
        candidate.pending_request_id = None
        self._commit(candidate)

    def _commit(self, candidate):
        values = deepcopy(candidate.__dict__)
        values['schema'] = 1
        values['steps'] = [
            {field: getattr(step, field) for field in step.get_fields_and_field_types()}
            for step in candidate.steps.values()]
        values['completed'] = sorted(candidate.completed)
        snapshot = json.dumps(values)
        with self.db:
            if self.revision == 0:
                self.db.execute('INSERT INTO mission VALUES (1, 1, ?)', (snapshot,))
            else:
                cursor = self.db.execute(
                    'UPDATE mission SET revision=revision+1, snapshot=? WHERE id=1 AND revision=?',
                    (snapshot, self.revision))
                if cursor.rowcount != 1:
                    raise RuntimeError('mission changed by another writer; reopen before retry')
        self.revision += 1
        self._state = candidate

    def __getattr__(self, name):
        if name in self.transitions:
            def transition(*args, **kwargs):
                if self.recovery_required:
                    if name == 'handoff' and args[0] in self._state.handoff_results:
                        # Replay stored results, including key conflicts, without a new transition.
                        return self._state.handoff(*args, **kwargs)
                    raise RuntimeError('external step reconciliation required after restart')
                candidate = deepcopy(self._state)
                result = getattr(candidate, name)(*args, **kwargs)
                self._commit(candidate)
                return result
            return transition
        value = getattr(self._state, name)
        return value if callable(value) else deepcopy(value)

    def reconcile(self, step_id, *, arrived):
        """A adapter calls this only after external/operator state confirmation."""
        if not self.recovery_required or step_id != self._state.current:
            raise ValueError('no matching recovered step')
        if type(arrived) is not bool:
            raise ValueError('arrived must be bool')
        candidate = deepcopy(self._state)
        candidate.waiting = arrived
        self._commit(candidate)
        self.recovery_required = False

    def invalidate_request(self):
        candidate = deepcopy(self._state)
        candidate.pending_request_id = None
        self._commit(candidate)

    def close(self):
        self.db.close()

"""A-owned SQLite snapshot and atomic copy-on-write state transitions."""

from copy import deepcopy
import json
import sqlite3
from uuid import uuid4

from logistics_msgs.msg import TaskStep
from mission_manager.contract import MissionState


class RecoveryPending(RuntimeError):
    """Retryable result: no new event has been recorded during recovery."""


class PersistentMission:
    transitions = frozenset({
        'add_steps', 'update_cost_revision', 'request', 'apply',
        'start_next', 'arrive', 'confirm', 'start_home_return', 'arrive_home',
        'bind_nav_goal',
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
            try:
                self._state = self._restore(row[1])
            except (ValueError, TypeError, KeyError, AssertionError) as error:
                self.db.close()
                raise ValueError('invalid mission snapshot') from error
        else:
            self._state = MissionState(mission_id, [])
        self.recovery_required = bool(self.recovered and (self._state.current or self._state.home_status == 'returning'))
        # Requests and deferred candidates never survive a process restart.
        candidate = deepcopy(self._state)
        candidate.pending_request_id = None
        self._commit(candidate)

    @staticmethod
    def _restore(snapshot):
        values = json.loads(snapshot)
        if values.pop('schema') != 1:
            raise ValueError('unsupported mission snapshot schema')
        # Existing schema-1 snapshots did not track the final verified waypoint.
        values.setdefault('home_status', 'away')
        values.setdefault('attempt_id', None)
        values.setdefault('nav_goal_id', None)
        steps = [TaskStep(**step) for step in values['steps']]
        state = MissionState(values['mission_id'], steps)
        if set(values) != set(state.__dict__):
            raise ValueError('snapshot fields do not match schema')
        for field in ('active_version', 'latest_request_id'):
            if type(values[field]) is not int or not 0 <= values[field] < 2**64:
                raise ValueError('invalid version counter')
        pending = values['pending_request_id']
        if pending is not None and (
                type(pending) is not int or not 0 < pending <= values['latest_request_id']):
            raise ValueError('invalid pending request')
        if type(values['waiting']) is not bool:
            raise ValueError('invalid waiting flag')
        current = values['current']
        if current is not None and current not in state.steps:
            raise ValueError('unknown current step')
        if values['waiting'] and current is None:
            raise ValueError('waiting without current step')
        revision = values['cost_revision']
        if revision is not None and (not isinstance(revision, str) or not revision.strip()):
            raise ValueError('invalid cost revision')
        for field in ('completed', 'order'):
            items = values[field]
            if (not isinstance(items, list) or len(items) != len(set(items))
                    or not set(items) <= state.steps.keys()):
                raise ValueError('invalid step collection')
        completed = set(values['completed'])
        if current in completed or completed.intersection(values['order']) or current in values['order']:
            raise ValueError('overlapping step states')
        for key in completed:
            if not completed.issuperset(state.steps[key].prerequisite_ids):
                raise ValueError('completed step has incomplete prerequisites')
        seen = completed.copy()
        for key in ([current] if current else []) + values['order']:
            if not seen.issuperset(state.steps[key].prerequisite_ids):
                raise ValueError('plan violates prerequisites')
            seen.add(key)
        if values['home_status'] not in ('away', 'returning', 'home'):
            raise ValueError('invalid HOME status')
        if values['home_status'] == 'returning' and (
                not state.steps or completed != state.steps.keys()):
            raise ValueError('HOME return with unfinished steps')
        attempt_id = values['attempt_id']
        if attempt_id is not None and (
                not isinstance(attempt_id, str) or not attempt_id.strip()
                or (current is None and values['home_status'] != 'returning')):
            raise ValueError('invalid execution attempt')
        nav_goal_id = values['nav_goal_id']
        if nav_goal_id is not None and (
                not isinstance(nav_goal_id, str) or len(nav_goal_id) != 32
                or any(char not in '0123456789abcdef' for char in nav_goal_id)
                or attempt_id is None
                or (current is None and values['home_status'] != 'returning')):
            raise ValueError('invalid Nav2 Goal ID mapping')
        events = values['handoff_results']
        if not isinstance(events, dict):
            raise ValueError('invalid event ledger')
        for event_id, result in events.items():
            if (not event_id.strip() or not isinstance(result, list) or len(result) != 2
                    or not isinstance(result[0], str) or result[1] not in ('accepted', 'rejected')
                    or (result[1] == 'accepted' and result[0] not in completed)):
                raise ValueError('invalid event result')
        values['steps'] = state.steps
        values['completed'] = completed
        state.__dict__.update(values)
        return state

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
                    raise RecoveryPending('external step reconciliation required after restart')
                candidate = deepcopy(self._state)
                result = getattr(candidate, name)(*args, **kwargs)
                self._commit(candidate)
                return result
            return transition
        value = getattr(self._state, name)
        return value if callable(value) else deepcopy(value)

    def handoff(self, event_id, mission_id, step_id):
        outcome, _ = self.handoff_transition(event_id, mission_id, step_id)
        return outcome

    def handoff_transition(self, event_id, mission_id, step_id):
        """Return the ACK and whether this call durably completed the current step."""
        if event_id in self._state.handoff_results:
            return self._state.handoff(event_id, mission_id, step_id), False
        if self.recovery_required:
            raise RecoveryPending('external step reconciliation required after restart')
        candidate = deepcopy(self._state)
        outcome = candidate.handoff(event_id, mission_id, step_id)
        completed_now = (outcome == 'accepted' and step_id == self._state.current
                         and candidate.current is None)
        self._commit(candidate)
        return outcome, completed_now

    def reconcile(self, step_id, *, arrived):
        """A adapter calls this only after external/operator state confirmation."""
        if not self.recovery_required or step_id != self._state.current:
            raise ValueError('no matching recovered step')
        if type(arrived) is not bool:
            raise ValueError('arrived must be bool')
        candidate = deepcopy(self._state)
        if step_id is not None:
            candidate.waiting = arrived
        elif arrived:
            candidate.arrive_home()
        self._commit(candidate)
        self.recovery_required = not arrived

    def _commit_recovery_retry(self, previous_attempt_id):
        """Persist a new attempt after the A boundary authorizes retry."""
        if (not self.recovery_required or self._state.waiting
                or self._state.attempt_id != previous_attempt_id):
            raise ValueError('no matching recoverable execution attempt')
        candidate = deepcopy(self._state)
        candidate.attempt_id = uuid4().hex
        candidate.nav_goal_id = None
        self._commit(candidate)
        self.recovery_required = False

    def invalidate_request(self):
        candidate = deepcopy(self._state)
        candidate.pending_request_id = None
        self._commit(candidate)

    def close(self):
        self.db.close()

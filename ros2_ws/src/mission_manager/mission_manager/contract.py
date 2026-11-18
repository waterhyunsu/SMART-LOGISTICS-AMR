"""In-memory A-side contract fixture for MOCK. No ROS nodes or robot I/O."""

from copy import deepcopy
from graphlib import CycleError, TopologicalSorter
from math import isfinite

from logistics_msgs.msg import TaskStep
from logistics_msgs.srv import OptimizeRoute


class MockMission:
    def __init__(self, mission_id, steps):
        if not mission_id.strip():
            raise ValueError('missing mission ID')
        self.mission_id = mission_id
        self.steps = {}
        self.pending_request_id = None
        self.handoff_results = {}
        self.add_steps(steps)
        self.completed = set()
        self.current = None
        self.waiting = False
        self.active_version = 0
        self.order = []
        self.cost_revision = None
        self.latest_request_id = 0

    def add_steps(self, steps):
        """Register complete pickup/dropoff pairs atomically at order intake."""
        additions = {}
        jobs = {}
        known_jobs = {s.job_id for s in self.steps.values()}
        for step in steps:
            if not all(value.strip() for value in (step.job_id, step.step_id, step.waypoint_id)):
                raise ValueError('missing job, step or waypoint ID')
            if step.step_id in self.steps or step.step_id in additions or step.job_id in known_jobs:
                raise ValueError('duplicate job or step')
            pair = jobs.setdefault(step.job_id, {})
            if step.kind in pair or step.kind not in (TaskStep.PICKUP, TaskStep.DROPOFF):
                raise ValueError('invalid or repeated step kind')
            pair[step.kind] = step
            additions[step.step_id] = step

        for pair in jobs.values():
            pickup = pair.get(TaskStep.PICKUP)
            dropoff = pair.get(TaskStep.DROPOFF)
            if not pickup or not dropoff or pickup.step_id not in dropoff.prerequisite_ids:
                raise ValueError('delivery requires its own pickup')
        for step in additions.values():
            if not set(step.prerequisite_ids) <= self.steps.keys() | additions.keys():
                raise ValueError('unknown prerequisite')
        # Existing steps were validated at intake and cannot depend on a new ID.
        graph = TopologicalSorter({key: step.prerequisite_ids for key, step in additions.items()})
        try:
            graph.prepare()
        except CycleError as error:
            raise ValueError('cyclic step prerequisites') from error
        self.steps.update(deepcopy(additions))
        if additions:
            self.pending_request_id = None

    def update_cost_revision(self, cost_revision):
        if not cost_revision.strip():
            raise ValueError('missing cost revision')
        if cost_revision != self.cost_revision:
            self.cost_revision = cost_revision
            self.pending_request_id = None

    def request(self, start_waypoint_id, cost_revision):
        if not start_waypoint_id.strip():
            raise ValueError('missing start waypoint')
        if self.current and start_waypoint_id != self.steps[self.current].waypoint_id:
            raise ValueError('start waypoint must match the current step destination')
        request = OptimizeRoute.Request(
            request_id=self.latest_request_id + 1,
            mission_id=self.mission_id, base_version=self.active_version,
            start_waypoint_id=start_waypoint_id, cost_revision=cost_revision,
            current_step_id=self.current or '', completed_step_ids=sorted(self.completed),
            pending_steps=[deepcopy(s) for key, s in self.steps.items()
                           if key != self.current and key not in self.completed])

        self.update_cost_revision(cost_revision)
        self.pending_request_id = request.request_id
        self.latest_request_id = request.request_id
        return request

    def apply(self, result):
        """Validate against live state at the step boundary before replacing the suffix."""
        if (self.current or not result.success or result.mission_id != self.mission_id
                or result.request_id != self.pending_request_id
                or result.base_version != self.active_version
                or result.cost_revision != self.cost_revision
                or not isfinite(result.estimated_distance_m) or result.estimated_distance_m < 0):
            return False
        order = list(result.ordered_step_ids)
        pending = self.steps.keys() - self.completed
        if len(order) != len(pending) or set(order) != pending:
            return False
        seen = self.completed.copy()
        for key in order:
            if not seen.issuperset(self.steps[key].prerequisite_ids):
                return False
            seen.add(key)
        self.order = order
        self.pending_request_id = None
        self.active_version += 1
        return True

    def start_next(self):
        if self.current is None and self.order:
            self.current = self.order.pop(0)
        return self.current

    def arrive(self, step_id):
        if not self.current or self.current != step_id:
            return False
        self.waiting = True
        return True

    def confirm(self, mission_id, step_id):
        if mission_id != self.mission_id or step_id != self.current or not self.waiting:
            return False
        self.completed.add(step_id)
        self.current = None
        self.waiting = False
        return True

    def handoff(self, event_id, mission_id, step_id):
        """Process one operator event; replay the recorded outcome in this MOCK instance."""
        if not event_id.strip() or mission_id != self.mission_id:
            return 'rejected'
        if event_id in self.handoff_results:
            original_step, outcome = self.handoff_results[event_id]
            return outcome if original_step == step_id else 'conflict'
        outcome = 'accepted' if self.confirm(mission_id, step_id) else 'rejected'
        self.handoff_results[event_id] = (step_id, outcome)
        return outcome

"""A-owned Nav2 dispatch contract. No Nav2 client or robot I/O.

The trusted D adapter must use the persisted UUID for the Nav2 goal and inspect
both active goals and delayed outbound sends after a process restart.
"""
from dataclasses import dataclass
from uuid import uuid4

from mission_manager.node import NavigationGoal, NavigationObservation, RecoveryCheck


@dataclass(frozen=True)
class DispatchTicket:
    goal: NavigationGoal
    nav_goal_id: str  # Nav2 UUID as lowercase hexadecimal, 32 characters


@dataclass(frozen=True)
class RecoveryLookup:
    check: RecoveryCheck
    nav_goal_id: str | None


@dataclass(frozen=True)
class RecoveryReport:
    lookup: RecoveryLookup
    observation: NavigationObservation
    no_pending_dispatch: bool = False


class NavigationDispatchBoundary:
    """Single-executor A/D bridge: persist before handing a goal to D."""

    def __init__(self, manager):
        self.manager = manager
        self._prepared = set()
        self._lookup = None
        self._absence = None

    def prepare_dispatch(self, goal):
        """Persist one Nav2 UUID for this attempt before any external send."""
        state = self.manager.state
        if (not isinstance(goal, NavigationGoal)
                or goal != self.manager.current_goal()
                or state.recovery_required or state.waiting
                or not goal.attempt_id or state.nav_goal_id is not None):
            return None
        nav_goal_id = uuid4().hex
        state.bind_nav_goal(goal.attempt_id, nav_goal_id)
        self._prepared.add(nav_goal_id)
        return DispatchTicket(goal, nav_goal_id)

    def submit_once(self, ticket, sender):
        """Invoke the D sender once; invocation does NOT prove Nav2 accepted it.

        Even if sending throws, do not automatically retry the same ticket.
        Reconcile with D before authorizing a new execution attempt.
        """
        if (not isinstance(ticket, DispatchTicket)
                or ticket.nav_goal_id not in self._prepared
                or self.manager.state.recovery_required
                or ticket.goal != self.manager.current_goal()
                or self.manager.state.nav_goal_id != ticket.nav_goal_id):
            return False
        self._prepared.remove(ticket.nav_goal_id)
        sender(ticket)
        return True

    def observe(self, ticket, observation):
        """Ignore any result not matching the current attempt AND Nav2 UUID."""
        if (not isinstance(ticket, DispatchTicket)
                or ticket.goal != self.manager.current_goal()
                or self.manager.state.recovery_required
                or self.manager.state.nav_goal_id != ticket.nav_goal_id):
            return False
        return self.manager.handle_navigation_observation(ticket.goal, observation)

    def begin_recovery_lookup(self):
        """Create a new correlation ID for inspection of the persisted UUID."""
        self._absence = None
        check = self.manager.begin_recovery_check()
        self._lookup = RecoveryLookup(check, self.manager.state.nav_goal_id)
        return self._lookup

    def handle_recovery_report(self, report):
        """NO_ACTIVE_GOAL also requires proof that delayed dispatch is excluded."""
        if (not isinstance(report, RecoveryReport) or report.lookup != self._lookup
                or report.lookup.nav_goal_id != self.manager.state.nav_goal_id
                or not isinstance(report.observation, NavigationObservation)):
            return False
        if (report.observation is NavigationObservation.NO_ACTIVE_GOAL
                and report.no_pending_dispatch is not True):
            return False
        accepted = self.manager.handle_recovery_observation(
            report.lookup.check, report.observation)
        self._lookup = None
        self._absence = (
            report.lookup if accepted and
            report.observation is NavigationObservation.NO_ACTIVE_GOAL else None)
        return accepted

    def approve_retry(self, lookup, *, explicitly_approved=False):
        """A authorizes one retry after trusted D absence proof; no auto-send."""
        if (explicitly_approved is not True or lookup != self._absence
                or self.manager.state.recovery_required is not True):
            return None
        goal = self.manager.approve_goal_retry(lookup.check)
        if goal is not None:
            self._absence = None
        return goal

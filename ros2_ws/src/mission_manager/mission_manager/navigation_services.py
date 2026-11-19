"""Gate 1 ROS services for the A-owned navigation boundary.

Run on the Mission Manager's single-threaded executor. Only the caller may
start a dispatch or explicitly approve a retry.
"""
from dataclasses import dataclass
from math import isfinite
from time import monotonic

from rclpy.clock import Clock, ClockType

from logistics_msgs.srv import DispatchGoal, LookupGoal, ReportGoalEvent
from mission_manager.navigation_boundary import (
    DispatchTicket, NavigationDispatchBoundary, RecoveryReport,
)
from mission_manager.node import NavigationObservation


def _execution_context(goal, goal_id):
    return dict(mission_id=goal.mission_id, step_id=goal.step_id or '',
                plan_version=goal.plan_version, waypoint_id=goal.waypoint_id,
                attempt_id=goal.attempt_id, nav_goal_id=goal_id)


def _matches(message, context):
    return all(getattr(message, field) == value for field, value in context.items())


def _response(future):
    try:
        return future.result()
    except Exception:
        # Service exceptions are inconclusive observations.
        return None


@dataclass
class _Dispatch:
    ticket: DispatchTicket
    receipt: object
    deadline: float
    accepted: bool = False
    event_seq: int = 0


class NavigationServices:
    """Translate wire requests to the existing durable dispatch/recovery API."""

    def __init__(self, manager, *, timeout_sec=5.0):
        if not isfinite(timeout_sec) or timeout_sec <= 0:
            raise ValueError('navigation timeout must be finite and positive')
        self.manager = manager
        self.boundary = NavigationDispatchBoundary(manager)
        self.timeout = timeout_sec
        self.dispatch_client = manager.create_client(DispatchGoal, 'dispatch_goal')
        self.lookup_client = manager.create_client(LookupGoal, 'lookup_goal')
        self.event_service = manager.create_service(
            ReportGoalEvent, 'report_goal_event', self._report_event)
        self.timer = manager.create_timer(
            min(0.05, self.timeout), self._poll, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self._dispatch = None
        self._lookup = None
        self.last_report = None

    def dispatch(self, goal):
        """Send a claimed goal once. True means the ROS request was submitted."""
        if not self.dispatch_client.service_is_ready():
            return False
        ticket = self.boundary.prepare_dispatch(goal)
        if ticket is None:
            return False

        def send(ticket):
            request = DispatchGoal.Request(**_execution_context(ticket.goal, ticket.nav_goal_id))
            future = self.dispatch_client.call_async(request)
            self._dispatch = _Dispatch(ticket, future, monotonic() + self.timeout)

        try:
            return self.boundary.submit_once(ticket, send)
        except Exception as exc:
            self.manager.get_logger().warning(f'Dispatch stopped for recovery: {exc}')
            return False

    def lookup(self):
        """Start a correlated lookup for the current bound UUID under recovery."""
        state = self.manager.state
        if (self._lookup is not None or not state.recovery_required
                or not state.nav_goal_id):
            return None
        lookup = self.boundary.begin_recovery_lookup()
        request = LookupGoal.Request(
            check_id=lookup.check.check_id, **_execution_context(lookup.check, lookup.nav_goal_id))
        self.last_report = None
        try:
            future = (self.lookup_client.call_async(request)
                      if self.lookup_client.service_is_ready() else None)
        except Exception:
            future = None
        if future is None:
            self._record_lookup(lookup, NavigationObservation.DISCONNECTED)
        else:
            self._lookup = (lookup, request, future, monotonic() + self.timeout)
        return lookup

    def approve_retry(self, lookup, *, explicitly_approved=False):
        """Approve one new attempt after a matching absence report, without sending."""
        return self.boundary.approve_retry(lookup, explicitly_approved=explicitly_approved)

    def _poll(self):
        self._poll_dispatch()
        self._poll_lookup()

    def _poll_dispatch(self):
        active = self._dispatch
        if active is None:
            return
        awaiting_acceptance = active.receipt is not None or not active.accepted
        if ((awaiting_acceptance and monotonic() >= active.deadline)
                or (not self.dispatch_client.service_is_ready()
                    and not self.lookup_client.service_is_ready())):
            self._hold_dispatch(active)
            return
        receipt = active.receipt
        if receipt is not None and receipt.done():
            active.receipt = None
            response = _response(receipt)
            if (response is None or response.attempt_id != active.ticket.goal.attempt_id
                    or response.nav_goal_id != active.ticket.nav_goal_id
                    or response.result not in (DispatchGoal.Response.RESULT_ACCEPTED,
                                               DispatchGoal.Response.RESULT_DUPLICATE)
                    or response.goal_state == DispatchGoal.Response.STATE_UNCERTAIN):
                self._hold_dispatch(active)

    def _finish_dispatch(self, active):
        if active.receipt is not None:
            self.dispatch_client.remove_pending_request(active.receipt)
        self._dispatch = None

    def _hold_dispatch(self, active):
        self.boundary.report_dispatch_uncertain(active.ticket)
        self._finish_dispatch(active)

    def _report_event(self, request, response):
        self._poll_dispatch()
        active = self._dispatch
        goal = self.manager.current_goal()
        if (active is None or goal != active.ticket.goal
                or self.manager.state.recovery_required
                or self.manager.state.nav_goal_id != active.ticket.nav_goal_id
                or not _matches(request, _execution_context(goal, active.ticket.nav_goal_id))
                or request.event_seq <= active.event_seq):
            response.reason = 'stale or duplicate event'
            return response

        event = request.event
        if event == request.EVENT_ACCEPTED:
            active.accepted = True
        elif event == request.EVENT_SUCCEEDED:
            if not self.boundary.observe(active.ticket, NavigationObservation.ARRIVED):
                response.reason = 'arrival did not apply'
                return response
            self._finish_dispatch(active)
        elif event in (request.EVENT_REJECTED, request.EVENT_ABORTED,
                       request.EVENT_CANCELED, request.EVENT_UNCERTAIN):
            self._hold_dispatch(active)
        else:
            response.reason = 'unknown event'
            return response
        active.event_seq = request.event_seq
        response.handled = True
        return response

    def _poll_lookup(self):
        if self._lookup is None:
            return
        lookup, request, future, deadline = self._lookup
        if monotonic() >= deadline:
            self.lookup_client.remove_pending_request(future)
            self._record_lookup(lookup, NavigationObservation.TIMEOUT)
        elif future.done():
            response = _response(future)
            context = _execution_context(lookup.check, lookup.nav_goal_id)
            if (response is not None and response.check_id == request.check_id
                    and _matches(response, context)):
                observations = {getattr(LookupGoal.Response, f'OBS_{value.name}'): value
                                for value in NavigationObservation}
                observation = observations.get(response.observation, NavigationObservation.UNKNOWN)
                self._record_lookup(lookup, observation, response.no_pending_dispatch)
            else:
                self._record_lookup(lookup, NavigationObservation.UNKNOWN)

    def _record_lookup(self, lookup, observation, no_pending_dispatch=False):
        self._lookup = None
        if observation is NavigationObservation.NO_ACTIVE_GOAL and not no_pending_dispatch:
            observation = NavigationObservation.UNKNOWN
        report = RecoveryReport(lookup, observation, no_pending_dispatch)
        self.boundary.handle_recovery_report(report)
        self.last_report = report

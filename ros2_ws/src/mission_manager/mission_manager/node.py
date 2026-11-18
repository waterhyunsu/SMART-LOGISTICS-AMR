"""Single-executor mission manager. No navigation commands or web endpoints."""

import argparse
from math import isfinite
import sys
from time import monotonic
from uuid import uuid4

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.utilities import remove_ros_args

from logistics_msgs.msg import TaskStep
from logistics_msgs.srv import OptimizeRoute
from mission_manager.contract import MissionState


class MissionManager(Node):
    def __init__(self, **kwargs):
        super().__init__('mission_manager', **kwargs)
        mission_id = self.declare_parameter('mission_id', str(uuid4())).value
        self.state = MissionState(mission_id, [])
        self.timeout = self.declare_parameter('optimizer_timeout_sec', 5.0).value
        if not isfinite(self.timeout) or self.timeout <= 0:
            raise ValueError('optimizer_timeout_sec must be finite and positive')
        self.optimizer = self.create_client(OptimizeRoute, 'optimize_route')
        self.request = None
        self.future = None
        self.candidate = None
        self.plan_status = 'idle'
        self.timer = self.create_timer(
            min(0.02, self.timeout), self._poll, clock=Clock(clock_type=ClockType.STEADY_TIME),
            autostart=False)

    def register_jobs(self, steps, start_waypoint_id, cost_revision):
        """Order intake boundary; while busy, plan from the current destination."""
        if self.state.current:
            start_waypoint_id = self.state.steps[self.state.current].waypoint_id
        if not start_waypoint_id.strip() or not cost_revision.strip():
            raise ValueError('missing start waypoint or cost revision')
        self.state.add_steps(steps)
        return self.request_plan(start_waypoint_id, cost_revision)

    def request_plan(self, start_waypoint_id, cost_revision):
        request = self.state.request(start_waypoint_id, cost_revision)
        self._stop_call()
        self.request = request
        self.candidate = None
        self.deadline = monotonic() + self.timeout
        self.plan_status = 'pending'
        self.timer.reset()
        return request.request_id

    def _stop_call(self):
        self.timer.cancel()
        if self.future is not None:
            self.optimizer.remove_pending_request(self.future)
        self.future = None
        self.request = None

    def _reject(self, status, reason):
        self._stop_call()
        self.state.pending_request_id = None
        self.plan_status = status
        self.get_logger().warning(f'Optimization {status}: {reason}')

    def _poll(self):
        if self.request is None:
            return
        if monotonic() >= self.deadline:
            self._reject('timeout', 'no response before deadline')
            return
        try:
            if self.future is None:
                if self.optimizer.service_is_ready():
                    self.future = self.optimizer.call_async(self.request)
                return
            if not self.future.done():
                return
            result = self.future.result()
        except Exception as error:
            self._reject('failed', str(error))
            return
        if not self.state.valid_candidate(result):
            self._reject('rejected', result.failure_reason or 'candidate does not match mission')
            return
        self._stop_call()
        self.candidate = result
        if self.state.current:
            self.plan_status = 'deferred'
        else:
            self._apply_candidate()

    def _apply_candidate(self):
        accepted = self.state.apply(self.candidate)
        self.candidate = None
        self.plan_status = 'applied' if accepted else 'rejected'
        self.get_logger().info(f'Plan {self.plan_status}, version {self.state.active_version}')

    def arrive(self, step_id):
        return self.state.arrive(step_id)

    def handoff(self, event_id, mission_id, step_id):
        outcome = self.state.handoff(event_id, mission_id, step_id)
        if outcome == 'accepted' and self.state.current is None and self.candidate is not None:
            self._apply_candidate()
        return outcome


def main(args=None):
    argv = sys.argv if args is None else ['mission_manager', *args]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--job', nargs=3, action='append', metavar=('ID', 'PICKUP', 'DROPOFF'))
    parser.add_argument('--start', help='current waypoint for initial planning')
    parser.add_argument('--cost-revision', help='approved optimizer cost snapshot')
    options = parser.parse_args(remove_ros_args(argv)[1:])
    if options.job and not (options.start and options.cost_revision):
        parser.error('--job requires --start and --cost-revision')
    rclpy.init(args=argv)
    node = None
    try:
        node = MissionManager()
        if options.job:
            steps = []
            for job_id, pickup, dropoff in options.job:
                load = TaskStep(job_id=job_id, step_id=f'{job_id}/pickup',
                                kind=TaskStep.PICKUP, waypoint_id=pickup)
                unload = TaskStep(job_id=job_id, step_id=f'{job_id}/dropoff',
                                  kind=TaskStep.DROPOFF, waypoint_id=dropoff,
                                  prerequisite_ids=[load.step_id])
                steps.extend((load, unload))
            node.register_jobs(steps, options.start, options.cost_revision)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()

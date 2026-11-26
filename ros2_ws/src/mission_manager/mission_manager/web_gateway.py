"""Loopback HTTP entry point for an operator-driven, single-order airport SIM."""
import argparse
import asyncio
from dataclasses import asdict
import os
import sqlite3
import sys

from aiohttp import web
import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.utilities import remove_ros_args

from logistics_msgs.msg import TaskStep
from mission_manager.navigation_services import NavigationServices
from mission_manager.node import MissionManager
from mission_manager.persistence import RecoveryPending
from route_optimizer.cost_snapshot import load_snapshot


class InputConflict(ValueError):
    """The client supplied a stale identity or changed a registered job."""


class WebMission:
    def __init__(self, manager, navigation, revision, waypoints):
        self.manager = manager
        self.navigation = navigation
        self.revision = revision
        self.waypoints = waypoints
        self.authorized = False
        if 'HOME' not in waypoints or manager.state.cost_revision not in (None, revision):
            raise ValueError('cost snapshot must contain HOME and match the stored mission')

    def check_mission(self, mission_id):
        if mission_id != self.manager.state.mission_id:
            raise InputConflict('mission_id differs from the active mission')

    def register(self, mission_id, job_id, pickup, dropoff):
        self.check_mission(mission_id)
        if pickup == dropoff or not {pickup, dropoff} <= self.waypoints:
            raise ValueError('choose two distinct waypoints from the configured cost snapshot')
        load = TaskStep(job_id=job_id, step_id=f'{job_id}-PICKUP',
                        kind=TaskStep.PICKUP, waypoint_id=pickup)
        unload = TaskStep(job_id=job_id, step_id=f'{job_id}-DROPOFF',
                          kind=TaskStep.DROPOFF, waypoint_id=dropoff,
                          prerequisite_ids=[load.step_id])
        stored = self.manager.state.steps
        if stored:
            if stored.get(load.step_id) != load or stored.get(unload.step_id) != unload:
                raise InputConflict('single-order mission already contains a different order')
        else:
            self.manager.register_jobs([load, unload], 'HOME', self.revision)
        return {'outcome': 'accepted', 'mission_id': mission_id, 'job_id': job_id}

    def ready(self):
        return (self.manager.optimizer.service_is_ready()
                and self.navigation.dispatch_client.service_is_ready()
                and self.navigation.lookup_client.service_is_ready())

    def run(self, mission_id):
        self.check_mission(mission_id)
        state = self.manager.state
        reason = ''
        if state.recovery_required:
            raise RecoveryPending('external step reconciliation required')
        if self.manager.mission_status == 'completed' or not state.steps:
            reason = 'mission is complete or empty'
        elif self.authorized:
            return {'outcome': 'accepted', 'reason': 'already authorized'}
        elif not self.ready():
            reason = 'C or D service is unavailable'
        elif not state.active_version:
            # A process can stop after intake but before C returns the first plan.
            if self.manager.request is None:
                self.manager.request_plan('HOME', self.revision)
            reason = 'waiting for an approved plan'
        else:
            self.authorized = True
        return {'outcome': 'rejected' if reason else 'accepted', 'reason': reason}

    def advance(self):
        if not self.authorized:
            return
        state = self.manager.state
        if state.recovery_required or self.manager.mission_status == 'completed':
            self.authorized = False
            return
        if state.waiting or state.nav_goal_id or not self.ready():
            return
        # If D disappeared between discovery and dispatch, the in-process claim
        # can still be unbound. A bound UUID always takes the recovery path.
        goal = self.manager.current_goal() or self.manager.claim_next_goal()
        if goal is not None:
            self.navigation.dispatch(goal)

    def handoff(self, mission_id, step_id, event_id):
        self.check_mission(mission_id)
        return {'outcome': self.manager.handoff(event_id, mission_id, step_id)}

    def snapshot(self):
        manager = self.manager
        state = manager.state
        goal = manager.current_goal()
        steps = [{
            'job_id': step.job_id, 'step_id': step.step_id, 'waypoint_id': step.waypoint_id,
            'kind': 'PICKUP' if step.kind == TaskStep.PICKUP else 'DROPOFF',
            'prerequisite_ids': list(step.prerequisite_ids), 'status': manager.step_status(step.step_id),
        } for step in state.steps.values()]
        return {
            'source': 'SIM', 'mission_id': state.mission_id, 'mission_status': manager.mission_status,
            'plan_status': manager.plan_status, 'cost_revision': self.revision,
            'active_version': state.active_version, 'recovery_required': state.recovery_required,
            'run_authorized': self.authorized and manager.mission_status not in ('completed', 'recovery_required'),
            'ready': self.ready(), 'current_goal': asdict(goal) if goal else None,
            'waiting_confirmation': state.waiting and not state.recovery_required,
            'steps': steps, 'order': state.order,
            'jobs': [{'job_id': job, 'status': manager.job_status(job)}
                     for job in dict.fromkeys(s['job_id'] for s in steps)],
        }


def make_app(bridge):
    app = web.Application(client_max_size=16 * 1024)

    async def snapshot(request):
        return web.json_response(bridge.snapshot())

    app.router.add_get('/v1/state', snapshot)

    def post(path, action, fields):
        async def handle(request):
            if request.content_type != 'application/json':
                return web.json_response({'error': 'expected application/json'}, status=415)
            try:
                body = await request.json()
                if (not isinstance(body, dict) or set(body) != set(fields)
                        or not all(isinstance(v, str) and v.strip() for v in body.values())):
                    raise ValueError('required string fields: ' + ', '.join(fields))
                return web.json_response(action(**body))
            except RecoveryPending as exc:
                return web.json_response({'error': 'recovery_pending', 'reason': str(exc)}, status=409)
            except InputConflict as exc:
                return web.json_response({'error': 'conflict', 'reason': str(exc)}, status=409)
            except ValueError as exc:
                return web.json_response({'error': 'invalid_input', 'reason': str(exc)}, status=422)
            except sqlite3.Error:
                return web.json_response({'error': 'unconfirmed'}, status=503)
        app.router.add_post(path, handle)

    post('/v1/jobs', bridge.register, ('mission_id', 'job_id', 'pickup', 'dropoff'))
    post('/v1/run', bridge.run, ('mission_id',))
    post('/v1/handoffs', bridge.handoff, ('mission_id', 'step_id', 'event_id'))
    return app


async def serve(manager, snapshot_path, port):
    revision, waypoints, _ = load_snapshot(snapshot_path)
    bridge = WebMission(manager, NavigationServices(manager), revision, waypoints)
    executor = SingleThreadedExecutor(context=manager.context)
    executor.add_node(manager)
    manager.create_timer(0.1, bridge.advance)
    runner = web.AppRunner(make_app(bridge))
    try:
        await runner.setup()
        await web.TCPSite(runner, '127.0.0.1', port).start()
        manager.get_logger().info(f'A web gateway listening on 127.0.0.1:{port}')
        # ROS callbacks and HTTP mutations both execute on the SQLite owner thread.
        while manager.context.ok():
            executor.spin_once(timeout_sec=0.0)
            await asyncio.sleep(0.005)
    finally:
        await runner.cleanup()
        executor.shutdown()


def main(args=None):
    argv = sys.argv if args is None else ['mission_web_gateway', *args]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cost-snapshot', required=True)
    parser.add_argument('--port', type=int, default=8765)
    options = parser.parse_args(remove_ros_args(argv)[1:])
    if (os.getenv('ROS_AUTOMATIC_DISCOVERY_RANGE') != 'LOCALHOST'
            or not os.getenv('ROS_DOMAIN_ID', '').isdigit()):
        parser.error('select an isolated SIM ROS_DOMAIN_ID and LOCALHOST discovery')
    rclpy.init(args=argv)
    manager = None
    try:
        manager = MissionManager()
        if not manager.get_parameter('use_sim_time').value:
            parser.error('the airport gateway requires use_sim_time:=true')
        asyncio.run(serve(manager, options.cost_snapshot, options.port))
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if manager is not None:
            manager.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

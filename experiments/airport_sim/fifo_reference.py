"""Experiment-only OptimizeRoute baseline. Job numbers encode reception order."""

import argparse

import rclpy
from logistics_msgs.msg import TaskStep
from logistics_msgs.srv import OptimizeRoute
from rclpy.executors import ExternalShutdownException
from route_optimizer.cost_snapshot import load_snapshot
from route_optimizer.planner import RouteUnavailable, build_order, total_distance


def reception_key(step):
    return int(step.job_id.removeprefix("job-")), step.kind != TaskStep.PICKUP


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("snapshot")
    revision, _, costs = load_snapshot(parser.parse_args().snapshot)

    def fifo(request, response):
        response.request_id = request.request_id
        response.mission_id = request.mission_id
        response.base_version = request.base_version
        response.cost_revision = revision
        response.method = "fifo"
        if request.cost_revision != revision:
            response.failure_reason = "Cost revision mismatch"
            return response
        pending = sorted(request.pending_steps, key=reception_key)
        waypoints = {s.step_id: s.waypoint_id for s in pending}
        prerequisites = {s.step_id: s.prerequisite_ids for s in pending}
        satisfied = set(request.completed_step_ids) | {request.current_step_id}
        try:
            order = build_order(waypoints, prerequisites, costs, request.start_waypoint_id,
                                greedy=False, satisfied=satisfied)
            if tuple(waypoints) != order:
                raise RouteUnavailable("FIFO order is not reachable or violates prerequisites")
            response.estimated_distance_m = total_distance(order, waypoints, costs,
                                                           request.start_waypoint_id)
        except RouteUnavailable as exc:
            response.failure_reason = str(exc)
            return response
        response.ordered_step_ids = list(order)
        response.success = True
        return response

    rclpy.init(args=[])
    node = rclpy.create_node("airport_fifo_reference")
    node.create_service(OptimizeRoute, "optimize_route", fifo)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

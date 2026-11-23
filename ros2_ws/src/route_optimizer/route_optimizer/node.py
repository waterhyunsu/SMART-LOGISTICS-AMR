"""OptimizeRoute server backed by a saved cost snapshot or the MOCK table."""

from functools import partial

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from logistics_msgs.srv import OptimizeRoute

from route_optimizer.cost_snapshot import load_snapshot
from route_optimizer.mock_costs import COST, COST_REVISION, WAYPOINTS
from route_optimizer.planner import RouteUnavailable, find_best_order


def optimize_route(
    request,
    response,
    *,
    costs=COST,
    cost_revision=COST_REVISION,
    waypoint_ids=WAYPOINTS,
):
    response.request_id = request.request_id
    response.mission_id = request.mission_id
    response.base_version = request.base_version
    response.cost_revision = cost_revision
    response.method = "exhaustive"

    if request.cost_revision != cost_revision:
        response.failure_reason = f"Cost revision mismatch: expected {cost_revision}"
        return response

    satisfied = set(request.completed_step_ids)
    if request.current_step_id:
        satisfied.add(request.current_step_id)

    waypoints = {step.step_id: step.waypoint_id for step in request.pending_steps}
    if (
        len(waypoints) != len(request.pending_steps)
        or "" in waypoints
        or satisfied.intersection(waypoints)
    ):
        response.failure_reason = "Invalid, duplicate, or already satisfied pending step"
        return response

    if request.start_waypoint_id not in waypoint_ids:
        response.failure_reason = "No reachable route: unknown start waypoint"
        return response

    # Factorial search is intended for small missions.
    if len(waypoints) > 8:
        response.failure_reason = "Exhaustive search supports at most eight pending steps"
        return response

    prerequisites = {
        step.step_id: step.prerequisite_ids for step in request.pending_steps
    }
    try:
        order, distance = find_best_order(
            waypoints,
            prerequisites,
            costs,
            request.start_waypoint_id,
            satisfied=satisfied,
        )
    except RouteUnavailable as error:
        response.failure_reason = str(error)
        return response

    response.ordered_step_ids = list(order)
    response.estimated_distance_m = float(distance)
    response.success = True
    return response


class RouteOptimizer(Node):
    def __init__(self):
        super().__init__("route_optimizer")
        path = self.declare_parameter("cost_snapshot_path", "").value
        revision, waypoint_ids, costs = (
            load_snapshot(path) if path else (COST_REVISION, WAYPOINTS, COST)
        )

        callback = partial(
            optimize_route,
            costs=costs,
            cost_revision=revision,
            waypoint_ids=waypoint_ids,
        )
        self.service = self.create_service(OptimizeRoute, "optimize_route", callback)
        self.get_logger().info(f"Using cost revision: {revision}")


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RouteOptimizer()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
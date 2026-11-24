"""Initialize airport AMCL, record Nav2 costs, or exercise A–C–D in airport SIM."""

import argparse
import csv
import json
import math
import os
from hashlib import sha256
from itertools import permutations
from pathlib import Path
from time import monotonic

import rclpy
import yaml
from compare_orders import EXTRA_ORDER, INITIAL_ORDERS
from action_msgs.msg import GoalStatusArray
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from logistics_msgs.msg import TaskStep
from mission_manager.navigation_services import NavigationServices
from mission_manager.node import MissionManager
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import GetParameters
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_action_status_default, qos_profile_sensor_data
from rclpy.time import Time
from route_optimizer.cost_snapshot import build_snapshot, load_snapshot
from route_optimizer.planner import segment_cost
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

ASSETS = Path(__file__).resolve().parent / "assets"
STARTUP_TIMEOUT = 45
SCENARIOS = {
    "single": ([("S4", "G8")], None),
    "multi": (INITIAL_ORDERS, EXTRA_ORDER),
}


def wait(node, predicate, timeout, detail):
    deadline = monotonic() + timeout
    while not predicate():
        if monotonic() >= deadline:
            raise TimeoutError(detail)
        rclpy.spin_once(node, timeout_sec=0.05)


def result(node, future, timeout=10):
    wait(node, future.done, timeout, "ROS request timed out")
    return future.result()


def pose(x, y, yaw):
    msg = PoseStamped()
    msg.header.frame_id = "map"
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.orientation.z = math.sin(yaw / 2)
    msg.pose.orientation.w = math.cos(yaw / 2)
    return msg


def active(node, names):
    for name in names:
        client = node.create_client(GetState, f"/{name}/get_state")
        try:
            wait(node, client.service_is_ready, STARTUP_TIMEOUT, f"{name} service absent")
            wait(node, lambda: result(node, client.call_async(GetState.Request()))
                 .current_state.id == 3, STARTUP_TIMEOUT, f"{name} not active")
        finally:
            node.destroy_client(client)


def initialize(node, home):
    buffer = Buffer(node=node)
    listener = TransformListener(buffer, node)
    observed = set()
    subscriptions = [
        node.create_subscription(msg_type, topic, lambda _, key=topic: observed.add(key), qos)
        for msg_type, topic, qos in (
            (LaserScan, "/scan", qos_profile_sensor_data),
            (Odometry, "/odom", qos_profile_sensor_data),
            (PoseWithCovarianceStamped, "/amcl_pose", 10),
        )
    ]
    initialpose = node.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
    try:
        active(node, ("amcl",))
        wait(node, lambda: {"/scan", "/odom"} <= observed
             and node.get_clock().now().nanoseconds > 0
             and initialpose.get_subscription_count() > 0,
             STARTUP_TIMEOUT, "clock, scan, odometry or AMCL not ready")
        first_clock = node.get_clock().now().nanoseconds
        wait(node, lambda: node.get_clock().now().nanoseconds > first_clock,
             STARTUP_TIMEOUT, "simulation clock stopped")
        home_pose = pose(**home)
        msg = PoseWithCovarianceStamped(header=home_pose.header)
        msg.pose.pose = home_pose.pose
        msg.pose.covariance[0] = msg.pose.covariance[7] = 0.01
        msg.pose.covariance[35] = 0.01
        observed.discard("/amcl_pose")
        initialpose.publish(msg)
        wait(node, lambda: "/amcl_pose" in observed
             and buffer.can_transform("map", "base_link", Time()),
             STARTUP_TIMEOUT, "AMCL pose or map to base_link transform not ready")
        active(node, ("planner_server", "controller_server", "bt_navigator"))
        translation = buffer.lookup_transform("map", "base_link", Time()).transform.translation
        error = math.hypot(translation.x - home["x"], translation.y - home["y"])
        if error > 0.25:
            raise RuntimeError(f"AMCL HOME position error is {error:.3f} m")
        print(f"HOME AMCL error {error:.3f} m; clock, scan, odom, TF and Nav2 ready", flush=True)
        return error
    finally:
        for subscription in subscriptions:
            node.destroy_subscription(subscription)
        node.destroy_publisher(initialpose)
        listener.unregister()


def digest(path):
    return sha256(path.read_bytes()).hexdigest()


def record_costs(node, waypoints, output):
    active(node, ("planner_server",))
    planner = node.create_client(GetParameters, "/planner_server/get_parameters")
    wait(node, planner.service_is_ready, STARTUP_TIMEOUT, "planner parameters unavailable")
    values = result(node, planner.call_async(GetParameters.Request(names=[
        "GridBased.plugin", "GridBased.use_astar", "GridBased.tolerance",
        "GridBased.allow_unknown",
    ]))).values
    plugin, astar, tolerance, unknown = values
    if plugin.string_value != "nav2_navfn_planner::NavfnPlanner" or astar.bool_value:
        raise RuntimeError("Expected GridBased NavFn Dijkstra")

    client = ActionClient(node, ComputePathToPose, "/compute_path_to_pose")
    wait(node, client.server_is_ready, STARTUP_TIMEOUT, "ComputePathToPose unavailable")
    recordings = []
    for start, end in permutations(waypoints, 2):
        goal = ComputePathToPose.Goal(
            planner_id="GridBased", use_start=True,
            start=pose(**waypoints[start]), goal=pose(**waypoints[end]),
        )
        handle = result(node, client.send_goal_async(goal))
        if handle.accepted:
            reply = result(node, handle.get_result_async())
            code = reply.result.error_code
            reached = reply.status == 4
            xy = [[p.pose.position.x, p.pose.position.y] for p in reply.result.path.poses]
            reason = reply.result.error_msg
        else:
            reached, code, xy, reason = False, -1, [], "ComputePathToPose rejected"
        recordings.append(dict(start=start, goal=end, succeeded=reached,
                               error_code=code, poses=xy, failure_reason=reason))
    params = Path(get_package_share_directory("nav2_bringup")) / "params" / "nav2_params.yaml"
    document = {
        "map": {"yaml_sha256": digest(ASSETS / "airport.yaml"),
                "pgm_sha256": digest(ASSETS / "airport.pgm"),
                "world_xacro_sha256": digest(ASSETS / "airport.sdf.xacro")},
        "planner": {"id": "GridBased", "plugin": plugin.string_value,
                    "use_astar": astar.bool_value, "tolerance": tolerance.double_value,
                    "allow_unknown": unknown.bool_value},
        "global_costmap": {"params_sha256": digest(params),
                           **yaml.safe_load(params.read_text())["global_costmap"]
                           ["global_costmap"]["ros__parameters"]},
        "waypoints": waypoints,
    }
    snapshot = build_snapshot(document, recordings)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.with_suffix(".recordings.json").write_text(json.dumps(
        {"metadata": document, "results": recordings}, separators=(",", ":")) + "\n")
    output.write_text(json.dumps(snapshot, indent=2) + "\n")
    failures = [segment for segment in snapshot["segments"] if not segment["reachable"]]
    print(f"Reachable: {len(snapshot['segments']) - len(failures)}/{len(snapshot['segments'])} "
          f"{snapshot['cost_revision']}", flush=True)
    if failures:
        raise RuntimeError(f"Unreachable directed pairs: {failures}")


def steps(order_number, pickup, drop):
    job = f"job-{order_number}"
    return [
        TaskStep(job_id=job, step_id=f"{job}-pickup", kind=TaskStep.PICKUP, waypoint_id=pickup),
        TaskStep(job_id=job, step_id=f"{job}-drop", kind=TaskStep.DROPOFF, waypoint_id=drop,
                 prerequisite_ids=[f"{job}-pickup"]),
    ]


def check_payload(order, step_map, onboard):
    onboard = set(onboard)
    for key in order:
        step = step_map[key]
        if step.kind == TaskStep.PICKUP:
            if step.job_id in onboard or len(onboard) == 4:
                raise ValueError(f"Invalid virtual payload at {key}")
            onboard.add(step.job_id)
        else:
            onboard.remove(step.job_id)
    if onboard:
        raise ValueError("Plan leaves bags onboard")


def run_mission(manager, scenario, snapshot, goal_timeout, waypoints, trajectory):
    revision, _, costs = load_snapshot(snapshot)
    bridge = NavigationServices(manager)
    statuses = {}
    odom = None
    traveled = 0.0
    last_stamp = None
    max_gap = 0.0
    sample_count = 0
    loaded = set()

    def track(message):
        nonlocal odom, traveled, last_stamp, max_gap, sample_count
        stamp = Time.from_msg(message.header.stamp).nanoseconds
        position = message.pose.pose.position
        point = (position.x, position.y)
        if odom is not None:
            gap = (stamp - last_stamp) / 1e9
            if not 0 < gap <= 0.25:
                raise ValueError(f"World pose stream has a {gap:.3f}s gap")
            max_gap = max(max_gap, gap)
            delta = math.dist(point, odom)
            traveled += delta
        odom = point
        last_stamp = stamp
        sample_count += 1
        trajectory.writerow((stamp, *point))

    manager.create_subscription(Odometry, "/airport/world_odom", track, 500)

    def observe(message):
        for item in message.status_list:
            statuses[bytes(item.goal_info.goal_id.uuid).hex()] = item.status

    manager.create_subscription(GoalStatusArray, "/navigate_to_pose/_action/status",
                                observe, qos_profile_action_status_default)
    wait(manager, lambda: bridge.dispatch_client.service_is_ready()
         and bridge.lookup_client.service_is_ready(), STARTUP_TIMEOUT, "D services not ready")
    wait(manager, lambda: odom is not None and manager.get_clock().now().nanoseconds > 0,
         STARTUP_TIMEOUT, "SIM odometry or clock missing")
    home = waypoints["HOME"]
    if math.dist(odom, (home["x"], home["y"])) > 0.25:
        raise RuntimeError("Start a fresh airport SIM at HOME before running each mission")
    start_real = monotonic()
    start_sim = manager.get_clock().now().nanoseconds
    if statuses:
        raise RuntimeError("Found existing Nav2 goals. Start a fresh isolated airport SIM.")
    initial, extra = SCENARIOS[scenario]
    work = [step for number, order in enumerate(initial, 1) for step in steps(number, *order)]
    manager.register_jobs(work, "HOME", revision)
    state = manager.state
    wait(manager, lambda: state.active_version == 1, 20, "C initial plan not applied")
    check_payload(state.order, state.steps, loaded)
    initial_order = list(state.order)
    events = []
    max_loaded = 0
    injected = False
    injection = None
    departure = "HOME"
    # Start distance at the first dispatch. All subsequent motion up to the
    # acknowledgement belongs to the load on departure, including recovery maneuvers.
    leg_origin = traveled
    leg_start_stamp = last_stamp
    for _ in range(2 * (len(initial) + (extra is not None)) + 1):
        start_position = odom
        goal = manager.claim_next_goal()
        if goal is None or not bridge.dispatch(goal):
            raise RuntimeError(f"Goal not dispatched: {goal}")
        uuid = state.nav_goal_id
        print(f"Sent {goal.step_id} → {goal.waypoint_id} (v{goal.plan_version}, {uuid})", flush=True)
        wait(manager, lambda: uuid in statuses, 15, "A/D UUID missing from Nav2 statuses")
        is_home = goal.step_id is None
        injection_here = extra and not injected and goal.step_id and goal.step_id.endswith("-drop")
        if injection_here:
            wait(manager, lambda: math.dist(odom, start_position) >= 0.50
                 or statuses[uuid] >= 4, 40, "No displacement during injection leg")
            if statuses[uuid] != 2:
                raise RuntimeError("Extra order must arrive during active navigation")
            injection = {"step": goal.step_id, "waypoint": goal.waypoint_id, "uuid": uuid,
                         "sim_stamp_ns": last_stamp, "at_xy": odom,
                         "displacement_m": math.dist(odom, start_position)}
            work.extend(steps(len(initial) + 1, *extra))
            manager.register_jobs(work[-2:], goal.waypoint_id, revision)
            wait(manager, lambda: manager.plan_status == "deferred"
                 or state.active_version > goal.plan_version, 20, "C replacement plan did not arrive")
            if manager.current_goal() != goal or state.active_version != goal.plan_version:
                raise RuntimeError("Candidate replaced the moving goal")
            check_payload([goal.step_id, *manager.candidate.ordered_step_ids], state.steps, loaded)
            injection["candidate_after_handoff"] = list(manager.candidate.ordered_step_ids)
            injected = True

        def arrived():
            if state.recovery_required or statuses.get(uuid) in (5, 6):
                raise RuntimeError(f"Navigation failed or entered recovery: {goal}")
            return statuses.get(uuid) == 4 and (
                manager.mission_status == "completed" if is_home
                else manager.step_status(goal.step_id) == "waiting_confirmation"
            )

        wait(manager, arrived, goal_timeout, f"Goal or handoff wait timed out: {goal}")
        settled = manager.get_clock().now().nanoseconds + 250_000_000
        wait(manager, lambda: last_stamp >= settled, 5, "Missing world pose at arrival")
        target = waypoints[goal.waypoint_id]
        error = math.dist(odom, (target["x"], target["y"]))
        if error > 0.40:
            raise RuntimeError(f"Gazebo world pose is {error:.3f} m away from {goal.waypoint_id}")
        event = {"uuid": uuid, "step": goal.step_id, "waypoint": goal.waypoint_id,
                 "world_error_m": error, "world_xy": odom, "from": departure,
                 "bags_at_departure": sorted(loaded), "start_ns": leg_start_stamp,
                 "end_ns": last_stamp, "planned_m": segment_cost(costs, departure, goal.waypoint_id),
                 "sim_driven_m": traveled - leg_origin,
                 "plan_version_at_dispatch": goal.plan_version, "nav2": "SUCCEEDED"}
        if not is_home:
            if manager.claim_next_goal() is not None:
                raise RuntimeError("A claimed a goal before handoff")
            answer = manager.handoff(f"event-{goal.step_id}", state.mission_id, goal.step_id)
            if answer != "accepted":
                raise RuntimeError(f"Handoff rejected: {answer}")
            if injection_here and state.active_version != goal.plan_version + 1:
                raise RuntimeError("A did not apply the new plan after handoff")
            step = next(step for step in work if step.step_id == goal.step_id)
            if step.kind == TaskStep.PICKUP:
                loaded.add(step.job_id)
                max_loaded = max(max_loaded, len(loaded))
            else:
                loaded.remove(step.job_id)
            event["handoff"] = answer
        events.append(event)
        leg_start_stamp, leg_origin, departure = last_stamp, traveled, goal.waypoint_id
        print(f"Confirmed {goal.waypoint_id}: {event}", flush=True)
    if loaded or (extra and not injected) or manager.mission_status != "completed":
        raise RuntimeError("Mission incomplete")
    if set(statuses) != {event["uuid"] for event in events}:
        raise RuntimeError("Unexpected additional Nav2 goal UUID")
    if len({event["uuid"] for event in events}) != len(events):
        raise RuntimeError("Repeated navigation goal UUID")
    return {"mission_status": manager.mission_status, "cost_revision": revision,
            "initial_order": initial_order, "injection": injection,
            "world_samples": sample_count, "max_sample_gap_s": max_gap,
            "dynamic_order_injected": injected, "final_plan_version": state.active_version,
            "max_simultaneous_virtual_bags": max_loaded, "goals": events,
            "sim_time_s": (manager.get_clock().now().nanoseconds - start_sim) / 1e9,
            "wall_time_s": monotonic() - start_real,
            "planned_m": sum(e["planned_m"] for e in events),
            "planned_empty_m": sum(e["planned_m"] for e in events if not e["bags_at_departure"]),
            "sim_driven_m": sum(e["sim_driven_m"] for e in events),
            "sim_empty_m": sum(e["sim_driven_m"] for e in events if not e["bags_at_departure"])}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_subparsers(dest="mode", required=True)
    modes.add_parser("init")
    costs = modes.add_parser("costs")
    costs.add_argument("output", type=Path)
    mission = modes.add_parser("mission")
    mission.add_argument("scenario", choices=SCENARIOS)
    mission.add_argument("snapshot", type=Path)
    mission.add_argument("result", type=Path)
    mission.add_argument("--goal-timeout", type=float, default=240)
    args = parser.parse_args()
    if not os.environ.get("ROS_DOMAIN_ID", "").isdigit() or (
            os.environ.get("ROS_AUTOMATIC_DISCOVERY_RANGE") != "LOCALHOST"):
        parser.error("Use a dedicated SIM ROS_DOMAIN_ID and LOCALHOST discovery")
    waypoints = yaml.safe_load((ASSETS / "waypoints.yaml").read_text())["waypoints"]
    rclpy.init(args=[])
    try:
        parameter = Parameter("use_sim_time", value=True)
        if args.mode == "mission":
            node = MissionManager(parameter_overrides=[parameter])
        else:
            node = rclpy.create_node("airport_verifier", parameter_overrides=[parameter])

        try:
            if args.mode == "init":
                initialize(node, waypoints["HOME"])
            elif args.mode == "costs":
                record_costs(node, waypoints, args.output)
            else:
                args.result.parent.mkdir(parents=True, exist_ok=True)
                with args.result.with_suffix(".trajectory.csv").open("w", newline="") as stream:
                    writer = csv.writer(stream)
                    writer.writerow(("stamp_ns", "x_m", "y_m"))
                    summary = run_mission(node, args.scenario, args.snapshot, args.goal_timeout,
                                          waypoints, writer)
                args.result.write_text(json.dumps(summary, indent=2) + "\n")
        finally:
            node.destroy_node()
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

"""Reconcile both five-order SIM runs with the planner snapshot and Gazebo traces."""

import argparse
import csv
import json
import math
import re
from collections import Counter
from pathlib import Path

from compare_orders import EXTRA_ORDER, INITIAL_ORDERS, comparison
from route_optimizer.cost_snapshot import load_snapshot
from route_optimizer.planner import segment_cost


def require(condition, message):
    if not condition:
        raise ValueError(message)


def full_step(short):
    kind = "pickup" if short[0] == "P" else "drop"
    return f"job-{short[1:]}-{kind}"


def audit(summary, trace, snapshot, dispatch_log, expected_steps):
    revision, _, costs = load_snapshot(snapshot)
    run = json.loads(summary.read_text())
    goals = run["goals"]
    require(run["mission_status"] == "completed" and revision == run["cost_revision"],
            "Mission or cost revision mismatch")
    require(run["dynamic_order_injected"] and run["final_plan_version"] == 2, "Missing replan")
    require(len(goals) == 11 and [g["step"] for g in goals] == [*expected_steps, None],
            "Executed steps differ from expected five-order route")
    ids = [g["uuid"] for g in goals]
    log = dispatch_log.read_text()
    require(len(set(ids)) == 11 and Counter(ids) == Counter(re.findall(r"\[SEND\] ([0-9a-f]{32})", log)),
            "Sent UUIDs differ from mission")
    successes = re.findall(r"\[RESULT\] ([0-9a-f]{32}) → SUCCEEDED", log)
    require(Counter(ids) == Counter(successes), "Missing D success")
    order = (*INITIAL_ORDERS, EXTRA_ORDER)
    positions = {}
    with trace.open() as stream:
        for row in csv.DictReader(stream):
            stamp = int(row["stamp_ns"])
            if positions:
                require(0 < stamp - previous_stamp <= 250_000_000, "World trace gap")
            previous_stamp = stamp
            positions[stamp] = (float(row["x_m"]), float(row["y_m"]))
    require(len(positions) == run["world_samples"], "World sample count mismatch")
    stamps = list(positions)
    by_stamp = {stamp: index for index, stamp in enumerate(stamps)}
    previous, onboard = "HOME", set()
    peak = 0
    values = {k: 0.0 for k in ("planned_m", "planned_empty_m", "sim_driven_m", "sim_empty_m")}
    first_delivery = next(i for i, g in enumerate(goals) if g["step"].endswith("-drop"))
    injection = run["injection"]
    g = goals[first_delivery]
    require(g["start_ns"] < injection["sim_stamp_ns"] < g["end_ns"]
            and g["uuid"] == injection["uuid"] and injection["displacement_m"] >= 0.5,
            "Order was not injected during first delivery")
    for i, g in enumerate(goals):
        step = g["step"]
        require(g["plan_version_at_dispatch"] == (1 if i <= first_delivery else 2), "Version changed early")
        require(g["nav2"] == "SUCCEEDED" and g["world_error_m"] <= 0.40, "Goal outside criteria")
        require(set(g["bags_at_departure"]) == onboard and previous == g["from"], "Leg start mismatch")
        if i:
            require(g["start_ns"] == goals[i-1]["end_ns"], "Non-contiguous trace")
        start, end = by_stamp[g["start_ns"]], by_stamp[g["end_ns"]]
        distance = sum(math.dist(positions[stamps[n-1]], positions[stamps[n]]) for n in range(start+1,end+1))
        planned = segment_cost(costs, previous, g["waypoint"])
        require(math.isclose(distance, g["sim_driven_m"], abs_tol=1e-8)
                and math.isclose(planned, g["planned_m"], abs_tol=1e-8), "Leg distance mismatch")
        values["planned_m"] += planned
        values["sim_driven_m"] += distance
        if not onboard:
            values["planned_empty_m"] += planned
            values["sim_empty_m"] += distance
        if step:
            _, number, kind = step.split("-")
            job = f"job-{number}"
            wp = order[int(number)-1][0 if kind == "pickup" else 1]
            require(g["waypoint"] == wp and g["handoff"] == "accepted", "Wrong bag or handoff")
            if kind == "pickup":
                require(job not in onboard and len(onboard) < 4, "Capacity or duplicate pickup")
                onboard.add(job)
            else:
                onboard.remove(job)
            peak = max(peak, len(onboard))
        else:
            require(g["waypoint"] == "HOME" and not onboard, "HOME with outstanding baggage")
        previous = g["waypoint"]
    for key, value in values.items():
        require(math.isclose(run[key], value, abs_tol=1e-8), f"Total mismatch in {key}")
    require(peak == run["max_simultaneous_virtual_bags"], "Peak load mismatch")
    return {**values, "peak_bags": peak, "goals": len(goals), "accepted_handoffs": len(goals)-1,
            "world_samples": len(stamps), "max_pose_gap_s": run["max_sample_gap_s"],
            "home_error_m": goals[-1]["world_error_m"], "injected_on": injection["waypoint"],
            "visited": ["HOME"] + [g["waypoint"] for g in goals]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    root = parser.parse_args().directory
    snapshot = root / "cost_snapshot.json"
    result = comparison(snapshot)
    optimized = result["dynamic_from_first_delivery_handoff"]
    sequences = {"optimized": [*optimized["completed"], *optimized["replan"]["order"]],
                 "fifo": [kind+str(i) for i in range(1,6) for kind in ("P","D")]}
    results = {"cost_revision": result["cost_revision"]}
    for name, short_steps in sequences.items():
        results[name] = audit(root/f"{name}.json", root/f"{name}.trajectory.csv", snapshot,
                              root/("opt.d.log" if name == "optimized" else "fifo.d.log"),
                              [full_step(s) for s in short_steps])
    print(json.dumps(results, indent=2))


if __name__ == "__main__":
    main()

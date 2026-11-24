"""Compare FIFO and the existing C solver on the same airport Nav2 costs."""

import argparse
import json
from pathlib import Path
from time import perf_counter

from route_optimizer.cost_snapshot import load_snapshot
from route_optimizer.planner import build_order, find_best_order, segment_cost

# The SIM harness checks four virtual bags before executing a plan.
INITIAL_ORDERS = (("S1", "G5"), ("S3", "G1"), ("S2", "G6"), ("S4", "G8"))
EXTRA_ORDER = ("S2", "G7")


def planned_distances(order, waypoints, costs, *, start, loaded=()):
    loaded = set(loaded)
    empty_m = total_m = 0
    max_loaded = len(loaded)
    for step in order:
        destination = waypoints[step]
        leg = segment_cost(costs, start, destination)
        total_m += leg
        if not loaded:
            empty_m += leg
        if step.startswith("P"):
            loaded.add(step[1:])
            max_loaded = max(max_loaded, len(loaded))
        else:
            loaded.remove(step[1:])
        start = destination
    if loaded:
        raise ValueError("Not all requested deliveries are included")
    home_m = segment_cost(costs, start, "HOME")
    return {"to_last_task_m": total_m, "return_home_m": home_m,
            "with_home_m": total_m + home_m, "empty_with_home_m": empty_m + home_m,
            "max_simultaneous_virtual_bags": max_loaded}


def comparison(snapshot):
    revision, _, costs = load_snapshot(snapshot)
    waypoints = {}
    prerequisites = {}
    for number, (pickup, drop) in enumerate((*INITIAL_ORDERS, EXTRA_ORDER), 1):
        waypoints[f"P{number}"] = pickup
        waypoints[f"D{number}"] = drop
        prerequisites[f"D{number}"] = [f"P{number}"]
    initial = {key: value for key, value in waypoints.items() if not key.endswith("5")}
    fifo = build_order(initial, prerequisites, costs, "HOME", greedy=False)
    started = perf_counter()
    best, _ = find_best_order(initial, prerequisites, costs, "HOME")
    elapsed_initial = perf_counter() - started

    # Hold the same active delivery during both counterfactuals and compare
    # from its destination after its handoff.
    through_first_delivery = next(index for index, step in enumerate(best)
                                  if step.startswith("D")) + 1
    finished = best[:through_first_delivery]
    departure = waypoints[finished[-1]]
    remaining = {key: value for key, value in waypoints.items() if key not in finished}
    loaded = [key[1:] for key in finished if key.startswith("P") and f"D{key[1:]}" not in finished]
    no_replan = (*best[through_first_delivery:], "P5", "D5")
    started = perf_counter()
    replanned, _ = find_best_order(remaining, prerequisites, costs, departure, satisfied=finished)
    elapsed_replan = perf_counter() - started
    return {
        "cost_revision": revision,
        "initial_from_home": {
            "fifo": {"order": fifo, **planned_distances(fifo, waypoints, costs, start="HOME")},
            "optimized": {"order": best, "compute_s": elapsed_initial,
                          **planned_distances(best, waypoints, costs, start="HOME")},
        },
        "dynamic_from_first_delivery_handoff": {
            "completed": finished, "start": departure,
            "append_new_order": {"order": no_replan,
                                 **planned_distances(no_replan, waypoints, costs,
                                                     start=departure, loaded=loaded)},
            "replan": {"order": replanned, "compute_s": elapsed_replan,
                       **planned_distances(replanned, waypoints, costs,
                                           start=departure, loaded=loaded)},
        },
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("snapshot", type=Path)
    print(json.dumps(comparison(parser.parse_args().snapshot), indent=2, allow_nan=False))

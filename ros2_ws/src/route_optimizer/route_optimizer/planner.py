"""Pure Python planning over a directed cost snapshot."""

from itertools import pairwise, permutations
from math import isfinite


class RouteUnavailable(ValueError):
    """A route segment or full visit order is unreachable."""


def segment_cost(costs, start, end):
    distance = costs.get((start, end))
    if distance is None or not isfinite(distance) or distance < 0:
        raise RouteUnavailable(f"Missing cost: {start} -> {end}")
    return distance


def total_distance(order, waypoints, costs, start, end=None):
    route = [start, *(waypoints[step] for step in order)]
    if end is not None:
        route.append(end)
    return sum(segment_cost(costs, a, b) for a, b in pairwise(route))


def valid_orders(waypoints, prerequisites, satisfied=()):
    for order in permutations(waypoints):
        seen = set(satisfied)
        for step in order:
            if not seen.issuperset(prerequisites.get(step, ())):
                break
            seen.add(step)
        else:
            yield order


def build_order(waypoints, prerequisites, costs, start, *, greedy, satisfied=()):
    pending = list(waypoints)
    seen = set(satisfied)
    order = []
    while pending:
        candidates = []
        for step in pending:
            if not seen.issuperset(prerequisites.get(step, ())):
                continue
            try:
                distance = segment_cost(costs, start, waypoints[step])
            except RouteUnavailable:
                continue
            candidates.append((step, distance))

        if not candidates:
            raise RouteUnavailable("No reachable route")
        chosen = min(candidates, key=lambda candidate: candidate[1]) if greedy else candidates[0]
        step = chosen[0]
        order.append(step)
        pending.remove(step)
        seen.add(step)
        start = waypoints[step]
    return tuple(order)


def find_best_order(waypoints, prerequisites, costs, start, *, satisfied=(), end=None):
    best = None
    satisfied = tuple(satisfied)
    for order in valid_orders(waypoints, prerequisites, satisfied):
        try:
            distance = total_distance(order, waypoints, costs, start, end)
        except RouteUnavailable:
            continue
        if best is None or distance < best[1]:
            best = order, distance
    if best is None:
        raise RouteUnavailable("No reachable route")
    return best

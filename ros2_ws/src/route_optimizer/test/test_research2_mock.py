from itertools import permutations, pairwise
from unittest.mock import patch
import unittest


WAYPOINTS = ("HOME", "A", "B", "C", "D")
COST_ROWS = (
    (0, 3, 6, 8, 9),
    (4, 0, 2, 5, 6),
    (6, 3, 0, 2, 4),
    (5, 4, 3, 0, 2),
    (7, 6, 3, 4, 0),
)
COST = {
    (start, end): distance
    for start, row in zip(WAYPOINTS, COST_ROWS)
    for end, distance in zip(WAYPOINTS, row)
}

# 딕셔너리에 적힌 순서를 작업 등록 순서로 사용합니다.
STEP_WAYPOINT = {"S1": "A", "S2": "C", "S3": "B", "S4": "D"}
PREREQUISITES = {"S2": {"S1"}, "S4": {"S3"}}


def total_distance(order, *, costs=COST, start_waypoint="HOME"):
    """지정한 위치에서 출발해 모든 단계를 방문하고 HOME으로 돌아오는 비용."""
    route = (start_waypoint, *(STEP_WAYPOINT[step] for step in order), "HOME")
    total = 0
    for start, end in pairwise(route):
        if (start, end) not in costs:
            raise ValueError(f"Missing cost: {start} -> {end}")
        total += costs[start, end]
    return total


def build_order(*, greedy):
    pending = list(STEP_WAYPOINT)
    completed = set()
    order = []
    position = "HOME"

    while pending:
        ready = [
            step for step in pending
            if PREREQUISITES.get(step, set()) <= completed
        ]
        if greedy:
            chosen = min(ready, key=lambda step: COST[position, STEP_WAYPOINT[step]])
        else:
            chosen = ready[0]

        order.append(chosen)
        pending.remove(chosen)
        completed.add(chosen)
        position = STEP_WAYPOINT[chosen]

    return tuple(order)


def valid_orders(*, satisfied_steps=()):
    for order in permutations(STEP_WAYPOINT):
        completed = set(satisfied_steps)
        for step in order:
            if not PREREQUISITES.get(step, set()) <= completed:
                break
            completed.add(step)
        else:
            yield order

def find_best_order(*, costs=COST, start_waypoint="HOME", satisfied_steps=()):
    best = None
    for order in valid_orders(satisfied_steps=satisfied_steps):
        try:
            distance = total_distance(
                order, costs=costs, start_waypoint=start_waypoint
            )
        except ValueError:
            continue

        if best is None or distance < best[1]:
            best = (order, distance)

    if best is None:
        raise ValueError("No reachable route")

    return best

class Research2MockTest(unittest.TestCase):
    def test_fifo_uses_registration_order(self):
        order = build_order(greedy=False)
        self.assertEqual(order, ("S1", "S2", "S3", "S4"))
        self.assertEqual(total_distance(order), 22)

    def test_greedy_chooses_nearest_ready_step(self):
        order = build_order(greedy=True)
        self.assertEqual(order, ("S1", "S3", "S2", "S4"))
        self.assertEqual(total_distance(order), 16)

    def test_exhaustive_search_finds_minimum_cost(self):
        candidates = list(valid_orders())
        self.assertEqual(len(candidates), 6)
        best = min(candidates, key=total_distance)
        self.assertEqual(best, ("S1", "S3", "S2", "S4"))
        self.assertEqual(total_distance(best), 16)

    def test_cost_depends_on_direction(self):
        self.assertEqual(COST["A", "B"], 2)
        self.assertEqual(COST["B", "A"], 3)

    def test_missing_cost_identifies_the_segment(self):
        costs = COST.copy()
        del costs["A", "B"]

        with self.assertRaisesRegex(ValueError, "Missing cost: A -> B"):
            total_distance(("S1", "S3", "S2", "S4"), costs=costs)

    def test_exhaustive_search_finds_alternative_when_cost_is_missing(self):
        costs = COST.copy()
        del costs["A", "B"]

        order, distance = find_best_order(costs=costs)

        self.assertEqual(order, ("S1", "S2", "S3", "S4"))
        self.assertEqual(distance, 22)

    def test_exhaustive_search_reports_no_reachable_route(self):
        costs = COST.copy()
        del costs["HOME", "A"]
        del costs["HOME", "B"]

        with self.assertRaisesRegex(ValueError, "No reachable route"):
            find_best_order(costs=costs)

    def test_pickup_precedes_dropoff_even_if_registered_later(self):
        delivery_first = {"S2": "C", "S4": "D", "S1": "A", "S3": "B"}

        with patch.dict(STEP_WAYPOINT, delivery_first, clear=True):
            results = {
                "FIFO": build_order(greedy=False),
                "Greedy": build_order(greedy=True),
                "완전탐색": find_best_order()[0],
            }

        for method, order in results.items():
            with self.subTest(method=method):
                self.assertCountEqual(order, ("S1", "S2", "S3", "S4"))
                self.assertLess(order.index("S1"), order.index("S2"))
                self.assertLess(order.index("S3"), order.index("S4"))

    def test_distinct_steps_at_same_waypoint_are_preserved(self):
        with patch.dict(STEP_WAYPOINT, {"S4": "C"}):
            results = {
                "FIFO": build_order(greedy=False),
                "Greedy": build_order(greedy=True),
                "완전탐색": find_best_order()[0],
            }

        for method, order in results.items():
            with self.subTest(method=method):
                self.assertCountEqual(order, ("S1", "S2", "S3", "S4"))
                self.assertLess(order.index("S1"), order.index("S2"))
                self.assertLess(order.index("S3"), order.index("S4"))

    def test_replan_keeps_only_pending_steps_after_current_handoff(self):
        pending = {"S2": "C", "S4": "D", "S5": "A", "S6": "C"}
        costs = COST.copy()
        for waypoint in ("A", "C", "D"):
            del costs["HOME", waypoint]

        with patch.dict(STEP_WAYPOINT, pending, clear=True):
            with patch.dict(PREREQUISITES, {"S6": {"S5"}}):
                order, _ = find_best_order(
                    costs=costs,
                    start_waypoint="B",
                    satisfied_steps=("S1", "S3"),
                )

        self.assertCountEqual(order, ("S2", "S4", "S5", "S6"))
        self.assertLess(order.index("S5"), order.index("S6"))
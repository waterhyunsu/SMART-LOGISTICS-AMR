import unittest
from unittest.mock import patch

from route_optimizer import planner
from route_optimizer.mock_costs import COST


# 딕셔너리에 적힌 순서를 작업 등록 순서로 사용합니다.
STEP_WAYPOINT = {"S1": "A", "S2": "C", "S3": "B", "S4": "D"}
PREREQUISITES = {"S2": {"S1"}, "S4": {"S3"}}


def total_distance(order, *, costs=COST):
    return planner.total_distance(order, STEP_WAYPOINT, costs, "HOME", "HOME")


def build_order(*, greedy, costs=COST):
    return planner.build_order(
        STEP_WAYPOINT, PREREQUISITES, costs, "HOME", greedy=greedy
    )


def find_best_order(*, costs=COST, start_waypoint="HOME", satisfied_steps=()):
    return planner.find_best_order(
        STEP_WAYPOINT, PREREQUISITES, costs, start_waypoint,
        satisfied=satisfied_steps, end="HOME",
    )


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
        candidates = list(planner.valid_orders(STEP_WAYPOINT, PREREQUISITES))
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
    def test_greedy_skips_a_missing_segment(self):
        costs = dict(COST)
        del costs["A", "B"]
        self.assertEqual(build_order(greedy=True, costs=costs), ("S1", "S2", "S3", "S4"))

    def test_greedy_reports_no_reachable_segment(self):
        costs = dict(COST)
        del costs["HOME", "A"]
        del costs["HOME", "B"]
        with self.assertRaisesRegex(planner.RouteUnavailable, "No reachable route"):
            build_order(greedy=True, costs=costs)

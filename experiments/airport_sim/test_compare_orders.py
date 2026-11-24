"""Pin loaded/empty distance accounting and the separate HOME leg."""

from compare_orders import planned_distances


def test_home_and_empty_legs_are_counted_separately():
    steps = {"P1": "S1", "D1": "G1", "P2": "S2", "D2": "G2"}
    costs = {("HOME", "S1"): 1, ("S1", "G1"): 2, ("G1", "S2"): 3,
             ("S2", "G2"): 4, ("G2", "HOME"): 5}
    summary = planned_distances(tuple(steps), steps, costs, start="HOME")
    assert summary == {"to_last_task_m": 10, "return_home_m": 5,
                       "with_home_m": 15, "empty_with_home_m": 9,
                       "max_simultaneous_virtual_bags": 1}

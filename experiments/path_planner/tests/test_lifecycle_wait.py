"""MOCK tests of the lifecycle gate: no trial may start before planner_server is active. No ROS."""

import pytest

import bench_core as core


class Clock:
    def __init__(self):
        self.now = 0.0

    def sleep(self, seconds):
        self.now += seconds

    def __call__(self):
        return self.now


def test_waits_through_inactive_until_every_node_is_active():
    clock = Clock()
    # planner_server answers "inactive" for the first second: its action server is already discoverable then
    states = {'map_server': lambda t: (3, 'active'),
              'planner_server': lambda t: (2, 'inactive') if t < 1.0 else (3, 'active')}
    asked = []

    def get_state(name):
        asked.append((name, round(clock.now, 1)))
        return states[name](clock.now)

    seen = core.wait_until_active(get_state, ['map_server', 'planner_server'], 30.0, sleep=clock.sleep, clock=clock)
    assert seen == {'map_server': 'active [3]', 'planner_server': 'active [3]'}
    assert clock.now == pytest.approx(1.0) and asked[0] == ('map_server', 0.0)
    assert [n for n, _ in asked].count('planner_server') == 6            # polled every 0.2 s until active


def test_a_node_that_stays_inactive_stops_the_run_and_names_its_state():
    clock = Clock()
    with pytest.raises(RuntimeError, match=r'/planner_server is not active after 5.0 s \(last state: inactive \[2\]\)'):
        core.wait_until_active(lambda name: (2, 'inactive'), ['planner_server'], 5.0, sleep=clock.sleep, clock=clock)
    assert clock.now >= 5.0


def test_a_node_that_never_answers_stops_the_run():
    clock = Clock()
    with pytest.raises(RuntimeError, match='last state: no answer'):
        core.wait_until_active(lambda name: None, ['planner_server'], 2.0, sleep=clock.sleep, clock=clock)


def test_the_timeout_covers_all_nodes_together():
    clock = Clock()

    def get_state(name):
        return (3, 'active') if name == 'map_server' and clock.now >= 4.0 else (1, 'unconfigured')

    with pytest.raises(RuntimeError, match='/planner_server is not active'):
        core.wait_until_active(get_state, ['map_server', 'planner_server'], 5.0, sleep=clock.sleep, clock=clock)
    assert clock.now < 6.0

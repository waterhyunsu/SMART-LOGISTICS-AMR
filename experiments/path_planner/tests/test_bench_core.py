"""MOCK tests of the planner comparison core: no ROS, the planner is a test double."""

import csv
import json
import math
import os

import pytest

import bench_core as core

BENCH_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCENARIOS = os.path.join(BENCH_DIR, 'scenarios', 'lecture_hall_v1.yaml')
PARAMS = os.path.join(BENCH_DIR, 'config', 'navfn_compare.yaml')


@pytest.fixture
def scenarios():
    return core.load_scenarios(SCENARIOS)


@pytest.fixture
def planners():
    return core.load_planner_configs(PARAMS)


def write(tmp_path, name, text):
    path = tmp_path / name
    path.write_text(text, encoding='utf-8')
    return str(path)


SCENARIO_TEMPLATE = """
scenario_set: t
map: {{name: m, yaml: m.yaml}}
points: {{a: [0, 0, 0], b: [1, 0, 90]}}
pairs:
{pairs}
"""


def test_lecture_hall_runs_are_directed_and_include_failures(scenarios):
    ids = [r.run_id for r in scenarios.runs]
    assert ids == ['R1-A:fwd', 'R1-A:rev', 'R1-B:fwd', 'R1-B:rev', 'R1-C:fwd', 'R1-C:rev',
                   'FAIL-WALLED:fwd', 'FAIL-OFFMAP:fwd']
    fwd, rev = scenarios.runs[0], scenarios.runs[1]
    assert (fwd.start, fwd.goal) == (rev.goal, rev.start)
    assert {r.run_id for r in scenarios.runs if r.expect == 'failure'} == {'FAIL-WALLED:fwd', 'FAIL-OFFMAP:fwd'}
    assert scenarios.frame_id == 'map' and os.path.isfile(scenarios.map_yaml)


def test_failure_goals_lie_outside_free_space_of_the_map(scenarios):
    info = core.map_info(scenarios.map_yaml)
    assert info == {'width': 190, 'height': 213, 'resolution': 0.05, 'origin': [-2.45, -1.05]}
    x_max = info['origin'][0] + info['width'] * info['resolution']
    offmap = next(r for r in scenarios.runs if r.pair_id == 'FAIL-OFFMAP').goal
    assert offmap.x > x_max


@pytest.mark.parametrize('pairs, message', [
    ('  - {id: p, from: a, to: c, expect: success}', 'unknown point'),
    ('  - {id: p, from: a, to: b, expect: maybe}', 'expect must be'),
    ('  - {id: p, from: a, to: a, expect: success}', 'same point'),
    ('  - {id: p, from: a, to: b, expect: success}\n  - {id: p, from: b, to: a, expect: success}', 'duplicate'),
    ('  - {from: a, to: b, expect: success}', 'missing or duplicate'),
])
def test_invalid_scenarios_are_rejected(tmp_path, pairs, message):
    path = write(tmp_path, 's.yaml', SCENARIO_TEMPLATE.format(pairs=pairs))
    with pytest.raises(ValueError, match=message):
        core.load_scenarios(path)


def test_bad_point_and_missing_keys_are_rejected(tmp_path):
    bad_point = SCENARIO_TEMPLATE.format(pairs='  - {id: p, from: a, to: b, expect: success}').replace(
        'b: [1, 0, 90]', 'b: [1, 0]')
    with pytest.raises(ValueError, match='expected'):
        core.load_scenarios(write(tmp_path, 'p.yaml', bad_point))
    with pytest.raises(ValueError, match='missing key'):
        core.load_scenarios(write(tmp_path, 'k.yaml', 'scenario_set: t\npoints: {}\n'))


def test_config_pair_differs_only_in_use_astar(planners):
    assert [(c.planner_id, c.algorithm) for c in planners] == [('NavFnAStar', 'astar'), ('NavFnDijkstra', 'dijkstra')]
    assert core.config_differences(planners) == ['use_astar']
    core.check_comparison_pair(planners)


def test_config_pair_with_another_difference_is_refused(tmp_path):
    with open(PARAMS, encoding='utf-8') as f:
        text = f.read()
    changed = text.replace('tolerance: 0.5\n      use_astar: false', 'tolerance: 0.25\n      use_astar: false')
    assert changed != text
    configs = core.load_planner_configs(write(tmp_path, 'p.yaml', changed))
    with pytest.raises(ValueError, match='tolerance'):
        core.check_comparison_pair(configs)
    same = text.replace('use_astar: false', 'use_astar: true')
    with pytest.raises(ValueError, match='exactly one'):
        core.check_comparison_pair(core.load_planner_configs(write(tmp_path, 'q.yaml', same)))
    with pytest.raises(ValueError, match='true or false'):
        core.load_planner_configs(write(tmp_path, 'r.yaml', text.replace('use_astar: true', 'use_astar: "yes"')))


def test_versions_ignore_line_endings_but_track_content(tmp_path):
    lf = tmp_path / 'lf' / 'c.yaml'
    crlf = tmp_path / 'crlf' / 'c.yaml'
    for p in (lf, crlf):
        p.parent.mkdir()
    lf.write_bytes(b'a: 1\nb: 2\n')
    crlf.write_bytes(b'a: 1\r\nb: 2\r\n')
    assert core.config_version(str(lf)) == core.config_version(str(crlf))
    crlf.write_bytes(b'a: 1\r\nb: 3\r\n')
    assert core.config_version(str(lf)) != core.config_version(str(crlf))


def test_map_version_changes_with_the_image(tmp_path):
    (tmp_path / 'm.pgm').write_bytes(b'P5\n2 1\n255\n\x00\xfe')
    yaml_path = write(tmp_path, 'm.yaml', 'image: m.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n')
    before = core.map_version('m', yaml_path)
    assert core.map_info(yaml_path)['width'] == 2
    (tmp_path / 'm.pgm').write_bytes(b'P5\n2 1\n255\n\xfe\xfe')
    assert core.map_version('m', yaml_path) != before and before.startswith('m@')


def test_geometry_helpers():
    assert core.path_length([(0, 0), (3, 4), (3, 5)]) == pytest.approx(6.0)
    assert core.path_length([(1, 1)]) == 0
    assert core.end_error([(0, 0), (1, 0)], core.Pose2D(1, 1, 0)) == pytest.approx(1.0)
    assert core.end_error([], core.Pose2D(1, 1, 0)) is None
    _, _, z, w = core.yaw_quaternion(90)
    assert (z, w) == pytest.approx((math.sqrt(0.5), math.sqrt(0.5)))


def test_error_table_reads_result_constants():
    class Result:
        NONE = 0
        GOAL_OUTSIDE_MAP = 204
        NO_VALID_PATH = 208
        flag = True
    assert core.error_table(Result) == {0: 'NONE', 204: 'GOAL_OUTSIDE_MAP', 208: 'NO_VALID_PATH'}


class FakePlanner:
    """Straight-line planner double; goals off the map fail like Nav2 does."""

    def __init__(self, fail_goals=(), tolerate=False):
        self.calls, self.fail_goals, self.tolerate = [], set(fail_goals), tolerate

    def __call__(self, planner_id, start, goal):
        self.calls.append((planner_id, start, goal))
        if (goal.x, goal.y) in self.fail_goals and not self.tolerate:
            return core.PlanOutcome(False, planning_time_ms=1.0, roundtrip_ms=2.0, error_code=208,
                                    error_name='NO_VALID_PATH', failure_reason='NO_VALID_PATH')
        return core.PlanOutcome(True, [(start.x, start.y), (goal.x, goal.y)], 0.5, 1.5, 0, 'NONE')


def run(scenarios, planners, plan_fn, **kw):
    args = dict(repeat=2, warmup=1, run_mode='MOCK', experiment_id='e1', config_version='c@1',
                map_version='m@1', scenario_version='s@1', clock=lambda: 't')
    args.update(kw)
    return core.run_trials(scenarios, planners, plan_fn, **args)


def failing_goals(scenarios):
    return {(r.goal.x, r.goal.y) for r in scenarios.runs if r.expect == 'failure'}


def test_both_planners_get_identical_requests_in_alternating_order(scenarios, planners):
    fake = FakePlanner(failing_goals(scenarios))
    rows = run(scenarios, planners, fake)
    runs = len(scenarios.runs)
    assert len(rows) == 3 * runs * 2 and len(fake.calls) == len(rows)
    for a, b in zip(fake.calls[::2], fake.calls[1::2]):
        assert a[1:] == b[1:] and a[0] != b[0]
    first = [rows[i]['planner_id'] for i in range(0, len(rows), runs * 2)]
    assert first == ['NavFnAStar', 'NavFnDijkstra', 'NavFnAStar']
    assert [r['warmup'] for r in rows[::runs * 2]] == [True, False, False]
    assert [r['repetition'] for r in rows[::runs * 2]] == [0, 1, 2]


def test_rows_follow_experiment_result_and_keep_failures(scenarios, planners):
    rows = run(scenarios, planners, FakePlanner(failing_goals(scenarios)))
    assert set(rows[0]) == set(core.TRIAL_FIELDS)
    ok = next(r for r in rows if r['scenario_id'] == 'R1-A:fwd')
    assert ok['plan_success'] and ok['planned_distance'] == pytest.approx(6.6)
    assert ok['actual_distance'] == '' and ok['goal_success'] == '' and ok['path_end_error'] == 0
    bad = next(r for r in rows if r['scenario_id'] == 'FAIL-WALLED:fwd')
    assert not bad['plan_success'] and bad['planned_distance'] == '' and bad['failure_reason'] == 'NO_VALID_PATH'
    assert bad['expectation_met'] and bad['error_code'] == 208
    assert all(r['expectation_met'] for r in rows)


def test_unexpected_success_is_flagged(scenarios, planners):
    rows = run(scenarios, planners, FakePlanner(tolerate=True))
    missed = {r['scenario_id'] for r in rows if not r['expectation_met']}
    assert missed == {'FAIL-WALLED:fwd', 'FAIL-OFFMAP:fwd'}


def test_empty_path_and_client_errors_are_failures(scenarios, planners):
    def empty(planner_id, start, goal):
        return core.PlanOutcome(True, [], 0.1, 0.2, 0, 'NONE')

    def broken(planner_id, start, goal):
        raise TimeoutError('no server')

    assert {r['failure_reason'] for r in run(scenarios, planners, empty)} == {'EMPTY_PATH'}
    rows = run(scenarios, planners, broken)
    assert {r['failure_reason'] for r in rows} == {'CLIENT_ERROR:TimeoutError'}
    assert rows[0]['error_msg'] == 'no server' and not rows[0]['plan_success']


def test_invalid_run_arguments(scenarios, planners):
    with pytest.raises(ValueError, match='run_mode'):
        run(scenarios, planners, FakePlanner(), run_mode='GAZEBO')
    with pytest.raises(ValueError, match='repeat'):
        run(scenarios, planners, FakePlanner(), repeat=0)


def test_summary_excludes_warmup(scenarios, planners):
    rows = run(scenarios, planners, FakePlanner(failing_goals(scenarios)), repeat=3)
    summary = core.summarize(rows)
    assert len(summary) == len(scenarios.runs) * 2
    a = next(s for s in summary if s['scenario_id'] == 'R1-A:fwd' and s['algorithm'] == 'dijkstra')
    assert (a['trials'], a['plan_successes'], a['success_rate']) == (3, 3, 100.0)
    assert a['planned_distance_mean'] == pytest.approx(6.6) and a['planned_distance_std'] == 0
    f = next(s for s in summary if s['scenario_id'] == 'FAIL-OFFMAP:fwd' and s['algorithm'] == 'astar')
    assert (f['plan_successes'], f['expectation_met'], f['failure_reasons']) == (0, 3, 'NO_VALID_PATH')
    assert f['planned_distance_mean'] == ''


def test_results_are_written_once_and_separable_by_version(tmp_path, scenarios, planners):
    meta = {'experiment_id': 'e1', 'run_mode': 'MOCK', 'config_version': 'c@1', 'map_version': 'm@1',
            'scenario_version': 's@1', 'repeat': 2, 'warmup': 1, 'env': {}}
    rows = run(scenarios, planners, FakePlanner(failing_goals(scenarios)))
    rows += run(scenarios, planners, FakePlanner(failing_goals(scenarios)), config_version='c@2')
    out = core.write_results(str(tmp_path / 'e1'), rows, core.summarize(rows), meta)
    with open(os.path.join(out, 'trials.csv'), encoding='utf-8') as f:
        loaded = list(csv.DictReader(f))
    assert list(loaded[0]) == core.TRIAL_FIELDS
    assert {r['config_version'] for r in loaded} == {'c@1', 'c@2'}
    assert {r['map_version'] for r in loaded} == {'m@1'}
    with open(os.path.join(out, 'meta.json'), encoding='utf-8') as f:
        assert json.load(f)['run_mode'] == 'MOCK'
    with open(os.path.join(out, 'summary.md'), encoding='utf-8') as f:
        assert 'R1-A:fwd' in f.read()
    with pytest.raises(FileExistsError):
        core.write_results(out, rows, [], meta)

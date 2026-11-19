"""MOCK tests of the path quality metrics (clearance, turning), the stored paths and the path figures. No ROS."""

import csv
import json
import math
import os
import random

import pytest

import bench_core as core
import plot_paths

BENCH_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
R1D = os.path.join(BENCH_DIR, 'scenarios', 'lecture_hall_v1_r1d.yaml')
PARAMS = os.path.join(BENCH_DIR, 'config', 'navfn_compare.yaml')


def small_map(tmp_path, occupied=((4, 1),), width=10, height=4):
    """Free map with 1 m cells and origin (0, 0); ``occupied`` cells are (col, row from the bottom)."""
    pixels = bytearray([254]) * (width * height)
    for i, j in occupied:
        pixels[(height - 1 - j) * width + i] = 0
    (tmp_path / 's.pgm').write_bytes(b'P5\n%d %d\n255\n' % (width, height) + bytes(pixels))
    path = tmp_path / 's.yaml'
    path.write_text('image: s.pgm\nmode: trinary\nresolution: 1.0\norigin: [0.0, 0.0, 0.0]\nnegate: 0\n'
                    'occupied_thresh: 0.65\nfree_thresh: 0.196\n', encoding='utf-8')
    return str(path)


def test_clearance_is_the_distance_to_the_nearest_occupied_cell(tmp_path):
    field = core.clearance_field(small_map(tmp_path))
    assert field.at(4.5, 1.5) == 0.0
    assert field.at(7.5, 1.5) == pytest.approx(3.0)
    assert field.at(0.2, 3.9) == pytest.approx(math.hypot(4, 2))
    assert field.at(-0.1, 1.0) is None and field.at(3.0, 4.0) is None


def test_map_without_obstacles_has_no_clearance(tmp_path):
    assert core.clearance_field(small_map(tmp_path, occupied=())) is None
    assert core.path_clearance(None, [(1.0, 1.0)]) == (None, None)


def test_clearance_field_matches_brute_force_on_the_lecture_hall_map():
    scenarios = core.load_scenarios(R1D)
    field = core.clearance_field(scenarios.map_yaml)
    width, height, _, pixels = core.read_pgm_p5(core.map_image_path(scenarios.map_yaml))
    occupied = [(i, j) for j in range(height) for i in range(width) if pixels[(height - 1 - j) * width + i] == 0]
    rng = random.Random(1)
    for _ in range(60):
        i, j = rng.randrange(width), rng.randrange(height)
        want = min(math.hypot(i - a, j - b) for a, b in occupied) * field.resolution
        assert field.dist[j * width + i] == pytest.approx(want, abs=1e-9)


def test_path_clearance_and_turning(tmp_path):
    field = core.clearance_field(small_map(tmp_path))
    low, mean = core.path_clearance(field, [(6.5, 1.5), (7.5, 1.5), (30.0, 1.5)])     # the last point is off the map
    assert (low, mean) == pytest.approx((2.0, 2.5))
    straight = [(x / 20, 0.0) for x in range(100)]
    corner = straight + [(straight[-1][0], y / 20) for y in range(1, 100)]
    assert core.turn_total(straight) == pytest.approx(0.0)
    assert core.turn_total(corner) == pytest.approx(90.0, abs=1.0)
    assert core.turn_total(corner[::-1]) == pytest.approx(90.0, abs=1.0)
    assert core.turn_total([(0, 0), (1, 0)]) == 0.0
    jitter = [(x / 100, 0.001 * (x % 2)) for x in range(300)]       # 1 mm wiggle every 1 cm
    assert core.turn_total(jitter) < 20.0 < 1000.0 < core.turn_total(jitter, step=0.001)


def test_resampled_length_drops_fine_zigzag_but_keeps_the_route():
    zigzag = [(x / 100, 0.02 * (x % 2)) for x in range(301)]         # 3 m long route with a 2 cm zig-zag
    assert core.path_length(zigzag) > 6.0
    assert core.resampled_length(zigzag) == pytest.approx(3.0, abs=0.05)
    corner = [(x / 20, 0.0) for x in range(41)] + [(2.0, y / 20) for y in range(1, 41)]
    assert core.resampled_length(corner) == pytest.approx(core.path_length(corner), abs=0.05)  # corner cut a little
    assert core.resampled_length([(0.0, 0.0), (0.03, 0.0)]) == pytest.approx(0.03)   # the last point always counts
    assert core.resampled_length([]) == 0 and core.resampled_length([(1.0, 1.0)]) == 0


def test_max_separation_is_symmetric_and_zero_for_the_same_path():
    a = [(x / 10, 0.0) for x in range(31)]
    b = [(x / 10, 0.0) for x in range(16)] + [(1.5, y / 10) for y in range(1, 21)]
    assert core.max_separation(a, a) == 0.0
    # the end of b, (1.5, 2.0), is 2.0 m from a; the end of a, (3.0, 0.0), is 1.5 m from b
    assert core.max_separation(a, b) == core.max_separation(b, a) == pytest.approx(2.0)
    assert core.max_separation(a, []) is None


class Fake:
    """Straight line on the plain map, an L-shaped detour on R1-D1, NO_VALID_PATH on R1-D2."""

    def __init__(self):
        self.variant = ''

    def set_map(self, variant):
        self.variant = variant

    def __call__(self, planner_id, start, goal):
        if self.variant == 'R1-D2':
            return core.PlanOutcome(False, error_code=208, error_name='NO_VALID_PATH', failure_reason='NO_VALID_PATH')
        line = [(start.x + (goal.x - start.x) * t / 50, start.y + (goal.y - start.y) * t / 50) for t in range(51)]
        if self.variant == 'R1-D1':
            line = [(start.x, start.y), (start.x + 2.3, start.y), (goal.x + 2.3, goal.y), (goal.x, goal.y)]
        return core.PlanOutcome(True, line, 0.5, 1.5, 0, 'NONE')


def mock_run(tmp_path, with_fields=True):
    scenarios = core.load_scenarios(R1D)
    planners = core.load_planner_configs(PARAMS)
    fake, store = Fake(), core.PathStore()
    fields = None
    if with_fields:
        fields = {'': core.clearance_field(scenarios.map_yaml)}
        for variant, boxes in scenarios.map_variants().items():
            fields[variant] = core.clearance_field(
                core.write_blocked_map(scenarios.map_yaml, boxes, str(tmp_path / 'maps'), variant))
    rows = core.run_trials(
        scenarios, planners, fake, repeat=2, warmup=1, run_mode='MOCK', experiment_id='e', config_version='c@1',
        map_version='m@1', scenario_version=core.scenario_version(R1D), clock=lambda: 't', set_map=fake.set_map,
        variant_map_versions={'R1-D1': 'a', 'R1-D2': 'b'}, clearance_fields=fields, path_store=store)
    return scenarios, rows, store


def test_rows_carry_quality_metrics_only_for_returned_paths(tmp_path):
    _, rows, _ = mock_run(tmp_path)
    plain = next(r for r in rows if r['scenario_id'] == 'R1-D1:fwd')
    assert plain['turn_sum_deg'] == 0.0 and 0.0 < plain['min_clearance'] <= plain['mean_clearance']
    detour = next(r for r in rows if r['scenario_id'] == 'R1-D1:fwd:blocked')
    assert detour['turn_sum_deg'] == pytest.approx(180.0, abs=0.5)
    failed = next(r for r in rows if r['scenario_id'] == 'R1-D2:fwd:blocked')
    assert (failed['min_clearance'], failed['mean_clearance'], failed['turn_sum_deg']) == ('', '', '')
    summary = {(s['scenario_id'], s['algorithm']): s for s in core.summarize(rows)}
    assert summary[('R1-D1:fwd', 'astar')]['min_clearance_min'] == plain['min_clearance']
    assert summary[('R1-D2:fwd:blocked', 'astar')]['turn_sum_deg_mean'] == ''


def test_blocked_runs_use_the_clearance_of_the_blocked_map(tmp_path):
    scenarios, _, _ = mock_run(tmp_path)
    box = scenarios.map_variants()['R1-D1'][0]
    plain = core.clearance_field(scenarios.map_yaml)
    blocked = core.clearance_field(
        core.write_blocked_map(scenarios.map_yaml, [box], str(tmp_path / 'again'), 'm'))
    assert plain.at(box.cx, box.cy) > 0.0 and blocked.at(box.cx, box.cy) == 0.0


def test_without_fields_the_clearance_columns_stay_empty(tmp_path):
    _, rows, _ = mock_run(tmp_path, with_fields=False)
    assert {r['min_clearance'] for r in rows} == {''}
    assert any(r['turn_sum_deg'] != '' for r in rows)


def test_path_store_keeps_one_measured_path_and_counts_identical_ones(tmp_path):
    _, rows, store = mock_run(tmp_path)
    items = {(p['scenario_id'], p['algorithm']): p for p in store.as_list()}
    assert len(items) == 12 and ('R1-D2:fwd:blocked', 'astar') not in items       # failures store no path
    item = items[('R1-D1:fwd:blocked', 'dijkstra')]
    assert (item['repetition'], item['successful_trials'], item['identical_trials']) == (1, 2, 2)
    assert item['poses'][0] == [0.0, 0.0] and item['map_variant'] == 'R1-D1'
    changed = dict(next(r for r in rows if r['scenario_id'] == 'R1-D1:fwd' and not r['warmup']))
    store.add(changed, [(0.0, 0.0), (9.0, 9.0)])
    assert store.items[('R1-D1:fwd', changed['planner_id'])]['identical_trials'] == 2
    assert store.items[('R1-D1:fwd', changed['planner_id'])]['successful_trials'] == 3


def test_figures_are_written_for_every_run(tmp_path):
    _, rows, store = mock_run(tmp_path)
    meta = {'experiment_id': 'e', 'run_mode': 'MOCK', 'config_version': 'c@1', 'map_version': 'm@1',
            'scenario_version': core.scenario_version(R1D), 'repeat': 2, 'warmup': 1, 'env': {},
            'scenario_file': 'scenarios/lecture_hall_v1_r1d.yaml'}
    out = core.write_results(str(tmp_path / 'e'), rows, core.summarize(rows), meta, paths=store.as_list())
    with open(os.path.join(out, 'paths.json'), encoding='utf-8') as f:
        assert len(json.load(f)['paths']) == 12
    with open(os.path.join(out, 'trials.csv'), encoding='utf-8') as f:
        assert next(csv.reader(f))[-4:] == ['min_clearance', 'mean_clearance', 'turn_sum_deg', 'resampled_distance']
    names = plot_paths.plot_results(out)
    assert len(names) == 8 and 'paths_R1-D1_fwd_blocked.svg' in names
    assert not os.path.exists(os.path.join(out, '.plot_maps'))
    with open(os.path.join(out, 'paths_R1-D1_fwd_blocked.svg'), encoding='utf-8') as f:
        detour = f.read()
    assert detour.count('<polyline') == 2 and 'NavFn Dijkstra: ' in detour and '꺾인 각도 합 180°' in detour
    assert 'Dijkstra 경로와의 최대 간격: NavFn A* 0.00 m' in detour and '(0.10 m 간격 ' in detour
    with open(os.path.join(out, 'paths_R1-D2_rev_blocked.svg'), encoding='utf-8') as f:
        failed = f.read()
    assert '<polyline' not in failed and failed.count('경로 없음 (NO_VALID_PATH)') == 2


def test_figures_refuse_a_changed_scenario_file(tmp_path):
    _, rows, store = mock_run(tmp_path)
    meta = {'experiment_id': 'e', 'run_mode': 'MOCK', 'config_version': 'c@1', 'map_version': 'm@1',
            'scenario_version': 'lecture_hall_v1_r1d@000000000000', 'repeat': 2, 'warmup': 1, 'env': {},
            'scenario_file': 'scenarios/lecture_hall_v1_r1d.yaml'}
    out = core.write_results(str(tmp_path / 'e'), rows, core.summarize(rows), meta, paths=store.as_list())
    with pytest.raises(ValueError, match='not the version'):
        plot_paths.plot_results(out)

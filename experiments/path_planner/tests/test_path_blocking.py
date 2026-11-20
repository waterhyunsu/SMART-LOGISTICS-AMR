"""MOCK tests of path blocking (R1-D): obstacle parsing, blocked map copies, map switching order. No ROS."""

import csv
import os

import pytest

import bench_core as core

BENCH_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
R1D = os.path.join(BENCH_DIR, 'scenarios', 'lecture_hall_v1_r1d.yaml')
PLAIN = os.path.join(BENCH_DIR, 'scenarios', 'lecture_hall_v1.yaml')
PARAMS = os.path.join(BENCH_DIR, 'config', 'navfn_compare.yaml')
FIRST_RESULT = os.path.join(BENCH_DIR, 'results', '20261118T061956Z_SIM_lecture_hall_v1')


@pytest.fixture
def r1d():
    return core.load_scenarios(R1D)


@pytest.fixture
def planners():
    return core.load_planner_configs(PARAMS)


def write(tmp_path, name, text):
    path = tmp_path / name
    path.write_text(text, encoding='utf-8')
    return str(path)


TEMPLATE = """
scenario_set: t
map: {{name: m, yaml: m.yaml}}
points: {{a: [0, 0, 0], b: [4, 0, 90]}}
pairs:
  - {{id: p, from: a, to: b, expect: success{extra}}}
"""


def test_r1d_pairs_are_planned_before_and_after_blocking(r1d):
    assert [r.run_id for r in r1d.runs] == [
        'R1-D1:fwd', 'R1-D1:rev', 'R1-D1:fwd:blocked', 'R1-D1:rev:blocked',
        'R1-D2:fwd', 'R1-D2:rev', 'R1-D2:fwd:blocked', 'R1-D2:rev:blocked']
    expect = {r.run_id: (r.expect, r.map_variant) for r in r1d.runs}
    assert expect['R1-D1:fwd'] == ('success', '') and expect['R1-D1:fwd:blocked'] == ('success', 'R1-D1')
    assert expect['R1-D2:rev'] == ('success', '') and expect['R1-D2:rev:blocked'] == ('failure', 'R1-D2')
    variants = r1d.map_variants()
    assert list(variants) == ['R1-D1', 'R1-D2'] and [len(v) for v in variants.values()] == [1, 3]
    # the plain-map runs keep the boxes so their path can be checked against the spot that gets blocked
    assert all(r.obstacles for r in r1d.runs)


def test_scenarios_without_obstacles_have_no_map_variants():
    plain = core.load_scenarios(PLAIN)
    assert plain.map_variants() == {} and all(not r.obstacles and not r.map_variant for r in plain.runs)


@pytest.mark.parametrize('extra, message', [
    (', obstacles: [[2, 0, 1, 1]]', 'needs expect_blocked'),
    (', expect_blocked: failure', 'only valid with obstacles'),
    (', obstacles: [[2, 0, 1]], expect_blocked: failure', 'obstacle must be'),
    (', obstacles: [[2, 0, 0, 1]], expect_blocked: failure', 'positive size'),
    (', obstacles: [[2, 0, x, 1]], expect_blocked: failure', 'non-numeric'),
    (', obstacles: [[2, 0, 1, 1]], expect_blocked: maybe', 'needs expect_blocked'),
    (', obstacles: [[0, 0, 1, 1]], expect_blocked: failure', "covers point 'a'"),
])
def test_invalid_obstacles_are_rejected(tmp_path, extra, message):
    with pytest.raises(ValueError, match=message):
        core.load_scenarios(write(tmp_path, 's.yaml', TEMPLATE.format(extra=extra)))


def small_map(tmp_path, negate=0):
    """10 x 4 free map, 1 m cells, origin (0, 0)."""
    (tmp_path / 's.pgm').write_bytes(b'P5\n# test\n10 4\n255\n' + bytes([254]) * 40)
    return write(tmp_path, 's.yaml', f'image: s.pgm\nmode: trinary\nresolution: 1.0\norigin: [0.0, 0.0, 0.0]\n'
                                     f'negate: {negate}\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')


def test_box_cells_follow_the_box_and_its_rotation():
    # cells count when their centre is inside the box grown by half a cell (0.5 m here)
    box = core.Box(5.5, 1.5, 2.6, 0.6)
    assert core.box_cells(box, 10, 4, 1.0, (0, 0)) == [(4, 1), (5, 1), (6, 1)]
    turned = core.Box(5.5, 1.5, 2.6, 0.6, 90)
    assert core.box_cells(turned, 10, 4, 1.0, (0, 0)) == [(5, 0), (5, 1), (5, 2)]
    assert core.box_cells(core.Box(30.0, 1.0, 1.0, 1.0), 10, 4, 1.0, (0, 0)) == []


def test_blocked_map_marks_only_box_cells_and_is_reproducible(tmp_path):
    base = small_map(tmp_path)
    out = core.write_blocked_map(base, [core.Box(5.5, 1.5, 2.6, 0.6)], str(tmp_path / 'v'), 's_P')
    _, _, _, pixels = core.read_pgm_p5(os.path.join(tmp_path, 'v', 's_P.pgm'))
    # PGM rows start at the top: map row 1 (from the bottom) of a 4-row map is image row 2
    assert [i for i, v in enumerate(pixels) if v != 254] == [2 * 10 + 4, 2 * 10 + 5, 2 * 10 + 6]
    assert {pixels[2 * 10 + 4]} == {0}
    assert core.map_info(out) == core.map_info(base)
    again = core.write_blocked_map(base, [core.Box(5.5, 1.5, 2.6, 0.6)], str(tmp_path / 'w'), 's_P')
    assert core.map_version('m', out) == core.map_version('m', again) != core.map_version('m', base)


def test_blocked_map_respects_negate_and_rejects_boxes_off_the_map(tmp_path):
    base = small_map(tmp_path, negate=1)
    core.write_blocked_map(base, [core.Box(0.5, 0.5, 0.5, 0.5)], str(tmp_path / 'v'), 'n')
    _, _, maxval, pixels = core.read_pgm_p5(os.path.join(tmp_path, 'v', 'n.pgm'))
    assert pixels[3 * 10 + 0] == maxval == 255
    with pytest.raises(ValueError, match='does not cover any cell'):
        core.write_blocked_map(base, [core.Box(30.0, 1.0, 1.0, 1.0)], str(tmp_path / 'x'), 'n')


def test_r1d_boxes_block_open_aisle_cells_of_the_lecture_hall_map(r1d, tmp_path):
    info = core.map_info(r1d.map_yaml)
    width, height, _, pixels = core.read_pgm_p5(core.map_image_path(r1d.map_yaml))
    for variant, boxes in r1d.map_variants().items():
        for box in boxes:
            cells = core.box_cells(box, width, height, info['resolution'], info['origin'])
            free = sum(1 for i, j in cells if pixels[(height - 1 - j) * width + i] == 254)
            # 0.8 x 0.3 m is 96 cells of 5 cm; the half-cell margin adds a ring. All of them are open aisle now.
            assert len(cells) >= 96 and free == len(cells), (variant, box, len(cells), free)
        out = core.write_blocked_map(r1d.map_yaml, boxes, str(tmp_path), f'm_{variant}')
        assert core.map_info(out) == info


def test_obstacle_hits_counts_path_points_inside_boxes():
    boxes = (core.Box(0.0, 3.3, 0.8, 0.3),)
    straight = [(0.0, y / 10) for y in range(0, 66)]
    assert core.obstacle_hits(straight, boxes) == 3
    assert core.obstacle_hits([(2.3, 3.3), (1.0, 3.3)], boxes) == 0


class FakeMap:
    """Planner double that knows which map is loaded: straight line if free, else a detour, else NO_VALID_PATH."""

    def __init__(self, scenarios, fail_switch=None):
        self.boxes = {'': ()} | scenarios.map_variants()
        self.variant, self.switches, self.calls, self.fail_switch = '', [], [], fail_switch

    def set_map(self, variant):
        if variant == self.fail_switch:
            raise RuntimeError(f'costmap did not show {variant}')
        self.variant = variant
        self.switches.append(variant)

    def __call__(self, planner_id, start, goal):
        self.calls.append(self.variant)
        line = [(start.x + (goal.x - start.x) * t / 40, start.y + (goal.y - start.y) * t / 40) for t in range(41)]
        boxes = self.boxes[self.variant]
        if not core.obstacle_hits(line, boxes):
            return core.PlanOutcome(True, line, 0.5, 1.5, 0, 'NONE')
        if len(boxes) >= 3:
            return core.PlanOutcome(False, planning_time_ms=None, roundtrip_ms=3.0, error_code=208,
                                    error_name='NO_VALID_PATH', failure_reason='NO_VALID_PATH')
        detour = [(start.x, start.y), (start.x + 2.3, start.y), (goal.x + 2.3, goal.y), (goal.x, goal.y)]
        return core.PlanOutcome(True, detour, 0.7, 1.7, 0, 'NONE')


def run(scenarios, planners, fake, **kw):
    args = dict(repeat=2, warmup=1, run_mode='MOCK', experiment_id='e', config_version='c@1', map_version='m@1',
                scenario_version='s@1', clock=lambda: 't', set_map=fake.set_map,
                variant_map_versions={'R1-D1': 'm+R1-D1@1', 'R1-D2': 'm+R1-D2@1'})
    args.update(kw)
    return core.run_trials(scenarios, planners, fake, **args)


def test_maps_switch_once_per_group_and_rows_carry_the_variant_version(r1d, planners):
    fake = FakeMap(r1d)
    rows = run(r1d, planners, fake)
    assert fake.switches == ['', 'R1-D1', 'R1-D2']
    assert len(rows) == 3 * len(r1d.runs) * 2
    for r in rows:
        assert r['map_version'] == {'': 'm@1', 'R1-D1': 'm+R1-D1@1', 'R1-D2': 'm+R1-D2@1'}[r['map_variant']]
        assert r['expectation_met'], r['scenario_id']
    # every request ran on the map its run belongs to
    assert fake.calls == [r['map_variant'] for r in rows]


def test_blocking_turns_a_crossing_path_into_a_detour_or_a_failure(r1d, planners):
    rows = run(r1d, planners, FakeMap(r1d))
    by_id = {}
    for r in rows:
        by_id.setdefault(r['scenario_id'], []).append(r)
    assert {r['obstacle_hits'] > 0 for r in by_id['R1-D1:fwd']} == {True}        # original route crossed the box
    assert {r['obstacle_hits'] for r in by_id['R1-D1:fwd:blocked']} == {0}        # regenerated route avoids it
    assert by_id['R1-D1:fwd:blocked'][0]['planned_distance'] > by_id['R1-D1:fwd'][0]['planned_distance']
    blocked = by_id['R1-D2:rev:blocked'][0]
    assert not blocked['plan_success'] and blocked['failure_reason'] == 'NO_VALID_PATH'
    assert blocked['planned_distance'] == '' and blocked['obstacle_hits'] == ''
    summary = {(s['scenario_id'], s['algorithm']): s for s in core.summarize(rows)}
    assert summary[('R1-D1:fwd', 'astar')]['obstacle_crossings'] == 2
    assert summary[('R1-D1:fwd:blocked', 'astar')]['obstacle_crossings'] == 0
    assert summary[('R1-D2:fwd:blocked', 'dijkstra')]['obstacle_crossings'] == ''
    assert summary[('R1-D2:fwd:blocked', 'dijkstra')]['map_variant'] == 'R1-D2'


def test_a_failed_map_switch_stops_before_measuring_on_that_map(r1d, planners):
    fake = FakeMap(r1d, fail_switch='R1-D2')
    with pytest.raises(RuntimeError, match='R1-D2'):
        run(r1d, planners, fake)
    assert 'R1-D2' not in fake.calls and fake.switches == ['', 'R1-D1']


def test_map_variants_need_set_map_and_versions(r1d, planners):
    with pytest.raises(ValueError, match='no set_map'):
        run(r1d, planners, FakeMap(r1d), set_map=None)
    with pytest.raises(ValueError, match='no map_version'):
        run(r1d, planners, FakeMap(r1d), variant_map_versions={'R1-D1': 'x'})


def test_plain_scenarios_never_switch_maps(planners):
    plain = core.load_scenarios(PLAIN)
    fake = FakeMap(plain)
    rows = run(plain, planners, fake, variant_map_versions=None)
    assert fake.switches == [] and {r['map_variant'] for r in rows} == {''}
    assert {r['obstacle_hits'] for r in rows} == {''}


def test_new_columns_only_append_to_the_earlier_result_format():
    for name, fields in (('trials.csv', core.TRIAL_FIELDS), ('summary.csv', core.SUMMARY_FIELDS)):
        with open(os.path.join(FIRST_RESULT, name), encoding='utf-8') as f:
            earlier = next(csv.reader(f))
        assert fields[:len(earlier)] == earlier, name

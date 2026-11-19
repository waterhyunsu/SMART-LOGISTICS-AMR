"""MOCK tests of the three-candidate comparison (NavFn A*, NavFn Dijkstra, Smac Planner 2D). No ROS."""

import os

import pytest
import yaml

import bench_core as core
import drive_core as drive
import plot_compare
import plot_paths

BENCH_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PAIR = os.path.join(BENCH_DIR, 'config', 'navfn_compare.yaml')
CANDIDATES = os.path.join(BENCH_DIR, 'config', 'planner_candidates.yaml')
DRIVE = os.path.join(BENCH_DIR, 'config', 'nav2_drive.yaml')
SCENARIOS = os.path.join(BENCH_DIR, 'scenarios', 'lecture_hall_v1.yaml')


def load(path):
    with open(path, encoding='utf-8') as f:
        return yaml.safe_load(f)


def test_candidates_are_the_navfn_pair_plus_smac_2d():
    configs = core.load_planner_configs(CANDIDATES)
    assert [(c.planner_id, c.algorithm) for c in configs] == [
        ('NavFnAStar', 'astar'), ('NavFnDijkstra', 'dijkstra'), ('Smac2D', 'smac2d')]
    core.check_comparison_pair(configs)
    pair, cand = load(PAIR), load(CANDIDATES)
    for pid in ('NavFnAStar', 'NavFnDijkstra'):
        assert cand['planner_server']['ros__parameters'][pid] == pair['planner_server']['ros__parameters'][pid]
    assert cand['global_costmap'] == pair['global_costmap'] and cand['map_server'] == pair['map_server']


def test_drive_config_uses_the_same_smac_block():
    smac = load(CANDIDATES)['planner_server']['ros__parameters']['Smac2D']
    nav = load(DRIVE)['planner_server']['ros__parameters']
    assert nav['planner_plugins'] == ['GridBased', 'Smac2D'] and nav['Smac2D'] == smac


def test_nested_plugin_parameters_get_their_server_names():
    smac = next(c for c in core.load_planner_configs(CANDIDATES) if c.algorithm == 'smac2d')
    flat = core.flat_params(smac.params)
    assert flat['smoother.w_data'] == 0.2 and flat['tolerance'] == 0.5 and 'smoother' not in flat


def test_unknown_plugins_and_duplicate_candidates_are_refused(tmp_path):
    text = open(CANDIDATES, encoding='utf-8').read()
    other = tmp_path / 'o.yaml'
    other.write_text(text.replace('nav2_smac_planner::SmacPlanner2D', 'nav2_theta_star_planner::ThetaStarPlanner'),
                     encoding='utf-8')
    with pytest.raises(ValueError, match='not a NavFn planner plugin or one of'):
        core.load_planner_configs(str(other))
    twice = tmp_path / 't.yaml'
    twice.write_text(text.replace('"Smac2D"]', '"Smac2D", "Smac2D"]'), encoding='utf-8')
    with pytest.raises(ValueError, match='may appear once'):
        core.check_comparison_pair(core.load_planner_configs(str(twice)))


def test_three_planners_rotate_through_every_position():
    scenarios = core.load_scenarios(SCENARIOS)
    planners = core.load_planner_configs(CANDIDATES)
    calls = []

    def fake(planner_id, start, goal):
        calls.append(planner_id)
        return core.PlanOutcome(True, [(start.x, start.y), (goal.x, goal.y)], 0.5, 1.0, 0, 'NONE')

    rows = core.run_trials(scenarios, planners, fake, repeat=3, warmup=0, run_mode='MOCK', experiment_id='e',
                           config_version='c', map_version='m', clock=lambda: 't')
    per_rep = len(scenarios.runs) * 3
    assert [calls[i * per_rep:i * per_rep + 3] for i in range(3)] == [
        ['NavFnAStar', 'NavFnDijkstra', 'Smac2D'], ['NavFnDijkstra', 'Smac2D', 'NavFnAStar'],
        ['Smac2D', 'NavFnAStar', 'NavFnDijkstra']]
    assert {r['algorithm'] for r in rows} == {'astar', 'dijkstra', 'smac2d'}
    assert len(core.summarize(rows)) == len(scenarios.runs) * 3


def test_figures_show_a_third_candidate_only_when_it_is_in_the_result():
    run = core.load_scenarios(SCENARIOS).runs[0]
    info = {'resolution': 1.0, 'origin': [-3.0, -2.0]}
    layers = (12, 12, 'M0 0h12v12h-12z', '')
    line = {'poses': [[run.start.x, run.start.y], [run.goal.x, run.goal.y]]}
    stat = {'planned_distance_mean': '6.6', 'resampled_distance_mean': '6.6', 'min_clearance_min': '0.85',
            'turn_sum_deg_mean': '0.0'}
    two = plot_paths.render_svg(run, info, layers, {'astar': line, 'dijkstra': line},
                                {'astar': stat, 'dijkstra': stat}, False)
    three = plot_paths.render_svg(run, info, layers, {'astar': line, 'dijkstra': line, 'smac2d': line},
                                  {'astar': stat, 'dijkstra': stat, 'smac2d': stat}, False)
    assert two.count('<polyline') == 2 and 'Smac 2D' not in two
    assert three.count('<polyline') == 3 and 'Smac 2D: 6.60 m' in three
    assert 'Dijkstra 경로와의 최대 간격: NavFn A* 0.00 m, Smac 2D 0.00 m' in three
    chart = plot_compare.render_chart('t', 's', 'm', 'n', [('R1-A:fwd', {'astar': (6.6, None), 'dijkstra': (6.6, None),
                                                                         'smac2d': (8.0, None)})])
    assert chart.count('<path d=') == 3 and 'Smac 2D' in chart and '>8<' in chart
    assert 'Smac 2D' not in plot_compare.render_chart('t', 's', 'm', 'n', [('R1-A:fwd', {'astar': (6.6, None)})])


POSES = '''pose {
  name: "waffle_pi"
  id: 70
  position {
    x: 1.25
    z: 0.01
  }
  orientation {
    z: 0.7
    w: 0.7
  }
}
pose {
  name: "base_footprint"
  position {
    x: 9
    y: 9
  }
}
'''


def test_true_pose_is_read_from_the_gazebo_pose_dump():
    assert drive.parse_model_pose(POSES, 'waffle_pi') == (1.25, 0.0)          # y is omitted when it is zero
    assert drive.parse_model_pose(POSES, 'base_footprint') == (9.0, 9.0)
    assert drive.parse_model_pose(POSES, 'box') is None and drive.parse_model_pose('', 'waffle_pi') is None

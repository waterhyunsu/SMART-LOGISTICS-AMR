"""MOCK tests of the driving comparison's ROS-free part: rows, summary, files, world conversion. No ROS, no Gazebo."""

import csv
import json
import math
import os

import pytest
import yaml

import bench_core as core
import drive_core as drive
import make_sim_world

BENCH_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def row(scenario_id, algorithm, ok=True, time_s=30.0, dist=6.6, recoveries=0, reason=''):
    r = {f: '' for f in drive.DRIVE_FIELDS}
    r.update({'scenario_id': scenario_id, 'pair_id': scenario_id.split(':')[0], 'direction': 'fwd',
              'map_variant': '', 'expected': 'success', 'algorithm': algorithm, 'goal_success': ok,
              'expectation_met': ok, 'execution_time': time_s if ok else 180.0, 'actual_distance': dist,
              'planned_distance': 6.6, 'recoveries': recoveries, 'final_error': 0.1 if ok else '',
              'failure_reason': reason})
    return r


def test_summary_uses_arrived_trials_for_time_and_distance():
    rows = [row('R1-A:fwd', 'astar', time_s=30.0, dist=6.6), row('R1-A:fwd', 'astar', time_s=34.0, dist=6.8),
            row('R1-A:fwd', 'astar', ok=False, dist=20.0, recoveries=7, reason='TIMEOUT'),
            row('R1-A:fwd', 'dijkstra', time_s=31.0)]
    summary = {(s['scenario_id'], s['algorithm']): s for s in drive.summarize(rows)}
    a = summary[('R1-A:fwd', 'astar')]
    assert (a['trials'], a['goal_successes'], a['success_rate'], a['expectation_met']) == (3, 2, 66.7, 2)
    assert a['execution_time_mean'] == 32.0 and a['execution_time_std'] == 2.0
    assert (a['execution_time_min'], a['execution_time_max']) == (30.0, 34.0)
    assert a['actual_distance_mean'] == pytest.approx(6.7)            # the timed-out 20 m is not averaged in
    assert a['recoveries_mean'] == pytest.approx(2.33) and a['recoveries_max'] == 7
    assert a['failure_reasons'] == 'TIMEOUT'
    assert summary[('R1-A:fwd', 'dijkstra')]['failure_reasons'] == ''


def test_summary_of_a_run_without_arrivals_has_empty_statistics():
    s = drive.summarize([row('R1-D2:fwd:blocked', 'astar', ok=False, reason='TIMEOUT')])[0]
    assert (s['goal_successes'], s['execution_time_mean'], s['actual_distance_mean']) == (0, '', '')


def test_final_error_is_dead_reckoned_from_the_start_pose():
    start = (1.0, 2.0, math.pi / 2)                  # map: at (1, 2) facing +y
    odom_start, odom_end = (5.0, 5.0, 0.0), (8.0, 5.0, 0.0)          # odom: 3 m straight ahead
    assert drive.relative_pose_error(start, odom_start, odom_end, (1.0, 5.0)) == pytest.approx(0.0, abs=1e-9)
    assert drive.relative_pose_error(start, odom_start, odom_end, (1.0, 5.5)) == pytest.approx(0.5)
    assert drive.relative_pose_error(start, odom_start, odom_start, (1.0, 2.0)) == pytest.approx(0.0)


def test_results_follow_experiment_result_and_do_not_overwrite(tmp_path):
    rows = [row('R1-A:fwd', 'astar'), row('R1-A:fwd', 'dijkstra')]
    meta = {'experiment_id': 'e', 'run_mode': 'SIM', 'note': 'n', 'config_version': 'c@1', 'map_version': 'm@1',
            'scenario_version': 's@1', 'repeat': 1, 'controller_id': 'dwb', 'env': {}}
    out = drive.write_results(str(tmp_path / 'e'), rows, drive.summarize(rows), meta)
    with open(os.path.join(out, 'trials.csv'), encoding='utf-8') as f:
        header = next(csv.reader(f))
    assert header[:len(core.EXPERIMENT_RESULT_FIELDS)] == core.EXPERIMENT_RESULT_FIELDS
    with open(os.path.join(out, 'summary.md'), encoding='utf-8') as f:
        text = f.read()
    assert 'R1-A:fwd' in text and '실제 주행 거리가 아니다' in text
    with open(os.path.join(out, 'meta.json'), encoding='utf-8') as f:
        assert json.load(f)['run_mode'] == 'SIM'
    with pytest.raises(FileExistsError):
        drive.write_results(out, rows, [], meta)


CLASSIC = """<?xml version="1.0"?>
<sdf version="1.6">
  <world name="hall">
    <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so"/>
    <include><uri>model://ground_plane</uri></include>
    <include><uri>model://sun</uri></include>
    <model name="walls"><static>true</static></model>
    <model name="desk_1"><static>true</static></model>
  </world>
</sdf>
"""


def test_world_conversion_keeps_models_and_drops_classic_parts(tmp_path):
    source = tmp_path / 'hall.world'
    source.write_text(CLASSIC, encoding='utf-8')
    name, models = make_sim_world.convert(str(source), str(tmp_path / 'out' / 'hall.sdf'))
    text = (tmp_path / 'out' / 'hall.sdf').read_text(encoding='utf-8')
    assert (name, models) == ('hall', 2)
    assert 'libgazebo' not in text and 'model://' not in text
    assert 'gz::sim::systems::Sensors' in text and 'gz::sim::systems::UserCommands' in text
    assert text.count('<model name=') == 3 and '<world name="hall">' in text      # 2 models + ground plane
    source.write_text(CLASSIC.replace('<model name="desk_1">', '<model name="desk_1"><include><uri>model://x</uri></include>'),
                      encoding='utf-8')
    with pytest.raises(ValueError, match='Classic-only'):
        make_sim_world.convert(str(source), str(tmp_path / 'out2' / 'hall.sdf'))


def test_committed_world_matches_the_legacy_world_and_the_scenario_map_name():
    scenarios = core.load_scenarios(os.path.join(BENCH_DIR, 'scenarios', 'lecture_hall_v1.yaml'))
    world = os.path.join(BENCH_DIR, 'sim', 'worlds', scenarios.map_name + '.sdf')
    with open(world, encoding='utf-8') as f:
        text = f.read()
    with open(os.path.join(BENCH_DIR, 'legacy_r1', 'worlds', 'lecture_hall_real.world'), encoding='utf-8') as f:
        legacy = f.read()
    assert f'<world name="{scenarios.map_name}">' in text
    assert text.count('<model name=') == legacy.count('<model name=') + 1


def test_drive_params_differ_from_the_bench_only_where_documented():
    with open(os.path.join(BENCH_DIR, 'config', 'nav2_drive.yaml'), encoding='utf-8') as f:
        nav = yaml.safe_load(f)
    with open(os.path.join(BENCH_DIR, 'config', 'navfn_compare.yaml'), encoding='utf-8') as f:
        bench = yaml.safe_load(f)
    planner = nav['planner_server']['ros__parameters']
    assert planner['planner_plugins'] == ['GridBased']
    assert planner['GridBased']['plugin'] == 'nav2_navfn_planner::NavfnPlanner'
    ref = bench['planner_server']['ros__parameters']['NavFnDijkstra']
    assert {k: planner['GridBased'][k] for k in ('tolerance', 'allow_unknown')} == \
        {k: ref[k] for k in ('tolerance', 'allow_unknown')}
    want = bench['global_costmap']['global_costmap']['ros__parameters']
    for name in ('global_costmap', 'local_costmap'):
        got = nav[name][name]['ros__parameters']
        assert got['robot_radius'] == want['robot_radius']
        assert got['inflation_layer']['inflation_radius'] == want['inflation_layer']['inflation_radius']
        assert got['inflation_layer']['cost_scaling_factor'] == want['inflation_layer']['cost_scaling_factor']
    assert 'yaml_filename' not in nav['map_server']['ros__parameters']

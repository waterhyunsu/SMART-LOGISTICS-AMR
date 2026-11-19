"""MOCK tests of the comparison bar charts. No ROS."""

import csv
import json
import os

import plot_compare


def write_result(tmp_path, fields, rows):
    out = tmp_path / 'e'
    out.mkdir()
    with open(out / 'summary.csv', 'w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    (out / 'meta.json').write_text(json.dumps({'experiment_id': 'e', 'run_mode': 'SIM', 'map_version': 'm@1'}),
                                   encoding='utf-8')
    return str(out)


def test_planner_results_get_distance_and_time_charts(tmp_path):
    fields = ['scenario_id', 'algorithm', 'planned_distance_mean', 'planned_distance_std', 'planning_time_median']
    rows = [dict(zip(fields, r)) for r in [
        ('R1-A:fwd', 'astar', '6.6', '0.0', '0.38'), ('R1-A:fwd', 'dijkstra', '6.6', '0.0', '0.39'),
        ('R1-C:fwd', 'astar', '17.5687', '0.0', '1.1'), ('R1-C:fwd', 'dijkstra', '9.239', '0.0', '0.9'),
        ('FAIL-WALLED:fwd', 'astar', '', '', ''), ('FAIL-WALLED:fwd', 'dijkstra', '', '', '')]]
    out = write_result(tmp_path, fields, rows)
    assert plot_compare.plot_results(out) == ['compare_planned_distance.svg', 'compare_planning_time.svg']
    with open(os.path.join(out, 'compare_planned_distance.svg'), encoding='utf-8') as f:
        svg = f.read()
    assert svg.count('<path d=') == 4 and 'FAIL-WALLED' not in svg          # runs without any value are left out
    assert '>17.57<' in svg and '>9.239<' in svg                            # the pair that differs is labelled
    assert '>6.6<' not in svg                                               # equal pairs are not
    assert '실제 주행 거리가 아니다' in svg


def test_drive_results_get_time_and_distance_charts_with_spread(tmp_path):
    fields = ['scenario_id', 'algorithm', 'execution_time_mean', 'execution_time_std', 'actual_distance_mean',
              'actual_distance_std']
    rows = [dict(zip(fields, r)) for r in [
        ('R1-A:fwd', 'astar', '30.0', '1.5', '6.6', '0.05'), ('R1-A:fwd', 'dijkstra', '', '', '', '')]]
    out = write_result(tmp_path, fields, rows)
    assert plot_compare.plot_results(out) == ['compare_execution_time.svg', 'compare_actual_distance.svg']
    with open(os.path.join(out, 'compare_execution_time.svg'), encoding='utf-8') as f:
        svg = f.read()
    assert svg.count('<path d=') == 1 and '값 없음' in svg and 'stroke-width="1.2"' in svg

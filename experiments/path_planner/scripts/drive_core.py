"""ROS-free part of the driving comparison: result rows, summary and files. ``run_drive.py`` fills the rows."""

import csv
import json
import math
import os
import statistics

import bench_core as core

# docs/INTERFACES.md §4.7 ExperimentResult fields first, then driving extras.
# planned_distance: m, length of the first plan of the trial. actual_distance: m, wheel odometry path length while
# the goal was active. execution_time: s of simulation time from sending the goal to its result.
# planning_time and mission_success stay empty: the planner is called many times per drive (replanning), and
# there is no mission in this bench.
DRIVE_FIELDS = core.EXPERIMENT_RESULT_FIELDS + [
    'scenario_set', 'scenario_version', 'pair_id', 'direction', 'map_variant', 'repetition', 'order', 'expected',
    'expectation_met', 'nav_status', 'error_code', 'recoveries', 'plan_count', 'last_plan_distance', 'final_error',
    'timeout', 'start_x', 'start_y', 'start_yaw_deg', 'goal_x', 'goal_y', 'goal_yaw_deg', 'stamp']
DRIVE_SUMMARY_FIELDS = [
    'scenario_id', 'pair_id', 'direction', 'map_variant', 'expected', 'algorithm', 'trials', 'goal_successes',
    'success_rate', 'expectation_met', 'execution_time_mean', 'execution_time_std', 'execution_time_min',
    'execution_time_max', 'actual_distance_mean', 'actual_distance_std', 'planned_distance_mean',
    'recoveries_mean', 'recoveries_max', 'final_error_max', 'failure_reasons']


def relative_pose_error(start, odom_start, odom_end, goal):
    """Distance (m) from the goal to where the robot ended, dead-reckoned from the known start pose.

    ``start``/``goal`` are (x, y, yaw_rad) or (x, y) in the map frame, ``odom_*`` are (x, y, yaw_rad) wheel
    odometry at goal send and at the result. In simulation wheel odometry over one trial is close to ground truth.
    """
    dx, dy = odom_end[0] - odom_start[0], odom_end[1] - odom_start[1]
    turn = start[2] - odom_start[2]
    x = start[0] + math.cos(turn) * dx - math.sin(turn) * dy
    y = start[1] + math.sin(turn) * dx + math.cos(turn) * dy
    return math.dist((x, y), goal[:2])


def _num(rows, key):
    return [float(r[key]) for r in rows if r[key] != '']


def _stat(values, fn, digits=3):
    return round(fn(values), digits) if values else ''


def summarize(rows):
    """Per (scenario run, algorithm) statistics. Time and distance statistics use successful trials only."""
    groups = {}
    for r in rows:
        groups.setdefault((r['scenario_id'], r['algorithm']), []).append(r)
    out = []
    for (scenario_id, algorithm), rs in groups.items():
        ok = [r for r in rs if r['goal_success'] is True or r['goal_success'] == 'True']
        times, dist = _num(ok, 'execution_time'), _num(ok, 'actual_distance')
        rec = _num(rs, 'recoveries')
        out.append({
            'scenario_id': scenario_id, 'pair_id': rs[0]['pair_id'], 'direction': rs[0]['direction'],
            'map_variant': rs[0]['map_variant'], 'expected': rs[0]['expected'], 'algorithm': algorithm,
            'trials': len(rs), 'goal_successes': len(ok), 'success_rate': round(100.0 * len(ok) / len(rs), 1),
            'expectation_met': sum(1 for r in rs if r['expectation_met'] is True or r['expectation_met'] == 'True'),
            'execution_time_mean': _stat(times, statistics.fmean, 2), 'execution_time_std': _stat(times, statistics.pstdev, 2),
            'execution_time_min': _stat(times, min, 2), 'execution_time_max': _stat(times, max, 2),
            'actual_distance_mean': _stat(dist, statistics.fmean), 'actual_distance_std': _stat(dist, statistics.pstdev),
            'planned_distance_mean': _stat(_num(ok, 'planned_distance'), statistics.fmean),
            'recoveries_mean': _stat(rec, statistics.fmean, 2), 'recoveries_max': _stat(rec, max, 0),
            'final_error_max': _stat(_num(ok, 'final_error'), max),
            'failure_reasons': ';'.join(sorted({r['failure_reason'] for r in rs if r['failure_reason']}))})
    return out


def _cell(value):
    return '-' if value == '' else str(value)


def render_summary_md(summary, meta):
    lines = [
        f"# {meta['experiment_id']}", '',
        f"- run_mode: **{meta['run_mode']}** ({meta.get('note') or '-'})",
        f"- config_version: `{meta['config_version']}` / map_version: `{meta['map_version']}` / "
        f"scenario_version: `{meta['scenario_version']}`",
        f"- repeat {meta['repeat']} / controller `{meta['controller_id']}` / ROS {meta['env'].get('ros_distro')} / "
        f"nav2 {meta['env'].get('nav2_bringup')} / Gazebo {meta['env'].get('gz_sim')}",
        '- 주행 시간은 목표 전송부터 결과까지의 시뮬레이션 시간(s), 주행 거리는 그동안의 바퀴 오도메트리 경로 길이(m)다. '
        '시간·거리 통계는 도착한 시행만 쓴다.',
        '- 첫 계획 길이는 목표를 보낸 뒤 처음 받은 계획 경로의 길이(m)이며 실제 주행 거리가 아니다.',
        '',
        '| 실행 | 기대 | 알고리즘 | 도착 | 기대 일치 | 주행 시간 평균 (s) | 표준편차 | 최소~최대 | 주행 거리 평균 (m) | 표준편차 '
        '| 첫 계획 길이 평균 (m) | 복구 평균 | 복구 최대 | 실패 사유 |',
        '| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |']
    for s in summary:
        span = '-' if s['execution_time_min'] == '' else f"{s['execution_time_min']}~{s['execution_time_max']}"
        lines.append(
            f"| {s['scenario_id']} | {s['expected']} | {s['algorithm']} | {s['goal_successes']}/{s['trials']} "
            f"| {s['expectation_met']}/{s['trials']} | {_cell(s['execution_time_mean'])} "
            f"| {_cell(s['execution_time_std'])} | {span} | {_cell(s['actual_distance_mean'])} "
            f"| {_cell(s['actual_distance_std'])} | {_cell(s['planned_distance_mean'])} "
            f"| {_cell(s['recoveries_mean'])} | {_cell(s['recoveries_max'])} | {s['failure_reasons'] or '-'} |")
    return '\n'.join(lines) + '\n'


def write_results(out_dir, rows, summary, meta):
    """Write trials.csv, summary.csv, summary.md and meta.json into a new directory (never overwrites)."""
    os.makedirs(out_dir, exist_ok=False)
    for name, fields, data in (('trials.csv', DRIVE_FIELDS, rows), ('summary.csv', DRIVE_SUMMARY_FIELDS, summary)):
        with open(os.path.join(out_dir, name), 'w', newline='', encoding='utf-8') as f:
            writer = csv.DictWriter(f, fieldnames=fields)
            writer.writeheader()
            writer.writerows(data)
    with open(os.path.join(out_dir, 'summary.md'), 'w', encoding='utf-8') as f:
        f.write(render_summary_md(summary, meta))
    with open(os.path.join(out_dir, 'meta.json'), 'w', encoding='utf-8') as f:
        json.dump(meta, f, ensure_ascii=False, indent=2)
        f.write('\n')
    return out_dir

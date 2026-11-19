#!/usr/bin/env python3
"""Bar charts of a result folder's summary.csv: NavFn A* next to Dijkstra for every scenario run.

Planner-only results get compare_planned_distance.svg and compare_planning_time.svg; driving results get
compare_execution_time.svg and compare_actual_distance.svg. Plain SVG, no ROS and no plotting library.

    python3 experiments/path_planner/scripts/plot_compare.py experiments/path_planner/results/<experiment_id>
"""

import csv
import json
import os
import sys
from xml.sax.saxutils import escape

SURFACE, INK, MUTED, GRID = '#fcfcfb', '#0b0b0b', '#52514e', '#e4e3df'
# same colours as plot_paths; a planner that is not in the result is left out
SERIES = [('astar', '#eb6834', 'NavFn A*'), ('dijkstra', '#2a78d6', 'NavFn Dijkstra'), ('smac2d', '#1baf7a', 'Smac 2D')]
LABEL_W, PLOT_W, RIGHT, BAR_H, GAP, GROUP_GAP, TOP = 190, 470, 150, 11, 2, 12, 66
# (file stem, value column, spread column, title, unit, note)
PLANNER_CHARTS = [
    ('compare_planned_distance', 'planned_distance_mean', 'planned_distance_std', '계획 경로 길이', 'm',
     '모든 Pose 사이 거리의 합. 실제 주행 거리가 아니다'),
    ('compare_planning_time', 'planning_time_median', '', '계획 시간 (중앙값)', 'ms',
     'planner_server 가 보고한 값. 실행 환경에 따라 달라지므로 같은 실험 안에서만 비교한다')]
DRIVE_CHARTS = [
    ('compare_execution_time', 'execution_time_mean', 'execution_time_std', '주행 시간 (평균 ± 표준편차)', 's',
     '목표 전송부터 결과까지의 시뮬레이션 시간. 도착한 시행만'),
    ('compare_actual_distance', 'actual_distance_mean', 'actual_distance_std', '실제 주행 거리 (평균 ± 표준편차)', 'm',
     '바퀴 오도메트리 경로 길이. 도착한 시행만')]


def nice_step(top):
    for step in (0.1, 0.2, 0.5, 1, 2, 5, 10, 20, 50, 100, 200, 500):
        if top / step <= 6:
            return step
    return 1000


def fmt(value):
    return f'{value:.3f}'.rstrip('0').rstrip('.') if value < 10 else f'{value:.2f}'.rstrip('0').rstrip('.')


def render_chart(title, subtitle, unit, note, groups):
    """SVG text. ``groups``: [(label, {algorithm: (value, spread or None) or None})] in display order."""
    series = [s for s in SERIES if any(s[0] in d for _, d in groups)]
    values = [v[0] + (v[1] or 0.0) for _, d in groups for v in d.values() if v]
    top = max(values) if values else 1.0
    step = nice_step(top)
    axis_max = step * (int(top / step) + 1)
    group_h = len(series) * BAR_H + (len(series) - 1) * GAP
    height = TOP + len(groups) * (group_h + GROUP_GAP) + 46
    width = LABEL_W + PLOT_W + RIGHT

    def x(v):
        return round(LABEL_W + v / axis_max * PLOT_W, 1)

    o = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {width} {height}" width="{width}" height="{height}" '
         f'font-family="sans-serif" role="img" aria-label="{escape(title)}">',
         f'<rect width="{width}" height="{height}" fill="{SURFACE}"/>',
         f'<text x="12" y="20" font-size="15" font-weight="600" fill="{INK}">{escape(title)} ({unit})</text>',
         f'<text x="12" y="37" font-size="11.5" fill="{MUTED}">{escape(subtitle)}</text>']
    lx = 12
    for _, color, label in series:                                  # legend row
        o.append(f'<rect x="{lx}" y="46" width="14" height="8" rx="2" fill="{color}"/>')
        o.append(f'<text x="{lx + 19}" y="54" font-size="11.5" fill="{INK}">{label}</text>')
        lx += 30 + 7 * len(label)
    bottom = TOP + len(groups) * (group_h + GROUP_GAP) - GROUP_GAP + 4
    tick = 0.0
    while tick <= axis_max + 1e-9:
        o.append(f'<line x1="{x(tick)}" y1="{TOP - 4}" x2="{x(tick)}" y2="{bottom}" stroke="{GRID}" stroke-width="1"/>')
        o.append(f'<text x="{x(tick)}" y="{bottom + 14}" font-size="10.5" fill="{MUTED}" text-anchor="middle">'
                 f'{fmt(tick)}</text>')
        tick += step
    y = TOP
    for label, data in groups:
        o.append(f'<text x="{LABEL_W - 8}" y="{y + group_h / 2 + 4}" font-size="11.5" fill="{INK}" '
                 f'text-anchor="end">{escape(label)}</text>')
        present = [data[a][0] for a, _, _ in series if data.get(a)]
        differs = len(present) >= 2 and min(present) > 0 and max(present) / min(present) >= 1.05
        for i, (alg, color, name) in enumerate(series):
            by = y + i * (BAR_H + GAP)
            v = data.get(alg)
            if not v:
                o.append(f'<text x="{LABEL_W + 4}" y="{by + BAR_H - 2}" font-size="10" fill="{MUTED}">값 없음</text>')
                continue
            value, spread = v
            w = max(x(value) - LABEL_W, 1)
            o.append(f'<path d="M{LABEL_W} {by}h{round(w - 3, 1)}a3 3 0 0 1 3 3v{BAR_H - 6}a3 3 0 0 1 -3 3h-{round(w - 3, 1)}z" '
                     f'fill="{color}"><title>{escape(label)} {name}: {fmt(value)} {unit}</title></path>')
            end = x(value)
            if spread:
                cy = by + BAR_H / 2
                o.append(f'<line x1="{x(max(value - spread, 0))}" y1="{cy}" x2="{x(value + spread)}" y2="{cy}" '
                         f'stroke="{INK}" stroke-width="1.2"/>')
                end = x(value + spread)
            if differs:                                   # label only the pairs that differ by 5 % or more
                o.append(f'<text x="{end + 5}" y="{by + BAR_H - 1.5}" font-size="10.5" fill="{INK}">{fmt(value)}</text>')
        y += group_h + GROUP_GAP
    o.append(f'<text x="12" y="{height - 10}" font-size="10.5" fill="{MUTED}">{escape(note)}. '
             f'숫자는 값이 5% 이상 벌어진 실행에만 적었다.</text>')
    o.append('</svg>')
    return '\n'.join(o) + '\n'


def plot_results(result_dir):
    with open(os.path.join(result_dir, 'summary.csv'), encoding='utf-8') as f:
        summary = list(csv.DictReader(f))
    with open(os.path.join(result_dir, 'meta.json'), encoding='utf-8') as f:
        meta = json.load(f)
    charts = DRIVE_CHARTS if 'execution_time_mean' in summary[0] else PLANNER_CHARTS
    subtitle = f"{meta['experiment_id']} · {meta['run_mode']} · {meta['map_version']}"
    written = []
    for stem, column, spread_column, title, unit, note in charts:
        groups = {}
        for s in summary:
            value = float(s[column]) if s[column] else None
            spread = float(s[spread_column]) if spread_column and s.get(spread_column) else None
            groups.setdefault(s['scenario_id'], {})[s['algorithm']] = (value, spread) if value is not None else None
        groups = [(k, v) for k, v in groups.items() if any(v.values())]     # runs with no value at all are left out
        with open(os.path.join(result_dir, stem + '.svg'), 'w', encoding='utf-8', newline='\n') as f:
            f.write(render_chart(title, subtitle, unit, note, groups))
        written.append(stem + '.svg')
    return written


if __name__ == '__main__':
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    for name in plot_results(sys.argv[1]):
        print(name)

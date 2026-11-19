#!/usr/bin/env python3
"""Draw the planned paths of a result folder on its map: one SVG per scenario run, both planners overlaid.

Reads meta.json, summary.csv and paths.json of the folder plus the scenario and map files meta.json names, and
writes paths_<run>.svg next to them. No ROS and no plotting library needed.

    python3 experiments/path_planner/scripts/plot_paths.py experiments/path_planner/results/<experiment_id>
"""

import csv
import json
import os
import sys
from xml.sax.saxutils import escape

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import bench_core as core  # noqa: E402

BENCH_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCALE = 4                      # px per map cell
PAD, TITLE_H, LEGEND_ROW = 12, 44, 22
SURFACE, INK, MUTED = '#fcfcfb', '#0b0b0b', '#52514e'
UNKNOWN, FREE, OCCUPIED, BOX = '#e4e3df', '#ffffff', '#2b2b2a', '#8a8983'
# fixed colour per planner (validated set), drawn in this order: NavFn A* is the wide line underneath, Smac 2D the
# middle one, NavFn Dijkstra the thin line on top. A planner that is not in the result is left out.
STYLE = {'astar': ('#eb6834', 6, 'NavFn A*'), 'smac2d': ('#1baf7a', 4, 'Smac 2D'),
         'dijkstra': ('#2a78d6', 2, 'NavFn Dijkstra')}


def _runs(row, value):
    """[(start, length)] of consecutive cells equal to ``value`` in one image row."""
    out, start = [], None
    for i, v in enumerate(row):
        if v == value and start is None:
            start = i
        elif v != value and start is not None:
            out.append((start, i - start))
            start = None
    if start is not None:
        out.append((start, len(row) - start))
    return out


def map_layers(map_yaml):
    """(width, height, free path data, occupied path data) in cell units, image rows from the top."""
    doc = core._load_yaml(map_yaml)
    width, height, maxval, pixels = core.read_pgm_p5(core.map_image_path(map_yaml))
    negate = int(doc.get('negate', 0))
    occ_t, free_t = float(doc.get('occupied_thresh', 0.65)), float(doc.get('free_thresh', 0.196))
    free, occupied = [], []
    for r in range(height):
        row = []
        for v in pixels[r * width:(r + 1) * width]:
            p = (v if negate else maxval - v) / maxval
            row.append(2 if p > occ_t else (1 if p < free_t else 0))
        free += [f'M{s} {r}h{n}v1h-{n}z' for s, n in _runs(row, 1)]
        occupied += [f'M{s} {r}h{n}v1h-{n}z' for s, n in _runs(row, 2)]
    return width, height, ''.join(free), ''.join(occupied)


def render_svg(run, info, layers, paths, stats, blocked):
    """SVG text of one scenario run. ``paths``/``stats``: {algorithm: path item / summary row}."""
    width, height, free, occupied = layers
    res, (ox, oy) = info['resolution'], info['origin']
    map_w, map_h = width * SCALE, height * SCALE
    style = {alg: v for alg, v in STYLE.items() if alg in stats or alg in paths}
    legend_h = LEGEND_ROW * (len(style) + 1) + PAD
    total_w, total_h = map_w + 2 * PAD, TITLE_H + map_h + legend_h + PAD

    def px(x, y):
        return (round(PAD + (x - ox) / res * SCALE, 1), round(TITLE_H + map_h - (y - oy) / res * SCALE, 1))

    title = f'{run.run_id} {run.name}'
    sub = '차단 지도: 상자를 점유 칸으로 넣음' if blocked else (
        '차단 전 지도 (점선: 상자가 놓일 자리)' if run.obstacles else '시나리오 지도')
    o = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {total_w} {total_h}" width="{total_w}" '
         f'height="{total_h}" font-family="sans-serif" role="img" aria-label="{escape(title)}">',
         f'<rect width="{total_w}" height="{total_h}" fill="{SURFACE}"/>',
         f'<text x="{PAD}" y="18" font-size="15" font-weight="600" fill="{INK}">{escape(title)}</text>',
         f'<text x="{PAD}" y="36" font-size="12" fill="{MUTED}">{escape(sub)} · 눈금 1 m · '
         f'길이는 계획 경로 길이(실제 주행 거리 아님)</text>',
         f'<g transform="translate({PAD} {TITLE_H}) scale({SCALE})">',
         f'<rect width="{width}" height="{height}" fill="{UNKNOWN}"/>',
         f'<path d="{free}" fill="{FREE}"/>', f'<path d="{occupied}" fill="{OCCUPIED}"/>', '</g>']
    # 1 m grid ticks along the left and bottom edges (recessive)
    x = int(ox) if ox <= int(ox) else int(ox) + 1
    while x <= ox + width * res:
        gx, gy = px(x, oy)
        o.append(f'<line x1="{gx}" y1="{gy}" x2="{gx}" y2="{gy - 5}" stroke="{MUTED}" stroke-width="1"/>')
        x += 1
    y = int(oy) if oy <= int(oy) else int(oy) + 1
    while y <= oy + height * res:
        gx, gy = px(ox, y)
        o.append(f'<line x1="{gx}" y1="{gy}" x2="{gx + 5}" y2="{gy}" stroke="{MUTED}" stroke-width="1"/>')
        y += 1
    for box in run.obstacles:
        cx, cy = px(box.cx, box.cy)
        w, h = box.size_x / res * SCALE, box.size_y / res * SCALE
        look = f'fill="{BOX}" stroke="{INK}" stroke-width="1"' if blocked else \
            f'fill="none" stroke="{INK}" stroke-width="1.2" stroke-dasharray="4 3"'
        o.append(f'<rect x="{round(cx - w / 2, 1)}" y="{round(cy - h / 2, 1)}" width="{round(w, 1)}" '
                 f'height="{round(h, 1)}" transform="rotate({-box.yaw_deg} {cx} {cy})" {look}/>')
    for alg, (color, stroke, label) in style.items():
        if alg in paths:
            pts = ' '.join('{},{}'.format(*px(x, y)) for x, y in paths[alg]['poses'])
            o.append(f'<polyline points="{pts}" fill="none" stroke="{color}" stroke-width="{stroke}" '
                     f'stroke-linejoin="round" stroke-linecap="round"><title>{label}</title></polyline>')
    sx, sy = px(run.start.x, run.start.y)
    gx, gy = px(run.goal.x, run.goal.y)
    o.append(f'<circle cx="{sx}" cy="{sy}" r="6" fill="{INK}" stroke="{SURFACE}" stroke-width="2"/>')
    o.append(f'<rect x="{gx - 6}" y="{gy - 6}" width="12" height="12" fill="{SURFACE}" stroke="{INK}" '
             f'stroke-width="2.5"/>')
    ly = TITLE_H + map_h + PAD + 6
    for alg, (color, stroke, label) in style.items():
        s = stats.get(alg, {})
        if alg in paths:
            text = (f"{label}: {float(s['planned_distance_mean']):.2f} m"
                    + (f" (0.10 m 간격 {float(s['resampled_distance_mean']):.2f} m)"
                       if s.get('resampled_distance_mean') else '')
                    + (f" · 최소 장애물 거리 {float(s['min_clearance_min']):.2f} m"
                       if s.get('min_clearance_min') else '')
                    + (f" · 꺾인 각도 합 {float(s['turn_sum_deg_mean']):.0f}°" if s.get('turn_sum_deg_mean') else ''))
        else:
            text = f"{label}: 경로 없음 ({s.get('failure_reasons') or '-'})"
        o.append(f'<line x1="{PAD}" y1="{ly}" x2="{PAD + 28}" y2="{ly}" stroke="{color}" stroke-width="{stroke}" '
                 f'stroke-linecap="round"/>')
        o.append(f'<text x="{PAD + 36}" y="{ly + 4}" font-size="12.5" fill="{INK}">{escape(text)}</text>')
        ly += LEGEND_ROW
    o.append(f'<circle cx="{PAD + 6}" cy="{ly}" r="5" fill="{INK}"/>')
    o.append(f'<text x="{PAD + 18}" y="{ly + 4}" font-size="12.5" fill="{MUTED}">출발</text>')
    o.append(f'<rect x="{PAD + 62}" y="{ly - 5}" width="10" height="10" fill="{SURFACE}" stroke="{INK}" '
             f'stroke-width="2"/>')
    note = '목표'
    if 'dijkstra' in paths and len(paths) > 1:      # how far each other path strays from the Dijkstra path
        gaps = [f"{style[alg][2]} {core.max_separation(paths[alg]['poses'], paths['dijkstra']['poses']):.2f} m"
                for alg in style if alg in paths and alg != 'dijkstra']
        note += ' · Dijkstra 경로와의 최대 간격: ' + ', '.join(gaps)
    o.append(f'<text x="{PAD + 80}" y="{ly + 4}" font-size="12.5" fill="{MUTED}">{escape(note)}</text>')
    o.append('</svg>')
    return '\n'.join(o) + '\n'


def plot_results(result_dir, bench_dir=BENCH_DIR):
    """Write paths_<run>.svg for every scenario run of a result folder; returns the written file names."""
    with open(os.path.join(result_dir, 'meta.json'), encoding='utf-8') as f:
        meta = json.load(f)
    with open(os.path.join(result_dir, 'paths.json'), encoding='utf-8') as f:
        items = json.load(f)['paths']
    with open(os.path.join(result_dir, 'summary.csv'), encoding='utf-8') as f:
        summary = list(csv.DictReader(f))
    scenario_path = os.path.join(bench_dir, meta['scenario_file'])
    if core.scenario_version(scenario_path) != meta['scenario_version']:
        raise ValueError(f"{scenario_path} is not the version of this result ({meta['scenario_version']})")
    scenarios = core.load_scenarios(scenario_path)
    info = core.map_info(scenarios.map_yaml)
    work = os.path.join(result_dir, '.plot_maps')
    layers = {'': map_layers(scenarios.map_yaml)}
    try:
        for variant, boxes in scenarios.map_variants().items():
            layers[variant] = map_layers(core.write_blocked_map(scenarios.map_yaml, boxes, work, f'map_{variant}'))
    finally:
        if os.path.isdir(work):
            for name in os.listdir(work):
                os.remove(os.path.join(work, name))
            os.rmdir(work)
    written = []
    for run in scenarios.runs:
        paths = {p['algorithm']: p for p in items if p['scenario_id'] == run.run_id}
        stats = {s['algorithm']: s for s in summary if s['scenario_id'] == run.run_id}
        name = 'paths_' + run.run_id.replace(':', '_') + '.svg'
        with open(os.path.join(result_dir, name), 'w', encoding='utf-8', newline='\n') as f:
            f.write(render_svg(run, info, layers[run.map_variant], paths, stats, bool(run.map_variant)))
        written.append(name)
    return written


if __name__ == '__main__':
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    for written_name in plot_results(sys.argv[1]):
        print(written_name)

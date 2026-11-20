"""Build a large logistics warehouse world (warehouse_large, 40 x 30 m) for research 1's large-map test.

    python3 tools/make_warehouse_world.py [-o worlds/warehouse_large.world]

Layout (world coordinates, interior x 0..40, y 0..30, origin = inner bottom-left corner):
- 3 rack zones along x: 3..11, 14..22, 25..33 (racks 8 m long, 1.0 m deep, 2.0 m high = solid lines to the LiDAR)
- rack rows every 2.5 m along y (aisles 1.5 m between racks); the row at y=15.5 is left out -> a 4 m main corridor
- cross aisles 3 m wide between zones (x 11..14, 22..25) with a building pillar (0.4 m) at y 9 and 21 in each
- perimeter aisle x 0..3 (west), dock area x 33..40 (east, 4 pallets), aisles south (y 0..2.5) and north (y 26..30)
- home / robot start = dock (37, 15.5)
Only boxes, so tools/world_to_map.py turns it into the exact matching map.
"""
import argparse
import os

W, H = 40.0, 30.0
WALL_T, WALL_H = 0.1, 0.5
ZONES = [(3.0, 11.0), (14.0, 22.0), (25.0, 33.0)]
RACK_D, RACK_H, PITCH, Y_FIRST, N_ROWS, SKIP_ROW = 1.0, 2.0, 2.5, 3.0, 10, 5
PILLARS = [(12.5, 9.0), (12.5, 21.0), (23.5, 9.0), (23.5, 21.0)]
PALLETS = [(34.5, 8.0), (39.0, 11.0), (34.5, 21.0), (39.0, 24.5)]   # 1.2 x 1.0 x 0.5 m, clear of the line x=36.5

GRAY = '<material><ambient>0.6 0.6 0.6 1</ambient><diffuse>0.7 0.7 0.7 1</diffuse></material>'
RACK = '<material><ambient>0.2 0.3 0.55 1</ambient><diffuse>0.25 0.35 0.65 1</diffuse></material>'
WOOD = '<material><ambient>0.45 0.3 0.15 1</ambient><diffuse>0.6 0.42 0.2 1</diffuse></material>'


def box_model(name, x, y, sx, sy, sz, material):
    geom = f'<box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box>'
    return f'''    <model name="{name}">
      <static>true</static>
      <pose>{x:.3f} {y:.3f} {sz / 2:.3f} 0 0 0</pose>
      <link name="link">
        <collision name="c"><geometry>{geom}</geometry></collision>
        <visual name="v"><geometry>{geom}</geometry>{material}</visual>
      </link>
    </model>
'''


def world_sdf(name):
    parts = [
        box_model('wall_s', W / 2, -WALL_T / 2, W + 2 * WALL_T, WALL_T, WALL_H, GRAY),
        box_model('wall_n', W / 2, H + WALL_T / 2, W + 2 * WALL_T, WALL_T, WALL_H, GRAY),
        box_model('wall_w', -WALL_T / 2, H / 2, WALL_T, H, WALL_H, GRAY),
        box_model('wall_e', W + WALL_T / 2, H / 2, WALL_T, H, WALL_H, GRAY),
    ]
    racks = 0
    for zi, (x0, x1) in enumerate(ZONES):
        for k in range(N_ROWS):
            if k == SKIP_ROW:
                continue
            parts.append(box_model(f'rack_{zi}_{k}', (x0 + x1) / 2, Y_FIRST + k * PITCH, x1 - x0, RACK_D, RACK_H, RACK))
            racks += 1
    for i, (x, y) in enumerate(PILLARS):
        parts.append(box_model(f'pillar_{i}', x, y, 0.4, 0.4, 3.0, GRAY))
    for i, (x, y) in enumerate(PALLETS):
        parts.append(box_model(f'pallet_{i}', x, y, 1.2, 1.0, 0.5, WOOD))
    body = ''.join(parts)
    return f'''<?xml version="1.0"?>
<!-- tools/make_warehouse_world.py 로 만든 월드: {name}
     물류 창고 40 x 30 m: 랙 구역 3개(x 3~11, 14~22, 25~33), 랙 {racks}개(길이 8 m, 깊이 1.0 m, 높이 2.0 m), 랙 사이 통로 1.5 m,
     y=15.5 주 통로 4 m, 구역 사이 통로 3 m (기둥 4개), 서쪽 통로 3 m, 동쪽 하역장 7 m (팔레트 4개). 홈 = (37, 15.5) -->
<sdf version="1.6">
  <world name="{name}">
    <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
      <ros><namespace>/</namespace></ros>
      <update_rate>20.0</update_rate>
    </plugin>
    <include><uri>model://ground_plane</uri></include>
    <include><uri>model://sun</uri></include>
    <scene>
      <shadows>false</shadows>
    </scene>
    <physics type="ode">
      <real_time_update_rate>1000.0</real_time_update_rate>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
    </physics>
{body}  </world>
</sdf>
''', racks


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('-o', '--output')
    ap.add_argument('--name', default='warehouse_large')
    args = ap.parse_args()
    out = args.output or os.path.join(here, '..', 'worlds', f'{args.name}.world')
    sdf, racks = world_sdf(args.name)
    with open(out, 'w', encoding='utf-8', newline='\n') as f:
        f.write(sdf)
    print(f'{os.path.normpath(out)}: 창고 {W:.0f} x {H:.0f} m, 랙 {racks}개, 기둥 {len(PILLARS)}개, 팔레트 {len(PALLETS)}개')


if __name__ == '__main__':
    main()

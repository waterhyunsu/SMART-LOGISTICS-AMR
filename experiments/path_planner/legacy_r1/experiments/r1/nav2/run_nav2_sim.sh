#!/bin/bash
# 연구 1 Nav2 실행: Gazebo (실물 사양 Waffle Pi, scripts/start_sim.sh) + Nav2 (map_server, AMCL, NavFn AStar/Dijkstra,
# DWB, BT navigator; experiments/r1/nav2/nav2_r1.launch.py). 프로젝트 브리지(A* + Frenet)는 띄우지 않는다.
#   experiments/r1/nav2/run_nav2_sim.sh [월드]        (기본 lecture_hall_real, 맵은 experiments/r1/scenarios.yaml)
#   Docker:  docker exec wsim /app/experiments/r1/nav2/run_nav2_sim.sh lecture_hall_real
# 로그: logs/sim.log, logs/nav2_r1.log. 끄기: scripts/stop_ros.sh
# 다음: source scripts/env.sh && python3 tools/r1_bench.py nav2plan|nav2sim --world <월드>
set -e
DIR="$(cd "$(dirname "$0")/../../.." && pwd)"
LOGS="$DIR/logs"
WORLD="${1:-lecture_hall_real}"
read -r MAP SPAWN_X SPAWN_Y < <(python3 - "$DIR/experiments/r1/scenarios.yaml" "$WORLD" <<'EOF'
import os, sys, yaml
cfg = yaml.safe_load(open(sys.argv[1], encoding='utf-8'))['worlds'].get(sys.argv[2])
if cfg is None:
    sys.exit(f'scenarios.yaml 에 월드 {sys.argv[2]} 가 없습니다')
m = cfg['map']
if m == 'turtlebot3_navigation2':
    m = '/opt/ros/humble/share/turtlebot3_navigation2/map/map.yaml'
elif not os.path.isabs(m):
    m = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(sys.argv[1])))), m)
x, y = cfg.get('spawn', [0.0, 0.0])
print(m, x, y)
EOF
)
[ -f "$MAP" ] || { echo "맵 없음: $MAP"; exit 1; }

mkdir -p "$LOGS"
"$DIR/scripts/stop_ros.sh" >/dev/null || true
cat > "$LOGS/run_mode.env" <<EOF
# experiments/r1/nav2/run_nav2_sim.sh 가 기록한 실행 모드 ($(date '+%F %T'))
WAREHOUSE_ROBOT=sim
WAREHOUSE_MODE=map
WAREHOUSE_NAV=nav2
WAREHOUSE_MAP=$MAP
WAREHOUSE_MAP_NAME=$(basename "$MAP" .yaml)
WAREHOUSE_WORLD=$WORLD
WAREHOUSE_SPAWN_X=$SPAWN_X
WAREHOUSE_SPAWN_Y=$SPAWN_Y
EOF
source "$DIR/scripts/env.sh"
ros2 daemon stop >/dev/null 2>&1 || true   # a stale CLI daemon answers '!rclpy.ok()'; the checks below use --no-daemon
echo "== 연구 1 Nav2: 월드 $WORLD, 맵 $MAP, 출발 ($SPAWN_X, $SPAWN_Y), 도메인 $ROS_DOMAIN_ID"

echo "1) Gazebo (logs/sim.log)"
setsid "$DIR/scripts/start_sim.sh" "$WORLD" --x "$SPAWN_X" --y "$SPAWN_Y" > "$LOGS/sim.log" 2>&1 < /dev/null &
for _ in $(seq 60); do
  timeout 3 ros2 topic echo --once /scan sensor_msgs/msg/LaserScan --field header.frame_id >/dev/null 2>&1 && break
  sleep 1
done

echo "2) Nav2 (logs/nav2_r1.log)"
setsid ros2 launch "$DIR/experiments/r1/nav2/nav2_r1.launch.py" map:="$MAP" use_sim_time:=true \
  > "$LOGS/nav2_r1.log" 2>&1 < /dev/null &
ok=0
for _ in $(seq 90); do
  if [ "$(timeout 10 ros2 lifecycle get --no-daemon /bt_navigator 2>/dev/null | cut -d' ' -f1)" = active ]; then ok=1; break; fi
  sleep 1
done
[ "$ok" = 1 ] || { echo "   Nav2 가 활성화되지 않았습니다 (logs/nav2_r1.log)"; tail -20 "$LOGS/nav2_r1.log"; exit 1; }
echo "   bt_navigator active"

echo "3) 초기 위치: 스캔 맞추기 → /initialpose"
python3 "$DIR/experiments/r1/nav2/set_initial_pose.py" "$SPAWN_X" "$SPAWN_Y" 0 --localize 2>&1 | grep -v -i warn | sed "s/^/   /"
echo "== 완료. 끄기: scripts/stop_ros.sh"

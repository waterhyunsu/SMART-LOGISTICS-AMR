#!/bin/bash
# 연구 1 Nav2 실차 실행 (기획안 5.6-5, D 담당과 함께): PC 에서 Nav2 (experiments/r1/nav2/nav2_r1.yaml) 를 띄운다.
# 로봇 쪽 bringup·명령 감시는 Pi 에서 돌아야 한다 (--pi 를 주면 scripts/run.sh --robot real 과 같은 방법으로 띄움).
#
#   experiments/r1/nav2/run_nav2_real.sh <월드> [X Y 방향°] [--pi user@host] [--no-watchdog]
#     월드      experiments/r1/scenarios.yaml 의 실차 항목 (맵 = 강의실에서 만든 SLAM 맵, 지점 = 실측 좌표)
#     X Y 방향  로봇을 놓은 위치 (기본: 그 월드의 spawn, 방향 0). 이 근처에서 스캔 맞추기로 초기 위치를 정한다
#     --pi      Pi 에 SSH 로 turtlebot3 bringup (없으면) + 명령 감시(scripts/pi_cmd_watchdog.py) 재시작
#     --no-watchdog  명령 감시 없이 Nav2 가 cmd_vel 로 바로 보냄 (권장하지 않음: 통신이 끊기면 로봇이 마지막 명령으로 계속 감)
# 로그: logs/nav2_r1.log. 끄기: scripts/stop_ros.sh (Pi bringup 은 건드리지 않음)
# 다음: source scripts/env.sh && python3 tools/r1_bench.py nav2sim --real --world <월드>
set -e
DIR="$(cd "$(dirname "$0")/../../.." && pwd)"
LOGS="$DIR/logs"
WORLD=""; PI_HOST=""; CMD_OUT=cmd_vel_pc; POSE=()
while [ $# -gt 0 ]; do
  case "$1" in
    --pi) PI_HOST="$2"; shift ;;
    --no-watchdog) CMD_OUT=cmd_vel ;;
    -h|--help) sed -n '2,14p' "$0"; exit 0 ;;
    *) if [ -z "$WORLD" ]; then WORLD="$1"; else POSE+=("$1"); fi ;;
  esac
  shift
done
[ -n "$WORLD" ] || { echo "월드를 지정하세요 (scenarios.yaml 의 실차 항목)"; exit 1; }
read -r MAP SPAWN_X SPAWN_Y < <(python3 - "$DIR/experiments/r1/scenarios.yaml" "$WORLD" <<'EOF'
import os, sys, yaml
cfg = yaml.safe_load(open(sys.argv[1], encoding='utf-8'))['worlds'].get(sys.argv[2])
if cfg is None:
    sys.exit(f'scenarios.yaml 에 월드 {sys.argv[2]} 가 없습니다')
m = cfg['map']
if not os.path.isabs(m):
    m = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(sys.argv[1])))), m)
x, y = cfg.get('spawn', [0.0, 0.0])
print(m, x, y)
EOF
)
[ -f "$MAP" ] || { echo "맵 없음: $MAP"; exit 1; }
X="${POSE[0]:-$SPAWN_X}"; Y="${POSE[1]:-$SPAWN_Y}"; YAW="${POSE[2]:-0}"

mkdir -p "$LOGS"
"$DIR/scripts/stop_ros.sh" >/dev/null || true
cat > "$LOGS/run_mode.env" <<EOF
# experiments/r1/nav2/run_nav2_real.sh 가 기록한 실행 모드 ($(date '+%F %T'))
WAREHOUSE_ROBOT=real
WAREHOUSE_MODE=map
WAREHOUSE_NAV=nav2
WAREHOUSE_MAP=$MAP
WAREHOUSE_MAP_NAME=$(basename "$MAP" .yaml)
WAREHOUSE_WORLD=$WORLD
EOF
source "$DIR/scripts/env.sh"
ros2 daemon stop >/dev/null 2>&1 || true
echo "== 연구 1 Nav2 실차: 월드 $WORLD, 맵 $MAP, 놓은 위치 ($X, $Y, ${YAW}°), 도메인 $ROS_DOMAIN_ID, 속도 출력 $CMD_OUT"

if [ -n "$PI_HOST" ]; then
  echo "1) Pi ($PI_HOST): bringup + 명령 감시"
  ssh -o BatchMode=yes -o ConnectTimeout=5 "$PI_HOST" true 2>/dev/null || { echo "   Pi 에 접속할 수 없습니다"; exit 1; }
  ssh -o BatchMode=yes "$PI_HOST" "pgrep -f '[t]urtlebot3_ros' >/dev/null && echo '   bringup 이미 실행 중' || {
    source /opt/ros/humble/setup.bash; source ~/turtlebot3_ws/install/setup.bash
    export ROS_DOMAIN_ID=$ROS_DOMAIN_ID LDS_MODEL=LDS-02 TURTLEBOT3_MODEL=waffle_pi
    setsid nohup ros2 launch turtlebot3_bringup robot.launch.py > ~/warehouse_bringup.log 2>&1 < /dev/null &
    sleep 12; grep -q 'Run!' ~/warehouse_bringup.log && echo '   bringup 시작됨' || echo '   bringup 확인 필요: Pi ~/warehouse_bringup.log'; }"
  if [ "$CMD_OUT" = cmd_vel_pc ]; then
    ssh -o BatchMode=yes "$PI_HOST" 'cat > ~/warehouse_cmd_watchdog.py' < "$DIR/scripts/pi_cmd_watchdog.py"
    ssh -o BatchMode=yes "$PI_HOST" 'pkill -f "[w]arehouse_cmd_watchdog.py"; sleep 0.5'
    ssh -o BatchMode=yes "$PI_HOST" "source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=$ROS_DOMAIN_ID
      setsid nohup python3 ~/warehouse_cmd_watchdog.py > ~/warehouse_cmd_watchdog.log 2>&1 < /dev/null &"
    sleep 3
  fi
else
  echo "1) Pi bringup·명령 감시: 이미 실행 중이어야 함 (--pi user@host 로 여기서 띄울 수 있음)"
fi

echo "2) 로봇 토픽 확인"
for t in scan odom; do
  timeout 10 ros2 topic echo --once "/$t" --no-daemon >/dev/null 2>&1 \
    || { echo "   /$t 가 오지 않습니다 (bringup, ROS_DOMAIN_ID=$ROS_DOMAIN_ID, 네트워크 확인)"; exit 1; }
done
echo "   /scan, /odom 수신"
if [ "$CMD_OUT" = cmd_vel_pc ] && ! python3 "$DIR/experiments/r1/nav2/set_initial_pose.py" --check-cmd >/dev/null 2>&1; then
  echo "   명령 감시(cmd_vel_pc 구독자)가 없습니다: Pi 에서 pi_cmd_watchdog 을 띄우거나 --pi 를 주세요"; exit 1
fi

echo "3) Nav2 (logs/nav2_r1.log)"
setsid ros2 launch "$DIR/experiments/r1/nav2/nav2_r1.launch.py" map:="$MAP" use_sim_time:=false cmd_vel_out:="$CMD_OUT" \
  > "$LOGS/nav2_r1.log" 2>&1 < /dev/null &
ok=0
for _ in $(seq 90); do
  if [ "$(timeout 10 ros2 lifecycle get --no-daemon /bt_navigator 2>/dev/null | cut -d' ' -f1)" = active ]; then ok=1; break; fi
  sleep 1
done
[ "$ok" = 1 ] || { echo "   Nav2 가 활성화되지 않았습니다 (logs/nav2_r1.log)"; tail -20 "$LOGS/nav2_r1.log"; exit 1; }
echo "   bt_navigator active"

echo "4) 초기 위치: 스캔 맞추기 → /initialpose"
python3 "$DIR/experiments/r1/nav2/set_initial_pose.py" "$X" "$Y" "$YAW" --localize 2>&1 | grep -v -i warn | sed "s/^/   /"
echo "== 완료. RViz 로 위치가 맞는지 확인한 뒤 실험. 끄기: scripts/stop_ros.sh"

#!/bin/bash
# 연구 1 Gazebo 실행: experiments/r1/scenarios.yaml 에 적힌 그 월드의 맵·AMCL 설정으로 scripts/run.sh 를 실행한다.
#   experiments/r1/run_sim.sh [월드] [run.sh 에 더 넘길 옵션...]     (기본 월드 lecture_hall_real)
#   Docker:  docker exec wsim /app/experiments/r1/run_sim.sh lecture_hall_real
# 끝나면 같은 셸에서: source scripts/env.sh && python3 tools/r1_bench.py sim --world <월드>
set -e
DIR="$(cd "$(dirname "$0")/../.." && pwd)"
WORLD="${1:-lecture_hall_real}"
[ $# -gt 0 ] && shift
read -r MAP AMCL < <(python3 - "$DIR/experiments/r1/scenarios.yaml" "$WORLD" <<'EOF'
import sys, yaml
cfg = yaml.safe_load(open(sys.argv[1], encoding='utf-8'))['worlds'].get(sys.argv[2])
if cfg is None:
    sys.exit(f'scenarios.yaml 에 월드 {sys.argv[2]} 가 없습니다')
print(cfg['map'], cfg.get('amcl', 'default'))
EOF
)
if [ "$MAP" = turtlebot3_navigation2 ]; then
  exec "$DIR/scripts/run.sh" --robot sim --mode map --world "$WORLD" --amcl "$AMCL" "$@"
fi
exec "$DIR/scripts/run.sh" --robot sim --mode map --map "$DIR/$MAP" --amcl "$AMCL" "$@"

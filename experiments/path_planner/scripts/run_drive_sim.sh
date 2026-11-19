#!/usr/bin/env bash
# Driving comparison in Gazebo: start the world + robot + Nav2, run run_drive.py, stop everything.
#
#   experiments/path_planner/scripts/run_drive_sim.sh <scenario.yaml> --run-mode SIM [run_drive.py options]
#
# Needs ROS 2 Jazzy with Gazebo Harmonic, Nav2 and turtlebot3_gazebo (docker/Dockerfile.sim) and a display or GPU
# for the simulated LiDAR. SIM_GUI=1 also opens the Gazebo window. The world is sim/worlds/<map name>.sdf.
# [초안-팀합의필요] B's temporary driving environment, not D's deployment setup.
set -euo pipefail

BENCH="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SCENARIO="${1:?usage: run_drive_sim.sh <scenario.yaml> --run-mode SIM [run_drive.py options]}"
shift
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}"
export TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-waffle_pi}"

read -r MAP MAP_NAME < <(python3 -c 'import sys; sys.path.insert(0, sys.argv[1]); import bench_core as c
s = c.load_scenarios(sys.argv[2]); print(s.map_yaml, s.map_name)' "$BENCH/scripts" "$SCENARIO")
WORLD="$BENCH/sim/worlds/$MAP_NAME.sdf"
[ -f "$WORLD" ] || { echo "no world for map $MAP_NAME: $WORLD" >&2; exit 2; }
LOG="${DRIVE_LAUNCH_LOG:-$(mktemp -d)/drive_bench_launch.log}"
echo "world: $WORLD"
echo "map: $MAP"
echo "launch log: $LOG"

GUI=false
[ "${SIM_GUI:-0}" = 1 ] && GUI=true
ros2 launch "$BENCH/launch/drive_bench.launch.py" world:="$WORLD" map:="$MAP" gui:="$GUI" >"$LOG" 2>&1 &
LAUNCH_PID=$!
cleanup() {
    kill -TERM "$LAUNCH_PID" 2>/dev/null || true
    for _ in $(seq 1 30); do
        kill -0 "$LAUNCH_PID" 2>/dev/null || break
        sleep 0.5
    done
    kill -KILL "$LAUNCH_PID" 2>/dev/null || true
    pkill -KILL -f "gz sim" 2>/dev/null || true
    wait "$LAUNCH_PID" 2>/dev/null || true
}
trap cleanup EXIT

if ! python3 "$BENCH/scripts/run_drive.py" --scenarios "$SCENARIO" --world "$MAP_NAME" "$@"; then
    echo "run_drive.py failed; last launch log lines:" >&2
    tail -n 40 "$LOG" >&2
    exit 1
fi

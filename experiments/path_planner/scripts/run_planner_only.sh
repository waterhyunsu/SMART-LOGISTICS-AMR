#!/usr/bin/env bash
# Planner-only run: start map_server + planner_server on the scenario map, run the comparison, stop the stack.
#
#   experiments/path_planner/scripts/run_planner_only.sh <scenario.yaml> --run-mode SIM [run_compare.py options]
#
# Needs a sourced ROS 2 Jazzy + Nav2 environment (native Ubuntu 24.04 or the image in ../docker).
# DDS discovery is limited to this host unless ROS_AUTOMATIC_DISCOVERY_RANGE is already set, so the bench never
# talks to another PC's or the robot's planner_server.
set -euo pipefail

BENCH="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SCENARIO="${1:?usage: run_planner_only.sh <scenario.yaml> --run-mode SIM [run_compare.py options]}"
shift
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}"

MAP="$(python3 -c 'import sys; sys.path.insert(0, sys.argv[1]); import bench_core as c; print(c.load_scenarios(sys.argv[2]).map_yaml)' \
    "$BENCH/scripts" "$SCENARIO")"
LOG="$(mktemp -d)/planner_bench_launch.log"
echo "map: $MAP"
echo "launch log: $LOG"

# the launch file must load the same params file run_compare.py checks against (--params, default navfn_compare)
PARAMS="$BENCH/config/navfn_compare.yaml"
ARGS=("$@")
for i in "${!ARGS[@]}"; do
    [ "${ARGS[$i]}" = "--params" ] && PARAMS="$(realpath "${ARGS[$((i + 1))]}")"
done
echo "params: $PARAMS"
ros2 launch "$BENCH/launch/planner_bench.launch.py" map:="$MAP" params_file:="$PARAMS" >"$LOG" 2>&1 &
LAUNCH_PID=$!
cleanup() {
    # SIGINT is ignored by background jobs of a non-interactive shell, so stop ros2 launch with SIGTERM.
    kill -TERM "$LAUNCH_PID" 2>/dev/null || true
    for _ in $(seq 1 20); do
        kill -0 "$LAUNCH_PID" 2>/dev/null || break
        sleep 0.5
    done
    kill -KILL "$LAUNCH_PID" 2>/dev/null || true
    wait "$LAUNCH_PID" 2>/dev/null || true
}
trap cleanup EXIT

if ! python3 "$BENCH/scripts/run_compare.py" --scenarios "$SCENARIO" "$@"; then
    echo "run_compare.py failed; last launch log lines:" >&2
    tail -n 30 "$LOG" >&2
    exit 1
fi

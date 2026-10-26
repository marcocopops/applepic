#!/usr/bin/env bash
# M6 check: headless sim + detector + gripper + mission for LOOPS loop(s) from the depot.
# Logs the trajectory to artifacts/mission_trajectory.csv (plotted to .png) and metrics to
# artifacts/mission_metrics.json. Usage: scripts/check_mission.sh [min_ratio [timeout [capacity]]]
set -o pipefail
cd "$(dirname "$0")/.."
source /opt/ros/jazzy/setup.bash && source install/setup.bash
MIN=${1:-0.8}; TIMEOUT=${2:-1800}; CAP=${3:-10}
mkdir -p artifacts
LOG=artifacts/check_mission.log
# Job control: the launch gets its own process group, stopped by PGID (never by name).
set -m
ros2 launch applepic_bringup mission.launch.py headless:=true capacity:="$CAP" loops:=1 \
  trajectory_csv:="$PWD/artifacts/mission_trajectory.csv" > "$LOG" 2>&1 &
LAUNCH=$!
cleanup() {
  kill -INT -- -"$LAUNCH" 2>/dev/null
  for _ in $(seq 10); do kill -0 -- -"$LAUNCH" 2>/dev/null || return 0; sleep 1; done
  kill -KILL -- -"$LAUNCH" 2>/dev/null || true
}
trap cleanup EXIT
python3 scripts/eval_mission.py --min-ratio "$MIN" --timeout "$TIMEOUT"
status=$?
cleanup  # stop the nodes first so the trajectory CSV is flushed and closed
python3 scripts/plot_trajectory.py artifacts/mission_trajectory.csv artifacts/mission_trajectory.png \
  src/applepic_gazebo/config/orchard.yaml "ApplePic mission (1 loop)" \
  || true
echo "--- mission transitions ---"
grep -E "\[mission.*(->|done|unloaded)" "$LOG" | sed 's/^/  /' | tail -60
exit $status

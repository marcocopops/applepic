#!/usr/bin/env bash
# M3 check: headless sim + waypoint follower for one orchard loop (6 trees, back to depot).
# Logs the /odom trajectory to artifacts/nav_trajectory.csv, plots artifacts/nav_trajectory.png
# and checks every waypoint was reached. Usage: scripts/check_navigation.sh [timeout_s]
set -o pipefail
cd "$(dirname "$0")/.."
source /opt/ros/jazzy/setup.bash && source install/setup.bash
TIMEOUT=${1:-400}
mkdir -p artifacts
set -m  # background jobs get their own process group: stop by PGID, never by name
ros2 launch applepic_bringup sim.launch.py headless:=true > artifacts/check_nav_sim.log 2>&1 &
LAUNCH=$!
cleanup() {
  kill -INT -- -"$LAUNCH" 2>/dev/null
  for _ in $(seq 10); do kill -0 -- -"$LAUNCH" 2>/dev/null || return 0; sleep 1; done
  kill -KILL -- -"$LAUNCH" 2>/dev/null || true
}
trap cleanup EXIT
sleep 12
start=$(date +%s)
timeout "$TIMEOUT" ros2 run applepic_navigation waypoint_follower --ros-args \
  -p use_sim_time:=true -p loops:=1 -p trajectory_csv:="$PWD/artifacts/nav_trajectory.csv" \
  2>&1 | tee artifacts/check_nav_follower.log
code=$?
echo "follower exit code $code after $(( $(date +%s) - start )) s (wall)"
python3 scripts/plot_trajectory.py artifacts/nav_trajectory.csv artifacts/nav_trajectory.png || exit 1
grep -q 'done: 1 loop' artifacts/check_nav_follower.log

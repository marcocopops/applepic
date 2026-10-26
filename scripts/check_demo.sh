#!/usr/bin/env bash
# M8 check: demo launch in headless mode without RViz (sim + mission + mission_viz).
# Validates the RViz-facing topics (orchard markers, HUD, trajectory) while the mission runs.
# Usage: scripts/check_demo.sh [timeout [rtf]]
set -o pipefail
cd "$(dirname "$0")/.."
source /opt/ros/jazzy/setup.bash && source install/setup.bash
TIMEOUT=${1:-240}; RTF=${2:-2}
mkdir -p artifacts
LOG=artifacts/check_demo.log
# Job control: the launch gets its own process group, stopped by PGID (never by name).
set -m
ros2 launch applepic_bringup demo.launch.py headless:=true rviz:=false rtf:="$RTF" > "$LOG" 2>&1 &
LAUNCH=$!
cleanup() {
  kill -INT -- -"$LAUNCH" 2>/dev/null
  for _ in $(seq 10); do kill -0 -- -"$LAUNCH" 2>/dev/null || return 0; sleep 1; done
  kill -KILL -- -"$LAUNCH" 2>/dev/null || true
}
trap cleanup EXIT
python3 scripts/eval_demo.py --timeout "$TIMEOUT"
status=$?
echo "--- launch errors ---"
grep -E "ERROR|Traceback|process has died" "$LOG" | sed 's/^/  /' | tail -20
grep -E "real-time factor" "$LOG" | sed 's/^/  /'
exit $status

#!/usr/bin/env bash
# M5 check: headless sim + detector + gripper + picker, robot spawned near a tree.
# Waits until the basket holds at least MIN apples (or TIMEOUT s of wall time), then checks that
# the picked apple models are gone from gz and that /basket/count matches.
# Usage: scripts/check_picking.sh [x y yaw [min_picked [timeout [capacity]]]]
set -o pipefail
cd "$(dirname "$0")/.."
source /opt/ros/jazzy/setup.bash && source install/setup.bash
X=${1:-2.6}; Y=${2:-4.0}; YAW=${3:-0.0}; MIN=${4:-2}; TIMEOUT=${5:-120}; CAP=${6:-10}
mkdir -p artifacts
LOG=artifacts/check_picking.log
# Job control: the launch gets its own process group, stopped by PGID (never by name).
set -m
ros2 launch applepic_bringup picking.launch.py headless:=true x:="$X" y:="$Y" yaw:="$YAW" \
  capacity:="$CAP" > "$LOG" 2>&1 &
LAUNCH=$!
cleanup() {
  kill -INT -- -"$LAUNCH" 2>/dev/null
  for _ in $(seq 10); do kill -0 -- -"$LAUNCH" 2>/dev/null || return 0; sleep 1; done
  kill -KILL -- -"$LAUNCH" 2>/dev/null || true
}
trap cleanup EXIT
python3 scripts/eval_picking.py --min-picked "$MIN" --timeout "$TIMEOUT"
status=$?
echo "--- gripper / picker log ---"
grep -E "\[(gripper|apple_picker)" "$LOG" | sed 's/^/  /' | tail -30
exit $status

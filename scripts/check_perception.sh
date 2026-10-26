#!/usr/bin/env bash
# M4 check: headless sim + apple_detector, robot spawned at a fixed pose; compares
# /apples/detections with the ground-truth manifest and saves the annotated camera image.
# Usage: scripts/check_perception.sh [x y yaw [name [min_matched]]]
set -o pipefail
cd "$(dirname "$0")/.."
source /opt/ros/jazzy/setup.bash && source install/setup.bash
X=${1:-2.6}; Y=${2:-4.0}; YAW=${3:-0.0}; NAME=${4:-tree0}; MIN=${5:-2}
mkdir -p artifacts
# Job control: the launch gets its own process group, stopped by PGID (never by name).
set -m
ros2 launch applepic_bringup perception.launch.py headless:=true x:="$X" y:="$Y" yaw:="$YAW" \
  > "artifacts/check_perception_${NAME}.log" 2>&1 &
LAUNCH=$!
cleanup() {
  kill -INT -- -"$LAUNCH" 2>/dev/null
  for _ in $(seq 10); do kill -0 -- -"$LAUNCH" 2>/dev/null || return 0; sleep 1; done
  kill -KILL -- -"$LAUNCH" 2>/dev/null || true
}
trap cleanup EXIT
sleep 15
python3 scripts/eval_detections.py --seconds 5 --min-matched "$MIN" \
  --out "artifacts/perception_${NAME}.png"

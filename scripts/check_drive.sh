#!/usr/bin/env bash
# M2 smoke test: launch the headless sim, drive 3 s forward at 0.5 m/s, check /odom moved,
# save camera frames to artifacts/. Usage: scripts/check_drive.sh [x y yaw]
set -o pipefail
cd "$(dirname "$0")/.."
source /opt/ros/jazzy/setup.bash && source install/setup.bash
X=${1:-0.0}; Y=${2:-0.0}; YAW=${3:-0.0}
mkdir -p artifacts
# Job control: the background launch gets its own process group (PGID == $LAUNCH), so the
# whole tree (ros2 launch, gz sim server, bridge...) is stopped by PGID, never by name pattern.
set -m
ros2 launch applepic_bringup sim.launch.py headless:=true x:="$X" y:="$Y" yaw:="$YAW" \
  > artifacts/check_drive_sim.log 2>&1 &
LAUNCH=$!
cleanup() {
  kill -INT -- -"$LAUNCH" 2>/dev/null
  for _ in $(seq 10); do kill -0 -- -"$LAUNCH" 2>/dev/null || return 0; sleep 1; done
  kill -KILL -- -"$LAUNCH" 2>/dev/null || true
}
trap cleanup EXIT
sleep 15
python3 scripts/capture_camera.py artifacts/robot_camera_start.png
x0=$(ros2 topic echo --once /odom --field pose.pose.position.x | head -1)
timeout 3 ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}" > /dev/null
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}" > /dev/null
sleep 2
x1=$(ros2 topic echo --once /odom --field pose.pose.position.x | head -1)
python3 scripts/capture_camera.py artifacts/robot_camera_end.png
echo "odom x: $x0 -> $x1"
python3 -c "import sys; d=float('$x1')-float('$x0'); print(f'moved {d:.2f} m'); sys.exit(0 if d > 0.5 else 1)"

# ApplePic — context for Claude sessions

Simulated proof of concept of an apple-picking robot (differential base ~60x45 cm, camera, basket,
"magic" gripper until M20 replaces it with a MoveIt arm and a real two-finger gripper).
Roadmap and full instructions: `docs/ai-workflow/KICKOFF.md`. Memory between sessions: `docs/ai-workflow/progress.md`
(state + next milestone), `docs/ai-workflow/blockers.md`, `docs/decisions.md`, `docs/roadmap.md` (done / next), git log.
Audit: `docs/ai-workflow/audit.md`. Architecture overview: `docs/architecture.md`.

## Environment
- Ubuntu 24.04, ROS 2 **Jazzy** + **gz Harmonic (gz sim 8)** installed natively in `/opt/ros/jazzy`.
- Native workflow by default (optional Docker, never verified: `make docker-image`, then `USE_DOCKER=1 make ...`).
- NVIDIA GPU available: headless camera rendering (`gz sim -s --headless-rendering`) works.
- Robot topics: `/cmd_vel`, `/odom` (ground truth, world frame), `/wheel_odom`, `/tf`, `/joint_states`,
  `/camera/image_raw`, `/camera/camera_info`, `/clock`, `/imu`, `/gps/raw` (perfect fix), `/gps/fix` (noisy, gps mode).
- Bumper: `/bumper` (gz contacts of chassis + strip; `<topic>` goes INSIDE `<contact>`, the Contact plugin in the world).
- Collisions: `collide_bitmask` 0x01 wheels/caster, 0x02 apples (the robot drives over them without moving them).
- Trunks: `/trees/trunks` (robot frame, `apple_detector`), `/localization/trunk_fix` (`localizer`).
- Pose for all nodes: `/localization/odom` (`map` frame) + TF `map → odom → base_footprint`, published by
  `localizer` (`localization:=truth`: truth + identity; `gps`: EKF on GPS + wheels + gyro).
  Only the magic gripper reads `/odom` (ground truth).
- Not installed: MoveIt planners and config tools, ros2_controllers, gz_ros2_control (see `docs/ai-workflow/human-actions.md`, needed from M14);
  Nav2 and robot_localization (out of scope).

## Commands
- `make build`: `colcon build --symlink-install`
- `make test`: build + `colcon test` + results
- `make world`: regenerate the orchard SDF world from the YAML config
- `make sim-headless`: run the headless sim for `SIM_SECONDS` (default 20)
- `scripts/check_drive.sh [x y yaw]`: driving smoke test (cmd_vel → odom) + camera captures
- `scripts/check_navigation.sh [timeout]`: full headless loop (waypoint_follower) + `artifacts/nav_trajectory.png`
- `scripts/check_perception.sh [x y yaw name min]`: headless HSV detection compared to the manifest + `artifacts/perception_<name>.png`
- `ros2 launch applepic_bringup perception.launch.py x:=2.6 y:=4.0`: sim + `apple_detector` (`/apples/detections`, `/apples/debug_image`)
- `scripts/check_picking.sh [x y yaw min timeout capacity]`: headless picking (detector + gripper + apple_picker), cross-checks `/basket/*` and `gz model --list`
- `ros2 launch applepic_bringup picking.launch.py x:=9.6 y:=9.5 yaw:=1.57 [capacity:=10]`: sim + detection + picking
- `make e2e [E2E_SEED=n]` / `scripts/e2e_mission.sh [min_ratio sim_limit rtf capacity wall_timeout seed]`: M7 headless end-to-end
  scenario (1 loop, RTF 3 by default, ~5 min), PASS/FAIL, metrics in `artifacts/e2e_metrics.json` (including final poses of
  apples left on the ground), plot `artifacts/e2e_trajectory.png`; with a seed, world regenerated in `artifacts/orchard_seed<n>.sdf`.
  Diagnosis: `grep -E "gave up|left tree" artifacts/e2e.log`
- gz-transport Python: `GzNode.request` holds the GIL while waiting (freezes the whole node) → for a gz service,
  run `gz service` as a subprocess (see `gripper.py`)
- GPS localization: `make e2e|demo LOCALIZATION=gps [GPS_NOISE=0.02 GPS_BIAS=0.0]`, or
  `localization:=gps gps_noise:=… gps_bias:=… gps_bias_tau:=… gps_seed:=…` on all launch files;
  `artifacts/e2e_metrics.json` → key `localization` (error vs truth), truth plotted in orange.
- Hard orchard (grass, orange/dark/bruised apples): `make e2e ORCHARD_CONFIG=src/applepic_gazebo/config/orchard_hard.yaml`.
- Orchard with unknown obstacles: `make e2e ORCHARD_CONFIG=src/applepic_gazebo/config/orchard_obstacles.yaml`.
  Run a series in the background without `setsid` (it returns at once: false end notification).
- `scripts/eval_trunks.py [--output f]` next to a sim: error of `/trees/trunks` against truth (stop with SIGTERM).
  In a background e2e series, avoid an unguarded `wait` on a helper script (a loop has already hung on it).
- `scripts/log_bumps.py [--output f]` next to a sim: what the bumper touches, where around the chassis (truth, diagnosis).
- `make demo [DEMO_RTF=2]` / `ros2 launch applepic_bringup demo.launch.py [rtf:=2 rviz:=false headless:=true]`:
  M8 demo (Gazebo GUI + RViz, `mission_viz` node: `/mission_viz/markers|path`)
- `scripts/check_demo.sh [timeout rtf]`: headless demo without RViz, checks markers/HUD/trajectory.
  **Today it stops after ~14 simulated seconds: it proves little until M10.**
- Capture a window (no Xvfb/ImageMagick): `DISPLAY=:1 xwd -id <win>` (id from `xwininfo -root -tree`), then decode XWD
  in Python (PIL cannot read XWD). The decoder used so far produced striped images: not usable as is.
- `ros2 launch applepic_bringup mission.launch.py [headless:=false capacity:=10 loops:=1]`: sim + detection + mission
- `ros2 launch applepic_bringup navigation.launch.py loops:=1 [headless:=false]`: sim + loop following
- `scripts/capture_camera.py out.png`: save an image from `/camera/image_raw` (sim running)
- Stop a sim running in the background: by PGID (`set -m` then `kill -INT -- -$PID`), never by pattern.
  `set -m` must run BEFORE the `... &` line (not inside the backgrounded chain), otherwise `$!` is not the PGID.
- Manual shell: `source /opt/ros/jazzy/setup.bash && source install/setup.bash`
- Autonomous loop: `docs/ai-workflow/run_applepic.sh [HH:MM]` (works from any folder; `logs/` and `STOP` stay at the
  repo root). It refuses to start while `STOP` exists; it stops on `STOP`, at the deadline, or after two sessions
  without a commit. Logs are `stream-json` in `logs/`.

## Layout
- `src/applepic_description`: robot URDF/xacro
- `src/applepic_gazebo`: world generator (orchard), SDF worlds, ros_gz bridge
- `src/applepic_navigation`: `controller.py` (point following), `route.py` (tree loop, A* around bump points), `waypoint_follower` node,
  `gps.py` (ENU ↔ geodetic, GPS error model), `localization.py` (map → odom EKF, dead reckoning, `map_shift`), `gps_noise` and `localizer` nodes
- `src/applepic_perception`: `detector.py` (HSV), `trunk.py` (trunk foot), `projection.py` (pixel → ground), `apple_detector` node
- `src/applepic_mission`: `basket.py`, `picking.py`, `apple_tracker.py` (detection fusion), `approach.py` (gripper → point), nodes `gripper`
  (magic picking: `/basket/count|full|picked|picked_point`, srv `/basket/unload`), `apple_picker` (M5 demo), `coverage.py` (camera ground
  footprint + per-tree coverage grid), `mission.py` (pure state machine) and `mission` node (`/mission/state|status|stats`),
  `viz.py` + `mission_viz` node (demo RViz markers)
- `src/applepic_bringup`: top-level launch files
- `docs/`: `architecture.md`, `decisions.md`, `roadmap.md`, `media/` (README images); `docs/ai-workflow/`: autonomous-loop
  files (KICKOFF, runner, progress, blockers, human actions, audit)
All packages are `ament_python`. Pure logic (no rclpy) lives in separate modules testable by pytest with no ROS running.

## Conventions
- Everything in the repo is in English: code, names, comments, docs, README, commits.
- Small, frequent conventional commits; never rewrite history; never push.
- No sudo. Human actions → `docs/ai-workflow/human-actions.md`.
- Navigation and coarse-GPS tuning are frozen: no work on them outside a roadmap milestone.

## Verify
- `make test` must pass (0 failures).
- Sim: `make sim-headless`, then in another shell `gz topic -l` / `ros2 topic list`.
- Camera captures: saved as PNG in `artifacts/` (ignored by git).
- Before claiming a gain, compare several runs: results are not deterministic.

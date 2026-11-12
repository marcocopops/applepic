# Decisions

Design decisions, one per entry: context, decision, rejected options and known limits.
New entries are appended by the autonomous sessions (see [`ai-workflow/`](ai-workflow/)).

| # | Topic | Decision in one line |
|---|---|---|
| D1 | Environment | Native ROS 2 Jazzy + gz Harmonic; Docker optional |
| D2 | Packages | All `ament_python`, pure logic separated from nodes |
| D3 | World | Orchard SDF generated from YAML, apple manifest as ground truth |
| D4 | Robot | Ground-truth `/odom`, wheel odometry apart, tilted camera |
| D5 | Navigation | Home-made point controller and tree loop (no Nav2) |
| D6 | Perception | HSV segmentation + projection on the flat ground |
| D7 | Picking | Magic gripper: apple removed from gz within 7 cm (ground truth) |
| D8 | Mission | Pure state machine, search circle around each tree |
| D9 | Testing | Headless end-to-end scenario at 3x real time, scored against gz |
| D10 | Demo | Dedicated launch file + RViz marker node |
| D11 | Search | Driven by a camera coverage grid (888 s → 656 s) |
| D12 | Robustness | Full coverage, raised chassis, no pivot over an apple |
| D13 | Localization | Simulated GPS + home-made EKF on `map → odom`, gyro for heading |
| D14 | Localization | Apples anchored to the odometry frame |
| D15 | Localization | Pose corrected on tree trunks seen by the camera |
| D16 | Perception | Separate hard orchard, grass only visual |
| D17 | Safety | Contact bumper and backing off |
| D18 | Obstacles | Unknown obstacles remembered on contact |
| D19 | Physics | Wheels and caster do not collide with apples |
| D20 | Obstacles | A* around bump points, unreachable tree put off |
| D21 | Scope | Real arm + gripper through MoveIt 2; navigation frozen |

## D1 — Native environment rather than Docker (session 1)
- Context: Docker works without sudo, but ROS 2 Jazzy + gz Harmonic + ros_gz are already installed natively; only ~13 GB of disk is free; Docker has no NVIDIA runtime, so camera rendering in a container would be software (slow, fragile). Native headless GPU rendering works (verified).
- Decision: native workflow by default through the `Makefile`. A light `docker/Dockerfile` (ros-base + ros_gz) stays available (`USE_DOCKER=1`), not used for verification.
- Rejected: `osrf/ros:jazzy-desktop-full` (disk size, no GPU).

## D2 — All packages `ament_python`
- Context: nothing to compile in C++ for the POC.
- Decision: `ament_python` everywhere, pure logic separated from rclpy nodes so that pytest can test it.
- Rejected: ament_cmake for description/gazebo (useless, data_files is enough).

## D3 — Orchard world generated and versioned (session 2)
- Context: M1 asks for a configurable, reproducible world.
- Decision: `applepic_gazebo/orchard.py` (pure Python) generates `worlds/orchard.sdf` + a ground-truth manifest `worlds/orchard_apples.yaml` from `config/orchard.yaml` (fixed seed). Both generated files are committed (`make world` regenerates them). 6 trees in a loop, 4 to 7 apples per tree in a 0.45–1.6 m ring, blue depot zone (visual only, no collision) at (0,0).
- Apples are dynamic spheres (r = 4 cm, 150 g): stable at rest on flat ground (verified), the robot can push them. World plugins already included: UserCommands (entity removal for magic picking), Sensors ogre2, Imu, NavSat + `spherical_coordinates` (for the M9 GPS).
- Rejected: static apples (unrealistic if touched), Fuel models (network download, useless for HSV), textured grass (plain green is enough for segmentation).

## D4 — Robot and odometry (session 3)
- Context: M2, navigation in world coordinates first.
- Decision: `/odom` = gz OdometryPublisher (ground-truth pose, `odom` frame ≡ world, verified with an off-origin spawn); DiffDrive wheel odometry published separately on `/wheel_odom` (TF not bridged) for later localization. Camera at 0.50 m, tilt 0.85 rad (~49°), HFOV 1.3 rad: sees the ground about 0.2 to 2.5 m ahead; the gripper shows at the bottom of the image. Gripper = a plain `gripper_link` frame (M5 magic picking uses its distance to apples).
- Rejected: gz_ros2_control (useless for this POC at the time), physical basket with collisions.

## D5 — Home-made navigation controller (session 4)
- Context: M3, Nav2 not installed, flat world with trunks as only obstacles, ground-truth pose.
- Decision: `applepic_navigation/controller.py` (rotate in place if heading error > 0.6 rad, then forward speed proportional to distance, capped at 0.5 m/s; 0.15 m tolerance) + `route.py` building the loop: one approach point per tree, 2.2 m from the trunk **on the inside of the loop** (towards the centroid, outside the apple ring ≤ 1.6 m), then back to the depot; any segment passing within 2.0 m of a trunk gets a detour point (`via`). The `waypoint_follower` node reads trees from `applepic_gazebo/config/orchard.yaml`.
- Rejected: Nav2 (not installed, heavy for 6 points on flat ground), pure pursuit (useless at this scale), occupancy map.
- Consequence for M6: the mission reuses `compute_command` to reach apples and search points; `waypoint_follower` serves as the M3 demo.

## D6 — HSV perception + plane projection (session 5)
- Context: M4, plain red apples on green grass, flat ground, camera pose known through TF (ground truth).
- Decision: HSV segmentation (two hue bands around red, S/V thresholds excluding the brown trunk, hue ≈ 18) + blob centroids; centroid projected onto the plane z = apple radius (0.04 m), approximating the sphere centre (measured error ≤ 1.5 cm). Blobs touching the image border are ignored (biased centroid), 4 m max range. Output as `geometry_msgs/PoseArray` (frame `odom`), without identity or tracking: temporal fusion belongs to M5/M6.
- Rejected: circle detection (Hough, fragile with shading), distance from apparent radius (less precise than flat-ground projection), custom message (PoseArray is enough and avoids an ament_cmake interface package).

## D7 — Magic picking by entity removal, ground-truth apple poses (session 6)
- Context: M5, "magic" picking allowed; the robot must know when the gripper touches an apple.
- Decision: `gripper` node comparing the gripper position (`/odom` + 0.46 m along the axis) with **ground-truth** apple poses from the gz topic `/world/orchard/pose/info`; under 7 cm (horizontal) it removes the model through the gz service `/world/orchard/remove` (Python bindings `gz.transport13` / `gz.msgs10` shipped with Harmonic, no bridge) and increments the basket. Automatic pick whenever an apple is in reach (even in transit), refused when full; `/basket/unload` (std_srvs/Trigger) empties the basket, called by the mission. The gripper sticks out 6 cm ahead of the chassis: a centred apple is picked before being touched.
- Decision side: `apple_picker` fuses detections (`AppleTracker`) and approaches with `approach.py` (robot facing the apple at 0.46 m), finishing blind on the remembered position once the apple leaves the camera view.
- Rejected: DetachableJoint (needs one joint per apple declared in the SDF, and an attached apple would stay visible to the camera), triggering from the camera estimate (ground truth simulates a contact sensor), `ros_gz_sim remove`/`delete_entity` as a subprocess (slow, and `remove` loops if it finds no world).

## D8 — Pure mission state machine + circular search pattern (session 7)
- Context: M6, chain trees, search, picking, depot and unloading; transitions testable without ROS.
- Decision: pure class `applepic_mission.mission.Mission` (states GOTO_TREE, SEARCH, PICK, GOTO_BASE, UNLOAD, DONE) receiving events (`on_detections`, `on_picked`, `on_basket_full`, `on_unloaded`) and returning the velocity command at each `step(pose, t)`; the `mission` node is only a ROS adapter (it replaces `apple_picker`, kept for the M5 demo).
  - SEARCH: 8 points on a 2.2 m circle around the trunk (starting closest to the robot); at each one the robot faces the trunk and looks for 1.5 s; the camera view (0.4–2.3 m ahead) then covers the apple ring (0.45–1.6 m) on the robot side. Moves between points keep 1.9 m from the trunk (the chassis does not drive over the ring).
  - PICK: confirmed apple (`AppleTracker`, 2 views) within 1.9 m of the current trunk, nearest first, only if the straight approach keeps the base ≥ 0.45 m from the trunk axis (stop ≥ 0.5 m); otherwise it is picked from another circle point. After each pick the robot goes back to the current search point and looks again.
  - A pick reported on `/basket/picked` is matched to the track nearest the gripper (handles apples picked "in passing").
  - Basket full → GOTO_BASE then back to the same tree (search restarted); end of loop → GOTO_BASE → UNLOAD → new loop or DONE (`loops`, 0 = forever).
- Rejected: Nav2/BehaviorTree (not installed, oversized), spiral or spin in place from the tree (more risk of crushing apples), SMACH/YASMIN (one more dependency for 6 states).

## D9 — M7 end-to-end scenario sped up by RTF (session 8)
- Context: one mission loop lasts ~15 simulated minutes; M7 needs a timed, reproducible, quick scenario.
- Decision: `scripts/e2e_mission.sh` (`make e2e`) runs the headless mission (1 loop, starting at the depot) and sets the real-time factor to 3 through the gz service `/world/orchard/set_physics` (camera rendering caps around 4 on this machine). Timing in **simulated time** (from the first mission transition), 1200 s limit, 80 % deposited-apple threshold. Score from the pure module `applepic_mission/metrics.py` (tested): PASS if mission DONE, ratio reached, within limit, gz queried, picked apples absent from gz and counters consistent. Outputs `artifacts/e2e_metrics.json` + `e2e_trajectory.png`. Replaces `check_mission.sh` / `eval_mission.py`.
- Rejected: launch_testing (heavier to debug, no gain here); real-time timing (depends on machine load); unlimited RTF (`real_time_factor: 0`): irregular camera sensors, mission not verified in that mode.

## D10 — M8 demo: dedicated launch + visualization node (session 9)
- Context: the human must be able to watch the mission; RViz has nothing to show of the orchard (no map) and the mission only publishes text states.
- Decision: `demo.launch.py` includes `mission.launch.py` (Gazebo GUI by default) + RViz (`rviz/demo.rviz`, fixed frame `odom` ≡ world) + `mission_viz` node (`applepic_mission`) publishing `~/markers` (trunks, canopies, search circles, depot, labels from `orchard.yaml`, state text above the robot) and `~/path` (trajectory decimated to 5 cm, from `/odom`). Pure logic in `viz.py` (tested). `rtf` argument to speed up through `/world/orchard/set_physics` (default 1: the GUI keeps up). Automatic headless check: `scripts/check_demo.sh` (RViz topic contents during the mission).
- Rejected: showing real apples (ground truth) in RViz (visual cheating on perception); custom RViz HUD plugin (3D text is enough); automatic video recording (no screen capture tool, no Xvfb).

## D11 — Search driven by camera coverage (session 10)
- Context: one loop took 888 simulated seconds, 531 s of it in SEARCH (8 fixed points per tree, even when everything was already seen). The real camera ground footprint, computed from the URDF, is 0.44–1.69 m ahead of the base (not 0.4–2.3 m as assumed in session 7): 8 fixed views at 2.2 m only cover ~75 % of the ring where apples fall; the rest was seen while driving to apples.
- Decision: pure module `coverage.py`: footprint = projection of the image corners (reduced to 85 %: margin for blobs cut at the border) onto the apple plane; 10 cm grid of the 0.4–1.7 m ring around each trunk, marked at every tick in SEARCH/PICK (trunk occlusion included). The mission skips points whose view would add < 1 % of the ring, leaves the tree at 98 % seen, unless an active track (even seen once) remains near the tree; once the circle is done, up to 3 "fill-in" views (best gain among 24 positions × 3 headings on the circle, gain ≥ 2 %). Circle reduced to 2.0 m (the footprint end reaches the trunk), 1.0 s look, rotation to the trunk at 1.2 rad/s. Settings chosen by a sweep in a unicycle simulator (9 orchards, 304 apples), then verified in Gazebo: 656 s instead of 888 s, 29/29.
- Rejected: stopping after N points without a new apple (no coverage guarantee); full next-best-view planner (slower to develop, marginal measured gain); more aggressive thresholds (90–95 % coverage, 3–4 % gain): 600–620 s but 1 to 8 apples lost out of 304; driving closer to trunks (risk of pushing apples with the chassis).

## D12 — Multi-seed robustness: full coverage, raised chassis, no pivot over an apple (session 11)
- Context: `make e2e` on other orchards (seeds 7, 123, 2024; same trees, other apples) passed (≥ 80 %) but left 1 to 3 apples. Two distinct causes, established from final apple poses in gz and trajectory replay:
  1. **Apples never seen** (still in place): the tree was left at 94–98 % coverage. From the 2.0 m circle facing the trunk, the footprint (0.48–1.44 m ahead) leaves two blind zones: trunk foot (r < 0.6 m) and ring edge (r ≈ 1.5–1.7 m); fill-in views (gain ≥ 2 %) did not catch them.
  2. **Apples pushed or thrown** (found 60–160 m away): the chassis bottom was at 5 cm, an apple is 8 cm tall; above all, when pivoting towards an apple that had passed under the chassis, the front caster rolled on it and the physics engine launched it.
- Decision:
  - Coverage: 1.9 m circle, 99.5 % goal, up to 4 fill-in views with gain ≥ 0.5 %, candidates on 3 radii (1.75 / 1.9 / 2.1 m) × 24 positions × 5 headings. Chosen by a unicycle sweep on 20 generated orchards (640 apples) with a strict fake detector (exactly the planned footprint): 7 apples lost → 0, but +13 % mission time.
  - Robot: 10 cm ground clearance (the chassis passes above apples; camera and gripper, defined relative to `base_footprint`, unchanged).
  - Approach: if the apple is closer than `offset + 0.1 m` to the base and misaligned, the robot drives straight away from it before pivoting (the caster sweeps a 0.32 m circle).
  - Gripper: the gz `remove` request through the Python gz-transport binding holds the GIL while waiting: the whole node (gz pose stream, `/odom`) froze up to 3 s per pick, the reply often timed out (although gz removed the apple) and the mission gave up the next apple. Now: `gz service` as a subprocess from a thread; the apple counts as soon as it leaves the pose stream (or on a positive reply), retry possible after `remove_timeout + confirm_timeout` if still there. The gripper also publishes `/basket/picked_point` (position of the picked apple), used by the mission to retire the right track: with the current gripper position, a late confirmation retired the next targeted track (apple left on the ground, seen with seed 42).
- Rejected: removing apple/robot collisions (would hide a real design problem); longer-range magic gripper (same); keeping 98 % (faster, apples lost).

## D13 — Simulated GPS + home-made localization (EKF on map → odom), gyroscope for heading (session 12)
- Context: M9 "noisy simulated GPS + robot_localization". robot_localization is not installed (no sudo). The gz NavSat sensor noise is expressed **in degrees** (σ = 1 → ~100 km, measured), identical in latitude and longitude (hence anisotropic in metres), and not reproducible.
- Decision:
  - Noise-free gz `navsat` sensor (`/gps/raw`) + `gps_noise` node: white noise + Gauss-Markov bias (σ, τ), isotropic in metres, seeded. Default: RTK (σ = 2 cm, no bias), like commercial robot mowers.
  - `localizer` node: 3-state EKF on the map → odom transform (not on the pose), updated by each GPS fix (Mahalanobis gate), prediction = odometric drift (variance ∝ distance, angle, time). Known initial heading (robot docked at the depot, `yaw` argument). After 20 rejected fixes in a row, restart on the current fix.
  - Odometry: wheel speed + **gyroscope** (gz IMU sensor, noise + bias, seeded by `gz sim --seed`). Measured: wheels under-measure in-place pivots by **7 %** (slip); without the gyroscope, the heading drifted 0.3 rad in 100 s and the filter diverged (10 m, 24/29 apples).
  - A single contract for consumers: `/localization/odom` (`map` frame) and TF `map → odom → base_footprint`, whatever the mode (`localization:=truth` publishes the identity and republishes the ground truth). The "magic" gripper keeps the ground truth (it is the physics).
- Rejected: robot_localization (not installed; reconsider if the human installs it); gz noise in degrees (anisotropic, not reproducible); filter on the full pose (same result, but the map → odom offset directly gives the standard REP-105 TF); magnetometer (gyroscope is enough with GPS corrections).

## D14 — Apples anchored to the odometry frame (session 12)
- Context: with a biased GPS (σ 0.3 m + 0.3 m bias), the map → odom estimate moves while approaching an apple: the re-detected apple fell outside the association gate (0.15 m) → ghost tracks, 8 give-ups, 27/29.
- Decision: the mission node applies each map → odom change to all tracks (and to late pick reports), through `localization.map_shift`. Robot ↔ apple geometry is then purely odometric, insensitive to GPS; the tree, search circle and coverage stay in the map frame. The gripper publishes the picked point relative to the robot (`base_footprint`, stamped at trigger time) and the mission places it with its own estimate.
- Result: same scenario → 29/29, 0 give-ups.
- Rejected: everything in the odom frame (trees and coverage would drift); wider association gate (would merge nearby apples).
- Measured limit: with a "consumer" GPS (σ 0.5 m + 1 m bias), the search around trees is shifted by 1 to 2 m → zones marked "seen" that were not, 23/29 (FAIL). This needs RTK, or correcting the pose on trunks seen by the camera.

## D15 — Pose correction on trunks seen by the camera (session 13)
- Context: "consumer" GPS (σ 0.5 m + 1 m bias) → 23/29, search shifted by 1 to 2 m (D14). The camera looks at the ground: only the trunk foot is visible, and it is just out of view from the search circle (1.9 m).
- Decision:
  - Foot detection: fit a circle of known radius on the lower edge of the brown blob projected on the ground (unbiased from the side, unlike the centroid), silhouette points discarded.
  - EKF: same measurement model as GPS, the measured point being the trunk. Association to the nearest tree (< 2.8 m; trees are 6 m apart). Unlikely innovation → inflated covariance, no rejection: a GPS bias makes the filter overconfident, and the trunk is the reliable measurement.
  - New `ANCHOR` state on arrival at each tree: facing the trunk, slow creep (≤ 1.2 m, stop at 0.7 m estimated), look at ± 0.8 rad, then SEARCH even without a fix. Active only if σ or bias > 0.1 m (seen: with RTK, 27/29 and 806 s versus 29/29 and 723 s without).
- Rejected: smaller search circle or raised camera (coverage and perception settings to redo); GPS bias as a filter state (more complex, reconsider to reduce the error between trees).
- Result: 84/87 apples over 3 noise seeds (≥ 27/29 each), at the cost of ~20–30 % more time.

## D16 — Separate hard orchard, grass only visual (session 14)
- Context: M9 "tall grass, hidden or damaged apples". The default orchard serves as the regression reference.
- Decision: separate config `orchard_hard.yaml`, same apple positions (appearance drawn from a distinct random generator). Grass blades without collision: the robot drives through them. Red band widened to 0–14 and trunks raised to 15 (measured rendered hues: orange apple 12, trunk 18). 7 px morphological closing.
- Rejected: modifying the default orchard (would lose the comparison); colliding grass (physics cost, no use for the camera); machine learning (YOLO): colour is still enough on this world.
- Limit: 3–4 hue values between orange apple and trunk; two apples less than 7 px apart merge.

## D17 — Contact bumper and backing off (session 14)
- Context: in coarse GPS, the pose error can lead the robot into a trunk; it stayed there forever (20/29).
- Decision: gz contact sensor on the chassis and on a strip wider than the wheels, like commercial robot mowers. Any contact other than ground and apples triggers a 2 s reverse (forward if still stuck), then gives up the targeted apple or view. 40 s timeout on SEARCH views as a safety net. The Contact plugin is in the world.
- Rejected: sensors on wheels (always touching the ground: 2000 messages/s on `/bumper`); timeout alone (40 s lost at each hit, and the robot keeps pushing); slip detection (wheels slip with no reliable signal).
- Limit: a bump does not improve the pose, and the targeted apple is lost for the loop.

## D18 — Unknown obstacles remembered on contact (session 15)
- Context: the bumper (D17) backed off then left without remembering anything. The camera looks at the ground and detects no obstacle.
- Decision: no new perception. The bumper edge in the direction of travel touched, lateral position unknown: 3 points across its width are remembered, anchored to odometry like apples. 0.7 m clearance when travelling (0.5 m pivot radius), 0.55 m when approaching an apple. A bump within 0.5 m of a known trunk is that trunk: nothing remembered. Bump while pivoting: nothing either (unknown direction). Interrupted apple or view: one more try. In transit, from the 2nd bump: sidestep of 1, 2, 3 m, then on the other side; at the 8th bump the tree is given up so the mission ends.
- Rejected: contact position from gz (world frame, would need ground truth); a single point straight ahead (19 bumps in the first run); alternating sidestep at each bump (back-and-forth seen in unicycle simulation); Nav2 + costmap (not installed).
- Limit: giving up a tree costs a lot in coarse GPS (18/29 on one seed). Revised in D20 (sidestep removed, tree put off).

## D19 — Wheels and caster go through apples (session 15)
- Context: apples thrown tens of metres on wheel contact, since session 11 despite several workarounds.
- Decision: `collide_bitmask` 0x01 on wheels and caster, 0x02 on apples. The ground keeps the default mask, the chassis passes above (10 cm).
- Rejected: contact settings `max_vel`/`kp` (not tried, uncertain support with the default physics engine); forbidding reversing over a known apple (does not cover unseen apples).
- Limit: the robot neither pushes nor crushes any apple: a real robot would.

## D20 — Grid search around bump points, tree put off (session 16)
- Context: bursts of bumps at the same spot, exhausting the counter and losing trees. Cause: going around a trunk brought the path back through the obstacle just hit.
- Decision: 8-connected A* on a 20 cm grid, only if the existing detour passes within 0.7 m of a remembered point. Clearances reduced to 0.7 m (trunk) and 0.55 m (contact) if there is no path, old path as a last resort. A zone containing the start or goal is reduced to that distance. Sidestep removed: each bump adds points, so the next path changes. Tree not reached after 8 bumps: put off to the end of the loop, given up on the 2nd failure.
- Rejected: A* for all paths (would change obstacle-free orchard paths, no gain); counting bumps per obstacle (treats the symptom); Nav2 + costmap (not installed); segmented bumper (recorded contacts are already on the remembered front edge).
- Limit: putting off a tree has not yet been used in a gz run, only in a unit test.

## D21 — Re-scoping (2026-11-01, human decisions)
- Context: audit in `docs/ai-workflow/audit.md`: no arm, milestone M9+ without criteria, sessions drifting towards coarse-GPS tuning.
- Decision: real arm and two-finger gripper through MoveIt 2, apple attached on contact of both fingers (replaces D7 at M20). Priorities: arm, demo, then perception. HSV kept and measured on scenes with traps. Done means M23. Navigation frozen. Repo in English, Apache-2.0.
- Decided by the audit session: a custom arm with at least 4 joints rather than an imported model (it must fit the chassis and reach the ground, and needs no download); `KICKOFF.md` and `run_applepic.sh` stay public (no secrets, they show how the project was built).
- Rejected: friction-only grasping (long, unstable contact tuning on a sphere); YOLO (torch, 13 GB of free disk, several sessions, not needed before HSV is measured); Nav2.

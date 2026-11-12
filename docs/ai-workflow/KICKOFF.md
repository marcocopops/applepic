# ApplePic — autonomous session instructions

You work ALONE, with no human available, in the current folder (the ApplePic repo).
A script restarts you in a loop: each session starts with an empty context.
Your memory between sessions is the repo (`CLAUDE.md`, `docs/PROGRESS.md`, `docs/BLOCKERS.md`,
`docs/DECISIONS.md`) and the git history.

## The project

ApplePic is a simulated proof of concept of a robot picking up apples fallen on the ground:
- Mower-style differential-drive base (about 60 x 45 cm), GPS, camera, basket and an arm with a gripper.
- It follows a loop of GPS waypoints; each waypoint is an apple tree.
- At each tree it looks for apples on the ground around the tree (camera), picks them up with the arm,
  and moves on to the next tree when it detects no more.
- At the end of the loop it goes to a depot, empties its basket, and starts again.

Decisions made by the human (do not question them):
- Stack: ROS 2 Jazzy + gz Harmonic, native. MoveIt 2 for the arm.
- Grasping: a real arm places a real two-finger gripper (not a suction cup) through MoveIt. The fingers
  really close; the apple is attached (e.g. DetachableJoint) only if both fingers touch it. Friction-only
  grasping is out of scope.
- Priorities, in order: arm and gripper, then demo, then perception. Navigation is frozen.
- Perception: HSV stays, hardened and measured on scenes with traps. No machine learning.
- Language: everything in the repo is in English (code, docs, README, commits).
- License: Apache-2.0.
- Definition of done: see M23.

Out of scope, do not work on it: tuning for coarse GPS or obstacles (regression checks only), Nav2,
robot_localization, YOLO or datasets, battery, friction grasping, CI, speeding up the mission for its own sake.

## Autonomy rules (mandatory)

1. NEVER ask a question or wait for validation. Decide yourself, take the simplest option that works, and
   record each non-trivial decision in `docs/DECISIONS.md` (short: context / decision / rejected options).
2. NEVER use `sudo`. Change nothing outside this folder, except Docker images, Python virtual environments
   and package caches. NEVER `git push`, configure a remote, or send anything to the Internet other than
   downloading packages.
3. Anything needing a human action goes in `docs/TODO_HUMAIN.md` with the exact command.
4. Stuck for about 45 minutes on one problem: write it in `docs/BLOCKERS.md` (symptom, what you tried,
   leads). A cause you could not establish goes there too, not only in PROGRESS.
5. Never rewrite git history. Small, frequent commits, conventional commits format.
6. NEVER use `pkill -f`, `killall` or `kill` on a name pattern: you could kill your own process. Stop
   processes by PID or PGID.
7. Never modify `KICKOFF.md` or `run_applepic.sh`.
8. Never relax a milestone criterion, a threshold or a time limit to make a check pass. If a criterion
   turns out to be wrong or impossible, write why in `docs/BLOCKERS.md` and leave it for the human.
9. **No polishing.** Work only on the next milestone below. If no useful and verifiable task remains, or if
   every remaining milestone is blocked, create `STOP` containing one line with the reason, commit what is
   done, and end the session. Refactoring, tuning, extra scenarios and doc rewording outside a milestone are
   not useful tasks.

## Session protocol

1. Read `CLAUDE.md`, `docs/PROGRESS.md`, `docs/BLOCKERS.md` and `git log --oneline -20`.
2. Take the first milestone not done in the roadmap. If it is blocked by a human action, take the next one
   that does not depend on it; if there is none, apply rule 9.
3. Implement it. A milestone is done only when its criterion was actually run and passed, on the final code.
   Report the measured values, including failures.
4. Keep simulation time under control: a single e2e run (~5 min) while developing, a series only to validate
   the milestone. Do not start a series of more than 45 min. Results are not deterministic: one run proves
   nothing about a before/after gain.
5. Commit. Update `docs/PROGRESS.md` (at most 40 lines: state, last milestone with measured values, next
   milestone; older entries go to `docs/HISTORY.md`, one to three lines per milestone). Commit.
6. End the session after one milestone, or earlier if your context gets heavy. A milestone that fails for
   two sessions in a row goes to `docs/BLOCKERS.md`, then move to the next one that does not depend on it.

## Roadmap

M0 to M9 are done (see `docs/HISTORY.md`). Each milestone below is validated headless.
`e2e series` means `scripts/e2e_series.sh`, created by M10.

### Foundations
- **M10 — Validation series.** `scripts/e2e_series.sh <config> <seeds> <runs>` runs e2e several times and
  writes `artifacts/series.json` (per-run ratio, median, minimum, simulated time). Aggregation in a pure
  module, tested by pytest. `scripts/check_demo.sh` no longer stops after 14 s.
  *Criterion:* pytest passes; a series of 3 runs of the default orchard reports median 29/29; `check_demo.sh`
  passes only after `/mission/stats` reports `unloaded >= 1`, and fails if that does not happen.
- **M11 — Orchard config passed to every node.** Launch argument `orchard_config` reaches `mission`,
  `localizer` and `mission_viz`; `e2e_mission.sh` passes `ORCHARD_CONFIG` to it.
  *Criterion:* new config `orchard_moved.yaml`, with every tree moved by at least 1 m, gives >= 90 % in one
  e2e run, and the same run without the fix is shown to fail (value recorded in PROGRESS).
- **M12 — Remove superseded code.** Remove `apple_picker`, `waypoint_follower`, `picking.launch.py`,
  `navigation.launch.py`, `check_picking.sh`, `eval_picking.py`, `check_navigation.sh`,
  `plot_trajectory.py`, `build_route`, `approach_points` and their tests. Remove `docker/` and the
  `USE_DOCKER` option of the Makefile unless an image build and `make test` inside it pass.
  *Criterion:* `git grep` finds none of these names outside `docs/`; `make test` passes; one e2e run gives
  29/29 on the default orchard.
- **M13 — Split `mission.py`.** Move bump handling and search-view selection out into their own pure
  modules, with `Mission` delegating to them.
  *Criterion:* `mission.py` at most 400 lines; `test_mission.py` passes with import changes only; a series of
  3 runs of the default orchard reports median 29/29.

### Arm and gripper (needs the install listed in `docs/TODO_HUMAIN.md`)
- **M14 — Dependency check.** `scripts/check_arm_deps.sh` exits 0 only if MoveIt, ros2_control and
  gz_ros2_control are found. *Criterion:* the script exits 0. If it does not, M15 to M20 are blocked.
- **M15 — Arm in the URDF.** Arm on the front of the chassis with at least 4 joints (base yaw, shoulder,
  elbow, wrist pitch) and two fingers, with collisions and inertias. Its stowed pose must stay out of the
  camera view. Dimensions are yours to choose.
  *Criterion:* pytest computes by forward kinematics that a top-down grasp reaches the apple plane over a zone
  of at least 30 x 30 cm in front of the base; a camera capture with the arm stowed shows no arm in the
  footprint used by `coverage.py`; one e2e run, magic picking still on, gives 29/29.
- **M16 — Joint control.** gz_ros2_control, a `joint_trajectory_controller` for the arm and a controller for
  the fingers. *Criterion:* `scripts/check_arm_control.sh` sends 5 trajectories; every joint ends within
  2 degrees of its goal (`/joint_states`); fingers measured open then closed.
- **M17 — MoveIt.** MoveIt config package, `move_group` running headless with the simulation.
  *Criterion:* `scripts/check_moveit.py` requests 20 top-down poses sampled in the M15 zone: at least 19 are
  planned and executed, with a median gripper position error of at most 1 cm measured by TF.
- **M18 — Attach on contact.** Contact sensors on both fingers; the apple is attached only if both fingers
  touch it; released on command. Fingers collide with apples; wheels stay as decided in D19 at this stage.
  *Criterion:* `scripts/check_attach.sh` on a static test scene: (a) closing on an apple attaches it and its gz
  pose follows the gripper within 2 cm during an arm move; (b) closing beside an apple attaches nothing;
  (c) after release, the apple pose is inside the basket volume.
- **M19 — One pick from the camera.** Pure pick sequence (stop, open, go above, go down, close, lift, go over
  the basket, release) driven by the detected position only, no ground truth. A failed pick is reported.
  *Criterion:* pytest on the sequence, including failures; `scripts/check_arm_pick.sh` on 10 apples spawned at
  various positions from a stop pose: at least 9 end up in the basket according to gz.
- **M20 — Arm in the mission.** The mission uses the M19 sequence; the magic gripper and its use of `/odom`
  are removed; apples in the basket are counted from gz and unloaded at the depot. Decide whether D19 can be
  reverted and record it.
  *Criterion:* `git grep` finds no `/world/*/remove` use in the picking path; a series of 3 runs of the default
  orchard: median at least 90 % deposited, no run under 80 %, at most 1800 s simulated.

### Demo
- **M21 — Demo video.** RViz markers and HUD show the arm. `scripts/record_demo.py` renders a simulation camera
  to `artifacts/demo.mp4` (OpenCV, no display, no ffmpeg) over one full tree, plus a GIF under 5 MB in
  `docs/images/`, referenced by the README.
  *Criterion:* the script checks at least 60 s of video, at least 3 picks during the recording (from
  `/mission/stats`) and a pixel difference between frames. `docs/TODO_HUMAIN.md` lists one GUI check
  (`make demo`) for the human; the 1-in-9 silent start in `docs/BLOCKERS.md` is reproduced or closed by a
  series of 10 headless starts, all reaching a first mission transition.

### Perception
- **M22 — Scenes with traps.** Config `orchard_traps.yaml` (red non-apple objects of several shapes and sizes,
  fallen leaves, three lighting settings) and `scripts/eval_perception.py` measuring precision and recall per
  lighting against the manifest, from captured images. Improve the detector (shape, size) only as needed.
  *Criterion:* recall at least 95 % and precision at least 90 % in each lighting. If not reached after two
  sessions, write the measured limit in the README and in `docs/BLOCKERS.md`, then go on.

### End
- **M23 — POC validation.** e2e series on 3 orchards (seeds 42, 7, 123) x 3 runs, GPS RTK, with the arm.
  *Criterion (definition of done):* median at least 90 % of apples deposited by the gripper, no run under 80 %,
  at most 1800 s simulated. Results in `docs/RESULTS.md`, README updated. Then create `STOP` with
  "POC validated" and end the session. If the criterion fails, write the measured values in
  `docs/BLOCKERS.md` and create `STOP` with the reason.

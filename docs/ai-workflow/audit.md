# Project audit (2026-11-01)

## Actual state (replayed today on commit `99b9b9a`)
1. `make test`: 163 tests, 0 failures, 22 s.
2. Default orchard, true pose, `scripts/e2e_mission.sh` run twice: **29/29 PASS** in 722 and 721 simulated seconds (~4 min 20 of real time).
3. Coarse GPS (0.5 m + 1 m bias), orchard seed 7, never tested before: **31/31 PASS**, 1000 s, 4 tracks given up.
4. Coarse GPS + obstacles, noise seed 0: **26/29 PASS**, 949 s (the docs said 27/29; same order of magnitude, runs are not deterministic).
5. `scripts/check_demo.sh`: PASS, but it stops after **14 simulated seconds** and 5 m driven (12 s of real time). It proves the RViz topics exist, not that the demo completes.
6. GUI demo not checked by this audit, nor by any session since session 14. One start in 9 stayed silent, cause unknown.
7. The docs are reliable on numbers: no claim was contradicted. They are less clear about what follows.
8. **The robot has no arm or gripper.** The "gripper" is a frame: within 7 cm of the ground truth, the apple is removed from the world.
9. Wheels go through apples (D19) and the world stays easy: flat ground, plain colours, no red distractor.
10. Conclusion: this is a good navigation and ground-search demo. It is not yet an apple picker.

## Why progress slowed down
Measured from `logs/runner.log` and git: 22 launches, 10.6 h of sessions, 52 commits.

| | Oct 26 | Oct 30 |
|---|---|---|
| Length of a useful session | 4 to 20 min (M1–M8), then 33 and 125 min | 48 to 98 min |
| Commits per hour | 5.7 | 3.7 |
| Result | M0 to M8 in ~2 h 10 | 5 M9 sub-topics in 6.5 h |

1. **Roadmap exhausted after ~2 h.** M9+ is a list of 7 ideas with no criteria, "prioritize it yourself". Nothing defines "done".
2. **Drift towards a self-imposed scenario.** RTK GPS already gave 29/29 in session 12. Sessions 13 to 16 (~5 h) worked on coarse GPS, and each fix opened the next: trunks → `ANCHOR` → stuck robot → bumper → obstacles → A*. Gain: ~94–95 % on a scenario nobody asked for.
3. **Saturated metric.** The reference scenario has been at 100 % since session 8. Sessions have no signal left and create harder scenarios to get one back.
4. **Waiting for simulation.** A validation series lasts 35 to 45 min (session 15: ~80 min out of 96), and runs are not deterministic with one run per seed. The 18 → 29/29 of session 16 overstates the gain, as PROGRESS admitted.
5. **The arm is never chosen.** Last on the list, it depends on packages that are not installed (`sudo`, forbidden to sessions): sessions take what they can finish.
6. **The stop rule does not apply.** The session judges whether "a useful improvement" remains, and one always does. It never created `STOP`. Both `STOP` files (Oct 26 17:45, Oct 30 23:24) came neither from sessions nor from a script, so probably from the human.
7. **No loop since Oct 30 23:37.** `STOP` is still present: `run_applepic.sh` would exit immediately.

Ruled out: regressions (`make test` broke only once, `258fb8a`, fixed by the next commit), sessions too short (the 2 s and 52–61 s ones at the start came from the usage limit and from `pkill -f`, solved on Oct 26).
Rediscovered every session: apples being thrown (sessions 11, 13, 14, solved in 15 by D19) and "GUI demo not rerun" (repeated 5 times, never handled). `BLOCKERS.md` stayed empty despite several unknown causes.

## Strengths
- Pure logic separated from nodes: 97–100 % coverage on `mission`, `route`, `localization`, `detector`, `coverage`, `tracker`; fast tests.
- `e2e_mission.sh` cross-checks the score with Gazebo, produces metrics and gives the poses of lost apples.
- Decisions (D1–D20) and limits written honestly, causes established by measurement.
- Clean history: small commits, no secret, no heavy file.

## Problems, most severe first
**Critical**
1. No manipulation: no arm, no gripper, no contact. The core of the POC is not simulated.
2. Roadmap without criteria, see above.

**High**
3. Weak validation: 80 % threshold, one run per seed, limit moved from 1200 to 1500 s between series, `check_demo.sh` looking at 14 s only, no CI.
4. Perception never challenged: no distractor, fixed lighting, 3–4 hue values between orange apple and trunk, exact tree positions assumed known.
5. `mission.py`: a single class of 667 lines (303 when created), about forty parameters, over 30 state attributes. Adding the arm without splitting it would make it unmanageable.
6. ROS nodes at 0 % in unit tests (`gripper`, `mission_node`, `localizer`, `apple_detector`), covered only by e2e.

**Medium**
7. Latent bug: `mission`, `localizer` and `mission_viz` always read the default `orchard.yaml`, launch files never pass `orchard_config`. An orchard with other trees gets out of sync with no error (hidden: the 3 configs share the same trees).
8. Constants duplicated between URDF, `CameraParams` and nodes (camera, `offset` 0.46 m).
9. `ANCHOR` is enabled from the noise level given to the launch file: the robot "knows" its GPS quality by configuration.
10. Home-made stack (controller, EKF, A*) chosen because Nav2 and `robot_localization` are not installed, not on the merits.

**Low**
11. Code from superseded milestones: `apple_picker`, `waypoint_follower`, their launch files and scripts; unused import in `waypoint_follower.py`.
12. Heavy docs: PROGRESS 340 lines (30 KB) reread each session, BILAN written "for tonight" and poorly ordered. Dockerfile never verified.
13. `run_applepic.sh`: logs reduced to the last message (impossible to diagnose afterwards), no pause after a late error (`code 1` at 495–503 s), deadline already past = silent exit, no per-session limit, no lock against two loops, local changes never committed.

## Recommendations
**Keep:** pure logic + nodes architecture, configurable orchard, HSV detection, tracker, coverage, `e2e_mission.sh` and its metrics, GPS localization (as is), commit rules.

**Redo:**
- the roadmap, with a verifiable criterion per milestone;
- `mission.py`, split before adding the arm;
- the gripper: real arm, real fingers, ground-truth `/odom` removed from picking;
- D19 (wheels going through apples), to reassess once the arm is in place;
- validation: several runs per scenario, median score, higher threshold, `check_demo.sh` up to the first unload;
- `run_applepic.sh` and the format of PROGRESS / BILAN.

**Freeze:** coarse-GPS and obstacle tuning. No more work on them without an explicit request; the scenario serves as a regression check.

**Remove:** `apple_picker`, `waypoint_follower`, `picking.launch.py`, `navigation.launch.py`, `check_picking.sh`, `eval_picking.py`, `check_navigation.sh`, `plot_trajectory.py`, `build_route` and `approach_points`; the Dockerfile if it stays unverified.

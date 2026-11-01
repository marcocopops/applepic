# History

Condensed from the session log. The full original logs, in French, are `docs/PROGRESS.md` and `docs/BILAN.md` at commit `f6610b5`.

## M0 to M8 (2026-10-26)
- **M0 Infrastructure:** ROS 2 workspace with 6 `ament_python` packages, Makefile, optional Dockerfile. Headless camera rendering works.
- **M1 Orchard:** YAML-driven SDF generator, 6 trees, 29 apples, apple manifest as ground truth.
- **M2 Robot:** differential base, camera, visual basket, gripper frame, gz plugins, ros_gz bridge.
- **M3 Navigation:** home-made point controller and tree loop, 40.5 m, never within 2 m of a trunk.
- **M4 Perception:** HSV segmentation and flat-ground projection, error at most 1.5 cm, no false positive on this world.
- **M5 Picking:** magic gripper (apple removed within 7 cm, using ground truth), basket capacity, apple tracker.
- **M6 Mission:** pure state machine GOTO_TREE, SEARCH, PICK, GOTO_BASE, UNLOAD, DONE.
- **M7 End to end:** `make e2e` at RTF 3, scored against gz: 29/29, 888 s at first.
- **M8 Demo:** `make demo` with Gazebo GUI, RViz markers and HUD; `scripts/check_demo.sh`.

## M9 (2026-10-26 to 2026-10-30)
- **Session 10, coverage-driven search (D11):** 888 s down to 656 s, 29/29.
- **Session 11, other orchards (D12):** seeds 42, 7, 123, 2024: 128/128 apples after four fixes (blind zones, chassis height, caster over apples, gripper freezing on the GIL). Mission back up to 725 s on the default orchard.
- **Session 12, GPS (D13, D14):** home-made EKF on GPS, wheels and gyro. RTK 29/29; 0.5 m + 1 m bias: 23/29 FAIL.
- **Session 13, trunks (D15):** pose correction on trunks seen by the camera, `ANCHOR` state. Coarse GPS: 84/87.
- **Session 14, hard orchard and bumper (D16, D17):** grass and apple variants 29/29 after fixes; bumper against stuck robots. Coarse GPS: 82/87.
- **Session 15, obstacles (D18, D19):** remembered bump points; wheels go through apples. Obstacles + coarse GPS: 71/87.
- **Session 16, grid search (D20):** A* around bump points, unreachable trees put off. Obstacles + coarse GPS: 83/87, one run per seed.

## Re-scoping (2026-11-01)
Audit in `docs/AUDIT.md`. Roadmap rewritten around a real arm and gripper (M10 to M23).

# Progress

Keep this file under 40 lines. Older entries go to the "Done" section of `docs/roadmap.md`.

## State (2026-11-01, project re-scoping, interactive session)
- Measured on `99b9b9a`: `make test` 163 tests, 0 failures. Default orchard, ground truth: 29/29 PASS twice (722 s, 721 s).
  Coarse GPS, orchard seed 7 (first test): 31/31 PASS, 1000 s. Coarse GPS + obstacles, noise seed 0: 26/29 PASS, 949 s.
- `scripts/check_demo.sh` passes but only covers ~14 simulated seconds. The GUI demo was not checked.
- The robot has no arm: picking is magic, based on ground truth (D7). Wheels go through apples (D19).
- Full analysis: `docs/ai-workflow/audit.md`. Known unexplained issues: `docs/ai-workflow/blockers.md`.

## Done in this session
- Roadmap rewritten in `KICKOFF.md` (M10 to M23), repo moved to English, `run_applepic.sh` fixed.

## Human decisions
- Attach on contact with a real gripper through MoveIt 2; priorities arm, then demo, then perception; HSV with
  trap scenes; done means M23.

## Publication
- `LICENSE`, public `README.md`, `.gitignore` done. History scan: no secret and no heavy file. A personal email found in old `package.xml` and `setup.py` versions was removed by a history rewrite the human approved. Nothing pushed, no remote.

## Next milestone: M10 — Validation series
See `docs/ai-workflow/KICKOFF.md`. M10 to M13 need no installation. M14 onward needs `docs/ai-workflow/human-actions.md`.

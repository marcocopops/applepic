# Blockers and unexplained issues

## Waiting for the human
- M14 to M20 need the install in `docs/ai-workflow/human-actions.md`.

## Unexplained, not being worked on
- **Silent GUI demo start** (session 14): 1 GUI launch out of 9 had no mission transition for 90 s. Not reproduced in 6 cold
  starts. Handled by M21.
- **EKF divergence** (session 13): one coarse-GPS run reached 6.8 m and 1.6 rad with 2 filter resets, and still finished 29/29.
  A robot stuck against a trunk is plausible, not proven; the run is gone. Coarse GPS is frozen.
- **`ANCHOR` without a trunk fix** (session 13): 9 arrivals out of 24 in coarse GPS. Frozen.
- **Apples lost in coarse GPS + obstacles** (sessions 16 and audit): 2 to 3 apples per run, cause not analyzed. Frozen.

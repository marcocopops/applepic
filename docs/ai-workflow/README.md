# How ApplePic was built: autonomous Claude Code sessions

Most of the code in this repository was written by [Claude Code](https://claude.com/claude-code) running
unattended, in a loop, with a human setting the goals and reviewing the results between runs.

## The loop

```text
run_applepic.sh ──► claude -p "Read docs/ai-workflow/KICKOFF.md and follow it"
       ▲                    │  reads CLAUDE.md, progress.md, blockers.md, git log
       │                    │  takes the next milestone, implements it, runs its headless check
       │                    │  commits, updates progress.md / roadmap.md / decisions.md
       └────────────────────┘  stops on: STOP file, deadline, two sessions without a commit
```

- Each session starts with an **empty context**. Its only memory is the repository: `CLAUDE.md` (environment
  and commands), [`progress.md`](progress.md) (state, next milestone), [`blockers.md`](blockers.md),
  [`../decisions.md`](../decisions.md) and the git log.
- [`KICKOFF.md`](KICKOFF.md) holds the project brief, the autonomy rules (no questions, no `sudo`, no push,
  never relax a criterion, no polishing) and the roadmap. Every milestone has a criterion that a script checks
  headless: a test, a metric from the end-to-end run, or an image.
- [`run_applepic.sh`](run_applepic.sh) restarts sessions until a deadline (`run_applepic.sh 23:30`), pauses
  15 min after a failed or very short session (usage limit), kills a session stuck for 3 h and logs every
  tool call as `stream-json` in `logs/`.
- Anything that needs a human (installing packages with `sudo`) goes to [`human-actions.md`](human-actions.md).

## In numbers (up to the 2026-11-01 audit)
22 launches, 10.6 h of session time, 52 commits. M0 to M8 (world, robot, navigation, perception, picking,
mission, end-to-end test, demo) took about 2 h 10. Details in [`audit.md`](audit.md).

## Lessons learned
- **A milestone needs a verifiable criterion.** Once the initial roadmap was done (after ~2 h), a vague
  "improve what you think matters" list sent sessions into 5 h of tuning on a scenario nobody asked for. The
  rewritten roadmap gives each milestone a headless check.
- **A saturated metric pushes sessions to invent harder ones.** At 29/29 apples there is no signal left.
- **Sessions take what they can finish.** The arm, which needs packages only a human can install, was never
  picked. Blocked work must be explicit, with the exact command for the human.
- **The stop rule must be mechanical.** "Stop when nothing useful remains" never triggered; the runner now
  also stops after two sessions without a commit.
- **Process hygiene matters.** A `pkill -f gz` run by a session killed the session itself, because the whole
  prompt was in the command line. The prompt is now one short line, and killing by name is forbidden.
- **Keep the memory files short.** A 340-line progress log re-read every session cost context; it is now
  capped at 40 lines and old entries move to [`../roadmap.md`](../roadmap.md).

## Files
| File | Purpose |
|---|---|
| [`KICKOFF.md`](KICKOFF.md) | Brief, rules, protocol and roadmap given to every session |
| [`run_applepic.sh`](run_applepic.sh) | The loop runner |
| [`progress.md`](progress.md) | Current state and next milestone (session memory) |
| [`blockers.md`](blockers.md) | Unexplained issues and work waiting for a human |
| [`human-actions.md`](human-actions.md) | Commands the human must run |
| [`audit.md`](audit.md) | Interactive audit of the first 16 sessions and why progress slowed down |

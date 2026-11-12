#!/usr/bin/env bash
# Run Claude Code in a loop on docs/ai-workflow/KICKOFF.md until the deadline, a STOP file, or two sessions without a commit.
# Failed or very short sessions usually mean the usage limit was reached: wait, then retry.
# Run from anywhere: it works at the repository root (logs/ and STOP live there).

cd "$(dirname "$0")/../.." || exit 1
mkdir -p logs

exec 9> logs/.lock
if ! flock -n 9; then
  echo "Another run_applepic.sh is already running."
  exit 1
fi

if [ -f STOP ]; then
  echo "STOP file present, not starting: $(cat STOP)"
  echo "Remove it to start the loop: rm STOP"
  exit 0
fi

DEADLINE_HOUR="${1:-23:30}"                      # e.g. docs/ai-workflow/run_applepic.sh 22:00
MAX_SESSION="${MAX_SESSION:-3h}"                 # a session stuck longer than this is killed
DEADLINE=$(date -d "today $DEADLINE_HOUR" +%s)
# A deadline already past today means tomorrow.
[ "$DEADLINE" -le "$(date +%s)" ] && DEADLINE=$(date -d "tomorrow $DEADLINE_HOUR" +%s)
echo "Deadline: $(date -d "@$DEADLINE" '+%a %H:%M')" | tee -a logs/runner.log

# Deliberately short prompt: it shows up in the command line of the claude process. With all of
# KICKOFF.md as an argument, a "pkill -f gz" or "pkill -f ros2" run by Claude killed Claude (code 143).
PROMPT="Read the file docs/ai-workflow/KICKOFF.md in this folder and follow its instructions to the letter."
i=0
no_commit=0
reason="deadline reached"

while [ "$(date +%s)" -lt "$DEADLINE" ]; do
  if [ -f STOP ]; then
    reason="STOP file: $(cat STOP)"
    break
  fi
  i=$((i + 1))
  start=$(date +%s)
  head_before=$(git rev-parse HEAD)
  LOG="logs/session_$(date '+%Y%m%d_%H%M%S').log"   # timestamped: a restart overwrites nothing
  echo "=== Session $i — $(date '+%H:%M:%S') — $LOG ===" | tee -a logs/runner.log

  # stream-json keeps every tool call: the plain output only has the last message.
  timeout -k 60 "$MAX_SESSION" claude -p "$PROMPT" --dangerously-skip-permissions \
    --output-format stream-json --verbose < /dev/null > "$LOG" 2>&1
  code=$?

  duration=$(( $(date +%s) - start ))
  echo "    end (code $code) after ${duration}s" | tee -a logs/runner.log

  [ -f STOP ] && continue     # a session ending on purpose: no pause
  if [ "$code" -ne 0 ] || [ "$duration" -lt 120 ]; then
    tail -n 3 "$LOG" | cut -c1-300 | sed 's/^/    | /' | tee -a logs/runner.log
    echo "    pause 15 min" | tee -a logs/runner.log
    sleep 900
    continue
  fi

  if [ "$(git rev-parse HEAD)" = "$head_before" ]; then
    no_commit=$((no_commit + 1))
    echo "    no new commit ($no_commit in a row)" | tee -a logs/runner.log
    if [ "$no_commit" -ge 2 ]; then
      reason="two sessions in a row without a commit"
      break
    fi
  else
    no_commit=0
  fi
done

echo "=== Stop — $(date '+%H:%M:%S') — $reason ===" | tee -a logs/runner.log

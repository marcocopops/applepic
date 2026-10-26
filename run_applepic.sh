#!/usr/bin/env bash
# Relance Claude Code en boucle sur KICKOFF.md jusqu'à l'heure limite ou la présence d'un fichier STOP.
# Quand la limite d'usage est atteinte, les sessions échouent vite : on attend puis on réessaie.

cd "$(dirname "$0")" || exit 1
mkdir -p logs

DEADLINE_HOUR="${1:-23:30}"                      # ex: ./run_applepic.sh 22:00
DEADLINE=$(date -d "today $DEADLINE_HOUR" +%s)
PROMPT="$(cat KICKOFF.md)"
i=0

while [ "$(date +%s)" -lt "$DEADLINE" ] && [ ! -f STOP ]; do
  i=$((i + 1))
  start=$(date +%s)
  echo "=== Session $i — $(date '+%H:%M:%S') ===" | tee -a logs/runner.log

  claude -p "$PROMPT" --dangerously-skip-permissions \
    > "logs/session_$(printf '%03d' "$i").log" 2>&1
  code=$?

  duration=$(( $(date +%s) - start ))
  echo "    fin (code $code) après ${duration}s" | tee -a logs/runner.log

  # Session très courte = probablement limite d'usage atteinte ou erreur : on patiente.
  if [ "$duration" -lt 120 ]; then
    tail -n 3 "logs/session_$(printf '%03d' "$i").log" | sed 's/^/    | /' | tee -a logs/runner.log
    echo "    pause 15 min" | tee -a logs/runner.log
    sleep 900
  fi
done

echo "=== Arrêt — $(date '+%H:%M:%S') ===" | tee -a logs/runner.log

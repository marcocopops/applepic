#!/usr/bin/env bash
# M7 end-to-end scenario: headless sim + detector + gripper + mission, one loop from the depot,
# simulation sped up to RTF (camera rendering caps it around 4 on the dev machine).
# PASS if the mission reaches DONE within SIM_LIMIT simulated seconds with at least MIN_RATIO of
# the apples unloaded at the depot, and gz agrees (picked models removed).
# Outputs: artifacts/e2e_metrics.json, artifacts/e2e_trajectory.{csv,png}, artifacts/e2e.log.
# With a seed, a world with other apple positions (same trees) is generated in artifacts/ first.
# Usage: scripts/e2e_mission.sh [min_ratio [sim_limit [rtf [capacity [wall_timeout [seed]]]]]]
# World (env): ORCHARD_CONFIG=<yaml> generates the world from that config (e.g. orchard_hard.yaml).
# Localization (env): LOCALIZATION=truth|gps, GPS_NOISE, GPS_BIAS, GPS_BIAS_TAU (m, m, s), GPS_SEED.
set -o pipefail
cd "$(dirname "$0")/.."
source /opt/ros/jazzy/setup.bash && source install/setup.bash
LOC=${LOCALIZATION:-truth}; GPS_NOISE=${GPS_NOISE:-0.02}; GPS_BIAS=${GPS_BIAS:-0.0}
GPS_BIAS_TAU=${GPS_BIAS_TAU:-60.0}; GPS_SEED=${GPS_SEED:-0}
MIN=${1:-0.8}; SIM_LIMIT=${2:-1200}; RTF=${3:-3}; CAP=${4:-10}; TIMEOUT=${5:-1800}; SEED=${6:-}
mkdir -p artifacts
WORLD=orchard.sdf
MANIFEST=src/applepic_gazebo/worlds/orchard_apples.yaml
CONFIG=${ORCHARD_CONFIG:-src/applepic_gazebo/config/orchard.yaml}
if [ -n "$SEED" ] || [ -n "${ORCHARD_CONFIG:-}" ]; then
  STEM=$(basename "$CONFIG" .yaml)
  WORLD="$PWD/artifacts/${STEM}_seed${SEED:-default}.sdf"
  MANIFEST="$PWD/artifacts/${STEM}_seed${SEED:-default}_apples.yaml"
  ros2 run applepic_gazebo generate_world --config "$CONFIG" \
    ${SEED:+--seed "$SEED"} --output "$WORLD" --manifest "$MANIFEST" || exit 2
fi
LOG=artifacts/e2e.log
CSV="$PWD/artifacts/e2e_trajectory.csv"
# Job control: the launch gets its own process group, stopped by PGID (never by name).
set -m
ros2 launch applepic_bringup mission.launch.py headless:=true world:="$WORLD" capacity:="$CAP" loops:=1 \
  localization:="$LOC" gps_noise:="$GPS_NOISE" gps_bias:="$GPS_BIAS" gps_bias_tau:="$GPS_BIAS_TAU" \
  gps_seed:="$GPS_SEED" trajectory_csv:="$CSV" > "$LOG" 2>&1 &
LAUNCH=$!
cleanup() {
  kill -INT -- -"$LAUNCH" 2>/dev/null
  for _ in $(seq 10); do kill -0 -- -"$LAUNCH" 2>/dev/null || return 0; sleep 1; done
  kill -KILL -- -"$LAUNCH" 2>/dev/null || true
}
trap cleanup EXIT
if [ "$RTF" != "1" ]; then
  ok=0
  for _ in $(seq 60); do
    if gz service -s /world/orchard/set_physics --reqtype gz.msgs.Physics \
        --reptype gz.msgs.Boolean --timeout 2000 \
        --req "max_step_size: 0.001, real_time_factor: $RTF" 2>/dev/null | grep -q true; then
      ok=1; break
    fi
    sleep 1
  done
  [ $ok = 1 ] && echo "real-time factor set to $RTF" || echo "WARN: could not set RTF $RTF"
fi
python3 scripts/eval_e2e.py --manifest "$MANIFEST" ${SEED:+--seed "$SEED"} \
  --truth-csv artifacts/e2e_truth.csv --localization "$LOC" --gps-noise "$GPS_NOISE" --gps-bias "$GPS_BIAS" --min-ratio "$MIN" --sim-limit "$SIM_LIMIT" --timeout "$TIMEOUT"
status=$?
cleanup  # stop the nodes first so the trajectory CSV is flushed and closed
python3 scripts/plot_e2e.py "$CSV" artifacts/e2e_metrics.json artifacts/e2e_trajectory.png \
  "$MANIFEST" artifacts/e2e_truth.csv || true
echo "--- mission transitions ---"
grep -E "\[mission.*(->|done|unloaded)" "$LOG" | sed 's/^/  /' | tail -60
exit $status

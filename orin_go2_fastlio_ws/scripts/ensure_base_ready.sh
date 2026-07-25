#!/usr/bin/env bash
set -euo pipefail

WS=${GO2_WS:-/home/unitree/go2_fastlio_ws}
BASE_SCRIPT=${GO2_BASE_BRINGUP_SCRIPT:-${WS}/scripts/base_bringup.sh}
BASE_LOG=${GO2_BASE_LOG:-/tmp/go2_saas_base.log}
MAX_ODOM_ABS_XY=${GO2_MAX_ODOM_ABS_XY:-100.0}
MAX_ODOM_ABS_Z=${GO2_MAX_ODOM_ABS_Z:-5.0}
MAX_ODOM_AGE=${GO2_MAX_ODOM_AGE:-0.35}
FASTLIO_SNAPSHOT=${GO2_FASTLIO_SNAPSHOT:-/dev/shm/go2_fastlio_latest_odom.txt}
FASTLIO_FRESH_CHECKER=${GO2_FASTLIO_FRESH_CHECKER:-${WS}/scripts/check_fastlio_freshness.py}
FASTLIO_FRESH_FRAMES=${GO2_FASTLIO_FRESH_FRAMES:-10}
FASTLIO_FRESH_MAX_AGE_MS=${GO2_FASTLIO_FRESH_MAX_AGE_MS:-250}
FASTLIO_FRESH_MIN_AGE_MS=${GO2_FASTLIO_FRESH_MIN_AGE_MS:--100}
FASTLIO_FRESH_MIN_FRAME_DT_MS=${GO2_FASTLIO_FRESH_MIN_FRAME_DT_MS:-50}
FASTLIO_FRESH_MAX_FRAME_DT_MS=${GO2_FASTLIO_FRESH_MAX_FRAME_DT_MS:-150}
FASTLIO_FRESH_POLL_MS=${GO2_FASTLIO_FRESH_POLL_MS:-20}
PATROL_START_MAX_CPU=${GO2_PATROL_START_MAX_CPU:-85.0}
PATROL_START_STABLE_SAMPLES=${GO2_PATROL_START_STABLE_SAMPLES:-3}
PATROL_START_TIMEOUT=${GO2_PATROL_START_TIMEOUT:-120}
export MAX_ODOM_ABS_XY MAX_ODOM_ABS_Z MAX_ODOM_AGE

cd "$WS"

topic_ready() {
  local topic=$1
  timeout 4 bash -lc "source '${WS}/scripts/env_common.sh' >/dev/null 2>&1 || true; ros2 topic list 2>/dev/null" | grep -qx "$topic"
}

wait_topic() {
  local topic=$1
  local timeout_s=$2
  local elapsed=0
  while ! topic_ready "$topic"; do
    if [ "$elapsed" -ge "$timeout_s" ]; then
      echo "MISSING_TOPIC:${topic}"
      exit 3
    fi
    sleep 1
    elapsed=$((elapsed + 1))
  done
  echo "TOPIC_READY:${topic}"
}

wait_fastlio_fresh() {
  local timeout_s=$1
  local frame_count=${2:-$FASTLIO_FRESH_FRAMES}
  if [ ! -f "$FASTLIO_FRESH_CHECKER" ]; then
    echo "FASTLIO_FRESH_CHECKER_MISSING:path=${FASTLIO_FRESH_CHECKER}"
    return 1
  fi

  python3 -u "$FASTLIO_FRESH_CHECKER" \
    --snapshot "$FASTLIO_SNAPSHOT" \
    --frames "$frame_count" \
    --max-age-ms "$FASTLIO_FRESH_MAX_AGE_MS" \
    --min-age-ms "$FASTLIO_FRESH_MIN_AGE_MS" \
    --min-frame-dt-ms "$FASTLIO_FRESH_MIN_FRAME_DT_MS" \
    --max-frame-dt-ms "$FASTLIO_FRESH_MAX_FRAME_DT_MS" \
    --poll-ms "$FASTLIO_FRESH_POLL_MS" \
    --timeout "$timeout_s"
}

read_cpu_counters() {
  local label user nice system idle iowait irq softirq steal guest guest_nice
  read -r label user nice system idle iowait irq softirq steal guest guest_nice < /proc/stat
  local idle_all=$((idle + iowait))
  local total=$((user + nice + system + idle + iowait + irq + softirq + steal))
  printf '%s %s\n' "$total" "$idle_all"
}

sample_system_cpu() {
  local total_1 idle_1 total_2 idle_2 total_delta idle_delta
  read -r total_1 idle_1 < <(read_cpu_counters)
  sleep 1
  read -r total_2 idle_2 < <(read_cpu_counters)
  total_delta=$((total_2 - total_1))
  idle_delta=$((idle_2 - idle_1))
  awk -v total="$total_delta" -v idle="$idle_delta" \
    'BEGIN {
      if (total <= 0) {
        print "100.0"
      } else {
        printf "%.1f", 100.0 * (total - idle) / total
      }
    }'
}

wait_patrol_start_gate() {
  local start_epoch
  local elapsed=0
  local stable=0
  local sample=0
  local cpu_pct=100.0
  local fresh=0
  local cpu_ok=0
  local freshness_output=""

  start_epoch=$(date +%s)
  if ! wait_fastlio_fresh "$PATROL_START_TIMEOUT"; then
    echo "PATROL_START_GATE_TIMEOUT reason=fastlio_initial_freshness"
    return 1
  fi

  elapsed=$(($(date +%s) - start_epoch))
  while [ "$elapsed" -lt "$PATROL_START_TIMEOUT" ]; do
    cpu_pct=$(sample_system_cpu)
    sample=$((sample + 1))

    fresh=0
    if freshness_output=$(wait_fastlio_fresh 2 1 2>&1); then
      fresh=1
    fi
    printf '%s\n' "$freshness_output"

    cpu_ok=0
    if awk -v cpu="$cpu_pct" -v limit="$PATROL_START_MAX_CPU" \
      'BEGIN { exit !(cpu <= limit) }'; then
      cpu_ok=1
    fi

    if [ "$fresh" -eq 1 ] && [ "$cpu_ok" -eq 1 ]; then
      stable=$((stable + 1))
    else
      stable=0
    fi
    echo "PATROL_START_GATE sample=${sample} cpu_pct=${cpu_pct} cpu_limit=${PATROL_START_MAX_CPU} fastlio_fresh=${fresh} stable=${stable}/${PATROL_START_STABLE_SAMPLES}"
    if [ "$stable" -ge "$PATROL_START_STABLE_SAMPLES" ]; then
      echo "PATROL_START_GATE_READY cpu_pct=${cpu_pct} fastlio_fresh=${fresh}"
      return 0
    fi
    elapsed=$(($(date +%s) - start_epoch))
  done

  echo "PATROL_START_GATE_TIMEOUT timeout=${PATROL_START_TIMEOUT}s cpu_pct=${cpu_pct} fastlio_fresh=${fresh}"
  return 1
}

if [ "${1:-}" = "--fresh-only" ]; then
  wait_fastlio_fresh 120
  exit $?
fi

if [ "${1:-}" = "--patrol-start-gate" ]; then
  wait_patrol_start_gate
  exit $?
fi

need_base=0
# Match the executable name, not arbitrary caller command lines that happen to
# contain the text "fastlio_mapping".
fastlio_count=$(pgrep -xc 'fastlio_mapping' || true)
if [ "$fastlio_count" -eq 0 ]; then
  echo "BASE_RESTART_REASON:fastlio_mapping_absent"
  need_base=1
elif [ "$fastlio_count" -ne 1 ]; then
  echo "BASE_RESTART_REASON:fastlio_mapping_count=${fastlio_count}"
  need_base=1
else
  for topic in /livox/lidar /livox/imu /Odometry /cloud_registered /cloud_registered_body; do
    if ! topic_ready "$topic"; then
      echo "BASE_RESTART_REASON:topic_not_ready:${topic}"
      need_base=1
      break
    fi
  done
  if [ "$need_base" = "0" ]; then
    freshness_check=$(wait_fastlio_fresh 120 2>&1) || {
      echo "BASE_RESTART_REASON:fastlio_output_stale"
      need_base=1
    }
    [ "$need_base" = "0" ] && echo "$freshness_check"
  fi
fi

if [ "$need_base" = "1" ]; then
  old_base=$(pgrep -f '[s]cripts/base_bringup.sh' || true)
  if [ -n "$old_base" ]; then
    kill $old_base 2>/dev/null || true
    sleep 2
    old_base=$(pgrep -f '[s]cripts/base_bringup.sh' || true)
    [ -n "$old_base" ] && kill -9 $old_base 2>/dev/null || true
  fi
  setsid nohup bash "$BASE_SCRIPT" </dev/null >"$BASE_LOG" 2>&1 &
  echo "BASE_STARTING pid=$!"
  sleep 5
else
  echo "BASE_ALREADY_READY"
fi

wait_topic /livox/lidar 120
wait_topic /livox/imu 60
wait_topic /Odometry 150
wait_topic /cloud_registered 90
wait_topic /cloud_registered_body 90
wait_fastlio_fresh 120 || exit 4

echo BASE_READY

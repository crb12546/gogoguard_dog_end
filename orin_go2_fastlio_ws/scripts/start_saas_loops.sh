#!/usr/bin/env bash
set -u

WS=${GO2_WS:-/home/unitree/go2_fastlio_ws}
ENV_FILE=${GO2_SAAS_ENV_FILE:-/home/unitree/.config/go2_saas.env}
PY=${GO2_PYTHON:-python3}
AGENT=${GO2_SAAS_AGENT:-${WS}/scripts/go2_saas_agent.py}
COMMAND_RUN=${GO2_SAAS_COMMAND_RUN:-/tmp/go2_saas_command.run}
VIDEO_RUN=${GO2_SAAS_VIDEO_RUN:-/tmp/go2_saas_video.run}
OUTBOX_RUN=${GO2_SAAS_OUTBOX_RUN:-/tmp/go2_saas_outbox.run}
COMMAND_LOG=${GO2_SAAS_COMMAND_LOG:-/tmp/go2_saas_command.log}
VIDEO_LOG=${GO2_SAAS_VIDEO_LOG:-/tmp/go2_saas_video_loop.log}
OUTBOX_LOG=${GO2_SAAS_OUTBOX_LOG:-/tmp/go2_saas_outbox.log}
EXECUTE_SAFE=${GO2_SAAS_EXECUTE_SAFE:-1}
WAIT_VALID_TIME_SCRIPT=${GO2_WAIT_VALID_TIME_SCRIPT:-${WS}/scripts/wait_valid_time.sh}
TIME_WAIT_TIMEOUT=${GO2_TIME_WAIT_TIMEOUT:-180}

load_env() {
  # shellcheck disable=SC1091
  source "${WS}/scripts/env_common.sh" >/dev/null 2>&1 || true
  if [ -f "$ENV_FILE" ]; then
    # shellcheck disable=SC1090
    source "$ENV_FILE"
  fi
}

start_one() {
  local name=$1
  local pattern=$2
  local logfile=$3
  shift 3
  if pgrep -f "$pattern" >/dev/null 2>&1; then
    echo "${name}_ALREADY_RUNNING"
    return 0
  fi
  nohup "$@" >>"$logfile" 2>&1 &
  echo "${name}_STARTED pid=$!"
}

start_loops() {
  load_env
  if [ -x "$WAIT_VALID_TIME_SCRIPT" ]; then
    "$WAIT_VALID_TIME_SCRIPT" "$TIME_WAIT_TIMEOUT" || return $?
  fi
  touch "$COMMAND_RUN" "$VIDEO_RUN" "$OUTBOX_RUN"
  command_args=( "$PY" -u "$AGENT" command-loop --interval 5 --run-file "$COMMAND_RUN" --ros-timeout 1.5 --post-timeout 5 )
  if [ "$EXECUTE_SAFE" = "1" ]; then
    command_args+=( --execute-safe )
  fi
  start_one COMMAND "[g]o2_saas_agent.py command-loop" "$COMMAND_LOG" "${command_args[@]}"
  start_one VIDEO "[g]o2_saas_agent.py video-loop" "$VIDEO_LOG" "$PY" -u "$AGENT" video-loop --seconds 20 --upload --run-file "$VIDEO_RUN"
  start_one OUTBOX "[g]o2_saas_agent.py outbox-loop" "$OUTBOX_LOG" "$PY" -u "$AGENT" outbox-loop --interval 10 --max-jobs 2 --run-file "$OUTBOX_RUN"
}

stop_loops() {
  rm -f "$COMMAND_RUN" "$VIDEO_RUN" "$OUTBOX_RUN"
  pkill -TERM -f "[g]o2_saas_agent.py command-loop" 2>/dev/null || true
  pkill -TERM -f "[g]o2_saas_agent.py video-loop" 2>/dev/null || true
  pkill -TERM -f "[g]o2_saas_agent.py outbox-loop" 2>/dev/null || true
  echo SAAS_LOOPS_STOP_REQUESTED
}

status_loops() {
  ps -eo pid=,comm=,args= | grep '[g]o2_saas_agent.py' || true
}

case "${1:-start}" in
  start)
    start_loops
    ;;
  stop)
    stop_loops
    ;;
  restart)
    stop_loops
    sleep 2
    start_loops
    ;;
  status)
    status_loops
    ;;
  *)
    echo "usage: $0 {start|stop|restart|status}" >&2
    exit 2
    ;;
esac

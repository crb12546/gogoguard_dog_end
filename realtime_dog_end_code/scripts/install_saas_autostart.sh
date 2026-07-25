#!/usr/bin/env bash
set -euo pipefail

WS=${GO2_WS:-/home/unitree/go2_fastlio_ws}
ENV_FILE=${GO2_SAAS_ENV_FILE:-/home/unitree/.config/go2_saas.env}
EXECUTE_SAFE=${GO2_SAAS_EXECUTE_SAFE:-1}
COMMAND_INTERVAL=${GO2_SAAS_COMMAND_INTERVAL:-5}
VIDEO_SECONDS=${GO2_SAAS_VIDEO_SECONDS:-20}
POST_TIMEOUT=${GO2_SAAS_POST_TIMEOUT:-5}
SAAS_USER=${GO2_SAAS_USER:-unitree}
SAAS_HOME=${GO2_SAAS_HOME:-/home/${SAAS_USER}}
ROS_LOG_DIR=${GO2_ROS_LOG_DIR:-/tmp/ros_logs}
SEEN_FILE=${GO2_SAAS_SEEN_FILE:-${WS}/patrol_logs/go2_saas_seen_commands.json}
RUN_DIR=${GO2_SAAS_RUN_DIR:-${WS}/patrol_logs/run}
COMMAND_RUN=${GO2_SAAS_COMMAND_RUN:-${RUN_DIR}/command.run}
VIDEO_RUN=${GO2_SAAS_VIDEO_RUN:-${RUN_DIR}/video.run}
OUTBOX_RUN=${GO2_SAAS_OUTBOX_RUN:-${RUN_DIR}/outbox.run}
WAIT_VALID_TIME_SCRIPT=${GO2_WAIT_VALID_TIME_SCRIPT:-${WS}/scripts/wait_valid_time.sh}

if [ "$EXECUTE_SAFE" = "1" ]; then
  EXECUTE_SAFE_ARG="--execute-safe"
else
  EXECUTE_SAFE_ARG=""
fi

sudo_run() {
  if [ -n "${SUDO_PASSWORD:-}" ]; then
    printf '%s\n' "$SUDO_PASSWORD" | sudo -S -p '' "$@"
  else
    sudo "$@"
  fi
}

write_service() {
  local name=$1
  local command=$2
  local tmp
  tmp=$(mktemp)
  cat >"$tmp" <<EOF
[Unit]
Description=${name}
After=network.target
Wants=network.target
After=network-online.target time-sync.target
Wants=network-online.target time-sync.target

[Service]
Type=simple
User=${SAAS_USER}
WorkingDirectory=${WS}
Environment=HOME=${SAAS_HOME}
Environment=ROS_LOG_DIR=${ROS_LOG_DIR}
ExecStartPre=/bin/bash -lc '${WAIT_VALID_TIME_SCRIPT} 0'
ExecStart=/bin/bash -lc '${command}'
Restart=always
RestartSec=5
TimeoutStartSec=0
StandardOutput=append:/tmp/${name}.log
StandardError=append:/tmp/${name}.log

[Install]
WantedBy=multi-user.target
EOF
  sudo_run install -m 0644 "$tmp" "/etc/systemd/system/${name}.service"
  rm -f "$tmp"
}

common_prefix="mkdir -p ${ROS_LOG_DIR} ${WS}/patrol_logs ${RUN_DIR}; source ${WS}/scripts/env_common.sh >/dev/null 2>&1 || true; test -f ${ENV_FILE} && source ${ENV_FILE};"

write_service "go2-saas-command" "${common_prefix} touch ${COMMAND_RUN}; exec python3 -u ${WS}/scripts/go2_saas_agent.py command-loop --interval ${COMMAND_INTERVAL} --run-file ${COMMAND_RUN} --seen-file ${SEEN_FILE} --ros-timeout 1.5 --post-timeout ${POST_TIMEOUT} ${EXECUTE_SAFE_ARG}"

write_service "go2-saas-video" "${common_prefix} touch ${VIDEO_RUN}; exec python3 -u ${WS}/scripts/go2_saas_agent.py video-loop --seconds ${VIDEO_SECONDS} --upload --run-file ${VIDEO_RUN}"

write_service "go2-saas-outbox" "${common_prefix} touch ${OUTBOX_RUN}; exec python3 -u ${WS}/scripts/go2_saas_agent.py outbox-loop --interval 10 --max-jobs 2 --run-file ${OUTBOX_RUN}"

sudo_run systemctl daemon-reload
sudo_run systemctl enable --now go2-saas-command.service go2-saas-video.service go2-saas-outbox.service
sudo_run systemctl --no-pager --full status go2-saas-command.service go2-saas-video.service go2-saas-outbox.service || true

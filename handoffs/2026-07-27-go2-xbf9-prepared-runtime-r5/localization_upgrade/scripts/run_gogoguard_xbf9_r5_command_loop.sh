#!/usr/bin/env bash
set -euo pipefail

# Replacement ExecStart for go2-saas-command.service.  The original SaaS agent
# remains untouched and continues to implement transport/heartbeat/results.
# gogoguard_xbf9_r5_agent.py replaces only start_patrol and stop_patrol.

bundle_root="/home/unitree/localization_upgrade"
workspace="${GO2_WS:-/home/unitree/go2_fastlio_ws}"
environment_file="${GO2_SAAS_ENV_FILE:-/home/unitree/.config/go2_saas.env}"
run_dir="${GO2_SAAS_RUN_DIR:-${workspace}/patrol_logs/run}"
command_run="${GO2_SAAS_COMMAND_RUN:-${run_dir}/command.run}"
seen_file="${
  GO2_SAAS_SEEN_FILE:-${workspace}/patrol_logs/go2_saas_seen_commands.json
}"
command_interval="${GO2_SAAS_COMMAND_INTERVAL:-5}"
post_timeout="${GO2_SAAS_POST_TIMEOUT:-5}"

[[ -f "${workspace}/scripts/env_common.sh" ]] ||
  {
    echo "缺少狗端 env_common.sh：${workspace}/scripts/env_common.sh" >&2
    exit 2
  }

# shellcheck disable=SC1090
source "${workspace}/scripts/env_common.sh"
if [[ -f "${environment_file}" ]]; then
  # shellcheck disable=SC1090
  source "${environment_file}"
fi

export GO2_XBF_FIXED_BUNDLE_ROOT="${bundle_root}"
export GO2_XBF_RUNTIME_DIR="${GO2_XBF_RUNTIME_DIR:-/tmp/go2_xbf_patrol}"
export GO2_SAAS_BASE_AGENT="${workspace}/scripts/go2_saas_agent.py"
if [[ "${CYCLONEDDS_URI:-}" == file:///* ]]; then
  export GO2_XBF_CYCLONEDDS_URI="${CYCLONEDDS_URI}"
fi

mkdir -p "${run_dir}" "${GO2_XBF_RUNTIME_DIR}/logs"
touch "${command_run}"

exec python3 -u \
  "${bundle_root}/scripts/gogoguard_xbf9_r5_agent.py" \
  command-loop \
  --interval "${command_interval}" \
  --run-file "${command_run}" \
  --seen-file "${seen_file}" \
  --ros-timeout 1.5 \
  --post-timeout "${post_timeout}" \
  --execute-safe

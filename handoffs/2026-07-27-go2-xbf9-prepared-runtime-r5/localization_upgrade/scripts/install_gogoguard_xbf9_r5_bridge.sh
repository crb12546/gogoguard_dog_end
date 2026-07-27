#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
bundle_root="$(cd -- "${script_dir}/.." && pwd -P)"
active_root="/home/unitree/localization_upgrade"
unit_name="go2-saas-command.service"
dropin_dir="/etc/systemd/system/${unit_name}.d"
dropin_path="${dropin_dir}/90-xbf9-r5.conf"
dropin_source="${bundle_root}/config/go2-saas-command-xbf9-r5.conf"
record_root="/home/unitree/gogoguard_deployments/xbf9-r5-bridge"
record_dir="${record_root}/$(date +%Y%m%dT%H%M%S%z)"
previous_dropin="${record_dir}/previous-90-xbf9-r5.conf"
cutover_started=0
cutover_complete=0

sudo_run() {
  if [[ -n "${SUDO_PASSWORD:-}" ]]; then
    printf '%s\n' "${SUDO_PASSWORD}" | sudo -S -p '' "$@"
  else
    sudo "$@"
  fi
}

restore_command_service_on_error() {
  local exit_code=$?
  if [[ "${cutover_started}" == "1" &&
    "${cutover_complete}" != "1" ]]; then
    set +e
    {
      echo "桥接安装失败，正在恢复安装前的 GoGoGuard command-loop。"
      sudo_run systemctl stop "${unit_name}"
      if [[ -f "${previous_dropin}" ]]; then
        sudo_run install -m 0644 "${previous_dropin}" "${dropin_path}"
      else
        sudo_run rm -f -- "${dropin_path}"
      fi
      sudo_run systemctl daemon-reload
      sudo_run systemctl start "${unit_name}"
      sudo_run systemctl is-active --quiet "${unit_name}"
    } >>"${record_dir}/automatic-rollback.txt" 2>&1
    set -e
  fi
  exit "${exit_code}"
}

[[ "$(readlink -f "${active_root}")" == "${bundle_root}" ]] ||
  {
    echo "错误：${active_root} 没有指向当前 R5 release。" >&2
    echo "current=$(readlink -f "${active_root}" 2>/dev/null || true)" >&2
    echo "expected=${bundle_root}" >&2
    exit 2
  }
[[ -f "${bundle_root}/overlay/install/setup.bash" ]] ||
  {
    echo "错误：R5 overlay 尚未构建。" >&2
    exit 2
  }
[[ -f "${dropin_source}" ]] ||
  {
    echo "错误：缺少 systemd drop-in 模板：${dropin_source}" >&2
    exit 2
  }
systemctl cat "${unit_name}" >/dev/null
python3 "${bundle_root}/scripts/verify_xbf_bundle_offline.py"

mkdir -p "${record_dir}"
systemctl cat "${unit_name}" >"${record_dir}/unit-before.txt"
systemctl show "${unit_name}" \
  -p MainPID -p ActiveState -p SubState -p FragmentPath \
  >"${record_dir}/state-before.txt"
(
  set +u
  # shellcheck disable=SC1090
  source /home/unitree/go2_fastlio_ws/scripts/env_common.sh
  set -u
  GO2_XBF_FIXED_BUNDLE_ROOT="${bundle_root}" \
    GO2_SAAS_BASE_AGENT="/home/unitree/go2_fastlio_ws/scripts/go2_saas_agent.py" \
    python3 "${bundle_root}/scripts/gogoguard_xbf9_r5_agent.py" \
      --bridge-self-check
) >"${record_dir}/bridge-self-check.txt"
if sudo_run test -f "${dropin_path}"; then
  sudo_run cp -a "${dropin_path}" "${previous_dropin}"
fi

# Freeze the platform consumer before changing its entrypoint.  Then stop both
# possible patrol implementations: the old SaaS chain and the fixed XBF chain.
cutover_started=1
trap restore_command_service_on_error EXIT
sudo_run systemctl stop "${unit_name}"
(
  set +u
  # shellcheck disable=SC1090
  source /home/unitree/go2_fastlio_ws/scripts/env_common.sh
  set -u
  python3 /home/unitree/go2_fastlio_ws/scripts/go2_saas_agent.py patrol-stop
) >"${record_dir}/legacy-patrol-stop.txt" 2>&1 || true
bash "${bundle_root}/scripts/stop_xbf_patrol.sh" \
  >"${record_dir}/xbf-patrol-stop.txt" 2>&1 || true
sudo_run install -d -m 0755 "${dropin_dir}"
sudo_run install -m 0644 "${dropin_source}" "${dropin_path}"
sudo_run systemctl daemon-reload
sudo_run systemctl start "${unit_name}"
sudo_run systemctl is-active --quiet "${unit_name}"

systemctl cat "${unit_name}" >"${record_dir}/unit-after.txt"
systemctl show "${unit_name}" \
  -p MainPID -p ActiveState -p SubState -p FragmentPath \
  >"${record_dir}/state-after.txt"
main_pid="$(systemctl show "${unit_name}" -p MainPID --value)"
[[ "${main_pid}" =~ ^[1-9][0-9]*$ ]] ||
  {
    echo "错误：桥接后的 command service 没有有效 MainPID。" >&2
    exit 3
  }
for _ in $(seq 1 50); do
  if [[ -r "/proc/${main_pid}/cmdline" ]] &&
    tr '\0' ' ' <"/proc/${main_pid}/cmdline" |
      grep -Fq "gogoguard_xbf9_r5_agent.py"; then
    echo "GoGoGuard 固定任务桥接已启用。"
    echo "下一条 start_patrol 将忽略 CSV/URL 并启动 ${active_root}。"
    echo "记录：${record_dir}"
    cutover_complete=1
    trap - EXIT
    exit 0
  fi
  sleep 0.1
done

echo "错误：command service 已启动，但主进程不是 XBF9 R5 bridge。" >&2
exit 3

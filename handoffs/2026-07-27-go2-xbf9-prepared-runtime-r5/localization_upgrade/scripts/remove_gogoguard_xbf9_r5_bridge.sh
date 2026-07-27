#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
bundle_root="$(cd -- "${script_dir}/.." && pwd -P)"
unit_name="go2-saas-command.service"
dropin_path="/etc/systemd/system/${unit_name}.d/90-xbf9-r5.conf"

sudo_run() {
  if [[ -n "${SUDO_PASSWORD:-}" ]]; then
    printf '%s\n' "${SUDO_PASSWORD}" | sudo -S -p '' "$@"
  else
    sudo "$@"
  fi
}

bash "${bundle_root}/scripts/stop_xbf_patrol.sh" || true
sudo_run systemctl stop "${unit_name}"
sudo_run rm -f -- "${dropin_path}"
sudo_run systemctl daemon-reload
sudo_run systemctl start "${unit_name}"
sudo_run systemctl is-active --quiet "${unit_name}"

main_pid="$(systemctl show "${unit_name}" -p MainPID --value)"
if [[ "${main_pid}" =~ ^[1-9][0-9]*$ ]] &&
  [[ -r "/proc/${main_pid}/cmdline" ]] &&
  tr '\0' ' ' <"/proc/${main_pid}/cmdline" |
    grep -Fq "gogoguard_xbf9_r5_agent.py"; then
  echo "错误：drop-in 已移除，但 command service 仍在运行 XBF bridge。" >&2
  exit 3
fi
echo "GoGoGuard 固定任务桥接已移除，原 command-loop 已恢复。"

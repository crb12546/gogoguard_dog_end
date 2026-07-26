#!/usr/bin/env bash
set -euo pipefail

# shellcheck source=_xbf_patrol_common.sh
source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)/_xbf_patrol_common.sh"

pid_file="${XBF_RUNTIME_DIR}/patrol.pids"

pid_value() {
  local key="$1"
  awk -F= -v wanted="${key}" '$1 == wanted {print $2}' "${pid_file}"
}

wait_for_exit() {
  local pid="$1"
  local attempt
  for ((attempt = 1; attempt <= 35; attempt++)); do
    kill -0 "${pid}" 2>/dev/null || return 0
    sleep 1
  done
  return 1
}

stop_exact_group() {
  local pgid="$1"
  local signal_name="$2"
  [[ "${pgid}" =~ ^[0-9]+$ ]] || return 0
  kill -0 -- "-${pgid}" 2>/dev/null || return 0
  kill "-${signal_name}" -- "-${pgid}" 2>/dev/null || true
}

send_final_stopmove() {
  local probe="$1"
  local interface="$2"
  [[ -x "${probe}" ]] || {
    echo "警告：找不到可执行 StopMove probe：${probe:-<空>}" >&2
    return 0
  }
  [[ "${interface}" =~ ^[A-Za-z0-9_.:-]+$ ]] || {
    echo "警告：记录的 SDK 网卡名无效：${interface:-<空>}" >&2
    return 0
  }
  echo "通过 SDK2 motion probe 发送最终 StopMove……"
  "${probe}" --iface "${interface}" stop \
    >>"${XBF_RUNTIME_DIR}/logs/stopmove.log" 2>&1 || {
      echo "警告：StopMove probe 返回失败；请立即人工确认机器狗已停止。" >&2
      return 0
    }
}

if [[ ! -f "${pid_file}" ]]; then
  echo "没有找到运行记录：${pid_file}"
  echo "未使用 pkill，避免误停狗端其他 ROS 进程。"
  exit 0
fi

supervisor_pid="$(pid_value supervisor_pid)"
localizer_pgid="$(pid_value localizer_pgid)"
coordinator_pgid="$(pid_value coordinator_pgid)"
sdk_receiver_pgid="$(pid_value sdk_receiver_pgid)"
cmd_vel_sender_pgid="$(pid_value cmd_vel_sender_pgid)"
safe_cmd_pgid="$(pid_value safe_cmd_pgid)"
follower_pgid="$(pid_value follower_pgid)"
sdk_interface="$(pid_value sdk_interface)"
motion_probe="$(pid_value motion_probe)"

if [[ "${supervisor_pid}" =~ ^[0-9]+$ ]] &&
  kill -0 "${supervisor_pid}" 2>/dev/null; then
  echo "通知 XBF 巡检监督进程按顺序停车……"
  kill -INT "${supervisor_pid}" 2>/dev/null || true
  if wait_for_exit "${supervisor_pid}"; then
    echo "XBF 巡检已停止。"
    exit 0
  fi
  echo "监督进程未及时退出，改用记录的精确 PID 停止。" >&2
fi

# 监督脚本已经不在时仍保持相同顺序：先停 follower，让协调器、safe cmd
# 和 UDP 链持续发送零；随后再自上而下断开控制链，最后用 SDK2 明确 StopMove。
stop_exact_group "${follower_pgid}" INT
sleep 2
stop_exact_group "${localizer_pgid}" INT
sleep 1
stop_exact_group "${coordinator_pgid}" INT
sleep 1
stop_exact_group "${safe_cmd_pgid}" INT
stop_exact_group "${cmd_vel_sender_pgid}" INT
stop_exact_group "${sdk_receiver_pgid}" INT

for pgid in \
  "${follower_pgid}" \
  "${localizer_pgid}" \
  "${coordinator_pgid}" \
  "${safe_cmd_pgid}" \
  "${cmd_vel_sender_pgid}" \
  "${sdk_receiver_pgid}"; do
  if [[ "${pgid}" =~ ^[0-9]+$ ]] &&
    kill -0 -- "-${pgid}" 2>/dev/null; then
    kill -TERM -- "-${pgid}" 2>/dev/null || true
  fi
done
sleep 1

send_final_stopmove "${motion_probe}" "${sdk_interface}"
rm -f -- "${pid_file}" "${XBF_RUNTIME_DIR}/localizer.runtime.yaml"
echo "XBF 巡检已停止；没有使用广域 pkill。"

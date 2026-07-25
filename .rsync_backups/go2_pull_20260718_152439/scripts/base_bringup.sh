#!/usr/bin/env bash
set -e

source /home/unitree/go2_fastlio_ws/scripts/env_common.sh

mkdir -p "${LOG_DIR}"


ensure_lidar_network() {
  local iface="eth0"
  local ip_cidr="192.168.1.5/24"

  echo "[base_bringup] checking lidar network ${iface} ${ip_cidr} ..."

  if ! ip link show "${iface}" >/dev/null 2>&1; then
    echo "[base_bringup] ERROR: network interface ${iface} not found."
    echo "[base_bringup] Please check the Ethernet interface name."
    return 1
  fi

  if ip -4 addr show dev "${iface}" | grep -q "192.168.1.5/24"; then
    echo "[base_bringup] ${iface} already has ${ip_cidr}"
    return 0
  fi

  echo "[base_bringup] adding ${ip_cidr} to ${iface} ..."

  if command -v sudo >/dev/null 2>&1; then
    sudo -n ip link set "${iface}" up || true
    sudo -n ip addr add "${ip_cidr}" dev "${iface}" || true
  else
    ip link set "${iface}" up || true
    ip addr add "${ip_cidr}" dev "${iface}" || true
  fi

  if ip -4 addr show dev "${iface}" | grep -q "192.168.1.5/24"; then
    echo "[base_bringup] lidar network ready:"
    ip -4 addr show dev "${iface}"
    return 0
  fi

  echo "[base_bringup] WARNING: failed to add ${ip_cidr} to ${iface}."
  echo "[base_bringup] If running as user service, install go2-lidar-network.service first."
  return 1
}


wait_topic() {
  local topic="$1"
  local timeout_sec="$2"
  local start_time
  start_time=$(date +%s)

  echo "[base_bringup] waiting for topic: ${topic}"

  while true; do
    if ros2 topic list 2>/dev/null | grep -qx "${topic}"; then
      echo "[base_bringup] topic ready: ${topic}"
      return 0
    fi

    now=$(date +%s)
    if [ $((now - start_time)) -ge "${timeout_sec}" ]; then
      echo "[base_bringup] ERROR: timeout waiting for ${topic}"
      return 1
    fi

    sleep 1
  done
}

cleanup_children() {
  echo "[base_bringup] received exit signal, stopping children..."
  pkill -INT -f "ros2 launch fast_lio mapping.launch.py"
  pkill -INT -f "ros2 launch livox_ros_driver2 msg_MID360s_launch.py"
  sleep 1
}

trap cleanup_children INT TERM EXIT

ensure_lidar_network || true

echo "[base_bringup] cleaning old base processes..."
pkill -INT -f "ros2 launch fast_lio mapping.launch.py" || true
pkill -INT -f "ros2 launch livox_ros_driver2 msg_MID360s_launch.py" || true
sleep 1

echo "[base_bringup] starting Livox MID-360S driver..."
cd "${WS}"
bash -lc "source ${WS}/scripts/env_common.sh && ros2 launch livox_ros_driver2 msg_MID360s_launch.py" \
  > "${LOG_DIR}/livox.log" 2>&1 &

LIVOX_PID=$!
echo "[base_bringup] Livox PID: ${LIVOX_PID}"

wait_topic "/livox/lidar" 60
wait_topic "/livox/imu" 60

echo "[base_bringup] Livox is ready. Starting FAST-LIO..."
bash -lc "source ${WS}/scripts/env_common.sh && ros2 launch fast_lio mapping.launch.py config_file:=go2_mid360s.yaml rviz:=false" \
  > "${LOG_DIR}/fast_lio.log" 2>&1 &

FASTLIO_PID=$!
echo "[base_bringup] FAST-LIO PID: ${FASTLIO_PID}"

wait_topic "/Odometry" 90

echo "[base_bringup] FAST-LIO is ready."
echo "[base_bringup] logs:"
echo "  ${LOG_DIR}/livox.log"
echo "  ${LOG_DIR}/fast_lio.log"

wait ${LIVOX_PID} ${FASTLIO_PID}

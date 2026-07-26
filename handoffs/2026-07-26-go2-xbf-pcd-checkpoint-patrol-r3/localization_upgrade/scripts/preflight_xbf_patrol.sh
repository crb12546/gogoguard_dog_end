#!/usr/bin/env bash
set -euo pipefail

# shellcheck source=_xbf_patrol_common.sh
source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)/_xbf_patrol_common.sh"

usage() {
  cat >&2 <<'EOF'
用法：
  preflight_xbf_patrol.sh [reviewed-map目录] [路线CSV] [checkpoint.json]

默认地图：
  maps/xbf-2026-07-26-map-reviewed-r2
默认路线：
  routes/xbf9_horizontal_clean.map-reviewed-r2.csv
默认 checkpoint：
  routes/xbf9_horizontal_clean.map-reviewed-r2.checkpoints.json
EOF
}

if [[ $# -gt 3 ]]; then
  usage
  exit 2
fi

xbf_resolve_inputs "$@"
xbf_source_runtime

for command_name in ros2 python3 sha256sum awk grep setsid ps ip ss; do
  command -v "${command_name}" >/dev/null 2>&1 ||
    xbf_fail "找不到命令：${command_name}"
done

echo "[1/7] 校验地图 manifest 与 reviewed publication 哈希链……"
expected_manifest_sha="$(
  awk 'NR == 1 {print tolower($1)}' "${XBF_MAP_MANIFEST_CHECKSUM}"
)"
actual_manifest_sha="$(sha256sum "${XBF_MAP_MANIFEST}" | awk '{print $1}')"
[[ "${expected_manifest_sha}" =~ ^[0-9a-f]{64}$ ]] ||
  xbf_fail "manifest.sha256 第一列不是 SHA-256"
[[ "${actual_manifest_sha}" == "${expected_manifest_sha}" ]] ||
  xbf_fail "manifest.json 与 manifest.sha256 不一致"
ros2 run go2_map_tools go2-map verify-reviewed "${XBF_MAP_ROOT}" >/dev/null

echo "[2/7] 校验路线、checkpoint 与地图来源绑定……"
identity_output="$(
  python3 - "${XBF_MAP_MANIFEST}" "${XBF_MAP_PUBLICATION}" \
    "${XBF_ROUTE_FILE}" "${XBF_CHECKPOINT_FILE}" \
    "${XBF_ROUTE_METADATA}" <<'PY'
import hashlib
import json
import re
import sys
from pathlib import Path

(
    manifest_path,
    publication_path,
    route_path,
    checkpoint_path,
    route_metadata_path,
) = map(Path, sys.argv[1:])
manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
publication = json.loads(publication_path.read_text(encoding="utf-8"))
sidecar = json.loads(checkpoint_path.read_text(encoding="utf-8"))
route_metadata = json.loads(route_metadata_path.read_text(encoding="utf-8"))

required = {
    "schema",
    "source_pcd_sha256",
    "source_csv_sha256",
    "route_csv_sha256",
    "route_revision",
    "checkpoints",
}
if set(sidecar) != required:
    raise SystemExit("checkpoint sidecar 根字段不符合 v1 合约")
if sidecar["schema"] != "go2.route_checkpoints/v1":
    raise SystemExit("checkpoint sidecar schema 错误")
for key in ("source_pcd_sha256", "source_csv_sha256", "route_csv_sha256"):
    if not isinstance(sidecar[key], str) or not re.fullmatch(r"[0-9a-f]{64}", sidecar[key]):
        raise SystemExit(f"checkpoint sidecar {key} 不是小写 SHA-256")
route_sha = hashlib.sha256(route_path.read_bytes()).hexdigest()
if route_sha != sidecar["route_csv_sha256"]:
    raise SystemExit("路线原始字节 SHA-256 与 checkpoint sidecar 不一致")
checkpoint_sha = hashlib.sha256(checkpoint_path.read_bytes()).hexdigest()
publication_pcd = publication.get("source_pcd", {}).get("sha256")
if publication_pcd != sidecar["source_pcd_sha256"]:
    raise SystemExit("checkpoint 的 source PCD 与 reviewed map 不一致")
if manifest.get("map_id") != publication.get("map_id"):
    raise SystemExit("manifest map_id 与 reviewed publication 不一致")
compiled_sha = publication.get("compiled_map", {}).get("manifest_sha256")
actual_manifest_sha = hashlib.sha256(manifest_path.read_bytes()).hexdigest()
if compiled_sha != actual_manifest_sha:
    raise SystemExit("reviewed publication 未绑定当前 manifest")
if route_metadata.get("schema") != "go2.deployment_route/v1":
    raise SystemExit("路线部署元数据 schema 错误")
if route_metadata.get("route_file") != route_path.name:
    raise SystemExit("路线部署元数据没有绑定当前 CSV 文件名")
if route_metadata.get("route_csv_sha256") != route_sha:
    raise SystemExit("路线部署元数据没有绑定当前 CSV 原始字节")
if route_metadata.get("checkpoint_file") != checkpoint_path.name:
    raise SystemExit("路线部署元数据没有绑定当前 checkpoint 文件名")
if route_metadata.get("checkpoint_sha256") != checkpoint_sha:
    raise SystemExit("路线部署元数据没有绑定当前 checkpoint 原始字节")
metadata_source = route_metadata.get("source", {})
if metadata_source.get("pcd_sha256") != sidecar["source_pcd_sha256"]:
    raise SystemExit("路线部署元数据与 checkpoint 的源 PCD 不一致")
if metadata_source.get("csv_sha256") != sidecar["source_csv_sha256"]:
    raise SystemExit("路线部署元数据与 checkpoint 的源 CSV 不一致")
metadata_map = route_metadata.get("map", {})
if metadata_map.get("map_id") != manifest.get("map_id"):
    raise SystemExit("路线部署元数据没有绑定当前 map_id")
if metadata_map.get("manifest_sha256") != actual_manifest_sha:
    raise SystemExit("路线部署元数据没有绑定当前 manifest SHA-256")
alignment = route_metadata.get("alignment", {})
if alignment.get("type") != "SE2":
    raise SystemExit("路线部署元数据缺少 SE2 对齐记录")
checkpoints = sidecar.get("checkpoints")
if not isinstance(checkpoints, list) or not checkpoints:
    raise SystemExit("checkpoint sidecar 没有 checkpoint")
print(manifest["map_id"])
print(sidecar["source_csv_sha256"])
print(sidecar["source_pcd_sha256"])
print(len(checkpoints))
print(str(bool(publication.get("deployment_ready", False))).lower())
print(str(bool(alignment.get("field_truth_verified", False))).lower())
PY
)"
map_id="$(printf '%s\n' "${identity_output}" | sed -n '1p')"
source_csv_sha="$(printf '%s\n' "${identity_output}" | sed -n '2p')"
source_pcd_sha="$(printf '%s\n' "${identity_output}" | sed -n '3p')"
checkpoint_count="$(printf '%s\n' "${identity_output}" | sed -n '4p')"
deployment_ready="$(printf '%s\n' "${identity_output}" | sed -n '5p')"
field_truth_verified="$(printf '%s\n' "${identity_output}" | sed -n '6p')"

# 使用即将部署的纯 Python 解析器再做一次严格五列、id/index、速度与 sidecar 检查。
python3 - "${XBF_ROUTE_FILE}" "${XBF_CHECKPOINT_FILE}" \
  "${source_csv_sha}" "${source_pcd_sha}" <<'PY'
import sys
from go2_checkpoint_patrol.checkpoint_core import load_route

route = load_route(
    sys.argv[1],
    default_checkpoint_radius_m=0.60,
    default_search_radius_m=12.0,
    checkpoint_file=sys.argv[2],
    expected_source_csv_sha256=sys.argv[3],
    expected_source_pcd_sha256=sys.argv[4],
)
count = sum(point.is_checkpoint for point in route)
if count < 1:
    raise SystemExit("严格解析后没有 checkpoint")
print(f"  严格解析通过：{len(route)} 个路线点，{count} 个 checkpoint")
PY

if [[ "${deployment_ready}" != "true" ]]; then
  echo "  提示：reviewed publication 的 deployment_ready=false。" >&2
  echo "        当前工具把它作为发布记录，不作为运行时开关；请确认这是预期地图版本。" >&2
fi
if [[ "${field_truth_verified}" != "true" ]]; then
  echo "  提示：CSV→PCD 的平移尚未用现场真值复核。" >&2
  echo "        首次部署先用 GO2_XBF_CALIBRATION_ONLY=1 静止核对，再做短距离低速测试。" >&2
fi

echo "[3/7] 校验 ROS 包与可执行程序……"
ros2 pkg prefix go2_map_localizer >/dev/null
ros2 pkg prefix go2_checkpoint_patrol >/dev/null
ros2 pkg prefix go2_fastlio_patrol >/dev/null
ros2 pkg prefix go2_cmd_vel_bridge >/dev/null
ros2 pkg executables go2_checkpoint_patrol |
  grep -Eq '(^|[[:space:]])checkpoint-localization-coordinator$' ||
  xbf_fail "未找到 checkpoint-localization-coordinator"
ros2 pkg executables go2_fastlio_patrol |
  grep -Eq '(^|[[:space:]])waypoint_follower_go2_2$' ||
  xbf_fail "未找到原 waypoint_follower_go2_2"
ros2 pkg executables go2_fastlio_patrol |
  grep -Eq '(^|[[:space:]])unitree_safe_cmd_node$' ||
  xbf_fail "未找到原 unitree_safe_cmd_node"
ros2 pkg executables go2_cmd_vel_bridge |
  grep -Eq '(^|[[:space:]])cmd_vel_udp_sender$' ||
  xbf_fail "未找到 go2_cmd_vel_bridge/cmd_vel_udp_sender"
ros2 pkg executables go2_cmd_vel_bridge |
  grep -Eq '(^|[[:space:]])go2_sdk2_udp_receiver$' ||
  xbf_fail "未找到 go2_cmd_vel_bridge/go2_sdk2_udp_receiver"
follower_source="$(
  xbf_absolute_file \
    "${XBF_FASTLIO_WORKSPACE}/src/go2_fastlio_patrol/go2_fastlio_patrol/waypoint_follower_go2_2.py"
)" || xbf_fail "找不到狗端当前 waypoint_follower_go2_2.py"
safe_cmd_source="$(
  xbf_absolute_file \
    "${XBF_FASTLIO_WORKSPACE}/src/go2_fastlio_patrol/go2_fastlio_patrol/unitree_safe_cmd_node.py"
)" || xbf_fail "找不到狗端当前 unitree_safe_cmd_node.py"
[[ "$(sha256sum "${follower_source}" | awk '{print $1}')" == \
  "d205a596fc6118ad7fa191871c646173cb545ab4b136e46de360598e38261120" ]] ||
  xbf_fail "狗端 waypoint_follower_go2_2.py 已变化；不能假设当前接线/参数仍一致"
[[ "$(sha256sum "${safe_cmd_source}" | awk '{print $1}')" == \
  "c80902bbebd52fbe90e1d655dd04e5d5f0625a5de305e754295004d9c7be9e1b" ]] ||
  xbf_fail "狗端 unitree_safe_cmd_node.py 已变化；不能假设当前运动输出仍一致"
sender_source="$(
  xbf_absolute_file \
    "${XBF_FASTLIO_WORKSPACE}/src/go2_cmd_vel_bridge/src/cmd_vel_udp_sender.cpp"
)" || xbf_fail "找不到狗端当前 cmd_vel_udp_sender.cpp"
receiver_source="$(
  xbf_absolute_file \
    "${XBF_FASTLIO_WORKSPACE}/src/go2_cmd_vel_bridge/src/go2_sdk2_udp_receiver.cpp"
)" || xbf_fail "找不到狗端当前 go2_sdk2_udp_receiver.cpp"
motion_probe_source="$(
  xbf_absolute_file \
    "${XBF_FASTLIO_WORKSPACE}/src/go2_cmd_vel_bridge/src/go2_sdk2_motion_probe.cpp"
)" || xbf_fail "找不到狗端当前 go2_sdk2_motion_probe.cpp"
[[ "$(sha256sum "${sender_source}" | awk '{print $1}')" == \
  "d87c2121624c8896df4823efeee87071b6c88877915492a7c6ae36bfb8d83bdb" ]] ||
  xbf_fail "狗端 cmd_vel_udp_sender.cpp 已变化；不能假设 UDP 包格式与限幅一致"
[[ "$(sha256sum "${receiver_source}" | awk '{print $1}')" == \
  "94aa743fc0dfe7b4d040c067e97c2e7a5e676d6871d05e2cf859f60d87b02a12" ]] ||
  xbf_fail "狗端 go2_sdk2_udp_receiver.cpp 已变化；不能假设 SDK2 输出一致"
[[ "$(sha256sum "${motion_probe_source}" | awk '{print $1}')" == \
  "1a605aa25c4cc2ede6bd0674b44931c18136d5fee2279fcfee9d5fddcf3daf85" ]] ||
  xbf_fail "狗端 go2_sdk2_motion_probe.cpp 已变化；不能保证退出时 StopMove"

XBF_SDK_INTERFACE="${GO2_SDK_IF:-eth0}"
[[ "${XBF_SDK_INTERFACE}" =~ ^[A-Za-z0-9_.:-]+$ ]] ||
  xbf_fail "GO2_SDK_IF 含非法字符：${XBF_SDK_INTERFACE}"
ip link show dev "${XBF_SDK_INTERFACE}" >/dev/null 2>&1 ||
  xbf_fail "找不到 GO2 SDK 网卡：${XBF_SDK_INTERFACE}"
XBF_SDK_RECEIVER="$(
  xbf_absolute_file \
    "${XBF_FASTLIO_WORKSPACE}/build/go2_cmd_vel_bridge/go2_sdk2_udp_receiver"
)" || xbf_fail "找不到狗端 SDK2 UDP receiver 二进制"
XBF_MOTION_PROBE="$(
  xbf_absolute_file \
    "${XBF_FASTLIO_WORKSPACE}/build/go2_cmd_vel_bridge/go2_sdk2_motion_probe"
)" || xbf_fail "找不到狗端 SDK2 StopMove probe 二进制"
[[ -x "${XBF_SDK_RECEIVER}" ]] ||
  xbf_fail "SDK2 UDP receiver 不可执行：${XBF_SDK_RECEIVER}"
[[ -x "${XBF_MOTION_PROBE}" ]] ||
  xbf_fail "SDK2 motion probe 不可执行：${XBF_MOTION_PROBE}"

echo "[4/7] 校验 FAST-LIO 输入 Topic……"
odom_type="$(ros2 topic type /Odometry 2>/dev/null | head -n 1 || true)"
cloud_type="$(
  ros2 topic type /cloud_registered_body 2>/dev/null | head -n 1 || true
)"
[[ "${odom_type}" == "nav_msgs/msg/Odometry" ]] ||
  xbf_fail "/Odometry 类型错误或不存在：${odom_type:-<空>}"
[[ "${cloud_type}" == "sensor_msgs/msg/PointCloud2" ]] ||
  xbf_fail "/cloud_registered_body 类型错误或不存在：${cloud_type:-<空>}"

echo "[5/7] 校验没有旧巡检、UDP 运动桥或定位实例……"
for topic_spec in \
  "/patrol_cmd:publisher" \
  "/patrol_cmd:subscriber" \
  "/cmd_vel:publisher" \
  "/checkpoint_localization/gated_cmd:publisher" \
  "/checkpoint_localization/follower_cmd:publisher" \
  "/localization/status:publisher" \
  "/localization/odometry:publisher" \
  "/checkpoint_localization/aligned_odometry:publisher"; do
  topic_name="${topic_spec%:*}"
  topic_kind="${topic_spec#*:}"
  topic_count="$(xbf_topic_count "${topic_name}" "${topic_kind}")"
  [[ "${topic_count}" == "0" ]] ||
    xbf_fail "${topic_name} 已有 ${topic_count} 个 ${topic_kind}；请先停止旧 SaaS/XBF 任务"
done
for service_name in \
  /localization/set_active \
  /localization/reset \
  /localization/global_relocalize \
  /checkpoint_localization/retry_after_fault; do
  if ros2 service list 2>/dev/null | grep -Fxq "${service_name}"; then
    xbf_fail "发现旧定位服务 ${service_name}；请先停止 shadow/旧 XBF localizer"
  fi
done
if ss -H -lun 2>/dev/null |
  awk '$5 ~ /:5005$/ {found=1} END {exit found ? 0 : 1}'; then
  xbf_fail "UDP 5005 已被占用；请先停止旧 go2_sdk2_udp_receiver"
fi

echo "[6/7] 校验真实狗端 UDP→SDK2 输出链准备完成……"
echo "  GO2 SDK 网卡：${XBF_SDK_INTERFACE}"
echo "  UDP receiver：${XBF_SDK_RECEIVER}"
echo "  StopMove probe：${XBF_MOTION_PROBE}"

echo "[7/7] 预检通过"
echo "  map_id: ${map_id}"
echo "  manifest_sha256: ${actual_manifest_sha}"
echo "  route: ${XBF_ROUTE_FILE}"
echo "  checkpoints: ${checkpoint_count}"
echo "  source_csv_sha256: ${source_csv_sha}"
echo "  source_pcd_sha256: ${source_pcd_sha}"

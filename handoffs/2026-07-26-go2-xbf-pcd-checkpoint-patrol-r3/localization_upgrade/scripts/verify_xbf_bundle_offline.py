#!/usr/bin/env python3
"""Offline integrity and wiring checks for the bundled XBF patrol task.

This program deliberately does not import rclpy, create a ROS node, or publish
motion.  It verifies the immutable map, route/checkpoint identities, selected
checkpoint set, production parameter intent, and the known-good follower
source snapshot.
"""

from __future__ import annotations

import hashlib
import json
import os
import sys
from pathlib import Path


EXPECTED_FOLLOWER_SHA256 = (
    "d205a596fc6118ad7fa191871c646173cb545ab4b136e46de360598e38261120"
)
EXPECTED_SAFE_CMD_SHA256 = (
    "c80902bbebd52fbe90e1d655dd04e5d5f0625a5de305e754295004d9c7be9e1b"
)
EXPECTED_CMD_VEL_UDP_SENDER_SHA256 = (
    "d87c2121624c8896df4823efeee87071b6c88877915492a7c6ae36bfb8d83bdb"
)
EXPECTED_SDK2_UDP_RECEIVER_SHA256 = (
    "94aa743fc0dfe7b4d040c067e97c2e7a5e676d6871d05e2cf859f60d87b02a12"
)
EXPECTED_SDK2_MOTION_PROBE_SHA256 = (
    "1a605aa25c4cc2ede6bd0674b44931c18136d5fee2279fcfee9d5fddcf3daf85"
)
EXPECTED_ROUTE_SHA256 = (
    "6fafe8b87e50ad0ad1a3f3aac671373d62120df904e7ffd86dfcb09ba4b211a4"
)
EXPECTED_PCD_SHA256 = (
    "3526e4f116586d3594c0afa45efb3fb254e4eca1bf89fa21f18896a558ee5aa2"
)
EXPECTED_SOURCE_CSV_SHA256 = (
    "b4abadd38c30f5904f4cfe10eb529b8c1a4940ba023019847ea3959c48fd53a2"
)
EXPECTED_MAP_ID = "xbf-2026-07-26-map-reviewed-r2"
EXPECTED_CHECKPOINTS = (373, 585, 787)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def require(condition: bool, message: str) -> None:
    if not condition:
        raise RuntimeError(message)


def find_workspace_file(bundle_root: Path, relative: Path) -> Path:
    candidates = []
    explicit_root = os.environ.get("GO2_XBF_REAL_DOG_ROOT", "")
    if explicit_root:
        candidates.append(Path(explicit_root) / relative)
    candidates.append(bundle_root.parent / relative)
    fastlio_root = Path(
        os.environ.get("GO2_FASTLIO_WS", "/home/unitree/go2_fastlio_ws")
    )
    candidates.append(fastlio_root / relative)
    candidates.append(
        bundle_root.parent.parent / "orin_go2_fastlio_ws" / relative
    )
    # A handoff archive may be verified on a laptop that does not contain the
    # original robot workspace.  This reference is read-only evidence for the
    # offline hash audit; the online preflight never falls back to it and still
    # verifies /home/unitree/go2_fastlio_ws itself.
    candidates.append(
        bundle_root / "reference/known_good_workspace" / relative
    )
    for candidate in candidates:
        if candidate.is_file():
            return candidate
    rendered = ", ".join(str(path) for path in candidates)
    raise RuntimeError(
        f"cannot find current dog file {relative}; checked: {rendered}"
    )


def main() -> int:
    bundle_root = Path(__file__).resolve().parent.parent
    map_tools_source = bundle_root / "overlay/src/go2_map_tools"
    checkpoint_source = bundle_root / "overlay/src/go2_checkpoint_patrol"
    sys.path[:0] = [str(map_tools_source), str(checkpoint_source)]

    from go2_checkpoint_patrol.checkpoint_core import load_route
    from go2_map_tools.reviewed_publish import verify_reviewed_map_bundle

    map_root = bundle_root / "maps" / EXPECTED_MAP_ID
    route_file = (
        bundle_root / "routes/xbf9_horizontal_clean.map-reviewed-r2.csv"
    )
    checkpoint_file = (
        bundle_root
        / "routes/xbf9_horizontal_clean.map-reviewed-r2.checkpoints.json"
    )
    route_metadata_file = (
        bundle_root / "routes/xbf9_horizontal_clean.map-reviewed-r2.route.json"
    )
    follower_file = find_workspace_file(
        bundle_root,
        Path(
            "src/go2_fastlio_patrol/go2_fastlio_patrol/"
            "waypoint_follower_go2_2.py"
        ),
    )
    safe_cmd_file = find_workspace_file(
        bundle_root,
        Path(
            "src/go2_fastlio_patrol/go2_fastlio_patrol/"
            "unitree_safe_cmd_node.py"
        ),
    )
    sender_file = find_workspace_file(
        bundle_root,
        Path("src/go2_cmd_vel_bridge/src/cmd_vel_udp_sender.cpp"),
    )
    receiver_file = find_workspace_file(
        bundle_root,
        Path("src/go2_cmd_vel_bridge/src/go2_sdk2_udp_receiver.cpp"),
    )
    motion_probe_file = find_workspace_file(
        bundle_root,
        Path("src/go2_cmd_vel_bridge/src/go2_sdk2_motion_probe.cpp"),
    )
    production_localizer = bundle_root / "config/localizer-u2-production.yaml"
    production_coordinator = (
        bundle_root
        / "overlay/src/go2_checkpoint_patrol/config/"
        "checkpoint-coordinator.production.yaml"
    )
    start_script = bundle_root / "scripts/start_xbf_patrol.sh"
    stop_script = bundle_root / "scripts/stop_xbf_patrol.sh"

    publication = verify_reviewed_map_bundle(map_root)
    require(publication["map_id"] == EXPECTED_MAP_ID, "unexpected map_id")
    require(
        publication["source_pcd"]["sha256"] == EXPECTED_PCD_SHA256,
        "reviewed map source PCD mismatch",
    )
    require(
        publication["annotation"]["revision"] == "xbf-algorithm-reviewed-r2",
        "unexpected annotation revision",
    )
    require(
        publication["stable_layer"]["point_count"] >= 1000,
        "stable layer is unexpectedly small",
    )

    sidecar = json.loads(checkpoint_file.read_text(encoding="utf-8"))
    require(
        sidecar["route_csv_sha256"] == EXPECTED_ROUTE_SHA256,
        "sidecar route hash mismatch",
    )
    require(
        sidecar["source_pcd_sha256"] == EXPECTED_PCD_SHA256,
        "sidecar source PCD mismatch",
    )
    require(
        sidecar["source_csv_sha256"] == EXPECTED_SOURCE_CSV_SHA256,
        "sidecar source CSV mismatch",
    )
    require(
        sha256_file(route_file) == EXPECTED_ROUTE_SHA256,
        "route bytes changed",
    )
    route = load_route(
        str(route_file),
        default_checkpoint_radius_m=0.60,
        default_search_radius_m=12.0,
        checkpoint_file=str(checkpoint_file),
        expected_source_csv_sha256=EXPECTED_SOURCE_CSV_SHA256,
        expected_source_pcd_sha256=EXPECTED_PCD_SHA256,
    )
    checkpoint_indexes = tuple(
        point.index for point in route if point.is_checkpoint
    )
    require(
        checkpoint_indexes == EXPECTED_CHECKPOINTS,
        "checkpoint indexes are not the reviewed set",
    )

    route_metadata = json.loads(route_metadata_file.read_text(encoding="utf-8"))
    require(
        route_metadata["route_csv_sha256"] == sha256_file(route_file),
        "route metadata does not bind route bytes",
    )
    require(
        route_metadata["checkpoint_sha256"] == sha256_file(checkpoint_file),
        "route metadata does not bind checkpoint bytes",
    )
    require(
        route_metadata["map"]["map_id"] == EXPECTED_MAP_ID,
        "route metadata map_id mismatch",
    )
    require(
        route_metadata["map"]["manifest_sha256"]
        == publication["compiled_map"]["manifest_sha256"],
        "route metadata manifest hash mismatch",
    )

    require(
        sha256_file(follower_file) == EXPECTED_FOLLOWER_SHA256,
        "known-good real-dog follower source changed",
    )
    require(
        sha256_file(safe_cmd_file) == EXPECTED_SAFE_CMD_SHA256,
        "known-good real-dog safe command source changed",
    )
    require(
        sha256_file(sender_file) == EXPECTED_CMD_VEL_UDP_SENDER_SHA256,
        "known-good cmd_vel UDP sender source changed",
    )
    require(
        sha256_file(receiver_file) == EXPECTED_SDK2_UDP_RECEIVER_SHA256,
        "known-good SDK2 UDP receiver source changed",
    )
    require(
        sha256_file(motion_probe_file)
        == EXPECTED_SDK2_MOTION_PROBE_SHA256,
        "known-good SDK2 motion probe source changed",
    )
    localizer_text = production_localizer.read_text(encoding="utf-8")
    coordinator_text = production_coordinator.read_text(encoding="utf-8")
    for token in (
        "input_extrinsics_verified: false",
        "tracking:\n      enabled: false",
        "auto_relocalize: false",
        "anchored:",
        "maximum_registration_attempts: 36",
    ):
        require(token in localizer_text, "production localizer missing " + token)
    for token in (
        "integration_enabled: true",
        "gated_cmd_topic: /checkpoint_localization/gated_cmd",
        "graph_guard_enabled: true",
        "checkpoint_default_radius_m: 0.60",
        "checkpoint_default_search_radius_m: 12.0",
    ):
        require(token in coordinator_text, "production coordinator missing " + token)

    start_text = start_script.read_text(encoding="utf-8")
    stop_text = stop_script.read_text(encoding="utf-8")
    for token in (
        'sdk_interface="${GO2_SDK_IF:-eth0}"',
        "go2_sdk2_udp_receiver",
        '"${sdk_receiver}" "${sdk_interface}" 5005',
        "wait_for_udp_receiver 10",
        "cmd_vel_udp_sender",
        "-p target_ip:=127.0.0.1",
        "-p target_port:=5005",
        "-p cmd_topic:=/checkpoint_localization/gated_cmd",
        "-p output_cmd_topic:=/cmd_vel",
        "-p publish_rate:=20.0",
        "-p actuator_command_topic:=/cmd_vel",
        '"${motion_probe}" --iface "${sdk_interface}" stop',
    ):
        require(token in start_text, "start script missing " + token)
    for token in (
        "for ((attempt = 1; attempt <= 35; attempt++))",
        'stop_exact_group "${cmd_vel_sender_pgid}" INT',
        'stop_exact_group "${sdk_receiver_pgid}" INT',
        '"${probe}" --iface "${interface}" stop',
    ):
        require(token in stop_text, "stop script missing " + token)

    print("XBF offline deployment bundle verification passed")
    print("map_id:", publication["map_id"])
    print("manifest_sha256:", publication["compiled_map"]["manifest_sha256"])
    print("stable_layer_points:", publication["stable_layer"]["point_count"])
    print("route_points:", len(route))
    print("checkpoints:", ",".join(map(str, checkpoint_indexes)))
    print("follower_source_checked:", follower_file)
    print("safe_cmd_source_checked:", safe_cmd_file)
    print("cmd_vel_sender_source_checked:", sender_file)
    print("sdk2_receiver_source_checked:", receiver_file)
    print("sdk2_motion_probe_source_checked:", motion_probe_file)
    print("known_good_follower_sha256:", EXPECTED_FOLLOWER_SHA256)
    print("known_good_safe_cmd_sha256:", EXPECTED_SAFE_CMD_SHA256)
    print(
        "known_good_cmd_vel_udp_sender_sha256:",
        EXPECTED_CMD_VEL_UDP_SENDER_SHA256,
    )
    print(
        "known_good_sdk2_udp_receiver_sha256:",
        EXPECTED_SDK2_UDP_RECEIVER_SHA256,
    )
    print(
        "known_good_sdk2_motion_probe_sha256:",
        EXPECTED_SDK2_MOTION_PROBE_SHA256,
    )
    print("no ROS node was started and no motion command was published")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as error:
        print("XBF offline verification failed:", error, file=sys.stderr)
        raise SystemExit(2)

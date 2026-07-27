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
import math
import os
import sys
import zipfile
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
    "973c906c89a753f1eee6ab21052f92f9195015df3252a443498e14a6f4564f55"
)
EXPECTED_PCD_SHA256 = (
    "3526e4f116586d3594c0afa45efb3fb254e4eca1bf89fa21f18896a558ee5aa2"
)
EXPECTED_SOURCE_CSV_SHA256 = (
    "b4abadd38c30f5904f4cfe10eb529b8c1a4940ba023019847ea3959c48fd53a2"
)
EXPECTED_PREPARATION_ZIP_SHA256 = (
    "8b65b4e6f08ebdc4dd98a72536343a462a3aa309775f3e177120081b3f429b9c"
)
EXPECTED_ANNOTATION_SHA256 = (
    "8a5d1e0dca68ac9db1aa407cef625bfb51bbffbd1c60a06c6e319e29733bdc88"
)
EXPECTED_MAP_ID = "xbf9-horizontal-clean-r1"
EXPECTED_ANNOTATION_REVISION = "review-20260727084530215Z"
EXPECTED_STABLE_LAYER_POINTS = 34313
EXPECTED_CHECKPOINTS = (26, 161, 274, 368, 577, 737, 907, 1040)
EXPECTED_ALIGNMENT_THETA_RAD = -0.27474701469097695
EXPECTED_APPROVED_LANDMARKS = (
    "AUTO-P07",
    "AUTO-P09",
    "AUTO-P117",
    "AUTO-P143",
    "AUTO-P152",
    "AUTO-P156",
    "AUTO-P163",
    "AUTO-P170",
    "AUTO-P172",
    "AUTO-P58",
    "AUTO-P59",
    "AUTO-P90",
    "AUTO-W01",
    "AUTO-W02",
    "AUTO-W03",
    "AUTO-W04",
    "AUTO-W05",
    "AUTO-W06",
    "AUTO-W07",
    "AUTO-W08",
    "AUTO-W09",
    "AUTO-W10",
    "AUTO-W13",
    "AUTO-W14",
    "AUTO-W16",
    "AUTO-W17",
    "AUTO-W19",
    "AUTO-W20",
)
EXPECTED_CANDIDATE_LANDMARKS = ("AUTO-P74",)
EXPECTED_REJECTED_LANDMARKS = (
    "AUTO-C01",
    "AUTO-P157",
    "AUTO-P162",
    "AUTO-P20",
    "AUTO-P203",
    "AUTO-P81",
    "AUTO-W11",
)


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
    route_file = bundle_root / "routes/xbf9_horizontal_clean.aligned.csv"
    checkpoint_file = (
        bundle_root / "routes/xbf9_horizontal_clean.checkpoints.json"
    )
    route_metadata_file = (
        bundle_root / "routes/xbf9_horizontal_clean.aligned.route.json"
    )
    provenance_root = bundle_root / "task_provenance"
    preparation_zip = (
        provenance_root / "xbf9_horizontal_clean.go2-patrol-preparation.zip"
    )
    preparation_file = provenance_root / "preparation.json"
    alignment_file = provenance_root / "alignment.json"
    annotation_file = provenance_root / "annotations.json"
    source_csv_file = provenance_root / "source.csv"
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
    common_script = bundle_root / "scripts/_xbf_patrol_common.sh"
    preflight_script = bundle_root / "scripts/preflight_xbf_patrol.sh"
    session_exec = bundle_root / "scripts/xbf_session_exec.py"
    group_guard = bundle_root / "scripts/xbf_group_guard.py"
    route_ready_waiter = bundle_root / "scripts/wait_route_ready.py"
    timestamp_probe = bundle_root / "scripts/measure_input_timestamps.py"
    platform_importer = bundle_root / "scripts/import_platform_preparation.py"
    platform_import_wrapper = (
        bundle_root / "scripts/import_platform_preparation.sh"
    )
    map_manifest_source = (
        bundle_root
        / "overlay/src/go2_map_localizer/src/map_manifest.cpp"
    )
    annotation_filter_source = (
        bundle_root
        / "overlay/src/go2_map_tools/go2_map_tools/annotation_filter.py"
    )

    publication = verify_reviewed_map_bundle(map_root)
    require(publication["map_id"] == EXPECTED_MAP_ID, "unexpected map_id")
    require(
        publication["source_pcd"]["sha256"] == EXPECTED_PCD_SHA256,
        "reviewed map source PCD mismatch",
    )
    require(
        publication["annotation"]["revision"] == EXPECTED_ANNOTATION_REVISION,
        "unexpected annotation revision",
    )
    require(
        publication["annotation"]["sha256"] == EXPECTED_ANNOTATION_SHA256,
        "unexpected annotation identity",
    )
    require(
        publication["stable_layer"]["point_count"]
        == EXPECTED_STABLE_LAYER_POINTS,
        "stable layer point count changed",
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
        route_metadata["map"]["annotation_revision"]
        == EXPECTED_ANNOTATION_REVISION,
        "route metadata annotation revision mismatch",
    )
    alignment = route_metadata["alignment"]
    require(alignment["type"] == "SE2", "route alignment is not SE2")
    require(
        alignment["operator_confirmed"] is True,
        "platform alignment was not confirmed",
    )
    require(
        alignment["field_truth_verified"] is False,
        "offline bundle must not claim real-dog field verification",
    )
    require(
        math.isclose(
            float(alignment["theta_rad"]),
            EXPECTED_ALIGNMENT_THETA_RAD,
            rel_tol=0.0,
            abs_tol=1e-15,
        ),
        "route alignment yaw changed",
    )
    require(
        alignment["translation_m"] == [0, 0],
        "route alignment translation changed",
    )

    require(
        sha256_file(preparation_zip) == EXPECTED_PREPARATION_ZIP_SHA256,
        "platform preparation ZIP changed",
    )
    require(
        sha256_file(source_csv_file) == EXPECTED_SOURCE_CSV_SHA256,
        "provenance source CSV changed",
    )
    require(
        sha256_file(annotation_file) == EXPECTED_ANNOTATION_SHA256,
        "provenance annotation changed",
    )
    preparation = json.loads(preparation_file.read_text(encoding="utf-8"))
    require(
        preparation["schema"] == "go2.patrol_preparation/v1",
        "unexpected preparation schema",
    )
    require(
        preparation["source"]["pcd_sha256"] == EXPECTED_PCD_SHA256,
        "preparation source PCD mismatch",
    )
    require(
        preparation["source"]["csv_sha256"] == EXPECTED_SOURCE_CSV_SHA256,
        "preparation source CSV mismatch",
    )
    require(
        preparation["alignment"]["confirmed"] is True,
        "preparation alignment is not confirmed",
    )
    require(
        math.isclose(
            float(preparation["alignment"]["theta_rad"]),
            EXPECTED_ALIGNMENT_THETA_RAD,
            rel_tol=0.0,
            abs_tol=1e-15,
        ),
        "preparation yaw changed",
    )
    require(
        preparation["alignment"]["translation_m"] == [0, 0],
        "preparation translation changed",
    )
    require(
        preparation["trim"]["exported_point_count"] == 1277
        and preparation["trim"]["source_point_count"] == 1277
        and preparation["trim"]["source_start_index"] == 0
        and preparation["trim"]["source_end_index"] == 1276,
        "preparation route trim changed",
    )
    require(
        tuple(preparation["landmarks"]["approved_ids"])
        == EXPECTED_APPROVED_LANDMARKS,
        "approved landmark set changed",
    )
    require(
        tuple(preparation["landmarks"]["candidate_ids"])
        == EXPECTED_CANDIDATE_LANDMARKS,
        "candidate landmark set changed",
    )
    require(
        tuple(preparation["landmarks"]["rejected_ids"])
        == EXPECTED_REJECTED_LANDMARKS,
        "rejected landmark set changed",
    )
    require(
        tuple(item["waypoint_index"] for item in preparation["checkpoints"])
        == EXPECTED_CHECKPOINTS,
        "preparation checkpoint set changed",
    )

    expected_zip_members = {
        "README.zh-CN.txt",
        "preparation.json",
        "source/source.csv",
        "xbf-2-2.3526e4f11658.annotations.json",
        "xbf9_horizontal_clean.aligned.csv",
        "xbf9_horizontal_clean.alignment.json",
        "xbf9_horizontal_clean.checkpoints.json",
    }
    with zipfile.ZipFile(preparation_zip) as archive:
        require(
            set(archive.namelist()) == expected_zip_members,
            "platform preparation ZIP members changed",
        )
        require(
            archive.read("source/source.csv") == source_csv_file.read_bytes(),
            "ZIP source CSV does not match provenance copy",
        )
        require(
            archive.read("xbf9_horizontal_clean.aligned.csv")
            == route_file.read_bytes(),
            "ZIP aligned route does not match deployed route",
        )
        require(
            archive.read("xbf9_horizontal_clean.checkpoints.json")
            == checkpoint_file.read_bytes(),
            "ZIP checkpoints do not match deployed checkpoints",
        )
        require(
            json.loads(archive.read("preparation.json"))
            == json.loads(preparation_file.read_bytes()),
            "ZIP preparation does not semantically match provenance copy",
        )
        require(
            archive.read("xbf9_horizontal_clean.alignment.json")
            == alignment_file.read_bytes(),
            "ZIP alignment does not match provenance copy",
        )
        require(
            archive.read("xbf-2-2.3526e4f11658.annotations.json")
            == annotation_file.read_bytes(),
            "ZIP annotations do not match provenance copy",
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
        "landmark_verification:",
        "maximum_correspondence_m: 0.60",
        "support_radius_m: 1.50",
        "minimum_matches: 40",
        "minimum_unique_voxels: 16",
        "minimum_support_match_ratio: 0.10",
        "minimum_major_spread_m: 1.5",
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
    common_text = common_script.read_text(encoding="utf-8")
    preflight_text = preflight_script.read_text(encoding="utf-8")
    session_exec_text = session_exec.read_text(encoding="utf-8")
    group_guard_text = group_guard.read_text(encoding="utf-8")
    route_ready_text = route_ready_waiter.read_text(encoding="utf-8")
    timestamp_probe_text = timestamp_probe.read_text(encoding="utf-8")
    platform_importer_text = platform_importer.read_text(encoding="utf-8")
    platform_import_wrapper_text = platform_import_wrapper.read_text(
        encoding="utf-8"
    )
    map_manifest_text = map_manifest_source.read_text(encoding="utf-8")
    annotation_filter_text = annotation_filter_source.read_text(encoding="utf-8")
    for token in (
        'XBF_RMW_IMPLEMENTATION="rmw_cyclonedds_cpp"',
        "file:///tmp/go2_cyclonedds_eth0.xml",
        'export RMW_IMPLEMENTATION="${XBF_RMW_IMPLEMENTATION}"',
        'export CYCLONEDDS_URI="${XBF_CYCLONEDDS_URI}"',
        "xbf_udp_port_in_use()",
        "xbf_process_start_ticks()",
        'if ($field ~ (":" port "$"))',
    ):
        require(token in common_text, "runtime setup missing " + token)
    for token in (
        '"${RMW_IMPLEMENTATION}" == "rmw_cyclonedds_cpp"',
        "ros2 pkg prefix rmw_cyclonedds_cpp",
    ):
        require(token in preflight_text, "preflight missing " + token)
    for token in (
        'sdk_interface="${GO2_SDK_IF:-eth0}"',
        'runtime_coordinator_config="${XBF_RUNTIME_DIR}/coordinator.runtime.yaml"',
        'json.dumps(sys.argv[11])',
        '"expected_source_csv_sha256": sys.argv[9]',
        '"expected_source_pcd_sha256": sys.argv[10]',
        '--params-file "${runtime_coordinator_config}"',
        "go2_sdk2_udp_receiver",
        '"${sdk_receiver}" "${sdk_interface}" 5005',
        "wait_for_udp_receiver 30",
        "cmd_vel_udp_sender",
        "-p target_ip:=127.0.0.1",
        "-p target_port:=5005",
        "-p cmd_topic:=/checkpoint_localization/gated_cmd",
        "-p output_cmd_topic:=/cmd_vel",
        "-p publish_rate:=20.0",
        '"${motion_probe}" --iface "${sdk_interface}" stop',
        'session_exec="${XBF_SCRIPT_DIR}/xbf_session_exec.py"',
        'group_guard="${XBF_SCRIPT_DIR}/xbf_group_guard.py"',
        "spawn_component",
        "PID=PGID=SID",
        "supervisor_start_ticks=",
        "require_pre_bridge_graph_clean",
        "运动桥接通前 /cmd_vel",
        'timestamp_probe="${XBF_SCRIPT_DIR}/measure_input_timestamps.py"',
        'route_ready_waiter="${XBF_SCRIPT_DIR}/wait_route_ready.py"',
        "/checkpoint_localization/follower_cmd 1 20",
        "/checkpoint_localization/aligned_odometry 1 20",
    ):
        require(token in start_text, "start script missing " + token)
    require(
        "setsid " not in start_text,
        "start script must not reintroduce the asynchronous setsid PID race",
    )
    for path in (session_exec, group_guard, route_ready_waiter, timestamp_probe):
        require(os.access(path, os.X_OK), f"runtime helper is not executable: {path}")
    for path in (platform_importer, platform_import_wrapper):
        require(
            os.access(path, os.X_OK),
            f"platform import helper is not executable: {path}",
        )
    for token in ("os.setsid()", "os.execvpe("):
        require(token in session_exec_text, "session exec helper missing " + token)
    for token in ("GO2_XBF_RUN_ID=", "os.getpgid(", "os.killpg("):
        require(token in group_guard_text, "group guard missing " + token)
    for token in ("RouteStatus.RUNNING", "RouteStatus.FAULT", "localization_ready"):
        require(token in route_ready_text, "route-ready probe missing " + token)
    for token in ("p95_age_sec", "non_increasing_count", "minimum_samples"):
        require(token in timestamp_probe_text, "timestamp probe missing " + token)
    for token in (
        'PREPARATION_SCHEMA = "go2.patrol_preparation/v1"',
        "publish_reviewed_map_bundle(",
        "load_route(",
        '"source/source.csv"',
        '"deployment.env"',
        '"real_dog_running_verified": False',
        '"field_truth_verified": False',
    ):
        require(token in platform_importer_text, "platform importer missing " + token)
    for token in (
        'OVERLAY_SETUP="${BUNDLE_ROOT}/overlay/install/setup.bash"',
        'exec python3 "${SCRIPT_DIR}/import_platform_preparation.py" "$@"',
    ):
        require(
            token in platform_import_wrapper_text,
            "platform import wrapper missing " + token,
        )
    require(
        'raise SystemExit("严格解析后没有 checkpoint")' not in preflight_text,
        "preflight must allow startup-only calibration with zero checkpoints",
    )
    for token in (
        "kMaximumDescriptorIndexBytes",
        "Json::CharReaderBuilder",
        'builder["rejectDupKeys"] = true',
        "has more entries than manifest tiles",
    ):
        require(token in map_manifest_text, "bounded descriptor parser missing " + token)
    for token in (
        'review_status == "approved"',
        "not self.legacy_candidate",
        '"roi_geometry_union"',
        "_parse_platform_v2_annotation",
    ):
        require(token in annotation_filter_text, "annotation contract missing " + token)
    require(
        "xbf_udp_port_in_use 5005" in preflight_text,
        "preflight does not use the column-independent UDP port check",
    )
    for forbidden in (
        '-p expected_map_id:="${map_id}"',
        '-p expected_source_csv_sha256:="${source_csv_sha}"',
        '-p expected_source_pcd_sha256:="${source_pcd_sha}"',
    ):
        require(
            forbidden not in start_text,
            "runtime identity must not rely on Foxy CLI override precedence: "
            + forbidden,
        )
    for token in (
        "for ((attempt = 1; attempt <= 35; attempt++))",
        'stop_exact_group "${cmd_vel_sender_pgid}" INT',
        'stop_exact_group "${sdk_receiver_pgid}" INT',
        '"${probe}" --iface "${interface}" stop',
        'run_id="$(pid_value run_id)"',
        '"${group_guard}" signal',
    ):
        require(token in stop_text, "stop script missing " + token)

    print("XBF offline deployment bundle verification passed")
    print("map_id:", publication["map_id"])
    print("manifest_sha256:", publication["compiled_map"]["manifest_sha256"])
    print("stable_layer_points:", publication["stable_layer"]["point_count"])
    print("route_points:", len(route))
    print("checkpoints:", ",".join(map(str, checkpoint_indexes)))
    print("approved_landmarks:", len(EXPECTED_APPROVED_LANDMARKS))
    print("alignment_theta_rad:", EXPECTED_ALIGNMENT_THETA_RAD)
    print("alignment_translation_m: 0,0")
    print("preparation_zip_sha256:", EXPECTED_PREPARATION_ZIP_SHA256)
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

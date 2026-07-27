#!/usr/bin/env python3
"""Compile one platform preparation ZIP into immutable dog-runtime assets.

The browser intentionally does not put a large PCD inside the ZIP. This
importer therefore requires the exact source PCD as a second input, verifies
every cross-file hash, builds the reviewed map, and emits route metadata that
``preflight_xbf_patrol.sh`` already understands.
"""

from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shlex
import shutil
import stat
import tempfile
from typing import Any, Dict, Iterable, Mapping, Optional, Sequence, Tuple
import zipfile

from go2_checkpoint_patrol.checkpoint_core import load_route
from go2_map_tools.reviewed_publish import publish_reviewed_map_bundle


PREPARATION_SCHEMA = "go2.patrol_preparation/v1"
ALIGNMENT_SCHEMA = "go2.route_alignment/v1"
ANNOTATION_SCHEMA = "go2.map_review_annotations/v2"
CHECKPOINT_SCHEMA = "go2.route_checkpoints/v1"
DEPLOYMENT_ROUTE_SCHEMA = "go2.deployment_route/v1"
IMPORT_REPORT_SCHEMA = "go2.platform_preparation_import/v1"
SHA256_PATTERN = re.compile(r"^[0-9a-f]{64}$")
SAFE_MAP_ID = re.compile(r"^[A-Za-z0-9._-]{1,120}$")
MAX_ZIP_BYTES = 32 * 1024 * 1024
MAX_ZIP_FILES = 64
MAX_UNCOMPRESSED_BYTES = 64 * 1024 * 1024
MAX_JSON_BYTES = 4 * 1024 * 1024
MAX_SOURCE_PCD_BYTES = 2 * 1024 * 1024 * 1024
ALIGNMENT_FLOAT_TOLERANCE = 1e-12
ROUTE_XY_TOLERANCE_M = 5.1e-7
ROUTE_YAW_TOLERANCE_RAD = 5.1e-10
ROUTE_SPEED_TOLERANCE_MPS = 5.1e-4


class PreparationImportError(ValueError):
    """Raised when a platform bundle is incomplete, ambiguous, or unbound."""


def _strict_object(pairs: Sequence[Tuple[str, Any]]) -> Dict[str, Any]:
    result: Dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise PreparationImportError(
                "JSON contains duplicate key %r" % key
            )
        result[key] = value
    return result


def _reject_constant(value: str) -> None:
    raise PreparationImportError(
        "JSON contains non-standard numeric constant %s" % value
    )


def _json_bytes(document: Mapping[str, Any]) -> bytes:
    return (
        json.dumps(
            document,
            ensure_ascii=False,
            allow_nan=False,
            indent=2,
            sort_keys=True,
        )
        + "\n"
    ).encode("utf-8")


def _parse_json(payload: bytes, label: str) -> Mapping[str, Any]:
    if not 1 <= len(payload) <= MAX_JSON_BYTES:
        raise PreparationImportError(
            "%s must be 1..%d bytes" % (label, MAX_JSON_BYTES)
        )
    try:
        text = payload.decode("utf-8", errors="strict")
        value = json.loads(
            text,
            object_pairs_hook=_strict_object,
            parse_constant=_reject_constant,
        )
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise PreparationImportError(
            "%s is not strict UTF-8 JSON: %s" % (label, exc)
        ) from exc
    if not isinstance(value, Mapping):
        raise PreparationImportError("%s must be a JSON object" % label)
    return value


def _mapping(value: Any, label: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise PreparationImportError("%s must be an object" % label)
    return value


def _list(value: Any, label: str) -> Sequence[Any]:
    if not isinstance(value, list):
        raise PreparationImportError("%s must be an array" % label)
    return value


def _exact_keys(
    value: Mapping[str, Any],
    required: Iterable[str],
    label: str,
    optional: Iterable[str] = (),
) -> None:
    required_set = set(required)
    allowed = required_set | set(optional)
    missing = sorted(required_set - set(value))
    unknown = sorted(set(value) - allowed)
    if missing or unknown:
        raise PreparationImportError(
            "%s keys mismatch; missing=%s unknown=%s"
            % (label, missing or "none", unknown or "none")
        )


def _sha256_bytes(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _sha(value: Any, label: str) -> str:
    if not isinstance(value, str) or SHA256_PATTERN.fullmatch(value) is None:
        raise PreparationImportError("%s must be lowercase SHA-256" % label)
    return value


def _finite(value: Any, label: str) -> float:
    if isinstance(value, bool):
        raise PreparationImportError("%s must be finite" % label)
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise PreparationImportError("%s must be finite" % label) from exc
    if not math.isfinite(result):
        raise PreparationImportError("%s must be finite" % label)
    return result


def _safe_archive_name(name: str) -> None:
    candidate = Path(name)
    if (
        not name
        or len(name) > 255
        or candidate.is_absolute()
        or "\\" in name
        or any(part in ("", ".", "..") for part in candidate.parts)
        or any(ord(character) < 0x20 or ord(character) == 0x7F for character in name)
    ):
        raise PreparationImportError("unsafe ZIP entry name: %r" % name)


def _read_zip(path: Path) -> Dict[str, bytes]:
    if not path.is_file():
        raise PreparationImportError("preparation ZIP does not exist: %s" % path)
    size = path.stat().st_size
    if not 1 <= size <= MAX_ZIP_BYTES:
        raise PreparationImportError(
            "preparation ZIP must be 1..%d bytes" % MAX_ZIP_BYTES
        )
    try:
        with zipfile.ZipFile(str(path), "r") as archive:
            infos = archive.infolist()
            if not 1 <= len(infos) <= MAX_ZIP_FILES:
                raise PreparationImportError(
                    "preparation ZIP must contain 1..%d files" % MAX_ZIP_FILES
                )
            total = 0
            result: Dict[str, bytes] = {}
            for info in infos:
                _safe_archive_name(info.filename)
                unix_mode = (info.external_attr >> 16) & 0xFFFF
                if info.is_dir() or stat.S_ISLNK(unix_mode):
                    raise PreparationImportError(
                        "ZIP directories and symlinks are forbidden: %s"
                        % info.filename
                    )
                if info.flag_bits & 0x1:
                    raise PreparationImportError("encrypted ZIP entries are forbidden")
                total += info.file_size
                if total > MAX_UNCOMPRESSED_BYTES:
                    raise PreparationImportError(
                        "ZIP uncompressed payload exceeds %d bytes"
                        % MAX_UNCOMPRESSED_BYTES
                    )
                if info.filename in result:
                    raise PreparationImportError(
                        "duplicate ZIP entry: %s" % info.filename
                    )
                result[info.filename] = archive.read(info)
            return result
    except zipfile.BadZipFile as exc:
        raise PreparationImportError("invalid preparation ZIP: %s" % exc) from exc


def _documents(entries: Mapping[str, bytes]) -> Dict[str, Tuple[str, Mapping[str, Any]]]:
    documents: Dict[str, Tuple[str, Mapping[str, Any]]] = {}
    for name, payload in entries.items():
        if not name.endswith(".json"):
            continue
        document = _parse_json(payload, name)
        schema = document.get("schema")
        if not isinstance(schema, str):
            raise PreparationImportError("%s does not declare schema" % name)
        if schema in documents:
            raise PreparationImportError(
                "bundle contains duplicate schema %s" % schema
            )
        documents[schema] = (name, document)
    required = {
        PREPARATION_SCHEMA,
        ALIGNMENT_SCHEMA,
        ANNOTATION_SCHEMA,
        CHECKPOINT_SCHEMA,
    }
    missing = sorted(required - set(documents))
    if missing:
        raise PreparationImportError(
            "bundle is missing required schemas: %s" % ", ".join(missing)
        )
    return documents


def _single_suffix(entries: Mapping[str, bytes], suffix: str) -> Tuple[str, bytes]:
    matches = [(name, payload) for name, payload in entries.items() if name.endswith(suffix)]
    if len(matches) != 1:
        raise PreparationImportError(
            "bundle must contain exactly one %s file; found %d"
            % (suffix, len(matches))
        )
    return matches[0]


def _validate_preparation(
    preparation: Mapping[str, Any],
) -> Tuple[Mapping[str, Any], Mapping[str, Any], Mapping[str, Any]]:
    _exact_keys(
        preparation,
        ("schema", "source", "alignment", "trim", "landmarks", "checkpoints"),
        "preparation",
    )
    source = _mapping(preparation["source"], "preparation.source")
    _exact_keys(
        source,
        ("pcd_file_name", "pcd_sha256", "csv_file_name", "csv_sha256"),
        "preparation.source",
    )
    if not isinstance(source["pcd_file_name"], str) or not source["pcd_file_name"]:
        raise PreparationImportError("preparation source PCD filename is empty")
    if not isinstance(source["csv_file_name"], str) or not source["csv_file_name"]:
        raise PreparationImportError("preparation source CSV filename is empty")
    _sha(source["pcd_sha256"], "preparation.source.pcd_sha256")
    _sha(source["csv_sha256"], "preparation.source.csv_sha256")

    alignment = _mapping(preparation["alignment"], "preparation.alignment")
    _exact_keys(
        alignment,
        ("type", "theta_rad", "translation_m", "confirmed"),
        "preparation.alignment",
    )
    if alignment["type"] != "SE2" or alignment["confirmed"] is not True:
        raise PreparationImportError(
            "platform SE(2) alignment must be explicitly confirmed"
        )
    theta = _finite(alignment["theta_rad"], "preparation.alignment.theta_rad")
    if abs(theta) > math.pi:
        raise PreparationImportError("alignment yaw must be within [-pi, pi]")
    translation = _list(
        alignment["translation_m"], "preparation.alignment.translation_m"
    )
    if len(translation) != 2:
        raise PreparationImportError("alignment translation must contain tx, ty")
    if max(abs(_finite(item, "alignment translation")) for item in translation) > 1e6:
        raise PreparationImportError("alignment translation is outside limits")

    trim = _mapping(preparation["trim"], "preparation.trim")
    _exact_keys(
        trim,
        (
            "source_start_index",
            "source_end_index",
            "source_point_count",
            "exported_point_count",
        ),
        "preparation.trim",
    )
    integer_values = {key: trim[key] for key in trim}
    if any(type(value) is not int for value in integer_values.values()):
        raise PreparationImportError("preparation trim values must be integers")
    if (
        trim["source_point_count"] < 2
        or trim["source_start_index"] < 0
        or trim["source_end_index"] >= trim["source_point_count"]
        or trim["source_start_index"] >= trim["source_end_index"]
        or trim["exported_point_count"]
        != trim["source_end_index"] - trim["source_start_index"] + 1
    ):
        raise PreparationImportError("preparation trim contract is inconsistent")

    landmarks = _mapping(preparation["landmarks"], "preparation.landmarks")
    _exact_keys(
        landmarks,
        ("approved_ids", "candidate_ids", "rejected_ids"),
        "preparation.landmarks",
    )
    for key in ("approved_ids", "candidate_ids", "rejected_ids"):
        identifiers = _list(landmarks[key], "preparation.landmarks.%s" % key)
        if any(not isinstance(item, str) or not item for item in identifiers):
            raise PreparationImportError("landmark ids must be non-empty strings")
        if len(identifiers) != len(set(identifiers)):
            raise PreparationImportError("landmark ids must be unique")
    if not landmarks["approved_ids"]:
        raise PreparationImportError(
            "at least one approved fixed-object ROI is required to build the stable layer"
        )
    _list(preparation["checkpoints"], "preparation.checkpoints")
    return source, alignment, trim


def _validated_csv_row_count(payload: bytes, label: str) -> int:
    try:
        text = payload.decode("utf-8-sig", errors="strict")
    except UnicodeDecodeError as exc:
        raise PreparationImportError("%s is not strict UTF-8" % label) from exc
    rows = list(csv.DictReader(text.splitlines()))
    if not rows or set(rows[0]) != {"id", "x", "y", "yaw", "v"}:
        raise PreparationImportError("%s is not the strict five-column route CSV" % label)
    return len(rows)


def _normalize_planar_angle(value: float) -> float:
    """Match the browser's modulo-based [-pi, pi) normalization."""

    wrapped = (
        ((value + math.pi) % (math.pi * 2.0) + math.pi * 2.0)
        % (math.pi * 2.0)
        - math.pi
    )
    return 0.0 if wrapped == 0.0 else wrapped


def _verify_aligned_route(
    source_route: Sequence[Any],
    aligned_route: Sequence[Any],
    alignment: Mapping[str, Any],
    trim: Mapping[str, Any],
) -> None:
    """Prove the executable route is exactly one SE(2)-transformed trim.

    The browser serializes x/y to six decimals, yaw to nine decimals, and
    speed to three decimals.  The tolerances below are one serialization
    half-step plus a small cross-language floating-point allowance.
    """

    start = trim["source_start_index"]
    end = trim["source_end_index"]
    expected_source = source_route[start : end + 1]
    if len(expected_source) != len(aligned_route):
        raise PreparationImportError(
            "aligned route is not the declared contiguous source CSV trim"
        )
    theta = _finite(alignment["theta_rad"], "preparation.alignment.theta_rad")
    translation = _list(
        alignment["translation_m"], "preparation.alignment.translation_m"
    )
    tx = _finite(translation[0], "preparation.alignment.translation_m[0]")
    ty = _finite(translation[1], "preparation.alignment.translation_m[1]")
    cosine = math.cos(theta)
    sine = math.sin(theta)

    for offset, (source_point, aligned_point) in enumerate(
        zip(expected_source, aligned_route)
    ):
        expected_x = (
            cosine * source_point.x - sine * source_point.y + tx
        )
        expected_y = (
            sine * source_point.x + cosine * source_point.y + ty
        )
        expected_yaw = _normalize_planar_angle(source_point.yaw + theta)
        row_number = offset + 2
        if aligned_point.point_id != source_point.point_id:
            raise PreparationImportError(
                "aligned route row %d changed the source waypoint id"
                % row_number
            )
        if (
            abs(aligned_point.x - expected_x) > ROUTE_XY_TOLERANCE_M
            or abs(aligned_point.y - expected_y) > ROUTE_XY_TOLERANCE_M
            or abs(aligned_point.yaw - expected_yaw)
            > ROUTE_YAW_TOLERANCE_RAD
            or abs(aligned_point.speed - source_point.speed)
            > ROUTE_SPEED_TOLERANCE_MPS
        ):
            raise PreparationImportError(
                "aligned route row %d is not the declared SE(2)+trim "
                "of source CSV" % row_number
            )


def _write(path: Path, payload: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("xb") as handle:
        handle.write(payload)
        handle.flush()
        os.fsync(handle.fileno())


def prepare_platform_bundle(
    bundle_path: os.PathLike,
    source_pcd_path: os.PathLike,
    output_root_path: os.PathLike,
    *,
    map_id: str,
    minimum_stable_points: int = 1000,
) -> Mapping[str, Any]:
    bundle = Path(bundle_path).expanduser().resolve()
    source_pcd = Path(source_pcd_path).expanduser().resolve()
    requested_output = Path(output_root_path).expanduser()
    if not SAFE_MAP_ID.fullmatch(map_id):
        raise PreparationImportError("map_id must match %s" % SAFE_MAP_ID.pattern)
    if requested_output.exists() or requested_output.is_symlink():
        raise PreparationImportError("output root must not already exist")
    output = requested_output.resolve()
    if output in (Path(output.anchor), Path.home().resolve()):
        raise PreparationImportError("refusing filesystem root or home as output")
    if (
        not source_pcd.is_file()
        or not 1 <= source_pcd.stat().st_size <= MAX_SOURCE_PCD_BYTES
    ):
        raise PreparationImportError(
            "source PCD must be a readable 1-byte..2-GiB regular file"
        )

    entries = _read_zip(bundle)
    documents = _documents(entries)
    _, preparation = documents[PREPARATION_SCHEMA]
    alignment_name, alignment_document = documents[ALIGNMENT_SCHEMA]
    annotation_name, annotation_document = documents[ANNOTATION_SCHEMA]
    checkpoint_name, checkpoint_document = documents[CHECKPOINT_SCHEMA]
    source, alignment, trim = _validate_preparation(preparation)
    route_name, route_bytes = _single_suffix(entries, ".aligned.csv")
    if "source/source.csv" not in entries:
        raise PreparationImportError("bundle is missing exact source/source.csv bytes")
    source_csv_bytes = entries["source/source.csv"]

    pcd_sha = _sha256_file(source_pcd)
    source_csv_sha = _sha256_bytes(source_csv_bytes)
    route_sha = _sha256_bytes(route_bytes)
    if pcd_sha != source["pcd_sha256"]:
        raise PreparationImportError("source PCD SHA-256 does not match preparation")
    if source_csv_sha != source["csv_sha256"]:
        raise PreparationImportError("source CSV SHA-256 does not match preparation")

    _exact_keys(
        alignment_document,
        ("schema", "status", "method", "source", "transform", "evidence"),
        "alignment document",
    )
    alignment_source = _mapping(alignment_document["source"], "alignment.source")
    _exact_keys(
        alignment_source,
        ("pcd_sha256", "csv_sha256"),
        "alignment.source",
    )
    if (
        alignment_document["status"] != "reviewed"
        or alignment_source["pcd_sha256"] != pcd_sha
        or alignment_source["csv_sha256"] != source_csv_sha
    ):
        raise PreparationImportError("alignment document source binding is inconsistent")
    alignment_transform = _mapping(
        alignment_document["transform"], "alignment.transform"
    )
    _exact_keys(
        alignment_transform,
        ("type", "theta_rad", "translation_m"),
        "alignment.transform",
    )
    document_theta = _finite(
        alignment_transform["theta_rad"], "alignment.transform.theta_rad"
    )
    preparation_theta = _finite(
        alignment["theta_rad"], "preparation.alignment.theta_rad"
    )
    document_translation = _list(
        alignment_transform["translation_m"],
        "alignment.transform.translation_m",
    )
    preparation_translation = _list(
        alignment["translation_m"],
        "preparation.alignment.translation_m",
    )
    if (
        alignment_transform["type"] != "SE2"
        or len(document_translation) != 2
        or len(preparation_translation) != 2
        or abs(document_theta - preparation_theta)
        > ALIGNMENT_FLOAT_TOLERANCE
        or any(
            abs(
                _finite(document_translation[index], "alignment translation")
                - _finite(
                    preparation_translation[index],
                    "preparation alignment translation",
                )
            )
            > ALIGNMENT_FLOAT_TOLERANCE
            for index in range(2)
        )
    ):
        raise PreparationImportError("alignment document differs from preparation")

    annotation_map = _mapping(annotation_document.get("map"), "annotation.map")
    if annotation_map.get("sha256") != pcd_sha:
        raise PreparationImportError("annotation document is bound to another PCD")
    annotations = _list(annotation_document.get("annotations"), "annotations")
    approved_ids = sorted(
        annotation.get("id")
        for annotation in annotations
        if isinstance(annotation, Mapping)
        and annotation.get("category") == "stable_include"
        and annotation.get("review_status") == "approved"
        and annotation.get("legacy_candidate") is False
    )
    preparation_approved = sorted(
        _list(
            _mapping(preparation["landmarks"], "preparation.landmarks")[
                "approved_ids"
            ],
            "preparation.landmarks.approved_ids",
        )
    )
    if approved_ids != preparation_approved:
        raise PreparationImportError(
            "approved stable-object IDs differ between preparation and annotations"
        )

    _exact_keys(
        checkpoint_document,
        (
            "schema",
            "source_pcd_sha256",
            "source_csv_sha256",
            "route_csv_sha256",
            "route_revision",
            "checkpoints",
        ),
        "checkpoint document",
    )
    if (
        checkpoint_document["source_pcd_sha256"] != pcd_sha
        or checkpoint_document["source_csv_sha256"] != source_csv_sha
        or checkpoint_document["route_csv_sha256"] != route_sha
    ):
        raise PreparationImportError("checkpoint sidecar hash binding is inconsistent")

    route_count = _validated_csv_row_count(route_bytes, route_name)
    source_count = _validated_csv_row_count(source_csv_bytes, "source/source.csv")
    if route_count != trim["exported_point_count"]:
        raise PreparationImportError(
            "aligned route row count differs from preparation trim"
        )
    if source_count != trim["source_point_count"]:
        raise PreparationImportError(
            "source route row count differs from preparation trim"
        )
    preparation_checkpoints = _list(
        preparation["checkpoints"], "preparation.checkpoints"
    )
    sidecar_checkpoints = _list(
        checkpoint_document["checkpoints"], "checkpoint.checkpoints"
    )
    preparation_waypoints = [
        (item.get("waypoint_id"), item.get("waypoint_index"))
        for item in preparation_checkpoints
        if isinstance(item, Mapping)
    ]
    sidecar_waypoints = [
        (item.get("waypoint_id"), item.get("waypoint_index"))
        for item in sidecar_checkpoints
        if isinstance(item, Mapping)
    ]
    if (
        len(preparation_waypoints) != len(preparation_checkpoints)
        or preparation_waypoints != sidecar_waypoints
    ):
        raise PreparationImportError(
            "checkpoint waypoint bindings differ from preparation"
        )

    output.parent.mkdir(parents=True, exist_ok=True)
    staging = Path(
        tempfile.mkdtemp(
            prefix=".%s.platform-import." % output.name,
            dir=str(output.parent),
        )
    )
    try:
        input_root = staging / "_input"
        route_input = input_root / route_name
        checkpoint_input = input_root / checkpoint_name
        annotation_input = input_root / annotation_name
        _write(route_input, route_bytes)
        _write(checkpoint_input, entries[checkpoint_name])
        _write(annotation_input, entries[annotation_name])
        source_csv_input = input_root / "source.csv"
        _write(source_csv_input, source_csv_bytes)
        alignment_input = input_root / alignment_name
        _write(alignment_input, entries[alignment_name])

        # Reuse the exact dog-side parser before copying the route into runtime.
        parsed_route = load_route(
            str(route_input),
            default_checkpoint_radius_m=0.60,
            default_search_radius_m=12.0,
            checkpoint_file=str(checkpoint_input),
            expected_source_csv_sha256=source_csv_sha,
            expected_source_pcd_sha256=pcd_sha,
        )
        source_route = load_route(
            str(source_csv_input),
            default_checkpoint_radius_m=0.60,
            default_search_radius_m=12.0,
        )
        _verify_aligned_route(source_route, parsed_route, alignment, trim)
        parsed_checkpoint_count = sum(point.is_checkpoint for point in parsed_route)
        if parsed_checkpoint_count != len(sidecar_checkpoints):
            raise PreparationImportError("dog-side checkpoint parser count mismatch")

        map_root = staging / "maps" / map_id
        publication = publish_reviewed_map_bundle(
            source_pcd,
            annotation_input,
            map_root,
            map_id=map_id,
            minimum_stable_points=minimum_stable_points,
        )
        manifest_sha = _sha256_file(map_root / "manifest.json")
        routes_root = staging / "routes"
        routes_root.mkdir()
        final_route_name = Path(route_name).name
        final_checkpoint_name = Path(checkpoint_name).name
        final_route = routes_root / final_route_name
        final_checkpoint = routes_root / final_checkpoint_name
        shutil.copy2(str(route_input), str(final_route))
        shutil.copy2(str(checkpoint_input), str(final_checkpoint))

        evidence_by_waypoint = {
            (item["waypoint_id"], item["waypoint_index"]): item["landmark_id"]
            for item in preparation_checkpoints
            if isinstance(item, Mapping)
        }
        checkpoint_metadata = []
        for item in sidecar_checkpoints:
            checkpoint = _mapping(item["checkpoint"], "checkpoint policy")
            waypoint_key = (item["waypoint_id"], item["waypoint_index"])
            matching = next(
                (
                    candidate
                    for candidate in preparation_checkpoints
                    if isinstance(candidate, Mapping)
                    and (
                        candidate.get("waypoint_id"),
                        candidate.get("waypoint_index"),
                    )
                    == waypoint_key
                ),
                None,
            )
            checkpoint_metadata.append(
                {
                    "waypoint_index": item["waypoint_index"],
                    "route_s_m": (
                        matching.get("progress_m") if matching is not None else None
                    ),
                    "evidence": [evidence_by_waypoint[waypoint_key]],
                    "timeout_s": checkpoint["stop_timeout_s"],
                }
            )

        annotation_revision = annotation_document.get("revision")
        route_metadata = {
            "schema": DEPLOYMENT_ROUTE_SCHEMA,
            "route_id": "%s-%s" % (map_id, route_sha[:12]),
            "frame_id": "map",
            "route_file": final_route.name,
            "route_csv_sha256": route_sha,
            "checkpoint_file": final_checkpoint.name,
            "checkpoint_sha256": _sha256_file(final_checkpoint),
            "source": {
                "pcd_sha256": pcd_sha,
                "csv_sha256": source_csv_sha,
            },
            "map": {
                "map_id": map_id,
                "manifest_sha256": manifest_sha,
                "annotation_revision": annotation_revision,
            },
            "alignment": {
                "type": "SE2",
                "theta_rad": alignment["theta_rad"],
                "translation_m": alignment["translation_m"],
                "method": alignment_document["method"],
                "operator_confirmed": True,
                # Platform confirmation proves that the two offline assets are
                # intentionally bound. It does not prove real-dog localization
                # or that the route lies on the physical road at runtime.
                "field_truth_verified": False,
                "field_truth_note_zh": (
                    "平台已确认 PCD 与 CSV 的平面配准；真狗起点定位与短距离"
                    "路线落地仍待现场复核。"
                ),
            },
            "checkpoints": checkpoint_metadata,
            "runtime": {
                "normal_segment": (
                    "frozen map<-odom SE2 plus original waypoint_follower_go2_2"
                ),
                "calibration": (
                    "stationary anchored scan-to-cleaned-map registration"
                ),
                "continuous_registration": False,
            },
            "note_zh": (
                "由平台已确认的 PCD+CSV SE(2)、对象级固定物和 checkpoint "
                "自动生成；起点仍须在真实狗上完成定位并进入 RUNNING。"
            ),
        }
        metadata_path = final_route.with_suffix(".route.json")
        _write(metadata_path, _json_bytes(route_metadata))

        preparation_assets = staging / "preparation_assets"
        preparation_assets.mkdir()
        shutil.copy2(str(bundle), str(preparation_assets / bundle.name))
        shutil.copy2(str(source_csv_input), str(preparation_assets / "source.csv"))
        shutil.copy2(str(alignment_input), str(preparation_assets / "alignment.json"))
        shutil.copy2(str(annotation_input), str(preparation_assets / "annotations.json"))
        # Store the normalized preparation separately; the ZIP remains the
        # immutable byte-for-byte source artifact.
        _write(
            preparation_assets / "preparation.json",
            _json_bytes(preparation),
        )

        final_map = output / "maps" / map_id
        final_route_path = output / "routes" / final_route.name
        final_checkpoint_path = output / "routes" / final_checkpoint.name
        report = {
            "schema": IMPORT_REPORT_SCHEMA,
            "created_utc": datetime.now(timezone.utc)
            .isoformat()
            .replace("+00:00", "Z"),
            "source_bundle": {
                "file_name": bundle.name,
                "sha256": _sha256_file(bundle),
            },
            "source_pcd": {
                "file_name": source_pcd.name,
                "sha256": pcd_sha,
            },
            "source_csv_sha256": source_csv_sha,
            "route_csv_sha256": route_sha,
            "map_id": map_id,
            "manifest_sha256": manifest_sha,
            "stable_layer_points": publication["stable_layer"]["point_count"],
            "route_points": len(parsed_route),
            "checkpoint_count": parsed_checkpoint_count,
            "approved_landmark_ids": preparation_approved,
            "runtime_inputs": {
                "map_root": str(final_map),
                "route_file": str(final_route_path),
                "checkpoint_file": str(final_checkpoint_path),
            },
            "real_dog_running_verified": False,
        }
        _write(staging / "IMPORT_REPORT.json", _json_bytes(report))
        environment = "\n".join(
            [
                "# Generated by import_platform_preparation.py",
                "export GO2_XBF_MAP_ROOT=%s" % shlex.quote(str(final_map)),
                "export GO2_XBF_ROUTE_FILE=%s"
                % shlex.quote(str(final_route_path)),
                "export GO2_XBF_CHECKPOINT_FILE=%s"
                % shlex.quote(str(final_checkpoint_path)),
                "",
            ]
        ).encode("utf-8")
        _write(staging / "deployment.env", environment)
        readme = (
            "# 平台准备包导入结果\n\n"
            "本目录已完成源 PCD、源 CSV、SE(2)、annotations、执行 CSV、"
            "checkpoint 和 reviewed map 的交叉哈希校验。\n\n"
            "部署时先 source `deployment.env`，再运行 release 内的"
            " `scripts/preflight_xbf_patrol.sh`。首次只允许静止校准，"
            "并确认 coordinator 明确进入 RUNNING；本导入动作本身没有启动"
            " ROS 节点，也没有发送运动命令。\n"
        ).encode("utf-8")
        _write(staging / "README.zh-CN.md", readme)

        shutil.rmtree(str(input_root))
        if output.exists() or output.is_symlink():
            raise PreparationImportError("output root appeared during import")
        os.rename(str(staging), str(output))
        return report
    except Exception:
        if staging.exists():
            shutil.rmtree(str(staging))
        raise


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Verify a platform preparation ZIP and compile reviewed map + "
            "route assets for the current dog runtime."
        )
    )
    parser.add_argument("bundle_zip", type=Path)
    parser.add_argument("source_pcd", type=Path)
    parser.add_argument("output_root", type=Path)
    parser.add_argument("--map-id", required=True)
    parser.add_argument("--minimum-stable-points", type=int, default=1000)
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    arguments = build_parser().parse_args(argv)
    try:
        report = prepare_platform_bundle(
            arguments.bundle_zip,
            arguments.source_pcd,
            arguments.output_root,
            map_id=arguments.map_id,
            minimum_stable_points=arguments.minimum_stable_points,
        )
    except (
        OSError,
        PreparationImportError,
        ValueError,
        zipfile.BadZipFile,
    ) as exc:
        print("准备包导入失败：%s" % exc, file=os.sys.stderr)
        return 2
    print(json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True))
    print("没有启动 ROS 节点，也没有发送运动命令。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

"""Deterministic, offline PCD review-bundle generation.

The browser review console must not parse an unbounded source PCD or infer
recording health from the final merged cloud alone.  This module strictly reads
the existing PCD format, derives bounded preview and quality artifacts, and
optionally binds recorder keyframe/session evidence.
"""

from __future__ import annotations

import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import struct
import tempfile
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple, cast

from .pcd import PCDCloud, PCDDataError, read_pcd


REVIEW_BUNDLE_SCHEMA = "go2.map_review_bundle/v1"
REGISTERED_SESSION_SCHEMA = "go2.registered_pcd_session/v1"
PREVIEW_FILENAME = "preview.xyz.bin"
REVIEW_FILENAME = "review.json"
MAX_PREVIEW_POINTS = 500_000
MAX_REVIEW_DENSITY_CELLS = 500_000
MAX_JSON_BYTES = 16 * 1024 * 1024
MAX_REVIEW_JSON_BYTES = 64 * 1024 * 1024
MAX_JSONL_LINE_CHARACTERS = 1024 * 1024
MAX_KEYFRAMES = 250_000
MAX_SAFE_JSON_INTEGER = 9_007_199_254_740_991
MAX_FLOAT32 = 3.4028234663852886e38
_FLOAT32_XYZ = struct.Struct("<fff")
_SHA256_HEX = frozenset("0123456789abcdef")


class ReviewBundleError(ValueError):
    """Raised when review inputs or the requested output are unsafe."""


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    try:
        with path.open("rb") as handle:
            for block in iter(lambda: handle.read(1024 * 1024), b""):
                digest.update(block)
    except OSError as exc:
        raise ReviewBundleError("cannot hash %s: %s" % (path, exc)) from exc
    return digest.hexdigest()


def _is_int(value: Any) -> bool:
    return isinstance(value, int) and not isinstance(value, bool)


def _required_int(
    document: Mapping[str, Any],
    key: str,
    label: str,
    minimum: int = 0,
) -> int:
    value = document.get(key)
    if not _is_int(value):
        raise ReviewBundleError(
            "%s.%s must be an integer >= %d" % (label, key, minimum)
        )
    result = cast(int, value)
    if result < minimum:
        raise ReviewBundleError(
            "%s.%s must be an integer >= %d" % (label, key, minimum)
        )
    return result


def _finite_number(value: Any, label: str) -> float:
    if isinstance(value, bool):
        raise ReviewBundleError("%s must be a finite number" % label)
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ReviewBundleError("%s must be a finite number" % label) from exc
    if not math.isfinite(result):
        raise ReviewBundleError("%s must be a finite number" % label)
    return result


def _reject_json_constant(value: str) -> None:
    raise ReviewBundleError("JSON contains non-standard constant %s" % value)


def _unique_json_object(pairs: Sequence[Tuple[str, Any]]) -> Dict[str, Any]:
    result: Dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise ReviewBundleError("JSON contains duplicate key %r" % key)
        result[key] = value
    return result


def _strict_json_loads(content: str, label: str) -> Any:
    try:
        return json.loads(
            content,
            parse_constant=_reject_json_constant,
            object_pairs_hook=_unique_json_object,
        )
    except ReviewBundleError:
        raise
    except (TypeError, ValueError, json.JSONDecodeError) as exc:
        raise ReviewBundleError(
            "%s is not strict JSON: %s" % (label, exc)
        ) from exc


def _load_json_file(path: Path, label: str) -> Tuple[Mapping[str, Any], str]:
    if not path.is_file():
        raise ReviewBundleError("%s is not a regular file: %s" % (label, path))
    try:
        size = path.stat().st_size
    except OSError as exc:
        raise ReviewBundleError(
            "cannot inspect %s: %s" % (label, exc)
        ) from exc
    if size <= 0 or size > MAX_JSON_BYTES:
        raise ReviewBundleError(
            "%s size must be in [1, %d] bytes" % (label, MAX_JSON_BYTES)
        )
    digest_before = _sha256_file(path)
    try:
        content = path.read_text(encoding="utf-8")
    except (OSError, UnicodeError) as exc:
        raise ReviewBundleError(
            "cannot read %s as UTF-8: %s" % (label, exc)
        ) from exc
    document = _strict_json_loads(content, label)
    digest_after = _sha256_file(path)
    if digest_before != digest_after:
        raise ReviewBundleError("%s changed while it was being read" % label)
    if not isinstance(document, Mapping):
        raise ReviewBundleError("%s must contain one JSON object" % label)
    return document, digest_after


def _validate_sha256(value: Any, label: str) -> str:
    digest = str(value).lower()
    if len(digest) != 64 or any(
        character not in _SHA256_HEX for character in digest
    ):
        raise ReviewBundleError(
            "%s must be a lowercase SHA-256 digest" % label
        )
    return digest


def _nearest_rank(values: Sequence[int], percentile: float) -> Optional[int]:
    if not values:
        return None
    ordered = sorted(values)
    rank = max(1, int(math.ceil(percentile * len(ordered))))
    return ordered[rank - 1]


def _number_summary(values: Sequence[int]) -> Dict[str, Any]:
    if not values:
        return {
            "count": 0,
            "min": None,
            "p50": None,
            "p95": None,
            "max": None,
        }
    return {
        "count": len(values),
        "min": min(values),
        "p50": _nearest_rank(values, 0.50),
        "p95": _nearest_rank(values, 0.95),
        "max": max(values),
    }


def _xyz_offsets(cloud: PCDCloud) -> Tuple[int, int, int]:
    fields = {field.name: field for field in cloud.fields}
    for name in ("x", "y", "z"):
        if name not in fields:
            raise ReviewBundleError(
                "source PCD is missing scalar field %r" % name
            )
        if fields[name].count != 1:
            raise ReviewBundleError(
                "source PCD field %r must be scalar" % name
            )
    return (
        cloud.field_offset("x"),
        cloud.field_offset("y"),
        cloud.field_offset("z"),
    )


def _analyze_cloud(cloud: PCDCloud) -> Dict[str, Any]:
    if cloud.point_count <= 0:
        raise ReviewBundleError("source PCD contains no points")
    x_offset, y_offset, z_offset = _xyz_offsets(cloud)
    minimum = [math.inf, math.inf, math.inf]
    maximum = [-math.inf, -math.inf, -math.inf]
    density: Dict[Tuple[int, int], int] = {}

    for index, row in enumerate(cloud.rows):
        point = (
            float(row[x_offset]),
            float(row[y_offset]),
            float(row[z_offset]),
        )
        if not all(math.isfinite(value) for value in point):
            # read_pcd rejects this already; retain the invariant at this
            # boundary.
            raise ReviewBundleError(
                "source PCD point %d is non-finite" % index
            )
        if any(abs(value) > MAX_FLOAT32 for value in point):
            raise ReviewBundleError(
                "source PCD point %d cannot be represented in float32 "
                "preview" % index
            )
        for axis, value in enumerate(point):
            minimum[axis] = min(minimum[axis], value)
            maximum[axis] = max(maximum[axis], value)
        ix = math.floor(point[0])
        iy = math.floor(point[1])
        if abs(ix) > MAX_SAFE_JSON_INTEGER or abs(iy) > MAX_SAFE_JSON_INTEGER:
            raise ReviewBundleError(
                "source XY coordinates exceed safe 1 m grid index range"
            )
        cell = (ix, iy)
        if cell not in density and len(density) >= MAX_REVIEW_DENSITY_CELLS:
            raise ReviewBundleError(
                "source PCD exceeds %d occupied review cells"
                % MAX_REVIEW_DENSITY_CELLS
            )
        density[cell] = density.get(cell, 0) + 1

    cells = sorted(density.items())
    cell_set = set(density)
    min_ix = min(cell[0] for cell in cell_set)
    max_ix = max(cell[0] for cell in cell_set)
    min_iy = min(cell[1] for cell in cell_set)
    max_iy = max(cell[1] for cell in cell_set)
    bounding_cell_count = (max_ix - min_ix + 1) * (max_iy - min_iy + 1)
    if bounding_cell_count > MAX_SAFE_JSON_INTEGER:
        raise ReviewBundleError(
            "1 m density grid is too large for review JSON"
        )

    unvisited = set(cell_set)
    component_sizes: List[int] = []
    while unvisited:
        pending = [unvisited.pop()]
        size = 0
        while pending:
            cell = pending.pop()
            size += 1
            for neighbour in (
                (cell[0] - 1, cell[1]),
                (cell[0] + 1, cell[1]),
                (cell[0], cell[1] - 1),
                (cell[0], cell[1] + 1),
            ):
                if neighbour in unvisited:
                    unvisited.remove(neighbour)
                    pending.append(neighbour)
        component_sizes.append(size)
    component_sizes.sort(reverse=True)

    isolated_cells = sum(
        1
        for ix, iy in cell_set
        if not any(
            neighbour in cell_set
            for neighbour in (
                (ix - 1, iy),
                (ix + 1, iy),
                (ix, iy - 1),
                (ix, iy + 1),
            )
        )
    )
    point_counts = [count for _, count in cells]
    occupied = len(cells)
    largest = component_sizes[0]
    return {
        "bounds": {"min": minimum, "max": maximum},
        "height": {
            "min_m": minimum[2],
            "max_m": maximum[2],
            "span_m": maximum[2] - minimum[2],
        },
        "density_1m": {
            "cell_size_m": 1.0,
            "grid_bounds": {
                "min": [min_ix, min_iy],
                "max": [max_ix, max_iy],
            },
            "occupied_cell_count": occupied,
            "bounding_cell_count": bounding_cell_count,
            "coverage_ratio": occupied / bounding_cell_count,
            "points_per_occupied_cell": {
                "min": min(point_counts),
                "mean": sum(point_counts) / occupied,
                "p50": _nearest_rank(point_counts, 0.50),
                "p95": _nearest_rank(point_counts, 0.95),
                "max": max(point_counts),
            },
            # Compact, deterministic [ix, iy, point_count] records for
            # heatmaps.
            "cells": [[cell[0], cell[1], count] for cell, count in cells],
        },
        "connectivity_4": {
            "component_count": len(component_sizes),
            "component_sizes": component_sizes,
            "largest_component_cells": largest,
            "largest_component_ratio": largest / occupied,
            "isolated_cell_count": isolated_cells,
        },
    }


def _read_keyframes(
    path: Path,
    expected_point_count: int,
) -> Tuple[List[Mapping[str, Any]], str]:
    if not path.is_file():
        raise ReviewBundleError("keyframes is not a regular file: %s" % path)
    digest_before = _sha256_file(path)
    records: List[Mapping[str, Any]] = []
    expected_offset = 0
    previous_cloud_stamp = 0
    previous_odometry_stamp = 0

    try:
        with path.open("r", encoding="utf-8") as handle:
            line_number = 0
            while True:
                line = handle.readline(MAX_JSONL_LINE_CHARACTERS + 1)
                if not line:
                    break
                line_number += 1
                if len(line) > MAX_JSONL_LINE_CHARACTERS:
                    raise ReviewBundleError(
                        "keyframes line %d exceeds %d characters"
                        % (line_number, MAX_JSONL_LINE_CHARACTERS)
                    )
                if not line.endswith("\n"):
                    # A valid final line may omit LF, but it still has to fit
                    # the bound.
                    remainder = handle.read(1)
                    if remainder:
                        raise ReviewBundleError(
                            "keyframes line %d exceeds %d characters"
                            % (line_number, MAX_JSONL_LINE_CHARACTERS)
                        )
                if not line.strip():
                    raise ReviewBundleError(
                        "keyframes line %d must not be blank" % line_number
                    )
                document = _strict_json_loads(
                    line, "keyframes line %d" % line_number
                )
                if not isinstance(document, Mapping):
                    raise ReviewBundleError(
                        "keyframes line %d must be a JSON object" % line_number
                    )
                index = _required_int(document, "index", "keyframe", minimum=0)
                if index != len(records):
                    raise ReviewBundleError(
                        "keyframe index %d is not the expected sequential "
                        "index %d"
                        % (index, len(records))
                    )
                point_offset = _required_int(
                    document, "point_offset", "keyframe", minimum=0
                )
                point_count = _required_int(
                    document, "point_count", "keyframe", minimum=1
                )
                if point_offset != expected_offset:
                    raise ReviewBundleError(
                        "keyframe %d point_offset is not contiguous" % index
                    )
                expected_offset += point_count

                cloud_stamp = _required_int(
                    document, "cloud_stamp_ns", "keyframe", minimum=1
                )
                odometry_stamp = _required_int(
                    document, "odometry_stamp_ns", "keyframe", minimum=1
                )
                sync_error = _required_int(
                    document, "sync_error_ns", "keyframe", minimum=0
                )
                if cloud_stamp <= previous_cloud_stamp:
                    raise ReviewBundleError(
                        "keyframe cloud timestamps must be strictly increasing"
                    )
                if records and odometry_stamp < previous_odometry_stamp:
                    raise ReviewBundleError(
                        "keyframe odometry timestamps must not decrease"
                    )
                if sync_error != abs(cloud_stamp - odometry_stamp):
                    raise ReviewBundleError(
                        "keyframe %d sync_error_ns does not match timestamps"
                        % index
                    )
                previous_cloud_stamp = cloud_stamp
                previous_odometry_stamp = odometry_stamp

                pose = document.get("pose")
                if not isinstance(pose, Mapping):
                    raise ReviewBundleError(
                        "keyframe %d pose must be an object" % index
                    )
                x = _finite_number(pose.get("x"), "keyframe.pose.x")
                y = _finite_number(pose.get("y"), "keyframe.pose.y")
                yaw = _finite_number(pose.get("yaw"), "keyframe.pose.yaw")
                if yaw < -math.pi or yaw > math.pi:
                    raise ReviewBundleError(
                        "keyframe %d pose.yaw must be in [-pi, pi]" % index
                    )
                input_count = _required_int(
                    document, "input_point_count", "keyframe", minimum=1
                )
                invalid_count = _required_int(
                    document, "invalid_point_count", "keyframe", minimum=0
                )
                if invalid_count > input_count:
                    raise ReviewBundleError(
                        "keyframe %d invalid_point_count exceeds input count"
                        % index
                    )
                if point_count > input_count - invalid_count:
                    raise ReviewBundleError(
                        "keyframe %d point_count exceeds valid input count"
                        % index
                    )
                truncated = document.get("truncated")
                if not isinstance(truncated, bool):
                    raise ReviewBundleError(
                        "keyframe %d truncated must be boolean" % index
                    )

                records.append(
                    {
                        "index": index,
                        "point_offset": point_offset,
                        "point_count": point_count,
                        "cloud_stamp_ns": cloud_stamp,
                        "odometry_stamp_ns": odometry_stamp,
                        "sync_error_ns": sync_error,
                        "pose": {"x": x, "y": y, "yaw": yaw},
                        "input_point_count": input_count,
                        "invalid_point_count": invalid_count,
                        "truncated": truncated,
                    }
                )
                if len(records) > MAX_KEYFRAMES:
                    raise ReviewBundleError(
                        "keyframes exceeds %d records" % MAX_KEYFRAMES
                    )
    except ReviewBundleError:
        raise
    except (OSError, UnicodeError) as exc:
        raise ReviewBundleError("cannot read keyframes: %s" % exc) from exc

    digest_after = _sha256_file(path)
    if digest_before != digest_after:
        raise ReviewBundleError("keyframes changed while it was being read")
    if not records:
        raise ReviewBundleError("keyframes contains no records")
    if expected_offset != expected_point_count:
        raise ReviewBundleError(
            "keyframes account for %d points but source PCD has %d"
            % (expected_offset, expected_point_count)
        )
    return records, digest_after


def _trajectory_report(records: Sequence[Mapping[str, Any]]) -> Dict[str, Any]:
    distance = 0.0
    intervals: List[int] = []
    sync_errors: List[int] = []
    truncated_indices: List[int] = []
    previous = None
    samples: List[Dict[str, Any]] = []

    for record in records:
        pose = record["pose"]
        if previous is not None:
            previous_pose = previous["pose"]
            distance += math.hypot(
                pose["x"] - previous_pose["x"],
                pose["y"] - previous_pose["y"],
            )
            intervals.append(
                record["cloud_stamp_ns"] - previous["cloud_stamp_ns"]
            )
        sync_errors.append(record["sync_error_ns"])
        if record["truncated"]:
            truncated_indices.append(record["index"])
        samples.append(dict(record))
        previous = record

    first_pose = records[0]["pose"]
    final_pose = records[-1]["pose"]
    closure_gap = math.hypot(
        final_pose["x"] - first_pose["x"],
        final_pose["y"] - first_pose["y"],
    )
    return {
        "keyframe_count": len(records),
        "duration_ns": records[-1]["cloud_stamp_ns"]
        - records[0]["cloud_stamp_ns"],
        "distance_m": distance,
        "interval_ns": _number_summary(intervals),
        "sync_error_ns": {
            "p95": _nearest_rank(sync_errors, 0.95),
            "max": max(sync_errors),
        },
        "truncated_keyframe_count": len(truncated_indices),
        "truncated_keyframe_indices": truncated_indices,
        "loop_closure_gap_m": closure_gap,
        "samples": samples,
    }


def _session_report(
    path: Path,
    source_filename: str,
    source_sha256: str,
    source_point_count: int,
    keyframes_filename: Optional[str],
    keyframes_sha256: Optional[str],
    keyframe_count: Optional[int],
) -> Tuple[Dict[str, Any], List[Dict[str, str]]]:
    document, session_sha256 = _load_json_file(path, "session")
    if document.get("schema") != REGISTERED_SESSION_SCHEMA:
        raise ReviewBundleError("session has unsupported schema")
    if document.get("state") != "complete":
        raise ReviewBundleError("session state must be 'complete'")
    point_count = _required_int(document, "point_count", "session", minimum=1)
    if point_count != source_point_count:
        raise ReviewBundleError(
            "session point_count does not match source PCD point count"
        )
    declared_keyframes = _required_int(
        document, "keyframe_count", "session", minimum=1
    )
    if keyframe_count is not None and declared_keyframes != keyframe_count:
        raise ReviewBundleError(
            "session keyframe_count does not match keyframes JSONL"
        )
    source_frame = document.get("source_frame")
    if not isinstance(source_frame, str) or not source_frame.strip():
        raise ReviewBundleError(
            "session.source_frame must be a non-empty string"
        )
    map_state = document.get("map_state")
    if map_state is not None and (
        not isinstance(map_state, str) or not map_state.strip()
    ):
        raise ReviewBundleError("session.map_state must be a non-empty string")
    motion_commands = document.get("motion_commands_published")
    if motion_commands is not None and not isinstance(motion_commands, bool):
        raise ReviewBundleError(
            "session.motion_commands_published must be boolean"
        )

    artifacts = document.get("artifacts")
    if not isinstance(artifacts, Mapping):
        raise ReviewBundleError("session.artifacts must be an object")
    normalized_artifacts: Dict[str, str] = {}
    for filename, digest in artifacts.items():
        if not isinstance(filename, str) or not filename:
            raise ReviewBundleError(
                "session artifact names must be non-empty strings"
            )
        normalized_artifacts[filename] = _validate_sha256(
            digest, "session artifact %s" % filename
        )

    pcd_artifact_hash = normalized_artifacts.get(source_filename)
    if pcd_artifact_hash is None:
        pcd_artifact_hash = normalized_artifacts.get("registered_map.pcd")
    if pcd_artifact_hash is None:
        raise ReviewBundleError(
            "session does not bind the source PCD artifact"
        )
    if pcd_artifact_hash != source_sha256:
        raise ReviewBundleError("session source PCD hash does not match")

    verified_artifacts = [source_filename]
    if keyframes_sha256 is not None:
        keyframe_artifact_hash = normalized_artifacts.get(
            keyframes_filename or "keyframes.jsonl"
        )
        if keyframe_artifact_hash is None:
            keyframe_artifact_hash = normalized_artifacts.get(
                "keyframes.jsonl"
            )
        if keyframe_artifact_hash is None:
            raise ReviewBundleError("session does not bind keyframes JSONL")
        if keyframe_artifact_hash != keyframes_sha256:
            raise ReviewBundleError("session keyframes hash does not match")
        verified_artifacts.append(keyframes_filename or "keyframes.jsonl")

    warnings: List[Dict[str, str]] = []
    if map_state == "registered_fast_lio_output_not_loop_optimized":
        warnings.append(
            {
                "code": "map_not_loop_optimized",
                "severity": "warning",
                "message": (
                    "Session declares registered FAST-LIO output without "
                    "loop optimization."
                ),
            }
        )
    if motion_commands is True:
        warnings.append(
            {
                "code": "motion_commands_published_during_mapping",
                "severity": "warning",
                "message": (
                    "Session declares that motion commands were published "
                    "while mapping."
                ),
            }
        )
    return (
        {
            "filename": path.name,
            "sha256": session_sha256,
            "schema": REGISTERED_SESSION_SCHEMA,
            "state": "complete",
            "source_frame": source_frame.strip(),
            "map_state": map_state,
            "point_count": point_count,
            "keyframe_count": declared_keyframes,
            "motion_commands_published": motion_commands,
            "verified_artifacts": verified_artifacts,
        },
        warnings,
    )


def _write_bytes(path: Path, payload: bytes) -> None:
    try:
        with path.open("xb") as handle:
            handle.write(payload)
            handle.flush()
            os.fsync(handle.fileno())
    except OSError as exc:
        raise ReviewBundleError("cannot write %s: %s" % (path, exc)) from exc


def _write_preview(
    staging: Path,
    cloud: PCDCloud,
    max_preview_points: int,
) -> Dict[str, Any]:
    x_offset, y_offset, z_offset = _xyz_offsets(cloud)
    preview_count = min(cloud.point_count, max_preview_points)
    source_indices = [
        min(
            cloud.point_count - 1,
            (index * cloud.point_count) // preview_count,
        )
        for index in range(preview_count)
    ]
    path = staging / PREVIEW_FILENAME
    digest = hashlib.sha256()
    try:
        with path.open("xb") as handle:
            for source_index in source_indices:
                row = cloud.rows[source_index]
                try:
                    packed = _FLOAT32_XYZ.pack(
                        float(row[x_offset]),
                        float(row[y_offset]),
                        float(row[z_offset]),
                    )
                except (OverflowError, struct.error) as exc:
                    raise ReviewBundleError(
                        "source point %d cannot be encoded as little-endian "
                        "float32"
                        % source_index
                    ) from exc
                handle.write(packed)
                digest.update(packed)
            handle.flush()
            os.fsync(handle.fileno())
    except ReviewBundleError:
        raise
    except OSError as exc:
        raise ReviewBundleError("cannot write preview: %s" % exc) from exc

    preview_sha256 = digest.hexdigest()
    checksum_name = PREVIEW_FILENAME + ".sha256"
    _write_bytes(
        staging / checksum_name,
        ("%s  %s\n" % (preview_sha256, PREVIEW_FILENAME)).encode("ascii"),
    )
    return {
        "path": PREVIEW_FILENAME,
        "sha256": preview_sha256,
        "sha256_path": checksum_name,
        "byte_size": preview_count * _FLOAT32_XYZ.size,
        "point_count": preview_count,
        "source_point_count": cloud.point_count,
        "encoding": "little-endian-float32-xyz",
        "stride_bytes": _FLOAT32_XYZ.size,
        "coordinate_mode": "absolute",
        "origin": [0.0, 0.0, 0.0],
        "sampling": "systematic-floor-index",
    }


def _write_review_document(staging: Path, document: Mapping[str, Any]) -> str:
    try:
        payload = (
            json.dumps(
                document,
                ensure_ascii=False,
                allow_nan=False,
                indent=2,
                sort_keys=True,
            )
            + "\n"
        ).encode("utf-8")
    except (TypeError, ValueError) as exc:
        raise ReviewBundleError("review document is not finite JSON") from exc
    if len(payload) > MAX_REVIEW_JSON_BYTES:
        raise ReviewBundleError(
            "review.json exceeds browser limit of %d bytes"
            % MAX_REVIEW_JSON_BYTES
        )
    _write_bytes(staging / REVIEW_FILENAME, payload)
    digest = hashlib.sha256(payload).hexdigest()
    _write_bytes(
        staging / "review.sha256",
        ("%s  %s\n" % (digest, REVIEW_FILENAME)).encode("ascii"),
    )
    return digest


def build_review_bundle(
    source_pcd: os.PathLike,
    output_directory: os.PathLike,
    *,
    keyframes_path: Optional[os.PathLike] = None,
    session_path: Optional[os.PathLike] = None,
    max_preview_points: int = MAX_PREVIEW_POINTS,
) -> Mapping[str, Any]:
    """Build a deterministic review bundle and atomically publish it.

    The requested output directory must not exist.  Source inputs are never
    copied or modified; their exact byte hashes are bound into ``review.json``.
    """

    if (
        not _is_int(max_preview_points)
        or max_preview_points < 1
        or max_preview_points > MAX_PREVIEW_POINTS
    ):
        raise ReviewBundleError(
            "max_preview_points must be an integer in [1, %d]"
            % MAX_PREVIEW_POINTS
        )

    source = Path(source_pcd).expanduser().resolve()
    if not source.is_file():
        raise ReviewBundleError(
            "source PCD is not a regular file: %s" % source
        )
    requested_output = Path(output_directory).expanduser()
    if requested_output.exists() or requested_output.is_symlink():
        raise ReviewBundleError("output directory must not already exist")
    output = requested_output.resolve()
    if output == Path(output.anchor) or output == Path.home().resolve():
        raise ReviewBundleError("refusing filesystem root or home as output")
    try:
        source.relative_to(output)
    except ValueError:
        pass
    else:
        raise ReviewBundleError(
            "source PCD must not be inside output directory"
        )

    source_sha256_before = _sha256_file(source)
    try:
        cloud = read_pcd(source)
    except PCDDataError:
        raise
    source_sha256 = _sha256_file(source)
    if source_sha256_before != source_sha256:
        raise ReviewBundleError("source PCD changed while it was being read")
    try:
        source_byte_size = source.stat().st_size
    except OSError as exc:
        raise ReviewBundleError("cannot inspect source PCD: %s" % exc) from exc
    if source_byte_size <= 0 or source_byte_size > MAX_SAFE_JSON_INTEGER:
        raise ReviewBundleError(
            "source PCD byte size is outside the safe range"
        )
    quality = _analyze_cloud(cloud)

    keyframes: Optional[List[Mapping[str, Any]]] = None
    keyframes_sha256: Optional[str] = None
    resolved_keyframes: Optional[Path] = None
    trajectory: Optional[Dict[str, Any]] = None
    if keyframes_path is not None:
        resolved_keyframes = Path(keyframes_path).expanduser().resolve()
        keyframes, keyframes_sha256 = _read_keyframes(
            resolved_keyframes, cloud.point_count
        )
        trajectory = _trajectory_report(keyframes)

    warnings: List[Dict[str, str]] = []
    connectivity = quality["connectivity_4"]
    if connectivity["component_count"] > 1:
        warnings.append(
            {
                "code": "xy_grid_disconnected",
                "severity": "warning",
                "message": (
                    "The occupied 1 m XY grid has multiple 4-connected "
                    "components."
                ),
            }
        )
    occupied_cells = quality["density_1m"]["occupied_cell_count"]
    isolated_cells = connectivity["isolated_cell_count"]
    if isolated_cells / occupied_cells > 0.02:
        warnings.append(
            {
                "code": "xy_grid_isolated_cells",
                "severity": "warning",
                "message": "More than 2% of occupied 1 m cells are isolated.",
            }
        )
    if cloud.point_count > max_preview_points:
        warnings.append(
            {
                "code": "preview_sampled",
                "severity": "info",
                "message": (
                    "Preview is a deterministic sample, not the full "
                    "source cloud."
                ),
            }
        )

    keyframe_evidence: Optional[Dict[str, Any]] = None
    if resolved_keyframes is not None and keyframes_sha256 is not None:
        keyframe_evidence = {
            "filename": resolved_keyframes.name,
            "sha256": keyframes_sha256,
            "record_count": len(keyframes or ()),
        }
        assert trajectory is not None
        if trajectory["sync_error_ns"]["max"] > 200_000_000:
            warnings.append(
                {
                    "code": "keyframe_sync_error_high",
                    "severity": "warning",
                    "message": (
                        "At least one keyframe exceeds 200 ms odometry "
                        "sync error."
                    ),
                }
            )
        interval_max = trajectory["interval_ns"]["max"]
        if interval_max is not None and interval_max > 3_000_000_000:
            warnings.append(
                {
                    "code": "keyframe_interval_high",
                    "severity": "warning",
                    "message": (
                        "At least one keyframe interval exceeds 3 seconds."
                    ),
                }
            )
        if trajectory["truncated_keyframe_count"] > 0:
            warnings.append(
                {
                    "code": "keyframes_truncated",
                    "severity": "warning",
                    "message": (
                        "One or more keyframes reached the configured "
                        "point cap."
                    ),
                }
            )
        if (
            trajectory["distance_m"] >= 10.0
            and trajectory["loop_closure_gap_m"]
            > max(2.0, trajectory["distance_m"] * 0.05)
        ):
            warnings.append(
                {
                    "code": "trajectory_not_closed",
                    "severity": "warning",
                    "message": (
                        "Trajectory endpoint is far from its start; "
                        "inspect loop closure."
                    ),
                }
            )
    else:
        warnings.append(
            {
                "code": "keyframes_missing",
                "severity": "warning",
                "message": (
                    "PCD-only review cannot attribute divergence to a "
                    "keyframe."
                ),
            }
        )

    session_evidence: Optional[Dict[str, Any]] = None
    if session_path is not None:
        resolved_session = Path(session_path).expanduser().resolve()
        session_evidence, session_warnings = _session_report(
            resolved_session,
            source.name,
            source_sha256,
            cloud.point_count,
            resolved_keyframes.name if resolved_keyframes else None,
            keyframes_sha256,
            len(keyframes) if keyframes is not None else None,
        )
        warnings.extend(session_warnings)
    else:
        warnings.append(
            {
                "code": "session_metadata_missing",
                "severity": "warning",
                "message": "Recorder session metadata was not supplied.",
            }
        )

    output.parent.mkdir(parents=True, exist_ok=True)
    staging = Path(
        tempfile.mkdtemp(
            prefix=".%s.review-staging." % output.name,
            dir=str(output.parent),
        )
    )
    published = False
    try:
        preview = _write_preview(staging, cloud, max_preview_points)
        warnings.sort(
            key=lambda warning: (warning["code"], warning["severity"])
        )
        review: Dict[str, Any] = {
            "schema": REVIEW_BUNDLE_SCHEMA,
            "source": {
                "filename": source.name,
                "sha256": source_sha256,
                "byte_size": source_byte_size,
                "point_count": cloud.point_count,
                "pcd_encoding": cloud.data_encoding,
                "fields": [field.name for field in cloud.fields],
                "bounds": quality["bounds"],
                "height": quality["height"],
            },
            "preview": preview,
            "quality": {
                "density_1m": quality["density_1m"],
                "connectivity_4": quality["connectivity_4"],
            },
            "evidence": {
                "keyframes": keyframe_evidence,
                "session": session_evidence,
            },
            "trajectory": trajectory,
            "warnings": warnings,
        }
        review_sha256 = _write_review_document(staging, review)
        if output.exists() or output.is_symlink():
            raise ReviewBundleError("output directory appeared during build")
        os.rename(str(staging), str(output))
        published = True
        result = dict(review)
        result["review_sha256"] = review_sha256
        return result
    except ReviewBundleError:
        raise
    except OSError as exc:
        raise ReviewBundleError(
            "cannot publish review bundle: %s" % exc
        ) from exc
    finally:
        if not published and staging.exists():
            shutil.rmtree(str(staging))

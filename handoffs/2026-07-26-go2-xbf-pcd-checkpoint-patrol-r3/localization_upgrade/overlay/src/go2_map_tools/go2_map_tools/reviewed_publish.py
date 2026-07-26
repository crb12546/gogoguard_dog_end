"""Atomic publication and fail-closed verification of reviewed map bundles."""

from __future__ import annotations

from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import shutil
import tempfile
from typing import Any, Dict, Mapping, Optional, Sequence, Tuple

from .annotation_filter import (
    CLEANED_PCD_FILENAME,
    FILTER_REPORT_FILENAME,
    STABLE_LAYER_PCD_FILENAME,
    AnnotationFilterError,
    filter_pcd_with_annotations,
)
from .descriptor import (
    DescriptorError,
    PolarDescriptorConfig,
    build_descriptor_index,
    verify_map_bundle,
)
from .pcd import read_pcd
from .tiles import MapManifestError, build_tiled_map, sha256_file


REVIEWED_PUBLICATION_SCHEMA = "go2.reviewed_map_publication/v1"
REVIEWED_PUBLICATION_FILENAME = "reviewed_map_publication.json"
REVIEWED_PUBLICATION_CHECKSUM_FILENAME = REVIEWED_PUBLICATION_FILENAME + ".sha256"
REVIEW_ASSET_DIRECTORY = "review_assets"


class ReviewedMapPublicationError(ValueError):
    """Raised when a reviewed map cannot be published or verified."""


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


def _write_exclusive(path: Path, payload: bytes) -> None:
    try:
        with path.open("xb") as handle:
            handle.write(payload)
            handle.flush()
            os.fsync(handle.fileno())
    except OSError as exc:
        raise ReviewedMapPublicationError(
            "cannot write reviewed map artifact %s: %s" % (path, exc)
        ) from exc


def _copy_asset(source: Path, destination: Path) -> None:
    if not source.is_file():
        raise ReviewedMapPublicationError("review asset is missing: %s" % source)
    try:
        shutil.copy2(str(source), str(destination))
    except OSError as exc:
        raise ReviewedMapPublicationError(
            "cannot copy review asset %s: %s" % (source, exc)
        ) from exc


def _asset(path: str, digest: str, point_count: Optional[int] = None) -> Dict[str, Any]:
    result: Dict[str, Any] = {"path": path, "sha256": digest}
    if point_count is not None:
        result["point_count"] = int(point_count)
    return result


def publish_reviewed_map_bundle(
    source_pcd: os.PathLike,
    annotation_json: os.PathLike,
    output_directory: os.PathLike,
    *,
    map_id: str,
    frame_id: str = "map",
    tile_size_m: float = 20.0,
    voxel_size_m: float = 0.20,
    descriptor_config: Optional[PolarDescriptorConfig] = None,
    minimum_stable_points: int = 1000,
) -> Mapping[str, Any]:
    """Filter, compile, bind, verify, and atomically publish one map version."""

    source = Path(source_pcd).expanduser().resolve()
    annotations = Path(annotation_json).expanduser().resolve()
    requested_output = Path(output_directory).expanduser()
    if requested_output.exists() or requested_output.is_symlink():
        raise ReviewedMapPublicationError(
            "reviewed map output must not already exist; versions are immutable"
        )
    output = requested_output.resolve()
    if output == Path(output.anchor) or output == Path.home().resolve():
        raise ReviewedMapPublicationError(
            "refusing filesystem root or home as reviewed map output"
        )
    minimum_stable = int(minimum_stable_points)
    if minimum_stable <= 0:
        raise ReviewedMapPublicationError("minimum_stable_points must be positive")
    if not map_id or "/" in map_id or "\\" in map_id:
        raise ReviewedMapPublicationError("map_id must be a safe non-empty name")

    output.parent.mkdir(parents=True, exist_ok=True)
    staging = Path(
        tempfile.mkdtemp(
            prefix=".%s.reviewed-publish." % output.name,
            dir=str(output.parent),
        )
    )
    try:
        filtered_root = staging / "filtered"
        report = filter_pcd_with_annotations(source, annotations, filtered_root)
        stable_count = int(report["stable_layer"]["point_count"])
        if stable_count < minimum_stable:
            raise ReviewedMapPublicationError(
                "stable layer has %d points, below required %d"
                % (stable_count, minimum_stable)
            )

        bundle = staging / "bundle"
        build_tiled_map(
            filtered_root / CLEANED_PCD_FILENAME,
            bundle,
            map_id=map_id,
            frame_id=frame_id,
            tile_size_m=tile_size_m,
            voxel_size_m=voxel_size_m,
            overwrite=False,
        )
        review_assets = bundle / REVIEW_ASSET_DIRECTORY
        review_assets.mkdir()
        copied_assets = (
            (
                annotations,
                review_assets / "annotations.json",
            ),
            (
                filtered_root / CLEANED_PCD_FILENAME,
                review_assets / CLEANED_PCD_FILENAME,
            ),
            (
                filtered_root / (CLEANED_PCD_FILENAME + ".sha256"),
                review_assets / (CLEANED_PCD_FILENAME + ".sha256"),
            ),
            (
                filtered_root / STABLE_LAYER_PCD_FILENAME,
                review_assets / STABLE_LAYER_PCD_FILENAME,
            ),
            (
                filtered_root / (STABLE_LAYER_PCD_FILENAME + ".sha256"),
                review_assets / (STABLE_LAYER_PCD_FILENAME + ".sha256"),
            ),
            (
                filtered_root / FILTER_REPORT_FILENAME,
                review_assets / FILTER_REPORT_FILENAME,
            ),
            (
                filtered_root / (FILTER_REPORT_FILENAME + ".sha256"),
                review_assets / (FILTER_REPORT_FILENAME + ".sha256"),
            ),
        )
        for source_path, destination_path in copied_assets:
            _copy_asset(source_path, destination_path)

        index = build_descriptor_index(
            bundle / "manifest.json",
            bundle / "descriptor_index.json",
            descriptor_config,
            source_layer_pcd=(review_assets / STABLE_LAYER_PCD_FILENAME),
        )
        manifest, verified_index = verify_map_bundle(bundle / "manifest.json")
        if verified_index.source_layer is None:
            raise ReviewedMapPublicationError(
                "descriptor index did not bind the stable layer"
            )

        manifest_path = bundle / "manifest.json"
        index_path = bundle / "descriptor_index.json"
        report_path = review_assets / FILTER_REPORT_FILENAME
        annotation_path = review_assets / "annotations.json"
        cleaned_path = review_assets / CLEANED_PCD_FILENAME
        stable_path = review_assets / STABLE_LAYER_PCD_FILENAME
        publication = {
            "schema": REVIEWED_PUBLICATION_SCHEMA,
            "created_utc": datetime.now(timezone.utc)
            .isoformat()
            .replace("+00:00", "Z"),
            "map_id": manifest.map_id,
            "frame_id": manifest.frame_id,
            "source_pcd": {
                "filename": report["source"]["filename"],
                "sha256": report["source"]["sha256"],
                "point_count": report["source"]["point_count"],
            },
            "annotation": {
                "path": "%s/annotations.json" % REVIEW_ASSET_DIRECTORY,
                "schema": report["annotation_export"]["schema"],
                "revision": report["annotation_export"]["revision"],
                "sha256": sha256_file(annotation_path),
            },
            "cleaned_static_map": _asset(
                "%s/%s" % (REVIEW_ASSET_DIRECTORY, CLEANED_PCD_FILENAME),
                sha256_file(cleaned_path),
                read_pcd(cleaned_path).point_count,
            ),
            "stable_layer": _asset(
                "%s/%s" % (REVIEW_ASSET_DIRECTORY, STABLE_LAYER_PCD_FILENAME),
                sha256_file(stable_path),
                read_pcd(stable_path).point_count,
            ),
            "filter_report": _asset(
                "%s/%s" % (REVIEW_ASSET_DIRECTORY, FILTER_REPORT_FILENAME),
                sha256_file(report_path),
            ),
            "compiled_map": {
                "manifest_path": "manifest.json",
                "manifest_sha256": sha256_file(manifest_path),
                "descriptor_index_path": "descriptor_index.json",
                "descriptor_index_sha256": sha256_file(index_path),
                "tile_count": len(manifest.tiles),
                "descriptor_entry_count": len(index.entries),
                "tracking_layer": "cleaned_static_map",
                "global_retrieval_layer": "stable_layer",
            },
            "deployment_ready": False,
            "deployment_notice_zh": (
                "本制品完成了哈希绑定和算法层分离，但仍须通过真狗外参、"
                "静止全局定位、rosbag 回放和路线审核后才能授权运动。"
            ),
        }
        payload = _json_bytes(publication)
        publication_path = bundle / REVIEWED_PUBLICATION_FILENAME
        _write_exclusive(publication_path, payload)
        publication_sha256 = hashlib.sha256(payload).hexdigest()
        _write_exclusive(
            bundle / REVIEWED_PUBLICATION_CHECKSUM_FILENAME,
            ("%s  %s\n" % (publication_sha256, REVIEWED_PUBLICATION_FILENAME)).encode(
                "ascii"
            ),
        )
        verify_reviewed_map_bundle(publication_path)

        if output.exists() or output.is_symlink():
            raise ReviewedMapPublicationError(
                "reviewed map output appeared during publication"
            )
        os.rename(str(bundle), str(output))
        result = dict(publication)
        result["publication_sha256"] = publication_sha256
        result["output_directory"] = str(output)
        return result
    except (
        AnnotationFilterError,
        DescriptorError,
        MapManifestError,
        ReviewedMapPublicationError,
    ):
        raise
    except OSError as exc:
        raise ReviewedMapPublicationError(
            "cannot publish reviewed map: %s" % exc
        ) from exc
    finally:
        if staging.exists():
            shutil.rmtree(str(staging))


def _strict_object(pairs: Sequence[Tuple[str, Any]]) -> Dict[str, Any]:
    result: Dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise ReviewedMapPublicationError(
                "publication JSON contains duplicate key %r" % key
            )
        result[key] = value
    return result


def _safe_asset(root: Path, entry: Mapping[str, Any], label: str) -> Path:
    if not isinstance(entry, Mapping):
        raise ReviewedMapPublicationError("%s must be an object" % label)
    relative = Path(str(entry.get("path", "")))
    if (
        relative.is_absolute()
        or ".." in relative.parts
        or relative.as_posix() in ("", ".")
    ):
        raise ReviewedMapPublicationError("%s path must be safe and relative" % label)
    path = (root / relative).resolve()
    try:
        path.relative_to(root)
    except ValueError as exc:
        raise ReviewedMapPublicationError("%s path escapes bundle" % label) from exc
    if not path.is_file():
        raise ReviewedMapPublicationError("%s asset is missing" % label)
    expected = str(entry.get("sha256", "")).lower()
    if len(expected) != 64 or sha256_file(path) != expected:
        raise ReviewedMapPublicationError("%s SHA-256 mismatch" % label)
    if "point_count" in entry:
        if read_pcd(path).point_count != int(entry["point_count"]):
            raise ReviewedMapPublicationError("%s point count mismatch" % label)
    return path


def verify_reviewed_map_bundle(
    publication_path: os.PathLike,
) -> Mapping[str, Any]:
    """Verify the complete reviewed publication and every cross-file anchor."""

    path = Path(publication_path)
    if path.is_dir():
        path = path / REVIEWED_PUBLICATION_FILENAME
    path = path.resolve()
    root = path.parent
    try:
        payload = path.read_bytes()
        document = json.loads(
            payload.decode("utf-8"),
            parse_constant=lambda value: (_reject_constant(value)),
            object_pairs_hook=_strict_object,
        )
    except ReviewedMapPublicationError:
        raise
    except (OSError, UnicodeError, ValueError, json.JSONDecodeError) as exc:
        raise ReviewedMapPublicationError(
            "cannot parse reviewed map publication: %s" % exc
        ) from exc
    if not isinstance(document, Mapping):
        raise ReviewedMapPublicationError("publication must be an object")
    expected_root = {
        "schema",
        "created_utc",
        "map_id",
        "frame_id",
        "source_pcd",
        "annotation",
        "cleaned_static_map",
        "stable_layer",
        "filter_report",
        "compiled_map",
        "deployment_ready",
        "deployment_notice_zh",
    }
    if set(document.keys()) != expected_root:
        raise ReviewedMapPublicationError("publication fields do not match schema")
    if document["schema"] != REVIEWED_PUBLICATION_SCHEMA:
        raise ReviewedMapPublicationError("unsupported publication schema")
    checksum_path = root / REVIEWED_PUBLICATION_CHECKSUM_FILENAME
    try:
        tokens = checksum_path.read_text(encoding="ascii").split()
    except OSError as exc:
        raise ReviewedMapPublicationError(
            "cannot read publication checksum: %s" % exc
        ) from exc
    actual_publication_sha = hashlib.sha256(payload).hexdigest()
    if (
        len(tokens) != 2
        or tokens[1] != REVIEWED_PUBLICATION_FILENAME
        or tokens[0].lower() != actual_publication_sha
    ):
        raise ReviewedMapPublicationError("publication checksum mismatch")

    annotation_path = _safe_asset(root, document["annotation"], "annotation")
    cleaned_path = _safe_asset(
        root, document["cleaned_static_map"], "cleaned_static_map"
    )
    stable_path = _safe_asset(root, document["stable_layer"], "stable_layer")
    report_path = _safe_asset(root, document["filter_report"], "filter_report")
    report = json.loads(report_path.read_text(encoding="utf-8"))
    source_binding = document["source_pcd"]
    if (
        report.get("source", {}).get("sha256") != source_binding.get("sha256")
        or report.get("source", {}).get("point_count")
        != source_binding.get("point_count")
        or report.get("annotation_export", {}).get("sha256")
        != document["annotation"].get("sha256")
        or report.get("annotation_export", {}).get("revision")
        != document["annotation"].get("revision")
        or report.get("cleaned_static_map", {}).get("sha256")
        != document["cleaned_static_map"].get("sha256")
        or report.get("stable_layer", {}).get("sha256")
        != document["stable_layer"].get("sha256")
    ):
        raise ReviewedMapPublicationError(
            "filter report provenance does not match publication"
        )
    # Ensure the copied annotation still declares the same revision/schema.
    annotation_document = json.loads(annotation_path.read_text(encoding="utf-8"))
    annotation_schema = annotation_document.get("schema")
    annotation_revision = (
        annotation_document.get("revision")
        if annotation_schema == "go2.map_review_annotations/v2"
        else "legacy-v1-" + sha256_file(annotation_path)[:12]
    )
    if annotation_schema != document["annotation"].get(
        "schema"
    ) or annotation_revision != document["annotation"].get("revision"):
        raise ReviewedMapPublicationError(
            "annotation identity does not match publication"
        )

    compiled = document["compiled_map"]
    manifest_path = _safe_asset(
        root,
        {
            "path": compiled.get("manifest_path"),
            "sha256": compiled.get("manifest_sha256"),
        },
        "compiled_map.manifest",
    )
    index_path = _safe_asset(
        root,
        {
            "path": compiled.get("descriptor_index_path"),
            "sha256": compiled.get("descriptor_index_sha256"),
        },
        "compiled_map.descriptor_index",
    )
    manifest, index = verify_map_bundle(manifest_path)
    if (
        manifest.map_id != document["map_id"]
        or manifest.frame_id != document["frame_id"]
        or manifest.source_sha256 != document["cleaned_static_map"].get("sha256")
        or manifest.source_filename != cleaned_path.name
        or len(manifest.tiles) != int(compiled.get("tile_count", -1))
        or len(index.entries) != int(compiled.get("descriptor_entry_count", -1))
    ):
        raise ReviewedMapPublicationError(
            "compiled map identity does not match publication"
        )
    if (
        compiled.get("tracking_layer") != "cleaned_static_map"
        or compiled.get("global_retrieval_layer") != "stable_layer"
        or index.source_layer is None
        or index.source_layer.path != stable_path.relative_to(root).as_posix()
        or index.source_layer.sha256 != document["stable_layer"].get("sha256")
        or index.source_layer.point_count != document["stable_layer"].get("point_count")
        or index_path.name != "descriptor_index.json"
    ):
        raise ReviewedMapPublicationError(
            "descriptor stable-layer binding does not match publication"
        )
    result = dict(document)
    result["publication_sha256"] = actual_publication_sha
    result["root_directory"] = str(root)
    return result


def _reject_constant(value: str) -> None:
    raise ReviewedMapPublicationError(
        "publication JSON contains non-finite constant %s" % value
    )

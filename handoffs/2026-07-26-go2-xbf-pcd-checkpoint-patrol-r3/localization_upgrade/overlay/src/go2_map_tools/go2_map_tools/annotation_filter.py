"""Fail-closed publication of reviewed static-map layers.

Annotation v1 remains readable.  Its XY circles are promoted to legacy
all-height ROIs.  Annotation v2 adds object-level cylinders, oriented boxes,
and polygon prisms with explicit Z ranges.

The cleaned static map retains every unannotated point and removes only
explicit ``*_exclude`` ROIs.  The stable layer contains cleaned points that
fall inside at least one ``stable_include`` ROI.  The source PCD is immutable.
"""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import tempfile
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple, cast

from .pcd import (
    PCDCloud,
    PCDDataError,
    read_pcd,
    write_pcd_ascii,
    write_pcd_binary,
)


ANNOTATION_EXPORT_SCHEMA_V1 = "go2.map_review_annotations/v1"
ANNOTATION_EXPORT_SCHEMA_V2 = "go2.map_review_annotations/v2"
# Backward-compatible public name used by callers that still construct v1.
ANNOTATION_EXPORT_SCHEMA = ANNOTATION_EXPORT_SCHEMA_V1
FILTER_REPORT_SCHEMA = "go2.map_annotation_filter_report/v2"
CLEANED_PCD_FILENAME = "cleaned_static_map.pcd"
FILTERED_PCD_FILENAME = CLEANED_PCD_FILENAME
STABLE_LAYER_PCD_FILENAME = "stable_layer.pcd"
FILTER_REPORT_FILENAME = "filter_report.json"
MAX_ANNOTATION_JSON_BYTES = 16 * 1024 * 1024
MAX_ANNOTATIONS = 100_000
MAX_EXCLUDE_MASKS = 10_000
MAX_MASK_BUCKET_REFERENCES = 2_000_000
MAX_ABS_COORDINATE_M = 100_000.0
_SHA256 = re.compile(r"^[0-9a-f]{64}$")

_CATEGORY_POLICY_V1: Mapping[str, Tuple[str, str]] = {
    "stable_include": ("trusted_region_hint", "none"),
    "dynamic_exclude": (
        "offline_map_filter_mask",
        "exclude_xy_circle_all_z",
    ),
    "vegetation_exclude": (
        "offline_map_filter_mask",
        "exclude_xy_circle_all_z",
    ),
    "parking_exclude": (
        "offline_map_filter_mask",
        "exclude_xy_circle_all_z",
    ),
    "low_confidence": ("review_finding", "none"),
    "repetitive_geometry": ("review_finding", "none"),
    "ghosting": ("review_finding", "none"),
    "drift_suspect": ("review_finding", "none"),
    "sparse": ("review_finding", "none"),
    "blind_zone": ("review_finding", "none"),
    "manual_review": ("review_finding", "none"),
}
_CATEGORY_POLICY_V2: Mapping[str, Tuple[str, str]] = {
    "stable_include": ("stable_layer_roi", "extract_to_stable_layer"),
    "dynamic_exclude": ("static_map_exclude_roi", "exclude_from_cleaned_map"),
    "vegetation_exclude": (
        "static_map_exclude_roi",
        "exclude_from_cleaned_map",
    ),
    "parking_exclude": ("static_map_exclude_roi", "exclude_from_cleaned_map"),
    "low_confidence": ("review_finding", "none"),
    "repetitive_geometry": ("review_finding", "none"),
    "ghosting": ("review_finding", "none"),
    "drift_suspect": ("review_finding", "none"),
    "sparse": ("review_finding", "none"),
    "blind_zone": ("review_finding", "none"),
    "manual_review": ("review_finding", "none"),
}
_CATEGORY_POLICY = _CATEGORY_POLICY_V1
EXCLUDE_CATEGORIES = frozenset(
    category
    for category, policy in _CATEGORY_POLICY.items()
    if policy[0] == "offline_map_filter_mask"
)


class AnnotationFilterError(ValueError):
    """Raised when annotations cannot be safely applied to a source map."""


@dataclass(frozen=True)
class MapAnnotation:
    id: str
    category: str
    x: float
    y: float
    z: Optional[float]
    radius_m: float
    note: Optional[str]
    geometry_type: str = "xy_circle_all_z"
    z_min_m: Optional[float] = None
    z_max_m: Optional[float] = None
    size_x_m: Optional[float] = None
    size_y_m: Optional[float] = None
    yaw_rad: float = 0.0
    vertices_xy_m: Tuple[Tuple[float, float], ...] = ()

    @property
    def is_exclude_mask(self) -> bool:
        return self.category in EXCLUDE_CATEGORIES

    @property
    def is_stable_roi(self) -> bool:
        return self.category == "stable_include"

    def xy_bounds(self) -> Tuple[float, float, float, float]:
        if self.geometry_type in ("xy_circle_all_z", "cylinder"):
            return (
                self.x - self.radius_m,
                self.y - self.radius_m,
                self.x + self.radius_m,
                self.y + self.radius_m,
            )
        if self.geometry_type == "oriented_box":
            half_x = float(self.size_x_m) * 0.5
            half_y = float(self.size_y_m) * 0.5
            cosine = abs(math.cos(self.yaw_rad))
            sine = abs(math.sin(self.yaw_rad))
            extent_x = cosine * half_x + sine * half_y
            extent_y = sine * half_x + cosine * half_y
            return (
                self.x - extent_x,
                self.y - extent_y,
                self.x + extent_x,
                self.y + extent_y,
            )
        xs = [vertex[0] for vertex in self.vertices_xy_m]
        ys = [vertex[1] for vertex in self.vertices_xy_m]
        return min(xs), min(ys), max(xs), max(ys)

    def contains(self, x: float, y: float, z: float) -> bool:
        if self.z_min_m is not None and z < self.z_min_m:
            return False
        if self.z_max_m is not None and z > self.z_max_m:
            return False
        if self.geometry_type in ("xy_circle_all_z", "cylinder"):
            dx = x - self.x
            dy = y - self.y
            return dx * dx + dy * dy <= self.radius_m * self.radius_m
        if self.geometry_type == "oriented_box":
            cosine = math.cos(self.yaw_rad)
            sine = math.sin(self.yaw_rad)
            dx = x - self.x
            dy = y - self.y
            local_x = cosine * dx + sine * dy
            local_y = -sine * dx + cosine * dy
            return (
                abs(local_x) <= float(self.size_x_m) * 0.5
                and abs(local_y) <= float(self.size_y_m) * 0.5
            )
        inside = False
        vertices = self.vertices_xy_m
        previous = vertices[-1]
        for current in vertices:
            x1, y1 = previous
            x2, y2 = current
            if ((y1 > y) != (y2 > y)) and (x <= (x2 - x1) * (y - y1) / (y2 - y1) + x1):
                inside = not inside
            previous = current
        return inside

    def to_report(self) -> Dict[str, Any]:
        result: Dict[str, Any] = {
            "id": self.id,
            "category": self.category,
            "geometry_type": self.geometry_type,
        }
        if self.geometry_type == "xy_circle_all_z":
            result.update(
                {
                    "center_m": {"x": self.x, "y": self.y, "z": self.z},
                    "radius_m": self.radius_m,
                }
            )
        else:
            roi: Dict[str, Any] = {
                "shape": self.geometry_type,
                "z_range_m": {"min": self.z_min_m, "max": self.z_max_m},
            }
            if self.geometry_type == "cylinder":
                roi.update(
                    {
                        "center_xy_m": {"x": self.x, "y": self.y},
                        "radius_m": self.radius_m,
                    }
                )
            elif self.geometry_type == "oriented_box":
                roi.update(
                    {
                        "center_xy_m": {"x": self.x, "y": self.y},
                        "size_xy_m": {
                            "x": self.size_x_m,
                            "y": self.size_y_m,
                        },
                        "yaw_rad": self.yaw_rad,
                    }
                )
            else:
                roi["vertices_xy_m"] = [
                    [vertex[0], vertex[1]] for vertex in self.vertices_xy_m
                ]
            result["roi"] = roi
        if self.note is not None:
            result["note"] = self.note
        return result


@dataclass(frozen=True)
class AnnotationExport:
    path: Path
    sha256: str
    map_sha256: str
    map_filename: str
    exported_at_utc: str
    schema: str
    revision: str
    annotations: Tuple[MapAnnotation, ...]


def _sha256_file(path: Path, label: str) -> str:
    digest = hashlib.sha256()
    try:
        with path.open("rb") as handle:
            for block in iter(lambda: handle.read(1024 * 1024), b""):
                digest.update(block)
    except OSError as exc:
        raise AnnotationFilterError(
            "cannot hash %s %s: %s" % (label, path, exc)
        ) from exc
    return digest.hexdigest()


def _reject_json_constant(value: str) -> None:
    raise AnnotationFilterError(
        "annotation JSON contains non-standard constant %s" % value
    )


def _unique_json_object(pairs: Sequence[Tuple[str, Any]]) -> Dict[str, Any]:
    result: Dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise AnnotationFilterError(
                "annotation JSON contains duplicate key %r" % key
            )
        result[key] = value
    return result


def _mapping(value: Any, label: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise AnnotationFilterError("%s must be an object" % label)
    return cast(Mapping[str, Any], value)


def _exact_keys(
    value: Mapping[str, Any],
    required: Sequence[str],
    label: str,
    optional: Sequence[str] = (),
) -> None:
    required_set = set(required)
    allowed = required_set | set(optional)
    missing = sorted(required_set - set(value))
    unknown = sorted(set(value) - allowed)
    if missing:
        raise AnnotationFilterError(
            "%s is missing keys: %s" % (label, ", ".join(missing))
        )
    if unknown:
        raise AnnotationFilterError(
            "%s contains unsupported keys: %s" % (label, ", ".join(unknown))
        )


def _nonempty_string(value: Any, label: str, maximum_length: int) -> str:
    if not isinstance(value, str) or not value or len(value) > maximum_length:
        raise AnnotationFilterError(
            "%s must be a non-empty string no longer than %d characters"
            % (label, maximum_length)
        )
    return value


def _finite_number(value: Any, label: str) -> float:
    if isinstance(value, bool):
        raise AnnotationFilterError("%s must be a finite number" % label)
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise AnnotationFilterError("%s must be a finite number" % label) from exc
    if not math.isfinite(result):
        raise AnnotationFilterError("%s must be a finite number" % label)
    return result


def _utc_timestamp(value: Any) -> str:
    timestamp = _nonempty_string(value, "exported_at_utc", 64)
    if not timestamp.endswith("Z"):
        raise AnnotationFilterError(
            "exported_at_utc must be an ISO-8601 UTC timestamp ending in Z"
        )
    try:
        parsed = datetime.fromisoformat(timestamp[:-1] + "+00:00")
    except ValueError as exc:
        raise AnnotationFilterError(
            "exported_at_utc must be an ISO-8601 UTC timestamp"
        ) from exc
    if parsed.tzinfo is None:
        raise AnnotationFilterError("exported_at_utc must include a timezone")
    return timestamp


def _validate_coordinate_system(value: Any, schema: str) -> None:
    coordinate_system = _mapping(value, "coordinate_system")
    _exact_keys(
        coordinate_system,
        (
            "frame_id",
            "coordinate_mode",
            "linear_unit",
            "handedness",
            "axes",
            "region_geometry",
        ),
        "coordinate_system",
    )
    expected_scalars = {
        "frame_id": "map",
        "coordinate_mode": "absolute",
        "linear_unit": "metre",
        "handedness": "right_handed",
    }
    for key, expected in expected_scalars.items():
        if coordinate_system.get(key) != expected:
            raise AnnotationFilterError(
                "coordinate_system.%s must be %r" % (key, expected)
            )

    axes = _mapping(coordinate_system["axes"], "coordinate_system.axes")
    _exact_keys(axes, ("x", "y", "z"), "coordinate_system.axes")
    if axes != {"x": "map +X", "y": "map +Y", "z": "map +Z"}:
        raise AnnotationFilterError(
            "coordinate_system.axes does not match the map frame"
        )

    geometry = _mapping(
        coordinate_system["region_geometry"],
        "coordinate_system.region_geometry",
    )
    if schema == ANNOTATION_EXPORT_SCHEMA_V1:
        _exact_keys(
            geometry,
            (
                "type",
                "center_xy_fields",
                "display_height_field",
                "radius_field",
            ),
            "coordinate_system.region_geometry",
        )
        if (
            geometry.get("type") != "xy_circle_all_z"
            or geometry.get("center_xy_fields") != ["center_m.x", "center_m.y"]
            or geometry.get("display_height_field") != "center_m.z"
            or geometry.get("radius_field") != "radius_m"
        ):
            raise AnnotationFilterError(
                "coordinate_system.region_geometry is unsupported"
            )
        return
    _exact_keys(
        geometry,
        ("type", "supported_shapes", "z_range_fields"),
        "coordinate_system.region_geometry",
    )
    if (
        geometry.get("type") != "object_roi_v2"
        or geometry.get("supported_shapes")
        != ["cylinder", "oriented_box", "polygon_prism"]
        or geometry.get("z_range_fields") != ["roi.z_range_m.min", "roi.z_range_m.max"]
    ):
        raise AnnotationFilterError("coordinate_system.region_geometry is unsupported")


def _validate_category_semantics(value: Any, schema: str) -> None:
    semantics = _mapping(value, "category_semantics")
    policy = (
        _CATEGORY_POLICY_V1
        if schema == ANNOTATION_EXPORT_SCHEMA_V1
        else _CATEGORY_POLICY_V2
    )
    if set(semantics) != set(policy):
        raise AnnotationFilterError(
            "category_semantics must contain exactly the supported categories"
        )
    for category, expected in policy.items():
        entry = _mapping(
            semantics[category],
            "category_semantics.%s" % category,
        )
        _exact_keys(
            entry,
            (
                "label_zh",
                "role",
                "offline_map_action",
                "description_zh",
            ),
            "category_semantics.%s" % category,
        )
        _nonempty_string(
            entry["label_zh"],
            "category_semantics.%s.label_zh" % category,
            200,
        )
        _nonempty_string(
            entry["description_zh"],
            "category_semantics.%s.description_zh" % category,
            2000,
        )
        if entry.get("role") != expected[0]:
            raise AnnotationFilterError(
                "category_semantics.%s.role must be %r" % (category, expected[0])
            )
        if entry.get("offline_map_action") != expected[1]:
            raise AnnotationFilterError(
                "category_semantics.%s.offline_map_action must be %r"
                % (category, expected[1])
            )


def _validate_safety(value: Any) -> None:
    safety = _mapping(value, "safety")
    _exact_keys(
        safety,
        (
            "source_map_mutation",
            "mask_application_requires_preview",
            "filtered_map_requires_new_sha256",
            "localization_constraint_notice_zh",
        ),
        "safety",
    )
    if safety.get("source_map_mutation") != "forbidden":
        raise AnnotationFilterError("safety.source_map_mutation must be 'forbidden'")
    if safety.get("mask_application_requires_preview") is not True:
        raise AnnotationFilterError(
            "safety.mask_application_requires_preview must be true"
        )
    if safety.get("filtered_map_requires_new_sha256") is not True:
        raise AnnotationFilterError(
            "safety.filtered_map_requires_new_sha256 must be true"
        )
    _nonempty_string(
        safety["localization_constraint_notice_zh"],
        "safety.localization_constraint_notice_zh",
        2000,
    )


def _parse_annotation_v1(value: Any, index: int, identifiers: set) -> MapAnnotation:
    label = "annotations[%d]" % index
    annotation = _mapping(value, label)
    _exact_keys(
        annotation,
        ("id", "category", "center_m", "radius_m"),
        label,
        optional=("note",),
    )
    identifier = _nonempty_string(annotation["id"], label + ".id", 128)
    if identifier in identifiers:
        raise AnnotationFilterError("annotation id is duplicated: %s" % identifier)
    identifiers.add(identifier)
    category = _nonempty_string(annotation["category"], label + ".category", 64)
    if category not in _CATEGORY_POLICY:
        raise AnnotationFilterError(
            "%s.category is unsupported: %s" % (label, category)
        )
    center = _mapping(annotation["center_m"], label + ".center_m")
    _exact_keys(center, ("x", "y", "z"), label + ".center_m")
    x = _finite_number(center["x"], label + ".center_m.x")
    y = _finite_number(center["y"], label + ".center_m.y")
    if max(abs(x), abs(y)) > MAX_ABS_COORDINATE_M:
        raise AnnotationFilterError(
            "%s center exceeds the %.0f m coordinate limit"
            % (label, MAX_ABS_COORDINATE_M)
        )
    z_value = center["z"]
    z = None if z_value is None else _finite_number(z_value, label + ".center_m.z")
    if z is not None and abs(z) > MAX_ABS_COORDINATE_M:
        raise AnnotationFilterError(
            "%s center exceeds the %.0f m coordinate limit"
            % (label, MAX_ABS_COORDINATE_M)
        )
    radius = _finite_number(annotation["radius_m"], label + ".radius_m")
    if radius < 0.1 or radius > 100.0:
        raise AnnotationFilterError("%s.radius_m must be within [0.1, 100]" % label)
    note_value = annotation.get("note")
    if note_value is None:
        note = None
    elif not isinstance(note_value, str) or len(note_value) > 1000:
        raise AnnotationFilterError(
            "%s.note must be a string no longer than 1000 characters" % label
        )
    else:
        note = note_value
    return MapAnnotation(
        id=identifier,
        category=category,
        x=x,
        y=y,
        z=z,
        radius_m=radius,
        note=note,
    )


def _z_range(value: Any, label: str) -> Tuple[float, float]:
    z_range = _mapping(value, label)
    _exact_keys(z_range, ("min", "max"), label)
    minimum = _finite_number(z_range["min"], label + ".min")
    maximum = _finite_number(z_range["max"], label + ".max")
    if maximum <= minimum or max(abs(minimum), abs(maximum)) > MAX_ABS_COORDINATE_M:
        raise AnnotationFilterError(
            "%s must be ordered and within the coordinate limit" % label
        )
    return minimum, maximum


def _xy_center(value: Any, label: str) -> Tuple[float, float]:
    center = _mapping(value, label)
    _exact_keys(center, ("x", "y"), label)
    x = _finite_number(center["x"], label + ".x")
    y = _finite_number(center["y"], label + ".y")
    if max(abs(x), abs(y)) > MAX_ABS_COORDINATE_M:
        raise AnnotationFilterError(
            "%s exceeds the %.0f m coordinate limit" % (label, MAX_ABS_COORDINATE_M)
        )
    return x, y


def _parse_annotation_v2(value: Any, index: int, identifiers: set) -> MapAnnotation:
    label = "annotations[%d]" % index
    annotation = _mapping(value, label)
    _exact_keys(
        annotation,
        ("id", "category", "roi"),
        label,
        optional=("note",),
    )
    identifier = _nonempty_string(annotation["id"], label + ".id", 128)
    if identifier in identifiers:
        raise AnnotationFilterError("annotation id is duplicated: %s" % identifier)
    identifiers.add(identifier)
    category = _nonempty_string(annotation["category"], label + ".category", 64)
    if category not in _CATEGORY_POLICY_V2:
        raise AnnotationFilterError(
            "%s.category is unsupported: %s" % (label, category)
        )
    note_value = annotation.get("note")
    if note_value is None:
        note = None
    elif not isinstance(note_value, str) or len(note_value) > 1000:
        raise AnnotationFilterError(
            "%s.note must be a string no longer than 1000 characters" % label
        )
    else:
        note = note_value

    roi_label = label + ".roi"
    roi = _mapping(annotation["roi"], roi_label)
    shape = _nonempty_string(roi.get("shape"), roi_label + ".shape", 64)
    if shape not in ("cylinder", "oriented_box", "polygon_prism"):
        raise AnnotationFilterError("%s.shape is unsupported: %s" % (roi_label, shape))
    minimum_z, maximum_z = _z_range(roi.get("z_range_m"), roi_label + ".z_range_m")
    if shape == "cylinder":
        _exact_keys(
            roi,
            ("shape", "center_xy_m", "radius_m", "z_range_m"),
            roi_label,
        )
        x, y = _xy_center(roi["center_xy_m"], roi_label + ".center_xy_m")
        radius = _finite_number(roi["radius_m"], roi_label + ".radius_m")
        if radius < 0.1 or radius > 100.0:
            raise AnnotationFilterError(
                "%s.radius_m must be within [0.1, 100]" % roi_label
            )
        return MapAnnotation(
            id=identifier,
            category=category,
            x=x,
            y=y,
            z=(minimum_z + maximum_z) * 0.5,
            radius_m=radius,
            note=note,
            geometry_type=shape,
            z_min_m=minimum_z,
            z_max_m=maximum_z,
        )
    if shape == "oriented_box":
        _exact_keys(
            roi,
            ("shape", "center_xy_m", "size_xy_m", "yaw_rad", "z_range_m"),
            roi_label,
        )
        x, y = _xy_center(roi["center_xy_m"], roi_label + ".center_xy_m")
        size = _mapping(roi["size_xy_m"], roi_label + ".size_xy_m")
        _exact_keys(size, ("x", "y"), roi_label + ".size_xy_m")
        size_x = _finite_number(size["x"], roi_label + ".size_xy_m.x")
        size_y = _finite_number(size["y"], roi_label + ".size_xy_m.y")
        if size_x < 0.1 or size_y < 0.1 or size_x > 200.0 or size_y > 200.0:
            raise AnnotationFilterError(
                "%s.size_xy_m values must be within [0.1, 200]" % roi_label
            )
        yaw = _finite_number(roi["yaw_rad"], roi_label + ".yaw_rad")
        if abs(yaw) > math.pi:
            raise AnnotationFilterError(
                "%s.yaw_rad must be within [-pi, pi]" % roi_label
            )
        return MapAnnotation(
            id=identifier,
            category=category,
            x=x,
            y=y,
            z=(minimum_z + maximum_z) * 0.5,
            radius_m=0.0,
            note=note,
            geometry_type=shape,
            z_min_m=minimum_z,
            z_max_m=maximum_z,
            size_x_m=size_x,
            size_y_m=size_y,
            yaw_rad=yaw,
        )

    _exact_keys(
        roi,
        ("shape", "vertices_xy_m", "z_range_m"),
        roi_label,
    )
    raw_vertices = roi["vertices_xy_m"]
    if not isinstance(raw_vertices, list) or not (3 <= len(raw_vertices) <= 128):
        raise AnnotationFilterError(
            "%s.vertices_xy_m must contain between 3 and 128 vertices" % roi_label
        )
    vertices: List[Tuple[float, float]] = []
    for vertex_index, value in enumerate(raw_vertices):
        vertex_label = "%s.vertices_xy_m[%d]" % (roi_label, vertex_index)
        if not isinstance(value, list) or len(value) != 2:
            raise AnnotationFilterError("%s must contain [x, y]" % vertex_label)
        vertex = (
            _finite_number(value[0], vertex_label + "[0]"),
            _finite_number(value[1], vertex_label + "[1]"),
        )
        if max(abs(vertex[0]), abs(vertex[1])) > MAX_ABS_COORDINATE_M:
            raise AnnotationFilterError(
                "%s exceeds the coordinate limit" % vertex_label
            )
        vertices.append(vertex)
    area_twice = abs(
        sum(
            vertices[index][0] * vertices[(index + 1) % len(vertices)][1]
            - vertices[(index + 1) % len(vertices)][0] * vertices[index][1]
            for index in range(len(vertices))
        )
    )
    if area_twice < 0.02:
        raise AnnotationFilterError("%s polygon area is too small" % roi_label)
    return MapAnnotation(
        id=identifier,
        category=category,
        x=sum(vertex[0] for vertex in vertices) / len(vertices),
        y=sum(vertex[1] for vertex in vertices) / len(vertices),
        z=(minimum_z + maximum_z) * 0.5,
        radius_m=0.0,
        note=note,
        geometry_type=shape,
        z_min_m=minimum_z,
        z_max_m=maximum_z,
        vertices_xy_m=tuple(vertices),
    )


def load_annotation_export(path: os.PathLike) -> AnnotationExport:
    annotation_path = Path(path).expanduser().resolve()
    if not annotation_path.is_file():
        raise AnnotationFilterError(
            "annotation JSON is not a regular file: %s" % annotation_path
        )
    try:
        size = annotation_path.stat().st_size
    except OSError as exc:
        raise AnnotationFilterError("cannot inspect annotation JSON: %s" % exc) from exc
    if size <= 0 or size > MAX_ANNOTATION_JSON_BYTES:
        raise AnnotationFilterError(
            "annotation JSON size must be within [1, %d] bytes"
            % MAX_ANNOTATION_JSON_BYTES
        )
    digest_before = _sha256_file(annotation_path, "annotation JSON")
    try:
        text = annotation_path.read_text(encoding="utf-8")
    except (OSError, UnicodeError) as exc:
        raise AnnotationFilterError(
            "cannot read annotation JSON as UTF-8: %s" % exc
        ) from exc
    try:
        document = json.loads(
            text,
            parse_constant=_reject_json_constant,
            object_pairs_hook=_unique_json_object,
        )
    except AnnotationFilterError:
        raise
    except (TypeError, ValueError, json.JSONDecodeError) as exc:
        raise AnnotationFilterError(
            "annotation JSON is not strict JSON: %s" % exc
        ) from exc
    digest_after = _sha256_file(annotation_path, "annotation JSON")
    if digest_before != digest_after:
        raise AnnotationFilterError("annotation JSON changed while it was being read")

    root = _mapping(document, "annotation export")
    schema = root.get("schema")
    if schema not in (
        ANNOTATION_EXPORT_SCHEMA_V1,
        ANNOTATION_EXPORT_SCHEMA_V2,
    ):
        raise AnnotationFilterError("unsupported annotation schema %r" % schema)
    required_root_keys = [
        "schema",
        "exported_at_utc",
        "map",
        "coordinate_system",
        "category_semantics",
        "safety",
        "annotations",
    ]
    if schema == ANNOTATION_EXPORT_SCHEMA_V2:
        required_root_keys.append("revision")
    _exact_keys(root, tuple(required_root_keys), "annotation export")
    exported_at = _utc_timestamp(root["exported_at_utc"])
    revision = (
        _nonempty_string(root["revision"], "revision", 128)
        if schema == ANNOTATION_EXPORT_SCHEMA_V2
        else "legacy-v1-" + digest_after[:12]
    )

    map_binding = _mapping(root["map"], "map")
    _exact_keys(map_binding, ("sha256", "file_name"), "map")
    map_sha256 = map_binding.get("sha256")
    if not isinstance(map_sha256, str) or not _SHA256.fullmatch(map_sha256):
        raise AnnotationFilterError(
            "map.sha256 must be a complete lowercase SHA-256 digest"
        )
    map_filename = _nonempty_string(map_binding["file_name"], "map.file_name", 1024)
    if (
        Path(map_filename).name != map_filename
        or "/" in map_filename
        or "\\" in map_filename
        or "\x00" in map_filename
    ):
        raise AnnotationFilterError("map.file_name must be a basename")

    _validate_coordinate_system(root["coordinate_system"], schema)
    _validate_category_semantics(root["category_semantics"], schema)
    _validate_safety(root["safety"])

    values = root["annotations"]
    if not isinstance(values, list):
        raise AnnotationFilterError("annotations must be an array")
    if len(values) > MAX_ANNOTATIONS:
        raise AnnotationFilterError(
            "annotations exceeds the %d item limit" % MAX_ANNOTATIONS
        )
    identifiers: set = set()
    parser = (
        _parse_annotation_v1
        if schema == ANNOTATION_EXPORT_SCHEMA_V1
        else _parse_annotation_v2
    )
    annotations = tuple(
        parser(value, index, identifiers) for index, value in enumerate(values)
    )
    mask_count = sum(1 for annotation in annotations if annotation.is_exclude_mask)
    if mask_count > MAX_EXCLUDE_MASKS:
        raise AnnotationFilterError(
            "exclude masks exceeds the %d item limit" % MAX_EXCLUDE_MASKS
        )
    return AnnotationExport(
        path=annotation_path,
        sha256=digest_after,
        map_sha256=map_sha256,
        map_filename=map_filename,
        exported_at_utc=exported_at,
        schema=schema,
        revision=revision,
        annotations=annotations,
    )


def _xyz_field_offsets(cloud: PCDCloud) -> Tuple[int, int, int]:
    fields = {field.name: field for field in cloud.fields}
    for name in ("x", "y", "z"):
        field = fields.get(name)
        if field is None or field.count != 1:
            raise AnnotationFilterError(
                "source PCD must contain scalar x, y, and z fields"
            )
    return (
        cloud.field_offset("x"),
        cloud.field_offset("y"),
        cloud.field_offset("z"),
    )


def _roi_buckets(
    rois: Sequence[MapAnnotation],
) -> Tuple[float, Mapping[Tuple[int, int], Tuple[int, ...]]]:
    if not rois:
        return 1.0, {}
    maximum_extent = max(
        max(
            roi.xy_bounds()[2] - roi.xy_bounds()[0],
            roi.xy_bounds()[3] - roi.xy_bounds()[1],
        )
        for roi in rois
    )
    bucket_size = max(1.0, min(10.0, maximum_extent * 0.5))
    mutable: Dict[Tuple[int, int], List[int]] = {}
    reference_count = 0
    for roi_index, roi in enumerate(rois):
        minimum_x, minimum_y, maximum_x, maximum_y = roi.xy_bounds()
        min_ix = math.floor(minimum_x / bucket_size)
        max_ix = math.floor(maximum_x / bucket_size)
        min_iy = math.floor(minimum_y / bucket_size)
        max_iy = math.floor(maximum_y / bucket_size)
        for ix in range(min_ix, max_ix + 1):
            for iy in range(min_iy, max_iy + 1):
                reference_count += 1
                if reference_count > MAX_MASK_BUCKET_REFERENCES:
                    raise AnnotationFilterError(
                        "ROI spatial index exceeds the safe limit"
                    )
                mutable.setdefault((ix, iy), []).append(roi_index)
    return bucket_size, {key: tuple(indices) for key, indices in mutable.items()}


def _filter_cloud(
    cloud: PCDCloud,
    annotations: Sequence[MapAnnotation],
) -> Tuple[
    PCDCloud,
    PCDCloud,
    List[MapAnnotation],
    List[int],
    List[MapAnnotation],
    List[int],
    int,
    int,
]:
    if cloud.point_count <= 0:
        raise AnnotationFilterError("source PCD must contain at least one point")
    x_offset, y_offset, z_offset = _xyz_field_offsets(cloud)
    masks = [annotation for annotation in annotations if annotation.is_exclude_mask]
    stable_rois = [annotation for annotation in annotations if annotation.is_stable_roi]
    bucket_size, buckets = _roi_buckets(masks)
    stable_bucket_size, stable_buckets = _roi_buckets(stable_rois)
    matched_counts = [0 for _mask in masks]
    stable_matched_counts = [0 for _roi in stable_rois]
    retained_rows = []
    stable_rows = []
    removed_count = 0
    stable_excluded_overlap_count = 0

    for row in cloud.rows:
        x = float(row[x_offset])
        y = float(row[y_offset])
        z = float(row[z_offset])
        key = (math.floor(x / bucket_size), math.floor(y / bucket_size))
        matched = False
        for mask_index in buckets.get(key, ()):
            mask = masks[mask_index]
            if mask.contains(x, y, z):
                matched_counts[mask_index] += 1
                matched = True
        stable_key = (
            math.floor(x / stable_bucket_size),
            math.floor(y / stable_bucket_size),
        )
        matched_stable_indices = [
            stable_index
            for stable_index in stable_buckets.get(stable_key, ())
            if stable_rois[stable_index].contains(x, y, z)
        ]
        if matched:
            removed_count += 1
            if matched_stable_indices:
                stable_excluded_overlap_count += 1
        else:
            retained_rows.append(row)
            if matched_stable_indices:
                stable_rows.append(row)
                for stable_index in matched_stable_indices:
                    stable_matched_counts[stable_index] += 1

    if not retained_rows:
        raise AnnotationFilterError(
            "exclude masks would remove every source point; " "no output published"
        )
    cleaned = PCDCloud(
        fields=cloud.fields,
        rows=tuple(retained_rows),
        width=len(retained_rows),
        height=1,
        viewpoint=cloud.viewpoint,
        data_encoding=cloud.data_encoding,
    )
    stable = PCDCloud(
        fields=cloud.fields,
        rows=tuple(stable_rows),
        width=len(stable_rows),
        height=1,
        viewpoint=cloud.viewpoint,
        data_encoding=cloud.data_encoding,
    )
    return (
        cleaned,
        stable,
        masks,
        matched_counts,
        stable_rois,
        stable_matched_counts,
        removed_count,
        stable_excluded_overlap_count,
    )


def _write_bytes(path: Path, payload: bytes) -> None:
    try:
        with path.open("xb") as handle:
            handle.write(payload)
            handle.flush()
            os.fsync(handle.fileno())
    except OSError as exc:
        raise AnnotationFilterError(
            "cannot write staged artifact %s: %s" % (path.name, exc)
        ) from exc


def _checksum_payload(digest: str, filename: str) -> bytes:
    return ("%s  %s\n" % (digest, filename)).encode("ascii")


def _write_output_pcd(cloud: PCDCloud, path: Path, encoding: str) -> None:
    if encoding == "binary":
        write_pcd_binary(cloud, path)
    elif encoding == "ascii":
        write_pcd_ascii(cloud, path)
    else:
        raise AnnotationFilterError("unsupported source PCD encoding %r" % encoding)


def filter_pcd_with_annotations(
    source_pcd: os.PathLike,
    annotation_json: os.PathLike,
    output_directory: os.PathLike,
) -> Mapping[str, Any]:
    """Apply explicit exclusion masks and publish a new immutable result.

    The output directory must not exist.  The source PCD is hash-checked before
    and after processing and is never opened for writing.
    """

    source = Path(source_pcd).expanduser().resolve()
    if not source.is_file():
        raise AnnotationFilterError("source PCD is not a regular file: %s" % source)
    annotation_path = Path(annotation_json).expanduser().resolve()
    if not annotation_path.is_file():
        raise AnnotationFilterError(
            "annotation JSON is not a regular file: %s" % annotation_path
        )
    if source == annotation_path:
        raise AnnotationFilterError(
            "source PCD and annotation JSON must be different files"
        )

    requested_output = Path(output_directory).expanduser()
    if requested_output.exists() or requested_output.is_symlink():
        raise AnnotationFilterError(
            "output directory must not already exist; overwrite is forbidden"
        )
    output = requested_output.resolve()
    if output == Path(output.anchor) or output == Path.home().resolve():
        raise AnnotationFilterError("refusing filesystem root or home as output")
    for input_path, label in (
        (source, "source PCD"),
        (annotation_path, "annotation JSON"),
    ):
        try:
            input_path.relative_to(output)
        except ValueError:
            pass
        else:
            raise AnnotationFilterError(
                "%s must not be inside the output directory" % label
            )

    source_sha256_before = _sha256_file(source, "source PCD")
    export = load_annotation_export(annotation_path)
    if export.map_sha256 != source_sha256_before:
        raise AnnotationFilterError(
            "annotation map SHA-256 does not match the complete source PCD"
        )
    try:
        cloud = read_pcd(source)
    except PCDDataError:
        raise
    source_sha256_after_read = _sha256_file(source, "source PCD")
    if source_sha256_after_read != source_sha256_before:
        raise AnnotationFilterError("source PCD changed while it was being read")

    (
        cleaned,
        stable_layer,
        masks,
        matched_counts,
        stable_rois,
        stable_matched_counts,
        removed_count,
        stable_excluded_overlap_count,
    ) = _filter_cloud(cloud, export.annotations)
    review_findings = [
        annotation.to_report()
        for annotation in export.annotations
        if not annotation.is_exclude_mask and annotation.category != "stable_include"
    ]

    output.parent.mkdir(parents=True, exist_ok=True)
    staging = Path(
        tempfile.mkdtemp(
            prefix=".%s.annotation-filter-staging." % output.name,
            dir=str(output.parent),
        )
    )
    published = False
    try:
        staged_pcd = staging / CLEANED_PCD_FILENAME
        _write_output_pcd(cleaned, staged_pcd, encoding=cloud.data_encoding)
        verified = read_pcd(staged_pcd)
        if (
            verified.point_count != cleaned.point_count
            or verified.fields != cleaned.fields
            or verified.data_encoding != cloud.data_encoding
        ):
            raise AnnotationFilterError("staged cleaned PCD failed verification")
        output_sha256 = _sha256_file(staged_pcd, "cleaned PCD")
        _write_bytes(
            staging / (CLEANED_PCD_FILENAME + ".sha256"),
            _checksum_payload(output_sha256, CLEANED_PCD_FILENAME),
        )
        stable_path = staging / STABLE_LAYER_PCD_FILENAME
        _write_output_pcd(stable_layer, stable_path, encoding=cloud.data_encoding)
        verified_stable = read_pcd(stable_path)
        if (
            verified_stable.point_count != stable_layer.point_count
            or verified_stable.fields != stable_layer.fields
            or verified_stable.data_encoding != cloud.data_encoding
        ):
            raise AnnotationFilterError("staged stable-layer PCD failed verification")
        stable_sha256 = _sha256_file(stable_path, "stable-layer PCD")
        _write_bytes(
            staging / (STABLE_LAYER_PCD_FILENAME + ".sha256"),
            _checksum_payload(stable_sha256, STABLE_LAYER_PCD_FILENAME),
        )

        report: Dict[str, Any] = {
            "schema": FILTER_REPORT_SCHEMA,
            "created_utc": datetime.now(timezone.utc)
            .isoformat()
            .replace("+00:00", "Z"),
            "source": {
                "filename": source.name,
                "sha256": source_sha256_before,
                "byte_size": source.stat().st_size,
                "pcd_encoding": cloud.data_encoding,
                "point_count": cloud.point_count,
            },
            "annotation_export": {
                "filename": export.path.name,
                "sha256": export.sha256,
                "schema": export.schema,
                "revision": export.revision,
                "exported_at_utc": export.exported_at_utc,
                "declared_map_filename": export.map_filename,
                "declared_map_sha256": export.map_sha256,
                "frame_id": "map",
                "linear_unit": "metre",
                "annotation_count": len(export.annotations),
            },
            "policy": {
                "mask_categories": sorted(EXCLUDE_CATEGORIES),
                "supported_roi_geometry": [
                    "xy_circle_all_z",
                    "cylinder",
                    "oriented_box",
                    "polygon_prism",
                ],
                "stable_include": ("extracted_to_stable_layer_without_inverse_filter"),
                "unannotated_points": "retained",
                "source_map_overwrite": "forbidden",
                "exclude_precedes_stable": True,
                "per_mask_counts_may_overlap": True,
            },
            "filtering": {
                "exclude_mask_count": len(masks),
                "removed_point_count": removed_count,
                "retained_point_count": cleaned.point_count,
                "stable_roi_count": len(stable_rois),
                "stable_layer_point_count": stable_layer.point_count,
                "stable_excluded_overlap_point_count": (stable_excluded_overlap_count),
                "filter_masks": [
                    {
                        **mask.to_report(),
                        "matched_point_count": matched_counts[index],
                    }
                    for index, mask in enumerate(masks)
                ],
                "stable_rois": [
                    {
                        **roi.to_report(),
                        "matched_retained_point_count": stable_matched_counts[index],
                    }
                    for index, roi in enumerate(stable_rois)
                ],
                # Kept as an output alias for older report consumers.
                "stable_include_hints": [roi.to_report() for roi in stable_rois],
                "review_findings": review_findings,
            },
            "cleaned_static_map": {
                "filename": CLEANED_PCD_FILENAME,
                "sha256": output_sha256,
                "pcd_encoding": cloud.data_encoding,
                "point_count": cleaned.point_count,
            },
            "stable_layer": {
                "filename": STABLE_LAYER_PCD_FILENAME,
                "sha256": stable_sha256,
                "pcd_encoding": cloud.data_encoding,
                "point_count": stable_layer.point_count,
            },
        }
        # Backward-compatible alias.  It refers to the cleaned static map.
        report["output"] = dict(report["cleaned_static_map"])
        report_payload = (
            json.dumps(
                report,
                ensure_ascii=False,
                indent=2,
                sort_keys=True,
                allow_nan=False,
            )
            + "\n"
        ).encode("utf-8")
        report_path = staging / FILTER_REPORT_FILENAME
        _write_bytes(report_path, report_payload)
        report_sha256 = hashlib.sha256(report_payload).hexdigest()
        _write_bytes(
            staging / (FILTER_REPORT_FILENAME + ".sha256"),
            _checksum_payload(report_sha256, FILTER_REPORT_FILENAME),
        )

        if _sha256_file(source, "source PCD") != source_sha256_before:
            raise AnnotationFilterError("source PCD changed during filtering")
        if _sha256_file(annotation_path, "annotation JSON") != export.sha256:
            raise AnnotationFilterError("annotation JSON changed during filtering")
        if output.exists() or output.is_symlink():
            raise AnnotationFilterError(
                "output directory appeared during filtering; overwrite refused"
            )
        os.rename(str(staging), str(output))
        published = True
        result = dict(report)
        result["report_sha256"] = report_sha256
        result["output_directory"] = str(output)
        return result
    except AnnotationFilterError:
        raise
    except OSError as exc:
        raise AnnotationFilterError("cannot publish filtered map: %s" % exc) from exc
    finally:
        if not published and staging.exists():
            shutil.rmtree(str(staging))

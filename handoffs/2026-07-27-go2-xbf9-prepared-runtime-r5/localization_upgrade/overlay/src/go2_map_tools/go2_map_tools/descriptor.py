"""Clean-room polar place descriptor and deterministic JSON index.

This implementation is based only on the general idea of discretizing a local
point cloud into radial and angular bins.  It does not depend on or reproduce
third-party Scan Context source code.  Each cell stores the maximum normalized
height, while circular sector shifts provide a yaw-tolerant similarity search.
"""

from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from functools import lru_cache
from pathlib import Path
import shutil
import tempfile
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple
import uuid

from .pcd import PCDDataError, read_pcd
from .tiles import (
    MAP_MANIFEST_SCHEMA,
    MapManifest,
    MapManifestError,
    bind_descriptor_index,
    build_tiled_map,
    load_manifest,
    require_replaceable_map_directory,
    sha256_file,
)


DESCRIPTOR_INDEX_SCHEMA = "go2.polar_descriptor_index/v1"


class DescriptorError(ValueError):
    """Raised when descriptor parameters or index data are invalid."""


def _finite(value: Any, name: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise DescriptorError("%s must be numeric" % name) from exc
    if not math.isfinite(result):
        raise DescriptorError("%s must be finite" % name)
    return result


@dataclass(frozen=True)
class PolarDescriptorConfig:
    rings: int = 20
    sectors: int = 60
    max_radius_m: float = 80.0
    min_z_m: float = -3.0
    max_z_m: float = 5.0
    value: str = "max_height_normalized"

    def __post_init__(self) -> None:
        rings = int(self.rings)
        sectors = int(self.sectors)
        if rings <= 0 or sectors < 4:
            raise DescriptorError(
                "rings must be positive and sectors must be at least 4"
            )
        if rings > 1000 or sectors > 4096:
            raise DescriptorError("descriptor dimensions are unreasonably large")
        maximum_radius = _finite(self.max_radius_m, "max_radius_m")
        minimum_z = _finite(self.min_z_m, "min_z_m")
        maximum_z = _finite(self.max_z_m, "max_z_m")
        if maximum_radius <= 0.0:
            raise DescriptorError("max_radius_m must be positive")
        if maximum_z <= minimum_z:
            raise DescriptorError("max_z_m must exceed min_z_m")
        if self.value != "max_height_normalized":
            raise DescriptorError("unsupported descriptor value mode %r" % self.value)
        object.__setattr__(self, "rings", rings)
        object.__setattr__(self, "sectors", sectors)
        object.__setattr__(self, "max_radius_m", maximum_radius)
        object.__setattr__(self, "min_z_m", minimum_z)
        object.__setattr__(self, "max_z_m", maximum_z)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "rings": self.rings,
            "sectors": self.sectors,
            "max_radius_m": self.max_radius_m,
            "min_z_m": self.min_z_m,
            "max_z_m": self.max_z_m,
            "value": self.value,
        }

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "PolarDescriptorConfig":
        expected = {
            "rings",
            "sectors",
            "max_radius_m",
            "min_z_m",
            "max_z_m",
            "value",
        }
        if not isinstance(data, Mapping) or set(data.keys()) != expected:
            raise DescriptorError("descriptor parameters do not match v1 schema")
        return cls(
            rings=data["rings"],
            sectors=data["sectors"],
            max_radius_m=data["max_radius_m"],
            min_z_m=data["min_z_m"],
            max_z_m=data["max_z_m"],
            value=data["value"],
        )


@dataclass(frozen=True)
class PolarDescriptor:
    config: PolarDescriptorConfig
    values: Tuple[float, ...]
    ring_key: Tuple[float, ...]
    sector_key: Tuple[float, ...]

    def __post_init__(self) -> None:
        values = tuple(_finite(value, "descriptor value") for value in self.values)
        ring_key = tuple(_finite(value, "ring key value") for value in self.ring_key)
        sector_key = tuple(
            _finite(value, "sector key value") for value in self.sector_key
        )
        expected = self.config.rings * self.config.sectors
        if len(values) != expected:
            raise DescriptorError(
                "descriptor has %d values, expected %d" % (len(values), expected)
            )
        if len(ring_key) != self.config.rings:
            raise DescriptorError("ring key length does not match rings")
        if len(sector_key) != self.config.sectors:
            raise DescriptorError("sector key length does not match sectors")
        if any(value < 0.0 or value > 1.0 for value in values):
            raise DescriptorError("descriptor values must be in [0, 1]")
        object.__setattr__(self, "values", values)
        object.__setattr__(self, "ring_key", ring_key)
        object.__setattr__(self, "sector_key", sector_key)


def compute_polar_descriptor(
    points: Iterable[Sequence[float]],
    config: Optional[PolarDescriptorConfig] = None,
    center_xyz: Sequence[float] = (0.0, 0.0, 0.0),
) -> PolarDescriptor:
    settings = config or PolarDescriptorConfig()
    if len(center_xyz) != 3:
        raise DescriptorError("center_xyz must contain x, y, z")
    center = tuple(_finite(value, "center value") for value in center_xyz)
    values = [0.0] * (settings.rings * settings.sectors)
    z_span = settings.max_z_m - settings.min_z_m
    for point_index, point in enumerate(points):
        if len(point) < 3:
            raise DescriptorError("point %d has fewer than three values" % point_index)
        x = _finite(point[0], "point x") - center[0]
        y = _finite(point[1], "point y") - center[1]
        z = _finite(point[2], "point z") - center[2]
        radius = math.hypot(x, y)
        if radius > settings.max_radius_m:
            continue
        ring = min(
            settings.rings - 1,
            int((radius / settings.max_radius_m) * settings.rings),
        )
        angle = math.atan2(y, x)
        # A tiny epsilon makes points exactly on a sector boundary stable under
        # rigid rotations despite atan2 floating-point roundoff.  Modulo maps
        # +pi to the same half-open bin as -pi.
        sector_coordinate = ((angle + math.pi) / (2.0 * math.pi)) * settings.sectors
        sector = int(math.floor(sector_coordinate + 1.0e-12)) % settings.sectors
        normalized_height = (
            min(settings.max_z_m, max(settings.min_z_m, z)) - settings.min_z_m
        ) / z_span
        offset = ring * settings.sectors + sector
        if normalized_height > values[offset]:
            values[offset] = normalized_height
    ring_key = tuple(
        sum(values[ring * settings.sectors : (ring + 1) * settings.sectors])
        / settings.sectors
        for ring in range(settings.rings)
    )
    sector_key = tuple(
        sum(values[ring * settings.sectors + sector] for ring in range(settings.rings))
        / settings.rings
        for sector in range(settings.sectors)
    )
    return PolarDescriptor(settings, tuple(values), ring_key, sector_key)


def _similarity_at_shift(
    query: PolarDescriptor, candidate: PolarDescriptor, shift: int
) -> float:
    sectors = query.config.sectors
    dot = 0.0
    query_norm = 0.0
    candidate_norm = 0.0
    for ring in range(query.config.rings):
        row_offset = ring * sectors
        for sector in range(sectors):
            query_value = query.values[row_offset + sector]
            candidate_value = candidate.values[
                row_offset + ((sector + shift) % sectors)
            ]
            dot += query_value * candidate_value
            query_norm += query_value * query_value
            candidate_norm += candidate_value * candidate_value
    if query_norm == 0.0 and candidate_norm == 0.0:
        # Featureless scans are not evidence that two places are the same.
        return 0.0
    if query_norm == 0.0 or candidate_norm == 0.0:
        return 0.0
    return dot / math.sqrt(query_norm * candidate_norm)


def descriptor_similarity(
    query: PolarDescriptor, candidate: PolarDescriptor
) -> Tuple[float, int, float]:
    """Return best cosine score, sector shift, and yaw offset in radians.

    A positive shift means candidate sectors are sampled at increasing indices
    to align with the query.
    """

    if query.config != candidate.config:
        raise DescriptorError("descriptor configurations do not match")
    scores = [
        _similarity_at_shift(query, candidate, shift)
        for shift in range(query.config.sectors)
    ]
    shift = max(range(len(scores)), key=lambda index: (scores[index], -index))
    yaw = shift * (2.0 * math.pi / query.config.sectors)
    if yaw >= math.pi:
        yaw -= 2.0 * math.pi
    return scores[shift], shift, yaw


@dataclass(frozen=True)
class DescriptorEntry:
    id: str
    tile_id: str
    center: Tuple[float, float, float]
    source_sha256: str
    descriptor: PolarDescriptor

    def __post_init__(self) -> None:
        entry_id = str(self.id).strip()
        tile_id = str(self.tile_id).strip()
        if not entry_id or not tile_id:
            raise DescriptorError("descriptor id and tile_id must not be empty")
        if len(self.center) != 3:
            raise DescriptorError("descriptor center must have three values")
        center = tuple(_finite(value, "descriptor center") for value in self.center)
        digest = str(self.source_sha256).lower()
        if len(digest) != 64 or any(
            character not in "0123456789abcdef" for character in digest
        ):
            raise DescriptorError("descriptor source_sha256 is invalid")
        object.__setattr__(self, "id", entry_id)
        object.__setattr__(self, "tile_id", tile_id)
        object.__setattr__(self, "center", center)
        object.__setattr__(self, "source_sha256", digest)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "id": self.id,
            "tile_id": self.tile_id,
            "center": list(self.center),
            "source_sha256": self.source_sha256,
            "values": list(self.descriptor.values),
            "ring_key": list(self.descriptor.ring_key),
            "sector_key": list(self.descriptor.sector_key),
        }


@dataclass(frozen=True)
class DescriptorMatch:
    id: str
    tile_id: str
    center: Tuple[float, float, float]
    score: float
    sector_shift: int
    yaw_offset_rad: float


@dataclass(frozen=True)
class DescriptorSourceLayer:
    path: str
    sha256: str
    point_count: int
    role: str = "global_retrieval"

    def __post_init__(self) -> None:
        relative = Path(str(self.path))
        if (
            relative.is_absolute()
            or ".." in relative.parts
            or relative.as_posix() in ("", ".")
        ):
            raise DescriptorError(
                "descriptor source-layer path must be a safe relative path"
            )
        digest = str(self.sha256).lower()
        if len(digest) != 64 or any(
            character not in "0123456789abcdef" for character in digest
        ):
            raise DescriptorError("descriptor source-layer sha256 is invalid")
        count = int(self.point_count)
        if count <= 0:
            raise DescriptorError(
                "descriptor source-layer point_count must be positive"
            )
        if self.role != "global_retrieval":
            raise DescriptorError("descriptor source-layer role is unsupported")
        object.__setattr__(self, "path", relative.as_posix())
        object.__setattr__(self, "sha256", digest)
        object.__setattr__(self, "point_count", count)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "role": self.role,
            "path": self.path,
            "sha256": self.sha256,
            "point_count": self.point_count,
        }


class PolarDescriptorIndex:
    def __init__(
        self,
        map_id: str,
        config: Optional[PolarDescriptorConfig] = None,
        entries: Sequence[DescriptorEntry] = (),
        created_utc: Optional[str] = None,
        source_layer: Optional[DescriptorSourceLayer] = None,
    ) -> None:
        self.map_id = str(map_id).strip()
        if not self.map_id:
            raise DescriptorError("map_id must not be empty")
        self.config = config or PolarDescriptorConfig()
        self.created_utc = created_utc or datetime.now(
            timezone.utc
        ).isoformat().replace("+00:00", "Z")
        self.entries = tuple(entries)
        self.source_layer = source_layer
        if len({entry.id for entry in self.entries}) != len(self.entries):
            raise DescriptorError("descriptor entry ids must be unique")
        if any(entry.descriptor.config != self.config for entry in self.entries):
            raise DescriptorError("entry descriptor configuration mismatch")

    def query(
        self, descriptor: PolarDescriptor, limit: int = 5
    ) -> Tuple[DescriptorMatch, ...]:
        limit = int(limit)
        if limit <= 0:
            raise DescriptorError("query limit must be positive")
        if descriptor.config != self.config:
            raise DescriptorError("query descriptor configuration mismatch")
        matches = []
        for entry in self.entries:
            score, shift, yaw = descriptor_similarity(descriptor, entry.descriptor)
            matches.append(
                DescriptorMatch(
                    id=entry.id,
                    tile_id=entry.tile_id,
                    center=entry.center,
                    score=score,
                    sector_shift=shift,
                    yaw_offset_rad=yaw,
                )
            )
        matches.sort(key=lambda match: (-match.score, match.id))
        return tuple(matches[:limit])

    def to_dict(self) -> Dict[str, Any]:
        result = {
            "schema": DESCRIPTOR_INDEX_SCHEMA,
            "map_id": self.map_id,
            "created_utc": self.created_utc,
            "parameters": self.config.to_dict(),
            "entries": [entry.to_dict() for entry in self.entries],
        }
        if self.source_layer is not None:
            result["source_layer"] = self.source_layer.to_dict()
        return result


def _atomic_json(path: Path, data: Mapping[str, Any]) -> bytes:
    payload = (
        json.dumps(
            data,
            ensure_ascii=False,
            allow_nan=False,
            indent=2,
            sort_keys=True,
        )
        + "\n"
    ).encode("utf-8")
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=".%s." % path.name, suffix=".tmp", dir=str(path.parent)
    )
    try:
        with os.fdopen(descriptor, "wb") as handle:
            handle.write(payload)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary_name, str(path))
    except Exception:
        try:
            os.unlink(temporary_name)
        except OSError:
            pass
        raise
    return payload


def save_descriptor_index(index: PolarDescriptorIndex, path: os.PathLike) -> None:
    target = Path(path)
    payload = _atomic_json(target, index.to_dict())
    digest = hashlib.sha256(payload).hexdigest()
    checksum_path = target.with_suffix(target.suffix + ".sha256")
    _atomic_json_checksum(checksum_path, digest, target.name)


def _atomic_json_checksum(path: Path, digest: str, filename: str) -> None:
    content = ("%s  %s\n" % (digest, filename)).encode("ascii")
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=".%s." % path.name, suffix=".tmp", dir=str(path.parent)
    )
    try:
        with os.fdopen(descriptor, "wb") as handle:
            handle.write(content)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary_name, str(path))
    except Exception:
        try:
            os.unlink(temporary_name)
        except OSError:
            pass
        raise


def _index_from_dict(data: Mapping[str, Any]) -> PolarDescriptorIndex:
    expected = {"schema", "map_id", "created_utc", "parameters", "entries"}
    if not isinstance(data, Mapping) or not (
        set(data.keys()) == expected or set(data.keys()) == expected | {"source_layer"}
    ):
        raise DescriptorError("descriptor index does not match v1 schema")
    if data["schema"] != DESCRIPTOR_INDEX_SCHEMA:
        raise DescriptorError("unsupported descriptor index schema %r" % data["schema"])
    config = PolarDescriptorConfig.from_dict(data["parameters"])
    raw_entries = data["entries"]
    if not isinstance(raw_entries, list):
        raise DescriptorError("descriptor entries must be an array")
    entries = []
    expected_entry = {
        "id",
        "tile_id",
        "center",
        "source_sha256",
        "values",
        "ring_key",
        "sector_key",
    }
    for index, raw in enumerate(raw_entries):
        if not isinstance(raw, Mapping) or set(raw.keys()) != expected_entry:
            raise DescriptorError("descriptor entry %d has invalid fields" % index)
        entries.append(
            DescriptorEntry(
                id=raw["id"],
                tile_id=raw["tile_id"],
                center=tuple(raw["center"]),
                source_sha256=raw["source_sha256"],
                descriptor=PolarDescriptor(
                    config=config,
                    values=tuple(raw["values"]),
                    ring_key=tuple(raw["ring_key"]),
                    sector_key=tuple(raw["sector_key"]),
                ),
            )
        )
    source_layer = None
    if "source_layer" in data:
        raw_source = data["source_layer"]
        if not isinstance(raw_source, Mapping) or set(raw_source.keys()) != {
            "role",
            "path",
            "sha256",
            "point_count",
        }:
            raise DescriptorError("descriptor source_layer has invalid fields")
        source_layer = DescriptorSourceLayer(
            role=raw_source["role"],
            path=raw_source["path"],
            sha256=raw_source["sha256"],
            point_count=raw_source["point_count"],
        )
    return PolarDescriptorIndex(
        map_id=data["map_id"],
        created_utc=data["created_utc"],
        config=config,
        entries=entries,
        source_layer=source_layer,
    )


def load_descriptor_index(
    path: os.PathLike, verify_checksum: bool = True
) -> PolarDescriptorIndex:
    source = Path(path)
    try:
        payload = source.read_bytes()
        data = json.loads(
            payload.decode("utf-8"),
            parse_constant=lambda value: (_reject_constant(value)),
        )
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise DescriptorError(
            "cannot read descriptor index %s: %s" % (source, exc)
        ) from exc
    if verify_checksum:
        checksum_path = source.with_suffix(source.suffix + ".sha256")
        try:
            tokens = checksum_path.read_text(encoding="ascii").split()
        except OSError as exc:
            raise DescriptorError("cannot read descriptor checksum: %s" % exc) from exc
        expected = hashlib.sha256(payload).hexdigest()
        if (
            len(tokens) != 2
            or tokens[1] != source.name
            or tokens[0].lower() != expected
        ):
            raise DescriptorError("descriptor index checksum mismatch")
    return _index_from_dict(data)


def _reject_constant(value: str) -> None:
    raise DescriptorError("non-finite JSON number %s is not allowed" % value)


def build_descriptor_index(
    manifest_path: os.PathLike,
    output_path: Optional[os.PathLike] = None,
    config: Optional[PolarDescriptorConfig] = None,
    source_layer_pcd: Optional[os.PathLike] = None,
) -> PolarDescriptorIndex:
    """Build one descriptor per tile.

    By default the descriptor source is the cleaned tracking tiles.  When
    ``source_layer_pcd`` is supplied it must live inside the map bundle and its
    exact identity is embedded in the index.  Tracking tiles remain unchanged.
    """

    manifest_file = Path(manifest_path)
    if manifest_file.is_dir():
        manifest_file = manifest_file / "manifest.json"
    manifest = load_manifest(manifest_file, verify_hashes=True)
    settings = config or PolarDescriptorConfig()
    root = manifest_file.parent.resolve()
    destination = (
        Path(output_path).resolve()
        if output_path is not None
        else root / "descriptor_index.json"
    )
    try:
        destination.relative_to(root)
    except ValueError as exc:
        raise MapManifestError(
            "descriptor index must be inside the map directory"
        ) from exc
    tiles_by_grid = {(tile.ix, tile.iy): tile for tile in manifest.tiles}
    tiles_by_id = {tile.id: tile for tile in manifest.tiles}
    source_layer = None
    stable_points_by_grid: Optional[
        Dict[Tuple[int, int], List[Tuple[float, float, float]]]
    ] = None
    if source_layer_pcd is not None:
        stable_path = Path(source_layer_pcd).resolve()
        try:
            stable_relative = stable_path.relative_to(root)
        except ValueError as exc:
            raise DescriptorError(
                "descriptor source layer must be inside the map directory"
            ) from exc
        stable_cloud = read_pcd(stable_path)
        if stable_cloud.point_count <= 0:
            raise DescriptorError("descriptor source layer contains no points")
        stable_points_by_grid = {}
        for point in stable_cloud.xyz_points():
            key = (
                math.floor(point[0] / manifest.tile_size_m),
                math.floor(point[1] / manifest.tile_size_m),
            )
            stable_points_by_grid.setdefault(key, []).append(point)
        source_layer = DescriptorSourceLayer(
            path=stable_relative.as_posix(),
            sha256=sha256_file(stable_path),
            point_count=stable_cloud.point_count,
        )

    @lru_cache(maxsize=128)
    def tile_points(tile_id: str) -> Tuple[Tuple[float, float, float], ...]:
        tile = tiles_by_id[tile_id]
        return read_pcd(root / tile.path).xyz_points()

    grid_radius = int(math.ceil(settings.max_radius_m / manifest.tile_size_m))
    entries = []
    for tile in manifest.tiles:
        center = (
            (tile.grid_min[0] + tile.grid_max[0]) * 0.5,
            (tile.grid_min[1] + tile.grid_max[1]) * 0.5,
            0.0,
        )
        neighborhood: List[Tuple[float, float, float]] = []
        for ix in range(tile.ix - grid_radius, tile.ix + grid_radius + 1):
            for iy in range(tile.iy - grid_radius, tile.iy + grid_radius + 1):
                if stable_points_by_grid is not None:
                    neighborhood.extend(stable_points_by_grid.get((ix, iy), ()))
                else:
                    neighbor = tiles_by_grid.get((ix, iy))
                    if neighbor is not None:
                        neighborhood.extend(tile_points(neighbor.id))
        descriptor = compute_polar_descriptor(neighborhood, settings, center)
        entries.append(
            DescriptorEntry(
                id=tile.id,
                tile_id=tile.id,
                center=center,
                source_sha256=tile.sha256,
                descriptor=descriptor,
            )
        )
    index = PolarDescriptorIndex(
        map_id=manifest.map_id,
        config=settings,
        entries=entries,
        source_layer=source_layer,
    )
    save_descriptor_index(index, destination)
    bind_descriptor_index(manifest_file, destination)
    return index


def compile_map_bundle(
    source_pcd: os.PathLike,
    output_directory: os.PathLike,
    map_id: Optional[str] = None,
    frame_id: str = "map",
    tile_size_m: float = 20.0,
    voxel_size_m: float = 0.20,
    descriptor_config: Optional[PolarDescriptorConfig] = None,
    overwrite: bool = False,
) -> Tuple[MapManifest, PolarDescriptorIndex]:
    """Build and verify a complete v2 map before atomically publishing it."""

    source = Path(source_pcd).resolve()
    requested_output = Path(output_directory)
    if requested_output.is_symlink():
        raise DescriptorError("map output must not be a symlink")
    output = requested_output.resolve()
    if output == Path(output.anchor) or output == Path.home().resolve():
        raise DescriptorError("refusing to use a filesystem root or home as map output")
    try:
        source.relative_to(output)
    except ValueError:
        pass
    else:
        raise DescriptorError("source PCD must not be inside the output directory")
    if output.exists() and not output.is_dir():
        raise DescriptorError("output path exists and is not a directory")
    if output.exists() and any(output.iterdir()):
        if not overwrite:
            raise DescriptorError(
                "output directory exists and is not empty; pass overwrite=True explicitly"
            )
        require_replaceable_map_directory(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    staging_root = Path(
        tempfile.mkdtemp(
            prefix=".%s.bundle-staging." % output.name,
            dir=str(output.parent),
        )
    )
    candidate = staging_root / "bundle"
    backup: Optional[Path] = None
    try:
        build_tiled_map(
            source_pcd,
            candidate,
            map_id=map_id,
            frame_id=frame_id,
            tile_size_m=tile_size_m,
            voxel_size_m=voxel_size_m,
            overwrite=False,
        )
        build_descriptor_index(
            candidate / "manifest.json",
            candidate / "descriptor_index.json",
            descriptor_config,
        )
        manifest, index = verify_map_bundle(candidate / "manifest.json")
        if output.exists():
            if any(output.iterdir()):
                backup = output.with_name(
                    ".%s.backup.%s" % (output.name, uuid.uuid4().hex)
                )
                os.replace(str(output), str(backup))
            else:
                output.rmdir()
        try:
            os.replace(str(candidate), str(output))
        except Exception:
            if backup is not None and not output.exists():
                os.replace(str(backup), str(output))
                backup = None
            raise
        if backup is not None:
            shutil.rmtree(str(backup))
            backup = None
        return manifest, index
    finally:
        if staging_root.exists():
            shutil.rmtree(str(staging_root))
        if backup is not None and backup.exists() and not output.exists():
            os.replace(str(backup), str(output))


def verify_map_bundle(
    manifest_path: os.PathLike,
) -> Tuple[MapManifest, PolarDescriptorIndex]:
    """Verify every hash and cross-file reference in a compiled map bundle."""

    manifest_file = Path(manifest_path)
    if manifest_file.is_dir():
        manifest_file = manifest_file / "manifest.json"
    manifest = load_manifest(manifest_file, verify_hashes=True)
    if manifest.schema != MAP_MANIFEST_SCHEMA:
        raise DescriptorError(
            "production map bundle requires go2.map_tiles/v2 descriptor binding"
        )
    root = manifest_file.parent.resolve()
    index_path = (root / str(manifest.descriptor_index_path)).resolve()
    try:
        index_path.relative_to(root)
    except ValueError as exc:
        raise DescriptorError("descriptor index path escapes map directory") from exc
    if sha256_file(index_path) != manifest.descriptor_index_sha256:
        raise DescriptorError("descriptor index hash does not match manifest anchor")
    index = load_descriptor_index(index_path, verify_checksum=True)
    if index.map_id != manifest.map_id:
        raise DescriptorError("descriptor index map_id does not match manifest")
    if index.source_layer is not None:
        source_layer_path = (root / index.source_layer.path).resolve()
        try:
            source_layer_path.relative_to(root)
        except ValueError as exc:
            raise DescriptorError(
                "descriptor source-layer path escapes map directory"
            ) from exc
        if not source_layer_path.is_file():
            raise DescriptorError("descriptor source-layer asset is missing")
        if sha256_file(source_layer_path) != index.source_layer.sha256:
            raise DescriptorError("descriptor source-layer hash mismatch")
        source_layer_cloud = read_pcd(source_layer_path)
        if source_layer_cloud.point_count != index.source_layer.point_count:
            raise DescriptorError("descriptor source-layer point count mismatch")
    tiles = {tile.id: tile for tile in manifest.tiles}
    tile_ids = [entry.tile_id for entry in index.entries]
    if len(tile_ids) != len(set(tile_ids)):
        raise DescriptorError("descriptor tile_id values must be unique")
    entries = {entry.tile_id: entry for entry in index.entries}
    if set(entries.keys()) != set(tiles.keys()):
        raise DescriptorError(
            "descriptor entries must cover every manifest tile exactly once"
        )
    tolerance = max(1.0e-8, manifest.tile_size_m * 1.0e-9)
    for tile_id, entry in entries.items():
        tile = tiles[tile_id]
        if entry.source_sha256 != tile.sha256:
            raise DescriptorError(
                "descriptor source hash does not match tile %s" % tile_id
            )
        expected_center = (
            (tile.grid_min[0] + tile.grid_max[0]) * 0.5,
            (tile.grid_min[1] + tile.grid_max[1]) * 0.5,
            0.0,
        )
        if any(
            abs(actual - expected) > tolerance
            for actual, expected in zip(entry.center, expected_center)
        ):
            raise DescriptorError("descriptor center does not match tile %s" % tile_id)
    return manifest, index

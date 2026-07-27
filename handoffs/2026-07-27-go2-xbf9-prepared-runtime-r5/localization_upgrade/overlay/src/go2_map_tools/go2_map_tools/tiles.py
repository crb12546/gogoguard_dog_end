"""Build and verify deterministic 20-metre PCD map tiles."""

from dataclasses import dataclass, replace
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import struct
import tempfile
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple
import uuid

from .pcd import PCDCloud, PCDDataError, PCDField, read_pcd, write_pcd_ascii
from .voxel import voxel_downsample


INTERMEDIATE_MAP_MANIFEST_SCHEMA = "go2.map_tiles/v1"
MAP_MANIFEST_SCHEMA = "go2.map_tiles/v2"
_TILE_ID = re.compile(r"^x[+-][0-9]{6}_y[+-][0-9]{6}$")


class MapManifestError(ValueError):
    """Raised when a map manifest or tile asset fails validation."""


def sha256_file(path: os.PathLike) -> str:
    digest = hashlib.sha256()
    try:
        with Path(path).open("rb") as handle:
            for block in iter(lambda: handle.read(1024 * 1024), b""):
                digest.update(block)
    except OSError as exc:
        raise MapManifestError("cannot hash %s: %s" % (path, exc)) from exc
    return digest.hexdigest()


def _finite(value: Any, name: str, positive: bool = False) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise MapManifestError("%s must be numeric" % name) from exc
    if not math.isfinite(result):
        raise MapManifestError("%s must be finite" % name)
    if positive and result <= 0.0:
        raise MapManifestError("%s must be positive" % name)
    return result


def _vector(value: Any, size: int, name: str) -> Tuple[float, ...]:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)):
        raise MapManifestError("%s must be an array" % name)
    if len(value) != size:
        raise MapManifestError("%s must have %d values" % (name, size))
    return tuple(_finite(item, "%s value" % name) for item in value)


@dataclass(frozen=True)
class MapTile:
    id: str
    ix: int
    iy: int
    path: str
    sha256: str
    point_count: int
    bounds_min: Tuple[float, float, float]
    bounds_max: Tuple[float, float, float]
    grid_min: Tuple[float, float]
    grid_max: Tuple[float, float]

    def __post_init__(self) -> None:
        tile_id = str(self.id)
        if not _TILE_ID.fullmatch(tile_id):
            raise MapManifestError("invalid tile id %r" % tile_id)
        ix = int(self.ix)
        iy = int(self.iy)
        if tile_id != tile_id_for(ix, iy):
            raise MapManifestError("tile id does not match ix/iy")
        relative_path = Path(str(self.path))
        if relative_path.is_absolute() or ".." in relative_path.parts:
            raise MapManifestError("tile path must be a safe relative path")
        if relative_path.as_posix() != "tiles/%s.pcd" % tile_id:
            raise MapManifestError("tile path does not match tile id")
        digest = str(self.sha256).lower()
        if not re.fullmatch(r"[0-9a-f]{64}", digest):
            raise MapManifestError("tile sha256 is invalid")
        point_count = int(self.point_count)
        if point_count <= 0:
            raise MapManifestError("tile point_count must be positive")
        minimum = _vector(self.bounds_min, 3, "tile bounds min")
        maximum = _vector(self.bounds_max, 3, "tile bounds max")
        grid_min = _vector(self.grid_min, 2, "tile grid min")
        grid_max = _vector(self.grid_max, 2, "tile grid max")
        if any(a > b for a, b in zip(minimum, maximum)):
            raise MapManifestError("tile bounds are reversed")
        if any(a >= b for a, b in zip(grid_min, grid_max)):
            raise MapManifestError("tile grid bounds are reversed")
        object.__setattr__(self, "id", tile_id)
        object.__setattr__(self, "ix", ix)
        object.__setattr__(self, "iy", iy)
        object.__setattr__(self, "path", relative_path.as_posix())
        object.__setattr__(self, "sha256", digest)
        object.__setattr__(self, "point_count", point_count)
        object.__setattr__(self, "bounds_min", minimum)
        object.__setattr__(self, "bounds_max", maximum)
        object.__setattr__(self, "grid_min", grid_min)
        object.__setattr__(self, "grid_max", grid_max)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "id": self.id,
            "ix": self.ix,
            "iy": self.iy,
            "path": self.path,
            "sha256": self.sha256,
            "point_count": self.point_count,
            "bounds": {"min": list(self.bounds_min), "max": list(self.bounds_max)},
            "grid_bounds": {"min": list(self.grid_min), "max": list(self.grid_max)},
        }


@dataclass(frozen=True)
class MapManifest:
    map_id: str
    created_utc: str
    frame_id: str
    tile_size_m: float
    voxel_size_m: float
    source_filename: str
    source_sha256: str
    source_point_count: int
    bounds_min: Tuple[float, float, float]
    bounds_max: Tuple[float, float, float]
    tiles: Tuple[MapTile, ...]
    descriptor_index_path: Optional[str] = None
    descriptor_index_sha256: Optional[str] = None
    schema: str = MAP_MANIFEST_SCHEMA

    def __post_init__(self) -> None:
        if self.schema not in (
            INTERMEDIATE_MAP_MANIFEST_SCHEMA,
            MAP_MANIFEST_SCHEMA,
        ):
            raise MapManifestError("unsupported manifest schema %r" % self.schema)
        map_id = str(self.map_id).strip()
        frame_id = str(self.frame_id).strip()
        if not map_id or "/" in map_id or "\\" in map_id:
            raise MapManifestError("map_id must be a safe non-empty name")
        if not frame_id:
            raise MapManifestError("frame_id must not be empty")
        created_utc = str(self.created_utc).strip()
        try:
            parsed_created = datetime.fromisoformat(created_utc.replace("Z", "+00:00"))
        except ValueError as exc:
            raise MapManifestError("created_utc must be an ISO-8601 timestamp") from exc
        if parsed_created.tzinfo is None:
            raise MapManifestError("created_utc must include a timezone")
        tile_size = _finite(self.tile_size_m, "tile_size_m", positive=True)
        voxel_size = _finite(self.voxel_size_m, "voxel_size_m", positive=True)
        source_value = str(self.source_filename)
        source_name = Path(source_value).name
        if not source_name or source_name != source_value:
            raise MapManifestError("source filename must be a basename")
        source_digest = str(self.source_sha256).lower()
        if not re.fullmatch(r"[0-9a-f]{64}", source_digest):
            raise MapManifestError("source sha256 is invalid")
        source_count = int(self.source_point_count)
        if source_count <= 0:
            raise MapManifestError("source point count must be positive")
        minimum = _vector(self.bounds_min, 3, "map bounds min")
        maximum = _vector(self.bounds_max, 3, "map bounds max")
        if any(a > b for a, b in zip(minimum, maximum)):
            raise MapManifestError("map bounds are reversed")
        tiles = tuple(self.tiles)
        if not tiles:
            raise MapManifestError("manifest must contain at least one tile")
        if len({tile.id for tile in tiles}) != len(tiles):
            raise MapManifestError("tile ids must be unique")
        if tuple(sorted(tiles, key=lambda tile: (tile.ix, tile.iy))) != tiles:
            raise MapManifestError("tiles must be sorted by ix then iy")
        tolerance = max(1.0e-8, tile_size * 1.0e-9)
        for tile in tiles:
            expected_grid_min = (tile.ix * tile_size, tile.iy * tile_size)
            expected_grid_max = (
                (tile.ix + 1) * tile_size,
                (tile.iy + 1) * tile_size,
            )
            if any(
                abs(actual - expected) > tolerance
                for actual, expected in zip(tile.grid_min, expected_grid_min)
            ) or any(
                abs(actual - expected) > tolerance
                for actual, expected in zip(tile.grid_max, expected_grid_max)
            ):
                raise MapManifestError("tile grid bounds do not match ix/iy")
            if (
                tile.bounds_min[0] < tile.grid_min[0] - tolerance
                or tile.bounds_min[1] < tile.grid_min[1] - tolerance
                or tile.bounds_max[0] > tile.grid_max[0] + tolerance
                or tile.bounds_max[1] > tile.grid_max[1] + tolerance
            ):
                raise MapManifestError("tile point bounds exceed its grid cell")
        derived_minimum = tuple(
            min(tile.bounds_min[index] for tile in tiles) for index in range(3)
        )
        derived_maximum = tuple(
            max(tile.bounds_max[index] for tile in tiles) for index in range(3)
        )
        if any(
            abs(actual - expected) > tolerance
            for actual, expected in zip(minimum, derived_minimum)
        ) or any(
            abs(actual - expected) > tolerance
            for actual, expected in zip(maximum, derived_maximum)
        ):
            raise MapManifestError("map bounds do not match tile bounds")
        if sum(tile.point_count for tile in tiles) > source_count:
            raise MapManifestError("tiled point count exceeds source point count")
        descriptor_path: Optional[str] = None
        descriptor_digest: Optional[str] = None
        if self.schema == MAP_MANIFEST_SCHEMA:
            if self.descriptor_index_path is None:
                raise MapManifestError(
                    "v2 manifest must bind a descriptor index path"
                )
            relative_descriptor_path = Path(str(self.descriptor_index_path))
            if (
                relative_descriptor_path.is_absolute()
                or ".." in relative_descriptor_path.parts
                or relative_descriptor_path.as_posix() in ("", ".")
            ):
                raise MapManifestError(
                    "descriptor index path must be a safe relative path"
                )
            descriptor_path = relative_descriptor_path.as_posix()
            descriptor_digest = str(self.descriptor_index_sha256 or "").lower()
            if not re.fullmatch(r"[0-9a-f]{64}", descriptor_digest):
                raise MapManifestError("descriptor index sha256 is invalid")
        elif (
            self.descriptor_index_path is not None
            or self.descriptor_index_sha256 is not None
        ):
            raise MapManifestError(
                "intermediate v1 manifest must not claim a descriptor identity"
            )
        object.__setattr__(self, "map_id", map_id)
        object.__setattr__(self, "created_utc", created_utc)
        object.__setattr__(self, "frame_id", frame_id)
        object.__setattr__(self, "tile_size_m", tile_size)
        object.__setattr__(self, "voxel_size_m", voxel_size)
        object.__setattr__(self, "source_filename", source_name)
        object.__setattr__(self, "source_sha256", source_digest)
        object.__setattr__(self, "source_point_count", source_count)
        object.__setattr__(self, "bounds_min", minimum)
        object.__setattr__(self, "bounds_max", maximum)
        object.__setattr__(self, "tiles", tiles)
        object.__setattr__(self, "descriptor_index_path", descriptor_path)
        object.__setattr__(self, "descriptor_index_sha256", descriptor_digest)

    @property
    def tiled_point_count(self) -> int:
        return sum(tile.point_count for tile in self.tiles)

    def to_dict(self) -> Dict[str, Any]:
        result = {
            "schema": self.schema,
            "map_id": self.map_id,
            "created_utc": self.created_utc,
            "frame_id": self.frame_id,
            "tile_size_m": self.tile_size_m,
            "voxel_size_m": self.voxel_size_m,
            "source": {
                "filename": self.source_filename,
                "sha256": self.source_sha256,
                "point_count": self.source_point_count,
            },
            "bounds": {"min": list(self.bounds_min), "max": list(self.bounds_max)},
            "tiles": [tile.to_dict() for tile in self.tiles],
        }
        if self.schema == MAP_MANIFEST_SCHEMA:
            result["descriptor_index"] = {
                "path": self.descriptor_index_path,
                "sha256": self.descriptor_index_sha256,
            }
        return result


def tile_id_for(ix: int, iy: int) -> str:
    ix = int(ix)
    iy = int(iy)
    if abs(ix) > 999999 or abs(iy) > 999999:
        raise MapManifestError("tile index exceeds six-digit manifest format")
    return "x%+07d_y%+07d" % (ix, iy)


def _bounds(
    points: Sequence[Sequence[float]],
) -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
    if not points:
        raise MapManifestError("cannot calculate bounds of an empty point set")
    return (
        (
            min(point[0] for point in points),
            min(point[1] for point in points),
            min(point[2] for point in points),
        ),
        (
            max(point[0] for point in points),
            max(point[1] for point in points),
            max(point[2] for point in points),
        ),
    )


def _float32(value: float) -> float:
    try:
        return struct.unpack("<f", struct.pack("<f", value))[0]
    except (OverflowError, struct.error) as exc:
        raise MapManifestError(
            "source coordinate cannot be represented in float32 tiles"
        ) from exc


def _manifest_json(manifest: MapManifest) -> bytes:
    return (
        json.dumps(
            manifest.to_dict(),
            ensure_ascii=False,
            allow_nan=False,
            indent=2,
            sort_keys=True,
        )
        + "\n"
    ).encode("utf-8")


def _write_manifest(root: Path, manifest: MapManifest) -> None:
    payload = _manifest_json(manifest)
    manifest_path = root / "manifest.json"
    root.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=".manifest.", suffix=".tmp", dir=str(root)
    )
    try:
        with os.fdopen(descriptor, "wb") as handle:
            handle.write(payload)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary_name, str(manifest_path))
    except Exception:
        try:
            os.unlink(temporary_name)
        except OSError:
            pass
        raise
    digest = hashlib.sha256(payload).hexdigest()
    checksum_payload = ("%s  manifest.json\n" % digest).encode("ascii")
    checksum_descriptor, checksum_temporary_name = tempfile.mkstemp(
        prefix=".manifest.sha256.", suffix=".tmp", dir=str(root)
    )
    try:
        with os.fdopen(checksum_descriptor, "wb") as handle:
            handle.write(checksum_payload)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(checksum_temporary_name, str(root / "manifest.sha256"))
    except Exception:
        try:
            os.unlink(checksum_temporary_name)
        except OSError:
            pass
        raise


def require_replaceable_map_directory(output: Path) -> None:
    """Refuse to overwrite broad or unrelated directories."""

    resolved = output.resolve()
    if resolved == Path(resolved.anchor) or resolved == Path.home().resolve():
        raise MapManifestError("refusing to use a filesystem root or home as map output")
    if output.is_symlink():
        raise MapManifestError("map output must not be a symlink")
    if output.exists() and any(output.iterdir()):
        required = (
            output / "manifest.json",
            output / "manifest.sha256",
            output / "tiles",
        )
        if (
            not required[0].is_file()
            or not required[1].is_file()
            or not required[2].is_dir()
        ):
            raise MapManifestError("refusing to overwrite a non-map directory")
        existing = load_manifest(output, verify_hashes=False)
        if existing.schema == MAP_MANIFEST_SCHEMA:
            raise MapManifestError(
                "published v2 map bundles are immutable; compile to a new directory"
            )


def build_tiled_map(
    source_pcd: os.PathLike,
    output_directory: os.PathLike,
    map_id: Optional[str] = None,
    frame_id: str = "map",
    tile_size_m: float = 20.0,
    voxel_size_m: float = 0.20,
    overwrite: bool = False,
) -> MapManifest:
    """Build tiles in a staging directory and atomically publish the result."""

    source = Path(source_pcd).resolve()
    requested_output = Path(output_directory)
    if requested_output.is_symlink():
        raise MapManifestError("map output must not be a symlink")
    output = requested_output.resolve()
    tile_size = _finite(tile_size_m, "tile_size_m", positive=True)
    voxel_size = _finite(voxel_size_m, "voxel_size_m", positive=True)
    if voxel_size > tile_size:
        raise MapManifestError("voxel_size_m must not exceed tile_size_m")
    if output.exists() and not output.is_dir():
        raise MapManifestError("output path exists and is not a directory")
    try:
        source.relative_to(output)
    except ValueError:
        pass
    else:
        raise MapManifestError("source PCD must not be inside the output directory")
    if output.exists() and any(output.iterdir()):
        if not overwrite:
            raise MapManifestError(
                "output directory exists and is not empty; pass overwrite=True explicitly"
            )
        require_replaceable_map_directory(output)
    cloud = read_pcd(source)
    xyz = cloud.xyz_points()
    if not xyz:
        raise MapManifestError("source PCD contains no points")
    downsampled = voxel_downsample(xyz, voxel_size)
    # Tile files declare float32 XYZ. Quantize before assigning grid cells so a
    # rounded boundary point always lands in the tile a reader will calculate.
    published_points = tuple(
        (_float32(point[0]), _float32(point[1]), _float32(point[2]))
        for point in downsampled
    )
    grouped: Dict[Tuple[int, int], List[Tuple[float, ...]]] = {}
    for point in published_points:
        key = (
            math.floor(point[0] / tile_size),
            math.floor(point[1] / tile_size),
        )
        grouped.setdefault(key, []).append(point)

    output.parent.mkdir(parents=True, exist_ok=True)
    staging = Path(
        tempfile.mkdtemp(prefix=".%s.staging." % output.name, dir=str(output.parent))
    )
    backup: Optional[Path] = None
    try:
        tile_directory = staging / "tiles"
        tile_directory.mkdir()
        tile_records = []
        xyz_fields = (
            PCDField("x", 4, "F"),
            PCDField("y", 4, "F"),
            PCDField("z", 4, "F"),
        )
        for (ix, iy), points in sorted(grouped.items()):
            tile_id = tile_id_for(ix, iy)
            relative_path = "tiles/%s.pcd" % tile_id
            tile_path = staging / relative_path
            quantized_points = tuple(points)
            tile_cloud = PCDCloud(
                fields=xyz_fields,
                rows=quantized_points,
                width=len(quantized_points),
                height=1,
            )
            write_pcd_ascii(tile_cloud, tile_path)
            minimum, maximum = _bounds(quantized_points)
            tile_records.append(
                MapTile(
                    id=tile_id,
                    ix=ix,
                    iy=iy,
                    path=relative_path,
                    sha256=sha256_file(tile_path),
                    point_count=len(quantized_points),
                    bounds_min=minimum,
                    bounds_max=maximum,
                    grid_min=(ix * tile_size, iy * tile_size),
                    grid_max=((ix + 1) * tile_size, (iy + 1) * tile_size),
                )
            )
        minimum, maximum = _bounds(published_points)
        source_digest = sha256_file(source)
        effective_map_id = map_id or "%s-%s" % (source.stem, source_digest[:12])
        manifest = MapManifest(
            map_id=effective_map_id,
            created_utc=datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
            frame_id=frame_id,
            tile_size_m=tile_size,
            voxel_size_m=voxel_size,
            source_filename=source.name,
            source_sha256=source_digest,
            source_point_count=cloud.point_count,
            bounds_min=minimum,
            bounds_max=maximum,
            tiles=tuple(tile_records),
            schema=INTERMEDIATE_MAP_MANIFEST_SCHEMA,
        )
        _write_manifest(staging, manifest)

        if output.exists():
            if any(output.iterdir()):
                backup = output.with_name(
                    ".%s.backup.%s" % (output.name, uuid.uuid4().hex)
                )
                os.replace(str(output), str(backup))
            else:
                output.rmdir()
        try:
            os.replace(str(staging), str(output))
        except Exception:
            if backup is not None and not output.exists():
                os.replace(str(backup), str(output))
                backup = None
            raise
        if backup is not None:
            shutil.rmtree(str(backup))
            backup = None
        return manifest
    finally:
        if staging.exists():
            shutil.rmtree(str(staging))
        if backup is not None and backup.exists() and not output.exists():
            os.replace(str(backup), str(output))


def _manifest_from_dict(data: Mapping[str, Any]) -> MapManifest:
    if not isinstance(data, Mapping):
        raise MapManifestError("manifest must be a JSON object")
    expected = {
        "schema",
        "map_id",
        "created_utc",
        "frame_id",
        "tile_size_m",
        "voxel_size_m",
        "source",
        "bounds",
        "tiles",
    }
    schema = data.get("schema")
    if schema == MAP_MANIFEST_SCHEMA:
        expected.add("descriptor_index")
    unknown = set(data.keys()) - expected
    missing = expected - set(data.keys())
    if unknown or missing:
        raise MapManifestError(
            "manifest fields mismatch; missing=%s unknown=%s"
            % (sorted(missing), sorted(unknown))
        )
    source = data["source"]
    bounds = data["bounds"]
    tiles_data = data["tiles"]
    if not isinstance(source, Mapping) or not isinstance(bounds, Mapping):
        raise MapManifestError("manifest source and bounds must be objects")
    if not isinstance(tiles_data, list):
        raise MapManifestError("manifest tiles must be an array")
    tiles = []
    for index, item in enumerate(tiles_data):
        if not isinstance(item, Mapping):
            raise MapManifestError("tile %d must be an object" % index)
        try:
            tile_bounds = item["bounds"]
            grid_bounds = item["grid_bounds"]
            tiles.append(
                MapTile(
                    id=item["id"],
                    ix=item["ix"],
                    iy=item["iy"],
                    path=item["path"],
                    sha256=item["sha256"],
                    point_count=item["point_count"],
                    bounds_min=tile_bounds["min"],
                    bounds_max=tile_bounds["max"],
                    grid_min=grid_bounds["min"],
                    grid_max=grid_bounds["max"],
                )
            )
        except (KeyError, TypeError) as exc:
            raise MapManifestError("tile %d has malformed fields" % index) from exc
    try:
        descriptor_metadata = data.get("descriptor_index")
        if schema == MAP_MANIFEST_SCHEMA and not isinstance(
            descriptor_metadata, Mapping
        ):
            raise MapManifestError("manifest descriptor_index must be an object")
        if descriptor_metadata is not None and set(descriptor_metadata.keys()) != {
            "path",
            "sha256",
        }:
            raise MapManifestError(
                "manifest descriptor_index fields must be path and sha256"
            )
        return MapManifest(
            schema=data["schema"],
            map_id=data["map_id"],
            created_utc=data["created_utc"],
            frame_id=data["frame_id"],
            tile_size_m=data["tile_size_m"],
            voxel_size_m=data["voxel_size_m"],
            source_filename=source["filename"],
            source_sha256=source["sha256"],
            source_point_count=source["point_count"],
            bounds_min=bounds["min"],
            bounds_max=bounds["max"],
            tiles=tuple(tiles),
            descriptor_index_path=(
                descriptor_metadata["path"] if descriptor_metadata else None
            ),
            descriptor_index_sha256=(
                descriptor_metadata["sha256"] if descriptor_metadata else None
            ),
        )
    except KeyError as exc:
        raise MapManifestError("manifest is missing nested field %s" % exc.args[0]) from exc


def load_manifest(
    manifest_path: os.PathLike, verify_hashes: bool = False
) -> MapManifest:
    path = Path(manifest_path)
    if path.is_dir():
        path = path / "manifest.json"
    try:
        raw = path.read_bytes()
        data = json.loads(
            raw.decode("utf-8"),
            parse_constant=lambda value: (_reject_json_constant(value)),
        )
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise MapManifestError("cannot read manifest %s: %s" % (path, exc)) from exc
    manifest = _manifest_from_dict(data)
    if verify_hashes:
        verify_manifest(path, manifest)
    return manifest


def _reject_json_constant(value: str) -> None:
    raise MapManifestError("non-finite JSON number %s is not allowed" % value)


def verify_manifest(
    manifest_path: os.PathLike, manifest: Optional[MapManifest] = None
) -> MapManifest:
    path = Path(manifest_path)
    if path.is_dir():
        path = path / "manifest.json"
    current = manifest or load_manifest(path, verify_hashes=False)
    root = path.parent
    checksum_path = root / "manifest.sha256"
    try:
        checksum_tokens = checksum_path.read_text(encoding="ascii").split()
    except OSError as exc:
        raise MapManifestError("cannot read manifest.sha256: %s" % exc) from exc
    if len(checksum_tokens) != 2 or checksum_tokens[1] != "manifest.json":
        raise MapManifestError("manifest.sha256 has invalid format")
    actual_manifest_hash = sha256_file(path)
    if checksum_tokens[0].lower() != actual_manifest_hash:
        raise MapManifestError("manifest.json hash mismatch")
    for tile in current.tiles:
        tile_path = root / tile.path
        if not tile_path.is_file():
            raise MapManifestError("missing tile %s" % tile.path)
        if sha256_file(tile_path) != tile.sha256:
            raise MapManifestError("tile hash mismatch: %s" % tile.path)
        cloud = read_pcd(tile_path)
        if cloud.point_count != tile.point_count:
            raise MapManifestError("tile point count mismatch: %s" % tile.path)
        if tuple(field.name for field in cloud.fields) != ("x", "y", "z"):
            raise MapManifestError("tile fields must be exactly x, y, z: %s" % tile.path)
        points = cloud.xyz_points()
        minimum, maximum = _bounds(points)
        tolerance = max(1.0e-8, current.voxel_size_m * 1.0e-6)
        if any(
            abs(actual - expected) > tolerance
            for actual, expected in zip(minimum, tile.bounds_min)
        ) or any(
            abs(actual - expected) > tolerance
            for actual, expected in zip(maximum, tile.bounds_max)
        ):
            raise MapManifestError("tile bounds mismatch: %s" % tile.path)
    return current


def bind_descriptor_index(
    manifest_path: os.PathLike, descriptor_index_path: os.PathLike
) -> MapManifest:
    """Upgrade a verified tile manifest to v2 and bind the exact index bytes.

    The descriptor must live inside the map directory.  The index and its own
    checksum are written first; publishing the v2 manifest last makes an
    interrupted build fail closed as either v1 or a checksum mismatch.
    """

    path = Path(manifest_path)
    if path.is_dir():
        path = path / "manifest.json"
    current = load_manifest(path, verify_hashes=True)
    if current.schema != INTERMEDIATE_MAP_MANIFEST_SCHEMA:
        raise MapManifestError(
            "published v2 map bundles are immutable; bind a new v1 build"
        )
    root = path.parent.resolve()
    descriptor_path = Path(descriptor_index_path).resolve()
    try:
        relative_path = descriptor_path.relative_to(root)
    except ValueError as exc:
        raise MapManifestError(
            "descriptor index must be inside the map directory"
        ) from exc
    if not descriptor_path.is_file():
        raise MapManifestError("descriptor index does not exist")
    checksum_path = descriptor_path.with_suffix(descriptor_path.suffix + ".sha256")
    try:
        checksum_tokens = checksum_path.read_text(encoding="ascii").split()
    except OSError as exc:
        raise MapManifestError("cannot read descriptor index checksum: %s" % exc) from exc
    digest = sha256_file(descriptor_path)
    if (
        len(checksum_tokens) != 2
        or checksum_tokens[1] != descriptor_path.name
        or checksum_tokens[0].lower() != digest
    ):
        raise MapManifestError("descriptor index checksum mismatch")
    finalized = replace(
        current,
        schema=MAP_MANIFEST_SCHEMA,
        descriptor_index_path=relative_path.as_posix(),
        descriptor_index_sha256=digest,
    )
    _write_manifest(root, finalized)
    return finalized

"""Strict PCD v0.7 ASCII and uncompressed-binary reader/writer.

The parser is intentionally small and auditable.  It supports scalar and
``COUNT > 1`` fields and rejects compressed payloads, malformed dimensions,
truncation, trailing binary bytes, and non-finite values by default.
"""

from dataclasses import dataclass
import math
import os
from pathlib import Path
import struct
import tempfile
from typing import Dict, Iterable, List, Optional, Sequence, Tuple


class PCDDataError(ValueError):
    """Raised when a PCD document is malformed or unsafe to consume."""


_STRUCT_CODES = {
    ("F", 4): "f",
    ("F", 8): "d",
    ("I", 1): "b",
    ("I", 2): "h",
    ("I", 4): "i",
    ("I", 8): "q",
    ("U", 1): "B",
    ("U", 2): "H",
    ("U", 4): "I",
    ("U", 8): "Q",
}
_INTEGER_LIMITS = {
    ("I", 1): (-(2 ** 7), 2 ** 7 - 1),
    ("I", 2): (-(2 ** 15), 2 ** 15 - 1),
    ("I", 4): (-(2 ** 31), 2 ** 31 - 1),
    ("I", 8): (-(2 ** 63), 2 ** 63 - 1),
    ("U", 1): (0, 2 ** 8 - 1),
    ("U", 2): (0, 2 ** 16 - 1),
    ("U", 4): (0, 2 ** 32 - 1),
    ("U", 8): (0, 2 ** 64 - 1),
}


@dataclass(frozen=True)
class PCDField:
    name: str
    size: int
    type: str
    count: int = 1

    def __post_init__(self) -> None:
        name = str(self.name).strip()
        field_type = str(self.type).upper().strip()
        size = int(self.size)
        count = int(self.count)
        if not name or any(character.isspace() for character in name):
            raise PCDDataError("PCD field name must be a non-empty token")
        if (field_type, size) not in _STRUCT_CODES:
            raise PCDDataError(
                "unsupported PCD field representation %s%d" % (field_type, size)
            )
        if count <= 0:
            raise PCDDataError("PCD field count must be positive")
        object.__setattr__(self, "name", name)
        object.__setattr__(self, "type", field_type)
        object.__setattr__(self, "size", size)
        object.__setattr__(self, "count", count)


@dataclass(frozen=True)
class PCDCloud:
    fields: Tuple[PCDField, ...]
    rows: Tuple[Tuple[float, ...], ...]
    width: int
    height: int = 1
    viewpoint: str = "0 0 0 1 0 0 0"
    data_encoding: str = "memory"

    def __post_init__(self) -> None:
        fields = tuple(self.fields)
        rows = tuple(tuple(row) for row in self.rows)
        if not fields:
            raise PCDDataError("PCD cloud must define at least one field")
        if len({field.name for field in fields}) != len(fields):
            raise PCDDataError("PCD field names must be unique")
        width = int(self.width)
        height = int(self.height)
        if width < 0 or height <= 0:
            raise PCDDataError("PCD dimensions are invalid")
        if width * height != len(rows):
            raise PCDDataError(
                "PCD dimensions declare %d points but cloud has %d"
                % (width * height, len(rows))
            )
        scalar_count = sum(field.count for field in fields)
        for index, row in enumerate(rows):
            if len(row) != scalar_count:
                raise PCDDataError(
                    "row %d has %d scalars, expected %d"
                    % (index, len(row), scalar_count)
                )
        viewpoint_tokens = str(self.viewpoint).split()
        if len(viewpoint_tokens) != 7:
            raise PCDDataError("VIEWPOINT must contain exactly seven numbers")
        try:
            viewpoint_values = tuple(float(value) for value in viewpoint_tokens)
        except ValueError as exc:
            raise PCDDataError("VIEWPOINT values must be numeric") from exc
        if not all(math.isfinite(value) for value in viewpoint_values):
            raise PCDDataError("VIEWPOINT values must be finite")
        if all(value == 0.0 for value in viewpoint_values[3:]):
            raise PCDDataError("VIEWPOINT quaternion must not be all zero")
        object.__setattr__(self, "fields", fields)
        object.__setattr__(self, "rows", rows)
        object.__setattr__(self, "width", width)
        object.__setattr__(self, "height", height)
        object.__setattr__(
            self, "viewpoint", " ".join("%.17g" % value for value in viewpoint_values)
        )
        object.__setattr__(self, "data_encoding", str(self.data_encoding))

    @property
    def point_count(self) -> int:
        return len(self.rows)

    @property
    def scalar_count(self) -> int:
        return sum(field.count for field in self.fields)

    def field_offset(self, name: str) -> int:
        offset = 0
        for field in self.fields:
            if field.name == name:
                return offset
            offset += field.count
        raise PCDDataError("PCD is missing required field %r" % name)

    def xyz_points(self) -> Tuple[Tuple[float, float, float], ...]:
        x_offset = self.field_offset("x")
        y_offset = self.field_offset("y")
        z_offset = self.field_offset("z")
        return tuple(
            (float(row[x_offset]), float(row[y_offset]), float(row[z_offset]))
            for row in self.rows
        )


def _parse_positive_int(values: List[str], key: str, allow_zero: bool = False) -> int:
    if len(values) != 1:
        raise PCDDataError("%s requires exactly one value" % key)
    try:
        value = int(values[0], 10)
    except ValueError as exc:
        raise PCDDataError("%s must be an integer" % key) from exc
    if value < 0 if allow_zero else value <= 0:
        raise PCDDataError("%s is out of range" % key)
    return value


def _read_header(handle) -> Tuple[Dict[str, List[str]], str]:
    header: Dict[str, List[str]] = {}
    bytes_read = 0
    while True:
        line = handle.readline()
        if not line:
            raise PCDDataError("PCD ended before DATA header")
        bytes_read += len(line)
        if bytes_read > 1024 * 1024:
            raise PCDDataError("PCD header exceeds 1 MiB")
        try:
            text = line.decode("ascii").strip()
        except UnicodeDecodeError as exc:
            raise PCDDataError("PCD header must be ASCII") from exc
        if not text or text.startswith("#"):
            continue
        parts = text.split()
        key = parts[0].upper()
        values = parts[1:]
        if key in header:
            raise PCDDataError("duplicate PCD header key %s" % key)
        header[key] = values
        if key == "DATA":
            if len(values) != 1:
                raise PCDDataError("DATA requires exactly one encoding")
            return header, values[0].lower()


def _fields_from_header(header: Dict[str, List[str]]) -> Tuple[PCDField, ...]:
    for key in ("FIELDS", "SIZE", "TYPE"):
        if key not in header:
            raise PCDDataError("PCD is missing %s" % key)
    names = header["FIELDS"]
    sizes = header["SIZE"]
    types = header["TYPE"]
    counts = header.get("COUNT", ["1"] * len(names))
    if not names:
        raise PCDDataError("FIELDS must not be empty")
    if not (len(names) == len(sizes) == len(types) == len(counts)):
        raise PCDDataError("FIELDS, SIZE, TYPE, and COUNT lengths do not match")
    fields = []
    for name, size, field_type, count in zip(names, sizes, types, counts):
        try:
            fields.append(
                PCDField(
                    name=name,
                    size=int(size, 10),
                    type=field_type,
                    count=int(count, 10),
                )
            )
        except ValueError as exc:
            raise PCDDataError("SIZE and COUNT values must be integers") from exc
    return tuple(fields)


def _dimensions(header: Dict[str, List[str]]) -> Tuple[int, int, int]:
    if "POINTS" not in header:
        raise PCDDataError("PCD is missing POINTS")
    points = _parse_positive_int(header["POINTS"], "POINTS", allow_zero=True)
    width = (
        _parse_positive_int(header["WIDTH"], "WIDTH", allow_zero=True)
        if "WIDTH" in header
        else points
    )
    height = (
        _parse_positive_int(header["HEIGHT"], "HEIGHT")
        if "HEIGHT" in header
        else 1
    )
    if width * height != points:
        raise PCDDataError("WIDTH * HEIGHT must equal POINTS")
    return width, height, points


def _ensure_finite(rows: Sequence[Sequence[float]], reject_nonfinite: bool) -> None:
    if not reject_nonfinite:
        return
    for row_index, row in enumerate(rows):
        for column_index, value in enumerate(row):
            if not math.isfinite(float(value)):
                raise PCDDataError(
                    "non-finite value at point %d scalar %d"
                    % (row_index, column_index)
                )


def _parse_ascii(
    payload: bytes,
    fields: Sequence[PCDField],
    points: int,
    reject_nonfinite: bool,
) -> Tuple[Tuple[float, ...], ...]:
    try:
        text = payload.decode("ascii")
    except UnicodeDecodeError as exc:
        raise PCDDataError("ASCII PCD payload is not ASCII") from exc
    scalar_count = sum(field.count for field in fields)
    field_specs: List[Tuple[str, int]] = []
    for field in fields:
        field_specs.extend([(field.type, field.size)] * field.count)
    data_lines = [
        line.strip()
        for line in text.splitlines()
        if line.strip() and not line.lstrip().startswith("#")
    ]
    if len(data_lines) != points:
        raise PCDDataError(
            "ASCII PCD has %d point rows, expected %d" % (len(data_lines), points)
        )
    rows = []
    for row_index, line in enumerate(data_lines):
        tokens = line.split()
        if len(tokens) != scalar_count:
            raise PCDDataError(
                "point row %d has %d values, expected %d"
                % (row_index, len(tokens), scalar_count)
            )
        values = []
        for token, (field_type, field_size) in zip(tokens, field_specs):
            try:
                if field_type == "F":
                    numeric = float(token)
                    scalar_struct = struct.Struct(
                        "<" + _STRUCT_CODES[(field_type, field_size)]
                    )
                    # Honor the declared scalar width. This makes an ASCII F4
                    # document yield exactly the same values as binary F4 and
                    # catches overflow before downstream map calculations.
                    numeric = scalar_struct.unpack(scalar_struct.pack(numeric))[0]
                    values.append(numeric)
                else:
                    numeric = int(token, 10)
                    minimum, maximum = _INTEGER_LIMITS[(field_type, field_size)]
                    if numeric < minimum or numeric > maximum:
                        raise ValueError("integer is out of declared range")
                    values.append(numeric)
            except (ValueError, OverflowError, struct.error) as exc:
                raise PCDDataError(
                    "invalid numeric token at point %d" % row_index
                ) from exc
        rows.append(tuple(values))
    _ensure_finite(rows, reject_nonfinite)
    return tuple(rows)


def _binary_struct(fields: Sequence[PCDField]) -> struct.Struct:
    codes = []
    for field in fields:
        code = _STRUCT_CODES[(field.type, field.size)]
        codes.extend([code] * field.count)
    return struct.Struct("<" + "".join(codes))


def _parse_binary(
    payload: bytes,
    fields: Sequence[PCDField],
    points: int,
    reject_nonfinite: bool,
) -> Tuple[Tuple[float, ...], ...]:
    unpacker = _binary_struct(fields)
    expected = unpacker.size * points
    if len(payload) != expected:
        raise PCDDataError(
            "binary PCD payload is %d bytes, expected %d" % (len(payload), expected)
        )
    rows = tuple(tuple(values) for values in struct.iter_unpack(unpacker.format, payload))
    _ensure_finite(rows, reject_nonfinite)
    return rows


def read_pcd(path: os.PathLike, reject_nonfinite: bool = True) -> PCDCloud:
    source = Path(path)
    try:
        with source.open("rb") as handle:
            header, encoding = _read_header(handle)
            fields = _fields_from_header(header)
            width, height, points = _dimensions(header)
            payload = handle.read()
    except OSError as exc:
        raise PCDDataError("cannot read PCD %s: %s" % (source, exc)) from exc
    if encoding == "ascii":
        rows = _parse_ascii(payload, fields, points, reject_nonfinite)
    elif encoding == "binary":
        rows = _parse_binary(payload, fields, points, reject_nonfinite)
    elif encoding == "binary_compressed":
        raise PCDDataError("binary_compressed PCD is not supported")
    else:
        raise PCDDataError("unsupported PCD DATA encoding %r" % encoding)
    viewpoint = " ".join(header.get("VIEWPOINT", ["0", "0", "0", "1", "0", "0", "0"]))
    return PCDCloud(
        fields=fields,
        rows=rows,
        width=width,
        height=height,
        viewpoint=viewpoint,
        data_encoding=encoding,
    )


def _header(cloud: PCDCloud, encoding: str) -> bytes:
    return (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS %s\n"
        "SIZE %s\n"
        "TYPE %s\n"
        "COUNT %s\n"
        "WIDTH %d\n"
        "HEIGHT %d\n"
        "VIEWPOINT %s\n"
        "POINTS %d\n"
        "DATA %s\n"
        % (
            " ".join(field.name for field in cloud.fields),
            " ".join(str(field.size) for field in cloud.fields),
            " ".join(field.type for field in cloud.fields),
            " ".join(str(field.count) for field in cloud.fields),
            cloud.width,
            cloud.height,
            cloud.viewpoint,
            cloud.point_count,
            encoding,
        )
    ).encode("ascii")


def _atomic_bytes(path: Path, content: bytes) -> None:
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


def write_pcd_ascii(cloud: PCDCloud, path: os.PathLike) -> None:
    _ensure_finite(cloud.rows, True)
    lines = []
    field_specs: List[Tuple[str, int]] = []
    for field in cloud.fields:
        field_specs.extend([(field.type, field.size)] * field.count)
    for row in cloud.rows:
        tokens = []
        for value, (field_type, field_size) in zip(row, field_specs):
            if field_type == "F":
                try:
                    struct.pack(
                        "<" + _STRUCT_CODES[(field_type, field_size)], float(value)
                    )
                except (OverflowError, struct.error) as exc:
                    raise PCDDataError(
                        "floating PCD value exceeds declared field range"
                    ) from exc
                tokens.append("%.9g" % float(value))
            else:
                integer = int(value)
                if float(integer) != float(value):
                    raise PCDDataError("integer PCD field contains a fractional value")
                minimum, maximum = _INTEGER_LIMITS[(field_type, field_size)]
                if integer < minimum or integer > maximum:
                    raise PCDDataError(
                        "integer PCD value exceeds declared field range"
                    )
                tokens.append(str(integer))
        lines.append(" ".join(tokens))
    payload = ("\n".join(lines) + ("\n" if lines else "")).encode("ascii")
    _atomic_bytes(Path(path), _header(cloud, "ascii") + payload)


def write_pcd_binary(cloud: PCDCloud, path: os.PathLike) -> None:
    _ensure_finite(cloud.rows, True)
    packer = _binary_struct(cloud.fields)
    payload = bytearray()
    field_specs: List[Tuple[str, int]] = []
    for field in cloud.fields:
        field_specs.extend([(field.type, field.size)] * field.count)
    for row in cloud.rows:
        values = []
        for value, (field_type, field_size) in zip(row, field_specs):
            if field_type == "F":
                values.append(float(value))
            else:
                integer = int(value)
                if float(integer) != float(value):
                    raise PCDDataError(
                        "integer PCD field contains a fractional value"
                    )
                minimum, maximum = _INTEGER_LIMITS[(field_type, field_size)]
                if integer < minimum or integer > maximum:
                    raise PCDDataError(
                        "integer PCD value exceeds declared field range"
                    )
                values.append(integer)
        try:
            payload.extend(packer.pack(*values))
        except (struct.error, OverflowError) as exc:
            raise PCDDataError("PCD value cannot be represented by declared field") from exc
    _atomic_bytes(Path(path), _header(cloud, "binary") + bytes(payload))

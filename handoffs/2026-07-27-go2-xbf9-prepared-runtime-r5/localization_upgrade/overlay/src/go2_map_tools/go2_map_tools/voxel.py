"""Deterministic centroid voxel downsampling."""

from collections import defaultdict
import math
from typing import DefaultDict, Iterable, List, Sequence, Tuple

from .pcd import PCDCloud, PCDDataError


def _voxel_size(value: float) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError("voxel_size_m must be numeric") from exc
    if not math.isfinite(result) or result <= 0.0:
        raise ValueError("voxel_size_m must be finite and positive")
    return result


def voxel_downsample(
    points: Iterable[Sequence[float]], voxel_size_m: float
) -> Tuple[Tuple[float, ...], ...]:
    """Replace all points in each 3-D voxel with their numeric centroid.

    Every row must have the same width and at least ``x, y, z``.  Voxel keys
    are sorted so repeated builds yield byte-identical PCD tile files.
    """

    size = _voxel_size(voxel_size_m)
    sums: DefaultDict[Tuple[int, int, int], List[float]] = defaultdict(list)
    counts: DefaultDict[Tuple[int, int, int], int] = defaultdict(int)
    width = None
    for row_index, row_value in enumerate(points):
        row = tuple(float(value) for value in row_value)
        if width is None:
            width = len(row)
            if width < 3:
                raise PCDDataError("point rows must contain at least x, y, z")
        elif len(row) != width:
            raise PCDDataError("point rows have inconsistent widths")
        if not all(math.isfinite(value) for value in row):
            raise PCDDataError("non-finite point at row %d" % row_index)
        key = (
            math.floor(row[0] / size),
            math.floor(row[1] / size),
            math.floor(row[2] / size),
        )
        if not sums[key]:
            sums[key] = [0.0] * len(row)
        for index, value in enumerate(row):
            sums[key][index] += value
        counts[key] += 1
    if width is None:
        return ()
    return tuple(
        tuple(value / counts[key] for value in sums[key])
        for key in sorted(sums.keys())
    )


def voxel_downsample_cloud(cloud: PCDCloud, voxel_size_m: float) -> PCDCloud:
    rows = voxel_downsample(cloud.rows, voxel_size_m)
    return PCDCloud(
        fields=cloud.fields,
        rows=rows,
        width=len(rows),
        height=1,
        viewpoint=cloud.viewpoint,
        data_encoding="memory",
    )

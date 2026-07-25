#!/usr/bin/env python3
"""Build the exact xbf9 route variants requested for desktop handoff."""

from __future__ import annotations

import csv
import hashlib
import json
import math
import shutil
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parent
PATROL_ROOT = ROOT.parent / "xunjian_20260725_xbf9_patrol_17"
CANONICAL = PATROL_ROOT / "route_horizontal.csv"
TRACE = PATROL_ROOT / "follower_control_trace.jsonl"
PATROL_ANCHOR = PATROL_ROOT / "manual_anchor.json"
PCD_METADATA = ROOT / "xbf.leveling.json"
PCD = ROOT / "xbf.horizontal.downloaded.pcd"

DESKTOP = Path("/Users/constantine/Desktop")
CLEAN_OUTPUT = DESKTOP / "xbf9_horizontal_clean.csv"
PCD_OUTPUT = DESKTOP / "xbf9_aligned_to_xbf_pcd_20260725.csv"
PCD_OUTPUT_METADATA = DESKTOP / "xbf9_aligned_to_xbf_pcd_20260725.json"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def quaternion_rotation_matrix(q_xyzw) -> np.ndarray:
    x, y, z, w = [float(value) for value in q_xyzw]
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.asarray(
        [
            [
                1.0 - 2.0 * (y * y + z * z),
                2.0 * (x * y - z * w),
                2.0 * (x * z + y * w),
            ],
            [
                2.0 * (x * y + z * w),
                1.0 - 2.0 * (x * x + z * z),
                2.0 * (y * z - x * w),
            ],
            [
                2.0 * (x * z - y * w),
                2.0 * (y * z + x * w),
                1.0 - 2.0 * (x * x + y * y),
            ],
        ],
        dtype=np.float64,
    )


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def main() -> None:
    frame_ready = None
    route_anchor = None
    with TRACE.open(encoding="utf-8") as handle:
        for line in handle:
            row = json.loads(line)
            if row.get("kind") == "horizontal_frame_ready":
                frame_ready = row
            elif row.get("kind") == "horizontal_route_anchored":
                route_anchor = row
            if frame_ready is not None and route_anchor is not None:
                break
    if frame_ready is None or route_anchor is None:
        raise RuntimeError("patrol frame or route anchor evidence is missing")

    pcd_metadata = json.loads(PCD_METADATA.read_text(encoding="utf-8"))
    patrol_session = json.loads(PATROL_ANCHOR.read_text(encoding="utf-8"))[
        "localization_session"
    ]
    pcd_session = pcd_metadata["localization_session"]["localization_session"]
    if patrol_session != pcd_session:
        raise RuntimeError(
            "patrol and PCD are not from the same FAST-LIO process identity"
        )

    patrol_q = frame_ready["calibration"]["q_ground_from_map"]
    pcd_q = pcd_metadata["level_frame"]["q_ground_from_map"]
    patrol_rotation = quaternion_rotation_matrix(patrol_q)
    pcd_rotation = quaternion_rotation_matrix(pcd_q)
    pcd_from_patrol = pcd_rotation @ patrol_rotation.T

    rows = []
    with CANONICAL.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            rows.append(
                {
                    "id": int(row["id"]),
                    "x": float(row["x"]),
                    "y": float(row["y"]),
                    "yaw": float(row["yaw"]),
                    "v": float(row["v"]),
                }
            )

    canonical_start = np.asarray(
        [
            float(route_anchor["canonical_start"]["x"]),
            float(route_anchor["canonical_start"]["y"]),
        ]
    )
    anchor_xy = np.asarray(
        [float(route_anchor["anchor_x"]), float(route_anchor["anchor_y"])]
    )
    route_angle = math.radians(float(route_anchor["route_rotation_deg"]))
    route_rotation = np.asarray(
        [
            [math.cos(route_angle), -math.sin(route_angle)],
            [math.sin(route_angle), math.cos(route_angle)],
        ]
    )

    patrol_xy = (
        np.asarray([[row["x"], row["y"]] for row in rows]) - canonical_start
    ) @ route_rotation.T + anchor_xy
    patrol_yaw = np.asarray(
        [wrap_angle(row["yaw"] + route_angle) for row in rows]
    )

    patrol_xyz = np.c_[patrol_xy, np.zeros(len(patrol_xy))]
    pcd_xyz = patrol_xyz @ pcd_from_patrol.T
    pcd_yaw = []
    for yaw in patrol_yaw:
        direction = pcd_from_patrol @ np.asarray(
            [math.cos(yaw), math.sin(yaw), 0.0]
        )
        pcd_yaw.append(math.atan2(direction[1], direction[0]))
    pcd_yaw = np.asarray(pcd_yaw)

    shutil.copyfile(CANONICAL, CLEAN_OUTPUT)
    with PCD_OUTPUT.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle, lineterminator="\n")
        writer.writerow(["id", "x", "y", "yaw", "v"])
        for index, row in enumerate(rows):
            writer.writerow(
                [
                    row["id"],
                    f"{pcd_xyz[index, 0]:.9f}",
                    f"{pcd_xyz[index, 1]:.9f}",
                    f"{pcd_yaw[index]:.9f}",
                    f"{row['v']:.3f}",
                ]
            )

    relative_angle = math.degrees(
        math.acos(
            float(
                np.clip(
                    (np.trace(pcd_from_patrol) - 1.0) / 2.0,
                    -1.0,
                    1.0,
                )
            )
        )
    )
    metadata = {
        "schema": "go2.route_aligned_to_horizontal_pcd.v1",
        "purpose": (
            "Visualization/registration with the specific 2026-07-25 xbf "
            "horizontal PCD; do not use as the canonical route for a future "
            "FAST-LIO session."
        ),
        "canonical_horizontal_route": {
            "path": str(CANONICAL),
            "sha256": sha256(CANONICAL),
            "desktop_copy": str(CLEAN_OUTPUT),
            "desktop_copy_sha256": sha256(CLEAN_OUTPUT),
        },
        "pcd": {
            "path": str(PCD),
            "sha256": sha256(PCD),
        },
        "shared_localization_session": patrol_session,
        "runtime_route_anchor": route_anchor,
        "patrol_q_ground_from_map_xyzw": patrol_q,
        "pcd_q_ground_from_map_xyzw": pcd_q,
        "pcd_from_patrol_rotation_matrix": pcd_from_patrol.tolist(),
        "patrol_to_pcd_frame_rotation_angle_deg": relative_angle,
        "maximum_xy_adjustment_from_patrol_aligned_route_m": float(
            np.max(np.linalg.norm(pcd_xyz[:, :2] - patrol_xy, axis=1))
        ),
        "maximum_yaw_adjustment_deg": float(
            np.max(
                np.abs(
                    np.degrees(
                        np.arctan2(
                            np.sin(pcd_yaw - patrol_yaw),
                            np.cos(pcd_yaw - patrol_yaw),
                        )
                    )
                )
            )
        ),
        "output": {
            "path": str(PCD_OUTPUT),
            "sha256": sha256(PCD_OUTPUT),
            "rows": len(rows),
            "columns": ["id", "x", "y", "yaw", "v"],
            "z_policy": (
                "This remains a 2D patrol CSV. For 3D visualization, its XY "
                "is in the PCD frame but a viewer must choose a display Z."
            ),
        },
        "transform_contract": (
            "canonical route -> recorded patrol start anchor in patrol "
            "horizontal frame -> small same-session patrol-to-PCD gravity "
            "frame rotation"
        ),
    }
    PCD_OUTPUT_METADATA.write_text(
        json.dumps(metadata, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(metadata, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Replay the body-yaw adapter against the captured xbf8 patrol evidence.

This is a counterfactual controller-input replay, not a simulated trajectory:
it asks what heading error the unchanged Go2_2 controller would have seen at
each recorded pose if body IMU yaw had replaced FAST-LIO Euler yaw.
"""

from __future__ import annotations

import bisect
import csv
import json
import math
import sys
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parent
REPO = Path(__file__).resolve().parents[2]
SCRIPTS = REPO / "orin_go2_fastlio_ws" / "scripts"
sys.path.insert(0, str(SCRIPTS))

from body_yaw_alignment import BodyYawAlignment, normalize_angle  # noqa: E402


def read_jsonl(path: Path) -> list[dict[str, Any]]:
    with path.open(encoding="utf-8") as handle:
        return [json.loads(line) for line in handle if line.strip()]


def percentile(values: list[float], percentage: float) -> float | None:
    finite = sorted(value for value in values if math.isfinite(value))
    if not finite:
        return None
    rank = (len(finite) - 1) * percentage / 100.0
    left = int(math.floor(rank))
    right = int(math.ceil(rank))
    if left == right:
        return finite[left]
    fraction = rank - left
    return finite[left] * (1.0 - fraction) + finite[right] * fraction


def summary(values: list[float], degrees: bool = False) -> dict[str, Any]:
    scale = 180.0 / math.pi if degrees else 1.0
    return {
        "count": len(values),
        "p05": percentile(values, 5.0) * scale if values else None,
        "p50": percentile(values, 50.0) * scale if values else None,
        "p95": percentile(values, 95.0) * scale if values else None,
        "max": percentile(values, 100.0) * scale if values else None,
    }


def unique_odom_controls(
    rows: list[dict[str, Any]],
) -> list[dict[str, Any]]:
    result: list[dict[str, Any]] = []
    seen: set[int] = set()
    for row in rows:
        if (
            row.get("kind") != "control"
            or row.get("pose", {}).get("yaw") is None
        ):
            continue
        sequence = int(row["odom_callback_sequence"])
        if sequence in seen:
            continue
        seen.add(sequence)
        result.append(row)
    return result


def route_cumulative(path: Path) -> list[float]:
    with path.open(newline="", encoding="utf-8-sig") as handle:
        points = [
            (float(row["x"]), float(row["y"]))
            for row in csv.DictReader(handle)
        ]
    cumulative = [0.0]
    for left, right in zip(points, points[1:]):
        cumulative.append(
            cumulative[-1] + math.hypot(
                right[0] - left[0],
                right[1] - left[1],
            )
        )
    return cumulative


def main() -> int:
    controls = unique_odom_controls(
        read_jsonl(ROOT / "patrol" / "follower_control_trace.jsonl")
    )
    sport = [
        row
        for row in read_jsonl(
            ROOT / "patrol" / "experiment_telemetry.jsonl"
        )
        if row.get("kind") == "sport"
    ]
    sport_times = [float(row["wall_time"]) for row in sport]
    cumulative = route_cumulative(
        ROOT / "patrol" / "route_runtime.csv"
    )

    def latest_body_yaw(
        wall_time: float,
    ) -> tuple[float | None, float | None]:
        index = bisect.bisect_right(sport_times, wall_time) - 1
        if index < 0:
            return None, None
        age = wall_time - sport_times[index]
        try:
            yaw = float(sport[index]["data"]["imu"]["rpy"][2])
        except (IndexError, KeyError, TypeError, ValueError):
            return None, age
        return yaw, age

    alignment = BodyYawAlignment(
        minimum_samples=10,
        max_spread_rad=math.radians(2.0),
    )
    startup_pair_ages: list[float] = []
    lock_wall_time = None
    for row in controls:
        body_yaw, age = latest_body_yaw(float(row["wall_time"]))
        if (
            body_yaw is None
            or age is None
            or age < 0.0
            or age > 0.25
        ):
            continue
        startup_pair_ages.append(age)
        if alignment.add_pair(float(row["pose"]["yaw"]), body_yaw):
            lock_wall_time = float(row["wall_time"])
            break
    if not alignment.ready:
        raise RuntimeError("captured data did not satisfy startup lock")

    replay: list[dict[str, float | int]] = []
    for row in controls:
        if (
            not row["control"].get("motion_enabled")
            or not row.get("projection")
            or row.get("target", {}).get("angle") is None
        ):
            continue
        body_yaw, age = latest_body_yaw(float(row["wall_time"]))
        if (
            body_yaw is None
            or age is None
            or age < 0.0
            or age > 0.25
        ):
            continue
        aligned_yaw = alignment.aligned_yaw(body_yaw)
        if aligned_yaw is None:
            continue
        segment = int(row["projection"]["segment"])
        fraction = float(row["projection"]["fraction"])
        progress = cumulative[segment] + fraction * (
            cumulative[segment + 1] - cumulative[segment]
        )
        target_angle = float(row["target"]["angle"])
        raw_alpha = float(row["control"]["selected_alpha"])
        body_alpha = normalize_angle(target_angle - aligned_yaw)
        replay.append(
            {
                "direction": int(row["control"]["direction"]),
                "progress_m": progress,
                "body_sample_age_s": age,
                "raw_alpha_rad": raw_alpha,
                "body_alpha_rad": body_alpha,
                "heading_source_delta_rad": normalize_angle(
                    aligned_yaw - float(row["pose"]["yaw"])
                ),
                "raw_yaw_rate_rps": float(
                    row["control"]["cmd_yaw_rate"]
                ),
                "body_yaw_rate_rps": max(
                    -0.45,
                    min(0.45, 0.9 * body_alpha),
                ),
            }
        )

    def section(
        name: str,
        direction: int,
        start_m: float,
        end_m: float,
    ) -> dict[str, Any]:
        rows = [
            row
            for row in replay
            if row["direction"] == direction
            and start_m <= row["progress_m"] < end_m
        ]
        return {
            "name": name,
            "direction": direction,
            "progress_m": [start_m, end_m],
            "sample_count": len(rows),
            "raw_lio_alpha_deg": summary(
                [float(row["raw_alpha_rad"]) for row in rows],
                degrees=True,
            ),
            "body_aligned_alpha_deg": summary(
                [float(row["body_alpha_rad"]) for row in rows],
                degrees=True,
            ),
            "absolute_heading_source_delta_deg": summary(
                [
                    abs(float(row["heading_source_delta_rad"]))
                    for row in rows
                ],
                degrees=True,
            ),
            "raw_yaw_rate_rps": summary(
                [float(row["raw_yaw_rate_rps"]) for row in rows]
            ),
            "body_aligned_yaw_rate_rps": summary(
                [float(row["body_yaw_rate_rps"]) for row in rows]
            ),
            "body_sample_age_ms": summary(
                [
                    float(row["body_sample_age_s"]) * 1000.0
                    for row in rows
                ]
            ),
        }

    sections = [
        section("first_forward_before_major_turn", 1, 0.0, 55.0),
        section("first_forward_after_major_turn", 1, 60.0, 140.0),
        section("reverse_after_major_turn", -1, 60.0, 140.0),
    ]
    post_turn = sections[1]
    result = {
        "method": (
            "Counterfactual input replay on captured poses. It proves the "
            "adapter changes the controller's mistaken near-zero heading "
            "error into a corrective error; it does not predict the final "
            "closed-loop trajectory."
        ),
        "adapter_configuration": {
            "minimum_samples": alignment.minimum_samples,
            "max_spread_deg": math.degrees(
                alignment.max_spread_rad
            ),
            "body_max_age_s": 0.25,
            "k_yaw": 0.9,
            "max_yaw_rate_rps": 0.45,
        },
        "startup_lock": {
            "locked": alignment.ready,
            "lock_wall_time": lock_wall_time,
            "offset_deg": math.degrees(alignment.offset_rad),
            "spread_deg": math.degrees(alignment.spread_rad),
            "sample_count": alignment.sample_count,
            "pair_age_ms": summary(
                [age * 1000.0 for age in startup_pair_ages]
            ),
        },
        "sections": sections,
        "decision": {
            "offline_gate_passed": (
                alignment.ready
                and alignment.spread_rad <= math.radians(2.0)
                and post_turn["sample_count"] > 100
                and abs(
                    post_turn["raw_lio_alpha_deg"]["p50"]
                ) < 3.0
                and abs(
                    post_turn["body_aligned_alpha_deg"]["p50"]
                ) > 10.0
            ),
            "interpretation": (
                "After the major turn, the old input told the controller it "
                "was almost aligned, while body yaw reveals a large steering "
                "correction toward the lookahead point."
            ),
        },
    }
    output = ROOT / "body_yaw_adapter_offline_validation.json"
    output.write_text(
        json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True)
        + "\n",
        encoding="utf-8",
    )
    print(json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

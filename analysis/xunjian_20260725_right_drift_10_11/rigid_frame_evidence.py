#!/usr/bin/env python3
"""Quantify the rigid-frame evidence behind the July 25 route bias.

This script deliberately uses only saved telemetry and run snapshots.  It does
not infer a mounting angle from the photograph and it does not modify the robot.
"""

from __future__ import annotations

import json
import math
import re
from pathlib import Path
from typing import Any

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


HERE = Path(__file__).resolve().parent
PROJECT = HERE.parents[1]
FORMAL = PROJECT / "analysis" / "xunjian_20260725_formal_xbf8"
COMPARISON = HERE / "comparison_metrics.json"

RUNS = {
    "recording": {
        "telemetry": FORMAL / "recording" / "experiment_telemetry.jsonl",
        "plane_window": (1784960669.242379, 1784960945.7207854),
        "motion_start": 1784960669.242379,
    },
    "08_raw_lio_yaw": {
        "telemetry": FORMAL / "patrol" / "experiment_telemetry.jsonl",
        "plane_window": (1784961384.956508, 1784961728.8032901),
        "motion_start": 1784961384.956508,
    },
    "10_body_yaw_first": {
        "telemetry": HERE / "xunjian-20260725-10" / "experiment_telemetry.jsonl",
        "plane_window": (1784968744.0242708, 1784969084.7731261),
        "motion_start": 1784968744.0242708,
    },
    "11_body_yaw_second": {
        "telemetry": HERE / "xunjian-20260725-11" / "experiment_telemetry.jsonl",
        "plane_window": (1784969369.6975238, 1784969712.193693),
        "motion_start": 1784969369.6975238,
    },
}


def percentile(values: np.ndarray, points: tuple[float, ...]) -> dict[str, float]:
    if len(values) == 0:
        return {f"p{point:g}": math.nan for point in points}
    results = np.percentile(values, points)
    return {
        f"p{point:g}": float(value) for point, value in zip(points, results)
    }


def quaternion_rotation(orientation: dict[str, float]) -> Rotation:
    return Rotation.from_quat(
        [
            orientation["qx"],
            orientation["qy"],
            orientation["qz"],
            orientation["qw"],
        ]
    )


def load_telemetry(path: Path) -> dict[str, list[dict[str, Any]]]:
    selected: dict[str, list[dict[str, Any]]] = {
        "odom": [],
        "sport": [],
        "livox_imu": [],
    }
    with path.open() as stream:
        for line in stream:
            record = json.loads(line)
            kind = record.get("kind")
            if kind in selected:
                selected[kind].append(record)
    return selected


def odom_arrays(
    records: list[dict[str, Any]], window: tuple[float, float]
) -> tuple[np.ndarray, np.ndarray, Rotation]:
    times: list[float] = []
    positions: list[list[float]] = []
    rotations: list[Rotation] = []
    for record in records:
        wall_time = float(record["wall_time"])
        if not window[0] <= wall_time <= window[1]:
            continue
        data = record["data"]
        times.append(wall_time)
        positions.append([data["position"][axis] for axis in "xyz"])
        rotations.append(quaternion_rotation(data["orientation"]))
    return (
        np.asarray(times),
        np.asarray(positions),
        Rotation.concatenate(rotations),
    )


def sport_arrays(
    records: list[dict[str, Any]], window: tuple[float, float]
) -> tuple[np.ndarray, np.ndarray, np.ndarray, Rotation]:
    times: list[float] = []
    accelerometers: list[list[float]] = []
    gyroscopes: list[list[float]] = []
    rotations: list[Rotation] = []
    for record in records:
        wall_time = float(record["wall_time"])
        if not window[0] <= wall_time <= window[1]:
            continue
        imu = record["data"]["imu"]
        times.append(wall_time)
        accelerometers.append(imu["accelerometer"])
        gyroscopes.append(imu["gyroscope"])
        rotations.append(quaternion_rotation(imu["orientation"]))
    return (
        np.asarray(times),
        np.asarray(accelerometers),
        np.asarray(gyroscopes),
        Rotation.concatenate(rotations),
    )


def livox_arrays(
    records: list[dict[str, Any]], window: tuple[float, float]
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    times: list[float] = []
    accelerometers: list[list[float]] = []
    gyroscopes: list[list[float]] = []
    for record in records:
        wall_time = float(record["wall_time"])
        if not window[0] <= wall_time <= window[1]:
            continue
        data = record["data"]
        times.append(wall_time)
        accelerometers.append(
            [data["linear_acceleration"][axis] for axis in "xyz"]
        )
        gyroscopes.append([data["angular_velocity"][axis] for axis in "xyz"])
    return (
        np.asarray(times),
        np.asarray(accelerometers),
        np.asarray(gyroscopes),
    )


def fit_ground_plane(positions: np.ndarray) -> dict[str, Any]:
    design = np.column_stack([positions[:, 0], positions[:, 1], np.ones(len(positions))])
    coefficients = np.linalg.lstsq(design, positions[:, 2], rcond=None)[0]
    predicted = design @ coefficients
    residual = positions[:, 2] - predicted
    total_variance = np.sum((positions[:, 2] - positions[:, 2].mean()) ** 2)
    residual_variance = np.sum(residual**2)
    slope_norm = float(np.linalg.norm(coefficients[:2]))
    normal = np.asarray([-coefficients[0], -coefficients[1], 1.0])
    normal /= np.linalg.norm(normal)
    return {
        "samples": int(len(positions)),
        "model": "z = a*x + b*y + c",
        "a": float(coefficients[0]),
        "b": float(coefficients[1]),
        "c": float(coefficients[2]),
        "normal_xyz": normal.tolist(),
        "tilt_deg": math.degrees(math.atan(slope_norm)),
        "r_squared": float(1.0 - residual_variance / total_variance),
        "z_range_m": float(np.ptp(positions[:, 2])),
        "absolute_residual_cm": percentile(
            np.abs(residual) * 100.0, (50, 95, 99, 100)
        ),
    }


def normalized_median_acceleration(
    acceleration: np.ndarray, gyro: np.ndarray
) -> tuple[np.ndarray, int]:
    quiet = np.linalg.norm(gyro, axis=1) < 0.04
    if np.count_nonzero(quiet) < 10:
        limit = np.percentile(np.linalg.norm(gyro, axis=1), 20)
        quiet = np.linalg.norm(gyro, axis=1) <= limit
    median = np.median(acceleration[quiet], axis=0)
    return median / np.linalg.norm(median), int(np.count_nonzero(quiet))


def gravity_evidence(
    telemetry: dict[str, list[dict[str, Any]]], motion_start: float
) -> dict[str, Any]:
    static_window = (motion_start - 10.0, motion_start - 1.0)
    _, sport_acceleration, sport_gyro, _ = sport_arrays(
        telemetry["sport"], static_window
    )
    _, livox_acceleration, livox_gyro = livox_arrays(
        telemetry["livox_imu"], static_window
    )
    # Recording started close to the formal gate.  Fall back to its first nine
    # seconds if the requested pre-motion window predates the telemetry file.
    if len(sport_acceleration) < 10 or len(livox_acceleration) < 10:
        first_time = min(
            float(telemetry["sport"][0]["wall_time"]),
            float(telemetry["livox_imu"][0]["wall_time"]),
        )
        static_window = (first_time, first_time + 9.0)
        _, sport_acceleration, sport_gyro, _ = sport_arrays(
            telemetry["sport"], static_window
        )
        _, livox_acceleration, livox_gyro = livox_arrays(
            telemetry["livox_imu"], static_window
        )

    body_gravity, body_samples = normalized_median_acceleration(
        sport_acceleration, sport_gyro
    )
    lidar_gravity, lidar_samples = normalized_median_acceleration(
        livox_acceleration, livox_gyro
    )
    relative_angle = math.degrees(
        math.acos(float(np.clip(np.dot(body_gravity, lidar_gravity), -1.0, 1.0)))
    )
    return {
        "static_window_epoch": list(static_window),
        "body_quiet_samples": body_samples,
        "lidar_quiet_samples": lidar_samples,
        "body_gravity_unit_xyz": body_gravity.tolist(),
        "lidar_gravity_unit_xyz": lidar_gravity.tolist(),
        "body_tilt_from_positive_z_deg": math.degrees(
            math.acos(float(np.clip(body_gravity[2], -1.0, 1.0)))
        ),
        "lidar_tilt_from_positive_z_deg": math.degrees(
            math.acos(float(np.clip(lidar_gravity[2], -1.0, 1.0)))
        ),
        "minimum_relative_tilt_deg": relative_angle,
    }


def pair_orientations(
    odom_time: np.ndarray,
    odom_rotation: Rotation,
    sport_time: np.ndarray,
    sport_rotation: Rotation,
    maximum_gap_s: float = 0.04,
) -> tuple[np.ndarray, Rotation, Rotation, np.ndarray]:
    pairs: list[tuple[int, int, float]] = []
    for odom_index, wall_time in enumerate(odom_time):
        insertion = int(np.searchsorted(sport_time, wall_time))
        candidates = [
            index
            for index in (insertion - 1, insertion)
            if 0 <= index < len(sport_time)
        ]
        sport_index = min(
            candidates, key=lambda index: abs(sport_time[index] - wall_time)
        )
        gap = sport_time[sport_index] - wall_time
        if abs(gap) <= maximum_gap_s:
            pairs.append((odom_index, sport_index, gap))
    return (
        np.asarray([odom_time[odom_index] for odom_index, _, _ in pairs]),
        odom_rotation[[odom_index for odom_index, _, _ in pairs]],
        sport_rotation[[sport_index for _, sport_index, _ in pairs]],
        np.asarray([gap for _, _, gap in pairs]),
    )


def fit_common_rigid_rotation(
    orientation_sets: dict[str, tuple[np.ndarray, Rotation, Rotation, np.ndarray]]
) -> dict[str, Any]:
    decimated: dict[str, tuple[Rotation, Rotation]] = {}
    for name, (_, lidar_rotation, body_rotation, _) in orientation_sets.items():
        decimated[name] = (lidar_rotation[::5], body_rotation[::5])

    sensor_from_body_initial = Rotation.from_euler("xyz", [0.0, -32.0, 0.0], degrees=True)
    world_alignments_initial = [
        (body_rotation * (lidar_rotation * sensor_from_body_initial).inv()).mean()
        for lidar_rotation, body_rotation in decimated.values()
    ]
    initial = np.concatenate(
        [sensor_from_body_initial.as_rotvec()]
        + [rotation.as_rotvec() for rotation in world_alignments_initial]
    )

    def residual(parameters: np.ndarray) -> np.ndarray:
        sensor_from_body = Rotation.from_rotvec(parameters[:3])
        pieces: list[np.ndarray] = []
        for run_index, (lidar_rotation, body_rotation) in enumerate(
            decimated.values()
        ):
            world_alignment = Rotation.from_rotvec(
                parameters[3 + 3 * run_index : 6 + 3 * run_index]
            )
            predicted_body = world_alignment * lidar_rotation * sensor_from_body
            pieces.append(
                (body_rotation.inv() * predicted_body).as_rotvec().reshape(-1)
            )
        return np.concatenate(pieces)

    solution = least_squares(
        residual,
        initial,
        loss="soft_l1",
        f_scale=math.radians(2.0),
        max_nfev=200,
    )
    sensor_from_body = Rotation.from_rotvec(solution.x[:3])
    singular_values = np.linalg.svd(solution.jac, compute_uv=False)

    fitted: dict[str, Any] = {}
    startup_aligned: dict[str, Any] = {}
    for run_index, (name, (_, lidar_rotation, body_rotation, pairing_gap)) in enumerate(
        orientation_sets.items()
    ):
        world_alignment = Rotation.from_rotvec(
            solution.x[3 + 3 * run_index : 6 + 3 * run_index]
        )
        prediction = world_alignment * lidar_rotation * sensor_from_body
        fitted_error = (body_rotation.inv() * prediction).magnitude()

        sample_time = orientation_sets[name][0]
        startup = sample_time <= sample_time[0] + 2.0
        startup_world_alignment = (
            body_rotation[startup]
            * (lidar_rotation[startup] * sensor_from_body).inv()
        ).mean()
        startup_prediction = (
            startup_world_alignment * lidar_rotation * sensor_from_body
        )
        startup_rotation_error = (
            body_rotation.inv() * startup_prediction
        ).magnitude()
        actual_yaw = body_rotation.as_euler("xyz")[:, 2]
        predicted_yaw = startup_prediction.as_euler("xyz")[:, 2]
        yaw_error = np.abs(
            np.arctan2(
                np.sin(predicted_yaw - actual_yaw),
                np.cos(predicted_yaw - actual_yaw),
            )
        )

        fitted[name] = {
            "samples": int(len(fitted_error)),
            "pairing_gap_ms": percentile(
                np.abs(pairing_gap) * 1000.0, (50, 95, 100)
            ),
            "rotation_error_deg": percentile(
                np.degrees(fitted_error), (50, 90, 95, 99, 100)
            ),
        }
        startup_aligned[name] = {
            "startup_samples": int(np.count_nonzero(startup)),
            "rotation_error_deg": percentile(
                np.degrees(startup_rotation_error), (50, 90, 95, 99, 100)
            ),
            "yaw_error_deg": percentile(
                np.degrees(yaw_error), (50, 90, 95, 99, 100)
            ),
        }

    return {
        "model": "R_body_world(t) = R_world_alignment(run) * "
        "R_lio_world_sensor(t) * R_sensor_from_body",
        "common_sensor_from_body_euler_xyz_deg": sensor_from_body.as_euler(
            "xyz", degrees=True
        ).tolist(),
        "common_sensor_from_body_rotation_magnitude_deg": math.degrees(
            sensor_from_body.magnitude()
        ),
        "jacobian_condition_number": float(singular_values[0] / singular_values[-1]),
        "fit_using_per_run_world_alignment": fitted,
        "startup_only_world_alignment": startup_aligned,
        "interpretation": (
            "The pitch component is strongly observable and agrees with the "
            "independent gravity measurement. Roll/yaw and translation must not "
            "be treated as metrology-grade mount parameters without a dedicated "
            "calibration manoeuvre."
        ),
    }


def configuration_evidence() -> dict[str, Any]:
    snapshot_path = HERE / "xunjian-20260725-10" / "system_start.json"
    snapshot = json.loads(snapshot_path.read_text())
    fastlio_text = snapshot["configuration_files"][
        "install/fast_lio/share/fast_lio/config/go2_mid360s.yaml"
    ]["text"]
    livox_text = snapshot["configuration_files"][
        "install/livox_ros_driver2/share/livox_ros_driver2/config/MID360s_config.json"
    ]["text"]
    return {
        "fastlio_extrinsic_estimation_disabled": bool(
            re.search(r"extrinsic_est_en:\s*false", fastlio_text)
        ),
        "fastlio_lidar_to_its_imu_rotation_is_identity": bool(
            re.search(
                r"extrinsic_R:\s*\[\s*1\.,\s*0\.,\s*0\.,"
                r"\s*0\.,\s*1\.,\s*0\.,\s*0\.,\s*0\.,\s*1\.",
                fastlio_text,
            )
        ),
        "livox_driver_external_extrinsic_all_zero": all(
            token in livox_text
            for token in (
                '"roll": 0.0',
                '"pitch": 0.0',
                '"yaw": 0.0',
                '"x": 0',
                '"y": 0',
                '"z": 0',
            )
        ),
        "explicit_lidar_to_dog_body_transform_present": False,
        "note": (
            "The saved parameters describe Mid360 lidar-to-integrated-IMU "
            "calibration. They do not contain the external Mid360-to-Go2-body "
            "mount transform."
        ),
    }


def shortcut_control_evidence() -> dict[str, Any]:
    metrics = json.loads(COMPARISON.read_text())
    output: dict[str, Any] = {}
    for run_name in ("10_body_yaw_first", "11_body_yaw_second"):
        long_section = next(
            section
            for section in metrics["run_metrics"][run_name]["outbound"]["sections"]
            if section["progress_m"] == [60, 140]
        )
        output[run_name] = {
            "used_minus_raw_lio_yaw_median_deg": long_section[
                "used_minus_raw_yaw_deg"
            ]["p50"],
            "route_tangent_minus_raw_lio_yaw_median_deg": long_section[
                "route_tangent_minus_raw_yaw_deg"
            ]["p50"],
            "route_tangent_minus_used_yaw_median_deg": long_section[
                "route_tangent_minus_used_yaw_deg"
            ]["p50"],
            "predicted_signed_cross_track_median_cm": 100.0
            * long_section["predicted_signed_cross_track_m"]["p50"],
            "observed_signed_cross_track_median_cm": 100.0
            * long_section["signed_cross_track_m"]["p50"],
        }
    return output


def main() -> None:
    telemetry_by_run: dict[str, dict[str, list[dict[str, Any]]]] = {}
    plane_results: dict[str, Any] = {}
    gravity_results: dict[str, Any] = {}
    orientation_sets: dict[
        str, tuple[np.ndarray, Rotation, Rotation, np.ndarray]
    ] = {}

    for name, definition in RUNS.items():
        telemetry = load_telemetry(definition["telemetry"])
        telemetry_by_run[name] = telemetry
        odom_time, positions, odom_rotation = odom_arrays(
            telemetry["odom"], definition["plane_window"]
        )
        sport_time, _, _, sport_rotation = sport_arrays(
            telemetry["sport"], definition["plane_window"]
        )
        plane_results[name] = fit_ground_plane(positions)
        gravity_results[name] = gravity_evidence(
            telemetry, definition["motion_start"]
        )
        orientation_sets[name] = pair_orientations(
            odom_time, odom_rotation, sport_time, sport_rotation
        )

    output = {
        "schema": "go2.rigid_frame_evidence.v1",
        "data_only_claims": {
            "flat_floor_lio_plane": plane_results,
            "simultaneous_static_gravity": gravity_results,
            "common_rigid_rotation_model": fit_common_rigid_rotation(
                orientation_sets
            ),
            "current_shortcut_control_closure": shortcut_control_evidence(),
            "saved_configuration": configuration_evidence(),
        },
        "limits": {
            "photograph": (
                "A single oblique photograph has no metric scale, ground-normal "
                "reference, or orthogonal views; it is not used to estimate an "
                "angle or translation."
            ),
            "translation_observability": (
                "The exact Mid360-to-body-center translation is not uniquely "
                "observable from the route logs because the two position sources "
                "have independent origins/drift and the route lacks a dedicated "
                "fixed-center rotation manoeuvre."
            ),
            "external_ground_truth": (
                "Onboard data proves internal frame inconsistency and its control "
                "effect. It cannot alone certify centimetre-level physical floor "
                "truth; that requires an external reference during validation."
            ),
        },
    }
    destination = HERE / "rigid_frame_evidence.json"
    destination.write_text(json.dumps(output, indent=2, ensure_ascii=False) + "\n")
    print(json.dumps(output, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()

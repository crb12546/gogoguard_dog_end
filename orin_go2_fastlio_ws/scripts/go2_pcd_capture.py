#!/usr/bin/env python3
"""Capture FAST-LIO registered clouds into a proven gravity-level PCD.

The registered cloud is expressed in FAST-LIO's session-local map frame.  With
an angled lidar mount that frame is not necessarily gravity-level.  At startup
this recorder pairs the full FAST-LIO orientation with both the lidar and Go2
body IMUs, freezes one map-to-ground rigid rotation, and applies exactly that
rotation to every saved point.

The requested output path is the gravity-level PCD.  An unmodified raw-map PCD
and a JSON proof bundle are written alongside it for audit and recovery.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import signal
import sys
import time
from pathlib import Path

import numpy as np


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from horizontal_frame import (  # noqa: E402
    HorizontalFrameEstimator,
    angle_between,
    quaternion_normalize,
    quaternion_rotate,
)


CALIBRATION_SCHEMA = "go2.horizontal_frame_calibration.v1"
OUTPUT_SCHEMA = "go2.horizontal_pcd.v1"
DEFAULT_CALIBRATION = (
    SCRIPT_DIR.parent / "config" / "horizontal_frame_calibration.json"
)


def sha256_file(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        while True:
            chunk = handle.read(1024 * 1024)
            if not chunk:
                break
            digest.update(chunk)
    return digest.hexdigest()


def load_calibration(path):
    path = Path(path)
    document = json.loads(path.read_text())
    if document.get("schema") != CALIBRATION_SCHEMA:
        raise RuntimeError(
            "PCD_CALIBRATION_SCHEMA_INVALID expected=%s actual=%s"
            % (CALIBRATION_SCHEMA, document.get("schema"))
        )
    calibration = document.get("mount_and_gravity_calibration")
    if not isinstance(calibration, dict):
        raise RuntimeError("PCD_CALIBRATION_MISSING")
    for key in (
        "q_sensor_from_body_xyzw",
        "q_lidar_gravity_correction_xyzw",
    ):
        if quaternion_normalize(calibration.get(key)) is None:
            raise RuntimeError("PCD_CALIBRATION_QUATERNION_INVALID key=%s" % key)
    return document, calibration


def quaternion_rotation_matrix(quaternion):
    quaternion = quaternion_normalize(quaternion)
    if quaternion is None:
        raise ValueError("invalid quaternion")
    columns = [
        quaternion_rotate(quaternion, axis)
        for axis in (
            (1.0, 0.0, 0.0),
            (0.0, 1.0, 0.0),
            (0.0, 0.0, 1.0),
        )
    ]
    return np.asarray(columns, dtype=np.float64).T


def level_points(points, q_ground_from_map):
    points = np.asarray(points, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("points must have shape (N, 3)")
    if not np.isfinite(points).all():
        raise ValueError("points contain non-finite values")
    rotation = quaternion_rotation_matrix(q_ground_from_map)
    return points @ rotation.T


def voxel_downsample(points, voxel):
    if len(points) == 0 or voxel <= 0.0:
        return points
    keys = np.floor(points / float(voxel)).astype(np.int64)
    _, indices = np.unique(keys, axis=0, return_index=True)
    return points[np.sort(indices)]


def pcd_header(point_count):
    return (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z\n"
        "SIZE 4 4 4\n"
        "TYPE F F F\n"
        "COUNT 1 1 1\n"
        "WIDTH %d\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        "POINTS %d\n"
        "DATA ascii\n"
    ) % (point_count, point_count)


def write_pcd_xyz(path, points):
    path = Path(path)
    points = np.asarray(points, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("points must have shape (N, 3)")
    if not len(points) or not np.isfinite(points).all():
        raise ValueError("cannot write an empty or non-finite point cloud")
    path.parent.mkdir(parents=True, exist_ok=True)
    partial = Path(str(path) + ".partial.%d" % os.getpid())
    try:
        with partial.open("w") as handle:
            handle.write(pcd_header(len(points)))
            np.savetxt(handle, points, fmt="%.4f %.4f %.4f")
        os.replace(str(partial), str(path))
    finally:
        try:
            partial.unlink()
        except OSError:
            pass


def write_json(path, document):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    partial = Path(str(path) + ".partial.%d" % os.getpid())
    try:
        partial.write_text(
            json.dumps(
                document,
                indent=2,
                sort_keys=True,
                allow_nan=False,
            )
            + "\n"
        )
        os.replace(str(partial), str(path))
    finally:
        try:
            partial.unlink()
        except OSError:
            pass


def default_companion_paths(output):
    output = Path(output)
    raw_output = output.parent / "raw" / output.name
    metadata_output = output.with_suffix(".leveling.json")
    return raw_output, metadata_output


def point_cloud_stats(points):
    points = np.asarray(points, dtype=np.float64)
    return {
        "point_count": int(len(points)),
        "minimum_xyz": [float(value) for value in np.min(points, axis=0)],
        "maximum_xyz": [float(value) for value in np.max(points, axis=0)],
        "centroid_xyz": [float(value) for value in np.mean(points, axis=0)],
    }


def rigid_transform_proof(points, transformed, rotation):
    points = np.asarray(points, dtype=np.float64)
    transformed = np.asarray(transformed, dtype=np.float64)
    if len(points) != len(transformed):
        raise ValueError("point counts differ")
    sample_count = min(512, len(points))
    indices = np.linspace(
        0, len(points) - 1, num=sample_count, dtype=np.int64
    )
    raw_delta = points[indices] - points[indices[0]]
    leveled_delta = transformed[indices] - transformed[indices[0]]
    distance_error = np.abs(
        np.linalg.norm(raw_delta, axis=1)
        - np.linalg.norm(leveled_delta, axis=1)
    )
    identity_error = rotation.T @ rotation - np.eye(3)
    return {
        "sample_count": int(sample_count),
        "maximum_pair_distance_error_m": float(np.max(distance_error)),
        "rotation_determinant": float(np.linalg.det(rotation)),
        "rotation_orthogonality_max_error": float(
            np.max(np.abs(identity_error))
        ),
    }


def cloud_xyz(message, point_stride=2):
    offsets = {}
    for field in message.fields:
        if field.name in ("x", "y", "z") and field.datatype == 7:
            offsets[field.name] = int(field.offset)
    if len(offsets) != 3:
        return None
    width = int(message.width)
    height = int(message.height)
    point_step = int(message.point_step)
    row_step = int(message.row_step)
    if width <= 0 or height <= 0 or point_step <= 0:
        return None
    raw = bytes(message.data)
    required = (height - 1) * row_step + width * point_step
    if len(raw) < required:
        return None
    endian = ">" if bool(message.is_bigendian) else "<"
    rows = []
    for row in range(height):
        row_offset = row * row_step
        columns = []
        for name in ("x", "y", "z"):
            columns.append(
                np.ndarray(
                    shape=(width,),
                    dtype=endian + "f4",
                    buffer=raw,
                    offset=row_offset + offsets[name],
                    strides=(point_step,),
                )
            )
        rows.append(np.stack(columns, axis=1))
    points = np.concatenate(rows, axis=0).astype(np.float64, copy=False)
    points = points[np.isfinite(points).all(axis=1)]
    return points[:: max(1, int(point_stride))]


def read_json_if_present(path):
    if not path:
        return None
    try:
        return json.loads(Path(path).read_text())
    except (OSError, ValueError):
        return None


def run_capture(args):
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import (
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )
    from sensor_msgs.msg import Imu, PointCloud2
    from unitree_go.msg import SportModeState

    calibration_document, calibration = load_calibration(args.calibration)
    estimator = HorizontalFrameEstimator(
        q_sensor_from_body=calibration["q_sensor_from_body_xyzw"],
        q_lidar_gravity_correction=calibration[
            "q_lidar_gravity_correction_xyzw"
        ],
        minimum_samples=args.level_samples,
        maximum_spread_rad=math.radians(args.max_spread_deg),
        maximum_source_disagreement_rad=math.radians(
            args.max_source_disagreement_deg
        ),
        maximum_gyro_rad_s=args.max_gyro,
    )

    latest_qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT,
    )

    class CaptureNode(Node):
        def __init__(self):
            super().__init__("go2map_capture")
            self.estimator = estimator
            self.lidar_acceleration = None
            self.lidar_gyro = None
            self.lidar_received_at = None
            self.body_acceleration = None
            self.body_gyro = None
            self.body_received_at = None
            self.frames_received = 0
            self.frames_after_lock = 0
            self.frames_sampled = 0
            self.frames_dropped_before_lock = 0
            self.invalid_cloud_frames = 0
            self.chunks = []
            self.total = 0
            self.locked_at_wall_time = None
            self.lidar_subscription = self.create_subscription(
                Imu, args.lidar_imu_topic, self.lidar_callback, latest_qos
            )
            self.body_subscription = self.create_subscription(
                SportModeState,
                args.body_imu_topic,
                self.body_callback,
                latest_qos,
            )
            self.odom_subscription = self.create_subscription(
                Odometry, args.odom_topic, self.odom_callback, latest_qos
            )
            self.cloud_subscription = self.create_subscription(
                PointCloud2,
                args.cloud_topic,
                self.cloud_callback,
                latest_qos,
            )

        def lidar_callback(self, message):
            self.lidar_acceleration = (
                float(message.linear_acceleration.x),
                float(message.linear_acceleration.y),
                float(message.linear_acceleration.z),
            )
            self.lidar_gyro = (
                float(message.angular_velocity.x),
                float(message.angular_velocity.y),
                float(message.angular_velocity.z),
            )
            self.lidar_received_at = time.monotonic()

        def body_callback(self, message):
            try:
                acceleration = tuple(
                    float(value)
                    for value in message.imu_state.accelerometer
                )
                gyro = tuple(
                    float(value) for value in message.imu_state.gyroscope
                )
            except (AttributeError, TypeError, ValueError):
                return
            if len(acceleration) != 3 or len(gyro) != 3:
                return
            self.body_acceleration = acceleration
            self.body_gyro = gyro
            self.body_received_at = time.monotonic()

        def calibration_inputs_fresh(self, now):
            return (
                self.lidar_acceleration is not None
                and self.lidar_gyro is not None
                and self.body_acceleration is not None
                and self.body_gyro is not None
                and self.lidar_received_at is not None
                and self.body_received_at is not None
                and 0.0 <= now - self.lidar_received_at <= args.imu_max_age
                and 0.0 <= now - self.body_received_at <= args.imu_max_age
            )

        def odom_callback(self, message):
            if self.estimator.ready:
                return
            now = time.monotonic()
            if not self.calibration_inputs_fresh(now):
                return
            orientation = message.pose.pose.orientation
            locked = self.estimator.add_sample(
                (
                    float(orientation.x),
                    float(orientation.y),
                    float(orientation.z),
                    float(orientation.w),
                ),
                self.lidar_acceleration,
                self.body_acceleration,
                self.lidar_gyro,
                self.body_gyro,
            )
            if locked:
                self.locked_at_wall_time = time.time()
                diagnostics = self.estimator.diagnostics()
                tilt_deg = math.degrees(
                    angle_between(
                        diagnostics["map_up"], (0.0, 0.0, 1.0)
                    )
                )
                print(
                    "LEVEL_FRAME_READY samples=%d tilt_deg=%.6f "
                    "spread_deg=%.6f source_disagreement_deg=%.6f"
                    % (
                        diagnostics["sample_count"],
                        tilt_deg,
                        diagnostics["spread_deg"],
                        diagnostics["source_disagreement_deg"],
                    ),
                    flush=True,
                )
                for subscription_name in (
                    "lidar_subscription",
                    "body_subscription",
                    "odom_subscription",
                ):
                    subscription = getattr(self, subscription_name)
                    if subscription is not None:
                        self.destroy_subscription(subscription)
                        setattr(self, subscription_name, None)

        def cloud_callback(self, message):
            self.frames_received += 1
            if not self.estimator.ready:
                self.frames_dropped_before_lock += 1
                return
            self.frames_after_lock += 1
            if self.frames_after_lock % args.frame_stride:
                return
            points = cloud_xyz(message, args.point_stride)
            if points is None or not len(points):
                self.invalid_cloud_frames += 1
                return
            self.frames_sampled += 1
            self.chunks.append(points)
            self.total += len(points)

        def compact(self):
            if len(self.chunks) <= 1:
                return
            merged = voxel_downsample(
                np.vstack(self.chunks), args.voxel
            )
            self.chunks = [merged]
            self.total = len(merged)

    output = Path(args.output)
    raw_default, metadata_default = default_companion_paths(output)
    raw_output = Path(args.raw_output) if args.raw_output else raw_default
    metadata_output = (
        Path(args.metadata_output)
        if args.metadata_output
        else metadata_default
    )
    if output == raw_output:
        raise RuntimeError("PCD_RAW_AND_LEVEL_OUTPUT_MUST_DIFFER")

    rclpy.init()
    node = CaptureNode()
    alive = [True]
    signal.signal(signal.SIGINT, lambda *_: alive.__setitem__(0, False))
    signal.signal(signal.SIGTERM, lambda *_: alive.__setitem__(0, False))
    started_at = time.time()
    last_status = 0.0
    last_compact = time.monotonic()
    try:
        while rclpy.ok() and alive[0]:
            rclpy.spin_once(node, timeout_sec=0.10)
            now = time.monotonic()
            if now - last_status >= 2.0:
                last_status = now
                diagnostics = estimator.diagnostics()
                if estimator.ready:
                    tilt_deg = math.degrees(
                        angle_between(
                            diagnostics["map_up"], (0.0, 0.0, 1.0)
                        )
                    )
                    print(
                        "POINTS %d FRAMES %d/%d LEVEL_TILT_DEG %.6f"
                        % (
                            node.total,
                            node.frames_sampled,
                            node.frames_received,
                            tilt_deg,
                        ),
                        flush=True,
                    )
                else:
                    print(
                        "CALIBRATING samples=%d/%d "
                        "dropped_cloud_frames=%d rejected_motion=%d "
                        "rejected_source=%d"
                        % (
                            diagnostics["sample_count"],
                            diagnostics["minimum_samples"],
                            node.frames_dropped_before_lock,
                            diagnostics["rejected_motion_samples"],
                            diagnostics["rejected_source_samples"],
                        ),
                        flush=True,
                    )
            if now - last_compact >= args.compact_interval:
                node.compact()
                last_compact = now
    finally:
        try:
            node.destroy_node()
        finally:
            rclpy.shutdown()

    diagnostics = estimator.diagnostics()
    if not estimator.ready:
        print(
            "LEVEL_FRAME_NOT_READY samples=%d/%d "
            "rejected_motion=%d rejected_source=%d"
            % (
                diagnostics["sample_count"],
                diagnostics["minimum_samples"],
                diagnostics["rejected_motion_samples"],
                diagnostics["rejected_source_samples"],
            ),
            flush=True,
        )
        return 4
    if not node.chunks:
        print("NO_POINTS", flush=True)
        return 5

    raw_points = voxel_downsample(np.vstack(node.chunks), args.voxel)
    q_ground_from_map = diagnostics["q_ground_from_map"]
    rotation = quaternion_rotation_matrix(q_ground_from_map)
    horizontal_points = raw_points @ rotation.T
    map_up_after = quaternion_rotate(
        q_ground_from_map, diagnostics["map_up"]
    )
    residual_tilt_deg = math.degrees(
        angle_between(map_up_after, (0.0, 0.0, 1.0))
    )
    transform_proof = rigid_transform_proof(
        raw_points, horizontal_points, rotation
    )

    write_pcd_xyz(raw_output, raw_points)
    write_pcd_xyz(output, horizontal_points)
    ended_at = time.time()
    metadata = {
        "schema": OUTPUT_SCHEMA,
        "created_at_epoch": ended_at,
        "capture_started_at_epoch": started_at,
        "level_frame_locked_at_epoch": node.locked_at_wall_time,
        "source_topic": args.cloud_topic,
        "frame_contract": {
            "raw": "FAST-LIO session-local map frame",
            "output": "gravity-level session-local ground frame",
            "transform": "p_ground = q_ground_from_map * p_fastlio_map",
            "translation_applied": False,
            "z_percentile_shift_applied": False,
            "yaw_normalization_applied": False,
        },
        "outputs": {
            "horizontal_pcd": str(output),
            "horizontal_pcd_sha256": sha256_file(output),
            "raw_pcd": str(raw_output),
            "raw_pcd_sha256": sha256_file(raw_output),
        },
        "capture": {
            "frames_received": node.frames_received,
            "frames_dropped_before_level_lock": (
                node.frames_dropped_before_lock
            ),
            "frames_sampled": node.frames_sampled,
            "invalid_cloud_frames": node.invalid_cloud_frames,
            "frame_stride": args.frame_stride,
            "point_stride": args.point_stride,
            "voxel_m": args.voxel,
        },
        "calibration": {
            "path": str(args.calibration),
            "sha256": sha256_file(args.calibration),
            "document": calibration_document,
        },
        "level_frame": diagnostics,
        "proof": {
            "map_up_before_xyz": diagnostics["map_up"],
            "map_up_after_xyz": map_up_after,
            "tilt_removed_deg": math.degrees(
                angle_between(
                    diagnostics["map_up"], (0.0, 0.0, 1.0)
                )
            ),
            "residual_gravity_tilt_deg": residual_tilt_deg,
            "rotation_matrix_ground_from_map": rotation.tolist(),
            **transform_proof,
        },
        "raw_stats": point_cloud_stats(raw_points),
        "horizontal_stats": point_cloud_stats(horizontal_points),
        "localization_session": read_json_if_present(
            args.session_metadata
        ),
    }
    write_json(metadata_output, metadata)
    print(
        "SAVED %s %d RAW %s METADATA %s "
        "LEVEL_TILT_DEG %.6f RESIDUAL_TILT_DEG %.9f"
        % (
            output,
            len(horizontal_points),
            raw_output,
            metadata_output,
            metadata["proof"]["tilt_removed_deg"],
            residual_tilt_deg,
        ),
        flush=True,
    )
    return 0


def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument("output", help="gravity-level output PCD")
    parser.add_argument("--raw-output", default="")
    parser.add_argument("--metadata-output", default="")
    parser.add_argument(
        "--calibration",
        default=os.environ.get(
            "GO2_HORIZONTAL_FRAME_CALIBRATION",
            str(DEFAULT_CALIBRATION),
        ),
    )
    parser.add_argument("--session-metadata", default="")
    parser.add_argument("--cloud-topic", default="/cloud_registered")
    parser.add_argument("--odom-topic", default="/Odometry")
    parser.add_argument("--lidar-imu-topic", default="/livox/imu")
    parser.add_argument("--body-imu-topic", default="/lf/sportmodestate")
    parser.add_argument("--voxel", type=float, default=0.08)
    parser.add_argument("--frame-stride", type=int, default=3)
    parser.add_argument("--point-stride", type=int, default=2)
    parser.add_argument("--compact-interval", type=float, default=20.0)
    parser.add_argument("--level-samples", type=int, default=15)
    parser.add_argument("--max-spread-deg", type=float, default=1.5)
    parser.add_argument(
        "--max-source-disagreement-deg", type=float, default=3.0
    )
    parser.add_argument("--max-gyro", type=float, default=0.08)
    parser.add_argument("--imu-max-age", type=float, default=0.20)
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)
    args.frame_stride = max(1, int(args.frame_stride))
    args.point_stride = max(1, int(args.point_stride))
    args.level_samples = max(5, int(args.level_samples))
    args.voxel = max(0.0, float(args.voxel))
    args.compact_interval = max(1.0, float(args.compact_interval))
    args.max_spread_deg = max(0.0, float(args.max_spread_deg))
    args.max_source_disagreement_deg = max(
        0.0, float(args.max_source_disagreement_deg)
    )
    args.max_gyro = max(0.0, float(args.max_gyro))
    args.imu_max_age = max(0.05, float(args.imu_max_age))
    return run_capture(args)


if __name__ == "__main__":
    raise SystemExit(main())

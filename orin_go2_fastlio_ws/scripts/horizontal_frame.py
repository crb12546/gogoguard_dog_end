#!/usr/bin/env python3
"""Pure rigid-frame helpers for gravity-level Go2 route control.

Quaternion values use ROS order ``(x, y, z, w)``.  The runtime estimator
freezes one map-to-ground rotation while the dog is stationary.  It never
blends a body yaw scalar into a lidar-frame position.
"""

from __future__ import annotations

import collections
import math


_EPSILON = 1e-12


def finite_vector(values, size=3):
    try:
        result = tuple(float(value) for value in values)
    except (TypeError, ValueError):
        return None
    if len(result) != size or not all(math.isfinite(value) for value in result):
        return None
    return result


def vector_norm(vector):
    return math.sqrt(sum(float(value) ** 2 for value in vector))


def normalize_vector(vector):
    values = finite_vector(vector)
    if values is None:
        return None
    magnitude = vector_norm(values)
    if magnitude <= _EPSILON:
        return None
    return tuple(value / magnitude for value in values)


def dot(left, right):
    return sum(float(a) * float(b) for a, b in zip(left, right))


def cross(left, right):
    return (
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    )


def add_vectors(*vectors):
    return tuple(sum(vector[index] for vector in vectors) for index in range(3))


def angle_between(left, right):
    left_unit = normalize_vector(left)
    right_unit = normalize_vector(right)
    if left_unit is None or right_unit is None:
        return None
    cosine = max(-1.0, min(1.0, dot(left_unit, right_unit)))
    return math.acos(cosine)


def quaternion_normalize(quaternion):
    values = finite_vector(quaternion, size=4)
    if values is None:
        return None
    magnitude = math.sqrt(sum(value * value for value in values))
    if magnitude <= _EPSILON:
        return None
    normalized = tuple(value / magnitude for value in values)
    # q and -q are the same rotation.  A stable sign makes traces comparable.
    if normalized[3] < 0.0:
        normalized = tuple(-value for value in normalized)
    return normalized


def quaternion_conjugate(quaternion):
    normalized = quaternion_normalize(quaternion)
    if normalized is None:
        return None
    return (-normalized[0], -normalized[1], -normalized[2], normalized[3])


def quaternion_multiply(left, right):
    """Return the composition ``left * right``."""
    left = quaternion_normalize(left)
    right = quaternion_normalize(right)
    if left is None or right is None:
        return None
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return quaternion_normalize(
        (
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
            lw * rw - lx * rx - ly * ry - lz * rz,
        )
    )


def quaternion_rotate(quaternion, vector):
    quaternion = quaternion_normalize(quaternion)
    vector = finite_vector(vector)
    if quaternion is None or vector is None:
        return None
    x, y, z, w = quaternion
    vx, vy, vz = vector
    # Expanded q * [v, 0] * conjugate(q).
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def quaternion_from_two_vectors(source, target):
    """Return the shortest rotation that maps ``source`` to ``target``."""
    source = normalize_vector(source)
    target = normalize_vector(target)
    if source is None or target is None:
        return None
    cosine = max(-1.0, min(1.0, dot(source, target)))
    if cosine >= 1.0 - 1e-10:
        return (0.0, 0.0, 0.0, 1.0)
    if cosine <= -1.0 + 1e-10:
        # Select a deterministic axis orthogonal to source.
        basis = (1.0, 0.0, 0.0)
        if abs(source[0]) > 0.8:
            basis = (0.0, 1.0, 0.0)
        axis = normalize_vector(cross(source, basis))
        return (axis[0], axis[1], axis[2], 0.0)
    axis = cross(source, target)
    return quaternion_normalize((axis[0], axis[1], axis[2], 1.0 + cosine))


def quaternion_yaw(quaternion):
    quaternion = quaternion_normalize(quaternion)
    if quaternion is None:
        return None
    x, y, z, w = quaternion
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle):
    return math.atan2(math.sin(float(angle)), math.cos(float(angle)))


def align_route_to_pose(route, anchor_x, anchor_y, anchor_yaw):
    """Rigidly align route point zero to a measured horizontal body pose."""
    if len(route) < 2:
        raise ValueError("route must contain at least two points")
    start_x = float(route[0]["x"])
    start_y = float(route[0]["y"])
    start_yaw = float(route[0]["yaw"])
    rotation = normalize_angle(float(anchor_yaw) - start_yaw)
    cosine = math.cos(rotation)
    sine = math.sin(rotation)
    aligned = []
    for point in route:
        dx = float(point["x"]) - start_x
        dy = float(point["y"]) - start_y
        transformed = dict(point)
        transformed["x"] = float(anchor_x) + cosine * dx - sine * dy
        transformed["y"] = float(anchor_y) + sine * dx + cosine * dy
        transformed["yaw"] = normalize_angle(float(point["yaw"]) + rotation)
        aligned.append(transformed)
    return aligned, rotation


class HorizontalFrameEstimator:
    """Estimate and freeze one gravity-level map frame before motion.

    ``q_sensor_from_body`` is the fixed mount rotation established from the
    route-recording black box.  ``q_lidar_gravity_correction`` corrects the
    external IMU gravity direction against the fitted flat-floor plane.
    Each accepted sample produces two independent map-up estimates:

      map_from_sensor * corrected_lidar_gravity
      map_from_sensor * sensor_from_body * body_gravity

    They must agree before the frame can lock.
    """

    def __init__(
        self,
        q_sensor_from_body,
        q_lidar_gravity_correction=(0.0, 0.0, 0.0, 1.0),
        minimum_samples=15,
        maximum_spread_rad=math.radians(1.5),
        maximum_source_disagreement_rad=math.radians(3.0),
        maximum_gyro_rad_s=0.08,
    ):
        self.q_sensor_from_body = quaternion_normalize(q_sensor_from_body)
        self.q_lidar_gravity_correction = quaternion_normalize(
            q_lidar_gravity_correction
        )
        if self.q_sensor_from_body is None:
            raise ValueError("invalid q_sensor_from_body")
        if self.q_lidar_gravity_correction is None:
            raise ValueError("invalid q_lidar_gravity_correction")
        self.minimum_samples = max(5, int(minimum_samples))
        self.maximum_spread_rad = max(0.0, float(maximum_spread_rad))
        self.maximum_source_disagreement_rad = max(
            0.0, float(maximum_source_disagreement_rad)
        )
        self.maximum_gyro_rad_s = max(0.0, float(maximum_gyro_rad_s))
        self._map_up_samples = collections.deque(maxlen=self.minimum_samples)
        self.q_ground_from_map = None
        self.map_up = None
        self.spread_rad = None
        self.last_source_disagreement_rad = None
        self.rejected_motion_samples = 0
        self.rejected_source_samples = 0
        self.rejected_invalid_samples = 0

    @property
    def ready(self):
        return self.q_ground_from_map is not None

    @property
    def sample_count(self):
        return len(self._map_up_samples)

    def _quiet(self, gyro):
        if gyro is None:
            return True
        values = finite_vector(gyro)
        return (
            values is not None
            and vector_norm(values) <= self.maximum_gyro_rad_s
        )

    def add_sample(
        self,
        q_map_from_sensor,
        lidar_acceleration,
        body_acceleration,
        lidar_gyro=None,
        body_gyro=None,
    ):
        """Add a paired stationary sample; return True on the first lock."""
        if self.ready:
            return False
        q_map_from_sensor = quaternion_normalize(q_map_from_sensor)
        lidar_up = normalize_vector(lidar_acceleration)
        body_up = normalize_vector(body_acceleration)
        if (
            q_map_from_sensor is None
            or lidar_up is None
            or body_up is None
        ):
            self.rejected_invalid_samples += 1
            return False
        if not self._quiet(lidar_gyro) or not self._quiet(body_gyro):
            self.rejected_motion_samples += 1
            return False

        corrected_lidar_up = quaternion_rotate(
            self.q_lidar_gravity_correction, lidar_up
        )
        sensor_up_from_body = quaternion_rotate(
            self.q_sensor_from_body, body_up
        )
        map_up_from_lidar = normalize_vector(
            quaternion_rotate(q_map_from_sensor, corrected_lidar_up)
        )
        map_up_from_body = normalize_vector(
            quaternion_rotate(q_map_from_sensor, sensor_up_from_body)
        )
        disagreement = angle_between(map_up_from_lidar, map_up_from_body)
        self.last_source_disagreement_rad = disagreement
        if (
            disagreement is None
            or disagreement > self.maximum_source_disagreement_rad
        ):
            self.rejected_source_samples += 1
            return False
        fused = normalize_vector(
            add_vectors(map_up_from_lidar, map_up_from_body)
        )
        if fused is None:
            self.rejected_invalid_samples += 1
            return False
        self._map_up_samples.append(fused)
        if len(self._map_up_samples) < self.minimum_samples:
            return False

        mean_up = normalize_vector(add_vectors(*self._map_up_samples))
        spread = max(
            angle_between(sample, mean_up)
            for sample in self._map_up_samples
        )
        self.spread_rad = spread
        if spread > self.maximum_spread_rad:
            return False

        self.map_up = mean_up
        self.q_ground_from_map = quaternion_from_two_vectors(
            mean_up, (0.0, 0.0, 1.0)
        )
        return True

    def transform_position(self, position):
        if not self.ready:
            return None
        return quaternion_rotate(self.q_ground_from_map, position)

    def transform_body_orientation(self, q_map_from_sensor):
        if not self.ready:
            return None
        q_ground_from_sensor = quaternion_multiply(
            self.q_ground_from_map, q_map_from_sensor
        )
        return quaternion_multiply(
            q_ground_from_sensor, self.q_sensor_from_body
        )

    def diagnostics(self):
        return {
            "ready": self.ready,
            "sample_count": self.sample_count,
            "minimum_samples": self.minimum_samples,
            "map_up": self.map_up,
            "q_ground_from_map": self.q_ground_from_map,
            "spread_deg": (
                math.degrees(self.spread_rad)
                if self.spread_rad is not None
                else None
            ),
            "source_disagreement_deg": (
                math.degrees(self.last_source_disagreement_rad)
                if self.last_source_disagreement_rad is not None
                else None
            ),
            "rejected_motion_samples": self.rejected_motion_samples,
            "rejected_source_samples": self.rejected_source_samples,
            "rejected_invalid_samples": self.rejected_invalid_samples,
        }

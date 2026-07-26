"""Pure checkpoint-localization state and planar frame helpers.

This module intentionally has no ROS imports.  It is the safety-critical,
deterministic part of the candidate integration and can be unit-tested on a
developer laptop without starting a robot or publishing a motion command.
"""

from __future__ import annotations

import csv
import hashlib
import io
import json
import math
import re
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple


def normalize_angle(angle: float) -> float:
    """Wrap an angle to [-pi, pi]."""

    if not math.isfinite(angle):
        raise ValueError("angle must be finite")
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(frozen=True)
class Pose2:
    x: float
    y: float
    yaw: float

    def __post_init__(self) -> None:
        if not all(math.isfinite(value) for value in (self.x, self.y, self.yaw)):
            raise ValueError("pose values must be finite")


@dataclass(frozen=True)
class Transform2:
    """Rigid planar transform from odom coordinates into map coordinates."""

    tx: float
    ty: float
    yaw: float

    def __post_init__(self) -> None:
        if not all(math.isfinite(value) for value in (self.tx, self.ty, self.yaw)):
            raise ValueError("transform values must be finite")

    def apply(self, odom_pose: Pose2) -> Pose2:
        cosine = math.cos(self.yaw)
        sine = math.sin(self.yaw)
        return Pose2(
            x=cosine * odom_pose.x - sine * odom_pose.y + self.tx,
            y=sine * odom_pose.x + cosine * odom_pose.y + self.ty,
            yaw=normalize_angle(odom_pose.yaw + self.yaw),
        )


def derive_map_from_odom(map_pose: Pose2, odom_pose: Pose2) -> Transform2:
    """Return T_map_odom from synchronized T_map_body and T_odom_body."""

    yaw = normalize_angle(map_pose.yaw - odom_pose.yaw)
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    rotated_x = cosine * odom_pose.x - sine * odom_pose.y
    rotated_y = sine * odom_pose.x + cosine * odom_pose.y
    return Transform2(
        tx=map_pose.x - rotated_x,
        ty=map_pose.y - rotated_y,
        yaw=yaw,
    )


def transform_difference(first: Transform2, second: Transform2) -> Tuple[float, float]:
    """Return translation and wrapped-yaw distance between two frozen mappings."""

    return (
        math.hypot(second.tx - first.tx, second.ty - first.ty),
        abs(normalize_angle(second.yaw - first.yaw)),
    )


@dataclass(frozen=True)
class RoutePoint:
    index: int
    point_id: str
    x: float
    y: float
    yaw: float
    speed: float = 0.0
    checkpoint_id: Optional[str] = None
    checkpoint_radius_m: float = 0.0
    checkpoint_search_radius_m: float = 0.0
    checkpoint_stop_timeout_s: int = 0
    checkpoint_required: bool = True

    @property
    def is_checkpoint(self) -> bool:
        return self.checkpoint_id is not None

    @property
    def pose(self) -> Pose2:
        return Pose2(self.x, self.y, self.yaw)


class CheckpointFileError(ValueError):
    """Raised when a checkpoint sidecar does not bind exactly to its route."""


_MAX_SIDECAR_BYTES = 512 * 1024
_MAX_CHECKPOINTS = 5_000
_MAX_WAYPOINT_ID_CHARACTERS = 120
_MAX_SAFE_INTEGER = 9_007_199_254_740_991
_MAX_ROUTE_BYTES = 1_000_000
_MAX_ABS_COORDINATE_M = 1_000_000.0
_MAX_SPEED_MPS = 0.3
_DECIMAL_NUMBER = re.compile(
    r"^[+-]?(?:(?:\d+(?:\.\d*)?)|(?:\.\d+))(?:[eE][+-]?\d+)?$"
)


def _finite_float(raw: object, label: str) -> float:
    try:
        value = float(raw)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{label} must be numeric") from error
    if not math.isfinite(value):
        raise ValueError(f"{label} must be finite")
    return value


def _csv_float(raw: object, label: str) -> float:
    if not isinstance(raw, str) or _DECIMAL_NUMBER.fullmatch(raw) is None:
        raise ValueError(f"{label} must be a decimal number")
    return _finite_float(raw, label)


def _sha256_bytes(content: bytes) -> str:
    return hashlib.sha256(content).hexdigest()


def _strict_json_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise CheckpointFileError(f"duplicate JSON key: {key}")
        result[key] = value
    return result


def _exact_keys(document: object, keys: set, label: str) -> Dict[str, object]:
    if not isinstance(document, dict):
        raise CheckpointFileError(f"{label} must be a JSON object")
    actual = set(document.keys())
    if actual != keys:
        missing = sorted(keys - actual)
        unknown = sorted(actual - keys)
        raise CheckpointFileError(
            f"{label} fields mismatch; missing={missing}, unknown={unknown}"
        )
    return document


def _sha256_field(document: Dict[str, object], key: str, label: str) -> str:
    value = document[key]
    if (
        not isinstance(value, str)
        or len(value) != 64
        or any(character not in "0123456789abcdef" for character in value)
    ):
        raise CheckpointFileError(
            f"{label}.{key} must be a lowercase SHA-256 hex string"
        )
    return value


def _merge_checkpoint_sidecar(
    route: Sequence[RoutePoint],
    *,
    route_bytes: bytes,
    checkpoint_file: str,
    expected_source_csv_sha256: str,
    expected_source_pcd_sha256: str,
    default_checkpoint_radius_m: float,
    default_search_radius_m: float,
) -> List[RoutePoint]:
    expected_hashes = (
        expected_source_csv_sha256,
        expected_source_pcd_sha256,
    )
    if any(
        len(value) != 64
        or any(character not in "0123456789abcdef" for character in value)
        for value in expected_hashes
    ):
        raise CheckpointFileError(
            "checkpoint_file requires exact expected source CSV and PCD SHA-256"
        )
    checkpoint_path = Path(checkpoint_file)
    sidecar_bytes = checkpoint_path.read_bytes()
    if len(sidecar_bytes) > _MAX_SIDECAR_BYTES:
        raise CheckpointFileError("checkpoint sidecar exceeds 512 KiB")
    try:
        document = json.loads(
            sidecar_bytes.decode("utf-8"),
            object_pairs_hook=_strict_json_object,
        )
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise CheckpointFileError(
            f"checkpoint sidecar is not strict UTF-8 JSON: {error}"
        ) from error
    document = _exact_keys(
        document,
        {
            "schema",
            "source_pcd_sha256",
            "source_csv_sha256",
            "route_csv_sha256",
            "route_revision",
            "checkpoints",
        },
        "checkpoint sidecar",
    )
    if document["schema"] != "go2.route_checkpoints/v1":
        raise CheckpointFileError("unsupported checkpoint sidecar schema")
    source_pcd_hash = _sha256_field(
        document, "source_pcd_sha256", "checkpoint sidecar"
    )
    source_csv_hash = _sha256_field(
        document, "source_csv_sha256", "checkpoint sidecar"
    )
    route_csv_hash = _sha256_field(
        document, "route_csv_sha256", "checkpoint sidecar"
    )
    if source_pcd_hash != expected_source_pcd_sha256:
        raise CheckpointFileError(
            "checkpoint source PCD hash does not match reviewed configuration"
        )
    if source_csv_hash != expected_source_csv_sha256:
        raise CheckpointFileError(
            "checkpoint source CSV hash does not match reviewed configuration"
        )
    actual_route_hash = _sha256_bytes(route_bytes)
    if route_csv_hash != actual_route_hash:
        raise CheckpointFileError(
            "checkpoint route_csv_sha256 does not match the exact route_file bytes"
        )

    revision = document["route_revision"]
    if revision is not None and (
        type(revision) is not int
        or revision < 1
        or revision > _MAX_SAFE_INTEGER
    ):
        raise CheckpointFileError(
            "route_revision must be null or a positive safe integer"
        )
    entries = document["checkpoints"]
    if (
        not isinstance(entries, list)
        or not entries
        or len(entries) > _MAX_CHECKPOINTS
    ):
        raise CheckpointFileError(
            "checkpoint sidecar must contain 1..5000 checkpoints"
        )

    route_ids = [point.point_id for point in route]
    if len(route_ids) != len(set(route_ids)):
        raise CheckpointFileError(
            "route waypoint ids must be unique when a checkpoint sidecar is used"
        )
    replacements: Dict[int, RoutePoint] = {}
    seen_ids = set()
    previous_index = -1
    for entry_number, raw_entry in enumerate(entries):
        entry = _exact_keys(
            raw_entry,
            {"waypoint_id", "waypoint_index", "checkpoint"},
            f"checkpoints[{entry_number}]",
        )
        waypoint_id = entry["waypoint_id"]
        waypoint_index = entry["waypoint_index"]
        if (
            not isinstance(waypoint_id, str)
            or not waypoint_id
            or len(waypoint_id) > _MAX_WAYPOINT_ID_CHARACTERS
            or waypoint_id != waypoint_id.strip()
            or any(
                ord(character) < 0x20 or ord(character) == 0x7F
                for character in waypoint_id
            )
        ):
            raise CheckpointFileError(
                f"checkpoints[{entry_number}].waypoint_id is unsafe"
            )
        if type(waypoint_index) is not int:
            raise CheckpointFileError(
                f"checkpoints[{entry_number}].waypoint_index must be an integer"
            )
        if (
            waypoint_index < 0
            or waypoint_index >= _MAX_CHECKPOINTS
            or waypoint_index >= len(route)
        ):
            raise CheckpointFileError(
                f"checkpoints[{entry_number}] waypoint_index is outside route"
            )
        if waypoint_index <= previous_index:
            raise CheckpointFileError(
                "checkpoint entries must have unique, strictly increasing indexes"
            )
        previous_index = waypoint_index
        if waypoint_id in seen_ids:
            raise CheckpointFileError("duplicate checkpoint waypoint_id")
        seen_ids.add(waypoint_id)
        point = route[waypoint_index]
        if point.point_id != waypoint_id:
            raise CheckpointFileError(
                f"checkpoints[{entry_number}] waypoint id/index binding mismatch"
            )
        checkpoint = _exact_keys(
            entry["checkpoint"],
            {"mode", "stop_timeout_s", "required"},
            f"checkpoints[{entry_number}].checkpoint",
        )
        if checkpoint["mode"] != "relocalize":
            raise CheckpointFileError(
                f"checkpoints[{entry_number}] mode must be relocalize"
            )
        stop_timeout = checkpoint["stop_timeout_s"]
        if type(stop_timeout) is not int or not 5 <= stop_timeout <= 600:
            raise CheckpointFileError(
                f"checkpoints[{entry_number}] stop_timeout_s must be 5..600"
            )
        required = checkpoint["required"]
        if type(required) is not bool:
            raise CheckpointFileError(
                f"checkpoints[{entry_number}] required must be boolean"
            )
        replacements[waypoint_index] = RoutePoint(
            index=point.index,
            point_id=point.point_id,
            x=point.x,
            y=point.y,
            yaw=point.yaw,
            speed=point.speed,
            checkpoint_id=f"waypoint-{point.point_id}",
            checkpoint_radius_m=default_checkpoint_radius_m,
            checkpoint_search_radius_m=default_search_radius_m,
            checkpoint_stop_timeout_s=stop_timeout,
            checkpoint_required=required,
        )
    return [
        replacements.get(index, point) for index, point in enumerate(route)
    ]


def load_route(
    path: str,
    *,
    default_checkpoint_radius_m: float,
    default_search_radius_m: float,
    checkpoint_file: str = "",
    expected_source_csv_sha256: str = "",
    expected_source_pcd_sha256: str = "",
) -> List[RoutePoint]:
    """Load an exact legacy five-column CSV and an optional strict sidecar.

    The CSV itself remains byte-compatible with the known-good follower.
    Checkpoints can only come from a hash-bound
    ``go2.route_checkpoints/v1`` JSON sidecar.  Omitting ``checkpoint_file``
    always produces zero checkpoints.
    """

    default_radius = _finite_float(
        default_checkpoint_radius_m, "default_checkpoint_radius_m"
    )
    default_search = _finite_float(
        default_search_radius_m, "default_search_radius_m"
    )
    if default_radius <= 0.0 or default_search <= 0.0:
        raise ValueError("checkpoint radii must be positive")

    route_path = Path(path)
    route_bytes = route_path.read_bytes()
    if not 1 <= len(route_bytes) <= _MAX_ROUTE_BYTES:
        raise ValueError("route CSV must be 1..1000000 bytes")
    if b"\r" in route_bytes.replace(b"\r\n", b""):
        raise ValueError("route CSV contains a bare carriage return")
    points: List[RoutePoint] = []
    point_ids = set()
    try:
        route_text = route_bytes.decode("utf-8-sig")
    except UnicodeDecodeError as error:
        raise ValueError("route CSV must be UTF-8") from error
    reader = csv.DictReader(io.StringIO(route_text, newline=""))
    if reader.fieldnames != ["id", "x", "y", "yaw", "v"]:
        raise ValueError("route CSV header must be exactly id,x,y,yaw,v")
    for row_number, row in enumerate(reader, start=2):
        if None in row:
            raise ValueError(f"route row {row_number} has extra columns")
        point_id = str(row["id"])
        if (
            not point_id
            or len(point_id) > _MAX_WAYPOINT_ID_CHARACTERS
            or point_id != point_id.strip()
            or any(
                ord(character) < 0x20 or ord(character) == 0x7F
                for character in point_id
            )
        ):
            raise ValueError(f"route row {row_number} id is unsafe")
        if point_id in point_ids:
            raise ValueError(f"route row {row_number} id is duplicated")
        point_ids.add(point_id)
        x = _csv_float(row["x"], f"route row {row_number} x")
        y = _csv_float(row["y"], f"route row {row_number} y")
        yaw = _csv_float(row["yaw"], f"route row {row_number} yaw")
        speed = _csv_float(row["v"], f"route row {row_number} v")
        if abs(x) > _MAX_ABS_COORDINATE_M or abs(y) > _MAX_ABS_COORDINATE_M:
            raise ValueError(f"route row {row_number} coordinate is out of range")
        if yaw < -math.pi or yaw > math.pi:
            raise ValueError(f"route row {row_number} yaw is outside [-pi, pi]")
        if speed < 0.0 or speed > _MAX_SPEED_MPS:
            raise ValueError(f"route row {row_number} speed is outside [0, 0.3]")
        if points and points[-1].x == x and points[-1].y == y:
            raise ValueError(
                f"route row {row_number} duplicates the previous coordinate"
            )
        points.append(
            RoutePoint(
                index=len(points),
                point_id=point_id,
                x=x,
                y=y,
                yaw=yaw,
                speed=speed,
            )
        )
    if not 2 <= len(points) <= _MAX_CHECKPOINTS:
        raise ValueError("route CSV must contain 2..5000 points")
    if checkpoint_file:
        points = _merge_checkpoint_sidecar(
            points,
            route_bytes=route_bytes,
            checkpoint_file=checkpoint_file,
            expected_source_csv_sha256=expected_source_csv_sha256,
            expected_source_pcd_sha256=expected_source_pcd_sha256,
            default_checkpoint_radius_m=default_radius,
            default_search_radius_m=default_search,
        )
    return points


class MissedCheckpoint(RuntimeError):
    """Raised when route progress passed a required checkpoint without stopping."""


class CheckpointTracker:
    """Monotonic route-progress observer independent of follower control."""

    def __init__(
        self,
        route: Sequence[RoutePoint],
        *,
        search_window: int = 8,
        trigger_index_slop: int = 3,
        missed_index_margin: int = 5,
    ) -> None:
        if len(route) < 2:
            raise ValueError("route must contain at least two points")
        if search_window < 1 or trigger_index_slop < 0 or missed_index_margin < 1:
            raise ValueError("invalid checkpoint tracker window")
        self.route = list(route)
        self.search_window = int(search_window)
        self.trigger_index_slop = int(trigger_index_slop)
        self.missed_index_margin = int(missed_index_margin)
        self.progress_index = 0
        self._completed = set()

    def start_at(self, index: int) -> None:
        if index < 0 or index >= len(self.route):
            raise ValueError("start index outside route")
        self.progress_index = int(index)
        # The startup calibration replaces a checkpoint at/before the initial
        # route anchor; do not stop twice at index zero.
        for point in self.route:
            if point.is_checkpoint and point.index <= index:
                self._completed.add(point.index)

    def complete(self, index: int) -> None:
        if index < 0 or index >= len(self.route):
            raise ValueError("checkpoint index outside route")
        self._completed.add(int(index))
        self.progress_index = max(self.progress_index, int(index))

    def nearest_global(self, pose: Pose2) -> Tuple[int, float]:
        distances = [
            math.hypot(point.x - pose.x, point.y - pose.y) for point in self.route
        ]
        index = min(range(len(distances)), key=distances.__getitem__)
        return index, distances[index]

    def observe(self, pose: Pose2) -> Optional[RoutePoint]:
        start = max(0, self.progress_index - self.search_window)
        end = min(len(self.route) - 1, self.progress_index + self.search_window)
        nearest = min(
            range(start, end + 1),
            key=lambda index: math.hypot(
                self.route[index].x - pose.x,
                self.route[index].y - pose.y,
            ),
        )
        if nearest >= self.progress_index:
            # Match the original follower's conservative, one-index-at-a-time
            # forward progress rather than jumping at a route crossing.
            self.progress_index = min(nearest, self.progress_index + 1)

        for checkpoint in self.route:
            if not checkpoint.is_checkpoint or checkpoint.index in self._completed:
                continue
            if self.progress_index > checkpoint.index + self.missed_index_margin:
                raise MissedCheckpoint(
                    f"passed required {checkpoint.checkpoint_id} at route index "
                    f"{checkpoint.index}"
                )
            if abs(self.progress_index - checkpoint.index) > self.trigger_index_slop:
                continue
            distance = math.hypot(checkpoint.x - pose.x, checkpoint.y - pose.y)
            if distance <= checkpoint.checkpoint_radius_m:
                return checkpoint
            # Checkpoints are ordered; a later one cannot be valid first.
            break
        return None


class GateState(str, Enum):
    DISABLED = "disabled"
    ACTIVATING = "activating"
    SETTLING = "settling"
    RESETTING = "resetting"
    LOCALIZING = "localizing"
    CAPTURING = "capturing"
    DEACTIVATING = "deactivating"
    RUNNING = "running"
    FAULT_HOLD = "fault_hold"


class GateAction(str, Enum):
    ACTIVATE = "activate"
    RESET = "reset"
    RELOCALIZE = "relocalize"
    CAPTURE_ALIGNMENT = "capture_alignment"
    DEACTIVATE = "deactivate"


@dataclass(frozen=True)
class CalibrationPhase:
    kind: str
    checkpoint_index: Optional[int] = None
    checkpoint_id: Optional[str] = None
    stop_timeout_sec: Optional[float] = None
    required: bool = True


class CheckpointLocalizationGate:
    """Fail-closed, latching state machine around the unchanged CSV follower."""

    def __init__(
        self,
        *,
        enabled: bool = False,
        settle_before_reset_sec: float = 1.5,
        collect_after_reset_sec: float = 1.5,
        relocalize_retry_sec: float = 0.7,
        localization_timeout_sec: float = 30.0,
    ) -> None:
        timings = (
            settle_before_reset_sec,
            collect_after_reset_sec,
            relocalize_retry_sec,
            localization_timeout_sec,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in timings):
            raise ValueError("all gate timings must be finite and positive")
        self.enabled = bool(enabled)
        self.settle_before_reset_sec = float(settle_before_reset_sec)
        self.collect_after_reset_sec = float(collect_after_reset_sec)
        self.relocalize_retry_sec = float(relocalize_retry_sec)
        self.localization_timeout_sec = float(localization_timeout_sec)
        self.state = GateState.DISABLED
        self.phase: Optional[CalibrationPhase] = None
        self.last_completed_phase: Optional[CalibrationPhase] = None
        self.fault_reason = ""
        self._deadline = 0.0
        self._not_before = 0.0
        self._action_in_flight: Optional[GateAction] = None

    @property
    def hold_required(self) -> bool:
        return self.state != GateState.RUNNING

    def begin(self, now: float) -> None:
        if not self.enabled:
            self.state = GateState.DISABLED
            return
        self._begin_phase(CalibrationPhase(kind="startup"), now)

    def checkpoint_reached(
        self,
        *,
        index: int,
        checkpoint_id: str,
        stop_timeout_sec: float,
        required: bool,
        now: float,
    ) -> None:
        if self.state != GateState.RUNNING:
            raise RuntimeError("checkpoint can only start while running")
        if (
            not math.isfinite(stop_timeout_sec)
            or stop_timeout_sec < 5.0
            or stop_timeout_sec > 600.0
        ):
            raise ValueError("checkpoint stop_timeout_sec must be 5..600")
        if type(required) is not bool:
            raise ValueError("checkpoint required must be boolean")
        self._begin_phase(
            CalibrationPhase(
                kind="checkpoint",
                checkpoint_index=int(index),
                checkpoint_id=str(checkpoint_id),
                stop_timeout_sec=float(stop_timeout_sec),
                required=required,
            ),
            now,
        )

    def retry_from_fault(self, now: float) -> bool:
        if self.state != GateState.FAULT_HOLD or self.phase is None:
            return False
        phase = self.phase
        self._begin_phase(phase, now)
        return True

    def _begin_phase(self, phase: CalibrationPhase, now: float) -> None:
        if not math.isfinite(now):
            raise ValueError("now must be finite")
        self.phase = phase
        self.fault_reason = ""
        # Keep the map localizer inactive while physical motion settles.  This
        # prevents moving scans from spending point-cloud CPU and guarantees
        # that activation starts a fresh stationary collection window.
        self.state = GateState.SETTLING
        timeout = (
            phase.stop_timeout_sec
            if phase.stop_timeout_sec is not None
            else self.localization_timeout_sec
        )
        self._deadline = now + timeout
        self._not_before = now + self.settle_before_reset_sec
        self._action_in_flight = None

    def tick(self, now: float) -> None:
        if not math.isfinite(now):
            raise ValueError("now must be finite")
        if self.state in (
            GateState.DISABLED,
            GateState.RUNNING,
            GateState.FAULT_HOLD,
        ):
            return
        if now >= self._deadline:
            phase_name = self.phase.kind if self.phase else "calibration"
            required = self.phase.required if self.phase else True
            self.fail(
                f"{phase_name} localization timeout "
                f"(required={str(required).lower()}); "
                "automatic bypass is unsupported"
            )
            return
        if self.state == GateState.SETTLING and now >= self._not_before:
            self.state = GateState.ACTIVATING

    def take_action(self, now: float) -> Optional[GateAction]:
        self.tick(now)
        if self._action_in_flight is not None or now < self._not_before:
            return None
        action: Optional[GateAction] = None
        if self.state == GateState.ACTIVATING:
            action = GateAction.ACTIVATE
        elif self.state == GateState.RESETTING:
            action = GateAction.RESET
        elif self.state == GateState.LOCALIZING and now >= self._not_before:
            action = GateAction.RELOCALIZE
        elif self.state == GateState.CAPTURING:
            action = GateAction.CAPTURE_ALIGNMENT
        elif self.state == GateState.DEACTIVATING:
            action = GateAction.DEACTIVATE
        if action is not None:
            self._action_in_flight = action
        return action

    def defer_action(
        self, action: GateAction, *, now: float, retry_after_sec: float
    ) -> None:
        """Return an undispatched action to its state without changing safety."""

        if action != self._action_in_flight:
            raise RuntimeError("deferred action is not the in-flight gate action")
        if not math.isfinite(retry_after_sec) or retry_after_sec <= 0.0:
            raise ValueError("retry_after_sec must be finite and positive")
        self._action_in_flight = None
        self._not_before = now + retry_after_sec

    def action_result(
        self,
        action: GateAction,
        *,
        success: bool,
        now: float,
        reason: str = "",
    ) -> None:
        if action != self._action_in_flight:
            raise RuntimeError("result does not match the in-flight gate action")
        self._action_in_flight = None
        if action == GateAction.ACTIVATE:
            if not success:
                self.fail(reason or "localizer activation failed")
                return
            self.state = GateState.RESETTING
            self._not_before = now
        elif action == GateAction.RESET:
            if not success:
                self.fail(reason or "localization reset failed")
                return
            self.state = GateState.LOCALIZING
            self._not_before = now + self.collect_after_reset_sec
        elif action == GateAction.RELOCALIZE:
            # An individual rejection (usually not enough fresh points yet) is
            # retryable only inside this bounded phase.  It never authorizes
            # movement; the independent status/evidence gate does that.
            self.state = GateState.LOCALIZING
            self._not_before = now + self.relocalize_retry_sec
        elif action == GateAction.CAPTURE_ALIGNMENT:
            if not success:
                self.fail(reason or "alignment capture failed")
                return
            self.state = GateState.DEACTIVATING
        elif action == GateAction.DEACTIVATE:
            if not success:
                self.fail(reason or "localizer deactivation failed")
                return
            self.last_completed_phase = self.phase
            self.state = GateState.RUNNING

    def localization_evidence(
        self, *, ready: bool, lost: bool, now: float, reason: str = ""
    ) -> None:
        self.tick(now)
        if self.state != GateState.LOCALIZING:
            return
        if lost:
            self.fail(reason or "localizer reported LOST")
        elif ready and self._action_in_flight is None:
            self.state = GateState.CAPTURING
            self._not_before = now

    def fail(self, reason: str) -> None:
        self.state = GateState.FAULT_HOLD
        self.fault_reason = str(reason or "unspecified calibration fault")
        self._action_in_flight = None


@dataclass(frozen=True)
class LocalizationEvidence:
    state: int
    map_valid: bool
    pose_valid: bool
    safe_to_move: bool
    startup_precision_verified: bool
    global_confirmation_pending: bool
    map_id: str
    map_hash: str
    status_receive_age_sec: float
    corrected_odom_receive_age_sec: float


def evidence_is_ready(
    evidence: LocalizationEvidence,
    *,
    expected_map_id: str,
    expected_map_hash: str,
    maximum_status_age_sec: float,
    maximum_corrected_odom_age_sec: float,
) -> Tuple[bool, str]:
    """Apply all handoff gates; TRACKING has the stable interface value 2."""

    if evidence.state == 4:
        return False, "localizer reported LOST"
    checks: Iterable[Tuple[bool, str]] = (
        (evidence.state == 2, "state is not TRACKING"),
        (evidence.map_valid, "map is invalid"),
        (evidence.pose_valid, "pose is invalid"),
        (evidence.safe_to_move, "safe_to_move is false"),
        (
            evidence.startup_precision_verified,
            "startup repeatability is not verified",
        ),
        (
            not evidence.global_confirmation_pending,
            "global confirmation is still pending",
        ),
        (
            evidence.map_id == expected_map_id,
            "map_id does not match the reviewed route",
        ),
        (
            evidence.map_hash == expected_map_hash,
            "map hash does not match the reviewed route",
        ),
        (
            0.0 <= evidence.status_receive_age_sec <= maximum_status_age_sec,
            "localization status is stale",
        ),
        (
            0.0
            <= evidence.corrected_odom_receive_age_sec
            <= maximum_corrected_odom_age_sec,
            "corrected odometry is stale",
        ),
    )
    for passed, reason in checks:
        if not passed:
            return False, reason
    return True, ""


def validate_anchor_pose(
    pose: Pose2,
    anchor: RoutePoint,
    *,
    maximum_distance_m: float,
    maximum_yaw_error_rad: float,
) -> Tuple[bool, str]:
    distance = math.hypot(pose.x - anchor.x, pose.y - anchor.y)
    yaw_error = abs(normalize_angle(pose.yaw - anchor.yaw))
    if distance > maximum_distance_m:
        return (
            False,
            f"localized pose is {distance:.3f} m from route anchor "
            f"(limit {maximum_distance_m:.3f} m)",
        )
    if yaw_error > maximum_yaw_error_rad:
        return (
            False,
            f"localized yaw differs from route anchor by "
            f"{math.degrees(yaw_error):.3f} deg "
            f"(limit {math.degrees(maximum_yaw_error_rad):.3f} deg)",
        )
    return True, ""

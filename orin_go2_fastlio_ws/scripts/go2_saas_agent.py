#!/usr/bin/env python3
"""Go2 SaaS adapter for GoGoGuard heartbeat, video upload, and commands.

Designed for the Orin ROS2 Foxy environment. The script intentionally uses only
Python standard-library modules so it can run on the robot without extra pip
packages. Secrets are read from the process environment, normally sourced from
`~/.config/go2_saas.env` on the Orin; do not store tokens in this repository.
"""
from __future__ import print_function

import argparse
import csv
import hashlib
import json
import math
import os
import re
import shlex
import ssl
import subprocess
import sys
import time
import urllib.error
import urllib.parse
import urllib.request
import uuid
from pathlib import Path


WS = os.environ.get("GO2_WS", "/home/unitree/go2_fastlio_ws")
ROUTES_DIR = Path(os.environ.get("GO2_ROUTES_DIR", str(Path(WS) / "src/go2_fastlio_patrol/routes")))
PCD_DIR = Path(os.environ.get("GO2_PCD_DIR", str(Path(WS) / "maps/console")))
VIDEO_DIR = Path(os.environ.get("GO2_VIDEO_DIR", str(Path(WS) / "patrol_logs/videos")))
OUTBOX_DIR = Path(os.environ.get("GO2_OUTBOX_DIR", str(Path(WS) / "patrol_logs/outbox")))
PATROL_RUNS_DIR = Path(os.environ.get("GO2_PATROL_RUNS_DIR", str(Path(WS) / "patrol_logs/runs")))
VIDEO_RUN_DIR = Path(os.environ.get("GO2_VIDEO_RUN_DIR", str(Path(WS) / "patrol_logs/run")))
SERVICE_VIDEO_RUN_FILE = VIDEO_RUN_DIR / "video.run"
PATROL_VIDEO_ACTIVE_FILE = VIDEO_RUN_DIR / "patrol_video.active"
CURRENT_PATROL_RUN_FILE = VIDEO_RUN_DIR / "current_patrol_run"
DEFAULT_BACKEND_BASE = "https://39.96.37.187/api/v1"
DEFAULT_ROBOT_ID = "go2-tju-01"
DEFAULT_ASSET_ENDPOINT = "/robot/asset/upload"
DEFAULT_VIDEO_ENDPOINT = "/robot/video/upload"
DEFAULT_PLAN_ENDPOINT = "/devices/plan"
OUTBOX_MAX_BACKOFF = int(os.environ.get("GO2_OUTBOX_MAX_BACKOFF", "300"))
MIN_VALID_EPOCH = int(os.environ.get("GO2_MIN_VALID_EPOCH", "1704067200"))
MIN_VALID_YEAR = int(os.environ.get("GO2_MIN_VALID_YEAR", "2024"))
MEDIA_TIMESTAMP_RE = re.compile(r"(?:^|[_-])(?P<date>\d{8})[_-]\d{6}")

GOTO_COMMANDS = set(["move", "walk", "go", "goto", "navigate"])
START_PATROL_COMMANDS = set(["start_patrol", "patrol_start", "follow_route", "start_route", "auto_patrol", "auto_inspection"])
STOP_PATROL_COMMANDS = set(["stop_patrol", "patrol_stop", "stop_route"])
SAFE_COMMANDS = set(["ping", "noop", "status", "start_base", "stop_base", "camera_start_loop", "camera_stop_loop"])

ROUTE_NAME_RE = re.compile(r"^[A-Za-z0-9_\-]{1,80}(\.csv)?$")
PATROL_ROSBAG_TOPICS = (
    "/Odometry",
    "/livox/imu",
    "/lf/sportmodestate",
    "/patrol_cmd",
    "/cmd_vel",
    "/api/sport/request",
    "/tf",
    "/tf_static",
)
PATROL_ROSBAG_READY_ATTEMPTS = 50
PATROL_ROSBAG_READY_POLL_SECONDS = 0.2
PATROL_GUARD_READY_ATTEMPTS = 50
PATROL_GUARD_READY_POLL_SECONDS = 0.2


def now_local_text():
    return time.strftime("%Y-%m-%d %H:%M:%S")


def now_text():
    return time.strftime("%Y-%m-%dT%H:%M:%S%z")


def wall_time_is_valid(epoch=None):
    return float(time.time() if epoch is None else epoch) >= MIN_VALID_EPOCH


def wait_for_valid_wall_time(timeout=180, sleep_sec=2):
    started = time.monotonic()
    while not wall_time_is_valid():
        elapsed = time.monotonic() - started
        print("WAIT_VALID_TIME wall clock not synced yet: %s" % now_local_text(), flush=True)
        if timeout > 0 and elapsed >= timeout:
            print("WAIT_VALID_TIME_TIMEOUT elapsed=%.1fs" % elapsed, file=sys.stderr, flush=True)
            return False
        time.sleep(sleep_sec)
    return True


def loop_wall_time_ready(label, sleep_sec=5):
    if wall_time_is_valid():
        return True
    print("%s_WAIT_VALID_TIME wall clock invalid: %s" % (label, now_local_text()), flush=True)
    time.sleep(sleep_sec)
    return False


def invalid_media_time_reason(path, stat=None):
    name = Path(path).name
    match = MEDIA_TIMESTAMP_RE.search(name)
    if match:
        try:
            year = int(match.group("date")[:4])
            if year < MIN_VALID_YEAR:
                return "filename_year_%s" % year
        except ValueError:
            pass
    try:
        stat = stat or Path(path).stat()
        if stat.st_mtime < MIN_VALID_EPOCH:
            return "mtime_%s" % int(stat.st_mtime)
    except OSError as exc:
        return "stat_error_%s" % exc
    return ""


def backend_base():
    return os.environ.get("GO2_BACKEND_BASE", DEFAULT_BACKEND_BASE).rstrip("/")


def build_url(path):
    if not path.startswith("/"):
        path = "/" + path
    return backend_base() + path


def robot_id_arg(value=None):
    return value or os.environ.get("GO2_ROBOT_ID", DEFAULT_ROBOT_ID)


def auth_headers():
    token = os.environ.get("GO2_AUTH_TOKEN") or os.environ.get("GOGOGUARD_TOKEN")
    if not token:
        return {}
    header = os.environ.get("GO2_AUTH_HEADER", "Authorization")
    if header.lower() == "authorization" and not token.lower().startswith(("bearer ", "basic ")):
        token = "Bearer " + token
    return {header: token}


def device_token_headers():
    token = os.environ.get("GO2_DEVICE_TOKEN") or os.environ.get("GO2_PLAN_TOKEN")
    return {"X-Device-Token": token} if token else {}


def tls_context(url):
    verify = os.environ.get("GO2_BACKEND_VERIFY_TLS", "0").lower() in ("1", "true", "yes")
    if url.startswith("https://") and not verify:
        return ssl._create_unverified_context()
    return None


def parse_json_text(text):
    try:
        return json.loads(text)
    except Exception:  # noqa: BLE001
        return None


def outbox_dirs():
    return {
        "pending": OUTBOX_DIR / "pending",
        "inflight": OUTBOX_DIR / "inflight",
        "failed": OUTBOX_DIR / "failed",
    }


def ensure_outbox_dirs():
    for path in outbox_dirs().values():
        path.mkdir(parents=True, exist_ok=True)


def safe_job_part(value, limit=80):
    value = re.sub(r"[^A-Za-z0-9_.-]+", "_", str(value or "job"))
    value = value.strip("._-") or "job"
    return value[:limit]


def read_json_file(path):
    try:
        return json.loads(Path(path).read_text(encoding="utf-8"))
    except Exception:
        return None


def outbox_has_dedupe(dedupe_key):
    if not dedupe_key:
        return False
    ensure_outbox_dirs()
    dirs = outbox_dirs()
    for base in (dirs["pending"], dirs["inflight"]):
        for path in base.glob("*.json"):
            data = read_json_file(path)
            if isinstance(data, dict) and data.get("dedupeKey") == dedupe_key:
                return True
    return False


def enqueue_outbox_job(kind, endpoint, payload=None, file_path=None, fields=None, timeout=12, dedupe_key="", reason=""):
    ensure_outbox_dirs()
    dedupe_key = dedupe_key or "%s:%s:%s" % (kind, endpoint, uuid.uuid4().hex)
    if outbox_has_dedupe(dedupe_key):
        print("OUTBOX_DUPLICATE", dedupe_key)
        return 0

    job_id = "%s_%s_%s" % (int(time.time()), safe_job_part(kind), uuid.uuid4().hex[:10])
    job = {
        "schema": 1,
        "id": job_id,
        "kind": kind,
        "endpoint": endpoint,
        "payload": payload,
        "file": str(file_path) if file_path else "",
        "fields": fields or {},
        "timeout": timeout,
        "dedupeKey": dedupe_key,
        "reason": reason,
        "attempts": 0,
        "createdAt": now_text(),
        "createdAtEpoch": time.time(),
        "nextAttemptEpoch": 0,
    }
    pending = outbox_dirs()["pending"]
    tmp = pending / (job_id + ".tmp")
    final = pending / (job_id + ".json")
    tmp.write_text(json.dumps(job, ensure_ascii=False, sort_keys=True, indent=2), encoding="utf-8")
    tmp.replace(final)
    print("OUTBOX_ENQUEUED", kind, dedupe_key, "reason=%s" % reason)
    return 0


def requeue_outbox_job(path, job, error):
    dirs = outbox_dirs()
    attempts = int(job.get("attempts") or 0) + 1
    delay = min(OUTBOX_MAX_BACKOFF, max(5, 5 * (2 ** min(attempts - 1, 6))))
    job["attempts"] = attempts
    job["lastError"] = str(error)[-500:]
    job["lastAttemptAt"] = now_text()
    job["nextAttemptEpoch"] = time.time() + delay
    target = dirs["pending"] / Path(path).name
    Path(path).write_text(json.dumps(job, ensure_ascii=False, sort_keys=True, indent=2), encoding="utf-8")
    Path(path).replace(target)
    print("OUTBOX_RETRY", job.get("id"), "attempts=%s" % attempts, "delay=%ss" % delay, "error=%s" % job["lastError"])


def fail_outbox_job(path, job, error):
    dirs = outbox_dirs()
    job["failedAt"] = now_text()
    job["lastError"] = str(error)[-500:]
    target = dirs["failed"] / Path(path).name
    Path(path).write_text(json.dumps(job, ensure_ascii=False, sort_keys=True, indent=2), encoding="utf-8")
    Path(path).replace(target)
    print("OUTBOX_FAILED", job.get("id"), error)


def recover_stale_inflight(max_age=600):
    ensure_outbox_dirs()
    dirs = outbox_dirs()
    now = time.time()
    for path in dirs["inflight"].glob("*.json"):
        try:
            if now - path.stat().st_mtime >= max_age:
                path.replace(dirs["pending"] / path.name)
        except Exception as exc:  # noqa: BLE001
            print("OUTBOX_RECOVER_ERROR", repr(exc), file=sys.stderr)


def send_outbox_job(job):
    kind = job.get("kind")
    endpoint = job.get("endpoint")
    timeout = float(job.get("timeout") or 12)
    if kind == "json":
        return post_json(endpoint, job.get("payload") or {}, dry_run=False, timeout=timeout)
    if kind == "file":
        file_path = Path(job.get("file") or "")
        if not file_path.exists() or file_path.stat().st_size <= 0:
            return 2
        fields = job.get("fields") or {}
        asset_kind = str(fields.get("kind") or fields.get("assetType") or "").lower()
        if asset_kind in ("video", "image"):
            reason = invalid_media_time_reason(file_path)
            if reason:
                print("OUTBOX_DROP_INVALID_MEDIA_TIME file=%s reason=%s" % (file_path, reason), file=sys.stderr)
                return 2
        return post_multipart(file_path, job.get("fields") or {}, endpoint, dry_run=False, timeout=timeout)
    print("OUTBOX_UNKNOWN_KIND", kind, file=sys.stderr)
    return 2


def drain_outbox_once(max_jobs=20):
    ensure_outbox_dirs()
    recover_stale_inflight()
    dirs = outbox_dirs()
    now = time.time()
    sent = 0
    attempted = 0
    for path in sorted(dirs["pending"].glob("*.json")):
        if attempted >= max_jobs:
            break
        inflight = dirs["inflight"] / path.name
        try:
            path.replace(inflight)
        except Exception:
            continue
        job = read_json_file(inflight)
        if not isinstance(job, dict):
            fail_outbox_job(inflight, {"id": inflight.stem}, "invalid json job")
            continue
        if float(job.get("nextAttemptEpoch") or 0) > now:
            try:
                inflight.replace(path)
            except OSError as exc:
                print("OUTBOX_REQUEUE_NOT_DUE_ERROR", repr(exc), file=sys.stderr)
            continue
        attempted += 1
        rc = send_outbox_job(job)
        if rc == 0:
            print("OUTBOX_SENT", job.get("id"), job.get("kind"), job.get("dedupeKey"))
            try:
                inflight.unlink()
            except OSError:
                pass
            sent += 1
        elif rc == 2:
            fail_outbox_job(inflight, job, "permanent send error")
        else:
            requeue_outbox_job(inflight, job, "send failed rc=%s" % rc)
    print("OUTBOX_DRAIN attempted=%s sent=%s" % (attempted, sent))
    return 0 if attempted == sent else 1


def post_json_capture(path, payload, dry_run=False, timeout=12):
    url = build_url(path)
    body = json.dumps(payload, ensure_ascii=False, sort_keys=True, indent=2)
    if dry_run:
        print("DRY_RUN_POST", url)
        print(body)
        return 0, None, body, payload

    headers = {"Content-Type": "application/json", "Accept": "application/json"}
    headers.update(auth_headers())
    request = urllib.request.Request(url, data=body.encode("utf-8"), headers=headers, method="POST")
    try:
        response = urllib.request.urlopen(request, timeout=timeout, context=tls_context(url))
        text = response.read().decode("utf-8", errors="replace")
        print("POST", url, "status=%s" % response.getcode())
        print(text)
        return (0 if 200 <= response.getcode() < 300 else 1), response.getcode(), text, parse_json_text(text)
    except urllib.error.HTTPError as exc:
        text = exc.read().decode("utf-8", errors="replace")
        print("POST", url, "status=%s" % exc.code)
        print(text)
        return (0 if 200 <= exc.code < 300 else 1), exc.code, text, parse_json_text(text)
    except Exception as exc:  # noqa: BLE001
        print("POST_ERROR", url, repr(exc), file=sys.stderr)
        return 1, None, repr(exc), None


def post_json(path, payload, dry_run=False, timeout=12):
    rc, _, _, _ = post_json_capture(path, payload, dry_run=dry_run, timeout=timeout)
    return rc


def get_json_capture(path, dry_run=False, timeout=12, include_auth=True, include_device_token=True):
    url = build_url(path)
    if dry_run:
        print("DRY_RUN_GET", url)
        return 0, None, "", None

    headers = {"Accept": "application/json"}
    if include_auth:
        headers.update(auth_headers())
    if include_device_token:
        headers.update(device_token_headers())
    request = urllib.request.Request(url, headers=headers, method="GET")
    try:
        response = urllib.request.urlopen(request, timeout=timeout, context=tls_context(url))
        text = response.read().decode("utf-8", errors="replace")
        print("GET", url, "status=%s" % response.getcode())
        print(text)
        return (0 if 200 <= response.getcode() < 300 else 1), response.getcode(), text, parse_json_text(text)
    except urllib.error.HTTPError as exc:
        text = exc.read().decode("utf-8", errors="replace")
        print("GET", url, "status=%s" % exc.code)
        print(text)
        return (0 if 200 <= exc.code < 300 else 1), exc.code, text, parse_json_text(text)
    except Exception as exc:  # noqa: BLE001
        print("GET_ERROR", url, repr(exc), file=sys.stderr)
        return 1, None, repr(exc), None


def endpoint_for(kind):
    env_key = {
        "route": "GO2_ROUTE_UPLOAD_ENDPOINT",
        "pcd": "GO2_PCD_UPLOAD_ENDPOINT",
        "video": "GO2_VIDEO_UPLOAD_ENDPOINT",
        "image": "GO2_VIDEO_UPLOAD_ENDPOINT",
    }.get(kind, "GO2_ASSET_UPLOAD_ENDPOINT")
    default = DEFAULT_VIDEO_ENDPOINT if kind in ("video", "image") else DEFAULT_ASSET_ENDPOINT
    return os.environ.get(env_key) or os.environ.get("GO2_ASSET_UPLOAD_ENDPOINT") or default


def guess_mime(path):
    suffix = path.suffix.lower()
    if suffix == ".csv":
        return "text/csv"
    if suffix == ".pcd":
        return "application/octet-stream"
    if suffix == ".mp4":
        return "video/mp4"
    if suffix in (".jpg", ".jpeg"):
        return "image/jpeg"
    return "application/octet-stream"


def post_multipart(path, fields, endpoint, dry_run=False, timeout=180):
    url = build_url(endpoint)
    if dry_run:
        print("DRY_RUN_UPLOAD", url)
        print(json.dumps({"file": str(path), "fields": fields}, ensure_ascii=False, sort_keys=True, indent=2))
        return 0

    boundary = "----go2saas" + uuid.uuid4().hex
    chunks = []
    for key, value in fields.items():
        chunks.append((
            "--%s\r\n"
            "Content-Disposition: form-data; name=\"%s\"\r\n\r\n%s\r\n"
        ) % (boundary, key, value))
    chunks.append((
        "--%s\r\n"
        "Content-Disposition: form-data; name=\"file\"; filename=\"%s\"\r\n"
        "Content-Type: %s\r\n\r\n"
    ) % (boundary, path.name, guess_mime(path)))

    try:
        body = b"".join(item.encode("utf-8") for item in chunks)
        body += path.read_bytes()
        body += ("\r\n--%s--\r\n" % boundary).encode("utf-8")
        headers = {"Content-Type": "multipart/form-data; boundary=" + boundary, "Accept": "application/json"}
        headers.update(auth_headers())
        request = urllib.request.Request(url, data=body, headers=headers, method="POST")
        response = urllib.request.urlopen(request, timeout=timeout, context=tls_context(url))
        text = response.read().decode("utf-8", errors="replace")
        print("HTTP status=%s" % response.getcode())
        print(text)
        return 0 if 200 <= response.getcode() < 300 else 1
    except urllib.error.HTTPError as exc:
        text = exc.read().decode("utf-8", errors="replace")
        print("HTTP status=%s" % exc.code)
        print(text)
        return 0 if 200 <= exc.code < 300 else 1
    except Exception as exc:  # noqa: BLE001
        print("UPLOAD_ERROR", url, repr(exc), file=sys.stderr)
        return 1


def shell_out(cmd, timeout=3):
    try:
        run = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    except Exception as exc:  # noqa: BLE001
        return "", str(exc), 255
    return run.stdout.strip(), run.stderr.strip(), run.returncode


def process_status():
    out, _, _ = shell_out(["ps", "-eo", "comm=,args="], timeout=4)
    checks = {
        "livox": r"livox_ros_driver2_node|livox_ros_drive",
        "fastlio": r"fastlio_mapping",
        "recorder": r"route_recorder",
        "safe": r"unitree_safe_cmd_node",
        "follower": r"waypoint_follower",
        "pcd": r"go2map_capture",
        "camera_loop": r"z1pro_video_loop\.sh",
    }
    status = {key: bool(re.search(pattern, out)) for key, pattern in checks.items()}
    # The systemd video worker remains alive while idle.  Report recording only
    # when a console loop exists or a patrol has explicitly enabled video.
    status["camera_loop"] = status["camera_loop"] or PATROL_VIDEO_ACTIVE_FILE.exists()
    return status


def patrol_control_running():
    out, _, _ = shell_out(["ps", "-eo", "comm=,args="], timeout=4)
    # The follower defines the patrol lifetime.  Safety/UDP bridge processes can
    # intentionally remain alive after a once-only route has completed.
    return bool(re.search(r"waypoint_follower", out))


def count_csv_points(path):
    try:
        with path.open("r") as handle:
            return max(0, sum(1 for _ in handle) - 1)
    except Exception:  # noqa: BLE001
        return None


def pcd_points(path):
    try:
        with path.open("r") as handle:
            for line in handle:
                if line.startswith("POINTS "):
                    return int(line.split()[1])
    except Exception:  # noqa: BLE001
        return None
    return None


def describe_file(path, kind):
    stat = path.stat()
    item = {
        "kind": kind,
        "path": str(path),
        "name": path.name,
        "size": stat.st_size,
        "mtime": int(stat.st_mtime),
        "mtimeText": time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(stat.st_mtime)),
    }
    if kind == "route":
        item["points"] = count_csv_points(path)
    elif kind == "pcd":
        item["points"] = pcd_points(path)
    return item


def latest_paths(patterns, limit):
    paths = []
    for pattern in patterns:
        paths.extend(pattern)
    existing = [path for path in paths if path.is_file()]
    existing.sort(key=lambda path: path.stat().st_mtime, reverse=True)
    return existing[:limit]


def asset_manifest(robot_id, limit=8):
    routes = latest_paths([ROUTES_DIR.glob("*.csv"), ROUTES_DIR.glob("records/*/route.csv")], limit)
    pcds = latest_paths([PCD_DIR.glob("*.pcd")], limit)
    videos = valid_media_files(limit=limit * 4)[:limit]
    return {
        "robotId": robot_id,
        "generatedAt": now_text(),
        "routes": [describe_file(path, "route") for path in routes],
        "pcds": [describe_file(path, "pcd") for path in pcds],
        "media": [describe_file(path, "video" if path.suffix.lower() == ".mp4" else "image") for path in videos],
    }


def resolve_named_path(value, base_dir, suffix):
    if not value:
        return None
    path = Path(value)
    if path.is_absolute():
        return path
    name = value if value.endswith(suffix) else value + suffix
    return base_dir / name


def resolve_route(value):
    return resolve_named_path(value, ROUTES_DIR, ".csv")


def resolve_pcd(value):
    return resolve_named_path(value, PCD_DIR, ".pcd")


def env_bool(name, default=False):
    value = os.environ.get(name)
    if value in (None, ""):
        return default
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def bool_param(params, keys, default=False):
    for key in keys:
        value = params.get(key)
        if value in (None, ""):
            continue
        if isinstance(value, bool):
            return value
        return str(value).strip().lower() in ("1", "true", "yes", "on")
    return default


def strict_bool(value, name):
    if isinstance(value, bool):
        return value
    normalized = str(value).strip().lower()
    if normalized in ("1", "true", "yes", "on"):
        return True
    if normalized in ("0", "false", "no", "off"):
        return False
    raise ValueError("invalid boolean for %s: %r" % (name, value))


def localization_mode_from_params(params):
    """Choose a startup frame policy without making PCD the default.

    The normal mode is a manual start anchor: the operator places the dog at
    the physical route start, and a temporary route is rigidly transformed
    into the current FAST-LIO session.  PCD/ICP remains an explicit mode.

    Historical ``origin``/``direct`` requests are intentionally treated as
    manual anchors.  Comparing their old absolute CSV coordinates with a new
    FAST-LIO origin after a reboot is not meaningful.
    """
    mode_keys = ("localizationMode", "localization_mode")
    requested_modes = []
    for key in mode_keys:
        value = params.get(key)
        if value in (None, ""):
            continue
        normalized = str(value).strip().lower().replace("-", "_")
        aliases = {
            "manual": "manual_anchor",
            "manual_anchor": "manual_anchor",
            "start_anchor": "manual_anchor",
            "origin": "manual_anchor",
            "direct": "manual_anchor",
            "origin_direct": "manual_anchor",
            "none": "manual_anchor",
            "pcd": "pcd",
            "relocalize": "pcd",
            "relocalization": "pcd",
            "map": "pcd",
        }
        if normalized not in aliases:
            raise ValueError(
                "invalid localization mode %r; use manual_anchor or pcd" % value
            )
        requested_modes.append(aliases[normalized])

    if requested_modes:
        if len(set(requested_modes)) != 1:
            raise ValueError("conflicting localization mode parameters")
        mode = requested_modes[0]
    else:
        legacy_keys = (
            "relocalize",
            "useRelocalization",
            "use_relocalization",
            "mapRelocalize",
            "map_relocalize",
        )
        legacy_values = []
        for key in legacy_keys:
            value = params.get(key)
            if value in (None, ""):
                continue
            legacy_values.append(strict_bool(value, key))
        if legacy_values:
            if len(set(legacy_values)) != 1:
                raise ValueError("conflicting relocalize parameters")
            mode = "pcd" if legacy_values[0] else "manual_anchor"
        else:
            configured = os.environ.get(
                "GO2_PATROL_LOCALIZATION_MODE", "manual_anchor"
            )
            mode = (
                str(configured).strip().lower().replace("-", "_")
                or "manual_anchor"
            )
            if mode in (
                "manual",
                "start_anchor",
                "origin",
                "direct",
                "origin_direct",
                "none",
            ):
                mode = "manual_anchor"
            elif mode in ("relocalize", "relocalization", "map"):
                mode = "pcd"
            if mode not in ("manual_anchor", "pcd"):
                raise ValueError(
                    "invalid GO2_PATROL_LOCALIZATION_MODE=%r; "
                    "use manual_anchor or pcd"
                    % configured
                )

    require_keys = (
        "requireRelocalize",
        "require_relocalize",
        "requireRelocalization",
        "require_relocalization",
    )
    required = False
    for key in require_keys:
        value = params.get(key)
        if value in (None, ""):
            continue
        required = strict_bool(value, key)
        break
    if required and mode != "pcd":
        raise ValueError("requireRelocalize=true requires localizationMode=pcd")
    return mode


def route_map_candidates(route_path):
    stem = Path(route_path).stem
    stems = []

    def add(value):
        value = str(value or "").strip("._-")
        if value and value not in stems:
            stems.append(value)

    add(stem)
    lower = stem.lower()
    for suffix in ("-lx", "_lx", "-route", "_route", "-csv", "_csv", "csv"):
        if lower.endswith(suffix) and len(stem) > len(suffix):
            add(stem[: -len(suffix)])
    add(stem.replace("-lx", "").replace("_lx", ""))
    return stems


def resolve_route_map(route_path, params):
    expected_names = {stem + ".pcd" for stem in route_map_candidates(route_path)}
    explicit = param_first(params, [
        "pcd", "pcdFile", "pcd_file", "map", "mapFile", "map_file", "mapName", "map_name",
    ])
    if explicit:
        path = Path(explicit)
        if path.is_absolute():
            candidate = path
        else:
            name = path.name if path.name.endswith(".pcd") else path.name + ".pcd"
            candidate = PCD_DIR / name
        if not candidate.is_file():
            raise RuntimeError("explicit map pcd not found: %s" % candidate)
        if candidate.name not in expected_names:
            raise RuntimeError(
                "explicit map pcd must match route name (expected one of %s, got %s)"
                % (", ".join(sorted(expected_names)), candidate.name)
            )
        return candidate, "explicit"

    for stem in route_map_candidates(route_path):
        candidate = PCD_DIR / (stem + ".pcd")
        if candidate.is_file():
            return candidate, "stem:%s" % stem
    return None, ""


def route_relocalization_plan(route_path, params):
    mode = localization_mode_from_params(params)
    plan = {
        "mode": mode,
        "enabled": True,
        "manualAnchor": mode == "manual_anchor",
        "pcdRelocalization": mode == "pcd",
        "required": True,
        "usesPcd": mode == "pcd",
        "mapPath": "",
        "mapSource": "",
        "runtimeRoutePath": "",
        "reason": "",
    }
    if mode == "manual_anchor":
        plan["reason"] = (
            "manual_start_anchor: current stationary pose is treated as "
            "route waypoint 0"
        )
        return plan

    map_path, source = resolve_route_map(route_path, params)
    if map_path is None:
        raise RuntimeError(
            "route relocalization requires a same-name pcd map for: %s; "
            "to run without any transform, explicitly send relocalize=false only after verifying both use the same odometry frame"
            % Path(route_path).name
        )

    runtime_name = "%s_%d.csv" % (safe_job_part(Path(route_path).stem, limit=70), int(time.time()))
    runtime_path = Path("/tmp/go2_route_runtime") / runtime_name
    plan.update({
        "enabled": True,
        "mapPath": str(map_path),
        "mapSource": source,
        "runtimeRoutePath": str(runtime_path),
        "reason": "map relocalization before patrol",
    })
    return plan


def valid_media_files(limit=40):
    paths = latest_paths(
        [
            VIDEO_DIR.glob("z1pro*.mp4"),
            VIDEO_DIR.glob("z1pro*.jpg"),
            VIDEO_DIR.glob("z1pro*.jpeg"),
            VIDEO_DIR.glob("unitree_builtin*.mp4"),
            VIDEO_DIR.glob("unitree_builtin*.jpg"),
            VIDEO_DIR.glob("unitree_builtin*.jpeg"),
        ],
        limit,
    )
    valid = []
    for path in paths:
        try:
            stat = path.stat()
        except OSError:
            continue
        if stat.st_size <= 0:
            continue
        reason = invalid_media_time_reason(path, stat=stat)
        if reason:
            print("SKIP_MEDIA_INVALID_TIME file=%s reason=%s" % (path, reason), file=sys.stderr)
            continue
        valid.append(path)
    return valid


def resolve_media(value, cycle_index=0):
    if not value or value == "latest":
        files = valid_media_files(limit=40)
        return files[0] if files else None
    if value == "cycle":
        files = valid_media_files(limit=40)
        if not files:
            return None
        return files[cycle_index % len(files)]
    path = Path(value)
    if path.is_absolute():
        return path
    return VIDEO_DIR / value


def yaw_from_quat(q):
    import math

    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def collect_ros(timeout_sec=1.5):
    data = {"ok": False, "errors": []}
    try:
        import rclpy
        from nav_msgs.msg import Odometry
        from rclpy.node import Node
    except Exception as exc:  # noqa: BLE001
        data["errors"].append("ros_import: " + repr(exc))
        return data

    try:
        from unitree_go.msg import LowState, SportModeState
    except Exception as exc:  # noqa: BLE001
        LowState = None
        SportModeState = None
        data["errors"].append("unitree_go_import: " + repr(exc))

    class Collector(Node):
        def __init__(self):
            super().__init__("go2_saas_agent")
            self.odom = None
            self.low = None
            self.sport = None
            self.create_subscription(Odometry, "/Odometry", self.on_odom, 5)
            if LowState is not None:
                self.create_subscription(LowState, "/lf/lowstate", self.on_low, 5)
            if SportModeState is not None:
                self.create_subscription(SportModeState, "/lf/sportmodestate", self.on_sport, 5)

        def on_odom(self, msg):
            self.odom = msg

        def on_low(self, msg):
            self.low = msg

        def on_sport(self, msg):
            self.sport = msg

    try:
        rclpy.init(args=None)
        node = Collector()
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.odom is not None and (LowState is None or node.low is not None) and (SportModeState is None or node.sport is not None):
                break
        if node.odom is not None:
            pose = node.odom.pose.pose
            data["pose"] = {
                "frame": "fast_lio_map",
                "source": "/Odometry",
                "x": round(float(pose.position.x), 3),
                "y": round(float(pose.position.y), 3),
                "z": round(float(pose.position.z), 3),
                "yaw": round(float(yaw_from_quat(pose.orientation)), 3),
            }
        if node.low is not None:
            low = node.low
            temps = [int(item.temperature) for item in low.motor_state[:12]]
            data["battery"] = {
                "soc": int(low.bms_state.soc),
                "powerV": round(float(low.power_v), 2),
                "powerA": round(float(low.power_a), 2),
                "motorMaxTemp": max(temps) if temps else None,
                "bodyTemp": [int(low.temperature_ntc1), int(low.temperature_ntc2)],
            }
        if node.sport is not None:
            sport = node.sport
            data["sport"] = {
                "errorCode": int(sport.error_code),
                "mode": int(sport.mode),
                "gait": int(sport.gait_type),
                "vx": round(float(sport.velocity[0]), 3),
                "vyaw": round(float(sport.yaw_speed), 3),
            }
        data["ok"] = bool(data.get("pose") or data.get("battery") or data.get("sport"))
        node.destroy_node()
        rclpy.shutdown()
    except Exception as exc:  # noqa: BLE001
        data["errors"].append("ros_collect: " + repr(exc))
        try:
            rclpy.shutdown()
        except Exception:  # noqa: BLE001
            pass
    return data


def current_pose(timeout_sec=0.8):
    return collect_ros(timeout_sec).get("pose")


def network_summary():
    hostname, _, _ = shell_out(["hostname"], timeout=2)
    addrs, _, _ = shell_out(["hostname", "-I"], timeout=2)
    route, _, _ = shell_out(["sh", "-c", "ip route show default 2>/dev/null | head -1"], timeout=2)
    summary = {"hostname": hostname, "addresses": addrs.split(), "defaultRoute": route}
    try:
        state = json.loads(Path("/run/go2-4g-manager-state.json").read_text())
    except Exception:  # noqa: BLE001
        state = None
    if isinstance(state, dict):
        modem = state.get("modem") if isinstance(state.get("modem"), dict) else {}
        link = state.get("usb_link") if isinstance(state.get("usb_link"), dict) else {}
        summary["cellular"] = {
            "state": state.get("state"),
            "reason": state.get("reason"),
            "updatedAt": state.get("updated_at"),
            "interface": state.get("interface"),
            "addresses": state.get("addresses") or [],
            "managerVersion": state.get("manager_version"),
            "csq": modem.get("csq"),
            "registered": modem.get("registered"),
            "usbLink": {
                "device": link.get("device"),
                "speedMbps": link.get("speed_mbps"),
                "expectedHighSpeedMbps": link.get("expected_high_speed_mbps"),
                "degraded": bool(link.get("degraded")),
                "maxPower": link.get("max_power"),
                "protocolWarningCount": link.get("protocol_warning_count", 0),
            },
            "rebootRequired": bool(state.get("reboot_required")),
            "recovery": state.get("recovery"),
            "protectedWorkloads": state.get("protected_workloads") or [],
        }
    return summary


def infer_status(processes):
    if processes.get("follower") or processes.get("safe"):
        return "patrolling"
    if processes.get("pcd"):
        return "scan"
    if processes.get("camera_loop"):
        return "video_recording"
    if processes.get("fastlio") or processes.get("livox"):
        return "ready"
    return "idle"


def current_route_file():
    out, _, _ = shell_out(["ps", "-eo", "args="], timeout=3)
    match = re.search(r"route_file:=([^\s]+\.csv)", out)
    return match.group(1) if match else ""


def heartbeat_payload(args):
    rid = robot_id_arg(args.robot_id)
    ros = collect_ros(args.ros_timeout)
    pose = ros.get("pose")
    sport = ros.get("sport") or {}
    processes = process_status()
    route_file = current_route_file()
    payload = {
        "robotId": rid,
        "time": now_local_text(),
        "timestamp": int(time.time()),
        "sentAt": now_text(),
        "agent": "go2_saas_agent",
        "status": infer_status(processes),
        "pose": pose,
        "position": pose,
        "motion": {
            "position": pose,
            "yaw_rad": pose.get("yaw") if pose else None,
            "velocity": {"vx": sport.get("vx"), "vyaw": sport.get("vyaw")},
        },
        "patrol": {
            "running": bool(processes.get("follower") or processes.get("safe")),
            "route_file": Path(route_file).name if route_file else "",
        },
        "battery": ros.get("battery"),
        "diagnostics": {
            "processes": processes,
            "network": network_summary(),
        },
        "telemetry": {
            "battery": ros.get("battery"),
            "sport": ros.get("sport"),
            "rosOk": ros.get("ok"),
            "rosErrors": ros.get("errors", []),
        },
        "assets": asset_manifest(rid, limit=3),
    }
    command_response_path = Path(os.environ.get("GO2_COMMAND_RESPONSE_FILE", "/tmp/go2_command_response.json"))
    if command_response_path.is_file():
        try:
            payload["commandResponse"] = json.loads(command_response_path.read_text())
        except Exception as exc:  # noqa: BLE001
            payload["commandResponseError"] = repr(exc)
    return payload


def upload_asset(path, kind, robot_id, dry_run=False, endpoint=None, timeout=180, patrol_id=""):
    if path is None:
        print("SKIP_%s missing path" % kind.upper())
        return 0
    if not path.is_file() or path.stat().st_size <= 0:
        print("SKIP_%s unavailable_or_empty %s" % (kind.upper(), path))
        return 1
    if kind in ("video", "image"):
        reason = invalid_media_time_reason(path)
        if reason:
            print("SKIP_%s invalid_media_time %s reason=%s" % (kind.upper(), path, reason), file=sys.stderr)
            return 0
    info = describe_file(path, kind)
    recorded_at = info["mtimeText"]
    pose = current_pose() if kind in ("video", "image") else None
    meta = {
        "kind": kind,
        "source": "go2_saas_agent",
        "path": str(path),
        "mtime": info["mtime"],
        "points": info.get("points"),
        "recordedAt": recorded_at,
    }
    if pose:
        meta["pose"] = pose
        meta["position"] = pose
    fields = {
        "robotId": robot_id,
        "fileName": path.name,
        "fileSize": str(info["size"]),
        "kind": kind,
        "assetType": kind,
        "time": recorded_at,
        "meta": json.dumps(meta, ensure_ascii=False, sort_keys=True),
    }
    if pose:
        fields.update({
            "x": str(pose.get("x")),
            "y": str(pose.get("y")),
            "z": str(pose.get("z")),
            "yaw": str(pose.get("yaw")),
            "position": json.dumps(pose, ensure_ascii=False, sort_keys=True),
            "pose": json.dumps(pose, ensure_ascii=False, sort_keys=True),
        })
    if patrol_id:
        fields["patrolId"] = patrol_id
    print("UPLOAD_ASSET kind=%s file=%s size=%s" % (kind, path, info["size"]))
    target_endpoint = endpoint or endpoint_for(kind)
    rc = post_multipart(path, fields, target_endpoint, dry_run=dry_run, timeout=timeout)
    if rc != 0 and not dry_run:
        dedupe = "file:%s:%s:%s:%s" % (target_endpoint, path, info["size"], info.get("mtime"))
        return enqueue_outbox_job(
            "file",
            target_endpoint,
            file_path=path,
            fields=fields,
            timeout=timeout,
            dedupe_key=dedupe,
            reason="upload failed rc=%s" % rc,
        )
    return rc


def command_id(command, index=0):
    for key in ("commandId", "id", "cmdId", "command_id"):
        value = command.get(key)
        if value:
            return str(value)
    return "generated-%d-%d" % (int(time.time()), index)


def command_action(command):
    for key in ("action", "type", "name", "command", "cmd"):
        value = command.get(key)
        if value:
            return str(value).strip().lower().replace("-", "_")
    return ""


def command_params(command):
    params = command.get("params") or command.get("payload") or command.get("data") or {}
    return params if isinstance(params, dict) else {"value": params}


def param_first(params, keys):
    for key in keys:
        value = params.get(key)
        if value not in (None, ""):
            return str(value).strip()
    return ""


def is_url(value):
    return value.startswith("http://") or value.startswith("https://")


def route_url_from_params(params):
    value = param_first(params, ["routeUrl", "route_url", "downloadUrl", "download_url", "fileUrl", "file_url", "url"])
    route_value = param_first(params, ["route", "routeFile", "route_file", "fileName", "file_name", "filename", "csv"])
    if is_url(route_value):
        return route_value
    return value


def route_name_from_params(params):
    value = param_first(params, ["fileName", "file_name", "filename", "routeFile", "route_file", "route", "routeName", "route_name", "name", "csv"])
    if not value:
        value = route_url_from_params(params)
    if is_url(value):
        value = Path(urllib.parse.urlparse(value).path).name
    else:
        value = Path(value).name
    if value and not value.endswith(".csv"):
        value += ".csv"
    if not value or not ROUTE_NAME_RE.match(value):
        raise ValueError("invalid route file name: %s" % value)
    return value


def build_download_url(value):
    if is_url(value):
        parsed = urllib.parse.urlparse(value)
        if parsed.hostname == "gogoguard.cn":
            backend = urllib.parse.urlparse(backend_base())
            if backend.scheme and backend.netloc:
                return urllib.parse.urlunparse((
                    backend.scheme,
                    backend.netloc,
                    parsed.path,
                    parsed.params,
                    parsed.query,
                    parsed.fragment,
                ))
        return value
    return build_url(value)


def download_route_csv(route_url, route_path, timeout=45, attempts=3):
    url = build_download_url(route_url)
    headers = {"Accept": "text/csv,*/*"}
    headers.update(auth_headers())
    headers.update(device_token_headers())
    request = urllib.request.Request(url, headers=headers, method="GET")
    last_error = None
    for attempt in range(1, attempts + 1):
        try:
            response = urllib.request.urlopen(request, timeout=timeout, context=tls_context(url))
            data = response.read()
            if response.getcode() < 200 or response.getcode() >= 300:
                raise RuntimeError("route download status=%s" % response.getcode())
            if not data.strip():
                raise RuntimeError("route download returned empty file")
            if data.lstrip().startswith(b"<"):
                raise RuntimeError("route download returned non-csv content")
            route_path.parent.mkdir(parents=True, exist_ok=True)
            tmp_path = route_path.with_suffix(route_path.suffix + ".tmp")
            tmp_path.write_bytes(data)
            tmp_path.replace(route_path)
            return {"downloaded": True, "bytes": len(data), "url": url, "downloadAttempts": attempt}
        except Exception as exc:  # noqa: BLE001
            last_error = exc
            print("ROUTE_DOWNLOAD_RETRY", attempt, repr(exc), file=sys.stderr)
            if attempt < attempts:
                time.sleep(min(8, 2 * attempt))
    raise RuntimeError("route download failed after %s attempts: %s" % (attempts, last_error))


def validate_route_csv(route_path):
    if not route_path.is_file() or route_path.stat().st_size <= 0:
        raise RuntimeError("route csv missing or empty: %s" % route_path)
    with route_path.open("r", newline="") as handle:
        reader = csv.DictReader(handle)
        fieldnames = reader.fieldnames or []
        required_columns = ("x", "y", "yaw", "v")
        missing = [name for name in required_columns if name not in fieldnames]
        if missing:
            raise RuntimeError(
                "route csv missing required columns %s: %s"
                % (", ".join(missing), route_path)
            )
        point_count = 0
        for line_number, row in enumerate(reader, start=2):
            if not row or not any(value not in (None, "") for value in row.values()):
                continue
            try:
                values = [float(row[name]) for name in required_columns]
            except (TypeError, ValueError) as exc:
                raise RuntimeError(
                    "route csv invalid numeric value at line %s: %s" % (line_number, exc)
                )
            if not all(math.isfinite(value) for value in values):
                raise RuntimeError(
                    "route csv non-finite value at line %s" % line_number
                )
            point_count += 1
    if point_count < 2:
        raise RuntimeError(
            "route csv too short: %s valid_points=%s" % (route_path, point_count)
        )
    return point_count + 1


def sha256_file(path):
    digest = hashlib.sha256()
    try:
        with Path(path).open("rb") as handle:
            while True:
                chunk = handle.read(1024 * 1024)
                if not chunk:
                    break
                digest.update(chunk)
    except OSError:
        return ""
    return digest.hexdigest()


def route_recording_evidence(route_path):
    """Resolve a recorder sidecar only when it still matches the exact CSV."""
    route_path = Path(route_path)
    sidecar_path = Path(str(route_path) + ".recording.json")
    actual_sha256 = sha256_file(route_path)
    evidence = {
        "available": False,
        "routeSha256": actual_sha256,
        "sidecarPath": str(sidecar_path),
    }
    if not sidecar_path.is_file():
        evidence["reason"] = (
            "route file is unavailable while constructing command"
            if not actual_sha256
            else "recording sidecar not found"
        )
        return evidence
    try:
        sidecar = json.loads(sidecar_path.read_text())
    except Exception as exc:  # noqa: BLE001
        raise RuntimeError(
            "route recording sidecar is unreadable: %s: %s"
            % (sidecar_path, exc)
        )
    expected_sha256 = str(sidecar.get("route_sha256", ""))
    if not expected_sha256 or expected_sha256 != actual_sha256:
        raise RuntimeError(
            "ROUTE_RECORDING_LINK_MISMATCH route=%s expected=%s actual=%s"
            % (route_path, expected_sha256 or "missing", actual_sha256)
        )
    recording_status = str(sidecar.get("status", ""))
    if recording_status != "complete":
        raise RuntimeError(
            "ROUTE_RECORDING_EVIDENCE_INCOMPLETE route=%s status=%s "
            "errors=%s"
            % (
                route_path,
                recording_status or "missing",
                json.dumps(
                    sidecar.get("errors", []),
                    ensure_ascii=False,
                ),
            )
        )
    if sidecar.get(
        "same_fastlio_session_at_start_and_stop"
    ) is False:
        raise RuntimeError(
            "ROUTE_RECORDING_SESSION_CHANGED route=%s" % route_path
        )
    evidence.update(
        {
            "available": True,
            "reason": "route hash matches recording blackbox",
            "recordingRunDir": sidecar.get("recording_run_dir", ""),
            "recordingStatus": recording_status,
            "sameFastlioSessionDuringRecording": sidecar.get(
                "same_fastlio_session_at_start_and_stop"
            ),
            "sidecar": sidecar,
        }
    )
    return evidence


def prepare_route_csv(params, dry_run=False):
    route_name = route_name_from_params(params)
    route_path = ROUTES_DIR / route_name
    route_url = route_url_from_params(params)
    info = {"routeName": route_name, "routePath": str(route_path), "downloaded": False}
    if route_url:
        info["routeUrl"] = route_url
        if not dry_run:
            info.update(download_route_csv(route_url, route_path))
    elif not route_path.is_file():
        raise RuntimeError("route csv not found locally and no routeUrl provided: %s" % route_name)
    if not dry_run:
        info["lines"] = validate_route_csv(route_path)
        info["recordingEvidence"] = route_recording_evidence(
            route_path
        )
    info["relocalization"] = route_relocalization_plan(route_path, params)
    info["localizationMode"] = info["relocalization"]["mode"]
    return route_path, info


def bounded_float(value, default, low, high):
    try:
        number = float(value)
    except Exception:  # noqa: BLE001
        number = default
    return max(low, min(high, number))


def next_patrol_run_dir(root=None, date_text=None):
    """Return the next date-scoped xunjian run directory."""
    root = Path(root) if root is not None else PATROL_RUNS_DIR
    date_text = str(date_text or time.strftime("%Y%m%d"))
    day_dir = root / date_text
    name_re = re.compile(
        r"^xunjian-%s-(\d+)$" % re.escape(date_text)
    )
    highest = 0
    if day_dir.is_dir():
        for child in day_dir.iterdir():
            match = name_re.match(child.name)
            if match:
                highest = max(highest, int(match.group(1)))
    return day_dir / (
        "xunjian-%s-%02d" % (date_text, highest + 1)
    )


def ros_env_prefix():
    env = Path(WS) / "scripts/env_common.sh"
    return "cd %s; source %s >/dev/null 2>&1 || true; " % (shlex.quote(WS), shlex.quote(str(env)))


def detached_command(inner, log_path, pgid_path=None):
    command = "setsid nohup bash -c %s </dev/null >%s 2>&1 &" % (
        shlex.quote(ros_env_prefix() + inner),
        shlex.quote(log_path),
    )
    if pgid_path is not None:
        command += " printf '%%s\\n' \"$!\" >%s;" % shlex.quote(
            str(pgid_path)
        )
    return command


def non_shell_process_probe(pattern):
    """Return a shell predicate that cannot match its own bash wrapper."""
    return (
        "ps -eo comm=,args= | "
        "awk '$1 != \"bash\" && $1 != \"sh\" {print}' | "
        "grep -Eq -- %s"
    ) % shlex.quote(pattern)


def non_shell_process_ids(pattern):
    """Return a pipeline that prints matching non-shell process IDs."""
    return (
        "ps -eo pid=,comm=,args= | "
        "awk '$2 != \"bash\" && $2 != \"sh\" {print}' | "
        "grep -E -- %s | awk '{print $1}'"
    ) % shlex.quote(pattern)


def detached_group_probe(pgid_path):
    """Return a shell predicate that checks the saved detached process group."""
    return (
        "pgid=$(cat %s 2>/dev/null || true); "
        "[ -n \"$pgid\" ] && [ \"$pgid\" -gt 1 ] 2>/dev/null && "
        "ps -eo pgid= | awk -v target=\"$pgid\" "
        "'$1 == target {found=1} END {exit !found}'"
    ) % shlex.quote(str(pgid_path))


def wait_for_detached_ready(
    process_probe,
    pgid_path,
    extra_probe="true",
    attempts=50,
    poll_seconds=0.2,
    ready_var="detached_ready",
):
    """Return a bounded shell loop for a detached process to become usable."""
    if not re.match(r"^[A-Za-z_][A-Za-z0-9_]*$", ready_var):
        raise ValueError("invalid shell variable name: %s" % ready_var)
    attempts = max(1, int(attempts))
    poll_seconds = max(0.01, float(poll_seconds))
    return (
        "%s=0; for startup_attempt in $(seq 1 %d); do "
        "if { %s; } && { %s; }; then "
        "%s=1; break; fi; "
        "if ! { %s; }; then break; fi; "
        "sleep %.3f; done;"
    ) % (
        ready_var,
        attempts,
        process_probe,
        extra_probe,
        ready_var,
        detached_group_probe(pgid_path),
        poll_seconds,
    )


def terminate_detached_group_command(
    pgid_path,
    graceful_signal="TERM",
    graceful_wait_attempts=10,
):
    """Return targeted cleanup for a detached launcher and all its children."""
    graceful_signal = str(graceful_signal).upper()
    if graceful_signal not in ("INT", "TERM"):
        raise ValueError("unsupported graceful signal: %s" % graceful_signal)
    graceful_wait_attempts = max(1, int(graceful_wait_attempts))
    pgid_arg = shlex.quote(str(pgid_path))
    group_alive = (
        "ps -eo pgid= | awk -v target=\"$pgid\" "
        "'$1 == target {found=1} END {exit !found}'"
    )
    return (
        "pgid=$(cat %s 2>/dev/null || true); "
        "if [ -n \"$pgid\" ] && [ \"$pgid\" -gt 1 ] 2>/dev/null "
        "&& %s; then "
        "kill -%s -- \"-$pgid\" 2>/dev/null || true; "
        "for cleanup_attempt in $(seq 1 %d); do "
        "if ! %s; then break; fi; sleep 0.2; done; "
        "if %s; then kill -TERM -- \"-$pgid\" 2>/dev/null || true; "
        "sleep 0.5; fi; "
        "if %s; then kill -KILL -- \"-$pgid\" 2>/dev/null || true; fi; "
        "fi; rm -f %s;"
    ) % (
        pgid_arg,
        group_alive,
        graceful_signal,
        graceful_wait_attempts,
        group_alive,
        group_alive,
        group_alive,
        pgid_arg,
    )


def start_patrol_command(route_path, params, route_info=None):
    speed = bounded_float(
        params.get("speed", params.get("vBase", params.get("v_base", os.environ.get("GO2_PATROL_EXEC_SPEED", "0.50")))),
        0.50,
        0.05,
        0.8,
    )
    # Go2_2's production start_patrol bounded v_base/max_vx to 0.50 m/s.
    speed = min(speed, 0.50)
    start_max_distance = bounded_float(
        params.get("startMaxDistance", params.get("start_max_distance", 0.35)),
        0.35,
        0.2,
        0.5,
    )
    start_max_yaw_error = bounded_float(
        params.get("startMaxYawError", params.get("start_max_yaw_error", 0.35)),
        0.35,
        0.2,
        0.6,
    )
    max_yaw_rate = bounded_float(
        params.get("maxYawRate", params.get("max_yaw_rate", 0.60)),
        0.60,
        0.15,
        0.8,
    )
    max_non_corner_yaw_rate = bounded_float(
        params.get(
            "maxNonCornerYawRate",
            params.get("max_non_corner_yaw_rate", 0.60),
        ),
        0.60,
        0.10,
        0.70,
    )
    k_yaw = bounded_float(
        params.get("kYaw", params.get("k_yaw", 1.20)),
        1.20,
        0.10,
        1.20,
    )
    tracking_lookahead_distance = bounded_float(
        params.get(
            "trackingLookaheadDistance",
            params.get(
                "tracking_lookahead_distance",
                params.get(
                    "rejoinLookaheadDistance",
                    params.get("rejoin_lookahead_distance", 0.50),
                ),
            ),
        ),
        0.50,
        0.15,
        0.80,
    )
    max_correction_angle_deg = bounded_float(
        params.get(
            "maxCorrectionAngleDeg",
            params.get(
                "max_correction_angle_deg",
                params.get(
                    "maxRejoinAngleDeg",
                    params.get("max_rejoin_angle_deg", 12.0),
                ),
            ),
        ),
        12.0,
        8.0,
        18.0,
    )
    course_heading_window = bounded_float(
        params.get(
            "courseHeadingWindow",
            params.get("course_heading_window", 0.80),
        ),
        0.80,
        0.30,
        1.20,
    )
    course_heading_min_distance = bounded_float(
        params.get(
            "courseHeadingMinDistance",
            params.get("course_heading_min_distance", 0.12),
        ),
        0.12,
        0.05,
        0.20,
    )
    line_deadband = bounded_float(
        params.get("lineDeadband", params.get("line_deadband", 0.030)),
        0.030,
        0.010,
        0.050,
    )
    minimum_moving_speed = bounded_float(
        params.get(
            "minimumMovingSpeed",
            params.get("minimum_moving_speed", 0.35),
        ),
        0.35,
        0.30,
        0.50,
    )
    min_track_yaw_rate = bounded_float(
        params.get(
            "minTrackYawRate",
            params.get("min_track_yaw_rate", 0.20),
        ),
        0.20,
        0.05,
        0.25,
    )
    yaw_deadband = bounded_float(
        params.get(
            "yawDeadband",
            params.get("yaw_deadband", 0.03),
        ),
        0.03,
        0.01,
        0.08,
    )
    heading_slow_angle_deg = bounded_float(
        params.get(
            "headingSlowAngleDeg",
            params.get("heading_slow_angle_deg", 20.0),
        ),
        20.0,
        10.0,
        35.0,
    )
    heading_stop_angle_deg = max(
        heading_slow_angle_deg + 10.0,
        bounded_float(
            params.get(
                "headingStopAngleDeg",
                params.get("heading_stop_angle_deg", 75.0),
            ),
            75.0,
            45.0,
            90.0,
        ),
    )
    corner_angle_deg = bounded_float(
        params.get("cornerAngleDeg", params.get("corner_angle_deg", 30.0)),
        30.0,
        20.0,
        90.0,
    )
    corner_slowdown_distance = bounded_float(
        params.get(
            "cornerSlowdownDistance",
            params.get("corner_slowdown_distance", 0.90),
        ),
        0.90,
        0.30,
        2.00,
    )
    corner_min_speed = bounded_float(
        params.get("cornerMinSpeed", params.get("corner_min_speed", 0.35)),
        0.35,
        0.30,
        0.50,
    )
    use_route_speed = bool_param(params, ("useRouteSpeed", "use_route_speed"), False)
    loop_mode = str(params.get("loop_mode", params.get("loopMode", "once")))
    if loop_mode not in ("once", "pingpong"):
        loop_mode = "once"

    relocalization = route_relocalization_plan(route_path, params)
    recording_evidence = (
        route_info.get("recordingEvidence")
        if route_info is not None
        else None
    )
    if not isinstance(recording_evidence, dict):
        recording_evidence = route_recording_evidence(route_path)
    if route_info is not None:
        route_info["relocalization"] = relocalization
        route_info["localizationMode"] = relocalization["mode"]
        route_info["recordingEvidence"] = recording_evidence

    patrol_run_dir = next_patrol_run_dir()
    effective_route_path = patrol_run_dir / "route_runtime.csv"
    relocalization["runtimeRoutePath"] = str(effective_route_path)
    if route_info is not None:
        route_info["patrolRunDir"] = str(patrol_run_dir)

    route_arg = shlex.quote(str(effective_route_path))
    source_route_arg = shlex.quote(str(route_path))
    # Go2_2 does not consume a route-transform JSON.  In PCD localization
    # mode, give it the already transformed runtime CSV instead.
    follower_route_arg = route_arg
    route_transform_param = ""
    if relocalization.get("pcdRelocalization"):
        route_transform_param = "-p route_transform_json:=%s " % shlex.quote(
            str(effective_route_path) + ".relocalize.json"
        )
    if route_info is not None:
        route_info["alignmentRoutePath"] = str(effective_route_path)
        route_info["followerRoutePath"] = str(effective_route_path)
        route_info["routeTransformEnabled"] = False
        route_info["routePretransformed"] = bool(relocalization.get("enabled"))
    speed_arg = shlex.quote("%.3f" % speed)
    start_max_distance_arg = shlex.quote("%.3f" % start_max_distance)
    start_max_yaw_error_arg = shlex.quote("%.3f" % start_max_yaw_error)
    max_yaw_rate_arg = shlex.quote("%.3f" % max_yaw_rate)
    max_non_corner_yaw_rate_arg = shlex.quote("%.3f" % max_non_corner_yaw_rate)
    k_yaw_arg = shlex.quote("%.3f" % k_yaw)
    tracking_lookahead_distance_arg = shlex.quote(
        "%.3f" % tracking_lookahead_distance
    )
    max_correction_angle_deg_arg = shlex.quote(
        "%.3f" % max_correction_angle_deg
    )
    course_heading_window_arg = shlex.quote(
        "%.3f" % course_heading_window
    )
    course_heading_min_distance_arg = shlex.quote(
        "%.3f" % course_heading_min_distance
    )
    line_deadband_arg = shlex.quote("%.3f" % line_deadband)
    minimum_moving_speed_arg = shlex.quote(
        "%.3f" % minimum_moving_speed
    )
    min_track_yaw_rate_arg = shlex.quote(
        "%.3f" % min_track_yaw_rate
    )
    yaw_deadband_arg = shlex.quote("%.3f" % yaw_deadband)
    heading_slow_angle_deg_arg = shlex.quote(
        "%.3f" % heading_slow_angle_deg
    )
    heading_stop_angle_deg_arg = shlex.quote(
        "%.3f" % heading_stop_angle_deg
    )
    corner_angle_deg_arg = shlex.quote("%.3f" % corner_angle_deg)
    corner_slowdown_distance_arg = shlex.quote(
        "%.3f" % corner_slowdown_distance
    )
    corner_min_speed_arg = shlex.quote("%.3f" % corner_min_speed)
    use_route_speed_arg = "true" if use_route_speed else "false"
    loop_arg = shlex.quote(loop_mode)
    sdk_if = str(params.get("sdkInterface", params.get("sdk_interface", params.get("sdkIface", params.get("sdk_if", os.environ.get("GO2_SDK_IF", "eth0")))))).strip() or "eth0"
    if not re.match(r"^[A-Za-z0-9_.:-]+$", sdk_if):
        raise RuntimeError("invalid SDK interface: %s" % sdk_if)
    sdk_if_arg = shlex.quote(sdk_if)
    ensure_base_script = shlex.quote(str(Path(WS) / "scripts/ensure_base_ready.sh"))
    alignment_script = shlex.quote(str(Path(WS) / "scripts/check_route_start_alignment.py"))
    manual_anchor_script = shlex.quote(
        str(Path(WS) / "scripts/manual_route_anchor.py")
    )
    session_guard_script = shlex.quote(
        str(Path(WS) / "scripts/localization_session_guard.py")
    )
    agent_script = shlex.quote(str(Path(WS) / "scripts/go2_saas_agent.py"))
    health_watchdog_script = shlex.quote(str(Path(WS) / "scripts/go2_base_health_watchdog.py"))
    health_watchdog_log = shlex.quote(str(Path(WS) / "patrol_logs/service/go2-base-health-watchdog.log"))
    performance_monitor_script = shlex.quote(
        str(Path(WS) / "scripts/patrol_performance_monitor.py")
    )
    experiment_telemetry_script = shlex.quote(
        str(Path(WS) / "scripts/go2_experiment_telemetry.py")
    )
    experiment_snapshot_script = shlex.quote(
        str(Path(WS) / "scripts/go2_experiment_snapshot.py")
    )
    follower_trace_script_path = (
        Path(WS) / "scripts/waypoint_follower_go2_2_trace.py"
    )
    follower_trace_script = shlex.quote(
        str(follower_trace_script_path)
    )
    controller_source_path = (
        Path(WS)
        / "src/go2_fastlio_patrol/go2_fastlio_patrol/"
        "waypoint_follower_go2_2.py"
    )
    patrol_day_dir_arg = shlex.quote(str(patrol_run_dir.parent))
    patrol_run_dir_arg = shlex.quote(str(patrol_run_dir))
    motion_enable_file = str(patrol_run_dir / ".motion_enabled")
    motion_enable_file_arg = shlex.quote(motion_enable_file)
    patrol_active_file = str(patrol_run_dir / ".patrol_active")
    patrol_active_file_arg = shlex.quote(patrol_active_file)
    anchor_metadata = str(patrol_run_dir / "manual_anchor.json")
    anchor_metadata_arg = shlex.quote(anchor_metadata)
    patrol_manifest_arg = shlex.quote(str(patrol_run_dir / "manifest.txt"))
    route_copy_arg = shlex.quote(str(patrol_run_dir / "route_original.csv"))
    sdk_receiver_log = str(patrol_run_dir / "go2_sdk2_udp_receiver.log")
    cmd_vel_sender_log = str(patrol_run_dir / "cmd_vel_udp_sender.log")
    safe_log = str(patrol_run_dir / "unitree_safe_cmd_node.log")
    follower_log = str(patrol_run_dir / "waypoint_follower.log")
    follower_trace = str(
        patrol_run_dir / "follower_control_trace.jsonl"
    )
    performance_log = str(patrol_run_dir / "performance_monitor.log")
    experiment_telemetry_log = str(
        patrol_run_dir / "experiment_telemetry.log"
    )
    experiment_telemetry_jsonl = str(
        patrol_run_dir / "experiment_telemetry.jsonl"
    )
    session_guard_log = str(patrol_run_dir / "localization_session_guard.log")
    rosbag_log = str(patrol_run_dir / "rosbag_record.log")
    rosbag_dir = str(patrol_run_dir / "rosbag")
    performance_pgid_file = str(patrol_run_dir / "performance_monitor.pgid")
    experiment_telemetry_pgid_file = str(
        patrol_run_dir / "experiment_telemetry.pgid"
    )
    session_guard_pgid_file = str(patrol_run_dir / "localization_session_guard.pgid")
    rosbag_pgid_file = str(patrol_run_dir / "rosbag.pgid")
    rosbag_topics_text = " ".join(PATROL_ROSBAG_TOPICS)
    rosbag_cmd = "ros2 bag record -o %s %s" % (
        shlex.quote(rosbag_dir),
        " ".join(shlex.quote(topic) for topic in PATROL_ROSBAG_TOPICS),
    )
    rosbag_process_pattern = (
        r"[r]os2 bag record.*" + re.escape(patrol_run_dir.name)
    )
    current_patrol_run_arg = shlex.quote(
        str(CURRENT_PATROL_RUN_FILE)
    )
    route_sha256 = (
        recording_evidence.get("routeSha256")
        or sha256_file(route_path)
        or "unavailable_at_command_construction"
    )
    route_recording_state = (
        "linked_hash_verified"
        if recording_evidence.get("available")
        else "not_available"
    )
    controller_source_sha256 = (
        sha256_file(controller_source_path) or "unavailable"
    )
    follower_trace_sha256 = (
        sha256_file(follower_trace_script_path) or "unavailable"
    )
    manifest_text = (
        "started_at=%s\n"
        "run_name=%s\n"
        "route=%s\n"
        "route_copy=route_original.csv\n"
        "route_sha256=%s\n"
        "route_recording_evidence=%s\n"
        "follower_route=%s\n"
        "speed=%s\n"
        "loop=%s\n"
        "sdk_if=%s\n"
        "localization_mode=%s\n"
        "manual_anchor_metadata=manual_anchor.json\n"
        "runtime_route=route_runtime.csv\n"
        "localization_session_guard=required\n"
        "localization_restart_policy=abort_current_patrol\n"
        "controller=deployed_go2_2_nearest_lookahead_unchanged\n"
        "controller_executable=waypoint_follower_go2_2_trace.py\n"
        "controller_source_sha256=%s\n"
        "controller_trace_wrapper_sha256=%s\n"
        "controller_trace_wrapper_policy=subclass_calls_base_control_unchanged\n"
        "start_alignment=runtime_anchor_point0_0.35m_0.35rad\n"
        "command_vy=0.000\n"
        "go2_2_k_yaw=0.900\n"
        "go2_2_max_yaw_rate=0.450\n"
        "go2_2_lookahead_distance=0.600\n"
        "go2_2_reach_distance=0.400\n"
        "go2_2_goal_distance=0.250\n"
        "go2_2_search_window=6\n"
        "go2_2_turn_in_place_angle=1.000\n"
        "go2_2_slow_down_angle=0.500\n"
        "go2_2_stuck_time=3.000\n"
        "go2_2_relocalize_distance=1.500\n"
        "udp_sdk_vx_limit=0.500\n"
        "fast_lio_input_qos=lidar_2_best_effort_imu_400_reliable\n"
        "cloud_freshness_source=local_receive_time\n"
        "fast_lio_freshness_gate=local_output_heartbeat_min_5_samples_per_second\n"
        "startup_motion_interlock=base_gate_before_follower_start\n"
        "startup_cpu_max_pct=85.0\n"
        "startup_stable_samples=3\n"
        "runtime_safe_node=deployed_source_unchanged\n"
        "runtime_fastlio=deployed_source_unchanged\n"
        "base_watchdog_auto_restart=false\n"
        "timing_diagnostics=follower,safe,ros_sender,udp_sdk,fast_lio_input_output\n"
        "follower_control_trace=follower_control_trace.jsonl\n"
        "performance_monitor_interval=1.0\n"
        "performance_log=performance_monitor.log\n"
        "experiment_telemetry=experiment_telemetry.jsonl\n"
        "experiment_telemetry_profile=patrol_20hz_no_pointcloud\n"
        "system_snapshot_start=system_start.json\n"
        "system_snapshot_end=system_end.json\n"
        "raw_pointcloud_observer=false\n"
        "raw_pointcloud_evidence=livox_and_fastlio_in_process_timing_logs\n"
        "rosbag_profile=control_light\n"
        "rosbag_topics=%s\n"
    ) % (
        now_text(),
        patrol_run_dir.name,
        str(route_path),
        route_sha256,
        route_recording_state,
        str(effective_route_path),
        speed,
        loop_mode,
        sdk_if,
        relocalization["mode"],
        controller_source_sha256,
        follower_trace_sha256,
        rosbag_topics_text,
    )
    manifest_text_arg = shlex.quote(manifest_text)
    route_recording_copy_cmd = ""
    if recording_evidence.get("available"):
        route_recording_copy_cmd = "cp -- %s %s;" % (
            shlex.quote(recording_evidence["sidecarPath"]),
            shlex.quote(
                str(patrol_run_dir / "route_recording.json")
            ),
        )
    base_log_offsets_file = str(
        patrol_run_dir / "base_log_offsets.txt"
    )
    base_log_sources = (
        str(Path(WS) / "patrol_logs/livox.log"),
        str(Path(WS) / "patrol_logs/fast_lio.log"),
        str(
            Path(WS)
            / "patrol_logs/service/go2-base-health-watchdog.log"
        ),
        "/tmp/console_base.log",
        "/tmp/go2_saas_base.log",
    )
    base_log_offset_cmd = (
        ": >%s; for src in %s; do "
        "size=$(stat -c %%s \"$src\" 2>/dev/null || echo 0); "
        "printf '%%s|%%s\\n' \"$src\" \"$size\" >>%s; done;"
    ) % (
        shlex.quote(base_log_offsets_file),
        " ".join(shlex.quote(path) for path in base_log_sources),
        shlex.quote(base_log_offsets_file),
    )
    video_run_dir_arg = shlex.quote(str(VIDEO_RUN_DIR))
    service_video_run_arg = shlex.quote(str(SERVICE_VIDEO_RUN_FILE))
    patrol_video_active_arg = shlex.quote(str(PATROL_VIDEO_ACTIVE_FILE))
    route_prepare_cmd = ""
    if relocalization.get("manualAnchor"):
        route_prepare_cmd = (
            "rm -f %s %s; "
            "python3 -u %s --route-file %s --output-route %s "
            "--metadata %s --timeout 12.0 --stable-seconds 1.0 "
            "--min-updates 8 --max-local-gap 0.35 "
            "--max-translation 0.08 --max-yaw 0.08; "
        ) % (
            route_arg,
            anchor_metadata_arg,
            manual_anchor_script,
            source_route_arg,
            route_arg,
            anchor_metadata_arg,
        )
    elif relocalization.get("pcdRelocalization"):
        collect_seconds = bounded_float(
            params.get("relocalizeCollectSeconds", params.get("relocalize_collect_seconds", os.environ.get("GO2_RELOCALIZE_COLLECT_SECONDS", 3.0))),
            3.0,
            1.0,
            10.0,
        )
        xy_search_radius = bounded_float(
            params.get("relocalizeSearchRadius", params.get("relocalize_search_radius", 0.75)),
            0.75,
            0.3,
            1.0,
        )
        xy_search_step = bounded_float(
            params.get("relocalizeSearchStep", params.get("relocalize_search_step", os.environ.get("GO2_RELOCALIZE_SEARCH_STEP", 0.5))),
            0.5,
            0.2,
            1.0,
        )
        yaw_step = bounded_float(
            params.get("relocalizeYawStepDeg", params.get("relocalize_yaw_step_deg", os.environ.get("GO2_RELOCALIZE_YAW_STEP_DEG", 10.0))),
            10.0,
            5.0,
            20.0,
        )
        fitness_threshold = bounded_float(
            params.get("relocalizeFitnessThreshold", params.get("relocalize_fitness_threshold", os.environ.get("GO2_RELOCALIZE_FITNESS_THRESHOLD", 0.23))),
            0.23,
            0.02,
            0.30,
        )
        start_anchor_enabled = bool_param(
            params,
            ("anchorRouteStart", "anchor_route_start"),
            True,
        )
        start_anchor_max_residual = bounded_float(
            params.get(
                "startAnchorMaxResidual",
                params.get("start_anchor_max_residual", 0.75),
            ),
            0.75,
            0.10,
            0.80,
        )
        start_anchor_max_yaw_deg = bounded_float(
            params.get(
                "startAnchorMaxYawDeg",
                params.get("start_anchor_max_yaw_deg", 20.0),
            ),
            20.0,
            5.0,
            30.0,
        )
        route_prepare_cmd = (
            "mkdir -p %s; rm -f %s %s.relocalize.json; "
            "ros2 run go2_map_manager route_relocalizer --ros-args "
            "-p route_file:=%s -p map_file:=%s -p out_route_file:=%s "
            "-p cloud_topic:=/cloud_registered_body -p cloud_in_body_frame:=true -p odom_topic:=/Odometry "
            "-p collect_seconds:=%s -p min_points:=800 -p min_cloud_frames:=8 "
            "-p max_capture_translation:=0.10 -p max_capture_yaw_deg:=5.0 "
            "-p xy_search_radius:=%s -p xy_search_step:=%s "
            "-p yaw_search_deg:=30.0 -p yaw_step_deg:=%s -p fitness_threshold:=%s "
            "-p ambiguity_fitness_ratio:=1.20 -p ambiguity_min_translation:=1.00 "
            "-p ambiguity_min_yaw_deg:=20.0 -p anchor_route_start:=%s "
            "-p max_start_anchor_residual:=%s -p max_start_anchor_yaw_deg:=%s "
            "&& "
            "python3 -u %s --capture-only --metadata %s "
            "--timeout 12.0 --stable-seconds 1.0 --min-updates 8 "
            "--max-local-gap 0.35 --max-translation 0.08 "
            "--max-yaw 0.08; "
        ) % (
            shlex.quote(str(effective_route_path.parent)),
            route_arg,
            route_arg,
            source_route_arg,
            shlex.quote(relocalization["mapPath"]),
            route_arg,
            shlex.quote("%.3f" % collect_seconds),
            shlex.quote("%.3f" % xy_search_radius),
            shlex.quote("%.3f" % xy_search_step),
            shlex.quote("%.3f" % yaw_step),
            shlex.quote("%.3f" % fitness_threshold),
            "true" if start_anchor_enabled else "false",
            shlex.quote("%.3f" % start_anchor_max_residual),
            shlex.quote("%.3f" % start_anchor_max_yaw_deg),
            manual_anchor_script,
            anchor_metadata_arg,
        )
    sdk_receiver_cmd = (
        "GO2_SDK_MAX_VY=0.020 "
        "%s/build/go2_cmd_vel_bridge/go2_sdk2_udp_receiver %s 5005"
    ) % (shlex.quote(WS), sdk_if_arg)
    performance_monitor_cmd = (
        "python3 -u %s --interval 1.0 --filesystem-path %s"
    ) % (performance_monitor_script, patrol_run_dir_arg)
    experiment_telemetry_cmd = (
        "python3 -u %s --profile patrol --output %s"
    ) % (
        experiment_telemetry_script,
        shlex.quote(experiment_telemetry_jsonl),
    )
    experiment_snapshot_start_cmd = (
        "python3 -u %s --output %s --phase start "
        "--workspace %s --route-file %s --label %s"
    ) % (
        experiment_snapshot_script,
        shlex.quote(str(patrol_run_dir / "system_start.json")),
        shlex.quote(WS),
        route_copy_arg,
        shlex.quote(patrol_run_dir.name),
    )
    cmd_vel_sender_cmd = (
        "ros2 run go2_cmd_vel_bridge cmd_vel_udp_sender --ros-args "
        "-p target_ip:=127.0.0.1 -p target_port:=5005 "
        "-p max_vx:=%s -p max_vy:=0.000 -p max_vyaw:=%s"
    ) % (speed_arg, max_yaw_rate_arg)
    safe_cmd = (
        "ros2 run go2_fastlio_patrol unitree_safe_cmd_node --ros-args "
        "-p max_vx:=%s -p max_vy:=0.000 "
        "-p max_yaw_rate:=%s -p publish_rate:=20.0 "
        "-p cloud_timeout:=1.0 -p cmd_timeout:=0.5 "
        "-p pointcloud_topic:=/cloud_registered_body -p sport_request_topic:=/api/sport/request "
        "-p output_cmd_topic:=/cmd_vel "
        "-p stop_distance:=0.80 -p resume_distance:=1.00 -p min_stop_points:=15 "
        "-p roi_x_min:=0.35 -p roi_x_max:=1.50 -p roi_y_min:=-0.30 -p roi_y_max:=0.30 "
        "-p roi_z_min:=0.30 -p roi_z_max:=0.90"
    ) % (speed_arg, max_yaw_rate_arg)
    follower_cmd = (
        "python3 -u %s --ros-args "
        "-p route_file:=%s -p v_base:=%s -p max_vx:=%s "
        "-p k_yaw:=0.900 -p max_yaw_rate:=0.450 "
        "-p lookahead_distance:=0.600 "
        "-p reach_distance:=0.400 -p goal_distance:=0.250 "
        "-p loop_mode:=%s -p search_window:=6 "
        "-p turn_in_place_angle:=1.000 "
        "-p slow_down_angle:=0.500 "
        "-p stuck_time:=3.000 -p relocalize_distance:=1.500 "
        "-p trace_file:=%s"
    ) % (
        follower_trace_script,
        follower_route_arg,
        speed_arg,
        speed_arg,
        loop_arg,
        shlex.quote(follower_trace),
    )
    session_guard_cmd = (
        "python3 -u %s --metadata %s --active-file %s --mode patrol "
        "--enable-file %s --manifest %s --event-log %s"
    ) % (
        session_guard_script,
        anchor_metadata_arg,
        patrol_active_file_arg,
        motion_enable_file_arg,
        patrol_manifest_arg,
        shlex.quote(session_guard_log),
    )
    guard_pattern = (
        r"[l]ocalization_session_guard.py.*"
        + re.escape(patrol_run_dir.name)
    )
    telemetry_pattern = (
        r"[g]o2_experiment_telemetry.py.*"
        + re.escape(patrol_run_dir.name)
    )
    performance_pattern = (
        r"[p]atrol_performance_monitor.py.*"
        + re.escape(patrol_run_dir.name)
    )
    control_cleanup_ids = non_shell_process_ids(
        r"[g]o2_saas_agent.py video-loop|[w]aypoint_follower|"
        r"[u]nitree_safe_cmd_node|[c]md_vel_udp_sender|"
        r"[g]o2_sdk2_udp_receiver|[p]atrol_performance_monitor.py|"
        r"[g]o2_experiment_telemetry.py"
    )
    rosbag_cleanup_ids = non_shell_process_ids(rosbag_process_pattern)
    performance_group_cleanup = terminate_detached_group_command(
        performance_pgid_file,
        graceful_signal="TERM",
        graceful_wait_attempts=5,
    )
    telemetry_group_cleanup = terminate_detached_group_command(
        experiment_telemetry_pgid_file,
        graceful_signal="TERM",
        graceful_wait_attempts=10,
    )
    rosbag_group_cleanup = terminate_detached_group_command(
        rosbag_pgid_file,
        graceful_signal="INT",
        graceful_wait_attempts=10,
    )
    guard_group_cleanup = terminate_detached_group_command(
        session_guard_pgid_file,
        graceful_signal="TERM",
        graceful_wait_attempts=5,
    )
    startup_cleanup = (
        "rm -f %s %s %s %s; "
        "control_pids=$(%s || true); "
        "[ -n \"$control_pids\" ] && "
        "kill -TERM $control_pids 2>/dev/null || true; "
        "%s %s %s %s "
        "rosbag_pids=$(%s || true); "
        "[ -n \"$rosbag_pids\" ] && "
        "kill -INT $rosbag_pids 2>/dev/null || true"
    ) % (
        patrol_active_file_arg,
        motion_enable_file_arg,
        patrol_video_active_arg,
        service_video_run_arg,
        control_cleanup_ids,
        performance_group_cleanup,
        telemetry_group_cleanup,
        rosbag_group_cleanup,
        guard_group_cleanup,
        rosbag_cleanup_ids,
    )

    commands = [
        (
            "conflict=$(ps -eo pid=,comm=,args= | "
            "awk '$2 != \"bash\" && $2 != \"sh\" && "
            "$0 ~ /[r]oute_recorder|[g]o2map_capture|"
            "\\/tmp\\/[z]1pro_video_loop\\.sh/ {print $1}' | "
            "grep -vx \"$$\" || true); "
            "[ -n \"$conflict\" ] && { "
            "echo PATROL_MODE_CONFLICT recorder_or_pcd_or_manual_video; "
            "echo \"$conflict\"; exit 4; };"
        ),
        (
            "p=$(ps -eo pid=,comm=,args= | "
            "awk '$2 != \"bash\" && $2 != \"sh\" && "
            "$0 ~ /[w]aypoint_follower|[u]nitree_safe_cmd_node|"
            "[c]md_vel_udp_sender|[g]o2_sdk2_udp_receiver|"
            "[p]atrol_performance_monitor.py|"
            "[g]o2_experiment_telemetry.py|[r]os2 bag record/ "
            "{print $1}' | grep -vx \"$$\" || true); "
            "[ -n \"$p\" ] && { echo PATROL_ALREADY_RUNNING; "
            "echo \"$p\"; exit 4; };"
        ),
        "mkdir -p %s;" % patrol_day_dir_arg,
        (
            "if ! mkdir %s; then echo PATROL_LOG_DIR_EXISTS %s >&2; "
            "exit 5; fi;"
        ) % (patrol_run_dir_arg, patrol_run_dir_arg),
        "rm -f %s %s;" % (motion_enable_file_arg, patrol_active_file_arg),
        "cp -- %s %s || exit 5;" % (source_route_arg, route_copy_arg),
        route_recording_copy_cmd,
        "printf '%%s' %s >%s;" % (manifest_text_arg, patrol_manifest_arg),
        base_log_offset_cmd,
        (
            "mkdir -p %s; printf '%%s\\n' %s >%s;"
            % (video_run_dir_arg, patrol_run_dir_arg, current_patrol_run_arg)
        ),
        "ln -sfn %s /tmp/go2_saas_sdk_receiver.log;"
        % shlex.quote(sdk_receiver_log),
        "ln -sfn %s /tmp/go2_saas_cmd_vel_sender.log;"
        % shlex.quote(cmd_vel_sender_log),
        "ln -sfn %s /tmp/go2_saas_safe.log;" % shlex.quote(safe_log),
        "ln -sfn %s /tmp/go2_saas_follower.log;"
        % shlex.quote(follower_log),
        "ln -sfn %s /tmp/go2_saas_rosbag.log;"
        % shlex.quote(rosbag_log),
        "ln -sfn %s /tmp/go2_saas_performance.log;"
        % shlex.quote(performance_log),
        "ln -sfn %s /tmp/go2_saas_experiment_telemetry.log;"
        % shlex.quote(experiment_telemetry_log),
        "echo PATROL_LOG_DIR=%s;" % patrol_run_dir_arg,
        (
            "bash %s || { rc=$?; rm -f %s; exit $rc; };"
            % (ensure_base_script, current_patrol_run_arg)
        ),
        (
            "if ! %s >%s 2>&1; then "
            "echo EXPERIMENT_START_SNAPSHOT_FAILED >&2; "
            "tail -n 80 %s >&2; %s; exit 48; fi; "
            "echo EXPERIMENT_START_SNAPSHOT_READY;"
            % (
                experiment_snapshot_start_cmd,
                shlex.quote(
                    str(patrol_run_dir / "system_start.log")
                ),
                shlex.quote(
                    str(patrol_run_dir / "system_start.log")
                ),
                startup_cleanup,
            )
        ),
        "%s echo EXPERIMENT_TELEMETRY_LAUNCHED log=%s;"
        % (
            detached_command(
                experiment_telemetry_cmd,
                experiment_telemetry_log,
                experiment_telemetry_pgid_file,
            ),
            shlex.quote(experiment_telemetry_log),
        ),
        wait_for_detached_ready(
            non_shell_process_probe(telemetry_pattern),
            experiment_telemetry_pgid_file,
            extra_probe=(
                "grep -q %s %s"
                % (
                    shlex.quote('"kind":"recorder_ready"'),
                    shlex.quote(experiment_telemetry_jsonl),
                )
            ),
            attempts=50,
            poll_seconds=0.2,
            ready_var="telemetry_ready",
        ),
        (
            "if [ \"$telemetry_ready\" != 1 ]; then "
            "echo EXPERIMENT_TELEMETRY_NOT_READY >&2; "
            "tail -n 80 %s >&2; %s; exit 49; fi; "
            "echo EXPERIMENT_TELEMETRY_READY "
            "attempt=$startup_attempt;"
            % (
                shlex.quote(experiment_telemetry_log),
                startup_cleanup,
            )
        ),
        "%s echo PERFORMANCE_MONITOR_LAUNCHED log=%s;"
        % (
            detached_command(
                performance_monitor_cmd,
                performance_log,
                performance_pgid_file,
            ),
            shlex.quote(performance_log),
        ),
        wait_for_detached_ready(
            non_shell_process_probe(performance_pattern),
            performance_pgid_file,
            extra_probe=(
                "grep -q %s %s"
                % (
                    shlex.quote("PERF_MONITOR_START"),
                    shlex.quote(performance_log),
                )
            ),
            attempts=40,
            poll_seconds=0.2,
            ready_var="performance_ready",
        ),
        (
            "if [ \"$performance_ready\" != 1 ]; then "
            "echo PERFORMANCE_MONITOR_NOT_READY >&2; "
            "tail -n 80 %s >&2; %s; exit 50; fi; "
            "echo PERFORMANCE_MONITOR_READY "
            "attempt=$startup_attempt;"
            % (
                shlex.quote(performance_log),
                startup_cleanup,
            )
        ),
        "%s echo SDK_RECEIVER_STARTED iface=%s log=%s; sleep 4;"
        % (
            detached_command(sdk_receiver_cmd, sdk_receiver_log),
            sdk_if_arg,
            shlex.quote(sdk_receiver_log),
        ),
        (
            "if grep -Eq "
            "'StandUp ret=[1-9][0-9]*|BalanceStand ret=[1-9][0-9]*' %s; "
            "then echo MOTION_SDK_NOT_READY: "
            "StandUp/BalanceStand returned nonzero. >&2; %s; exit 42; fi;"
            % (shlex.quote(sdk_receiver_log), startup_cleanup)
        ),
        (
            "if ! %s; "
            "then echo MOTION_SDK_NOT_READY: "
            "SDK receiver exited before patrol start. >&2; %s; exit 42; fi;"
            % (
                non_shell_process_probe(r"[g]o2_sdk2_udp_receiver"),
                startup_cleanup,
            )
        ),
        "%s echo CMD_VEL_SENDER_STARTED log=%s; sleep 1;"
        % (
            detached_command(cmd_vel_sender_cmd, cmd_vel_sender_log),
            shlex.quote(cmd_vel_sender_log),
        ),
        "%s echo SAFE_STARTED log=%s; sleep 1;"
        % (
            detached_command(safe_cmd, safe_log),
            shlex.quote(safe_log),
        ),
        (
            "if ! bash %s --fresh-only; then "
            "echo FASTLIO_NOT_FRESH_AFTER_STARTUP >&2; %s; exit 44; fi;"
            % (ensure_base_script, startup_cleanup)
        ),
        (
            "if ! { %s }; then echo ROUTE_FRAME_PREPARATION_FAILED >&2; "
            "%s; exit 47; fi;"
            % (route_prepare_cmd, startup_cleanup)
        ),
        (
            "python3 %s --route-file %s --max-distance %s "
            "--max-yaw-error %s || { rc=$?; %s; exit $rc; };"
            % (
                alignment_script,
                route_arg,
                start_max_distance_arg,
                start_max_yaw_error_arg,
                startup_cleanup,
            )
        ),
        (
            "if ! %s; then setsid nohup python3 -u %s "
            "</dev/null >>%s 2>&1 & echo BASE_HEALTH_WATCHDOG_STARTED; fi;"
            % (
                non_shell_process_probe(r"[g]o2_base_health_watchdog.py"),
                health_watchdog_script,
                health_watchdog_log,
            )
        ),
        "%s echo ROSBAG_LAUNCHED log=%s;"
        % (
            detached_command(rosbag_cmd, rosbag_log, rosbag_pgid_file),
            shlex.quote(rosbag_log),
        ),
        wait_for_detached_ready(
            non_shell_process_probe(rosbag_process_pattern),
            rosbag_pgid_file,
            extra_probe=(
                "find %s -maxdepth 1 -type f -name '*.db3' "
                "-print -quit 2>/dev/null | grep -q ."
                % shlex.quote(rosbag_dir)
            ),
            attempts=PATROL_ROSBAG_READY_ATTEMPTS,
            poll_seconds=PATROL_ROSBAG_READY_POLL_SECONDS,
            ready_var="rosbag_ready",
        ),
        (
            "if [ \"$rosbag_ready\" != 1 ]; then "
            "echo ROSBAG_NOT_READY: recorder did not become ready within "
            "10 seconds. >&2; "
            "tail -n 40 %s >&2; %s; exit 41; fi;"
            % (
                shlex.quote(rosbag_log),
                startup_cleanup,
            )
        ),
        "echo ROSBAG_READY attempt=$startup_attempt log=%s;"
        % shlex.quote(rosbag_log),
        "mkdir -p %s; touch %s;" % (
            video_run_dir_arg,
            patrol_video_active_arg,
        ),
        (
            "if ! %s; then touch %s; "
            "setsid nohup python3 -u %s video-loop --seconds 20 --upload "
            "--run-file %s </dev/null >/tmp/go2_saas_video_loop.log "
            "2>&1 & echo VIDEO_LOOP_STARTED; fi;"
            % (
                non_shell_process_probe(
                    r"[g]o2_saas_agent.py video-loop"
                ),
                service_video_run_arg,
                agent_script,
                service_video_run_arg,
            )
        ),
        "touch %s;" % patrol_active_file_arg,
        "%s echo LOCALIZATION_SESSION_GUARD_LAUNCHED log=%s;"
        % (
            detached_command(
                session_guard_cmd,
                session_guard_log,
                session_guard_pgid_file,
            ),
            shlex.quote(session_guard_log),
        ),
        wait_for_detached_ready(
            non_shell_process_probe(guard_pattern),
            session_guard_pgid_file,
            extra_probe=(
                "grep -q %s %s"
                % (
                    shlex.quote("SESSION_GUARD_STARTED"),
                    shlex.quote(session_guard_log),
                )
            ),
            attempts=PATROL_GUARD_READY_ATTEMPTS,
            poll_seconds=PATROL_GUARD_READY_POLL_SECONDS,
            ready_var="guard_ready",
        ),
        (
            "if [ \"$guard_ready\" != 1 ]; then "
            "echo LOCALIZATION_SESSION_GUARD_NOT_READY: guard did not "
            "become ready within 10 seconds. >&2; "
            "%s; exit 46; fi;"
            % startup_cleanup
        ),
        "echo LOCALIZATION_SESSION_GUARD_READY "
        "attempt=$startup_attempt log=%s;"
        % shlex.quote(session_guard_log),
        (
            "if ! bash %s --patrol-start-gate; then "
            "echo PATROL_START_GATE_FAILED: CPU or FAST-LIO did not "
            "become stable. >&2; %s; exit 45; fi;"
            % (ensure_base_script, startup_cleanup)
        ),
        detached_command(follower_cmd, follower_log),
        (
            "follower_ready=0; for i in $(seq 1 30); do "
            "if grep -q 'FOLLOWER_EXACT_TRACE_READY' %s; "
            "then follower_ready=1; break; fi; sleep 0.5; done;"
            % shlex.quote(follower_log)
        ),
        (
            "if [ \"$follower_ready\" != 1 ]; then "
            "echo FOLLOWER_NOT_READY: waypoint_follower exited during "
            "startup. >&2; tail -n 40 %s >&2; %s; exit 43; fi;"
            % (shlex.quote(follower_log), startup_cleanup)
        ),
        "echo FOLLOWER_STARTED log=%s;" % shlex.quote(follower_log),
        (
            "echo PATROL_STARTED route=%s speed=%s loop=%s log_dir=%s"
            % (
                shlex.quote(route_path.name),
                speed_arg,
                loop_arg,
                patrol_run_dir_arg,
            )
        ),
    ]
    return ros_env_prefix() + " ".join(command for command in commands if command)


def stop_patrol_command():
    rosbag_pattern_arg = shlex.quote(
        r"[r]os2 bag record.*" + re.escape(str(PATROL_RUNS_DIR))
    )
    current_patrol_run_arg = shlex.quote(
        str(CURRENT_PATROL_RUN_FILE)
    )
    return ros_env_prefix() + (
        "rm -f %s /tmp/go2_saas_video.run; "
        "run_dir=$(cat %s 2>/dev/null || true); "
        "[ -n \"$run_dir\" ] && rm -f \"$run_dir/.patrol_active\" "
        "\"$run_dir/.motion_enabled\" || true; "
        "pkill -TERM -f '[l]ocalization_session_guard.py.*--mode patrol' "
        "2>/dev/null || true; "
        "p=$(ps -eo pid=,comm=,args= | awk '$2 != \"bash\" && $2 != \"sh\" && $0 ~ /[w]aypoint_follower|[u]nitree_safe_cmd_node|[u]nitree_cmd_node|[c]md_vel_udp_sender|[g]o2_sdk2_udp_receiver|[p]atrol_performance_monitor.py|[g]o2_experiment_telemetry.py/ {print $1}' | grep -vx \"$$\" | tr \"\\n\" \" \" || true); "
        "[ -n \"$p\" ] && kill -TERM $p 2>/dev/null || true; sleep 1; "
        "p2=$(ps -eo pid=,comm=,args= | awk '$2 != \"bash\" && $2 != \"sh\" && $0 ~ /[w]aypoint_follower|[u]nitree_safe_cmd_node|[u]nitree_cmd_node|[c]md_vel_udp_sender|[g]o2_sdk2_udp_receiver|[p]atrol_performance_monitor.py|[g]o2_experiment_telemetry.py/ {print $1}' | grep -vx \"$$\" | tr \"\\n\" \" \" || true); "
        "[ -n \"$p2\" ] && kill -KILL $p2 2>/dev/null || true; "
        "ros2 topic pub -1 /api/sport/request unitree_api/msg/Request \"{header: {identity: {id: 9999, api_id: 1003}, lease: {id: 0}, policy: {priority: 0, noreply: false}}, parameter: '{}', binary: []}\" >/dev/null 2>&1 || true; "
        "sdk_if=${GO2_SDK_IF:-eth0}; %s/build/go2_cmd_vel_bridge/go2_sdk2_motion_probe --iface \"$sdk_if\" stop >/dev/null 2>&1 || true; "
        "bp=$(pgrep -f %s | tr \"\\n\" \" \" || true); "
        "[ -n \"$bp\" ] && kill -INT $bp 2>/dev/null || true; sleep 2; "
        "bp2=$(pgrep -f %s | tr \"\\n\" \" \" || true); "
        "[ -n \"$bp2\" ] && kill -TERM $bp2 2>/dev/null || true; sleep 1; "
        "if [ -n \"$run_dir\" ] && [ -d \"$run_dir\" ]; then "
        "printf 'stopped_at=%%s\\n' \"$(date '+%%Y-%%m-%%dT%%H:%%M:%%S%%z')\" >>\"$run_dir/manifest.txt\"; "
        "if [ -d \"$run_dir/rosbag\" ]; then "
        "ros2 bag info \"$run_dir/rosbag\" >\"$run_dir/rosbag_info.txt\" 2>&1 || true; fi; "
        "python3 -u %s/scripts/manual_route_anchor.py --capture-only "
        "--metadata \"$run_dir/localization_session_end.json\" "
        "--timeout 12.0 --stable-seconds 1.0 --min-updates 8 "
        "--max-local-gap 0.35 --max-translation 0.08 --max-yaw 0.08 "
        ">\"$run_dir/localization_session_end.log\" 2>&1 || "
        "printf 'localization_session_end_warning=true\\n' >>\"$run_dir/manifest.txt\"; "
        "python3 -u %s/scripts/go2_experiment_snapshot.py "
        "--output \"$run_dir/system_end.json\" --phase stop "
        "--workspace %s --route-file \"$run_dir/route_original.csv\" "
        "--label \"$(basename \"$run_dir\")\" "
        ">\"$run_dir/system_end.log\" 2>&1 || "
        "printf 'system_end_snapshot_warning=true\\n' >>\"$run_dir/manifest.txt\"; "
        "mkdir -p \"$run_dir/base_logs\"; "
        ": >\"$run_dir/base_log_slices.txt\"; "
        "while IFS='|' read -r src start; do "
        "[ -f \"$src\" ] || continue; "
        "case \"$start\" in ''|*[!0-9]*) start=0;; esac; "
        "size=$(stat -c %%s \"$src\" 2>/dev/null || echo 0); "
        "truncated=false; if [ \"$size\" -lt \"$start\" ]; then "
        "start=0; truncated=true; fi; "
        "tail -c \"+$((start + 1))\" \"$src\" "
        ">\"$run_dir/base_logs/$(basename \"$src\")\" 2>/dev/null "
        "|| true; "
        "printf '%%s|%%s|%%s|%%s\\n' \"$src\" \"$start\" "
        "\"$size\" \"$truncated\" >>\"$run_dir/base_log_slices.txt\"; "
        "done <\"$run_dir/base_log_offsets.txt\"; "
        "python3 -u %s/scripts/go2_experiment_audit.py "
        "--run-dir \"$run_dir\" >\"$run_dir/experiment_audit.log\" "
        "2>&1 || printf 'experiment_audit_warning=true\\n' "
        ">>\"$run_dir/manifest.txt\"; "
        "sha256sum \"$run_dir\"/route_original.csv "
        "\"$run_dir\"/route_runtime.csv "
        "\"$run_dir\"/manual_anchor.json "
        "\"$run_dir\"/experiment_telemetry.jsonl "
        "\"$run_dir\"/follower_control_trace.jsonl "
        "2>/dev/null >\"$run_dir/evidence_sha256.txt\" || true; "
        "run_kib=$(du -sk \"$run_dir\" 2>/dev/null | awk '{print $1}'); "
        "printf 'disk_usage_kib=%%s\\n' \"${run_kib:-0}\" >>\"$run_dir/manifest.txt\"; fi; "
        "rm -f %s; "
        "echo PATROL_STOPPED"
    ) % (
        shlex.quote(str(PATROL_VIDEO_ACTIVE_FILE)),
        current_patrol_run_arg,
        shlex.quote(WS),
        rosbag_pattern_arg,
        rosbag_pattern_arg,
        shlex.quote(WS),
        shlex.quote(WS),
        shlex.quote(WS),
        shlex.quote(WS),
        current_patrol_run_arg,
    )


def start_base_command():
    return (
        "p=$(ps -eo pid=,comm=,args= | "
        "awk '$2 != \"bash\" && $2 != \"sh\" && "
        "$0 ~ /[r]oute_recorder|[g]o2map_capture|[w]aypoint_follower|"
        "[u]nitree_safe_cmd_node|[c]md_vel_udp_sender|"
        "[g]o2_sdk2_udp_receiver|[l]ocalization_session_guard.py/ "
        "{print $1}' | "
        "grep -vx \"$$\" || true); "
        "[ -n \"$p\" ] && { "
        "echo BASE_RESTART_BLOCKED_ACTIVE_MODE; "
        "[ -n \"$p\" ] && echo pids=$p; exit 4; }; "
        "setsid nohup bash %s/scripts/base_bringup.sh </dev/null "
        ">/tmp/console_base.log 2>&1 & echo STARTED"
    ) % WS


def run_start_patrol(params, execute_safe=False):
    try:
        route_path, route_info = prepare_route_csv(params, dry_run=not execute_safe)
    except Exception as exc:  # noqa: BLE001
        return "rejected", str(exc), {"action": "start_patrol", "params": params}
    if not execute_safe:
        route_info["dryRun"] = True
        return "success", "start_patrol dry-run accepted", {"action": "start_patrol", "params": params, "route": route_info}
    try:
        cmd = start_patrol_command(route_path, params, route_info=route_info)
    except Exception as exc:  # noqa: BLE001
        return "rejected", str(exc), {"action": "start_patrol", "params": params, "route": route_info}
    out, err, rc = shell_out(["bash", "-lc", cmd], timeout=240)
    status = "success" if rc == 0 else "failed"
    failure_finalization = None
    if rc != 0:
        expected_run = str(route_info.get("patrolRunDir", ""))
        try:
            current_run = CURRENT_PATROL_RUN_FILE.read_text().strip()
        except OSError:
            current_run = ""
        # Only finalize the directory allocated by this exact attempt.  A
        # duplicate-start rejection must never stop an already running patrol.
        if expected_run and current_run == expected_run:
            final_out, final_err, final_rc = shell_out(
                ["bash", "-lc", stop_patrol_command()],
                timeout=120,
            )
            failure_finalization = {
                "attempted": True,
                "rc": final_rc,
                "out": final_out,
                "err": final_err,
                "runDir": expected_run,
            }
            final_text = "\n".join(
                part
                for part in (final_err, final_out)
                if part
            )
            if final_text:
                err = "\n".join(
                    part
                    for part in (
                        err,
                        "START_FAILURE_EVIDENCE_FINALIZATION\n"
                        + final_text,
                    )
                    if part
                )
    if rc == 0:
        message = out or err or "rc=%s" % rc
    else:
        message = "\n".join(part for part in (err, out) if part) or "rc=%s" % rc
    return status, message, {
        "action": "start_patrol",
        "params": params,
        "route": route_info,
        "rc": rc,
        "out": out,
        "err": err,
        "failureFinalization": failure_finalization,
    }


def run_stop_patrol(params, execute_safe=False):
    if not execute_safe:
        return "success", "stop_patrol dry-run accepted", {"action": "stop_patrol", "params": params, "dryRun": True}
    out, err, rc = shell_out(
        ["bash", "-lc", stop_patrol_command()],
        timeout=120,
    )
    status = "success" if rc == 0 else "failed"
    return status, out or err or "rc=%s" % rc, {"action": "stop_patrol", "params": params, "rc": rc, "out": out, "err": err}


def cmd_patrol_start_cli(args):
    params = {
        "fileName": args.route_name,
        "speed": args.speed,
        "loopMode": args.loop_mode,
        "localizationMode": args.localization_mode,
    }
    status, message, detail = run_start_patrol(
        params,
        execute_safe=True,
    )
    print(
        "PATROL_CLI_%s %s"
        % (
            "STARTED" if status == "success" else "START_FAILED",
            message,
        ),
        flush=True,
    )
    print(
        "PATROL_CLI_DETAIL "
        + json.dumps(detail, ensure_ascii=False, sort_keys=True),
        flush=True,
    )
    return 0 if status == "success" else 1


def cmd_patrol_stop_cli(args):
    del args
    status, message, detail = run_stop_patrol(
        {},
        execute_safe=True,
    )
    print(
        "PATROL_CLI_%s %s"
        % (
            "STOPPED" if status == "success" else "STOP_FAILED",
            message,
        ),
        flush=True,
    )
    print(
        "PATROL_CLI_DETAIL "
        + json.dumps(detail, ensure_ascii=False, sort_keys=True),
        flush=True,
    )
    return 0 if status == "success" else 1


def run_safe_command(action, params, execute_safe=False):
    if action in ("ping", "noop", "status"):
        return "success", "command accepted", {"action": action, "dryRun": not execute_safe}
    if action in START_PATROL_COMMANDS:
        return run_start_patrol(params, execute_safe=execute_safe)
    if action in STOP_PATROL_COMMANDS:
        return run_stop_patrol(params, execute_safe=execute_safe)
    if action in GOTO_COMMANDS:
        return "rejected", "goto/free navigation is not implemented in v1.5", {"action": action, "params": params}
    if action not in SAFE_COMMANDS:
        return "rejected", "unknown command action: %s" % action, {"action": action, "params": params}
    if not execute_safe:
        return "success", "safe command dry-run accepted: %s" % action, {"action": action, "params": params, "dryRun": True}

    mapping = {
        "start_base": start_base_command(),
        "stop_base": "bash %s/scripts/base_stop.sh >/dev/null 2>&1 || true; echo STOPPED" % WS,
        "camera_start_loop": "test -x /tmp/z1pro_video_loop.sh && echo CAMERA_LOOP_ALREADY_CONFIGURED || echo CAMERA_LOOP_SCRIPT_MISSING",
        "camera_stop_loop": "rm -f /tmp/z1pro_video_loop.run; pkill -TERM -f '[z]1pro_video_loop.sh' 2>/dev/null || true; echo CAMERA_LOOP_STOPPED",
    }
    cmd = mapping.get(action)
    if not cmd:
        return "rejected", "no local mapping for %s" % action, {"action": action, "params": params}
    out, err, rc = shell_out(["bash", "-lc", cmd], timeout=20)
    status = "success" if rc == 0 else "failed"
    return status, out or err or "rc=%s" % rc, {"action": action, "params": params, "rc": rc, "out": out, "err": err}


def post_command_result(args, cmd_id, status, message, detail=None):
    action = detail.get("action") if isinstance(detail, dict) else None
    payload = {
        "robotId": robot_id_arg(args.robot_id),
        "commandId": cmd_id,
        "status": status,
        "message": message,
        "time": now_local_text(),
        "reportedAt": now_text(),
        "result": {
            "command_id": cmd_id,
            "ok": status in ("success", "running"),
            "msg": message,
            "action": action,
            "status": status,
        },
    }
    if detail is not None:
        payload["detail"] = detail
    rc = post_json(args.result_endpoint, payload, dry_run=args.dry_run_results, timeout=args.post_timeout)
    if rc != 0 and not args.dry_run_results:
        dedupe = "command-result:%s:%s:%s" % (args.result_endpoint, cmd_id, status)
        return enqueue_outbox_job(
            "json",
            args.result_endpoint,
            payload=payload,
            timeout=args.post_timeout,
            dedupe_key=dedupe,
            reason="command result post failed rc=%s" % rc,
        )
    return rc


def load_seen_commands(path):
    if not path:
        return set()
    try:
        data = json.loads(Path(path).read_text())
        return set(str(item) for item in data)
    except Exception:  # noqa: BLE001
        return set()


def save_seen_commands(path, seen):
    if not path:
        return
    try:
        Path(path).write_text(json.dumps(sorted(seen), ensure_ascii=False, indent=2))
    except Exception as exc:  # noqa: BLE001
        print("SEEN_SAVE_ERROR", repr(exc), file=sys.stderr)


def handle_commands(args, commands, seen=None):
    if not isinstance(commands, list):
        return 0
    seen = seen if seen is not None else set()
    rc = 0
    for index, command in enumerate(commands):
        if not isinstance(command, dict):
            rc |= post_command_result(args, "invalid-%d" % index, "rejected", "command is not an object", {"raw": command})
            continue
        cmd_id = command_id(command, index)
        if cmd_id in seen:
            print("COMMAND_SKIP_SEEN", cmd_id)
            continue
        action = command_action(command)
        params = command_params(command)
        status, message, detail = run_safe_command(action, params, execute_safe=args.execute_safe)
        print("COMMAND", cmd_id, action or "<missing>", status, message)
        rc |= post_command_result(args, cmd_id, status, message, detail)
        if rc == 0:
            seen.add(cmd_id)
    return rc


def cmd_heartbeat_once(args):
    return post_json(args.endpoint, heartbeat_payload(args), dry_run=args.dry_run, timeout=args.post_timeout)


def cmd_heartbeat_loop(args):
    while True:
        if not loop_wall_time_ready("HEARTBEAT_LOOP", sleep_sec=args.interval):
            continue
        rc = cmd_heartbeat_once(args)
        if args.once_on_error and rc != 0:
            return rc
        time.sleep(args.interval)


def cmd_asset_manifest(args):
    payload = asset_manifest(robot_id_arg(args.robot_id), limit=args.limit)
    if args.endpoint:
        return post_json(args.endpoint, payload, dry_run=args.dry_run, timeout=args.post_timeout)
    print(json.dumps(payload, ensure_ascii=False, sort_keys=True, indent=2))
    return 0


def cmd_plan_fetch(args):
    rc, _, _, _ = get_json_capture(args.endpoint, dry_run=args.dry_run, timeout=args.post_timeout)
    return rc


def cmd_command_result(args):
    payload = {
        "robotId": robot_id_arg(args.robot_id),
        "commandId": args.command_id,
        "status": args.status,
        "message": args.message,
        "time": now_local_text(),
        "reportedAt": now_text(),
        "result": {
            "command_id": args.command_id,
            "ok": args.status in ("success", "running"),
            "msg": args.message,
            "action": "console-test",
            "status": args.status,
        },
    }
    if args.detail_json:
        payload["detail"] = json.loads(args.detail_json)
    rc = post_json(args.endpoint, payload, dry_run=args.dry_run, timeout=args.post_timeout)
    if rc != 0 and not args.dry_run:
        dedupe = "command-result:%s:%s:%s" % (args.endpoint, args.command_id, args.status)
        return enqueue_outbox_job(
            "json",
            args.endpoint,
            payload=payload,
            timeout=args.post_timeout,
            dedupe_key=dedupe,
            reason="manual command result post failed rc=%s" % rc,
        )
    return rc


def cmd_command_poll_once(args, seen=None):
    rc, _, _, data = post_json_capture(args.heartbeat_endpoint, heartbeat_payload(args), dry_run=False, timeout=args.post_timeout)
    if rc != 0:
        return rc
    commands = data.get("commands", []) if isinstance(data, dict) else []
    print("COMMANDS_COUNT", len(commands) if isinstance(commands, list) else 0)
    return handle_commands(args, commands, seen=seen)


def cmd_command_loop(args):
    cycle = 0
    rc = 0
    seen = load_seen_commands(args.seen_file)
    while args.cycles <= 0 or cycle < args.cycles:
        if args.run_file and not Path(args.run_file).exists():
            print("COMMAND_LOOP_STOP_FILE_MISSING %s" % args.run_file, flush=True)
            break
        if not loop_wall_time_ready("COMMAND_LOOP", sleep_sec=args.interval):
            continue
        rc |= cmd_command_poll_once(args, seen=seen)
        save_seen_commands(args.seen_file, seen)
        drain_jobs = int(getattr(args, "drain_outbox_jobs", 0) or 0)
        if drain_jobs > 0:
            drain_outbox_once(max_jobs=drain_jobs)
        cycle += 1
        if args.cycles > 0 and cycle >= args.cycles:
            break
        time.sleep(args.interval)
    return rc


def cmd_video_segment(args):
    if not wait_for_valid_wall_time(timeout=float(os.environ.get("GO2_TIME_WAIT_TIMEOUT", "180"))):
        return 75
    script = Path(WS) / "scripts/z1pro_upload_segment.sh"
    if not script.exists():
        print("missing script: %s" % script, file=sys.stderr)
        return 1
    env = os.environ.copy()
    env["GO2_ROBOT_ID"] = robot_id_arg(args.robot_id)
    env["GO2_BACKEND_BASE"] = backend_base()
    env["Z1PRO_UPLOAD"] = "1" if args.upload else "0"
    cmd = [str(script), str(args.seconds)]
    timeout_sec = int(args.seconds) + int(os.environ.get("Z1PRO_SEGMENT_TIMEOUT_MARGIN", "60"))
    try:
        run = subprocess.run(cmd, env=env, timeout=timeout_sec)
        return run.returncode
    except subprocess.TimeoutExpired:
        print("VIDEO_SEGMENT_TIMEOUT seconds=%s timeout=%s" % (args.seconds, timeout_sec), file=sys.stderr, flush=True)
        subprocess.call(["pkill", "-TERM", "-f", "[g]st-launch-1.0.*rtsp://192.168.144.108"])
        time.sleep(2)
        subprocess.call(["pkill", "-KILL", "-f", "[g]st-launch-1.0.*rtsp://192.168.144.108"])
        return 124


def cmd_video_loop(args):
    cycle = 0
    final_rc = 0
    managed_service_loop = bool(args.run_file) and Path(args.run_file).absolute() == SERVICE_VIDEO_RUN_FILE.absolute()
    idle_sleep = max(0.5, float(os.environ.get("GO2_VIDEO_IDLE_SLEEP", "2.0")))
    patrol_grace = max(0.0, float(os.environ.get("GO2_VIDEO_PATROL_GRACE", "10.0")))
    idle_announced = False
    while args.cycles <= 0 or cycle < args.cycles:
        if args.run_file and not Path(args.run_file).exists():
            print("VIDEO_LOOP_STOP_FILE_MISSING %s" % args.run_file, flush=True)
            break
        if managed_service_loop and not PATROL_VIDEO_ACTIVE_FILE.exists():
            if not idle_announced:
                print("VIDEO_LOOP_IDLE waiting_for_patrol %s" % PATROL_VIDEO_ACTIVE_FILE, flush=True)
                idle_announced = True
            if args.cycles > 0:
                cycle += 1
            time.sleep(idle_sleep)
            continue
        idle_announced = False
        print("VIDEO_LOOP_CYCLE %d seconds=%s upload=%s" % (cycle + 1, args.seconds, args.upload), flush=True)
        rc = cmd_video_segment(args)
        if rc != 0:
            final_rc |= rc
            print("VIDEO_LOOP_SEGMENT_FAILED rc=%s" % rc, flush=True)
            time.sleep(args.error_sleep)
        else:
            if args.max_jobs > 0:
                drain_outbox_once(max_jobs=args.max_jobs)
            time.sleep(args.interval)
        if managed_service_loop and PATROL_VIDEO_ACTIVE_FILE.exists():
            try:
                active_age = time.time() - PATROL_VIDEO_ACTIVE_FILE.stat().st_mtime
                if active_age >= patrol_grace and not patrol_control_running():
                    PATROL_VIDEO_ACTIVE_FILE.unlink()
                    print("VIDEO_LOOP_PATROL_FINISHED idle_after_current_segment", flush=True)
            except OSError:
                pass
        cycle += 1
    return final_rc


def upload_patrol_assets(args, cycle_index=0):
    rid = robot_id_arg(args.robot_id)
    dry_run = not args.upload
    rc = 0
    if args.heartbeat:
        rc |= post_json(args.heartbeat_endpoint, heartbeat_payload(args), dry_run=dry_run, timeout=args.post_timeout)
    if args.route:
        rc |= upload_asset(resolve_route(args.route), "route", rid, dry_run=dry_run, endpoint=args.route_endpoint, timeout=args.file_timeout, patrol_id=args.patrol_id)
    if args.pcd:
        rc |= upload_asset(resolve_pcd(args.pcd), "pcd", rid, dry_run=dry_run, endpoint=args.pcd_endpoint, timeout=args.file_timeout, patrol_id=args.patrol_id)
    if args.video:
        media = resolve_media(args.video, cycle_index=cycle_index)
        kind = "image" if media and media.suffix.lower() in (".jpg", ".jpeg") else "video"
        rc |= upload_asset(media, kind, rid, dry_run=dry_run, endpoint=args.video_endpoint, timeout=args.file_timeout, patrol_id=args.patrol_id)
    return rc


def cmd_upload_once(args):
    return upload_patrol_assets(args, cycle_index=0)


def cmd_patrol_loop(args):
    cycle = 0
    final_rc = 0
    while args.cycles <= 0 or cycle < args.cycles:
        if args.run_file and not Path(args.run_file).exists():
            print("LOOP_STOP_FILE_MISSING %s" % args.run_file, flush=True)
            break
        if not loop_wall_time_ready("PATROL_LOOP", sleep_sec=args.interval):
            continue
        drain_outbox_once(max_jobs=5)
        print("LOOP_CYCLE %d upload=%s route=%s pcd=%s video=%s" % (cycle + 1, bool(args.upload), args.route or "-", args.pcd or "-", args.video or "-"), flush=True)
        final_rc |= upload_patrol_assets(args, cycle_index=cycle)
        cycle += 1
        if args.cycles > 0 and cycle >= args.cycles:
            break
        time.sleep(args.interval)
    return final_rc


def cmd_outbox_once(args):
    return drain_outbox_once(max_jobs=args.max_jobs)


def cmd_outbox_loop(args):
    while True:
        if args.run_file and not Path(args.run_file).exists():
            print("OUTBOX_LOOP_STOP_FILE_MISSING %s" % args.run_file, flush=True)
            break
        if not loop_wall_time_ready("OUTBOX_LOOP", sleep_sec=args.interval):
            continue
        drain_outbox_once(max_jobs=args.max_jobs)
        time.sleep(args.interval)
    return 0


def add_patrol_upload_args(parser):
    parser.add_argument("--route", default="", help="route name such as xiaoqu1, or an absolute CSV path")
    parser.add_argument("--pcd", default="", help="PCD name such as xiaoqu1, or an absolute PCD path")
    parser.add_argument("--video", default="", help="latest, cycle, file name, or absolute media path")
    parser.add_argument("--patrol-id", default="")
    parser.add_argument("--upload", action="store_true", help="actually POST files; default is dry-run")
    parser.add_argument("--heartbeat", action="store_true", default=True)
    parser.add_argument("--no-heartbeat", dest="heartbeat", action="store_false")
    parser.add_argument("--heartbeat-endpoint", default="/robot/heartbeat")
    parser.add_argument("--route-endpoint", default=os.environ.get("GO2_ROUTE_UPLOAD_ENDPOINT", DEFAULT_ASSET_ENDPOINT))
    parser.add_argument("--pcd-endpoint", default=os.environ.get("GO2_PCD_UPLOAD_ENDPOINT", DEFAULT_ASSET_ENDPOINT))
    parser.add_argument("--video-endpoint", default=os.environ.get("GO2_VIDEO_UPLOAD_ENDPOINT", DEFAULT_VIDEO_ENDPOINT))
    parser.add_argument("--ros-timeout", type=float, default=1.5)
    parser.add_argument("--post-timeout", type=float, default=12.0)
    parser.add_argument("--file-timeout", type=float, default=240.0)


def add_command_poll_args(parser):
    parser.add_argument("--heartbeat-endpoint", default="/robot/heartbeat")
    parser.add_argument("--result-endpoint", default="/robot/command/result")
    parser.add_argument("--ros-timeout", type=float, default=1.5)
    parser.add_argument("--post-timeout", type=float, default=12.0)
    parser.add_argument("--execute-safe", action="store_true")
    parser.add_argument("--dry-run-results", action="store_true")
    parser.add_argument("--seen-file", default="/tmp/go2_saas_seen_commands.json")


def build_parser():
    parser = argparse.ArgumentParser(description="Go2 SaaS adapter")
    parser.add_argument("--robot-id", default=None)
    sub = parser.add_subparsers(dest="cmd")
    sub.required = True

    heartbeat = sub.add_parser("heartbeat-once")
    heartbeat.add_argument("--endpoint", default="/robot/heartbeat")
    heartbeat.add_argument("--dry-run", action="store_true")
    heartbeat.add_argument("--ros-timeout", type=float, default=1.5)
    heartbeat.add_argument("--post-timeout", type=float, default=12.0)
    heartbeat.set_defaults(func=cmd_heartbeat_once)

    loop = sub.add_parser("heartbeat-loop")
    loop.add_argument("--endpoint", default="/robot/heartbeat")
    loop.add_argument("--dry-run", action="store_true")
    loop.add_argument("--interval", type=float, default=5.0)
    loop.add_argument("--ros-timeout", type=float, default=1.5)
    loop.add_argument("--post-timeout", type=float, default=12.0)
    loop.add_argument("--once-on-error", action="store_true")
    loop.set_defaults(func=cmd_heartbeat_loop)

    manifest = sub.add_parser("asset-manifest")
    manifest.add_argument("--limit", type=int, default=8)
    manifest.add_argument("--endpoint", default="")
    manifest.add_argument("--dry-run", action="store_true")
    manifest.add_argument("--post-timeout", type=float, default=20.0)
    manifest.set_defaults(func=cmd_asset_manifest)

    plan = sub.add_parser("plan-fetch")
    plan.add_argument("--endpoint", default=DEFAULT_PLAN_ENDPOINT)
    plan.add_argument("--dry-run", action="store_true")
    plan.add_argument("--post-timeout", type=float, default=20.0)
    plan.set_defaults(func=cmd_plan_fetch)

    result = sub.add_parser("command-result")
    result.add_argument("--endpoint", default="/robot/command/result")
    result.add_argument("--command-id", required=True)
    result.add_argument("--status", choices=("success", "failed", "running", "rejected"), required=True)
    result.add_argument("--message", default="")
    result.add_argument("--detail-json", default="")
    result.add_argument("--dry-run", action="store_true")
    result.add_argument("--post-timeout", type=float, default=12.0)
    result.set_defaults(func=cmd_command_result)

    command_once = sub.add_parser("command-poll-once")
    add_command_poll_args(command_once)
    command_once.set_defaults(func=cmd_command_poll_once)

    command_loop = sub.add_parser("command-loop")
    add_command_poll_args(command_loop)
    command_loop.add_argument("--interval", type=float, default=5.0)
    command_loop.add_argument("--cycles", type=int, default=0, help="0 means run forever")
    command_loop.add_argument("--run-file", default="")
    command_loop.add_argument("--drain-outbox-jobs", type=int, default=0)
    command_loop.set_defaults(func=cmd_command_loop)

    patrol_start = sub.add_parser("patrol-start")
    patrol_start.add_argument("--route-name", required=True)
    patrol_start.add_argument("--speed", type=float, default=0.50)
    patrol_start.add_argument(
        "--loop-mode",
        choices=("once", "pingpong"),
        default="once",
    )
    patrol_start.add_argument(
        "--localization-mode",
        choices=("manual_anchor", "pcd"),
        default="manual_anchor",
    )
    patrol_start.set_defaults(func=cmd_patrol_start_cli)

    patrol_stop = sub.add_parser("patrol-stop")
    patrol_stop.set_defaults(func=cmd_patrol_stop_cli)

    outbox_once = sub.add_parser("outbox-once")
    outbox_once.add_argument("--max-jobs", type=int, default=20)
    outbox_once.set_defaults(func=cmd_outbox_once)

    outbox_loop = sub.add_parser("outbox-loop")
    outbox_loop.add_argument("--interval", type=float, default=5.0)
    outbox_loop.add_argument("--max-jobs", type=int, default=20)
    outbox_loop.add_argument("--run-file", default="")
    outbox_loop.set_defaults(func=cmd_outbox_loop)

    video = sub.add_parser("video-segment")
    video.add_argument("--seconds", type=int, default=20)
    video.add_argument("--upload", action="store_true")
    video.set_defaults(func=cmd_video_segment)

    video_loop = sub.add_parser("video-loop")
    video_loop.add_argument("--seconds", type=int, default=20)
    video_loop.add_argument("--upload", action="store_true")
    video_loop.add_argument("--interval", type=float, default=1.0)
    video_loop.add_argument("--error-sleep", type=float, default=5.0)
    video_loop.add_argument("--cycles", type=int, default=0, help="0 means run forever")
    video_loop.add_argument("--run-file", default="")
    video_loop.add_argument("--max-jobs", type=int, default=0)
    video_loop.set_defaults(func=cmd_video_loop)

    upload_once = sub.add_parser("upload-once")
    add_patrol_upload_args(upload_once)
    upload_once.set_defaults(func=cmd_upload_once)

    patrol_loop = sub.add_parser("patrol-loop")
    add_patrol_upload_args(patrol_loop)
    patrol_loop.add_argument("--interval", type=float, default=20.0)
    patrol_loop.add_argument("--cycles", type=int, default=0, help="0 means run forever")
    patrol_loop.add_argument("--run-file", default="", help="optional sentinel file; loop exits when it disappears")
    patrol_loop.set_defaults(func=cmd_patrol_loop)
    return parser


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())

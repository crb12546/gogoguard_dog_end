#!/usr/bin/env python3
"""Capture a bounded, read-only snapshot around one Go2 field experiment.

The snapshot is intentionally made at experiment boundaries instead of on
every sensor update.  It records the configuration and exact executable/source
identity needed to explain a run without adding meaningful load to FAST-LIO.
No environment dump is taken because the SaaS environment can contain tokens.
"""

from __future__ import print_function

import argparse
import concurrent.futures
import datetime
import hashlib
import json
import os
import platform
import re
import socket
import subprocess
import tempfile
import time
from pathlib import Path


KEY_PROCESS_PATTERNS = re.compile(
    r"livox_ros_driver2_node|fastlio_mapping|route_recorder|"
    r"waypoint_follower|unitree_safe_cmd_node|cmd_vel_udp_sender|"
    r"go2_sdk2_udp_receiver|go2_experiment_telemetry|"
    r"patrol_performance_monitor|ros2 bag record|go2_saas_agent"
)

SAFE_ENV_KEYS = (
    "ROS_DOMAIN_ID",
    "RMW_IMPLEMENTATION",
    "CYCLONEDDS_URI",
    "FASTRTPS_DEFAULT_PROFILES_FILE",
    "GO2_WS",
    "GO2_SDK_IF",
)

SOURCE_CANDIDATES = (
    "scripts/base_bringup.sh",
    "scripts/base_stop.sh",
    "scripts/env_common.sh",
    "scripts/ensure_base_ready.sh",
    "scripts/go2_saas_agent.py",
    "scripts/go2_base_health_watchdog.py",
    "scripts/check_fastlio_freshness.py",
    "scripts/go2_experiment_audit.py",
    "scripts/go2_experiment_snapshot.py",
    "scripts/go2_experiment_telemetry.py",
    "scripts/localization_session_guard.py",
    "scripts/manual_route_anchor.py",
    "scripts/patrol_performance_monitor.py",
    "scripts/route_recording_blackbox.py",
    "scripts/waypoint_follower_go2_2_trace.py",
    "src/go2_fastlio_patrol/go2_fastlio_patrol/route_recorder.py",
    "src/go2_fastlio_patrol/go2_fastlio_patrol/"
    "unitree_safe_cmd_node.py",
    "src/go2_fastlio_patrol/go2_fastlio_patrol/"
    "go2_course_control.py",
    "src/go2_fastlio_patrol/go2_fastlio_patrol/"
    "waypoint_follower_go2_2.py",
    "src/go2_cmd_vel_bridge/src/cmd_vel_udp_sender.cpp",
    "src/go2_cmd_vel_bridge/src/go2_sdk2_udp_receiver.cpp",
    "src/FAST_LIO/config/go2_mid360s.yaml",
    "src/FAST_LIO/launch/mapping.launch.py",
    "src/livox_ros_driver2/src/lddc.cpp",
    "src/livox_ros_driver2/src/livox_ros_driver2.cpp",
    "src/livox_ros_driver2/config/MID360s_config.json",
    "src/livox_ros_driver2/launch_ROS2/msg_MID360s_launch.py",
    "src/FAST_LIO/src/laserMapping.cpp",
)

EXECUTABLE_CANDIDATES = (
    "install/fast_lio/share/fast_lio/config/go2_mid360s.yaml",
    "install/fast_lio/share/fast_lio/launch/mapping.launch.py",
    "install/livox_ros_driver2/share/livox_ros_driver2/"
    "config/MID360s_config.json",
    "install/livox_ros_driver2/share/livox_ros_driver2/"
    "launch/msg_MID360s_launch.py",
    "install/go2_fastlio_patrol/lib/go2_fastlio_patrol/route_recorder",
    "install/go2_fastlio_patrol/lib/go2_fastlio_patrol/"
    "unitree_safe_cmd_node",
    "install/go2_fastlio_patrol/lib/go2_fastlio_patrol/"
    "waypoint_follower_go2_2",
    "build/go2_cmd_vel_bridge/cmd_vel_udp_sender",
    "build/go2_cmd_vel_bridge/go2_sdk2_udp_receiver",
    "build/FAST_LIO/fastlio_mapping",
    "install/FAST_LIO/lib/fast_lio/fastlio_mapping",
    "install/livox_ros_driver2/lib/livox_ros_driver2/"
    "livox_ros_driver2_node",
)

CONFIGURATION_CANDIDATES = (
    "src/FAST_LIO/config/go2_mid360s.yaml",
    "src/FAST_LIO/launch/mapping.launch.py",
    "src/livox_ros_driver2/config/MID360s_config.json",
    "src/livox_ros_driver2/launch_ROS2/msg_MID360s_launch.py",
    "install/fast_lio/share/fast_lio/config/go2_mid360s.yaml",
    "install/fast_lio/share/fast_lio/launch/mapping.launch.py",
    "install/livox_ros_driver2/share/livox_ros_driver2/"
    "config/MID360s_config.json",
    "install/livox_ros_driver2/share/livox_ros_driver2/"
    "launch/msg_MID360s_launch.py",
)


def iso_time():
    return datetime.datetime.now().astimezone().isoformat(
        timespec="milliseconds"
    )


def read_text(path, default="", limit=1024 * 1024):
    try:
        with open(path, "r", errors="replace") as handle:
            return handle.read(limit).strip()
    except (OSError, ValueError):
        return default


def sha256_file(path):
    digest = hashlib.sha256()
    try:
        with open(path, "rb") as handle:
            while True:
                chunk = handle.read(1024 * 1024)
                if not chunk:
                    break
                digest.update(chunk)
    except OSError:
        return ""
    return digest.hexdigest()


def file_identity(path):
    candidate = Path(path)
    try:
        stat = candidate.stat()
    except OSError:
        return None
    return {
        "path": str(candidate),
        "size_bytes": int(stat.st_size),
        "mtime_ns": int(stat.st_mtime_ns),
        "sha256": sha256_file(str(candidate)),
    }


def run_command(argv, timeout=5, output_limit=128 * 1024):
    try:
        completed = subprocess.run(
            argv,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=timeout,
        )
        stdout = completed.stdout
        stderr = completed.stderr
        truncated = (
            len(stdout.encode("utf-8", errors="replace")) > output_limit
            or len(stderr.encode("utf-8", errors="replace")) > output_limit
        )
        return {
            "argv": argv,
            "returncode": completed.returncode,
            "stdout": stdout[:output_limit],
            "stderr": stderr[:output_limit],
            "truncated": truncated,
        }
    except (OSError, subprocess.TimeoutExpired) as exc:
        return {
            "argv": argv,
            "returncode": 124
            if isinstance(exc, subprocess.TimeoutExpired)
            else 127,
            "stdout": "",
            "stderr": repr(exc),
            "truncated": False,
        }


def cpu_snapshot():
    result = {
        "online": read_text("/sys/devices/system/cpu/online"),
        "possible": read_text("/sys/devices/system/cpu/possible"),
    }
    by_cpu = {}
    cpu_root = Path("/sys/devices/system/cpu")
    for cpu_path in sorted(
        cpu_root.glob("cpu[0-9]*"),
        key=lambda path: int(path.name[3:]),
    ):
        entry = {}
        for name in (
            "online",
            "cpufreq/scaling_governor",
            "cpufreq/scaling_cur_freq",
            "cpufreq/scaling_min_freq",
            "cpufreq/scaling_max_freq",
            "cpufreq/cpuinfo_min_freq",
            "cpufreq/cpuinfo_max_freq",
        ):
            value = read_text(str(cpu_path / name))
            if value:
                entry[name.replace("/", "_")] = value
        if entry:
            by_cpu[cpu_path.name] = entry
    result["by_cpu"] = by_cpu
    return result


def thermal_snapshot():
    result = {}
    for zone in sorted(Path("/sys/class/thermal").glob("thermal_zone*")):
        label = read_text(str(zone / "type"), zone.name)
        raw = read_text(str(zone / "temp"))
        try:
            value = float(raw)
        except ValueError:
            continue
        if value > 1000.0:
            value /= 1000.0
        result["%s:%s" % (zone.name, label)] = round(value, 2)
    return result


def filesystem_snapshot(path):
    try:
        stat = os.statvfs(path)
    except OSError:
        return {}
    total = stat.f_blocks * stat.f_frsize
    available = stat.f_bavail * stat.f_frsize
    return {
        "path": str(path),
        "total_bytes": total,
        "available_bytes": available,
        "used_percent": (
            round(100.0 * (total - available) / total, 2)
            if total
            else 0.0
        ),
    }


def process_start_ticks(stat_text):
    closing = stat_text.rfind(")")
    fields = stat_text[closing + 2 :].split()
    return int(fields[19]) if closing >= 0 and len(fields) > 19 else None


def process_snapshot():
    processes = []
    for process_dir in Path("/proc").glob("[0-9]*"):
        try:
            command = (
                (process_dir / "cmdline")
                .read_bytes()
                .replace(b"\0", b" ")
                .decode("utf-8", errors="replace")
                .strip()
            )
            if not command or not KEY_PROCESS_PATTERNS.search(command):
                continue
            status = (process_dir / "status").read_text(errors="replace")
            stat_text = (process_dir / "stat").read_text(errors="replace")
            executable = os.readlink(str(process_dir / "exe"))
            pid = int(process_dir.name)
        except (OSError, ValueError):
            continue
        allowed = re.search(
            r"^Cpus_allowed_list:\s*(.+)$",
            status,
            flags=re.MULTILINE,
        )
        threads = re.search(
            r"^Threads:\s*(\d+)$",
            status,
            flags=re.MULTILINE,
        )
        try:
            scheduler = (
                os.sched_getscheduler(pid)
                if hasattr(os, "sched_getscheduler")
                else None
            )
        except OSError:
            scheduler = None
        try:
            nice = os.getpriority(os.PRIO_PROCESS, pid)
        except OSError:
            nice = None
        processes.append(
            {
                "pid": pid,
                "command": command,
                "executable": executable,
                "executable_sha256": sha256_file(executable),
                "start_ticks": process_start_ticks(stat_text),
                "cpu_allowed_list": (
                    allowed.group(1).strip() if allowed else ""
                ),
                "threads": int(threads.group(1)) if threads else None,
                "scheduler": scheduler,
                "nice": nice,
            }
        )
    return sorted(processes, key=lambda item: item["pid"])


def source_identities(workspace):
    identities = []
    seen = set()
    for relative in SOURCE_CANDIDATES + EXECUTABLE_CANDIDATES:
        path = Path(workspace) / relative
        identity = file_identity(str(path))
        if identity and identity["path"] not in seen:
            identity["relative_path"] = relative
            identities.append(identity)
            seen.add(identity["path"])
    return identities


def configuration_contents(workspace):
    result = {}
    for relative in CONFIGURATION_CANDIDATES:
        path = Path(workspace) / relative
        if not path.is_file():
            continue
        result[relative] = {
            "sha256": sha256_file(str(path)),
            "text": read_text(str(path), limit=256 * 1024),
        }
    return result


def atomic_write_json(path, payload):
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(
        prefix=".%s." % output.name,
        suffix=".tmp",
        dir=str(output.parent),
        text=True,
    )
    try:
        with os.fdopen(fd, "w") as handle:
            json.dump(payload, handle, indent=2, sort_keys=True)
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, str(output))
    except Exception:
        try:
            os.unlink(temporary)
        except OSError:
            pass
        raise


def command_snapshot():
    specs = {
        "nvpmodel": (["nvpmodel", "-q", "--verbose"], 5),
        "jetson_clocks": (["jetson_clocks", "--show"], 5),
        "lscpu": (["lscpu"], 5),
        "timedatectl": (
            [
                "timedatectl",
                "show",
                "--property=NTPSynchronized",
                "--property=TimeUSec",
                "--property=Timezone",
            ],
            5,
        ),
        "ip_link": (["ip", "-s", "-details", "link"], 5),
        "ip_route": (["ip", "route", "show"], 5),
        "ros_nodes": (["ros2", "node", "list"], 6),
        "ros_topics": (["ros2", "topic", "list", "-t"], 6),
        "ros_qos_odom": (
            ["ros2", "topic", "info", "-v", "/Odometry"],
            6,
        ),
        "ros_qos_lidar": (
            ["ros2", "topic", "info", "-v", "/livox/lidar"],
            6,
        ),
        "ros_qos_imu": (
            ["ros2", "topic", "info", "-v", "/livox/imu"],
            6,
        ),
        "ros_qos_cloud_body": (
            [
                "ros2",
                "topic",
                "info",
                "-v",
                "/cloud_registered_body",
            ],
            6,
        ),
        "ros_qos_sport_state": (
            [
                "ros2",
                "topic",
                "info",
                "-v",
                "/lf/sportmodestate",
            ],
            6,
        ),
        "ros_qos_low_state": (
            ["ros2", "topic", "info", "-v", "/lf/lowstate"],
            6,
        ),
        "ros_qos_wireless": (
            [
                "ros2",
                "topic",
                "info",
                "-v",
                "/wirelesscontroller",
            ],
            6,
        ),
        "ros_params_fastlio": (
            ["ros2", "param", "dump", "/laser_mapping"],
            8,
        ),
        "ros_params_livox": (
            ["ros2", "param", "dump", "/livox_lidar_publisher"],
            8,
        ),
    }
    results = {}
    # ROS CLI discovery can occasionally consume several seconds.  Run these
    # read-only boundary probes concurrently so the dog does not have to stay
    # waiting at the start mark for the sum of every timeout.
    with concurrent.futures.ThreadPoolExecutor(max_workers=4) as executor:
        futures = {
            executor.submit(run_command, argv, timeout): label
            for label, (argv, timeout) in specs.items()
        }
        for future in concurrent.futures.as_completed(futures):
            label = futures[future]
            try:
                results[label] = future.result()
            except Exception as exc:  # noqa: BLE001
                results[label] = {
                    "argv": specs[label][0],
                    "returncode": 255,
                    "stdout": "",
                    "stderr": repr(exc),
                    "truncated": False,
                }
    return {
        label: results[label]
        for label in specs
    }


def build_snapshot(args):
    workspace = str(Path(args.workspace).resolve())
    route = file_identity(args.route_file) if args.route_file else None
    commands = command_snapshot()
    return {
        "schema": "go2.experiment_snapshot.v1",
        "phase": args.phase,
        "label": args.label,
        "created_at": iso_time(),
        "wall_time_epoch": time.time(),
        "monotonic_s": time.monotonic(),
        "hostname": socket.gethostname(),
        "platform": platform.platform(),
        "uname": list(platform.uname()),
        "boot_id": read_text("/proc/sys/kernel/random/boot_id"),
        "uptime": read_text("/proc/uptime"),
        "loadavg": read_text("/proc/loadavg"),
        "kernel_cmdline": read_text("/proc/cmdline"),
        "nvpmodel_status_file": read_text("/var/lib/nvpmodel/status"),
        "safe_environment": {
            key: os.environ.get(key, "")
            for key in SAFE_ENV_KEYS
        },
        "cpu": cpu_snapshot(),
        "memory": read_text("/proc/meminfo"),
        "thermal_c": thermal_snapshot(),
        "filesystem": filesystem_snapshot(workspace),
        "interrupts": read_text("/proc/interrupts"),
        "pressure": {
            resource: read_text("/proc/pressure/%s" % resource)
            for resource in ("cpu", "memory", "io")
        },
        "route": route,
        "processes": process_snapshot(),
        "files": source_identities(workspace),
        "configuration_files": configuration_contents(workspace),
        "commands": commands,
        "observer_policy": {
            "raw_pointcloud_subscriber": False,
            "reason": (
                "Avoid cloud deserialization/serialization observer load; "
                "use Livox and FAST-LIO in-process timing logs."
            ),
        },
    }


def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    parser.add_argument(
        "--phase",
        choices=("start", "stop", "failure", "manual"),
        required=True,
    )
    parser.add_argument("--label", default="")
    parser.add_argument(
        "--workspace",
        default=os.environ.get(
            "GO2_WS",
            "/home/unitree/go2_fastlio_ws",
        ),
    )
    parser.add_argument("--route-file", default="")
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)
    payload = build_snapshot(args)
    atomic_write_json(args.output, payload)
    print(
        "EXPERIMENT_SNAPSHOT_READY phase=%s output=%s processes=%d files=%d"
        % (
            args.phase,
            args.output,
            len(payload["processes"]),
            len(payload["files"]),
        ),
        flush=True,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

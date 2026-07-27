#!/usr/bin/env python3
"""Run the existing GoGoGuard agent with start/stop bound to the XBF9 R5 task.

The platform command transport, heartbeat, command de-duplication, result
upload, video loop and outbox remain implemented by the deployed
``go2_saas_agent.py``.  Only the two patrol handlers are replaced:

* every start-patrol alias starts the installed ``xbf9-horizontal-clean-r1``
  bundle and ignores every CSV name/URL supplied by the platform;
* every stop-patrol alias stops that exact XBF runtime.

This is intentionally a temporary fixed-task bridge.  It does not pretend that
the current SaaS protocol already carries a prepared-task identity.
"""

from __future__ import annotations

import importlib.util
import json
import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path
from types import ModuleType
from typing import Any, Callable, Dict, Optional, Tuple


TASK_ID = "xbf9-horizontal-clean-r1"
FIXED_SPEED_MPS = "0.20"
FIXED_MAX_YAW_RATE_RAD_S = "0.45"
FIXED_LOOP_MODE = "once"
DEFAULT_BUNDLE_ROOT = Path("/home/unitree/localization_upgrade")
DEFAULT_BASE_AGENT = Path(
    "/home/unitree/go2_fastlio_ws/scripts/go2_saas_agent.py"
)
DEFAULT_RUNTIME_DIR = Path("/tmp/go2_xbf_patrol")
PLATFORM_ROUTE_KEYS = {
    "csv",
    "downloadUrl",
    "download_url",
    "fileName",
    "fileUrl",
    "file_name",
    "file_url",
    "filename",
    "map",
    "mapFile",
    "mapName",
    "map_file",
    "map_name",
    "pcd",
    "pcdFile",
    "pcd_file",
    "route",
    "routeFile",
    "routeName",
    "routeUrl",
    "route_file",
    "route_name",
    "route_url",
    "url",
}

HandlerResult = Tuple[str, str, Dict[str, Any]]


def _atomic_json(path: Path, value: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp.%s" % os.getpid())
    temporary.write_text(
        json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    os.replace(str(temporary), str(path))


def _read_json(path: Path) -> Optional[Dict[str, Any]]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError, TypeError):
        return None
    return value if isinstance(value, dict) else None


def _proc_start_ticks(pid: int) -> Optional[int]:
    try:
        text = (Path("/proc") / str(pid) / "stat").read_text(encoding="ascii")
    except OSError:
        return None
    closing_parenthesis = text.rfind(")")
    if closing_parenthesis < 0:
        return None
    fields = text[closing_parenthesis + 2 :].split()
    if len(fields) <= 19 or not fields[19].isdigit():
        return None
    return int(fields[19])


def _pid_alive(pid: int) -> bool:
    if pid <= 1:
        return False
    try:
        os.kill(pid, 0)
    except OSError:
        return False
    return True


def _tail(path: Path, maximum_characters: int = 6000) -> str:
    try:
        text = path.read_text(encoding="utf-8", errors="replace")
    except OSError:
        return ""
    return text[-maximum_characters:]


class FixedTaskManager:
    """Start and stop one immutable patrol bundle from GoGoGuard commands."""

    def __init__(
        self,
        bundle_root: Optional[Path] = None,
        runtime_dir: Optional[Path] = None,
        popen_factory: Callable[..., Any] = subprocess.Popen,
        run_factory: Callable[..., Any] = subprocess.run,
        sleep: Callable[[float], None] = time.sleep,
        start_reaper: bool = True,
    ) -> None:
        self.bundle_root = Path(
            bundle_root
            or os.environ.get("GO2_XBF_FIXED_BUNDLE_ROOT", DEFAULT_BUNDLE_ROOT)
        ).resolve()
        self.runtime_dir = Path(
            runtime_dir
            or os.environ.get("GO2_XBF_RUNTIME_DIR", DEFAULT_RUNTIME_DIR)
        ).resolve()
        self.popen_factory = popen_factory
        self.run_factory = run_factory
        self.sleep = sleep
        self.start_reaper = start_reaper
        self.start_script = self.bundle_root / "scripts/start_xbf_patrol.sh"
        self.stop_script = self.bundle_root / "scripts/stop_xbf_patrol.sh"
        self.task_report = self.bundle_root / "TASK_BUILD_REPORT.json"
        self.pid_file = self.runtime_dir / "patrol.pids"
        self.ready_file = self.runtime_dir / "logs/route_ready.json"
        self.supervisor_log = (
            self.runtime_dir / "logs/gogoguard_xbf9_supervisor.log"
        )
        self.bridge_record = self.runtime_dir / "gogoguard_bridge.json"

    def _identity(self) -> Dict[str, Any]:
        report = _read_json(self.task_report)
        if report is None:
            raise RuntimeError(
                "fixed task report is missing or invalid: %s" % self.task_report
            )
        if report.get("task_id") != TASK_ID:
            raise RuntimeError(
                "fixed task identity mismatch: expected=%s actual=%s"
                % (TASK_ID, report.get("task_id"))
            )
        compiled = report.get("compiled_runtime")
        if not isinstance(compiled, dict):
            raise RuntimeError("fixed task report has no compiled_runtime")
        if compiled.get("map_id") != TASK_ID:
            raise RuntimeError(
                "fixed map identity mismatch: expected=%s actual=%s"
                % (TASK_ID, compiled.get("map_id"))
            )
        route_sha = str(compiled.get("aligned_route_sha256", ""))
        checkpoint_sha = str(compiled.get("checkpoint_sha256", ""))
        if len(route_sha) != 64 or len(checkpoint_sha) != 64:
            raise RuntimeError("fixed task report has invalid runtime hashes")
        for path in (self.start_script, self.stop_script):
            if not path.is_file() or not os.access(str(path), os.X_OK):
                raise RuntimeError(
                    "fixed task helper is missing or not executable: %s" % path
                )
        return report

    def _detail(
        self, action: str, params: Dict[str, Any], report: Dict[str, Any]
    ) -> Dict[str, Any]:
        compiled = report["compiled_runtime"]
        return {
            "action": action,
            "bridgeMode": "hardcoded_xbf9_r5",
            "taskId": TASK_ID,
            "bundleRoot": str(self.bundle_root),
            "runtimeDir": str(self.runtime_dir),
            "platformRouteIgnored": True,
            "ignoredPlatformRouteKeys": sorted(
                key for key in params if key in PLATFORM_ROUTE_KEYS
            ),
            "fixedSpeedMps": float(FIXED_SPEED_MPS),
            "fixedMaxYawRateRadS": float(FIXED_MAX_YAW_RATE_RAD_S),
            "fixedLoopMode": FIXED_LOOP_MODE,
            "mapId": compiled["map_id"],
            "alignedRouteSha256": compiled["aligned_route_sha256"],
            "checkpointSha256": compiled["checkpoint_sha256"],
        }

    def _supervisor_from_pid_file(self) -> Optional[int]:
        try:
            lines = self.pid_file.read_text(encoding="utf-8").splitlines()
        except OSError:
            return None
        values = {}
        for line in lines:
            if "=" in line:
                key, value = line.split("=", 1)
                values[key] = value
        value = values.get("supervisor_pid", "")
        return int(value) if value.isdigit() else None

    def _ready(self, minimum_mtime: float = 0.0) -> bool:
        try:
            if self.ready_file.stat().st_mtime < minimum_mtime:
                return False
        except OSError:
            return False
        value = _read_json(self.ready_file)
        return bool(value and value.get("ready") is True)

    def _active_supervisor(self) -> Optional[int]:
        pid = self._supervisor_from_pid_file()
        if pid is not None and _pid_alive(pid):
            return pid
        # start_xbf_patrol.sh writes patrol.pids only after preflight.  Keep a
        # second start command from opening a duplicate supervisor in that
        # window by consulting the bridge-owned PID + Linux starttime record.
        record = _read_json(self.bridge_record)
        if record and self._recorded_launch_matches(record):
            return int(record["supervisor_pid"])
        return None

    def _launch_environment(self) -> Dict[str, str]:
        environment = os.environ.copy()
        environment.update(
            {
                "GO2_INPUT_EXTRINSICS_VERIFIED": "1",
                "GO2_XBF_CALIBRATION_ONLY": "0",
                "GO2_XBF_PATROL_SPEED": FIXED_SPEED_MPS,
                "GO2_XBF_MAX_YAW_RATE": FIXED_MAX_YAW_RATE_RAD_S,
                "GO2_XBF_LOOP_MODE": FIXED_LOOP_MODE,
                "GO2_XBF_RUNTIME_DIR": str(self.runtime_dir),
            }
        )
        cyclone_uri = environment.get("CYCLONEDDS_URI", "")
        if cyclone_uri.startswith("file:///"):
            environment["GO2_XBF_CYCLONEDDS_URI"] = cyclone_uri
        return environment

    def _write_launch_record(
        self,
        pid: int,
        params: Dict[str, Any],
        report: Dict[str, Any],
        launched_at: float,
    ) -> None:
        start_ticks = None
        for _ in range(20):
            start_ticks = _proc_start_ticks(pid)
            if start_ticks is not None:
                break
            self.sleep(0.01)
        _atomic_json(
            self.bridge_record,
            {
                "schema": "go2.gogoguard_xbf_bridge/v1",
                "task_id": TASK_ID,
                "supervisor_pid": pid,
                "supervisor_start_ticks": start_ticks,
                "launched_at_epoch": launched_at,
                "platform_route_ignored": True,
                "ignored_platform_route_keys": sorted(
                    key for key in params if key in PLATFORM_ROUTE_KEYS
                ),
                "fixed_speed_mps": float(FIXED_SPEED_MPS),
                "fixed_max_yaw_rate_rad_s": float(
                    FIXED_MAX_YAW_RATE_RAD_S
                ),
                "fixed_loop_mode": FIXED_LOOP_MODE,
                "aligned_route_sha256": report["compiled_runtime"][
                    "aligned_route_sha256"
                ],
            },
        )

    def _reap(self, process: Any) -> None:
        try:
            return_code = process.wait()
        except Exception as error:  # noqa: BLE001
            return_code = "wait_error:%r" % (error,)
        record = _read_json(self.bridge_record)
        try:
            recorded_pid = int(record.get("supervisor_pid", -1)) if record else -1
            process_pid = int(getattr(process, "pid", -2))
        except (TypeError, ValueError):
            return
        if not record or recorded_pid != process_pid:
            return
        record["supervisor_exit_code"] = return_code
        record["supervisor_exited_at_epoch"] = time.time()
        try:
            _atomic_json(self.bridge_record, record)
        except OSError:
            pass

    def _start_reaper(self, process: Any) -> None:
        if not self.start_reaper or not callable(getattr(process, "wait", None)):
            return
        thread = threading.Thread(target=self._reap, args=(process,), daemon=True)
        thread.start()

    def start(
        self, params: Dict[str, Any], execute_safe: bool = False
    ) -> HandlerResult:
        try:
            report = self._identity()
        except Exception as error:  # noqa: BLE001
            return (
                "rejected",
                "GoGoGuard fixed XBF task is unavailable: %s" % error,
                {
                    "action": "start_patrol",
                    "bridgeMode": "hardcoded_xbf9_r5",
                    "taskId": TASK_ID,
                    "platformRouteIgnored": True,
                },
            )
        detail = self._detail("start_patrol", params, report)
        if not execute_safe:
            detail["dryRun"] = True
            return (
                "success",
                "fixed XBF9 R5 start_patrol dry-run accepted; platform CSV/URL ignored",
                detail,
            )

        active_pid = self._active_supervisor()
        if active_pid is not None:
            detail["supervisorPid"] = active_pid
            detail["alreadyActive"] = True
            if self._ready():
                return (
                    "success",
                    "fixed XBF9 R5 patrol is already RUNNING",
                    detail,
                )
            return (
                "running",
                "fixed XBF9 R5 patrol startup is already in progress",
                detail,
            )

        self.runtime_dir.mkdir(parents=True, exist_ok=True)
        self.supervisor_log.parent.mkdir(parents=True, exist_ok=True)
        for stale_path in (self.ready_file, self.bridge_record):
            try:
                stale_path.unlink()
            except FileNotFoundError:
                pass

        launched_at = time.time()
        log_handle = self.supervisor_log.open("a", encoding="utf-8")
        log_handle.write(
            "\n=== GoGoGuard fixed start task=%s epoch=%.6f ===\n"
            % (TASK_ID, launched_at)
        )
        log_handle.flush()
        try:
            process = self.popen_factory(
                ["/bin/bash", str(self.start_script)],
                cwd=str(self.bundle_root),
                env=self._launch_environment(),
                stdin=subprocess.DEVNULL,
                stdout=log_handle,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
        except Exception as error:  # noqa: BLE001
            log_handle.close()
            return (
                "failed",
                "failed to launch fixed XBF9 R5 patrol: %s" % error,
                detail,
            )
        finally:
            # The child owns the duplicated descriptor after Popen returns.
            if "process" in locals():
                log_handle.close()

        detail["supervisorPid"] = int(process.pid)
        self._write_launch_record(
            int(process.pid), params, report, launched_at
        )
        self._start_reaper(process)

        # Only catch immediate shell/configuration failures.  Do not block the
        # single SaaS command loop until localization reaches RUNNING, otherwise
        # a platform stop_patrol could not be consumed during startup.
        for _ in range(20):
            return_code = process.poll()
            if return_code is not None:
                detail["rc"] = int(return_code)
                detail["supervisorLogTail"] = _tail(self.supervisor_log)
                return (
                    "failed",
                    "fixed XBF9 R5 patrol exited during startup rc=%s"
                    % return_code,
                    detail,
                )
            if self._ready(minimum_mtime=launched_at):
                return (
                    "success",
                    "fixed XBF9 R5 patrol reached RUNNING",
                    detail,
                )
            self.sleep(0.1)

        return (
            "running",
            "fixed XBF9 R5 patrol launched; localization is converging and motion remains gated until RUNNING",
            detail,
        )

    def _recorded_launch_matches(self, record: Dict[str, Any]) -> bool:
        try:
            pid = int(record["supervisor_pid"])
        except (KeyError, TypeError, ValueError):
            return False
        if not _pid_alive(pid):
            return False
        expected_ticks = record.get("supervisor_start_ticks")
        if expected_ticks is None:
            return False
        actual_ticks = _proc_start_ticks(pid)
        try:
            expected_ticks_value = int(expected_ticks)
        except (TypeError, ValueError):
            return False
        return (
            actual_ticks is not None
            and actual_ticks == expected_ticks_value
        )

    def _stop_recorded_early_launch(self) -> bool:
        record = _read_json(self.bridge_record)
        if not record or not self._recorded_launch_matches(record):
            return False
        pid = int(record["supervisor_pid"])
        # Popen(start_new_session=True) guarantees an exact supervisor-owned
        # process group.  Signal the group so a preflight child cannot outlive
        # the shell before patrol.pids exists.
        if os.getpgid(pid) != pid:
            return False
        os.killpg(pid, signal.SIGINT)
        for _ in range(100):
            if not _pid_alive(pid):
                return True
            self.sleep(0.1)
        os.killpg(pid, signal.SIGTERM)
        for _ in range(50):
            if not _pid_alive(pid):
                return True
            self.sleep(0.1)
        os.killpg(pid, signal.SIGKILL)
        return True

    def stop(
        self, params: Dict[str, Any], execute_safe: bool = False
    ) -> HandlerResult:
        try:
            report = self._identity()
        except Exception as error:  # noqa: BLE001
            return (
                "rejected",
                "GoGoGuard fixed XBF task is unavailable: %s" % error,
                {
                    "action": "stop_patrol",
                    "bridgeMode": "hardcoded_xbf9_r5",
                    "taskId": TASK_ID,
                },
            )
        detail = self._detail("stop_patrol", params, report)
        if not execute_safe:
            detail["dryRun"] = True
            return (
                "success",
                "fixed XBF9 R5 stop_patrol dry-run accepted",
                detail,
            )

        try:
            completed = self.run_factory(
                ["/bin/bash", str(self.stop_script)],
                cwd=str(self.bundle_root),
                env=self._launch_environment(),
                capture_output=True,
                text=True,
                timeout=120,
                check=False,
            )
        except Exception as error:  # noqa: BLE001
            return (
                "failed",
                "failed to invoke fixed XBF9 R5 stop: %s" % error,
                detail,
            )

        # If stop arrives while start_xbf_patrol.sh is still in preflight, its
        # pidfile does not exist yet.  The bridge record closes that window.
        stopped_early_launch = False
        try:
            stopped_early_launch = self._stop_recorded_early_launch()
        except OSError:
            stopped_early_launch = False
        detail.update(
            {
                "rc": int(completed.returncode),
                "out": completed.stdout,
                "err": completed.stderr,
                "stoppedPrePidfileLaunch": stopped_early_launch,
            }
        )
        if completed.returncode != 0:
            return (
                "failed",
                completed.stderr
                or completed.stdout
                or "fixed XBF9 R5 stop failed rc=%s"
                % completed.returncode,
                detail,
            )
        return (
            "success",
            completed.stdout
            or "fixed XBF9 R5 patrol stopped; final StopMove requested",
            detail,
        )


def install_fixed_handlers(
    base_agent: ModuleType, manager: FixedTaskManager
) -> None:
    for name in ("run_start_patrol", "run_stop_patrol", "run_safe_command", "main"):
        if not hasattr(base_agent, name):
            raise RuntimeError("base SaaS agent is missing required symbol: %s" % name)

    def fixed_start(
        params: Dict[str, Any], execute_safe: bool = False
    ) -> HandlerResult:
        return manager.start(params, execute_safe=execute_safe)

    def fixed_stop(
        params: Dict[str, Any], execute_safe: bool = False
    ) -> HandlerResult:
        return manager.stop(params, execute_safe=execute_safe)

    base_agent.run_start_patrol = fixed_start
    base_agent.run_stop_patrol = fixed_stop


def load_base_agent(path: Path) -> ModuleType:
    path = path.resolve()
    if path == Path(__file__).resolve():
        raise RuntimeError("base SaaS agent points back to the XBF bridge")
    if not path.is_file():
        raise RuntimeError("base SaaS agent not found: %s" % path)
    specification = importlib.util.spec_from_file_location(
        "go2_saas_agent_deployed", str(path)
    )
    if specification is None or specification.loader is None:
        raise RuntimeError("cannot load base SaaS agent: %s" % path)
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module


def bridge_self_check(
    base_agent: ModuleType, manager: FixedTaskManager
) -> Dict[str, Any]:
    start_aliases = set(getattr(base_agent, "START_PATROL_COMMANDS", set()))
    stop_aliases = set(getattr(base_agent, "STOP_PATROL_COMMANDS", set()))
    if "start_patrol" not in start_aliases or "stop_patrol" not in stop_aliases:
        raise RuntimeError("base SaaS agent patrol aliases are incompatible")
    status, _, start_detail = base_agent.run_safe_command(
        "start_patrol",
        {
            "fileName": "must-be-ignored.csv",
            "routeUrl": "https://example.invalid/must-not-download.csv",
        },
        execute_safe=False,
    )
    if (
        status != "success"
        or start_detail.get("taskId") != TASK_ID
        or start_detail.get("platformRouteIgnored") is not True
    ):
        raise RuntimeError("fixed start_patrol dry-run did not use the XBF9 task")
    stop_status, _, stop_detail = base_agent.run_safe_command(
        "stop_patrol", {}, execute_safe=False
    )
    if (
        stop_status != "success"
        or stop_detail.get("taskId") != TASK_ID
        or stop_detail.get("platformRouteIgnored") is not True
    ):
        raise RuntimeError("fixed stop_patrol dry-run did not use the XBF9 task")
    return {
        "task_id": TASK_ID,
        "start_aliases": sorted(start_aliases),
        "stop_aliases": sorted(stop_aliases),
        "platform_route_ignored": True,
        "bundle_root": str(manager.bundle_root),
    }


def main(argv: Optional[list] = None) -> int:
    arguments = sys.argv[1:] if argv is None else argv
    base_path = Path(
        os.environ.get("GO2_SAAS_BASE_AGENT", str(DEFAULT_BASE_AGENT))
    )
    base_agent = load_base_agent(base_path)
    manager = FixedTaskManager()
    install_fixed_handlers(base_agent, manager)
    if arguments == ["--bridge-self-check"]:
        print(
            "GOGOGUARD_XBF9_R5_BRIDGE_SELF_CHECK "
            + json.dumps(
                bridge_self_check(base_agent, manager),
                ensure_ascii=False,
                sort_keys=True,
            ),
            flush=True,
        )
        return 0
    print(
        "GOGOGUARD_XBF9_R5_BRIDGE_ACTIVE task=%s platform_route_ignored=true"
        % TASK_ID,
        flush=True,
    )
    return int(base_agent.main(arguments))


if __name__ == "__main__":
    raise SystemExit(main())

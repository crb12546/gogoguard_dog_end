#!/usr/bin/env python3
"""Single-owner A7600C ECM connection manager for the Unitree Go2.

The vendor's Linux mode is USB ECM with DIALMODE=0 (modem auto dial).  The
modem network device is renamed to go2_4g and managed with DHCP.  The robot's
reserved eth0/eth1 interfaces are never selected or configured.
"""

import argparse
import fcntl
import json
import logging
import os
import re
import select
import signal
import socket
import subprocess
import sys
import tempfile
import termios
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Tuple


RESERVED_ROBOT_INTERFACES = {"eth0", "eth1"}
REGISTERED_CEREG_STATES = {1, 5}
MANAGER_VERSION = "2026-07-21-xhci-fatal-recovery-v1"
XHCI_FATAL_PATTERNS = (
    re.compile(r"xHCI host controller not responding, assume dead", re.IGNORECASE),
    re.compile(r"HC died; cleaning up", re.IGNORECASE),
    re.compile(r"USBSTS:.*HCHalted.*HCE", re.IGNORECASE),
)
XHCI_CONTEXT_PATTERN = re.compile(
    r"tegra-xusb|xhci|USBSTS|HC died|NETDEV WATCHDOG|cdc_ether",
    re.IGNORECASE,
)
PROTECTED_WORKLOADS = {
    "patrol": "waypoint_follower",
    "route_recording": "route_recorder",
    "pcd_recording": "go2map_capture",
}


def env_int(name: str, default: int) -> int:
    try:
        return int(os.environ.get(name, str(default)))
    except ValueError:
        raise SystemExit(f"invalid integer in {name}")


def env_bool(name: str, default: bool) -> bool:
    value = os.environ.get(name, "1" if default else "0").strip().lower()
    if value in {"1", "true", "yes", "on"}:
        return True
    if value in {"0", "false", "no", "off"}:
        return False
    raise SystemExit(f"invalid boolean in {name}")


def read_text(path: Path) -> str:
    try:
        return path.read_text(encoding="utf-8", errors="replace").strip()
    except (FileNotFoundError, PermissionError, OSError):
        return ""


def parse_cereg(text: str) -> Optional[int]:
    match = re.search(r"\+CEREG:\s*(?:\d+\s*,\s*)?(\d+)", text)
    return int(match.group(1)) if match else None


def parse_csq(text: str) -> Optional[int]:
    match = re.search(r"\+CSQ:\s*(\d+)", text)
    return int(match.group(1)) if match else None


def parse_dial_mode(text: str) -> Optional[int]:
    match = re.search(r"\+DIALMODE:\s*(\d+)", text, re.IGNORECASE)
    return int(match.group(1)) if match else None


def parse_usbnet_config(text: str) -> Tuple[Optional[int], Optional[int]]:
    match = re.search(
        r'\$MYCONFIG:\s*"usbnetmode"\s*,\s*(\d+)(?:\s*,\s*(\d+))?',
        text,
        re.IGNORECASE,
    )
    if not match:
        return None, None
    return int(match.group(1)), int(match.group(2)) if match.group(2) else None


def safe_ecm_interface(name: str) -> bool:
    return (name not in RESERVED_ROBOT_INTERFACES and
            bool(re.fullmatch(r"[A-Za-z0-9_.-]{1,15}", name)))


@dataclass(frozen=True)
class Config:
    sysfs_root: Path
    xhci_device: str
    usb_bus_device: str
    usb_vid: str
    usb_pid: str
    switch_vid: str
    switch_pid: str
    interface: str
    apn: str
    gateway: str
    cloud_host: str
    cloud_port: int
    fallback_host: str
    fallback_port: int
    loop_seconds: int
    usb_settle_seconds: int
    at_retry_seconds: int
    dhcp_retry_seconds: int
    dhcp_timeout_seconds: int
    health_failures: int
    link_restart_after: int
    modem_reset_after: int
    modem_reset_cooldown: int
    rapid_usb_limit: int
    rapid_usb_window: int
    radio_off_seconds: int
    state_file: Path
    auto_reboot_on_fatal: bool
    fatal_reboot_delay_seconds: int
    fatal_reboot_cooldown_seconds: int
    reboot_marker: Path
    fault_log: Path

    @classmethod
    def from_env(cls) -> "Config":
        return cls(
            sysfs_root=Path(os.environ.get("GO2_4G_SYSFS_ROOT", "/sys")),
            xhci_device=os.environ.get(
                "GO2_4G_XHCI_DEVICE", "3610000.xhci"),
            usb_bus_device=os.environ.get("GO2_4G_USB_BUS_DEVICE", "usb1"),
            usb_vid=os.environ.get("GO2_4G_USB_VID", "1e0e").lower(),
            usb_pid=os.environ.get("GO2_4G_USB_PID", "9011").lower(),
            switch_vid=os.environ.get("GO2_4G_SWITCH_VID", "2ecc").lower(),
            switch_pid=os.environ.get("GO2_4G_SWITCH_PID", "3001").lower(),
            interface=os.environ.get("GO2_4G_ECM_INTERFACE", "go2_4g"),
            apn=os.environ.get("GO2_4G_APN", "cmnet"),
            gateway=os.environ.get("GO2_4G_ECM_GATEWAY", "192.168.0.1"),
            cloud_host=os.environ.get("GO2_4G_CLOUD_HOST", "39.96.37.187"),
            cloud_port=env_int("GO2_4G_CLOUD_PORT", 443),
            fallback_host=os.environ.get("GO2_4G_FALLBACK_HOST", "223.5.5.5"),
            fallback_port=env_int("GO2_4G_FALLBACK_PORT", 53),
            loop_seconds=env_int("GO2_4G_LOOP_SECONDS", 5),
            usb_settle_seconds=env_int("GO2_4G_USB_SETTLE_SECONDS", 3),
            at_retry_seconds=env_int("GO2_4G_AT_RETRY_SECONDS", 20),
            dhcp_retry_seconds=env_int("GO2_4G_DHCP_RETRY_SECONDS", 10),
            dhcp_timeout_seconds=env_int("GO2_4G_DHCP_TIMEOUT_SECONDS", 25),
            health_failures=env_int("GO2_4G_HEALTH_FAILURES", 3),
            link_restart_after=env_int("GO2_4G_LINK_RESTART_AFTER", 45),
            modem_reset_after=env_int("GO2_4G_MODEM_RESET_AFTER", 180),
            modem_reset_cooldown=env_int("GO2_4G_MODEM_RESET_COOLDOWN", 300),
            rapid_usb_limit=env_int("GO2_4G_RAPID_USB_LIMIT", 3),
            rapid_usb_window=env_int("GO2_4G_RAPID_USB_WINDOW", 90),
            radio_off_seconds=env_int("GO2_4G_RADIO_OFF_SECONDS", 30),
            state_file=Path(os.environ.get(
                "GO2_4G_STATE_FILE", "/run/go2-4g-manager-state.json")),
            auto_reboot_on_fatal=env_bool(
                "GO2_4G_AUTO_REBOOT_ON_FATAL", True),
            fatal_reboot_delay_seconds=env_int(
                "GO2_4G_FATAL_REBOOT_DELAY_SECONDS", 90),
            fatal_reboot_cooldown_seconds=env_int(
                "GO2_4G_FATAL_REBOOT_COOLDOWN_SECONDS", 3600),
            reboot_marker=Path(os.environ.get(
                "GO2_4G_REBOOT_MARKER",
                "/var/lib/go2-4g-manager/last-auto-reboot.json")),
            fault_log=Path(os.environ.get(
                "GO2_4G_FAULT_LOG",
                "/home/unitree/go2_fastlio_ws/patrol_logs/service/"
                "go2-4g-faults.jsonl")),
        )

    def validate(self) -> None:
        if not re.fullmatch(r"[A-Za-z0-9_.-]{1,64}", self.xhci_device):
            raise SystemExit("invalid GO2_4G_XHCI_DEVICE")
        if not re.fullmatch(r"usb\d+", self.usb_bus_device):
            raise SystemExit("invalid GO2_4G_USB_BUS_DEVICE")
        if not safe_ecm_interface(self.interface):
            raise SystemExit("GO2_4G_ECM_INTERFACE must not be eth0 or eth1")
        if not re.fullmatch(r"[A-Za-z0-9._-]{1,64}", self.apn):
            raise SystemExit("invalid GO2_4G_APN")
        if not re.fullmatch(r"\d{1,3}(?:\.\d{1,3}){3}", self.gateway):
            raise SystemExit("invalid GO2_4G_ECM_GATEWAY")
        for value, label in ((self.usb_vid, "VID"), (self.usb_pid, "PID"),
                             (self.switch_vid, "switch VID"),
                             (self.switch_pid, "switch PID")):
            if not re.fullmatch(r"[0-9a-f]{4}", value):
                raise SystemExit(f"invalid {label}")
        if min(self.loop_seconds, self.dhcp_retry_seconds, self.health_failures,
               self.modem_reset_cooldown, self.rapid_usb_limit,
               self.rapid_usb_window, self.radio_off_seconds,
               self.fatal_reboot_delay_seconds,
               self.fatal_reboot_cooldown_seconds) <= 0:
            raise SystemExit("retry intervals and failure count must be positive")


class Manager:
    def __init__(self, config: Config) -> None:
        config.validate()
        self.cfg = config
        self.stop_requested = False
        self.state = "STARTING"
        self.reason = "manager starting"
        self.state_since = time.monotonic()
        self.last_state_log = 0.0
        self.usb_signature: Optional[str] = None
        self.usb_first_seen = 0.0
        self.cached_at_tty: Optional[str] = None
        self.last_at = 0.0
        self.last_modem_status: Dict[str, object] = {}
        self.configured_generation: Optional[str] = None
        self.dhcp_process: Optional[subprocess.Popen] = None
        self.dhcp_started_at = 0.0
        self.next_dhcp_attempt = 0.0
        self.dhcp_failures = 0
        self.health_failure_count = 0
        self.offline_since: Optional[float] = None
        self.last_link_restart = -float("inf")
        self.last_modem_reset = -float("inf")
        self.usb_generation_times = deque()  # type: ignore[var-annotated]
        self.radio_recovery_phase: Optional[str] = None
        self.radio_resume_after = 0.0
        self.radio_recovery_reason = ""
        self.usb_seen_this_boot = False
        self.usb_absent_since: Optional[float] = None
        self.xhci_fatal_since: Optional[float] = None
        self.xhci_fatal_evidence: List[str] = []
        self.fatal_event_logged = False
        self.reboot_requested = False
        self.resource_samples = deque(maxlen=24)  # type: ignore[var-annotated]
        self.previous_cpu_totals: Optional[Tuple[int, int]] = None

    @property
    def usb_root(self) -> Path:
        return self.cfg.sysfs_root / "bus/usb/devices"

    @property
    def tty_root(self) -> Path:
        return self.cfg.sysfs_root / "class/tty"

    @property
    def net_root(self) -> Path:
        return self.cfg.sysfs_root / "class/net"

    @property
    def xhci_root(self) -> Path:
        return (self.cfg.sysfs_root / "devices/platform" /
                self.cfg.xhci_device)

    def force_sysfs_value(self, path: Path, desired: str, label: str) -> None:
        """Keep a recoverability-related sysfs setting at its desired value.

        On this Jetson, repeated A7600C disconnects let tegra-xusb enter ELPG.
        The controller has twice failed to reload its Falcon firmware on the
        following resume, leaving the complete USB host in runtime_status=error
        until the Orin is rebooted.  Avoiding runtime suspend is safer than
        unbinding a controller which is already faulty; that remove path emits
        kernel IRQ warnings and cannot bind again.
        """
        current = read_text(path)
        if not current or current == desired:
            return
        try:
            path.write_text(f"{desired}\n", encoding="ascii")
        except OSError as exc:
            logging.warning("cannot force %s=%s: %s", label, desired, exc)
            return
        logging.warning("forced %s=%s (was %s)", label, desired, current)

    def force_power_control_on(self, path: Path, label: str) -> None:
        self.force_sysfs_value(path, "on", f"{label} power/control")

    def ensure_hcd_reinit_disabled(self) -> None:
        # Do not use R35.3.1's remove/probe recovery path on this Orin.  Both a
        # healthy-controller test and an already-failed-controller test emitted
        # repeated "NULL pointer, cannot free irq" warnings in tegra_xusb_remove
        # and left Falcon at 0xffffffff.  Prevent both the mailbox IRQ handler
        # and reload_hcd from entering that unsafe path.
        self.force_sysfs_value(
            self.cfg.sysfs_root
            / "module/xhci_tegra/parameters/en_hcd_reinit",
            "N",
            "xhci_tegra en_hcd_reinit",
        )

    def ensure_usb_host_awake(self) -> str:
        self.ensure_hcd_reinit_disabled()
        self.force_power_control_on(
            self.xhci_root / "power/control", "Tegra XUSB controller")
        self.force_power_control_on(
            self.usb_root / self.cfg.usb_bus_device / "power/control",
            f"USB root bus {self.cfg.usb_bus_device}",
        )
        return read_text(self.xhci_root / "power/runtime_status")

    def command(self, argv: List[str], timeout: int = 10) -> subprocess.CompletedProcess:
        try:
            return subprocess.run(
                argv,
                stdin=subprocess.DEVNULL,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                timeout=timeout,
            )
        except subprocess.TimeoutExpired as exc:
            logging.warning("command timed out: %s", argv)
            return subprocess.CompletedProcess(argv, 124, stdout=str(exc))
        except FileNotFoundError as exc:
            return subprocess.CompletedProcess(argv, 127, stdout=str(exc))

    def active_workloads(self) -> List[str]:
        """Return robot jobs during which an automatic reboot is unsafe."""
        active = set()
        proc_root = Path("/proc")
        try:
            entries = list(proc_root.iterdir())
        except OSError:
            return []
        for entry in entries:
            if not entry.name.isdigit() or entry.name == str(os.getpid()):
                continue
            try:
                command_line = (entry / "cmdline").read_bytes().replace(b"\0", b" ").decode(
                    "utf-8", errors="replace")
            except (FileNotFoundError, PermissionError, ProcessLookupError, OSError):
                continue
            for label, needle in PROTECTED_WORKLOADS.items():
                if needle in command_line:
                    active.add(label)
        return sorted(active)

    def sample_resources(self) -> Dict[str, object]:
        """Keep a short rolling record so a USB failure can be compared to load."""
        sample: Dict[str, object] = {
            "sampled_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
            "workloads": self.active_workloads(),
        }
        try:
            load = os.getloadavg()
            sample["load_1m"] = round(load[0], 2)
            sample["load_5m"] = round(load[1], 2)
            sample["load_15m"] = round(load[2], 2)
        except OSError:
            pass

        cpu_line = read_text(Path("/proc/stat")).splitlines()
        if cpu_line and cpu_line[0].startswith("cpu "):
            try:
                values = [int(value) for value in cpu_line[0].split()[1:]]
                total = sum(values)
                idle = values[3] + (values[4] if len(values) > 4 else 0)
                if self.previous_cpu_totals is not None:
                    previous_total, previous_idle = self.previous_cpu_totals
                    total_delta = total - previous_total
                    idle_delta = idle - previous_idle
                    if total_delta > 0:
                        sample["cpu_busy_percent"] = round(
                            100.0 * (total_delta - idle_delta) / total_delta, 1)
                self.previous_cpu_totals = (total, idle)
            except (ValueError, IndexError):
                pass

        meminfo: Dict[str, int] = {}
        for line in read_text(Path("/proc/meminfo")).splitlines():
            match = re.match(r"^(MemTotal|MemAvailable):\s+(\d+)", line)
            if match:
                meminfo[match.group(1)] = int(match.group(2))
        if meminfo:
            sample["memory_total_mb"] = round(meminfo.get("MemTotal", 0) / 1024, 1)
            sample["memory_available_mb"] = round(
                meminfo.get("MemAvailable", 0) / 1024, 1)

        temperatures: List[float] = []
        thermal_root = self.cfg.sysfs_root / "class/thermal"
        try:
            zones = list(thermal_root.glob("thermal_zone*/temp"))
        except OSError:
            zones = []
        for path in zones:
            try:
                value = float(read_text(path))
            except ValueError:
                continue
            temperatures.append(value / 1000.0 if abs(value) >= 1000 else value)
        if temperatures:
            sample["max_temperature_c"] = round(max(temperatures), 1)
        self.resource_samples.append(sample)
        return sample

    def kernel_log_text(self) -> str:
        dmesg = next((path for path in ("/bin/dmesg", "/usr/bin/dmesg")
                      if Path(path).exists()), "dmesg")
        result = self.command([dmesg, "--color=never"], timeout=5)
        return result.stdout or "" if result.returncode == 0 else ""

    def fatal_kernel_evidence(self) -> List[str]:
        text = self.kernel_log_text()
        if not any(pattern.search(text) for pattern in XHCI_FATAL_PATTERNS):
            return []
        context = [line.strip() for line in text.splitlines()
                   if XHCI_CONTEXT_PATTERN.search(line)]
        return context[-30:]

    def append_fault_event(self, event: str, **details: object) -> None:
        payload: Dict[str, object] = {
            "event": event,
            "recorded_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
            "boot_id": read_text(Path("/proc/sys/kernel/random/boot_id")),
            "manager_version": MANAGER_VERSION,
            "interface": self.cfg.interface,
        }
        payload.update(details)
        try:
            self.cfg.fault_log.parent.mkdir(parents=True, exist_ok=True)
            with self.cfg.fault_log.open("a", encoding="utf-8") as handle:
                json.dump(payload, handle, ensure_ascii=False, sort_keys=True)
                handle.write("\n")
                handle.flush()
                os.fsync(handle.fileno())
        except OSError as exc:
            logging.warning("cannot append 4G fault log %s: %s", self.cfg.fault_log, exc)

    def reboot_cooldown_remaining(self) -> float:
        try:
            marker = json.loads(read_text(self.cfg.reboot_marker))
            previous = float(marker["requested_at_unix"])
        except (ValueError, TypeError, KeyError, json.JSONDecodeError):
            return 0.0
        age = time.time() - previous
        if age < 0:
            # The Jetson RTC may be stale before network time is restored.
            return float(self.cfg.fatal_reboot_cooldown_seconds)
        return max(0.0, self.cfg.fatal_reboot_cooldown_seconds - age)

    def write_reboot_marker(self, reason: str) -> None:
        payload = {
            "requested_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
            "requested_at_unix": time.time(),
            "boot_id": read_text(Path("/proc/sys/kernel/random/boot_id")),
            "reason": reason,
        }
        self.cfg.reboot_marker.parent.mkdir(parents=True, exist_ok=True)
        fd, temporary = tempfile.mkstemp(
            prefix=self.cfg.reboot_marker.name + ".",
            dir=str(self.cfg.reboot_marker.parent),
        )
        os.fchmod(fd, 0o600)
        with os.fdopen(fd, "w", encoding="utf-8") as handle:
            json.dump(payload, handle, ensure_ascii=False, sort_keys=True)
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, self.cfg.reboot_marker)

    def handle_fatal_controller(self, now: float, controller_status: str) -> None:
        if self.xhci_fatal_since is None:
            self.xhci_fatal_since = now
        fatal_age = now - self.xhci_fatal_since
        workloads = self.active_workloads()
        cooldown = self.reboot_cooldown_remaining()
        if not self.fatal_event_logged:
            self.append_fault_event(
                "xhci_fatal_detected",
                reason="Jetson XUSB host controller stopped responding",
                controller_runtime_status=controller_status or "unknown",
                usb_seen_this_boot=self.usb_seen_this_boot,
                kernel_evidence=self.xhci_fatal_evidence,
                recent_resources=list(self.resource_samples),
                workloads=workloads,
            )
            self.fatal_event_logged = True

        recovery = "waiting before controlled reboot"
        if not self.cfg.auto_reboot_on_fatal:
            recovery = "automatic reboot is disabled"
        elif workloads:
            recovery = "automatic reboot deferred while a robot job is active"
        elif cooldown > 0:
            recovery = "automatic reboot suppressed by persistent cooldown"
        elif fatal_age < self.cfg.fatal_reboot_delay_seconds:
            recovery = "waiting before controlled reboot"
        elif self.reboot_requested:
            recovery = "controlled reboot has already been requested"
        else:
            reason = "fatal Tegra XUSB controller failure removed the 4G modem"
            try:
                self.write_reboot_marker(reason)
            except OSError as exc:
                logging.error("cannot persist automatic reboot marker: %s", exc)
                recovery = "automatic reboot blocked because cooldown marker could not be saved"
            else:
                self.reboot_requested = True
                self.append_fault_event(
                    "controlled_reboot_requested",
                    reason=reason,
                    fatal_age_seconds=round(fatal_age, 1),
                    kernel_evidence=self.xhci_fatal_evidence,
                    recent_resources=list(self.resource_samples),
                )
                self.set_state(
                    "USB_FATAL_REBOOTING",
                    "fatal Jetson XUSB failure; requesting one controlled reboot",
                    controller_runtime_status=controller_status or "unknown",
                    reboot_required=True,
                    automatic_reboot=True,
                    fatal_age_seconds=round(fatal_age, 1),
                )
                logging.critical("requesting controlled reboot after fatal Tegra XUSB failure")
                result = self.command(["/bin/systemctl", "reboot"], timeout=10)
                if result.returncode != 0:
                    logging.error("controlled reboot request failed: %s", result.stdout)
                    self.append_fault_event(
                        "controlled_reboot_failed",
                        returncode=result.returncode,
                        output=(result.stdout or "")[-1000:],
                    )
                return

        self.set_state(
            "USB_CONTROLLER_FATAL",
            "Jetson XUSB controller stopped responding; reboot is required",
            controller_runtime_status=controller_status or "unknown",
            reboot_required=True,
            automatic_reboot_enabled=self.cfg.auto_reboot_on_fatal,
            automatic_reboot_after_seconds=self.cfg.fatal_reboot_delay_seconds,
            fatal_age_seconds=round(fatal_age, 1),
            cooldown_remaining_seconds=round(cooldown, 1),
            recovery=recovery,
            protected_workloads=workloads,
            kernel_evidence=self.xhci_fatal_evidence[-8:],
        )

    def set_state(self, state: str, reason: str, **details: object) -> None:
        now = time.monotonic()
        changed = state != self.state or reason != self.reason
        if state != self.state:
            self.state_since = now
        self.state, self.reason = state, reason
        if changed or now - self.last_state_log >= 60:
            logging.info("state=%s reason=%s details=%s", state, reason,
                         json.dumps(details, ensure_ascii=False, sort_keys=True))
            self.last_state_log = now
        payload: Dict[str, object] = {
            "state": state,
            "reason": reason,
            "updated_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
            "state_age_seconds": round(now - self.state_since, 1),
            "mode": "ecm-auto",
            "manager_version": MANAGER_VERSION,
            "interface": self.cfg.interface,
            "usb_signature": self.usb_signature,
            "health_failures": self.health_failure_count,
            "dhcp_failures": self.dhcp_failures,
            "modem": self.last_modem_status,
        }
        payload.update(details)
        self.write_state(payload)

    def write_state(self, payload: Dict[str, object]) -> None:
        try:
            self.cfg.state_file.parent.mkdir(parents=True, exist_ok=True)
            fd, temporary = tempfile.mkstemp(
                prefix=self.cfg.state_file.name + ".",
                dir=str(self.cfg.state_file.parent),
            )
            os.fchmod(fd, 0o644)
            with os.fdopen(fd, "w", encoding="utf-8") as handle:
                json.dump(payload, handle, ensure_ascii=False, sort_keys=True)
                handle.write("\n")
            os.replace(temporary, self.cfg.state_file)
        except OSError as exc:
            logging.warning("cannot write state file %s: %s", self.cfg.state_file, exc)

    def find_usb(self, vid: str, pid: str) -> Optional[Path]:
        if not self.usb_root.exists():
            return None
        for entry in sorted(self.usb_root.iterdir()):
            if ":" in entry.name:
                continue
            if (read_text(entry / "idVendor").lower() == vid and
                    read_text(entry / "idProduct").lower() == pid):
                return entry
        return None

    @staticmethod
    def usb_signature_for(device: Path) -> str:
        return (f"{device.name}@{read_text(device / 'busnum') or '?'}:"
                f"{read_text(device / 'devnum') or '?'}")

    def note_usb_generation(self, device: Path, now: float) -> None:
        signature = self.usb_signature_for(device)
        if signature == self.usb_signature:
            return
        self.stop_dhcp("USB generation changed")
        self.usb_signature = signature
        self.usb_first_seen = now
        self.cached_at_tty = None
        self.last_at = 0.0
        self.last_modem_status = {}
        self.configured_generation = None
        self.next_dhcp_attempt = now
        self.dhcp_failures = 0
        self.health_failure_count = 0
        self.offline_since = None
        self.usb_generation_times.append(now)
        while (self.usb_generation_times and
               now - self.usb_generation_times[0] > self.cfg.rapid_usb_window):
            self.usb_generation_times.popleft()
        logging.info("new A7600C USB generation: %s", signature)

    def tty_map(self, device: Path) -> List[Tuple[str, str]]:
        result: List[Tuple[str, str]] = []
        if not self.tty_root.exists():
            return result
        for tty in self.tty_root.glob("ttyUSB*"):
            try:
                usb_interface = (tty / "device").resolve().parent
            except OSError:
                continue
            if not usb_interface.name.startswith(device.name + ":"):
                continue
            result.append((read_text(usb_interface / "bInterfaceNumber").lower(),
                           f"/dev/{tty.name}"))
        return result

    def at_candidates(self, device: Path) -> List[Tuple[str, str]]:
        preference = {"05": 0, "03": 1, "02": 2, "04": 3}
        candidates = [item for item in self.tty_map(device) if item[0] in preference]
        candidates.sort(key=lambda item: preference[item[0]])
        if self.cached_at_tty:
            candidates.sort(key=lambda item: item[1] != self.cached_at_tty)
        return candidates

    @staticmethod
    def at_exchange(tty: str, commands: Iterable[str], timeout: float = 1.0) -> str:
        fd = os.open(tty, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            attrs = termios.tcgetattr(fd)
            attrs[0] = attrs[1] = attrs[3] = 0
            attrs[2] = termios.CS8 | termios.CLOCAL | termios.CREAD
            attrs[4] = attrs[5] = termios.B115200
            termios.tcsetattr(fd, termios.TCSANOW, attrs)
            termios.tcflush(fd, termios.TCIOFLUSH)
            transcript = ""
            for command in commands:
                os.write(fd, (command + "\r").encode("ascii"))
                deadline = time.monotonic() + timeout
                response = b""
                while time.monotonic() < deadline:
                    readable, _, _ = select.select(
                        [fd], [], [], min(0.1, max(0.0, deadline - time.monotonic())))
                    if not readable:
                        continue
                    try:
                        response += os.read(fd, 4096)
                    except BlockingIOError:
                        continue
                    decoded = response.decode("ascii", errors="replace")
                    if "\r\nOK\r\n" in decoded or "\r\nERROR\r\n" in decoded:
                        break
                transcript += response.decode("ascii", errors="replace")
            return transcript
        finally:
            os.close(fd)

    def modem_status(self, device: Path, force: bool = False) -> Dict[str, object]:
        now = time.monotonic()
        if not force and self.last_modem_status and now - self.last_at < self.cfg.at_retry_seconds:
            return self.last_modem_status
        self.last_at = now
        for number, tty in self.at_candidates(device):
            try:
                if "OK" not in self.at_exchange(tty, ["AT"], timeout=0.7):
                    continue
                transcript = self.at_exchange(
                    tty,
                    ["AT+CPIN?", "AT+CSQ", "AT+CEREG?", "AT+DIALMODE?", "AT$MYCONFIG?"],
                    timeout=1.0,
                )
                cereg = parse_cereg(transcript)
                usbnet_mode, usbnet_port = parse_usbnet_config(transcript)
                status: Dict[str, object] = {
                    "responsive": True,
                    "tty": tty,
                    "usb_interface": number,
                    "sim_ready": "+CPIN: READY" in transcript,
                    "cereg": cereg,
                    "registered": None if cereg is None else cereg in REGISTERED_CEREG_STATES,
                    "csq": parse_csq(transcript),
                    "dial_mode": parse_dial_mode(transcript),
                    "usbnet_mode": usbnet_mode,
                    "usbnet_port": usbnet_port,
                }
                self.cached_at_tty = tty
                self.last_modem_status = status
                logging.info("AT status=%s", json.dumps(status, sort_keys=True))
                return status
            except (OSError, termios.error) as exc:
                logging.debug("AT probe failed tty=%s: %s", tty, exc)
        self.cached_at_tty = None
        self.last_modem_status = {"responsive": False}
        return self.last_modem_status

    def ensure_vendor_linux_mode(self, status: Dict[str, object]) -> bool:
        if self.configured_generation == self.usb_signature:
            return True
        if not status.get("responsive") or not status.get("tty"):
            return False
        tty = str(status["tty"])
        if status.get("dial_mode") != 0:
            logging.warning("restoring vendor auto dial mode (AT+DIALMODE=0)")
            response = self.at_exchange(
                tty,
                [f'AT+CGDCONT=1,"IP","{self.cfg.apn}"', "AT+DIALMODE=0"],
                timeout=1.5,
            )
            if response.count("OK") < 2:
                logging.warning("auto dial configuration was not fully acknowledged")
                return False
            status["dial_mode"] = 0
        if status.get("usbnet_mode") != 1:
            logging.warning("restoring vendor Linux ECM composition")
            response = self.at_exchange(
                tty, ['AT$MYCONFIG="usbnetmode",1,0'], timeout=2.0)
            if "OK" not in response:
                logging.warning("ECM composition was not acknowledged")
                return False
            self.set_state(
                "USB_MODE_SWITCH",
                "A7600C is switching to the vendor Linux ECM composition",
                modem=status,
            )
            return False
        self.configured_generation = self.usb_signature
        return True

    def find_modem_net_interface(self, device: Path) -> Optional[str]:
        try:
            device_real = device.resolve()
        except OSError:
            return None
        if not self.net_root.exists():
            return None
        for entry in sorted(self.net_root.iterdir()):
            if entry.name == "lo":
                continue
            try:
                actual = (entry / "device").resolve()
            except OSError:
                continue
            if device_real == actual or device_real in actual.parents:
                return entry.name
        return None

    def ensure_interface_name(self, device: Path) -> Optional[str]:
        raw = self.find_modem_net_interface(device)
        if raw is None:
            return None
        if raw == self.cfg.interface:
            return raw
        if (self.net_root / self.cfg.interface).exists():
            logging.error("refusing to rename modem: %s already exists", self.cfg.interface)
            return None
        # The ancestry check above proves this temporary name belongs to the
        # A7600C, not to a robot Ethernet controller.
        self.command(["/usr/sbin/ip", "link", "set", "dev", raw, "down"])
        result = self.command(
            ["/usr/sbin/ip", "link", "set", "dev", raw, "name", self.cfg.interface])
        if result.returncode != 0:
            logging.warning("cannot rename A7600C interface %s: %s", raw, result.stdout)
            return None
        logging.info("renamed A7600C USB network interface %s to %s", raw, self.cfg.interface)
        return self.cfg.interface

    def ipv4_addresses(self) -> List[str]:
        if not safe_ecm_interface(self.cfg.interface):
            return []
        result = self.command(
            ["/usr/sbin/ip", "-o", "-4", "addr", "show", "dev", self.cfg.interface])
        if result.returncode != 0:
            return []
        return [address for address in re.findall(r"\binet\s+([^\s/]+)", result.stdout or "")
                if not address.startswith("169.254.")]

    def start_dhcp(self, now: float) -> bool:
        if self.dhcp_process is not None or now < self.next_dhcp_attempt:
            return False
        dhclient = next((path for path in ("/sbin/dhclient", "/usr/sbin/dhclient")
                         if Path(path).exists()), None)
        if dhclient is None:
            logging.error("dhclient is not installed")
            self.next_dhcp_attempt = now + self.cfg.dhcp_retry_seconds
            return False
        self.command(["/usr/sbin/ip", "link", "set", "dev", self.cfg.interface, "up"])
        argv = [
            dhclient, "-4", "-1", "-d", "-v",
            "-pf", f"/run/go2-4g-dhclient-{self.cfg.interface}.pid",
            "-lf", f"/var/lib/dhcp/dhclient.{self.cfg.interface}.leases",
            self.cfg.interface,
        ]
        try:
            self.dhcp_process = subprocess.Popen(argv, stdin=subprocess.DEVNULL)
        except OSError as exc:
            logging.error("cannot start dhclient: %s", exc)
            self.next_dhcp_attempt = now + self.cfg.dhcp_retry_seconds
            return False
        self.dhcp_started_at = now
        self.next_dhcp_attempt = now + self.cfg.dhcp_retry_seconds
        logging.info("started scoped DHCP pid=%s interface=%s",
                     self.dhcp_process.pid, self.cfg.interface)
        return True

    def poll_dhcp(self, now: float) -> None:
        if self.dhcp_process is None:
            return
        code = self.dhcp_process.poll()
        if code is None:
            if self.ipv4_addresses():
                self.stop_dhcp("DHCP lease acquired")
                self.dhcp_failures = 0
                return
            if now - self.dhcp_started_at >= self.cfg.dhcp_timeout_seconds:
                self.stop_dhcp("DHCP timeout")
                self.dhcp_failures += 1
            return
        logging.info("dhclient exited rc=%s after %.1fs", code, now - self.dhcp_started_at)
        self.dhcp_process = None
        if not self.ipv4_addresses():
            self.dhcp_failures += 1
            self.next_dhcp_attempt = now + min(
                60, self.cfg.dhcp_retry_seconds * max(1, self.dhcp_failures))

    def stop_dhcp(self, reason: str) -> None:
        process = self.dhcp_process
        self.dhcp_process = None
        if process is None or process.poll() is not None:
            return
        logging.info("stopping dhclient pid=%s reason=%s", process.pid, reason)
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=2)

    def renew_link(self, now: float, reason: str) -> None:
        self.stop_dhcp(reason)
        if safe_ecm_interface(self.cfg.interface):
            self.command(["/usr/sbin/ip", "addr", "flush", "dev", self.cfg.interface])
            self.command(["/usr/sbin/ip", "link", "set", "dev", self.cfg.interface, "down"])
            time.sleep(1)
            self.command(["/usr/sbin/ip", "link", "set", "dev", self.cfg.interface, "up"])
        self.next_dhcp_attempt = now + 1

    def ensure_default_route(self) -> None:
        if not self.ipv4_addresses():
            return
        result = self.command(
            ["/usr/sbin/ip", "route", "show", "default", "dev", self.cfg.interface])
        if result.returncode == 0 and (result.stdout or "").strip():
            return
        self.command([
            "/usr/sbin/ip", "route", "replace", "default", "via", self.cfg.gateway,
            "dev", self.cfg.interface, "metric", "50",
        ])

    def tcp_check(self, host: str, port: int, timeout: float = 3.0) -> bool:
        sock: Optional[socket.socket] = None
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(timeout)
            if sys.platform.startswith("linux"):
                sock.setsockopt(socket.SOL_SOCKET, 25,
                                self.cfg.interface.encode("ascii") + b"\0")
            sock.connect((host, port))
            return True
        except OSError:
            return False
        finally:
            if sock is not None:
                sock.close()

    def start_radio_recovery(self, status: Dict[str, object], now: float,
                             reason: str) -> bool:
        if (not status.get("responsive") or not status.get("tty") or
                now - self.last_modem_reset < self.cfg.modem_reset_cooldown):
            return False
        self.stop_dhcp(reason)
        if (safe_ecm_interface(self.cfg.interface) and
                (self.net_root / self.cfg.interface).exists()):
            self.command(["/usr/sbin/ip", "addr", "flush", "dev", self.cfg.interface])
        self.radio_recovery_phase = "off_wait"
        self.radio_resume_after = now + self.cfg.radio_off_seconds
        self.radio_recovery_reason = reason
        self.set_state("RADIO_RECOVERY_OFF", reason, modem=status,
                       resume_in_seconds=self.cfg.radio_off_seconds)
        logging.warning("turning A7600C radio off to clear a stuck baseband state")
        try:
            response = self.at_exchange(str(status["tty"]), ["AT+CFUN=0"], timeout=2.0)
        except (OSError, termios.error):
            self.radio_recovery_phase = None
            return False
        if "OK" not in response:
            self.radio_recovery_phase = None
            return False
        self.last_modem_reset = now
        return True

    def continue_radio_recovery(self, device: Path, now: float) -> bool:
        if self.radio_recovery_phase != "off_wait":
            return False
        remaining = self.radio_resume_after - now
        if remaining > 0:
            self.set_state("RADIO_RECOVERY_WAIT",
                           "A7600C radio is off while the baseband fault clears",
                           resume_in_seconds=round(remaining, 1))
            return True
        status = self.modem_status(device, force=True)
        if not status.get("responsive") or not status.get("tty"):
            self.set_state("RADIO_RECOVERY_WAIT",
                           "waiting for an AT port before restoring the A7600C radio")
            return True
        try:
            response = self.at_exchange(str(status["tty"]), ["AT+CFUN=1"], timeout=2.0)
        except (OSError, termios.error):
            return True
        if "OK" not in response:
            return True
        logging.warning("A7600C radio restored after controlled baseband recovery")
        self.radio_recovery_phase = None
        self.usb_generation_times.clear()
        self.health_failure_count = 0
        self.offline_since = None
        self.next_dhcp_attempt = now + 3
        self.set_state("RADIO_RECOVERY_ON",
                       "A7600C radio restored; waiting for registration and DHCP",
                       modem=status)
        return True

    def tick(self) -> None:
        now = time.monotonic()
        self.sample_resources()
        self.poll_dhcp(now)
        controller_status = self.ensure_usb_host_awake()
        device = self.find_usb(self.cfg.usb_vid, self.cfg.usb_pid)
        switch_device = self.find_usb(self.cfg.switch_vid, self.cfg.switch_pid)
        if device is None:
            if self.usb_absent_since is None:
                self.usb_absent_since = now
            self.stop_dhcp("A7600C absent")
            self.usb_signature = None
            self.cached_at_tty = None
            self.last_modem_status = {}
            self.configured_generation = None
            self.health_failure_count = 0
            self.offline_since = None
            evidence = self.fatal_kernel_evidence()
            if evidence:
                self.xhci_fatal_evidence = evidence
            if controller_status in {"error", "unsupported"} or self.xhci_fatal_evidence:
                self.handle_fatal_controller(now, controller_status)
            elif switch_device is not None:
                self.set_state("USB_SWITCHING", "A7600C WUKONG recovery identity is present")
            else:
                self.set_state(
                    "USB_ABSENT",
                    "A7600C USB device is absent; the awake host is waiting for re-enumeration",
                    controller_runtime_status=controller_status or "unknown",
                )
            return

        if self.xhci_fatal_since is not None:
            self.append_fault_event(
                "usb_recovered_without_reboot",
                usb_signature=self.usb_signature_for(device),
                kernel_evidence=self.xhci_fatal_evidence,
            )
        self.usb_seen_this_boot = True
        self.usb_absent_since = None
        self.xhci_fatal_since = None
        self.xhci_fatal_evidence = []
        self.fatal_event_logged = False
        self.reboot_requested = False
        self.force_power_control_on(
            device / "power/control", "A7600C USB device")
        self.note_usb_generation(device, now)
        usb_age = now - self.usb_first_seen
        if usb_age < self.cfg.usb_settle_seconds:
            self.set_state("USB_SETTLING", "new A7600C enumeration is settling",
                           usb_age_seconds=round(usb_age, 1))
            return

        if self.continue_radio_recovery(device, now):
            return

        status = self.modem_status(device)
        if not self.ensure_vendor_linux_mode(status):
            self.set_state("MODE_CONFIG", "restoring A7600C vendor Linux auto-connect mode",
                           modem=status)
            return
        if status.get("responsive") and not status.get("sim_ready", True):
            self.set_state("SIM_WAIT", "SIM is not ready", modem=status)
            return

        while (self.usb_generation_times and
               now - self.usb_generation_times[0] > self.cfg.rapid_usb_window):
            self.usb_generation_times.popleft()
        if (len(self.usb_generation_times) >= self.cfg.rapid_usb_limit and
                self.start_radio_recovery(
                    status,
                    now,
                    (f"A7600C re-enumerated {len(self.usb_generation_times)} times "
                     f"within {self.cfg.rapid_usb_window}s"),
                )):
            return

        interface = self.ensure_interface_name(device)
        if interface is None:
            self.set_state("ECM_WAIT", "waiting for the A7600C ECM network interface",
                           modem=status)
            return

        addresses = self.ipv4_addresses()
        if not addresses:
            if self.dhcp_process is None:
                self.start_dhcp(now)
            reason = ("cellular registration is pending" if status.get("registered") is False
                      else "requesting an address from the A7600C")
            self.set_state("CONNECTING", reason, modem=status)
            return

        self.ensure_default_route()
        if self.tcp_check(self.cfg.cloud_host, self.cfg.cloud_port):
            self.health_failure_count = 0
            self.dhcp_failures = 0
            self.offline_since = None
            self.set_state("ONLINE", "4G ECM can reach the command server",
                           addresses=addresses,
                           cloud=f"{self.cfg.cloud_host}:{self.cfg.cloud_port}")
            return
        if self.tcp_check(self.cfg.fallback_host, self.cfg.fallback_port):
            self.health_failure_count = 0
            self.offline_since = None
            self.set_state("CLOUD_UNREACHABLE",
                           "4G internet works but the command server is unreachable",
                           addresses=addresses,
                           cloud=f"{self.cfg.cloud_host}:{self.cfg.cloud_port}")
            return

        self.health_failure_count += 1
        if self.offline_since is None:
            self.offline_since = now
        offline_age = now - self.offline_since
        if self.health_failure_count < self.cfg.health_failures:
            self.set_state("DEGRADED", "4G health check failed; confirming before recovery",
                           addresses=addresses, offline_seconds=round(offline_age, 1))
            return
        status = self.modem_status(device, force=True)
        if status.get("responsive") and status.get("registered") is False:
            self.set_state("REGISTERING", "cellular registration was lost; modem auto dial will retry",
                           addresses=addresses, offline_seconds=round(offline_age, 1), modem=status)
        else:
            self.set_state("LINK_RECOVERY", "4G is offline; recycling only go2_4g DHCP",
                           addresses=addresses, offline_seconds=round(offline_age, 1), modem=status)
        if (offline_age >= self.cfg.link_restart_after and
                now - self.last_link_restart >= self.cfg.link_restart_after):
            self.last_link_restart = now
            self.renew_link(now, "4G health recovery")
        if offline_age >= self.cfg.modem_reset_after:
            self.start_radio_recovery(status, now, "prolonged 4G outage")

    def run(self, once: bool = False) -> int:
        def request_stop(_signum: int, _frame: object) -> None:
            self.stop_requested = True

        signal.signal(signal.SIGTERM, request_stop)
        signal.signal(signal.SIGINT, request_stop)
        try:
            while not self.stop_requested:
                try:
                    self.tick()
                except Exception:
                    logging.exception("manager tick failed")
                if once:
                    break
                for _ in range(self.cfg.loop_seconds * 10):
                    if self.stop_requested:
                        break
                    time.sleep(0.1)
        finally:
            self.stop_dhcp("manager stopped")
            self.set_state("STOPPED", "manager stopped")
        return 0


def self_test() -> int:
    assert parse_cereg("+CEREG: 0,1") == 1
    assert parse_cereg("+CEREG: 5") == 5
    assert parse_cereg("ERROR") is None
    assert parse_csq("+CSQ: 23,99") == 23
    assert parse_dial_mode("+DIALMODE: 0") == 0
    assert parse_usbnet_config('$MYCONFIG: "usbnetmode",1,0') == (1, 0)
    assert parse_usbnet_config('$MYCONFIG: "usbnetmode",1') == (1, None)
    assert safe_ecm_interface("go2_4g")
    assert not safe_ecm_interface("eth0")
    assert not safe_ecm_interface("eth1")
    cfg = Config.from_env()
    cfg.validate()
    with tempfile.TemporaryDirectory(prefix="go2-4g-sysfs-") as temporary:
        root = Path(temporary)
        xhci_power = root / "devices/platform/3610000.xhci/power"
        bus_power = root / "bus/usb/devices/usb1/power"
        reinit_parameter = (
            root / "module/xhci_tegra/parameters/en_hcd_reinit")
        xhci_power.mkdir(parents=True)
        bus_power.mkdir(parents=True)
        reinit_parameter.parent.mkdir(parents=True)
        (xhci_power / "control").write_text("auto\n", encoding="ascii")
        (xhci_power / "runtime_status").write_text("active\n", encoding="ascii")
        (bus_power / "control").write_text("auto\n", encoding="ascii")
        reinit_parameter.write_text("Y\n", encoding="ascii")
        test_cfg = Config(
            **{**cfg.__dict__, "sysfs_root": root,
               "state_file": root / "manager-state.json",
               "auto_reboot_on_fatal": False,
               "reboot_marker": root / "last-auto-reboot.json",
               "fault_log": root / "faults.jsonl"}
        )
        test_manager = Manager(test_cfg)
        assert test_manager.ensure_usb_host_awake() == "active"
        assert read_text(xhci_power / "control") == "on"
        assert read_text(bus_power / "control") == "on"
        assert read_text(reinit_parameter) == "N"
        test_manager.kernel_log_text = lambda: (  # type: ignore[method-assign]
            "tegra-xusb 3610000.xhci: controller error detected\n"
            "xhci-hcd 3610000.xhci: xHCI host controller not responding, assume dead\n"
            "xhci-hcd 3610000.xhci: HC died; cleaning up\n"
        )
        test_manager.tick()
        assert test_manager.state == "USB_CONTROLLER_FATAL"
        state = json.loads(read_text(test_cfg.state_file))
        assert state["reboot_required"] is True
        assert state["automatic_reboot_enabled"] is False
        events = [json.loads(line) for line in read_text(test_cfg.fault_log).splitlines()]
        assert events[0]["event"] == "xhci_fatal_detected"
    print("go2_4g_manager ECM self-test: PASS")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--once", action="store_true")
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--debug", action="store_true")
    args = parser.parse_args()
    logging.basicConfig(
        level=logging.DEBUG if args.debug else logging.INFO,
        format="%(asctime)s %(levelname)s go2-4g-manager %(message)s",
    )
    if args.self_test:
        return self_test()
    return Manager(Config.from_env()).run(once=args.once)


if __name__ == "__main__":
    raise SystemExit(main())

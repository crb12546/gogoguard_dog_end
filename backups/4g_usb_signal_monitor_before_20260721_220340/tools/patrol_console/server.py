#!/usr/bin/env python3
"""Go2 巡检现场管理台（本地运行）。

设计原则:
- 不修改狗(Orin)上的任何文件, 只通过 SSH 调用已存在的脚本/命令。
- 遥测脚本通过 stdin 传给远端 python3 运行, 不在狗上落盘。
- 只监听 127.0.0.1, 仅本机浏览器可访问。

用法:
    pip install -r requirements.txt
    python3 server.py
    浏览器打开 http://127.0.0.1:8642
"""
import json
import base64
import os
import re
import shlex
import struct
import subprocess
import threading
import time
from pathlib import Path
from typing import Optional

import uvicorn
from fastapi import FastAPI, Request
from fastapi.responses import FileResponse, JSONResponse, Response
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel

# ---------------------------------------------------------------- 配置
HOSTS = ["go2wired", "go2", "go2home"]   # SSH 别名: 网线最快优先, 热点次之
PORT = 8642
WS = "/home/unitree/go2_fastlio_ws"
ROUTES_DIR = f"{WS}/src/go2_fastlio_patrol/routes"
PCD_DIR = f"{WS}/maps/console"
VIDEO_DIR = f"{WS}/patrol_logs/videos"
PATROL_VIDEO_ACTIVE_FILE = f"{WS}/patrol_logs/run/patrol_video.active"
Z1PRO_CAPTURE = f"{WS}/scripts/z1pro_capture.sh"
Z1PRO_GCU = f"{WS}/scripts/z1pro_gcu_control.py"
Z1PRO_PRESET = f"{WS}/scripts/z1pro_preset.sh"
SAAS_AGENT = f"{WS}/scripts/go2_saas_agent.py"
SAAS_LOOP_RUN_FILE = "/tmp/go2_saas_loop.run"
ENV = f"source {WS}/scripts/env_common.sh >/dev/null 2>&1 || true"
NAME_RE = re.compile(r"^[A-Za-z0-9_\-]{1,40}$")
ROUTE_PATH_RE = re.compile(r"^[A-Za-z0-9_\-/\.]{1,200}\.csv$")
PCD_PATH_RE = re.compile(r"^[A-Za-z0-9_\-/\.]{1,200}\.pcd$")
MEDIA_PATH_RE = re.compile(r"^[A-Za-z0-9_\-/\.]{1,220}\.(mp4|jpg|jpeg)$")
COMMAND_ID_RE = re.compile(r"^[A-Za-z0-9_:\.\-]{1,80}$")

STATIC_DIR = Path(__file__).parent / "static"

# ---------------------------------------------------------------- 共享状态
LOCK = threading.Lock()
STATE = {
    "host": None,              # 当前使用的 SSH 别名
    "ssh_ok": False,
    "telemetry": {},           # 最新一帧遥测
    "telemetry_age": None,     # 距最新遥测的秒数
    "processes": {},           # 远端关键进程运行状态
    "recorder_file": None,     # 正在录制的文件(由本端记录)
    "recorder_lines": None,
    "recorder_progress": None, # 录制器最后一条状态/失败日志
    "pcd_progress": None,      # 点云累积进度文本
    "follower_status": {},     # 巡线状态摘要
    "safe_status": {},         # 安全节点状态摘要
    "sys": {},                 # Orin 健康: cpu_temp / wifi_dbm
    "actions_log": [],         # 最近动作结果
}


def log_action(text: str):
    with LOCK:
        STATE["actions_log"].append({"t": time.strftime("%H:%M:%S"), "text": text})
        STATE["actions_log"] = STATE["actions_log"][-50:]


# ---------------------------------------------------------------- SSH 基础
def ssh_run(host: str, cmd: str, timeout: float = 12):
    try:
        r = subprocess.run(
            ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=5", host, cmd],
            capture_output=True, text=True, timeout=timeout,
        )
        return r.returncode, r.stdout.strip(), r.stderr.strip()
    except subprocess.TimeoutExpired:
        return 124, "", "ssh timeout"
    except Exception as e:  # noqa: BLE001
        return 255, "", str(e)


def detect_host() -> Optional[str]:
    for h in HOSTS:
        rc, out, _ = ssh_run(h, "echo ok", timeout=8)
        if rc == 0 and out == "ok":
            return h
    return None


def current_host() -> Optional[str]:
    with LOCK:
        return STATE["host"]


def require_host() -> Optional[str]:
    h = current_host()
    if h:
        return h
    h = detect_host()
    with LOCK:
        STATE["host"] = h
        STATE["ssh_ok"] = h is not None
    return h


# ---------------------------------------------------------------- 遥测流(远端脚本经 stdin 注入, 不落盘)
TELEMETRY_SCRIPT = r"""
import json, math, time
import rclpy
from rclpy.node import Node
from unitree_go.msg import LowState, SportModeState
from nav_msgs.msg import Odometry


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class T(Node):
    def __init__(self):
        super().__init__('patrol_console_telemetry')
        self.low = None
        self.low_t = 0.0
        self.sport = None
        self.sport_t = 0.0
        self.odom = None
        self.odom_t = 0.0
        self.create_subscription(LowState, '/lf/lowstate', self.cb_low, 5)
        self.create_subscription(SportModeState, '/lf/sportmodestate', self.cb_sport, 5)
        self.create_subscription(Odometry, '/Odometry', self.cb_odom, 5)

    def cb_low(self, m):
        self.low = m
        self.low_t = time.time()

    def cb_sport(self, m):
        self.sport = m
        self.sport_t = time.time()

    def cb_odom(self, m):
        self.odom = m
        self.odom_t = time.time()


rclpy.init()
n = T()
last = 0.0
while rclpy.ok():
    rclpy.spin_once(n, timeout_sec=0.2)
    now = time.time()
    if now - last < 1.0:
        continue
    last = now
    out = {"ts": now}
    if n.low is not None:
        m = n.low
        temps = [int(x.temperature) for x in m.motor_state[:12]]
        b = m.bms_state
        out["low"] = {
            "age": round(now - n.low_t, 1),
            "power_v": round(float(m.power_v), 2),
            "power_a": round(float(m.power_a), 2),
            "ntc1": int(m.temperature_ntc1),
            "ntc2": int(m.temperature_ntc2),
            "soc": int(b.soc),
            "bms_current": int(b.current),
            "bq_ntc": [int(x) for x in b.bq_ntc],
            "motor_temps": temps,
            "motor_max": max(temps) if temps else None,
            "foot_force": [int(x) for x in m.foot_force],
        }
    if n.sport is not None:
        sp = n.sport
        out["sport"] = {
            "age": round(now - n.sport_t, 1),
            "error_code": int(sp.error_code),
            "mode": int(sp.mode),
            "gait": int(sp.gait_type),
            "vx": round(float(sp.velocity[0]), 2),
            "vyaw": round(float(sp.yaw_speed), 2),
            "body_h": round(float(sp.body_height), 2),
        }
    if n.odom is not None:
        p = n.odom.pose.pose
        out["odom"] = {
            "age": round(now - n.odom_t, 1),
            "x": round(p.position.x, 3),
            "y": round(p.position.y, 3),
            "z": round(p.position.z, 3),
            "yaw": round(yaw_of(p.orientation), 3),
        }
    print(json.dumps(out), flush=True)
"""

REMOTE_WRAPPER = (
    f"bash -c '{ENV}; exec python3 -u -'"
)


def telemetry_worker():
    while True:
        host = require_host()
        if not host:
            time.sleep(3)
            continue
        proc = None
        try:
            proc = subprocess.Popen(
                ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=5", host, REMOTE_WRAPPER],
                stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL, text=True, bufsize=1,
            )
            proc.stdin.write(TELEMETRY_SCRIPT)
            proc.stdin.close()
            for line in proc.stdout:
                line = line.strip()
                if not line.startswith("{"):
                    continue
                try:
                    data = json.loads(line)
                except json.JSONDecodeError:
                    continue
                with LOCK:
                    STATE["telemetry"] = data
                    STATE["telemetry_age"] = 0.0
                    STATE["ssh_ok"] = True
        except Exception:  # noqa: BLE001
            pass
        finally:
            if proc is not None:
                try:
                    proc.kill()
                except Exception:  # noqa: BLE001
                    pass
        with LOCK:
            STATE["ssh_ok"] = False
            STATE["host"] = None   # 触发重新探测(热点/网线切换)
        time.sleep(2)


# ---------------------------------------------------------------- 进程状态轮询
PROC_KEYS = ["livox", "fastlio", "recorder", "safe", "follower", "cmd", "pcd", "camera_loop", "saas_loop"]

PROC_STATUS = r'''ps -eo comm=,args= | awk '
BEGIN { livox=fastlio=recorder=safe=follower=cmd=pcd=camera_loop=saas_loop=0 }
$1 ~ /^livox_ros_drive/ || $0 ~ /[l]ivox_ros_driver2_node/ { livox=1 }
$1 == "fastlio_mapping" || $0 ~ /[f]astlio_mapping/ { fastlio=1 }
$1 == "route_recorder" || ($1 == "ros2" && $0 ~ /[r]oute_recorder/) { recorder=1 }
$0 ~ /[u]nitree_safe_cmd_node/ { safe=1 }
$0 ~ /[w]aypoint_follower/ { follower=1 }
$0 ~ /[u]nitree_cmd_node/ { cmd=1 }
$1 ~ /^python/ && $0 ~ /[g]o2map_capture/ { pcd=1 }
$1 == "bash" && $0 ~ /bash \/tmp\/[z]1pro_video_loop\.sh/ { camera_loop=1 }
$0 ~ /[g]o2_saas_agent\.py patrol-loop/ { saas_loop=1 }
END {
    if (system("test -e /home/unitree/go2_fastlio_ws/patrol_logs/run/patrol_video.active") == 0) camera_loop=1;
    print "livox=" livox;
    print "fastlio=" fastlio;
    print "recorder=" recorder;
    print "safe=" safe;
    print "follower=" follower;
    print "cmd=" cmd;
    print "pcd=" pcd;
    print "camera_loop=" camera_loop;
    print "saas_loop=" saas_loop;
}' '''


def _last_line(text: str, contains: str = "") -> str:
    for line in reversed(text.strip().splitlines()):
        line = line.strip()
        if line and (not contains or contains in line):
            return line
    return ""


def summarize_follower(log_text: str, is_running: bool):
    if is_running:
        if "stuck recovery" in log_text:
            line = _last_line(log_text, "nearest=") or _last_line(log_text, "stuck recovery")
            return {"state": "stuck", "text": "巡线运行中但没有路线进展: " + line}
        line = _last_line(log_text, "nearest=")
        if line:
            return {"state": "running", "text": "巡线运行中: " + line}
        return {"state": "starting", "text": "巡线启动中，等待第一条控制日志"}
    if "goal reached, stop" in log_text:
        return {"state": "finished", "text": "巡线已结束: 到达终点"}
    if "RuntimeError" in log_text or "Traceback" in log_text:
        line = _last_line(log_text, "RuntimeError") or _last_line(log_text)
        return {"state": "failed", "text": "巡线启动失败: " + line}
    return {"state": "idle", "text": "巡线未运行"}


def summarize_safe(log_text: str, is_running: bool):
    if not is_running:
        return {"state": "idle", "text": "安全节点未运行"}
    line = _last_line(log_text)
    if "SAFE OVERRIDE obstacle" in line:
        return {"state": "blocked", "text": "安全节点限停: 前方障碍物/近距点云"}
    if "SAFE OVERRIDE cloud_timeout" in line:
        return {"state": "blocked", "text": "安全节点限停: 点云超时，先恢复底座"}
    if "SAFE OVERRIDE cmd_timeout" in line:
        return {"state": "waiting", "text": "安全节点运行中: 等待巡线命令"}
    if line:
        return {"state": "running", "text": "安全节点运行中: " + line}
    return {"state": "running", "text": "安全节点运行中"}


def status_worker():
    while True:
        host = current_host()
        if not host:
            time.sleep(2)
            continue
        cmd = PROC_STATUS + "; echo __WC__; "
        rec_file = None
        with LOCK:
            rec_file = STATE["recorder_file"]
        if rec_file:
            cmd += f'wc -l < "{rec_file}" 2>/dev/null || echo 0'
        else:
            cmd += "echo -"
        cmd += '; echo __RECMARK__; tail -n 1 /tmp/console_recorder.log 2>/dev/null || true'
        cmd += '; echo __PCDMARK__; tail -n 1 /tmp/console_pcd.log 2>/dev/null || true'
        cmd += '; echo __FOLLOWER__; tail -n 12 /tmp/console_follower.log 2>/dev/null || true'
        cmd += '; echo __SAFELOG__; tail -n 8 /tmp/console_safe.log 2>/dev/null || true'
        cmd += ('; echo __SYS__; cat /sys/class/thermal/thermal_zone0/temp 2>/dev/null || echo 0'
            '; iw dev wlan0 link 2>/dev/null | grep -oE "signal: -[0-9]+" || echo signal_na'
            '; ping -c 1 -W 1 192.168.1.161 >/dev/null 2>&1 && echo livox_ping_ok || echo livox_ping_bad'
            '; ping -c 1 -W 1 192.168.144.108 >/dev/null 2>&1 && echo z1_ping_ok || echo z1_ping_bad'
            '; timeout 1 bash -lc "</dev/tcp/192.168.144.108/554" >/dev/null 2>&1 && echo z1_rtsp_ok || echo z1_rtsp_bad'
            '; timeout 1 bash -lc "</dev/tcp/192.168.144.108/2332" >/dev/null 2>&1 && echo z1_gcu_ok || echo z1_gcu_bad')
        rc, out, _ = ssh_run(host, cmd, timeout=10)
        if rc in (0, 1):
            body, _, rest = out.partition("__WC__")
            wc_part, _, rest2 = rest.partition("__RECMARK__")
            recorder_part, _, rest3 = rest2.partition("__PCDMARK__")
            pcd_part, _, rest3 = rest3.partition("__FOLLOWER__")
            follower_part, _, rest4 = rest3.partition("__SAFELOG__")
            safe_part, _, sys_part = rest4.partition("__SYS__")
            procs = {k: False for k in PROC_KEYS}
            for line in body.strip().splitlines():
                if "=" not in line:
                    continue
                key, value = line.strip().split("=", 1)
                if key in procs:
                    procs[key] = value.strip() == "1"
            lines = None
            if rec_file:
                txt = wc_part.strip().splitlines()
                try:
                    lines = int(txt[-1]) if txt else None
                except ValueError:
                    lines = None
            pcd_txt = pcd_part.strip().splitlines()
            recorder_txt = recorder_part.strip().splitlines()
            sys_info = {}
            for ln in sys_part.strip().splitlines():
                ln = ln.strip()
                if ln.isdigit():
                    sys_info["cpu_temp"] = round(int(ln) / 1000.0, 1)
                elif ln.startswith("signal:"):
                    sys_info["wifi_dbm"] = int(ln.split(":")[1])
                elif ln in ("livox_ping_ok", "livox_ping_bad"):
                    sys_info["livox_ping"] = ln.endswith("ok")
                elif ln in ("z1_ping_ok", "z1_ping_bad"):
                    sys_info["z1_ping"] = ln.endswith("ok")
                elif ln in ("z1_rtsp_ok", "z1_rtsp_bad"):
                    sys_info["z1_rtsp"] = ln.endswith("ok")
                elif ln in ("z1_gcu_ok", "z1_gcu_bad"):
                    sys_info["z1_gcu"] = ln.endswith("ok")
            with LOCK:
                STATE["processes"] = procs
                STATE["recorder_lines"] = lines
                STATE["recorder_progress"] = (
                    recorder_txt[-1] if recorder_txt else None
                )
                STATE["pcd_progress"] = pcd_txt[-1] if pcd_txt else None
                STATE["follower_status"] = summarize_follower(follower_part, procs.get("follower", False))
                STATE["safe_status"] = summarize_safe(safe_part, procs.get("safe", False))
                STATE["sys"] = sys_info
                if STATE["telemetry"]:
                    STATE["telemetry_age"] = round(time.time() - STATE["telemetry"].get("ts", 0), 1)
        time.sleep(3)


# ---------------------------------------------------------------- 动作(全部白名单)
def _detached(cmd: str, logname: str) -> str:
    return f"{ENV}; setsid nohup bash -c {json.dumps(cmd)} </dev/null >/tmp/console_{logname}.log 2>&1 & echo STARTED"


def act_start_base(_):
    return f"setsid nohup bash {WS}/scripts/base_bringup.sh </dev/null >/tmp/console_base.log 2>&1 & echo STARTED"


def act_stop_base(_):
    guard = (
        'p=$(ps -eo pid=,comm=,args= | awk \'$0 ~ /[w]aypoint_follower|[u]nitree_safe_cmd_node|[u]nitree_cmd_node/ {print $1}\' | grep -vx "$$" || true); '
        '[ -n "$p" ] && { echo CONTROL_RUNNING_STOP_CONTROL_FIRST; echo "$p"; exit 0; }; '
    )
    return (
        guard +
        f"bash {WS}/scripts/base_stop.sh >/dev/null 2>&1; sleep 1; "
        'pkill -INT -f "[l]ivox_ros_driver2_node" 2>/dev/null; '
        'pkill -INT -f "[f]astlio_mapping" 2>/dev/null; sleep 1; '
        'pkill -TERM -f "[l]ivox_ros_driver2_node" 2>/dev/null; '
        'pkill -TERM -f "[f]astlio_mapping" 2>/dev/null; echo STOPPED'
    )


def _mode_conflict_guard(target: str, pattern: str, include_patrol_flag: bool = True):
    """Build an authoritative remote guard so heavy operating modes cannot overlap."""
    active_probe = (
        f'active=""; [ -e "{PATROL_VIDEO_ACTIVE_FILE}" ] && active=patrol || true; '
        if include_patrol_flag else 'active=""; '
    )
    return (
        "p=$(ps -eo pid=,comm=,args= | "
        f"awk '$2 != \"bash\" && $2 != \"sh\" && $0 ~ /{pattern}/ {{print $1}}' | "
        'grep -vx "$$" || true); '
        + active_probe
        + f'[ -n "$p$active" ] && {{ echo "MODE_CONFLICT target={target}"; '
        + 'echo "pids=$p active=$active"; exit 4; }; '
    )


def act_start_recorder(p):
    name = str(p.get("route_name", ""))
    if not NAME_RE.match(name):
        raise ValueError("路线名只允许字母/数字/下划线/短横线")
    path = f"{ROUTES_DIR}/{name}.csv"
    overwrite = bool(p.get("overwrite", False))
    dup_guard = (
        "p=$(ps -eo pid=,comm=,args= | "
        "awk '$2 != \"bash\" && $2 != \"sh\" && $0 ~ /[r]oute_recorder/ {print $1}' | "
        "grep -vx \"$$\" || true); "
        "[ -n \"$p\" ] && { echo ALREADY_RUNNING; echo \"$p\"; exit 0; }; "
    )
    guard = "" if overwrite else f'[ -e "{path}" ] && {{ echo EXISTS; exit 0; }}; '
    with LOCK:
        STATE["recorder_file"] = path
        STATE["recorder_lines"] = None
    inner = (
        f"ros2 run go2_fastlio_patrol route_recorder --ros-args "
        f"-p route_file:={path}"
    )
    mode_guard = _mode_conflict_guard(
        "recorder",
        r"[g]o2map_capture|[w]aypoint_follower|[u]nitree_safe_cmd_node|[c]md_vel_udp_sender|[g]o2_sdk2_udp_receiver|\/tmp\/[z]1pro_video_loop\.sh",
    )
    return mode_guard + dup_guard + guard + _detached(inner, "recorder")


def act_stop_recorder(_):
    with LOCK:
        f = STATE["recorder_file"]
    wc = f'; [ -f "{f}" ] && echo LINES=$(wc -l < "{f}")' if f else ""
    quality = ""
    if f:
        route_name = os.path.splitext(os.path.basename(f))[0]
        quality_file = f'{ROUTES_DIR}/quality/{route_name}.quality.json'
        quality = (
            f'; echo __QUALITY__; '
            f'[ -f "{quality_file}" ] && cat "{quality_file}" || true'
        )
    stop = (
        'pkill -TERM -f "[r]oute_recorder" && { '
        'for i in $(seq 1 100); do '
        'pgrep -f "[r]oute_recorder" >/dev/null || break; sleep 0.1; '
        'done; '
        'if pgrep -f "[r]oute_recorder" >/dev/null; then '
        'pkill -KILL -f "[r]oute_recorder" 2>/dev/null; echo STOPPED_FORCED; '
        'else echo STOPPED; fi; '
        '} || echo NOT_RUNNING'
    )
    return stop + wc + quality


def act_start_safe(p):
    max_vx = float(p.get("max_vx", 0.5))
    if not 0.05 <= max_vx <= 0.5:
        raise ValueError("max_vx 需在 0.05~0.5")
    dry = bool(p.get("dry_run", False))
    topic = "/debug/sport/request" if dry else "/api/sport/request"
    dup_guard = (
        "p=$(ps -eo pid=,comm=,args= | "
        "awk '$2 != \"bash\" && $2 != \"sh\" && $0 ~ /[u]nitree_safe_cmd_node/ {print $1}' | "
        "grep -vx \"$$\" || true); "
        "[ -n \"$p\" ] && { echo SAFE_ALREADY_RUNNING; echo \"$p\"; exit 0; }; "
    )
    inner = (
        "ros2 run go2_fastlio_patrol unitree_safe_cmd_node --ros-args "
        f"-p max_vx:={max_vx} -p max_yaw_rate:=0.45 -p publish_rate:=20.0 "
        f"-p pointcloud_topic:=/cloud_registered_body -p sport_request_topic:={topic} "
        "-p stop_distance:=0.40 -p resume_distance:=0.50 -p min_stop_points:=15 "
        "-p roi_x_min:=0.35 -p roi_x_max:=0.90 -p roi_y_min:=-0.30 -p roi_y_max:=0.30 "
        "-p roi_z_min:=0.30 -p roi_z_max:=0.90"
    )
    mode_guard = _mode_conflict_guard(
        "safe",
        r"[r]oute_recorder|[g]o2map_capture|\/tmp\/[z]1pro_video_loop\.sh",
    )
    return mode_guard + dup_guard + _detached(inner, "safe")


def act_stop_safe(_):
    return (
        'p=$(ps -eo pid=,comm=,args= | awk \'($2 == "ros2" && $0 ~ /[u]nitree_safe_cmd_node/) || $2 ~ /^unitree_safe_cm/ {print $1}\' | tr "\\n" " " || true); '
        '[ -z "$p" ] && { echo NOT_RUNNING; exit 0; }; '
        'kill -TERM $p 2>/dev/null || true; sleep 1; '
        'p2=$(ps -eo pid=,comm=,args= | awk \'($2 == "ros2" && $0 ~ /[u]nitree_safe_cmd_node/) || $2 ~ /^unitree_safe_cm/ {print $1}\' | tr "\\n" " " || true); '
        '[ -n "$p2" ] && kill -KILL $p2 2>/dev/null || true; echo STOPPED'
    )


def act_stop_follower(_):
    return (
        'p=$(ps -eo pid=,comm=,args= | awk \'($2 == "ros2" && $0 ~ /[w]aypoint_follower/) || $2 ~ /^waypoint_follow/ {print $1}\' | tr "\\n" " " || true); '
        '[ -z "$p" ] && { echo NOT_RUNNING; exit 0; }; '
        'kill -TERM $p 2>/dev/null || true; sleep 1; '
        'p2=$(ps -eo pid=,comm=,args= | awk \'($2 == "ros2" && $0 ~ /[w]aypoint_follower/) || $2 ~ /^waypoint_follow/ {print $1}\' | tr "\\n" " " || true); '
        '[ -n "$p2" ] && kill -KILL $p2 2>/dev/null || true; echo FOLLOWER_STOPPED'
    )


def act_start_follower(p):
    path = str(p.get("route_path", ""))
    if not (path.startswith(ROUTES_DIR) and ROUTE_PATH_RE.match(path) and ".." not in path):
        raise ValueError("非法路线路径")
    v = float(p.get("v_base", 0.5))
    if not 0.05 <= v <= 0.5:
        raise ValueError("v_base 需在 0.05~0.5")
    loop = str(p.get("loop_mode", "pingpong"))
    if loop not in ("once", "pingpong"):
        raise ValueError("loop_mode 只能 once/pingpong")
    route_check = f'lines=$(wc -l < "{path}" 2>/dev/null || echo 0); [ "$lines" -lt 3 ] && {{ echo ROUTE_TOO_SHORT lines=$lines; exit 0; }}; '
    dup_guard = (
        "p=$(ps -eo pid=,comm=,args= | "
        "awk '$2 != \"bash\" && $2 != \"sh\" && $0 ~ /[w]aypoint_follower/ {print $1}' | "
        "grep -vx \"$$\" || true); "
        "[ -n \"$p\" ] && { echo FOLLOWER_ALREADY_RUNNING; echo \"$p\"; exit 0; }; "
    )
    inner = (
        "ros2 run go2_fastlio_patrol waypoint_follower --ros-args "
        f"-p route_file:={path} -p v_base:={v} -p max_vx:={v} "
        "-p k_yaw:=0.9 -p max_yaw_rate:=0.45 -p lookahead_distance:=0.6 "
        "-p reach_distance:=0.4 -p goal_distance:=0.25 "
        f"-p loop_mode:={loop} -p search_window:=6 -p turn_in_place_angle:=1.0 "
        "-p slow_down_angle:=0.5 -p stuck_time:=3.0 -p relocalize_distance:=1.5"
    )
    mode_guard = _mode_conflict_guard(
        "follower",
        r"[r]oute_recorder|[g]o2map_capture|\/tmp\/[z]1pro_video_loop\.sh",
    )
    return mode_guard + dup_guard + route_check + _detached(inner, "follower")


def act_estop(_):
    # 可靠优先: TERM+KILL 杀 follower, safe 节点 0.5s 内因 cmd 超时持续发 Move(0,0,0)
    return ('pkill -TERM -f "[w]aypoint_follower"; sleep 0.5; '
            'pkill -KILL -f "[w]aypoint_follower" 2>/dev/null; echo ESTOP_SENT')


def act_stop_all_control(_):
    return (
        'p=$(ps -eo pid=,comm=,args= | awk \'($2 == "ros2" && $0 ~ /[w]aypoint_follower|[u]nitree_safe_cmd_node|[u]nitree_cmd_node/) || $2 ~ /^waypoint_follow/ || $2 ~ /^unitree_safe_cm/ || $2 ~ /^unitree_cmd/ {print $1}\' | tr "\\n" " " || true); '
        '[ -n "$p" ] && kill -TERM $p 2>/dev/null || true; sleep 1; '
        'p2=$(ps -eo pid=,comm=,args= | awk \'($2 == "ros2" && $0 ~ /[w]aypoint_follower|[u]nitree_safe_cmd_node|[u]nitree_cmd_node/) || $2 ~ /^waypoint_follow/ || $2 ~ /^unitree_safe_cm/ || $2 ~ /^unitree_cmd/ {print $1}\' | tr "\\n" " " || true); '
        '[ -n "$p2" ] && kill -KILL $p2 2>/dev/null || true; '
        'echo ALL_CONTROL_STOPPED'
    )


def act_tail_log(p):
    name = str(p.get("log", ""))
    if name not in ("base", "recorder", "safe", "follower", "pcd", "saas_loop"):
        raise ValueError("log 只能 base/recorder/safe/follower/pcd/saas_loop")
    return f"tail -n 40 /tmp/console_{name}.log 2>/dev/null || echo EMPTY"


# -------- 在线点云累积(内存注入脚本, 输出 PCD 数据文件到狗的 maps/console/) --------
# 注意: Foxy 没有 sensor_msgs_py, 必须手动解析 PointCloud2 二进制
PCD_SCRIPT = r'''
import os, signal, sys, time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

OUT = sys.argv[1]
VOXEL, FRAME_STRIDE, POINT_STRIDE = 0.08, 3, 2


def voxel_ds(P, v):
    if not len(P):
        return P
    k = np.floor(P / v).astype(np.int64)
    _, idx = np.unique(k, axis=0, return_index=True)
    return P[idx]


def cloud_xyz(msg):
    off = {}
    for f in msg.fields:
        if f.name in ('x', 'y', 'z') and f.datatype == 7:   # 7 = FLOAT32
            off[f.name] = f.offset
    if len(off) != 3:
        return None
    n = msg.width * msg.height
    if n == 0:
        return None
    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    need = n * msg.point_step
    if len(buf) < need:
        return None
    buf = buf[:need].reshape(n, msg.point_step)
    cols = []
    for name in ('x', 'y', 'z'):
        o = off[name]
        cols.append(buf[:, o:o + 4].copy().view(np.float32).ravel())
    P = np.stack(cols, axis=1).astype(np.float64)
    P = P[~np.isnan(P).any(axis=1)]
    return P[::POINT_STRIDE]


class Acc(Node):
    def __init__(self):
        super().__init__('go2map_capture')
        self.frames = 0
        self.chunks = []
        self.total = 0
        self.create_subscription(PointCloud2, '/cloud_registered', self.cb, 10)

    def cb(self, msg):
        self.frames += 1
        if self.frames % FRAME_STRIDE:
            return
        P = cloud_xyz(msg)
        if P is not None and len(P):
            self.chunks.append(P)
            self.total += len(P)


rclpy.init()
n = Acc()
alive = [True]
signal.signal(signal.SIGINT, lambda *a: alive.__setitem__(0, False))
signal.signal(signal.SIGTERM, lambda *a: alive.__setitem__(0, False))
last_log = 0.0
last_compact = time.time()
while rclpy.ok() and alive[0]:
    rclpy.spin_once(n, timeout_sec=0.2)
    now = time.time()
    if now - last_log > 2:
        last_log = now
        print("POINTS %d FRAMES %d" % (n.total, n.frames), flush=True)
    if now - last_compact > 20 and len(n.chunks) > 1:
        merged = voxel_ds(np.vstack(n.chunks), VOXEL)
        n.chunks = [merged]
        n.total = len(merged)
        last_compact = now
if n.chunks:
    P = voxel_ds(np.vstack(n.chunks), VOXEL)
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    with open(OUT, "w") as f:
        f.write("# .PCD v0.7 - Point Cloud Data file format\n")
        f.write("VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n")
        f.write("WIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA ascii\n" % (len(P), len(P)))
        for p in P:
            f.write("%.4f %.4f %.4f\n" % (p[0], p[1], p[2]))
    print("SAVED %s %d" % (OUT, len(P)), flush=True)
else:
    print("NO_POINTS", flush=True)
'''


def act_start_pcd(p):
    name = str(p.get("name", ""))
    if not NAME_RE.match(name):
        raise ValueError("地图名只允许字母/数字/下划线/短横线")
    out = f"{PCD_DIR}/{name}.pcd"
    mode_guard = _mode_conflict_guard(
        "pcd",
        r"[r]oute_recorder|[w]aypoint_follower|[u]nitree_safe_cmd_node|[c]md_vel_udp_sender|[g]o2_sdk2_udp_receiver|\/tmp\/[z]1pro_video_loop\.sh",
    )
    return mode_guard + (
        'p=$(ps -eo pid=,comm=,args= | awk \'$2 ~ /^python/ && $0 ~ /[g]o2map_capture/ {print $1}\' | grep -vx "$$" || true); '
        '[ -n "$p" ] && { echo ALREADY_RUNNING; echo "$p"; exit 0; }; '
        + f"mkdir -p {PCD_DIR}; cat > /tmp/go2map_capture.py <<'PYEOF'\n"
        + PCD_SCRIPT
        + "\nPYEOF\n"
        + f'{ENV}; setsid nohup python3 -u /tmp/go2map_capture.py "{out}" '
        + "</dev/null >/tmp/console_pcd.log 2>&1 & echo PCD_CAPTURE_STARTED"
    )


def act_stop_pcd(_):
    # 注意: PCD 脚本靠 TERM 触发"保存后退出", 不能用 KILL(会丢数据)
    return 'pkill -TERM -f "[g]o2map_capture" && echo PCD_SAVING || echo NOT_RUNNING'


def act_camera_probe(_):
    return (
        f'echo __GCU__; {Z1PRO_GCU} probe 2>&1 | head -4; '
        f'echo __RTSP__; {Z1PRO_CAPTURE} probe 2>&1 | head -8'
    )


def act_camera_preset(p):
    preset = str(p.get("preset", "probe"))
    allowed = {"probe", "front", "down", "up", "left", "right", "home"}
    if preset not in allowed:
        raise ValueError("摄像头预设只能 probe/front/down/up/left/right/home")
    return f'{Z1PRO_PRESET} {preset}'


def act_camera_snapshot(_):
    return f'{Z1PRO_CAPTURE} snapshot | tail -1 | sed "s/^/SNAPSHOT /"'


def act_camera_record(p):
    seconds = int(p.get("seconds", 20))
    if not 3 <= seconds <= 60:
        raise ValueError("视频秒数需在 3~60")
    return f'{Z1PRO_CAPTURE} record {seconds} | tail -1 | sed "s/^/VIDEO /"'


def act_camera_start_loop(p):
    seconds = int(p.get("seconds", 20))
    if not 5 <= seconds <= 60:
        raise ValueError("连续录制分段秒数需在 5~60")
    mode_guard = _mode_conflict_guard(
        "video",
        r"[r]oute_recorder|[g]o2map_capture|[w]aypoint_follower|[u]nitree_safe_cmd_node|[c]md_vel_udp_sender|[g]o2_sdk2_udp_receiver",
    )
    return mode_guard + (
        'p=$(ps -eo pid=,comm=,args= | awk \'$2 == "bash" && $0 ~ /^ *[0-9]+ +bash +\/tmp\/[z]1pro_video_loop\.sh/ {print $1}\' | grep -vx "$$" || true); '
        '[ -n "$p" ] && { echo CAMERA_LOOP_ALREADY_RUNNING; exit 0; }; '
        'rm -f /tmp/z1pro_video_loop.run; '
        'cat > /tmp/z1pro_video_loop.sh <<\'SH\'\n'
        '#!/usr/bin/env bash\n'
        f'touch /tmp/z1pro_video_loop.run\n'
        'while [ -f /tmp/z1pro_video_loop.run ]; do\n'
        f'  {Z1PRO_CAPTURE} record {seconds} || true\n'
        '  sleep 1\n'
        'done\n'
        'SH\n'
        'chmod +x /tmp/z1pro_video_loop.sh; '
        'setsid nohup bash /tmp/z1pro_video_loop.sh </dev/null >/tmp/console_z1pro_loop.log 2>&1 & echo CAMERA_LOOP_STARTED'
    )


def act_camera_stop_loop(_):
    return (
        'rm -f /tmp/z1pro_video_loop.run; '
        'pkill -TERM -f "[z]1pro_video_loop.sh" 2>/dev/null || true; '
        'pkill -TERM -f "[g]st-launch-1.0.*192.168.144.108" 2>/dev/null || true; '
        'echo CAMERA_LOOP_STOPPED'
    )


def _saas_cmd(args):
    private_env = 'test -f "$HOME/.config/go2_saas.env" && source "$HOME/.config/go2_saas.env" || true'
    return f"{ENV}; {private_env}; " + " ".join(shlex.quote(str(arg)) for arg in args)


def act_saas_heartbeat(p):
    args = ["python3", SAAS_AGENT, "heartbeat-once", "--ros-timeout", "1.5"]
    if bool(p.get("dry_run", True)):
        args.append("--dry-run")
    return _saas_cmd(args)


def act_saas_manifest(p):
    limit = int(p.get("limit", 8))
    if not 1 <= limit <= 20:
        raise ValueError("资产清单数量需在 1~20")
    args = ["python3", SAAS_AGENT, "asset-manifest", "--limit", str(limit)]
    return _saas_cmd(args)


def act_saas_command_result(p):
    command_id = str(p.get("command_id", "console-test"))
    if not COMMAND_ID_RE.match(command_id):
        raise ValueError("command_id 只能包含字母/数字/下划线/冒号/点/短横线")
    status = str(p.get("status", "success"))
    if status not in ("success", "failed", "running", "rejected"):
        raise ValueError("status 非法")
    message = str(p.get("message", "console connectivity test"))[:300]
    args = ["python3", SAAS_AGENT, "command-result", "--command-id", command_id, "--status", status, "--message", message]
    if bool(p.get("dry_run", True)):
        args.append("--dry-run")
    return _saas_cmd(args)


def act_saas_video_segment(p):
    seconds = int(p.get("seconds", 20))
    if not 5 <= seconds <= 60:
        raise ValueError("视频秒数需在 5~60")
    args = ["python3", SAAS_AGENT, "video-segment", "--seconds", str(seconds)]
    if bool(p.get("upload", False)):
        args.append("--upload")
    return _saas_cmd(args)


def act_saas_start_loop(p):
    route = str(p.get("route", "xiaoqu1"))
    pcd = str(p.get("pcd", "xiaoqu1"))
    patrol_id = str(p.get("patrol_id", route or "field-test"))
    interval = int(p.get("interval", 20))
    if not NAME_RE.match(route) or not NAME_RE.match(pcd) or not NAME_RE.match(patrol_id):
        raise ValueError("route/pcd/patrol_id 只允许字母/数字/下划线/短横线")
    if not 5 <= interval <= 300:
        raise ValueError("循环间隔需在 5~300 秒")
    args = [
        "python3", "-u", SAAS_AGENT, "patrol-loop",
        "--route", route, "--pcd", pcd, "--video", "cycle",
        "--patrol-id", patrol_id, "--interval", str(interval), "--ros-timeout", "1.5", "--run-file", SAAS_LOOP_RUN_FILE,
    ]
    if bool(p.get("upload", False)):
        args.append("--upload")
    inner = _saas_cmd(args)
    guard = (
        "p=$(ps -eo pid=,comm=,args= | "
        "awk '$2 ~ /^python/ && $0 ~ /[g]o2_saas_agent\\.py patrol-loop/ {print $1}' | "
        "grep -vx \"$$\" || true); "
        "[ -n \"$p\" ] && { echo SAAS_LOOP_ALREADY_RUNNING; echo \"$p\"; exit 0; }; "
    )
    return guard + f"touch {SAAS_LOOP_RUN_FILE}; setsid nohup bash -c {json.dumps(inner)} </dev/null >/tmp/console_saas_loop.log 2>&1 & echo SAAS_LOOP_STARTED"


def act_saas_stop_loop(_):
    return (
        f'rm -f {SAAS_LOOP_RUN_FILE}; '
        'p=$(ps -eo pid=,comm=,args= | awk \'$0 ~ /[g]o2_saas_agent\\.py patrol-loop/ {print $1}\' | tr "\\n" " " || true); '
        '[ -z "$p" ] && { echo NOT_RUNNING; exit 0; }; '
        'kill -TERM $p 2>/dev/null || true; sleep 1; '
        'p2=$(ps -eo pid=,comm=,args= | awk \'$0 ~ /[g]o2_saas_agent\\.py patrol-loop/ {print $1}\' | tr "\\n" " " || true); '
        '[ -n "$p2" ] && kill -KILL $p2 2>/dev/null || true; echo SAAS_LOOP_STOPPED'
    )


ACTIONS = {
    "start_base": act_start_base,
    "stop_base": act_stop_base,
    "start_recorder": act_start_recorder,
    "stop_recorder": act_stop_recorder,
    "start_safe": act_start_safe,
    "stop_safe": act_stop_safe,
    "start_follower": act_start_follower,
    "stop_follower": act_stop_follower,
    "estop": act_estop,
    "stop_all_control": act_stop_all_control,
    "tail_log": act_tail_log,
    "start_pcd": act_start_pcd,
    "stop_pcd": act_stop_pcd,
    "camera_probe": act_camera_probe,
    "camera_preset": act_camera_preset,
    "camera_snapshot": act_camera_snapshot,
    "camera_record": act_camera_record,
    "camera_start_loop": act_camera_start_loop,
    "camera_stop_loop": act_camera_stop_loop,
    "saas_heartbeat": act_saas_heartbeat,
    "saas_manifest": act_saas_manifest,
    "saas_command_result": act_saas_command_result,
    "saas_video_segment": act_saas_video_segment,
    "saas_start_loop": act_saas_start_loop,
    "saas_stop_loop": act_saas_stop_loop,
}

MOTION_ACTIONS = {"start_safe", "start_follower"}


# ---------------------------------------------------------------- API
app = FastAPI(title="Go2 Patrol Console")
app.mount("/vendor", StaticFiles(directory=STATIC_DIR / "vendor"), name="vendor")


class ActionReq(BaseModel):
    name: str
    params: dict = {}
    armed: bool = False


@app.get("/")
def index():
    return FileResponse(STATIC_DIR / "index.html")


@app.get("/api/status")
def api_status():
    with LOCK:
        snap = {
            "host": STATE["host"],
            "ssh_ok": STATE["ssh_ok"],
            "telemetry": STATE["telemetry"],
            "telemetry_age": STATE["telemetry_age"],
            "processes": STATE["processes"],
            "recorder_file": STATE["recorder_file"],
            "recorder_lines": STATE["recorder_lines"],
            "recorder_progress": STATE["recorder_progress"],
            "pcd_progress": STATE["pcd_progress"],
            "follower_status": STATE["follower_status"],
            "safe_status": STATE["safe_status"],
            "sys": STATE["sys"],
            "actions_log": STATE["actions_log"][-15:],
        }
    return JSONResponse(snap)


@app.get("/api/routes")
def api_routes():
    host = require_host()
    if not host:
        return JSONResponse({"error": "SSH 不可达"}, status_code=503)
    cmd = (
        f'for f in {ROUTES_DIR}/*.csv {ROUTES_DIR}/records/*/route.csv; do '
        f'[ -f "$f" ] || continue; '
        f'echo "$(stat -c %Y "$f")|$f|$(wc -l < "$f")|$(sed -n 2p "$f")"; done | sort -t "|" -k1,1nr'
    )
    rc, out, err = ssh_run(host, cmd, timeout=15)
    if rc != 0:
        return JSONResponse({"error": err or "查询失败"}, status_code=500)
    routes = []
    for line in out.splitlines():
        parts = line.split("|")
        if len(parts) != 4:
            continue
        _, path, lines, first = parts
        start = None
        cols = first.split(",")
        if len(cols) >= 3:
            try:
                start = [float(cols[1]), float(cols[2])]
            except ValueError:
                start = None
        routes.append({"path": path, "lines": int(lines), "start": start})
    return JSONResponse({"routes": routes})


@app.get("/api/route_points")
def api_route_points(path: str):
    if not (path.startswith(ROUTES_DIR) and ROUTE_PATH_RE.match(path) and ".." not in path):
        return JSONResponse({"error": "非法路径"}, status_code=400)
    host = require_host()
    if not host:
        return JSONResponse({"error": "SSH 不可达"}, status_code=503)
    rc, out, err = ssh_run(host, f'cat "{path}"', timeout=20)
    if rc != 0:
        return JSONResponse({"error": err or "读取失败"}, status_code=500)
    pts = []
    for i, line in enumerate(out.splitlines()):
        if i == 0:
            continue
        cols = line.split(",")
        if len(cols) >= 3:
            try:
                x = round(float(cols[1]), 3)
                y = round(float(cols[2]), 3)
                yaw = round(float(cols[3]), 3) if len(cols) >= 4 else 0.0
                pts.append([x, y, 0.0, yaw])
            except ValueError:
                continue
    return JSONResponse({"points": pts})


@app.get("/api/pcd_list")
def api_pcd_list():
    host = require_host()
    if not host:
        return JSONResponse({"error": "SSH 不可达"}, status_code=503)
    cmd = (
        f'for f in {PCD_DIR}/*.pcd; do [ -f "$f" ] || continue; '
        f'echo "$(stat -c %Y "$f")|$f|$(stat -c %s "$f")|$(grep -m1 ^POINTS "$f" | cut -d\  -f2)"; done | sort -t "|" -k1,1nr'
    )
    rc, out, err = ssh_run(host, cmd, timeout=15)
    files = []
    for line in out.splitlines():
        parts = line.split("|")
        if len(parts) == 4:
            files.append({"path": parts[1], "size": int(parts[2] or 0), "points": int(parts[3] or 0), "mtime": int(parts[0] or 0)})
    return JSONResponse({"files": files})


@app.get("/api/pcd_points")
def api_pcd_points(path: str, max_points: int = 40000):
    if not (path.startswith(PCD_DIR) and PCD_PATH_RE.match(path) and ".." not in path):
        return JSONResponse({"error": "非法路径"}, status_code=400)
    host = require_host()
    if not host:
        return JSONResponse({"error": "SSH 不可达"}, status_code=503)
    # 远端先数点数, 算抽样步长, 再用 awk 抽 x,y — 避免整个大文件传回来
    rc, out, _ = ssh_run(host, f'grep -m1 ^POINTS "{path}" | cut -d" " -f2', timeout=10)
    try:
        total = int(out.strip())
    except ValueError:
        return JSONResponse({"error": "PCD 头解析失败"}, status_code=500)
    stride = max(1, total // max_points)
    rc, out, err = ssh_run(
        host,
        f'awk \'/^DATA/{{d=1;next}} d && (++c % {stride} == 0) {{printf "%.2f,%.2f\\n",$1,$2}}\' "{path}"',
        timeout=60,
    )
    if rc != 0:
        return JSONResponse({"error": err or "读取失败"}, status_code=500)
    pts = []
    for line in out.splitlines():
        a, _, b = line.partition(",")
        try:
            pts.append([float(a), float(b)])
        except ValueError:
            continue
    return JSONResponse({"points": pts, "total": total, "stride": stride})


def pack_pcd_xyz(xyz):
    if not xyz:
        raise ValueError("empty point cloud")
    n = len(xyz)
    sx = sum(p[0] for p in xyz)
    sy = sum(p[1] for p in xyz)
    cx = sx / n
    cy = sy / n
    minz = min(p[2] for p in xyz)
    maxz = max(p[2] for p in xyz)
    minx = min(p[0] for p in xyz)
    maxx = max(p[0] for p in xyz)
    miny = min(p[1] for p in xyz)
    maxy = max(p[1] for p in xyz)
    zmax = max(maxz - minz, 1e-6)
    radius = max(max(maxx - minx, maxy - miny) * 0.5, 1.0)
    buf = bytearray()
    for x, y, z in xyz:
        buf.extend(struct.pack("<fff", float(x - cx), float(z - minz), float(y - cy)))
    return {
        "b64": base64.b64encode(bytes(buf)).decode("ascii"),
        "meta": {"count": n, "cx": cx, "cy": cy, "zmin": minz, "zmax": zmax, "radius": radius},
    }


@app.get("/api/pcd_pack")
def api_pcd_pack(path: str, max_points: int = 200000):
    if not (path.startswith(PCD_DIR) and PCD_PATH_RE.match(path) and ".." not in path):
        return JSONResponse({"error": "非法路径"}, status_code=400)
    host = require_host()
    if not host:
        return JSONResponse({"error": "SSH 不可达"}, status_code=503)
    rc, out, _ = ssh_run(host, f'grep -m1 ^POINTS "{path}" | cut -d" " -f2', timeout=10)
    try:
        total = int(out.strip())
    except ValueError:
        return JSONResponse({"error": "PCD 头解析失败"}, status_code=500)
    stride = max(1, total // max_points)
    rc, out, err = ssh_run(
        host,
        f'awk \'/^DATA/{{d=1;next}} d && (++c % {stride} == 0) {{printf "%.4f,%.4f,%.4f\\n",$1,$2,$3}}\' "{path}"',
        timeout=90,
    )
    if rc != 0:
        return JSONResponse({"error": err or "读取失败"}, status_code=500)
    xyz = []
    for line in out.splitlines():
        parts = line.split(",")
        if len(parts) != 3:
            continue
        try:
            xyz.append((float(parts[0]), float(parts[1]), float(parts[2])))
        except ValueError:
            continue
    try:
        packed = pack_pcd_xyz(xyz)
    except ValueError as e:
        return JSONResponse({"error": str(e)}, status_code=500)
    packed.update({"total": total, "stride": stride})
    return JSONResponse(packed)


@app.get("/api/camera_files")
def api_camera_files():
    host = require_host()
    if not host:
        return JSONResponse({"error": "SSH 不可达"}, status_code=503)
    cmd = (
        f"find {shlex.quote(VIDEO_DIR)} -maxdepth 1 -type f "
        r"\( -name 'z1pro*.mp4' -o -name 'z1pro*.jpg' -o -name 'z1pro*.jpeg' \) "
        r"-printf '%p|%s|%T@\n' | LC_ALL=C sort -t '|' -k3,3nr | head -60"
    )
    rc, out, err = ssh_run(host, cmd, timeout=20)
    if rc != 0:
        return JSONResponse({"error": err or "查询失败"}, status_code=500)
    files = []
    for line in out.splitlines():
        parts = line.split("|")
        if len(parts) != 3:
            continue
        path, size, mtime = parts
        lower = path.lower()
        files.append({
            "path": path,
            "name": path.rsplit("/", 1)[-1],
            "size": int(size or 0),
            "mtime": int(float(mtime or 0)),
            "kind": "video" if lower.endswith(".mp4") else "image",
        })
    return JSONResponse({"files": files})


def allowed_remote_file(path: str) -> bool:
    return (
        (path.startswith(PCD_DIR) and PCD_PATH_RE.match(path)) or
        (path.startswith(ROUTES_DIR) and ROUTE_PATH_RE.match(path)) or
        (path.startswith(VIDEO_DIR) and MEDIA_PATH_RE.match(path))
    ) and ".." not in path


def fetch_remote_bytes(path: str, timeout: int = 120):
    host = require_host()
    if not host:
        return None, JSONResponse({"error": "SSH 不可达"}, status_code=503)
    try:
        r = subprocess.run(
            ["ssh", "-o", "BatchMode=yes", host, f'cat "{path}"'],
            capture_output=True, timeout=timeout,
        )
    except subprocess.TimeoutExpired:
        return None, JSONResponse({"error": "下载超时"}, status_code=504)
    if r.returncode != 0:
        return None, JSONResponse({"error": r.stderr.decode(errors="ignore")[:200]}, status_code=500)
    return r.stdout, None


@app.get("/api/download")
def api_download(path: str):
    if not allowed_remote_file(path):
        return JSONResponse({"error": "非法路径"}, status_code=400)
    content, error = fetch_remote_bytes(path)
    if error:
        return error
    name = path.rsplit("/", 1)[-1]
    return Response(
        content=content,
        media_type="application/octet-stream",
        headers={"Content-Disposition": f'attachment; filename="{name}"'},
    )


@app.get("/api/file")
def api_file(path: str, request: Request):
    if not (path.startswith(VIDEO_DIR) and MEDIA_PATH_RE.match(path) and ".." not in path):
        return JSONResponse({"error": "非法路径"}, status_code=400)
    content, error = fetch_remote_bytes(path)
    if error:
        return error
    lower = path.lower()
    mt = "video/mp4" if lower.endswith(".mp4") else "image/jpeg"
    headers = {"Accept-Ranges": "bytes"}
    range_header = request.headers.get("range")
    if range_header and lower.endswith(".mp4"):
        m = re.match(r"bytes=(\d*)-(\d*)", range_header)
        if m:
            size = len(content)
            start_s, end_s = m.groups()
            start = int(start_s) if start_s else 0
            end = int(end_s) if end_s else size - 1
            start = max(0, min(start, size - 1))
            end = max(start, min(end, size - 1))
            part = content[start:end + 1]
            headers.update({
                "Content-Range": f"bytes {start}-{end}/{size}",
                "Content-Length": str(len(part)),
            })
            return Response(content=part, status_code=206, media_type=mt, headers=headers)
    headers["Content-Length"] = str(len(content))
    return Response(content=content, media_type=mt, headers=headers)


@app.post("/api/action")
def api_action(req: ActionReq):
    fn = ACTIONS.get(req.name)
    if fn is None:
        return JSONResponse({"error": "未知动作"}, status_code=400)
    if req.name in MOTION_ACTIONS and not req.armed:
        dry = bool(req.params.get("dry_run", False))
        if not dry:
            return JSONResponse({"error": "运动类动作需先解锁(armed)"}, status_code=403)
    host = require_host()
    if not host:
        return JSONResponse({"error": "SSH 不可达"}, status_code=503)
    try:
        cmd = fn(req.params)
    except ValueError as e:
        return JSONResponse({"error": str(e)}, status_code=400)
    timeout = 160 if req.name == "saas_video_segment" else 90 if req.name in {"camera_record", "camera_start_loop", "camera_snapshot", "saas_heartbeat", "saas_manifest", "saas_command_result"} else 25
    rc, out, err = ssh_run(host, cmd, timeout=timeout)
    result = out or err or f"rc={rc}"
    log_action(f"{req.name}: {result[:200]}")
    return JSONResponse({"rc": rc, "out": out, "err": err})


def main():
    threading.Thread(target=telemetry_worker, daemon=True).start()
    threading.Thread(target=status_worker, daemon=True).start()
    print(f"\n  Go2 巡检管理台已启动:  http://127.0.0.1:{PORT}\n")
    uvicorn.run(app, host="127.0.0.1", port=PORT, log_level="warning")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
import os
import re
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path


WS = Path("/home/unitree/go2_fastlio_ws")
SCRIPTS = WS / "scripts"
ENV = SCRIPTS / "env_common.sh"
ROUTE_ROOT = WS / "src/go2_fastlio_patrol/routes"
RECORD_ROOT = ROUTE_ROOT / "records"
LOG_DIR = WS / "patrol_logs"

DEFAULT_VX = 0.25
DEFAULT_YAW = 0.45
DEFAULT_PUBLISH_RATE = 20.0

DEFAULT_K_YAW = 0.9
DEFAULT_LOOKAHEAD = 0.6
DEFAULT_REACH = 0.4
DEFAULT_GOAL = 0.25
DEFAULT_SEARCH_WINDOW = 6
DEFAULT_RELOCALIZE = 1.5


def sh(cmd, check=False, capture=False):
    full = f"source {ENV} && {cmd}"
    if capture:
        return subprocess.run(
            ["bash", "-lc", full],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            check=check,
        ).stdout
    return subprocess.run(["bash", "-lc", full], check=check)


def ask(prompt, default=None):
    if default is None:
        s = input(f"{prompt}: ").strip()
        return s
    s = input(f"{prompt} [{default}]: ").strip()
    return s if s else str(default)


def ask_float(prompt, default, min_value=None, max_value=None):
    while True:
        s = ask(prompt, default)
        try:
            v = float(s)
        except ValueError:
            print("请输入数字。")
            continue

        if min_value is not None and v < min_value:
            print(f"不能小于 {min_value}")
            continue
        if max_value is not None and v > max_value:
            print(f"不能大于 {max_value}")
            continue
        return v


def ask_yes_no(prompt, default="n"):
    default = default.lower()
    hint = "Y/n" if default == "y" else "y/N"
    s = input(f"{prompt} ({hint}): ").strip().lower()
    if not s:
        s = default
    return s in ("y", "yes")


def safe_name(name):
    name = name.strip()
    name = re.sub(r"[^a-zA-Z0-9_\-]", "_", name)
    return name


def wait_topic(topic, timeout=30):
    print(f"[检查] 等待话题 {topic} ...")
    t0 = time.time()
    while time.time() - t0 < timeout:
        out = sh("ros2 topic list", capture=True)
        topics = set(line.strip() for line in out.splitlines())
        if topic in topics:
            print(f"[正常] 话题已存在：{topic}")
            return True
        time.sleep(1)
    print(f"[错误] 等待 {topic} 超时。")
    return False


def start_proc(name, cmd, log_file):
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    log_path = LOG_DIR / log_file
    f = open(log_path, "a")
    f.write(f"\n\n===== {datetime.now().isoformat()} start {name} =====\n")
    f.flush()

    full = f"source {ENV} && exec {cmd}"
    p = subprocess.Popen(
        ["bash", "-lc", full],
        stdout=f,
        stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,
    )

    print(f"[启动] {name}, PID={p.pid}, log={log_path}")
    return p, f


def stop_proc(name, p, f=None):
    if p is None:
        return

    print(f"[停止] {name}")
    try:
        os.killpg(os.getpgid(p.pid), signal.SIGINT)
        try:
            p.wait(timeout=3)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(p.pid), signal.SIGTERM)
            p.wait(timeout=2)
    except ProcessLookupError:
        pass
    finally:
        if f:
            f.close()


def send_stopmove():
    sh(
        "ros2 topic pub --once /api/sport/request unitree_api/msg/Request "
        "\"{header: {identity: {id: 9999, api_id: 1003}, lease: {id: 0}, "
        "policy: {priority: 0, noreply: false}}, parameter: '{}', binary: []}\" "
        ">/dev/null 2>&1"
    )


def cleanup_runtime():
    print("[清理] 停止巡检相关节点...")
    sh("pkill -INT -f 'ros2 run go2_fastlio_patrol waypoint_follower' || true")
    sh("pkill -INT -f 'ros2 run go2_fastlio_patrol route_recorder' || true")
    sh("pkill -INT -f 'ros2 run go2_fastlio_patrol unitree_cmd_node' || true")
    time.sleep(1)
    send_stopmove()
    print("[清理] 完成。")


def ensure_base_ready():
    ok = True
    ok = wait_topic("/livox/lidar", 5) and ok
    ok = wait_topic("/livox/imu", 5) and ok
    ok = wait_topic("/Odometry", 5) and ok

    if ok:
        return True

    print("\n[提示] 基础服务未全部就绪。")
    print("你可以先执行：")
    print("  systemctl --user start go2-fastlio-base.service")
    print("或者手动运行：")
    print("  ~/go2_fastlio_ws/scripts/base_bringup.sh\n")

    if ask_yes_no("是否现在尝试启动基础服务", "y"):
        sh("systemctl --user start go2-fastlio-base.service || true")
        return (
            wait_topic("/livox/lidar", 60)
            and wait_topic("/livox/imu", 60)
            and wait_topic("/Odometry", 90)
        )

    return False


def list_routes():
    routes = []

    for p in sorted(ROUTE_ROOT.glob("*.csv")):
        routes.append((p.stem, p))

    if RECORD_ROOT.exists():
        for p in sorted(RECORD_ROOT.glob("*/route.csv")):
            routes.append((p.parent.name, p))

    return routes


def select_route():
    routes = list_routes()
    if not routes:
        print("[错误] 没有找到任何路线文件。")
        return None

    print("\n已有路线：")
    for i, (name, path) in enumerate(routes, 1):
        print(f"  {i}. {name} -> {path}")

    while True:
        s = ask("请输入路线编号或路线名称")
        if s.isdigit():
            idx = int(s)
            if 1 <= idx <= len(routes):
                return routes[idx - 1][1]

        for name, path in routes:
            if s == name or s == str(path):
                return path

        print("没有找到该路线，请重新输入。")


def record_route():
    interval = ask_float("请输入打点间隔 min_distance，单位 m", 0.4, 0.05, 2.0)

    while True:
        name = safe_name(ask("请输入本次录制地图/路线文件夹名称，例如 lab_route_01"))
        if not name:
            print("名称不能为空。")
            continue

        route_dir = RECORD_ROOT / name
        route_file = route_dir / "route.csv"

        if route_file.exists():
            print(f"[错误] 路线已存在，不覆盖：{route_file}")
            continue

        route_dir.mkdir(parents=True, exist_ok=False)
        break

    print("\n[准备录制]")
    print("1. 请把 GO2 放到路线起点。")
    print("2. 机头朝向未来巡检起始方向。")
    print("3. 保持静止 3~5 秒，等待 FAST-LIO 稳定。")
    input("准备好后按 Enter 开始录制...")

    cmd = (
        "ros2 run go2_fastlio_patrol route_recorder --ros-args "
        f"-p min_distance:={interval} "
        f"-p route_file:={route_file}"
    )

    p, f = start_proc("route_recorder", cmd, f"record_{name}.log")

    print("\n[录制中]")
    print("现在人工遥控 GO2 沿路线走一遍。")
    print("走完后回到这里按 Enter 停止录制。")
    input("按 Enter 停止录制...")

    stop_proc("route_recorder", p, f)

    meta = route_dir / "meta.txt"
    meta.write_text(
        f"name={name}\n"
        f"route_file={route_file}\n"
        f"min_distance={interval}\n"
        f"created_at={datetime.now().isoformat()}\n"
    )

    if route_file.exists():
        print(f"[完成] 路线已保存：{route_file}")
        print(f"[信息] 元数据：{meta}")
    else:
        print("[错误] 没有生成路线文件，请检查日志。")


def start_patrol(route_file):
    print("\n[巡检准备]")
    print(f"路线文件：{route_file}")
    print("请把 GO2 放回该路线录制时的起点，机头朝向保持一致。")
    print("放好后让 GO2 静止 3~5 秒。")
    input("确认位置和朝向正确后按 Enter...")

    max_vx = ask_float("请输入速度限速 max_vx，单位 m/s", DEFAULT_VX, 0.05, 0.6)
    max_yaw = ask_float("请输入角速度限速 max_yaw_rate，单位 rad/s", DEFAULT_YAW, 0.1, 1.0)
    pub_rate = ask_float("请输入控制话题发布频率 publish_rate，单位 Hz", DEFAULT_PUBLISH_RATE, 5.0, 50.0)

    loop_mode = ask("请输入巡检模式 once/pingpong", "pingpong")
    if loop_mode not in ("once", "pingpong"):
        print("未知模式，使用 pingpong")
        loop_mode = "pingpong"

    lookahead = ask_float("请输入前视距离 lookahead_distance，单位 m", DEFAULT_LOOKAHEAD, 0.2, 2.0)

    print("\n[即将启动巡检]")
    print(f"max_vx={max_vx}")
    print(f"max_yaw_rate={max_yaw}")
    print(f"publish_rate={pub_rate}")
    print(f"loop_mode={loop_mode}")
    print(f"lookahead_distance={lookahead}")
    input("确认开始巡检请按 Enter，取消按 Ctrl+C...")

    cleanup_runtime()

    cmd_node = (
        "ros2 run go2_fastlio_patrol unitree_cmd_node --ros-args "
        f"-p max_vx:={max_vx} "
        f"-p max_yaw_rate:={max_yaw} "
        f"-p publish_rate:={pub_rate}"
    )

    follower = (
        "ros2 run go2_fastlio_patrol waypoint_follower --ros-args "
        f"-p route_file:={route_file} "
        f"-p v_base:={max_vx} "
        f"-p max_vx:={max_vx} "
        f"-p k_yaw:=0.9 "
        f"-p max_yaw_rate:={max_yaw} "
        f"-p lookahead_distance:={lookahead} "
        f"-p reach_distance:={DEFAULT_REACH} "
        f"-p goal_distance:={DEFAULT_GOAL} "
        f"-p loop_mode:={loop_mode} "
        f"-p search_window:={DEFAULT_SEARCH_WINDOW} "
        f"-p turn_in_place_angle:=1.0 "
        f"-p slow_down_angle:=0.5 "
        f"-p stuck_time:=3.0 "
        f"-p relocalize_distance:={DEFAULT_RELOCALIZE}"
    )

    p_cmd, f_cmd = start_proc("unitree_cmd_node", cmd_node, "unitree_cmd_node.log")
    time.sleep(1.0)
    p_follow, f_follow = start_proc("waypoint_follower", follower, "waypoint_follower.log")

    print("\n[巡检中]")
    print("按 Ctrl+C 停止巡检。")
    print(f"unitree_cmd_node 日志：{LOG_DIR / 'unitree_cmd_node.log'}")
    print(f"waypoint_follower 日志：{LOG_DIR / 'waypoint_follower.log'}")

    try:
        while True:
            if p_cmd.poll() is not None:
                print("[错误] unitree_cmd_node 已退出。")
                break
            if p_follow.poll() is not None:
                print("[提示] waypoint_follower 已退出。")
                break
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n[用户停止] 正在停车...")

    stop_proc("waypoint_follower", p_follow, f_follow)
    stop_proc("unitree_cmd_node", p_cmd, f_cmd)
    send_stopmove()
    print("[完成] 巡检已停止。")


def main():
    print("========================================")
    print(" GO2 FAST-LIO 固定路线巡检交互脚本")
    print("========================================")

    cleanup_runtime()

    if not ensure_base_ready():
        print("[错误] Livox / FAST-LIO 未就绪，退出。")
        sys.exit(1)

    if ask_yes_no("是否录制新路线", "n"):
        record_route()

    route_file = select_route()
    if route_file is None:
        sys.exit(1)

    start_patrol(route_file)


if __name__ == "__main__":
    main()

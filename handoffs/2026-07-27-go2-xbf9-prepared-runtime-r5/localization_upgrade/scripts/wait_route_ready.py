#!/usr/bin/env python3
"""Wait until checkpoint localization explicitly reaches RUNNING."""

from __future__ import annotations

import argparse
import json
import time

import rclpy
from go2_nav_interfaces.msg import RouteStatus
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class RouteReadyWaiter(Node):
    def __init__(self, topic: str) -> None:
        super().__init__("go2_xbf_route_ready_waiter")
        self.latest = None
        self.create_subscription(
            RouteStatus,
            topic,
            self._on_status,
            qos_profile_sensor_data,
        )

    def _on_status(self, message: RouteStatus) -> None:
        self.latest = message


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/checkpoint_localization/route_status")
    parser.add_argument("--timeout-sec", type=float, default=120.0)
    parser.add_argument("--output")
    arguments = parser.parse_args()
    if arguments.timeout_sec <= 0 or arguments.timeout_sec > 600:
        parser.error("--timeout-sec must be within (0, 600]")

    rclpy.init()
    node = RouteReadyWaiter(arguments.topic)
    deadline = time.monotonic() + arguments.timeout_sec
    last_state = None
    observations = []
    result = 1
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            message = node.latest
            if message is None:
                continue
            state = int(message.state)
            if state != last_state:
                observation = {
                    "monotonic_sec": time.monotonic(),
                    "state": state,
                    "reason": str(message.reason),
                    "localization_ready": bool(message.localization_ready),
                }
                observations.append(observation)
                print(json.dumps(observation, ensure_ascii=False), flush=True)
                last_state = state
            if state == int(RouteStatus.RUNNING) and bool(message.localization_ready):
                result = 0
                break
            if state == int(RouteStatus.FAULT):
                result = 2
                break
    finally:
        if arguments.output:
            with open(arguments.output, "w", encoding="utf-8") as handle:
                json.dump(
                    {
                        "ready": result == 0,
                        "exit_code": result,
                        "observations": observations,
                    },
                    handle,
                    ensure_ascii=False,
                    indent=2,
                )
                handle.write("\n")
        node.destroy_node()
        rclpy.shutdown()
    return result


if __name__ == "__main__":
    raise SystemExit(main())

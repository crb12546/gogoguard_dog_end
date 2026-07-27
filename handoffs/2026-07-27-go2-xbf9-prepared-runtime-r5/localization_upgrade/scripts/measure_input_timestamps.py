#!/usr/bin/env python3
"""Measure live FAST-LIO input timestamp age before starting localization."""

from __future__ import annotations

import argparse
import json
import math
import time

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2


def percentile(values: list[float], fraction: float) -> float:
    ordered = sorted(values)
    position = min(len(ordered) - 1, max(0, math.ceil(fraction * len(ordered)) - 1))
    return ordered[position]


class TimestampProbe(Node):
    def __init__(self, odom_topic: str, cloud_topic: str, warmup_sec: float) -> None:
        super().__init__("go2_xbf_input_timestamp_probe")
        self.started = time.monotonic()
        self.warmup_sec = warmup_sec
        self.ages = {"odometry": [], "point_cloud": []}
        self.last_stamp = {"odometry": None, "point_cloud": None}
        self.non_increasing = {"odometry": 0, "point_cloud": 0}
        self.create_subscription(
            Odometry,
            odom_topic,
            lambda message: self._observe("odometry", message),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PointCloud2,
            cloud_topic,
            lambda message: self._observe("point_cloud", message),
            qos_profile_sensor_data,
        )

    def _observe(self, label: str, message) -> None:
        stamp_ns = int(message.header.stamp.sec) * 1_000_000_000 + int(
            message.header.stamp.nanosec
        )
        previous = self.last_stamp[label]
        if previous is not None and stamp_ns <= previous:
            self.non_increasing[label] += 1
        self.last_stamp[label] = stamp_ns
        if time.monotonic() - self.started < self.warmup_sec:
            return
        now_ns = int(self.get_clock().now().nanoseconds)
        self.ages[label].append((now_ns - stamp_ns) * 1.0e-9)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--odom-topic", default="/Odometry")
    parser.add_argument("--cloud-topic", default="/cloud_registered_body")
    parser.add_argument("--duration-sec", type=float, default=10.0)
    parser.add_argument("--warmup-sec", type=float, default=2.0)
    parser.add_argument("--maximum-age-sec", type=float, default=0.50)
    parser.add_argument("--maximum-future-sec", type=float, default=0.10)
    parser.add_argument("--minimum-samples", type=int, default=20)
    parser.add_argument("--output")
    arguments = parser.parse_args()
    if arguments.duration_sec <= arguments.warmup_sec or arguments.minimum_samples < 1:
        parser.error("duration/warmup/minimum-samples are invalid")

    rclpy.init()
    node = TimestampProbe(
        arguments.odom_topic, arguments.cloud_topic, arguments.warmup_sec
    )
    deadline = time.monotonic() + arguments.duration_sec
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    result = {
        "schema": "go2.input_timestamp_probe/v1",
        "duration_sec": arguments.duration_sec,
        "warmup_sec": arguments.warmup_sec,
        "accepted_age_range_sec": [
            -arguments.maximum_future_sec,
            arguments.maximum_age_sec,
        ],
        "topics": {},
        "passed": True,
    }
    for label, values in node.ages.items():
        summary = {
            "sample_count": len(values),
            "minimum_age_sec": min(values) if values else None,
            "p50_age_sec": percentile(values, 0.50) if values else None,
            "p95_age_sec": percentile(values, 0.95) if values else None,
            "p99_age_sec": percentile(values, 0.99) if values else None,
            "maximum_age_sec": max(values) if values else None,
            "non_increasing_count": node.non_increasing[label],
        }
        summary["passed"] = bool(
            len(values) >= arguments.minimum_samples
            and summary["p95_age_sec"] <= arguments.maximum_age_sec
            and summary["minimum_age_sec"] >= -arguments.maximum_future_sec
            and node.non_increasing[label] == 0
        )
        result["topics"][label] = summary
        result["passed"] = bool(result["passed"] and summary["passed"])

    encoded = json.dumps(result, ensure_ascii=False, indent=2) + "\n"
    print(encoded, end="")
    if arguments.output:
        with open(arguments.output, "w", encoding="utf-8") as handle:
            handle.write(encoded)
    return 0 if result["passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())

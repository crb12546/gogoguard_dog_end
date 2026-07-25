#!/usr/bin/env python3
"""Pure helpers for expressing Go2 body yaw in the FAST-LIO route frame."""

import collections
import math


def normalize_angle(angle):
    return math.atan2(math.sin(float(angle)), math.cos(float(angle)))


class BodyYawAlignment:
    """Estimate one startup yaw-origin offset, then keep body yaw authoritative.

    FAST-LIO Euler yaw and Unitree body yaw agree near the route start but
    diverge after the dog turns in a tilted LIO world frame.  A constant
    startup offset expresses the gravity-referenced body yaw in the same
    initial route-angle convention without inheriting the later Euler
    coupling.
    """

    def __init__(self, minimum_samples=10, max_spread_rad=math.radians(2.0)):
        self.minimum_samples = max(3, int(minimum_samples))
        self.max_spread_rad = max(0.0, float(max_spread_rad))
        self._deltas = collections.deque(maxlen=self.minimum_samples)
        self.offset_rad = None
        self.spread_rad = None

    @property
    def ready(self):
        return self.offset_rad is not None

    @property
    def sample_count(self):
        return len(self._deltas)

    def add_pair(self, lio_yaw, body_yaw):
        """Add a locally time-paired sample; return True on first lock."""
        if self.ready:
            return False
        values = (float(lio_yaw), float(body_yaw))
        if not all(math.isfinite(value) for value in values):
            return False
        self._deltas.append(normalize_angle(values[0] - values[1]))
        if len(self._deltas) < self.minimum_samples:
            return False

        mean = math.atan2(
            sum(math.sin(value) for value in self._deltas),
            sum(math.cos(value) for value in self._deltas),
        )
        spread = max(
            abs(normalize_angle(value - mean))
            for value in self._deltas
        )
        self.spread_rad = spread
        if spread > self.max_spread_rad:
            return False

        self.offset_rad = normalize_angle(mean)
        return True

    def aligned_yaw(self, body_yaw):
        if not self.ready:
            return None
        value = float(body_yaw)
        if not math.isfinite(value):
            return None
        return normalize_angle(value + self.offset_rad)

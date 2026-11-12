"""Single-point LiDAR "light gates" looking straight across the road.

Each gate is a beam from (x_gate, mount_y) along +y.  It returns the range to
the near side of the vehicle while the vehicle blocks it.  The gate firmware
detects the clear -> blocked transition and reports a ``gate_enter`` event:

    front of vehicle at x_gate  at  t_enter           (quantised to the sample period)
    range                       =  lane_y - width/2 - mount_y

Two gates a known distance b apart give a speed estimate v = b / (t2 - t1)
with relative error sqrt((sigma_b/b)^2 + (sigma_t/dt)^2); the estimator does
the same thing implicitly when it fuses both events.
"""

from __future__ import annotations

import numpy as np

from ..config import LidarCfg
from .base import Measurement, Sensor


class LidarGate(Sensor):
    def __init__(self, cfg: LidarCfg, index: int, rng: np.random.Generator):
        super().__init__(cfg.timing, rng)
        self.cfg = cfg
        self.index = index
        self.name = f"lidar{index}"
        self.x = float(cfg.gates_x[index])
        offsets = cfg.gate_clock_offsets
        self.clock_offset = float(offsets[index]) if index < len(offsets) else 0.0
        # Random sampling phase -> edge quantisation error uniform in [0, Ts).
        self.next_t = rng.uniform(0.0, 1.0 / cfg.rate_hz)
        self.blocked = False
        self.ranges: list[float] = []

    def _range(self, t: float, world) -> float | None:
        x0, x1, y0, _ = world.vehicle.footprint(t)
        if x0 <= self.x <= x1:
            r = y0 - self.cfg.mount_y
            if 0.0 < r <= self.cfg.max_range:
                return r + self.cfg.range_noise * self.rng.standard_normal()
        return None

    def poll(self, t: float, world) -> list[Measurement]:
        out: list[Measurement] = []
        if not self.cfg.enabled:
            return out
        while self.next_t <= t + 1e-12:
            ts = self.next_t
            self.next_t += 1.0 / self.cfg.rate_hz
            r = self._range(ts, world)
            if r is not None and not self.blocked:
                self.blocked = True
                self.ranges = [r]
                m = self._make("gate_enter", ts, [r], [[self.cfg.range_noise ** 2]],
                               clock_offset=self.clock_offset, gate_x=self.x, mount_y=self.cfg.mount_y,
                               sample_period=1.0 / self.cfg.rate_hz)
                if m:
                    out.append(m)
            elif r is not None:
                self.ranges.append(r)
            elif self.blocked:
                self.blocked = False
                m = self._make("gate_exit", ts, [float(np.median(self.ranges))], [[self.cfg.range_noise ** 2]],
                               clock_offset=self.clock_offset, gate_x=self.x, mount_y=self.cfg.mount_y,
                               samples=len(self.ranges))
                if m:
                    out.append(m)
        return out

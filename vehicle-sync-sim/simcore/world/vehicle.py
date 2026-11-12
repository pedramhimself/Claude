"""Ground-truth vehicle kinematics.

The vehicle drives along the line y = lane_y towards +x.  Its speed follows

    v(t) = v0 + a*t + sigma_a * W(t)        (W = Wiener process)

so that a constant-velocity estimator faces a realistic, slightly wandering
target.  The whole trajectory is pre-computed on a fine grid so that any
module can query the *past* state, which the acoustic model needs (sound
heard now was emitted r/c seconds ago).
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..config import VehicleCfg

KMH = 1.0 / 3.6
REF_SPEED = 50.0 * KMH


@dataclass
class VehicleState:
    t: float
    x: float
    y: float
    vx: float
    vy: float

    @property
    def pos(self) -> np.ndarray:
        return np.array([self.x, self.y])

    @property
    def vel(self) -> np.ndarray:
        return np.array([self.vx, self.vy])


class Vehicle:
    def __init__(self, cfg: VehicleCfg, t_max: float, rng: np.random.Generator, dt: float = 0.001):
        self.cfg = cfg
        self.dt = dt
        n = int(np.ceil((t_max + 2.0) / dt)) + 1
        self.t = np.arange(n) * dt
        v0 = cfg.speed_kmh * KMH
        dv = cfg.accel * dt + cfg.accel_noise * np.sqrt(dt) * rng.standard_normal(n - 1)
        v = np.concatenate([[v0], v0 + np.cumsum(dv)])
        self.v = np.maximum(v, 0.0)
        self.x = cfg.x0 + np.concatenate([[0.0], np.cumsum(0.5 * (self.v[1:] + self.v[:-1]) * dt)])
        self.y = cfg.lane_y
        self.width = cfg.width
        self.length = cfg.length

    # ----------------------------------------------------------------- queries
    def _interp(self, arr: np.ndarray, t: float) -> float:
        # Uniform grid -> O(1) linear interpolation (called thousands of times per run).
        u = t / self.dt
        if u <= 0.0:
            return float(arr[0])
        i = int(u)
        if i >= len(arr) - 1:
            return float(arr[-1])
        f = u - i
        return float(arr[i] * (1.0 - f) + arr[i + 1] * f)

    def state(self, t: float) -> VehicleState:
        return VehicleState(t, self._interp(self.x, t), self.y, self._interp(self.v, t), 0.0)

    def speed(self, t: float) -> float:
        return self._interp(self.v, t)

    def time_at_x(self, x: float) -> float:
        """Time when the vehicle centre passes ``x`` (x is monotonic)."""
        if x <= self.x[0]:
            return float(self.t[0])
        if x >= self.x[-1]:
            return float("inf")
        return float(np.interp(x, self.x, self.t))

    def source_level_db(self, t: float) -> float:
        """Sound pressure level at 7.5 m (pass-by reference distance)."""
        v = max(self.speed(t), 0.5)
        return self.cfg.source_level_db + self.cfg.speed_exponent_db * np.log10(v / REF_SPEED)

    def footprint(self, t: float) -> tuple[float, float, float, float]:
        """Axis-aligned bounding box (x_min, x_max, y_min, y_max)."""
        s = self.state(t)
        return (s.x - self.length / 2, s.x + self.length / 2, s.y - self.width / 2, s.y + self.width / 2)

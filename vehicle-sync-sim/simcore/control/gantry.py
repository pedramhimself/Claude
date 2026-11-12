"""Two-axis (X along the road, Y across it) gantry carrying the launcher.

Each axis is a mass with viscous friction driven by a force-limited motor:

    m x'' = F - b x'

controlled by feed-forward plus PID around the online motion profile:

    F = m a_ref + b v_ref + K_p e + K_d e' + K_i ∫ e,     |F| <= F_max
    K_p = m w_n^2,   K_d = 2 zeta w_n m

so the closed-loop error dynamics are a 2nd-order system with natural
frequency w_n and damping zeta.  The encoder adds quantisation-like noise.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..config import GantryCfg
from .profiles import OnlineProfile


@dataclass
class AxisState:
    pos: float
    vel: float
    ref_pos: float
    ref_vel: float
    force: float


class Axis:
    def __init__(self, cfg: GantryCfg, pos: float, lo: float, hi: float, rng: np.random.Generator):
        self.cfg = cfg
        self.lo, self.hi = lo, hi
        self.profile = OnlineProfile(pos, cfg.v_max, cfg.a_max)
        self.pos, self.vel = pos, 0.0
        self.integral = 0.0
        self.force = 0.0
        self.rng = rng
        self.kp = cfg.axis_mass * cfg.natural_freq ** 2
        self.kd = 2 * cfg.damping * cfg.natural_freq * cfg.axis_mass

    def set_target(self, target: float) -> float:
        target = float(np.clip(target, self.lo, self.hi))
        self.profile.target = target
        return target

    def step(self, dt: float) -> None:
        c = self.cfg
        r_pos, r_vel, r_acc = self.profile.step(dt)
        meas = self.pos + c.position_noise * self.rng.standard_normal()
        e, de = r_pos - meas, r_vel - self.vel
        self.integral = np.clip(self.integral + e * dt, -0.05, 0.05)
        f = c.axis_mass * r_acc + c.friction * r_vel + self.kp * e + self.kd * de + c.ki * self.integral
        self.force = float(np.clip(f, -c.force_max, c.force_max))
        acc = (self.force - c.friction * self.vel) / c.axis_mass
        self.vel += acc * dt           # semi-implicit Euler
        self.pos += self.vel * dt
        if self.pos < self.lo or self.pos > self.hi:  # hard end stops
            self.pos = float(np.clip(self.pos, self.lo, self.hi))
            self.vel = 0.0

    def state(self) -> AxisState:
        return AxisState(self.pos, self.vel, self.profile.pos, self.profile.vel, self.force)


class Gantry:
    def __init__(self, cfg: GantryCfg, rng: np.random.Generator):
        self.cfg = cfg
        hx, hy = cfg.home
        self.x = Axis(cfg, hx, cfg.x_range[0], cfg.x_range[1], rng)
        self.y = Axis(cfg, hy, cfg.y_range[0], cfg.y_range[1], rng)

    def set_target(self, x: float, y: float) -> tuple[float, float]:
        return self.x.set_target(x), self.y.set_target(y)

    @property
    def target(self) -> np.ndarray:
        return np.array([self.x.profile.target, self.y.profile.target])

    def step(self, dt: float) -> None:
        self.x.step(dt)
        self.y.step(dt)

    @property
    def pos(self) -> np.ndarray:
        return np.array([self.x.pos, self.y.pos])

    @property
    def vel(self) -> np.ndarray:
        return np.array([self.x.vel, self.y.vel])

    def reachable(self, x: float, y: float) -> bool:
        c = self.cfg
        return c.x_range[0] <= x <= c.x_range[1] and c.y_range[0] <= y <= c.y_range[1]

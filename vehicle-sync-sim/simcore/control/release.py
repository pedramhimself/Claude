"""Release / launch actuator (servo-released spring launcher on the carriage).

The planner commands the release at t_cmd = t_rel - tau_mean.  The actual
release happens at t_cmd + tau, tau ~ N(tau_mean, sigma_tau): the mean is
compensated, the jitter is not.  At that instant the payload leaves with

    v0 = launcher velocity (speed u + noise, elevation alpha + noise) + carriage velocity
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..config import PayloadCfg, ReleaseCfg
from ..planning.ballistics import launch_velocity


@dataclass
class LaunchEvent:
    t_command: float
    t_release: float
    position: np.ndarray       # (x, y, z) at release
    velocity: np.ndarray       # (vx, vy, vz) at release
    launch_speed: float
    elevation_deg: float
    carriage_velocity: np.ndarray


class ReleaseActuator:
    def __init__(self, release: ReleaseCfg, payload: PayloadCfg, rng: np.random.Generator):
        self.cfg = release
        self.payload = payload
        self.rng = rng
        self.t_command: float | None = None
        self.t_fire: float | None = None
        self.fired = False

    def command(self, t: float) -> None:
        if self.t_command is None:
            self.t_command = t
            self.t_fire = t + max(0.0, self.cfg.latency + self.cfg.latency_jitter * self.rng.standard_normal())

    def due(self, t: float) -> bool:
        return not self.fired and self.t_fire is not None and t >= self.t_fire

    def fire(self, t: float, carriage_pos, carriage_vel) -> LaunchEvent:
        self.fired = True
        p = self.payload
        u = p.launch_speed + p.launch_speed_sigma * self.rng.standard_normal()
        el = p.elevation_deg + p.elevation_sigma_deg * self.rng.standard_normal()
        v0 = launch_velocity(u, el, 0.0, carriage_vel)
        pos = np.array([carriage_pos[0], carriage_pos[1], p.height])
        return LaunchEvent(self.t_command, t, pos, v0, u, el, np.asarray(carriage_vel, float).copy())

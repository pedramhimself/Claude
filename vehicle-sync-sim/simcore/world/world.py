"""Container for ground truth shared by sensors and the payload physics."""

from __future__ import annotations

import numpy as np

from ..config import Config
from .vehicle import Vehicle


class World:
    def __init__(self, cfg: Config, rng: np.random.Generator):
        self.cfg = cfg
        self.env = cfg.environment
        self.vehicle = Vehicle(cfg.vehicle, cfg.sim.t_max, rng, dt=cfg.sim.dt)
        forecast = np.asarray(cfg.environment.wind, dtype=float)
        # True wind = forecast + an unknown per-run deviation the planner cannot see.
        self.wind_forecast = forecast
        self.wind_true = forecast + cfg.environment.gust_sigma * rng.standard_normal(2)

    def vehicle_pos(self, t: float) -> np.ndarray:
        return self.vehicle.state(t).pos

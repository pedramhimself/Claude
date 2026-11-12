"""Continuous-wave Doppler radar (e.g. a 24 GHz speed sensor).

It measures only the *radial* speed of the strongest reflector inside its beam:

    v_r = (r . v) / |r|                      r = p_vehicle - p_radar
    f_d = 2 v_r f_0 / c_light                (24.125 GHz -> ~161 Hz per m/s)

Negative v_r means approaching.  Because the vehicle is offset from the
radar, |v_r| = |v| cos(phi) < |v| (cosine error); the estimator removes it
with the nonlinear measurement model rather than a fixed correction.
"""

from __future__ import annotations

import numpy as np

from ..config import RadarCfg
from .base import Measurement, Sensor

C_LIGHT = 299_792_458.0


class DopplerRadar(Sensor):
    name = "radar"

    def __init__(self, cfg: RadarCfg, rng: np.random.Generator):
        super().__init__(cfg.timing, rng)
        self.cfg = cfg
        self.pos = np.asarray(cfg.position, dtype=float)
        self.next_t = 0.0

    def sees(self, rel: np.ndarray) -> bool:
        r = float(np.linalg.norm(rel))
        if r > self.cfg.max_range or r < 0.5:
            return False
        az = np.degrees(np.arctan2(rel[0], rel[1]))
        return abs(az - self.cfg.boresight_deg) <= self.cfg.fov_deg / 2

    def poll(self, t: float, world) -> list[Measurement]:
        if not self.cfg.enabled or t + 1e-12 < self.next_t:
            return []
        self.next_t += 1.0 / self.cfg.rate_hz
        st = world.vehicle.state(t)
        # The strongest reflector is the front of the vehicle (bumper / grille).
        rel = st.pos + np.array([world.vehicle.length / 2, 0.0]) - self.pos
        if not self.sees(rel):
            return []
        vr_true = float(np.dot(rel, st.vel) / np.linalg.norm(rel))
        vr = vr_true + self.cfg.bias + self.cfg.noise * self.rng.standard_normal()
        fd = 2.0 * vr * self.cfg.carrier_hz / C_LIGHT
        m = self._make("radial_speed", t, [vr], [[self.cfg.noise ** 2]],
                       radar=self.pos.tolist(), doppler_hz=fd, truth=vr_true)
        return [m] if m else []

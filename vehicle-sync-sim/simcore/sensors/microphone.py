"""Two-microphone array: sound level, bearing (TDOA) and dominant tone.

Per analysis frame the array reports

* ``spl``      -- broadband level [dB], vehicle + fluctuating background + noise
* ``bearing``  -- sin(theta) = c * dt / D from the time difference of arrival,
                  theta measured from the array broadside (+y) towards +x.
                  Only reported when the vehicle is ``min_snr_db`` above background.
* ``tone``     -- Doppler-shifted frequency of the vehicle's dominant tone.

Everything is evaluated at the *retarded* emission time, so the bearing
lags the true vehicle position by roughly v * r / c (about 1 m at 30 m and
50 km/h).  The estimator can compensate for that (``compensate_sound_delay``).
"""

from __future__ import annotations

import numpy as np

from ..config import EnvironmentCfg, MicrophoneCfg
from ..world.acoustics import combine_db, doppler_observed, retarded_time, spl_at_distance
from .base import Measurement, Sensor


class MicrophoneArray(Sensor):
    name = "mic"

    def __init__(self, cfg: MicrophoneCfg, env: EnvironmentCfg, rng: np.random.Generator):
        super().__init__(cfg.timing, rng)
        self.cfg = cfg
        self.env = env
        self.pos = np.asarray(cfg.position, dtype=float)
        self.next_t = 0.0

    def poll(self, t: float, world) -> list[Measurement]:
        out: list[Measurement] = []
        if not self.cfg.enabled or t + 1e-12 < self.next_t:
            return out
        self.next_t += self.cfg.frame_dt
        veh = world.vehicle
        c = self.env.sound_speed
        te = retarded_time(lambda s: veh.state(s).pos, t, self.pos, c)
        st = veh.state(te)
        rel = st.pos - self.pos
        r = float(np.linalg.norm(rel))
        vehicle_db = spl_at_distance(veh.source_level_db(te), r)
        background = self.env.background_db + self.env.background_sigma_db * self.rng.standard_normal()
        level = combine_db(vehicle_db, background) + self.cfg.level_noise_db * self.rng.standard_normal()
        snr = vehicle_db - self.env.background_db
        m = self._make("spl", t, [level], [[self.cfg.level_noise_db ** 2]], snr=snr, true_db=vehicle_db)
        if m:
            out.append(m)
        if snr >= self.cfg.min_snr_db:
            sin_theta = np.clip(rel[0] / r + self.cfg.sin_bearing_sigma * self.rng.standard_normal(), -1.0, 1.0)
            m = self._make("bearing", t, [sin_theta], [[self.cfg.sin_bearing_sigma ** 2]],
                           mic=self.pos.tolist(), sound_speed=c)
            if m:
                out.append(m)
            f = doppler_observed(veh.cfg.tone_hz, st.pos, st.vel, self.pos, c)
            f += self.cfg.freq_sigma_hz * self.rng.standard_normal()
            m = self._make("tone", t, [f], [[self.cfg.freq_sigma_hz ** 2]])
            if m:
                out.append(m)
        return out

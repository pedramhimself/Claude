"""Turns raw sensor measurements into EKF updates and track summaries.

Track life cycle:  (waiting for trigger)  ->  initialised at the first light
gate  ->  updated by radar, gates, lateral ranges and microphone bearings.

Initialisation from the first ``gate_enter`` event:

    x_f  = x_gate                          at t_cross = t_stamp - Ts/2
    y_c  = mount_y + range + W_nom/2
    v_x  = v_r * rho / (x_f - x_radar)     from the latest radar reading (vy = 0)
           (falls back to the configured prior if the radar has not reported)
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..config import Config
from ..sensors.base import Measurement
from .ekf import TrackEKF, bearing_model, gate_model, lateral_model, radial_speed_model

KMH = 1.0 / 3.6


@dataclass
class TrackSnapshot:
    t: float
    x_front: float
    x_centre: float
    y: float
    vx: float
    vy: float
    speed: float
    sigma_x: float
    sigma_y: float
    sigma_v: float
    P: np.ndarray

    def to_dict(self) -> dict:
        d = {k: getattr(self, k) for k in ("t", "x_front", "x_centre", "y", "vx", "vy", "speed",
                                            "sigma_x", "sigma_y", "sigma_v")}
        d["P"] = self.P.tolist()
        return d


class Tracker:
    def __init__(self, cfg: Config):
        self.cfg = cfg
        e = cfg.estimator
        self.ecfg = e
        self.ekf = TrackEKF(e.accel_sigma, e.lateral_accel_sigma)
        self.half_len = e.nominal_length / 2
        self._last_radar: Measurement | None = None
        self.events: list[dict] = []
        self.innovations: list[dict] = []

    @property
    def active(self) -> bool:
        return self.ekf.initialized

    # ----------------------------------------------------------------- ingest
    def ingest(self, m: Measurement) -> None:
        e = self.ecfg
        if m.kind == "tone":
            return  # logged for display only; see world.acoustics.doppler_speed
        if m.kind == "radial_speed":
            if not e.use_radar:
                return
            self._last_radar = m
            if self.active:
                self._fuse(m, radial_speed_model(m.z[0], m.R[0, 0], m.meta["radar"]))
            return
        if m.kind in ("gate_enter", "gate_exit"):
            if not e.use_lidar:
                return
            y_c = m.meta["mount_y"] + m.z[0] + e.nominal_width / 2
            var_y = m.R[0, 0] + (e.width_sigma / 2) ** 2
            if m.kind == "gate_exit":
                if self.active:
                    self._fuse(m, lateral_model(y_c, var_y))
                return
            ts = m.meta["sample_period"]
            t_cross = m.t_stamp - ts / 2  # crossing happened somewhere in the last sample period
            if not self.active:
                self._initialize(t_cross, m.meta["gate_x"], y_c, var_y)
                self.events.append({"t": t_cross, "kind": "track_init", "sensor": m.sensor})
            else:
                self._fuse(m, gate_model(m.meta["gate_x"], y_c, ts ** 2 / 12 + 1e-6, var_y), t_cross)
            return
        if m.kind == "bearing" and e.use_mic_bearing and self.active:
            self._fuse(m, bearing_model(m.z[0], m.R[0, 0] * e.bearing_var_inflation, m.meta["mic"], self.half_len,
                                        m.meta["sound_speed"], e.compensate_sound_delay))

    def _initialize(self, t: float, x_gate: float, y_c: float, var_y: float) -> None:
        e = self.ecfg
        vx, var_v = e.init_speed_kmh * KMH, e.init_speed_sigma ** 2
        r = self._last_radar
        if r is not None and t - r.t_stamp < 0.5:
            pr = np.asarray(r.meta["radar"], float)
            rel = np.array([x_gate, y_c]) - pr
            rho = float(np.linalg.norm(rel))
            if abs(rel[0]) > 1.0:
                cos_phi = abs(rel[0]) / rho
                vx = abs(r.z[0]) / cos_phi
                var_v = r.R[0, 0] / cos_phi ** 2 + 0.3 ** 2
        P = np.diag([0.05 ** 2, var_y, var_v, 0.1 ** 2])
        self.ekf.initialize(t, np.array([x_gate, y_c, vx, 0.0]), P)

    def _fuse(self, m: Measurement, model, t: float | None = None) -> None:
        t = m.t_stamp if t is None else t
        res = self.ekf.process(t, model)
        self.innovations.append({"t": t, "sensor": m.sensor, "kind": model.name,
                                 "accepted": bool(res.accepted), "nis": float(res.nis)})

    # ---------------------------------------------------------------- outputs
    def snapshot(self, t: float) -> TrackSnapshot | None:
        if not self.active:
            return None
        s, P = self.ekf.predict(t)
        speed = float(np.hypot(s[2], s[3]))
        # variance of speed along the velocity direction
        u = s[2:] / max(speed, 1e-6)
        var_v = float(u @ P[2:, 2:] @ u)
        return TrackSnapshot(t, float(s[0]), float(s[0] - self.half_len), float(s[1]), float(s[2]), float(s[3]),
                             speed, float(np.sqrt(P[0, 0])), float(np.sqrt(P[1, 1])), float(np.sqrt(var_v)), P)

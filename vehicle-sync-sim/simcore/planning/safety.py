"""Safety gate evaluated before (and continuously up to) the release.

Release is permitted only when *every* check passes:

1. ``in_zone``         the k-sigma landing ellipse lies inside the declared zone
2. ``lane_clearance``  (y_c_hat - W/2 - k sigma_vehicle) - (y_aim + k sigma_land_y) >= clearance_min
3. ``track_quality``   predicted position sigma and speed sigma below limits
4. ``speed_plausible`` estimated speed inside the configured range
5. ``schedule``        plan is feasible (non-negative slack)
6. ``carriage_ready``  carriage within tolerance of the release point and nearly at rest
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from ..config import Config
from ..estimation.tracker import TrackSnapshot
from .ballistics import LaunchSolution
from .scheduler import Plan, landing_sigmas

KMH = 1.0 / 3.6


@dataclass
class SafetyStatus:
    ok: bool
    checks: dict[str, dict] = field(default_factory=dict)

    @property
    def failed(self) -> list[str]:
        return [k for k, v in self.checks.items() if not v["ok"]]


class SafetyGate:
    def __init__(self, cfg: Config, launch: LaunchSolution):
        self.cfg = cfg
        self.launch = launch

    def evaluate(self, now: float, track: TrackSnapshot | None, plan: Plan | None,
                 carriage_pos=None, carriage_vel=None, check_carriage: bool = True) -> SafetyStatus:
        cfg, s = self.cfg, self.cfg.safety
        k = s.sigma_k
        checks: dict[str, dict] = {}

        def add(name, ok, value=None, limit=None):
            checks[name] = {"ok": bool(ok), "value": None if value is None else float(value),
                            "limit": None if limit is None else float(limit)}

        if track is None or plan is None:
            add("track_and_plan", False)
            return SafetyStatus(False, checks)

        sx, sy = landing_sigmas(cfg, self.launch)
        z = cfg.zone
        inside = (z.x_range[0] <= plan.x_aim - k * sx and plan.x_aim + k * sx <= z.x_range[1]
                  and z.y_range[0] <= plan.y_aim - k * sy and plan.y_aim + k * sy <= z.y_range[1])
        add("in_zone", inside)

        sig_vehicle = np.hypot(track.sigma_y, cfg.estimator.width_sigma / 2)
        vehicle_edge = track.y - cfg.estimator.nominal_width / 2 - k * sig_vehicle
        landing_edge = plan.y_aim + k * sy
        gap = vehicle_edge - landing_edge
        add("lane_clearance", gap >= s.lane_clearance_min, gap, s.lane_clearance_min)

        pos_sigma = plan.sigma_t * track.speed
        add("track_quality", pos_sigma <= s.max_position_sigma and track.sigma_v <= s.max_speed_sigma,
            pos_sigma, s.max_position_sigma)
        lo, hi = s.speed_range_kmh
        add("speed_plausible", lo * KMH <= track.speed <= hi * KMH, track.speed / KMH, hi)
        add("schedule", plan.feasible or plan.committed, plan.slack, 0.0)

        if check_carriage and carriage_pos is not None:
            err = float(np.hypot(carriage_pos[0] - plan.release_x, carriage_pos[1] - plan.release_y))
            speed = float(np.hypot(*carriage_vel))
            add("carriage_ready", err <= cfg.release.settle_tolerance and speed <= 0.05, err,
                cfg.release.settle_tolerance)
        return SafetyStatus(all(c["ok"] for c in checks.values()), checks)

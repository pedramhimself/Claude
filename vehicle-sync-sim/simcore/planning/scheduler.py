"""Release planning: where to put the carriage and when to release.

The payload must land at a fixed lateral position y_aim inside the test zone
at the moment the vehicle centre is abeam the landing point (plus an optional
offset Delta).  For a candidate along-road aim x_a:

    t_abeam = t_now + (x_a - x_c_hat) / v_hat                (constant-velocity prediction)
    t_land  = t_abeam + Delta
    t_rel   = t_land - T_flight
    t_cmd   = t_rel - tau_release                             (actuator latency compensation)
    p_rel   = (x_a, y_aim) - (dx, dy)                         (landing offset of the launcher)
    slack   = t_cmd - (t_now + t_move(p_carriage -> p_rel))

Uncertainty of the predicted abeam time (x and v correlated, from the EKF):

    sigma_x_pred^2 = [1  h] P_xv [1  h]^T + (sigma_L / 2)^2,   h = t_abeam - t_now
    sigma_t        = sigma_x_pred / v

Candidates are scored by

    J = (v sigma_t)^2 + sigma_land_x^2                         expected along-track miss
      + w_slack * max(0, margin - slack)^2                     timing robustness
      + w_change * (x_a - x_a_prev)^2                          smooth re-planning

and the lowest-cost feasible candidate wins.  Close to the release the aim is
frozen ("committed") and only the release time keeps being refined.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass

import numpy as np

from ..config import Config
from ..control.profiles import move_time_from_state
from ..estimation.tracker import TrackSnapshot
from .ballistics import LaunchSolution


@dataclass
class Plan:
    t_plan: float
    x_aim: float
    y_aim: float
    release_x: float
    release_y: float
    t_abeam: float
    t_land: float
    t_release: float
    t_cmd: float
    time_of_flight: float
    slack: float
    sigma_t: float
    sigma_along: float
    feasible: bool
    committed: bool = False
    cost: float = 0.0

    def to_dict(self) -> dict:
        return {k: (float(v) if isinstance(v, (np.floating, float)) else v) for k, v in asdict(self).items()}


def landing_sigmas(cfg: Config, launch: LaunchSolution) -> tuple[float, float]:
    """1-sigma landing dispersion (x, y): launcher + wind + carriage positioning."""
    pn = cfg.gantry.position_noise
    return float(np.hypot(launch.sigma_x, pn)), float(np.hypot(launch.sigma_y, pn))


class ReleasePlanner:
    def __init__(self, cfg: Config, launch: LaunchSolution):
        self.cfg = cfg
        self.launch = launch
        z = cfg.zone
        self.y_aim = 0.5 * (z.y_range[0] + z.y_range[1])
        k = cfg.safety.sigma_k
        sx, _ = landing_sigmas(cfg, launch)
        margin = k * sx + 1e-6
        lo, hi = z.x_range[0] + margin, z.x_range[1] - margin
        if hi < lo:
            lo = hi = 0.5 * (z.x_range[0] + z.x_range[1])
        self.x_candidates = np.linspace(lo, hi, max(cfg.planner.aim_candidates, 1))
        self.prev: Plan | None = None

    def ready_pose(self) -> tuple[float, float]:
        """Where to park the carriage when the trigger fires (zone centre)."""
        xa = 0.5 * (self.cfg.zone.x_range[0] + self.cfg.zone.x_range[1])
        return xa - self.launch.dx, self.y_aim - self.launch.dy

    def _evaluate(self, now: float, track: TrackSnapshot, x_aim: float, gpos, gvel) -> Plan | None:
        cfg = self.cfg
        v = track.vx
        if v <= 0.1:
            return None
        h = (x_aim - track.x_centre) / v
        if h <= 0:
            return None
        t_abeam = now + h
        t_land = t_abeam + cfg.planner.landing_offset_s
        t_rel = t_land - self.launch.time_of_flight
        t_cmd = t_rel - cfg.release.latency
        rx, ry = x_aim - self.launch.dx, self.y_aim - self.launch.dy
        g = cfg.gantry
        tx = move_time_from_state(gpos[0], gvel[0], rx, g.v_max, g.a_max)
        ty = move_time_from_state(gpos[1], gvel[1], ry, g.v_max, g.a_max)
        slack = t_cmd - (now + max(tx, ty))
        P = track.P
        j = np.array([1.0, h])
        Pxv = P[np.ix_([0, 2], [0, 2])]
        var_x = float(j @ Pxv @ j) + (cfg.estimator.length_sigma / 2) ** 2
        sigma_t = np.sqrt(var_x + (v * cfg.release.latency_jitter) ** 2) / v
        sigma_along = float(np.hypot(v * sigma_t, self.launch.sigma_x))
        reachable = g.x_range[0] <= rx <= g.x_range[1] and g.y_range[0] <= ry <= g.y_range[1]
        feasible = reachable and slack >= 0.0 and t_cmd > now
        p = cfg.planner
        cost = sigma_along ** 2 + p.w_slack * max(0.0, p.slack_margin - slack) ** 2
        if self.prev is not None:
            cost += p.w_change * (x_aim - self.prev.x_aim) ** 2
        if not reachable:
            cost += 1e6
        return Plan(now, float(x_aim), self.y_aim, float(rx), float(ry), t_abeam, t_land, t_rel, t_cmd,
                    self.launch.time_of_flight, float(slack), float(sigma_t), sigma_along, bool(feasible),
                    cost=float(cost))

    def plan(self, now: float, track: TrackSnapshot, gantry_pos, gantry_vel) -> Plan | None:
        if self.prev is not None and self.prev.committed:
            # Aim frozen: refresh only the timing for the same landing point.
            p = self._evaluate(now, track, self.prev.x_aim, gantry_pos, gantry_vel)
            if p is not None:
                p.committed = True
                self.prev = p
            return self.prev
        plans = [p for p in (self._evaluate(now, track, xa, gantry_pos, gantry_vel) for xa in self.x_candidates)
                 if p is not None]
        if not plans:
            return None
        feasible = [p for p in plans if p.feasible]
        best = min(feasible, key=lambda p: p.cost) if feasible else max(plans, key=lambda p: p.slack)
        if now >= best.t_cmd - self.cfg.release.commit_lead:
            best.committed = True
        self.prev = best
        return best

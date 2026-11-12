"""Extended Kalman filter with out-of-sequence measurement handling.

State (plan view):

    s = [x_f, y_c, v_x, v_y]

x_f is the x position of the vehicle's *front* (what the light gates and the
radar actually see), y_c is the lane-centre offset of the vehicle.  The
vehicle centre used for planning is x_c = x_f - L_nom / 2.

Constant-velocity motion with white-noise acceleration:

    s_{k+1} = F s_k + w,      F = [[I, dt I], [0, I]]
    Q = blockdiag over axes of  sigma_a^2 [[dt^4/4, dt^3/2], [dt^3/2, dt^2]]

Measurements are processed at their *timestamp*.  If one arrives late
(timestamp earlier than the filter time) the filter rewinds to the last
stored state before that timestamp and re-processes everything after it, so
sensor latency never biases the estimate (only timestamp errors do).
"""

from __future__ import annotations

from bisect import insort
from dataclasses import dataclass, field
from typing import Callable

import numpy as np

# Chi-square 99.9 % quantiles for 1 and 2 degrees of freedom: outlier gate.
CHI2_GATE = {1: 10.83, 2: 13.82}


def transition(dt: float) -> np.ndarray:
    F = np.eye(4)
    F[0, 2] = F[1, 3] = dt
    return F


def process_noise(dt: float, sigma_ax: float, sigma_ay: float) -> np.ndarray:
    q = np.array([[dt ** 4 / 4, dt ** 3 / 2], [dt ** 3 / 2, dt ** 2]])
    Q = np.zeros((4, 4))
    for axis, sig in ((0, sigma_ax), (1, sigma_ay)):
        idx = [axis, axis + 2]
        Q[np.ix_(idx, idx)] = sig ** 2 * q
    return Q


def numeric_jacobian(h: Callable[[np.ndarray], np.ndarray], s: np.ndarray, eps: float = 1e-5) -> np.ndarray:
    z0 = np.atleast_1d(h(s))
    J = np.zeros((z0.size, s.size))
    for i in range(s.size):
        d = np.zeros_like(s)
        d[i] = eps
        J[:, i] = (np.atleast_1d(h(s + d)) - np.atleast_1d(h(s - d))) / (2 * eps)
    return J


@dataclass(order=True)
class _Entry:
    t: float
    seq: int
    model: "MeasurementModel" = field(compare=False)
    s: np.ndarray | None = field(default=None, compare=False)
    P: np.ndarray | None = field(default=None, compare=False)
    accepted: bool = field(default=True, compare=False)
    nis: float = field(default=0.0, compare=False)


@dataclass
class MeasurementModel:
    """z = h(s) + v,  v ~ N(0, R)."""
    name: str
    z: np.ndarray
    R: np.ndarray
    h: Callable[[np.ndarray], np.ndarray]
    H: Callable[[np.ndarray], np.ndarray] | None = None
    gate: bool = True

    def jacobian(self, s: np.ndarray) -> np.ndarray:
        return self.H(s) if self.H is not None else numeric_jacobian(self.h, s)


@dataclass
class UpdateResult:
    accepted: bool
    nis: float
    innovation: np.ndarray


class TrackEKF:
    def __init__(self, sigma_ax: float, sigma_ay: float, max_history: float = 3.0):
        self.sigma_ax = sigma_ax
        self.sigma_ay = sigma_ay
        self.max_history = max_history
        self.initialized = False
        self.t = 0.0
        self.s = np.zeros(4)
        self.P = np.eye(4)
        self._t0 = 0.0
        self._s0 = self.s
        self._P0 = self.P
        self._entries: list[_Entry] = []
        self._seq = 0
        self.rejected = 0
        self.replays = 0

    # ------------------------------------------------------------- lifecycle
    def initialize(self, t: float, s: np.ndarray, P: np.ndarray) -> None:
        self.t, self.s, self.P = t, np.asarray(s, float).copy(), np.asarray(P, float).copy()
        self._t0, self._s0, self._P0 = self.t, self.s.copy(), self.P.copy()
        self._entries.clear()
        self.initialized = True

    # ------------------------------------------------------------ primitives
    def _propagate(self, s: np.ndarray, P: np.ndarray, dt: float) -> tuple[np.ndarray, np.ndarray]:
        if abs(dt) < 1e-12:
            return s.copy(), P.copy()
        F = transition(dt)
        Q = process_noise(abs(dt), self.sigma_ax, self.sigma_ay)
        return F @ s, F @ P @ F.T + Q

    @staticmethod
    def _update(s: np.ndarray, P: np.ndarray, model: MeasurementModel) -> tuple[np.ndarray, np.ndarray, UpdateResult]:
        H = model.jacobian(s)
        y = model.z - np.atleast_1d(model.h(s))
        S = H @ P @ H.T + model.R
        Sinv = np.linalg.inv(S)
        nis = float(y @ Sinv @ y)
        if model.gate and nis > CHI2_GATE.get(y.size, 16.0):
            return s, P, UpdateResult(False, nis, y)
        K = P @ H.T @ Sinv
        I_KH = np.eye(len(s)) - K @ H
        P_new = I_KH @ P @ I_KH.T + K @ model.R @ K.T   # Joseph form
        return s + K @ y, 0.5 * (P_new + P_new.T), UpdateResult(True, nis, y)

    # ------------------------------------------------------------ public API
    def process(self, t_meas: float, model: MeasurementModel) -> UpdateResult:
        """Fuse one measurement taken at ``t_meas`` (may be older than the filter time)."""
        if not self.initialized:
            raise RuntimeError("filter not initialised")
        if t_meas < self._t0:
            return UpdateResult(False, float("nan"), np.zeros(model.z.size))
        self._seq += 1
        entry = _Entry(t_meas, self._seq, model)
        insort(self._entries, entry)
        idx = self._entries.index(entry)
        if idx < len(self._entries) - 1:
            self.replays += 1
        # Restore the state just before this measurement and replay forwards.
        if idx == 0:
            t, s, P = self._t0, self._s0, self._P0
        else:
            prev = self._entries[idx - 1]
            t, s, P = prev.t, prev.s, prev.P
        result = None
        for e in self._entries[idx:]:
            s, P = self._propagate(s, P, e.t - t)
            s, P, res = self._update(s, P, e.model)
            e.s, e.P, e.accepted, e.nis = s, P, res.accepted, res.nis
            t = e.t
            if e is entry:
                result = res
        self.t, self.s, self.P = t, s, P
        if result is not None and not result.accepted:
            self.rejected += 1
        self._prune()
        return result

    def _prune(self) -> None:
        horizon = self.t - self.max_history
        while len(self._entries) > 1 and self._entries[0].t < horizon:
            e = self._entries.pop(0)
            self._t0, self._s0, self._P0 = e.t, e.s, e.P

    def predict(self, t: float) -> tuple[np.ndarray, np.ndarray]:
        """State and covariance extrapolated to time ``t`` (no side effects)."""
        return self._propagate(self.s, self.P, t - self.t)


# ---------------------------------------------------------------------------
# Measurement models.  Each builder returns a MeasurementModel for one reading.
# ---------------------------------------------------------------------------

def radial_speed_model(z: float, var: float, radar_pos) -> MeasurementModel:
    pr = np.asarray(radar_pos, float)

    def h(s):
        rel = s[:2] - pr
        return np.array([rel @ s[2:] / np.linalg.norm(rel)])

    def H(s):
        rel = s[:2] - pr
        rho = np.linalg.norm(rel)
        v = s[2:]
        dpos = v / rho - (rel @ v) * rel / rho ** 3
        return np.array([[dpos[0], dpos[1], rel[0] / rho, rel[1] / rho]])

    return MeasurementModel("radial_speed", np.array([z]), np.array([[var]]), h, H)


def gate_model(gate_x: float, y_center: float, var_x: float, var_y: float) -> MeasurementModel:
    H = np.array([[1.0, 0, 0, 0], [0, 1.0, 0, 0]])
    return MeasurementModel("gate", np.array([gate_x, y_center]), np.diag([var_x, var_y]),
                            lambda s: H @ s, lambda s: H)


def lateral_model(y_center: float, var_y: float) -> MeasurementModel:
    H = np.array([[0, 1.0, 0, 0]])
    return MeasurementModel("lateral", np.array([y_center]), np.array([[var_y]]), lambda s: H @ s, lambda s: H)


def bearing_model(z_sin: float, var: float, mic_pos, centre_offset: float, sound_speed: float,
                  compensate_delay: bool) -> MeasurementModel:
    """sin(theta) of the sound source as heard at the array.

    The source (vehicle centre, x_f - L/2) is observed where it was r/c seconds
    ago:  p_e = p - v * r / c   (first-order retarded position).
    """
    pm = np.asarray(mic_pos, float)

    def h(s):
        p = np.array([s[0] - centre_offset, s[1]])
        if compensate_delay:
            r = np.linalg.norm(p - pm)
            p = p - s[2:] * r / sound_speed
        rel = p - pm
        return np.array([rel[0] / np.linalg.norm(rel)])

    return MeasurementModel("bearing", np.array([z_sin]), np.array([[var]]), h)

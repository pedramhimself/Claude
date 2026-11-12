"""Acoustic propagation helpers.

Point-source spherical spreading (free field):

    L(r) = L_ref - 20 log10(r / r_ref)

Sound heard at time t left the vehicle at the *retarded* time t_e that solves

    t_e = t - |p(t_e) - p_mic| / c

and its frequency is Doppler shifted:

    f_obs = f_0 * c / (c - v . u)        u = unit vector source -> receiver
"""

from __future__ import annotations

import numpy as np

R_REF = 7.5  # pass-by reference distance (ISO 362 style)


def spl_at_distance(level_ref_db: float, r: float, r_ref: float = R_REF) -> float:
    return level_ref_db - 20.0 * np.log10(max(r, 0.1) / r_ref)


def combine_db(*levels_db: float) -> float:
    """Energetic sum of incoherent sources."""
    return float(10.0 * np.log10(np.sum(10.0 ** (np.asarray(levels_db) / 10.0))))


def retarded_time(position_at, t: float, receiver: np.ndarray, c: float, iterations: int = 4) -> float:
    """Fixed-point solve of t_e = t - |p(t_e) - receiver| / c.

    Converges quickly because |v| << c (contraction factor ~ v/c).
    """
    te = t
    for _ in range(iterations):
        te = t - float(np.linalg.norm(position_at(te) - receiver)) / c
    return te


def doppler_observed(f0: float, src_pos: np.ndarray, src_vel: np.ndarray, receiver: np.ndarray, c: float) -> float:
    u = receiver - src_pos
    n = np.linalg.norm(u)
    if n < 1e-9:
        return f0
    v_towards = float(np.dot(src_vel, u / n))
    return f0 * c / (c - v_towards)


def doppler_speed(f_approach: float, f_recede: float, c: float) -> float:
    """Speed from the asymptotic approach / recede frequencies (independent of distance).

    f_a = f0 c/(c - v), f_r = f0 c/(c + v)  =>  v = c (f_a - f_r) / (f_a + f_r)
    """
    return c * (f_approach - f_recede) / (f_approach + f_recede)


def passby_level(t: np.ndarray, level_cpa: float, v: float, d: float, t0: float) -> np.ndarray:
    """Level history of a pass-by: L(t) = L_cpa - 10 log10(1 + (v (t - t0) / d)^2).

    Only the ratio v/d is identifiable from this curve, which is why a single
    microphone cannot measure distance without knowing the source level.
    """
    return level_cpa - 10.0 * np.log10(1.0 + (v * (np.asarray(t) - t0) / d) ** 2)

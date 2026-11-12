"""Payload flight model.

Equation of motion with quadratic drag relative to the air (wind w):

    m dv/dt = m g - 1/2 rho C_d A |v - w| (v - w)
    =>  dv/dt = g - k |v - w| (v - w),        k = rho C_d A / (2 m)

integrated with classical RK4 until the payload reaches the ground (z = 0).

Without drag (k = 0), launched from height h at speed u, elevation alpha:

    T = ( u sin(alpha) + sqrt(u^2 sin^2(alpha) + 2 g h) ) / g
    R = u cos(alpha) T

The initial velocity is the launcher velocity *plus the carriage velocity*,
so releasing from a moving gantry shifts the landing point by v_carriage * T.
"""

from __future__ import annotations

from dataclasses import dataclass
from functools import lru_cache

import numpy as np

from ..config import PayloadCfg


def drag_constant(p: PayloadCfg, air_density: float) -> float:
    area = np.pi * (p.diameter / 2) ** 2
    return 0.5 * air_density * p.drag_coeff * area / p.mass


def launch_velocity(speed: float, elevation_deg: float, azimuth_deg: float = 0.0,
                    carriage_vel=(0.0, 0.0)) -> np.ndarray:
    """Launcher points along +y (azimuth 0), azimuth rotates towards +x."""
    el, az = np.radians(elevation_deg), np.radians(azimuth_deg)
    horiz = speed * np.cos(el)
    return np.array([horiz * np.sin(az) + carriage_vel[0], horiz * np.cos(az) + carriage_vel[1],
                     speed * np.sin(el)])


@dataclass
class Flight:
    t: np.ndarray
    pos: np.ndarray        # (n, 3)
    time_of_flight: float
    landing: np.ndarray    # (x, y)
    impact_speed: float


def simulate_flight(p0, v0, k: float, g: float = 9.81, wind=(0.0, 0.0), dt: float = 0.002,
                    t_limit: float = 10.0) -> Flight:
    p = np.asarray(p0, float).copy()
    v = np.asarray(v0, float).copy()
    w = np.array([wind[0], wind[1], 0.0])
    gv = np.array([0.0, 0.0, -g])

    def acc(vel):
        rel = vel - w
        return gv - k * np.linalg.norm(rel) * rel

    ts, ps = [0.0], [p.copy()]
    t = 0.0
    while t < t_limit:
        # RK4 on (p, v)
        a1 = acc(v)
        k1p, k1v = v, a1
        k2p, k2v = v + 0.5 * dt * k1v, acc(v + 0.5 * dt * k1v)
        k3p, k3v = v + 0.5 * dt * k2v, acc(v + 0.5 * dt * k2v)
        k4p, k4v = v + dt * k3v, acc(v + dt * k3v)
        p_new = p + dt / 6 * (k1p + 2 * k2p + 2 * k3p + k4p)
        v_new = v + dt / 6 * (k1v + 2 * k2v + 2 * k3v + k4v)
        if p_new[2] <= 0.0:
            frac = p[2] / (p[2] - p_new[2])
            p_land = p + frac * (p_new - p)
            v_land = v + frac * (v_new - v)
            t += frac * dt
            ts.append(t)
            ps.append(p_land)
            return Flight(np.array(ts), np.array(ps), t, p_land[:2].copy(), float(np.linalg.norm(v_land)))
        p, v, t = p_new, v_new, t + dt
        ts.append(t)
        ps.append(p.copy())
    raise RuntimeError("payload did not land within the time limit")


def flight_time_no_drag(speed: float, elevation_deg: float, height: float, g: float = 9.81) -> float:
    vz = speed * np.sin(np.radians(elevation_deg))
    return (vz + np.sqrt(vz ** 2 + 2 * g * height)) / g


def range_no_drag(speed: float, elevation_deg: float, height: float, g: float = 9.81) -> float:
    return speed * np.cos(np.radians(elevation_deg)) * flight_time_no_drag(speed, elevation_deg, height, g)


@dataclass(frozen=True)
class LaunchSolution:
    """Landing offset relative to the release point (static carriage)."""
    dx: float
    dy: float
    time_of_flight: float
    sigma_x: float          # landing dispersion from launcher + wind uncertainty
    sigma_y: float
    impact_speed: float


@lru_cache(maxsize=256)
def launch_solution(speed: float, elevation_deg: float, height: float, k: float, g: float,
                    wind_x: float, wind_y: float, speed_sigma: float, elev_sigma_deg: float,
                    wind_sigma: float) -> LaunchSolution:
    """Nominal landing offset plus a first-order dispersion estimate.

    sigma_y^2 = (dR/du sigma_u)^2 + (dR/dalpha sigma_alpha)^2 + (dR/dw_y sigma_w)^2
    sigma_x^2 = (dX/dw_x sigma_w)^2
    """
    def land(u=speed, el=elevation_deg, wx=wind_x, wy=wind_y):
        f = simulate_flight([0, 0, height], launch_velocity(u, el), k, g, (wx, wy))
        return f

    nom = land()
    du, de, dw = 0.05, 0.2, 0.2
    dR_du = (land(u=speed + du).landing[1] - land(u=speed - du).landing[1]) / (2 * du)
    dR_de = (land(el=elevation_deg + de).landing[1] - land(el=elevation_deg - de).landing[1]) / (2 * de)
    dR_dwy = (land(wy=wind_y + dw).landing[1] - land(wy=wind_y - dw).landing[1]) / (2 * dw)
    dX_dwx = (land(wx=wind_x + dw).landing[0] - land(wx=wind_x - dw).landing[0]) / (2 * dw)
    sy = np.sqrt((dR_du * speed_sigma) ** 2 + (dR_de * elev_sigma_deg) ** 2 + (dR_dwy * wind_sigma) ** 2)
    sx = abs(dX_dwx * wind_sigma)
    return LaunchSolution(float(nom.landing[0]), float(nom.landing[1]), nom.time_of_flight,
                          float(sx), float(sy), nom.impact_speed)


def solve_speed_for_range(target_range: float, elevation_deg: float, height: float, k: float,
                          g: float = 9.81, lo: float = 0.5, hi: float = 30.0, tol: float = 1e-4) -> float:
    """Shooting method: bisection on launch speed so that the range hits the target.

    Range is monotonic in speed for a fixed elevation below ~45 deg.
    """
    def rng(u):
        return simulate_flight([0, 0, height], launch_velocity(u, elevation_deg), k, g).landing[1]

    if not (rng(lo) <= target_range <= rng(hi)):
        raise ValueError("target range not reachable with the given speed bounds")
    while hi - lo > tol:
        mid = 0.5 * (lo + hi)
        if rng(mid) < target_range:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)

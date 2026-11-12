import numpy as np
import pytest

from simcore.config import Config
from simcore.planning.ballistics import (drag_constant, flight_time_no_drag, launch_solution, launch_velocity,
                                         range_no_drag, simulate_flight, solve_speed_for_range)
from simcore.world.acoustics import (combine_db, doppler_observed, doppler_speed, passby_level, retarded_time,
                                     spl_at_distance)


def test_no_drag_flight_matches_closed_form():
    u, el, h = 7.0, 35.0, 1.0
    f = simulate_flight([0, 0, h], launch_velocity(u, el), k=0.0)
    assert f.time_of_flight == pytest.approx(flight_time_no_drag(u, el, h), abs=2e-4)
    assert f.landing[1] == pytest.approx(range_no_drag(u, el, h), abs=2e-3)
    assert f.landing[0] == pytest.approx(0.0, abs=1e-9)


def test_drag_shortens_range_and_carriage_velocity_shifts_landing():
    cfg = Config()
    k = drag_constant(cfg.payload, 1.2)
    no_drag = simulate_flight([0, 0, 1], launch_velocity(7, 35), 0.0)
    drag = simulate_flight([0, 0, 1], launch_velocity(7, 35), k)
    assert drag.landing[1] < no_drag.landing[1]
    moving = simulate_flight([0, 0, 1], launch_velocity(7, 35, carriage_vel=(0.5, 0.0)), 0.0)
    assert moving.landing[0] == pytest.approx(0.5 * moving.time_of_flight, rel=1e-3)


def test_shooting_method_inverts_range():
    k = drag_constant(Config().payload, 1.2)
    u = solve_speed_for_range(5.0, 35.0, 1.0, k)
    assert simulate_flight([0, 0, 1], launch_velocity(u, 35), k).landing[1] == pytest.approx(5.0, abs=1e-3)


def test_launch_solution_dispersion_positive():
    s = launch_solution(7.0, 35.0, 1.0, 0.036, 9.81, 0.0, 0.0, 0.08, 0.5, 0.3)
    assert s.sigma_y > 0 and s.sigma_x > 0 and s.time_of_flight > 0.5


def test_spreading_and_energetic_sum():
    assert spl_at_distance(72, 15.0) == pytest.approx(72 - 6.0206, abs=1e-3)   # double distance: -6 dB
    assert combine_db(50, 50) == pytest.approx(53.0103, abs=1e-3)


def test_doppler_speed_recovers_speed():
    c, f0, v = 343.0, 600.0, 15.0
    fa = doppler_observed(f0, np.array([-1e5, 0]), np.array([v, 0]), np.zeros(2), c)
    fr = doppler_observed(f0, np.array([1e5, 0]), np.array([v, 0]), np.zeros(2), c)
    assert doppler_speed(fa, fr, c) == pytest.approx(v, rel=1e-6)


def test_retarded_time_solves_light_cone():
    v, c = 20.0, 343.0
    pos = lambda t: np.array([-50.0 + v * t, 10.0])
    t = 2.0
    te = retarded_time(pos, t, np.zeros(2), c, iterations=8)
    assert t - te == pytest.approx(np.linalg.norm(pos(te)) / c, abs=1e-9)


def test_passby_curve_identifies_only_v_over_d():
    t = np.linspace(-3, 3, 50)
    a = passby_level(t, 80, 10.0, 5.0, 0.0)
    b = passby_level(t, 80, 20.0, 10.0, 0.0)
    assert np.allclose(a, b)

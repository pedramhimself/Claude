import numpy as np
import pytest

from simcore.config import Config
from simcore.control.gantry import Gantry
from simcore.control.profiles import OnlineProfile, move_time_from_state, trapezoid_time


@pytest.mark.parametrize("d", [0.05, 0.3, 1.5, 3.0])
def test_online_profile_matches_trapezoid_time(d):
    p = OnlineProfile(0.0, 1.0, 3.0)
    p.target = d
    t, dt = 0.0, 0.001
    while p.remaining_time > 0 and t < 10:
        p.step(dt)
        assert abs(p.vel) <= 1.0 + 1e-9
        t += dt
    assert t == pytest.approx(trapezoid_time(d, 1.0, 3.0), abs=0.01)


def test_move_time_from_moving_state():
    # already cruising towards the target: faster than starting from rest
    assert move_time_from_state(0.0, 1.0, 2.0, 1.0, 3.0) < trapezoid_time(2.0, 1.0, 3.0)
    # moving away: slower
    assert move_time_from_state(0.0, -1.0, 2.0, 1.0, 3.0) > trapezoid_time(2.0, 1.0, 3.0)


def test_gantry_tracks_and_respects_limits():
    g = Gantry(Config().gantry, np.random.default_rng(0))
    tx, ty = g.set_target(10.0, -5.0)          # outside travel -> clamped
    assert (tx, ty) == (4.0, 0.0)
    for _ in range(4000):
        g.step(0.001)
    assert g.pos == pytest.approx([4.0, 0.0], abs=0.003)

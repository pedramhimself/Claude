import numpy as np
import pytest

from simcore.config import Config
from simcore.detection.trigger import AcousticTrigger
from simcore.estimation.ekf import (TrackEKF, gate_model, numeric_jacobian, process_noise, radial_speed_model,
                                    transition)


def test_transition_and_process_noise_shapes():
    F = transition(0.1)
    assert F @ np.array([0, 0, 1, 2]) == pytest.approx([0.1, 0.2, 1, 2])
    Q = process_noise(0.1, 1.0, 0.5)
    assert np.allclose(Q, Q.T) and np.all(np.linalg.eigvalsh(Q) >= -1e-15)


def test_radial_speed_jacobian_matches_numeric():
    m = radial_speed_model(-10.0, 0.01, [0.0, 0.0])
    s = np.array([-20.0, 9.0, 13.0, 0.2])
    assert m.H(s) == pytest.approx(numeric_jacobian(m.h, s), abs=1e-6)


def _gate_sequence(v=14.0, y=10.0):
    ekf = TrackEKF(0.5, 0.05)
    ekf.initialize(0.0, np.array([-25.0, y, 12.0, 0.0]), np.diag([0.01, 0.04, 9.0, 0.01]))
    return ekf, v, y


def test_two_gates_estimate_speed():
    ekf, v, y = _gate_sequence()
    t2 = 3.0 / v
    ekf.process(t2, gate_model(-22.0, y, 1e-6, 0.01))
    assert ekf.s[2] == pytest.approx(v, abs=0.05)


def test_out_of_sequence_measurement_equals_in_order():
    rng = np.random.default_rng(3)
    truth_v, y = 14.0, 10.0
    meas = []
    for t in np.arange(0.05, 1.0, 0.05):
        x = -25.0 + truth_v * t
        rel = np.array([x, y])
        vr = rel @ np.array([truth_v, 0]) / np.linalg.norm(rel) + 0.1 * rng.standard_normal()
        meas.append((t, vr))

    def run(order):
        ekf = TrackEKF(0.5, 0.05)
        ekf.initialize(0.0, np.array([-25.0, y, 12.0, 0.0]), np.diag([0.01, 0.04, 9.0, 0.01]))
        for i in order:
            t, vr = meas[i]
            ekf.process(t, radial_speed_model(vr, 0.01, [0, 0]))
        return ekf.s

    in_order = run(range(len(meas)))
    shuffled = list(range(len(meas)))
    # deliver in pairs swapped (each measurement late by one slot)
    for i in range(0, len(shuffled) - 1, 2):
        shuffled[i], shuffled[i + 1] = shuffled[i + 1], shuffled[i]
    assert run(shuffled) == pytest.approx(in_order, abs=1e-9)


def test_outlier_is_gated():
    ekf, v, y = _gate_sequence()
    res = ekf.process(0.1, radial_speed_model(+40.0, 0.01, [0, 0]))   # receding at 40 m/s: impossible
    assert not res.accepted


def test_trigger_hysteresis_and_floor():
    trig = AcousticTrigger(Config().trigger, 50.0)
    events = []
    levels = [50] * 50 + [58] * 10 + [56] * 5 + [50] * 100
    for i, L in enumerate(levels):
        ev = trig.update(i * 0.02, L)
        if ev:
            events.append(ev.kind)
    assert events == ["on", "off"]

import json

import pytest

from simcore.config import ConfigError, load_config
from simcore.engine.runner import run_simulation
from simcore.experiments.monte_carlo import run_monte_carlo


def test_nominal_run_lands_in_zone_with_clearance():
    res = run_simulation(load_config())
    s = res.summary
    assert s["status"] == "LANDED"
    assert s["in_zone"]
    assert abs(s["timing_error"]) < 4 * s["predicted_sigma_t"] + 0.01
    assert s["lateral_clearance"] >= load_config().safety.lane_clearance_min
    json.dumps(res.to_json_dict())   # serialisable, no NaN


def test_safety_gate_blocks_release_when_lane_too_close():
    res = run_simulation(load_config("config/scenarios/lane_too_close.yaml"))
    assert res.summary["status"] == "ABORTED"
    assert "lane_clearance" in res.summary["abort_failed_checks"]
    assert res.flight is None


def test_quiet_vehicle_stands_down():
    res = run_simulation(load_config("config/scenarios/quiet_ev.yaml"))
    assert res.summary["status"] == "ABORTED" and res.flight is None


def test_seed_reproducibility():
    a = run_simulation(load_config(overrides={"sim.seed": 7})).summary
    b = run_simulation(load_config(overrides={"sim.seed": 7})).summary
    assert a == b


def test_unknown_override_rejected():
    with pytest.raises(ConfigError):
        load_config(overrides={"vehicle.colour": "red"})


def test_monte_carlo_error_model_is_consistent():
    mc = run_monte_carlo(load_config(), n=40, workers=1)
    st = mc["stats"]
    assert st["landed"] == 40 and st["in_zone_rate"] == 1.0
    # empirical timing spread within a factor ~1.6 of the planner's predicted sigma
    assert 0.6 < st["sigma_ratio"] < 1.6

"""Monte Carlo and sensitivity analysis.

Each trial re-draws every random quantity (vehicle speed wander, sensor
noise, dropouts, latency jitter, launch dispersion, wind gust) through a new
seed and, optionally, randomises scenario parameters.  The empirical spread
of the timing error is compared against the planner's own prediction
``sigma_t`` - if the model is right the two agree.
"""

from __future__ import annotations

import math
import os
from concurrent.futures import ProcessPoolExecutor
from typing import Any

import numpy as np

from ..config import Config
from ..engine.runner import run_simulation

DEFAULT_RANDOMIZE = {"vehicle.speed_kmh": 8.0, "vehicle.lane_y": 0.5}  # +- uniform half-widths

ROW_KEYS = ["seed", "status", "abort_reason", "true_speed_kmh", "lane_y", "timing_error", "along_track_error",
            "landing_x", "landing_y", "miss_x", "miss_y", "in_zone", "lateral_clearance", "predicted_sigma_t",
            "speed_error_at_cmd", "x_error_at_cmd", "y_error_at_cmd", "x_at_trigger"]


def _trial(args: tuple[dict, dict]) -> dict:
    cfg_dict, overrides = args
    from ..config import from_dict
    cfg = from_dict(Config, cfg_dict).with_overrides(overrides)
    s = run_simulation(cfg).summary
    return {k: s.get(k) for k in ROW_KEYS}


def _draw_overrides(cfg: Config, rng: np.random.Generator, randomize: dict[str, float]) -> dict[str, Any]:
    out = {}
    flat = cfg.to_dict()
    for path, half in randomize.items():
        node = flat
        for k in path.split("."):
            node = node[k]
        out[path] = float(node) + float(rng.uniform(-half, half))
    return out


def run_monte_carlo(cfg: Config, n: int = 100, randomize: dict[str, float] | None = None,
                    base_seed: int = 1000, workers: int | None = None) -> dict[str, Any]:
    randomize = DEFAULT_RANDOMIZE if randomize is None else randomize
    rng = np.random.default_rng(base_seed)
    jobs = []
    for i in range(n):
        ov = _draw_overrides(cfg, rng, randomize)
        ov["sim.seed"] = base_seed + i
        jobs.append((cfg.to_dict(), ov))
    workers = workers if workers is not None else min(8, os.cpu_count() or 1)
    if workers > 1 and n > 4:
        with ProcessPoolExecutor(max_workers=workers) as ex:
            rows = list(ex.map(_trial, jobs, chunksize=max(1, n // (workers * 4))))
    else:
        rows = [_trial(j) for j in jobs]
    return {"n": n, "randomize": randomize, "rows": rows, "stats": summarize(rows)}


def _std(values):
    return float(np.std(values, ddof=1)) if len(values) > 1 else math.nan


def summarize(rows: list[dict]) -> dict[str, Any]:
    landed = [r for r in rows if r["status"] == "LANDED"]
    statuses: dict[str, int] = {}
    reasons: dict[str, int] = {}
    for r in rows:
        statuses[r["status"]] = statuses.get(r["status"], 0) + 1
        if r.get("abort_reason"):
            reasons[r["abort_reason"]] = reasons.get(r["abort_reason"], 0) + 1
    out: dict[str, Any] = {"n": len(rows), "landed": len(landed), "statuses": statuses, "abort_reasons": reasons}
    if landed:
        te = np.array([r["timing_error"] for r in landed])
        at = np.array([r["along_track_error"] for r in landed])
        mx = np.array([r["miss_x"] for r in landed])
        my = np.array([r["miss_y"] for r in landed])
        ps = np.array([r["predicted_sigma_t"] for r in landed])
        clear = np.array([r["lateral_clearance"] for r in landed])
        out.update({
            "timing_error_mean": float(te.mean()), "timing_error_std": _std(te),
            "timing_error_p95_abs": float(np.percentile(np.abs(te), 95)),
            "along_track_mean": float(at.mean()), "along_track_std": _std(at),
            "miss_x_std": _std(mx), "miss_y_std": _std(my),
            "predicted_sigma_t_mean": float(ps.mean()),
            "sigma_ratio": float(_std(te) / ps.mean()) if ps.mean() > 0 else math.nan,
            "in_zone_rate": float(np.mean([r["in_zone"] for r in landed])),
            "min_lateral_clearance": float(clear.min()),
        })
    return out


def run_sweep(cfg: Config, param: str, values: list[float], n: int = 40, workers: int | None = None) -> dict:
    """Vary one parameter, run a Monte Carlo batch per value."""
    results = []
    for v in values:
        mc = run_monte_carlo(cfg.with_overrides({param: v}), n=n, randomize={}, workers=workers)
        results.append({"value": v, **mc["stats"]})
    return {"param": param, "values": values, "results": results}

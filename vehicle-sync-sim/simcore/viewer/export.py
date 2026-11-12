"""Write a self-contained HTML viewer with every bundled scenario pre-computed."""

from __future__ import annotations

import datetime as _dt
from pathlib import Path

from ..config import load_config
from ..engine.runner import run_simulation
from ..experiments.monte_carlo import DEFAULT_RANDOMIZE, ROW_KEYS, run_monte_carlo
from .page import build_page, list_scenarios


def export_static(out: str | Path, base_config: str | None = None, mc_trials: int = 150,
                  standalone: bool = True) -> Path:
    presets = []
    for sc in list_scenarios():
        run = run_simulation(load_config(sc["path"])).to_json_dict()
        presets.append({"id": sc["id"], "title": sc["title"], "description": sc["description"], "run": run})
    mc = None
    if mc_trials > 0:
        mc = run_monte_carlo(load_config(base_config), n=mc_trials, randomize=DEFAULT_RANDOMIZE)
        mc["rows"] = [{k: r.get(k) for k in ROW_KEYS} for r in mc["rows"]]
    data = {"generated": _dt.date.today().isoformat(), "presets": presets, "montecarlo": mc}
    path = Path(out)
    path.write_text(build_page(data, standalone), encoding="utf-8")
    return path

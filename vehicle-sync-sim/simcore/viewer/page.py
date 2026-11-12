"""Assemble the viewer page from its template, stylesheet and script."""

from __future__ import annotations

import json
from pathlib import Path

STATIC = Path(__file__).parent / "static"
SCENARIO_DIR = Path(__file__).resolve().parents[2] / "config" / "scenarios"

# Display order and titles for the bundled scenarios.
SCENARIO_ORDER = ["nominal", "fast_vehicle", "slow_vehicle", "degraded_sensors", "no_radar", "windy",
                  "accelerating", "lane_too_close", "quiet_ev"]
SCENARIO_TITLES = {
    "nominal": "Nominal, 50 km/h",
    "fast_vehicle": "Fast vehicle, 90 km/h",
    "slow_vehicle": "Slow vehicle, 30 km/h",
    "degraded_sensors": "Degraded sensors",
    "no_radar": "Radar failed",
    "windy": "Wind and gusts",
    "accelerating": "Accelerating vehicle",
    "lane_too_close": "Lane too close (safety stop)",
    "quiet_ev": "Quiet EV (no detection)",
}


def list_scenarios() -> list[dict]:
    found = {p.stem: p for p in SCENARIO_DIR.glob("*.yaml")}
    order = [s for s in SCENARIO_ORDER if s in found] + sorted(s for s in found if s not in SCENARIO_ORDER)
    out = []
    for sid in order:
        path = found[sid]
        lines = [ln[1:].strip() for ln in path.read_text(encoding="utf-8").splitlines() if ln.startswith("#")]
        out.append({"id": sid, "title": SCENARIO_TITLES.get(sid, sid.replace("_", " ").title()),
                    "description": " ".join(lines), "path": str(path)})
    return out


HEAD = ('<!doctype html>\n<html lang="en">\n<meta charset="utf-8">\n'
        '<meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">\n')


def build_page(static_data: dict | None = None, standalone: bool = True) -> str:
    """``standalone`` adds the document prologue; leave it off when a host wraps the page."""
    html = (STATIC / "template.html").read_text(encoding="utf-8")
    css = (STATIC / "style.css").read_text(encoding="utf-8")
    js = (STATIC / "app.js").read_text(encoding="utf-8")
    html = html.replace("/*__STYLE__*/", css).replace("/*__APP__*/", js)
    if static_data is not None:
        payload = json.dumps(static_data, separators=(",", ":")).replace("</", "<\\/")
        html = html.replace("<!--__DATA__-->", f"<script>window.SIM_STATIC = {payload};</script>")
    else:
        html = html.replace("<!--__DATA__-->", "")
    return (HEAD + html) if standalone else html

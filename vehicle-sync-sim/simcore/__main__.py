"""Command line interface.

    python -m simcore run          [--config FILE] [--set key=value ...] [--out run.json]
    python -m simcore montecarlo   [-n 200] [--randomize vehicle.speed_kmh=10 ...] [--out mc.json]
    python -m simcore sweep        --param radar.timing.latency --values 0,0.05,0.1,0.2 [-n 40]
    python -m simcore serve        [--port 8765]           # interactive viewer with live re-runs
    python -m simcore export-viewer [--out viewer.html]    # static, self-contained viewer
    python -m simcore dump-config  [--out config.yaml]
"""

from __future__ import annotations

import argparse
import json
import sys

import yaml

from .config import load_config, parse_override


def _cfg(args):
    overrides = dict(parse_override(s) for s in (args.set or []))
    return load_config(args.config, overrides)


def _print_summary(summary: dict) -> None:
    width = max(len(k) for k in summary)
    for k, v in summary.items():
        if isinstance(v, float):
            v = f"{v:.4f}"
        print(f"  {k:<{width}}  {v}")


def cmd_run(args) -> int:
    from .engine.runner import run_simulation
    res = run_simulation(_cfg(args))
    _print_summary(res.summary)
    if args.events:
        for e in res.events:
            print(f"  {e['t']:8.3f}  {e['kind']}")
    if args.out:
        with open(args.out, "w", encoding="utf-8") as fh:
            json.dump(res.to_json_dict(), fh)
        print(f"wrote {args.out}")
    return 0


def cmd_montecarlo(args) -> int:
    from .experiments.monte_carlo import DEFAULT_RANDOMIZE, run_monte_carlo
    randomize = DEFAULT_RANDOMIZE if args.randomize is None else {
        k: float(v) for k, v in (parse_override(s) for s in args.randomize)}
    mc = run_monte_carlo(_cfg(args), n=args.n, randomize=randomize, workers=args.workers)
    _print_summary(mc["stats"])
    if args.out:
        with open(args.out, "w", encoding="utf-8") as fh:
            json.dump(mc, fh)
        print(f"wrote {args.out}")
    return 0


def cmd_sweep(args) -> int:
    from .experiments.monte_carlo import run_sweep
    parsed = yaml.safe_load(args.values)       # "[[0,0],[0,0.01]]" -> list of lists
    values = parsed if isinstance(parsed, list) else [yaml.safe_load(v) for v in args.values.split(",")]
    res = run_sweep(_cfg(args), args.param, values, n=args.n, workers=args.workers)
    print(f"{args.param:>28} | landed | timing mean [ms] | timing std [ms] | predicted [ms] | along-track std [m]")
    for r in res["results"]:
        tm = r.get("timing_error_mean", float("nan")) * 1000
        ts = r.get("timing_error_std", float("nan")) * 1000
        ps = r.get("predicted_sigma_t_mean", float("nan")) * 1000
        at = r.get("along_track_std", float("nan"))
        print(f"{str(r['value']):>28} | {r['landed']:>3}/{r['n']:<3}| {tm:16.1f} | {ts:15.1f} | {ps:14.1f} | {at:8.3f}")
    if args.out:
        with open(args.out, "w", encoding="utf-8") as fh:
            json.dump(res, fh)
    return 0


def cmd_serve(args) -> int:
    from .viewer.server import serve
    serve(args.port, args.config)
    return 0


def cmd_export(args) -> int:
    from .viewer.export import export_static
    path = export_static(args.out, args.config, mc_trials=args.n, standalone=not args.fragment)
    print(f"wrote {path}")
    return 0


def cmd_dump(args) -> int:
    text = yaml.safe_dump(_cfg(args).to_dict(), sort_keys=False)
    if args.out:
        with open(args.out, "w", encoding="utf-8") as fh:
            fh.write(text)
    else:
        sys.stdout.write(text)
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="simcore", description="Vehicle-synchronised payload deployment simulator")
    sub = ap.add_subparsers(dest="cmd", required=True)

    def common(p):
        p.add_argument("--config", help="YAML scenario file (merged over the defaults)")
        p.add_argument("--set", action="append", metavar="KEY=VALUE", help="override, e.g. vehicle.speed_kmh=70")

    p = sub.add_parser("run", help="run one scenario")
    common(p)
    p.add_argument("--out")
    p.add_argument("--events", action="store_true", help="print the event log")
    p.set_defaults(fn=cmd_run)

    p = sub.add_parser("montecarlo", help="many randomised trials")
    common(p)
    p.add_argument("-n", type=int, default=200)
    p.add_argument("--randomize", action="append", metavar="KEY=HALFWIDTH")
    p.add_argument("--workers", type=int)
    p.add_argument("--out")
    p.set_defaults(fn=cmd_montecarlo)

    p = sub.add_parser("sweep", help="sensitivity of one parameter")
    common(p)
    p.add_argument("--param", required=True)
    p.add_argument("--values", required=True, help="comma separated, or a YAML list such as [[0,0],[0,0.01]]")
    p.add_argument("-n", type=int, default=40)
    p.add_argument("--workers", type=int)
    p.add_argument("--out")
    p.set_defaults(fn=cmd_sweep)

    p = sub.add_parser("serve", help="interactive viewer (http://localhost:PORT)")
    p.add_argument("--config")
    p.add_argument("--port", type=int, default=8765)
    p.set_defaults(fn=cmd_serve)

    p = sub.add_parser("export-viewer", help="write a self-contained HTML viewer with preset runs")
    p.add_argument("--config")
    p.add_argument("--out", default="viewer.html")
    p.add_argument("-n", type=int, default=150, help="Monte Carlo trials to embed")
    p.add_argument("--fragment", action="store_true", help="omit <!doctype>/<head> (for hosts that wrap the page)")
    p.set_defaults(fn=cmd_export)

    p = sub.add_parser("dump-config", help="print the effective configuration as YAML")
    common(p)
    p.add_argument("--out")
    p.set_defaults(fn=cmd_dump)

    args = ap.parse_args(argv)
    return args.fn(args)


if __name__ == "__main__":
    raise SystemExit(main())

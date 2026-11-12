"""Tiny local HTTP server for the interactive viewer (standard library only).

    GET  /                 viewer page
    GET  /api/presets      bundled scenarios
    POST /api/run          {"preset": id, "overrides": {"vehicle.speed_kmh": 70, ...}}
    POST /api/montecarlo   {"preset": id, "overrides": {...}, "n": 100}

Binds to 127.0.0.1 only.
"""

from __future__ import annotations

import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from ..config import ConfigError, load_config
from ..engine.runner import run_simulation
from ..experiments.monte_carlo import DEFAULT_RANDOMIZE, run_monte_carlo
from .page import build_page, list_scenarios


def _config_for(body: dict, base_config: str | None):
    presets = {p["id"]: p["path"] for p in list_scenarios()}
    preset = body.get("preset")
    path = presets.get(preset, base_config)
    overrides = body.get("overrides") or {}
    if not isinstance(overrides, dict):
        raise ConfigError("overrides must be an object")
    return load_config(path, overrides)


def make_handler(base_config: str | None):
    class Handler(BaseHTTPRequestHandler):
        def _send(self, code: int, body: bytes, ctype: str) -> None:
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _json(self, obj, code: int = 200) -> None:
            self._send(code, json.dumps(obj).encode("utf-8"), "application/json")

        def log_message(self, fmt, *args):  # quieter console
            if "/api/" in (args[0] if args else ""):
                super().log_message(fmt, *args)

        def do_GET(self):
            if self.path in ("/", "/index.html"):
                self._send(200, build_page(None).encode("utf-8"), "text/html; charset=utf-8")
            elif self.path == "/api/presets":
                self._json([{k: v for k, v in p.items() if k != "path"} for p in list_scenarios()])
            else:
                self._send(404, b"not found", "text/plain")

        def do_POST(self):
            try:
                length = int(self.headers.get("Content-Length", "0"))
                body = json.loads(self.rfile.read(length) or b"{}")
                cfg = _config_for(body, base_config)
                if self.path == "/api/run":
                    self._json(run_simulation(cfg).to_json_dict())
                elif self.path == "/api/montecarlo":
                    n = max(1, min(int(body.get("n", 100)), 2000))
                    self._json(run_monte_carlo(cfg, n=n, randomize=DEFAULT_RANDOMIZE))
                else:
                    self._send(404, b"not found", "text/plain")
            except (ConfigError, ValueError, TypeError, KeyError) as exc:
                self._send(400, str(exc).encode("utf-8"), "text/plain")

    return Handler


def serve(port: int = 8765, base_config: str | None = None) -> None:
    server = ThreadingHTTPServer(("127.0.0.1", port), make_handler(base_config))
    print(f"Viewer running at http://localhost:{port}  (Ctrl+C to stop)")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()

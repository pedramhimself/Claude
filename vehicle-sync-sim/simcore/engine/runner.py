"""Fixed-step simulation engine.

One loop, one clock (``dt``, default 1 ms):

    1. sensors sample the ground truth and emit timestamped measurements
       (each delayed by its own latency + processing time)
    2. measurements whose availability time has passed are delivered:
       sound level -> trigger, everything else -> tracker (after the trigger)
    3. the planner re-plans at ``planner_rate`` and re-targets the gantry
    4. the release controller checks every step whether the commanded time
       has come; if so it asks the safety gate and fires or aborts
    5. gantry and payload physics advance; a log frame is written every ``log_dt``

System states:  IDLE -> ARMED (trigger) -> TRACKING (track initialised)
                -> COMMITTED (aim frozen) -> RELEASED -> LANDED,   or ABORTED.
"""

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass, field
from typing import Any

import numpy as np

from ..config import Config
from ..control.gantry import Gantry
from ..control.release import ReleaseActuator
from ..detection.trigger import AcousticTrigger
from ..estimation.tracker import Tracker
from ..planning.ballistics import drag_constant, launch_solution, simulate_flight
from ..planning.safety import SafetyGate
from ..planning.scheduler import ReleasePlanner
from ..sensors.lidar import LidarGate
from ..sensors.microphone import MicrophoneArray
from ..sensors.radar import DopplerRadar
from ..world.world import World

STATES = ["IDLE", "ARMED", "TRACKING", "COMMITTED", "RELEASED", "LANDED", "ABORTED"]
PLANNER_RATE_HZ = 50.0


@dataclass
class RunResult:
    summary: dict[str, Any]
    frames: dict[str, list]
    measurements: dict[str, list]
    events: list[dict]
    flight: dict[str, list] | None
    scene: dict[str, Any]
    config: dict[str, Any]

    def to_json_dict(self, digits: int = 4) -> dict[str, Any]:
        return _clean({"summary": self.summary, "frames": self.frames, "measurements": self.measurements,
                       "events": self.events, "flight": self.flight, "scene": self.scene,
                       "config": self.config}, digits)


def _clean(obj, digits):
    if isinstance(obj, dict):
        return {k: _clean(v, digits) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_clean(v, digits) for v in obj]
    if isinstance(obj, np.ndarray):
        return _clean(obj.tolist(), digits)
    if isinstance(obj, (bool, np.bool_)):
        return bool(obj)
    if isinstance(obj, (int, np.integer)):
        return int(obj)
    if isinstance(obj, (float, np.floating)):
        f = float(obj)
        return None if math.isnan(f) or math.isinf(f) else round(f, digits)
    return obj


@dataclass
class _Flight:
    t0: float
    t: np.ndarray
    pos: np.ndarray
    t_land: float
    landing: np.ndarray

    def position(self, t: float) -> np.ndarray | None:
        if t < self.t0 or t > self.t_land:
            return None
        tau = t - self.t0
        return np.array([np.interp(tau, self.t, self.pos[:, i]) for i in range(3)])


class Simulation:
    def __init__(self, cfg: Config):
        self.cfg = cfg
        # Independent random streams so that e.g. changing radar noise does not
        # change the vehicle trajectory drawn for the same seed.
        if cfg.sim.t_max <= 0:
            cfg = cfg.copy()
            v = max(cfg.vehicle.speed_kmh / 3.6, 1.0)
            cfg.sim.t_max = (cfg.zone.x_range[1] + 30.0 - cfg.vehicle.x0) / v + 3.0
            self.cfg = cfg
        streams = [np.random.default_rng(s) for s in np.random.SeedSequence(cfg.sim.seed).spawn(10)]
        self.world = World(cfg, streams[0])
        self.sensors = [MicrophoneArray(cfg.microphone, cfg.environment, streams[1]),
                        DopplerRadar(cfg.radar, streams[2])]
        self.sensors += [LidarGate(cfg.lidar, i, np.random.default_rng(streams[3].integers(1 << 32)))
                         for i in range(len(cfg.lidar.gates_x))]
        self.compute_rng = streams[4]
        self.trigger = AcousticTrigger(cfg.trigger, cfg.environment.background_db)
        self.tracker = Tracker(cfg)
        env, p = cfg.environment, cfg.payload
        self.k_drag = drag_constant(p, env.air_density)
        wf = self.world.wind_forecast
        self.launch = launch_solution(p.launch_speed, p.elevation_deg, p.height, self.k_drag, env.gravity,
                                      float(wf[0]), float(wf[1]), p.launch_speed_sigma, p.elevation_sigma_deg,
                                      env.gust_sigma)
        self.planner = ReleasePlanner(cfg, self.launch)
        self.safety = SafetyGate(cfg, self.launch)
        self.gantry = Gantry(cfg.gantry, streams[5])
        self.release = ReleaseActuator(cfg.release, cfg.payload, streams[6])

        self.state = "IDLE"
        self.events: list[dict] = []
        self.plan = None
        self.safety_status = None
        self.flight: _Flight | None = None
        self.launch_event = None
        self.release_check = None
        self.t_end: float | None = None
        self.t_trigger: float | None = None
        self.plan_at_command = None
        self.track_at_command = None
        self._queue: list = []
        self._seq = 0
        self.last_spl = (math.nan, math.nan)
        self.meas: dict[str, list] = {"spl": [], "radar": [], "bearing": [], "tone": [], "gate": []}

    # --------------------------------------------------------------- helpers
    def _event(self, t: float, kind: str, **data) -> None:
        self.events.append({"t": t, "kind": kind, **data})

    def _set_state(self, t: float, state: str, **data) -> None:
        self.state = state
        self._event(t, f"state:{state}", **data)

    def _abort(self, t: float, reason: str, **data) -> None:
        self._set_state(t, "ABORTED", reason=reason, **data)
        hx, hy = self.cfg.gantry.home
        self.gantry.set_target(hx, hy)
        self.t_end = t + self.cfg.sim.post_landing_time

    def _record(self, m) -> None:
        if m.kind == "spl":
            self.meas["spl"].append([m.t_true, m.z[0], m.meta["true_db"]])
        elif m.kind == "radial_speed":
            self.meas["radar"].append([m.t_true, m.t_avail, m.z[0], m.meta["truth"]])
        elif m.kind == "bearing":
            self.meas["bearing"].append([m.t_true, m.z[0]])
        elif m.kind == "tone":
            self.meas["tone"].append([m.t_true, m.z[0]])
        elif m.kind in ("gate_enter", "gate_exit"):
            self.meas["gate"].append([m.t_true, m.sensor, m.kind, m.z[0], m.meta["gate_x"]])

    # ------------------------------------------------------------ main loop
    def run(self) -> RunResult:
        cfg = self.cfg
        dt = cfg.sim.dt
        n_steps = int(round(cfg.sim.t_max / dt))
        log_every = max(1, int(round(cfg.sim.log_dt / dt)))
        plan_every = max(1, int(round(1.0 / PLANNER_RATE_HZ / dt)))
        frames = _FrameLog()
        e = cfg.estimator
        zone_far = cfg.zone.x_range[1]

        for step in range(n_steps + 1):
            t = step * dt
            # 1. sensing
            for sensor in self.sensors:
                for m in sensor.poll(t, self.world):
                    m.t_avail += max(0.0, e.compute_latency + e.compute_jitter * self.compute_rng.standard_normal())
                    self._seq += 1
                    heapq.heappush(self._queue, (m.t_avail, self._seq, m))
                    self._record(m)
            # 2. delivery
            while self._queue and self._queue[0][0] <= t:
                _, _, m = heapq.heappop(self._queue)
                self._deliver(t, m)
            # 3. planning
            active = self.state in ("TRACKING", "COMMITTED") and self.plan_at_command is None
            if active and step % plan_every == 0:
                self._replan(t)
            # 4. release timing (checked every step, like a hardware timer)
            if (self.state in ("TRACKING", "COMMITTED") and self.plan is not None and self.plan_at_command is None
                    and t >= self.plan.t_cmd):
                self._try_release(t)
            if self.release.due(t):
                self._fire(t)
            # vehicle has left the zone without a release -> abort
            if self.state in ("ARMED", "TRACKING", "COMMITTED"):
                if self.world.vehicle.state(t).x > zone_far + 25.0:
                    self._abort(t, "no_track" if self.state == "ARMED" else "window_missed")
            # 5. physics
            self.gantry.step(dt)
            if self.flight is not None and self.state == "RELEASED" and t >= self.flight.t_land:
                self._set_state(self.flight.t_land, "LANDED", landing=self.flight.landing.tolist())
                self.t_end = self.flight.t_land + cfg.sim.post_landing_time
            if step % log_every == 0:
                frames.add(self, t)
            if self.t_end is not None and t >= self.t_end:
                break
        if self.state == "IDLE":
            self._event(t, "no_detection")
        elif self.state in ("ARMED", "TRACKING", "COMMITTED"):
            self._abort(t, "timeout")
        return self._result(frames)

    # ------------------------------------------------------------- stages
    def _deliver(self, t: float, m) -> None:
        if m.kind == "spl":
            self.last_spl = (float(m.z[0]), m.t_true)
            ev = self.trigger.update(m.t_stamp, float(m.z[0]))
            if ev is not None:
                self._event(t, f"trigger_{ev.kind}", level_db=ev.level_db, floor_db=ev.floor_db,
                            vehicle_x=self.world.vehicle.state(t).x)
                if ev.kind == "on" and self.state == "IDLE":
                    self.t_trigger = t
                    self._set_state(t, "ARMED")
                    rx, ry = self.planner.ready_pose()
                    self.gantry.set_target(rx, ry)
            return
        if self.state in ("IDLE", "LANDED", "ABORTED"):
            return  # system asleep or finished: data ignored
        was_active = self.tracker.active
        self.tracker.ingest(m)
        if not was_active and self.tracker.active and self.state == "ARMED":
            self._set_state(t, "TRACKING", sensor=m.sensor)
            self._replan(t)

    def _replan(self, t: float) -> None:
        snap = self.tracker.snapshot(t)
        if snap is None:
            return
        plan = self.planner.plan(t, snap, self.gantry.pos, self.gantry.vel)
        if plan is None:
            if self.plan is None or not self.plan.committed:
                self._abort(t, "no_valid_plan")
            return
        if self.plan is None:
            self._event(t, "first_plan", **plan.to_dict())
        self.plan = plan
        self.gantry.set_target(plan.release_x, plan.release_y)
        if plan.committed and self.state == "TRACKING":
            self._set_state(t, "COMMITTED", x_aim=plan.x_aim, t_cmd=plan.t_cmd)
        self.safety_status = self.safety.evaluate(t, snap, plan, self.gantry.pos, self.gantry.vel)

    def _try_release(self, t: float) -> None:
        snap = self.tracker.snapshot(t)
        # final timing refresh with everything known now
        plan = self.planner.plan(t, snap, self.gantry.pos, self.gantry.vel) if snap else None
        if plan is not None and plan.t_cmd > t + 1e-9:
            self.plan = plan  # newest estimate moved the command time later: keep waiting
            return
        plan = plan or self.plan
        status = self.safety.evaluate(t, snap, plan, self.gantry.pos, self.gantry.vel)
        self.safety_status = status
        self.release_check = status.checks
        if not status.ok:
            self._abort(t, "safety_gate", failed=status.failed, checks=status.checks)
            return
        self.plan_at_command = plan
        self.track_at_command = snap
        self.release.command(t)
        self._event(t, "release_command", **plan.to_dict())

    def _fire(self, t: float) -> None:
        ev = self.release.fire(t, self.gantry.pos, self.gantry.vel)
        self.launch_event = ev
        env = self.cfg.environment
        fl = simulate_flight(ev.position, ev.velocity, self.k_drag, env.gravity, self.world.wind_true)
        self.flight = _Flight(t, fl.t, fl.pos, t + fl.time_of_flight, fl.landing)
        self._set_state(t, "RELEASED", carriage=self.gantry.pos.tolist(), carriage_vel=self.gantry.vel.tolist(),
                        launch_speed=ev.launch_speed, elevation_deg=ev.elevation_deg)

    # ------------------------------------------------------------- results
    def _scene(self) -> dict[str, Any]:
        c = self.cfg
        return {
            "lane_y": c.vehicle.lane_y, "vehicle_width": c.vehicle.width, "vehicle_length": c.vehicle.length,
            "zone": {"x": c.zone.x_range, "y": c.zone.y_range},
            "gantry": {"x": c.gantry.x_range, "y": c.gantry.y_range, "home": c.gantry.home},
            "mic": c.microphone.position, "radar": {"pos": c.radar.position, "boresight_deg": c.radar.boresight_deg,
                                                    "fov_deg": c.radar.fov_deg, "max_range": c.radar.max_range},
            "lidar": {"gates_x": c.lidar.gates_x, "mount_y": c.lidar.mount_y, "max_range": c.lidar.max_range},
            "launch": {"dx": self.launch.dx, "dy": self.launch.dy, "time_of_flight": self.launch.time_of_flight,
                       "sigma_x": self.launch.sigma_x, "sigma_y": self.launch.sigma_y,
                       "impact_speed": self.launch.impact_speed},
            "y_aim": self.planner.y_aim, "horizon_s": c.prediction.horizon_s,
            "wind_forecast": self.world.wind_forecast.tolist(), "wind_true": self.world.wind_true.tolist(),
        }

    def _result(self, frames: "_FrameLog") -> RunResult:
        flight = None
        if self.flight is not None:
            f = self.flight
            flight = {"t": (f.t + f.t0).tolist(), "x": f.pos[:, 0].tolist(), "y": f.pos[:, 1].tolist(),
                      "z": f.pos[:, 2].tolist()}
        return RunResult(self._summary(), frames.data, self.meas, self.events, flight, self._scene(),
                         self.cfg.to_dict())

    def _summary(self) -> dict[str, Any]:
        cfg, veh = self.cfg, self.world.vehicle
        s: dict[str, Any] = {"status": self.state, "seed": cfg.sim.seed,
                             "true_speed_kmh": cfg.vehicle.speed_kmh, "lane_y": cfg.vehicle.lane_y,
                             "t_trigger": self.t_trigger,
                             "x_at_trigger": veh.state(self.t_trigger).x if self.t_trigger is not None else None,
                             "ekf_replays": self.tracker.ekf.replays, "ekf_rejected": self.tracker.ekf.rejected}
        if self.release_check is not None:
            s["release_checks"] = self.release_check
        aborts = [e for e in self.events if e["kind"] == "state:ABORTED"]
        if aborts:
            s["abort_reason"] = aborts[0].get("reason")
            s["abort_failed_checks"] = aborts[0].get("failed")
        if self.plan_at_command is not None:
            p, tr = self.plan_at_command, self.track_at_command
            t_cmd = p.t_plan
            truth = veh.state(t_cmd)
            s.update({
                "x_aim": p.x_aim, "y_aim": p.y_aim, "t_cmd": t_cmd, "predicted_sigma_t": p.sigma_t,
                "predicted_sigma_along": p.sigma_along,
                "speed_error_at_cmd": tr.speed - truth.vx,
                "x_error_at_cmd": tr.x_centre - truth.x,
                "y_error_at_cmd": tr.y - truth.y,
            })
        if self.flight is not None:
            land = self.flight.landing
            t_land = self.flight.t_land
            v_true = veh.speed(t_land)
            offset = cfg.planner.landing_offset_s
            along = veh.state(t_land).x - land[0] - v_true * offset
            z = cfg.zone
            s.update({
                "t_land": t_land, "landing_x": float(land[0]), "landing_y": float(land[1]),
                "miss_x": float(land[0] - self.plan_at_command.x_aim),
                "miss_y": float(land[1] - self.plan_at_command.y_aim),
                "along_track_error": float(along),          # + : vehicle ahead of the landing point
                "timing_error": float(along / v_true),
                "in_zone": bool(z.x_range[0] <= land[0] <= z.x_range[1] and z.y_range[0] <= land[1] <= z.y_range[1]),
                "lateral_clearance": float(cfg.vehicle.lane_y - cfg.vehicle.width / 2 - land[1]),
                "release_delay": self.launch_event.t_release - self.launch_event.t_command,
                "carriage_speed_at_release": float(np.hypot(*self.launch_event.carriage_velocity)),
            })
        return s


class _FrameLog:
    FIELDS = ["t", "state", "veh_x", "veh_v", "spl", "spl_floor", "spl_threshold", "trigger",
              "est_x", "est_y", "est_v", "sig_x", "sig_y", "sig_v", "pred_x", "pred_sig", "true_pred_x",
              "gx", "gy", "gtx", "gty", "gvx", "gvy", "aim_x", "t_land", "t_cmd", "slack", "sigma_t",
              "feasible", "safety_ok", "px", "py", "pz"]

    def __init__(self):
        self.data: dict[str, list] = {k: [] for k in self.FIELDS}

    def add(self, sim: Simulation, t: float) -> None:
        nan = math.nan
        d = self.data
        veh = sim.world.vehicle.state(t)
        d["t"].append(t)
        d["state"].append(STATES.index(sim.state))
        d["veh_x"].append(veh.x)
        d["veh_v"].append(veh.vx)
        d["spl"].append(sim.last_spl[0])
        d["spl_floor"].append(sim.trigger.floor if sim.trigger.floor is not None else nan)
        thr = sim.trigger.threshold_db
        d["spl_threshold"].append(thr if thr is not None else nan)
        d["trigger"].append(int(sim.trigger.active))
        snap = sim.tracker.snapshot(t) if sim.state not in ("IDLE",) else None
        tau = sim.cfg.prediction.horizon_s
        if snap is not None:
            d["est_x"].append(snap.x_centre)
            d["est_y"].append(snap.y)
            d["est_v"].append(snap.speed)
            d["sig_x"].append(snap.sigma_x)
            d["sig_y"].append(snap.sigma_y)
            d["sig_v"].append(snap.sigma_v)
            sp, Pp = sim.tracker.ekf.predict(t + tau)
            d["pred_x"].append(sp[0] - sim.tracker.half_len)
            d["pred_sig"].append(math.sqrt(Pp[0, 0]))
        else:
            for k in ("est_x", "est_y", "est_v", "sig_x", "sig_y", "sig_v", "pred_x", "pred_sig"):
                d[k].append(nan)
        d["true_pred_x"].append(sim.world.vehicle.state(t + tau).x)
        g = sim.gantry
        d["gx"].append(g.x.pos)
        d["gy"].append(g.y.pos)
        d["gtx"].append(g.x.profile.target)
        d["gty"].append(g.y.profile.target)
        d["gvx"].append(g.x.vel)
        d["gvy"].append(g.y.vel)
        p = sim.plan
        d["aim_x"].append(p.x_aim if p else nan)
        d["t_land"].append(p.t_land if p else nan)
        d["t_cmd"].append(p.t_cmd if p else nan)
        d["slack"].append(p.slack if p else nan)
        d["sigma_t"].append(p.sigma_t if p else nan)
        d["feasible"].append(int(p.feasible) if p else 0)
        d["safety_ok"].append(int(sim.safety_status.ok) if sim.safety_status else 0)
        pos = sim.flight.position(t) if sim.flight is not None else None
        for i, k in enumerate(("px", "py", "pz")):
            d[k].append(pos[i] if pos is not None else nan)


def run_simulation(cfg: Config) -> RunResult:
    return Simulation(cfg).run()

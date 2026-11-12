"""Typed configuration for the simulation.

All quantities are SI (metres, seconds, m/s, kg) unless the field name says
otherwise (``*_kmh``, ``*_deg``, ``*_db``, ``*_hz``).

Coordinate frame (plan view):
    x  -- along the road, vehicle travels towards +x
    y  -- perpendicular to the road, from the rig (y = 0) towards the lane
    z  -- up (only used for the payload flight)
The rig (microphone array, radar, gantry) sits near the origin; the lane
centreline is the line y = vehicle.lane_y.
"""

from __future__ import annotations

import copy
import dataclasses
import typing
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml


@dataclass
class SimCfg:
    dt: float = 0.001             # physics / control step
    log_dt: float = 0.01          # logging resolution
    t_max: float = 0.0            # hard stop; 0 = automatic (vehicle 30 m past the zone)
    post_landing_time: float = 1.0
    seed: int = 1


@dataclass
class VehicleCfg:
    speed_kmh: float = 50.0
    x0: float = -90.0             # start position along the road
    lane_y: float = 10.0          # "detection distance": lane centre from rig
    width: float = 1.8
    length: float = 4.5
    accel: float = 0.0            # constant longitudinal acceleration (model mismatch test)
    accel_noise: float = 0.15     # random-walk acceleration std (m/s^2/sqrt(s))
    source_level_db: float = 72.0  # SPL at 7.5 m when driving at 50 km/h
    speed_exponent_db: float = 30.0  # SPL increase per decade of speed
    tone_hz: float = 600.0        # dominant tonal component (for acoustic Doppler)


@dataclass
class EnvironmentCfg:
    background_db: float = 50.0
    background_sigma_db: float = 1.5
    sound_speed: float = 343.0
    air_density: float = 1.2
    gravity: float = 9.81
    wind: list[float] = field(default_factory=lambda: [0.0, 0.0])  # known/forecast wind (x, y)
    gust_sigma: float = 0.3       # unknown per-run deviation from the forecast wind


@dataclass
class SensorTiming:
    """Shared timing/fault model, embedded in each sensor config."""
    latency: float = 0.02         # mean delay from acquisition to availability
    latency_jitter: float = 0.003  # std of that delay
    clock_offset: float = 0.0     # error of the sensor's timestamp clock
    dropout: float = 0.0          # probability that a sample is lost


@dataclass
class MicrophoneCfg:
    enabled: bool = True
    position: list[float] = field(default_factory=lambda: [0.0, 0.0])
    spacing: float = 0.5          # two-microphone baseline along x
    frame_dt: float = 0.02        # analysis frame length
    level_noise_db: float = 1.0
    sin_bearing_sigma: float = 0.02  # TDOA noise expressed in sin(theta)
    min_snr_db: float = 8.0       # bearing is only reported above this SNR
    freq_sigma_hz: float = 2.0
    timing: SensorTiming = field(default_factory=lambda: SensorTiming(latency=0.03, latency_jitter=0.005))


@dataclass
class RadarCfg:
    enabled: bool = True
    position: list[float] = field(default_factory=lambda: [0.0, 0.0])
    boresight_deg: float = -70.0  # azimuth from +y towards +x (negative = upstream)
    fov_deg: float = 40.0
    max_range: float = 60.0
    rate_hz: float = 20.0
    noise: float = 0.15           # radial speed noise std (m/s)
    bias: float = 0.0             # radial speed bias (m/s)
    carrier_hz: float = 24.125e9
    timing: SensorTiming = field(default_factory=lambda: SensorTiming(latency=0.05, latency_jitter=0.01, dropout=0.05))


@dataclass
class LidarCfg:
    enabled: bool = True
    gates_x: list[float] = field(default_factory=lambda: [-25.0, -22.0])
    mount_y: float = 0.0
    rate_hz: float = 200.0
    range_noise: float = 0.03
    max_range: float = 40.0
    gate_clock_offsets: list[float] = field(default_factory=lambda: [0.0, 0.0])
    timing: SensorTiming = field(default_factory=lambda: SensorTiming(latency=0.01, latency_jitter=0.002))


@dataclass
class TriggerCfg:
    threshold_db: float = 5.0     # level above the adaptive noise floor
    release_db: float = 3.0       # hysteresis
    min_frames: int = 3
    hold_s: float = 1.5           # stay active at least this long once triggered
    floor_alpha: float = 0.002    # noise-floor smoothing when the level is above the floor
    floor_alpha_down: float = 0.05  # ... and when it is below (falls fast, rises slowly)


@dataclass
class EstimatorCfg:
    use_radar: bool = True
    use_lidar: bool = True
    use_mic_bearing: bool = True
    compensate_sound_delay: bool = True
    bearing_var_inflation: float = 2.0  # extended source / first-order delay model: trust bearings less
    accel_sigma: float = 0.6      # process noise (white acceleration)
    lateral_accel_sigma: float = 0.05
    init_speed_kmh: float = 50.0  # prior when radar has not reported yet
    init_speed_sigma: float = 6.0
    nominal_width: float = 1.8
    width_sigma: float = 0.15
    nominal_length: float = 4.5
    length_sigma: float = 0.4
    compute_latency: float = 0.004  # processing time added before a measurement is used
    compute_jitter: float = 0.001


@dataclass
class PredictionCfg:
    horizon_s: float = 1.0        # display "where will it be in tau seconds"


@dataclass
class PayloadCfg:
    mass: float = 0.03
    diameter: float = 0.07
    drag_coeff: float = 0.47
    launch_speed: float = 7.0
    launch_speed_sigma: float = 0.08
    elevation_deg: float = 35.0
    elevation_sigma_deg: float = 0.5
    height: float = 1.0           # launch height above ground


@dataclass
class GantryCfg:
    x_range: list[float] = field(default_factory=lambda: [0.0, 4.0])
    y_range: list[float] = field(default_factory=lambda: [0.0, 1.2])
    home: list[float] = field(default_factory=lambda: [2.0, 0.6])
    v_max: float = 1.0
    a_max: float = 3.0
    axis_mass: float = 2.0
    friction: float = 4.0         # viscous, N/(m/s)
    force_max: float = 40.0
    natural_freq: float = 40.0    # closed-loop bandwidth (rad/s) used to derive PID gains
    damping: float = 0.9
    ki: float = 200.0
    position_noise: float = 0.0005


@dataclass
class ReleaseCfg:
    latency: float = 0.02
    latency_jitter: float = 0.004
    commit_lead: float = 0.3      # aim point is frozen this long before the release command
    settle_tolerance: float = 0.01  # carriage must be within this of the release point


@dataclass
class ZoneCfg:
    x_range: list[float] = field(default_factory=lambda: [0.5, 3.5])
    y_range: list[float] = field(default_factory=lambda: [5.0, 6.5])


@dataclass
class PlannerCfg:
    landing_offset_s: float = 0.0  # land this long after the vehicle is abeam the aim point
    aim_candidates: int = 25
    slack_margin: float = 0.15
    w_change: float = 0.5          # penalty for moving the aim point between re-plans
    w_slack: float = 4.0


@dataclass
class SafetyCfg:
    lane_clearance_min: float = 1.5  # metres between landing 3-sigma edge and vehicle side
    sigma_k: float = 3.0
    max_position_sigma: float = 0.6
    max_speed_sigma: float = 1.0
    speed_range_kmh: list[float] = field(default_factory=lambda: [5.0, 130.0])


@dataclass
class Config:
    sim: SimCfg = field(default_factory=SimCfg)
    vehicle: VehicleCfg = field(default_factory=VehicleCfg)
    environment: EnvironmentCfg = field(default_factory=EnvironmentCfg)
    microphone: MicrophoneCfg = field(default_factory=MicrophoneCfg)
    radar: RadarCfg = field(default_factory=RadarCfg)
    lidar: LidarCfg = field(default_factory=LidarCfg)
    trigger: TriggerCfg = field(default_factory=TriggerCfg)
    estimator: EstimatorCfg = field(default_factory=EstimatorCfg)
    prediction: PredictionCfg = field(default_factory=PredictionCfg)
    payload: PayloadCfg = field(default_factory=PayloadCfg)
    gantry: GantryCfg = field(default_factory=GantryCfg)
    release: ReleaseCfg = field(default_factory=ReleaseCfg)
    zone: ZoneCfg = field(default_factory=ZoneCfg)
    planner: PlannerCfg = field(default_factory=PlannerCfg)
    safety: SafetyCfg = field(default_factory=SafetyCfg)

    # ------------------------------------------------------------------ helpers
    def to_dict(self) -> dict[str, Any]:
        return dataclasses.asdict(self)

    def copy(self) -> "Config":
        return copy.deepcopy(self)

    def with_overrides(self, overrides: dict[str, Any]) -> "Config":
        """Return a copy with dotted-path overrides applied, e.g. {"vehicle.speed_kmh": 60}."""
        data = self.to_dict()
        for path, value in overrides.items():
            _set_path(data, path, value)
        return from_dict(Config, data)


class ConfigError(ValueError):
    pass


def _set_path(data: dict, path: str, value: Any) -> None:
    keys = path.split(".")
    node = data
    for key in keys[:-1]:
        if key not in node or not isinstance(node[key], dict):
            raise ConfigError(f"unknown config section '{path}'")
        node = node[key]
    if keys[-1] not in node:
        raise ConfigError(f"unknown config key '{path}'")
    node[keys[-1]] = value


def _deep_merge(base: dict, extra: dict, prefix: str = "") -> dict:
    out = dict(base)
    for key, value in extra.items():
        if key not in out:
            raise ConfigError(f"unknown config key '{prefix}{key}'")
        if isinstance(value, dict) and isinstance(out[key], dict):
            out[key] = _deep_merge(out[key], value, f"{prefix}{key}.")
        else:
            out[key] = value
    return out


def from_dict(cls: type, data: dict[str, Any]):
    """Build a (nested) dataclass from a dict, rejecting unknown keys."""
    hints = typing.get_type_hints(cls)
    kwargs = {}
    names = {f.name for f in dataclasses.fields(cls)}
    for key, value in data.items():
        if key not in names:
            raise ConfigError(f"unknown key '{key}' for {cls.__name__}")
        hint = hints[key]
        if dataclasses.is_dataclass(hint) and isinstance(value, dict):
            kwargs[key] = from_dict(hint, value)
        elif hint is float and isinstance(value, (int, float)) and not isinstance(value, bool):
            kwargs[key] = float(value)
        elif typing.get_origin(hint) is list and isinstance(value, (list, tuple)):
            kwargs[key] = [float(v) if isinstance(v, int) and not isinstance(v, bool) else v for v in value]
        else:
            kwargs[key] = value
    return cls(**kwargs)


def load_config(path: str | Path | None = None, overrides: dict[str, Any] | None = None) -> Config:
    """Load defaults, then an optional YAML file, then dotted overrides."""
    data = Config().to_dict()
    if path is not None:
        with open(path, "r", encoding="utf-8") as fh:
            extra = yaml.safe_load(fh) or {}
        data = _deep_merge(data, extra)
    cfg = from_dict(Config, data)
    if overrides:
        cfg = cfg.with_overrides(overrides)
    return cfg


def parse_override(text: str) -> tuple[str, Any]:
    """Parse ``a.b=value`` (value is YAML, so numbers/lists/bools work)."""
    if "=" not in text:
        raise ConfigError(f"override must look like key.path=value, got '{text}'")
    key, raw = text.split("=", 1)
    return key.strip(), yaml.safe_load(raw)

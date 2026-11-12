# vehicle-sync-sim

A simulator for an experimental mechatronics rig. The rig detects a passing
vehicle, estimates its speed and lane offset, predicts when it will be abeam a
fixed test zone, then moves a two-axis gantry and releases a soft payload so
that it lands **inside the isolated zone** at that moment.

The landing zone is fixed and separated from the lane by a keep-out corridor. A
safety gate refuses any release whose 3σ landing ellipse could reach that
corridor. The project is a simulation plus a design for a bench-scale rig: a
model car on a track, a foam payload and an enclosed area.

```
 mic array ─┐                                   ┌─ safety gate ─┐
 radar ─────┼─▶ trigger ─▶ EKF tracker ─▶ prediction ─▶ release planner ┤               ├─▶ release actuator
 LiDAR gates┘   (wake-up)  (timestamped,        (x(t+τ), P(τ))  (ballistics,  └─▶ gantry X/Y ──┘      │
                           out-of-order safe)                    timing opt.)       (trapezoid+PID)    ▼
                                                                                                  payload (RK4, drag, wind)
```

## Quick start

```bash
pip install -r requirements.txt

python -m simcore run --events                         # nominal scenario, prints summary + event log
python -m simcore run --config config/scenarios/fast_vehicle.yaml
python -m simcore run --set vehicle.speed_kmh=70 --set vehicle.lane_y=12 --out run.json

python -m simcore serve                                # interactive viewer at http://localhost:8765
python -m simcore export-viewer --out viewer.html      # self-contained HTML with all scenarios

python -m simcore montecarlo -n 200                    # error statistics vs the planner's prediction
python -m simcore sweep --param radar.noise --values 0.05,0.15,0.5,1.0
python -m pytest                                       # 26 tests
```

Any configuration value can be overridden with `--set section.key=value`.
`python -m simcore dump-config` prints the full effective configuration, and
`config/default.yaml` lists every parameter.

## What the viewer shows

* **Plan view**: road and vehicle (truth), tracked estimate with ±2σ, the
  predicted position after τ, the radar beam, the LiDAR gates lighting up as
  they are blocked, microphone trigger rings, the test zone, the keep-out
  corridor, gantry travel and carriage, the aim point with its 3σ landing
  ellipse, and the payload ground track.
* **Side elevation**: the payload arc with drag and wind, against the vehicle
  profile.
* **Telemetry**: sound level with the adaptive floor and threshold; radar
  radial speed against true and estimated speed (the cosine effect is visible);
  lane-offset estimate; prediction error at t+τ with the filter's ±2σ; gantry
  axes against targets; and how the predicted landing time is revised as data
  arrives.
* **State machine and event log**: IDLE → ARMED → TRACKING → COMMITTED →
  RELEASED → LANDED, or ABORTED with a reason.
* **Safety gate**: each check with its value and limit.
* **Monte Carlo**: landing scatter relative to the vehicle, and a histogram of
  timing errors against the predicted normal distribution.

## Configurable parameters (selection)

| key | meaning | default |
|---|---|---|
| `vehicle.speed_kmh` | vehicle speed | 50 |
| `vehicle.lane_y` | detection distance: lane centre from the rig | 10 m |
| `vehicle.accel` | constant acceleration (model-mismatch test) | 0 |
| `radar.noise`, `radar.timing.latency` | radar noise σ, latency | 0.15 m/s, 50 ms |
| `microphone.timing.latency`, `lidar.timing.latency` | sensor latencies | 30 ms, 10 ms |
| `*.timing.clock_offset`, `lidar.gate_clock_offsets` | timestamp errors | 0 |
| `*.timing.dropout` | probability of a lost sample | radar 5 % |
| `estimator.compute_latency` | processing delay | 4 ms |
| `gantry.v_max`, `gantry.a_max` | actuator speed and acceleration limits | 1 m/s, 3 m/s² |
| `release.latency`, `release.latency_jitter` | release actuator timing | 20 ms ± 4 ms |
| `payload.launch_speed`, `payload.elevation_deg` | launcher | 7 m/s, 35° |
| `planner.landing_offset_s` | land this long after the vehicle is abeam | 0 |
| `prediction.horizon_s` | τ for the displayed prediction | 1 s |
| `safety.lane_clearance_min` | minimum gap between the 3σ landing edge and the vehicle side | 1.5 m |

## Scenarios (`config/scenarios/`)

| scenario | result |
|---|---|
| `nominal` | lands in zone; timing error ≈ 8 ms (σ_t predicted 18 ms) |
| `fast_vehicle` (90 km/h) | lands; σ_t ≈ 11 ms |
| `slow_vehicle` (30 km/h) | lands; longer horizon, σ_t ≈ 31 ms |
| `degraded_sensors` | lands; σ_t grows to 25 ms |
| `no_radar` | lands; speed from the gates only, σ_t ≈ 34 ms |
| `windy` | lands; wider lateral spread |
| `accelerating` (1.5 m/s²) | lands, but 63 ms late, about 5σ (constant-velocity model limit) |
| `lane_too_close` (7 m) | **safety gate refuses** (lane clearance) |
| `quiet_ev` | trigger fires after the gates, no track, **stands down** |

## Results

Monte Carlo, 200 trials, speed ±8 km/h, lane ±0.5 m: 200/200 landed in zone.
Timing error σ is **17.7 ms**, against a mean predicted **18.6 ms**. Along-track
σ relative to the vehicle is 0.24 m, 95 % of timing errors are within 35 ms. The minimum lateral clearance is 2.8 m.
Sensitivity sweeps and their interpretation are in [docs/MATH.md](docs/MATH.md#10-sensitivity-results).

## Code map

| module | responsibility |
|---|---|
| `simcore/config.py` | typed dataclass configuration, YAML loading, dotted overrides |
| `simcore/world/` | vehicle truth trajectory, acoustic propagation (spreading, retarded time, Doppler) |
| `simcore/sensors/` | microphone array, Doppler radar, LiDAR gates; shared latency, jitter, clock and dropout model |
| `simcore/detection/trigger.py` | adaptive-floor acoustic trigger with hysteresis |
| `simcore/estimation/` | EKF with out-of-sequence replay, χ² gating, measurement models, tracker |
| `simcore/planning/ballistics.py` | RK4 flight with drag and wind, inverse solver, dispersion |
| `simcore/planning/scheduler.py` | aim-point and release-time optimisation |
| `simcore/planning/safety.py` | release safety gate |
| `simcore/control/` | time-optimal re-targetable trapezoid profiles, PID axis model, release actuator |
| `simcore/engine/runner.py` | fixed-step loop, state machine, logging, metrics |
| `simcore/experiments/` | Monte Carlo and parameter sweeps |
| `simcore/viewer/` | browser viewer: live server (stdlib only) and static exporter |

Every sensor exposes the same `poll(t, world) -> [Measurement]` interface, and
the gantry and release actuator are plain classes. A hardware-in-the-loop setup
replaces them with drivers that fill the same timestamped fields; the
estimator, planner and safety gate do not change.

## Bench-rig hardware (proposed)

* **Computers:** Raspberry Pi 5 for estimation and planning; Teensy 4.1 or
  STM32 for the 1 kHz axis loops and the hardware release timer.
* **Sensors:** 2× INMP441 I2S MEMS microphones (0.5 m baseline), an
  OPS243-A 24 GHz Doppler radar, 2× TF-Luna or TF02-Pro single-point LiDAR as
  light gates. At 1:10 scale, HC-SR04 ultrasonic sensors also work.
* **Gantry:** 2× NEMA17 with TMC2209 drivers, GT2 belts on 2020 extrusion,
  limit switches.
* **Release:** a servo-released low-energy spring launcher with a foam
  payload.
* **Safety:** a hardware e-stop in series with motor power, and an enclosed
  test area.
* **Scaling:** use Froude scaling so ballistic timing stays similar:
  `v_model = v_full·√λ`. At λ = 1:10, 50 km/h becomes 4.4 m/s.

## Roadmap

* A constant-acceleration (or IMM) tracker for accelerating vehicles: the
  largest error source found so far.
* A moving-release mode (fly-by profile ending at non-zero carriage velocity).
* A 3D viewer.
* Hardware drivers implementing the sensor and actuator interfaces.

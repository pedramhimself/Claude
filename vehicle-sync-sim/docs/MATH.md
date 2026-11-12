# Mathematical model

This document derives every calculation the simulator performs and points to the
module that implements it. Units are SI throughout.

**Frame.** `x` runs along the road (vehicle travels towards +x), `y` runs
perpendicular from the rig (y = 0) towards the lane, `z` is up. The lane
centreline is `y = d` (`vehicle.lane_y`, the "detection distance", 10 m by
default). The microphone array and radar sit at the origin; the two LiDAR
light gates look across the road at `x = -25 m` and `x = -22 m`; the test zone
is the rectangle `x ∈ [0.5, 3.5]`, `y ∈ [5.0, 6.5]`.

---

## 1. Ground truth (`world/vehicle.py`)

Speed is a drifting random walk, so a constant-velocity estimator faces a
realistic target:

```
v(t) = v0 + a·t + σ_a·W(t)          x(t) = x0 + ∫ v dt          y(t) = d
```

The trajectory is pre-computed on the 1 ms grid so the acoustic model can query
past states.

## 2. Acoustic detection (`world/acoustics.py`, `sensors/microphone.py`, `detection/trigger.py`)

**Spreading.** Point source, free field, referenced to the 7.5 m pass-by distance:

```
L(r) = L_ref(v) − 20·log10(r / 7.5),      L_ref(v) = L_50 + 30·log10(v / 50 km/h)
```

Background noise is added energetically, `L = 10·log10(10^(L_v/10) + 10^(L_bg/10))`,
plus per-frame fluctuation.

**Why a single microphone cannot range.** For a pass-by at distance d,

```
L(t) = L_cpa − 10·log10(1 + (v·(t − t0)/d)²)
```

depends on v and d only through v/d (`passby_level`, see the unit test). Without
the source level, ±5 dB of loudness uncertainty means a range error of a factor
10^(±5/20), from 0.56× to 1.78×. The microphone is therefore used as a **wake-up
trigger** and as a **bearing** sensor, never for range.

**Retarded time.** Sound heard at t left the vehicle at t_e:

```
t_e = t − |p(t_e) − p_mic| / c          (fixed-point iteration, converges like (v/c)^k)
```

At 30 m and 50 km/h the source appears about v·r/c ≈ 1.2 m behind its true position.

**Bearing (two microphones, baseline D).** The time difference of arrival gives

```
sin θ = c·Δt / D,      θ measured from broadside (+y) towards +x
```

The array reports `sin θ` with noise σ = 0.02. That corresponds to about one
48 kHz sample at D = 0.5 m.

**Doppler.** Received frequency `f = f0·c / (c − v·u)`, where u is the unit
vector from the source to the receiver. The approach and recede asymptotes give
a speed that does not depend on distance: `v = c·(f_a − f_r)/(f_a + f_r)`
(`doppler_speed`). It is only available after the vehicle has passed, so it is
logged for validation and not used for control.

**Trigger.** The noise floor adapts asymmetrically: it falls quickly and rises
slowly, and it learns only from frames that look like background:

```
floor ← (1 − a)·floor + a·L     if L < floor + 3 dB,   a = 0.05 (L < floor) or 0.002 (L > floor)
on   when L > floor + 5 dB for 3 consecutive frames
off  when L < floor + 3 dB, and at least 1.5 s after triggering
```

A symmetric floor that learns from every frame follows a slowly approaching
vehicle upwards and fires too late. The Monte Carlo runs showed this as
"no track" failures.

## 3. Radar (`sensors/radar.py`)

A CW Doppler radar measures only the radial speed of the strongest reflector
(the vehicle front):

```
v_r = (r · v) / |r|,     r = p_front − p_radar
f_d = 2·v_r·f0 / c_light            (24.125 GHz → 161 Hz per m/s)
```

**Cosine error.** |v_r| = |v|·cos φ. A fixed mounting correction
v = v_r / cos φ is sensitive to the angle: δv/v = tan φ · δφ, which is 1.3 % for
φ = 20° and δφ = 2°. Because φ changes as the car approaches, the estimator uses
the full nonlinear h(s) instead of a fixed correction.

**Range from Doppler alone** (not used by default, because the radar faces
upstream). At closest approach, v_r crosses zero with slope dv_r/dt = v²/d, so
d = v² / (dv_r/dt).

## 4. LiDAR light gates (`sensors/lidar.py`)

Each gate samples at 200 Hz. The first blocked sample marks the moment the
vehicle front crosses `x_gate`. The crossing actually happened somewhere in the
previous sample period, so the tracker uses:

```
t_cross = t_stamp − T_s/2,       σ_t = T_s/√12   (uniform quantisation)
y_c     = mount_y + range + W_nom/2,   σ_y² = σ_range² + (σ_W/2)²
```

Two gates a distance b apart give the classic speed estimate, with error
propagation

```
v = b/Δt,      σ_v / v = √( (σ_b/b)² + (σ_Δt/Δt)² )
```

With b = 3 m, T_s = 5 ms and v = 13.9 m/s: σ_Δt = 2 ms over Δt = 216 ms, which is
0.9 %, or 0.13 m/s. The EKF does this implicitly when it fuses both gate events.

## 5. State estimation (`estimation/ekf.py`, `estimation/tracker.py`)

State: `s = [x_f, y_c, v_x, v_y]`. Here x_f is the position of the vehicle
**front** (what the gates and radar see) and y_c is the lane-centre offset. The
planner uses the vehicle centre, x_c = x_f − L_nom/2.

**Motion model** (constant velocity, white-noise acceleration σ_a):

```
s_{k+1} = F s_k + w,        F = [[I, Δt·I], [0, I]]
Q = blockdiag_axis  σ²·[[Δt⁴/4, Δt³/2], [Δt³/2, Δt²]]
```

**Update** (EKF, Joseph form for numerical stability):

```
y = z − h(s),   S = H P Hᵀ + R,   K = P Hᵀ S⁻¹
s ← s + K y,    P ← (I − K H) P (I − K H)ᵀ + K R Kᵀ
```

**Measurement models:**

| sensor | z | h(s) |
|---|---|---|
| radar | v_r | (p − p_r)·v / ‖p − p_r‖ (analytic Jacobian) |
| gate enter | (x_gate, y_c) | (x_f, y_c) |
| gate exit / range | y_c | y_c |
| mic bearing | sin θ | (x_e − x_m)/‖p_e − p_m‖, p_e = (x_f − L/2, y_c) − v·r/c |

The bearing model includes the first-order retarded position. Switching that
off (`estimator.compensate_sound_delay: false`) biases the lateral estimate by
0.15–0.25 m. Bearing variance is inflated ×2 to cover the extended-source and
first-order approximations. Without the inflation the filter's lateral σ was
about 20 % optimistic over 40 seeds; with it, σ matches the measured spread.

**Outlier gate.** An update is rejected if the normalised innovation squared
`yᵀS⁻¹y` exceeds the χ² 99.9 % quantile (10.8 for 1 dof, 13.8 for 2 dof).

**Latency (out-of-sequence measurements).** Each measurement is fused at its
timestamp. If it arrives after newer data has been processed, the filter
restores the stored state from just before that timestamp and re-processes
everything after it. Latency therefore costs only freshness, never accuracy.
The sweep below confirms this: 0 to 300 ms of radar latency leaves the timing
error unchanged. Timestamp (clock) errors cannot be fixed this way, and they do
show up.

**Prediction to any horizon τ:**

```
ŝ(t+τ) = F(τ) ŝ(t),       P(τ) = F(τ) P F(τ)ᵀ + Q(τ)
σ_x(τ)² = σ_x² + 2τ·σ_xv + τ²·σ_v² + Q_xx(τ)
```

## 6. Payload flight (`planning/ballistics.py`)

```
dv/dt = g − k·|v − w|·(v − w),        k = ρ·C_d·A / (2m)
```

This is integrated with RK4 (2 ms steps) until z = 0, with linear interpolation
of the touchdown point. For the default 30 g, 70 mm foam ball, k = 0.036 m⁻¹: at
7 m/s, drag decelerates the ball at 1.8 m/s², which is not negligible next to g.

Closed form without drag, used as a unit-test reference:

```
T = (u·sin α + √(u²·sin²α + 2gh)) / g,        R = u·cos α · T
```

**Carriage velocity.** The initial velocity is `v0 = v_launcher + v_carriage`,
so releasing from a moving gantry shifts the landing point by `v_carriage·T`.
The release controller fires at the commanded time even if the carriage has not
settled. The safety gate's `carriage_ready` check stops that from happening
unless the carriage is settled within 1 cm and moving slower than 5 cm/s.

**Inverse problem (shooting method).** For a fixed elevation the range R(u) is
monotonic, so `solve_speed_for_range` bisects on u until R(u) = R_target.

**Dispersion (first-order).**

```
σ_y² = (∂R/∂u·σ_u)² + (∂R/∂α·σ_α)² + (∂R/∂w_y·σ_w)²,       σ_x = |∂X/∂w_x|·σ_w
```

The partial derivatives come from central finite differences of the RK4 model.
With the defaults, σ_x ≈ 0.03 m and σ_y ≈ 0.10 m.

## 7. Gantry motion control (`control/profiles.py`, `control/gantry.py`)

**Time-optimal trapezoid** for a rest-to-rest move of distance D:

```
t(D) = D/v_max + v_max/a_max     if D ≥ v_max²/a_max
     = 2·√(D/a_max)              otherwise
```

**Re-targeting while moving.** The planner refines the aim point continuously.
Starting from velocity v0, with u0 = v0·sign(e) and d = |e|:

* u0 < 0 (moving away): brake to rest, then move rest to rest.
* u0²/(2a) > d (too fast to stop in time): brake past the target, then come back.
* Otherwise: accelerate to v_p = min(v_max, √(a·d + u0²/2)), cruise, then brake.

The same segments drive the reference trajectory and the planner's
move-time estimate, so the feasibility check uses the time actually achieved.

**Axis dynamics and control.** Each axis is a mass with viscous friction and a
force-limited motor:

```
m·ẍ = F − b·ẋ
F = m·a_ref + b·v_ref + K_p·e + K_d·ė + K_i·∫e,      |F| ≤ F_max
K_p = m·ω_n²,  K_d = 2ζω_n·m      (ω_n = 40 rad/s, ζ = 0.9)
```

Feed-forward handles the planned motion, and the PID only corrects residual
error. The tracking error stays around 1 mm during a 1 m/s move.

## 8. Release scheduling and timing optimisation (`planning/scheduler.py`)

The payload must land at y_aim (zone centre) at the moment the vehicle centre
is abeam the landing point, plus an optional offset Δ. For a candidate
along-road aim point x_a:

```
h        = (x_a − x̂_c) / v̂                      prediction horizon
t_abeam  = t_now + h
t_land   = t_abeam + Δ
t_rel    = t_land − T_flight
t_cmd    = t_rel − τ_release                     (mean actuator latency compensated)
p_rel    = (x_a, y_aim) − (dx, dy)               (launcher landing offset)
slack    = t_cmd − (t_now + t_move(p_carriage → p_rel))
```

**Timing uncertainty** propagates the EKF covariance, including the x–v
correlation:

```
σ_x,pred² = [1 h]·P_{x,v}·[1 h]ᵀ + (σ_L/2)² + (v̂·σ_τ,release)²
σ_t       = σ_x,pred / v̂
```

**Objective.** The aim point is chosen from 25 candidates spanning the zone
(shrunk by 3σ_x of landing dispersion), minimising

```
J(x_a) = (v̂·σ_t)² + σ_land,x²                      expected along-track miss
       + w_slack · max(0, margin − slack)²         robustness to timing errors
       + w_change · (x_a − x_a,prev)²              smooth re-planning
```

subject to slack ≥ 0 and reachability. The trade-off: an earlier aim point
gives a shorter horizon (smaller σ_t) but less time to move the gantry.

**Commit.** 0.3 s before t_cmd the aim point is frozen so the carriage can
settle. After that, only t_cmd keeps being refined, at 50 Hz in the planner and
at 1 kHz in the release timer.

**Worked error budget** (50 km/h, horizon ≈ 2 s at the first plan and ≈ 1 s at
the command):

```
σ_t ≈ √( (σ_x/v)² + (h·σ_v/v)² + (σ_L/2v)² + σ_τ² )
    ≈ √( 0.004² + 0.011² + 0.014² + 0.004² ) ≈ 0.018 s
```

At 13.9 m/s that is about 0.25 m along the road. Monte Carlo (200 trials, speed
±8 km/h, lane ±0.5 m) measured **17.7 ms**, against a mean predicted **18.6 ms**
(ratio 0.95).

## 9. Safety gate (`planning/safety.py`)

Release is permitted only if every check passes at the release instant (k = 3):

1. **in_zone**: the k·σ landing ellipse lies inside the declared zone.
2. **lane_clearance**: (ŷ_c − W/2 − k·σ_vehicle) − (y_aim + k·σ_land,y) ≥ 1.5 m.
3. **track_quality**: v̂·σ_t ≤ 0.6 m and σ_v ≤ 1 m/s.
4. **speed_plausible**: 5 to 130 km/h.
5. **schedule**: the plan is feasible or committed.
6. **carriage_ready**: within 1 cm of the release point and moving slower than 5 cm/s.

Any failure aborts the release, sends the carriage home and logs the failed
checks.

## 10. Sensitivity results

Generated with `python -m simcore sweep … -n 40` (seed-matched trials, nominal
scenario):

| parameter | value | timing mean | timing σ | predicted σ |
|---|---|---|---|---|
| radar noise [m/s] | 0.05 / 0.15 / 0.5 / 1.0 | | 19.4 / 19.3 / 22.7 / 26.0 ms | 17.4 / 18.2 / 23.1 / 28.4 ms |
| radar latency [s] | 0 / 0.1 / 0.3 | | 19.3 / 19.3 / 19.9 ms | 18.2 ms |
| release jitter [s] | 0 / 0.004 / 0.02 | | 19.1 / 19.3 / 24.6 ms | 17.8 / 18.2 / 26.8 ms |
| second gate clock offset [s] | 0 / 0.005 / 0.02 | 3 / **12** / 3 ms | 19–20 ms | 18–19 ms |
| radar used | yes / no | 3 / 5 ms | 19.3 / 27.7 ms | 18.2 / 34.0 ms |
| vehicle acceleration [m/s²] | 0 / 0.5 / 1.5 | 3 / **38** / **65** ms | ~16–19 ms | 12–18 ms |
| vehicle speed [km/h] | 30 / 50 / 90 / 120 | | 29.9 / 19.3 / 13.5 / 11.7 ms | 37.9 / 18.2 / 10.5 / 11.4 ms |

What the sweeps show:

* **Latency is free; clock errors are not.** Timestamped OOSM handling absorbs
  latency completely. A 5 ms clock offset on one gate produces a 12 ms timing
  bias that no variance reveals. A 20 ms offset is large enough that the χ²
  gate rejects the bad gate event.
* **The constant-velocity model is the dominant limitation for accelerating
  traffic**: 1.5 m/s² gives a 65 ms bias, about 5σ. The next step is an
  augmented constant-acceleration state (see the roadmap in the README).
* **The predicted σ is well calibrated across noise levels.** It is
  conservative at low speed (long horizons) and slightly optimistic at high
  speed, where gantry dynamics and the fixed release latency matter more.

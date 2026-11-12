"""Time-optimal trapezoidal motion profiles for one gantry axis.

Rest-to-rest move of distance D with limits v_max, a_max:

    t_move(D) = D / v_max + v_max / a_max      if D >= v_max^2 / a_max   (reaches cruise)
              = 2 sqrt(D / a_max)              otherwise                 (triangular)

General case (the planner re-targets the axis while it is moving), starting
with velocity v0 and finishing at rest.  Work along the direction to the
target, u0 = v0 . sign(e), d = |e|:

* u0 < 0 (moving away):     brake to rest, then rest-to-rest over d + u0^2/(2a)
* u0^2/(2a) > d (too fast): brake to rest past the target, then come back
* otherwise:                accelerate to v_p, cruise, brake, with
                            v_p = min(v_max, sqrt(a d + u0^2 / 2))

``OnlineProfile`` builds these segments whenever the target changes and
samples them every control step, so the reference is exact (no discretisation
drift) and the move time used by the planner is the time actually achieved.
"""

from __future__ import annotations

import math


def trapezoid_time(distance: float, v_max: float, a_max: float) -> float:
    d = abs(distance)
    if d < 1e-12:
        return 0.0
    if d >= v_max ** 2 / a_max:
        return d / v_max + v_max / a_max
    return 2.0 * math.sqrt(d / a_max)


def plan_segments(p0: float, v0: float, p1: float, v_max: float, a_max: float) -> list[tuple[float, float]]:
    """Return [(duration, acceleration), ...] moving from (p0, v0) to (p1, 0)."""
    e = p1 - p0
    if abs(e) < 1e-12 and abs(v0) < 1e-12:
        return []
    s = math.copysign(1.0, e) if abs(e) >= 1e-12 else -math.copysign(1.0, v0)
    d, u0 = abs(e), v0 * s
    a = a_max
    if u0 < 0 or u0 * u0 / (2 * a) > d + 1e-12:
        # brake to rest first, then a rest-to-rest move from wherever we stopped
        t_brake = abs(v0) / a
        p_stop = p0 + v0 * t_brake / 2
        return [(t_brake, -math.copysign(a, v0))] + plan_segments(p_stop, 0.0, p1, v_max, a_max)
    segs: list[tuple[float, float]] = []
    if u0 > v_max:  # entered faster than allowed (limit changed): brake to v_max first
        segs.append(((u0 - v_max) / a, -s * a))
        d -= (u0 * u0 - v_max * v_max) / (2 * a)
        u0 = v_max
    vp = min(v_max, math.sqrt(a * d + u0 * u0 / 2))
    d_acc = (vp * vp - u0 * u0) / (2 * a)
    d_dec = vp * vp / (2 * a)
    d_cruise = max(0.0, d - d_acc - d_dec)
    if vp > u0:
        segs.append(((vp - u0) / a, s * a))
    if d_cruise > 0:
        segs.append((d_cruise / vp, 0.0))
    segs.append((vp / a, -s * a))
    return segs


def move_time_from_state(pos: float, vel: float, target: float, v_max: float, a_max: float) -> float:
    return sum(dur for dur, _ in plan_segments(pos, vel, target, v_max, a_max))


class OnlineProfile:
    def __init__(self, pos: float, v_max: float, a_max: float):
        self.pos = pos
        self.vel = 0.0
        self.acc = 0.0
        self.v_max = v_max
        self.a_max = a_max
        self._target = pos
        self._segs: list[tuple[float, float]] = []
        self._duration = 0.0
        self._t = 0.0
        self._origin = (pos, 0.0)

    @property
    def target(self) -> float:
        return self._target

    @target.setter
    def target(self, value: float) -> None:
        if abs(value - self._target) < 1e-9:
            return
        self._target = value
        self._origin = (self.pos, self.vel)
        self._segs = plan_segments(self.pos, self.vel, value, self.v_max, self.a_max)
        self._duration = sum(d for d, _ in self._segs)
        self._t = 0.0

    @property
    def remaining_time(self) -> float:
        return max(0.0, self._duration - self._t) if self._segs else 0.0

    def _sample(self, t: float) -> tuple[float, float, float]:
        p, v = self._origin
        for dur, acc in self._segs:
            if t <= dur:
                return p + v * t + 0.5 * acc * t * t, v + acc * t, acc
            p, v, t = p + v * dur + 0.5 * acc * dur * dur, v + acc * dur, t - dur
        return self._target, 0.0, 0.0

    def step(self, dt: float) -> tuple[float, float, float]:
        if not self._segs:
            self.pos, self.vel, self.acc = self._target, 0.0, 0.0
            return self.pos, self.vel, self.acc
        self._t += dt
        if self._t >= self._duration:
            self._segs = []
            self.pos, self.vel, self.acc = self._target, 0.0, 0.0
        else:
            self.pos, self.vel, self.acc = self._sample(self._t)
        return self.pos, self.vel, self.acc

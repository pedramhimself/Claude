"""Common sensor interface and timing / fault model.

Every measurement carries three times:

    t_true   -- when the physical quantity was actually sampled (truth, for analysis)
    t_stamp  -- what the sensor *says* the time was (t_true + clock offset)
    t_avail  -- when the number reaches the processing computer (t_true + latency)

The estimator only ever sees ``t_stamp`` and is only allowed to use a
measurement once the simulation clock passes ``t_avail``.  A hardware driver
for the bench rig implements the same ``poll`` method and fills the same
fields from real timestamps.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import numpy as np

from ..config import SensorTiming


@dataclass
class Measurement:
    sensor: str
    kind: str
    t_true: float
    t_stamp: float
    t_avail: float
    z: np.ndarray
    R: np.ndarray
    meta: dict[str, Any] = field(default_factory=dict)

    def __lt__(self, other: "Measurement") -> bool:  # for heapq ordering
        return self.t_avail < other.t_avail


class Sensor:
    name = "sensor"

    def __init__(self, timing: SensorTiming, rng: np.random.Generator):
        self.timing = timing
        self.rng = rng

    def _make(self, kind: str, t_true: float, z, R, clock_offset: float | None = None, **meta) -> Measurement | None:
        if self.timing.dropout > 0 and self.rng.random() < self.timing.dropout:
            return None
        offset = self.timing.clock_offset if clock_offset is None else clock_offset
        latency = max(0.0, self.timing.latency + self.timing.latency_jitter * self.rng.standard_normal())
        return Measurement(
            sensor=self.name,
            kind=kind,
            t_true=t_true,
            t_stamp=t_true + offset,
            t_avail=t_true + latency,
            z=np.atleast_1d(np.asarray(z, dtype=float)),
            R=np.atleast_2d(np.asarray(R, dtype=float)),
            meta=meta,
        )

    def poll(self, t: float, world) -> list[Measurement]:  # pragma: no cover - interface
        raise NotImplementedError

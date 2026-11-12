"""Acoustic wake-up trigger with an adaptive noise floor and hysteresis.

    floor_k = (1 - a) * floor_{k-1} + a * L_k     (only while idle and L_k < floor + release)
              a = alpha_down if L_k < floor else alpha_up   (falls fast, rises slowly)
    trigger  when  L_k > floor + threshold   for  min_frames consecutive frames
    release  when  L_k < floor + release_db   and at least hold_s after triggering
"""

from __future__ import annotations

from dataclasses import dataclass

from ..config import TriggerCfg


@dataclass
class TriggerEvent:
    t: float
    kind: str        # "on" | "off"
    level_db: float
    floor_db: float


class AcousticTrigger:
    def __init__(self, cfg: TriggerCfg, initial_floor_db: float | None = None):
        self.cfg = cfg
        self.floor: float | None = initial_floor_db
        self.active = False
        self.count = 0
        self.t_on = -1e9

    @property
    def threshold_db(self) -> float | None:
        return None if self.floor is None else self.floor + self.cfg.threshold_db

    def update(self, t: float, level_db: float) -> TriggerEvent | None:
        if self.floor is None:
            self.floor = level_db
            return None
        if not self.active:
            if level_db > self.floor + self.cfg.threshold_db:
                self.count += 1
                if self.count >= self.cfg.min_frames:
                    self.active = True
                    self.t_on = t
                    return TriggerEvent(t, "on", level_db, self.floor)
            else:
                self.count = 0
                # Only learn the floor from frames that look like background, otherwise a
                # slowly approaching vehicle would drag the floor up with it.
                # Asymmetric smoothing: the floor drops quickly but rises slowly.
                if level_db < self.floor + self.cfg.release_db:
                    a = self.cfg.floor_alpha_down if level_db < self.floor else self.cfg.floor_alpha
                    self.floor = (1 - a) * self.floor + a * level_db
        elif level_db < self.floor + self.cfg.release_db and t - self.t_on >= self.cfg.hold_s:
            self.active = False
            self.count = 0
            return TriggerEvent(t, "off", level_db, self.floor)
        return None

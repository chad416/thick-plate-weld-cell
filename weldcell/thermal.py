"""Lumped thermal model (F05): part bulk temperature plus a local excess per seam zone."""
from __future__ import annotations

import math

from .part import Part


class Thermal:
    def __init__(self, part: Part):
        self.part = part
        th = part.thermal
        self.amb = float(th["ambient_c"])
        self.bulk = float(th["start_c"])
        self.excess = {s.id: 0.0 for s in part.seams}
        self.frac = float(th["local_fraction"])
        self.tau_local = float(th["tau_local_s"])
        self.tau_bulk = float(th["tau_bulk_s"])
        self.heater_w = float(th["heater_kw"]) * 1000.0
        self.mc = part.mass_kg * part.c
        self.zone_mc = {s.id: part.zone_mass_kg(s) * part.c for s in part.seams}

    def copy(self) -> "Thermal":
        t = Thermal.__new__(Thermal)
        t.__dict__.update(self.__dict__)
        t.excess = dict(self.excess)
        return t

    def step(self, dt: float, arc_seam: str | None = None, arc_power_w: float = 0.0, heater: bool = False) -> None:
        p_in = arc_power_w + (self.heater_w if heater else 0.0)
        self.bulk += p_in * dt / self.mc
        self.bulk = self.amb + (self.bulk - self.amb) * math.exp(-dt / self.tau_bulk)
        k = math.exp(-dt / self.tau_local)
        for s in self.excess:
            self.excess[s] *= k
        if arc_seam is not None and arc_power_w > 0:
            self.excess[arc_seam] += self.frac * arc_power_w * dt / self.zone_mc[arc_seam]

    def temp(self, seam: str) -> float:
        return self.bulk + self.excess[seam]

    def temps(self) -> dict[str, float]:
        return {s: self.bulk + e for s, e in self.excess.items()}

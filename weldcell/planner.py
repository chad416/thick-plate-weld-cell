"""Sequence planners (F06, F07): which pass to weld next, and when.

Both planners run the same time and thermal model forward. "naive" welds seam by seam
in file order and waits on the hot seam. "balanced" picks, at every decision, the next
pass of a seam that is inside its temperature window, scoring the distortion balance,
the positioner move and the seam temperature.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

from .part import Part, Pass, Wps, plan_part
from .thermal import Thermal

# Cell timing model, shared with the plant simulation.
APPROACH_S = 8.0
RETRACT_S = 5.0
PREARC_S = 2.5
POS_SPEED_DEG_S = 6.0
CLEAN_S = 20.0
CLEAN_EVERY = 6
HEATER_HYST_C = 10.0
MEAS_MARGIN_C = 6.0    # same margin the PLC keeps off both limits (pyrometer uncertainty)
DT = 5.0


@dataclass
class Step:
    p: Pass
    t_arc_start: float
    t_arc_end: float
    temp_start: float
    wait_s: float
    preheat_s: float
    pos_move_s: float


@dataclass
class Plan:
    policy: str
    steps: list[Step]
    cycle_s: float
    arc_s: float
    wait_s: float
    preheat_s: float
    pos_moves: int
    max_imbalance: float
    imbalance_trace: list[tuple[float, float]] = field(repr=False, default_factory=list)
    temp_trace: list[tuple[float, dict]] = field(repr=False, default_factory=list)

    @property
    def order(self) -> list[Pass]:
        return [s.p for s in self.steps]

    def summary(self) -> dict:
        return {"policy": self.policy, "passes": len(self.steps), "cycle_h": round(self.cycle_s / 3600, 2),
                "arc_on_share": round(self.arc_s / self.cycle_s, 3), "wait_h": round(self.wait_s / 3600, 2),
                "preheat_h": round(self.preheat_s / 3600, 2), "positioner_moves": self.pos_moves,
                "max_imbalance": round(self.max_imbalance, 3),
                "max_start_temp_c": round(max(s.temp_start for s in self.steps), 1),
                "min_start_temp_c": round(min(s.temp_start for s in self.steps), 1)}


def pos_time(a: tuple[float, float], b: tuple[float, float]) -> float:
    d = max(abs(a[0] - b[0]), min(abs(a[1] - b[1]) % 360, 360 - abs(a[1] - b[1]) % 360))
    return d / POS_SPEED_DEG_S


def _unit(v: tuple[float, float]) -> tuple[float, float]:
    n = math.hypot(*v)
    return (v[0] / n, v[1] / n) if n else (0.0, 0.0)


class _Engine:
    def __init__(self, part: Part, wps: Wps):
        self.part, self.wps = part, wps
        self.th = Thermal(part)
        self.t = 0.0
        self.orient = (0.0, 0.0)
        self.sx = self.sy = 0.0
        self.passes_since_clean = 0
        self.steps: list[Step] = []
        self.imb: list[tuple[float, float]] = []
        self.temps: list[tuple[float, dict]] = [(0.0, self.th.temps())]
        allp = [p for v in plan_part(part, wps).values() for p in v]
        self.w_ref = sum(p.energy_j for p in allp) / len(part.seams)
        self.wait = self.pre = 0.0
        self.moves = 0

    def advance(self, dur: float, arc: Pass | None = None, heater: bool = False) -> None:
        while dur > 1e-9:
            dt = min(DT, dur)
            power = arc.energy_j / arc.arc_time_s if arc else 0.0
            self.th.step(dt, arc.seam if arc else None, power, heater)
            self.t += dt
            dur -= dt
        self.temps.append((self.t, self.th.temps()))

    def imbalance_after(self, p: Pass) -> float:
        ux, uy = _unit(self.part.seam(p.seam).side)
        return math.hypot(self.sx + p.energy_j * ux, self.sy + p.energy_j * uy) / self.w_ref

    def do(self, p: Pass) -> None:
        seam = self.part.seam(p.seam)
        mv = pos_time(self.orient, seam.orient)
        if mv > 0:
            self.moves += 1
            self.advance(mv)
            self.orient = seam.orient
        w0 = self.t
        while self.th.temp(p.seam) > self.wps.interpass_max_c - MEAS_MARGIN_C:
            self.advance(DT)
        self.wait += self.t - w0
        h0 = self.t
        if self.th.temp(p.seam) < self.wps.preheat_min_c + MEAS_MARGIN_C:
            while self.th.temp(p.seam) < self.wps.preheat_min_c + HEATER_HYST_C:
                self.advance(DT, heater=True)
        self.pre += self.t - h0
        wait_s, preheat_s = h0 - w0, self.t - h0
        self.advance(PREARC_S + APPROACH_S)
        t_start, temp = self.t, self.th.temp(p.seam)
        self.advance(p.arc_time_s, arc=p)
        self.steps.append(Step(p, t_start, self.t, temp, round(wait_s, 1), round(preheat_s, 1), mv))
        ux, uy = _unit(seam.side)
        self.sx += p.energy_j * ux
        self.sy += p.energy_j * uy
        self.imb.append((self.t, math.hypot(self.sx, self.sy) / self.w_ref))
        self.advance(RETRACT_S)
        self.passes_since_clean += 1
        if self.passes_since_clean >= CLEAN_EVERY:
            self.advance(CLEAN_S)
            self.passes_since_clean = 0

    def result(self, policy: str) -> Plan:
        arc = sum(s.t_arc_end - s.t_arc_start for s in self.steps)
        return Plan(policy, self.steps, self.t, arc, self.wait, self.pre, self.moves,
                    max(v for _, v in self.imb), self.imb, self.temps)


def plan_naive(part: Part, wps: Wps) -> Plan:
    e = _Engine(part, wps)
    for s in part.seams:
        for p in plan_part(part, wps)[s.id]:
            e.do(p)
    return e.result("naive")


def plan_balanced(part: Part, wps: Wps, w_imb: float = 1.0, w_pos: float = 1 / 300, w_temp: float = 0.3) -> Plan:
    e = _Engine(part, wps)
    queues = {k: list(v) for k, v in plan_part(part, wps).items()}
    span = wps.interpass_max_c - wps.preheat_min_c
    while any(queues.values()):
        heads = [q[0] for q in queues.values() if q]
        best, best_score = None, math.inf
        for p in heads:
            seam = part.seam(p.seam)
            mv = pos_time(e.orient, seam.orient)
            temp = e.th.temp(p.seam)
            # temperature when the arc would start (after the positioner move)
            t_at = temp - (temp - e.th.bulk) * (1 - math.exp(-mv / e.th.tau_local))
            hot = max(0.0, t_at - (wps.interpass_max_c - MEAS_MARGIN_C))
            score = (w_imb * e.imbalance_after(p) + w_pos * mv + w_temp * max(0.0, t_at - wps.preheat_min_c) / span
                     + 10.0 * hot / span)
            # finish a seam's root before others start their fill (keeps the joint stable)
            if p.kind == "root":
                score -= 0.5
            if score < best_score:
                best, best_score = p, score
        queues[best.seam].pop(0)
        e.do(best)
    return e.result("balanced")


def check_window(plan: Plan, wps: Wps, tol: float = 1e-6) -> list[str]:
    """F07: every pass starts inside [preheat_min, interpass_max] (predicted)."""
    bad = []
    for s in plan.steps:
        if s.temp_start > wps.interpass_max_c + tol or s.temp_start < wps.preheat_min_c - tol:
            bad.append(f"{s.p.label} starts at {s.temp_start:.1f} C")
    return bad

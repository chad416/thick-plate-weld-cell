"""The simulated cell around the PLC function block: robot, positioner, power source, clamp,
fume extraction, gas, heater, pyrometer, light curtain and an operator who clears faults
(F09-F13, F16). One loop step = one PLC scan."""
from __future__ import annotations

import math
import random
from dataclasses import dataclass, field
from pathlib import Path

from . import planner as P
from .part import Part, Pass, Wps
from .stlang import FBInstance, compile_file
from .thermal import Thermal

ROOT = Path(__file__).resolve().parents[1]
ST_FILE = ROOT / "plc" / "FB_WeldCell.st"
S_WELD_START, S_WELD, S_FAULT, S_IDLE = 60, 61, 100, 0
FAULT_NAMES = {0: "", 1: "CLAMP", 2: "POSITIONER", 3: "GAS", 4: "FUME", 5: "ARC", 6: "SENSOR", 7: "ROBOT", 8: "WIRE"}
FAULT_KINDS = ("arcstart", "arcfail", "gas", "fume", "curtain", "clamp", "positioner", "pyro", "wfs")
OPERATOR_DELAY_S = 30.0
CURTAIN_BREAK_S = 15.0
CLAMP_S, FUME_S, GAS_S = 3.0, 2.0, 0.5
_COMPILED = {}


def fb_code():
    key = ST_FILE.stat().st_mtime
    if key not in _COMPILED:
        _COMPILED.clear()
        _COMPILED[key] = compile_file(ST_FILE)
    return _COMPILED[key]


@dataclass(frozen=True)
class CellFault:
    kind: str
    pass_no: int = 0
    factor: float = 1.0

    @property
    def spec(self) -> str:
        if self.kind == "clamp":
            return "clamp"
        if self.kind == "wfs":
            return f"wfs:{self.pass_no}:{self.factor}"
        return f"{self.kind}:{self.pass_no}"


def parse_fault(spec: str) -> CellFault:
    parts = spec.split(":")
    if parts[0] not in FAULT_KINDS:
        raise ValueError(f"unknown cell fault {parts[0]!r} (known: {', '.join(FAULT_KINDS)})")
    if parts[0] == "clamp":
        return CellFault("clamp")
    if parts[0] == "wfs":
        return CellFault("wfs", int(parts[1]), float(parts[2]))
    return CellFault(parts[0], int(parts[1]))


@dataclass
class WeldRecord:
    serial: str
    seq: int
    pass_label: str
    seam: str
    pass_no: int
    layer: int
    bead: int
    kind: str
    job: int
    current_a: float
    voltage_v: float
    wfs_m_min: float
    travel_mm_min: float
    arc_time_s: float
    heat_input_kj_mm: float
    temp_start_c: float          # measured by the PLC (pyrometer)
    temp_start_true_c: float     # plant truth, simulation only
    retries: int
    result: str                  # OK / NOK_PARAM / INTERRUPTED
    t_arc_start_s: float
    t_arc_end_s: float


@dataclass
class CellRun:
    serial: str
    part: str
    wps: str
    policy: str
    done: bool
    t_total_s: float
    records: list[WeldRecord]
    events: list[dict]
    violations: list[dict]
    temp_trace: list[tuple[float, dict]] = field(repr=False)
    heater_s: float = 0.0
    wait_s: float = 0.0
    faults: list[str] = field(default_factory=list)

    @property
    def counts(self) -> dict:
        c = {"OK": 0, "NOK_PARAM": 0, "INTERRUPTED": 0}
        for r in self.records:
            c[r.result] += 1
        return c


class Plant:
    def __init__(self, part: Part, wps: Wps, order: list[Pass], faults: list[CellFault], rng: random.Random):
        self.part, self.wps, self.rng = part, wps, rng
        self.th = Thermal(part)
        self.jobs = {p.job: p for p in order}
        self.faults = list(faults)
        self.t = 0.0
        self.clamped, self.clamp_t = False, 0.0
        self.tilt = self.rot = 0.0
        self.fume_t = self.gas_t = 0.0
        self.robot = "home"
        self.robot_t = 0.0
        self.job = 0
        self.progress = 0.0
        self.busy = self.done = self.arcfault = False
        self.home = True
        self.arc_on = False
        self.current = self.voltage = 0.0
        self.attempts: dict[int, int] = {}
        self.curtain_until = -1.0
        self.curtain_fired: set[int] = set()
        self.frozen: list[float] | None = None
        self.violations: list[dict] = []
        self.acc = {"i": 0.0, "u": 0.0, "n": 0, "arc": 0.0, "t0": None, "t1": None, "true_t0": None}
        self.pass_idx = 0

    def active(self, kind: str) -> CellFault | None:
        for f in self.faults:
            if f.kind == kind and (f.kind == "clamp" or self.pass_idx == f.pass_no or
                                   (f.kind in ("pyro", "wfs") and self.pass_idx >= f.pass_no)):
                return f
        return None

    def clear(self, kind: str) -> None:
        self.faults = [f for f in self.faults if f.kind != kind]

    def safety_ok(self) -> bool:
        return self.t >= self.curtain_until

    def violate(self, what: str) -> None:
        if not self.violations or self.violations[-1]["what"] != what or self.t - self.violations[-1]["t"] > 5:
            self.violations.append({"t": round(self.t, 1), "what": what, "pass": self.pass_idx})

    def temps_measured(self) -> list[float]:
        true = [self.th.temp(s.id) for s in self.part.seams]
        if self.active("pyro"):
            if self.frozen is None:
                self.frozen = [round(x + self.rng.gauss(0, 2.0), 1) for x in true]
            return list(self.frozen)
        self.frozen = None
        return [round(x + self.rng.gauss(0, 2.0), 1) for x in true]

    def step(self, dt: float, o) -> None:
        self.pass_idx = o.PassIdx
        # clamp
        if o.ClampCmd and not self.active("clamp"):
            self.clamp_t = min(self.clamp_t + dt, CLAMP_S)
        elif not o.ClampCmd:
            self.clamp_t = max(self.clamp_t - dt, 0.0)
        self.clamped = self.clamp_t >= CLAMP_S
        # fume, gas
        self.fume_t = self.fume_t + dt if o.FumeCmd and not self.active("fume") else 0.0
        self.gas_t = self.gas_t + dt if o.GasCmd and not self.active("gas") else 0.0
        # positioner (moves while PosStart, shortest way on the rotary axis)
        if o.PosStart and not self.active("positioner"):
            moved = False
            step = P.POS_SPEED_DEG_S * dt
            dtilt = o.PosTiltSet - self.tilt
            if abs(dtilt) > 1e-6:
                self.tilt += max(-step, min(step, dtilt))
                moved = True
            drot = (o.PosRotSet - self.rot + 180.0) % 360.0 - 180.0
            if abs(drot) > 1e-6:
                self.rot = (self.rot + max(-step, min(step, drot))) % 360.0
                moved = True
            if moved and not self.home:
                self.violate("POSITIONER_MOVED_ROBOT_OUT")
        self._robot(dt, o)
        # curtain scenario: break mid-arc of the given pass (seen by the PLC on its next scan)
        f = self.active("curtain")
        if f and self.robot == "arc" and f.pass_no not in self.curtain_fired:
            pj = self.jobs.get(self.job)
            if pj and self.progress > 0.3 * pj.length_mm:
                self.curtain_fired.add(f.pass_no)
                self.curtain_until = self.t + dt + CURTAIN_BREAK_S
        # heat
        power = 0.0
        seam = None
        if self.arc_on:
            pj = self.jobs[self.job]
            power = self.voltage * self.current * self.wps.k
            seam = pj.seam
        self.th.step(dt, seam, power, heater=o.HeaterOn)
        self.t += dt

    def in_position(self, o) -> bool:
        drot = abs((o.PosRotSet - self.rot + 180.0) % 360.0 - 180.0)
        return abs(o.PosTiltSet - self.tilt) < 0.2 and drot < 0.2

    def permissive(self, o) -> bool:
        return o.ArcEnable and not o.RobotHold

    def _robot(self, dt: float, o) -> None:
        self.arc_on = False
        self.current = self.voltage = 0.0
        hold = o.RobotHold
        if self.robot in ("home", "aborted", "done") and o.RobotStart and not self.busy:
            if not self.in_position(o) and o.RobotJob != 900:
                self.violate("ROBOT_START_POSITIONER_NOT_READY")
            self.job = o.RobotJob
            self.busy, self.done, self.arcfault, self.home = True, False, False, False
            self.robot_t = 0.0
            if self.job == 900:
                self.robot = "clean"
            else:
                self.robot = "approach"
        if hold:
            return
        if self.robot == "clean":
            self.robot_t += dt
            if self.robot_t >= P.CLEAN_S:
                self.robot, self.busy, self.done, self.home = "done", False, True, True
        elif self.robot == "approach":
            self.robot_t += dt
            if self.robot_t >= P.APPROACH_S and o.ArcEnable:
                self.attempts[self.job] = self.attempts.get(self.job, 0) + 1
                fa, ff = self.active("arcstart"), self.active("arcfail")
                if ff or (fa and self.attempts[self.job] == 1):
                    self.robot, self.busy, self.arcfault = "aborted", False, True
                    return
                if self.progress == 0.0 and self.acc["t0"] is None:
                    self.acc["t0"] = self.t
                    self.acc["true_t0"] = self.th.temp(self.jobs[self.job].seam)
                self.robot = "arc"
        elif self.robot == "arc":
            pj = self.jobs[self.job]
            if not o.ArcEnable:
                self.robot, self.busy, self.arcfault = "aborted", False, True
                return
            if not (self.fume_t >= FUME_S and self.gas_t >= GAS_S and self.safety_ok()):
                self.violate("ARC_WITHOUT_PERMISSIVE")
            wf = self.active("wfs")
            factor = wf.factor if wf else 1.0
            self.arc_on = True
            self.current = pj.current_a * factor + self.rng.gauss(0, 3.0)
            self.voltage = pj.voltage_v + self.rng.gauss(0, 0.3)
            if self.acc["n"] == 0 and self.acc["true_t0"] is not None:
                if self.acc["true_t0"] > self.wps.interpass_max_c + 1.0:
                    self.violate("INTERPASS_EXCEEDED")
            self.acc["i"] += self.current
            self.acc["u"] += self.voltage
            self.acc["n"] += 1
            self.acc["arc"] += dt
            self.progress += pj.travel_mm_min / 60.0 * dt
            if self.progress >= pj.length_mm:
                self.acc["t1"] = self.t + dt
                self.robot, self.robot_t = "retract", 0.0
        elif self.robot == "retract":
            self.robot_t += dt
            if self.robot_t >= P.RETRACT_S:
                self.robot, self.busy, self.done, self.home = "done", False, True, True
                self.progress = 0.0

    def take_record_data(self) -> dict:
        a = self.acc
        self.acc = {"i": 0.0, "u": 0.0, "n": 0, "arc": 0.0, "t0": None, "t1": None, "true_t0": None}
        self.progress = 0.0
        return a


def run_cell(part: Part, wps: Wps, order: list[Pass], faults: list[str | CellFault] = (), serial: str = "PART-0001",
             seed: int = 0, dt: float = 0.5, t_max_h: float = 30.0, policy: str = "balanced",
             trace_every_s: float = 60.0) -> CellRun:
    rng = random.Random(f"{seed}-{serial}")
    cf = [parse_fault(f) if isinstance(f, str) else f for f in faults]
    if len(order) > 200:
        raise ValueError("at most 200 passes (PLC recipe arrays)")
    fb = FBInstance(fb_code())
    fb.PassCount = len(order)
    fb.PreheatMin = wps.preheat_min_c
    fb.InterpassMax = wps.interpass_max_c
    fb.aSeam = [p.seam_index for p in order]
    fb.aJob = [p.job for p in order]
    fb.aTilt = [part.seam(p.seam).orient[0] for p in order]
    fb.aRot = [part.seam(p.seam).orient[1] % 360.0 for p in order]
    fb.aIlo = [p.current_band_a[0] for p in order]
    fb.aIhi = [p.current_band_a[1] for p in order]
    plant = Plant(part, wps, order, cf, rng)
    records: list[WeldRecord] = []
    events: list[dict] = []
    trace: list[tuple[float, dict]] = []
    next_trace = 0.0
    operator_at = None
    reset_pulse = False
    started = False
    last_fault = 0
    quality_seen = False
    heater_s = wait_s = 0.0
    dt_ms = int(round(dt * 1000))
    while plant.t < t_max_h * 3600:
        # inputs
        fb.Start = not started
        fb.Reset = reset_pulse
        reset_pulse = False
        fb.SafetyOK = plant.safety_ok()
        fb.Clamped = plant.clamped
        fb.RobotHome = plant.home
        fb.RobotBusy = plant.busy
        fb.RobotDone = plant.done
        fb.ArcFault = plant.arcfault
        fb.PosInPosition = plant.in_position(fb)
        fb.FumeOK = plant.fume_t >= FUME_S
        fb.GasOK = plant.gas_t >= GAS_S
        fb.WireOK = True
        fb.WeldCurrent = plant.current
        fb.aSeamTemp = plant.temps_measured()
        fb.scan(dt_ms)
        started = True
        # record a finished pass
        if fb.PassDone and (not records or records[-1].seq != fb.PassIdx):
            p = order[fb.PassIdx - 1]
            a = plant.take_record_data()
            n = max(a["n"], 1)
            i_mean, u_mean = a["i"] / n, a["u"] / n
            hi = wps.heat_input_kj_mm_of(i_mean, u_mean, p.travel_mm_min)
            lo, hi_band = p.current_band_a
            if fb.PassInterrupted:
                res = "INTERRUPTED"
            elif (fb.PassNok or not (lo <= i_mean <= hi_band)
                  or not (wps.heat_input_kj_mm[0] <= hi <= wps.heat_input_kj_mm[1])):
                res = "NOK_PARAM"
            else:
                res = "OK"
            wf = plant.active("wfs")
            records.append(WeldRecord(serial, fb.PassIdx, p.label, p.seam, p.no, p.layer, p.bead, p.kind, p.job,
                                      round(i_mean, 1), round(u_mean, 2),
                                      round(p.wfs_m_min * (wf.factor if wf else 1.0), 2), p.travel_mm_min,
                                      round(a["arc"], 1), round(hi, 3), round(fb.TempAtStart, 1),
                                      round(a["true_t0"] if a["true_t0"] is not None else math.nan, 1),
                                      fb.PassRetries, res, round(a["t0"] or 0.0, 1), round(a["t1"] or plant.t, 1)))
        # fault / safety events and the operator
        if fb.FaultCode != last_fault:
            if fb.FaultCode:
                events.append({"t": round(plant.t, 1), "event": "FAULT", "code": fb.FaultCode,
                               "name": FAULT_NAMES[fb.FaultCode], "pass": fb.PassIdx})
            last_fault = fb.FaultCode
        if fb.QualityAlarm and not quality_seen:
            events.append({"t": round(plant.t, 1), "event": "QUALITY_ALARM", "pass": fb.PassIdx})
        quality_seen = fb.QualityAlarm
        needs_operator = fb.State == S_FAULT or (fb.SafetyStop and plant.safety_ok())
        if needs_operator and operator_at is None:
            operator_at = plant.t + OPERATOR_DELAY_S
            if fb.SafetyStop and fb.State != S_FAULT:
                events.append({"t": round(plant.t, 1), "event": "SAFETY_STOP", "pass": fb.PassIdx})
        if operator_at is not None and plant.t >= operator_at:
            cause = {1: "clamp", 2: "positioner", 3: "gas", 4: "fume", 5: "arcfail", 6: "pyro"}.get(fb.FaultCode)
            if cause:
                plant.clear(cause)
                if cause == "arcfail":
                    plant.clear("arcstart")
            reset_pulse = True
            events.append({"t": round(plant.t, 1), "event": "OPERATOR_RESET", "cleared": cause or "safety",
                           "pass": fb.PassIdx})
            operator_at = None
        heater_s += dt if fb.HeaterOn else 0.0
        wait_s += dt if fb.Waiting and fb.State == 40 else 0.0
        plant.step(dt, fb)
        if plant.t >= next_trace:
            trace.append((round(plant.t, 1), {s.id: round(plant.th.temp(s.id), 1) for s in part.seams}))
            next_trace += trace_every_s
        if fb.Done:
            break
    return CellRun(serial, part.name, wps.wps, policy, bool(fb.Done), round(plant.t, 1), records, events,
                   plant.violations, trace, round(heater_s, 1), round(wait_s, 1), [f.spec for f in cf])

"""Part, WPS, groove geometry and pass plan (F01-F04)."""
from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
DATA = ROOT / "data"


class DataError(ValueError):
    pass


@dataclass(frozen=True)
class PassType:
    name: str
    current_a: float
    voltage_v: float
    wfs_m_min: float
    travel_mm_min: float
    current_band_a: tuple[float, float]


@dataclass(frozen=True)
class Wps:
    wps: str
    process: str
    k: float
    wire_d_mm: float
    dep_eff: float
    reinforcement: float
    bead_width_mm: float
    heat_input_kj_mm: tuple[float, float]
    preheat_min_c: float
    interpass_max_c: float
    pass_types: dict[str, PassType]

    def deposit_mm2(self, pt: PassType) -> float:
        """Deposited cross-section per pass: wire volume per mm of travel."""
        return math.pi * self.wire_d_mm ** 2 / 4 * pt.wfs_m_min * 1000 / pt.travel_mm_min * self.dep_eff

    def heat_input_kj_mm_of(self, current_a: float, voltage_v: float, travel_mm_min: float) -> float:
        return self.k * voltage_v * current_a / (travel_mm_min / 60.0) / 1000.0


@dataclass(frozen=True)
class Seam:
    id: str
    index: int            # 1-based, used by the PLC temperature array
    joint: str            # "V" or "FILLET"
    t_mm: float
    length_mm: float
    side: tuple[float, float]
    orient: tuple[float, float]
    angle_deg: float = 60.0
    gap_mm: float = 0.0
    face_mm: float = 0.0
    leg_mm: float = 0.0


@dataclass(frozen=True)
class Part:
    name: str
    density: float
    c: float
    thermal: dict
    plates: list[dict]
    seams: list[Seam]

    @property
    def mass_kg(self) -> float:
        return sum(self.density * p["t_mm"] * p["length_mm"] * p["width_mm"] * 1e-9 for p in self.plates)

    def zone_mass_kg(self, s: Seam) -> float:
        return self.density * s.length_mm * self.thermal["zone_width_mm"] * s.t_mm * 1e-9

    def seam(self, sid: str) -> Seam:
        for s in self.seams:
            if s.id == sid:
                return s
        raise KeyError(sid)


@dataclass(frozen=True)
class Pass:
    seam: str
    seam_index: int
    no: int               # pass number within the seam, 1-based
    layer: int
    bead: int
    kind: str             # root / fill / cap
    current_a: float
    voltage_v: float
    wfs_m_min: float
    travel_mm_min: float
    current_band_a: tuple[float, float]
    length_mm: float
    heat_input_kj_mm: float
    deposit_mm2: float

    @property
    def job(self) -> int:
        """Robot job number: seam index * 100 + pass number."""
        return self.seam_index * 100 + self.no

    @property
    def arc_time_s(self) -> float:
        return self.length_mm / (self.travel_mm_min / 60.0)

    @property
    def energy_j(self) -> float:
        return self.heat_input_kj_mm * 1000.0 * self.length_mm

    @property
    def label(self) -> str:
        return f"{self.seam}.{self.no:02d}"


def load_wps(name: str = "mag_135", data: Path = DATA) -> Wps:
    d = yaml.safe_load((data / "wps" / f"{name}.yaml").read_text(encoding="utf-8"))
    pts = {k: PassType(k, v["current_a"], v["voltage_v"], v["wfs_m_min"], v["travel_mm_min"],
                       tuple(v["current_band_a"])) for k, v in d["pass_types"].items()}
    w = Wps(d["wps"], d["process"], d["k"], d["wire_d_mm"], d["dep_eff"], d["reinforcement"], d["bead_width_mm"],
            tuple(d["heat_input_kj_mm"]), float(d["preheat_min_c"]), float(d["interpass_max_c"]), pts)
    errors = []
    if set(pts) != {"root", "fill", "cap"}:
        errors.append("pass_types must be root, fill and cap")
    for pt in pts.values():
        lo, hi = pt.current_band_a
        if not lo < pt.current_a < hi:
            errors.append(f"{pt.name}: nominal current outside its band")
        hi_kj = w.heat_input_kj_mm_of(pt.current_a, pt.voltage_v, pt.travel_mm_min)
        if not w.heat_input_kj_mm[0] <= hi_kj <= w.heat_input_kj_mm[1]:
            errors.append(f"{pt.name}: heat input {hi_kj:.2f} kJ/mm outside the WPS window")
    if not w.preheat_min_c < w.interpass_max_c:
        errors.append("preheat_min_c must be below interpass_max_c")
    if errors:
        raise DataError("; ".join(errors))
    return w


def load_part(name: str = "box_section", data: Path = DATA) -> Part:
    d = yaml.safe_load((data / "parts" / f"{name}.yaml").read_text(encoding="utf-8"))
    seams = []
    errors = []
    for i, s in enumerate(d["seams"], start=1):
        if s["joint"] not in ("V", "FILLET"):
            errors.append(f"{s['id']}: joint must be V or FILLET")
            continue
        if s["joint"] == "V" and not (0 < s["angle_deg"] < 120 and s["face_mm"] < s["t_mm"]):
            errors.append(f"{s['id']}: bad V geometry")
        if s["joint"] == "FILLET" and s.get("leg_mm", 0) <= 0:
            errors.append(f"{s['id']}: fillet needs leg_mm > 0")
        seams.append(Seam(s["id"], i, s["joint"], float(s["t_mm"]), float(s["length_mm"]),
                          tuple(float(x) for x in s["side"]), tuple(float(x) for x in s["orient"]),
                          float(s.get("angle_deg", 60)), float(s.get("gap_mm", 0)), float(s.get("face_mm", 0)),
                          float(s.get("leg_mm", 0))))
    ids = [s.id for s in seams]
    if len(set(ids)) != len(ids):
        errors.append("duplicate seam ids")
    if len(seams) > 16:
        errors.append("at most 16 seams (PLC temperature array)")
    if errors:
        raise DataError("; ".join(errors))
    m = d["material"]
    return Part(d["part"], float(m["density_kg_m3"]), float(m["c_j_kgk"]), d["thermal"], d["plates"], seams)


def groove_area_mm2(s: Seam, reinforcement: float = 1.0) -> float:
    """Cross-section to fill (F02). Single-V: (t-c)^2 tan(a/2) + g t. Fillet: z^2 / 2."""
    if s.joint == "V":
        depth = s.t_mm - s.face_mm
        a = depth ** 2 * math.tan(math.radians(s.angle_deg / 2)) + s.gap_mm * s.t_mm
    else:
        a = s.leg_mm ** 2 / 2
    return a * reinforcement


def groove_width_mm(s: Seam, y: float) -> float:
    """Width of a V groove at height y above the root."""
    return s.gap_mm + 2 * max(0.0, y - s.face_mm) * math.tan(math.radians(s.angle_deg / 2))


def plan_seam(s: Seam, w: Wps) -> list[Pass]:
    """Root, fill and cap passes with layer / bead numbers (F03)."""
    total = groove_area_mm2(s, w.reinforcement)
    a_root = w.deposit_mm2(w.pass_types["root"])
    a_fill = w.deposit_mm2(w.pass_types["fill"])
    layers: list[int] = []        # beads per layer
    if s.joint == "V":
        n_rest = max(0, math.ceil((total - a_root) / a_fill))
        layers.append(1)
        h = a_fill / w.bead_width_mm
        y = s.face_mm + h / 2
        placed = 0
        while placed < n_rest:
            beads = max(1, round(groove_width_mm(s, y + h / 2) / w.bead_width_mm))
            beads = min(beads, n_rest - placed)
            layers.append(beads)
            placed += beads
            y += h
    else:
        n = max(1, math.ceil(total / a_fill))
        k = 1
        while n > 0:
            layers.append(min(k, n))
            n -= min(k, n)
            k += 1
    passes = []
    no = 0
    for li, beads in enumerate(layers, start=1):
        for b in range(1, beads + 1):
            no += 1
            if s.joint == "V" and li == 1:
                kind = "root"
            elif li == len(layers) and len(layers) > 1:
                kind = "cap"
            else:
                kind = "fill"
            pt = w.pass_types[kind]
            hi = w.heat_input_kj_mm_of(pt.current_a, pt.voltage_v, pt.travel_mm_min)
            passes.append(Pass(s.id, s.index, no, li, b, kind, pt.current_a, pt.voltage_v, pt.wfs_m_min,
                               pt.travel_mm_min, pt.current_band_a, s.length_mm, round(hi, 4),
                               round(w.deposit_mm2(pt), 2)))
    return passes


def plan_part(p: Part, w: Wps) -> dict[str, list[Pass]]:
    return {s.id: plan_seam(s, w) for s in p.seams}

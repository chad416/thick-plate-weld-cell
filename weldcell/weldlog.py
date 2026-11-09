"""Weld records and per-part outputs (F13, F14): JSON, CSV and a 2-page PDF report."""
from __future__ import annotations

import csv
import json
from dataclasses import asdict
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.backends.backend_pdf import PdfPages  # noqa: E402
from matplotlib.patches import Rectangle  # noqa: E402

from . import __version__  # noqa: E402
from .cell import CellRun  # noqa: E402
from .part import Part, Wps  # noqa: E402

A4 = (8.27, 11.69)
SEAM_COLOURS = ["#1e88e5", "#43a047", "#fb8c00", "#8e24aa", "#00acc1", "#7cb342", "#f4511e", "#5e35b1",
                "#3949ab", "#c0ca33", "#6d4c41", "#d81b60", "#00897b", "#fdd835", "#546e7a", "#e53935"]
RES_COL = {"OK": "#2e7d32", "NOK_PARAM": "#c62828", "INTERRUPTED": "#ef6c00"}


def verdict(run: CellRun) -> str:
    if not run.done or run.violations:
        return "INCOMPLETE" if not run.done else "VIOLATION"
    return "OK" if run.counts["NOK_PARAM"] == 0 and run.counts["INTERRUPTED"] == 0 else "REVIEW"


def to_json(run: CellRun, part: Part, wps: Wps) -> dict:
    return {
        "serial": run.serial, "part": part.name, "wps": wps.wps, "policy": run.policy, "tool_version": __version__,
        "verdict": verdict(run), "done": run.done, "cycle_s": run.t_total_s,
        "arc_s": round(sum(r.arc_time_s for r in run.records), 1), "heater_s": run.heater_s,
        "interpass_wait_s": run.wait_s, "counts": run.counts, "events": run.events, "violations": run.violations,
        "simulation": {"injected_faults": run.faults},
        "records": [asdict(r) for r in run.records],
    }


def save(run: CellRun, part: Part, wps: Wps, out: Path, pdf: bool = True, png_dir: Path | None = None) -> dict:
    out.mkdir(parents=True, exist_ok=True)
    paths = {}
    j = out / f"{run.serial}.json"
    j.write_text(json.dumps(to_json(run, part, wps), indent=1), encoding="utf-8")
    paths["json"] = j
    c = out / f"{run.serial}_welds.csv"
    with open(c, "w", newline="", encoding="utf-8") as fh:
        if run.records:
            w = csv.DictWriter(fh, fieldnames=list(asdict(run.records[0])))
            w.writeheader()
            for r in run.records:
                w.writerow(asdict(r))
    paths["csv"] = c
    if pdf:
        paths["pdf"] = write_pdf(run, part, wps, out / f"{run.serial}.pdf", png_dir)
    return paths


def _page1(run: CellRun, part: Part, wps: Wps):
    fig = plt.figure(figsize=A4)
    fig.text(0.07, 0.955, "Weld record: robotic multi-pass cell", fontsize=15, fontweight="bold")
    fig.text(0.07, 0.935, "Simulated cell · thick-plate-weld-cell", fontsize=8, color="#555")
    v = verdict(run)
    col = {"OK": "#2e7d32", "REVIEW": "#ef6c00"}.get(v, "#c62828")
    fig.patches.append(Rectangle((0.70, 0.925), 0.23, 0.045, transform=fig.transFigure, facecolor=col))
    fig.text(0.815, 0.947, v, fontsize=15, fontweight="bold", color="white", ha="center", va="center")
    arc = sum(r.arc_time_s for r in run.records)
    meta = [("Serial", run.serial), ("Part", part.name), ("WPS", f"{wps.wps} ({wps.process})"),
            ("Sequence", run.policy), ("Tool", __version__),
            ("Cycle time", f"{run.t_total_s / 3600:.2f} h  (arc on {arc / max(run.t_total_s, 1) * 100:.0f} %)"),
            ("Preheat / interpass wait", f"{run.heater_s / 60:.0f} min / {run.wait_s / 60:.0f} min"),
            ("Passes", f"{len(run.records)}  OK {run.counts['OK']}, NOK {run.counts['NOK_PARAM']}, "
                       f"interrupted {run.counts['INTERRUPTED']}"),
            ("Interlock violations (plant)", str(len(run.violations)))]
    for i, (k, val) in enumerate(meta):
        fig.text(0.07, 0.90 - i * 0.017, k, fontsize=8, color="#555")
        fig.text(0.33, 0.90 - i * 0.017, val, fontsize=8)

    fig.text(0.07, 0.725, "Per seam", fontsize=10, fontweight="bold")
    ax = fig.add_axes([0.07, 0.52, 0.86, 0.19])
    ax.axis("off")
    rows = []
    for s in part.seams:
        rs = [r for r in run.records if r.seam == s.id]
        if not rs:
            continue
        his = [r.heat_input_kj_mm for r in rs]
        rows.append([s.id, s.joint, f"{s.t_mm:.0f}", f"{s.length_mm:.0f}", str(len(rs)), str(max(r.layer for r in rs)),
                     f"{min(his):.2f}-{max(his):.2f}", f"{min(r.temp_start_c for r in rs):.0f}-"
                     f"{max(r.temp_start_c for r in rs):.0f}", str(sum(r.result != 'OK' for r in rs))])
    t = ax.table(cellText=rows, colLabels=["Seam", "Joint", "t mm", "L mm", "Passes", "Layers", "HI kJ/mm",
                                           "T start °C", "Not OK"], loc="upper left", cellLoc="center")
    t.auto_set_font_size(False)
    t.set_fontsize(7.5)
    t.scale(1, 1.3)
    for j in range(9):
        t[0, j].set_facecolor("#eceff1")
    fig.text(0.07, 0.495, f"WPS window: heat input {wps.heat_input_kj_mm[0]}-{wps.heat_input_kj_mm[1]} kJ/mm, "
             f"preheat ≥ {wps.preheat_min_c:.0f} °C, interpass ≤ {wps.interpass_max_c:.0f} °C (PLC keeps a 6 K "
             "pyrometer margin)", fontsize=7, color="#555")

    fig.text(0.07, 0.465, "Events and passes needing review", fontsize=10, fontweight="bold")
    lines = [f"{e['t'] / 60:7.1f} min  {e['event']:<15} {e.get('name') or e.get('cleared') or ''}  pass {e['pass']}"
             for e in run.events]
    for r in run.records:
        if r.result != "OK":
            lines.append(f"   pass {r.seq:3d}  {r.pass_label:<7} {r.result:<12} I {r.current_a:.0f} A  "
                         f"HI {r.heat_input_kj_mm:.2f} kJ/mm")
    for v_ in run.violations:
        lines.append(f"   VIOLATION {v_['what']} at {v_['t'] / 60:.1f} min, pass {v_['pass']}")
    if not lines:
        lines = ["none"]
    for i, ln in enumerate(lines[:34]):
        fig.text(0.08, 0.445 - i * 0.0125, ln, fontsize=7, family="monospace")
    if len(lines) > 34:
        fig.text(0.08, 0.445 - 34 * 0.0125, f"... {len(lines) - 34} more in the JSON record", fontsize=7)
    return fig


def _page2(run: CellRun, part: Part, wps: Wps):
    fig = plt.figure(figsize=A4)
    fig.text(0.07, 0.96, f"{run.serial}: temperatures and pass sequence", fontsize=12, fontweight="bold")
    ax = fig.add_axes([0.10, 0.56, 0.84, 0.35])
    th = [t / 3600 for t, _ in run.temp_trace]
    for k, s in enumerate(part.seams):
        ax.plot(th, [d[s.id] for _, d in run.temp_trace], lw=0.9, color=SEAM_COLOURS[k], label=s.id)
    ax.axhline(wps.interpass_max_c, color="#c62828", ls="--", lw=1)
    ax.axhline(wps.preheat_min_c, color="#1565c0", ls="--", lw=1)
    ax.text(0, wps.interpass_max_c + 2, " interpass max", fontsize=7, color="#c62828")
    ax.text(0, wps.preheat_min_c + 2, " preheat min", fontsize=7, color="#1565c0")
    ax.set_ylabel("seam temperature °C (plant)", fontsize=8)
    ax.set_xlabel("time h", fontsize=8)
    ax.legend(fontsize=6, ncol=8, loc="lower right")
    ax.tick_params(labelsize=7)
    ax.set_title("Seam temperatures", fontsize=9, loc="left")

    ax = fig.add_axes([0.10, 0.08, 0.84, 0.40])
    ids = [s.id for s in part.seams]
    for r in run.records:
        y = ids.index(r.seam)
        ax.add_patch(Rectangle((r.t_arc_start_s / 3600, y - 0.35), (r.t_arc_end_s - r.t_arc_start_s) / 3600, 0.7,
                               facecolor=SEAM_COLOURS[y], edgecolor=RES_COL[r.result],
                               lw=0.4 if r.result == "OK" else 1.5))
    for e in run.events:
        if e["event"] in ("FAULT", "SAFETY_STOP"):
            ax.axvline(e["t"] / 3600, color="#c62828", lw=0.8, ls=":")
    ax.set_yticks(range(len(ids)))
    ax.set_yticklabels(ids, fontsize=7)
    ax.set_xlim(0, run.t_total_s / 3600)
    ax.set_ylim(-0.7, len(ids) - 0.3)
    ax.invert_yaxis()
    ax.set_xlabel("time h", fontsize=8)
    ax.tick_params(labelsize=7)
    ax.set_title("Arc-on per pass (red edge = not OK, dotted = fault / safety stop)", fontsize=9, loc="left")
    return fig


def write_pdf(run: CellRun, part: Part, wps: Wps, path: Path, png_dir: Path | None = None) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    f1, f2 = _page1(run, part, wps), _page2(run, part, wps)
    with PdfPages(path) as pdf:
        pdf.savefig(f1)
        pdf.savefig(f2)
    if png_dir is not None:
        png_dir.mkdir(parents=True, exist_ok=True)
        f1.savefig(png_dir / f"{run.serial}_p1.png", dpi=110)
        f2.savefig(png_dir / f"{run.serial}_p2.png", dpi=110)
    plt.close(f1)
    plt.close(f2)
    return path

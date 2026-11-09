"""Regenerate every image in docs/img from real runs (README screenshots and GIF)."""
from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path

os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.animation import FuncAnimation, PillowWriter  # noqa: E402
from matplotlib.patches import Circle, Polygon, Rectangle  # noqa: E402

from weldcell.cell import run_cell  # noqa: E402
from weldcell.part import groove_width_mm, load_part, load_wps, plan_seam  # noqa: E402
from weldcell.planner import plan_balanced, plan_naive  # noqa: E402
from weldcell.weldlog import SEAM_COLOURS, save  # noqa: E402

IMG = ROOT / "docs" / "img"
FAULTS = ["arcfail:12", "curtain:30", "wfs:80:1.2"]
KIND_COL = {"root": "#ef6c00", "fill": "#90a4ae", "cap": "#1e88e5"}


def terminal_png(args: list[str], path: Path) -> None:
    r = subprocess.run([sys.executable, "-m", "weldcell", *args], cwd=ROOT, capture_output=True, text=True,
                       env={**os.environ, "PYTHONIOENCODING": "utf-8"})
    out = r.stdout.replace(str(ROOT / "out") + os.sep, "out/").replace(str(ROOT / "out"), "out")
    lines = [f"$ python -m weldcell {' '.join(args)}"] + out.rstrip().splitlines()
    h = 0.22 * len(lines) + 0.5
    fig = plt.figure(figsize=(11, h), dpi=110)
    fig.patch.set_facecolor("#1e1e1e")
    for i, ln in enumerate(lines):
        col = "#9cdcfe" if i == 0 else "#f48771" if any(w in ln for w in ("FAULT", "NOK", "INTERRUPTED", "REVIEW",
                                                                            "SAFETY")) else \
            "#b5cea8" if ("OK" in ln or "RESET" in ln) else "#d4d4d4"
        fig.text(0.015, 1 - (i + 1) * 0.22 / h, ln, family="monospace", fontsize=8.5, color=col, va="top")
    fig.text(0.985, 0.99, "real output", ha="right", va="top", fontsize=7, color="#808080")
    fig.savefig(path, facecolor=fig.get_facecolor())
    plt.close(fig)


def cross_section(part, wps, path: Path) -> None:
    fig, axes = plt.subplots(1, 2, figsize=(10, 4.2), dpi=120, gridspec_kw={"width_ratios": [1.6, 1]})
    s = part.seam("L1")
    ax = axes[0]
    ps = plan_seam(s, wps)
    t = s.t_mm
    w_top = groove_width_mm(s, t)
    plate = "#cfd8dc"
    ax.add_patch(Polygon([(-45, 0), (-s.gap_mm / 2, 0), (-s.gap_mm / 2, s.face_mm), (-w_top / 2, t), (-45, t)],
                         facecolor=plate, edgecolor="#607d8b"))
    ax.add_patch(Polygon([(45, 0), (s.gap_mm / 2, 0), (s.gap_mm / 2, s.face_mm), (w_top / 2, t), (45, t)],
                         facecolor=plate, edgecolor="#607d8b"))
    layers: dict[int, list] = {}
    for p in ps:
        layers.setdefault(p.layer, []).append(p)
    n_layers = len(layers)
    h = (t + 2.5) / n_layers
    for li, beads in layers.items():
        y0, y1 = (li - 1) * h, li * h
        wl0, wl1 = groove_width_mm(s, max(y0, 0)), groove_width_mm(s, min(y1, t))
        for k, p in enumerate(beads):
            def x(frac, w):
                return -w / 2 + frac * w
            n = len(beads)
            poly = [(x(k / n, wl0), y0), (x((k + 1) / n, wl0), y0), (x((k + 1) / n, wl1), y1), (x(k / n, wl1), y1)]
            ax.add_patch(Polygon(poly, facecolor=KIND_COL[p.kind], edgecolor="white", lw=0.8, alpha=0.9))
            cx = sum(q[0] for q in poly) / 4
            ax.text(cx, (y0 + y1) / 2, str(p.no), ha="center", va="center", fontsize=6.5, color="white")
    ax.set_xlim(-45, 45)
    ax.set_ylim(-3, t + 6)
    ax.set_aspect("equal")
    ax.set_title(f"{s.id}: single-V {s.angle_deg:.0f}°, t {t:.0f} mm, gap {s.gap_mm:.0f}, face {s.face_mm:.0f} → "
                 f"{len(ps)} passes in {n_layers} layers", fontsize=9)
    ax.set_xlabel("mm", fontsize=8)
    ax.tick_params(labelsize=7)
    handles = [Rectangle((0, 0), 1, 1, facecolor=c) for c in KIND_COL.values()]
    ax.legend(handles, [f"{k} ({wps.pass_types[k].current_a:.0f} A, "
                        f"{wps.heat_input_kj_mm_of(wps.pass_types[k].current_a, wps.pass_types[k].voltage_v, wps.pass_types[k].travel_mm_min):.2f} kJ/mm)"  # noqa: E501
                        for k in KIND_COL], fontsize=7, loc="upper left")
    ax = axes[1]
    rows = []
    for sm in part.seams:
        pp = plan_seam(sm, wps)
        rows.append([sm.id, sm.joint, f"{sm.t_mm:.0f}", f"{sm.length_mm:.0f}", str(len(pp)),
                     f"{sum(p.arc_time_s for p in pp) / 60:.0f}"])
    ax.axis("off")
    tb = ax.table(cellText=rows, colLabels=["Seam", "Joint", "t", "L mm", "Passes", "Arc min"], loc="center",
                  cellLoc="center")
    tb.auto_set_font_size(False)
    tb.set_fontsize(8)
    tb.scale(1, 1.4)
    ax.set_title(f"{part.name}: {part.mass_kg:.0f} kg, {sum(len(plan_seam(x, wps)) for x in part.seams)} passes",
                 fontsize=9)
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def plan_compare(part, wps, path: Path) -> dict:
    n, b = plan_naive(part, wps), plan_balanced(part, wps)
    fig = plt.figure(figsize=(11, 7.2), dpi=110)
    ids = [s.id for s in part.seams]
    hmax = max(n.cycle_s, b.cycle_s) / 3600
    for row, pl in enumerate((n, b)):
        ax = fig.add_axes([0.07, 0.62 - row * 0.30, 0.62, 0.25])
        for st in pl.steps:
            y = ids.index(st.p.seam)
            ax.add_patch(Rectangle((st.t_arc_start / 3600, y - 0.38), (st.t_arc_end - st.t_arc_start) / 3600, 0.76,
                                   facecolor=SEAM_COLOURS[y], edgecolor="none"))
        ax.set_xlim(0, hmax)
        ax.set_ylim(-0.7, len(ids) - 0.3)
        ax.invert_yaxis()
        ax.set_yticks(range(len(ids)))
        ax.set_yticklabels(ids, fontsize=7)
        ax.tick_params(labelsize=7)
        s = pl.summary()
        ax.set_title(f"{pl.policy}: {s['cycle_h']} h, arc on {s['arc_on_share'] * 100:.0f} %, waiting {s['wait_h']} h, "
                     f"max imbalance {s['max_imbalance']}", fontsize=9, loc="left")
        if row == 1:
            ax.set_xlabel("time h", fontsize=8)
    ax = fig.add_axes([0.07, 0.06, 0.62, 0.17])
    for pl, col in ((n, "#c62828"), (b, "#2e7d32")):
        ax.step([t / 3600 for t, _ in pl.imbalance_trace], [v for _, v in pl.imbalance_trace], where="post",
                color=col, label=pl.policy, lw=1.2)
    ax.set_xlim(0, hmax)
    ax.set_ylabel("distortion\nimbalance", fontsize=8)
    ax.set_xlabel("time h", fontsize=8)
    ax.legend(fontsize=7)
    ax.tick_params(labelsize=7)
    ax = fig.add_axes([0.76, 0.10, 0.21, 0.78])
    ax.axis("off")
    sn, sb = n.summary(), b.summary()
    lines = [("", "naive", "balanced")] + [(k.replace("_", " "), str(sn[k]), str(sb[k])) for k in list(sn)[2:]]
    for i, (k, a_, b_) in enumerate(lines):
        ax.text(0.0, 1 - i * 0.07, k, fontsize=8, fontweight="bold" if i == 0 else None)
        ax.text(0.62, 1 - i * 0.07, a_, fontsize=8, ha="right", color="#c62828")
        ax.text(1.0, 1 - i * 0.07, b_, fontsize=8, ha="right", color="#2e7d32")
    fig.suptitle(f"Sequence planning, {part.name} ({len(b.steps)} passes): same welds, different order", fontsize=11)
    fig.savefig(path)
    plt.close(fig)
    return {"naive": sn, "balanced": sb}


def cell_gif(part, run, path: Path) -> None:
    pos = {"L1": (1, 1), "L2": (-1, 1), "L3": (-1, -1), "L4": (1, -1), "B1": (1.0, 0), "B2": (-1.0, 0),
           "B3": (0, 1.0), "B4": (0, -1.0)}
    totals = {s.id: sum(1 for r in run.records if r.seam == s.id) for s in part.seams}
    trace = run.temp_trace
    frames = list(range(0, len(trace), max(1, len(trace) // 60))) + [len(trace) - 1] * 10
    fig = plt.figure(figsize=(9, 4.4), dpi=72)
    ax = fig.add_axes([0.02, 0.05, 0.40, 0.85])
    ax2 = fig.add_axes([0.52, 0.14, 0.45, 0.72])
    cmap = plt.get_cmap("coolwarm")

    def frame(k):
        i = frames[k]
        t, temps = trace[i]
        ax.clear()
        ax.set_xlim(-1.6, 1.6)
        ax.set_ylim(-1.6, 1.6)
        ax.axis("off")
        ax.add_patch(Rectangle((-1, -1), 2, 2, facecolor="#eceff1", edgecolor="#607d8b", lw=2))
        ax.text(0, 0, "base", ha="center", va="center", fontsize=8, color="#90a4ae")
        for sid, (x, y) in pos.items():
            if sid not in temps:
                continue
            done = sum(1 for r in run.records if r.seam == sid and r.t_arc_end_s <= t)
            arc = any(r.seam == sid and r.t_arc_start_s <= t < r.t_arc_end_s for r in run.records)
            frac = (temps[sid] - 20) / 200
            r_ = 0.2 if sid.startswith("L") else 0.13
            sx, sy = (x * 0.62, y * 0.62) if sid.startswith("B") else (x, y)
            ax.add_patch(Circle((sx, sy), r_, facecolor=cmap(min(max(frac, 0), 1)),
                                edgecolor="#ffd600" if arc else "#37474f", lw=3 if arc else 1))
            ax.text(sx, sy + 0.02, sid, ha="center", va="center", fontsize=7, fontweight="bold")
            ax.text(sx, sy - r_ - 0.09, f"{done}/{totals[sid]}  {temps[sid]:.0f}°C", ha="center", fontsize=6.5)
        ax.set_title(f"{part.name}, balanced sequence   t = {t / 3600:4.2f} h", fontsize=9)
        ax2.clear()
        ts = [x / 3600 for x, _ in trace[:i + 1]]
        for j, s in enumerate(part.seams):
            ax2.plot(ts, [d[s.id] for _, d in trace[:i + 1]], lw=0.8, color=SEAM_COLOURS[j])
        ax2.axhline(200, color="#c62828", ls="--", lw=0.8)
        ax2.axhline(100, color="#1565c0", ls="--", lw=0.8)
        ax2.set_xlim(0, run.t_total_s / 3600)
        ax2.set_ylim(15, 215)
        ax2.set_xlabel("time h", fontsize=7)
        ax2.set_ylabel("seam temperature °C", fontsize=7)
        ax2.tick_params(labelsize=6)
        ax2.set_title("preheat 100 °C (blue) · interpass 200 °C (red)", fontsize=8)

    anim = FuncAnimation(fig, frame, frames=len(frames), interval=120)
    anim.save(path, writer=PillowWriter(fps=8))
    plt.close(fig)


def main() -> int:
    IMG.mkdir(parents=True, exist_ok=True)
    wps, box = load_wps(), load_part("box_section")
    cross_section(box, wps, IMG / "pass_plan.png")
    summary = plan_compare(box, wps, IMG / "plan_compare.png")
    order = plan_balanced(box, wps).order
    clean = run_cell(box, wps, order, serial="BOX-0101")
    save(clean, box, wps, ROOT / "out", png_dir=IMG)
    (IMG / "BOX-0101_p1.png").replace(IMG / "report_clean_p1.png")
    (IMG / "BOX-0101_p2.png").replace(IMG / "report_clean_p2.png")
    faulty = run_cell(box, wps, order, FAULTS, serial="BOX-0102")
    save(faulty, box, wps, ROOT / "out", png_dir=IMG)
    (IMG / "BOX-0102_p1.png").replace(IMG / "report_faults_p1.png")
    (IMG / "BOX-0102_p2.png").replace(IMG / "report_faults_p2.png")
    cell_gif(box, clean, IMG / "cell_run.gif")
    terminal_png(["compare"], IMG / "cli_compare.png")
    terminal_png(["run", "--serial", "BOX-0102", *sum([["--fault", f] for f in FAULTS], [])], IMG / "cli_faults.png")
    res = {"plans": summary, "clean_run": {"cycle_h": round(clean.t_total_s / 3600, 2), "counts": clean.counts,
                                           "violations": len(clean.violations)},
           "fault_run": {"cycle_h": round(faulty.t_total_s / 3600, 2), "counts": faulty.counts,
                         "events": faulty.events, "violations": len(faulty.violations)}}
    (ROOT / "docs" / "results.json").write_text(json.dumps(res, indent=1), encoding="utf-8")
    print(json.dumps(res, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())

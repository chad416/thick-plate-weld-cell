"""Command line: `python -m weldcell plan|compare|run|scl`."""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

from .cell import FAULT_KINDS, run_cell
from .part import load_part, load_wps, plan_part
from .planner import check_window, plan_balanced, plan_naive
from .scl import export
from .weldlog import save, verdict

FAULT_HELP = """cell faults (repeat --fault; <n> = pass number in the sequence):
  arcstart:<n>  arcfail:<n>  gas:<n>  fume:<n>  curtain:<n>  clamp  positioner:<n>  pyro:<n>  wfs:<n>:<factor>"""


def _plan(part, wps, policy):
    return plan_balanced(part, wps) if policy == "balanced" else plan_naive(part, wps)


def cmd_plan(a) -> int:
    part, wps = load_part(a.part), load_wps(a.wps)
    pp = plan_part(part, wps)
    print(f"{part.name}: {part.mass_kg:.0f} kg, {sum(len(v) for v in pp.values())} passes")
    for s in part.seams:
        ps = pp[s.id]
        kinds = "".join(p.kind[0] for p in ps)
        print(f"  {s.id}  {s.joint:<6} t {s.t_mm:4.0f}  L {s.length_mm:5.0f}  {len(ps):3d} passes  {kinds}")
    pl = _plan(part, wps, a.policy)
    head = " ".join(p.label for p in pl.order[:24])
    print(f"sequence ({a.policy}): {head}" + (" ..." if len(pl.order) > 24 else ""))
    print("  " + ", ".join(f"{k} {v}" for k, v in pl.summary().items()))
    return 0 if not check_window(pl, wps) else 1


def cmd_compare(a) -> int:
    part, wps = load_part(a.part), load_wps(a.wps)
    rows = [plan_naive(part, wps).summary(), plan_balanced(part, wps).summary()]
    keys = list(rows[0])
    print(f"{'':22}" + "".join(f"{r['policy']:>12}" for r in rows))
    for k in keys[1:]:
        print(f"{k:22}" + "".join(f"{r[k]!s:>12}" for r in rows))
    return 0


def cmd_run(a) -> int:
    part, wps = load_part(a.part), load_wps(a.wps)
    pl = _plan(part, wps, a.policy)
    run = run_cell(part, wps, pl.order, a.fault or [], a.serial, a.seed, policy=a.policy)
    out = Path(a.out)
    paths = save(run, part, wps, out, png_dir=out / "png" if a.png else None)
    print(f"{run.serial}  {part.name}  {a.policy}  ->  {verdict(run)}   cycle {run.t_total_s / 3600:.2f} h, "
          f"{len(run.records)} passes {run.counts}, violations {len(run.violations)}")
    for e in run.events:
        what = e.get("name") or e.get("cleared") or ""
        print(f"  {e['t'] / 60:7.1f} min  {e['event']:<15} {what}  (pass {e['pass']})")
    for r in run.records:
        if r.result != "OK":
            print(f"  pass {r.seq:3d} {r.pass_label:<7} {r.result:<12} I {r.current_a:.0f} A  "
                  f"HI {r.heat_input_kj_mm:.2f}")
    print("  " + "   ".join(f"{k}: {v}" for k, v in paths.items()))
    return 0 if verdict(run) == "OK" else 1


def cmd_scl(a) -> int:
    root = Path(__file__).resolve().parents[1]
    p = export(root / "plc" / "FB_WeldCell.st", Path(a.out))
    print(f"wrote {p}")
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="weldcell", description="Simulated robotic multi-pass weld cell.")
    sub = ap.add_subparsers(dest="cmd", required=True)
    for name, fn in (("plan", cmd_plan), ("compare", cmd_compare), ("run", cmd_run)):
        p = sub.add_parser(name, epilog=FAULT_HELP if name == "run" else None,
                           formatter_class=argparse.RawDescriptionHelpFormatter)
        p.add_argument("--part", default="box_section")
        p.add_argument("--wps", default="mag_135")
        p.set_defaults(fn=fn)
        if name in ("plan", "run"):
            p.add_argument("--policy", default="balanced", choices=["balanced", "naive"])
        if name == "run":
            p.add_argument("--serial", default="PART-0001")
            p.add_argument("--fault", action="append", help=f"inject a cell fault: {', '.join(FAULT_KINDS)}")
            p.add_argument("--seed", type=int, default=0)
            p.add_argument("--out", default="out")
            p.add_argument("--png", action="store_true")
    s = sub.add_parser("scl", help="export the PLC block as Siemens SCL")
    s.add_argument("--out", default="build/FB_WeldCell.scl")
    s.set_defaults(fn=cmd_scl)
    a = ap.parse_args(argv)
    return a.fn(a)


if __name__ == "__main__":
    sys.exit(main())

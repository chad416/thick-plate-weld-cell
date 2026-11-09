"""Integration tests IT01-IT05 (SPEC section 5)."""
import json
import random
import time

import pytest

from weldcell.cell import fb_code, run_cell
from weldcell.planner import plan_balanced, plan_naive
from weldcell.stlang import FBInstance
from weldcell.weldlog import save, verdict


@pytest.mark.it("IT01")
@pytest.mark.req("F09", "F13", "F16", "F17")
@pytest.mark.parametrize("which", ["coupon", "box"])
def test_it01_clean_run(request, wps, which):
    part = request.getfixturevalue(which)
    order = request.getfixturevalue(f"{which}_order")
    run = run_cell(part, wps, order, serial=f"IT01-{which}")
    assert run.done and verdict(run) == "OK"
    assert len(run.records) == len(order) and run.counts["OK"] == len(order)
    assert run.violations == [] and run.events == []
    assert [r.seq for r in run.records] == list(range(1, len(order) + 1))
    assert max(r.temp_start_true_c for r in run.records) <= wps.interpass_max_c
    for r in run.records:
        assert wps.heat_input_kj_mm[0] <= r.heat_input_kj_mm <= wps.heat_input_kj_mm[1]
    again = run_cell(part, wps, order, serial=f"IT01-{which}")
    key = [(r.current_a, r.t_arc_start_s) for r in run.records]
    assert key == [(r.current_a, r.t_arc_start_s) for r in again.records]


SCENARIOS = [
    # fault, expected fault name (or event), expected record results, retries on pass 2
    ("arcstart:2", None, {"OK": 5}, 1),
    ("arcfail:2", "ARC", {"OK": 5}, 2),
    ("gas:3", "GAS", {"OK": 5}, 0),
    ("fume:1", "FUME", {"OK": 5}, 0),
    ("curtain:2", "SAFETY_STOP", {"OK": 4, "INTERRUPTED": 1}, 0),
    ("clamp", "CLAMP", {"OK": 5}, 0),
    ("positioner:2", "POSITIONER", {"OK": 5}, 0),
    ("pyro:3", "SENSOR", {"OK": 5}, 0),
    ("wfs:3:1.25", "QUALITY_ALARM", {"OK": 2, "NOK_PARAM": 3}, 0),
]


@pytest.mark.it("IT02")
@pytest.mark.req("F10", "F11", "F12", "F13")
@pytest.mark.parametrize("fault,expect,results,retries", SCENARIOS, ids=[s[0] for s in SCENARIOS])
def test_it02_fault_scenarios(coupon, wps, coupon_order, fault, expect, results, retries):
    run = run_cell(coupon, wps, coupon_order, [fault], serial="IT02")
    assert run.done, run.events
    assert run.violations == []
    got = {k: v for k, v in run.counts.items() if v}
    assert got == results
    names = {e.get("name") or e["event"] for e in run.events}
    if expect:
        assert expect in names
    else:
        assert run.events == []
    assert run.records[1].retries == retries
    if expect and expect not in ("QUALITY_ALARM",):
        assert any(e["event"] == "OPERATOR_RESET" for e in run.events)


@pytest.mark.it("IT02")
@pytest.mark.req("F12")
def test_it02_sensor_no_rise_after_pass(box, wps, box_order):
    # pass 5 is a base fillet: the frozen reading is inside the window, so the pass runs and the
    # post-pass plausibility check (no temperature rise) trips, not the cooling timeout
    run = run_cell(box, wps, box_order, ["pyro:5"], serial="IT02-pyro")
    fault = [e for e in run.events if e["event"] == "FAULT"][0]
    assert fault["name"] == "SENSOR" and fault["pass"] == 5
    arc_end = run.records[4].t_arc_end_s
    assert fault["t"] - arc_end < 60
    assert run.done and run.counts["OK"] == len(box_order)


@pytest.mark.it("IT03")
@pytest.mark.req("F10", "F11")
@pytest.mark.parametrize("seed", range(5))
def test_it03_interlocks_hold_under_random_inputs(seed):
    """Random inputs for 4000 scans: the permissives must hold in every single scan."""
    rng = random.Random(seed)
    fb = FBInstance(fb_code())
    fb.PassCount = 6
    fb.aSeam = [1, 2, 1, 2, 1, 2]
    fb.aJob = [101, 201, 102, 202, 103, 203]
    fb.aTilt = [0, 45, 0, 45, 0, 45]
    fb.aRot = [0, 90, 0, 90, 0, 90]
    fb.aIlo = [100.0] * 6
    fb.aIhi = [300.0] * 6
    fb.PreheatMin, fb.InterpassMax = 100.0, 200.0
    states = set()
    for _ in range(4000):
        for name in ("Start", "Reset", "Clamped", "RobotHome", "RobotBusy", "RobotDone", "ArcFault", "PosInPosition",
                     "FumeOK", "GasOK", "WireOK"):
            setattr(fb, name, rng.random() < 0.6)
        fb.SafetyOK = rng.random() < 0.9
        fb.aSeamTemp = [rng.uniform(80, 220), rng.uniform(80, 220)]
        fb.WeldCurrent = rng.uniform(0, 350)
        fb.scan(500)
        states.add(fb.State)
        if fb.ArcEnable:
            assert fb.SafetyOK and fb.FumeOK and fb.GasOK and fb.WireOK and fb.FaultCode == 0
            assert fb.State in (60, 61)
        if fb.PosStart:
            assert fb.RobotHome
        if not fb.SafetyOK:
            assert fb.RobotHold and not fb.ArcEnable
        if fb.RobotStart:
            assert fb.State in (60, 80)
    assert len(states) >= 8                                         # the fuzz reaches deep into the sequence


@pytest.mark.it("IT04")
@pytest.mark.req("F06")
def test_it04_balanced_beats_naive(box, wps):
    n, b = plan_naive(box, wps), plan_balanced(box, wps)
    assert b.cycle_s < 0.8 * n.cycle_s
    assert b.max_imbalance < 0.2 * n.max_imbalance
    assert b.wait_s < n.wait_s
    run = run_cell(box, wps, b.order, serial="IT04")
    assert abs(run.t_total_s - b.cycle_s) / b.cycle_s < 0.03                # plan and cell agree within 3 %


@pytest.mark.it("IT05")
@pytest.mark.req("F14", "P01")
def test_it05_outputs_and_timing(box, wps, box_order, tmp_path):
    t0 = time.perf_counter()
    run = run_cell(box, wps, box_order, ["curtain:10", "wfs:40:1.2"], serial="IT05")
    assert time.perf_counter() - t0 < 60
    paths = save(run, box, wps, tmp_path, png_dir=tmp_path)
    data = json.loads(paths["json"].read_text())
    assert data["verdict"] == "REVIEW" and len(data["records"]) == len(box_order)
    assert paths["pdf"].read_bytes()[:5] == b"%PDF-" and (tmp_path / "IT05_p2.png").exists()
    rows = paths["csv"].read_text().splitlines()
    assert len(rows) == len(box_order) + 1 and rows[0].startswith("serial,seq,pass_label")

import math

import pytest

from weldcell.part import plan_part
from weldcell.planner import MEAS_MARGIN_C, check_window, plan_balanced, plan_naive, pos_time
from weldcell.thermal import Thermal


@pytest.mark.req("F05")
def test_thermal_heats_and_cools(box):
    th = Thermal(box)
    t0 = th.temp("L1")
    th.step(300, "L1", 6000.0)
    assert th.temp("L1") > th.temp("L2") > t0                  # local excess on top of a warmer bulk
    hot = th.excess["L1"]
    th.step(th.tau_local)
    assert th.excess["L1"] == pytest.approx(hot * math.exp(-1), rel=1e-9)


@pytest.mark.req("F05")
def test_thermal_energy_into_bulk(box):
    th = Thermal(box)
    b0 = th.bulk
    th.step(1.0, None, 0.0, heater=True)
    expected = (b0 + th.heater_w / th.mc - th.amb) * math.exp(-1 / th.tau_bulk) + th.amb
    assert th.bulk == pytest.approx(expected)


@pytest.mark.req("F06")
def test_positioner_time_wraps():
    assert pos_time((0, 315), (0, 45)) == pytest.approx(90 / 6)
    assert pos_time((0, 0), (90, 0)) == pytest.approx(15)


@pytest.mark.req("F06", "F07")
@pytest.mark.parametrize("fn", [plan_naive, plan_balanced])
@pytest.mark.parametrize("part_name", ["box", "coupon"])
def test_plans_complete_and_in_window(request, wps, fn, part_name):
    part = request.getfixturevalue(part_name)
    pl = fn(part, wps)
    want = sorted(p.label for v in plan_part(part, wps).values() for p in v)
    assert sorted(p.label for p in pl.order) == want
    assert check_window(pl, wps) == []
    assert max(s.temp_start for s in pl.steps) <= wps.interpass_max_c - MEAS_MARGIN_C + 1e-6
    for seam in part.seams:                                       # pass order within a seam is kept
        nos = [p.no for p in pl.order if p.seam == seam.id]
        assert nos == sorted(nos)


@pytest.mark.req("F06")
def test_naive_waits_on_hot_seam(box, wps):
    pl = plan_naive(box, wps)
    assert pl.wait_s > 3600                                       # hours spent waiting for interpass
    assert pl.pos_moves == len(box.seams)                         # but only one positioner move per seam


@pytest.mark.req("F17")
def test_deterministic(box, wps):
    a, b = plan_balanced(box, wps), plan_balanced(box, wps)
    assert [p.label for p in a.order] == [p.label for p in b.order] and a.cycle_s == b.cycle_s

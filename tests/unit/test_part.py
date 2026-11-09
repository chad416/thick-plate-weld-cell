import math
import shutil

import pytest

from weldcell.part import DATA, DataError, Seam, groove_area_mm2, load_part, load_wps, plan_part, plan_seam


@pytest.mark.req("F01")
def test_load(box, coupon, wps):
    assert len(box.seams) == 8 and len(coupon.seams) == 2
    assert box.mass_kg == pytest.approx(1300, rel=0.01)
    assert wps.pass_types["fill"].current_band_a == (255, 305)


@pytest.mark.req("F01")
def test_rejects_bad_data(tmp_path):
    d = tmp_path / "data"
    shutil.copytree(DATA, d)
    p = d / "parts" / "coupon.yaml"
    p.write_text(p.read_text().replace("joint: FILLET", "joint: PLUG"))
    with pytest.raises(DataError, match="joint must be V or FILLET"):
        load_part("coupon", d)
    w = d / "wps" / "mag_135.yaml"
    w.write_text(w.read_text().replace("current_band_a: [255, 305]", "current_band_a: [290, 305]"))
    with pytest.raises(DataError, match="fill: nominal current outside its band"):
        load_wps("mag_135", d)


@pytest.mark.req("F02")
def test_groove_areas():
    v = Seam("V", 1, "V", 30, 1000, (1, 0), (0, 0), angle_deg=60, gap_mm=2, face_mm=2)
    assert groove_area_mm2(v) == pytest.approx(28 ** 2 * math.tan(math.radians(30)) + 2 * 30)
    f = Seam("F", 2, "FILLET", 20, 1000, (1, 0), (0, 0), leg_mm=10)
    assert groove_area_mm2(f, 1.1) == pytest.approx(55.0)


@pytest.mark.req("F03")
def test_pass_plan_v(box, wps):
    ps = plan_seam(box.seam("L1"), wps)
    assert ps[0].kind == "root" and ps[0].layer == 1
    assert ps[-1].kind == "cap" and all(p.kind == "fill" for p in ps[1:-2])
    assert [p.no for p in ps] == list(range(1, len(ps) + 1))
    deposited = sum(p.deposit_mm2 for p in ps)
    assert deposited >= groove_area_mm2(box.seam("L1"), wps.reinforcement) * 0.95
    layers = [p.layer for p in ps]
    assert layers == sorted(layers)                                    # layers fill from the root up


@pytest.mark.req("F03")
def test_pass_plan_fillet_layers(box, wps):
    ps = plan_seam(box.seam("B1"), wps)
    assert [(p.layer, p.bead, p.kind) for p in ps] == [(1, 1, "fill"), (2, 1, "cap")]
    assert len({p.job for v in plan_part(box, wps).values() for p in v}) == 88   # unique robot jobs


@pytest.mark.req("F04")
def test_heat_input(wps):
    pt = wps.pass_types["fill"]
    q = wps.heat_input_kj_mm_of(pt.current_a, pt.voltage_v, pt.travel_mm_min)
    assert q == pytest.approx(0.8 * 29 * 280 / (380 / 60) / 1000)
    for p in plan_part(load_part("box_section"), wps)["L2"]:
        assert wps.heat_input_kj_mm[0] <= p.heat_input_kj_mm <= wps.heat_input_kj_mm[1]

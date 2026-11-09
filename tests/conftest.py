import os
import sys
from pathlib import Path

os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import pytest  # noqa: E402

from weldcell.part import load_part, load_wps  # noqa: E402
from weldcell.planner import plan_balanced  # noqa: E402


def pytest_configure(config):
    config.addinivalue_line("markers", "req(*ids): SPEC requirement ids this test verifies")
    config.addinivalue_line("markers", "it(id): SPEC integration test id")


@pytest.fixture(scope="session")
def wps():
    return load_wps()


@pytest.fixture(scope="session")
def box():
    return load_part("box_section")


@pytest.fixture(scope="session")
def coupon():
    return load_part("coupon")


@pytest.fixture(scope="session")
def coupon_order(coupon, wps):
    return plan_balanced(coupon, wps).order


@pytest.fixture(scope="session")
def box_order(box, wps):
    return plan_balanced(box, wps).order

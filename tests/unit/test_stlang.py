import pytest

from weldcell.scl import to_scl
from weldcell.stlang import FBInstance, STError, compile_st, time_ms

pytestmark = pytest.mark.req("F08")

SRC = """
FUNCTION_BLOCK FB_Test
VAR_INPUT
    Go : BOOL;
    X : REAL;
    N : INT;
END_VAR
VAR_OUTPUT
    Y : REAL;
    K : INT;
    Fired : BOOL;
    Q : BOOL;
    Code : INT;
END_VAR
VAR_IN_OUT
    Arr : ARRAY[1..4] OF INT;
END_VAR
VAR
    t1 : TON;
    r1 : R_TRIG;
END_VAR
VAR CONSTANT
    A : INT := 1;
    B : INT := 2;
    LIM : REAL := 2.5;
    PT1 : TIME := T#1S500MS;
END_VAR
(* comment with Y := 99; inside *)
t1(IN := Go, PT := PT1);
Q := t1.Q;
r1(CLK := Go);
IF r1.Q THEN Fired := TRUE; END_IF;
Y := ABS(X) * 2.0 - MIN(X, LIM);   // trailing comment
K := N MOD 3 + 7 / 2;
CASE N OF
    A, 5: Code := 10;
    B: Code := 20;
         IF X > 0.0 THEN Code := 21; ELSIF X < 0.0 THEN Code := 22; ELSE Code := 23; END_IF;
ELSE
    Code := Arr[N];
END_CASE;
Arr[1] := Arr[1] + 1;
END_FUNCTION_BLOCK
"""


def fb():
    return FBInstance(compile_st(SRC))


def test_time_literals():
    assert time_ms("T#1S500MS") == 1500 and time_ms("T#2H") == 7_200_000 and time_ms("t#1m30s") == 90_000


def test_expressions_and_case():
    f = fb()
    f.X, f.N = -3.0, 2
    f.scan(100)
    assert f.Y == pytest.approx(3.0 * 2.0 - (-3.0))
    assert f.K == 2 % 3 + 3                                        # 7 / 2 is integer division in ST
    assert f.Code == 22
    f.N = 5
    f.scan(100)
    assert f.Code == 10
    f.N = 3
    f.Arr = [0, 0, 7, 0]
    f.scan(100)
    assert f.Code == 7 and f.Arr[0] == 1


def test_ton_and_rtrig():
    f = fb()
    f.Go, f.N = True, 1
    hits = []
    for _ in range(20):
        f.scan(100)
        hits.append(f.Q)
    assert hits.index(True) == 15                                   # 1.5 s at 100 ms scans, counted from 0
    assert f.Fired
    f.Fired = False
    f.scan(100)
    assert not f.Fired                                              # R_TRIG fires once per edge
    f.Go = False
    f.scan(100)
    assert not f.Q and f.t1.ET == 0


def test_array_bounds_checked():
    f = fb()
    f.N = 9
    with pytest.raises(IndexError):
        f.scan(100)


@pytest.mark.parametrize("bad,msg", [
    ("Y := Missing;", "unknown identifier"),
    ("A := 3;", "cannot assign constant"),
    ("t1(IN := Go);", "TON needs IN and PT"),
    ("IF Go THEN Y := 1.0; END_CASE;", "unexpected keyword END_CASE"),
])
def test_compile_errors(bad, msg):
    src = SRC.replace("Arr[1] := Arr[1] + 1;", bad)
    with pytest.raises(STError, match=msg):
        compile_st(src)


def test_unknown_variable_on_instance():
    with pytest.raises(AttributeError):
        fb().Nope = 1


@pytest.mark.req("F15")
def test_scl_export():
    s = to_scl(SRC)
    assert s.startswith('FUNCTION_BLOCK "FB_Test"')
    assert "t1 : TON_TIME;" in s and "Arr : Array[1..4] of Int;" in s and "PT1 : Time := T#1500MS;" in s
    assert "#t1(IN := #Go, PT := #PT1);" in s and "#Q := #t1.Q;" in s
    assert "(* comment with Y := 99; inside *)" in s                 # comments untouched
    body = s.split("BEGIN", 1)[1]
    assert "#Arr[#N]" in body and "#r1.Q" in body


@pytest.mark.req("F15")
def test_committed_scl_is_current():
    from pathlib import Path

    root = Path(__file__).resolve().parents[2]
    st = (root / "plc" / "FB_WeldCell.st").read_text(encoding="utf-8")
    assert (root / "plc" / "FB_WeldCell.scl").read_text(encoding="utf-8") == to_scl(st)

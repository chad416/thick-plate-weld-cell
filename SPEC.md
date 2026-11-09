# SPEC: thick-plate-weld-cell

Rev B, 9 Nov 2026. (Rev A: first draft. Rev B: added the pyrometer margin and the temperature-step timeout, both found in fault testing.)

## 1. Purpose

A robotic multi-pass welding cell for a heavy thick-plate weldment, built in simulation, end to end:
1. **Pass planning:** from the joint geometry and the welding procedure (WPS window), work out how many passes each seam needs and what heat input each pass gets.
2. **Sequence planning:** order every pass of every seam so that each starts inside its preheat/interpass temperature window and the angular distortion stays balanced. The sequence should also keep the cycle time and the positioner moves low.
3. **Cell control:** a PLC function block written in IEC 61131-3 Structured Text. It runs the planned sequence with these interlocks: safety, fume extraction, gas, positioner/robot, interpass temperature, arc-fault retry and sensor plausibility. A robot / positioner / power-source / thermal model drives it.
4. **Traceability:** one weld record per pass (parameters, heat input, interpass temperature, result), plus a per-part report.

The PLC source in `plc/FB_WeldCell.st` is the code that is tested. A small ST-to-Python translator runs it in CI. The same source is exported as Siemens SCL for TIA Portal.

## 2. Requirements

| ID | Requirement |
|---|---|
| F01 | Load the part (plates, seams, joint type, thickness, length, side, orientation) and the WPS window from YAML. Reject inconsistent data. |
| F02 | Groove cross-section for single-V butt (angle, root gap, root face) and fillet (leg) joints. |
| F03 | Pass plan per seam: root, fill and cap passes from the deposited area per pass (wire diameter, wire feed, travel speed, deposition efficiency). Each pass gets its own layer and bead number. |
| F04 | Heat input per pass, Q = k·U·I/v (k = 0.8 for MAG), checked against the WPS window. |
| F05 | Thermal model: a local seam-zone temperature over a part bulk temperature, heated by the arc and cooling exponentially. It gives the predicted temperature at the start of every pass. |
| F06 | Sequence planners: (a) naive, seam by seam, waiting on the same seam; (b) balanced: choose the next ready pass by distortion balance and positioner moves. The output includes cycle time, arc-on share, waiting time, positioner moves and the maximum distortion imbalance. |
| F07 | In every planned sequence, no pass starts above `interpass_max_c` or below `preheat_min_c` (predicted). The planner keeps the same 6 K measurement margin as the PLC. |
| F08 | ST translator: runs the ST subset used by the FB (VAR sections, BOOL/INT/DINT/REAL/TIME, ARRAY, TON, R_TRIG, IF/ELSIF/ELSE, CASE, FB calls, arithmetic/logic/comparison, ABS/MIN/MAX) deterministically, one call per scan. |
| F09 | Cell sequence: clamp → per pass {position → temperature check (wait/preheat) → pre-arc checks → weld with robot handshake → post} → torch clean every N passes → unclamp. |
| F10 | Interlocks: the positioner moves only with the robot at home; no arc without safety OK, fume extraction OK, gas OK and wire OK; the robot starts only with the positioner in position. The temperature check keeps a 6 K pyrometer margin (3σ + 1 K) off both the interpass and preheat limits. |
| F11 | Safety stop: a light-curtain break → robot hold and arc off within one scan. Resume needs the curtain clear plus Reset. An interrupted pass is marked INTERRUPTED in its record. |
| F12 | Fault handling: an arc-start failure gets one retry, then FAULT; timeouts on clamp, positioner and gas; two temperature-sensor plausibility checks (no rise after a pass, or no cooling/heating within 1 h → FAULT SENSOR). FAULT latches a code; Reset with the cause removed resumes at the current pass. |
| F13 | Weld record per pass: serial, seam, pass, layer/bead, robot job, mean I/U, wire feed, travel speed, arc time, heat input, temperature at start, result (OK / NOK_PARAM / INTERRUPTED). Mean current outside the WPS band for a pass → NOK_PARAM and a quality alarm. |
| F14 | Per-part outputs: a JSON record, a CSV weld log and a 2-page PDF report (summary, weld map, temperature/Gantt chart). |
| F15 | SCL export of the FB for TIA Portal (`#` local variables, `TON_TIME`, quoted FB name). |
| F16 | The simulated run of the balanced plan finishes every pass, with no true interpass violation and no interlock violation recorded by the plant. |
| F17 | Same inputs, same results (deterministic). |
| P01 | A full-part simulation (≈ 90 passes) takes < 60 s wall time in CI. |

## 3. Models (simulated)

- **Deposited area per pass:** A = (π d²/4) · WFS / TS · η_dep. Groove: single-V A = (t − c)² · tan(α/2) + g · t; fillet A = z²/2 (+10 % reinforcement for both).
- **Thermal:**
  - Each pass adds ΔT_local = η_th · Q · L / (m_zone · c) to its seam zone and ΔT_bulk = Q · L / (m_part · c) to the part.
  - The local excess decays with τ_local, and the bulk decays to ambient with τ_bulk.
  - The seam temperature is bulk + local excess.
  - The PLC reads it through a simulated pyrometer (±2 K noise).
- **Robot:** a weld job = approach + arc time (L / v) + retract. Hold pauses the job. Job 900 cleans the torch.
- **Positioner:** 2 axes (tilt, rotate) at a set speed, with an in-position signal.
- **Power source:** a current/voltage set point per pass with noise. Faults can be injected.
- **Distortion proxy:** each pass adds w = Q·L to its seam's side vector (±x, ±y). The imbalance is |Σ w·side| relative to the total, and the sequence planner keeps its running maximum low.

## 4. Fault catalogue (scenarios)

| Fault | Expected reaction |
|---|---|
| `arcstart:<pass>` | first start fails, retry succeeds, the record notes 1 retry |
| `arcfail:<pass>` | the retry fails as well → FAULT ARC |
| `gas:<pass>` | gas flow lost before the arc → FAULT GAS (timeout), no arc |
| `fume:<pass>` | fume extraction not running → no arc, FAULT FUME |
| `curtain:<pass>` | light curtain broken mid-pass → hold, arc off, resume after Reset, pass INTERRUPTED |
| `clamp` | clamp not confirmed → FAULT CLAMP |
| `positioner:<pass>` | positioner stalls → FAULT POS |
| `pyro:<pass>` | pyrometer stuck from that pass on → FAULT SENSOR: after the pass if the frozen reading is inside the window (no rise), or after 1 h if it is frozen hot (no cooling) |
| `wfs:<pass>:<factor>` | wire-feed drift from that pass on → NOK_PARAM records, quality alarm |

Each scenario also includes an operator who presses Reset after the cause is cleared, so the part can finish.

## 5. Tests

| ID | What |
|---|---|
| UT-* | geometry, pass plan, heat input, thermal model, planners, ST translator, SCL export |
| IT01 | Test coupon and full part, balanced plan: every pass done, all records OK, no violations |
| IT02 | Every fault scenario in §4: expected state, fault code and record result |
| IT03 | The interlocks hold under a randomised stream of commands and disturbances (no positioner move away from home, no arc without permissives) |
| IT04 | The balanced plan beats the naive plan on cycle time and distortion imbalance for the full part |
| IT05 | Outputs: JSON, CSV and PDF written; SCL export generated; P01 timing |

## 6. Limits (honest)

- **The thermal model is lumped** (zone + bulk) with assumed constants. There is no FE analysis, no conduction between seams and no real cooling curve. The distortion proxy is a balance index, not millimetres.
- **The WPS numbers, preheat/interpass limits and material data are placeholders**, not values for any real steel or qualified procedure.
- **The robot is a time model.** There are no kinematics, no reach or collision check and no FANUC TP program in this repo.
- **The SCL export is generated text.** It was not compiled in TIA Portal here (no licence in CI).
- **The safety logic is safety-oriented demonstration only.** A real cell uses a certified safety PLC and safety devices.

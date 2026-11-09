# Changelog

## 0.1.0 (9 Nov 2026)

First release.
- **Pass planning:** single-V and fillet groove areas; root/fill/cap pass plan with layers and beads; heat input checked against the WPS window.
- **Thermal model:** lumped bulk + per-seam zone temperatures, with arc and heater input.
- **Sequence planners:** naive and balanced (temperature window, distortion balance, positioner moves). The balanced plan takes 8.3 h against 12.3 h for naive, with a distortion imbalance of 0.10 against 2.64.
- **Cell PLC:** `plc/FB_WeldCell.st`, an IEC 61131-3 ST function block. It covers the step chain, interlocks, light-curtain safety stop, arc-fault retry, pre-arc and positioner timeouts, a 6 K pyrometer margin, two sensor-plausibility checks and weld-current monitoring.
- **Running the PLC code:** an ST subset is translated to Python, so the ST source runs unchanged in tests and in the cell simulation. A Siemens SCL export is generated from the same source.
- **Cell simulation:** robot, positioner, power source, clamp, fume, gas, heater, pyrometer, light curtain and an operator model, with 9 fault scenarios.
- **Outputs:** a weld record per pass (JSON + CSV) and a 2-page PDF report per part.
- **Tests:** 45 tests and SPEC traceability. CI covers lint, traceability, tests, demo runs and the SCL check.

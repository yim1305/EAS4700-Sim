# Status

Updated 2026-10-15 on Landing-Sensor-Pointing-Constraints. README.md is the current guide, including all active constraints, decision variables and assumptions. DESIGN.md is the design summary. hopper_spec.md is retained as a historical specification with an archive notice.

Each hop has rise, ascent, coast, braking, descending observation and terminal phases. No hover. Observation ends at the decision gate; its time/altitude are optimized. Outbound gate is above the original target, altitude 150–1000 m. One prescribed 150 m uprange divert is assumed; worst-direction ranking and late-choice robustness are unproven.

Touchdown is optimized on the surface within 10 m of the divert target or return launch target, inside the 50 m requirement. Actual outbound touchdown position, normal and mass initialize return. Seven states per node, four powered controls, twelve durations and wet mass give 3321 variables in size mode. Gate timing and landing positions follow from those variables; no attitude variables.

Camera range remains 10 km. Hardware FOV 90 x 90 deg, margin 5 deg per edge, usable 80 x 80. Observation requires original target and actual touchdown visibility, not whole-patch coverage or later terminal tracking. Assumed sensing limits: downward 0.1–50 m/s, horizontal <=50 m/s, altitude >=10 m, acquisition >=1 s, processing 0.1 s. Effective slew cap 10 deg/s. Body-up follows thrust; no rotational dynamics or star trackers.

Latest run: wet mass 3766.68 kg, loaded propellant 2266.68 kg; both observation windows 1.10 s. Fixed-mass, fixed-gate, propagation and dense sensing checks pass. Focused tests: 19 passed; full suite last passed 43 tests before tightening/new margin test. Full mesh convergence is untested. JSON and three plots are updated. Higher initial gate guesses aid convergence.

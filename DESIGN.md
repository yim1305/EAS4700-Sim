# Lunar Hopper: Design

**Goal:** minimum wet mass for a round trip: hop 1 (start site to a divert site beside the target), then hop 2 (back to the exact start site). Used to value a new thruster.

**Model:** 3-DoF point mass in the rotating Moon-fixed frame, central gravity, spherical flat Moon. Open-loop optimal control (Hermite-Simpson collocation, CasADi + IPOPT). Each hop: vertical rise, ascent burn, coast, braking, terminal descent. The gate is the braking end; velocity is free except for nonpositive radial velocity. No hover or dwell is required. Controls: thrust vector and magnitude.

**Engine (edit `config.yaml` only):** 4 throttleable engines, 1112-6672 N each, Isp 310 s, O/F 1.65. Modelled as one equivalent engine, 1112-26,688 N (one engine at its floor) or off. No pulsing. Dry mass 1500 kg, fixed.

**Sizing rule:** loaded propellant = 1.1 x propellant used over the whole round trip. `mode: size` minimises m_wet subject to m_final >= m_wet - (m_wet - m_dry)/1.1.

**Constraints:** thrust bounds; m >= m_dry; altitude >= 50 m; gate 150-1000 m directly above the target; 45 deg glide slope to the landing site; tilt rate 10 deg/s; touchdown downward speed 0-1.5 m/s, horizontal <= 0.5 m/s; rise vertical to 50 m. Both touchdowns require thrust/body-up within 20 deg of the landing-site surface normal (`limits.td_tilt_max_deg`). This is an endpoint constraint, using aligned thrust as the body-axis proxy, without attitude or landing-contact dynamics.

**Divert:** one prescribed site, 150 m uprange (back toward launch), treated as the assumed worst case. Only that mission and its fixed-gate descent verification run by default. The optimizer can adapt its approach to the selected site; no comparison of all directions or late-choice robustness is claimed.

**Current result (150 m uprange, no hover, 20 deg touchdown tilt):** wet mass 3756.70 kg; propellant loaded 2256.70 kg; propellant used 2051.54 kg; total Delta-v 2401.30 m/s. Hop-1 gate 155.5 m, descending at 12.4 m/s. Both touchdowns use the 20 deg tilt limit. Main solve about 12 s. Saved in `results/roundtrip_uprange.json`; previous no-uprightness result preserved separately.

**Validation:** 35 tests pass, including both touchdown angles and speed components in fixed-mass and sizing modes. Independent chained propagation gives touchdown position errors of 0.2 mm outbound and 1.9 mm on return; fixed-gate uprange descent also converges and propagates correctly. Touchdown-speed scaling is corrected and IPOPT bound relaxation disabled. Terminal mesh increased from 25 to 50 intervals. No full mesh-convergence study is claimed.

**Code:** `hopper/problem.py` builds the two-hop problem; `initial_guess.py`, `constraints.py`, `dynamics.py`, `postprocess.py` (tables, propellant split), `validate.py` (re-propagation), `divert.py` (fixed-gate descent verification).

**Run:** `.\.venv\Scripts\python.exe -B -m pytest`, `.\.venv\Scripts\python.exe -B scripts/run_roundtrip.py`.

**Plots:** the runner saves and opens two Matplotlib figures after validation: a 3D trajectory over the curved Moon surface and six flight profiles (altitude, speed, thrust, mass, tilt, divert close-up). PNGs are saved under `results/`. Add `--no-show` to render without opening windows. Plotting is isolated in `hopper/plotting.py`.

**Placeholders:** gate floor, touchdown limits, glide slope, tilt rate.

**Not modelled:** sensor pointing, image acquisition/processing time, imaging speed limits, Isp loss at deep throttle, engine and tank mass scaling, perturbations, terrain, guidance and control, start-cycle rating (2 s restart not enforced). Sensor constraints are the next step after no-hover divert validation.

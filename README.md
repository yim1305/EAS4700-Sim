# Lunar Hopper Round-Trip Optimizer

## 1. What it is, what it does

A Python tool that designs the flight of a small lunar lander (a "hopper") and tells you **how heavy it must be at launch**.

The hopper starts near the Moon's south pole, flies about 200 km to a landing site, then flies back to exactly where it started. The current sizing case always lands **150 m uprange of the target**, back toward the launch site, to represent an assumed worst-case divert. Only that selected mission is optimized; its ranking against other directions is not established.

The tool finds the thrust profile and burn timing that use the **least propellant**, and from that it computes the **minimum wet mass** (the mass at launch, with full tanks). The purpose is to compare engines: change the engine numbers in `config.yaml`, run again, and see how much launch mass a better thruster saves.

**Current no-hover result: 150 m uprange divert, 20 degree touchdown tilt limit** (4 engines, 1112-6672 N each, Isp 310 s, dry mass 1500 kg):

| | |
|---|---|
| Minimum wet mass | **3757 kg** |
| Propellant loaded | 2257 kg (852 kg fuel + 1405 kg oxidizer) |
| Propellant used | 2052 kg |
| Total velocity change (Delta-v) | 2401 m/s (hop 1: 1231, hop 2: 1170) |
| Hop-1 moving gate | 155.5 m altitude, 12.4 m/s downward speed |
| Terminal divert descent | 14.7 s |
| Touchdown tilt (thrust/body-up proxy) | 20 degrees on both hops |
| Main solve time | about 12 seconds on this machine |

Saved in `results/roundtrip_uprange.json`. All 35 tests pass; chained independent propagation gives touchdown position errors of 0.2 mm outbound and 1.9 mm on return. The prior no-uprightness run (3754.01 kg) is preserved in `results/roundtrip_uprange_no_uprightness.json`; the new result also includes corrected touchdown-speed scaling and stricter bound handling. The historical `roundtrip_downrange.json` is the old 100 m downrange case with hover (3914 kg wet mass), so it is not a direct measure of hover savings for this different mission case.

## 2. Assumptions and limitations (read this first)

**Assumptions**
- **3-DoF point mass.** The vehicle is a point with position, velocity and mass. The body-up direction is approximated by resultant thrust, assuming aligned engines with no independent gimbal deflection. Both touchdowns require this direction within 20 degrees of the local surface normal. No rotational dynamics are modelled; a thrust-direction rate limit of 10 deg/s remains the slew approximation.
- **Moon:** a perfect sphere, flat terrain at zero elevation, point-mass gravity, plus the Moon's rotation (Coriolis and centrifugal terms). No Earth/Sun pull, no gravity anomalies.
- **Engines:** 4 engines that throttle continuously between 1112 N and 6672 N each, or turn off. No pulsing. They are modelled as **one equivalent engine** that produces 1112-26,688 N (one engine at its minimum, the others off) or zero. Isp is constant at 310 s.
- **Dry mass is fixed at 1500 kg** (structure, sensors and engines) whatever the engines or propellant load. Tank mass does not grow with propellant.
- **Sizing rule:** loaded propellant = 1.1 x the propellant used over the **whole round trip**. No other safety factor.
- **One prescribed divert site.** The default is uprange, 150 m, toward the launch site. The optimizer does not jointly size for all possible landing sites.
- **The selected divert is part of the sizing scenario.** The moving gate is the braking end, directly above the target. No stop or dwell is required. The optimizer knows the selected landing site in advance and can adapt its pre-gate trajectory and gate velocity to it; this is not a demonstration that other sites remain reachable after late hazard detection.
- **Surface time between the hops is ignored.** Hop 2 starts at rest, at the landing spot, with the same mass.

**Limitations**
- **Open-loop:** the result is the ideal reference trajectory. There is no guidance law, navigation error or disturbance.
- IPOPT is a local optimizer. The answer depends on the starting guess, which comes from a Lambert (ballistic-arc) estimate.
- Several limits are **placeholders** (marked in `config.yaml`): gate altitude range, minimum altitude, rise height, glide slope, tilt rate, touchdown speeds and tilt, phase durations.
- Sensor field of view, imaging speed limits, acquisition time and processing latency are not yet modelled. Removing hover does not establish sensor feasibility.
- Not modelled: Isp loss at deep throttle, engine restart time (2 s) and start-cycle rating, engine mass change.
- The engine numbers are inputs from the user and were not independently verified.
- Trajectory and flight-profile plots are saved automatically; trajectory CSV export is not yet implemented.

## 3. Setup and trajectory design

**Setup:** start site at latitude -85.5, longitude -87.1. Target 78.9 deg S, same longitude, about 200 km away. The vehicle leaves the start site at rest, flies to the target area, lands, then repeats the trip back.

**Each hop has five phases:**

| # | Phase | Engines | What happens |
|---|-------|---------|--------------|
| 1 | Rise | on | Straight up to 50 m (no sideways speed) |
| 2 | Ascent | on | Tilts over and accelerates toward the target (phases 1+2 are one burn) |
| 3 | Coast | **off** | Ballistic arc, about 7 minutes |
| 4 | Braking | on | Reduces speed; ends at the **moving gate**, descending or level |
| 5 | Terminal descent | on | Continues from the gate's position, velocity and mass to the landing site (phases 4+5 are the second burn) |

So each hop has **2 engine starts**, 4 for the round trip.

**Where the divert starts.** The hop-1 gate is directly above the target, between 150 and 1000 m altitude. Its speed is optimized, with no upward radial velocity. The terminal descent proceeds to the one selected site, 150 m uprange. The runner verifies that descent again from exactly the same gate position, velocity, mass and preceding thrust command. It does not optimize nominal or other-direction missions.

**Hop 1** lands on the divert site. **Hop 2** starts from there and returns to the exact start site. It has no divert, so its gate position is free.

The current timeline is printed by `scripts/run_roundtrip.py` and saved in the result JSON; it contains no hover phases.

## 4. Free variables (what the optimizer chooses)

For each hop, at every node (collocation point) of the trajectory:

1. **State:** position (3), velocity (3) and mass (1).
2. **Controls, in powered phases:** thrust vector (3 components, N) and thrust magnitude Gamma (N), which sets the throttle.
3. **Phase durations** of all five phases per hop: rise, ascent, coast, braking, terminal.
4. **Wet mass** (one variable for the whole problem, in `size` mode).

Everything else follows from these: when each burn starts and stops, how long the coast lasts, the shape of the arc, the throttle history, the **gate altitude** (free between 150 and 1000 m) and the landing speeds. The gate's horizontal position is fixed above the target on hop 1 and free on hop 2.

Size of the problem: 5 phases x 2 hops (rise 10, ascent 30, coast 20, braking 30, terminal 50 intervals). The terminal mesh is refined for moving-gate divert propagation. The runner prints the variable count.

## 5. Constraints (what the optimizer must satisfy)

| Constraint | Applies to | Value |
|---|---|---|
| Equations of motion (Hermite-Simpson collocation) | every phase | gravity, Coriolis, centrifugal, thrust, mass flow = Gamma/(g0 x Isp) |
| Thrust bounds | powered phases | 1112 N <= Gamma <= 26,688 N, and thrust vector length <= Gamma |
| Engines off | coast | thrust = 0 |
| Mass floor | all nodes | mass >= 1500 kg |
| Start condition | hop 1 start | at the start site, speed 0, mass = wet mass |
| Vertical rise | rise | no horizontal speed at any node; ends at height 50 m |
| Minimum altitude | ascent, coast, braking | >= 50 m |
| Moving gate | braking end, both hops | altitude within 150-1000 m; radial velocity <= 0; no required stop |
| Gate over target | hop 1 braking end | gate directly above the target |
| Glide slope | terminal descents | at least 45 deg elevation seen from the landing site (all nodes but the last) |
| Landing position | end of each terminal descent | exactly on the divert site (hop 1), exactly on the start site (hop 2) |
| Touchdown speed | end of each terminal descent | downward speed 0-1.5 m/s (no upward arrival), sideways <= 0.5 m/s |
| Touchdown uprightness | final node of both terminal descents | thrust/body-up within 20 deg of the landing-site surface normal |
| Tilt rate | powered phases, and across joins inside a burn (rise to ascent, braking to terminal) | thrust direction turns <= 10 deg/s |
| Phase duration limits | free phases | rise 1-120 s, ascent 1-600, coast 1-2000, braking 1-600, terminal 1-300 |
| Phase linking | between phases | end state of one phase = start state of the next |
| Surface stay | between hops | hop 2 starts at the hop 1 landing spot, speed 0, same mass |
| Wet mass bounds | size mode | 1.05 x dry mass to 3 x the config `m_wet` |
| **Sizing rule** | size mode | final mass >= wet mass - (wet mass - dry mass) / 1.1 |

**Objective:** minimise wet mass (`mode: size`). The alternative `mode: maximise_margin` fixes wet mass at the config value and maximises the final mass instead.

The uprightness constraint applies at touchdown, not throughout the entire divert. Earlier thrust directions remain subject to the existing turn-rate constraint. It does not check landing-gear stability, contact dynamics or achievable attitude-control torques. The 20 degree limit is a user-selected placeholder, configurable as `limits.td_tilt_max_deg`.

## 6. How the optimization works

**Method: direct collocation.** The flight is turned into one large numerical optimization problem:
1. Each phase is cut into equal time steps (the nodes of section 4).
2. The unknowns are the free variables of section 4 at every node.
3. The physics becomes equations. For each time step of length h, Hermite-Simpson collocation requires `s[k+1] - s[k] = (h/6) * (f[k] + 4*f[mid] + f[k+1])`, where `s` is the state, `f` is the equations of motion, and the midpoint state is `s[mid] = (s[k] + s[k+1])/2 + (h/8) * (f[k] - f[k+1])`. These "defects" must be zero. Thrust is treated as linear between nodes. Time inside each phase is normalised to 0-1 so durations can be variables.
4. All constraints of section 5 are added, giving one nonlinear program (NLP).
5. **IPOPT** (an interior-point NLP solver) solves it through CasADi's `Opti` interface, which supplies exact derivatives by automatic differentiation. Tolerance 1e-8, up to 3000 iterations; `acceptable_iter` and `bound_relax_factor` are set to zero. It is a local optimizer. Touchdown-speed constraints are scaled using their own limits to maintain useful derivatives and enforce the intended speed bounds. The runner refuses to save a result that fails its sizing cross-check or discards paid-for thrust.

**The function being minimised** (`Problem._build` in `problem.py`):
- `mode: size`: minimise **wet mass** `m_wet`, with the constraint `final mass >= m_wet - (m_wet - m_dry)/1.1`.
- `mode: maximise_margin`: minimise `-(final mass)` at a fixed `m_wet`.

Both mean "burn as little propellant as possible". Propellant flows at `Gamma / (g0 x Isp)`, so the rocket equation is built into the dynamics and nothing is assumed about Delta-v.

**Thrust model.** The optimizer picks a thrust vector and a magnitude `Gamma` with `|vector| <= Gamma`. Fuel burn follows `Gamma`, so wasting thrust costs fuel and the solver sets them equal. A post-check flags any node where it did not (none in the current run).

**Scaling.** Lengths are scaled by the start-to-target distance (about 200 km), time by `sqrt(L/g)`, mass by the config `m_wet`, and force by `m_wet x g`, so all variables are near 1.

**Starting guess** (`build_guess` in `initial_guess.py`): a Lambert arc (`lambert.py`) for the coast, rocket-equation estimates for the burn times, smooth curves between the phase end points, and the thrust that would produce them. Gate speed is zero only in this starting guess; the optimizer is free to change it.

**Checks after solving:** the solution is re-integrated with scipy (`validate.py`, touchdown position errors under 5 mm in the current result), the sizing is re-run at the fixed solved mass (loaded/used must be 1.100), and the selected terminal descent is re-solved and propagated from the same moving gate. The result is also compared with the ideal Lambert Delta-v.

## 7. Inputs: `config.yaml`

All numbers live here, nothing is hard-coded. The main ones:

| Section | Key | Meaning |
|---|---|---|
| `engine` | `n_engines`, `thrust_per_engine`, `isp_vac`, `min_throttle`, `mixture_ratio` | Engine design. `min_throttle` = lowest thrust as a fraction of total (0.041667 = one engine at 1112 N). |
| `vehicle` | `m_dry`, `m_wet` | Dry mass; `m_wet` is a scaling and starting value only in size mode |
| `mission` | `mode`, `propellant_reserve_factor`, `divert_radius`, `gate_alt_bounds`, `gate_over_target` | Sizing mode and rule, divert distance, gate rules |
| `limits` | rise height, minimum altitude, glide slope, tilt rate, touchdown speeds, `td_tilt_max_deg`, phase durations | Path and touchdown limits |
| `sites` | `start`, `target` | Latitude, longitude, elevation |
| `solver` | `nodes`, `ipopt` | Mesh size and solver settings |

The gate must satisfy the terminal glide-slope corridor relative to the divert site. For the current 45 degree slope and 150 m offset, the local-flat-ground geometric minimum is about 150 m; a higher gate remains available within the configured altitude bounds.

## 8. Install and run

Use a project-local Python environment. From the project folder on Windows:

```
py -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe -B scripts/run_roundtrip.py       # selected 150 m uprange divert and return
.\.venv\Scripts\python.exe -B scripts/run_roundtrip.py --no-show  # save plots without opening windows
.\.venv\Scripts\python.exe -B -m pytest
```

To try a different engine, edit the `engine` section of `config.yaml` and run again.

In VS Code, use **Python: Select Interpreter** from the command palette and select `.venv\Scripts\python.exe` for this workspace before using **Run Python File**. Dependencies are installed in that environment. Workspace settings prefer `.venv`; an existing interpreter selection may need to be switched once.

## 9. How to read the data

**Console.** The script prints start-up checks, the Lambert (ideal) Delta-v for each hop, the phase timeline, per-hop and total summaries, fuel/oxidizer split, sizing check, moving-gate velocity, independent propagation errors, and verification of the selected divert.

**Matplotlib plots.** After the mission converges and passes validation, two windows open:
- A rotatable 3D round-trip trajectory in launch-centered Cartesian coordinates, with the curved lunar surface, sites and moving gates. Blue is outbound; orange is return; dashed segments are coasts. Axis scales differ for visibility.
- Six flight panels: altitude, speed, commanded thrust, mass, thrust/body tilt, and a close-up of the outbound divert with its glide-slope boundary. Shaded time intervals show coasts; dotted vertical lines mark the gates. The tilt limit applies only at touchdown, marked by dots.

Both figures are saved in `results/trajectory_3d_<site>.png` and `results/flight_profiles_<site>.png`. Close the windows to finish the run, or use `--no-show` to save the PNGs without displaying them. No additional packages are needed beyond `requirements.txt`.

**JSON file.** `results/roundtrip_<site>.json` holds the same numbers:
- `summary`: `hop1`, `hop2`, `total` (`m_start` is the wet mass, `m_final`, `propellant_used`, `propellant_loaded`, `finite_burn_dv`, `engine_starts`), and the fuel/oxidizer splits
- `divert`: the selected fixed-gate descent status, duration, delta-v, propellant and propagation errors
- `lambert`: ideal Delta-v per hop
- `gate`: hop-1 gate altitude, Moon-fixed velocity and vertical speed
- `touchdown_limits`: maximum tilt and downward/horizontal speeds
- `validation`: independent propagation errors for each phase and both touchdowns
- `table`: the per-phase rows; `solve_wall_s`, `n_variables`

**Trajectory arrays (Python).** Run from the project folder:

```python
import numpy as np
from hopper.params import load_params
from hopper.problem import Problem

p = load_params("config.yaml")
pr = Problem(p, "uprange")
pr.set_guess()
sol = pr.solve(print_level=0)

ph = sol.phases["ascent"]            # phase names: see below
X, U, T = ph["X"], ph["U"], ph["t"]
t = np.linspace(0, T, X.shape[1])    # time within the phase (s)
r_abs = p.mission.r_start[:, None] + X[0:3]
altitude = np.linalg.norm(r_abs, axis=0) - p.moon.radius
```

- `X` has 7 rows: position offset from the start site x, y, z (m, Moon-fixed frame), velocity vx, vy, vz (m/s), mass (kg).
- `U` has 4 rows: thrust vector Tx, Ty, Tz (N) and magnitude Gamma (N). It is zero during coasts.
- `t` is the phase duration (s). Add durations in order for mission time.
- Phase names in flight order: `rise`, `ascent`, `coast`, `braking`, `terminal_<site>`, `rise2_<site>`, `ascent2_<site>`, `coast2_<site>`, `braking2_<site>`, `terminal2_<site>`. The moving gate is the last state of `braking`.
- `sol.m_wet` is the sized wet mass, `sol.z` the final mass, `sol.success` whether it converged.

## 10. What each file does

| File | Purpose |
|---|---|
| `config.yaml` | All inputs |
| `scripts/run_roundtrip.py` | Solves the selected mission, writes the JSON, checks sizing and verifies its divert |
| `hopper/params.py` | Loads and validates the config; start-up checks |
| `hopper/problem.py` | Builds the two-hop optimization problem and solves it |
| `hopper/phases.py` | Container for one phase's variables and constraints |
| `hopper/constraints.py` | Every constraint, one class each |
| `hopper/dynamics.py` | Equations of motion |
| `hopper/transcription.py` | Hermite-Simpson collocation (turns the dynamics into equations) |
| `hopper/initial_guess.py` | Starting trajectory from a Lambert arc and rocket-equation burn times |
| `hopper/lambert.py` | Ballistic transfer solver; gives the ideal Delta-v baseline |
| `hopper/postprocess.py` | Phase tables, Delta-v, propellant, fuel/oxidizer split, summaries |
| `hopper/plotting.py` | Saves and displays the 3D trajectory and flight profiles |
| `hopper/validate.py` | Re-integrates the solution independently to check it |
| `hopper/divert.py` | Solves a single descent from the moving gate; optional comparison helper retained for separate studies |
| `hopper/environment/` | Gravity model and terrain/altitude helper |
| `tests/` | Unit and end-to-end tests |
| `DESIGN.md` | One-page design summary |
| `hopper_spec.md` | Original specification (older multi-branch design) |

## 11. Adding or changing things

- **New constraint:** add a class in `constraints.py` and one line in the matching phase list in `problem.py` (`_make_hop`).
- **Different gravity or terrain:** implement the interface in `hopper/environment/`.
- **Different divert direction:** pass its name to the script or to `Problem`.

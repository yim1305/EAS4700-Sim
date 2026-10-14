# Lunar Hopper Round-Trip Optimizer

## 1. What it is, what it does

A Python tool that designs the flight of a small lunar lander (a "hopper") and tells you **how heavy it must be at launch**.

The hopper starts near the Moon's south pole, flies about 200 km to a landing site, then flies back to exactly where it started. It must also be able to **divert**: land up to 100 m away from the planned spot if a hazard (boulder, steep slope) is spotted on final approach.

The tool finds the thrust profile and burn timing that use the **least propellant**, and from that it computes the **minimum wet mass** (the mass at launch, with full tanks). The purpose is to compare engines: change the engine numbers in `config.yaml`, run again, and see how much launch mass a better thruster saves.

**Current result** (4 engines, 1112-6672 N each, Isp 310 s, dry mass 1500 kg):

| | |
|---|---|
| Minimum wet mass | **3914 kg** |
| Propellant loaded | 2414 kg (911 kg fuel + 1503 kg oxidizer) |
| Propellant used | 2195 kg |
| Total velocity change (Delta-v) | 2501 m/s (hop 1: 1268, hop 2: 1233) |
| Cost of the 100 m divert | about +15 kg wet mass |
| Run time | about 6 seconds |

## 2. Assumptions and limitations (read this first)

**Assumptions**
- **3-DoF point mass.** The vehicle is a point with position, velocity and mass. There is no attitude (rotation) model. A limit on how fast the thrust direction may turn (10 deg/s) stands in for it.
- **Moon:** a perfect sphere, flat terrain at zero elevation, point-mass gravity, plus the Moon's rotation (Coriolis and centrifugal terms). No Earth/Sun pull, no gravity anomalies.
- **Engines:** 4 engines that throttle continuously between 1112 N and 6672 N each, or turn off. No pulsing. They are modelled as **one equivalent engine** that produces 1112-26,688 N (one engine at its minimum, the others off) or zero. Isp is constant at 310 s.
- **Dry mass is fixed at 1500 kg** (structure, sensors and engines) whatever the engines or propellant load. Tank mass does not grow with propellant.
- **Sizing rule:** loaded propellant = 1.1 x the propellant used over the **whole round trip**. No other safety factor.
- **One divert site.** The tool sizes for the worst direction (downrange, 100 m). The other three directions were checked only at an earlier 5000 kg wet mass (all within 7 kg) and not at the final mass.
- **The hazard is known at the gate** (the hover point). Before the gate, every case flies the same path.
- **Surface time between the hops is ignored.** Hop 2 starts at rest, at the landing spot, with the same mass.

**Limitations**
- **Open-loop:** the result is the ideal reference trajectory. There is no guidance law, navigation error or disturbance.
- IPOPT is a local optimizer. The answer depends on the starting guess, which comes from a Lambert (ballistic-arc) estimate.
- Several limits are **placeholders** (marked in `config.yaml`): hover time, gate altitude range, minimum altitude, rise height, glide slope, tilt rate, touchdown speeds, phase durations.
- The gate sits exactly at its 150 m lower limit, and both landings use the full allowed touchdown speed. The wet mass therefore depends on these placeholders.
- Not modelled: Isp loss at deep throttle, engine restart time (2 s) and start-cycle rating, engine mass change.
- The engine numbers are inputs from the user and were not independently verified.
- No plots or trajectory CSV export yet (see section 9 for how to read the data).

## 3. Setup and trajectory design

**Setup:** start site at latitude -85.5, longitude -87.1. Target 78.9 deg S, same longitude, about 200 km away. The vehicle leaves the start site at rest, flies to the target area, hovers, lands, then repeats the trip back.

**Each hop has six phases:**

| # | Phase | Engines | What happens |
|---|-------|---------|--------------|
| 1 | Rise | on | Straight up to 50 m (no sideways speed) |
| 2 | Ascent | on | Tilts over and accelerates toward the target (phases 1+2 are one burn) |
| 3 | Coast | **off** | Ballistic arc, about 7 minutes |
| 4 | Braking | on | Cancels the speed; ends stationary at the **gate** |
| 5 | Hover | on | Holds still for 20 s at the gate: time to look for hazards |
| 6 | Terminal descent | on | Descends to the landing site (phases 4+5+6 are the second burn) |

So each hop has **2 engine starts**, 4 for the round trip.

**Where the divert starts.** The gate is a hover point directly above the target, at least 100 m up (150 m in the current run). At the end of the hover the vehicle commits:
- no hazard: it descends to the target, or
- hazard: it descends to a divert site up to 100 m away (uprange, downrange, left or right).

Before the gate all cases are identical. After it, the divert descent takes longer and costs more (about 19 s and 34 m/s, against 17 s and 26 m/s for the nominal landing). The tool sizes for downrange, the most expensive direction found.

**Hop 1** lands on the divert site. **Hop 2** starts from there and returns to the exact start site. It has no divert, so its gate position is free.

**Current timeline** (seconds from launch): rise 0-5, ascent to 85, coast to 510, braking to 575, hover to 595, landing at 614. Hop 2 repeats the pattern and lands at 1208.

## 4. Free variables (what the optimizer chooses)

For each hop, at every node (collocation point) of the trajectory:

1. **State:** position (3), velocity (3) and mass (1).
2. **Controls, in powered phases:** thrust vector (3 components, N) and thrust magnitude Gamma (N), which sets the throttle.
3. **Phase durations** of the five free phases per hop: rise, ascent, coast, braking, terminal. Hover is fixed at 20 s.
4. **Wet mass** (one variable for the whole problem, in `size` mode).

Everything else follows from these: when each burn starts and stops, how long the coast lasts, the shape of the arc, the throttle history, the **gate altitude** (free between 150 and 1000 m) and the landing speeds. The gate's horizontal position is fixed above the target on hop 1 and free on hop 2.

Size of the problem: 6 phases x 2 hops, 129 nodes per hop (rise 10, ascent 30, coast 20, braking 30, hover 8, terminal 25 intervals), **2681 variables**.

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
| Hover | hover | speed 0 at all nodes; altitude within 150-1000 m |
| Gate over target | hop 1 hover | gate directly above the target |
| Glide slope | terminal descents | at least 45 deg elevation seen from the landing site (all nodes but the last) |
| Landing position | end of each terminal descent | exactly on the divert site (hop 1), exactly on the start site (hop 2) |
| Touchdown speed | end of each terminal descent | down <= 1.5 m/s, sideways <= 0.5 m/s |
| Tilt rate | powered phases, and across joins inside a burn (rise to ascent, braking to hover to terminal) | thrust direction turns <= 10 deg/s |
| Phase duration limits | free phases | rise 1-120 s, ascent 1-600, coast 1-2000, braking 1-600, terminal 1-300 |
| Phase linking | between phases | end state of one phase = start state of the next |
| Surface stay | between hops | hop 2 starts at the hop 1 landing spot, speed 0, same mass |
| Wet mass bounds | size mode | 1.05 x dry mass to 3 x the config `m_wet` |
| **Sizing rule** | size mode | final mass >= wet mass - (wet mass - dry mass) / 1.1 |

**Objective:** minimise wet mass (`mode: size`). The alternative `mode: maximise_margin` fixes wet mass at the config value and maximises the final mass instead.

## 6. How the optimization works

**Method: direct collocation.** The flight is turned into one large numerical optimization problem:
1. Each phase is cut into equal time steps (the nodes of section 4).
2. The unknowns are the free variables of section 4 at every node.
3. The physics becomes equations. For each time step of length h, Hermite-Simpson collocation requires `s[k+1] - s[k] = (h/6) * (f[k] + 4*f[mid] + f[k+1])`, where `s` is the state, `f` is the equations of motion, and the midpoint state is `s[mid] = (s[k] + s[k+1])/2 + (h/8) * (f[k] - f[k+1])`. These "defects" must be zero. Thrust is treated as linear between nodes. Time inside each phase is normalised to 0-1 so durations can be variables.
4. All constraints of section 5 are added, giving one nonlinear program (NLP) with 2681 variables.
5. **IPOPT** (an interior-point NLP solver) solves it through CasADi's `Opti` interface, which supplies exact derivatives by automatic differentiation. Tolerance 1e-8, up to 3000 iterations. It is a local optimizer.

**The function being minimised** (`Problem._build` in `problem.py`):
- `mode: size`: minimise **wet mass** `m_wet`, with the constraint `final mass >= m_wet - (m_wet - m_dry)/1.1`.
- `mode: maximise_margin`: minimise `-(final mass)` at a fixed `m_wet`.

Both mean "burn as little propellant as possible". Propellant flows at `Gamma / (g0 x Isp)`, so the rocket equation is built into the dynamics and nothing is assumed about Delta-v.

**Thrust model.** The optimizer picks a thrust vector and a magnitude `Gamma` with `|vector| <= Gamma`. Fuel burn follows `Gamma`, so wasting thrust costs fuel and the solver sets them equal. A post-check flags any node where it did not (none in the current run).

**Scaling.** Lengths are scaled by the start-to-target distance (about 200 km), time by `sqrt(L/g)`, mass by the config `m_wet`, and force by `m_wet x g`, so all variables are near 1.

**Starting guess** (`build_guess` in `initial_guess.py`): a Lambert arc (`lambert.py`) for the coast, rocket-equation estimates for the burn times, smooth curves between the phase end points, and the thrust that would produce them. A good guess is why the solve takes only about 6 s from a cold start.

**Checks after solving:** the solution is re-integrated with scipy (`validate.py`, errors under 2 mm), the sizing is re-run at the fixed solved mass (loaded/used must be 1.100), and the result is compared with the ideal Lambert Delta-v.

## 7. Inputs: `config.yaml`

All numbers live here, nothing is hard-coded. The main ones:

| Section | Key | Meaning |
|---|---|---|
| `engine` | `n_engines`, `thrust_per_engine`, `isp_vac`, `min_throttle`, `mixture_ratio` | Engine design. `min_throttle` = lowest thrust as a fraction of total (0.041667 = one engine at 1112 N). |
| `vehicle` | `m_dry`, `m_wet` | Dry mass; `m_wet` is a scaling and starting value only in size mode |
| `mission` | `mode`, `propellant_reserve_factor`, `hover_time`, `divert_radius`, `gate_alt_bounds`, `gate_over_target` | Sizing mode and rule, divert distance, gate rules |
| `limits` | rise height, minimum altitude, glide slope, tilt rate, touchdown speeds, phase durations | Path limits |
| `sites` | `start`, `target` | Latitude, longitude, elevation |
| `solver` | `nodes`, `ipopt` | Mesh size and solver settings |

The gate floor must exceed `tan(glide slope) x divert radius` (100 m), or the divert is infeasible. The start-up check warns about this.

## 8. Install and run

Developed and tested on Python 3.14 (casadi 3.8, numpy 2.4, scipy 1.17). From the project folder:

```
pip install -r requirements.txt
py scripts/run_roundtrip.py            # main run (default divert site: downrange)
py scripts/run_roundtrip.py left       # other sites: nominal, uprange, downrange, left, right
py -m pytest                           # tests, about 15 s
```

To try a different engine, edit the `engine` section of `config.yaml` and run again.

## 9. How to read the data

**Console.** The script prints, in order: start-up checks; the Lambert (ideal) Delta-v for each hop; a table with one row per phase (start, end, duration, engine on/off, Delta-v, propellant used); per-hop and total summaries; the fuel/oxidizer split; the sizing check; re-propagation errors; and the cost of the divert.

**JSON file.** `results/roundtrip_<site>.json` holds the same numbers:
- `summary`: `hop1`, `hop2`, `total` (`m_start` is the wet mass, `m_final`, `propellant_used`, `propellant_loaded`, `finite_burn_dv`, `engine_starts`), and the fuel/oxidizer splits
- `divert`: cost of the divert for the descent alone and for the whole round trip
- `lambert`: ideal Delta-v per hop
- `table`: the per-phase rows; `solve_wall_s`, `n_variables`

**Trajectory arrays (Python).** Run from the project folder:

```python
import numpy as np
from hopper.params import load_params
from hopper.problem import Problem

p = load_params("config.yaml")
pr = Problem(p, "downrange")
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
- Phase names in flight order: `rise`, `ascent`, `coast`, `braking`, `hover`, `terminal_<site>`, `rise2_<site>`, `ascent2_<site>`, `coast2_<site>`, `braking2_<site>`, `hover2_<site>`, `terminal2_<site>`.
- `sol.m_wet` is the sized wet mass, `sol.z` the final mass, `sol.success` whether it converged.

## 10. What each file does

| File | Purpose |
|---|---|
| `config.yaml` | All inputs |
| `scripts/run_roundtrip.py` | Main run: solves, prints results, writes the JSON, checks the sizing, reports the divert cost |
| `hopper/params.py` | Loads and validates the config; start-up checks |
| `hopper/problem.py` | Builds the two-hop optimization problem and solves it |
| `hopper/phases.py` | Container for one phase's variables and constraints |
| `hopper/constraints.py` | Every constraint, one class each |
| `hopper/dynamics.py` | Equations of motion |
| `hopper/transcription.py` | Hermite-Simpson collocation (turns the dynamics into equations) |
| `hopper/initial_guess.py` | Starting trajectory from a Lambert arc and rocket-equation burn times |
| `hopper/lambert.py` | Ballistic transfer solver; gives the ideal Delta-v baseline |
| `hopper/postprocess.py` | Phase tables, Delta-v, propellant, fuel/oxidizer split, summaries |
| `hopper/validate.py` | Re-integrates the solution independently to check it |
| `hopper/divert.py` | Solves a single descent from the gate to compare divert and nominal landings |
| `hopper/environment/` | Gravity model and terrain/altitude helper |
| `tests/` | Unit and end-to-end tests |
| `DESIGN.md` | One-page design summary |
| `hopper_spec.md` | Original specification (older multi-branch design) |

## 11. Adding or changing things

- **New constraint:** add a class in `constraints.py` and one line in the matching phase list in `problem.py` (`_make_hop`).
- **Different gravity or terrain:** implement the interface in `hopper/environment/`.
- **Different divert direction:** pass its name to the script or to `Problem`.

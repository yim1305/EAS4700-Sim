# Lunar Hopper: Design

**Goal:** minimum wet mass for a round trip: hop 1 (start site to a divert site beside the target), then hop 2 (back to the exact start site). Used to value a new thruster.

**Model:** 3-DoF point mass in the rotating Moon-fixed frame, central gravity, spherical flat Moon. Open-loop optimal control (Hermite-Simpson collocation, CasADi + IPOPT). Each hop: vertical rise, ascent burn, coast, braking, 20 s hover at the gate, terminal descent. Controls: thrust vector and magnitude.

**Engine (edit `config.yaml` only):** 4 throttleable engines, 1112-6672 N each, Isp 310 s, O/F 1.65. Modelled as one equivalent engine, 1112-26,688 N (one engine at its floor) or off. No pulsing. Dry mass 1500 kg, fixed.

**Sizing rule:** loaded propellant = 1.1 x propellant used over the whole round trip. `mode: size` minimises m_wet subject to m_final >= m_wet - (m_wet - m_dry)/1.1.

**Constraints:** thrust bounds; m >= m_dry; altitude >= 50 m; gate 150-1000 m directly above the target; 45 deg glide slope to the landing site; tilt rate 10 deg/s; touchdown <= 1.5 m/s vertical, 0.5 m/s horizontal; rise vertical to 50 m.

**Divert:** one worst-case site (downrange, 100 m). Other sites differ by under 7 kg (checked at 5000 kg wet mass).

**Results (current config):** wet mass 3914 kg; propellant loaded 2414 kg (911 fuel, 1503 oxidizer); total Delta-v 2501 m/s; divert adds about 15 kg wet mass; solve about 5 s.

**Code:** `hopper/problem.py` builds the two-hop problem; `initial_guess.py`, `constraints.py`, `dynamics.py`, `postprocess.py` (tables, propellant split), `validate.py` (re-propagation), `divert.py` (divert-descent cost).

**Run:** `py -m pytest`, `py scripts/run_roundtrip.py [branch]`.

**Placeholders:** gate floor, touchdown limits, glide slope, tilt rate, hover time.

**Not modelled:** Isp loss at deep throttle, engine and tank mass scaling, perturbations, terrain, guidance and control, start-cycle rating (2 s restart not enforced). Mesh-refinement check not done.

# Status

Working on `Attitude-constraints-test`. Both hops have five phases: rise, ascent, coast, braking, terminal descent. The fixed 20 s hovers are removed. The gate is the braking endpoint, within the configured altitude window, with nonpositive radial velocity and full state continuity into terminal descent.

The user selected one assumed worst-case site, 150 m uprange toward launch. `Problem` and the runner default to this case; automatic nominal and other-direction comparisons are removed. The optimizer can adapt its approach and gate velocity to the selected site, so late-choice robustness and worst-direction ranking are not established.

Added the user-selected 20 deg touchdown uprightness limit to both terminal phases and standalone divert verification. Body-up is approximated by resultant thrust, assuming aligned engines, without rotational dynamics. The limit applies at the touchdown node. Config: `limits.td_tilt_max_deg`. Explicitly disallow upward touchdown velocity. Corrected lateral-speed constraint conditioning; disabled IPOPT bound relaxation and acceptable-iteration stopping. The runner rejects failed sizing cross-checks and discarded thrust.

Validation: 35 tests pass. Main solve succeeds; wet mass 3756.70 kg, loaded propellant 2256.70 kg, used 2051.54 kg, delta-v 2401.30 m/s. Both touchdown tilts approach 20 deg, with downward speeds below 1.5 m/s and lateral speeds below 0.5 m/s. Chained propagation errors: 0.2 mm outbound, 1.9 mm return. Fixed-gate divert and fixed-mass sizing cross-check pass. Results saved; previous no-uprightness baseline preserved separately. Full mesh convergence remains untested.

Plotting: the runner saves/opens a 3D trajectory and six flight profiles, including a divert close-up. `--no-show` saves PNGs without windows. Plot code is isolated in `hopper/plotting.py`. Full headless run, PNG visual inspection and interactive TkAgg availability checked.

VS Code workspace settings now prefer the existing `.venv` interpreter.

Sensor pointing/timing and star trackers remain deferred. Future hazard sensor assumption: bottom-mounted, 90 x 90 deg square FOV. Original `hopper_spec.md` remains unchanged.

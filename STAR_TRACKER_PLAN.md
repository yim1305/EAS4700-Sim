# Star tracker plan and implementation handoff

Updated 2026-10-16 on branch Star-Tracker-Constraints. This is the current authoritative handoff. The original opposite-tracker specification is archived in STAR_TRACKER_PLAN_ARCHIVE_2026-10-15.md and does not override the user's later mounting rules or rate waiver.

## Current state: implemented and independently validated

The user initially requested a plan and a feasible setup, then explicitly authorized implementing it while away. Both stages are now implemented:

1. A fixed-flight optimizer searches four independent top-mount angles.
2. A joint CasADi/IPOPT model includes four global mounting angles, all original flight variables, selected-head geometric constraints and dimensional update cadence.

The accepted combined result is results/star_tracker_coupled_uprange.json, with status numerically_validated_coupled and actual IPOPT status Solve_Succeeded. The same-mount coupled fixed-mass reproduction also returns Solve_Succeeded. The accepted workflow was reproduced a second time from the numeric caches.

No remaining work is required to demonstrate this baseline case. Next session should begin by reading the accepted report and the limitations below, rather than reimplementing the old plan or restoring antipodal trackers.

## Verified combined setup

Two fixed directions shared by both hops:

| Quantity | Tracker 1 | Tracker 2 |
|---|---:|---:|
| Elevation above body XY | 72.66012644 deg | 54.93565637 deg |
| Tilt from body +Z | 17.33987356 deg | 35.06434363 deg |
| Azimuth from +X toward +Y | 0 deg, toward +X | 180 deg, toward -X |
| Body boresight | [0.298039243834, 0, 0.954553617737] | [-0.574495988931, 0, 0.818507396852] |
| Lowest ray of the 20 deg FOV above XY | 62.66012644 deg | 44.93565637 deg |

Separation is 52.40421719 deg, not 180 deg. Both trackers and their full FOVs point above the top plane. Mounting positions are not designed.

Combined sizing model: 3325 scalar variables, 9192 constraints, 1134 assigned update events and 8 unique dimensional gap inequalities after deduplicating repeated spacings. The four mount angles are free global variables, not per-node pointing commands.

Results:

- Wet mass 3766.684646 kg, unchanged from the validated baseline (reported difference 0.0 kg).
- 567 enforced and independently checked actual updates on each hop.
- Maximum actual update gap 0.900000 s on both hops, under the required 1 s.
- Last-update age at block end: 0.417703 s outbound, 0.648017 s return.
- Minimum signed celestial margin at accepted events: 10.060225 deg outbound, 10.092590 deg return.
- Independent chained touchdown propagation errors: 2.434 mm outbound, 3.120 mm return.
- Same-mount coupled fixed-mass reproduction: loaded/used propellant 1.1000000008, final mass 1706.062242 kg.
- Both dense hazard-camera windows, thrust checks, selected fixed-gate divert, exact assigned tracker events and independently refined flexible tracker schedules pass.

### Explain the zero iterations accurately

IPOPT accepted the primal/dual warm start with zero additional iterations. The baseline minimum-mass trajectory was already feasible for the new tracker constraints with this pair. The combined constraint residual at the warm start is about 7.2e-13; recovered size-mode stationarity residual is about 8.7e-9. The mounting angles therefore stayed at their feasible seed values.

This is a real solve/certification of the combined NLP, not merely the separate post-analysis. However, it does NOT demonstrate reliable recovery from an arbitrary infeasible initial guess or a newly changed mission. The mass objective alone does not pick a unique most-robust mount. The separate four-angle search below is available for that design preference. Do not describe zero iterations as movement to a newly discovered trajectory/mount, and do not claim a global optimum or universal zero mass penalty.

## Current user requirements

| Item | Rule |
|---|---|
| Tracker count | Two first; the verified case does not need a third |
| Relative direction | Independent; no opposite-boresight, equal-elevation or fixed-azimuth symmetry requirement |
| Location | On top of hopper; exact locations/CAD not provided |
| Mount angles | One fixed elevation and azimuth per head, shared across both hops |
| Sun / Earth exclusions | Each 23.5 deg, boresight to apparent center |
| Circular FOV | 20 deg full cone, 10 deg half angle |
| Moon limb | Entire FOV outside apparent lunar disk, 0 deg extra margin |
| Update requirement | Either head can supply each event; maximum gap and boundary ages <=1 s |
| Coverage | One uninterrupted ascent+coast block per hop; no reset at that join |
| Other phases | Rise, braking, observation, terminal and surface stay are outside tracker coverage by default |
| Initial estimate | Age zero at each block start |
| Measurements / acquisition / switching | Instantaneous, zero delay |
| Rate gate | Disabled per user: enforce_body_rate is false |
| Attitude control | Assumed adequate by the user; not simulated or validated |
| Existing vehicle/camera limits | Preserved; do not lower the vehicle slew limit to 6 deg/s |
| Star availability | Clear cone assumed to contain enough usable stars |
| Redundancy | No requirement to survive either head failing, or for both heads to be simultaneously usable |

The user's +X/-X example is not a required azimuth symmetry. The production search permits all four angles to vary within configured bounds. Head 2 is never obtained by negating head 1 in independent_top mode.

## Body frame and obstruction proxy

Body +Z follows thrust/body-up. Body +X is the projected Moon-fixed downrange heading; +Y = +Z cross +X. For elevation e above body XY and azimuth beta from +X toward +Y:

    b_B = [cos(e)*cos(beta), cos(e)*sin(beta), sin(e)]

This is a body-frame condition, not a positive Moon-fixed/inertial Z condition at the lunar south pole.

An ideal opaque top-plane proxy conservatively requires each entire FOV above body XY:

    elevation >= full_FOV/2 + top_plane_fov_clearance_margin
    configured plane margin = 0 deg
    configured elevation bounds = [10, 90] deg for each head

The user prohibited downward boresights. Full-FOV plane clearance is our explicit additional proxy, not a supplied detailed hardware limit. The accepted pair has ample clearance. There is no vehicle CAD, optical-head height, baffle, structure/plume or mutual-obstruction model; do not invent physical mounting coordinates.

The new independent_top configuration has antipodal false. Historical antipodal mode remains explicit for old comparisons only. Pair-label exchange is an equivalence; sign negation is not.

## Fixed-flight mounting searches and seeds

scripts/check_top_mount_feasibility.py is a cache-only two-elevation search with azimuths fixed at +X/-X. It tested 407 pairs and found the verified seed above. Its separate robust schedule has about 49.4165 deg extra celestial margin and 511 updates per hop. Use robustness_blocks from that report for a high-clearance schedule, not the arbitrary first-head preference in its nominal schedule.

scripts/analyze_star_trackers.py is the production FOUR-angle search: feasible seed + perturbations + deterministic multistart + bounded derivative-free Powell refinement. It passes both hops, with a best tested pair:

- Elevations: 72.836365 / 54.737005 deg above XY.
- Azimuths: 347.662857 / 180 deg.
- About 49.6143 deg independently validated extra celestial margin.

This is a robustness-oriented fixed-flight local design search, not a four-dimensional global proof. The mass-minimizing joint case retains the simpler verified +X/-X seed, which is already feasible.

## Joint model and solver details

hopper/star_tracker_coupled.py subclasses the unchanged baseline Problem. TrackerProblem adds four global mount variables and each chosen head's Sun/Earth/Moon constraints at labeled events. The original wet-mass objective is preserved.

State interpolation at each event matches Hermite-Simpson; thrust controls interpolate linearly. Powered full frames prescribe roll. Coast uses a differentiable symbolic shortest quaternion interpolation between ascent-end and braking-start inertial frames, with shared endpoint calculations per coast. Zero coast thrust is never normalized. Symbolic/numeric rotation agreement, small-angle derivatives and near-pi cases are tested.

The initial event schedule uses 0.9 s gaps and 10 deg extra geometric margin as a seed choice. Actual acceptance remains the user's <=1 s and original exclusions. Tracker identities and phase/cell assignments are frozen for the current LOCAL NLP; states, controls, durations and mounting angles remain free. Event times include cumulative dimensional phase time and update when durations change. Gap constraints include first latency and final estimate age. No fractional mixture of heads is allowed.

Cold dual starts were impractically slow; restarting the nonlinear baseline from its solved state also failed to recover multipliers reliably. The final pipeline instead solves sparse LINEAR KKT stationarity equations with sign/complementarity-compatible multiplier bounds using SciPy HiGHS. This does not move the flight path and is only an IPOPT warm-start estimate. A simple distance-only active-set least-squares recovery was poorly conditioned and should not be restored.

Numeric multiplier caches are tied to effective baseline identity, mode and the exact scaled primal state. They are not accepted trajectory solutions. IPOPT must still return success and every independent acceptance check must pass.

The coupled solver keeps 1e-8 constraint feasibility (from configured tolerance), permits stated 1e-6 acceptable local optimality, has a 600 s CPU safeguard and logs progress every 100 iterations. The accepted runs actually return Solve_Succeeded. Very small warm-start bound/slack pushes preserve the already feasible point; do not silently loosen physical/event acceptance thresholds.

## Acceptance, artifacts and commands

A report is accepted only if its status is numerically_validated_coupled. An implemented model, successful multiplier LP, verification_incomplete report or failed IPOPT run is not sufficient.

Acceptance includes:

1. Exact optimized event times/head labels re-evaluated on independently integrated positions.
2. A separately reanalyzed flexible schedule with finer sampling, individual roots, signed margins and full-frame rate cross-checks.
3. Full flight repropagation, hazard-camera checks and thrust consistency.
4. The selected fixed-gate terminal solve (tracker phases intentionally do not include terminal).
5. A coupled fixed-mass sizing reproduction with the SAME fixed mount, event/cell assignments and active tracker requirements.

Important artifacts:

- results/star_tracker_coupled_uprange.json -- accepted combined result, exact events, zero-iteration flags and reproduction.
- results/star_tracker_coupled_trajectory_uprange.npz -- full combined numeric arrays.
- results/star_tracker_coupled_uprange_coverage.png / _mount.png -- accepted-event diagnostics and actual two upward vectors.
- results/star_tracker_independent_top_uprange.json and figures -- production fixed-flight four-angle search.
- results/star_tracker_top_mount_feasibility_uprange.json / .png -- initial feasible +X/-X seed and full-FOV plane diagram.
- results/star_tracker_trajectory_uprange.npz -- validated baseline flight cache.
- results/star_tracker_duals_size_uprange.npz and star_tracker_duals_maximise_margin_uprange.npz -- numeric multiplier guesses only.
- Old star_tracker_uprange reports/figures -- superseded antipodal design with a downward companion; historical only.

Reproduce from repository PowerShell:

    .\.venv\Scripts\python.exe -B scripts/run_tracker_coupled.py
    .\.venv\Scripts\python.exe -B scripts/analyze_star_trackers.py
    .\.venv\Scripts\python.exe -B scripts/check_top_mount_feasibility.py
    .\.venv\Scripts\python.exe -B -m pytest -q -p no:cacheprovider

The coupled sizing NLP took about 17-18 s in the verified runs; the full workflow also includes independent checks and a second fixed-mass NLP. Preserve original roundtrip JSON/figures. Do not reconstruct full trajectories from summary JSON or bypass stale cache checks. All NPZ loading uses allow_pickle=False.

Testing: the final full suite passes 114 tests in 202.17 s, including original round-trip integration, independent mounts, symbolic quaternion/event agreement, dimensional cadence and multiplier-LP/cache checks. The completed coupled workflow was also reproduced from its numeric caches after the final code/report changes.

## Geometry assumptions retained

Synthetic I is aligned with Moon-fixed F at first launch, not a real calendar epoch or J2000:

    Earth_I = [1, 0, 0]
    Sun_I = [0.05059294007671333, -0.9987193571841863, 0]
    direction_F(t) = Rz(-moon.omega*t) @ direction_I

Directions are inertially frozen, not fixed in F. Use cumulative seconds, infinitely distant point sources and zero surface stay. Exclusions remain active for occulted bodies. Powered roll and coast rotation are prescribed rather than controlled attitude trajectories. Full body rates above 6 deg/s are diagnostic under the explicit user waiver.

## Source map

| File | Responsibility |
|---|---|
| config.yaml; hopper/tracker_params.py | Independent-top mode, two bounds/seeds, top-plane proxy, rate waiver |
| hopper/star_tracker.py | Numeric frames, actual head geometry, four-angle search, intervals/events, independent checks |
| hopper/star_tracker_coupled.py | Four global angles, symbolic event/coast geometry, dimensional cadence, exact-event validation |
| scripts/run_tracker_coupled.py | Sparse multiplier LP/cache, joint solve and complete acceptance workflow |
| scripts/analyze_star_trackers.py | Production fixed-flight search and shared report/plot helpers |
| scripts/check_top_mount_feasibility.py | Initial two-elevation seed search, not a full flight solve |
| hopper/tracker_cache.py | Versioned numeric trajectory cache and stale identity checks |
| hopper/problem.py; hopper/phases.py; hopper/transcription.py | Original flight formulation and discretization |
| hopper/validate.py; scripts/run_roundtrip.py | Preserved flight/sensing/divert validation |
| tests/test_star_tracker.py; tests/test_star_tracker_coupled.py | Mounts, visibility/cadence, symbolic interpolation, coupling and multiplier LP/cache |

## Future changes, if requested

For different missions/geometry, first run the independent mounting search and validate a feasible seed. If a fixed event assignment becomes restrictive, reanalyze/update heads and cells in an outer loop. Do not conclude no physical installation exists from one failed local solve. Cold-start robustness remains less demonstrated than this certified feasible warm-start case.

A most-robust joint mount can use a separate secondary objective after retaining the primary optimal mass. The mass objective alone admits many feasible mount pairs. Do not add an undocumented mount-separation or redundancy requirement. A third tracker is optional only if the two-head setup becomes inadequate for a new case.

Real ephemerides, surface-stay timing, CAD/obstructions, hardware exposure/acquisition, star catalogs, navigation accuracy and actuator/attitude-control feasibility are separate scope additions. They are not established by the current results.

Inspect Git status before further edits: all current source/results/documentation are uncommitted; no user changes were discarded. Do not revert to b2 = -b1, restore the waived rate gate, confuse fixed-flight analysis with joint convergence, or claim the initial mount changed when the accepted warm start required zero iterations.

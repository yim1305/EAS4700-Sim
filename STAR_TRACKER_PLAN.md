# Two-star-tracker mounting and intermittent-coverage implementation plan

Prepared: 2026-10-15. Status: planning document only; the star-tracker feature has not been implemented.

## 1. Purpose and next-session starting point

Find a fixed mounting orientation for two identical, exactly opposite star trackers on the lunar hopper. During ascent and coast on both hops, either tracker may supply an attitude update. Both trackers may be unavailable between updates, provided the interval between usable updates does not exceed 1.0 second.

Keep the first implementation small: use the existing translational trajectory and a prescribed body-orientation history, search the two mounting angles, and independently validate an update schedule. This is a mounting-design optimization on a fixed flight path. It does not yet constrain or change the flight-path optimizer. Section 12 describes how to add actual trajectory constraints afterward if needed.

Next session should start by reading this document, `README.md`, and the relevant source files listed in section 13. Implement the fixed-trajectory analyzer first. There is enough information below to begin without asking again for the supplied numerical values. Assumptions are identified explicitly so they can be replaced when hardware definitions or a mission epoch become available.

The user requested documentation in this session, not implementation. Do not interpret this file's existence as evidence that tracker constraints, mounting results, or coverage validation already exist.

## 2. User requirements and numerical inputs

| Item | Value / requirement | Origin |
|---|---|---|
| Number of trackers | 2 | User |
| Relative boresight directions | Exactly 180 degrees apart | User |
| Sun exclusion | 23.5 deg | User |
| Earth exclusion | 23.5 deg | User |
| Moon-limb exclusion margin | 0 deg | User |
| Tracker field of view | Circular cone, 20 deg full angle; 10 deg half-angle | User |
| Maximum tracking slew rate | 6.0 deg/s, inclusive | User |
| Update cadence | At least one usable attitude update every 1.0 s | User |
| Continuous visibility | Not required | User |
| Both trackers simultaneously valid | Not required | User |
| Mounting orientation | To be found; one fixed design shared by both hops | User intent |
| Sun/Earth directions | Assumed geometry, clearly documented for presentation | User authorization |

The user described seeing stars/constellations periodically. In this simplified model, that means an attitude update is available whenever one tracker passes the geometry and rate tests. It does not mean a particular star must stay in view.

### Provisional definitions where the supplied numbers are ambiguous

Use these defaults to begin, record them in all results, and expose them in configuration:

1. **Sun and Earth:** 23.5 deg is the minimum angle from the tracker boresight to the apparent body-center direction. Do not automatically add the 10 deg FOV half-angle. A future hardware definition may instead specify an FOV-edge exclusion; that alternative would require 33.5 deg center separation for point-source directions under this convention.
2. **Moon limb:** 0 deg is the minimum clearance between the edge of the entire tracker FOV and the apparent lunar limb. Consequently, the boresight must clear the lunar limb by at least 10 deg. Zero margin does not permit the FOV to overlap the Moon. If the intended hardware definition is boresight-to-limb instead, implement that as an explicit alternate convention, not a silent change.
3. **Slew:** interpret 6 deg/s as the magnitude of the spacecraft's inertial angular velocity while an update is taken. This includes roll. A boresight-only rate can underestimate motion that rotates the star image about its center.
4. **Acquisition/exposure/switching:** assume instantaneous usable measurements and zero switching delay in version 1. No exposure or reacquisition duration was supplied. The 1 s cadence is not an exposure duration. Hardware dwell requirements can be added later.
5. **Star availability:** a clear cone is assumed to contain enough usable stars. No star catalog, limiting magnitude, recognition probability, or illumination/noise model is included.
6. **Mount restrictions:** assume unobstructed optics and unrestricted pair-axis orientation. Do not claim a physically installable design until vehicle structure, plume, and harness constraints are supplied.

These definitions are modeling assumptions, not verified statements about a selected tracker product.

## 3. Scope and phase timing

Use one coverage block per hop consisting of the phase named `ascent` followed immediately by `coast`:

- Outbound: `ascent`, `coast`.
- Return: `ascent2_<branch>`, `coast2_<branch>`.

The separate `rise` phase is excluded by the initial literal interpretation of the requested ascent/coast scope. Make the phase list configurable. If the user means the entire climb, add `rise` and `rise2_<branch>` explicitly and check orientation continuity at those joins; current thrust controls can change direction across powered-phase boundaries.

Ascent and coast form a single uninterrupted coverage block. Do not reset the update clock at their boundary. Do not require tracker updates through braking, observation, terminal descent, or surface stay in version 1. Report the two hops separately for cadence, while using the same mounting design on both.

Each block begins with an assumed valid initial attitude estimate with age zero. Require the first tracker update within 1 s of block start, every subsequent gap <= 1 s, and the age of the last update at block end <= 1 s. Record that initial-estimate assumption. A block lasting >= 1 s must contain an actual tracker update. A shorter block may rely on the initialized estimate; report that case explicitly.

Use elapsed dimensional seconds for all cadence and slew calculations. Phase duration variables in `Problem` are nondimensional; multiply by `p.scales.T` before comparing with 1 s. The extracted `Solution.phases[name]['t']` is already dimensional.

## 4. Assumed Earth/Sun geometry for presentation

### Coordinate conventions

Use the existing Moon-fixed Cartesian frame F, centered at the Moon:

- +Z: lunar north rotation axis.
- +X: equator at longitude 0 deg.
- +Y: equator at longitude +90 deg.

Define a notional inertial frame I aligned with F at the first launch, t = 0. This is a synthetic study epoch, not an actual calendar date or an ICRF/J2000 realization. Assume these unit directions are fixed in I for the entire round trip:

```text
Earth direction in I: e_I = [1.0, 0.0, 0.0]
Sun direction in I:   s_I = [0.05059294007671333, -0.9987193571841863, 0.0]
```

At t = 0, Earth is at assumed sub-Earth latitude 0 deg, longitude 0 deg. The Sun is at assumed subsolar latitude 0 deg, longitude -87.1 deg. Their angular separation is 87.1 deg. This selects a near-quarter-phase illustrative geometry, rather than a date-specific celestial arrangement.

At the configured launch site (latitude -85.5 deg, longitude -87.1 deg), these assumptions put the Sun center 4.5 deg above the geometric horizon and the Earth center about 0.227435 deg above it at t = 0. These are calculated consequences of the assumed directions, not observations. Earth/Sun angular sizes are not used to define visibility in version 1.

Transform directions into the rotating Moon-fixed frame at each absolute mission time:

```text
e_F(t) = Rz(-omega_moon * t) @ e_I
s_F(t) = Rz(-omega_moon * t) @ s_I
```

Use `p.moon.omega`, currently 2.6617e-6 rad/s. Do not hold these vectors constant in F while also claiming that they are inertially fixed. The frame rotation is approximately 0.183 deg over 1200 s.

Treat both bodies as infinitely distant direction sources. Ignore spacecraft parallax and true celestial motion in this initial study. In particular, Earth's direction is deliberately frozen in I even though a real ephemeris would move it. These approximations are acceptable for a labeled assumed-geometry study, not a verified launch-window assessment.

Use cumulative elapsed time across all flight phases when transforming the directions, including phases outside tracker coverage. The current mission ignores surface-stay duration; use zero stay time and state that assumption. A real stay duration or epoch would affect the return geometry.

Apply the Sun/Earth exclusions even if a body is behind the local lunar horizon. This is a simple conservative choice. Do not silently disable exclusion tests based on occultation. A future occultation model can relax them explicitly.

### Suggested presentation wording

> This study assumes two opposite, fixed star trackers with a 20-degree full-cone field of view. A usable attitude update requires one tracker to meet the 23.5-degree Sun and Earth boresight exclusions, keep its full field of view clear of the lunar limb, and operate at no more than 6 degrees per second spacecraft slew. Updates may come from either tracker with gaps no greater than one second. Earth and Sun directions use a documented synthetic geometry, with initial sub-Earth coordinates (0 degrees latitude, 0 degrees longitude) and subsolar coordinates (0 degrees latitude, -87.1 degrees longitude). The mounting result is conditional on that geometry and the assumed vehicle orientation history.

## 5. Existing model and the orientation extension

The existing model has 7 translational/mass states, powered controls `[T_x, T_y, T_z, Gamma]`, and no attitude state. Powered body-up is represented by thrust direction. Coast controls are zero, so thrust cannot define coast orientation. Existing `TiltRate` limits thrust-direction changes, not full inertial body angular velocity.

Do not normalize zero coast thrust. Do not use the velocity vector as body orientation without declaring a new attitude policy.

### Powered body frame

For the first analyzer, prescribe a right-handed body frame using the existing fixed Moon-frame heading `h = p.mission.downrange`:

```text
z_B_in_F = T_F / norm(T_F)
x_raw    = h - dot(h, z_B_in_F) * z_B_in_F
x_B_in_F = x_raw / norm(x_raw)
y_B_in_F = cross(z_B_in_F, x_B_in_F)
C_BF     = columns(x_B_in_F, y_B_in_F, z_B_in_F)
C_BI(t)  = Rz(omega_moon * t) @ C_BF(t)
```

`C_BF` maps body coordinates to Moon-fixed coordinates; `C_BI` maps body coordinates to the synthetic inertial frame. Verify orthonormality and determinant +1.

This uses the same projected heading idea as `hopper/sensor.py`, but define the body convention explicitly: the existing camera looks along body -Z, with image-X along body +X and image-Y along body -Y under this right-handed convention. Do not change camera behavior to implement the tracker analyzer.

If `norm(x_raw)` approaches zero, the prescribed roll is singular. Flag the case and use a documented continuous frame-transport policy or another fixed reference chosen for the whole affected segment. Do not independently switch reference axes at each sample: that can introduce artificial roll jumps. A singular frame invalidates a claimed coverage pass until resolved.

The mounting result depends on this prescribed roll. It is not an optimized free-roll result.

### Coast body frame: prescribed smooth rotation

Use a single shortest-path quaternion SLERP over the coast in inertial coordinates, from the full body frame at the end of ascent to the full body frame at the start of braking:

```text
C0 = C_BI(ascent end)
C1 = C_BI(braking start)
C_BI(coast t) = SLERP(C0, C1, fraction through coast)
```

Use `scipy.spatial.transform.Rotation` and `Slerp` for the numerical analyzer. Match quaternion signs for the shortest rotation. Handle a nearly zero endpoint rotation stably; flag an exact 180 deg ambiguity and choose/report a deterministic interpolation branch.

This gives continuous coast orientation and matches both adjacent powered orientations without adding translational states or requiring a torque model. It is a prescribed coast slew, not a solved attitude-control maneuver. It ignores torque, angular acceleration, actuator effort, and attitude-control propellant. Do not imply these capabilities have been validated.

For a constant-speed shortest SLERP, inertial body angular speed is the endpoint rotation angle divided by coast duration. If that exceeds 6 deg/s, no tracker mounting can make the prescribed coast motion trackable under the full-body-rate interpretation.

### Inertial angular rate

Evaluate the full body orientation rate in I. For dense numerical validation, use the magnitude of the relative rotation vector between nearby orientations divided by their time separation. Refine the time step to check convergence. Use one-sided limits at interpolation knots; do not count an update at an unresolved orientation jump.

For the instantaneous measurement approximation, require rate <= 6 deg/s at accepted update events. Do not automatically lower the vehicle's existing 10 deg/s thrust-direction cap everywhere. Faster motion in a visibility gap is allowed if the update-cadence and existing flight constraints still pass.

For fixed prescribed attitude, the full body angular rate is independent of tracker mounting and the same for both trackers. A rate-induced outage cannot be repaired by selecting the opposite tracker. Boresight rates may be reported separately but must not replace the full-body-rate gate.

## 6. Fixed mounting design variables

Represent tracker 1's fixed unit boresight in body coordinates by polar tilt alpha from +Z and azimuth beta from +X toward +Y:

```text
b1_B = [sin(alpha)*cos(beta), sin(alpha)*sin(beta), cos(alpha)]
b2_B = -b1_B
```

Use radians internally and degrees in configuration/results/presentation. Start with alpha in [0, 90] deg and beta in [0, 360) deg. Because the two trackers are identical and unlabeled, every opposite pair has a representative in this hemisphere. Equatorial directions and pole azimuths contain duplicates; handle them deterministically and do not mistake them for distinct designs.

At time t:

```text
bj_I(t) = C_BI(t) @ bj_B
bj_F(t) = C_BF(t) @ bj_B
```

The mount is global: never optimize a different alpha/beta at each node, phase, or hop. Opposite boresights remain opposite after every coordinate transformation. These are boresight mounting angles, not the spacecraft's attitude angles.

## 7. Geometry, validity, and angular margins

For each tracker j, use vectors in one common frame. Let `r_F` be absolute spacecraft position, `R` the lunar radius, and `n_F = -r_F/norm(r_F)` the direction toward the lunar center.

The Moon's apparent angular radius is:

```text
rho_M(t) = asin(R / norm(r_F(t)))
```

Validate altitude before evaluating; clip only roundoff-sized excursions of the ratio into [0, 1]. A materially subsurface trajectory is an error, not something to hide by clipping.

Define angular margins:

```text
m_sun,j   = angle(bj_F, s_F) - 23.5 deg
m_earth,j = angle(bj_F, e_F) - 23.5 deg
m_moon,j  = angle(bj_F, n_F) - rho_M - 10 deg - 0 deg
m_geom,j  = min(m_sun,j, m_earth,j, m_moon,j)
```

Use numerically stable angular calculations (`atan2(norm(cross(a,b)), dot(a,b))` for numerical reports). For symbolic implementations, normalized dot-product inequalities can avoid `acos` where appropriate:

```text
dot(bj_F, s_F) <= cos(23.5 deg)
dot(bj_F, e_F) <= cos(23.5 deg)
dot(bj_F, n_F) <= cos(rho_M + 10 deg)
```

The same tracker must satisfy every exclusion simultaneously:

```text
valid_j(t) = (m_geom,j(t) >= 0) AND (body_rate(t) <= 6 deg/s)
valid_any(t) = valid_1(t) OR valid_2(t)
```

Never apply a separate `tracker 1 OR tracker 2` choice to each celestial object. That incorrect formulation could combine Sun clearance from one tracker and Earth/Moon clearance from the other even when neither tracker is usable.

Geometric tracker handovers may occur between updates. Neither tracker must remain valid for the whole block. A small explicit numerical tolerance is acceptable, but report the raw signed margins; do not conceal a substantive negative margin.

## 8. The 1-second update requirement

This is a temporal availability requirement, not continuous visibility and not a requirement to look at a single star.

Let accepted events have times `u_1, ..., u_N` in one active block `[a, b]`. Each event selects one tracker that is valid at that time. Require:

```text
u_1 - a <= 1.0 s
u_(k+1) - u_k <= 1.0 s, for every consecutive event pair
b - u_N <= 1.0 s
```

Use the boundary initial-estimate convention from section 3. Report first-update latency, largest inter-update gap, final update age, and maximum of these three metrics. Reset only between the two required coverage blocks, not at ascent/coast joins.

Do not merely require one valid sample in each disjoint one-second bin: two events at opposite ends of adjacent bins can be almost two seconds apart. Do not require visibility at every collocation node: nodes are not spaced at one second, and continuous node constraints would be stronger than requested.

### Version 1 scheduling algorithm

Because the flight path and attitude are fixed, availability can be analyzed directly without binary decision variables:

1. Evaluate both trackers and body rate on a dense time grid, initially <= 0.05 s spacing, and include all phase/control interpolation knots and block endpoints.
2. Locate validity transitions by refining the roots of individual signed angular/rate margins. Sample refinement must also search for short valid/invalid intervals that a coarse grid could miss. Sampling alone is not a proof of continuous-time coverage.
3. Build valid time intervals for each tracker, then their union. Preserve tracker labels within intervals for selecting actual events.
4. Construct an explicit event schedule. Starting with the initialized estimate at block start, choose a valid time at or before the next one-second deadline; choosing the latest available valid time is a useful greedy rule for offline scheduling. Continue until the block end is covered. Do not count a virtual initial estimate as a physical tracker measurement.
5. Check every actual scheduled event independently, including its selected tracker, local rate, and all exclusion margins. Check event gaps and boundary ages explicitly.
6. If a deadline cannot be met, report the interval with no valid measurement opportunity, its cause, and the nearest available events. Distinguish numerical unresolved coverage from demonstrated infeasibility.

The existence of suitable valid intervals plus zero acquisition/switching delay makes scheduling simple. Do not convert a broad average availability percentage into a cadence pass. A design can have excellent overall availability and still have a multi-second outage.

As a simpler optional diagnostic, test a fixed 1 Hz event grid. Clearly label this as more restrictive than flexible update timing; failure on that fixed grid does not prove the user's maximum-gap requirement is infeasible.

## 9. Mounting-search objective

First seek a mounting that passes the cadence requirement on both hops. Then improve robustness by increasing angular clearance at available measurement opportunities. Do not optimize the minimum geometric margin over all times, because that would encourage continuous visibility unnecessarily.

A useful robust objective is:

1. For trial mount `(alpha, beta)` and trial extra angular margin `m >= 0`, count tracker j as geometrically eligible only if each of its three angular margins is >= m. Keep the 6 deg/s rate gate unchanged.
2. Test whether both hop blocks still admit a valid <= 1 s update schedule.
3. Find the largest feasible m by bisection for this mount, then maximize that m over mounting candidates.

For mounts that fail even at m = 0, rank diagnostic candidates by their worst update gap and failed-block count. A failed local search is not proof that no mounting exists.

Begin with a coarse hemisphere grid (for example 10 deg increments with explicit pole handling), retain several promising candidates, and refine around them or use a bounded derivative-free search such as SciPy differential evolution. The feasible-region switches are nonsmooth; do not assume ordinary smooth gradient optimization handles tracker selection reliably. Reuse cached orientation, position, and celestial-direction arrays for every mount candidate.

The two-tracker antipodal symmetry and pole degeneracy mean a unique angle pair is not guaranteed. Report a best tested design, its search resolution/local refinement, and equivalent pair orientations. Do not claim a globally optimal physical installation.

## 10. Proposed configuration contract

The following is a proposed future configuration block, not an edit to the current `config.yaml`:

```yaml
star_tracker:
  enabled: false
  n_trackers: 2
  antipodal: true
  full_cone_fov_deg: 20.0
  sun_exclusion_deg: 23.5
  earth_exclusion_deg: 23.5
  bright_body_exclusion_reference: boresight_to_center
  moon_limb_margin_deg: 0.0
  moon_limb_reference: fov_edge
  tracking_body_rate_max_deg_s: 6.0
  update_gap_max_s: 1.0
  acquisition_time_s: 0.0
  switching_delay_s: 0.0
  initial_estimate_age_s: 0.0
  required_phase_kinds: [ascent, coast]
  body_roll_policy: projected_downrange
  coast_attitude_policy: inertial_endpoint_slerp
  mount_search:
    tilt_bounds_deg: [0.0, 90.0]
    azimuth_bounds_deg: [0.0, 360.0]
    coarse_step_deg: 10.0
  assumed_geometry:
    label: synthetic_launch_aligned_geometry_not_ephemeris
    earth_direction_I: [1.0, 0.0, 0.0]
    sun_direction_I: [0.05059294007671333, -0.9987193571841863, 0.0]
    directions_frozen_in: I
    surface_stay_s: 0.0
  analysis:
    sample_step_s: 0.05
    transition_time_tolerance_s: 0.001
```

Validate units, angle ranges, exactly two opposite trackers, positive update gap and rate, normalized nonzero direction vectors, valid mount bounds, and supported convention/policy names. The raw zero-exposure/switching assumptions must appear in output. Do not introduce nonzero dwell times under this configuration unless scheduling logic is extended accordingly.

Parse degree inputs into radians once, as the existing parameters do. For backward compatibility, a missing tracker block should disable the feature and preserve the existing solver behavior.

## 11. First implementation deliverables and execution order

1. Add a separate tracker-parameter dataclass and optional config parsing, without changing trajectory objectives or constraints when disabled. Consider a defaulted optional field at the end of `Params` so existing construction remains compatible; inspect all construction sites before editing.
2. Add numerical frame, orientation, rate, exclusion-margin, availability-interval, and schedule functions in a dedicated module such as `hopper/star_tracker.py`. Keep these separate from the bottom-mounted hazard camera's square-FOV model.
3. Obtain a fully validated baseline `Solution` once, using the existing run/solve interfaces. The runner currently returns `(sol, summary)` from `main(..., show_plots=False)` and saves summary JSON/PNGs, not full trajectory arrays. Existing JSON alone is insufficient for this analysis. Do not reconstruct an optimized trajectory from the summary table.
4. Optionally save a versioned NPZ trajectory cache with configuration/branch metadata so mounting searches do not rerun IPOPT. Use named numeric arrays, no pickle/object deserialization, and verify stale-cache detection. If caching is not needed initially, retain the `Solution` in memory for the whole search.
5. Add `scripts/analyze_star_trackers.py` to run the mounting search and emit a report. Solve once, search many mounts. Preserve existing results; use separate tracker result filenames.
6. Search the same global mounting against both hop coverage blocks. First precheck rate-only opportunity intervals: long outages from the prescribed full-body rate cannot be repaired by mounting changes.
7. Validate the winning mount on a finer grid with transition refinement and an independently re-evaluated event schedule. If margins/gaps do not converge, label the result unresolved and refine further.
8. Save `results/star_tracker_<branch>.json` and presentation plots. Document an exact reproduction command once the script exists. Do not claim an unimplemented command has already been tested.
9. Update README/NOTES with the actual scope: mounting analysis on a fixed trajectory versus any later coupled constraints. Keep the base mission's Lambert seeding, objectives, and validation behavior intact.

### Required report contents

- Mount tilt/azimuth, both fixed body-frame boresight vectors, and 180 deg separation check.
- All user values, adopted reference conventions, body/coast policies, and geometry assumptions.
- Baseline branch/config identity, trajectory provenance, and timing/sampling settings.
- Per-hop cadence pass/fail, first-update latency, worst update gap, final estimate age, event count, and tracker selections.
- Worst angular margins at accepted events, worst accepted-event body rate, and limiting exclusion/phase/time.
- Both trackers' unavailable intervals, availability fraction as a secondary metric, and worst gap with its causes.
- Best robust angular margin, search method/resolution, and numerical validation tolerances.
- A plain statement that this analyzes usable attitude-update opportunities, not position fixes, navigation covariance, actuator feasibility, or measured tracker performance.

Suggested figures: body-frame view of opposite mounting directions; angular margins for each tracker; body rate with the 6 deg/s line; validity bands with actual update-event markers; and update age versus time with a 1 s limit. Use seconds for cadence detail even if trajectory plots use minutes.

## 12. Optional follow-on: actual trajectory constraints

Only use this stage when the goal includes modifying the optimized trajectory for tracker feasibility or quantifying its propellant/mass cost. A successful fixed-trajectory mount search alone must not be described as adding tracker constraints to IPOPT.

Retain one mounting design across both hops. Powered orientation may still follow thrust with prescribed roll. Coast needs an explicit orientation policy or new attitude-direction/orientation variables; it cannot use zero thrust. Full quaternion/torque dynamics are not required for a geometric feasibility model, but generic off-axis mounts require a full frame, including roll.

A manageable smooth local subproblem is to freeze a feasible measurement schedule's tracker identities and each event's phase/collocation-cell assignment from the analyzer, and impose all geometry/rate constraints on that chosen tracker at each event. Fix or carefully parameterize event locations within their assigned cells. Enforce actual dimensional event gaps <= 1 s and boundary ages as phase durations change. This avoids a binary tracker decision at every node, but is conditional on the frozen schedule/assignments and may miss better solutions. Reanalyze and update assignments in an outer loop if needed.

At an event inside a collocation cell, use a trajectory interpolation consistent with the existing transcription (Hermite-Simpson state interpolation and linearly interpolated controls). Do not treat arbitrary node fractions as one-second spacing. Evaluate Sun/Earth geometry at cumulative event time, not separately reset phase time.

Do not pass SciPy SLERP directly into CasADi. A coupled solver needs a differentiable symbolic orientation interpolation with stable small-angle/near-pi treatment, or a different explicit coast attitude parameterization. Handle full inertial angular rate consistently. Flag unresolved orientation discontinuities; do not manufacture zero-time slews.

If mounting becomes an IPOPT decision, alpha/beta are global scalars (or use one global unit axis). Keep the original mission objective, wet-mass sizing or final-mass maximization, with tracking as a feasibility requirement. Add robustness as a separate second-stage objective or stated small secondary objective; do not silently change the meaning of the primary sizing result.

An exact unrestricted either-tracker/event-time formulation is hybrid/disjunctive. Do not relax binary selections to continuous weights and report their fractional mixture as a usable tracker. Also avoid an optimistic smooth maximum that can claim feasibility when neither tracker passes. Begin with the fixed-schedule local approach rather than introducing a mixed-integer solver by default.

Re-run the fixed-mass sizing reproduction with the same mount, geometry, and tracker requirement if tracker constraints become part of the sized problem. Preserve independent propagation, dense hazard-camera validation, thrust checks, and fixed-gate divert verification. A standalone terminal solve lies outside the requested tracker phases; do not add tracker conditions there accidentally.

## 13. Source-file map

| Current file | Relevant behavior / proposed work |
|---|---|
| `config.yaml` | Physical inputs and mission settings; add optional tracker block only during implementation |
| `hopper/params.py` | Frozen dataclasses, degree conversion, config validation, and optional tracker settings |
| `hopper/problem.py` | `Solution`, phase construction, guess injection, and optional later coupled constraints/global mount variables |
| `hopper/phases.py` | Phase containers; coast `U` is zero, not an attitude variable |
| `hopper/constraints.py` | Existing thrust-direction `TiltRate`; optional later event constraints must include prescribed roll/inertial-rate treatment |
| `hopper/sensor.py` | Existing camera frame convention; use as a reference without changing camera behavior |
| `hopper/transcription.py` | Match interpolation to Hermite-Simpson in later event constraints |
| `hopper/postprocess.py` | `chain(branch)` defines flight order and cumulative timing |
| `hopper/validate.py` | Independent propagation/sensor checks; add or call a separate tracker report |
| `hopper/plotting.py` | Existing figures; tracker plots may live in a separate helper |
| `hopper/initial_guess.py` | Lambert-seeded translational guesses; preserve in first implementation |
| `scripts/run_roundtrip.py` | Solve, validation, and returned `Solution`; possible future tracker-report hook |
| `tests/conftest.py` | Existing parameter fixtures and override helper |
| `tests/test_roundtrip.py` | Integration checks; extend only if trajectory constraints are implemented |

Proposed new files: `hopper/star_tracker.py`, `scripts/analyze_star_trackers.py`, and `tests/test_star_tracker.py`. Inspect actual repository status and any applicable AGENTS.md before editing next session; this document is not a substitute for those instructions.

## 14. Meaningful tests and acceptance criteria

### Geometry and frames

- Exactly 23.5 deg Sun/Earth separation is accepted within stated numerical tolerance; slightly inside is rejected.
- A 20 deg full cone is treated as a 10 deg half-cone.
- At the lunar surface, the FOV-edge convention requires boresight/nadir separation >= 100 deg for a 0 deg limb margin. At altitude, the apparent lunar radius decreases as `asin(R/r)`.
- A case in which different trackers separately clear different bodies, but neither clears all bodies, is rejected.
- Opposite mounts remain opposite in every frame and share the same full-body rate.
- Body frames are right-handed and continuous where required. Inertial-to-fixed celestial transforms use the correct rotation sign.
- A synthetic inertially fixed attitude has zero inertial rate even though its Moon-fixed representation rotates.
- Coast SLERP matches both endpoint frames and has the expected constant inertial angular speed.

### Timing and tracking

- Tracker 1 and tracker 2 may alternate accepted updates; simultaneous visibility is unnecessary.
- Noncontinuous visibility with a valid <= 1 s schedule passes.
- A gap greater than 1 s fails even if total availability is high or every disjoint one-second bin contains an event.
- An outage spanning ascent/coast is counted across the boundary rather than reset.
- Initial latency and final estimate age are checked, including blocks shorter than 1 s.
- At an update event, 6.0 deg/s passes within tolerance; a rate above it fails even with clear geometry.
- Motion above 6 deg/s between accepted events does not by itself violate tracker cadence, provided the original vehicle constraints and update timing pass.
- An unresolved attitude jump or frame singularity cannot produce a claimed valid event.
- Independent finer sampling and transition refinement agree on worst gaps and limiting margins to the reported tolerance.

### Integration and reporting

- Disabled/missing tracker configuration preserves the current solver's behavior.
- One mount serves both hops; no per-node or per-hop remounting is permitted.
- Summary-only JSON is rejected as insufficient trajectory input.
- Result JSON records synthetic geometry, angle-reference conventions, zero dwell/switching assumptions, and whether the flight trajectory was actually constrained.
- No new numerical mounting angles, coverage guarantees, or mass penalties are published until a search and validation have run.

Use focused numerical tests for the tracker module first. Run the existing parameter/sensor tests after parameter parsing changes. A full round-trip integration run is appropriate when adding the analyzer hook or actual flight constraints; do not repeatedly rerun IPOPT for every unit geometry test.

## 15. Limitations and later refinements

The first result is conditional on assumed Sun/Earth geometry, prescribed powered roll, and prescribed coast motion. It assumes enough detectable stars in every geometrically clear cone, instantaneous measurements/switching, an initialized attitude estimate at each active block start, and no structural or plume blockage. It does not quantify IMU drift during the one-second gaps or show that the update cadence meets a navigation-accuracy requirement.

Potential later refinements are hardware-specific exclusion definitions, nonzero exposure/acquisition/processing times, actual ephemerides and surface stay, vehicle obstruction/plume geometry, roll optimization, attitude-actuator constraints, and robustness across multiple Sun directions/launch conditions. These are separate scope additions, not prerequisites for the initial clearly labeled mounting study.

NASA's [Guidance, Navigation, and Control overview](https://www.nasa.gov/smallsat-institute/sst-soa/guidance-navigation-and-control/) explains the general need for clear star fields and the influence of angular motion and bright-light interference. It is background only; the numerical tracker specifications above came from the user, and the proposed Earth/Sun vectors and modeling conventions are study assumptions.

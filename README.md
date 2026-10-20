# Lunar Hopper Round-Trip Optimizer

This project sizes a lunar hopper for an outbound flight of about 200 km and a return flight. It optimizes finite burns, coast time, descending hazard-camera observation, and terminal descent using Python, CasADi and IPOPT. Two independent top-mounted star trackers can be analyzed on a fixed flight path or included in a dedicated joint flight/mount optimization.

The default outbound landing target is **150 m uprange of the original target**, toward the launch site. Actual touchdown must be on the lunar surface **within 10 m of that divert target**. The return must land within 10 m of launch. The 10 m design tolerance sits inside the user's 50 m landing requirement. Only the selected divert scenario is optimized; uprange is an assumed worst case, not a proven worst direction.

Physical inputs and configurable limits live in [config.yaml](config.yaml). This README describes the current implementation. [DESIGN.md](DESIGN.md) summarizes the design, [NOTES.md](NOTES.md) records the current status, and [hopper_spec.md](hopper_spec.md) is the historical build specification.

## Latest verified result

Current settings: 10 m landing tolerance; 10 km camera range; 50 m/s downward and horizontal sensing limits; 90 x 90 degree hardware FOV with a 5 degree margin at each edge.

| Quantity | Result |
|---|---:|
| Launch wet mass | **3766.68 kg** |
| Loaded propellant | **2266.68 kg** |
| Loaded fuel / oxidizer, O/F = 1.65 | 855.35 / 1411.33 kg |
| Propellant consumed | 2060.62 kg |
| Remaining propellant after return | 206.06 kg |
| Total finite-burn Delta-v | 2407.75 m/s |
| Outbound / return Delta-v | 1236.12 / 1171.63 m/s |
| Total flight time, excluding surface stay | 1152.14 s (19.20 min) |
| Engine starts | 4 |
| Outbound decision gate | 245.01 m altitude, 20.91 m/s downward |
| Observation duration | 1.10 s on each hop |
| Outbound / return landing-target offset | 10.000 / 9.999 m |
| Main sizing solve | Solve_Succeeded, 1331 iterations, about 99 s |

The fixed-mass sizing cross-check, selected fixed-gate terminal solve, independent trajectory propagation and dense sensor checks pass. Chained propagation differs from the optimized touchdown positions by 2.4 mm outbound and 3.1 mm on return. The current full regression suite passes **114 tests**, including baseline flight integration and independent/coupled star-tracker tests.

The earlier 50 m landing-tolerance, 100 m/s sensing, zero-margin case was 3754.53 kg wet mass. The combined tightening adds about 12.15 kg; this comparison does not isolate individual constraints. Saved results: [results/roundtrip_uprange.json](results/roundtrip_uprange.json). Verification solves add runtime beyond the main solve.

## Install and run

From the project directory in Windows PowerShell:

~~~powershell
py -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe -B scripts/run_roundtrip.py
~~~

The run validates the solution, saves JSON and PNGs, and opens three Matplotlib windows. Close the windows to finish. To save without opening windows, or run checks:

~~~powershell
.\.venv\Scripts\python.exe -B scripts/run_roundtrip.py --no-show
.\.venv\Scripts\python.exe -B -m pytest -q -p no:cacheprovider
.\.venv\Scripts\python.exe -B -m pytest -q -p no:cacheprovider tests/test_params.py tests/test_sensor.py
~~~

In VS Code, choose **Python: Select Interpreter** and select this project's .venv\Scripts\python.exe. Running a different Python installation can give a missing-CasADi error. The optional positional branch argument selects a different supported case for a separate run; the default is uprange.

## Flight sequence and decision gate

Each hop has six phases and two engine starts:

| Phase | Engine | Purpose |
|---|---|---|
| Rise | On | Vertical departure to 50 m, with zero horizontal velocity |
| Ascent | On | Accelerate toward the destination |
| Coast | Off | Ballistic transfer |
| Braking | On | Reduce approach speed before sensing |
| Observation | On | Descend while observing the original target and actual touchdown point; includes processing time |
| Terminal | On | Continue from the moving decision gate to touchdown |

There is no hover phase or required stop. Rise and ascent form one burn; braking, observation and terminal form the other. The decision gate is the **end of observation**, when the terminal divert begins. Its time is optimized through the phase durations, and its altitude is free between 150 and 1000 m. The outbound gate is directly above the original target; the return gate's horizontal position is free.

After outbound touchdown, the return starts at rest at the actual landing position, with the same mass. Its launch vertical follows that position's surface normal. The surface stay has no duration or propellant consumption in the model.

## Free variables: what the optimizer chooses

| Decision variables | Where they exist | Meaning |
|---|---|---|
| Position, velocity and mass: 7 state components | Every node of all 12 phases | The complete trajectory and mass history |
| Thrust vector: 3 components, plus magnitude Gamma | Every powered node | Thrust direction and throttle; vector norm equals Gamma at nodes |
| Six phase durations per hop | 12 scalar variables | Burn/coast timing, sensing-window placement and duration, decision time and terminal duration |
| Launch wet mass | One scalar in size mode | Minimized while satisfying the propellant reserve rule |

**Actual touchdown locations are optimized endpoint states**, within 10 m of the prescribed targets. They are not pinned to the target centers. Their normals define touchdown uprightness and terminal glide-slope geometry. The outbound endpoint also defines the return launch point.

Gate position, altitude, velocity, observation start time and divert start time follow from the state and duration variables; they are not additional independent variables. Arrival velocity is optimized within touchdown limits. Acquisition minimum, processing delay, FOV, range, speed limits, reserve factor and nominal target locations are fixed inputs. There are no independent attitude, yaw, gimbal or sensor-steering variables.

With the current mesh, each hop has 10/30/20/30/12/50 intervals in rise/ascent/coast/braking/observation/terminal. Size mode has **3321 scalar decision variables**: 2212 states, 1096 controls, 12 durations and 1 wet mass. Fixed-mass mode removes the wet-mass variable.

## Active constraints

Unless specified otherwise, path constraints are imposed at collocation nodes. Sensor constraints also apply at Hermite–Simpson midpoints.

| Constraint | Applies to | Current setting |
|---|---|---|
| Equations of motion | Every phase | Central gravity, Moon rotation, thrust acceleration and propellant flow |
| Powered thrust | Powered nodes | Approximately 1112 <= Gamma <= 26688 N; norm of thrust vector = Gamma |
| Engine off | Coast | Thrust and propellant flow are zero |
| Mass floor | All nodes | Mass >= 1500 kg dry mass |
| Initial state | First launch | At launch site, zero Moon-fixed velocity, initial mass = wet mass |
| Vertical departure | Rise | Zero velocity perpendicular to launch normal; ends at 50 m altitude |
| Minimum altitude | Ascent, coast, braking | >=50 m above reference terrain |
| State continuity | Phase boundaries within a hop | Same position, velocity and mass |
| Decision gate | Observation end, both hops | Altitude 150–1000 m; nonpositive radial velocity |
| Outbound gate position | Outbound observation end | Directly above original target |
| Sensing descent speed | Observation nodes and midpoints | 0.1–50 m/s downward, using local radial vertical |
| Sensing horizontal speed | Observation nodes and midpoints | <=50 m/s in the local tangent plane |
| Sensing altitude | Observation nodes and midpoints | >=10 m; current gate floor is more restrictive during descending observation |
| Camera range and visibility | Both observed points during observation | Slant range <=10 km; points forward of camera and above the spherical horizon |
| Square FOV | Both observed points during observation | Horizontal and vertical image angles within +/-40 degrees: 90 x 90 hardware FOV minus 5 degrees per edge |
| Acquisition and processing | Each observation phase | Duration >=1.0 s acquisition + 0.1 s processing; sensing constraints hold throughout |
| Direction-rate approximation | Powered phases and adjacent powered-phase joins | Vehicle cap 10 deg/s; observation uses min(vehicle 10, sensor 30) = 10 deg/s |
| Terminal glide slope | Terminal nodes except touchdown | Height above actual landing-site plane >=tan(45 degrees) x horizontal distance to actual touchdown |
| Landing position | Both touchdowns | On the spherical surface, within 10 m Euclidean distance of selected target |
| Touchdown velocity | Both touchdown endpoints | Downward 0–1.5 m/s, horizontal <=0.5 m/s; upward arrival disallowed |
| Touchdown uprightness | Both touchdown endpoints | Thrust/body-up within 20 degrees of actual local surface normal |
| Surface linkage | Between hops | Actual landing position and mass retained; velocity reset to zero |
| Wet-mass bounds | Size mode | 1.05 x dry mass <= wet mass <=3 x configured wet mass: 1575–15000 kg |
| Reserve rule | Size mode | Loaded propellant >=1.1 x propellant consumed over the round trip |

Phase-duration bounds are rise 1–120 s, ascent 1–600 s, coast 1–2000 s, braking 1–600 s, observation 0.1–300 s, terminal 1–300 s. The sensing timing constraint raises the effective observation minimum to **1.1 s**.

The sensor does not have to track the site during ascent, coast, or the later terminal divert. No finite landing-patch radius or whole-disk imaging requirement is imposed. The two observed points are the destination's original target and the optimized actual touchdown point; on return, the original target is the launch site.

## Objectives and numerical method

In mission.mode: size, minimize launch wet mass with:

~~~text
m_final >= m_wet - (m_wet - m_dry) / reserve_factor
~~~

The reserve inequality is active in the current optimum, giving loaded/used propellant = 1.100. In maximise_margin mode, configured wet mass is fixed and final mass is maximized instead. Fuel and oxidizer are split from total propellant using the O/F mixture ratio.

Hermite–Simpson direct collocation discretizes the dynamics. Controls are linear between nodes. CasADi supplies derivatives to IPOPT; current settings are tolerance 1e-8 and at most 3000 iterations, with bound relaxation and acceptable-iteration stopping disabled. Both Solve_Succeeded and Solved_To_Acceptable_Level are accepted statuses, followed by checks. Results are local optima and can depend on the initial guess.

The starting guess uses a Lambert coast arc, rocket-equation mass estimates and Hermite burn interpolation. Gate seeds are 450 m outbound and 300 m return with the current configuration; these are guesses, not fixed gate constraints. Variables are scaled by hop distance, sqrt(distance/lunar gravity), configured wet mass and configured wet mass times lunar gravity.

## Assumptions and limits of the model

### Mission and environment

- The Moon is a sphere of radius 1737.4 km with central gravity, mu = 4.9028e12 m^3/s^2, and constant rotation 2.6617e-6 rad/s about its z-axis. The equations include Coriolis and centrifugal acceleration in a Moon-fixed frame.
- Terrain is the reference sphere with zero height. Both sites have zero elevation. No topography, rocks, slopes, terrain occlusion, gravity anomalies, Earth/Sun gravity, atmosphere or drag are included.
- Launch is at latitude -85.5 degrees, longitude -87.1 degrees; the original target is at -78.9044 degrees, the same longitude. Site locations are fixed in the Moon frame.
- One prescribed 150 m uprange divert scenario is assumed. The optimizer knows the chosen scenario before flight and can tailor its approach. There is no optimization over unknown hazard outcomes, proof of worst-direction ranking, or demonstration of late-choice reachability for other sites.
- The surface stay is ignored; it consumes no time or propellant. Contact settles the bounded touchdown velocity to zero before return, without modelling impact or landing-gear dynamics.
- The 10 m nominal landing tolerance provides geometric allowance inside the 50 m requirement. Navigation uncertainty and dispersion are not propagated into an error budget.

### Vehicle, propulsion and attitude

- Motion is a 3-DoF point-mass model with position, velocity and mass. Body-up is assumed aligned with resultant thrust; camera boresight is opposite it. No independent thrust gimbal deflection is modelled.
- There are no rotational equations, quaternions, inertia, angular momentum, torque limits or landing-contact stability calculations. The 20 degree touchdown limit applies only at touchdown.
- A 10 deg/s thrust-direction cap approximates attitude slew between powered nodes. At phase joins, the implementation allows one interval of the following phase for rotation; this is not a continuous attitude-dynamics guarantee.
- Four engines, each nominally 1112–6672 N, are represented by one equivalent continuously throttleable source from approximately 1112 to 26688 N, or off. The combined minimum represents one engine at minimum with the others off; individual engine scheduling, switching and pulsing are not simulated.
- Vacuum Isp is constant at 310 s, standard gravity is 9.80665 m/s^2, and O/F is 1.65. Throttle-dependent performance, transient fuel use, restart time and start-cycle limits are omitted. Engine input values are supplied assumptions, not independently verified specifications.
- Dry mass is fixed at 1500 kg, including structure, engines and sensors. Tank, engine and structure mass do not scale with propellant or engine changes. Configured wet mass 5000 kg supplies scaling and the initial guess in size mode; it is not the reported optimized mass.
- The reserve factor 1.1 applies to total round-trip propellant use. It is the only propellant reserve rule; there is no additional probabilistic margin or separate contingency mission.
- Vector thrust magnitude equals Gamma at powered nodes. Linear interpolation between nodes is a numerical approximation and does not enforce this equality at every instant.

### Camera and hazard detection

- One fixed bottom-mounted camera has an assumed 90 x 90 degree square hardware FOV and established assumed 10 km slant-range capability. A chosen 5 degree edge allowance gives an 80 x 80 degree usable FOV. These are project assumptions, not a validated camera design.
- Image orientation is prescribed by projecting a fixed Moon-frame downrange heading perpendicular to boresight. This fixes optical-axis roll geometrically; roll/yaw motion and mounting uncertainty are not independently optimized.
- Downward speed 0.1–50 m/s, horizontal speed <=50 m/s, minimum altitude 10 m, sensor slew cap 30 deg/s, acquisition minimum 1 s and processing delay 0.1 s are assumed operating placeholders. The lower vehicle slew cap of 10 deg/s governs the current model.
- Sensing operates only in the optimized observation phase, including processing, and both required points remain visible throughout that phase. Any observation time beyond the processing delay is treated as acquisition time; acquisition and processing are not separately optimized phases.
- Seeing the target points on a smooth sphere represents geometric visibility. The model does not prove hazard recognition or usable ground-area coverage: there is no required patch radius, terrain map, pixel resolution, exposure, frame rate, motion-blur, lighting/shadow, image-noise, detection-probability or processing-workload model.
- The baseline runner omits star trackers. The dedicated tracker runner adds geometric update constraints and four global mounting angles using prescribed body orientation, without rotational dynamics. Navigation filters, guidance feedback, disturbances, pointing errors, sensor failures and uncertainty propagation are not modelled.

### Numerical interpretation

- Most path constraints are node constraints. Observation constraints additionally use collocation midpoints and are checked using 401 independently integrated samples per window. Sampling is not a continuous-time proof.
- Independent validation uses scipy solve_ivp with the same physical equations and linearly interpolated optimized controls, rather than a different force model. No full mesh-convergence study or global-optimality proof has been performed.

## Plots, results and validation

The runner opens and saves three figures:

| Figure | Meaning | File |
|---|---|---|
| 3D trajectory | Outbound blue, return orange, dashed coast; curved lunar surface and decision gates. Cartesian axes have different scales. | results/trajectory_3d_uprange.png |
| Flight profiles | Altitude, speed, thrust, mass, thrust/body tilt and outbound terminal glide-slope close-up. Grey = coast, green = observation, dotted = decision gate. | results/flight_profiles_uprange.png |
| Camera angles | Original target dashed, actual touchdown solid; horizontal and vertical angles must lie inside the dotted +/-40 degree usable boundaries during observation. Zero is image center. | results/sensor_pointing_uprange.png |

JSON contains summary masses and Delta-v, target offsets, phase timeline, gate state, fixed-gate divert verification, touchdown limits, hardware/usable FOV and other sensor inputs, per-phase propagation errors, both chained touchdowns and dense sensing metrics. Target offset is distance from the selected target; propagation error is disagreement with the optimized endpoint. They are different quantities.

The runner rejects nonconvergence, failed sizing reproduction, discarded paid-for thrust or failed dense sensor validation before saving a new result. It also reports independent trajectory propagation and re-solves the selected terminal descent from its fixed moving gate. No alternate sites are optimized automatically. Old downrange and no-uprightness JSON files are historical cases, not the current configuration.

## Two independent top-mounted star trackers

Both trackers point upward from the hopper's body XY top plane. Each has its own elevation and azimuth, fixed in the body and shared by both hops. They need not be opposite or have equal elevation. The initial feasible +X/-X seed is 72.660 deg and 54.936 deg above XY, separated by 52.404 deg. The full 20 deg FOV must stay above an ideal opaque top plane; this is an obstruction proxy, not CAD clearance. Actual mounting locations are unspecified. See [STAR_TRACKER_PLAN.md](STAR_TRACKER_PLAN.md) for the design and current implementation status.

Run in PowerShell:

~~~powershell
.\.venv\Scripts\python.exe -B scripts/analyze_star_trackers.py
~~~

This fixed-flight analyzer searches four global mounting angles using a feasible seed, deterministic multistart and bounded derivative-free refinement. It explicitly enables the optional tracker configuration. The first run solves and validates the baseline, saving full numeric arrays in `results/star_tracker_trajectory_uprange.npz`. Later runs check and reuse that cache. Summary-only JSON is insufficient trajectory input. Use `--refresh-baseline` after a baseline change or `--output-dir` for another results directory. Outputs use `star_tracker_independent_top_uprange` filenames. Historical `star_tracker_uprange` results use the superseded antipodal design and are not a valid two-top-mount installation.

For the joint trajectory/mount solve:

~~~powershell
.\.venv\Scripts\python.exe -B scripts/run_tracker_coupled.py
~~~

The coupled runner starts from the validated baseline cache, constructs a labeled update schedule with a 0.9 s seed gap, then optimizes the full flight and four independent mounting angles with IPOPT. The original mass objective, engines and hazard-camera constraints are retained. A sizing problem has 3325 variables. Event head identities and collocation-cell assignments are fixed for this local subproblem; phase durations and actual dimensional event times change. The true acceptance bound remains 1 s. Symbolic quaternion coast interpolation and Hermite-Simpson event-state interpolation match the numerical analyzer. This is a local schedule formulation, not an unrestricted hybrid/global optimum.

The coupled result is accepted only after checking its exact optimized labeled events with independent propagation, finer flexible availability schedules, flight propagation, dense hazard-camera sensing, thrust, selected fixed-gate descent and a same-mount coupled fixed-mass sizing reproduction. Separate `star_tracker_coupled_uprange` reports/figures and a coupled trajectory NPZ preserve the baseline artifacts. A report marked `verification_incomplete` or a failure status is not an accepted solution.

Verified combined result: **Solve_Succeeded**, 3766.684646 kg wet mass, unchanged from baseline. Both actual event schedules have 567 updates and a maximum gap of 0.9 s. The installed directions have elevations 72.660 / 54.936 deg above XY and azimuths 0 / 180 deg; both FOVs clear the top plane. Exact event checks, independently refined flexible schedules, flight/camera checks, fixed-gate descent and same-mount coupled fixed-mass reproduction pass. The workflow was reproduced from caches.

IPOPT accepted the verified primal/dual warm start with **zero further iterations**: the minimum-mass baseline already satisfies the new constraints with this pair. The mount variables are free, but their values stayed at the feasible seed; the mass objective does not select a unique most-robust mount. Sparse sign/complementarity-constrained linear stationarity recovery supplies multiplier guesses without moving the flight. Those guesses alone are not an accepted optimization result. This demonstrates the combined formulation at a feasible warm start, without establishing reliable cold-start recovery or a global/universal zero mass penalty. The sizing NLP takes about 17-18 s in the verified runs, excluding other validation/reproduction work.

The active tracker requirements are a 20 deg full circular FOV, 23.5 deg Sun/Earth **boresight-to-center** exclusions, zero Moon-limb **FOV-edge** margin, and no more than 1 s between updates. Per the 2026-10-16 user instruction, `star_tracker.enforce_body_rate: false` waives the original 6 deg/s gate, assuming active attitude control outside this model. Full inertial body rate remains diagnostic and cannot reject measurements or cadence while disabled. Set the option to `true` to reinstate the gate. One tracker must pass all active tests at each accepted measurement; updates may alternate trackers and both can be unavailable between measurements. Each hop starts with an initialized attitude estimate of age zero. Acquisition, exposure and switching delay are assumed zero; a clear cone is assumed to contain enough usable stars. A passing geometry/cadence study does not validate attitude-control capability or tracker performance at those rates.

Powered attitude follows thrust with right-handed body axes and projected-downrange roll. Coast uses a shortest inertial quaternion SLERP between ascent-end and braking-start frames; zero coast thrust is never normalized. Full inertial body rate includes roll and lunar-frame rotation. Frame singularities or orientation jumps stop the analysis instead of producing a coverage pass. Including rise through `required_phase_kinds` also checks its ascent junction for continuity.

Earth/Sun directions are synthetic, frozen in an inertial frame aligned with the Moon-fixed frame at first launch. Their initial directions are `[1, 0, 0]` and `[0.05059294007671333, -0.9987193571841863, 0]`. The analyzer uses cumulative mission time and zero surface stay. Exclusions remain active for occulted bodies. These are presentation assumptions, not ephemerides or a launch-window assessment.

The standalone mounting objective first seeks schedules meeting cadence, then improves extra angular clearance at measurement opportunities. The winner is checked with independently propagated positions, finer time grids, individual margin-root refinement, explicit labeled measurement events and a finite-rotation body-rate cross-check. A change that does not converge is reported as unresolved. Dense sampling and transition refinement are numerical evidence, not a continuous-time proof; failed local mounting searches do not prove no installation can work.

Tracker figures show signed margins, diagnostic body rate, availability/actual updates, estimate age and both real mounting directions. JSON records geometry assumptions, obstruction proxy, search resolution, signed event margins, cadence, numerical tolerances and trajectory provenance. Exit code 0 means accepted validation; exit code 2 means no validated result. Torque, actuator capability, star counts, navigation accuracy, detailed structure/plume clearance and hardware performance remain outside this study.

The cache-only +X/-X feasibility search is reproducible with `scripts/check_top_mount_feasibility.py`: 407 tested pairs, both hops passing, and a separately validated robust schedule with about 49.4165 deg extra celestial margin. It does not constrain the flight path. The production four-angle fixed-flight analyzer also passes, finding elevations 72.836 / 54.737 deg and azimuths 347.663 / 180.000 deg. These are best tested local designs under the synthetic geometry, not global or physical installation guarantees.

Focused checks:

~~~powershell
.\.venv\Scripts\python.exe -B -m pytest -q -p no:cacheprovider tests/test_star_tracker.py tests/test_star_tracker_coupled.py tests/test_params.py tests/test_sensor.py
~~~

## Code map and trajectory access

| File | Responsibility |
|---|---|
| config.yaml; hopper/params.py | Physical inputs, limits, validation and scaling |
| scripts/run_roundtrip.py | Default solve, checks, JSON and plot launch |
| hopper/problem.py; hopper/phases.py | Two-hop decision variables, constraints and linkage |
| hopper/constraints.py; hopper/sensor.py | Path/landing constraints and square-camera geometry |
| hopper/dynamics.py; hopper/environment/ | Rotating-frame equations, gravity and terrain |
| hopper/transcription.py; hopper/initial_guess.py; hopper/lambert.py | Collocation and starting trajectory |
| hopper/postprocess.py; hopper/validate.py | Mass/Delta-v bookkeeping and independent propagation/sensing checks |
| hopper/divert.py | Selected fixed-gate terminal descent; optional comparison helper for separate studies |
| hopper/plotting.py; tests/ | Matplotlib output and unit/integration checks |
| hopper/tracker_params.py; hopper/star_tracker.py | Optional tracker settings, fixed attitude, mounting search and event validation |
| hopper/tracker_cache.py; scripts/analyze_star_trackers.py | Validated full-trajectory cache and separate tracker reports/figures |
| hopper/star_tracker_coupled.py; scripts/run_tracker_coupled.py | Four global top-mount angles, symbolic event geometry, dimensional cadence and coupled acceptance checks |

For a solved Solution, phases contain X (7 rows: offset position in m, velocity in m/s, mass in kg), U (4 rows: thrust vector and Gamma in N), and t (duration in seconds). Absolute position is mission.r_start + offset position. Coast controls are zero. sol.m_wet is launch mass and sol.z is final mass.

Outbound phase names are rise, ascent, coast, braking, observation, terminal_uprange. Return names are rise2_uprange, ascent2_uprange, coast2_uprange, braking2_uprange, observation2_uprange, terminal2_uprange. The decision gate is the observation endpoint. Trajectory arrays are available through the Python Solution object; the runner currently saves summaries and figures rather than trajectory CSV files.

# Current design

Updated 2026-10-15. See [README.md](README.md) for the complete assumptions and constraint tables; [config.yaml](config.yaml) is the parameter source. [hopper_spec.md](hopper_spec.md) is an archived original specification.

## Mission and optimization

Size one prescribed round trip near the lunar south pole. The outbound divert target is 150 m uprange of the original destination; actual surface touchdown is optimized within 10 m of it. Return touchdown is within 10 m of launch. The 10 m design tolerance sits inside the 50 m requirement. The assumed worst-case direction is prescribed, not proven by comparison or uncertainty analysis.

Each hop has rise, ascent, coast, braking, observation and terminal phases. Observation ends at the moving decision gate. There is no hover. The outbound gate is directly above the original target; return gate horizontal position is free. Both gate altitudes are constrained to 150–1000 m. Actual outbound landing position, normal and mass initialize the return, at rest after an ignored surface stay.

Free variables are seven states at every node, four controls at powered nodes, twelve phase durations, and one wet mass in size mode. Touchdown positions are endpoint state variables. Gate states, observation start and divert start follow from states and durations; there are no separate attitude, yaw, gimbal, sensor-pointing, acquisition-time or processing-time variables. Current size-mode count: 3321 scalar variables.

Minimize wet mass with loaded propellant >=1.1 times total propellant consumed. Alternative maximise_margin mode fixes wet mass and maximizes final mass. Hermite–Simpson collocation, CasADi derivatives and IPOPT give a local optimum. Controls are linear between nodes; powered-node thrust magnitude equals Gamma. Initial gate seeds are 450 m outbound and 300 m return, not fixed gate conditions.

## Active operating limits

| Limit | Current value |
|---|---|
| Equivalent powered thrust | Approximately 1112–26688 N; coast off |
| Dry mass; Isp; O/F | 1500 kg; 310 s; 1.65 |
| Rise; ascent/coast/braking altitude | Rise to 50 m; other listed phases >=50 m |
| Terminal glide slope | 45 degrees relative to actual touchdown |
| Touchdown | Surface, within 10 m of target; downward <=1.5 m/s; horizontal <=0.5 m/s; thrust/body-up tilt <=20 degrees |
| Vehicle direction-rate approximation | 10 deg/s |
| Camera hardware FOV; margin; usable FOV | 90 x 90 degrees; 5 degrees per edge; 80 x 80 degrees |
| Camera range; minimum altitude | 10 km slant range; 10 m altitude |
| Sensing downward; horizontal speed | 0.1–50 m/s downward; <=50 m/s horizontal |
| Sensor pointing-rate cap | 30 deg/s; vehicle cap makes current effective limit 10 deg/s |
| Observation duration | >=1 s acquisition +0.1 s processing |

The fixed bottom camera points opposite thrust/body-up. Image orientation follows a projected fixed Moon-frame downrange heading. During observation, including processing, the original target and actual touchdown must be forward of the camera, within the usable square FOV/range and above the spherical horizon. There is no required patch radius, whole-disk coverage, or post-gate terminal visibility constraint. Camera constraints apply at nodes and midpoints.

## Assumptions and scope

The vehicle is a 3-DoF point mass in a rotating Moon-fixed frame with central gravity, Coriolis and centrifugal acceleration, spherical zero-height terrain and no drag. Body-up follows thrust; optical roll is prescribed geometrically. Rotational dynamics, torque limits, gimbal motion, landing contact, terrain hazards, image formation, navigation errors, guidance feedback and uncertainty are omitted. The baseline runner omits trackers; the dedicated tracker runner adds geometric measurement constraints and four fixed body-frame mounting angles to the flight NLP.

## Independent top-mounted star trackers

Two independent upward boresights have separate elevation/azimuth angles, shared across both hops. They are not antipodal. Their full 20 deg optical cones must clear an ideal body XY top plane; physical locations/CAD are not modeled. `scripts/analyze_star_trackers.py` searches four angles on a fixed trajectory. Each event requires the same tracker to clear 23.5 deg Sun/Earth center exclusions and the full-FOV lunar limb. Update gaps and block-boundary ages must be <=1 s. The user waived the 6 deg/s body-rate gate, assuming active attitude control outside the model. Measurements/switching are instantaneous and each block begins with a zero-age estimate.

`scripts/run_tracker_coupled.py` uses the original minimum-mass objective with four global mount variables and fixed labeled event/cell assignments. All states, controls and phase durations remain optimized. Actual dimensional event times and <=1 s gaps are constrained as durations vary. Powered frames prescribe projected-downrange roll; symbolic quaternion interpolation matches inertial coast SLERP. The sizing NLP has 3325 variables. This is a local schedule formulation; outer schedule reassignment can explore alternatives.

Both modes preserve independent propagation, dense sensing checks and original engine architecture. Coupled acceptance also checks the exact selected-head events and a same-mount coupled fixed-mass reproduction. Synthetic geometry, numerical refinement and the top-plane proxy do not establish real launch-window feasibility, global optimality, detailed installation clearance, navigation accuracy or actuator capability. Baseline and coupled artifacts have separate filenames.

Four nominal engines are represented as one continuously throttleable equivalent source, with constant Isp and instantaneous on/off operation. Engine switching, pulsing, restart delays, transient consumption and start-cycle limits are omitted. Dry mass is fixed; engine/tank/structure mass do not scale. The surface stay consumes no time or propellant. The sole reserve rule is 1.1 times nominal round-trip consumption. Sensor motion, timing, edge margin and many flight limits are assumed placeholders; full assumptions are enumerated in the README.

Direction-rate constraints are discrete approximations; phase joins allow one following-phase interval for rotation. Thrust equality is imposed at nodes, not at every interpolated instant. No full mesh-convergence or global-optimality proof is claimed.

## Latest validation and output

Current result: wet mass 3766.68 kg, loaded propellant 2266.68 kg, consumed 2060.62 kg, Delta-v 2407.75 m/s. Outbound gate 245.01 m, descending at 20.91 m/s. Both observation windows last 1.10 s. Main sizing solve: Solve_Succeeded, 1331 iterations, about 99 s; fixed-mass cross-check reaches acceptable level and reproduces the result. Selected fixed-gate descent and 401-sample independent sensing checks pass. Chained touchdown propagation errors are 2.4 mm outbound and 3.1 mm return.

Current full suite: 114 passed, including baseline integration and tracker geometry/coupling/warm-start checks. Coupled sizing and same-mount reproduction both return Solve_Succeeded; IPOPT accepts the already feasible/near-stationary warm start with zero further iterations. Coupled wet mass remains 3766.684646 kg. The previous relaxed configuration was 3754.53 kg; the 12.15 kg increase combines landing, sensing-speed and FOV-margin changes, not a tracker penalty.

The runner saves JSON and opens three Matplotlib figures: 3D trajectory, flight profiles and camera angles. Use --no-show to save without windows. Run commands and plot interpretation are in the README.

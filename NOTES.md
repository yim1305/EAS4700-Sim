# Current status

Updated 2026-10-16 on Star-Tracker-Constraints. README.md is the user guide; STAR_TRACKER_PLAN.md is the current design/handoff. STAR_TRACKER_PLAN_ARCHIVE_2026-10-15.md preserves the original antipodal plan as history.

## Baseline flight

The original engines, dry mass, hazard camera, two-hop phase structure and flight objective remain unchanged. The baseline runner is scripts/run_roundtrip.py. Its sized wet mass is 3766.684646 kg with 2266.68 kg loaded propellant, 2060.62 kg consumed, approximately 2407.75 m/s finite-burn Delta-v and 1.10 s observation windows. The selected case is a 150 m uprange divert and a return to the launch region; touchdown tolerance is 10 m.

Rise, ascent, coast, braking, observation and terminal phases remain in each hop. No hover. Body-up follows thrust with prescribed projected-downrange roll. There are no rotational dynamics or independent body-attitude variables. The original vehicle slew and hazard-camera conditions remain active.

The validated full-array cache is results/star_tracker_trajectory_uprange.npz. Baseline identity: c91e408d86af3ce10583f83e9ac8c5f3549153d5ead45f208d0792b4f7503094. Summary JSON alone is insufficient trajectory input.

## Current tracker requirements

Two independent top-mounted trackers, each with its own fixed elevation and azimuth shared by both hops. They do not have to be opposite or have equal elevation. A +X-leaning / -X-leaning pair is an example, not a required azimuth symmetry.

Full circular FOV 20 deg; Sun/Earth boresight-center exclusions 23.5 deg; entire cone outside the apparent lunar disk with 0 deg extra limb margin. The study uses a conservative ideal-top-plane obstruction proxy: each full FOV stays above body XY. This does not establish physical mounting locations or detailed CAD/plume clearance.

Either head can provide each update; maximum gap and initialized-estimate boundary ages are 1 s across each uninterrupted ascent+coast block. No simultaneous visibility or one-failed-head redundancy is required. Instantaneous measurements/switching, sufficient stars in a clear cone and an initial zero-age estimate per block are assumed.

The user waived the original 6 deg/s gate, assuming active attitude control without modeling it. Config enforce_body_rate is false. Full inertial body rate remains diagnostic, not a feasibility gate. Do not change the existing vehicle slew constraint to 6 deg/s.

Synthetic Earth/Sun directions remain inertially frozen at [1,0,0] and [0.05059294007671333,-0.9987193571841863,0]. Use cumulative dimensional mission time, rotation into/from Moon-fixed coordinates, and zero surface stay. Powered roll and inertial endpoint coast SLERP are prescribed. There is no real ephemeris, star catalog, navigation covariance or actuator proof.

## Verified top-mount studies

The cache-only +X/-X search passes both hops. Elevations above XY are 72.66012644 / 54.93565637 deg, azimuths 0 / 180 deg, separation 52.40421719 deg. Both cones have substantial roof-plane clearance. Search tested 407 elevation pairs. A separately validated robust schedule supports about 49.4165 deg extra celestial margin and 511 events per hop. See results/star_tracker_top_mount_feasibility_uprange.json and matching PNG. No third tracker is required for this baseline case.

Production configuration/parser/search/plots now support independent_top, antipodal false and four independent global angles. The fixed-flight four-angle analyzer passes with elevations 72.836365 / 54.737005 deg, azimuths 347.662857 / 180 deg and about 49.6143 deg validated robust margin. Current outputs use star_tracker_independent_top_uprange filenames. Older star_tracker_uprange outputs contain an antipodal downward companion and are invalid as the new two-top-mount installation.

## Joint optimizer implementation

hopper/star_tracker_coupled.py subclasses the unchanged baseline Problem. TrackerProblem adds four global mounting angles (3325 sizing variables total), exact selected-head event geometry, symbolic powered/coast frames and actual dimensional gap constraints. The current schedule seed has 1134 events, 0.9 s design gaps, 10 deg extra seed clearance and 8 unique gap inequalities after deduplication. Acceptance remains the user's <=1 s requirement and original celestial exclusions.

Event head identities and phase/collocation-cell assignments are frozen for this local solve; states, thrust, phase durations and mounting angles remain decision variables. There is no binary mixture or independent per-node mounting direction. The original mass objective is retained. Symbolic quaternion interpolation handles small/near-pi rotations and matches the numerical SLERP; event positions use Hermite-Simpson interpolation.

scripts/run_tracker_coupled.py starts from the validated baseline cache, solves the joint NLP, checks the exact optimized events with independent propagation, reanalyzes flexible availability on finer sampling, validates flight/camera/thrust, verifies the fixed-gate descent and reproduces sizing with a coupled fixed-mass solve using the same fixed mount. It saves dedicated coupled artifacts, preserving the baseline.

Current result: results/star_tracker_coupled_uprange.json has status numerically_validated_coupled. Both sizing and same-mount coupled fixed-mass reproduction return Solve_Succeeded. Wet mass is 3766.684646 kg, reported difference 0.0 kg from baseline. There are 567 actual updates per hop, maximum gap 0.9 s, final ages 0.417703 / 0.648017 s and worst accepted celestial margins 10.060225 / 10.092590 deg. Independent chained touchdown errors remain 2.434 / 3.120 mm. Exact events, refined flexible schedules, flight/camera/thrust and fixed-gate checks pass. The complete workflow was reproduced from caches.

The accepted NLP performs zero further iterations: a sparse linear KKT multiplier warm start makes the already feasible baseline satisfy local optimality tests as well. Mount angles remain at the verified +X/-X seed. The four angles are free NLP variables, but the mass objective does not pick a unique robust mounting. Do not claim movement to a new optimum or robust convergence from arbitrary infeasible starts. Standalone four-angle mounting search remains available for clearance optimization.

Cold dual starts were slow; re-solving the baseline to recover multipliers also failed from the solved point. Final multiplier recovery uses a sign/complementarity-constrained sparse HiGHS linear program, without moving the flight. Size/fixed-mass stationarity residuals are about 8.7e-9 / 2.1e-9. Numeric multiplier caches are tied to baseline/mode/primal identity. They are only guesses; actual IPOPT success and independent acceptance are mandatory. Coupled feasibility tolerance is 1e-8, stated acceptable local optimality tolerance 1e-6, and a CPU safeguard/progress logging are included. The accepted runs return Solve_Succeeded. Sizing NLP wall time is about 17-18 s plus the other checks/reproduction.

## Tests and reproduction

Final full regression suite: 114 passed in 202.17 s, including existing round-trip integration and independent-mount/symbolic/cadence/multiplier-cache tests. The final coupled workflow was independently reproduced after the solver/plot/report updates. No mesh-convergence or global-optimality proof is claimed.

PowerShell entry points:

    .\.venv\Scripts\python.exe -B scripts/check_top_mount_feasibility.py
    .\.venv\Scripts\python.exe -B scripts/analyze_star_trackers.py
    .\.venv\Scripts\python.exe -B scripts/run_tracker_coupled.py
    .\.venv\Scripts\python.exe -B -m pytest -q -p no:cacheprovider

All current work is uncommitted. Preserve the user's files and historical baseline outputs. No publication or external messages were sent.

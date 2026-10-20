"""Joint minimum-mass trajectory and two independent upward tracker directions.

Starts from the validated baseline cache. Tracker identities/cells are fixed for
this local solve; all phase durations, states, controls and four mount angles vary.
"""
import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import sys
import time

import numpy as np
import casadi as ca
from scipy.optimize import linprog
from scipy.sparse import csc_matrix, diags, hstack, vstack

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from hopper.params import load_params
from hopper.postprocess import thrust_discard_flags
from hopper.problem import Problem
from hopper.star_tracker import build_blocks, mounting_report, validate_mount
from hopper.star_tracker_coupled import TrackerProblem, prepare_schedule, validate_assigned_events
from hopper.tracker_cache import baseline_identity, json_value, load_trajectory, save_trajectory
from hopper.validate import repropagate, sensor_report
from scripts.analyze_star_trackers import plot_report
from scripts.run_roundtrip import verify_divert


def recover_baseline_duals(p, branch, guess):
    """Recover sign/complementarity-compatible multipliers WITHOUT changing the flight.

    Solve the linear KKT stationarity equations with complementarity-derived bounds. This
    is a warm-start estimate, never a substitute for the subsequent IPOPT solve.
    """
    original = Problem(p, branch)
    original.set_guess(Problem.guess_from_solution(guess), m_wet0=guess.m_wet)
    initial = original.opti.initial()
    x = original.opti.debug.value(original.opti.x, initial)
    digest = hashlib.sha256(np.asarray(x, dtype=float).tobytes() +
                            baseline_identity(p, branch, ROOT).encode() + p.mission.mode.encode()).hexdigest()
    dual_cache = ROOT / "results" / f"star_tracker_duals_{p.mission.mode}_{branch}.npz"
    if dual_cache.exists():
        with np.load(dual_cache, allow_pickle=False) as saved:
            if (str(saved["identity"].item()) == digest and saved["duals"].shape == (original.opti.ng,) and
                    np.isfinite(saved["duals"]).all()):
                print(f"Loading multiplier warm start: {dual_cache.name}", flush=True)
                return guess, saved["duals"].copy()
    print("Recovering optimality multipliers by a sparse stationarity linear program", flush=True)
    t0 = time.time()
    evaluate = ca.Function("baseline_kkt", [original.opti.x], [original.opti.g,
                           ca.gradient(original.opti.f, original.opti.x),
                           ca.jacobian(original.opti.g, original.opti.x)])
    gv, gradient, jacobian = evaluate(x)
    g, gradient = np.asarray(gv).ravel(), np.asarray(gradient).ravel()
    lo, hi = (np.asarray(original.opti.debug.value(v, initial)).ravel()
              for v in (original.opti.lbg, original.opti.ubg))
    row, col = jacobian.sparsity().get_triplet()
    J = csc_matrix((np.asarray(jacobian.nonzeros()), (row, col)), shape=jacobian.shape)
    equality = np.isfinite(lo) & np.isfinite(hi) & (abs(hi - lo) < 1e-12)
    # A distance-only active-set cutoff misses weak but necessary throttle bounds.
    # Include all bounded rows and cap multiplier*slack, allowing the solved point's
    # finite-barrier KKT multipliers while keeping far-inactive multipliers tiny.
    comp_limit = 5e-9
    lower_dual = np.where(np.isfinite(lo), -comp_limit / np.maximum(g - lo, 1e-14), 0.)
    upper_dual = np.where(np.isfinite(hi), comp_limit / np.maximum(hi - g, 1e-14), 0.)
    lower_dual[equality], upper_dual[equality] = -np.inf, np.inf
    norms = np.sqrt(np.asarray(J.multiply(J).sum(axis=1))).ravel()
    active = (norms > 1e-16) & (np.isfinite(lo) | np.isfinite(hi))
    scaling = 1. / norms[active]
    A = J[active].T @ diags(scaling)
    A = A.tocsr()
    slack_column = csc_matrix(-np.ones((A.shape[0], 1)))
    matrix = vstack((hstack((A, slack_column)), hstack((-A, slack_column)))).tocsr()
    bounds = list(zip(lower_dual[active] / scaling, upper_dual[active] / scaling)) + [(0., None)]
    fit = linprog(np.r_[np.zeros(active.sum()), 1.], A_ub=matrix,
                  b_ub=np.r_[-gradient, gradient], bounds=bounds, method="highs",
                  options={"primal_feasibility_tolerance": 1e-9, "dual_feasibility_tolerance": 1e-9,
                           "time_limit": 60.})
    if not fit.success:
        raise ValueError(f"Stationarity multiplier LP failed: {fit.message}")
    duals = np.zeros(len(g))
    duals[active] = fit.x[:-1] * scaling
    stationarity = float(max(abs(J.T @ duals + gradient)))
    print(f"Multiplier recovery: {active.sum()} active rows, stationarity residual {stationarity:.3e}, "
          f"{time.time() - t0:.1f} s", flush=True)
    if stationarity < 1e-4:
        np.savez_compressed(dual_cache, identity=np.array(digest), duals=duals,
                            stationarity_residual=np.array(stationarity))
    return guess, duals


def main(branch="uprange"):
    config = str(ROOT / "config.yaml")
    p = load_params(config, {"star_tracker": {"enabled": True}})
    identity = baseline_identity(p, branch, ROOT)
    baseline, baseline_validation = load_trajectory(ROOT / "results" / f"star_tracker_trajectory_{branch}.npz",
                                                   p, branch, identity)
    seed = dict(elevations=list(p.star_tracker.initial_elevations), azimuths=list(p.star_tracker.initial_azimuths))
    warm, duals = recover_baseline_duals(p, branch, baseline)
    events, seed_reports = prepare_schedule(p, warm, branch, seed, gap_fraction=.9, extra_margin_deg=10.)
    print(f"Building joint NLP: {len(events)} labeled events, four global upward mounting angles", flush=True)
    pr = TrackerProblem(p, branch, events, seed, log_progress=True)
    pr.set_guess(Problem.guess_from_solution(warm), m_wet0=warm.m_wet)
    pr.set_dual_seed(duals)
    print(f"{pr.opti.nx} variables, {pr.opti.ng} constraints, "
          f"{pr.unique_gap_constraint_count} unique dimensional gap bounds", flush=True)
    initial_violation = pr.initial_violation()
    print(f"Warm-start maximum primal violation: {initial_violation:.3e}", flush=True)
    if not np.isfinite(initial_violation) or initial_violation > 1e-5:
        raise ValueError("Coupled warm start is unexpectedly infeasible; inspect event/frame interpolation")
    t0 = time.time()
    sol = pr.solve(print_level=0)
    elapsed = time.time() - t0
    print(f"Coupled solve: {sol.status}, {sol.iterations} iterations, {elapsed:.1f} s, wet mass {sol.m_wet:.6f} kg", flush=True)
    output = ROOT / "results" / f"star_tracker_coupled_{branch}.json"
    report = dict(schema_version=1, status="solve_failed" if not sol.success else "verification_incomplete",
                  trajectory_constrained=True, mounting_optimized=True, branch=branch,
                  solve_status=sol.status, iterations=sol.iterations, wall_s=elapsed,
                  n_variables=pr.opti.nx, n_constraints=pr.opti.ng,
                  baseline_identity=identity, baseline_wet_mass_kg=baseline.m_wet,
                  coupled_wet_mass_kg=sol.m_wet, wet_mass_difference_kg=sol.m_wet - baseline.m_wet,
                  warm_start_accepted_without_step=sol.iterations == 0,
                  body_rate_limit_enforced=False,
                  tracker_settings_radians_seconds=asdict(p.star_tracker),
                  formulation="Local fixed labeled event/cell assignments; four global mount angles; "
                              "original wet-mass objective; true dimensional cadence <=1 s",
                  seed_gap_s=.9 * p.star_tracker.update_gap_max, seed_angular_margin_deg=10.,
                  source_sha256=hashlib.sha256((ROOT / "hopper" / "star_tracker_coupled.py").read_bytes()).hexdigest(),
                  iteration_log=pr.iteration_log,
                  solver_policy="primal/dual warm start, 1e-8 feasibility, 1e-6 acceptable local optimality; "
                                "independent physical/event acceptance still required",
                  limitations=["Synthetic celestial geometry and prescribed powered-roll/coast attitude",
                               "Rate gate waived; active-control capability is assumed, not modeled",
                               "Full-FOV top-plane proxy, not detailed hardware installation clearance",
                               "Fixed local event assignments, not globally unrestricted hybrid optimization"])

    def checkpoint():
        output.write_text(json.dumps(report, indent=2, default=json_value, allow_nan=False), encoding="utf-8")

    checkpoint()
    if not sol.success:
        return report
    candidate, actual_events = pr.mounting_candidate(), pr.actual_events()
    report["mounting_changed_from_seed"] = bool(
        np.max(np.abs(np.asarray(candidate["elevations"]) - seed["elevations"])) > 1e-9 or
        np.max(np.abs(np.asarray(candidate["azimuths"]) - seed["azimuths"])) > 1e-9)
    report["mount"] = mounting_report(candidate, p.star_tracker)
    assigned = validate_assigned_events(p, sol, branch, candidate, actual_events)
    report["assigned_schedule_validation"] = assigned
    flexible = validate_mount(build_blocks(p, sol, branch), build_blocks(p, sol, branch, independent=True), candidate)
    report["independent_flexible_schedule"] = flexible
    if not all(row["passed"] and row["numerically_resolved"] for row in list(assigned.values()) + list(flexible.values())):
        report["status"] = "tracker_validation_failed"
        checkpoint()
        return report
    print("Exact optimized events and independent flexible tracker schedules pass; checking flight", flush=True)
    rep, td = repropagate(p, sol, branch)
    sensing = sensor_report(p, sol, branch)
    flags = thrust_discard_flags(p, sol)
    flight_passed = (not flags and max(r["pos_err"] for r in rep) < .05 and td["pos_err"] < .05 and
                     td["hop1"]["pos_err"] < .05 and all(row["passed"] for row in sensing.values()))
    report["flight_validation"] = dict(passed=bool(flight_passed), propagation=rep, touchdown=td,
                                       sensing=sensing, thrust_discard_flags=flags)
    if not flight_passed:
        report["status"] = "flight_validation_failed"
        checkpoint()
        return report
    report["fixed_gate_divert"] = verify_divert(p, branch, sol)
    print("Reproducing sized mass with the same trackers, labeled events and fixed mount", flush=True)
    p2 = load_params(config, {"vehicle": {"m_wet": sol.m_wet}, "mission": {"mode": "maximise_margin"},
                              "star_tracker": {"enabled": True}})
    check = TrackerProblem(p2, branch, events, candidate, fixed_mount=True, mode="maximise_margin", log_progress=True)
    fixed_warm, fixed_duals = recover_baseline_duals(p2, branch, sol)
    check.set_guess(Problem.guess_from_solution(fixed_warm))
    if check.initial_violation() > 1e-5:
        check.set_guess(Problem.guess_from_solution(sol))
    check.set_dual_seed(fixed_duals)
    csol = check.solve(print_level=0)
    ratio = (csol.m_wet - p2.vehicle.m_dry) / (csol.m_wet - csol.z)
    reproduced = (csol.success and abs(ratio / p.mission.reserve_factor - 1) <= 1e-3 and
                  abs(csol.z - sol.z) <= max(.05, .001 * sol.z))
    check_events = (validate_assigned_events(p2, csol, branch, candidate, check.actual_events())
                    if csol.success else {})
    reproduced = reproduced and bool(check_events) and all(v["passed"] for v in check_events.values())
    report["fixed_mass_reproduction"] = dict(passed=bool(reproduced), status=csol.status,
                                             loaded_over_used=ratio, final_mass_kg=csol.z,
                                             sized_final_mass_kg=sol.z, tracker_events=check_events)
    print(f"Fixed-mass check: {csol.status}, loaded/used {ratio:.6f}, passed {reproduced}", flush=True)
    if not reproduced:
        report["status"] = "fixed_mass_reproduction_failed"
        checkpoint()
        return report
    report["figures"] = plot_report(build_blocks(p, sol, branch, independent=True), candidate, assigned, output,
                                    trajectory_label="joint flight/mount solution")
    report["status"] = "numerically_validated_coupled"
    checkpoint()
    cache = ROOT / "results" / f"star_tracker_coupled_trajectory_{branch}.npz"
    save_trajectory(cache, p, sol, branch, identity, dict(passed=True, trajectory_constrained=True,
                                                       report=str(output), mounting=report["mount"]))
    print(f"Joint flight/mount optimization validated. Mount: {report['mount']}", flush=True)
    print(f"Report: {output}", flush=True)
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("branch", nargs="?", default="uprange")
    parser.add_argument("--prepare-duals", action="store_true", help="Prepare/check the baseline multiplier guess without solving the joint NLP")
    args = parser.parse_args()
    if args.prepare_duals:
        p = load_params(str(ROOT / "config.yaml"), {"star_tracker": {"enabled": True}})
        baseline, _ = load_trajectory(ROOT / "results" / f"star_tracker_trajectory_{args.branch}.npz", p,
                                      args.branch, baseline_identity(p, args.branch, ROOT))
        recover_baseline_duals(p, args.branch, baseline)
        sys.exit(0)
    result = main(args.branch)
    sys.exit(0 if result["status"] == "numerically_validated_coupled" else 2)

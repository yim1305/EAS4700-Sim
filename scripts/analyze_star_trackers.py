"""Optimize two fixed tracker directions on one validated round-trip trajectory.

Run: .venv\\Scripts\\python.exe -B scripts/analyze_star_trackers.py [branch]
The first run solves/validates the baseline once. Later runs reuse the checked NPZ.
Existing roundtrip JSON and PNG results are never overwritten by this script.
"""
import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from hopper.params import load_params
from hopper.postprocess import thrust_discard_flags
from hopper.star_tracker import (build_blocks, candidate_axes, geometry_margins, mounting_report, rate_precheck,
                                 search_mount, validate_mount)
from hopper.tracker_cache import baseline_identity, json_value, load_trajectory, save_trajectory
from hopper.validate import repropagate, sensor_report
from scripts.run_roundtrip import solve_round_trip, check_sized_mass, verify_divert


def validate_baseline(p, sol, branch):
    if not sol.success or thrust_discard_flags(p, sol):
        raise ValueError("Baseline failed solve/thrust validation")
    if p.mission.mode == "size":
        check_sized_mass(p, branch, sol)
    rep, td = repropagate(p, sol, branch)
    sensing = sensor_report(p, sol, branch)
    if (max(r["pos_err"] for r in rep) > .05 or td["pos_err"] > .05 or
            td["hop1"]["pos_err"] > .05 or not all(v["passed"] for v in sensing.values())):
        raise ValueError("Baseline failed independent propagation/sensor validation")
    divert = verify_divert(p, branch, sol)
    return dict(passed=True, propagation=rep, touchdown=td, sensing=sensing, divert=divert,
                fixed_mass_reproduction_checked=p.mission.mode == "size")


def plot_report(blocks, candidate, reports, output, *, trajectory_label="fixed trajectory"):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    settings = blocks[0].settings
    axis = candidate_axes(candidate)
    fig, axes = plt.subplots(4, 2, figsize=(15, 12), squeeze=False)
    for col, block in enumerate(blocks):
        samples = block.sample(settings.sample_step)
        t = samples.times - block.start
        margins = np.degrees(geometry_margins(samples, axis, settings))
        for j, color in enumerate(("tab:blue", "tab:orange")):
            for k, (label, style) in enumerate(zip(("Sun", "Earth", "Moon limb"), ("-", "--", ":"))):
                axes[0, col].plot(t, margins[:, j, k], color=color, ls=style,
                                  label=f"Tracker {j + 1}: {label}", lw=.8)
        axes[0, col].axhline(0, color="black", lw=.8)
        axes[0, col].set_ylabel("Signed clearance (deg)")
        axes[0, col].set_title(f"{block.name}: {trajectory_label}, synthetic geometry")
        axes[0, col].legend(fontsize=7, ncol=2)
        rate = np.degrees(samples.body_rates)
        axes[1, col].plot(t, rate)
        axes[1, col].axhline(np.degrees(settings.tracking_body_rate_max), ls="--",
                            color="red" if settings.enforce_body_rate else "gray",
                            label="Rate limit" if settings.enforce_body_rate else "6 deg/s reference; not enforced")
        axes[1, col].legend(fontsize=7)
        axes[1, col].set_ylabel("Inertial body rate (deg/s)")
        valid = margins.min(axis=-1) >= -np.degrees(settings.angular_tolerance)
        if settings.enforce_body_rate:
            valid &= rate[:, None] <= np.degrees(settings.tracking_body_rate_max + settings.rate_tolerance)
        for j in range(2):
            axes[2, col].fill_between(t, j + .7, j + 1.3, where=valid[:, j], alpha=.4,
                                      color=("tab:blue", "tab:orange")[j])
        events = reports[block.name]["events"]
        if events:
            axes[2, col].scatter([e["time_s"] - block.start for e in events],
                                 [e["tracker"] for e in events], s=4, color="black")
        axes[2, col].set(yticks=[1, 2], ylabel="Usable tracker / updates", ylim=(.5, 2.5))
        if not reports[block.name]["passed"]:
            axes[2, col].set_title("Best-effort updates shown; cadence FAILED", fontsize=10, color="darkred")
        et = np.array([block.start] + [e["time_s"] for e in events])
        age_times, ages = [], []
        for a, b in zip(et, np.r_[et[1:], block.end]):
            age_times.extend([a - block.start, b - block.start])
            ages.extend([0, b - a])
        axes[3, col].plot(age_times, ages, lw=.7)
        axes[3, col].axhline(settings.update_gap_max, ls="--", color="red")
        axes[3, col].set(xlabel="Seconds since coverage-block start", ylabel="Attitude estimate age (s)")
        axes[3, col].set_xlim(0, min(10., block.end - block.start))
        axes[3, col].set_title(f"First 10 s; worst gap {reports[block.name]['max_update_gap_s']:.3f} s", fontsize=10)
        if not reports[block.name]["passed"]:
            deadline = reports[block.name].get("failure_deadline_s", block.start) - block.start
            axes[3, col].axvline(deadline, color="darkred", ls=":")
            axes[3, col].set_xlim(0, min(10., block.end - block.start))
            axes[3, col].set_title("First 10 seconds: initial deadline missed", fontsize=10, color="darkred")
        for ax in axes[:, col]:
            ax.grid(alpha=.2)
    fig.tight_layout()
    coverage = output.with_name(output.stem + "_coverage.png")
    fig.savefig(coverage, dpi=150)
    plt.close(fig)
    fig = plt.figure(figsize=(6, 6))
    ax = fig.add_subplot(projection="3d")
    for j, (b, color) in enumerate(zip(axis, ("tab:blue", "tab:orange"))):
        ax.quiver(0, 0, 0, *b, color=color, arrow_length_ratio=.15)
        ax.text(*(b * 1.05), f"T{j + 1}", color=color)
    for b, label in zip(np.eye(3), ("+X body", "+Y body", "+Z body")):
        ax.quiver(0, 0, 0, *(.7 * b), color="gray", alpha=.5)
        ax.text(*(.8 * b), label)
    feasible = all(v["passed"] and v["numerically_resolved"] for v in reports.values())
    ax.set(xlim=(-1, 1), ylim=(-1, 1), zlim=(-1, 1), xlabel="Body X", ylabel="Body Y", zlabel="Body Z",
           title=f"{'Validated' if feasible else 'Diagnostic only: cadence FAILED'} fixed directions\n"
                 f"{settings.mount_model}")
    ax.set_box_aspect([1, 1, 1])
    mount = output.with_name(output.stem + "_mount.png")
    fig.savefig(mount, dpi=150)
    plt.close(fig)
    return [str(coverage), str(mount)]


def main(branch="uprange", refresh=False, output_dir=None):
    output_dir = Path(output_dir) if output_dir else ROOT / "results"
    output_dir.mkdir(parents=True, exist_ok=True)
    p = load_params(str(ROOT / "config.yaml"), {"star_tracker": {"enabled": True}})
    identity = baseline_identity(p, branch, ROOT)
    cache = output_dir / f"star_tracker_trajectory_{branch}.npz"
    if cache.exists() and not refresh:
        print(f"Loading validated baseline: {cache.name}", flush=True)
        sol, validation = load_trajectory(cache, p, branch, identity)
        provenance = "validated trajectory cache"
    else:
        print("Solving baseline once; mounting search does not change the flight path", flush=True)
        sol, wall, _ = solve_round_trip(p, branch)
        if not sol.success:
            raise ValueError(f"Baseline did not converge: {sol.status}")
        print(f"Baseline: {sol.status}, wet mass {sol.m_wet:.2f} kg, {wall:.1f} s; validating", flush=True)
        validation = validate_baseline(p, sol, branch)
        save_trajectory(cache, p, sol, branch, identity, validation)
        provenance = "fresh solve with independent validation"
    blocks = build_blocks(p, sol, branch)
    independent = build_blocks(p, sol, branch, independent=True)
    precheck = rate_precheck(blocks, p.star_tracker.sample_step)
    for row in precheck:
        print(f"{row['block']}: rate-only cadence {row['refined_rate_only_schedule_passed']} "
              f"({'enforced' if p.star_tracker.enforce_body_rate else 'diagnostic only'}), "
              f"peak {row['max_sampled_rate_deg_s']:.3f} deg/s", flush=True)
    print(f"Searching a shared pair: {p.star_tracker.mount_model}", flush=True)
    ranked, search = search_mount(blocks)
    candidate, reports = ranked[0], None
    # Recheck several promising designs if the best sampled candidate fails finer validation.
    attempts = []
    for trial in ranked[:5]:
        checked = validate_mount(blocks, independent, trial)
        passed = all(r["passed"] and r["numerically_resolved"] for r in checked.values())
        attempts.append(dict(mounting=mounting_report(trial, p.star_tracker), validated_pass=passed))
        if reports is None or passed:
            candidate, reports = trial, checked
        if passed:
            break
    passed = all(r["passed"] and r["numerically_resolved"] for r in reports.values())
    robust_margin, robust_reports = 0., None
    if passed and candidate["sampled_robust_margin"]:
        trial_margin = candidate["sampled_robust_margin"]
        for _ in range(5):
            check = validate_mount(blocks, independent, candidate, trial_margin)
            if all(r["passed"] and r["numerically_resolved"] for r in check.values()):
                robust_margin, robust_reports = trial_margin, check
                break
            trial_margin /= 2
    axis = candidate_axes(candidate)
    rate_infeasible = p.star_tracker.enforce_body_rate and any(
        r["numerically_resolved"] and not r["refined_rate_only_schedule_passed"] for r in precheck)
    status = ("baseline_rate_infeasible" if rate_infeasible else "numerically_validated" if passed else
              "unresolved" if any(not r["numerically_resolved"] for r in reports.values()) else
              "no_feasible_mount_found")
    report = dict(
        schema_version=2, status=status, cadence_passed=passed, branch=branch,
        trajectory_constrained=False, baseline_identity=identity, trajectory_provenance=provenance,
        trajectory_cache=str(cache), trajectory_cache_sha256=hashlib.sha256(cache.read_bytes()).hexdigest(),
        baseline_wet_mass_kg=sol.m_wet, baseline_validation=validation,
        config_yaml_sha256=hashlib.sha256((ROOT / "config.yaml").read_bytes()).hexdigest(),
        user_requirements=dict(n_trackers=2, antipodal=p.star_tracker.mount_model == "antipodal",
                               full_cone_fov_deg=float(np.degrees(p.star_tracker.full_cone_fov)),
                               sun_exclusion_deg=float(np.degrees(p.star_tracker.sun_exclusion)),
                               earth_exclusion_deg=float(np.degrees(p.star_tracker.earth_exclusion)),
                               moon_limb_margin_deg=float(np.degrees(p.star_tracker.moon_limb_margin)),
                               body_rate_limit_enforced=p.star_tracker.enforce_body_rate,
                               body_rate_max_deg_s=(float(np.degrees(p.star_tracker.tracking_body_rate_max))
                                                    if p.star_tracker.enforce_body_rate else None),
                               body_rate_reference_deg_s=float(np.degrees(p.star_tracker.tracking_body_rate_max)),
                               update_gap_max_s=p.star_tracker.update_gap_max),
        effective_tracker_settings_radians_seconds=asdict(p.star_tracker),
        mount=dict(design_role="validated_tested_design" if passed else "diagnostic_only_not_a_feasible_design",
                   **mounting_report(candidate, p.star_tracker)),
        conventions=dict(bright_body_exclusion_reference="boresight_to_center",
                         moon_limb_reference="full_fov_edge", body_roll_policy="projected_downrange",
                         coast_attitude_policy="inertial_endpoint_slerp",
                         body_rate="full inertial angular velocity including roll",
                         geometry_label=p.star_tracker.geometry_label, directions_frozen_in="I",
                         inertial_frame="Synthetic I aligned with Moon-fixed F at first launch; not J2000",
                         direction_transform="body_direction_F(t) = Rz(-moon.omega*t) @ body_direction_I",
                         surface_stay_s=0., acquisition_time_s=0., switching_delay_s=0.,
                         initial_estimate_age_s=0., exclusions_apply_when_occulted=True,
                         sufficient_stars_in_clear_cone_assumed=True, unobstructed_mount_assumed=True),
        body_rate_policy=("full inertial body-rate gate enforced" if p.star_tracker.enforce_body_rate else
                          "User-waived tracker rate gate: active attitude control assumed, not modeled or validated; "
                          "reported coverage checks geometry and update timing only"),
        rate_precheck=precheck, search=search, independently_checked_candidates=attempts,
        sampled_robust_margin_deg=(float(np.degrees(candidate["sampled_robust_margin"]))
                                   if candidate["sampled_robust_margin"] is not None else None),
        validated_robust_margin_lower_bound_deg=float(np.degrees(robust_margin)) if passed else None,
        blocks=reports, robust_margin_blocks=robust_reports,
        limitations=["Numerical grid/transition refinement is not a continuous-time proof or global mount optimum",
                     "Update opportunities only: no position fixes, navigation covariance, star catalog, "
                     "actuator feasibility, measured hardware performance or propellant/mass penalty",
                     "Conditional on synthetic Sun/Earth geometry and prescribed roll/coast rotation"])
    stem = "star_tracker_independent_top" if p.star_tracker.mount_model == "independent_top" else "star_tracker"
    output = output_dir / f"{stem}_{branch}.json"
    report["figures"] = plot_report(independent, candidate, reports, output)
    output.write_text(json.dumps(report, indent=2, default=json_value, allow_nan=False), encoding="utf-8")
    print(f"Tracker study: {status}; elevations {report['mount']['elevations_above_xy_deg']}, "
          f"azimuths {report['mount']['azimuths_deg']}", flush=True)
    for label, row in reports.items():
        print(f"  {label}: {row['status']}, {row['event_count']} updates, "
              f"worst both-unavailable interval {row['max_unavailable_interval_s']:.6f} s", flush=True)
    print(f"Report: {output}", flush=True)
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("branch", nargs="?", default="uprange",
                        choices=["uprange", "downrange", "left", "right", "nominal"])
    parser.add_argument("--refresh-baseline", action="store_true", help="Re-solve and replace only the tracker NPZ cache")
    parser.add_argument("--output-dir", type=Path, help="Directory for tracker-only results")
    args = parser.parse_args()
    result = main(args.branch, args.refresh_baseline, args.output_dir)
    sys.exit(0 if result["cadence_passed"] else 2)

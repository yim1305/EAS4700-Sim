"""Cache-only feasibility study for two upward trackers leaning toward +X and -X.

This does not couple trackers into IPOPT or replace the legacy antipodal CLI/config.
It produces a checked seed and evidence for STAR_TRACKER_PLAN.md's next-session work.
"""
import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import sys

import numpy as np
from scipy.optimize import minimize

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from hopper.params import load_params
from hopper.star_tracker import (angle_between, build_blocks, cadence_feasible, geometry_margins,
                                 mount_axis, sampled_intervals, validate_mount)
from hopper.tracker_cache import baseline_identity, json_value, load_trajectory


def top_axes(elevations):
    """Two independent elevations above body XY, with azimuths 0 / 180 degrees."""
    return np.array([mount_axis(np.pi / 2 - elevations[0], 0.),
                     mount_axis(np.pi / 2 - elevations[1], np.pi)])


def plane_clearances(axes, full_fov):
    """Lowest ray's elevation above an ideal opaque body XY top plane."""
    return np.arcsin(np.clip(np.asarray(axes)[:, 2], -1, 1)) - full_fov / 2


def search_elevations(blocks):
    settings = blocks[0].settings
    samples = [b.sample(settings.sample_step) for b in blocks]
    lower, upper = settings.full_cone_fov / 2, np.pi / 2
    cache, candidates = {}, {}

    def geometry(tracker, elevation):
        key = (tracker, float(elevation))
        if key not in cache:
            axis = mount_axis(np.pi / 2 - elevation, tracker * np.pi)
            cache[key] = [geometry_margins(s, axis, settings)[:, 0].min(axis=-1) for s in samples]
        return cache[key]

    def assess(elevations):
        elevations = np.asarray(elevations, dtype=float)
        if np.any(elevations < lower) or np.any(elevations > upper):
            return -1e6
        first, second = geometry(0, elevations[0]), geometry(1, elevations[1])

        def feasible(extra):
            failed, worst = 0, 0.
            for block, sample, g1, g2 in zip(blocks, samples, first, second):
                windows = [sampled_intervals(sample.times, (g - extra + settings.angular_tolerance)[:, None])
                           for g in (g1, g2)]
                passed, outage = cadence_feasible(windows, block.start, block.end, settings.update_gap_max)
                failed += not passed
                worst = max(worst, outage)
            return failed == 0, failed, worst

        passed, failed, worst = feasible(0.)
        margin = 0.
        if passed:
            high = np.pi
            for _ in range(13):
                mid = (margin + high) / 2
                if feasible(mid)[0]:
                    margin = mid
                else:
                    high = mid
            score = margin
        else:
            score = -1 - failed - worst
        candidates[tuple(elevations)] = dict(elevations=elevations.tolist(), sampled_passed=passed,
                                           sampled_margin=margin, score=score)
        return score

    grid = np.unique(np.r_[np.arange(lower, upper, np.radians(5)), upper])
    for e1 in grid:
        for e2 in grid:
            assess([e1, e2])
    coarse_count = len(candidates)
    for trial in sorted(candidates.values(), key=lambda c: c["score"], reverse=True)[:2]:
        minimize(lambda e: -assess(e), trial["elevations"], method="Powell",
                 bounds=[(lower, upper)] * 2,
                 options={"maxiter": 5, "maxfev": 60, "xtol": np.radians(.05), "ftol": 1e-4})
    return sorted(candidates.values(), key=lambda c: c["score"], reverse=True), dict(
        method="two independent elevation grid searches, then bounded Powell refinement",
        scope="azimuths fixed at 0 and 180 deg for a feasible +X/-X seed; full four-angle search is future work",
        elevation_bounds_deg=np.degrees([lower, upper]).tolist(), coarse_step_deg=5.,
        coarse_count=coarse_count, tested_count=len(candidates), global_optimality_proven=False)


def plot_setup(axes, settings, reports, output):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig = plt.figure(figsize=(12, 6))
    fig.suptitle("Two top-mounted trackers: fixed-trajectory feasibility", fontsize=13)
    ax = fig.add_subplot(121, projection="3d")
    grid = np.array([[-1., -1.], [1., 1.]])
    ax.plot_surface(grid, grid.T, np.zeros((2, 2)), alpha=.15, color="gray")
    for j, (b, color) in enumerate(zip(axes, ("tab:blue", "tab:orange"))):
        ax.quiver(0, 0, 0, *b, color=color, arrow_length_ratio=.12)
        elev = np.degrees(np.arcsin(b[2]))
        ax.text(*(1.08 * b), f"T{j + 1}", color=color, fontsize=10)
        ax.plot([], [], [], color=color, label=f"T{j + 1}: {elev:.2f} deg above XY")
        # Circular full FOV about each unit boresight; the lowest ray stays above XY.
        tangent = np.cross(b, [0., 1., 0.])
        tangent /= np.linalg.norm(tangent)
        other = np.cross(b, tangent)
        phi = np.linspace(0, 2 * np.pi, 100)
        rim = (np.cos(settings.full_cone_fov / 2) * b[:, None] +
               np.sin(settings.full_cone_fov / 2) * (tangent[:, None] * np.cos(phi) +
                                                   other[:, None] * np.sin(phi)))
        ax.plot(*rim, color=color, alpha=.6)
    ax.set(xlim=(-1.2, 1.2), ylim=(-1.2, 1.2), zlim=(-.1, 1.3), xlabel="Body X", ylabel="Body Y",
           zlabel="Body Z / top normal")
    ax.set_title("Both full FOVs above the top plane\nSchematic directions; positions are not designed", fontsize=10)
    ax.legend(fontsize=8, loc="upper left")
    ax.set_box_aspect([1, 1, .65])
    ax.view_init(elev=20, azim=-65)
    age = fig.add_subplot(122)
    for name, report in reports.items():
        start = report["start_s"]
        times = np.array([start] + [e["time_s"] for e in report["events"]])
        x, y = [], []
        for a, b in zip(times, np.r_[times[1:], report["end_s"]]):
            x.extend([a - start, b - start])
            y.extend([0., b - a])
        age.plot(x, y, label=name, lw=1, alpha=.7)
    age.axhline(settings.update_gap_max, ls="--", color="red", label="1 s maximum gap")
    age.set(xlim=(0, 10), xlabel="Seconds since ascent starts (first 10 shown)",
            ylabel="Attitude update age (s)", title="Geometry-only update timing; rate gate waived")
    age.grid(alpha=.2)
    age.legend()
    fig.tight_layout(rect=[0, 0, 1, .93])
    fig.savefig(output, dpi=160)
    plt.close(fig)


def main(branch="uprange"):
    p = load_params(str(ROOT / "config.yaml"), {"star_tracker": {"enabled": True, "enforce_body_rate": False}})
    identity = baseline_identity(p, branch, ROOT)
    cache = ROOT / "results" / f"star_tracker_trajectory_{branch}.npz"
    # Deliberately no automatic solve: this is a quick study on an already validated baseline.
    sol, validation = load_trajectory(cache, p, branch, identity)
    blocks = build_blocks(p, sol, branch)
    independent = build_blocks(p, sol, branch, independent=True)
    print("Searching two independent upward elevations toward +X and -X on the cached trajectory", flush=True)
    ranked, search = search_elevations(blocks)
    chosen, reports = ranked[0], None
    for trial in ranked[:5]:
        axes = top_axes(trial["elevations"])
        check = validate_mount(blocks, independent, None, body_axes=axes)
        if reports is None:
            chosen, reports = trial, check
        if all(r["passed"] and r["numerically_resolved"] for r in check.values()):
            chosen, reports = trial, check
            break
    axes = top_axes(chosen["elevations"])
    plane = plane_clearances(axes, p.star_tracker.full_cone_fov)
    passed = (np.all(plane >= -p.star_tracker.angular_tolerance) and
              all(r["passed"] and r["numerically_resolved"] for r in reports.values()))
    robust, robust_reports = 0., None
    if passed:
        trial_margin = chosen["sampled_margin"]
        for _ in range(5):
            check = validate_mount(blocks, independent, None, trial_margin, body_axes=axes)
            if all(r["passed"] and r["numerically_resolved"] for r in check.values()):
                robust, robust_reports = trial_margin, check
                break
            trial_margin /= 2
    # Add sampled per-tracker availability: pair feasibility does not imply either one alone passes.
    for block in independent:
        sample = block.sample(p.star_tracker.sample_step / 4)
        margins = geometry_margins(sample, axes, p.star_tracker)
        reports[block.name]["sampled_individual_usable_fractions"] = np.mean(
            margins.min(axis=-1) >= -p.star_tracker.angular_tolerance, axis=0).tolist()
    output = ROOT / "results" / f"star_tracker_top_mount_feasibility_{branch}.json"
    figure = output.with_suffix(".png")
    report = dict(
        schema_version=1, status="numerically_validated" if passed else "no_validated_seed_found",
        cadence_passed=bool(passed), trajectory_constrained=False, branch=branch,
        baseline_identity=identity, baseline_wet_mass_kg=sol.m_wet, trajectory_provenance="validated NPZ cache",
        cache_sha256=hashlib.sha256(cache.read_bytes()).hexdigest(), baseline_validation=validation,
        mounting=dict(n_trackers=2, antipodal_required=False, fixed_for_both_hops=True,
                      boresights_body=axes.tolist(), elevations_above_xy_deg=np.degrees(chosen["elevations"]).tolist(),
                      tilts_from_positive_z_deg=(90 - np.degrees(chosen["elevations"])).tolist(),
                      azimuths_deg=[0., 180.], separation_deg=float(np.degrees(angle_between(axes[0], axes[1]))),
                      minimum_fov_edge_elevations_deg=np.degrees(plane).tolist(),
                      top_plane_proxy="Full 20 deg cone stays above body XY; no CAD/obstruction model"),
        body_rate_limit_enforced=False,
        assumptions=dict(active_attitude_control_assumed_not_validated=True,
                         sun_earth_reference="boresight_to_center", moon_reference="full_fov_edge",
                         synthetic_geometry=p.star_tracker.geometry_label,
                         earth_direction_I=p.star_tracker.earth_direction_I,
                         sun_direction_I=p.star_tracker.sun_direction_I,
                         powered_roll_policy="projected_downrange", coast_policy="inertial_endpoint_slerp",
                         initial_estimate_age_s=0., acquisition_time_s=0., switching_delay_s=0.,
                         surface_stay_s=0., sufficient_stars_in_clear_cone_assumed=True,
                         required_phase_kinds=p.star_tracker.required_phase_kinds),
        tracker_settings_radians_seconds=asdict(p.star_tracker), search=search,
        validated_extra_celestial_margin_deg=float(np.degrees(robust)) if passed else None,
        blocks=reports, robustness_blocks=robust_reports,
        recommended_seed_schedule="robustness_blocks" if robust_reports is not None else "blocks",
        figure=str(figure),
        integration_status="Explicit boresight evaluation supports this study; legacy configuration/main CLI "
                           "still impose antipodal search. Refactor them next session before production use.",
        limitations=["Feasible seed on a fixed path, not a jointly optimized trajectory or full four-angle global search",
                     "Top-plane clearance is not detailed structural, sunshade, plume or mutual-obstruction validation",
                     "Numerical grid/transition refinement is not a continuous-time proof",
                     "No tracker redundancy/fault-tolerance requirement was imposed; either may supply an update"])
    plot_setup(axes, p.star_tracker, robust_reports if robust_reports is not None else reports, figure)
    output.write_text(json.dumps(report, indent=2, default=json_value, allow_nan=False), encoding="utf-8")
    print(f"Top-mount study: {report['status']}; elevations {report['mounting']['elevations_above_xy_deg']}", flush=True)
    print(f"Azimuths +X/-X: [0, 180] deg; separation {report['mounting']['separation_deg']:.3f} deg", flush=True)
    for name, row in reports.items():
        print(f"  {name}: {row['status']}, max update gap {row['max_update_gap_s']:.6f} s", flush=True)
    print(f"Report: {output}", flush=True)
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("branch", nargs="?", default="uprange")
    args = parser.parse_args()
    result = main(args.branch)
    sys.exit(0 if result["cadence_passed"] else 2)

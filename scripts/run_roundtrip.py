"""Main run: round trip with ONE assumed worst-case divert site. Hop 1 (start -> divert site) then hop 2 (divert site ->
exact start site). Config mission.mode "size" finds the minimum wet mass; "maximise_margin" keeps m_wet fixed.
Prints both hops, the propellant split, the sizing check and the fixed-gate divert verification.
Shows and saves trajectory plots after validation. Add --no-show to save without opening windows.
Usage: py scripts/run_roundtrip.py [branch] [--no-show]  (default: uprange, 150 m back toward launch)
"""
import argparse
import json
import math
import os
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from hopper.divert import gate_state, solve_branch
from hopper.lambert import min_dv_hop
from hopper.params import load_params, report_prechecks
from hopper.postprocess import phase_table, round_trip_summary, thrust_discard_flags
from hopper.problem import Problem
from hopper.validate import repropagate

CONFIG = os.path.join(ROOT, "config.yaml")


def solve_round_trip(p, branch, quiet=True, nodes=None, mode=None):
    """mode None = config mission.mode ("size": minimum wet mass with the reserve factor; else fixed m_wet)."""
    t0 = time.time()
    pr = Problem(p, branch, nodes=nodes, mode=mode)
    pr.set_guess()
    sol = pr.solve(print_level=0 if quiet else 5)
    return sol, time.time() - t0, pr


def print_table(rows):
    print(f"{'phase':<24}{'t_start':>9}{'t_end':>9}{'dur':>9}{'eng':>5}{'dv m/s':>10}{'prop kg':>10}")
    for r in rows:
        print(f"{r['phase']:<24}{r['t_start']:9.1f}{r['t_end']:9.1f}{r['duration']:9.1f}"
              f"{r['engine']:>5}{r['dv']:10.2f}{r['propellant']:10.2f}")


def main(branch="uprange", *, show_plots=True):
    p = load_params(CONFIG)
    print("== pre-checks"); report_prechecks(p)
    lam1 = min_dv_hop(p)
    lam2 = min_dv_hop(p, r1=p.mission.sites[branch], r2=p.mission.r_start)
    print(f"\nLambert hop 1 (start -> target): {lam1.dv:.2f} m/s, TOF {lam1.tof:.1f} s")
    print(f"Lambert hop 2 ({branch} site -> start): {lam2.dv:.2f} m/s, TOF {lam2.tof:.1f} s")

    size = p.mission.mode == "size"
    if size:
        print(f"\n== round trip, divert site = {branch}: SIZE mode, minimise m_wet with loaded propellant >= "
              f"{p.mission.reserve_factor:.2f} x propellant used")
    else:
        print(f"\n== round trip, divert site = {branch}, m_wet = {p.vehicle.m_wet:.0f} kg (fixed), maximise final mass")
    sol, wall, pr = solve_round_trip(p, branch)
    print(f"IPOPT: {sol.status} ({sol.iterations} iters), wall {wall:.1f} s, {pr.opti.nx} variables")
    if not sol.success:
        sys.exit("round trip did not converge")
    rows = phase_table(p, sol, branch)
    print_table(rows)
    summ = round_trip_summary(p, sol, branch, lam1.dv, lam2.dv)
    for tag in ("hop1", "hop2", "total"):
        print(f"  [{tag}]")
        for k, v in summ[tag].items():
            print(f"    {k:<24}{v:12.3f}")
    for lab, key in (("used", "propellant_used_split"), ("loaded", "propellant_loaded_split")):
        sp = summ[key]
        tot = sp["fuel"] + sp["oxidizer"]
        print(f"  propellant {lab}: {tot:.1f} kg = fuel {sp['fuel']:.1f} + oxidizer {sp['oxidizer']:.1f} "
              f"(O/F {p.engine.mixture_ratio})")
    if size:
        check_sized_mass(p, branch, sol)
    gate1 = sol.phases["braking"]["X"][:, -1]
    h_g1 = ((gate1[0:3] + p.mission.r_start) @ p.mission.up) - p.moon.radius
    print(f"  hop-1 gate altitude above the target (local vertical): {h_g1:.1f} m")
    print(f"  hop-1 moving gate: vertical speed {gate1[3:6] @ p.mission.up:.2f} m/s, "
          f"total speed {(gate1[3:6] @ gate1[3:6]) ** 0.5:.2f} m/s")
    flags = thrust_discard_flags(p, sol)
    print(f"  thrust-discard flags (|T| < 0.99 Gamma): {flags if flags else 'none'}")
    if flags:
        sys.exit("solution discards paid-for thrust; results were not saved")
    rep, td = repropagate(p, sol, branch)
    print("  re-propagation (solve_ivp), per-phase end error vs collocation:")
    for r in rep:
        print(f"    {r['phase']:<24} pos {r['pos_err']:.3e} m   vel {r['vel_err']:.3e} m/s")
    h1 = td.get("hop1", {})
    print(f"  hop-1 touchdown: pos err {h1.get('pos_err', float('nan')):.3e} m, vertical {h1.get('up_speed', float('nan')):.3f} m/s")
    print(f"  hop-2 touchdown (start site): pos err {td['pos_err']:.3e} m, speed {td['vel']:.3f} m/s, vertical {td['up_speed']:.3f} m/s")

    div = verify_divert(p, branch, sol)

    os.makedirs(os.path.join(ROOT, "results"), exist_ok=True)
    with open(os.path.join(ROOT, "results", f"roundtrip_{branch}.json"), "w") as fh:
        json.dump(dict(branch=branch, lambert=dict(hop1=lam1.dv, hop2=lam2.dv), summary=summ, divert=div,
                       touchdown_limits=dict(tilt_max_deg=math.degrees(p.limits.td_tilt_max),
                                             downward_speed_max=p.limits.td_vert_speed_max,
                                             horizontal_speed_max=p.limits.td_horiz_speed_max),
                       gate=dict(altitude=float(h_g1), velocity=gate1[3:6].tolist(),
                                 vertical_speed=float(gate1[3:6] @ p.mission.up)),
                       validation=dict(phases=rep, touchdown=td),
                       solve_wall_s=wall, n_variables=pr.opti.nx, table=rows), fh, indent=2)
    from hopper.plotting import plot_round_trip
    plot_round_trip(p, sol, branch, summ, os.path.join(ROOT, "results"), show=show_plots)
    return sol, summ


def check_sized_mass(p, branch, sol):
    """Acceptance check: re-run with m_wet fixed at the sized value; loaded / used propellant must come out at the
    reserve factor, and the maximised final mass must reproduce the sized one."""
    p2 = load_params(CONFIG, {"vehicle": {"m_wet": sol.m_wet}, "mission": {"mode": "maximise_margin"}})
    s2, _, _ = solve_round_trip(p2, branch)
    ratio = (s2.m_wet - p2.vehicle.m_dry) / (s2.m_wet - s2.z)
    print(f"  check: fixed m_wet = {sol.m_wet:.2f} kg -> {s2.status}, loaded/used = {ratio:.4f} "
          f"(required {p.mission.reserve_factor:.2f}), final mass {s2.z:.2f} vs sized {sol.z:.2f} kg")
    if not s2.success or abs(ratio / p.mission.reserve_factor - 1.0) > 1e-3:
        sys.exit("fixed-mass cross-check did not reproduce sizing; results were not saved")


def verify_divert(p, branch, sol):
    """Verify only the selected case from its fixed moving gate; no alternate landing sites are optimized."""
    res = solve_branch(p, gate_state(sol), branch, guess=sol.phases[f"terminal_{branch}"])
    print(f"\n== selected divert verification ({branch}, {p.mission.divert_radius:.0f} m)")
    print(f"  {res.status}; touchdown position error {res.touchdown_pos_err:.3e} m, "
          f"vertical speed {res.touchdown_up_speed:.3f} m/s, tilt {res.touchdown_tilt_deg:.2f} deg")
    if not res.success:
        sys.exit("selected fixed-gate divert did not converge")
    return dict(branch=branch, distance=p.mission.divert_radius, success=res.success, status=res.status,
                dv=res.dv, propellant=res.propellant, duration=res.phase["t"],
                touchdown_pos_err=res.touchdown_pos_err, touchdown_up_speed=res.touchdown_up_speed,
                touchdown_tilt_deg=res.touchdown_tilt_deg)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("branch", nargs="?", default="uprange")
    parser.add_argument("--no-show", action="store_true", help="Save plot PNGs without opening windows")
    args = parser.parse_args()
    main(args.branch, show_plots=not args.no_show)

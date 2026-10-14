"""Main run: round trip with ONE worst-case divert site. Hop 1 (start -> divert site) then hop 2 (divert site ->
exact start site). Config mission.mode "size" finds the minimum wet mass; "maximise_margin" keeps m_wet fixed.
Prints both hops, the propellant split, the sizing check and the cost of the divert.
Usage: py scripts/run_roundtrip.py [branch]      (default branch: downrange)
"""
import json
import os
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from hopper.divert import divert_descent_cost
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


def main(branch="downrange"):
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
    gate1 = sol.phases["hover"]["X"][:, -1]
    h_g1 = ((gate1[0:3] + p.mission.r_start) @ p.mission.up) - p.moon.radius
    print(f"  hop-1 gate altitude above the target (local vertical): {h_g1:.1f} m")
    flags = thrust_discard_flags(p, sol)
    print(f"  thrust-discard flags (|T| < 0.99 Gamma): {flags if flags else 'none'}")
    rep, td = repropagate(p, sol, branch)
    print("  re-propagation (solve_ivp), per-phase end error vs collocation:")
    for r in rep:
        print(f"    {r['phase']:<24} pos {r['pos_err']:.3e} m   vel {r['vel_err']:.3e} m/s")
    h1 = td.get("hop1", {})
    print(f"  hop-1 touchdown: pos err {h1.get('pos_err', float('nan')):.3e} m, vertical {h1.get('up_speed', float('nan')):.3f} m/s")
    print(f"  hop-2 touchdown (start site): pos err {td['pos_err']:.3e} m, speed {td['vel']:.3f} m/s, vertical {td['up_speed']:.3f} m/s")

    div = divert_report(p, branch, sol, summ)

    os.makedirs(os.path.join(ROOT, "results"), exist_ok=True)
    with open(os.path.join(ROOT, "results", f"roundtrip_{branch}.json"), "w") as fh:
        json.dump(dict(branch=branch, lambert=dict(hop1=lam1.dv, hop2=lam2.dv), summary=summ, divert=div,
                       solve_wall_s=wall, n_variables=pr.opti.nx, table=rows), fh, indent=2)
    return sol, summ


def check_sized_mass(p, branch, sol):
    """Acceptance check: re-run with m_wet fixed at the sized value; loaded / used propellant must come out at the
    reserve factor, and the maximised final mass must reproduce the sized one."""
    p2 = load_params(CONFIG, {"vehicle": {"m_wet": sol.m_wet}, "mission": {"mode": "maximise_margin"}})
    s2, _, _ = solve_round_trip(p2, branch)
    ratio = (s2.m_wet - p2.vehicle.m_dry) / (s2.m_wet - s2.z)
    print(f"  check: fixed m_wet = {sol.m_wet:.2f} kg -> {s2.status}, loaded/used = {ratio:.4f} "
          f"(required {p.mission.reserve_factor:.2f}), final mass {s2.z:.2f} vs sized {sol.z:.2f} kg")


def divert_report(p, branch, sol, summ):
    """Delta-v and propellant attributable to the divert: descent only, and the whole round trip."""
    print(f"\n== cost of the divert ({branch} site, {p.mission.divert_radius:.0f} m from the target)")
    d = divert_descent_cost(p, sol, branch)
    print("  [terminal descent only, from the same gate, gate mass %.1f kg]" % d["gate_mass"])
    print(f"    {'':<22}{'dv m/s':>9}{'prop kg':>10}{'time s':>9}")
    print(f"    {'nominal (target)':<22}{d['nominal']['dv']:9.2f}{d['nominal']['propellant']:10.2f}{d['nominal']['duration']:9.1f}")
    print(f"    {branch + ' (divert)':<22}{d['divert']['dv']:9.2f}{d['divert']['propellant']:10.2f}{d['divert']['duration']:9.1f}")
    print(f"    {'extra for the divert':<22}{d['extra_dv']:9.2f}{d['extra_propellant']:10.2f}")
    # whole round trip with a nominal landing, each case re-optimised (also captures hop 2 starting off-target)
    nsol, _, _ = solve_round_trip(p, "nominal")
    out = dict(descent_only=d)
    if nsol.success:
        if nsol.m_wet != sol.m_wet:
            print(f"    (sized wet mass: nominal landing {nsol.m_wet:.1f} kg, {branch} divert {sol.m_wet:.1f} kg, "
                  f"divert costs {sol.m_wet - nsol.m_wet:+.1f} kg of wet mass)")
            out["wet_mass_cost"] = sol.m_wet - nsol.m_wet
        lam1 = min_dv_hop(p)
        lam2n = min_dv_hop(p, r1=p.mission.sites["nominal"], r2=p.mission.r_start)
        ns = round_trip_summary(p, nsol, "nominal", lam1.dv, lam2n.dv)
        t, tn = summ["total"], ns["total"]
        print("  [whole round trip, divert vs nominal landing, each re-optimised]")
        print(f"    {'':<22}{'dv m/s':>9}{'prop kg':>10}")
        for lab, a in (("nominal landing", tn), (f"{branch} divert", t)):
            print(f"    {lab:<22}{a['finite_burn_dv']:9.2f}{a['propellant_used']:10.2f}")
        print(f"    {'extra for the divert':<22}{t['finite_burn_dv'] - tn['finite_burn_dv']:9.2f}"
              f"{t['propellant_used'] - tn['propellant_used']:10.2f}")
        for hop in ("hop1", "hop2"):
            print(f"      of which {hop}: dv {summ[hop]['finite_burn_dv'] - ns[hop]['finite_burn_dv']:+.2f} m/s, "
                  f"propellant {summ[hop]['propellant'] - ns[hop]['propellant']:+.2f} kg")
        out["round_trip"] = dict(nominal=tn, divert=t,
                                 extra_dv=t["finite_burn_dv"] - tn["finite_burn_dv"],
                                 extra_propellant=t["propellant_used"] - tn["propellant_used"])
    else:
        print("  nominal round trip did not converge; whole-trip comparison skipped")
    return out


if __name__ == "__main__":
    main(*sys.argv[1:2])

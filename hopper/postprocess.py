"""Delta-v, propellant and burn-timeline bookkeeping from a solved round trip."""
from __future__ import annotations

import numpy as np

from .params import Params

HOP_KINDS = ("rise", "ascent", "coast", "braking", "hover", "terminal")


def chain(branch: str) -> list:
    """Phase names in flight order: hop 1 (start -> divert site `branch`), then hop 2 (-> start site)."""
    return list(HOP_KINDS[:-1]) + [f"terminal_{branch}"] + [f"{k}2_{branch}" for k in HOP_KINDS]


def phase_dv(p: Params, ph: dict) -> float:
    """Delta-v = integral of Gamma/m dt = ve*ln(m0/m1) exactly for constant Isp."""
    m0, m1 = ph["X"][6, 0], ph["X"][6, -1]
    return p.engine.ve * np.log(m0 / m1) if ph["powered"] else 0.0


def phase_table(p: Params, sol, branch: str):
    rows, t = [], 0.0
    for name in chain(branch):
        ph = sol.phases[name]
        rows.append(dict(phase=name, t_start=t, t_end=t + ph["t"], duration=ph["t"],
                         engine="on" if ph["powered"] else "off", dv=phase_dv(p, ph),
                         propellant=ph["X"][6, 0] - ph["X"][6, -1]))
        t += ph["t"]
    return rows


def engine_starts(rows) -> int:
    starts, prev = 0, False
    for r in rows:
        on = r["engine"] == "on"
        starts += int(on and not prev)
        prev = on
    return starts


def propellant_split(p: Params, prop: float) -> dict:
    """Split a propellant mass into fuel and oxidizer with the mixture ratio O/F: fuel = P/(1+MR), ox = MR P/(1+MR)."""
    mr = p.engine.mixture_ratio
    return dict(fuel=prop / (1.0 + mr), oxidizer=prop * mr / (1.0 + mr))


def round_trip_summary(p: Params, sol, branch: str, lambert_dv_hop1: float, lambert_dv_hop2: float) -> dict:
    """Per-hop and total numbers for a solved round trip (hop 2 = divert site -> exact start site).

    Hop 1 includes the divert terminal descent. The surface stay between the hops is ignored, so engine starts
    are counted per hop and summed (the engine is off on the surface)."""
    rows = phase_table(p, sol, branch)
    hop1 = [r for r in rows if "2_" not in r["phase"]]
    hop2 = [r for r in rows if "2_" in r["phase"]]
    m_start = sol.phases["rise"]["X"][6, 0]
    m_land1 = sol.phases[f"terminal_{branch}"]["X"][6, -1]
    m_final = sol.phases[f"terminal2_{branch}"]["X"][6, -1]
    out = {}
    for tag, hop, lam in (("hop1", hop1, lambert_dv_hop1), ("hop2", hop2, lambert_dv_hop2)):
        dv = sum(r["dv"] for r in hop)
        dv_hover = sum(r["dv"] for r in hop if r["phase"].startswith("hover"))
        out[tag] = dict(lambert_dv=lam, finite_burn_dv=dv, hover_dv=dv_hover,
                        gravity_steering_loss=dv - dv_hover - lam,
                        propellant=sum(r["propellant"] for r in hop),
                        burn_time=sum(r["duration"] for r in hop if r["engine"] == "on"),
                        duration=sum(r["duration"] for r in hop), engine_starts=engine_starts(hop))
    prop_used = m_start - m_final
    loaded = m_start - p.vehicle.m_dry
    out["propellant_used_split"] = propellant_split(p, prop_used)
    out["propellant_loaded_split"] = propellant_split(p, loaded)
    out["total"] = dict(m_start=m_start, m_land_hop1=m_land1, m_final=m_final, propellant_used=prop_used,
                        propellant_loaded=loaded,
                        finite_burn_dv=out["hop1"]["finite_burn_dv"] + out["hop2"]["finite_burn_dv"],
                        engine_starts=out["hop1"]["engine_starts"] + out["hop2"]["engine_starts"],
                        margin=m_final - p.vehicle.m_dry,
                        loaded_over_used=loaded / prop_used)
    return out


def thrust_discard_flags(p: Params, sol, tol: float = 0.01):
    """Powered nodes where |T_vec| < 0.99 Gamma (solver is discarding thrust it pays fuel for)."""
    flags = {}
    for name, ph in sol.phases.items():
        if not ph["powered"]:
            continue
        T = np.linalg.norm(ph["U"][0:3], axis=0)
        G = ph["U"][3]
        bad = np.where(T < (1 - tol) * G)[0]
        if len(bad):
            flags[name] = bad.tolist()
    return flags

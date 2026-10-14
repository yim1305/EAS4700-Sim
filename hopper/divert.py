"""Cost of the divert descent: from a fixed gate state, solve the terminal descent to a landing site on its own."""
from __future__ import annotations

from dataclasses import dataclass
from types import SimpleNamespace

import casadi as ca
import numpy as np
from scipy.integrate import solve_ivp

from .constraints import GlideSlope, MassFloor, ThrustBounds, TiltRate, Touchdown
from .dynamics import make_numpy_rhs, make_scaled_dynamics
from .environment.terrain import FlatTerrain
from .initial_guess import _Builder
from .params import Params
from .phases import make_phase
from .postprocess import phase_dv
from .transcription import HermiteSimpson


@dataclass
class BranchResult:
    branch: str
    success: bool
    status: str
    iterations: int
    phase: dict | None = None      # same layout as Solution.phases entries (dimensional, offset positions)
    dv: float = float("nan")       # finite-burn delta-v of the terminal descent
    propellant: float = float("nan")
    m_final: float = float("nan")
    touchdown_pos_err: float = float("nan")    # re-propagation vs the branch site [m]
    touchdown_up_speed: float = float("nan")   # vertical speed at touchdown [m/s], negative = descending


def gate_state(sol) -> dict:
    """Gate = last node of the hop-1 hover (dimensional state, plus the last control for the tilt junction)."""
    hov = sol.phases["hover"]
    return dict(x=hov["X"][:, -1].copy(), u=hov["U"][:, -1].copy())


def solve_branch(p: Params, gate: dict, branch: str, nodes: int | None = None, print_level: int = 0,
                 terrain=None) -> BranchResult:
    """Terminal descent from the fixed gate state to this branch's site; maximises final mass."""
    s = p.scales
    n = nodes or p.solver.nodes["terminal"]
    terrain = terrain or FlatTerrain()
    opti = ca.Opti()
    f = make_scaled_dynamics(p)
    tr = HermiteSimpson()
    bnd = tuple(v / s.T for v in p.limits.phase_duration_bounds["terminal"])

    scale = np.array([s.L] * 3 + [s.V] * 3 + [s.M])
    site_abs = p.mission.sites[branch]
    site = ca.DM((site_abs - p.mission.r_start) / s.L)
    up = ca.DM(site_abs / np.linalg.norm(site_abs))
    # stand-in for the hover phase, so the tilt-rate junction constraint sees the gate control
    prev = SimpleNamespace(powered=True, U=ca.DM(gate["u"] / np.array([s.F] * 4)).reshape((4, 1)))
    ph = make_phase(opti, f"terminal_{branch}", "terminal", n, True, t_bounds=bnd, branch=branch, site=site,
                    site_up=up, prev=prev)
    args = (p, terrain)
    ph.constraints = [ThrustBounds(*args), MassFloor(*args), GlideSlope(*args), TiltRate(*args), Touchdown(*args)]

    opti.subject_to(ca.vec(tr.defects(f, ph.X, ph.U, ph.tdur, ph.n)) == 0)
    ph.apply_constraints(opti)
    opti.subject_to(ph.X[:, 0] == ca.DM(gate["x"] / scale))
    opti.subject_to(ph.X[0:3, -1] == site)
    opti.minimize(-ph.X[6, -1])

    x0 = gate["x"]
    lo, hi = p.limits.phase_duration_bounds["terminal"]
    t_term = float(np.clip(3.0 * np.sqrt(2.0 * max(_alt(p, x0), 10.0) / p.moon.g_surf), lo * 1.05, hi * 0.95))
    m_term = x0[6] * (1.0 - p.moon.g_surf * t_term / p.engine.ve)
    g = _Builder(p).powered(x0[0:3] + p.mission.r_start, np.zeros(3), x0[6], site_abs, np.zeros(3),
                            m_term, t_term, n)
    opti.set_initial(ph.X, g["X"] / scale[:, None])
    opti.set_initial(ph.U, g["U"] / np.array([s.F] * 4)[:, None])
    opti.set_initial(ph.tdur, g["t"] / s.T)

    opti.solver("ipopt", {"expand": True, "print_time": False},
                {"max_iter": int(p.solver.ipopt["max_iter"]), "tol": float(p.solver.ipopt["tol"]),
                 "print_level": print_level, "sb": "yes"})
    try:
        val = opti.solve().value
        ok = True
    except RuntimeError:
        val = opti.debug.value
        ok = False
    st = opti.stats()
    status = st["return_status"]
    ok = ok and status in ("Solve_Succeeded", "Solved_To_Acceptable_Level")
    out = dict(X=np.atleast_2d(val(ph.X)) * scale[:, None], U=np.atleast_2d(val(ph.U)) * s.F,
               t=float(val(ph.tdur)) * s.T, powered=True, kind="terminal", branch=branch)
    res = BranchResult(branch, ok, status, int(st.get("iter_count", -1)), out)
    res.dv = float(phase_dv(p, out))
    res.m_final = float(out["X"][6, -1])
    res.propellant = float(out["X"][6, 0] - out["X"][6, -1])
    res.touchdown_pos_err, res.touchdown_up_speed = _repropagate(p, out, branch)
    return res


def _alt(p: Params, x: np.ndarray) -> float:
    r = x[0:3] + p.mission.r_start
    return float(np.linalg.norm(r) - np.linalg.norm(p.mission.r_start))


def _repropagate(p: Params, ph: dict, branch: str):
    """Integrate the optimised controls (linear between nodes) from the gate and compare with the site."""
    f = make_numpy_rhs(p)
    T, n = ph["t"], ph["X"].shape[1] - 1
    tn = np.linspace(0.0, T, n + 1)
    U = ph["U"]
    sol = solve_ivp(lambda t, x: f(x, np.array([np.interp(t, tn, U[i]) for i in range(4)])), (0.0, T),
                    ph["X"][:, 0], rtol=1e-10, atol=1e-9, method="DOP853")
    x = sol.y[:, -1]
    site = p.mission.sites[branch]
    return (float(np.linalg.norm(x[0:3] - (site - p.mission.r_start))),
            float(x[3:6] @ (site / np.linalg.norm(site))))


def divert_descent_cost(p: Params, sol, branch: str) -> dict:
    """Cost of the divert descent alone: from the SAME gate (end of the hop-1 hover in `sol`), the terminal descent to
    the divert site against the terminal descent to the nominal site. Each is re-optimised from the gate."""
    gate = gate_state(sol)
    nom = solve_branch(p, gate, "nominal")
    div = solve_branch(p, gate, branch)
    ok = nom.success and div.success
    return dict(success=ok, gate_mass=float(gate["x"][6]),
                nominal=dict(dv=nom.dv, propellant=nom.propellant, duration=nom.phase["t"] if nom.phase else None),
                divert=dict(dv=div.dv, propellant=div.propellant, duration=div.phase["t"] if div.phase else None),
                extra_dv=div.dv - nom.dv, extra_propellant=div.propellant - nom.propellant)

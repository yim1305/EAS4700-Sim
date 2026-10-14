"""Independent check: re-propagate the optimised controls with scipy and compare with the collocation."""
from __future__ import annotations

import numpy as np
from scipy.integrate import solve_ivp

from .dynamics import make_numpy_rhs
from .params import Params
from .postprocess import chain


def repropagate(p: Params, sol, branch: str, rtol: float = 1e-10):
    """Integrate phase by phase from the optimiser's own phase-start state, with controls linearly
    interpolated between nodes (the collocation's control model). Returns per-phase end errors and the
    touchdown position/velocity error of the chained propagation: td["hop1"] at the divert site, then the
    vehicle restarts at rest on the surface and td at the start site (hop 2)."""
    f = make_numpy_rhs(p)
    report = []
    x_chain = None
    hop1_td = None
    for name in chain(branch):
        ph = sol.phases[name]
        if name.startswith("rise2_"):
            # surface stay between the hops: at rest in the Moon-fixed frame, same place, same mass
            hop1_td = _touchdown(p, x_chain, p.mission.sites[branch])
            x_chain = x_chain.copy()
            x_chain[3:6] = 0.0
        T, n = ph["t"], ph["X"].shape[1] - 1
        tn = np.linspace(0.0, T, n + 1)

        def u_of(t, U=ph["U"], tn=tn):
            return np.array([np.interp(t, tn, U[i]) for i in range(4)])

        x0 = ph["X"][:, 0] if x_chain is None else x_chain
        s = solve_ivp(lambda t, x: f(x, u_of(t)), (0.0, T), x0, rtol=rtol, atol=1e-9, method="DOP853")
        x_chain = s.y[:, -1]
        # error of this phase's propagation against the collocation end node (from the same start state)
        s2 = solve_ivp(lambda t, x: f(x, u_of(t)), (0.0, T), ph["X"][:, 0], rtol=rtol, atol=1e-9,
                       method="DOP853")
        e = s2.y[:, -1] - ph["X"][:, -1]
        report.append(dict(phase=name, pos_err=float(np.linalg.norm(e[0:3])),
                           vel_err=float(np.linalg.norm(e[3:6])), mass_err=float(abs(e[6]))))
    td = _touchdown(p, x_chain, p.mission.r_start)
    td["hop1"] = hop1_td
    return report, td


def _touchdown(p: Params, x, site_abs):
    """Position error to the site and speeds at the end of a chained propagation (x is the offset-frame state)."""
    return dict(pos_err=float(np.linalg.norm(x[0:3] - (site_abs - p.mission.r_start))),
                vel=float(np.linalg.norm(x[3:6])),
                up_speed=float(x[3:6] @ (site_abs / np.linalg.norm(site_abs))))

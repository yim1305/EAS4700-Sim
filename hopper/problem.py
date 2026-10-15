"""The round-trip optimal control problem: hop 1 (start site -> gate over the target -> divert site) then hop 2
(divert site -> gate -> exact start site, no divert). Each hop is rise, ascent burn, coast, braking, terminal
descent."""
from __future__ import annotations

from dataclasses import dataclass

import casadi as ca
import numpy as np

from .constraints import (GateAltitude, GateDescending, GateOverTarget, GlideSlope, MassFloor, MinAltitude,
                          ThrustBounds, TiltRate, Touchdown, TouchdownUpright, VerticalRise)
from .dynamics import make_scaled_dynamics
from .environment.terrain import FlatTerrain
from .initial_guess import build_guess
from .params import Params
from .phases import make_phase
from .transcription import HermiteSimpson

HOP_KINDS = ("rise", "ascent", "coast", "braking", "terminal")


@dataclass
class Solution:
    phases: dict          # name -> dict(X, U, t, powered, kind, branch)  (dimensional, offset positions)
    z: float              # final mass of the round trip [kg]
    status: str
    success: bool
    iterations: int
    solve_time: float
    m_wet: float = float("nan")   # initial mass: the config value, or the solved value in size mode


class Problem:
    def __init__(self, p: Params, branch: str = "uprange", nodes: dict | None = None, terrain=None,
                 transcription=None, mode: str | None = None):
        """`branch` is the divert site hop 1 lands on (one of mission.sites: nominal, uprange, downrange, left, right).

        mode "maximise_margin": fixed m_wet (config), maximise the final mass.
        mode "size": m_wet is a decision variable; minimise it subject to loaded propellant >= reserve_factor x the
        propellant used (m_final >= m_wet - (m_wet - m_dry) / reserve_factor, linear in m_wet).
        Default: config mission.mode."""
        mode = mode or p.mission.mode
        if mode not in ("size", "maximise_margin"):
            raise ValueError("mode must be 'size' or 'maximise_margin'")
        self.mode = mode
        self.p = p
        self.s = p.scales
        self.branch = branch
        self.nodes = dict(nodes or p.solver.nodes)
        self.terrain = terrain or FlatTerrain()
        self.tr = transcription or HermiteSimpson()
        self.f = make_scaled_dynamics(p)
        self.opti = ca.Opti()
        self._build()

    # ---- construction -----------------------------------------------------------------------
    def _make_hop(self, names, launch_up, site, site_up, gate_over_target):
        """The five phases of one hop; the braking end is a moving gate, with no required stop."""
        p, s, opti, n = self.p, self.s, self.opti, self.nodes
        bnd = {k: (lo / s.T, hi / s.T) for k, (lo, hi) in p.limits.phase_duration_bounds.items()}
        args = (p, self.terrain)
        rise = make_phase(opti, names[0], "rise", n["rise"], True, t_bounds=bnd["rise"], launch_up=launch_up)
        ascent = make_phase(opti, names[1], "ascent", n["ascent"], True, t_bounds=bnd["ascent"], prev=rise)
        coast = make_phase(opti, names[2], "coast", n["coast"], False, t_bounds=bnd["coast"], prev=ascent)
        braking = make_phase(opti, names[3], "braking", n["braking"], True, t_bounds=bnd["braking"], prev=coast)
        terminal = make_phase(opti, names[4], "terminal", n["terminal"], True, t_bounds=bnd["terminal"],
                              branch=self.branch, site=site, site_up=site_up, prev=braking)
        rise.constraints = [ThrustBounds(*args), MassFloor(*args), TiltRate(*args), VerticalRise(*args)]
        ascent.constraints = [ThrustBounds(*args), MassFloor(*args), MinAltitude(*args), TiltRate(*args)]
        coast.constraints = [MassFloor(*args), MinAltitude(*args)]
        braking.constraints = [ThrustBounds(*args), MassFloor(*args), MinAltitude(*args), TiltRate(*args),
                               GateAltitude(*args), GateDescending(*args)]
        if gate_over_target:
            braking.constraints.append(GateOverTarget(*args))
        terminal.constraints = [ThrustBounds(*args), MassFloor(*args), GlideSlope(*args), TiltRate(*args),
                                Touchdown(*args), TouchdownUpright(*args)]
        return [rise, ascent, coast, braking, terminal]

    def _build(self):
        p, s, opti, br = self.p, self.s, self.opti, self.branch
        m = p.mission
        site1 = m.sites[br]
        up_start = m.r_start / np.linalg.norm(m.r_start)
        up_site = site1 / np.linalg.norm(site1)

        # hop 1: start site -> divert site (gate over the target); hop 2: divert site -> exact start site
        self.hop1 = self._make_hop(("rise", "ascent", "coast", "braking", f"terminal_{br}"), up_start,
                                   ca.DM((site1 - m.r_start) / s.L), ca.DM(up_site), m.gate_over_target)
        self.hop2 = self._make_hop(tuple(f"{k}2_{br}" for k in HOP_KINDS), up_site,
                                   ca.DM.zeros(3, 1), ca.DM(up_start), False)   # start site = origin of the offset frame
        self.phases = self.hop1 + self.hop2

        for ph in self.phases:
            opti.subject_to(ca.vec(self.tr.defects(self.f, ph.X, ph.U, ph.tdur, ph.n)) == 0)
            ph.apply_constraints(opti)

        # boundary conditions and linkage
        rise1, term1, rise2, term2 = self.hop1[0], self.hop1[-1], self.hop2[0], self.hop2[-1]
        if self.mode == "size":
            self.mw = opti.variable()                       # nondimensional wet mass (scaled by config m_wet)
            opti.subject_to(opti.bounded(1.05 * p.vehicle.m_dry / s.M, self.mw, 3.0))
            opti.subject_to(rise1.X[0:6, 0] == 0)
            opti.subject_to(rise1.X[6, 0] == self.mw)
        else:
            self.mw = None
            opti.subject_to(rise1.X[:, 0] == ca.DM([0, 0, 0, 0, 0, 0, p.vehicle.m_wet / s.M]))
        for hop in (self.hop1, self.hop2):
            for a, b in zip(hop[:-1], hop[1:]):
                opti.subject_to(b.X[:, 0] == a.X[:, -1])
        opti.subject_to(term1.X[0:3, -1] == term1.site)      # lands on the divert site (altitude 0, flat terrain)
        # the vehicle sits on the surface between the hops (time ignored): same place, same mass, at rest
        opti.subject_to(rise2.X[0:3, 0] == term1.X[0:3, -1])
        opti.subject_to(rise2.X[3:6, 0] == 0)
        opti.subject_to(rise2.X[6, 0] == term1.X[6, -1])
        opti.subject_to(term2.X[0:3, -1] == term2.site)      # lands on the exact start site

        self.final = term2
        if self.mode == "size":
            # carry reserve_factor x the propellant used: m_final >= m_wet - (m_wet - m_dry) / f
            f, md = p.mission.reserve_factor, p.vehicle.m_dry / s.M
            opti.subject_to(term2.X[6, -1] >= self.mw - (self.mw - md) / f)
            opti.minimize(self.mw)
        else:
            opti.minimize(-term2.X[6, -1])                   # maximise the final mass

    # ---- guess ------------------------------------------------------------------------------
    @staticmethod
    def guess_from_solution(sol: "Solution") -> dict:
        """Turn a solved Solution into a guess dict (warm start; phase names must match)."""
        return {k: dict(X=v["X"], U=v["U"], t=v["t"]) for k, v in sol.phases.items()}

    def set_guess(self, guess: dict | None = None, m_wet0: float | None = None):
        """Set the initial guess. In size mode `m_wet0` [kg] seeds the wet-mass variable (default: config m_wet)."""
        s = self.s
        guess = guess or build_guess(self.p, self.branch, nodes=self.nodes)
        scale = np.array([s.L] * 3 + [s.V] * 3 + [s.M])
        uscale = np.array([s.F] * 4)
        for ph in self.phases:
            g = guess[ph.name]
            self.opti.set_initial(ph.X, g["X"] / scale[:, None])
            if ph.powered:
                self.opti.set_initial(ph.U, g["U"] / uscale[:, None])
            if not isinstance(ph.tdur, float):
                self.opti.set_initial(ph.tdur, g["t"] / s.T)
        if self.mode == "size":
            self.opti.set_initial(self.mw, (m_wet0 if m_wet0 is not None else self.p.vehicle.m_wet) / s.M)
        self.guess = guess

    # ---- solve ------------------------------------------------------------------------------
    def solve(self, print_level: int = 5) -> Solution:
        opts = {"max_iter": int(self.p.solver.ipopt["max_iter"]), "tol": float(self.p.solver.ipopt["tol"]),
                "acceptable_iter": 0, "bound_relax_factor": 0,
                "print_level": print_level, "sb": "yes"}
        self.opti.solver("ipopt", {"expand": True, "print_time": False}, opts)
        try:
            sol = self.opti.solve()
            val = sol.value
            success = True
        except RuntimeError:
            val = self.opti.debug.value
            success = False
        st = self.opti.stats()
        s = self.s
        out = {}
        scale = np.array([s.L] * 3 + [s.V] * 3 + [s.M])[:, None]
        for ph in self.phases:
            X = np.atleast_2d(val(ph.X)) * scale
            U = (np.atleast_2d(val(ph.U)) * s.F) if ph.powered else np.zeros((4, ph.n + 1))
            t = (float(val(ph.tdur)) if not isinstance(ph.tdur, float) else ph.tdur) * s.T
            out[ph.name] = dict(X=X, U=U, t=t, powered=ph.powered, kind=ph.kind, branch=ph.branch)
        z = float(val(self.final.X[6, -1])) * s.M
        m_wet = float(val(self.mw)) * s.M if self.mode == "size" else self.p.vehicle.m_wet
        return Solution(out, z, st["return_status"], success and
                        st["return_status"] in ("Solve_Succeeded", "Solved_To_Acceptable_Level"),
                        int(st.get("iter_count", -1)), float(st["t_wall_total"]) if "t_wall_total" in st else float("nan"),
                        m_wet)

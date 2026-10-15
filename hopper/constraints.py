"""Constraints: each is a class with apply(opti, phase). Add a new one with a class plus one line in problem.py.

All quantities are nondimensional (see Params.scales); positions are offsets from the start site.
"""
from __future__ import annotations

import casadi as ca
import numpy as np

from .environment.terrain import Terrain, altitude
from .params import Params


class Constraint:
    def apply(self, opti, phase):
        raise NotImplementedError


class _WithParams(Constraint):
    def __init__(self, p: Params, terrain: Terrain | None = None):
        self.p = p
        self.s = p.scales
        self.terrain = terrain
        self.rs = p.mission.r_start / self.s.L          # nondimensional start-site position
        self.R = p.moon.radius / self.s.L

    def alt(self, r):
        """Nondimensional altitude of offset position r (3 x n)."""
        if self.terrain is None:
            raise RuntimeError("constraint needs a terrain model")
        # terrain.height takes dimensional r_abs; scale in/out
        class _T:
            def height(_, r_abs):
                return self.terrain.height(tuple(c * self.s.L for c in r_abs)) / self.s.L
        return altitude(self.rs, self.R, r, _T())


class ThrustBounds(_WithParams):
    """T_min <= Gamma <= T_max and |T_vec| <= Gamma on powered phases."""

    def apply(self, opti, phase):
        if not phase.powered:
            return
        T = phase.U[0:3, :]
        G = phase.U[3, :]
        lo, hi = self.p.engine.T_min / self.s.F, self.p.engine.T_max / self.s.F
        opti.subject_to(opti.bounded(lo, G, hi))
        opti.subject_to(ca.sum1(T * T) <= G * G)


class MassFloor(_WithParams):
    """m >= m_dry at all nodes."""

    def apply(self, opti, phase):
        opti.subject_to(phase.X[6, :] >= self.p.vehicle.m_dry / self.s.M)


class MinAltitude(_WithParams):
    """h >= min_altitude (ascent, coast, braking)."""

    def apply(self, opti, phase):
        opti.subject_to(self.alt(phase.X[0:3, :]) >= self.p.limits.min_altitude / self.s.L)


class GateAltitude(_WithParams):
    """Altitude window at the braking end (the moving gate), not throughout braking."""

    def apply(self, opti, phase):
        lo, hi = self.p.mission.gate_alt_bounds
        h = self.alt(phase.X[0:3, -1])
        opti.subject_to(h >= lo / self.s.L)
        opti.subject_to(h <= hi / self.s.L)


class GlideSlope(_WithParams):
    """h_rel >= tan(slope) * d_horiz relative to this branch's site (terminal descents).

    Written as h_rel >= 0 and h_rel^2 >= tan^2 * d_horiz^2. The final node (on the site, where both are
    zero) is skipped; it is pinned by the landing-site equality.
    """

    def apply(self, opti, phase):
        up = phase.site_up
        d = phase.X[0:3, 0:phase.n] - phase.site
        hrel = ca.mtimes(up.T, d)
        dh2 = ca.sum1(d * d) - hrel * hrel
        opti.subject_to(hrel >= 0)
        opti.subject_to(hrel * hrel >= np.tan(self.p.limits.glide_slope) ** 2 * dh2)


class TiltRate(_WithParams):
    """u_k . u_{k+1} >= cos(rate * dt) with u = T_vec / Gamma, inside a phase and across a junction."""

    def apply(self, opti, phase):
        if not phase.powered:
            return
        rate_nd = self.p.limits.tilt_rate_max * self.s.T      # rad per nondimensional time
        T, G = phase.U[0:3, :], phase.U[3, :]
        n = phase.n
        dt = phase.tdur / n
        dot = ca.sum1(T[:, 0:n] * T[:, 1:n + 1])
        opti.subject_to(dot >= G[0:n] * G[1:n + 1] * ca.cos(rate_nd * dt))
        prev = phase.prev
        if prev is not None and prev.powered:
            # across the junction the control moves from the last node of prev to the first node of this
            # phase; allow one interval of this phase for the rotation (a zero-length gap would force
            # exactly parallel thrust and make the constraint degenerate)
            d = ca.sum1(prev.U[0:3, -1] * T[:, 0])
            opti.subject_to(d >= prev.U[3, -1] * G[0] * ca.cos(rate_nd * dt))


class Touchdown(_WithParams):
    """Descending or level, with bounded downward and horizontal speeds at the last node."""

    def apply(self, opti, phase):
        v = phase.X[3:6, -1]
        up = phase.site_up
        vup = ca.dot(up, v)
        vlimit = self.p.limits.td_vert_speed_max / self.s.V
        hlimit = self.p.limits.td_horiz_speed_max / self.s.V
        if vlimit == 0:
            opti.subject_to(vup == 0)
        else:
            opti.subject_to(opti.bounded(-1, vup / vlimit, 0))
        if hlimit == 0:
            e1, e2 = _tangents(np.asarray(up, dtype=float).reshape(3))
            opti.subject_to(ca.dot(ca.DM(e1), v) == 0)
            opti.subject_to(ca.dot(ca.DM(e2), v) == 0)
        else:
            # Divide by one power of the limit: order-one boundary gradients without a huge Hessian.
            vhorizontal = v - vup * up
            opti.subject_to(ca.dot(vhorizontal, vhorizontal) / hlimit <= hlimit)


class TouchdownUpright(_WithParams):
    """Bound the final thrust/body-up angle to the landing-site surface normal (no attitude dynamics).

    Since |T| <= Gamma, up.T*T >= cos(angle)*Gamma also guarantees the actual thrust angle.
    The zero-angle case uses two tangential equalities to avoid a degenerate zero-width cone.
    """

    def apply(self, opti, phase):
        T = phase.U[0:3, -1]
        up = phase.site_up
        angle = self.p.limits.td_tilt_max
        if angle == 0:
            e1, e2 = _tangents(np.asarray(up, dtype=float).reshape(3))
            opti.subject_to(ca.dot(ca.DM(e1), T) == 0)
            opti.subject_to(ca.dot(ca.DM(e2), T) == 0)
            opti.subject_to(ca.dot(up, T) >= 0)
        else:
            opti.subject_to(ca.dot(up, T) >= np.cos(angle) * phase.U[3, -1])


# ---- phase-specific boundary conditions -------------------------------------------------------

class VerticalRise(_WithParams):
    """Phase 1: horizontal velocity zero at every node, ends at h = h_rise."""

    def apply(self, opti, phase):
        e1, e2 = _tangents(phase.launch_up)
        v = phase.X[3:6, :]
        opti.subject_to(ca.mtimes(ca.DM(e1).T, v) == 0)
        opti.subject_to(ca.mtimes(ca.DM(e2).T, v) == 0)
        opti.subject_to(self.alt(phase.X[0:3, -1]) == self.p.limits.h_rise / self.s.L)


class GateDescending(_WithParams):
    """The gate is reached descending or level; its velocity is otherwise free."""

    def apply(self, opti, phase):
        r_abs = self.rs + phase.X[0:3, -1]
        opti.subject_to(ca.dot(r_abs, phase.X[3:6, -1]) <= 0)


class GateOverTarget(_WithParams):
    """Hop-1 gate (end of braking) horizontally directly above the target: no tangential offset from the
    target's local vertical, so the trunk does not favour any one divert site. Altitude stays free."""

    def apply(self, opti, phase):
        m = self.p.mission
        e1, e2 = _tangents(m.up)
        d = phase.X[0:3, -1] - ca.DM((m.r_target - m.r_start) / self.s.L)
        opti.subject_to(ca.mtimes(ca.DM(e1).T, d) == 0)
        opti.subject_to(ca.mtimes(ca.DM(e2).T, d) == 0)


def _tangents(up):
    """Two unit vectors spanning the plane tangent to `up` (two equations; a redundant third breaks LICQ)."""
    axis = np.eye(3)[int(np.argmin(np.abs(up)))]
    e1 = axis - (axis @ up) * up
    e1 /= np.linalg.norm(e1)
    return e1, np.cross(up, e1)

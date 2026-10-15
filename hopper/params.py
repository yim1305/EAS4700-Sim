"""Single source of truth: loads config.yaml into frozen dataclasses and derives values."""
from __future__ import annotations

import copy
import math
from dataclasses import dataclass

import numpy as np
import yaml

@dataclass(frozen=True)
class Moon:
    mu: float
    radius: float
    omega: float
    g_surf: float            # mu / R^2, used for scaling and the T/W checks

    @property
    def omega_vec(self) -> np.ndarray:
        return np.array([0.0, 0.0, self.omega])


@dataclass(frozen=True)
class Engine:
    n_engines: int
    thrust_per_engine: float
    isp_vac: float
    min_throttle: float
    g0: float
    T_max: float
    T_min: float
    mdot_max: float
    ve: float                # g0 * Isp
    mixture_ratio: float = 1.65           # O/F by mass, only used to split the propellant into oxidizer and fuel


@dataclass(frozen=True)
class Vehicle:
    m_wet: float
    m_dry: float
    m_prop: float


@dataclass(frozen=True)
class Mission:
    divert_radius: float
    gate_alt_bounds: tuple
    r_start: np.ndarray                  # absolute, Moon-fixed
    r_target: np.ndarray
    up: np.ndarray                       # target-local frame
    downrange: np.ndarray
    left: np.ndarray
    sites: dict                          # branch -> absolute site position
    L: float                             # start-to-target distance (chord)
    gate_over_target: bool = False       # hop-1 gate horizontally directly above the target
    mode: str = "maximise_margin"        # size | maximise_margin
    reserve_factor: float = 1.1          # loaded propellant / worst-case round-trip propellant used


@dataclass(frozen=True)
class Limits:
    h_rise: float
    min_altitude: float
    glide_slope: float                   # rad
    tilt_rate_max: float                 # rad/s
    td_vert_speed_max: float
    td_horiz_speed_max: float
    phase_duration_bounds: dict
    td_tilt_max: float                   # rad; body-up approximated by resultant thrust direction


@dataclass(frozen=True)
class Solver:
    nodes: dict
    ipopt: dict


@dataclass(frozen=True)
class Params:
    moon: Moon
    engine: Engine
    vehicle: Vehicle
    mission: Mission
    limits: Limits
    solver: Solver

    @property
    def tw0(self) -> float:
        """Initial lunar thrust-to-weight at wet mass."""
        return self.engine.T_max / (self.vehicle.m_wet * self.moon.g_surf)

    @property
    def scales(self) -> "Scales":
        g = self.moon.g_surf
        L = self.mission.L
        T = math.sqrt(L / g)
        M = self.vehicle.m_wet
        return Scales(L=L, T=T, M=M, V=L / T, F=M * g, g=g)


@dataclass(frozen=True)
class Scales:
    L: float   # length
    T: float   # time
    M: float   # mass
    V: float   # velocity
    F: float   # force
    g: float   # acceleration


def site_vector(lat_deg: float, lon_deg: float, elev: float, radius: float) -> np.ndarray:
    lat, lon = math.radians(lat_deg), math.radians(lon_deg)
    return (radius + elev) * np.array(
        [math.cos(lat) * math.cos(lon), math.cos(lat) * math.sin(lon), math.sin(lat)])


def _unit(v):
    return v / np.linalg.norm(v)


def target_frame(r_start: np.ndarray, r_target: np.ndarray):
    up = _unit(r_target)
    d = r_target - r_start
    d = d - np.dot(d, up) * up
    if np.linalg.norm(d) < 1e-9:
        raise ValueError("start and target sites coincide; downrange is undefined")
    downrange = _unit(d)
    left = np.cross(up, downrange)
    return up, downrange, left


def divert_sites(r_target, downrange, left, radius_div):
    """Target moved radius_div along +/-downrange and +/-left, projected back to the surface radius."""
    Rt = np.linalg.norm(r_target)
    out = {}
    for name, direction in (("downrange", downrange), ("uprange", -downrange),
                            ("left", left), ("right", -left)):
        p = r_target + radius_div * direction
        out[name] = p / np.linalg.norm(p) * Rt
    return out


def _deep_update(base: dict, upd: dict) -> dict:
    for k, v in upd.items():
        if isinstance(v, dict) and isinstance(base.get(k), dict):
            _deep_update(base[k], v)
        else:
            base[k] = v
    return base


def _coerce_numbers(obj):
    """PyYAML (YAML 1.1) reads '4.9e12' (no exponent sign) as a string; turn such strings into floats."""
    if isinstance(obj, dict):
        return {k: _coerce_numbers(v) for k, v in obj.items()}
    if isinstance(obj, list):
        return [_coerce_numbers(v) for v in obj]
    if isinstance(obj, str):
        try:
            return float(obj)
        except ValueError:
            return obj
    return obj


def _require(cond: bool, msg: str):
    if not cond:
        raise ValueError(f"config error: {msg}")


def load_params(path: str, overrides: dict | None = None) -> Params:
    """Load and validate config.yaml. `overrides` is a nested dict deep-merged over the file
    (used for sweeps and sensitivity tests, so no module ever hardcodes a value)."""
    with open(path, "r") as fh:
        cfg = _coerce_numbers(yaml.safe_load(fh))
    if overrides:
        cfg = _deep_update(copy.deepcopy(cfg), overrides)

    m = cfg["moon"]
    _require(m["radius"] > 1e5, "moon.radius must be in metres (got < 100 km)")
    _require(m["mu"] > 1e9, "moon.mu must be in m^3/s^2")
    _require(0 <= m["omega"] < 1e-3, "moon.omega must be in rad/s")
    moon = Moon(m["mu"], m["radius"], m["omega"], m["mu"] / m["radius"] ** 2)

    e = cfg["engine"]
    _require(e["n_engines"] >= 1, "engine.n_engines >= 1")
    _require(e["thrust_per_engine"] > 0 and e["isp_vac"] > 0, "thrust and Isp must be positive")
    _require(0 < e["min_throttle"] <= 1, "engine.min_throttle must be in (0, 1]")
    g0 = cfg["g0"]
    T_max = e["n_engines"] * e["thrust_per_engine"]
    engine = Engine(int(e["n_engines"]), e["thrust_per_engine"], e["isp_vac"], e["min_throttle"],
                    g0, T_max, e["min_throttle"] * T_max, T_max / (g0 * e["isp_vac"]),
                    g0 * e["isp_vac"], float(e.get("mixture_ratio", 1.65)))

    v = cfg["vehicle"]
    _require(v["m_wet"] > v["m_dry"] > 0, "need m_wet > m_dry > 0 (kg)")
    vehicle = Vehicle(v["m_wet"], v["m_dry"], v["m_wet"] - v["m_dry"])

    s = cfg["sites"]
    for k in ("start", "target"):
        _require(-90 <= s[k]["lat"] <= 90, f"sites.{k}.lat must be in degrees [-90, 90]")
        _require(-360 <= s[k]["lon"] <= 360, f"sites.{k}.lon must be in degrees")
    r_start = site_vector(s["start"]["lat"], s["start"]["lon"], s["start"]["elev"], moon.radius)
    r_target = site_vector(s["target"]["lat"], s["target"]["lon"], s["target"]["elev"], moon.radius)
    up, downrange, left = target_frame(r_start, r_target)

    ms = cfg["mission"]
    _require(ms["divert_radius"] >= 0, "mission.divert_radius >= 0")
    _require(ms.get("mode", "maximise_margin") in ("size", "maximise_margin"),
             "mission.mode must be 'size' or 'maximise_margin'")
    _require(float(ms.get("propellant_reserve_factor", 1.1)) >= 1.0, "mission.propellant_reserve_factor >= 1")
    lo, hi = ms["gate_alt_bounds"]
    _require(0 <= lo < hi, "mission.gate_alt_bounds must be [lo, hi] with 0 <= lo < hi")
    sites = {"nominal": r_target}
    sites.update(divert_sites(r_target, downrange, left, ms["divert_radius"]))
    mission = Mission(ms["divert_radius"], (lo, hi), r_start, r_target,
                      up, downrange, left, sites, float(np.linalg.norm(r_target - r_start)),
                      bool(ms.get("gate_over_target", False)), ms.get("mode", "maximise_margin"),
                      float(ms.get("propellant_reserve_factor", 1.1)))

    li = cfg["limits"]
    _require(0 < li["glide_slope_deg"] < 90, "limits.glide_slope_deg in (0, 90)")
    _require(li["tilt_rate_max_deg_s"] > 0, "limits.tilt_rate_max_deg_s > 0")
    _require(0 <= li["td_tilt_max_deg"] < 90, "limits.td_tilt_max_deg in [0, 90)")
    _require(li["td_vert_speed_max"] >= 0 and li["td_horiz_speed_max"] >= 0,
             "touchdown speed limits must be nonnegative")
    pdb = {k: tuple(vv) for k, vv in li["phase_duration_bounds"].items()}
    for k in ("rise", "ascent", "coast", "braking", "terminal"):
        _require(k in pdb and 0 < pdb[k][0] < pdb[k][1], f"limits.phase_duration_bounds.{k}")
    limits = Limits(li["h_rise"], li["min_altitude"], math.radians(li["glide_slope_deg"]),
                    math.radians(li["tilt_rate_max_deg_s"]), li["td_vert_speed_max"],
                    li["td_horiz_speed_max"], pdb, math.radians(li["td_tilt_max_deg"]))

    so = cfg["solver"]
    for k in ("rise", "ascent", "coast", "braking", "terminal"):
        _require(int(so["nodes"][k]) >= 2, f"solver.nodes.{k} >= 2")
    solver = Solver({k: int(vv) for k, vv in so["nodes"].items()}, dict(so["ipopt"]))

    return Params(moon, engine, vehicle, mission, limits, solver)


def report_prechecks(p: Params, out=print) -> list:
    """Print start-up pre-checks; returns the list of warning strings."""
    warnings = []
    g = p.moon.g_surf
    out(f"g_moon = {g:.4f} m/s^2   T_max = {p.engine.T_max:.1f} N   "
        f"T_min = {p.engine.T_min:.1f} N   mdot_max = {p.engine.mdot_max:.4f} kg/s")
    out(f"m_wet = {p.vehicle.m_wet:.1f} kg   m_dry = {p.vehicle.m_dry:.1f} kg   "
        f"m_prop = {p.vehicle.m_prop:.1f} kg   hop distance L = {p.mission.L / 1e3:.2f} km")
    ok = p.tw0 > 1.0
    out(f"[{'OK' if ok else 'FAIL'}] initial lunar T/W = {p.tw0:.3f} (need > 1 at wet mass)")
    if not ok:
        warnings.append("T/W <= 1 at wet mass: vehicle cannot lift off")
    h_min = math.tan(p.limits.glide_slope) * p.mission.divert_radius
    ok = p.mission.gate_alt_bounds[0] > h_min
    out(f"[{'OK' if ok else 'WARN'}] gate floor {p.mission.gate_alt_bounds[0]:.0f} m vs glide-slope minimum "
        f"{h_min:.0f} m (tan(slope) x divert radius) for a gate directly above the target")
    if not ok:
        warnings.append("gate_alt_bounds floor is not above the glide-slope minimum: divert branches infeasible")
    return warnings

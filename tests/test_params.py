import numpy as np
import pytest

from hopper.params import report_prechecks


def test_sites_on_sphere(params):
    R = params.moon.radius
    assert np.linalg.norm(params.mission.r_start) == pytest.approx(R, abs=1e-6)
    assert np.linalg.norm(params.mission.r_target) == pytest.approx(R, abs=1e-6)


def test_start_is_in_southern_hemisphere(params):
    assert params.mission.r_start[2] < 0


def test_target_frame_orthonormal(params):
    m = params.mission
    for a, b in ((m.up, m.downrange), (m.up, m.left), (m.downrange, m.left)):
        assert abs(a @ b) < 1e-12
    for v in (m.up, m.downrange, m.left):
        assert np.linalg.norm(v) == pytest.approx(1.0, abs=1e-12)
    assert np.cross(m.up, m.downrange) @ m.left == pytest.approx(1.0, abs=1e-12)


def test_downrange_points_toward_start_to_target(params):
    m = params.mission
    assert (m.r_target - m.r_start) @ m.downrange > 0


def test_divert_sites_100m_from_target_on_surface(params):
    m = params.mission
    Rt = np.linalg.norm(m.r_target)
    for name in ("uprange", "downrange", "left", "right"):
        site = m.sites[name]
        assert np.linalg.norm(site) == pytest.approx(Rt, abs=1e-6)
        assert np.linalg.norm(site - m.r_target) == pytest.approx(100.0, abs=1e-3)


def test_divert_directions(params):
    m = params.mission
    d = {k: m.sites[k] - m.r_target for k in ("uprange", "downrange", "left", "right")}
    assert d["downrange"] @ m.downrange > 99.9
    assert d["uprange"] @ m.downrange < -99.9
    assert d["left"] @ m.left > 99.9
    assert d["right"] @ m.left < -99.9


def test_derived_values(params):
    e, v = params.engine, params.vehicle
    assert e.T_max == pytest.approx(e.n_engines * e.thrust_per_engine)
    assert e.T_min == pytest.approx(e.min_throttle * e.T_max)
    assert e.mdot_max == pytest.approx(e.T_max / (e.g0 * e.isp_vac))
    assert e.ve == pytest.approx(e.g0 * e.isp_vac)
    assert v.m_prop == pytest.approx(v.m_wet - v.m_dry)
    assert params.tw0 == pytest.approx(e.T_max / (v.m_wet * params.moon.g_surf))


def test_prechecks_pass_for_default(params):
    assert report_prechecks(params, out=lambda *_: None) == []


def test_hover_warning_when_tmin_too_high(make_params):
    p = make_params({"engine": {"min_throttle": 0.5}})
    warns = report_prechecks(p, out=lambda *_: None)
    assert any("hover" in w for w in warns)


def test_rejects_bad_units(make_params):
    with pytest.raises(ValueError):
        make_params({"moon": {"radius": 1737.4}})   # km instead of m
    with pytest.raises(ValueError):
        make_params({"sites": {"start": {"lat": -189.9}}})
    with pytest.raises(ValueError):
        make_params({"vehicle": {"m_dry": 1e6}})        # dry mass above wet mass


def test_divert_radius_override(make_params):
    p = make_params({"mission": {"divert_radius": 50.0}})
    assert np.linalg.norm(p.mission.sites["left"] - p.mission.r_target) == pytest.approx(50.0, abs=1e-3)

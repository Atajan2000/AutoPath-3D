"""Phase 3 unit tests: dynamics, controller limits, swept-step clearance bounds, determinism."""

import numpy as np
import pytest

from autopath3d.environments import Box, get_scene
from autopath3d.simulation import (GUSTS, ControllerConfig, Disturbance, TrackingController, WorldDistance,
                                   discrete_spectral_radius, refine_min, step_bounds, step_control_points,
                                   zoh_step)
from autopath3d.simulation.controller import clip_norm, speed_governor


def test_zoh_step_is_exact_for_constant_acceleration():
    p, v, a = np.array([1.0, -2.0, 3.0]), np.array([0.5, 1.5, -0.2]), np.array([0.3, -1.1, 2.0])
    T = 0.7
    pk, vk = p.copy(), v.copy()
    for _ in range(70):                                     # 70 steps of 0.01 s
        pk, vk = zoh_step(pk, vk, a, 0.01)
    assert np.allclose(pk, p + v * T + 0.5 * a * T * T, atol=1e-12)
    assert np.allclose(vk, v + a * T, atol=1e-12)
    # the step's Bezier control points reproduce the quadratic motion exactly
    P = step_control_points(p, v, zoh_step(p, v, a, 0.01)[0], 0.01)
    u = 0.37
    bez = (1 - u) ** 2 * P[0] + 2 * u * (1 - u) * P[1] + u ** 2 * P[2]
    tau = u * 0.01
    assert np.allclose(bez, p + v * tau + 0.5 * a * tau * tau, atol=1e-15)


def test_closed_loop_discretisation_is_stable_and_gains_are_moderate():
    c = ControllerConfig()
    assert c.kd ** 2 == pytest.approx(4 * c.kp)             # critically damped
    for dt in (0.005, 0.01, 0.02, 0.05):
        assert discrete_spectral_radius(c.kp, c.kd, dt) < 1.0
    assert discrete_spectral_radius(c.kp, c.kd, 0.5) > 1.0  # the check can detect an unstable step size


def test_acceleration_saturation_and_speed_governor():
    rng = np.random.default_rng(3)
    ctrl = TrackingController(ControllerConfig())
    for _ in range(500):
        p, v = rng.normal(size=3) * 5, rng.normal(size=3)
        v *= rng.uniform(0, 3.0) / np.linalg.norm(v)
        a, sat, gov = ctrl.track(p, v, rng.normal(size=3) * 5, rng.normal(size=3) * 3, rng.normal(size=3) * 3, 0.01)
        assert np.linalg.norm(a) <= 2.0 + 1e-12
        assert np.linalg.norm(v + a * 0.01) <= 3.0 + 1e-12
    # governor is non-expansive: never increases |a| and lands exactly on the speed limit
    v = np.array([2.995, 0, 0])
    a, gov = speed_governor(v, np.array([1.5, 0.5, 0.0]), 0.01, 3.0)
    assert gov and np.linalg.norm(v + a * 0.01) == pytest.approx(3.0)
    assert np.linalg.norm(a) <= np.linalg.norm([1.5, 0.5, 0.0])
    b, s = clip_norm(np.array([3.0, 4.0, 0.0]), 2.0)
    assert s and np.linalg.norm(b) == pytest.approx(2.0)


def test_braking_command_is_bounded_and_reaches_rest():
    ctrl = TrackingController(ControllerConfig())
    v, p = np.array([2.0, -1.0, 0.5]), np.zeros(3)
    v0 = v / np.linalg.norm(v)
    for _ in range(1000):
        a, done = ctrl.brake(v, 0.01)
        assert np.linalg.norm(a) <= 3.0 + 1e-12
        p, v = zoh_step(p, v, a, 0.01)
        assert v @ v0 >= -1e-12                              # never reverses
        if done:
            break
    assert done and np.linalg.norm(v) < 1e-12
    s = np.linalg.norm([2.0, -1.0, 0.5])
    assert np.linalg.norm(p) == pytest.approx(s * s / 6.0, abs=0.02)   # v^2 / (2 a_dec)


@pytest.mark.parametrize("name", ["warehouse", "industrial", "urban_canyon"])
def test_world_distance_matches_ground_truth_scene_distance(name):
    scene = get_scene(name)
    w = WorldDistance.of_scene(scene)
    P = np.random.default_rng(0).uniform(scene.lo - 1, scene.hi + 1, size=(3000, 3))
    assert np.allclose(w(P), scene.distance(P), atol=1e-12)
    q, d, names = w.nearest(P[(w(P) > 0)][:300])
    inside = P[(w(P) > 0)][:300]
    assert np.allclose(np.linalg.norm(inside - q, axis=1), d, atol=1e-9)
    assert np.allclose(d, w(inside), atol=1e-9)


def test_step_bounds_bracket_dense_samples_and_refinement_is_tight():
    scene = get_scene("industrial")                          # boxes and cylinders
    w = WorldDistance.of_scene(scene)
    rng = np.random.default_rng(1)
    p = rng.uniform(scene.lo + 0.5, scene.hi - 0.5, size=(400, 3))
    v = rng.normal(size=(400, 3)) * 2.0
    a = rng.normal(size=(400, 3)) * 2.0
    dt = 0.05
    p1 = p + v * dt + 0.5 * a * dt * dt
    P = step_control_points(p, v, p1, dt)
    lo, hi = step_bounds(P, w)
    u = np.linspace(0, 1, 201)[:, None, None]
    dense = (1 - u) ** 2 * P[None, :, 0] + 2 * u * (1 - u) * P[None, :, 1] + u ** 2 * P[None, :, 2]
    dmin = w(dense.reshape(-1, 3)).reshape(201, 400).min(axis=0)
    assert np.all(lo <= dmin + 1e-12) and np.all(dmin <= hi + 1e-12)
    L, U = refine_min(P, w, tol=1e-5)
    assert L <= dmin.min() + 1e-12 <= U + 1e-12 and U - L <= 1e-5 + 1e-12


def test_swept_bound_detects_contact_missed_by_position_samples():
    """A thin plate between the samples of one control step is not missed."""
    plate = Box((0.004, -1, -1), (0.007, 1, 1), "plate")
    w = WorldDistance((-5, -5, -5), (5, 5, 5), [plate])
    p, v, dt = np.zeros(3), np.array([3.0, 0, 0]), 0.01   # 3 cm step through the plate
    P = step_control_points(p, v, p + v * dt, dt)[None]
    sampled = w(P[0, [0, 2]])
    mid = w(0.25 * P[0, 0] + 0.5 * P[0, 1] + 0.25 * P[0, 2])
    assert sampled.min() > 0.003 and mid[0] > 0.003        # endpoint and midpoint samples look clear
    lo, hi = refine_min(P, w)
    assert lo <= 0.0 <= hi + 1e-9 and hi < 1e-4             # the swept check finds the contact


def test_disturbance_is_deterministic_and_bounded():
    t = np.linspace(0, 200, 20001)
    d = np.array([GUSTS(x) for x in t])
    assert np.all(np.abs(d) <= GUSTS.axis_bound() + 1e-12)
    assert np.linalg.norm(d, axis=1).max() <= GUSTS.norm_bound()
    assert np.array_equal(d, np.array([GUSTS(x) for x in t]))
    assert np.allclose(Disturbance()(3.0), 0.0)

import numpy as np
import pytest

from autopath3d.collision import RobotModel
from autopath3d.environments import Box, get_scene
from autopath3d.evaluation.trajectory_validation import validate_trajectory
from autopath3d.evaluation.validation import validate_path
from autopath3d.pipeline import ClearancePolicy, run_pipeline, trajectory_checker
from autopath3d.trajectories import (PiecewisePolynomial, TrajectoryLimits, certify_curve, generate_trajectory,
                                     power_to_bernstein, quintic_hermite, shortcut_path)
from autopath3d.trajectories.polynomial import split_half

from conftest import make_world

LIM = TrajectoryLimits(3.0, 2.0, 3.0)


def _validate(tr, sc, c, start, goal, **kw):
    return validate_trajectory(tr.knots, tr.coeffs, sc.distance, c, LIM.max_speed, LIM.max_acceleration,
                               start=start, goal=goal, start_velocity=np.zeros(3), hold_point=tr.hold_point, **kw)


def test_quintic_hermite_boundary_conditions_and_bernstein():
    p0, v0, a0 = np.array([0, 1, 2.0]), np.array([1, 0, 0.5]), np.array([0, 0.3, 0])
    p1, v1, a1 = np.array([3, -1, 2.5]), np.array([0, 2, 0]), np.array([-1, 0, 0.2])
    T = 1.7
    tr = PiecewisePolynomial.from_pieces([quintic_hermite(p0, v0, a0, p1, v1, a1, T)], [T])
    for t, (p, v, a) in [(0.0, (p0, v0, a0)), (T, (p1, v1, a1))]:
        assert np.allclose(tr.evaluate(t, 0)[0], p) and np.allclose(tr.evaluate(t, 1)[0], v)
        assert np.allclose(tr.evaluate(t, 2)[0], a)
    P = power_to_bernstein(tr.coeffs[0], T)
    _, _, mid = split_half(P[None])
    assert np.allclose(mid[0], tr.evaluate(T / 2)[0])
    assert np.allclose(P[0], p0) and np.allclose(P[-1], p1)


def l_world():
    """Room with an inner-corner block: free L-shaped corridor around it."""
    return make_world([Box((1.0, 1.4, 0), (4.0, 4.0, 4.0), "block")], hi=(4, 4, 4))


def test_shortcut_simplifies_safely_and_keeps_needed_corner():
    sc, grid, ck = l_world()
    # dense L-shaped path: along y = 0.7 then up along x = 0.5 is blocked... use x=0.55 column
    leg1 = np.linspace([3.5, 0.6, 2.0], [0.55, 0.6, 2.0], 30)
    leg2 = np.linspace([0.55, 0.6, 2.0], [0.55, 3.5, 2.0], 30)[1:]
    path = np.vstack([leg1, leg2])
    assert ck.path_free(path)
    r = shortcut_path(path, ck)
    assert r.success and r.waypoints < r.original_waypoints and r.length <= r.original_length + 1e-12
    assert np.allclose(r.path[0], path[0]) and np.allclose(r.path[-1], path[-1])
    # the straight start-goal chord crosses the block: must have been rejected
    assert not ck.segment_free(path[0], path[-1])
    assert len(r.path) >= 3 and r.rejected_shortcuts >= 1
    assert validate_path(sc, grid, r.path, path[0], path[-1], ck.c).valid


def test_shortcut_rejects_uncertified_input():
    sc, grid, ck = make_world([Box((1.8, 0, 0), (2.2, 4, 4))])
    path = np.array([[1.0, 0.5, 2.0], [1.42, 2.0, 2.0], [1.0, 3.5, 2.0]])  # 0.38 m < 0.45? c=0.35 here
    assert ck.path_free(path)
    _, _, strict = make_world([Box((1.8, 0, 0), (2.2, 4, 4))], robot=RobotModel(0.25, 0.20))
    r = shortcut_path(path, strict)
    assert not r.success and r.path is None


def test_generated_trajectory_is_continuous_bounded_and_at_rest():
    sc = get_scene("warehouse")
    pol = ClearancePolicy()
    grid, ck = trajectory_checker(sc, 0.2, pol)
    t = sc.task("aisle_to_aisle")
    res = run_pipeline(sc, ck, t.start, t.goal, {"planner": "astar"}, LIM, pol)
    assert res.status == "success"
    tr = res.trajectory
    v = res.record["validation"]
    assert v["max_speed_upper"] <= 3.0 and v["max_accel_upper"] <= 2.0
    assert v["min_clearance_lower"] >= pol.trajectory
    assert v["max_velocity_jump"] < 1e-9 and v["max_accel_jump"] < 1e-9
    # independent numerical derivative check (central differences on dense samples)
    ts = np.linspace(tr.t_start, tr.t_end, 20001)
    p, vel, acc = tr.evaluate(ts, 0), tr.evaluate(ts, 1), tr.evaluate(ts, 2)
    h = ts[1] - ts[0]
    assert np.abs((p[2:] - p[:-2]) / (2 * h) - vel[1:-1]).max() < 1e-4
    assert np.abs((vel[2:] - vel[:-2]) / (2 * h) - acc[1:-1]).max() < 1e-3
    assert np.linalg.norm(vel, axis=1).max() <= v["max_speed_upper"] + 1e-12
    assert np.linalg.norm(acc, axis=1).max() <= v["max_accel_upper"] + 1e-12
    for k in (1, 2):
        assert np.allclose(tr.evaluate(tr.t_start, k), 0) and np.allclose(tr.evaluate(tr.t_end, k), 0)


def test_spline_overshoot_is_detected_and_rejected():
    sc, grid, ck = l_world()
    a, corner, b = np.array([3.5, 0.6, 2.0]), np.array([0.6, 0.6, 2.0]), np.array([0.6, 3.5, 2.0])
    assert ck.path_free(np.vstack([a, corner, b]))
    # aggressive corner velocity along the bisector -> the curve cuts toward the block
    vc = 2.5 * np.array([-1, 1, 0]) / np.sqrt(2)
    T = 2.0
    bad = PiecewisePolynomial.from_pieces(
        [quintic_hermite(a, 0 * vc, 0 * vc, corner + [0.3, 0.3, 0], vc, 0 * vc, T),
         quintic_hermite(corner + [0.3, 0.3, 0], vc, 0 * vc, b, 0 * vc, 0 * vc, T)], [T, T])
    rep = _validate(bad, sc, ck.c, a, b)
    assert not rep.valid and rep.min_clearance_upper < ck.c
    assert not certify_curve(bad.bernstein(0), ck.clearance, ck.c) or not certify_curve(bad.bernstein(1), ck.clearance, ck.c)
    # the generator never outputs such a curve: rejects candidates or falls back
    g = generate_trajectory(np.vstack([a, corner, b]), ck, LIM)
    assert g.success
    assert _validate(g.trajectory, sc, ck.c, a, b).valid


def test_generator_handles_duplicates_short_segments_and_zero_length():
    sc, grid, ck = make_world([])
    wp = np.array([[1, 1, 1.0], [1, 1, 1.0], [1.001, 1, 1.0], [2, 2, 2.0], [2, 2, 2.0], [3, 1, 2.5]])
    g = generate_trajectory(wp, ck, LIM)
    assert g.success and g.trajectory.n_pieces == 3
    assert _validate(g.trajectory, sc, ck.c, wp[0], wp[-1]).valid
    z = generate_trajectory(np.array([[2, 2, 2.0]]), ck, LIM)
    assert z.success and z.duration == 0.0
    assert _validate(z.trajectory, sc, ck.c, [2, 2, 2.0], [2, 2, 2.0]).valid


def test_validator_rejects_infeasible_trajectories():
    sc, grid, ck = make_world([])
    a, b = np.array([1.0, 1, 1]), np.array([3.0, 3, 3])
    fast = PiecewisePolynomial.from_pieces([quintic_hermite(a, 0 * a, 0 * a, b, 0 * a, 0 * a, 0.5)], [0.5])
    rep = _validate(fast, sc, ck.c, a, b)
    assert not rep.valid and any("speed" in f for f in rep.failures) and any("acceleration" in f for f in rep.failures)
    mid = (a + b) / 2
    jump = PiecewisePolynomial.from_pieces(
        [quintic_hermite(a, 0 * a, 0 * a, mid, [0.5, 0.5, 0.5], [0.3, 0, 0], 3.0),
         quintic_hermite(mid, [0.5, 0.5, 0.5], [-0.3, 0, 0], b, 0 * a, 0 * a, 3.0)], [3.0, 3.0])
    rep = _validate(jump, sc, ck.c, a, b)
    assert not rep.valid and any("acceleration discontinuity" in f for f in rep.failures)
    nan = PiecewisePolynomial(fast.knots.copy(), fast.coeffs.copy())
    nan.coeffs[0, 3, 0] = np.nan
    assert not _validate(nan, sc, ck.c, a, b).valid
    back = PiecewisePolynomial(np.array([0.0, 2.0, 1.0]), np.repeat(fast.coeffs, 2, axis=0))
    assert any("timestamps" in f for f in _validate(back, sc, ck.c, a, b).failures)
    slow = PiecewisePolynomial.from_pieces([quintic_hermite(a, 0 * a, 0 * a, b, 0 * a, 0 * a, 6.0)], [6.0])
    assert _validate(slow, sc, ck.c, a, b).valid
    assert any("goal" in f for f in _validate(slow, sc, ck.c, a, b + 0.1).failures)


def test_validator_corridor_check():
    sc, grid, ck = make_world([])
    a, b = np.array([1.0, 1, 2]), np.array([3.0, 1, 2])
    bow = PiecewisePolynomial.from_pieces(
        [quintic_hermite(a, [0, 2.0, 0], 0 * a, b, [0, -2.0, 0], 0 * a, 3.0)], [3.0])
    rep = _validate(bow, sc, ck.c, a, b, reference_path=np.vstack([a, b]), corridor_halfwidth=0.2)
    assert rep.max_deviation_upper > 0.2 and any("corridor" in f for f in rep.failures)


def test_narrow_passages_and_tracking_allowance():
    pol = ClearancePolicy()
    sc = get_scene("narrow_1200")
    grid, ck = trajectory_checker(sc, 0.2, pol)
    t = sc.tasks[0]
    r = run_pipeline(sc, ck, t.start, t.goal, {"planner": "astar"}, LIM, pol)
    assert r.status == "success" and r.record["validation"]["min_clearance_lower"] >= 0.45
    sc9 = get_scene("narrow_900")
    grid9, ck9 = trajectory_checker(sc9, 0.2, pol)
    r9 = run_pipeline(sc9, ck9, t.start, t.goal, {"planner": "astar"}, LIM, pol)
    assert r9.status == "plan_failed"
    # a nominal-clearance path (0.35 m) that is closer than c_traj is rejected, not smoothed
    _, _, ck_nom = make_world([Box((1.8, 0, 0), (2.2, 4, 4))])
    sc_w, _, ck_tr = make_world([Box((1.8, 0, 0), (2.2, 4, 4))], robot=pol.trajectory_robot())
    tight = np.array([[1.0, 0.6, 2.0], [1.4, 2.0, 2.0], [1.0, 3.4, 2.0]])     # 0.40 m from the wall
    assert ck_nom.path_free(tight) and not ck_tr.path_free(tight)
    rr = run_pipeline(sc_w, ck_tr, tight[0], tight[-1], {}, LIM, pol, path=tight)
    assert rr.status == "shortcut_rejected"


def test_pipeline_is_deterministic():
    pol = ClearancePolicy()
    sc = get_scene("industrial")
    grid, ck = trajectory_checker(sc, 0.2, pol)
    t = sc.task("cross_hall")
    spec = {"planner": "rrt_star", "max_iterations": 3000, "step_size": 2.5, "seed": 3}
    r1 = run_pipeline(sc, ck, t.start, t.goal, spec, LIM, pol)
    r2 = run_pipeline(sc, ck, t.start, t.goal, spec, LIM, pol)
    assert r1.status == r2.status
    if r1.trajectory is not None:
        assert np.array_equal(r1.trajectory.coeffs, r2.trajectory.coeffs)
        assert np.array_equal(r1.trajectory.knots, r2.trajectory.knots)

import numpy as np
import pytest

from autopath3d.environments import Box
from autopath3d.pipeline import ClearancePolicy
from autopath3d.replanning import (MotionState, ObstacleUpdate, Outcome, ReplanningSupervisor, SupervisorConfig,
                                   braking_trajectory, state_at, stopping_distance)
from autopath3d.replanning.scenarios import SCENARIOS, run_scenario
from autopath3d.trajectories import PiecewisePolynomial, TrajectoryLimits

from conftest import make_scene

POL, LIM, CFG = ClearancePolicy(), TrajectoryLimits(3.0, 2.0, 3.0), SupervisorConfig()


def test_stopping_distance_formula_and_braking_trajectory():
    assert stopping_distance(2.0, 4.0, 0.3) == pytest.approx(0.6 + 0.5)
    assert stopping_distance(0.0, 4.0, 0.3) == 0.0
    p, v = np.array([1.0, 2, 3]), np.array([0.0, 3.0, 4.0])          # 5 m/s
    br = braking_trajectory(p, v, 2.5, t0=7.0)
    assert br.t_start == 7.0 and br.duration == pytest.approx(2.0)
    assert np.allclose(br.evaluate(br.t_end)[0], p + v / 5 * 5.0)    # 25 / (2*2.5) = 5 m
    assert np.allclose(br.evaluate(br.t_end, 1)[0], 0) and np.allclose(br.evaluate(7.0, 1)[0], v)
    acc = br.evaluate(np.linspace(7, 9, 50), 2)
    assert np.allclose(np.linalg.norm(acc, axis=1), 2.5)
    assert br.evaluate(8.0, 1)[0] @ v > 0                              # never reverses
    assert braking_trajectory(p, 0 * v, 2.5, 0.0).n_pieces == 0


@pytest.fixture(scope="module")
def corridor():
    """10x6x4 room with a pillar row; a straight mission along y = 3."""
    sc = make_scene([Box((4.8, 0.0, 0.0), (5.2, 1.8, 4.0))], hi=(10, 6, 4))
    sup = ReplanningSupervisor(sc, POL, LIM, CFG)
    nominal = sup.plan_nominal([1.0, 3.0, 2.0], [9.0, 3.0, 2.0], {"planner": "astar"})
    assert nominal.status == "success"
    return sc, sup, nominal.trajectory


def _upd(name, t, *boxes):
    return ObstacleUpdate(name, t, list(boxes))


def test_obstacle_off_route_causes_no_conflict(corridor):
    sc, sup, tr = corridor
    rep = sup.handle_update(state_at(tr, 1.2), tr, [9, 3, 2.0], _upd("far", 1.0, Box((1, 5.2, 0), (2, 6, 1))))
    assert rep["outcome"] == Outcome.NO_CONFLICT and rep["goal_reached"]


def test_replanning_succeeds_and_new_trajectory_is_validated(corridor):
    sc, sup, tr = corridor
    wall = Box((6.0, 1.8, 0.0), (6.4, 4.5, 4.0), "partial_wall")       # leaves a gap at y > 4.5
    rep = sup.handle_update(state_at(tr, 1.2), tr, [9, 3, 2.0], _upd("wall", 1.0, wall))
    assert rep["conflict"] and rep["safe_stop"]
    assert rep["outcome"] == Outcome.REPLANNED and rep["goal_reached"]
    v = rep["attempts"][-1]["validation"]
    assert v["valid"] and v["min_clearance_lower"] >= POL.trajectory
    assert rep["t_resume"] >= rep["t_stop"]
    # the stop is physically consistent: no teleport, velocity continuous at t_brake
    assert rep["speed_at_brake"] == pytest.approx(np.linalg.norm(tr.evaluate(rep["t_brake"], 1)[0]))
    assert rep["stop_min_clearance"][0] >= POL.nominal


def test_no_alternative_route_holds_safely(corridor):
    sc, sup, tr = corridor
    seal = Box((6.0, 0.0, 0.0), (6.4, 6.0, 4.0), "seal")
    rep = sup.handle_update(state_at(tr, 1.2), tr, [9, 3, 2.0], _upd("seal", 1.0, seal))
    assert rep["outcome"] == Outcome.NO_ROUTE_HOLDING and not rep["goal_reached"]
    assert rep["hold_clearance"] >= POL.nominal


def test_insufficient_stopping_distance_is_reported(corridor):
    sc, sup, tr = corridor
    t = float(tr.t_start + 0.5 * tr.duration)                          # near peak speed
    st = state_at(tr, t)
    v = np.linalg.norm(st.velocity)
    face = st.position[0] + 0.5                                         # far inside the stopping distance
    assert stopping_distance(v, LIM.max_deceleration, 0.3) > 0.5 + POL.robot_radius
    blk = Box((face, 2.0, 1.0), (face + 0.4, 4.0, 3.0), "sudden")
    rep = sup.handle_update(state_at(tr, t + 0.2), tr, [9, 3, 2.0], _upd("sudden", t, blk))
    assert rep["outcome"] == Outcome.UNAVOIDABLE_COLLISION and not rep["goal_reached"]
    assert rep["stop_min_clearance"][1] < POL.robot_radius


def test_hold_position_made_unsafe():
    sc = make_scene([], hi=(6, 6, 4))
    sup = ReplanningSupervisor(sc, POL, LIM, CFG)
    hover = PiecewisePolynomial.stationary([3.0, 3.0, 2.0])
    st = MotionState(1.0, np.array([3.0, 3, 2]), np.zeros(3), np.zeros(3))
    rep = sup.handle_update(st, hover, [5, 5, 2.0], _upd("onto", 0.9, Box((3.1, 2.5, 1.5), (3.6, 3.5, 2.5))))
    assert rep["outcome"] == Outcome.HOLD_UNSAFE and not rep["goal_reached"]


def test_temporary_planning_failure_then_retry(corridor):
    sc, _, tr = corridor
    cfg = SupervisorConfig(replan_attempts=[{"planner": "rrt_star", "max_iterations": 5, "seed": 0},
                                            {"planner": "astar"}])
    sup = ReplanningSupervisor(sc, POL, LIM, cfg)
    wall = Box((6.0, 1.8, 0.0), (6.4, 4.5, 4.0))
    rep = sup.handle_update(state_at(tr, 1.2), tr, [9, 3, 2.0], _upd("wall", 1.0, wall))
    assert [a["status"] for a in rep["attempts"]] == ["plan_failed", "success"]
    assert rep["outcome"] == Outcome.REPLANNED_AFTER_RETRY
    assert rep["hold_clearance"] >= POL.nominal


@pytest.mark.parametrize("sc", [s for s in SCENARIOS if s.scene != "urban_canyon"], ids=lambda s: s.name)
def test_deterministic_scenarios_reproduce_expected_outcomes(sc):
    r1 = run_scenario(sc, POL, LIM, CFG)
    assert r1["matches_expected"], (r1["outcome"], r1["expected"])
    r2 = run_scenario(sc, POL, LIM, CFG)
    assert r2["outcome"] == r1["outcome"]
    for k in ("stop_point", "stop_min_clearance", "travel_after_appear_m"):
        assert r1.get(k) == r2.get(k)

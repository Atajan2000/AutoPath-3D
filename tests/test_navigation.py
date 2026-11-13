"""Phase 3 closed-loop tests: tracking, limits, executed-path safety, event timing, replanning outcomes."""

import numpy as np
import pytest
import yaml

from autopath3d.environments import Box
from autopath3d.pipeline import ClearancePolicy
from autopath3d.replanning import ObstacleUpdate, ReplanningSupervisor, SupervisorConfig, state_at
from autopath3d.simulation import GUSTS, IDEAL, Disturbance, Navigator, SimConfig, WorldDistance, refine_min
from autopath3d.simulation.dynamics import step_control_points
from autopath3d.simulation.navigator import M
from autopath3d.simulation.scenarios import ScenarioRunner, scenario_by_name
from autopath3d.trajectories import TrajectoryLimits

from conftest import make_scene

POL, LIM = ClearancePolicy(), TrajectoryLimits(3.0, 2.0, 3.0)
GOAL = [9.0, 3.0, 2.0]
WALL = Box((6.0, 1.8, 0.0), (6.4, 4.5, 4.0), "partial_wall")     # leaves a gap at y > 4.5
SEAL = Box((6.0, 0.0, 0.0), (6.4, 6.0, 4.0), "seal")


@pytest.fixture(scope="module")
def corridor():
    sc = make_scene([Box((4.8, 0.0, 0.0), (5.2, 1.8, 4.0))], hi=(10, 6, 4))
    sup = ReplanningSupervisor(sc, POL, LIM, SupervisorConfig())
    nom = sup.plan_nominal([1.0, 3.0, 2.0], GOAL, {"planner": "astar"})
    assert nom.status == "success"
    return sc, sup, nom


def run(corridor, dist=GUSTS, update=None, cfg=None, latency=None):
    sc, sup, nom = corridor
    if cfg is not None:
        sup = ReplanningSupervisor(sc, POL, LIM, cfg)
    return Navigator(sc, POL, LIM, disturbance=dist, supervisor=sup).run(nom.trajectory, GOAL, update, latency)


def assert_no_false_success(res):
    m = res.metrics
    if m["mission_success"]:
        assert m["safety_pass"] and m["goal_reached"]
        assert not any(e.level == "ERROR" for e in res.events)
    if any(e.level == "ERROR" for e in res.events):
        assert not m["safety_pass"] and not m["mission_success"]


# ---------------------------------------------------------------- tracking and limits
def test_tracking_rest_conditions_and_limits(corridor):
    ideal, gust = run(corridor, IDEAL), run(corridor, GUSTS)
    for res in (ideal, gust):
        A, m = res.arrays, res.metrics
        assert np.all(A["v"][0] == 0) and np.allclose(A["p"][0], [1.0, 3.0, 2.0])     # starts at rest at start
        assert m["goal_reached"] and m["safety_pass"] and m["mission_success"]
        assert m["goal_error_final"] <= 0.10 and m["speed_final"] <= 0.10
        assert m["max_speed"] <= 3.0 and m["max_accel_cmd_tracking_hold"] <= 2.0 + 1e-9
        assert np.all(np.linalg.norm(A["a_cmd"], axis=1) <= 2.0 + 1e-9)
        assert_no_false_success(res)
    assert ideal.metrics["tracking"]["position_error"]["max"] < 1e-4        # ZOH discretisation error only
    e = gust.metrics["tracking"]["position_error"]
    assert 0.005 < e["max"] < POL.tracking_allowance and e["rms"] < e["max"]  # disturbance is felt, within allowance


def test_drone_is_simulated_not_placed_on_reference(corridor):
    res = run(corridor, GUSTS)
    A = res.arrays
    # every state follows from the previous one through the ZOH model, never copied from the reference
    acc = A["a_cmd"] + A["a_dist"]
    p_pred = A["p"][:-1] + A["v"][:-1] * 0.01 + 0.5 * acc * 1e-4
    assert np.allclose(A["p"][1:], p_pred, atol=1e-12)
    assert np.linalg.norm(A["p"][:-1] - A["p_ref"], axis=1).max() > 1e-3


def test_time_grid_and_step_size_convergence(corridor):
    sc, sup, nom = corridor
    a = run(corridor, GUSTS)
    assert np.array_equal(a.arrays["t"], np.arange(len(a.arrays["t"])) * 0.01)
    b = Navigator(sc, POL, LIM, SimConfig(dt=0.005), disturbance=GUSTS).run(nom.trajectory, GOAL)
    pb = b.arrays["p"][::2]                                   # states at the same times t_k = k * 0.01
    n = min(len(a.arrays["p"]), len(pb))
    diff = np.linalg.norm(a.arrays["p"][:n] - pb[:n], axis=1)
    assert diff.max() < 2e-3                                  # halving dt changes the path by < 2 mm


def test_simulation_is_deterministic(corridor):
    r1 = run(corridor, GUSTS, ObstacleUpdate("w", 1.0, [WALL]), latency=0.5)
    r2 = run(corridor, GUSTS, ObstacleUpdate("w", 1.0, [WALL]), latency=0.5)
    for k in r1.arrays:
        assert np.array_equal(r1.arrays[k], r2.arrays[k], equal_nan=True), k
    assert [(e.k, e.name) for e in r1.events] == [(e.k, e.name) for e in r2.events]


def test_tracking_allowance_violation_fails_the_safety_test(corridor):
    strong = Disturbance("strong", bias=(0.0, 1.5, 0.0))       # 1.5 m/s^2 steady side wind
    res = run(corridor, strong)
    assert "TRACKING_ALLOWANCE_EXCEEDED" in res.event_names()
    assert res.metrics["tracking"]["position_error"]["max"] > POL.tracking_allowance
    assert not res.metrics["checks"]["tracking_within_allowance"]
    assert not res.metrics["safety_pass"] and not res.metrics["mission_success"]
    assert_no_false_success(res)


def test_executed_clearance_is_certified_and_consistent(corridor):
    sc, _, nom = corridor
    res = run(corridor, GUSTS)
    A, m = res.arrays, res.metrics
    w = WorldDistance.of_scene(sc)
    P = step_control_points(A["p"][:-1], A["v"][:-1], A["p"][1:], 0.01)
    u = np.linspace(0, 1, 21)[:, None, None]
    dense = ((1 - u) ** 2 * P[None, :, 0] + 2 * u * (1 - u) * P[None, :, 1] + u ** 2 * P[None, :, 2]).reshape(-1, 3)
    dmin = w(dense).min()
    c = m["clearance"]
    assert c["lower"] <= dmin + 1e-12 and c["upper"] - c["lower"] < 2e-3
    ref_clr = nom.record["validation"]["min_clearance_lower"]
    assert c["lower"] >= ref_clr - m["tracking"]["position_error"]["max"] - 0.02   # 1-Lipschitz consistency


# ---------------------------------------------------------------- obstacle updates
def test_event_latencies_and_schedule(corridor):
    res = run(corridor, GUSTS, ObstacleUpdate("w", 1.0, [WALL]), latency=2.0)
    e = {x.name: x for x in res.events}
    assert e["OBSTACLE_DETECTED"].t - e["OBSTACLE_APPEARED"].t == pytest.approx(0.2, abs=1e-9)
    assert e["BRAKING"].t - e["OBSTACLE_DETECTED"].t == pytest.approx(0.1, abs=1e-9)
    A = res.arrays
    kb = e["BRAKING"].k
    assert A["mode"][kb - 1] == M["FOLLOWING"] and A["mode"][kb] == M["BRAKING"]
    assert np.all(A["world"][: e["OBSTACLE_APPEARED"].k] == 0) and np.all(A["world"][e["OBSTACLE_APPEARED"].k:] == 1)
    # planning latency: simulated hold = 2.0 s, the replacement appears only afterwards
    assert e["RESUMING"].t - e["REPLANNING"].t == pytest.approx(2.0, abs=1e-9)
    times = [x.t for x in res.events]
    assert times == sorted(times)


def test_obstacle_off_route_is_handled_without_stopping(corridor):
    res = run(corridor, GUSTS, ObstacleUpdate("far", 1.0, [Box((1, 5.2, 0), (2, 6, 1))]))
    names = res.event_names()
    assert "NO_CONFLICT" in names and "BRAKING" not in names
    assert res.metrics["mission_success"]


def test_safe_hold_and_successful_replanning(corridor):
    res = run(corridor, GUSTS, ObstacleUpdate("w", 1.0, [WALL]), latency=3.0)
    m, A, e = res.metrics, res.arrays, {x.name: x for x in res.events}
    assert res.replan["outcome"].value == "replanned"
    assert m["goal_reached"] and m["safety_pass"] and m["mission_success"]
    hv = e["HOLD_VERIFIED"].data
    assert hv["hold_clearance"] >= POL.nominal and hv["flown_clearance"][0] >= POL.nominal
    # holding during the 3 s planning interval: the drone stays put (no teleport, no motion)
    hold = (A["mode"] == M["REPLANNING"])
    assert hold.sum() == 300
    assert np.linalg.norm(A["p"][:-1][hold] - A["hold"][hold], axis=1).max() < 0.05
    assert np.linalg.norm(A["v"][:-1][hold], axis=1).max() < 0.05
    # the replacement trajectory starts at the actual hold point when it is released
    tr1 = res.trajectories[1]
    assert tr1["knots"][0] == pytest.approx(e["RESUMING"].t)
    assert np.allclose(np.asarray(tr1["coeffs"])[0][0], hv["hold_point"])
    assert res.replan["sim_planning_s"] == pytest.approx(3.0)
    assert_no_false_success(res)


def test_measured_planning_time_advances_simulated_time(corridor):
    res = run(corridor, GUSTS, ObstacleUpdate("w", 1.0, [WALL]))
    rp = res.replan
    assert rp["sim_planning_s"] >= rp["measured_compute_s"] - 1e-12
    assert rp["sim_planning_s"] < rp["measured_compute_s"] + 0.01 + 1e-9


def test_planner_failure_then_retry_holds_until_validated(corridor):
    cfg = SupervisorConfig(replan_attempts=[{"planner": "rrt_star", "max_iterations": 5, "seed": 0}, {"planner": "astar"}])
    res = run(corridor, GUSTS, ObstacleUpdate("w", 1.0, [WALL]), cfg=cfg, latency=2.0)
    names = res.event_names()
    assert [a["status"] for a in res.replan["attempts"]] == ["plan_failed", "success"]
    assert names.index("REPLAN_ATTEMPT_FAILED") < names.index("REPLAN_SUCCEEDED") < names.index("RESUMING")
    e = {x.name: x for x in res.events}
    A = res.arrays
    between = slice(e["REPLAN_ATTEMPT_FAILED"].k, e["RESUMING"].k)
    assert np.all(A["mode"][between] == M["REPLANNING"])
    assert res.replan["outcome"].value == "replanned_after_retry" and res.metrics["mission_success"]


def test_no_route_holds_safely_and_does_not_claim_success(corridor):
    res = run(corridor, GUSTS, ObstacleUpdate("s", 1.0, [SEAL]))
    m = res.metrics
    assert "NO_ROUTE" in res.event_names() and m["final_mode"] == "HOLDING_ABORTED"
    assert not m["goal_reached"] and not m["mission_success"] and m["safety_pass"]
    assert m["clearance"]["lower"] >= POL.nominal
    assert_no_false_success(res)


def test_unavoidable_collision_terminates_and_fails(corridor):
    sc, sup, nom = corridor
    tr = nom.trajectory
    t = round(0.5 * tr.duration, 2)
    st = state_at(tr, t)
    face = st.position[0] + 0.5
    res = run(corridor, GUSTS, ObstacleUpdate("sudden", t, [Box((face, 2.0, 1.0), (face + 0.4, 4.0, 3.0))]))
    m = res.metrics
    col = res.first("COLLISION")
    assert col is not None and m["final_mode"] == "COLLIDED"
    assert res.arrays["t"][-1] == pytest.approx(col.t + 0.01)                 # terminated at the contact step
    assert m["clearance"]["lower"] < POL.robot_radius and not m["checks"]["no_collision"]
    assert not m["safety_pass"] and not m["goal_reached"]
    assert_no_false_success(res)


# ---------------------------------------------------------------- Phase 2 scenarios D1-D3 (fast scene)
@pytest.fixture(scope="module")
def scenario_cfg():
    cfg = yaml.safe_load(open("configs/phase3.yaml"))
    return cfg, SupervisorConfig(0.2, 0.1, 0.2, 3.0)


@pytest.mark.parametrize("name, expected", [("D1_too_close_collision", "unavoidable_collision"),
                                            ("D2_too_close_margin", "stop_margin_violation"),
                                            ("D3_far_enough", "replanned"),
                                            ("C_narrow_window_sealed", "no_route_holding")])
def test_phase2_classifications_preserved_without_disturbance(name, expected, scenario_cfg):
    cfg, sc_cfg = scenario_cfg
    res = ScenarioRunner(scenario_by_name(name), POL, LIM, sc_cfg, cfg).run(IDEAL)
    assert res.meta["outcome"] == expected and res.meta["matches_expected"]
    m = res.metrics
    if expected in ("unavoidable_collision", "stop_margin_violation"):
        assert not m["safety_pass"] and not m["goal_reached"]
    if expected == "stop_margin_violation":
        assert "SAFETY_VIOLATION" in res.event_names() and "MISSION_ABORTED" in res.event_names()
        assert POL.robot_radius <= m["clearance"]["lower"] < POL.nominal
    if expected == "replanned":
        assert m["mission_success"]
    assert_no_false_success(res)


def test_d2_with_gusts_is_reported_as_contact_not_hidden(scenario_cfg):
    """Documented Phase 3 finding: the supervisor's braking model ignores the wind; D2 degrades to contact."""
    cfg, sc_cfg = scenario_cfg
    res = ScenarioRunner(scenario_by_name("D2_too_close_margin"), POL, LIM, sc_cfg, cfg).run(GUSTS)
    assert res.meta["supervisor_prediction"] == "stop_margin_violation"
    assert res.meta["outcome"] == "unavoidable_collision" and not res.meta["matches_expected"]
    assert not res.metrics["safety_pass"]
    assert_no_false_success(res)


def test_executed_clearance_matches_refinement_after_update(corridor):
    res = run(corridor, IDEAL, ObstacleUpdate("w", 1.0, [WALL]))
    sc = corridor[0]
    A = res.arrays
    up = WorldDistance(sc.lo, sc.hi, list(sc.obstacles) + [WALL])
    sel = A["world"] == 1
    P = step_control_points(A["p"][:-1], A["v"][:-1], A["p"][1:], 0.01)[sel]
    lo, hi = refine_min(P, up)
    assert res.metrics["clearance"]["after_update"] == pytest.approx([lo, hi])

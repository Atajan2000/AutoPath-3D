"""Offline integration tests on data recorded from the running Webots warehouse.

data/webots_warehouse_geometry.json   collision geometry extracted by the scene manager from the
                                      live simulation (AUTOPATH_GEOMETRY_DUMP), 2026-11-10
data/characterisation_*.json          Crazyflie characterisation run in Webots, 2026-11-10

They run the same map-building code as map_node (geometry.build_planning_records) and the
unchanged AutoPath planners / validator / stopping assessment, without ROS or Webots, so mission
definitions and config values can be checked quickly. They are not a substitute for the
end-to-end Webots runs (test_e2e_webots.py / results/phase4/webots/).
"""

import copy
import glob
import json
import os

import numpy as np
import pytest

from autopath3d.evaluation.validation import validate_path
from autopath3d.maps.occupancy import UNKNOWN, OccupancyGrid
from autopath3d.pipeline import make_planner, run_pipeline, trajectory_checker
from autopath3d.replanning.stopping_assessment import assess_stopping

from autopath3d_ros import config as apcfg
from autopath3d_ros.geometry import WorldGeometry, build_planning_records

HERE = os.path.dirname(os.path.abspath(__file__))
CFG_PATH = os.path.join(os.path.dirname(HERE), "config", "webots_warehouse.yaml")


@pytest.fixture(scope="module")
def cfg():
    return apcfg.load(CFG_PATH)


@pytest.fixture(scope="module")
def dump():
    with open(os.path.join(HERE, "data", "webots_warehouse_geometry.json")) as f:
        return json.load(f)


def _scene(cfg, dump, updates=(), moved=None):
    base = dump["records"]
    if moved:
        base = [moved(r) for r in base]
    recs, audit = build_planning_records(base, dump["visual_records"], cfg, list(updates))
    b = cfg["bounds"]
    return WorldGeometry(np.asarray(b["lo"]), np.asarray(b["hi"]), recs).scene(), recs, audit


def _shift(obj, d):
    d = np.asarray(d, float)

    def f(r):
        if r["object"] != obj:
            return r
        r = copy.deepcopy(r)
        if r["kind"] == "box":
            r["lo"] = (np.asarray(r["lo"]) + d).tolist()
            r["hi"] = (np.asarray(r["hi"]) + d).tolist()
        else:
            r["center"] = (np.asarray(r["center"]) + d[:2]).tolist()
            r["z"] = [r["z"][0] + d[2], r["z"][1] + d[2]]
        return r
    return f


def _plan(cfg, scene, start, goal, pipeline=False):
    """planner node (A* + independent path validation) and, with pipeline=True, the trajectory
    node stage on that path (shortcut -> quintic generation -> continuous-time validation)."""
    pol = apcfg.policy(cfg)
    grid, ck = trajectory_checker(scene, cfg["map"]["resolution"], pol, cfg["map"]["clearance_cap"])
    pr = make_planner(ck, dict(cfg["planners"]["astar"])).plan(np.asarray(start, float), np.asarray(goal, float))
    val = validate_path(scene, grid, pr.path, start, goal, pol.trajectory) if pr.success else None
    if not pipeline:
        return pr, val
    res = run_pipeline(scene, ck, pr.path[0], pr.path[-1], {"planner": "given_path"}, apcfg.limits(cfg), pol,
                       path=pr.path) if pr.success else None
    return pr, val, res


def _start(m):
    return [m["start"][0], m["start"][1], m["takeoff_height"]]


def test_extraction_is_complete(dump):
    assert dump["unsupported_bounding_geometry"] == []
    # the only visual that cannot be audited: the forklift CAD mesh (collision boxes exist)
    assert dump["visual_unbounded"] == ["forklift/forklift: CadShape"]
    objs = {r["object"] for r in dump["records"]}
    for o in ["wall", "wall(1)", "wall(2)", "wall(3)", "mezzanine platform", "straight stairs", "ventilation duct",
              "storage cage", "cage gate", "forklift", "dropped load", *[f"rack {i}" for i in range(8)],
              *[f"column {i}" for i in range(5)]]:
        assert o in objs, o


def test_map_is_known_no_unknown_voxels(cfg, dump):
    scene, recs, audit = _scene(cfg, dump)
    grid = OccupancyGrid.from_scene(scene, cfg["map"]["resolution"])
    assert int((grid.states == UNKNOWN).sum()) == 0
    assert sum(r["source"] == "envelope" for r in recs) == 8
    # visual parts without collision geometry are added conservatively, never silently dropped
    assert all(r["source"] == "visual" for r in recs if r["object"].startswith("oil barrel") and r["kind"] == "box")
    assert len(audit) > 0


def test_mission_endpoints_have_trajectory_clearance(cfg, dump):
    scene, _, _ = _scene(cfg, dump)
    need = apcfg.policy(cfg).trajectory
    for name, m in cfg["missions"].items():
        for p in (_start(m), m["goal"]):
            d = float(scene.distance(np.asarray([p], float))[0])
            assert d >= need, f"{name}: {p} clearance {d:.3f} < {need}"


@pytest.mark.parametrize("name", ["A_navigation", "B_replan", "C1_no_route", "C2_unsafe_stop"])
def test_missions_have_validated_routes(cfg, dump, name):
    scene, _, _ = _scene(cfg, dump)
    m = cfg["missions"][name]
    pr, val, res = _plan(cfg, scene, _start(m), m["goal"], pipeline=True)
    assert pr.success, pr.reason
    assert val.valid, val.to_dict()
    assert res.status == "success", (res.status, res.reason)


def test_core_tie_limitation_at_c_035_is_documented(cfg, dump):
    """Known limitation of the unchanged Phase 1-3 A* + shortcut pair (reported, not patched):
    at 0.1 m resolution, c = 0.35 m = 7 x res/2 is a voxel-lattice distance, so A* returns lattice
    nodes whose clearance lower bound is exactly c, and the Lipschitz segment certification rejects
    the path (Demo A run 1, measured start). See test_tie_depends_on_c_and_resolution_only."""
    c2 = copy.deepcopy(cfg)
    c2["robot"]["tracking_allowance"] = 0.15
    scene, _, _ = _scene(c2, dump)
    start = [-8.500000000000012, -10.999999999999986, 0.9849998438093935]   # measured hover, run 1
    pr, _, res = _plan(c2, scene, start, cfg["missions"]["A_navigation"]["goal"], pipeline=True)
    assert pr.success and res.status == "shortcut_rejected"
    pr, _, res = _plan(cfg, scene, start, cfg["missions"]["A_navigation"]["goal"], pipeline=True)
    assert res.status == "success", (res.status, res.reason)


def _lattice_tie_values(res, c_max):
    """Clearance lower bounds that are constant along an axis-aligned lattice edge: distance from a
    voxel-centre line to a blocked cube face (k + 1/2) res, or to a cube edge parallel to the line,
    res/2 * sqrt(a^2 + b^2) with a, b odd. Only these can produce the flat tie."""
    odd = range(1, int(2 * c_max / res) + 2, 2)
    vals = {0.5 * res * a for a in odd} | {0.5 * res * float(np.hypot(a, b)) for a in odd for b in odd}
    return sorted(v for v in vals if v <= c_max)


def test_tie_depends_on_c_and_resolution_only():
    """Audit 2026-11-11: the tie is a property of (c, resolution), not of world faces lying on a
    0.05 m grid. A corridor whose faces are all OFF the 0.05 m grid (exact clearance of the path
    >= 0.43 m) still makes A* return nodes at a voxel lower bound of exactly 0.35 m, which the
    shortcut certificate rejects; any c that is not a lattice tie value is unaffected."""
    from autopath3d.collision.checker import CollisionChecker, RobotModel
    from autopath3d.environments.primitives import Box
    from autopath3d.environments.scenes import Scene
    from autopath3d.planners.astar import AStarPlanner
    from autopath3d.trajectories.shortcut import shortcut_path
    obs = [Box((0, -2.0, 0), (6.0, -1.03, 2.0), "s"), Box((0, 1.03, 0), (6.0, 2.0, 2.0), "n"),
           Box((2.93, -1.03, 0), (3.07, 0.02, 2.0), "fin")]
    scene = Scene("corridor", "", np.array([0, -2.0, 0]), np.array([6.0, 2.0, 2.0]), obs, [])
    grid = OccupancyGrid.from_scene(scene, 0.1)
    out = {}
    for c in (0.35, 0.37):
        ck = CollisionChecker(grid, RobotModel(c, 0.0), True, 2.0)
        pr = AStarPlanner(ck, 26).plan(np.array([0.55, 0.0, 1.05]), np.array([5.45, 0.0, 1.05]))
        assert pr.success
        out[c] = (float(ck.clearance(pr.path[1:-1]).min()), float(scene.distance(pr.path).min()),
                  shortcut_path(pr.path, ck).success)
    assert abs(out[0.35][0] - 0.35) < 1e-9 and out[0.35][1] > 0.42 and not out[0.35][2]
    assert out[0.37][2]


def test_trajectory_clearance_is_not_a_lattice_tie_value(cfg):
    """Guard for the configuration: the shipped trajectory clearance must not be one of the lattice
    tie values at the map resolution (0.35 is; 0.37 is not, nearest 0.354 and 0.381)."""
    c, res = apcfg.policy(cfg).trajectory, cfg["map"]["resolution"]
    ties = _lattice_tie_values(res, c + 0.1)
    assert any(abs(t - 0.35) < 1e-12 for t in ties)
    assert min(abs(t - c) for t in ties) > 0.01, (c, ties)


def test_c2_unsafe_stop_verdict_reproduced_offline(cfg, dump):
    """Independent (no Webots, no ROS) reproduction of the C2 verdicts from the state recorded at
    detection in results/phase4/webots/C2_unsafe_stop/run_20261110_234236.json: the conservative
    assessment gives collision_possible with the recorded bounds, and the unchanged Phase 2
    prediction on the offline-planned C2 trajectory gives unavoidable_collision."""
    from autopath3d.replanning.supervisor import MotionState, ObstacleUpdate, ReplanningSupervisor
    from autopath3d_ros.geometry import to_primitive
    xy = (-0.04410142635814229, -7.000004182885567)        # drop position sent to the scene manager
    p = np.array([-0.821000338115717, -7.0000046799939595, 1.2107744751641147])
    v = np.array([0.7730950299835863, 4.946775214903482e-07, -0.0011452939720555631])
    moved = _shift("dropped load", (xy[0], xy[1] + 7.0, 5.0))      # parked at (0, -7, -5) in the dump
    scene1, _, _ = _scene(cfg, dump, moved=moved)
    pol = apcfg.policy(cfg)
    sa = assess_stopping(p, v, scene1.distance, pol.robot_radius, pol.nominal, apcfg.stopping_assumptions(cfg))
    assert sa.classification.value == "collision_possible"
    assert abs(sa.clearance_lower - (-0.37750024161238016)) < 1e-6
    assert abs(sa.clearance_upper - 0.6078987788759342) < 1e-6
    m = cfg["missions"]["C2_unsafe_stop"]
    scene0, _, _ = _scene(cfg, dump)
    _, _, res = _plan(cfg, scene0, _start(m), m["goal"], pipeline=True)
    traj = res.trajectory
    ts = np.linspace(traj.knots[0], traj.knots[-1], 4000)
    xs = np.array([traj.evaluate(t, 0)[0][0] for t in ts])
    t = float(ts[np.argmin(np.abs(xs - p[0]))])                     # straight aisle flight along +x
    load = [to_primitive(moved(r)) for r in dump["records"] if r["object"] == "dropped load"]
    sup = ReplanningSupervisor(scene0, pol, apcfg.limits(cfg), apcfg.supervisor_config(cfg))
    pred = sup.assess_update(MotionState(t, p, v, np.zeros(3)), traj,
                             ObstacleUpdate("drop_load", t - cfg["supervisor"]["detection_latency_s"], load))
    assert pred["conflict"] and pred["outcome"].value == "unavoidable_collision"
    assert 0.40 < pred["distance_to_contact_m"] < 0.47           # Webots: contact 0.44 m after appearance


def test_b_forklift_blocks_cross_aisle_but_detour_exists(cfg, dump):
    m = cfg["missions"]["B_replan"]
    scene0, _, _ = _scene(cfg, dump)
    fp = cfg["scene_events"]["forklift"]
    scene1, _, _ = _scene(cfg, dump, moved=_shift("forklift", (fp["to"][0] - fp["from"][0],
                                                               fp["to"][1] - fp["from"][1], 0.0)))
    p0, _ = _plan(cfg, scene0, _start(m), m["goal"])
    # the original route passes the forklift's final position
    fork = [pr for pr in scene1.obstacles if "forklift" in pr.name]
    d_orig = min(float(np.min(f.distance(p0.path))) for f in fork)
    assert d_orig < apcfg.policy(cfg).trajectory
    p1, v1, r1 = _plan(cfg, scene1, _start(m), m["goal"], pipeline=True)
    assert p1.success and v1.valid and r1.status == "success", (r1.status, r1.reason)
    assert p1.length > p0.length


def test_brake_trigger_equals_phase2_conflict_check(cfg, dump):
    """The supervisor node brakes on geometry.conflict_check before running the slow Phase 2 stop
    prediction; its verdict must equal assess_update()['conflict'] (same criterion)."""
    from autopath3d.replanning.supervisor import MotionState, ObstacleUpdate, ReplanningSupervisor
    from autopath3d_ros.geometry import conflict_check, to_primitive
    m = cfg["missions"]["B_replan"]
    scene, _, _ = _scene(cfg, dump)
    pr, _, res = _plan(cfg, scene, _start(m), m["goal"], pipeline=True)
    traj = res.trajectory
    sup = ReplanningSupervisor(scene, apcfg.policy(cfg), apcfg.limits(cfg), apcfg.supervisor_config(cfg))
    fp = cfg["scene_events"]["forklift"]
    d = (fp["to"][0] - fp["from"][0], fp["to"][1] - fp["from"][1], 0.0)
    moved = _shift("forklift", d)
    fork = [to_primitive(moved(r)) for r in dump["records"] if r["object"] == "forklift"]
    load = [to_primitive(_shift("dropped load", (5.0, 0.0, 5.0))(r)) for r in dump["records"]
            if r["object"] == "dropped load"]                      # at (5, -7) on the floor, far from B
    t = 6.0
    p, v = traj.evaluate(t, 0)[0], traj.evaluate(t, 1)[0]
    for obs, expected in ((fork, True), (load, False)):
        upd = ObstacleUpdate("x", t - 0.2, obs)
        mine = conflict_check(traj, upd, t, apcfg.policy(cfg).trajectory)
        ref = sup.assess_update(MotionState(t, p, v, np.zeros(3)), traj, upd)["conflict"]
        assert mine == bool(ref) == expected


def test_c1_closed_gate_leaves_no_route(cfg, dump):
    m = cfg["missions"]["C1_no_route"]
    scene, _, _ = _scene(cfg, dump, moved=_shift("cage gate", (0.0, cfg["scene_events"]["gate"]["dy"], 0.0)))
    pr, _ = _plan(cfg, scene, _start(m), m["goal"])
    assert not pr.success


def _characterisation():
    with open(sorted(glob.glob(os.path.join(HERE, "data", "characterisation_*.json")))[-1]) as f:
        return json.load(f)


def test_stopping_assumptions_contain_measured_stops(cfg):
    """Every measured brake (Webots, 0.6-1.0 m/s) stops within [D_min, D_max] of the configured
    conservative assumptions."""
    a = apcfg.stopping_assumptions(cfg)
    runs = [r for r in _characterisation()["runs"] if "stop_distance_from_cmd" in r]
    assert len(runs) >= 3
    free = lambda q: np.full(len(q), 10.0)  # noqa: E731
    for r in runs:
        s = assess_stopping([0, 0, 1.2], [r["speed_at_cmd"], 0, 0], free, 0.1, 0.35, a)
        assert s.best_stop_distance <= r["stop_distance_from_cmd"] <= s.worst_stop_distance, (r, s.to_dict())


def test_limits_and_allowance_cover_measured_tracking(cfg):
    """Horizontal, vertical and diagonal flights up to the speed limit stay inside the allowance."""
    lim, pol = cfg["limits"], cfg["robot"]
    runs = [r for r in _characterisation()["runs"]
            if "stop_distance_from_cmd" not in r and r["v_max"] <= lim["max_speed"] + 1e-9]
    assert {r.get("kind") for r in runs} >= {None, "climb", "descend", "diagonal_up", "diagonal_down"}
    for r in runs:
        assert r["tracking"]["max"] < pol["tracking_allowance"], r

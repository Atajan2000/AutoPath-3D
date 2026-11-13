import numpy as np
import pytest

from autopath3d.collision import CollisionChecker, RobotModel
from autopath3d.environments import Box, get_scene
from autopath3d.evaluation.validation import validate_path, voxel_map_clearance
from autopath3d.maps import OccupancyGrid
from autopath3d.planners import RRTStarPlanner, RRTStarTree, Status, choose_parent, rewire

from conftest import make_world


def _tree(points, parents):
    t = RRTStarTree(np.asarray(points[0], float), 4)
    for p, par in zip(points[1:], parents[1:]):
        t.add(np.asarray(p, float), par, t.cost[par] + np.linalg.norm(np.subtract(p, t.pos[par])))
    return t


def test_choose_parent_selects_min_cost_collision_free():
    # a wall blocks the straight line from the cheapest candidates to the new point
    sc, grid, ck = make_world([Box((1.8, 1.5, 0), (2.2, 2.5, 4))], robot=RobotModel(0.1, 0.0))
    t = _tree([[0.5, 2.0, 2.0], [1.5, 2.0, 2.0], [2.0, 3.0, 2.0], [2.5, 0.5, 2.0]], [-1, 0, 0, 0])
    x = np.array([3.0, 2.0, 2.0])
    near = np.array([0, 1, 2, 3])
    parent, cost, checks = choose_parent(t, ck, x, near)
    costs = t.cost[near] + np.linalg.norm(t.pos[near] - x, axis=1)
    free = np.array([ck.segment_free(t.pos[i], x) for i in near])
    assert list(free) == [False, False, True, True]
    assert costs[~free].min() < costs[free].min()               # cheaper candidates are blocked
    assert parent == 2 and cost == pytest.approx(costs[2])
    assert costs[2] < costs[3]
    assert checks == 3


def test_rewire_updates_parent_and_descendant_costs():
    sc, grid, ck = make_world([], robot=RobotModel(0.1, 0.0))
    # 0 -> 1 -> 2 -> 3 is a long detour to node 3 which has children 4 and 5 (and 6 under 5)
    pts = [[0.5, 0.5, 2], [0.5, 3.5, 2], [3.5, 3.5, 2], [3.5, 0.5, 2], [3.5, 0.5, 3], [3.0, 0.5, 1], [2.6, 0.5, 1]]
    t = _tree(pts, [-1, 0, 1, 2, 3, 3, 5])
    old = t.cost.copy()
    new = t.add(np.array([2.0, 0.5, 2.0]), 0, 1.5)
    rw, checks, updates = rewire(t, ck, new, np.array([0, 1, 2, 3]))
    assert rw >= 1 and t.parent[3] == new
    assert updates >= 4                                          # node 3 + 3 descendants
    assert t.cost[3] == pytest.approx(3.0)
    assert t.cost[4] == pytest.approx(4.0)
    assert t.cost[6] == pytest.approx(3.0 + np.linalg.norm(np.subtract(pts[5], pts[3])) + 0.4)
    assert all(t.cost[i] < old[i] for i in (3, 4, 5, 6))
    assert 3 not in t.children[2] and 3 in t.children[new]
    assert t.check_consistency() == []


def test_rewire_skips_blocked_improvement():
    sc, grid, ck = make_world([Box((2.6, 0.0, 0.0), (2.8, 4.0, 4.0))], robot=RobotModel(0.1, 0.0))
    t = _tree([[0.5, 0.5, 2], [0.5, 3.5, 2], [3.5, 3.5, 2.0], [3.5, 0.5, 2.0]], [-1, 0, 1, 2])
    new = t.add(np.array([2.0, 0.5, 2.0]), 0, 1.5)
    rw, _, _ = rewire(t, ck, new, np.array([3]))
    assert rw == 0 and t.parent[3] == 2


def test_tree_consistency_detects_corruption():
    t = _tree([[0, 0, 0], [1, 0, 0], [2, 0, 0]], [-1, 0, 1])
    assert t.check_consistency() == []
    t.cost[2] = 0.5
    assert t.check_consistency()


@pytest.fixture(scope="module")
def narrow():
    sc = get_scene("narrow_1200")
    grid = OccupancyGrid.from_scene(sc, 0.2)
    return sc, grid, CollisionChecker(grid, RobotModel())


def test_narrow_passage_success_and_validation(narrow):
    sc, grid, ck = narrow
    t = sc.tasks[0]
    r = RRTStarPlanner(ck, max_iterations=3000, step_size=1.5, seed=0, record_tree=True).plan(t.start, t.goal)
    assert r.success and r.stats["tree_consistency_errors"] == []
    assert r.stats["rewires"] > 0
    v = validate_path(sc, grid, r.path, t.start, t.goal, ck.c)
    assert v.valid
    # every final tree edge passes exact voxel validation, too
    pos, par = r.stats["tree_pos"], r.stats["tree_parent"]
    rng = np.random.default_rng(0)
    for i in rng.choice(np.arange(1, len(pos)), size=200, replace=False):
        seg = np.vstack([pos[par[i]], pos[i]])
        assert voxel_map_clearance(grid, seg, 1.0)["min_clearance"] >= ck.c - 1e-9
    # history is monotone non-increasing and ends at the returned cost
    costs = [h[1] for h in r.stats["cost_history"]]
    assert all(b <= a for a, b in zip(costs, costs[1:]))
    assert costs[-1] == pytest.approx(r.length)


def test_determinism_and_budget_prefix(narrow):
    sc, grid, ck = narrow
    t = sc.tasks[0]
    a = RRTStarPlanner(ck, max_iterations=1500, seed=7, checkpoints=(700,)).plan(t.start, t.goal)
    b = RRTStarPlanner(ck, max_iterations=1500, seed=7).plan(t.start, t.goal)
    c = RRTStarPlanner(ck, max_iterations=700, seed=7).plan(t.start, t.goal)
    assert np.array_equal(a.path, b.path) and a.stats["tree_nodes"] == b.stats["tree_nodes"]
    cp = a.stats["checkpoints"][0]
    assert cp["iteration"] == 700 and cp["tree_nodes"] == c.stats["tree_nodes"]
    if c.success:
        assert cp["best_cost"] == pytest.approx(c.length)
    d = RRTStarPlanner(ck, max_iterations=1500, seed=8).plan(t.start, t.goal)
    assert d.stats["tree_nodes"] != a.stats["tree_nodes"] or not np.array_equal(d.path, a.path)


def test_infeasible_and_invalid_requests():
    sc = get_scene("narrow_500")
    grid = OccupancyGrid.from_scene(sc, 0.2)
    ck = CollisionChecker(grid, RobotModel())
    t = sc.tasks[0]
    r = RRTStarPlanner(ck, max_iterations=1500, seed=0).plan(t.start, t.goal)
    assert r.status == Status.BUDGET_EXHAUSTED and r.path is None
    assert RRTStarPlanner(ck).plan([6.0, 1.0, 2.0], t.goal).status == Status.INVALID_START
    assert RRTStarPlanner(ck).plan(t.start, [10.0, 4.0, 3.8]).status == Status.INVALID_GOAL
    z = RRTStarPlanner(ck).plan(t.start, t.start)
    assert z.success and z.length == 0.0

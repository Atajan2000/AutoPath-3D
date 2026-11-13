import itertools
import math

import numpy as np
import pytest
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import dijkstra

from autopath3d.collision import RobotModel
from autopath3d.environments import Box, get_scene
from autopath3d.evaluation.validation import validate_path, voxel_map_clearance
from autopath3d.maps import OccupancyGrid
from autopath3d.collision import CollisionChecker
from autopath3d.planners import AStarPlanner, Status, lattice_moves, subcube_corners

from conftest import make_world


def centre(grid, ijk):
    return grid.grid_to_world(np.array(ijk))[0]


@pytest.mark.parametrize("conn,expected", [
    (26, lambda d: d[0] + (math.sqrt(2) - 1) * d[1] + (math.sqrt(3) - math.sqrt(2)) * d[2]),
    (6, lambda d: sum(d)),
])
def test_cost_in_free_space_equals_lattice_distance(conn, expected):
    sc, grid, ck = make_world([], res=0.2, robot=RobotModel(0.05, 0.0))
    a, b = (3, 4, 5), (15, 9, 7)
    res = AStarPlanner(ck, connectivity=conn).plan(centre(grid, a), centre(grid, b))
    d = sorted((abs(x - y) for x, y in zip(a, b)), reverse=True)
    assert res.success
    assert res.length == pytest.approx(0.2 * expected(d), abs=1e-9)


def test_moves_and_subcube_corners():
    assert len(lattice_moves(6)) == 6 and len(lattice_moves(18)) == 18 and len(lattice_moves(26)) == 26
    assert subcube_corners((1, 0, 0)) == []
    assert sorted(subcube_corners((1, -1, 0))) == [(0, -1, 0), (1, 0, 0)]
    assert len(subcube_corners((1, 1, 1))) == 6


def test_no_diagonal_corner_cutting():
    # tiny robot so the lattice rule (not inflation) is what matters
    r = 1.0
    obs = [Box((1, 0, 0), (2, 1, 3)), Box((0, 1, 0), (1, 2, 3))]  # block (1,0) and (0,1) columns
    sc, grid, ck = make_world(obs, res=r, lo=(0, 0, 0), hi=(4, 4, 3), robot=RobotModel(0.01, 0.0))
    pl = AStarPlanner(ck, connectivity=26)
    res = pl.plan(centre(grid, (0, 0, 1)), centre(grid, (1, 1, 1)))
    # the only exits from (0,0,*) are diagonal through blocked corners -> no path
    assert res.status == Status.NO_PATH


def test_returned_lattice_moves_respect_corner_rule():
    sc = get_scene("warehouse")
    grid = OccupancyGrid.from_scene(sc, 0.2)
    ck = CollisionChecker(grid, RobotModel())
    t = sc.task("aisle_to_aisle")
    res = AStarPlanner(ck).plan(t.start, t.goal)
    assert res.success
    valid = ck.f_total >= ck.c
    idx = grid.world_to_grid(res.path[1:-1])
    for a, b in zip(idx[:-1], idx[1:]):
        m = tuple(int(x) for x in b - a)
        assert max(map(abs, m)) == 1
        for u in subcube_corners(m):
            assert valid[tuple(a + np.array(u))]


def test_lattice_edge_lemma_against_exact_validator():
    """Edges allowed by node validity + corner rule are exactly collision-free."""
    rng = np.random.default_rng(0)
    obs = [Box(tuple(lo), tuple(lo + rng.uniform(0.1, 0.7, 3))) for lo in rng.uniform(0, 3.3, size=(12, 3))]
    sc, grid, ck = make_world(obs, res=0.2, robot=RobotModel(0.2, 0.05))
    valid = ck.f_total >= ck.c
    cand = np.argwhere(valid)
    rng.shuffle(cand)
    checked = 0
    for a in cand[:150]:
        for m in lattice_moves(26):
            b = a + m
            if not grid.index_in_bounds(b[None])[0] or not valid[tuple(b)]:
                continue
            if not all(valid[tuple(a + np.array(u))] for u in subcube_corners(m)):
                continue
            seg = grid.grid_to_world(np.vstack([a, b]))
            assert voxel_map_clearance(grid, seg, 2.0)["min_clearance"] >= ck.c - 1e-9
            checked += 1
    assert checked > 1000


def test_optimal_cost_matches_independent_dijkstra():
    obs = [Box((1.0, 0.0, 0.0), (1.4, 3.2, 4.0)), Box((2.4, 0.8, 0.0), (2.8, 4.0, 4.0)),
           Box((0.0, 0.0, 2.4), (4.0, 4.0, 2.6))]
    sc, grid, ck = make_world(obs, res=0.2, robot=RobotModel(0.15, 0.05))
    valid = ck.f_total >= ck.c
    shape = valid.shape
    ids = -np.ones(shape, int)
    nodes = np.argwhere(valid)
    ids[tuple(nodes.T)] = np.arange(len(nodes))
    rows, cols, w = [], [], []
    for n, a in enumerate(nodes):
        for m in lattice_moves(26):
            b = a + m
            if np.any(b < 0) or np.any(b >= shape) or not valid[tuple(b)]:
                continue
            if all(valid[tuple(a + np.array(u))] for u in subcube_corners(m)):
                rows.append(n); cols.append(ids[tuple(b)]); w.append(0.2 * np.linalg.norm(m))
    G = coo_matrix((w, (rows, cols)), shape=(len(nodes), len(nodes))).tocsr()
    s, g = (2, 2, 2), (17, 17, 3)
    ref = dijkstra(G, indices=ids[s])[ids[g]]
    res = AStarPlanner(ck).plan(centre(grid, s), centre(grid, g))
    assert res.success and math.isfinite(ref)
    assert res.stats["lattice_cost"] == pytest.approx(ref, abs=1e-9)
    assert res.length == pytest.approx(ref, abs=1e-9)


def test_blocked_route_and_invalid_endpoints(wall_world):
    sc, grid, ck = wall_world
    pl = AStarPlanner(ck)
    assert pl.plan([1.0, 2, 2], [3.0, 2, 2]).status == Status.NO_PATH
    r = pl.plan([2.0, 2, 2], [3.0, 2, 2])
    assert r.status == Status.INVALID_START and r.reason == "inside_occupied_voxel"
    r = pl.plan([1.0, 2, 2], [1.6, 2, 2])
    assert r.status == Status.INVALID_GOAL and r.reason.startswith("insufficient_clearance")
    assert pl.plan([1.0, 2, 2], [5.0, 2, 2]).status == Status.INVALID_GOAL
    assert pl.plan([1.0, 2, 0.1], [1.0, 3, 2]).status == Status.INVALID_START


def test_unreachable_goal_enclosed_cell():
    # goal inside a closed box shell -> unreachable although the goal itself is valid
    shell = [Box((2, 2, 2), (3.6, 3.6, 2.2)), Box((2, 2, 3.4), (3.6, 3.6, 3.6)),
             Box((2, 2, 2.2), (2.2, 3.6, 3.4)), Box((3.4, 2, 2.2), (3.6, 3.6, 3.4)),
             Box((2.2, 2, 2.2), (3.4, 2.2, 3.4)), Box((2.2, 3.4, 2.2), (3.4, 3.6, 3.4))]
    sc, grid, ck = make_world(shell, hi=(5, 5, 5), robot=RobotModel(0.2, 0.05))
    goal = np.array([2.8, 2.8, 2.8])
    assert ck.point_free(goal[None])[0]
    assert AStarPlanner(ck).plan([1, 1, 1], goal).status == Status.NO_PATH


def test_zero_length_request(wall_world):
    _, _, ck = wall_world
    r = AStarPlanner(ck).plan([1.0, 2, 2], [1.0, 2, 2])
    assert r.success and r.length == 0.0 and len(r.path) == 1


def test_start_close_to_obstacle_boundary(wall_world):
    sc, grid, ck = wall_world
    start = np.array([1.8 - 0.36, 2.03, 2.07])                 # 1 cm above the limit
    r = AStarPlanner(ck).plan(start, [0.6, 0.6, 0.6])
    assert r.success
    assert validate_path(sc, grid, r.path, start, [0.6, 0.6, 0.6], ck.c).valid


def test_narrow_passages_and_determinism():
    for name, feasible in [("narrow_1200", True), ("narrow_500", False)]:
        sc = get_scene(name)
        grid = OccupancyGrid.from_scene(sc, 0.2)
        ck = CollisionChecker(grid, RobotModel())
        t = sc.tasks[0]
        r1 = AStarPlanner(ck).plan(t.start, t.goal)
        assert r1.success == feasible
        if feasible:
            r2 = AStarPlanner(ck).plan(t.start, t.goal)
            assert np.array_equal(r1.path, r2.path)
            v = validate_path(sc, grid, r1.path, t.start, t.goal, ck.c)
            assert v.valid and v.gt_min_clearance >= 0.35

import itertools

import numpy as np
import pytest

from autopath3d.environments import SCENES, Box, VerticalCylinder, get_scene
from autopath3d.maps import FREE, OCCUPIED, UNKNOWN, OccupancyGrid, cube_distance_field, surface_mask

from conftest import make_scene


def test_world_grid_roundtrip_and_faces():
    g = OccupancyGrid.empty((-1, 2, 0), (1, 4, 1), 0.25)
    assert g.shape == (8, 8, 4)
    idx = np.array([[0, 0, 0], [7, 7, 3], [3, 5, 1]])
    centres = g.grid_to_world(idx)
    assert np.allclose(centres[0], [-0.875, 2.125, 0.125])
    assert np.array_equal(g.world_to_grid(centres), idx)
    # points on faces belong to the upper voxel (floor convention)
    assert np.array_equal(g.world_to_grid([[-0.75, 2.0, 0.5]]), [[1, 0, 2]])
    assert np.array_equal(g.world_to_grid([[-1.0001, 2, 0]])[0, 0], -1)
    assert not g.index_in_bounds(np.array([[8, 0, 0]]))[0]


def test_bounds_must_be_multiples_of_resolution():
    with pytest.raises(ValueError):
        OccupancyGrid.empty((0, 0, 0), (1.1, 1, 1), 0.2)


def test_state_outside_grid_is_unknown():
    g = OccupancyGrid.empty((0, 0, 0), (1, 1, 1), 0.2)
    assert g.state_at([[0.5, 0.5, 0.5]])[0] == FREE
    assert g.state_at([[1.5, 0.5, 0.5]])[0] == UNKNOWN
    assert g.state_at([[0.5, 0.5, -0.01]])[0] == UNKNOWN


def test_box_insertion_exact_voxel_set():
    g = OccupancyGrid.empty((0, 0, 0), (2, 2, 2), 0.2)
    g.mark_box((0.2, 0.2, 0.2), (0.6, 0.5, 0.6), OCCUPIED)
    occ = {tuple(i) for i in np.argwhere(g.states == OCCUPIED)}
    # x: [0.2,0.6] -> voxels 1,2 (touching voxels 0 and 3 excluded); y: [0.2,0.5] -> 1,2
    assert occ == set(itertools.product([1, 2], [1, 2], [1, 2]))


def test_cylinder_insertion_matches_brute_force():
    g = OccupancyGrid.empty((0, 0, 0), (3, 3, 2), 0.1)
    cyl = VerticalCylinder((1.43, 1.57), 0.61, 0.25, 1.35)
    g.mark_primitive(cyl, OCCUPIED)
    r = 0.1
    for i, j, k in itertools.product(range(30), range(30), range(20)):
        x0, y0, z0 = i * r, j * r, k * r
        dx = max(x0 - 1.43, 1.43 - (x0 + r), 0.0)
        dy = max(y0 - 1.57, 1.57 - (y0 + r), 0.0)
        expect = (dx * dx + dy * dy < 0.61 ** 2) and (z0 < 1.35) and (z0 + r > 0.25)
        assert (g.states[i, j, k] == OCCUPIED) == expect


@pytest.mark.parametrize("name", list(SCENES))
def test_voxelisation_is_conservative(name):
    """Every interior point of every obstacle lies in an occupied voxel."""
    sc = get_scene(name)
    g = OccupancyGrid.from_scene(sc, 0.2)
    rng = np.random.default_rng(0)
    for ob in sc.obstacles:
        lo, hi = ob.aabb()
        p = rng.uniform(lo, hi, size=(200, 3))
        p = p[ob.distance(p) == 0.0]
        p = p[np.all((p > sc.lo) & (p < sc.hi), axis=1)]
        assert np.all(g.state_at(p) == OCCUPIED), ob.name


def test_unknown_regions_do_not_overwrite_obstacles():
    sc = make_scene([Box((1, 1, 1), (2, 2, 2))], unknown=[Box((0, 0, 0), (3, 3, 3))])
    g = OccupancyGrid.from_scene(sc, 0.5)
    assert g.state_at([[1.5, 1.5, 1.5]])[0] == OCCUPIED
    assert g.state_at([[0.25, 0.25, 0.25]])[0] == UNKNOWN
    assert g.state_at([[3.5, 3.5, 3.5]])[0] == FREE


def test_cube_distance_field_matches_brute_force():
    rng = np.random.default_rng(3)
    blocked = rng.random((9, 7, 6)) < 0.05
    res = 0.3
    f, cap = cube_distance_field(blocked, res, cap=1.0)
    bl = np.argwhere(blocked)
    for idx in itertools.product(range(9), range(7), range(6)):
        t = np.abs(bl - np.array(idx))
        exact = res * np.sqrt((np.maximum(t - 0.5, 0) ** 2).sum(axis=1)).min()
        assert f[idx] == pytest.approx(min(exact, cap), abs=1e-12)
        assert f[idx] <= exact + 1e-12


def test_surface_mask():
    b = np.zeros((5, 5, 5), bool)
    b[1:4, 1:4, 1:4] = True
    s = surface_mask(b)
    assert not s[2, 2, 2] and s[1, 2, 2] and s.sum() == 26

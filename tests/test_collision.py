import numpy as np
import pytest

from autopath3d.collision import RobotModel
from autopath3d.environments import Box, VerticalCylinder
from autopath3d.evaluation.validation import voxel_map_clearance

from conftest import make_world


def test_robot_clearance_includes_margin():
    assert RobotModel(0.25, 0.1).clearance == pytest.approx(0.35)


def test_robot_radius_inflation(wall_world):
    _, _, ck = wall_world
    # wall face at x = 1.8; robot centre must stay >= 0.35 m away
    assert ck.point_free([[1.8 - 0.36, 2, 2]])[0]
    assert not ck.point_free([[1.8 - 0.34, 2, 2]])[0]
    assert not ck.point_free([[2.0, 2, 2]])[0]                  # inside the wall
    assert ck.clearance([[1.3, 2, 2]])[0] == pytest.approx(0.5, abs=1e-9)   # exact near walls
    assert 0.5 <= ck.clearance([[1.0, 2, 2]])[0] <= 0.8                     # lower bound far away


def test_lattice_inflation_matches_definition(wall_world):
    _, grid, ck = wall_world
    valid = ck.f_total >= ck.c
    centres_x = grid.grid_to_world(np.array([[i, 10, 10] for i in range(20)]))[:, 0]
    # free iff centre is >= 0.35 from the wall and the bounds
    expect = ((centres_x <= 1.8 - 0.35) | (centres_x >= 2.2 + 0.35)) & (centres_x >= 0.35) & (centres_x <= 4 - 0.35)
    assert np.array_equal(valid[:, 10, 10], expect)


def test_bounds_are_obstacles(wall_world):
    _, _, ck = wall_world
    assert not ck.point_free([[1.0, 2.0, 0.3]])[0]      # too close to the floor
    assert ck.point_free([[1.0, 2.0, 0.4]])[0]
    assert not ck.point_free([[1.0, 2.0, -0.5]])[0]     # outside the map
    assert not ck.point_free([[1.0, 4.2, 2.0]])[0]


def test_unknown_space_blocks_unless_explicitly_free():
    unk = [Box((1.6, 0, 0), (2.4, 4, 4), "unobserved")]
    _, _, ck = make_world([], unknown=unk)
    assert not ck.point_free([[2.0, 2, 2]])[0]
    assert not ck.point_free([[1.4, 2, 2]])[0]
    assert not ck.segment_free(np.array([1.0, 2, 2]), np.array([3.0, 2, 2]))
    _, _, ck2 = make_world([], unknown=unk, treat_unknown=False)
    assert ck2.point_free([[2.0, 2, 2]])[0]
    assert ck2.segment_free(np.array([1.0, 2, 2]), np.array([3.0, 2, 2]))


def test_thin_obstacle_between_endpoints_is_detected():
    thin = Box((1.99, 0, 0), (2.01, 4, 4), "sheet")          # 2 cm sheet
    _, grid, ck = make_world([thin])
    a, b = np.array([1.0, 2, 2]), np.array([3.0, 2, 2])
    assert ck.point_free(np.vstack([a, b])).all()             # endpoints alone look fine
    assert not ck.segment_free(a, b)
    assert grid.state_at([[2.0, 2, 2]])[0] == 1


def test_segment_near_boundary_accepts_and_rejects(wall_world):
    _, _, ck = wall_world
    x_ok, x_bad = 1.8 - 0.36, 1.8 - 0.34
    assert ck.segment_free(np.array([x_ok, 0.5, 0.5]), np.array([x_ok, 3.5, 3.5]))
    assert not ck.segment_free(np.array([1.0, 0.5, 0.5]), np.array([x_bad, 3.5, 3.5]))


def test_clearance_is_lower_bound_and_tight_near_obstacles():
    obs = [Box((1.0, 1.0, 0.0), (1.6, 3.0, 2.2)), VerticalCylinder((3.0, 1.0), 0.33, 0.0, 3.1)]
    _, grid, ck = make_world(obs)
    rng = np.random.default_rng(0)
    p = rng.uniform(0.05, 3.95, size=(400, 3))
    lb = ck.clearance(p)
    exact = np.array([voxel_map_clearance(grid, q[None, :], 5.0)["min_clearance"] for q in p])
    assert np.all(lb <= exact + 1e-9)
    near = exact < ck.c + 0.1
    assert near.sum() > 20
    assert np.allclose(lb[near], exact[near], atol=1e-9)    # exact refinement near obstacles


def test_segment_checker_is_sound_against_exact_validator():
    obs = [Box((1.0, 1.0, 0.0), (1.6, 3.0, 2.2)), Box((2.5, 0.0, 1.0), (2.55, 4.0, 1.6)),
           VerticalCylinder((3.0, 3.0), 0.3, 0.0, 4.0)]
    _, grid, ck = make_world(obs)
    rng = np.random.default_rng(1)
    accepted = rejected_safe = 0
    for _ in range(300):
        a, b = rng.uniform(0.4, 3.6, size=(2, 3))
        exact = voxel_map_clearance(grid, np.vstack([a, b]), 5.0)["min_clearance"]
        if ck.segment_free(a, b):
            accepted += 1
            assert exact >= ck.c - 1e-9
        elif exact >= ck.c + 1e-3:
            rejected_safe += 1
    assert accepted > 30
    # the certificate is conservative only in near-grazing cases
    assert rejected_safe == 0

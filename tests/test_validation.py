import numpy as np
import pytest

from autopath3d.environments import Box, VerticalCylinder
from autopath3d.evaluation.validation import ground_truth_clearance, validate_path

from conftest import make_scene, make_world


def test_exact_segment_distances():
    sc = make_scene([Box((1, 1, 1), (2, 2, 2)), VerticalCylinder((3.0, 3.0), 0.5, 0.0, 4.0)], hi=(10, 10, 10))
    # segment parallel to the box top face at height 1.0 above it
    r = ground_truth_clearance(sc, np.array([[0.0, 1.5, 3.0], [5.0, 1.5, 3.0]]))
    assert r["min_obstacle_clearance"] == pytest.approx(1.0, abs=1e-9)
    # segment passing 0.2 m from the cylinder surface (closest point mid-segment)
    r = ground_truth_clearance(sc, np.array([[2.3, 4.7, 2.0], [4.7, 2.3, 2.0]]))
    # line x + y = 7 passes 1/sqrt(2) from the axis (3, 3)
    assert r["min_obstacle_clearance"] == pytest.approx(1 / np.sqrt(2) - 0.5, abs=1e-9)


def test_validator_catches_thin_obstacle_between_waypoints():
    thin = Box((1.99, 0, 0), (2.01, 4, 4), "sheet")
    sc, grid, _ = make_world([thin])
    path = np.array([[1.0, 2, 2], [3.0, 2, 2]])
    v = validate_path(sc, grid, path, path[0], path[-1], 0.35)
    assert not v.valid and v.gt_min_clearance < 1e-9 and v.closest_obstacle == "sheet"


def test_validator_rejects_wrong_endpoints_and_bounds():
    sc, grid, _ = make_world([])
    path = np.array([[1.0, 2, 2], [3.0, 2, 2]])
    assert not validate_path(sc, grid, path, path[0], [3.0, 2.0, 2.1], 0.35).valid
    low = np.array([[1.0, 2, 0.2], [3.0, 2, 2]])
    assert not validate_path(sc, grid, low, low[0], low[-1], 0.35).valid
    assert validate_path(sc, grid, path, path[0], path[-1], 0.35).valid

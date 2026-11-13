import numpy as np
import pytest

from autopath3d.collision import CollisionChecker, RobotModel
from autopath3d.environments import Box, Scene, Task
from autopath3d.maps import OccupancyGrid


def make_scene(obstacles, lo=(0, 0, 0), hi=(4, 4, 4), tasks=(), unknown=()):
    return Scene("test", "unit-test scene", np.array(lo, float), np.array(hi, float),
                 list(obstacles), list(tasks), list(unknown))


def make_world(obstacles, res=0.2, lo=(0, 0, 0), hi=(4, 4, 4), robot=RobotModel(0.25, 0.10),
               unknown=(), treat_unknown=True):
    sc = make_scene(obstacles, lo, hi, unknown=unknown)
    grid = OccupancyGrid.from_scene(sc, res)
    return sc, grid, CollisionChecker(grid, robot, treat_unknown)


@pytest.fixture
def robot():
    return RobotModel(0.25, 0.10)


@pytest.fixture
def wall_world():
    """4x4x4 room with a full 0.4 m thick wall at x in [1.8, 2.2] (grid-aligned)."""
    return make_world([Box((1.8, 0, 0), (2.2, 4, 4), "wall")])

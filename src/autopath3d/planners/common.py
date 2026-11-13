"""Planner result types and start/goal validation shared by all planners."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum

import numpy as np

from ..collision.checker import CollisionChecker
from ..maps.occupancy import OCCUPIED, UNKNOWN


class Status(str, Enum):
    SUCCESS = "success"
    INVALID_START = "invalid_start"
    INVALID_GOAL = "invalid_goal"
    NO_PATH = "no_path"                    # search space exhausted (A*)
    BUDGET_EXHAUSTED = "budget_exhausted"  # no solution within budget (RRT*)


@dataclass
class PlanResult:
    planner: str
    status: Status
    path: np.ndarray | None = None          # (N, 3) waypoints, metres
    length: float = float("nan")
    planning_time_s: float = 0.0            # search / sampling loop only
    setup_time_s: float = 0.0               # planner-specific preprocessing
    reason: str = ""
    stats: dict = field(default_factory=dict)

    @property
    def success(self) -> bool:
        return self.status == Status.SUCCESS


def path_length(path: np.ndarray) -> float:
    path = np.atleast_2d(path)
    if len(path) < 2:
        return 0.0
    return float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum())


def endpoint_problem(checker: CollisionChecker, p: np.ndarray) -> str | None:
    """Return None if p is a valid robot configuration, else a reason string."""
    p = np.asarray(p, float).reshape(1, 3)
    g = checker.grid
    if not g.point_in_bounds(p)[0]:
        return "out_of_bounds"
    st = g.state_at(p)[0]
    if st == OCCUPIED:
        return "inside_occupied_voxel"
    if st == UNKNOWN and checker.treat_unknown_as_occupied:
        return "inside_unknown_voxel"
    clr = checker.clearance(p)[0]
    if clr < checker.c:
        return f"insufficient_clearance({clr:.3f}<{checker.c:.3f})"
    return None


def validate_endpoints(checker: CollisionChecker, start, goal, planner: str) -> PlanResult | None:
    """Return a failure PlanResult if start or goal is invalid, otherwise None."""
    r = endpoint_problem(checker, start)
    if r is not None:
        return PlanResult(planner, Status.INVALID_START, reason=r)
    r = endpoint_problem(checker, goal)
    if r is not None:
        return PlanResult(planner, Status.INVALID_GOAL, reason=r)
    return None

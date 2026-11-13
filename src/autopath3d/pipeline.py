"""Plan -> shortcut -> trajectory -> independent validation, with separate timings.

Clearance tiers (metres):
  nominal   c_nom  = robot radius + safety margin           (Phase 1 geometric planning)
  trajectory c_traj = c_nom + tracking_allowance            (room for controller error)
Trajectory pipelines plan, shortcut and certify with a checker built for c_traj, and the
independent validator requires ground-truth clearance >= c_traj.
"""

from __future__ import annotations

import time
from dataclasses import dataclass

import numpy as np

from .collision.checker import CollisionChecker, RobotModel
from .evaluation.trajectory_validation import validate_trajectory
from .maps.occupancy import OccupancyGrid
from .planners.astar import AStarPlanner
from .planners.rrt_star import RRTStarPlanner
from .trajectories.generator import TrajectoryLimits, generate_trajectory
from .trajectories.polynomial import PiecewisePolynomial
from .trajectories.shortcut import shortcut_path


@dataclass(frozen=True)
class ClearancePolicy:
    robot_radius: float = 0.25
    safety_margin: float = 0.10
    tracking_allowance: float = 0.10

    @property
    def nominal(self) -> float:
        return self.robot_radius + self.safety_margin

    @property
    def trajectory(self) -> float:
        return self.nominal + self.tracking_allowance

    def trajectory_robot(self) -> RobotModel:
        return RobotModel(self.robot_radius, self.safety_margin + self.tracking_allowance)


def trajectory_checker(scene, resolution: float, policy: ClearancePolicy, clearance_cap: float = 3.0):
    grid = OccupancyGrid.from_scene(scene, resolution)
    return grid, CollisionChecker(grid, policy.trajectory_robot(), True, clearance_cap)


def make_planner(checker: CollisionChecker, spec: dict):
    spec = dict(spec)
    kind = spec.pop("planner")
    if kind == "astar":
        return AStarPlanner(checker, **spec)
    if kind == "rrt_star":
        return RRTStarPlanner(checker, **spec)
    raise ValueError(kind)


@dataclass
class PipelineResult:
    status: str                      # "success" or the failing stage
    reason: str
    path: np.ndarray | None
    simplified: np.ndarray | None
    trajectory: PiecewisePolynomial | None
    record: dict


def validate(traj: PiecewisePolynomial, scene, policy: ClearancePolicy, limits: TrajectoryLimits,
             start, goal, reference=None):
    return validate_trajectory(traj.knots, traj.coeffs, scene.distance, policy.trajectory,
                               limits.max_speed, limits.max_acceleration, start=start, goal=goal,
                               start_velocity=np.zeros(3), reference_path=reference,
                               hold_point=traj.hold_point)


def run_pipeline(scene, checker: CollisionChecker, start, goal, planner_spec: dict,
                 limits: TrajectoryLimits, policy: ClearancePolicy, path: np.ndarray | None = None,
                 t0: float = 0.0) -> PipelineResult:
    """If `path` is given, planning is skipped and that path is used as input."""
    rec: dict = {"planner": planner_spec.get("planner") if path is None else "given_path"}
    if path is None:
        pr = make_planner(checker, planner_spec).plan(start, goal)
        rec.update(plan_status=pr.status.value, plan_reason=pr.reason, planning_time_s=pr.planning_time_s,
                   planner_setup_s=pr.setup_time_s, path_length_m=pr.length,
                   path_waypoints=0 if pr.path is None else len(pr.path))
        if not pr.success:
            return PipelineResult("plan_failed", pr.status.value + ": " + pr.reason, None, None, None, rec)
        path = pr.path
    else:
        rec.update(planning_time_s=0.0, path_length_m=float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum()),
                   path_waypoints=len(path))
    sc = shortcut_path(path, checker)
    rec["shortcut"] = sc.summary()
    if not sc.success:
        return PipelineResult("shortcut_rejected", sc.reason, path, None, None, rec)
    gen = generate_trajectory(sc.path, checker, limits, t0=t0)
    rec["generation"] = gen.summary()
    if not gen.success:
        return PipelineResult("generation_failed", gen.reason, path, sc.path, None, rec)
    tv = time.perf_counter()
    val = validate(gen.trajectory, scene, policy, limits, start, goal, reference=sc.path)
    rec["validation_time_s"] = time.perf_counter() - tv
    rec["validation"] = val.to_dict()
    if not val.valid:
        return PipelineResult("validation_failed", "; ".join(val.failures), path, sc.path, gen.trajectory, rec)
    return PipelineResult("success", "", path, sc.path, gen.trajectory, rec)

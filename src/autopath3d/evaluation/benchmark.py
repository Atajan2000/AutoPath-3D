"""Reproducible planner benchmark helpers (no plotting, no visualisation)."""

from __future__ import annotations

import time
import tracemalloc
from dataclasses import dataclass

import numpy as np

from ..collision.checker import CollisionChecker, RobotModel
from ..environments.scenes import Scene, get_scene
from ..maps.occupancy import OccupancyGrid
from ..planners.astar import AStarPlanner
from ..planners.common import PlanResult
from ..planners.rrt_star import RRTStarPlanner
from .validation import validate_path


@dataclass
class World:
    scene: Scene
    grid: OccupancyGrid
    checker: CollisionChecker
    robot: RobotModel
    resolution: float
    treat_unknown_as_occupied: bool
    map_build_s: float
    field_build_s: float


def build_world(scene_name: str, cfg: dict, resolution: float | None = None) -> World:
    robot = RobotModel(cfg["robot"]["radius"], cfg["robot"]["safety_margin"])
    res = float(resolution if resolution is not None else cfg["map"]["resolution"])
    unk = bool(cfg["map"].get("treat_unknown_as_occupied", True))
    scene = get_scene(scene_name)
    t0 = time.perf_counter()
    grid = OccupancyGrid.from_scene(scene, res)
    t1 = time.perf_counter()
    checker = CollisionChecker(grid, robot, unk, cfg["map"].get("clearance_cap", 3.0))
    t2 = time.perf_counter()
    return World(scene, grid, checker, robot, res, unk, t1 - t0, t2 - t1)


def make_planner(world: World, planner: str, cfg: dict, **overrides):
    if planner == "astar":
        p = dict(cfg["astar"])
        p.update(overrides)
        return AStarPlanner(world.checker, connectivity=p["connectivity"],
                            connect_radius_voxels=p.get("connect_radius_voxels", 2.0))
    p = {k: v for k, v in cfg["rrt_star"].items() if k != "seeds"}
    p.update(overrides)
    return RRTStarPlanner(world.checker, **p)


def peak_memory_mb(fn) -> float:
    """Peak Python/NumPy heap allocation during fn() (separate run; slows execution)."""
    tracemalloc.start()
    tracemalloc.reset_peak()
    try:
        fn()
        _, peak = tracemalloc.get_traced_memory()
    finally:
        tracemalloc.stop()
    return peak / 2**20


def run_and_record(world: World, task_name: str, planner: str, cfg: dict,
                   measure_memory: bool = False, **overrides) -> dict:
    task = world.scene.task(task_name)
    pl = make_planner(world, planner, cfg, **overrides)
    res: PlanResult = pl.plan(task.start, task.goal)
    rec = {
        "scene": world.scene.name, "task": task_name, "planner": planner,
        "resolution": world.resolution, "robot_clearance": world.robot.clearance,
        "status": res.status.value, "success": res.success, "reason": res.reason,
        "length_m": res.length, "planning_time_s": res.planning_time_s,
        "planner_setup_s": res.setup_time_s, "map_build_s": world.map_build_s,
        "field_build_s": world.field_build_s, "n_waypoints": 0 if res.path is None else len(res.path),
        "straight_line_m": float(np.linalg.norm(np.subtract(task.goal, task.start))),
    }
    rec.update(overrides)
    st = dict(res.stats)
    st.pop("expansion_order", None)
    st.pop("tree_pos", None)
    st.pop("tree_parent", None)
    rec["stats"] = st
    if res.success:
        v = validate_path(world.scene, world.grid, res.path, task.start, task.goal,
                          world.robot.clearance, world.treat_unknown_as_occupied)
        rec.update({"validated": v.valid, "gt_min_clearance_m": v.gt_min_clearance,
                    "voxel_min_clearance_m": v.voxel_min_clearance,
                    "closest_obstacle": v.closest_obstacle,
                    "checker_path_free": bool(world.checker.path_free(res.path)),
                    "path": res.path.tolist()})
    else:
        rec.update({"validated": False, "gt_min_clearance_m": None, "voxel_min_clearance_m": None})
    if measure_memory:
        pl2 = make_planner(world, planner, cfg, **overrides)
        rec["peak_memory_mb"] = peak_memory_mb(lambda: pl2.plan(task.start, task.goal))
    return rec

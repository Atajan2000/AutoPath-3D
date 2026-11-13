"""Planner exploration data for the viewer (numpy only; no Rerun import).

Phase 1/2 saved only summary statistics, not search states. For the few viewer entries the
planners are rerun once, deterministically, with their existing recording flags
(A*: record_expansions, RRT*: record_tree), on the same map, clearance and parameters as
the Phase 2 pipeline. The rerun's path length is compared with the run being visualised
(`matches_pipeline`) so the displayed search provably belongs to that plan.
These reruns are not timed for any benchmark.
"""

from __future__ import annotations

import json

import numpy as np

from ..pipeline import ClearancePolicy, trajectory_checker
from ..planners.astar import AStarPlanner
from ..planners.rrt_star import RRTStarPlanner


def capture_astar(scene, start, goal, policy: ClearancePolicy, map_cfg: dict, connectivity: int = 26,
                  checker=None) -> dict:
    if checker is None:
        _, checker = trajectory_checker(scene, map_cfg["resolution"], policy, map_cfg["clearance_cap"])
    pr = AStarPlanner(checker, connectivity=connectivity, record_expansions=True).plan(start, goal)
    ijk = pr.stats.get("expansion_order", np.zeros((0, 3), np.int32))
    out = {"astar_expanded": checker.grid.grid_to_world(ijk).astype(np.float32) if len(ijk) else
           np.zeros((0, 3), np.float32),
           "astar_status": pr.status.value, "astar_nodes_expanded": int(pr.stats.get("nodes_expanded", 0)),
           "astar_planning_time_s": float(pr.planning_time_s),
           "astar_length": float(pr.length) if pr.success else float("nan")}
    out["astar_path"] = pr.path if pr.success else np.zeros((0, 3))
    return out


def capture_rrt(scene, start, goal, policy: ClearancePolicy, map_cfg: dict, spec: dict, checker=None) -> dict:
    if checker is None:
        _, checker = trajectory_checker(scene, map_cfg["resolution"], policy, map_cfg["clearance_cap"])
    s = {k: v for k, v in spec.items() if k != "planner"}
    pr = RRTStarPlanner(checker, record_tree=True, **s).plan(start, goal)
    hist = np.array(pr.stats.get("cost_history", []), float).reshape(-1, 3)
    return {"rrt_tree_pos": pr.stats["tree_pos"].astype(np.float32),
            "rrt_tree_parent": pr.stats["tree_parent"].astype(np.int32),
            "rrt_path": pr.path if pr.success else np.zeros((0, 3)),
            "rrt_status": pr.status.value, "rrt_iterations": int(pr.stats.get("iterations", 0)),
            "rrt_rewires": int(pr.stats.get("rewires", 0)), "rrt_tree_nodes": int(pr.stats.get("tree_nodes", 0)),
            "rrt_first_solution_iteration": pr.stats.get("first_solution_iteration"),
            "rrt_cost_history": hist, "rrt_planning_time_s": float(pr.planning_time_s),
            "rrt_length": float(pr.length) if pr.success else float("nan"), "rrt_seed": int(s.get("seed", 0))}


def save_exploration(path, data: dict) -> None:
    arrays = {k: v for k, v in data.items() if isinstance(v, np.ndarray)}
    scalars = {k: v for k, v in data.items() if not isinstance(v, np.ndarray)}
    np.savez(path, **arrays, _scalars=np.array([json.dumps(scalars)]))


def load_exploration(path) -> dict:
    with np.load(path, allow_pickle=False) as z:
        out = {k: z[k] for k in z.files if k != "_scalars"}
        out.update(json.loads(str(z["_scalars"][0])))
    return out

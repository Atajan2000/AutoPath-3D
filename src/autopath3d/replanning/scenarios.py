"""Deterministic obstacle-update scenarios on the Phase 1 scenes.

Nominal missions are planned with A* (deterministic) so the obstacle placement relative
to the route is reproducible. `ahead` obstacles are boxes placed at a fixed distance in
front of the robot along its (axis-aligned) direction of travel at t_appear.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field

import numpy as np

from ..environments.primitives import Box
from ..environments.scenes import get_scene
from ..pipeline import ClearancePolicy
from ..trajectories.generator import TrajectoryLimits
from .supervisor import (Outcome, ObstacleUpdate, ReplanningSupervisor, SupervisorConfig, state_at)

ASTAR = {"planner": "astar"}


@dataclass
class Scenario:
    name: str
    description: str
    scene: str
    task: str
    t_appear: float
    obstacles: list = field(default_factory=list)
    ahead: dict | None = None            # {"distance": m, "thickness": m, "half_width": m}
    attempts: list = field(default_factory=lambda: [ASTAR])
    expected: tuple = ()                 # acceptable outcomes (for tests / reporting)


SCENARIOS = [
    Scenario("A_warehouse_aisle_blocked",
             "pallet wall fills the central cross aisle on the route; alternatives exist",
             "warehouse", "aisle_to_aisle", 4.0,
             [Box((14.0, 8.5, 0.0), (16.0, 11.5, 6.0), "pallet_wall")],
             expected=(Outcome.REPLANNED,)),
    Scenario("B_urban_scaffold",
             "8 m scaffold tower appears in the street intersection on the route",
             "urban_canyon", "street_crossing", 8.0,
             [Box((10.0, 17.5, 0.0), (14.0, 22.0, 8.0), "scaffold")],
             expected=(Outcome.REPLANNED,)),
    Scenario("C_narrow_window_sealed",
             "panel seals the only window; no alternative route exists",
             "narrow_1200", "through_window", 0.8,
             [Box((5.5, 3.0, 1.0), (5.9, 5.0, 3.0), "panel")],
             expected=(Outcome.NO_ROUTE_HOLDING,)),
    Scenario("D1_too_close_collision",
             "obstacle appears 1.5 m ahead at ~2.6 m/s: inside the stopping distance",
             "narrow_1200", "through_window", 2.0, ahead={"distance": 1.5},
             expected=(Outcome.UNAVOIDABLE_COLLISION,)),
    Scenario("D2_too_close_margin",
             "obstacle appears just beyond the stopping distance: stops without contact "
             "but inside the safety margin",
             "narrow_1200", "through_window", 2.0, ahead={"distance": 2.5},
             expected=(Outcome.STOP_MARGIN_VIOLATION,)),
    Scenario("D3_far_enough",
             "same obstacle 3.5 m ahead: safe stop and replanning around it",
             "narrow_1200", "through_window", 2.0, ahead={"distance": 3.5},
             expected=(Outcome.REPLANNED,)),
    Scenario("E_warehouse_retry",
             "as A, but the first replanning attempt (RRT*, 50 iterations) fails; the robot "
             "holds at a verified safe stop and the A* retry succeeds",
             "warehouse", "aisle_to_aisle", 4.0,
             [Box((14.0, 8.5, 0.0), (16.0, 11.5, 6.0), "pallet_wall")],
             attempts=[{"planner": "rrt_star", "max_iterations": 50, "step_size": 2.5, "seed": 0}, ASTAR],
             expected=(Outcome.REPLANNED_AFTER_RETRY,)),
]


def _ahead_box(p, v, distance, thickness=0.4, half_width=0.6) -> Box:
    d = v / np.linalg.norm(v)
    ax = int(np.argmax(np.abs(d)))
    if abs(d[ax]) < 0.999:
        raise ValueError("ahead obstacles require axis-aligned travel")
    lo = p - half_width
    hi = p + half_width
    s = np.sign(d[ax])
    face = p[ax] + s * distance
    lo[ax], hi[ax] = sorted((face, face + s * thickness))
    return Box(tuple(lo), tuple(hi), "ahead_obstacle")


def run_scenario(sc: Scenario, policy: ClearancePolicy, limits: TrajectoryLimits,
                 base_cfg: SupervisorConfig) -> dict:
    scene = get_scene(sc.scene)
    task = scene.task(sc.task)
    cfg = SupervisorConfig(base_cfg.detection_latency_s, base_cfg.decision_latency_s, base_cfg.resolution,
                           base_cfg.clearance_cap, list(sc.attempts))
    tic = time.perf_counter()
    sup = ReplanningSupervisor(scene, policy, limits, cfg)
    setup = time.perf_counter() - tic
    nominal = sup.plan_nominal(task.start, task.goal, ASTAR)
    out = {"scenario": sc.name, "description": sc.description, "scene": sc.scene, "task": sc.task,
           "expected": [e.value for e in sc.expected], "supervisor_setup_s": setup,
           "nominal_status": nominal.status}
    if nominal.status != "success":
        out.update(outcome=Outcome.NOMINAL_PLAN_FAILED.value, goal_reached=False)
        return out
    tr = nominal.trajectory
    rec = nominal.record
    out["nominal"] = {"path_length_m": rec["path_length_m"], "simplified_length_m": rec["shortcut"]["length"],
                      "duration_s": tr.duration, "planning_time_s": rec["planning_time_s"],
                      "min_clearance": [rec["validation"]["min_clearance_lower"],
                                        rec["validation"]["min_clearance_upper"]]}
    obstacles = list(sc.obstacles)
    if sc.ahead is not None:
        st = state_at(tr, sc.t_appear)
        obstacles.append(_ahead_box(st.position, st.velocity, sc.ahead["distance"]))
    upd = ObstacleUpdate(sc.name, sc.t_appear, obstacles)
    t_detect = sc.t_appear + cfg.detection_latency_s
    rep = sup.handle_update(state_at(tr, t_detect), tr, task.goal, upd)
    rep.pop("new_trajectory", None)
    rep["outcome"] = rep["outcome"].value
    out.update(rep)
    out["obstacles"] = [{"name": o.name, "lo": list(o.lo), "hi": list(o.hi)} for o in obstacles]
    out["matches_expected"] = rep["outcome"] in out["expected"]
    return out

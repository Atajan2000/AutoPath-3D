"""Stop-and-replan supervisor for newly discovered static obstacles.

Protocol (documented, conservative):
  t_appear              new obstacle exists in the world; robot keeps flying its trajectory
  t_detect = t_appear + detection_latency       map update received (MotionState at t_detect)
  conflict check        remaining trajectory vs the new obstacle geometry, continuous-time
                        certificate at the trajectory clearance c_traj
  t_brake = t_detect + decision_latency         if in conflict, constant-deceleration stop
                        along the current velocity direction (no teleporting, no velocity reset)
  stop classification   minimum clearance of everything flown after t_appear and of the hold
                        point, w.r.t. the updated geometry:
                          attained clearance < robot radius   -> UNAVOIDABLE_COLLISION
                          certified clearance < nominal c      -> STOP_MARGIN_VIOLATION
                          moving at detection & hold unsafe    (covered above)
                          hovering & hold clearance < c_nom    -> HOLD_UNSAFE
                          otherwise                            -> safe stop, replan from rest
  replanning            rebuild voxel map, then each configured planner attempt runs the full
                        plan -> shortcut -> trajectory -> independent validation pipeline; the
                        robot resumes only with a validated trajectory. A* exhausting its open
                        set proves no lattice route -> NO_ROUTE_HOLDING.
Moving-obstacle prediction is out of scope: updates are static geometry.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from enum import Enum

import numpy as np

from ..environments.scenes import Scene
from ..pipeline import ClearancePolicy, run_pipeline, trajectory_checker
from ..trajectories.generator import TrajectoryLimits, certify_curve, curve_min_bounds
from ..trajectories.polynomial import PiecewisePolynomial
from .braking import braking_trajectory, stopping_distance


class Outcome(str, Enum):
    NO_CONFLICT = "no_conflict"
    REPLANNED = "replanned"
    REPLANNED_AFTER_RETRY = "replanned_after_retry"
    NO_ROUTE_HOLDING = "no_route_holding"
    REPLAN_FAILED_HOLDING = "replan_failed_holding"
    HOLD_UNSAFE = "hold_unsafe"
    STOP_MARGIN_VIOLATION = "stop_margin_violation"
    UNAVOIDABLE_COLLISION = "unavoidable_collision"
    NOMINAL_PLAN_FAILED = "nominal_plan_failed"


GOAL_REACHED = {Outcome.NO_CONFLICT, Outcome.REPLANNED, Outcome.REPLANNED_AFTER_RETRY}


@dataclass
class MotionState:
    t: float
    position: np.ndarray
    velocity: np.ndarray
    acceleration: np.ndarray


@dataclass
class ObstacleUpdate:
    name: str
    t_appear: float
    obstacles: list


@dataclass
class SupervisorConfig:
    detection_latency_s: float = 0.2
    decision_latency_s: float = 0.1
    resolution: float = 0.2
    clearance_cap: float = 3.0
    replan_attempts: list = field(default_factory=lambda: [{"planner": "astar"}])


def updated_scene(scene: Scene, update: ObstacleUpdate) -> Scene:
    return Scene(f"{scene.name}+{update.name}", scene.description, scene.lo, scene.hi,
                 list(scene.obstacles) + list(update.obstacles), [], list(scene.unknown_regions))


def _arc_until(traj: PiecewisePolynomial, t0: float, pred, dt: float = 0.005):
    """Arc length (sampled, informative only) from t0 to the first time pred(p) holds."""
    if traj.n_pieces == 0:
        return 0.0 if pred(traj.hold_point[None])[0] else None
    ts = np.arange(t0, traj.t_end + dt, dt)
    p = traj.evaluate(ts)
    hit = np.flatnonzero(pred(p))
    if len(hit) == 0:
        return None
    seg = np.linalg.norm(np.diff(p[: hit[0] + 1], axis=0), axis=1)
    return float(seg.sum())


class ReplanningSupervisor:
    def __init__(self, scene: Scene, policy: ClearancePolicy, limits: TrajectoryLimits,
                 cfg: SupervisorConfig) -> None:
        self.scene, self.policy, self.limits, self.cfg = scene, policy, limits, cfg
        self.grid, self.checker = trajectory_checker(scene, cfg.resolution, policy, cfg.clearance_cap)

    def plan_nominal(self, start, goal, spec: dict):
        return run_pipeline(self.scene, self.checker, start, goal, spec, self.limits, self.policy)

    def handle_update(self, state: MotionState, traj: PiecewisePolynomial, goal,
                      update: ObstacleUpdate) -> dict:
        """Full protocol: assess_update, then (after a predicted safe stop) replan_from_rest."""
        rep = self.assess_update(state, traj, update)
        if "outcome" in rep:
            return rep
        rep.update(self.replan_from_rest(np.asarray(rep["stop_point"]), goal, update, rep["t_stop"],
                                         tuple(rep["stop_min_clearance"])))
        return rep

    def assess_update(self, state: MotionState, traj: PiecewisePolynomial, update: ObstacleUpdate) -> dict:
        """Steps 1-2: conflict check, predicted emergency stop and its classification.

        Terminal results set rep["outcome"] (NO_CONFLICT, HOLD_UNSAFE, UNAVOIDABLE_COLLISION,
        STOP_MARGIN_VIOLATION). Otherwise rep["safe_stop"] is True and no outcome is set yet.
        """
        pol, cfg, lim = self.policy, self.cfg, self.limits
        c_traj, c_nom, rad = pol.trajectory, pol.nominal, pol.robot_radius
        a_dec = lim.max_deceleration
        new_scene = updated_scene(self.scene, update)

        def new_dist(p):
            p = np.atleast_2d(p)
            return np.min([ob.distance(p) for ob in update.obstacles], axis=0)

        rep: dict = {"update": update.name, "t_appear": update.t_appear, "t_detect": state.t,
                     "speed_at_appear": float(np.linalg.norm(traj.evaluate(update.t_appear, 1)[0])),
                     "speed_at_detect": float(np.linalg.norm(state.velocity))}
        lat = cfg.detection_latency_s + cfg.decision_latency_s
        rep["stopping_distance_baseline_m"] = stopping_distance(rep["speed_at_appear"], a_dec, lat)
        rep["distance_to_contact_m"] = _arc_until(traj, update.t_appear, lambda p: new_dist(p) < rad)
        rep["distance_to_conflict_m"] = _arc_until(traj, update.t_appear, lambda p: new_dist(p) < c_traj)

        # 1) conflict check of the remaining trajectory (continuous-time certificate)
        tic = time.perf_counter()
        if traj.n_pieces == 0:
            conflict = bool(new_dist(traj.hold_point)[0] < c_traj)
        else:
            conflict = any(not certify_curve(P, new_dist, c_traj)
                           for P in traj.bernstein_window(state.t, traj.t_end))
        rep["conflict_check_s"] = time.perf_counter() - tic
        rep["conflict_check_within_decision_budget"] = rep["conflict_check_s"] <= cfg.decision_latency_s
        rep["conflict"] = conflict

        if not conflict:
            end = traj.hold_point if traj.n_pieces == 0 else traj.evaluate(traj.t_end)[0]
            lo = hi = float(new_scene.distance(end[None])[0])
            for P in traj.bernstein_window(update.t_appear, traj.t_end):
                a, b = curve_min_bounds(P, new_scene.distance)
                lo, hi = min(lo, a), min(hi, b)
            rep.update(outcome=Outcome.NO_CONFLICT, post_update_min_clearance=[lo, hi],
                       goal_reached=True, mission_end_t=traj.t_end)
            return rep

        # 2) emergency stop after the decision latency
        t_b = min(state.t + cfg.decision_latency_s, traj.t_end)
        p_b = traj.evaluate(t_b, 0)[0]
        v_b = traj.evaluate(t_b, 1)[0]
        brake = braking_trajectory(p_b, v_b, a_dec, t_b)
        p_stop = brake.evaluate(brake.t_end)[0]
        pieces = traj.bernstein_window(update.t_appear, t_b) + \
            [brake.bernstein(i) for i in range(brake.n_pieces)]
        lo = hi = float(new_scene.distance(p_stop[None])[0])
        for P in pieces:
            a, b = curve_min_bounds(P, new_scene.distance)
            lo, hi = min(lo, a), min(hi, b)
        latency_pts = traj.evaluate(np.linspace(update.t_appear, t_b, 400))
        travel = float(np.linalg.norm(np.diff(latency_pts, axis=0), axis=1).sum() + np.linalg.norm(p_stop - p_b))
        hold_clr = float(new_scene.distance(p_stop[None])[0])
        rep.update(t_brake=t_b, speed_at_brake=float(np.linalg.norm(v_b)),
                   braking_distance_m=float(np.linalg.norm(p_stop - p_b)), t_stop=brake.t_end,
                   stop_point=p_stop.tolist(), travel_after_appear_m=travel,
                   stop_min_clearance=[lo, hi], hold_clearance=hold_clr,
                   hold_clearance_meets_trajectory_tier=hold_clr >= c_traj)
        if rep["speed_at_detect"] < 1e-6 and hold_clr < c_nom:
            rep.update(outcome=Outcome.HOLD_UNSAFE, goal_reached=False)
            return rep
        if hi < rad:
            rep.update(outcome=Outcome.UNAVOIDABLE_COLLISION, goal_reached=False)
            return rep
        if lo < c_nom:
            rep.update(outcome=Outcome.STOP_MARGIN_VIOLATION, goal_reached=False)
            return rep
        rep["safe_stop"] = True
        return rep

    def replan_from_rest(self, p_stop, goal, update: ObstacleUpdate, t_rest: float,
                         stop_clearance: tuple[float, float]) -> dict:
        """Step 3: rebuild the map and run the configured attempts from rest at p_stop.

        `stop_clearance` = certified [lower, upper] clearance of the motion before rest; it is
        merged into post_update_min_clearance. The new trajectory starts at t_rest + compute.
        """
        pol, cfg, lim = self.policy, self.cfg, self.limits
        lo, hi = stop_clearance
        new_scene = updated_scene(self.scene, update)
        rep: dict = {}
        # 3) replan from rest at the stop point on the updated map
        tic = time.perf_counter()
        grid2, ck2 = trajectory_checker(new_scene, cfg.resolution, pol, cfg.clearance_cap)
        rep["map_update_s"] = time.perf_counter() - tic
        compute = rep["map_update_s"]
        attempts = []
        outcome, new_traj = Outcome.REPLAN_FAILED_HOLDING, None
        for k, spec in enumerate(cfg.replan_attempts):
            tic = time.perf_counter()
            res = run_pipeline(new_scene, ck2, p_stop, goal, spec, lim, pol)
            dt = time.perf_counter() - tic
            compute += dt
            r = res.record
            attempts.append({"attempt": k, "planner": spec.get("planner"), "status": res.status,
                             "reason": res.reason, "compute_s": dt,
                             "planning_time_s": r.get("planning_time_s"),
                             "shortcut_time_s": r.get("shortcut", {}).get("time_s"),
                             "generation_time_s": r.get("generation", {}).get("generation_time_s"),
                             "validation_time_s": r.get("validation_time_s"),
                             "path_length_m": r.get("path_length_m"),
                             "simplified_length_m": r.get("shortcut", {}).get("length"),
                             "duration_s": r.get("generation", {}).get("duration"),
                             "rejected_candidates": r.get("generation", {}).get("rejected_candidates"),
                             "validation": r.get("validation")})
            if res.status == "success":
                new_traj = res.trajectory
                outcome = Outcome.REPLANNED if k == 0 else Outcome.REPLANNED_AFTER_RETRY
                break
            if spec.get("planner") == "astar" and r.get("plan_status") == "no_path":
                outcome = Outcome.NO_ROUTE_HOLDING
                break
        rep["attempts"] = attempts
        rep["replan_compute_s"] = compute
        rep["hold_duration_s"] = compute
        rep["outcome"] = outcome
        rep["goal_reached"] = outcome in GOAL_REACHED
        if new_traj is not None:
            t_resume = t_rest + compute
            new_traj.knots = new_traj.knots + t_resume
            v = attempts[-1]["validation"]
            rep.update(t_resume=t_resume, mission_end_t=new_traj.t_end, new_trajectory=new_traj,
                       post_update_min_clearance=[min(lo, v["min_clearance_lower"]),
                                                  min(hi, v["min_clearance_upper"])])
        else:
            rep["post_update_min_clearance"] = [lo, hi]
        return rep


def state_at(traj: PiecewisePolynomial, t: float) -> MotionState:
    return MotionState(t, traj.evaluate(t, 0)[0], traj.evaluate(t, 1)[0], traj.evaluate(t, 2)[0])

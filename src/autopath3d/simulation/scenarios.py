"""Closed-loop runs of the Phase 2 pipelines and of the Phase 2 obstacle-update scenarios.

The scenario definitions (scene, task, obstacle, t_appear, planner attempts, expected outcome)
are imported unchanged from replanning.scenarios. As in Phase 2, nominal missions are planned
with A*, and `ahead` obstacles are placed relative to the *reference* state at t_appear, so the
obstacle geometry is identical to Phase 2. What changes is that the drone is simulated.
"""

from __future__ import annotations

import time

import numpy as np

from ..environments.scenes import get_scene
from ..pipeline import ClearancePolicy, run_pipeline, trajectory_checker
from ..replanning.scenarios import ASTAR, SCENARIOS, Scenario, _ahead_box
from ..replanning.supervisor import ObstacleUpdate, Outcome, ReplanningSupervisor, SupervisorConfig, state_at
from ..trajectories.generator import TrajectoryLimits
from .controller import ControllerConfig
from .dynamics import Disturbance
from .navigator import Navigator, SimConfig, SimResult


def scenario_by_name(name: str) -> Scenario:
    for s in SCENARIOS:
        if s.name == name:
            return s
    raise KeyError(f"unknown scenario {name!r}; available: {[s.name for s in SCENARIOS]}")


def classify(res: SimResult) -> str:
    """Outcome of the simulated run in the Phase 2 vocabulary, from what actually happened."""
    names = res.event_names()
    pred = (res.supervisor or {}).get("outcome")
    pred = getattr(pred, "value", pred)
    if "COLLISION" in names:
        return Outcome.UNAVOIDABLE_COLLISION.value
    if "MISSION_ABORTED" in names:
        ab = res.first("MISSION_ABORTED").data
        c_nom = res.meta["policy"]["nominal"]
        if pred == Outcome.HOLD_UNSAFE.value:        # Phase 2: only when already hovering at detection
            return Outcome.HOLD_UNSAFE.value
        if (ab["flown_clearance"][0] < c_nom or ab["hold_clearance"] < c_nom
                or pred == Outcome.STOP_MARGIN_VIOLATION.value):
            return Outcome.STOP_MARGIN_VIOLATION.value
        return "aborted_" + str(pred)
    if "NO_CONFLICT" in names:
        return Outcome.NO_CONFLICT.value
    if res.replan is not None:
        o = res.replan["outcome"]
        return getattr(o, "value", o)
    return "navigation"


def _sim_objects(cfg: dict):
    sim = SimConfig(**cfg["simulation"])
    ctrl = ControllerConfig(**cfg["controller"])
    return sim, ctrl


def run_navigation(scene_name: str, task_name: str, planner_spec: dict, policy: ClearancePolicy,
                   limits: TrajectoryLimits, cfg: dict, disturbances: list[Disturbance], map_cfg: dict):
    """Phase 2 pipeline (unchanged) -> closed-loop simulation for each disturbance profile."""
    scene = get_scene(scene_name)
    task = scene.task(task_name)
    tic = time.perf_counter()
    _, ck = trajectory_checker(scene, map_cfg["resolution"], policy, map_cfg["clearance_cap"])
    setup = time.perf_counter() - tic
    pipe = run_pipeline(scene, ck, task.start, task.goal, planner_spec, limits, policy)
    sim, ctrl = _sim_objects(cfg)
    runs = []
    if pipe.status == "success":
        for dist in disturbances:
            nav = Navigator(scene, policy, limits, sim, ctrl, dist)
            res = nav.run(pipe.trajectory, task.goal, label=f"{scene_name}/{task_name}/{planner_spec['planner']}")
            res.meta.update(kind="navigation", task=task_name, planner=planner_spec["planner"],
                            start=list(task.start), map_setup_s=setup,
                            pipeline=_pipeline_summary(pipe), reference_path=pipe.path.tolist(),
                            reference_simplified=pipe.simplified.tolist())
            runs.append(res)
    return pipe, runs


def _pipeline_summary(pipe) -> dict:
    r = pipe.record
    v = r.get("validation", {})
    return {"status": pipe.status, "reason": pipe.reason, "planner": r.get("planner"),
            "planning_time_s": r.get("planning_time_s"), "path_length_m": r.get("path_length_m"),
            "path_waypoints": r.get("path_waypoints"),
            "shortcut_time_s": r.get("shortcut", {}).get("time_s"),
            "simplified_length_m": r.get("shortcut", {}).get("length"),
            "simplified_waypoints": r.get("shortcut", {}).get("waypoints"),
            "generation_time_s": r.get("generation", {}).get("generation_time_s"),
            "duration_s": r.get("generation", {}).get("duration"),
            "validation_time_s": r.get("validation_time_s"),
            "ref_max_speed_upper": v.get("max_speed_upper"), "ref_max_accel_upper": v.get("max_accel_upper"),
            "ref_min_clearance": [v.get("min_clearance_lower"), v.get("min_clearance_upper")]}


class ScenarioRunner:
    """Builds the supervisor and nominal mission once; simulates it under several disturbances."""

    def __init__(self, sc: Scenario, policy: ClearancePolicy, limits: TrajectoryLimits,
                 base_cfg: SupervisorConfig, cfg: dict) -> None:
        self.sc, self.policy, self.limits, self.cfg = sc, policy, limits, cfg
        self.scene = get_scene(sc.scene)
        self.task = self.scene.task(sc.task)
        self.sup_cfg = SupervisorConfig(base_cfg.detection_latency_s, base_cfg.decision_latency_s,
                                        base_cfg.resolution, base_cfg.clearance_cap, list(sc.attempts))
        tic = time.perf_counter()
        self.sup = ReplanningSupervisor(self.scene, policy, limits, self.sup_cfg)
        self.setup_s = time.perf_counter() - tic
        self.nominal = self.sup.plan_nominal(self.task.start, self.task.goal, ASTAR)
        if self.nominal.status != "success":
            raise RuntimeError(f"{sc.name}: nominal plan failed ({self.nominal.reason})")
        tr = self.nominal.trajectory
        obstacles = list(sc.obstacles)
        if sc.ahead is not None:
            st = state_at(tr, sc.t_appear)
            obstacles.append(_ahead_box(st.position, st.velocity, sc.ahead["distance"]))
        self.update = ObstacleUpdate(sc.name, sc.t_appear, obstacles)

    def run(self, dist: Disturbance, planning_latency_s: float | None = None) -> SimResult:
        sim, ctrl = _sim_objects(self.cfg)
        nav = Navigator(self.scene, self.policy, self.limits, sim, ctrl, dist, self.sup)
        res = nav.run(self.nominal.trajectory, self.task.goal, self.update, planning_latency_s,
                      label=self.sc.name)
        outcome = classify(res)
        pred = (res.supervisor or {}).get("outcome", "safe_stop")
        pred = getattr(pred, "value", pred)
        res.meta.update(kind="scenario", scenario=self.sc.name, description=self.sc.description,
                        task=self.sc.task, start=list(self.task.start), supervisor_setup_s=self.setup_s,
                        expected=[e.value for e in self.sc.expected], outcome=outcome,
                        supervisor_prediction=pred,
                        matches_expected=outcome in [e.value for e in self.sc.expected],
                        attempts=[a.get("planner") for a in self.sc.attempts],
                        pipeline=_pipeline_summary(self.nominal),
                        reference_path=self.nominal.path.tolist(),
                        reference_simplified=self.nominal.simplified.tolist())
        return res

"""Closed-loop navigation executive for the simulated drone.

The drone state evolves only through the double-integrator model (dynamics.py) driven by
the bounded controller (controller.py); it is never placed on the reference.

Timeline (integer control steps k, t_k = k dt; all latencies are multiples of dt):
  FOLLOWING          track the validated reference trajectory
  t_appear           new obstacle exists in the world (safety monitor uses it from now on)
  t_detect = t_appear + detection latency
                     OBSTACLE_DETECTED: the Phase 2 supervisor assesses the update from the
                     *actual* simulated state (conflict check, predicted stop, classification)
  t_brake = t_detect + decision latency   (only if in conflict)
  BRAKING            the actual emergency stop: a = -a_dec v/|v| on the true state
  HOLDING            at rest: PD hold at the actual stop point; the hold is verified against
                     the updated ground truth (hold clearance and the certified clearance of
                     everything flown since t_appear must be >= nominal clearance)
  REPLANNING         the supervisor rebuilds the map and runs its planner attempts from the
                     actual hold point. Simulated time advances by the measured wall-clock
                     compute (rounded up to dt) while the drone keeps holding.
  FOLLOWING_REPLAN   tracking the validated replacement trajectory (event RESUMING)
  terminal           GOAL_REACHED | HOLDING_ABORTED (no route / unsafe stop) | COLLIDED |
                     TIMEOUT
The safety monitor runs on every step against the true geometry present at that time.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field

import numpy as np

from ..replanning.supervisor import MotionState, ObstacleUpdate, Outcome, ReplanningSupervisor
from ..trajectories.polynomial import PiecewisePolynomial
from .controller import ControllerConfig, TrackingController
from .dynamics import IDEAL, Disturbance, step_control_points, zoh_step
from .safety import WorldDistance, refine_min, step_bounds

MODES = ["FOLLOWING", "BRAKING", "HOLDING", "REPLANNING", "FOLLOWING_REPLAN",
         "GOAL_REACHED", "HOLDING_ABORTED", "COLLIDED", "TIMEOUT"]
M = {m: i for i, m in enumerate(MODES)}
TRACKING_MODES = (M["FOLLOWING"], M["FOLLOWING_REPLAN"])
HOLD_MODES = (M["HOLDING"], M["REPLANNING"], M["HOLDING_ABORTED"])
UNSAFE_PREDICTIONS = (Outcome.UNAVOIDABLE_COLLISION, Outcome.STOP_MARGIN_VIOLATION, Outcome.HOLD_UNSAFE)


@dataclass(frozen=True)
class SimConfig:
    dt: float = 0.01                 # s, control and integration period (100 Hz)
    goal_position_tol: float = 0.10  # m
    goal_speed_tol: float = 0.10     # m/s
    settle_timeout_s: float = 5.0    # s after the reference ends
    hold_speed_tol: float = 0.05     # m/s, at-rest test before the hold is verified
    post_abort_hold_s: float = 3.0   # s simulated after an aborted mission (holding)
    max_sim_time_s: float = 600.0


@dataclass
class Event:
    k: int
    t: float
    name: str
    level: str          # INFO (nominal), WARN (contingency handled), ERROR (safety failure)
    message: str
    data: dict = field(default_factory=dict)

    def to_dict(self) -> dict:
        return {"k": self.k, "t": self.t, "name": self.name, "level": self.level,
                "message": self.message, "data": jsonable(self.data)}


@dataclass
class SimResult:
    meta: dict
    arrays: dict                      # see Navigator.run
    events: list
    trajectories: list                # [{"id", "label", "knots", "coeffs"}]
    supervisor: dict | None = None    # assess_update report (predictions)
    replan: dict | None = None        # replan_from_rest report
    metrics: dict = field(default_factory=dict)

    def event_names(self) -> list[str]:
        return [e.name for e in self.events]

    def first(self, name: str) -> Event | None:
        return next((e for e in self.events if e.name == name), None)


def jsonable(o):
    if isinstance(o, dict):
        return {str(k): jsonable(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [jsonable(x) for x in o]
    if isinstance(o, np.ndarray):
        return jsonable(o.tolist())
    if isinstance(o, Outcome):
        return o.value
    if isinstance(o, (np.floating,)):
        return float(o)
    if isinstance(o, (np.integer,)):
        return int(o)
    if isinstance(o, (np.bool_,)):
        return bool(o)
    if isinstance(o, float) and not math.isfinite(o):
        return None if math.isnan(o) else ("inf" if o > 0 else "-inf")
    if isinstance(o, PiecewisePolynomial):
        return o.to_dict()
    return o


def _steps(seconds: float, dt: float) -> int:
    n = seconds / dt
    k = int(round(n))
    if abs(n - k) > 1e-6:
        raise ValueError(f"{seconds} s is not a multiple of the control period {dt} s")
    return k


def _ref_arrays(traj: PiecewisePolynomial, k0: int, n: int, dt: float):
    t = (k0 + np.arange(n)) * dt
    return traj.evaluate(t, 0), traj.evaluate(t, 1), traj.evaluate(t + 0.5 * dt, 2)


class Navigator:
    def __init__(self, scene, policy, limits, sim: SimConfig = SimConfig(),
                 ctrl: ControllerConfig = ControllerConfig(), disturbance: Disturbance = IDEAL,
                 supervisor: ReplanningSupervisor | None = None) -> None:
        self.scene, self.policy, self.limits = scene, policy, limits
        self.sim, self.ctrl_cfg, self.disturbance, self.sup = sim, ctrl, disturbance, supervisor
        self.ctrl = TrackingController(ctrl)
        if ctrl.a_brake_max != limits.max_deceleration:
            raise ValueError("controller braking capability must equal the supervisor's max_deceleration")

    def run(self, traj: PiecewisePolynomial, goal, update: ObstacleUpdate | None = None,
            planning_latency_s: float | None = None, label: str = "") -> SimResult:
        """Simulate the mission. `planning_latency_s` replaces the measured replanning compute
        time in the simulated timeline (for bit-reproducible tests); default: measured."""
        sim, pol, dist_fn = self.sim, self.policy, self.disturbance
        dt = sim.dt
        rad, c_nom, allow = pol.robot_radius, pol.nominal, pol.tracking_allowance
        goal = np.asarray(goal, float)
        if update is not None and self.sup is None:
            raise ValueError("an obstacle update requires a supervisor")
        if abs(traj.t_start) > 1e-12:
            raise ValueError("the mission trajectory must start at t = 0")
        if np.linalg.norm(traj.evaluate(traj.t_start, 1)[0]) > 1e-9:
            raise ValueError("the mission must start at rest")
        wall0 = time.perf_counter()
        t_safety = 0.0

        world = WorldDistance.of_scene(self.scene)
        world_id = 0
        if update is not None:
            upd_world = WorldDistance(self.scene.lo, self.scene.hi, list(self.scene.obstacles) + list(update.obstacles))
            k_appear = _steps(update.t_appear, dt)
            k_detect = k_appear + _steps(self.sup.cfg.detection_latency_s, dt)
            k_dec = _steps(self.sup.cfg.decision_latency_s, dt)
        else:
            upd_world, k_appear, k_detect, k_dec = None, -1, -1, 0
        k_brake = -1

        settle = int(math.ceil(sim.settle_timeout_s / dt)) + 2

        def load_ref(tr: PiecewisePolynomial, k0: int):
            n = int(math.ceil((tr.t_end - k0 * dt) / dt)) + settle
            return _ref_arrays(tr, k0, max(n, 1), dt)

        ref, ref_id, k_ref0 = traj, 0, 0
        RP, RV, RA = load_ref(traj, 0)
        trajectories = [{"id": 0, "label": "original", **traj.to_dict()}]

        p = traj.evaluate(0.0)[0].copy()
        v = np.zeros(3)
        a_last = np.zeros(3)
        mode = M["FOLLOWING"]
        hold_point = None
        events: list[Event] = []

        def ev(k, name, level, msg, **data):
            events.append(Event(k, round(k * dt, 9), name, level, msg, data))

        L = {key: [] for key in ("p", "v", "a_cmd", "a_dist", "mode", "ref_id", "p_ref", "v_ref", "a_ref",
                                 "hold", "clr_lo", "clr_hi", "world", "saturated", "governed")}
        nan3 = np.full(3, np.nan)
        assess = replan = None
        replan_wall = 0.0
        schedule: dict[int, list] = {}
        hold_checked = False
        flag_track = flag_margin = False
        k_stop = k_end = -1
        brake_done = False
        end_reason = None
        k = 0
        ev(0, "MISSION_START", "INFO", f"following the validated reference ({traj.duration:.2f} s)",
           start=p, goal=goal)

        while True:
            # ---------------------------------------------------------------- world / supervisor
            if k == k_appear:
                world, world_id = upd_world, 1
                ev(k, "OBSTACLE_APPEARED", "INFO",
                   f"{len(update.obstacles)} new obstacle(s) exist in the world; not yet known to the drone",
                   obstacles=[o.name for o in update.obstacles])
            if k == k_detect and mode in TRACKING_MODES:
                st = MotionState(k * dt, p.copy(), v.copy(), a_last.copy())
                tic = time.perf_counter()
                assess = self.sup.assess_update(st, ref, update)
                assess["assess_wall_s"] = time.perf_counter() - tic
                ev(k, "OBSTACLE_DETECTED", "WARN",
                   f"map update received {self.sup.cfg.detection_latency_s:.2f} s after appearance; "
                   f"speed {np.linalg.norm(v):.2f} m/s", speed=float(np.linalg.norm(v)))
                if assess.get("outcome") == Outcome.NO_CONFLICT:
                    ev(k, "NO_CONFLICT", "INFO", "remaining trajectory certified clear of the new obstacles")
                else:
                    k_brake = k + k_dec
                    pred = assess.get("outcome", "safe_stop")
                    ev(k, "CONFLICT", "WARN",
                       f"remaining trajectory violates {pol.trajectory:.2f} m clearance; supervisor "
                       f"predicts: {getattr(pred, 'value', pred)} (stopping distance "
                       f"{assess['stopping_distance_baseline_m']:.2f} m)",
                       predicted=getattr(pred, "value", pred))
            if k == k_brake:
                mode = M["BRAKING"]
                ev(k, "BRAKING", "WARN",
                   f"emergency stop at {self.ctrl_cfg.a_brake_max:.1f} m/s^2 from {np.linalg.norm(v):.2f} m/s",
                   speed=float(np.linalg.norm(v)))

            # ---------------------------------------------------------------- control
            sat = gov = False
            done = False
            if mode in TRACKING_MODES:
                i = min(k - k_ref0, len(RP) - 1)
                pr, vr, ar = RP[i], RV[i], RA[i]
                a, sat, gov = self.ctrl.track(p, v, pr, vr, ar, dt)
            elif mode == M["BRAKING"]:
                pr = vr = ar = nan3
                a, done = self.ctrl.brake(v, dt)
            else:
                pr = vr = ar = nan3
                a, sat, gov = self.ctrl.hold(p, v, hold_point, dt)
            d = dist_fn(k * dt + 0.5 * dt)
            p1, v1 = zoh_step(p, v, a + d, dt)

            # ---------------------------------------------------------------- safety monitor
            ts = time.perf_counter()
            P = step_control_points(p, v, p1, dt)[None]
            lo, hi = step_bounds(P, world)
            lo, hi = float(lo[0]), float(hi[0])
            if lo < c_nom:
                lo, hi = refine_min(P, world)
            t_safety += time.perf_counter() - ts
            for key, val in (("p", p), ("v", v), ("a_cmd", a), ("a_dist", d), ("mode", mode),
                             ("ref_id", ref_id if mode in TRACKING_MODES else -1), ("p_ref", pr),
                             ("v_ref", vr), ("a_ref", ar), ("hold", hold_point if hold_point is not None
                                                            and mode in HOLD_MODES else nan3),
                             ("clr_lo", lo), ("clr_hi", hi), ("world", world_id), ("saturated", sat),
                             ("governed", gov)):
                L[key].append(np.array(val, copy=True) if isinstance(val, np.ndarray) else val)
            if lo < rad:
                spd = float(np.linalg.norm(v))
                ev(k, "COLLISION", "ERROR",
                   f"robot sphere (r = {rad:.2f} m) contacts an obstacle in [{k * dt:.2f}, {(k + 1) * dt:.2f}] s "
                   f"at {spd:.2f} m/s; certified clearance [{lo:.4f}, {hi:.4f}] m, penetration up to "
                   f"{1000 * (rad - lo):.1f} mm; run terminated (no contact dynamics modelled)",
                   clearance=[lo, hi], speed=spd, penetration_m=rad - lo)
                k += 1
                p, v = p1, v1
                mode, end_reason = M["COLLIDED"], "collision"
                break
            if lo < c_nom and not flag_margin:
                flag_margin = True
                ev(k, "SAFETY_VIOLATION", "ERROR",
                   f"clearance not certified >= {c_nom:.2f} m (certified [{lo:.3f}, {hi:.3f}] m)",
                   clearance=[lo, hi])
            if mode in TRACKING_MODES and not flag_track:
                e = float(np.linalg.norm(p - pr))
                if e > allow:
                    flag_track = True
                    ev(k, "TRACKING_ALLOWANCE_EXCEEDED", "ERROR",
                       f"position error {e:.3f} m exceeds the {allow:.2f} m tracking allowance", error=e)

            k += 1
            a_last = a + d
            p, v = p1, v1
            t = k * dt

            # ---------------------------------------------------------------- transitions at t_k
            if mode == M["BRAKING"] and done:
                mode, hold_point, k_stop = M["HOLDING"], p.copy(), k
                pred_stop = np.asarray(assess["stop_point"]) if assess and "stop_point" in assess else None
                ev(k, "STOPPED", "WARN",
                   "at rest" + ("" if pred_stop is None else
                                f"; {np.linalg.norm(p - pred_stop):.3f} m from the supervisor's predicted stop point"),
                   stop_point=p, predicted_stop_point=pred_stop)
            if mode == M["HOLDING"] and not hold_checked and np.linalg.norm(v) <= sim.hold_speed_tol:
                hold_checked = True
                mode, replan, replan_wall = self._verify_hold_and_replan(
                    k, hold_point, goal, update, assess, L, k_appear, upd_world, planning_latency_s, schedule, ev)
                if mode == M["HOLDING_ABORTED"]:
                    k_end = k + int(round(sim.post_abort_hold_s / dt))
            for item in schedule.pop(k, []):
                action = item(k)
                if action is None:
                    continue
                if action[0] == "resume":
                    new_traj = action[1]
                    ref, ref_id, k_ref0 = new_traj, 1, k
                    RP, RV, RA = load_ref(new_traj, k)
                    trajectories.append({"id": 1, "label": "replanned", **new_traj.to_dict()})
                    mode = M["FOLLOWING_REPLAN"]
                else:                                   # ("abort",): no route, keep holding
                    mode = M["HOLDING_ABORTED"]
                    k_end = k + int(round(sim.post_abort_hold_s / dt))
            if mode in TRACKING_MODES and t >= ref.t_end - 1e-9:
                if np.linalg.norm(p - goal) <= sim.goal_position_tol and np.linalg.norm(v) <= sim.goal_speed_tol:
                    ev(k, "GOAL_REACHED", "INFO",
                       f"within {sim.goal_position_tol:.2f} m / {sim.goal_speed_tol:.2f} m/s of the goal "
                       f"(error {np.linalg.norm(p - goal):.3f} m)", error=float(np.linalg.norm(p - goal)))
                    mode, end_reason = M["GOAL_REACHED"], "goal_reached"
                    break
                if t > ref.t_end + sim.settle_timeout_s:
                    ev(k, "GOAL_TIMEOUT", "ERROR", "goal tolerance not reached after the reference ended")
                    mode, end_reason = M["TIMEOUT"], "timeout"
                    break
            if mode == M["HOLDING_ABORTED"] and k >= k_end:
                end_reason = "aborted_holding"
                break
            if t >= sim.max_sim_time_s:
                mode, end_reason = M["TIMEOUT"], "max_sim_time"
                break

        A = {key: np.array(val) for key, val in L.items()}
        A["p"] = np.vstack([A["p"], p])
        A["v"] = np.vstack([A["v"], v])
        A["t"] = np.arange(len(A["p"])) * dt
        A["mode"] = A["mode"].astype(np.int8)
        A["ref_id"] = A["ref_id"].astype(np.int8)
        A["world"] = A["world"].astype(np.int8)
        meta = {"label": label, "scene": self.scene.name, "dt": dt, "goal": goal, "end_reason": end_reason,
                "final_mode": MODES[mode], "k_appear": k_appear, "k_detect": k_detect, "k_brake": k_brake,
                "k_stop": k_stop, "disturbance": dist_fn.name, "sim_wall_s": time.perf_counter() - wall0,
                "safety_monitor_wall_s": t_safety, "planning_latency_override_s": planning_latency_s,
                "policy": {"robot_radius": rad, "nominal": c_nom, "trajectory": pol.trajectory,
                           "tracking_allowance": allow},
                "controller": self.ctrl_cfg.__dict__, "sim_config": sim.__dict__,
                "update": None if update is None else
                {"name": update.name, "t_appear": update.t_appear,
                 "obstacles": [{"name": o.name, "lo": list(o.lo), "hi": list(o.hi)} for o in update.obstacles]}}
        meta["replan_wall_s"] = replan_wall
        res = SimResult(meta, A, events, trajectories, assess, replan)
        from .metrics import compute_metrics          # local import: metrics imports this module
        worlds = {0: WorldDistance.of_scene(self.scene)}
        if upd_world is not None:
            worlds[1] = upd_world
        res.metrics = compute_metrics(res, worlds, self.limits)
        return res

    def _verify_hold_and_replan(self, k, hold_point, goal, update, assess, L, k_appear, upd_world,
                                planning_latency_s, schedule, ev):
        """Verify the actual hold; if safe, replan from it and schedule the result.

        Returns (mode, replan report or None, replan wall-clock seconds)."""
        pol, dt = self.policy, self.sim.dt
        c_nom = pol.nominal
        hold_clr = float(upd_world(hold_point)[0])
        # certified clearance of everything flown since t_appear (updated world), from the log
        p = np.array(L["p"][k_appear:] + [hold_point])
        v = np.array(L["v"][k_appear:])
        P = step_control_points(p[:-1], v, p[1:], dt)
        lo, hi = refine_min(P, upd_world)
        pred = assess.get("outcome") if assess else None
        data = dict(hold_point=hold_point, hold_clearance=hold_clr, flown_clearance=[lo, hi],
                    predicted=getattr(pred, "value", pred))
        if pred in UNSAFE_PREDICTIONS or lo < c_nom or hold_clr < c_nom:
            why = []
            if pred in UNSAFE_PREDICTIONS:
                why.append(f"supervisor predicted {pred.value}")
            if lo < c_nom:
                why.append(f"executed stop clearance {max(lo, 0):.3f} m < {c_nom:.2f} m")
            if hold_clr < c_nom:
                why.append(f"hold clearance {hold_clr:.3f} m < {c_nom:.2f} m")
            ev(k, "MISSION_ABORTED", "ERROR", "unsafe stop, no replanning: " + "; ".join(why), **data)
            return M["HOLDING_ABORTED"], None, 0.0
        ev(k, "HOLD_VERIFIED", "INFO",
           f"hold point clearance {hold_clr:.3f} m, executed stop clearance >= {lo:.3f} m (>= {c_nom:.2f} m)", **data)
        tic = time.perf_counter()
        rep = self.sup.replan_from_rest(hold_point, goal, update, k * dt, (lo, hi))
        wall = time.perf_counter() - tic
        new_traj = rep.pop("new_trajectory", None)
        total = rep["replan_compute_s"]
        scale = 1.0 if planning_latency_s is None else planning_latency_s / max(total, 1e-12)
        sim_total = total * scale
        k_done = k + max(1, int(math.ceil(sim_total / dt - 1e-9)))
        rep.update(t_hold=k * dt, sim_planning_s=(k_done - k) * dt, measured_compute_s=total,
                   t_result=k_done * dt)
        ev(k, "REPLANNING", "WARN",
           f"rebuilding the voxel map and planning from the hold point ({len(self.sup.cfg.replan_attempts)} "
           f"attempt(s) configured); measured compute {total:.2f} s wall-clock -> simulated hold "
           f"{(k_done - k) * dt:.2f} s", measured_compute_s=total)
        cum = rep["map_update_s"]
        for a in rep["attempts"]:
            cum += a["compute_s"]
            ka = min(k + max(1, int(math.ceil(cum * scale / dt - 1e-9))), k_done)
            if a["status"] != "success":
                schedule.setdefault(ka, []).append(
                    lambda kk, a=a: ev(kk, "REPLAN_ATTEMPT_FAILED", "WARN",
                                       f"attempt {a['attempt'] + 1} ({a['planner']}): {a['status']} "
                                       f"({a['reason'] or 'no detail'}); compute {a['compute_s']:.2f} s",
                                       attempt=a["attempt"], planner=a["planner"]))
        outcome = rep["outcome"]

        def finish(kk):
            if new_traj is not None:
                last = rep["attempts"][-1]
                t0 = kk * dt
                new_traj.knots = new_traj.knots - new_traj.knots[0] + t0
                rep["t_resume_sim"] = t0
                ev(kk, "REPLAN_SUCCEEDED", "INFO",
                   f"{last['planner']} route validated (length {last['simplified_length_m']:.2f} m, "
                   f"duration {last['duration_s']:.2f} s, clearance >= "
                   f"{last['validation']['min_clearance_lower']:.3f} m)")
                ev(kk, "RESUMING", "INFO", "following the replacement trajectory")
                return ("resume", new_traj)
            name = "NO_ROUTE" if outcome == Outcome.NO_ROUTE_HOLDING else "REPLAN_FAILED"
            ev(kk, name, "WARN",
               ("A* exhausted its open set: no route exists on the updated map; holding safely"
                if name == "NO_ROUTE" else "all planner attempts failed; holding safely"))
            return ("abort",)
        schedule.setdefault(k_done, []).append(finish)
        return M["REPLANNING"], rep, wall

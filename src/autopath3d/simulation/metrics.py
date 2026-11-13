"""Tracking, limit, clearance and outcome metrics computed from a SimResult's logs.

Definitions (all measured on the simulated state, never assumed):
  position / velocity tracking error   |p - p_ref|, |v - v_ref| at the step start times
                                       t_k while tracking a reference (FOLLOWING modes)
  RMS                                  sqrt(mean(e^2)) over those steps
  speed                                |v| at every logged state
  commanded acceleration               |a_cmd| over each ZOH step (controller output)
  net acceleration                     |a_cmd + d| (includes the wind disturbance)
  clearance                            certified [lower, upper] on the minimum ground-truth
                                       distance of the robot centre over the executed
                                       piecewise-quadratic path (true world at each time)
Safety test (a run fails if any item fails):
  no collision (clearance >= robot radius), clearance certified >= nominal clearance,
  tracking error <= tracking allowance while tracking, speed <= v_max,
  |a_cmd| <= nominal limit while tracking/holding and <= braking capability while braking.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

from .dynamics import step_control_points
from .navigator import HOLD_MODES, M, MODES, TRACKING_MODES, Event, SimResult, jsonable
from .safety import WorldDistance, refine_min


def _stats(x: np.ndarray) -> dict:
    if len(x) == 0:
        return {"max": None, "rms": None, "mean": None, "p95": None}
    return {"max": float(x.max()), "rms": float(math.sqrt(np.mean(x * x))), "mean": float(x.mean()),
            "p95": float(np.percentile(x, 95))}


def executed_clearance(res: SimResult, worlds: dict[int, WorldDistance]) -> dict:
    """Certified clearance of the whole executed path, each step against its active world."""
    A, dt = res.arrays, res.meta["dt"]
    P = step_control_points(A["p"][:-1], A["v"][:-1], A["p"][1:], dt)
    out = {}
    lo_all, hi_all = math.inf, math.inf
    for wid, world in worlds.items():
        sel = A["world"] == wid
        if not sel.any():
            continue
        lo, hi = refine_min(P[sel], world)
        out[{0: "before_update", 1: "after_update"}[wid]] = [lo, hi]
        lo_all, hi_all = min(lo_all, lo), min(hi_all, hi)
    k_min = int(np.argmin(A["clr_lo"]))
    out.update(lower=lo_all, upper=hi_all, t_min=float(k_min * dt), p_min=A["p"][k_min].tolist())
    return out


def compute_metrics(res: SimResult, worlds: dict[int, WorldDistance], limits) -> dict:
    A, meta = res.arrays, res.meta
    pol, ctrl = meta["policy"], meta["controller"]
    mode = A["mode"]
    p, v = A["p"][:-1], A["v"][:-1]
    trk = np.isin(mode, TRACKING_MODES)
    ep = np.linalg.norm(p - A["p_ref"], axis=1)[trk]
    evel = np.linalg.norm(v - A["v_ref"], axis=1)[trk]
    per_ref = {}
    for rid, name in ((0, "original"), (1, "replanned")):
        s = trk & (A["ref_id"] == rid)
        if s.any():
            per_ref[name] = {"position_error": _stats(np.linalg.norm(p - A["p_ref"], axis=1)[s]),
                             "velocity_error": _stats(np.linalg.norm(v - A["v_ref"], axis=1)[s]),
                             "steps": int(s.sum())}
    speed = np.linalg.norm(A["v"], axis=1)
    acmd = np.linalg.norm(A["a_cmd"], axis=1)
    anet = np.linalg.norm(A["a_cmd"] + A["a_dist"], axis=1)
    brk = mode == M["BRAKING"]
    nb = ~brk
    hold = np.isin(mode, HOLD_MODES)
    drift = np.linalg.norm(p - A["hold"], axis=1)[hold]
    clr = executed_clearance(res, worlds)
    goal = np.asarray(meta["goal"], float)
    names = res.event_names()
    collision = "COLLISION" in names
    max_ep = float(ep.max()) if len(ep) else 0.0
    checks = {
        "no_collision": not collision and clr["lower"] >= pol["robot_radius"],
        "clearance_ge_nominal": clr["lower"] >= pol["nominal"],
        "tracking_within_allowance": max_ep <= pol["tracking_allowance"],
        "speed_within_limit": float(speed.max()) <= limits.max_speed,
        "accel_within_nominal_limit": bool((acmd[nb] <= ctrl["a_track_max"] + 1e-9).all()),
        "braking_within_capability": bool((acmd[brk] <= ctrl["a_brake_max"] + 1e-9).all()),
    }
    goal_reached = "GOAL_REACHED" in names
    ge = next((e for e in res.events if e.name == "GOAL_REACHED"), None)
    t_ref_end = None
    if res.trajectories:
        t_ref_end = float(res.trajectories[-1]["knots"][-1])
    k_end_ref = None if t_ref_end is None else min(int(round(t_ref_end / meta["dt"])), len(A["p"]) - 1)
    mode_time = {m: float((mode == M[m]).sum() * meta["dt"]) for m in MODES if (mode == M[m]).any()}
    return {
        "goal_reached": goal_reached,
        "safety_pass": all(checks.values()),
        "mission_success": goal_reached and all(checks.values()),
        "checks": checks,
        "collision": collision,
        "final_mode": meta["final_mode"],
        "end_reason": meta["end_reason"],
        "execution_duration_s": float(A["t"][-1]),
        "time_in_mode_s": mode_time,
        "tracking": {"position_error": _stats(ep), "velocity_error": _stats(evel), "per_reference": per_ref,
                     "tracking_steps": int(trk.sum())},
        "max_speed": float(speed.max()),
        "max_reference_speed": float(np.nanmax(np.linalg.norm(A["v_ref"], axis=1))) if trk.any() else None,
        "max_accel_cmd_tracking_hold": float(acmd[nb].max()) if nb.any() else 0.0,
        "max_accel_cmd_braking": float(acmd[brk].max()) if brk.any() else None,
        "max_accel_net": float(anet.max()),
        "saturated_fraction": float(A["saturated"].mean()),
        "governor_steps": int(A["governed"].sum()),
        "hold_drift_max": float(drift.max()) if len(drift) else None,
        "clearance": clr,
        "goal_error_final": float(np.linalg.norm(A["p"][-1] - goal)),
        "goal_error_at_reference_end": (None if k_end_ref is None else
                                        float(np.linalg.norm(A["p"][k_end_ref] - goal))),
        "speed_final": float(speed[-1]),
        "t_goal_reached": None if ge is None else ge.t,
    }


# ---------------------------------------------------------------- persistence
def save_result(res: SimResult, directory, name: str, series: bool = True) -> None:
    d = Path(directory)
    d.mkdir(parents=True, exist_ok=True)
    if series:
        np.savez(d / f"{name}.npz", **res.arrays)
    doc = {"meta": res.meta, "events": [e.to_dict() for e in res.events], "trajectories": res.trajectories,
           "supervisor": res.supervisor, "replan": res.replan, "metrics": res.metrics}
    (d / f"{name}.json").write_text(json.dumps(jsonable(doc), indent=1), encoding="utf-8")


def load_result(directory, name: str) -> SimResult:
    d = Path(directory)
    doc = json.loads((d / f"{name}.json").read_text(encoding="utf-8"))
    with np.load(d / f"{name}.npz") as z:
        arrays = {k: z[k] for k in z.files}
    events = [Event(e["k"], e["t"], e["name"], e["level"], e["message"], e["data"]) for e in doc["events"]]
    return SimResult(doc["meta"], arrays, events, doc["trajectories"], doc["supervisor"], doc["replan"],
                     doc["metrics"])


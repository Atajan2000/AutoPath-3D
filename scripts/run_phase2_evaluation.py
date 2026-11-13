"""Phase 2 evaluation: shortcutting, trajectories, independent validation, replanning.

Usage: python scripts/run_phase2_evaluation.py [--config configs/phase2.yaml] [--out results/phase2]
Writes results.json and summary.md. No figures or media.
"""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

import numpy as np
import yaml

from autopath3d.environments import get_scene
from autopath3d.pipeline import ClearancePolicy, run_pipeline, trajectory_checker
from autopath3d.replanning import SupervisorConfig, stopping_distance
from autopath3d.replanning.scenarios import SCENARIOS, run_scenario
from autopath3d.trajectories import TrajectoryLimits

ROOT = Path(__file__).resolve().parents[1]


def f(x, nd=2):
    if x is None or (isinstance(x, float) and not math.isfinite(x)):
        return "–"
    return f"{x:.{nd}f}" if isinstance(x, float) else str(x)


def flatten(scene, task, label, res) -> dict:
    r = res.record
    s, g, v = r.get("shortcut", {}), r.get("generation", {}), r.get("validation", {})
    return {"scene": scene, "task": task, "source": label, "status": res.status, "reason": res.reason,
            "planning_time_s": r.get("planning_time_s"), "path_length_m": r.get("path_length_m"),
            "path_waypoints": r.get("path_waypoints"), "simplified_length_m": s.get("length"),
            "simplified_waypoints": s.get("waypoints"), "shortcut_time_s": s.get("time_s"),
            "rejected_shortcuts": s.get("rejected_shortcuts"),
            "duration_s": g.get("duration"), "generation_time_s": g.get("generation_time_s"),
            "rejected_candidates": g.get("rejected_candidates"), "fallback_pieces": g.get("fallback_pieces"),
            "timing_adjustments": g.get("timing_adjustments"),
            "validation_time_s": r.get("validation_time_s"), "valid": v.get("valid", False),
            "max_speed_upper": v.get("max_speed_upper"), "max_accel_upper": v.get("max_accel_upper"),
            "min_clearance_lower": v.get("min_clearance_lower"), "min_clearance_upper": v.get("min_clearance_upper"),
            "max_velocity_jump": v.get("max_velocity_jump"), "max_accel_jump": v.get("max_accel_jump"),
            "max_deviation_upper": v.get("max_deviation_upper"), "validation_failures": v.get("failures")}


def part_pipeline(cfg, pol, lim, log):
    out = []
    for name in cfg["scenes"]:
        sc = get_scene(name)
        _, ck = trajectory_checker(sc, cfg["map"]["resolution"], pol, cfg["map"]["clearance_cap"])
        for t in sc.tasks:
            for key in ("astar", "rrt_star"):
                res = run_pipeline(sc, ck, t.start, t.goal, cfg["planners"][key], lim, pol)
                out.append(flatten(name, t.name, key, res))
                log(f"pipeline {name}/{t.name}/{key}: {res.status}")
    return out


def part_phase1_paths(cfg, lim, log):
    runs = json.loads((ROOT / cfg["phase1_runs"]).read_text())["main"]
    runs = [r for r in runs if r["success"]]
    out = []
    for allowance in (0.0, cfg["clearance"]["tracking_allowance"]):
        pol = ClearancePolicy(cfg["clearance"]["robot_radius"], cfg["clearance"]["safety_margin"], allowance)
        cache = {}
        for r in runs:
            if r["scene"] not in cache:
                sc = get_scene(r["scene"])
                cache[r["scene"]] = (sc, trajectory_checker(sc, cfg["map"]["resolution"], pol,
                                                            cfg["map"]["clearance_cap"])[1])
            sc, ck = cache[r["scene"]]
            t = sc.task(r["task"])
            res = run_pipeline(sc, ck, t.start, t.goal, {}, lim, pol, path=np.array(r["path"]))
            row = flatten(r["scene"], r["task"], f"phase1_{r['planner']}_seed{r.get('seed', '-')}", res)
            row.update(tracking_allowance=allowance, required_clearance=pol.trajectory,
                       phase1_gt_clearance=r["gt_min_clearance_m"])
            out.append(row)
        log(f"phase-1 paths, allowance {allowance}: "
            f"{sum(x['status'] == 'success' for x in out if x['tracking_allowance'] == allowance)}/{len(runs)} ok")
    return out


def summarise(cfg, pipe, p1, scen, stop_rows) -> str:
    c = cfg["clearance"]
    L = cfg["limits"]
    lines = ["# Phase 2 results", "",
             f"Robot sphere {c['robot_radius']} m + safety margin {c['safety_margin']} m = nominal clearance "
             f"{c['robot_radius'] + c['safety_margin']:.2f} m; tracking allowance {c['tracking_allowance']} m -> "
             f"trajectory clearance {c['robot_radius'] + c['safety_margin'] + c['tracking_allowance']:.2f} m. "
             f"Limits: v ≤ {L['max_speed']} m/s, |a| ≤ {L['max_acceleration']} m/s², braking {L['max_deceleration']} m/s². "
             "All bounds below are certified intervals from the independent validator "
             "(Bezier hull + 1-Lipschitz branch and bound against ground-truth geometry).", "",
             "## 1. Path → simplified path → trajectory (planned at the 0.45 m trajectory clearance)", "",
             "| scene / task | planner | status | path L [m] / wp | simplified L [m] / wp | duration [s] | v max ≤ [m/s] "
             "| a max ≤ [m/s²] | min clearance [m] | rejected curves | fallback pieces | plan / shortcut / gen / validate [s] |",
             "|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for r in pipe:
        lines.append(
            f"| {r['scene']} / {r['task']} | {r['source']} | {r['status']} | {f(r['path_length_m'])} / {f(r['path_waypoints'])} "
            f"| {f(r['simplified_length_m'])} / {f(r['simplified_waypoints'])} | {f(r['duration_s'])} | {f(r['max_speed_upper'], 3)} "
            f"| {f(r['max_accel_upper'], 3)} | {f(r['min_clearance_lower'], 3)} | {f(r['rejected_candidates'])} | {f(r['fallback_pieces'])} "
            f"| {f(r['planning_time_s'])} / {f(r['shortcut_time_s'], 3)} / {f(r['generation_time_s'], 3)} / {f(r['validation_time_s'], 3)} |")
    ok = [r for r in pipe if r["status"] == "success"]
    lines += ["", f"Pipelines: {len(pipe)}; validated trajectories: {len(ok)}; failures: "
              + ", ".join(f"{r['scene']}/{r['source']}: {r['status']}" for r in pipe if r["status"] != "success") + ".",
              f"Over all validated trajectories: max speed ≤ {max(r['max_speed_upper'] for r in ok):.3f} m/s, "
              f"max |a| ≤ {max(r['max_accel_upper'] for r in ok):.3f} m/s², min clearance ≥ "
              f"{min(r['min_clearance_lower'] for r in ok):.3f} m, max velocity jump {max(r['max_velocity_jump'] for r in ok):.1e} m/s, "
              f"max acceleration jump {max(r['max_accel_jump'] for r in ok):.1e} m/s².", ""]

    lines += ["## 2. Reusing the Phase 1 paths (planned at 0.35 m) without replanning", "",
              "| tracking allowance [m] | required clearance [m] | paths | validated | rejected at shortcut (input clearance too small) | other failures |",
              "|---|---|---|---|---|---|"]
    for a in sorted({r["tracking_allowance"] for r in p1}):
        rows = [r for r in p1 if r["tracking_allowance"] == a]
        lines.append(f"| {a:.2f} | {rows[0]['required_clearance']:.2f} | {len(rows)} | "
                     f"{sum(r['status'] == 'success' for r in rows)} | "
                     f"{sum(r['status'] == 'shortcut_rejected' for r in rows)} | "
                     f"{sum(r['status'] not in ('success', 'shortcut_rejected') for r in rows)} |")
    worst = min((r for r in p1 if r["tracking_allowance"] == 0.0), key=lambda r: r["phase1_gt_clearance"])
    lines += ["", f"The tightest Phase 1 path ({worst['scene']}/{worst['task']}, {worst['source']}, ground-truth "
              f"clearance {worst['phase1_gt_clearance']:.3f} m) → allowance 0: {worst['status']} "
              f"(trajectory clearance ≥ {f(worst['min_clearance_lower'], 3)} m); allowance 0.10: "
              + next(r["status"] for r in p1 if r["tracking_allowance"] > 0 and r["source"] == worst["source"]
                     and r["scene"] == worst["scene"] and r["task"] == worst["task"]) + ".", ""]

    lines += ["## 3. Stopping distance baseline d = v·t_L + v²/(2·a_dec)", "",
              f"t_L = detection {cfg['supervisor']['detection_latency_s']} s + decision "
              f"{cfg['supervisor']['decision_latency_s']} s, a_dec = {L['max_deceleration']} m/s².", "",
              "| speed [m/s] | latency distance [m] | braking distance [m] | total [m] |", "|---|---|---|---|"]
    for s in stop_rows:
        lines.append(f"| {s['speed']:.1f} | {s['latency_m']:.2f} | {s['braking_m']:.2f} | {s['total_m']:.2f} |")

    lines += ["", "## 4. Obstacle-update scenarios", "",
              "| scenario | outcome | expected | speed at appear [m/s] | baseline stop dist [m] | modelled travel to rest [m] "
              "| path dist to contact [m] | stop-manoeuvre clearance [m] | hold clearance [m] | replan compute [s] "
              "| attempts | new traj L [m] / T [s] | post-update min clearance [m] | goal reached |",
              "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for r in scen:
        at = r.get("attempts", [])
        last = at[-1] if at else {}
        sm = r.get("stop_min_clearance")
        pm = r.get("post_update_min_clearance")
        lines.append(
            f"| {r['scenario']} | **{r['outcome']}** | {', '.join(r['expected'])} | {f(r.get('speed_at_appear'))} "
            f"| {f(r.get('stopping_distance_baseline_m'))} | {f(r.get('travel_after_appear_m'))} | {f(r.get('distance_to_contact_m'))} "
            f"| {'–' if sm is None else f'{max(sm[0], 0.0):.3f}'} | {f(r.get('hold_clearance'), 3)} | {f(r.get('replan_compute_s'))} "
            f"| {' → '.join(a['planner'] + ':' + a['status'] for a in at) or '–'} "
            f"| {f(last.get('simplified_length_m'))} / {f(last.get('duration_s'))} "
            f"| {'–' if pm is None else f'{max(pm[0], 0.0):.3f}'} | {r['goal_reached']} |")
    n_ok = sum(r["matches_expected"] for r in scen)
    lines += ["", f"Scenarios matching the expected outcome: {n_ok}/{len(scen)}. Clearances are certified lower "
              "bounds w.r.t. the updated ground-truth geometry (0.000 = contact). Replanning compute includes the "
              "voxel-map rebuild and all pipeline stages; the robot holds at the verified stop point meanwhile."]
    return "\n".join(lines) + "\n"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default=str(ROOT / "configs" / "phase2.yaml"))
    ap.add_argument("--out", default=str(ROOT / "results" / "phase2"))
    args = ap.parse_args()
    cfg = yaml.safe_load(Path(args.config).read_text())
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    c = cfg["clearance"]
    pol = ClearancePolicy(c["robot_radius"], c["safety_margin"], c["tracking_allowance"])
    lim = TrajectoryLimits(**cfg["limits"])
    sup_cfg = SupervisorConfig(cfg["supervisor"]["detection_latency_s"], cfg["supervisor"]["decision_latency_s"],
                               cfg["map"]["resolution"], cfg["map"]["clearance_cap"])
    t0 = time.perf_counter()
    log = lambda m: print(f"[{time.perf_counter() - t0:6.1f}s] {m}", flush=True)

    pipe = part_pipeline(cfg, pol, lim, log)
    p1 = part_phase1_paths(cfg, lim, log)
    lat = sup_cfg.detection_latency_s + sup_cfg.decision_latency_s
    stop_rows = [{"speed": v, "latency_m": v * lat, "braking_m": v * v / (2 * lim.max_deceleration),
                  "total_m": stopping_distance(v, lim.max_deceleration, lat)} for v in cfg["stopping_table"]["speeds"]]
    scen = []
    for s in SCENARIOS:
        r = run_scenario(s, pol, lim, sup_cfg)
        scen.append(r)
        log(f"scenario {s.name}: {r['outcome']} (expected {r['expected']})")
    (out / "results.json").write_text(json.dumps(
        {"config": cfg, "pipeline": pipe, "phase1_paths": p1, "stopping_table": stop_rows, "scenarios": scen},
        indent=1, default=lambda o: o.tolist() if isinstance(o, np.ndarray) else float(o) if isinstance(o, np.floating)
        else bool(o) if isinstance(o, np.bool_) else str(o)))
    (out / "summary.md").write_text(summarise(cfg, pipe, p1, scen, stop_rows), encoding="utf-8")
    log(f"wrote {out}")


if __name__ == "__main__":
    main()

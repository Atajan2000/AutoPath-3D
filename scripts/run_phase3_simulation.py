"""Phase 3: closed-loop drone simulation, executed-path safety verification, replanning scenarios.

Usage:
  python scripts/run_phase3_simulation.py [--config configs/phase3.yaml] [--out results/phase3]
                                          [--parts navigation,scenarios,exploration]
Writes results.json, summary.md, sims/<run>.npz|json (time series + events, read by
scripts/view.py) and planning/<viewer entry>.npz (planner exploration data). No media.
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
from autopath3d.pipeline import ClearancePolicy
from autopath3d.replanning import SupervisorConfig, updated_scene
from autopath3d.replanning.scenarios import SCENARIOS
from autopath3d.simulation import get_disturbance, load_result, save_result
from autopath3d.simulation.navigator import jsonable
from autopath3d.simulation.scenarios import ScenarioRunner, run_navigation, scenario_by_name
from autopath3d.trajectories import TrajectoryLimits
from autopath3d.viz.exploration import capture_astar, capture_rrt, save_exploration

ROOT = Path(__file__).resolve().parents[1]


def run_name(kind: str, *parts: str) -> str:
    return "__".join([kind, *parts])


def f(x, nd=3):
    if x is None or (isinstance(x, float) and not math.isfinite(x)):
        return "–"
    if isinstance(x, bool):
        return "yes" if x else "**no**"
    return f"{x:.{nd}f}" if isinstance(x, float) else str(x)


def row_from(res) -> dict:
    m, meta = res.metrics, res.meta
    tr = m["tracking"]
    return {"run": meta.get("run_name"), "disturbance": meta["disturbance"], "goal_reached": m["goal_reached"],
            "safety_pass": m["safety_pass"], "mission_success": m["mission_success"],
            "failed_checks": [k for k, v in m["checks"].items() if not v], "final_mode": m["final_mode"],
            "execution_duration_s": m["execution_duration_s"],
            "pos_err_max": tr["position_error"]["max"], "pos_err_rms": tr["position_error"]["rms"],
            "vel_err_max": tr["velocity_error"]["max"], "vel_err_rms": tr["velocity_error"]["rms"],
            "max_speed": m["max_speed"], "max_accel_cmd": m["max_accel_cmd_tracking_hold"],
            "max_accel_brake": m["max_accel_cmd_braking"], "max_accel_net": m["max_accel_net"],
            "min_clearance_lower": m["clearance"]["lower"], "min_clearance_upper": m["clearance"]["upper"],
            "goal_error_final": m["goal_error_final"], "saturated_fraction": m["saturated_fraction"],
            "hold_drift_max": m["hold_drift_max"], "sim_wall_s": meta["sim_wall_s"],
            "safety_monitor_wall_s": meta["safety_monitor_wall_s"]}


# ---------------------------------------------------------------- parts
def part_navigation(cfg, p2, pol, lim, dists, out, log):
    p2rows = {}
    pr = ROOT / cfg["phase2_results"]
    if pr.exists():
        for r in json.loads(pr.read_text())["pipeline"]:
            p2rows[(r["scene"], r["task"], r["source"])] = r
    rows = []
    for name in p2["scenes"]:
        scene = get_scene(name)
        for t in scene.tasks:
            for key in cfg["navigation"]["planners"]:
                spec = p2["planners"][key]
                pipe, runs = run_navigation(name, t.name, spec, pol, lim, cfg, dists, p2["map"])
                ps = runs[0].meta["pipeline"] if runs else {"status": pipe.status, "reason": pipe.reason,
                                                            "planning_time_s": pipe.record.get("planning_time_s")}
                ref = p2rows.get((name, t.name, key))
                repro = None
                if ref is not None and pipe.status == "success":
                    repro = (abs(ref["duration_s"] - ps["duration_s"]) < 1e-9
                             and abs(ref["simplified_length_m"] - ps["simplified_length_m"]) < 1e-9)
                elif ref is not None:
                    repro = ref["status"] == pipe.status
                if not runs:
                    rows.append({"scene": name, "task": t.name, "planner": key, "pipeline_status": pipe.status,
                                 "pipeline_reason": pipe.reason, "reproduces_phase2": repro, "simulated": False,
                                 "planning_time_s": ps.get("planning_time_s")})
                    log(f"navigation {name}/{t.name}/{key}: {pipe.status} -> not simulated")
                for res in runs:
                    rn = run_name("nav", name, t.name, key, res.meta["disturbance"])
                    res.meta["run_name"] = rn
                    save_result(res, out / "sims", rn)
                    row = row_from(res)
                    row.update(scene=name, task=t.name, planner=key, pipeline_status=pipe.status, simulated=True,
                               reproduces_phase2=repro, ref_duration_s=ps["duration_s"],
                               ref_min_clearance=ps["ref_min_clearance"][0], ref_max_speed=ps["ref_max_speed_upper"],
                               ref_max_accel=ps["ref_max_accel_upper"], planning_time_s=ps["planning_time_s"],
                               shortcut_time_s=ps["shortcut_time_s"], generation_time_s=ps["generation_time_s"],
                               validation_time_s=ps["validation_time_s"])
                    rows.append(row)
                    log(f"navigation {name}/{t.name}/{key}/{res.meta['disturbance']}: goal={row['goal_reached']} "
                        f"safety={row['safety_pass']} err={row['pos_err_max']:.4f} clr={row['min_clearance_lower']:.3f}")
    return rows


def part_scenarios(cfg, p2, pol, lim, sup_cfg, dists, out, log):
    rows = []
    for sc in SCENARIOS:
        runner = ScenarioRunner(sc, pol, lim, sup_cfg, cfg)
        for dist in dists:
            res = runner.run(dist)
            rn = run_name("scn", sc.name, dist.name)
            res.meta["run_name"] = rn
            save_result(res, out / "sims", rn)
            row = row_from(res)
            s = res.supervisor or {}
            stp = res.first("STOPPED")
            hv = res.first("HOLD_VERIFIED") or res.first("MISSION_ABORTED")
            col = res.first("COLLISION")
            rp = res.replan or {}
            dev = None
            if stp is not None and stp.data.get("predicted_stop_point") is not None:
                dev = float(np.linalg.norm(np.asarray(stp.data["stop_point"]) - np.asarray(stp.data["predicted_stop_point"])))
            after = res.metrics["clearance"].get("after_update")
            row.update(scenario=sc.name, expected=res.meta["expected"], outcome=res.meta["outcome"],
                       prediction=res.meta["supervisor_prediction"], matches_expected=res.meta["matches_expected"],
                       speed_at_detect=s.get("speed_at_detect"), stopping_distance_baseline_m=s.get("stopping_distance_baseline_m"),
                       distance_to_contact_m=s.get("distance_to_contact_m"),
                       predicted_stop_clearance=(s.get("stop_min_clearance") or [None])[0],
                       actual_clearance_after_update=None if after is None else after[0],
                       stop_deviation_m=dev, hold_clearance=None if hv is None else hv.data.get("hold_clearance"),
                       contact_speed=None if col is None else col.data["speed"],
                       contact_penetration_m=None if col is None else col.data["penetration_m"],
                       t_contact=None if col is None else col.t,
                       replan_compute_s=rp.get("measured_compute_s"), sim_planning_s=rp.get("sim_planning_s"),
                       map_update_s=rp.get("map_update_s"),
                       attempts=[(a["planner"], a["status"], a["compute_s"], a.get("planning_time_s"))
                                 for a in rp.get("attempts", [])],
                       t_resume=rp.get("t_resume_sim"), assess_wall_s=s.get("assess_wall_s"),
                       conflict_check_s=s.get("conflict_check_s"),
                       nominal_planning_time_s=res.meta["pipeline"]["planning_time_s"],
                       events=[(e.t, e.name) for e in res.events])
            rows.append(row)
            log(f"scenario {sc.name}/{dist.name}: {row['outcome']} (expected {row['expected']}, "
                f"predicted {row['prediction']}) goal={row['goal_reached']} safety={row['safety_pass']}")
    return rows


def part_exploration(cfg, p2, pol, lim, out, log):
    """Narrow deterministic reruns with recording flags, only for the viewer entries."""
    rows = {}
    cache = {}
    sims = out / "sims"
    default = cfg["disturbances"][0]
    rrt_spec = p2["planners"]["rrt_star"]
    for entry, spec in cfg["viewer"].items():
        if spec["kind"] == "navigation":
            scene_name, task_name = spec["scene"], spec["task"]
            res = load_result(sims, run_name("nav", scene_name, task_name, spec["planner"], default))
        else:
            sc = scenario_by_name(spec["scenario"])
            scene_name, task_name = sc.scene, sc.task
            res = load_result(sims, run_name("scn", sc.name, default))
        scene = get_scene(scene_name)
        task = scene.task(task_name)
        key = (scene_name, task_name)
        if key not in cache:
            tic = time.perf_counter()
            d = capture_astar(scene, task.start, task.goal, pol, p2["map"])
            d.update(capture_rrt(scene, task.start, task.goal, pol, p2["map"], rrt_spec))
            d["capture_wall_s"] = time.perf_counter() - tic
            cache[key] = d
        data = dict(cache[key])
        ps = res.meta["pipeline"]
        data["astar_matches_pipeline"] = (ps["planner"] == "astar" and
                                          abs(data["astar_length"] - ps["path_length_m"]) < 1e-9)
        if res.replan is not None and res.meta.get("update"):
            hv = res.first("HOLD_VERIFIED")
            hold = np.asarray(hv.data["hold_point"], float)
            sc = scenario_by_name(spec["scenario"])
            from autopath3d.replanning.supervisor import ObstacleUpdate
            from autopath3d.environments import Box
            obs = [Box(tuple(o["lo"]), tuple(o["hi"]), o["name"]) for o in res.meta["update"]["obstacles"]]
            new_scene = updated_scene(scene, ObstacleUpdate(sc.name, sc.t_appear, obs))
            for a in res.replan["attempts"]:
                k = a["attempt"]
                aspec = sc.attempts[k]
                if aspec["planner"] == "astar":
                    r = capture_astar(new_scene, hold, task.goal, pol, p2["map"])
                    data.update({f"replan{k}_{kk}": v for kk, v in r.items()})
                    data[f"replan{k}_matches_attempt"] = (a["status"] != "success" and r["astar_status"] != "success") or \
                        (a["path_length_m"] is not None and abs(r["astar_length"] - a["path_length_m"]) < 1e-9)
                else:
                    r = capture_rrt(new_scene, hold, task.goal, pol, p2["map"], aspec)
                    data.update({f"replan{k}_{kk}": v for kk, v in r.items()})
                    data[f"replan{k}_matches_attempt"] = (r["rrt_status"] == "success") == (a["status"] == "success")
                data[f"replan{k}_planner"] = aspec["planner"]
        save_exploration(out / "planning" / f"{entry}.npz", data)
        rows[entry] = {k: v for k, v in data.items() if not isinstance(v, np.ndarray)}
        log(f"exploration {entry}: A* {data['astar_nodes_expanded']} expansions "
            f"(matches pipeline: {data['astar_matches_pipeline']}), RRT* {data['rrt_tree_nodes']} nodes")
    return rows


# ---------------------------------------------------------------- summary
def summarise(cfg, p2, nav, scen, expl) -> str:
    c, L, ctrl, sim = p2["clearance"], p2["limits"], cfg["controller"], cfg["simulation"]
    c_nom = c["robot_radius"] + c["safety_margin"]
    lines = ["# Phase 3 results: closed-loop simulation", "",
             f"Model: translational double integrator, zero-order hold, dt = {sim['dt']} s. Controller: "
             f"feedforward + PD (kp = {ctrl['kp']}, kd = {ctrl['kd']}), |a_cmd| ≤ {ctrl['a_track_max']} m/s² "
             f"tracking/holding, ≤ {ctrl['a_brake_max']} m/s² braking, speed governor v ≤ {ctrl['v_max']} m/s. "
             f"Disturbance profiles: `gusts` (deterministic wind, |d| ≤ 0.42 m/s²) and `ideal` (none). "
             f"Robot radius {c['robot_radius']} m, nominal clearance {c_nom:.2f} m, trajectory clearance "
             f"{c_nom + c['tracking_allowance']:.2f} m, tracking allowance {c['tracking_allowance']} m.", "",
             "Clearances are certified lower bounds on the ground-truth distance of the robot centre over the "
             "executed piecewise-quadratic path (convex-hull bound per control step, refined near the minimum); "
             "they are rigorous for the simulated model only. Tracking errors are measured at every control step "
             "while a reference is tracked. A run fails the safety test if clearance < nominal clearance, any "
             "contact occurs, tracking error > allowance, speed > v_max, or |a_cmd| exceeds its limit.", ""]

    # 1. navigation
    lines += ["## 1. Closed-loop execution of every Phase 2 pipeline", "",
              "| scene / task | planner | profile | goal | safety | ref T [s] | exec T [s] | pos err max / RMS [m] "
              "| vel err max / RMS [m/s] | v max [m/s] | a_cmd max [m/s²] | a_net max [m/s²] | ref clearance ≥ [m] "
              "| executed clearance ≥ [m] | goal err [m] | saturated steps |",
              "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for r in nav:
        if not r["simulated"]:
            lines.append(f"| {r['scene']} / {r['task']} | {r['planner']} | – | not simulated: Phase 2 pipeline "
                         f"{r['pipeline_status']} (no trajectory) | | | | | | | | | | | | |")
            continue
        lines.append(f"| {r['scene']} / {r['task']} | {r['planner']} | {r['disturbance']} | {f(r['goal_reached'])} "
                     f"| {f(r['safety_pass'])} | {f(r['ref_duration_s'], 2)} | {f(r['execution_duration_s'], 2)} "
                     f"| {r['pos_err_max']:.4f} / {r['pos_err_rms']:.4f} | {r['vel_err_max']:.4f} / {r['vel_err_rms']:.4f} "
                     f"| {f(r['max_speed'])} | {f(r['max_accel_cmd'])} | {f(r['max_accel_net'])} "
                     f"| {f(r['ref_min_clearance'])} | {f(r['min_clearance_lower'])} | {f(r['goal_error_final'])} "
                     f"| {100 * r['saturated_fraction']:.1f} % |")
    sims = [r for r in nav if r["simulated"]]
    lines += [""]
    for prof in cfg["disturbances"]:
        rs = [r for r in sims if r["disturbance"] == prof]
        if not rs:
            continue
        lines.append(f"- `{prof}`: {len(rs)} runs, goal reached {sum(r['goal_reached'] for r in rs)}/{len(rs)}, "
                     f"safety pass {sum(r['safety_pass'] for r in rs)}/{len(rs)}, collisions 0 expected / "
                     f"{sum('no_collision' in r['failed_checks'] for r in rs)} observed; max position error "
                     f"{max(r['pos_err_max'] for r in rs):.4f} m (largest RMS {max(r['pos_err_rms'] for r in rs):.4f} m), "
                     f"max velocity error {max(r['vel_err_max'] for r in rs):.4f} m/s, max speed "
                     f"{max(r['max_speed'] for r in rs):.3f} m/s, max |a_cmd| {max(r['max_accel_cmd'] for r in rs):.3f} m/s², "
                     f"max |a_net| {max(r['max_accel_net'] for r in rs):.3f} m/s², min executed clearance "
                     f"{min(r['min_clearance_lower'] for r in rs):.3f} m, max goal error {max(r['goal_error_final'] for r in rs):.3f} m.")
    worst = min((r for r in sims if r["disturbance"] == cfg["disturbances"][0]), key=lambda r: r["min_clearance_lower"])
    lines += [f"- Tightest run ({cfg['disturbances'][0]}): {worst['scene']}/{worst['task']}/{worst['planner']}: reference "
              f"clearance ≥ {worst['ref_min_clearance']:.3f} m, executed ≥ {worst['min_clearance_lower']:.3f} m, max "
              f"tracking error {worst['pos_err_max']:.4f} m.",
              f"- Pipelines reproduce Phase 2 (same duration and simplified length): "
              f"{sum(1 for r in nav if r['reproduces_phase2'] and (not r['simulated'] or r['disturbance'] == cfg['disturbances'][0]))}"
              f"/{len({(r['scene'], r['task'], r['planner']) for r in nav})}.", ""]

    # 2. scenarios
    lines += ["## 2. Obstacle-update scenarios with the simulated drone", "",
              "Outcome = what the simulated run actually did; prediction = the Phase 2 supervisor's classification "
              "made at detection from the actual state (it assumes exact 3 m/s² braking and no disturbance).", "",
              "| scenario | profile | expected (Phase 2) | predicted | simulated outcome | matches | goal | safety "
              "| speed at detect [m/s] | stop dist baseline [m] | predicted stop clearance [m] | executed clearance after update [m] "
              "| actual vs predicted stop point [m] | hold clearance [m] | replan compute (wall) [s] | simulated hold [s] "
              "| exec T [s] | pos err max [m] |",
              "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for r in scen:
        extra = ""
        if r["contact_speed"] is not None:
            extra = f" (contact at {r['t_contact']:.2f} s, {r['contact_speed']:.2f} m/s, {1000 * r['contact_penetration_m']:.1f} mm)"
        lines.append(
            f"| {r['scenario']} | {r['disturbance']} | {', '.join(r['expected'])} | {r['prediction']} "
            f"| **{r['outcome']}**{extra} | {f(r['matches_expected'])} | {f(r['goal_reached'])} | {f(r['safety_pass'])} "
            f"| {f(r['speed_at_detect'], 2)} | {f(r['stopping_distance_baseline_m'], 2)} | {f(r['predicted_stop_clearance'])} "
            f"| {f(r['actual_clearance_after_update'])} | {f(r['stop_deviation_m'])} | {f(r['hold_clearance'])} "
            f"| {f(r['replan_compute_s'], 2)} | {f(r['sim_planning_s'], 2)} | {f(r['execution_duration_s'], 2)} "
            f"| {r['pos_err_max']:.4f} |")
    lines += ["", "Negative predicted clearances denote predicted penetration (certified lower bound). Executed clearance "
              "for collision runs is the minimum up to the terminating contact step.", ""]

    lines += ["### Event timelines (default profile)", ""]
    for r in scen:
        if r["disturbance"] != cfg["disturbances"][0]:
            continue
        lines.append(f"- **{r['scenario']}**: " + " → ".join(f"{t:.2f} {n}" for t, n in r["events"]))
    lines += [""]

    # 3. timing
    lines += ["## 3. Computation time by stage (wall clock, this machine)", "",
              "Planner search, trajectory pipeline, supervisor and simulation are timed separately. The simulated "
              "hold during replanning equals the measured replanning compute rounded up to dt. Viewer logging and "
              "playback speed are not part of these numbers.", "",
              "| run | nominal planning [s] | assess (conflict + stop prediction) [s] | map rebuild [s] "
              "| replan attempts: planner / status / compute [s] | simulation loop [s] | of which safety monitor [s] |",
              "|---|---|---|---|---|---|---|"]
    for r in scen:
        if r["disturbance"] != cfg["disturbances"][0]:
            continue
        att = "; ".join(f"{p} / {s} / {c_:.2f}" for p, s, c_, _ in r["attempts"]) or "–"
        lines.append(f"| {r['scenario']} | {f(r['nominal_planning_time_s'], 2)} | {f(r['assess_wall_s'], 4)} "
                     f"| {f(r['map_update_s'], 2)} | {att} | {f(r['sim_wall_s'], 2)} | {f(r['safety_monitor_wall_s'], 2)} |")
    for r in sims:
        if r["disturbance"] == cfg["disturbances"][0]:
            lines.append(f"| nav {r['scene']}/{r['task']}/{r['planner']} | {f(r['planning_time_s'], 2)} (shortcut "
                         f"{f(r['shortcut_time_s'], 3)}, generation {f(r['generation_time_s'], 3)}, validation "
                         f"{f(r['validation_time_s'], 3)}) | – | – | – | {f(r['sim_wall_s'], 2)} | {f(r['safety_monitor_wall_s'], 2)} |")
    lines += [""]

    if expl:
        lines += ["## 4. Planner exploration data for the viewer (narrow deterministic reruns)", "",
                  "| viewer entry | A* expansions | A* rerun = visualised plan | RRT* nodes / rewires / first solution it. "
                  "| replanning reruns |", "|---|---|---|---|---|"]
        for e, d in expl.items():
            rp = [f"{d[f'replan{k}_planner']}: " + ("matches" if d.get(f"replan{k}_matches_attempt") else "**mismatch**")
                  for k in range(3) if f"replan{k}_planner" in d]
            lines.append(f"| {e} | {d['astar_nodes_expanded']} | {f(d['astar_matches_pipeline'])} "
                         f"| {d['rrt_tree_nodes']} / {d['rrt_rewires']} / {d['rrt_first_solution_iteration']} "
                         f"| {', '.join(rp) or '–'} |")
        lines += [""]
    return "\n".join(lines) + "\n"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default=str(ROOT / "configs" / "phase3.yaml"))
    ap.add_argument("--out", default=str(ROOT / "results" / "phase3"))
    ap.add_argument("--parts", default="navigation,scenarios,exploration")
    args = ap.parse_args()
    cfg = yaml.safe_load(Path(args.config).read_text())
    p2 = yaml.safe_load((ROOT / cfg["phase2_config"]).read_text())
    out = Path(args.out)
    (out / "sims").mkdir(parents=True, exist_ok=True)
    (out / "planning").mkdir(parents=True, exist_ok=True)
    c = p2["clearance"]
    pol = ClearancePolicy(c["robot_radius"], c["safety_margin"], c["tracking_allowance"])
    lim = TrajectoryLimits(**p2["limits"])
    sup_cfg = SupervisorConfig(p2["supervisor"]["detection_latency_s"], p2["supervisor"]["decision_latency_s"],
                               p2["map"]["resolution"], p2["map"]["clearance_cap"])
    dists = [get_disturbance(n) for n in cfg["disturbances"]]
    parts = set(args.parts.split(","))
    t0 = time.perf_counter()
    log = lambda m: print(f"[{time.perf_counter() - t0:6.1f}s] {m}", flush=True)
    prev = json.loads((out / "results.json").read_text()) if (out / "results.json").exists() else {}
    nav = part_navigation(cfg, p2, pol, lim, dists, out, log) if "navigation" in parts else prev.get("navigation", [])
    scen = part_scenarios(cfg, p2, pol, lim, sup_cfg, dists, out, log) if "scenarios" in parts else prev.get("scenarios", [])
    expl = part_exploration(cfg, p2, pol, lim, out, log) if "exploration" in parts else prev.get("exploration", {})
    (out / "results.json").write_text(json.dumps(jsonable({"config": cfg, "navigation": nav, "scenarios": scen,
                                                           "exploration": expl}), indent=1), encoding="utf-8")
    (out / "summary.md").write_text(summarise(cfg, p2, nav, scen, expl), encoding="utf-8")
    log(f"wrote {out}")


if __name__ == "__main__":
    main()

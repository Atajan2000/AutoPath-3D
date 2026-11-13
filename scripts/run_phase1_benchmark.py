"""Phase 1 planning benchmark: A* vs RRT* on deterministic scenes.

Usage:  python scripts/run_phase1_benchmark.py [--config configs/phase1.yaml] [--out results/phase1]
Writes runs.json (all records), summary.md (tables) and rrt_budget_convergence.png.
"""

from __future__ import annotations

import argparse
import json
import math
import statistics as stats
import time
from pathlib import Path

import numpy as np
import yaml

from autopath3d.evaluation.benchmark import build_world, run_and_record

ROOT = Path(__file__).resolve().parents[1]


def _fmt(x, nd=2):
    if x is None or (isinstance(x, float) and not math.isfinite(x)):
        return "–"
    return f"{x:.{nd}f}"


def main_benchmark(cfg, log):
    runs = []
    seeds = cfg["rrt_star"]["seeds"]
    mem = cfg.get("measure_memory", False)
    for scene in cfg["scenes"]:
        world = build_world(scene, cfg)
        for task in world.scene.tasks:
            r = run_and_record(world, task.name, "astar", cfg, measure_memory=mem)
            runs.append(r)
            log(f"{scene}/{task.name} A*: {r['status']} L={_fmt(r['length_m'])} "
                f"t={_fmt(r['planning_time_s'])}s valid={r['validated']}")
            for s in seeds:
                r = run_and_record(world, task.name, "rrt_star", cfg, measure_memory=mem and s == seeds[0], seed=s)
                runs.append(r)
            ok = sum(x["validated"] for x in runs[-len(seeds):])
            log(f"{scene}/{task.name} RRT*: validated {ok}/{len(seeds)}")
    return runs


def resolution_study(cfg, log):
    st = cfg["studies"]["resolution"]
    runs = []
    for res in st["resolutions"]:
        worlds = {}
        for scene, task in st["tasks"]:
            if scene not in worlds:
                worlds[scene] = build_world(scene, cfg, resolution=res)
            w = worlds[scene]
            r = run_and_record(w, task, "astar", cfg)
            r["study"] = "resolution"
            r["grid_voxels"] = int(np.prod(w.grid.shape))
            runs.append(r)
            for s in st["rrt_seeds"]:
                r = run_and_record(w, task, "rrt_star", cfg, seed=s)
                r["study"] = "resolution"
                r["grid_voxels"] = int(np.prod(w.grid.shape))
                runs.append(r)
            log(f"resolution {res} {scene}/{task}: A* {runs[-1 - len(st['rrt_seeds'])]['status']}")
    return runs


def connectivity_study(cfg, log):
    st = cfg["studies"]["connectivity"]
    runs = []
    for scene, task in st["tasks"]:
        w = build_world(scene, cfg)
        for conn in st["values"]:
            r = run_and_record(w, task, "astar", cfg, connectivity=conn)
            r["study"] = "connectivity"
            runs.append(r)
            log(f"connectivity {conn} {scene}/{task}: {r['status']} L={_fmt(r['length_m'])}")
    return runs


def budget_study(cfg, log):
    """One run per seed at the largest budget; smaller budgets are exact prefixes of it
    (fixed seed, iteration-budgeted, budget-independent algorithm)."""
    st = cfg["studies"]["rrt_budget"]
    budgets = sorted(st["iterations"])
    runs = []
    for scene, task in st["tasks"]:
        w = build_world(scene, cfg)
        for s in st["seeds"]:
            r = run_and_record(w, task, "rrt_star", cfg, seed=s, max_iterations=budgets[-1],
                               checkpoints=tuple(budgets))
            r["study"] = "rrt_budget"
            runs.append(r)
        log(f"budget {scene}/{task}: validated {sum(x['validated'] for x in runs[-len(st['seeds']):])}/{len(st['seeds'])}")
    return runs


def summarise(main, resol, conn, budget, cfg) -> str:
    L = ["# Phase 1 benchmark results", "",
         f"Robot clearance c = {cfg['robot']['radius']} + {cfg['robot']['safety_margin']} m; "
         f"resolution {cfg['map']['resolution']} m; A* {cfg['astar']['connectivity']}-connected; "
         f"RRT* {cfg['rrt_star']['max_iterations']} iterations, step {cfg['rrt_star']['step_size']} m, "
         f"goal bias {cfg['rrt_star']['goal_bias']}, seeds {cfg['rrt_star']['seeds']}.",
         "Times are planner-only (map/field build excluded). 'Valid' = independent exact "
         "validation against ground truth AND voxel map (min clearance ≥ c).", "",
         "## A* vs RRT*", "",
         "| scene / task | straight [m] | A* status | A* valid | A* L [m] | A* t [s] | A* expanded | A* clr [m] | A* mem [MB] "
         "| RRT* valid | RRT* L mean±sd (best) [m] | RRT* t [s] | RRT* nodes | RRT* seg checks | RRT* 1st sol it (median) | RRT* clr min [m] | RRT* mem [MB] |",
         "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
    keys = []
    for r in main:
        k = (r["scene"], r["task"])
        if k not in keys:
            keys.append(k)
    for k in keys:
        a = next(r for r in main if (r["scene"], r["task"]) == k and r["planner"] == "astar")
        rr = [r for r in main if (r["scene"], r["task"]) == k and r["planner"] == "rrt_star"]
        ok = [r for r in rr if r["validated"]]
        lens = [r["length_m"] for r in ok]
        lstr = "–" if not lens else (f"{stats.mean(lens):.2f}±{stats.pstdev(lens):.2f} ({min(lens):.2f})")
        fs = [r["stats"]["first_solution_iteration"] for r in ok]
        mem_r = next((r.get("peak_memory_mb") for r in rr if "peak_memory_mb" in r), None)
        L.append(
            f"| {k[0]} / {k[1]} | {a['straight_line_m']:.1f} | {a['status']} | {a['validated']} | {_fmt(a['length_m'])} "
            f"| {_fmt(a['planning_time_s'])} | {a['stats'].get('nodes_expanded', '–')} | {_fmt(a['gt_min_clearance_m'], 3)} "
            f"| {_fmt(a.get('peak_memory_mb'), 1)} | {len(ok)}/{len(rr)} | {lstr} "
            f"| {_fmt(stats.mean(r['planning_time_s'] for r in rr))} | {round(stats.mean(r['stats']['tree_nodes'] for r in rr))} "
            f"| {round(stats.mean(r['stats']['segment_checks'] for r in rr))} | {stats.median(fs) if fs else '–'} "
            f"| {_fmt(min((r['gt_min_clearance_m'] for r in ok), default=None), 3)} | {_fmt(mem_r, 1)} |")
    succ = [r for r in main if r["success"]]
    L += ["", f"Returned paths: {len(succ)}; independently validated: {sum(r['validated'] for r in succ)}; "
          f"planner checker agrees: {sum(r['checker_path_free'] for r in succ)}.", ""]

    L += ["## Resolution study", "", "| scene / task | res [m] | voxels | A* status | A* L [m] | A* t [s] | A* clr [m] "
          "| RRT* valid | RRT* L mean [m] | RRT* t mean [s] | field build [s] |", "|---|---|---|---|---|---|---|---|---|---|---|"]
    for res in sorted({r["resolution"] for r in resol}):
        for k in dict.fromkeys((r["scene"], r["task"]) for r in resol):
            sel = [r for r in resol if r["resolution"] == res and (r["scene"], r["task"]) == k]
            if not sel:
                continue
            a = next(r for r in sel if r["planner"] == "astar")
            rr = [r for r in sel if r["planner"] == "rrt_star"]
            ok = [r for r in rr if r["validated"]]
            L.append(f"| {k[0]} / {k[1]} | {res} | {a['grid_voxels']} | {a['status']} | {_fmt(a['length_m'])} "
                     f"| {_fmt(a['planning_time_s'])} | {_fmt(a['gt_min_clearance_m'], 3)} | {len(ok)}/{len(rr)} "
                     f"| {_fmt(stats.mean(r['length_m'] for r in ok) if ok else None)} "
                     f"| {_fmt(stats.mean(r['planning_time_s'] for r in rr))} | {_fmt(a['field_build_s'])} |")

    L += ["", "## A* connectivity study", "", "| scene / task | connectivity | status | L [m] | t [s] | expanded |",
          "|---|---|---|---|---|---|"]
    for r in conn:
        L.append(f"| {r['scene']} / {r['task']} | {r['connectivity']} | {r['status']} | {_fmt(r['length_m'])} "
                 f"| {_fmt(r['planning_time_s'])} | {r['stats']['nodes_expanded']} |")

    L += ["", "## RRT* budget study (prefix checkpoints of fixed-seed runs)", "",
          "| scene / task | iterations | solved | best cost mean±sd [m] | elapsed mean [s] | tree nodes mean |",
          "|---|---|---|---|---|---|"]
    for k in dict.fromkeys((r["scene"], r["task"]) for r in budget):
        rr = [r for r in budget if (r["scene"], r["task"]) == k]
        for b in sorted(cfg["studies"]["rrt_budget"]["iterations"]):
            cps = [next(c for c in r["stats"]["checkpoints"] if c["iteration"] == b) for r in rr]
            costs = [c["best_cost"] for c in cps if math.isfinite(c["best_cost"])]
            L.append(f"| {k[0]} / {k[1]} | {b} | {len(costs)}/{len(cps)} "
                     f"| {(f'{stats.mean(costs):.2f}±{stats.pstdev(costs):.2f}') if costs else '–'} "
                     f"| {stats.mean(c['elapsed_s'] for c in cps):.2f} | {round(stats.mean(c['tree_nodes'] for c in cps))} |")
    return "\n".join(L) + "\n"


def plot_budget(budget, main, out: Path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    keys = list(dict.fromkeys((r["scene"], r["task"]) for r in budget))
    fig, axes = plt.subplots(1, len(keys), figsize=(6 * len(keys), 4))
    axes = np.atleast_1d(axes)
    for ax, k in zip(axes, keys):
        for r in (x for x in budget if (x["scene"], x["task"]) == k):
            h = r["stats"]["cost_history"]
            if h:
                its = [e[0] for e in h] + [r["stats"]["iterations"]]
                cs = [e[1] for e in h] + [h[-1][1]]
                ax.step(its, cs, where="post", label=f"RRT* seed {r['seed']}")
        a = next((x for x in main if (x["scene"], x["task"]) == k and x["planner"] == "astar"), None)
        if a and a["success"]:
            ax.axhline(a["length_m"], color="k", ls="--", label="A* (26-conn lattice)")
        ax.set_title(f"{k[0]} / {k[1]}")
        ax.set_xlabel("iteration")
        ax.set_ylabel("best path cost [m]")
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(out, dpi=120)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default=str(ROOT / "configs" / "phase1.yaml"))
    ap.add_argument("--out", default=str(ROOT / "results" / "phase1"))
    args = ap.parse_args()
    cfg = yaml.safe_load(Path(args.config).read_text())
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    t0 = time.perf_counter()
    log = lambda m: print(f"[{time.perf_counter() - t0:7.1f}s] {m}", flush=True)
    main_runs = main_benchmark(cfg, log)
    resol = resolution_study(cfg, log)
    conn = connectivity_study(cfg, log)
    budget = budget_study(cfg, log)
    (out / "runs.json").write_text(json.dumps(
        {"config": cfg, "main": main_runs, "resolution": resol, "connectivity": conn, "rrt_budget": budget},
        indent=1, default=float))
    (out / "summary.md").write_text(summarise(main_runs, resol, conn, budget, cfg), encoding="utf-8")
    plot_budget(budget, main_runs, out / "rrt_budget_convergence.png")
    log(f"wrote {out}")


if __name__ == "__main__":
    main()

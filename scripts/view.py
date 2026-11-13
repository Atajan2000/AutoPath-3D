"""Open a saved Phase 3 simulation in the Rerun viewer (no planning or simulation is rerun).

  python scripts/view.py --scenario warehouse_replan
  python scripts/view.py --scenario multilevel_navigation
  python scripts/view.py --scenario unsafe_stop
  python scripts/view.py --scenario unsafe_margin --disturbance ideal
  python scripts/view.py --scenario nav:urban_canyon/street_crossing/rrt_star
  python scripts/view.py --list
  python scripts/view.py --scenario urban_replan --save urban.rrd     # write a recording, no window
Recordings are written to results/phase3/recordings/ (git-ignored) and opened from there.

Regenerate the saved results with: python scripts/run_phase3_simulation.py
Playback, pause, scrubbing, orbit/zoom and entity inspection are Rerun's native controls.
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]


def resolve(entry: str, cfg: dict, profile: str) -> tuple[str, dict]:
    if entry.startswith("nav:"):
        scene, task, planner = entry[4:].split("/")
        return f"nav__{scene}__{task}__{planner}__{profile}", {"kind": "navigation", "scene": scene}
    if entry not in cfg["viewer"]:
        raise SystemExit(f"unknown scenario {entry!r}; run with --list")
    spec = cfg["viewer"][entry]
    if spec["kind"] == "navigation":
        return f"nav__{spec['scene']}__{spec['task']}__{spec['planner']}__{profile}", spec
    return f"scn__{spec['scenario']}__{profile}", spec


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scenario", default="warehouse_replan")
    ap.add_argument("--disturbance", default=None, help="saved profile (default: first in configs/phase3.yaml)")
    ap.add_argument("--results", default=str(ROOT / "results" / "phase3"))
    ap.add_argument("--config", default=str(ROOT / "configs" / "phase3.yaml"))
    ap.add_argument("--save", default=None, help="write an .rrd file instead of opening the viewer")
    ap.add_argument("--no-open", action="store_true", help="write the recording but do not open the viewer")
    ap.add_argument("--list", action="store_true")
    args = ap.parse_args()
    cfg = yaml.safe_load(Path(args.config).read_text())
    res_dir = Path(args.results)
    profile = args.disturbance or cfg["disturbances"][0]
    if args.list:
        print("viewer entries (configs/phase3.yaml):")
        for k, v in cfg["viewer"].items():
            print(f"  {k:24s} {v.get('scenario') or v['scene'] + '/' + v['task'] + '/' + v['planner']}")
        runs = sorted(p.stem for p in (res_dir / "sims").glob("nav__*.npz"))
        print("benchmark runs (use --scenario nav:<scene>/<task>/<planner>):")
        for r in runs:
            print("  " + r)
        return 0

    import rerun as rr

    from autopath3d.environments import get_scene
    from autopath3d.simulation import load_result
    from autopath3d.viz.exploration import load_exploration
    from autopath3d.viz.rerun_viewer import blueprint, build_recording

    run, spec = resolve(args.scenario, cfg, profile)
    if not (res_dir / "sims" / f"{run}.npz").exists():
        raise SystemExit(f"no saved run {run} in {res_dir / 'sims'}; run scripts/run_phase3_simulation.py first")
    res = load_result(res_dir / "sims", run)
    scene = get_scene(res.meta["scene"])
    ex = None
    ex_path = res_dir / "planning" / f"{args.scenario}.npz"
    if ex_path.exists():
        ex = load_exploration(ex_path)
        if profile != cfg["disturbances"][0]:      # replanning reruns belong to the default-profile hold point
            ex = {k: val for k, val in ex.items() if not k.startswith("replan")}
    bp = blueprint(scene, ex is not None)
    # Write the recording to a file and open that file in the viewer. Opening a file needs no
    # network server (rr.spawn would start one and trigger a firewall prompt on Windows).
    out = Path(args.save) if args.save else res_dir / "recordings" / f"{args.scenario.replace(':', '_').replace('/', '_')}__{profile}.rrd"
    out.parent.mkdir(parents=True, exist_ok=True)
    rec = rr.RecordingStream("autopath3d_phase3", recording_id=f"{args.scenario}__{profile}")
    rec.save(str(out), default_blueprint=bp)
    rec.send_recording_name(f"{args.scenario} ({profile})")
    build_recording(rec, res, scene, args.scenario, ex)
    rec.flush()
    rec.disconnect()
    print(f"{args.scenario} ({profile}): outcome {res.meta.get('outcome', 'navigation')}, "
          f"safety {'PASS' if res.metrics['safety_pass'] else 'FAIL'}, goal reached {res.metrics['goal_reached']}")
    print(f"recording: {out}")
    if args.save or args.no_open:
        return 0
    exe_dir = Path(sys.executable).parent
    exe = next((p for p in (exe_dir / "rerun.exe", exe_dir / "rerun", exe_dir / "Scripts" / "rerun.exe") if p.exists()),
               None)
    if exe is None:
        raise SystemExit("Rerun viewer binary not found next to the Python interpreter (pip install rerun-sdk)")
    flags = (subprocess.DETACHED_PROCESS | subprocess.CREATE_NEW_PROCESS_GROUP) if os.name == "nt" else 0
    subprocess.Popen([str(exe), "--bind", "127.0.0.1", str(out)], stdin=subprocess.DEVNULL,
                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, creationflags=flags,
                     start_new_session=os.name != "nt")
    print("opened in the Rerun viewer")
    return 0


if __name__ == "__main__":
    sys.exit(main())

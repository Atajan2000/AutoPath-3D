"""Phase 4: apply the conservative stopping assessment to the saved Phase 3 D scenarios.

Reads results/phase3/sims/scn__D*__{gusts,ideal}.{json,npz} (unchanged) and writes
results/phase4/stopping_assessment_phase3.{json,md}. Nothing in Phase 1-3 is re-run or modified.

Assumption bounds for the Phase 3 point-mass model (justified by that model, not by a quadrotor):
  latency from the state measured at detection: the 0.1 s decision latency (the 0.2 s detection
  latency has already elapsed when that state is read); exact in the simulation (integer steps)
  braking exactly 3.0 m/s^2 (controller a_brake_max)
  tracking acceleration during the latency <= 2.0 m/s^2 (controller saturation a_track_max)
  disturbance <= 0.42 m/s^2 for `gusts` (documented profile bound), 0 for `ideal`
  state error 0 (the Phase 3 controller and supervisor read the true state)
"""

from __future__ import annotations

import glob
import json
import os

import numpy as np

from autopath3d.environments.primitives import Box
from autopath3d.environments.scenes import get_scene
from autopath3d.replanning.stopping_assessment import StoppingAssumptions, assess_stopping

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
GUST_BOUND = 0.42


def main():
    rows = []
    for jf in sorted(glob.glob(os.path.join(ROOT, "results/phase3/sims/scn__D*__*.json"))):
        d = json.load(open(jf))
        m = d["meta"]
        z = np.load(jf.replace(".json", ".npz"))
        k = m["k_detect"]
        p, v = z["p"][k], z["v"][k]
        scene = get_scene(m["scene"])
        obs = [Box(tuple(o["lo"]), tuple(o["hi"]), o["name"]) for o in m["update"]["obstacles"]]

        def dist(q, scene=scene, obs=obs):
            q = np.atleast_2d(q)
            out = scene.distance(q)
            for b in obs:
                out = np.minimum(out, b.distance(q))
            return out

        dmax = GUST_BOUND if m["disturbance"] == "gusts" else 0.0
        a = StoppingAssumptions(latency_s=0.1, latency_min_s=0.1, brake_decel=3.0, brake_decel_max=3.0,
                                disturbance_accel=dmax, tracking_accel=2.0, position_error=0.0)
        pol = m["policy"]
        r = assess_stopping(p, v, dist, pol["robot_radius"], pol["nominal"], a)
        # attained minimum clearance after t_appear (certified lower/upper per step, Phase 3 monitor)
        ka = m["k_appear"]
        actual_lo = float(np.nanmin(z["clr_lo"][ka:]))
        actual_hi = float(np.nanmin(z["clr_hi"][ka:]))
        rows.append({
            "run": os.path.basename(jf)[:-5], "disturbance": m["disturbance"],
            "phase2_prediction": d["supervisor"].get("outcome", "safe_stop"),
            "phase2_predicted_hold_clearance": d["supervisor"].get("hold_clearance"),
            "phase3_actual": m["end_reason"] if d["metrics"].get("collision") else m.get("outcome"),
            "phase3_collision": bool(d["metrics"].get("collision")),
            "phase3_min_clearance_after_appear": [actual_lo, actual_hi],
            "assessment": r.to_dict(),
            # Phase 3 terminates a run at first contact, so for collided runs the recorded minimum
            # is not the minimum over the whole stop: only the lower bound and the collision
            # verdict can be checked there.
            "actual_consistent_with_bounds": bool(
                r.clearance_lower - 1e-6 <= actual_hi
                and (actual_lo <= r.clearance_upper + 1e-6 if not d["metrics"].get("collision")
                     else r.clearance_lower < pol["robot_radius"])),
        })
    out_dir = os.path.join(ROOT, "results/phase4")
    os.makedirs(out_dir, exist_ok=True)
    json.dump(rows, open(os.path.join(out_dir, "stopping_assessment_phase3.json"), "w"), indent=1)
    lines = ["# Conservative stopping assessment applied to the saved Phase 3 D scenarios", "",
             "Phase 1-3 results are unchanged; this table only adds the Phase 4 assessment. Clearances in m",
             "(robot radius 0.25, nominal clearance 0.35). Bounds: [certified lower, upper valid for every admitted motion].", "",
             "| run | Phase 2 prediction | Phase 3 actual | actual min clearance | Phase 4 class | bounds | D_max m | rho m | consistent |",
             "|---|---|---|---|---|---|---|---|---|"]
    for r in rows:
        a = r["assessment"]
        lo, hi = r["phase3_min_clearance_after_appear"]
        lines.append(f"| {r['run']} | {r['phase2_prediction']} | {r['phase3_actual']} | {lo:.3f}..{hi:.3f} | "
                     f"{a['classification']} | [{a['clearance_lower']:.3f}, {a['clearance_upper']:.3f}] | "
                     f"{a['worst_stop_distance']:.3f} | {a['lateral_radius']:.3f} | {r["actual_consistent_with_bounds"]} |")
    open(os.path.join(out_dir, "stopping_assessment_phase3.md"), "w").write("\n".join(lines) + "\n")
    print("\n".join(lines))


if __name__ == "__main__":
    main()

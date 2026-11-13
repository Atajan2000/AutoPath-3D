# Results

Measured outputs of Phases 1–4, as produced by the scripts and runs named below. No values have been
edited, re-run or regenerated for publication.

| Path | Content | Produced by |
|---|---|---|
| `phase1/summary.md`, `runs.json`, `rrt_budget_convergence.png` | geometric planning benchmark (A*, RRT*) | `scripts/run_phase1_benchmark.py` |
| `phase2/summary.md`, `results.json` | trajectory pipeline and replanning scenarios | `scripts/run_phase2_evaluation.py` |
| `phase3/summary.md`, `results.json`, `sims/`, `planning/` | point-mass closed-loop simulation; `sims/` and `planning/` feed the Rerun viewer | `scripts/run_phase3_simulation.py` |
| `phase3/viewer_warehouse_replan.png` | Rerun viewer frame | `scripts/view.py` |
| `phase4/stopping_assessment_phase3.*` | conservative stopping assessment applied to Phase 3 runs | `scripts/run_phase4_stopping_assessment.py` |
| `phase4/webots/<scenario>/run_*.json`, `*_series.npz` | archived Webots flights (Demos A, A2, B, C1, C2) | `ros2 launch autopath3d_ros warehouse.launch.py scenario:=…` |
| `phase4/webots/characterisation/` | measured Crazyflie tracking and braking | `… characterise:=true` |
| `phase4/webots/superseded/` | earlier runs invalidated by bugs, kept for traceability (`docs/PHASE4_REPORT.md` §7) | — |
| `phase4/webots/screenshot_B_replan.jpg` | one genuine Webots R2025a frame from a Demo B run (not the archived B run; its overlay shows that run's goal error, 0.009 m). Webots models © Cyberbotics Ltd., see `THIRD_PARTY_NOTICES.md` §4 | scene manager |
| `phase4/e2e/` | compact, sanitised records of end-to-end test sessions | `test_e2e_webots.py` |

Phase 3 Rerun recordings (`*.rrd`) and full Webots launch logs are not published. They are large and
machine-specific, and can be regenerated with the commands in the main README.

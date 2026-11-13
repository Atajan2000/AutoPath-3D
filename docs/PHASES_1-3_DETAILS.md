# Phases 1–3: detailed results and limitations

Moved verbatim from the pre-release README (development version, before the public release) when the README was rewritten for
publication. Full tables: `results/phase1/summary.md`, `results/phase2/summary.md`,
`results/phase3/summary.md`.

## Phase 3 results (summary)

Full tables: `results/phase3/summary.md`. Translational double integrator, dt = 0.01 s, FF + PD
(kp 9, kd 6), |a_cmd| ≤ 2 m/s² tracking, 3 m/s² braking, v ≤ 3 m/s. Two disturbance profiles:
`gusts` (deterministic wind, |d| ≤ 0.42 m/s²) and `ideal`.

- All 18 validated Phase 2 trajectories executed in closed loop under both profiles: 36/36 goal
  reached and safety pass, 0 collisions. Gusts: max position error 0.033 m (RMS ≤ 0.022 m),
  max velocity error 0.034 m/s, within the 0.10 m allowance; max speed 2.914 m/s, max |a_cmd|
  2.000 m/s² (saturation active on 0.6 % of steps of one run), min certified executed clearance
  0.452 m, goal error ≤ 0.029 m. Ideal: error < 1e-4 m (ZOH discretisation only), executed
  clearance equal to the Phase 2 certified clearance within 1 mm. The 4 narrow_900/500
  pipelines have no trajectory (Phase 2 planning failure) and are reported as not simulated.
- Scenarios, ideal profile: all 7 reproduce the Phase 2 classification (A, B, D3 replanned;
  E replanned after retry; C no route, holding safely; D1 contact at 2.36 m/s; D2 stop
  clearance 0.329 m < 0.35 m).
- Scenarios, gusts: A, B, C, D1, D3, E as expected. **D2 degrades from margin violation to
  contact** (0.3 mm into the 0.25 m sphere at 0.22 m/s, just before rest): the supervisor's
  braking prediction (0.328 m) assumes exactly 3 m/s² and no wind; a mean tailwind of
  0.17 m/s² during braking (≈ +8 cm braking distance at 2.84 m/s) plus 1.4 cm along-track
  error at brake onset consumed its 7.8 cm. Reported as a failed
  run, not hidden. Actual stop points differed from the predicted ones by 3–9 cm.
- Replanning holds advance simulated time by the measured compute: B (urban) held 13.6 s
  while A* ran (wall clock on this run; Phase 2 measured 22.3 s on the same problem).

## Phase 1 results (summary)

Full tables: `results/phase1/summary.md`. Robot envelope: 0.25 m sphere + 0.10 m margin
(c = 0.35 m), 0.2 m voxels, A* 26-connected, RRT* 10k iterations × 5 seeds.

- 9 feasible tasks in 5 scenes: A* 9/9, RRT* 45/45 runs succeeded. All 57 returned
  paths passed independent exact validation against both ground-truth geometry and the
  voxel map; the minimum ground-truth clearance was 0.352 m.
- `narrow_500` (0.5 m window < 2c): both planners correctly fail.
- `narrow_900`: A* reports no path at 0.2 m and 0.4 m, while RRT* finds validated
  paths (3/5 seeds). The free voxel band in the window is 0.8 m wide: continuous points
  near its centre clear the walls by 0.40 m, but the nearest lattice centres clear them
  by only 0.30 m. A* at 0.1 m succeeds.
- Path length (mean over seeds): RRT* (10k iterations) is shorter than the A* lattice
  path on 6/9 tasks (for example 79.9 vs 81.4 m on `multilevel/ground_to_top`), equal on
  `narrow_1200`, and longer on both `urban_canyon` tasks (71.1 vs 64.3 m and 50.7 vs
  47.0 m), where 10k samples in the 32,000 m³ volume are too few.
- Planning time: A* 0.01–17 s (Python, up to 416k expansions); RRT* 3.4–6.9 s for 10k
  iterations.

## Phase 2 results (summary)

Full tables: `results/phase2/summary.md`. Trajectory clearance 0.45 m (0.35 m nominal +
0.10 m tracking allowance), v ≤ 3 m/s, |a| ≤ 2 m/s², braking 3 m/s², latency 0.2 + 0.1 s.

- 18/18 feasible pipelines (9 tasks × A*/RRT*) produced independently validated C2
  trajectories: certified max speed ≤ 2.910 m/s, max |a| ≤ 1.940 m/s², min ground-truth
  clearance ≥ 0.452 m, velocity/acceleration jumps at knots ≤ 3e-14. The 4 narrow_900 /
  narrow_500 pipelines fail at planning (reported, not hidden).
- Shortcutting reduced A* paths from 40–336 to 2–11 waypoints (0.7–8.3 % shorter).
- Curved candidates rejected by the continuous certificate: 10 in total. 6 pieces are
  rest-to-rest straight motions: 4 are fallbacks after rejection, and 2 are single-segment
  paths that start and end at rest anyway.
- Phase 1 paths (planned at 0.35 m) reused directly: 57/57 valid without tracking allowance,
  but only 14/57 at 0.45 m. The 43 others, including the 0.352 m RRT* path, are rejected
  rather than smoothed.
- Replanning scenarios, all 7 matching the expected outcome: A warehouse aisle blocked →
  replanned (0.83 s); B urban scaffold → replanned (22.3 s of A* compute while holding);
  C window sealed → no route, holding safely; D1 obstacle 1.5 m ahead at 2.6 m/s →
  unavoidable collision; D2 2.5 m ahead → stopped 0.33 m from it (safety margin violated);
  D3 3.5 m ahead → safe stop and replanned; E first attempt fails (RRT* budget) → held,
  A* retry succeeded.

## Known limitations

Phase 3:
- Translational double integrator with perfect state knowledge: no attitude, thrust, rotor,
  drag or actuator dynamics, no sensing noise. Passing these tests says nothing about quadrotor
  stability, attitude control or aerodynamic feasibility, or real-world flight safety.
- With exact feedforward and a linear controller, tracking error depends only on the
  disturbance (except under saturation), so the tracking results characterise the chosen gust
  profile and gains rather than each trajectory. One deterministic gust profile was tested.
- The supervisor's stopping prediction ignores disturbances and tracking error; scenario D2
  shows it can be optimistic by several centimetres. No margin for this is added yet.
- Clearance bounds are rigorous for the simulated piecewise-quadratic motion, not a formal
  guarantee for a physical vehicle. Contact terminates the run; no collision dynamics.
- Simulated replanning time is the measured wall-clock compute, so regenerated runs differ
  slightly after the replan (the gust phase shifts); everything before it is bit-identical,
  and a fixed planning latency makes whole runs bit-reproducible (used in tests).
- Exploration data (A* expansions, RRT* tree) comes from reruns for the viewer entries only.

Phase 2:
- Translational double-integrator model only, so these trajectories are not proven
  dynamically feasible for a quadrotor. No attitude, thrust or rotor limits.
- Stop-and-replan only: any conflict triggers a full stop, even when the conflict is far
  away. There is no replanning on the fly from a moving state and no moving-obstacle
  prediction; updates are static geometry with known shape.
- The emergency stop is a straight-line constant-deceleration manoeuvre; jerk and
  acceleration jump at braking onset are not limited.
- Waypoint speeds are heuristic (bisector rule); trajectories are not time-optimal.
- Replanning compute is wall-clock Python. Urban A* replanning took 22 s while hovering.
  The supervisor's conflict check (< 2 ms) fits the 0.1 s decision budget, but the map
  rebuild does not and is deliberately done after stopping.
- Collision certificates reject configurations within about 1 mm of a clearance limit.

Phase 1:

- Planners return geometric paths; timing and smoothing come from the Phase 2 pipeline.
- The robot is a bounding sphere; attitude-dependent geometry is not modelled.
- Conservative voxelisation loses up to one voxel per obstacle face; A* is complete and
  optimal only on its lattice (see `narrow_900`).
- Segment certification is rigorous but conservative: configurations within about 1 mm
  of the clearance limit may be rejected (affects start/goal points placed exactly on
  the limit).
- RRT*: brute-force nearest neighbour (O(n) per query), finite-budget results only, no
  optimality guarantee; success on narrow passages depends on the seed.
- A* is pure Python: about 25k expansions/s, 17 s on the 4M-voxel urban map.
- Only synthetic, fully known scenes so far. UNKNOWN space is supported and blocks
  planning by default, but real point-cloud maps are not imported yet.
- Peak memory is measured with tracemalloc in separate runs (Python heap only).

## Viewer (Rerun)

Opens saved simulation results; nothing is replanned or re-simulated.

```bash
python scripts/view.py --scenario multilevel_navigation   # Demo 1: autonomous navigation
python scripts/view.py --scenario warehouse_replan        # Demo 2: dynamic replanning
python scripts/view.py --scenario unsafe_stop             # Demo 3: insufficient stopping distance
python scripts/view.py --list                             # all entries and benchmark runs
```

Other entries: `warehouse_navigation`, `urban_replan`, `no_route`, `unsafe_margin`,
`safe_stop_replan`, `replan_retry`, and any benchmark run as `nav:<scene>/<task>/<planner>`.
`--disturbance ideal` shows the disturbance-free run; `--save file.rrd` writes a recording
without opening a window. Layout: 3D navigation view and planning-exploration tab (A*
expansions, RRT* tree), navigation-state lane, synchronised plots (desired vs actual position,
tracking error, speed, acceleration, clearance) and an event log / run summary. Play, pause,
scrub, orbit and zoom are Rerun's native controls.

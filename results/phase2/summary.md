# Phase 2 results

Robot sphere 0.25 m + safety margin 0.1 m = nominal clearance 0.35 m; tracking allowance 0.1 m -> trajectory clearance 0.45 m. Limits: v ≤ 3.0 m/s, |a| ≤ 2.0 m/s², braking 3.0 m/s². All bounds below are certified intervals from the independent validator (Bezier hull + 1-Lipschitz branch and bound against ground-truth geometry).

## 1. Path → simplified path → trajectory (planned at the 0.45 m trajectory clearance)

| scene / task | planner | status | path L [m] / wp | simplified L [m] / wp | duration [s] | v max ≤ [m/s] | a max ≤ [m/s²] | min clearance [m] | rejected curves | fallback pieces | plan / shortcut / gen / validate [s] |
|---|---|---|---|---|---|---|---|---|---|---|---|
| warehouse / cross_warehouse | astar | success | 37.42 / 136 | 36.32 / 4 | 23.40 | 2.681 | 1.641 | 0.512 | 1 | 0 | 3.05 / 0.113 / 0.008 / 0.029 |
| warehouse / cross_warehouse | rrt_star | success | 36.07 / 21 | 35.60 / 6 | 25.45 | 2.425 | 1.467 | 0.574 | 0 | 0 | 7.06 / 0.037 / 0.019 / 0.093 |
| warehouse / aisle_to_aisle | astar | success | 21.48 / 96 | 20.14 / 6 | 17.00 | 2.461 | 1.670 | 0.567 | 1 | 0 | 1.05 / 0.037 / 0.014 / 0.033 |
| warehouse / aisle_to_aisle | rrt_star | success | 20.91 / 12 | 20.69 / 5 | 15.50 | 2.613 | 1.578 | 0.715 | 0 | 0 | 6.84 / 0.010 / 0.005 / 0.034 |
| urban_canyon / street_crossing | astar | success | 65.10 / 285 | 60.64 / 9 | 39.07 | 2.531 | 1.537 | 0.580 | 0 | 0 | 18.80 / 0.125 / 0.028 / 0.219 |
| urban_canyon / street_crossing | rrt_star | success | 72.53 / 35 | 67.75 / 13 | 49.71 | 2.910 | 1.940 | 0.452 | 3 | 2 | 4.73 / 0.032 / 0.059 / 0.262 |
| urban_canyon / rooftop_landing | astar | success | 47.01 / 170 | 45.29 / 6 | 30.33 | 2.397 | 1.354 | 0.496 | 0 | 0 | 1.71 / 0.080 / 0.012 / 0.059 |
| urban_canyon / rooftop_landing | rrt_star | success | 51.11 / 27 | 48.85 / 7 | 32.80 | 2.581 | 1.581 | 0.484 | 0 | 0 | 4.80 / 0.018 / 0.009 / 0.068 |
| industrial / ground_to_mezzanine | astar | success | 30.19 / 125 | 28.03 / 6 | 20.78 | 2.910 | 1.940 | 0.499 | 3 | 2 | 2.94 / 0.080 / 0.022 / 0.190 |
| industrial / ground_to_mezzanine | rrt_star | success | 29.32 / 19 | 28.68 / 8 | 23.56 | 2.103 | 1.584 | 0.466 | 0 | 0 | 9.02 / 0.020 / 0.014 / 0.120 |
| industrial / cross_hall | astar | success | 33.84 / 138 | 31.04 / 6 | 22.88 | 2.662 | 1.485 | 0.560 | 0 | 0 | 1.92 / 0.039 / 0.006 / 0.062 |
| industrial / cross_hall | rrt_star | success | 31.60 / 17 | 31.22 / 8 | 26.18 | 2.733 | 1.568 | 0.458 | 1 | 0 | 8.58 / 0.016 / 0.025 / 0.099 |
| multilevel / ground_to_top | astar | success | 82.45 / 336 | 76.24 / 11 | 53.78 | 2.743 | 1.356 | 0.500 | 0 | 0 | 6.94 / 0.227 / 0.015 / 0.099 |
| multilevel / ground_to_top | rrt_star | success | 80.74 / 41 | 79.54 / 11 | 54.96 | 2.868 | 1.278 | 0.463 | 1 | 0 | 8.43 / 0.025 / 0.021 / 0.102 |
| multilevel / ground_to_second | astar | success | 32.62 / 152 | 30.73 / 4 | 20.94 | 2.796 | 1.289 | 0.539 | 0 | 0 | 4.09 / 0.083 / 0.009 / 0.050 |
| multilevel / ground_to_second | rrt_star | success | 31.33 / 17 | 30.82 / 4 | 21.01 | 2.771 | 1.032 | 0.576 | 0 | 0 | 9.30 / 0.014 / 0.004 / 0.026 |
| narrow_1200 / through_window | astar | success | 8.06 / 40 | 8.00 / 2 | 5.15 | 2.910 | 1.738 | 0.599 | 0 | 1 | 0.01 / 0.014 / 0.001 / 0.010 |
| narrow_1200 / through_window | rrt_star | success | 8.01 / 7 | 8.00 / 2 | 5.15 | 2.910 | 1.738 | 0.599 | 0 | 1 | 6.39 / 0.005 / 0.001 / 0.018 |
| narrow_900 / through_window | astar | plan_failed | – / 0 | – / – | – | – | – | – | – | – | 0.35 / – / – / – |
| narrow_900 / through_window | rrt_star | plan_failed | – / 0 | – / – | – | – | – | – | – | – | 6.11 / – / – / – |
| narrow_500 / through_window | astar | plan_failed | – / 0 | – / – | – | – | – | – | – | – | 0.27 / – / – / – |
| narrow_500 / through_window | rrt_star | plan_failed | – / 0 | – / – | – | – | – | – | – | – | 6.41 / – / – / – |

Pipelines: 22; validated trajectories: 18; failures: narrow_900/astar: plan_failed, narrow_900/rrt_star: plan_failed, narrow_500/astar: plan_failed, narrow_500/rrt_star: plan_failed.
Over all validated trajectories: max speed ≤ 2.910 m/s, max |a| ≤ 1.940 m/s², min clearance ≥ 0.452 m, max velocity jump 2.3e-14 m/s, max acceleration jump 1.9e-14 m/s².

## 2. Reusing the Phase 1 paths (planned at 0.35 m) without replanning

| tracking allowance [m] | required clearance [m] | paths | validated | rejected at shortcut (input clearance too small) | other failures |
|---|---|---|---|---|---|
| 0.00 | 0.35 | 57 | 57 | 0 | 0 |
| 0.10 | 0.45 | 57 | 14 | 43 | 0 |

The tightest Phase 1 path (urban_canyon/rooftop_landing, phase1_rrt_star_seed0, ground-truth clearance 0.352 m) → allowance 0: success (trajectory clearance ≥ 0.523 m); allowance 0.10: shortcut_rejected.

## 3. Stopping distance baseline d = v·t_L + v²/(2·a_dec)

t_L = detection 0.2 s + decision 0.1 s, a_dec = 3.0 m/s².

| speed [m/s] | latency distance [m] | braking distance [m] | total [m] |
|---|---|---|---|
| 0.5 | 0.15 | 0.04 | 0.19 |
| 1.0 | 0.30 | 0.17 | 0.47 |
| 1.5 | 0.45 | 0.38 | 0.83 |
| 2.0 | 0.60 | 0.67 | 1.27 |
| 2.5 | 0.75 | 1.04 | 1.79 |
| 3.0 | 0.90 | 1.50 | 2.40 |

## 4. Obstacle-update scenarios

| scenario | outcome | expected | speed at appear [m/s] | baseline stop dist [m] | modelled travel to rest [m] | path dist to contact [m] | stop-manoeuvre clearance [m] | hold clearance [m] | replan compute [s] | attempts | new traj L [m] / T [s] | post-update min clearance [m] | goal reached |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| A_warehouse_aisle_blocked | **replanned** | replanned | 0.94 | 0.43 | 0.49 | 3.62 | 0.576 | 0.605 | 0.83 | astar:success | 16.57 / 13.77 | 0.504 | True |
| B_urban_scaffold | **replanned** | replanned | 1.95 | 1.22 | 1.15 | 9.11 | 1.324 | 1.325 | 22.26 | astar:success | 51.80 / 36.57 | 0.479 | True |
| C_narrow_window_sealed | **no_route_holding** | no_route_holding | 0.80 | 0.35 | 0.60 | 3.02 | 1.999 | 2.000 | 0.30 | astar:plan_failed | – / – | 1.999 | False |
| D1_too_close_collision | **unavoidable_collision** | unavoidable_collision | 2.63 | 1.94 | 2.17 | 1.25 | 0.000 | 0.271 | – | – | – / – | – | False |
| D2_too_close_margin | **stop_margin_violation** | stop_margin_violation | 2.63 | 1.94 | 2.17 | 2.26 | 0.328 | 0.329 | – | – | – / – | – | False |
| D3_far_enough | **replanned** | replanned | 2.63 | 1.94 | 2.17 | 3.25 | 0.599 | 0.748 | 0.05 | astar:success | 4.42 / 6.28 | 0.599 | True |
| E_warehouse_retry | **replanned_after_retry** | replanned_after_retry | 0.94 | 0.43 | 0.49 | 3.62 | 0.576 | 0.605 | 0.86 | rrt_star:plan_failed → astar:success | 16.57 / 13.77 | 0.504 | True |

Scenarios matching the expected outcome: 7/7. Clearances are certified lower bounds w.r.t. the updated ground-truth geometry (0.000 = contact). Replanning compute includes the voxel-map rebuild and all pipeline stages; the robot holds at the verified stop point meanwhile.

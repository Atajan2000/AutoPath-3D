# Phase 3 results: closed-loop simulation

Model: translational double integrator, zero-order hold, dt = 0.01 s. Controller: feedforward + PD (kp = 9.0, kd = 6.0), |a_cmd| ≤ 2.0 m/s² tracking/holding, ≤ 3.0 m/s² braking, speed governor v ≤ 3.0 m/s. Disturbance profiles: `gusts` (deterministic wind, |d| ≤ 0.42 m/s²) and `ideal` (none). Robot radius 0.25 m, nominal clearance 0.35 m, trajectory clearance 0.45 m, tracking allowance 0.1 m.

Clearances are certified lower bounds on the ground-truth distance of the robot centre over the executed piecewise-quadratic path (convex-hull bound per control step, refined near the minimum); they are rigorous for the simulated model only. Tracking errors are measured at every control step while a reference is tracked. A run fails the safety test if clearance < nominal clearance, any contact occurs, tracking error > allowance, speed > v_max, or |a_cmd| exceeds its limit.

## 1. Closed-loop execution of every Phase 2 pipeline

| scene / task | planner | profile | goal | safety | ref T [s] | exec T [s] | pos err max / RMS [m] | vel err max / RMS [m/s] | v max [m/s] | a_cmd max [m/s²] | a_net max [m/s²] | ref clearance ≥ [m] | executed clearance ≥ [m] | goal err [m] | saturated steps |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| warehouse / cross_warehouse | astar | gusts | yes | yes | 23.40 | 23.41 | 0.0333 / 0.0201 | 0.0344 / 0.0180 | 2.681 | 1.549 | 1.614 | 0.512 | 0.517 | 0.002 | 0.0 % |
| warehouse / cross_warehouse | astar | ideal | yes | yes | 23.40 | 23.41 | 0.0000 / 0.0000 | 0.0000 / 0.0000 | 2.681 | 1.641 | 1.641 | 0.512 | 0.512 | 0.000 | 0.0 % |
| warehouse / cross_warehouse | rrt_star | gusts | yes | yes | 25.45 | 25.46 | 0.0333 / 0.0199 | 0.0344 / 0.0179 | 2.409 | 1.212 | 1.431 | 0.574 | 0.572 | 0.028 | 0.0 % |
| warehouse / cross_warehouse | rrt_star | ideal | yes | yes | 25.45 | 25.46 | 0.0000 / 0.0000 | 0.0000 / 0.0000 | 2.425 | 1.467 | 1.467 | 0.574 | 0.574 | 0.000 | 0.0 % |
| warehouse / aisle_to_aisle | astar | gusts | yes | yes | 17.00 | 17.00 | 0.0333 / 0.0193 | 0.0344 / 0.0183 | 2.464 | 1.673 | 1.656 | 0.567 | 0.568 | 0.012 | 0.0 % |
| warehouse / aisle_to_aisle | astar | ideal | yes | yes | 17.00 | 17.00 | 0.0000 / 0.0000 | 0.0001 / 0.0000 | 2.461 | 1.670 | 1.670 | 0.567 | 0.568 | 0.000 | 0.0 % |
| warehouse / aisle_to_aisle | rrt_star | gusts | yes | yes | 15.50 | 15.51 | 0.0333 / 0.0197 | 0.0344 / 0.0182 | 2.613 | 1.520 | 1.546 | 0.715 | 0.712 | 0.018 | 0.0 % |
| warehouse / aisle_to_aisle | rrt_star | ideal | yes | yes | 15.50 | 15.51 | 0.0000 / 0.0000 | 0.0000 / 0.0000 | 2.613 | 1.578 | 1.578 | 0.715 | 0.716 | 0.000 | 0.0 % |
| urban_canyon / street_crossing | astar | gusts | yes | yes | 39.07 | 39.08 | 0.0333 / 0.0200 | 0.0344 / 0.0180 | 2.556 | 1.662 | 1.548 | 0.580 | 0.582 | 0.016 | 0.0 % |
| urban_canyon / street_crossing | astar | ideal | yes | yes | 39.07 | 39.08 | 0.0000 / 0.0000 | 0.0000 / 0.0000 | 2.531 | 1.537 | 1.537 | 0.580 | 0.581 | 0.000 | 0.0 % |
| urban_canyon / street_crossing | rrt_star | gusts | yes | yes | 49.71 | 49.72 | 0.0333 / 0.0201 | 0.0344 / 0.0180 | 2.914 | 2.000 | 1.917 | 0.452 | 0.452 | 0.024 | 0.6 % |
| urban_canyon / street_crossing | rrt_star | ideal | yes | yes | 49.71 | 49.72 | 0.0000 / 0.0000 | 0.0001 / 0.0000 | 2.910 | 1.940 | 1.940 | 0.452 | 0.452 | 0.000 | 0.0 % |
| urban_canyon / rooftop_landing | astar | gusts | yes | yes | 30.33 | 30.33 | 0.0333 / 0.0200 | 0.0344 / 0.0179 | 2.412 | 1.531 | 1.350 | 0.496 | 0.493 | 0.028 | 0.0 % |
| urban_canyon / rooftop_landing | astar | ideal | yes | yes | 30.33 | 30.33 | 0.0000 / 0.0000 | 0.0000 / 0.0000 | 2.397 | 1.354 | 1.354 | 0.496 | 0.496 | 0.000 | 0.0 % |
| urban_canyon / rooftop_landing | rrt_star | gusts | yes | yes | 32.80 | 32.80 | 0.0333 / 0.0203 | 0.0344 / 0.0180 | 2.581 | 1.464 | 1.542 | 0.484 | 0.498 | 0.026 | 0.0 % |
| urban_canyon / rooftop_landing | rrt_star | ideal | yes | yes | 32.80 | 32.80 | 0.0000 / 0.0000 | 0.0000 / 0.0000 | 2.581 | 1.581 | 1.581 | 0.484 | 0.485 | 0.000 | 0.0 % |
| industrial / ground_to_mezzanine | astar | gusts | yes | yes | 20.78 | 20.78 | 0.0333 / 0.0207 | 0.0344 / 0.0182 | 2.905 | 1.817 | 1.933 | 0.499 | 0.491 | 0.029 | 0.0 % |
| industrial / ground_to_mezzanine | astar | ideal | yes | yes | 20.78 | 20.78 | 0.0000 / 0.0000 | 0.0000 / 0.0000 | 2.910 | 1.940 | 1.940 | 0.499 | 0.500 | 0.000 | 0.0 % |
| industrial / ground_to_mezzanine | rrt_star | gusts | yes | yes | 23.56 | 23.56 | 0.0333 / 0.0201 | 0.0344 / 0.0180 | 2.091 | 1.601 | 1.605 | 0.466 | 0.467 | 0.002 | 0.0 % |
| industrial / ground_to_mezzanine | rrt_star | ideal | yes | yes | 23.56 | 23.56 | 0.0000 / 0.0000 | 0.0000 / 0.0000 | 2.103 | 1.584 | 1.584 | 0.466 | 0.467 | 0.000 | 0.0 % |
| industrial / cross_hall | astar | gusts | yes | yes | 22.88 | 22.89 | 0.0333 / 0.0204 | 0.0344 / 0.0182 | 2.659 | 1.302 | 1.451 | 0.560 | 0.552 | 0.006 | 0.0 % |
| industrial / cross_hall | astar | ideal | yes | yes | 22.88 | 22.89 | 0.0000 / 0.0000 | 0.0000 / 0.0000 | 2.662 | 1.485 | 1.485 | 0.560 | 0.561 | 0.000 | 0.0 % |
| industrial / cross_hall | rrt_star | gusts | yes | yes | 26.18 | 26.19 | 0.0333 / 0.0200 | 0.0344 / 0.0179 | 2.709 | 1.404 | 1.536 | 0.458 | 0.462 | 0.021 | 0.0 % |
| industrial / cross_hall | rrt_star | ideal | yes | yes | 26.18 | 26.19 | 0.0000 / 0.0000 | 0.0000 / 0.0000 | 2.733 | 1.568 | 1.568 | 0.458 | 0.459 | 0.000 | 0.0 % |
| multilevel / ground_to_top | astar | gusts | yes | yes | 53.78 | 53.78 | 0.0333 / 0.0198 | 0.0344 / 0.0181 | 2.727 | 1.501 | 1.354 | 0.500 | 0.502 | 0.014 | 0.0 % |
| multilevel / ground_to_top | astar | ideal | yes | yes | 53.78 | 53.78 | 0.0000 / 0.0000 | 0.0000 / 0.0000 | 2.743 | 1.356 | 1.356 | 0.500 | 0.501 | 0.000 | 0.0 % |
| multilevel / ground_to_top | rrt_star | gusts | yes | yes | 54.96 | 54.96 | 0.0333 / 0.0197 | 0.0344 / 0.0179 | 2.854 | 1.386 | 1.289 | 0.463 | 0.463 | 0.012 | 0.0 % |
| multilevel / ground_to_top | rrt_star | ideal | yes | yes | 54.96 | 54.96 | 0.0000 / 0.0000 | 0.0001 / 0.0000 | 2.868 | 1.278 | 1.278 | 0.463 | 0.464 | 0.000 | 0.0 % |
| multilevel / ground_to_second | astar | gusts | yes | yes | 20.94 | 20.95 | 0.0333 / 0.0208 | 0.0344 / 0.0182 | 2.796 | 1.332 | 1.308 | 0.539 | 0.559 | 0.027 | 0.0 % |
| multilevel / ground_to_second | astar | ideal | yes | yes | 20.94 | 20.95 | 0.0000 / 0.0000 | 0.0000 / 0.0000 | 2.796 | 1.289 | 1.289 | 0.539 | 0.540 | 0.000 | 0.0 % |
| multilevel / ground_to_second | rrt_star | gusts | yes | yes | 21.01 | 21.02 | 0.0333 / 0.0208 | 0.0344 / 0.0182 | 2.758 | 1.237 | 1.064 | 0.576 | 0.572 | 0.026 | 0.0 % |
| multilevel / ground_to_second | rrt_star | ideal | yes | yes | 21.01 | 21.02 | 0.0000 / 0.0000 | 0.0000 / 0.0000 | 2.771 | 1.032 | 1.032 | 0.576 | 0.577 | 0.000 | 0.0 % |
| narrow_1200 / through_window | astar | gusts | yes | yes | 5.15 | 5.16 | 0.0300 / 0.0220 | 0.0344 / 0.0201 | 2.912 | 1.905 | 1.770 | 0.599 | 0.594 | 0.018 | 0.0 % |
| narrow_1200 / through_window | astar | ideal | yes | yes | 5.15 | 5.16 | 0.0000 / 0.0000 | 0.0000 / 0.0000 | 2.910 | 1.738 | 1.738 | 0.599 | 0.600 | 0.000 | 0.0 % |
| narrow_1200 / through_window | rrt_star | gusts | yes | yes | 5.15 | 5.16 | 0.0300 / 0.0220 | 0.0344 / 0.0201 | 2.912 | 1.905 | 1.770 | 0.599 | 0.594 | 0.018 | 0.0 % |
| narrow_1200 / through_window | rrt_star | ideal | yes | yes | 5.15 | 5.16 | 0.0000 / 0.0000 | 0.0000 / 0.0000 | 2.910 | 1.738 | 1.738 | 0.599 | 0.600 | 0.000 | 0.0 % |
| narrow_900 / through_window | astar | – | not simulated: Phase 2 pipeline plan_failed (no trajectory) | | | | | | | | | | | | |
| narrow_900 / through_window | rrt_star | – | not simulated: Phase 2 pipeline plan_failed (no trajectory) | | | | | | | | | | | | |
| narrow_500 / through_window | astar | – | not simulated: Phase 2 pipeline plan_failed (no trajectory) | | | | | | | | | | | | |
| narrow_500 / through_window | rrt_star | – | not simulated: Phase 2 pipeline plan_failed (no trajectory) | | | | | | | | | | | | |

- `gusts`: 18 runs, goal reached 18/18, safety pass 18/18, collisions 0 expected / 0 observed; max position error 0.0333 m (largest RMS 0.0220 m), max velocity error 0.0344 m/s, max speed 2.914 m/s, max |a_cmd| 2.000 m/s², max |a_net| 1.933 m/s², min executed clearance 0.452 m, max goal error 0.029 m.
- `ideal`: 18 runs, goal reached 18/18, safety pass 18/18, collisions 0 expected / 0 observed; max position error 0.0000 m (largest RMS 0.0000 m), max velocity error 0.0001 m/s, max speed 2.910 m/s, max |a_cmd| 1.940 m/s², max |a_net| 1.940 m/s², min executed clearance 0.452 m, max goal error 0.000 m.
- Tightest run (gusts): urban_canyon/street_crossing/rrt_star: reference clearance ≥ 0.452 m, executed ≥ 0.452 m, max tracking error 0.0333 m.
- Pipelines reproduce Phase 2 (same duration and simplified length): 22/22.

## 2. Obstacle-update scenarios with the simulated drone

Outcome = what the simulated run actually did; prediction = the Phase 2 supervisor's classification made at detection from the actual state (it assumes exact 3 m/s² braking and no disturbance).

| scenario | profile | expected (Phase 2) | predicted | simulated outcome | matches | goal | safety | speed at detect [m/s] | stop dist baseline [m] | predicted stop clearance [m] | executed clearance after update [m] | actual vs predicted stop point [m] | hold clearance [m] | replan compute (wall) [s] | simulated hold [s] | exec T [s] | pos err max [m] |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| A_warehouse_aisle_blocked | gusts | replanned | safe_stop | **replanned** | yes | yes | yes | 1.01 | 0.43 | 0.576 | 0.569 | 0.032 | 0.633 | 0.58 | 0.59 | 18.96 | 0.0333 |
| A_warehouse_aisle_blocked | ideal | replanned | safe_stop | **replanned** | yes | yes | yes | 1.02 | 0.43 | 0.576 | 0.577 | 0.000 | 0.605 | 0.57 | 0.57 | 18.92 | 0.0000 |
| B_urban_scaffold | gusts | replanned | safe_stop | **replanned** | yes | yes | yes | 1.89 | 1.22 | 1.324 | 0.470 | 0.040 | 1.357 | 13.57 | 13.58 | 59.15 | 0.0398 |
| B_urban_scaffold | ideal | replanned | safe_stop | **replanned** | yes | yes | yes | 1.90 | 1.22 | 1.324 | 0.480 | 0.000 | 1.325 | 13.48 | 13.49 | 58.99 | 0.0000 |
| C_narrow_window_sealed | gusts | no_route_holding | safe_stop | **no_route_holding** | yes | **no** | yes | 1.15 | 0.35 | 1.999 | 1.990 | 0.045 | 1.996 | 0.18 | 0.18 | 4.75 | 0.0257 |
| C_narrow_window_sealed | ideal | no_route_holding | safe_stop | **no_route_holding** | yes | **no** | yes | 1.14 | 0.35 | 1.999 | 2.000 | 0.000 | 2.000 | 0.18 | 0.19 | 4.73 | 0.0000 |
| D1_too_close_collision | gusts | unavoidable_collision | unavoidable_collision | **unavoidable_collision** (contact at 2.45 s, 2.41 m/s, 5.1 mm) | yes | **no** | **no** | 2.78 | 1.94 | -0.001 | 0.245 | – | – | – | – | 2.46 | 0.0264 |
| D1_too_close_collision | ideal | unavoidable_collision | unavoidable_collision | **unavoidable_collision** (contact at 2.46 s, 2.36 m/s, 13.9 mm) | yes | **no** | **no** | 2.79 | 1.94 | -0.001 | 0.236 | – | – | – | – | 2.47 | 0.0000 |
| D2_too_close_margin | gusts | stop_margin_violation | stop_margin_violation | **unavoidable_collision** (contact at 3.23 s, 0.22 m/s, 0.3 mm) | **no** | **no** | **no** | 2.78 | 1.94 | 0.328 | 0.250 | – | – | – | – | 3.24 | 0.0264 |
| D2_too_close_margin | ideal | stop_margin_violation | stop_margin_violation | **stop_margin_violation** | yes | **no** | **no** | 2.79 | 1.94 | 0.328 | 0.329 | 0.000 | 0.329 | – | – | 6.25 | 0.0000 |
| D3_far_enough | gusts | replanned | safe_stop | **replanned** | yes | yes | yes | 2.78 | 1.94 | 0.599 | 0.590 | 0.092 | 0.779 | 0.04 | 0.04 | 9.58 | 0.0264 |
| D3_far_enough | ideal | replanned | safe_stop | **replanned** | yes | yes | yes | 2.79 | 1.94 | 0.599 | 0.600 | 0.000 | 0.748 | 0.04 | 0.05 | 9.58 | 0.0000 |
| E_warehouse_retry | gusts | replanned_after_retry | safe_stop | **replanned_after_retry** | yes | yes | yes | 1.01 | 0.43 | 0.576 | 0.569 | 0.032 | 0.633 | 0.60 | 0.60 | 18.97 | 0.0333 |
| E_warehouse_retry | ideal | replanned_after_retry | safe_stop | **replanned_after_retry** | yes | yes | yes | 1.02 | 0.43 | 0.576 | 0.577 | 0.000 | 0.605 | 0.58 | 0.59 | 18.94 | 0.0000 |

Negative predicted clearances denote predicted penetration (certified lower bound). Executed clearance for collision runs is the minimum up to the terminating contact step.

### Event timelines (default profile)

- **A_warehouse_aisle_blocked**: 0.00 MISSION_START → 4.00 OBSTACLE_APPEARED → 4.20 OBSTACLE_DETECTED → 4.20 CONFLICT → 4.30 BRAKING → 4.66 STOPPED → 4.66 HOLD_VERIFIED → 4.66 REPLANNING → 5.25 REPLAN_SUCCEEDED → 5.25 RESUMING → 18.96 GOAL_REACHED
- **B_urban_scaffold**: 0.00 MISSION_START → 8.00 OBSTACLE_APPEARED → 8.20 OBSTACLE_DETECTED → 8.20 CONFLICT → 8.30 BRAKING → 8.91 STOPPED → 8.91 HOLD_VERIFIED → 8.91 REPLANNING → 22.49 REPLAN_SUCCEEDED → 22.49 RESUMING → 59.15 GOAL_REACHED
- **C_narrow_window_sealed**: 0.00 MISSION_START → 0.80 OBSTACLE_APPEARED → 1.00 OBSTACLE_DETECTED → 1.00 CONFLICT → 1.10 BRAKING → 1.57 STOPPED → 1.57 HOLD_VERIFIED → 1.57 REPLANNING → 1.75 REPLAN_ATTEMPT_FAILED → 1.75 NO_ROUTE
- **D1_too_close_collision**: 0.00 MISSION_START → 2.00 OBSTACLE_APPEARED → 2.20 OBSTACLE_DETECTED → 2.20 CONFLICT → 2.30 BRAKING → 2.41 SAFETY_VIOLATION → 2.45 COLLISION
- **D2_too_close_margin**: 0.00 MISSION_START → 2.00 OBSTACLE_APPEARED → 2.20 OBSTACLE_DETECTED → 2.20 CONFLICT → 2.30 BRAKING → 3.02 SAFETY_VIOLATION → 3.23 COLLISION
- **D3_far_enough**: 0.00 MISSION_START → 2.00 OBSTACLE_APPEARED → 2.20 OBSTACLE_DETECTED → 2.20 CONFLICT → 2.30 BRAKING → 3.31 STOPPED → 3.31 HOLD_VERIFIED → 3.31 REPLANNING → 3.35 REPLAN_SUCCEEDED → 3.35 RESUMING → 9.58 GOAL_REACHED
- **E_warehouse_retry**: 0.00 MISSION_START → 4.00 OBSTACLE_APPEARED → 4.20 OBSTACLE_DETECTED → 4.20 CONFLICT → 4.30 BRAKING → 4.66 STOPPED → 4.66 HOLD_VERIFIED → 4.66 REPLANNING → 4.82 REPLAN_ATTEMPT_FAILED → 5.26 REPLAN_SUCCEEDED → 5.26 RESUMING → 18.97 GOAL_REACHED

## 3. Computation time by stage (wall clock, this machine)

Planner search, trajectory pipeline, supervisor and simulation are timed separately. The simulated hold during replanning equals the measured replanning compute rounded up to dt. Viewer logging and playback speed are not part of these numbers.

| run | nominal planning [s] | assess (conflict + stop prediction) [s] | map rebuild [s] | replan attempts: planner / status / compute [s] | simulation loop [s] | of which safety monitor [s] |
|---|---|---|---|---|---|---|
| A_warehouse_aisle_blocked | 0.58 | 0.0075 | 0.14 | astar / success / 0.44 | 0.69 | 0.05 |
| B_urban_scaffold | 11.28 | 0.0112 | 1.07 | astar / success / 12.50 | 13.90 | 0.16 |
| C_narrow_window_sealed | 0.01 | 0.0044 | 0.00 | astar / plan_failed / 0.17 | 0.21 | 0.01 |
| D1_too_close_collision | 0.01 | 0.0031 | – | – | 0.02 | 0.01 |
| D2_too_close_margin | 0.01 | 0.0026 | – | – | 0.02 | 0.01 |
| D3_far_enough | 0.01 | 0.0043 | 0.00 | astar / success / 0.03 | 0.09 | 0.02 |
| E_warehouse_retry | 0.62 | 0.0073 | 0.14 | rrt_star / plan_failed / 0.01; astar / success / 0.44 | 0.71 | 0.05 |
| nav warehouse/cross_warehouse/astar | 2.45 (shortcut 0.058, generation 0.005, validation 0.022) | – | – | – | 0.12 | 0.06 |
| nav warehouse/cross_warehouse/rrt_star | 4.09 (shortcut 0.014, generation 0.006, validation 0.030) | – | – | – | 0.13 | 0.07 |
| nav warehouse/aisle_to_aisle/astar | 0.59 (shortcut 0.032, generation 0.011, validation 0.028) | – | – | – | 0.09 | 0.05 |
| nav warehouse/aisle_to_aisle/rrt_star | 4.22 (shortcut 0.007, generation 0.004, validation 0.023) | – | – | – | 0.08 | 0.04 |
| nav urban_canyon/street_crossing/astar | 11.34 (shortcut 0.078, generation 0.011, validation 0.066) | – | – | – | 0.20 | 0.10 |
| nav urban_canyon/street_crossing/rrt_star | 2.90 (shortcut 0.018, generation 0.039, validation 0.159) | – | – | – | 0.26 | 0.13 |
| nav urban_canyon/rooftop_landing/astar | 1.10 (shortcut 0.059, generation 0.010, validation 0.044) | – | – | – | 0.16 | 0.08 |
| nav urban_canyon/rooftop_landing/rrt_star | 2.89 (shortcut 0.012, generation 0.008, validation 0.046) | – | – | – | 0.17 | 0.09 |
| nav industrial/ground_to_mezzanine/astar | 1.82 (shortcut 0.051, generation 0.016, validation 0.118) | – | – | – | 0.14 | 0.08 |
| nav industrial/ground_to_mezzanine/rrt_star | 5.48 (shortcut 0.014, generation 0.011, validation 0.073) | – | – | – | 0.15 | 0.08 |
| nav industrial/cross_hall/astar | 1.22 (shortcut 0.026, generation 0.006, validation 0.053) | – | – | – | 0.14 | 0.08 |
| nav industrial/cross_hall/rrt_star | 4.99 (shortcut 0.011, generation 0.019, validation 0.088) | – | – | – | 0.19 | 0.11 |
| nav multilevel/ground_to_top/astar | 4.23 (shortcut 0.118, generation 0.012, validation 0.076) | – | – | – | 0.33 | 0.19 |
| nav multilevel/ground_to_top/rrt_star | 5.42 (shortcut 0.017, generation 0.014, validation 0.072) | – | – | – | 0.33 | 0.19 |
| nav multilevel/ground_to_second/astar | 2.49 (shortcut 0.051, generation 0.003, validation 0.020) | – | – | – | 0.13 | 0.07 |
| nav multilevel/ground_to_second/rrt_star | 5.90 (shortcut 0.009, generation 0.005, validation 0.022) | – | – | – | 0.13 | 0.08 |
| nav narrow_1200/through_window/astar | 0.01 (shortcut 0.008, generation 0.001, validation 0.009) | – | – | – | 0.03 | 0.01 |
| nav narrow_1200/through_window/rrt_star | 3.84 (shortcut 0.002, generation 0.001, validation 0.009) | – | – | – | 0.03 | 0.02 |

## 4. Planner exploration data for the viewer (narrow deterministic reruns)

| viewer entry | A* expansions | A* rerun = visualised plan | RRT* nodes / rewires / first solution it. | replanning reruns |
|---|---|---|---|---|
| multilevel_navigation | 269188 | yes | 4826 / 5884 / 5898 | – |
| warehouse_navigation | 33234 | yes | 6231 / 9483 / 104 | – |
| warehouse_replan | 33234 | yes | 6231 / 9483 / 104 | astar: matches |
| urban_replan | 484229 | yes | 5906 / 7919 / 2685 | astar: matches |
| no_route | 149 | yes | 6486 / 4357 / 4453 | astar: matches |
| unsafe_stop | 149 | yes | 6486 / 4357 / 4453 | – |
| unsafe_margin | 149 | yes | 6486 / 4357 / 4453 | – |
| safe_stop_replan | 149 | yes | 6486 / 4357 / 4453 | astar: matches |
| replan_retry | 33234 | yes | 6231 / 9483 / 104 | rrt_star: matches, astar: matches |


# Phase 1 benchmark results

Robot clearance c = 0.25 + 0.1 m; resolution 0.2 m; A* 26-connected; RRT* 10000 iterations, step 2.5 m, goal bias 0.05, seeds [0, 1, 2, 3, 4].
Times are planner-only (map/field build excluded). 'Valid' = independent exact validation against ground truth AND voxel map (min clearance ≥ c).

## A* vs RRT*

| scene / task | straight [m] | A* status | A* valid | A* L [m] | A* t [s] | A* expanded | A* clr [m] | A* mem [MB] | RRT* valid | RRT* L mean±sd (best) [m] | RRT* t [s] | RRT* nodes | RRT* seg checks | RRT* 1st sol it (median) | RRT* clr min [m] | RRT* mem [MB] |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| warehouse / cross_warehouse | 31.1 | success | True | 37.38 | 3.27 | 117891 | 0.500 | 28.1 | 5/5 | 35.78±0.12 (35.67) | 6.26 | 6736 | 18499 | 349 | 0.374 | 1.5 |
| warehouse / aisle_to_aisle | 16.2 | success | True | 21.08 | 0.62 | 30033 | 0.500 | 12.8 | 5/5 | 20.36±0.27 (20.04) | 4.76 | 6746 | 19489 | 175 | 0.368 | 1.5 |
| urban_canyon / street_crossing | 53.7 | success | True | 64.28 | 17.10 | 415837 | 0.500 | 95.9 | 5/5 | 71.05±0.67 (70.09) | 3.41 | 6167 | 14775 | 1047 | 0.353 | 1.7 |
| urban_canyon / rooftop_landing | 42.4 | success | True | 46.97 | 1.35 | 50556 | 0.500 | 78.1 | 5/5 | 50.71±0.98 (49.17) | 3.43 | 6189 | 14920 | 135 | 0.352 | 1.7 |
| industrial / ground_to_mezzanine | 18.9 | success | True | 29.93 | 2.04 | 101637 | 0.500 | 20.2 | 5/5 | 28.82±0.35 (28.35) | 6.26 | 7560 | 27076 | 389 | 0.382 | 1.9 |
| industrial / cross_hall | 27.5 | success | True | 33.84 | 1.38 | 67794 | 0.500 | 25.4 | 5/5 | 31.90±0.34 (31.47) | 6.01 | 7594 | 27248 | 1087 | 0.368 | 1.5 |
| multilevel / ground_to_top | 18.1 | success | True | 81.43 | 4.62 | 267094 | 0.500 | 14.0 | 5/5 | 79.85±0.56 (78.80) | 6.85 | 5374 | 26645 | 6129 | 0.362 | 1.9 |
| multilevel / ground_to_second | 23.1 | success | True | 32.37 | 2.96 | 129341 | 0.500 | 17.0 | 5/5 | 31.29±0.17 (31.08) | 6.82 | 5742 | 27830 | 1162 | 0.356 | 1.9 |
| narrow_1200 / through_window | 8.0 | success | True | 8.06 | 0.01 | 149 | 0.500 | 1.0 | 5/5 | 8.06±0.04 (8.03) | 4.43 | 8434 | 16296 | 210 | 0.426 | 2.0 |
| narrow_900 / through_window | 8.0 | no_path | False | – | 0.20 | 14404 | – | 1.6 | 3/5 | 8.05±0.03 (8.03) | 4.23 | 5907 | 13392 | 3296 | 0.403 | 1.2 |
| narrow_500 / through_window | 8.0 | no_path | False | – | 0.20 | 14400 | – | 1.6 | 0/5 | – | 4.00 | 4282 | 11570 | – | – | 1.1 |

Returned paths: 57; independently validated: 57; planner checker agrees: 57.

## Resolution study

| scene / task | res [m] | voxels | A* status | A* L [m] | A* t [s] | A* clr [m] | RRT* valid | RRT* L mean [m] | RRT* t mean [s] | field build [s] |
|---|---|---|---|---|---|---|---|---|---|---|
| narrow_900 / through_window | 0.1 | 384000 | success | 8.03 | 0.01 | 0.400 | 2/3 | 8.06 | 5.00 | 0.14 |
| warehouse / aisle_to_aisle | 0.1 | 4800000 | success | 20.21 | 4.57 | 0.350 | 3/3 | 19.81 | 5.85 | 2.26 |
| narrow_900 / through_window | 0.2 | 48000 | no_path | – | 0.20 | – | 2/3 | 8.06 | 4.12 | 0.01 |
| warehouse / aisle_to_aisle | 0.2 | 600000 | success | 21.08 | 0.59 | 0.500 | 3/3 | 20.38 | 4.67 | 0.14 |
| narrow_900 / through_window | 0.4 | 6000 | no_path | – | 0.03 | – | 0/3 | – | 4.01 | 0.00 |
| warehouse / aisle_to_aisle | 0.4 | 75000 | success | 22.24 | 0.08 | 0.600 | 3/3 | 20.79 | 5.19 | 0.01 |

## A* connectivity study

| scene / task | connectivity | status | L [m] | t [s] | expanded |
|---|---|---|---|---|---|
| warehouse / cross_warehouse | 6 | success | 42.50 | 0.05 | 3897 |
| warehouse / cross_warehouse | 18 | success | 37.54 | 0.79 | 164374 |
| warehouse / cross_warehouse | 26 | success | 37.38 | 2.85 | 117891 |
| industrial / ground_to_mezzanine | 6 | success | 38.08 | 0.44 | 89011 |
| industrial / ground_to_mezzanine | 18 | success | 30.23 | 0.50 | 108442 |
| industrial / ground_to_mezzanine | 26 | success | 29.93 | 1.92 | 101637 |

## RRT* budget study (prefix checkpoints of fixed-seed runs)

| scene / task | iterations | solved | best cost mean±sd [m] | elapsed mean [s] | tree nodes mean |
|---|---|---|---|---|---|
| industrial / cross_hall | 2500 | 5/5 | 40.60±6.83 | 0.85 | 1593 |
| industrial / cross_hall | 5000 | 5/5 | 36.22±5.07 | 2.30 | 3589 |
| industrial / cross_hall | 10000 | 5/5 | 31.90±0.34 | 6.30 | 7594 |
| industrial / cross_hall | 20000 | 5/5 | 31.29±0.26 | 14.39 | 15558 |
| multilevel / ground_to_top | 2500 | 0/5 | – | 0.87 | 651 |
| multilevel / ground_to_top | 5000 | 1/5 | 89.19±0.00 | 2.15 | 1808 |
| multilevel / ground_to_top | 10000 | 5/5 | 79.85±0.56 | 6.65 | 5374 |
| multilevel / ground_to_top | 20000 | 5/5 | 77.73±0.32 | 14.89 | 12731 |

# Conservative stopping assessment applied to the saved Phase 3 D scenarios

Phase 1-3 results are unchanged; this table only adds the Phase 4 assessment. Clearances in m
(robot radius 0.25, nominal clearance 0.35). Bounds: [certified lower, upper valid for every admitted motion].

| run | Phase 2 prediction | Phase 3 actual | actual min clearance | Phase 4 class | bounds | D_max m | rho m | consistent |
|---|---|---|---|---|---|---|---|---|
| scn__D1_too_close_collision__gusts | unavoidable_collision | collision | 0.245..0.245 | collision_possible | [-0.345, 0.340] | 2.061 | 0.340 | True |
| scn__D1_too_close_collision__ideal | unavoidable_collision | collision | 0.236..0.236 | collision_unavoidable | [-0.005, 0.000] | 1.775 | 0.000 | True |
| scn__D2_too_close_margin__gusts | stop_margin_violation | collision | 0.250..0.250 | collision_possible | [-0.345, 0.874] | 2.061 | 0.340 | True |
| scn__D2_too_close_margin__ideal | stop_margin_violation | stop_margin_violation | 0.329..0.329 | collision_possible | [0.182, 0.385] | 1.775 | 0.000 | True |
| scn__D3_far_enough__gusts | safe_stop | replanned | 0.581..0.590 | collision_possible | [0.249, 0.933] | 2.061 | 0.340 | True |
| scn__D3_far_enough__ideal | safe_stop | replanned | 0.590..0.600 | safe_stop_feasible | [0.595, 0.600] | 1.775 | 0.000 | True |

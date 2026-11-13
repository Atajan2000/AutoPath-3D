# End-to-end test sessions (Webots + ROS 2)

`e2e_sessions.jsonl` has one line per Webots session started by
`ros2/autopath3d_ros/test/test_e2e_webots.py` (harness: `test/webots_session.py`). Each line holds:

- the harness record: start-up status and time, Webots exit code, launch exit code, cleanup steps
  and leftover processes;
- the key metrics of the flight result that the session wrote.

Values are copied from the local records. Home-directory paths have been replaced by `~`. Full
launch logs and per-run result files were not published (`results/README.md`).

| Suite | Code version | Scope | Result |
|---|---|---|---|
| `suite_20261111_151924` | release audit, after the harness rewrite | A, B, C1, C2 | 4 passed in 369 s |
| `suite_20261111_165048` | Phase 5 release candidate (code identical to this release) | A, B, C1, C2 | 4 passed in 330.7 s |
| `suite_20261111_165622`, `_165759`, `_165943` | same as above | A2 (RRT*), bounded repeat ×3 | 3 passed (100.7, 94.0, 93.9 s) |

All 11 sessions reached READY in 10.4–11.7 s. None had a start-up failure. No session left
processes behind. Further details:

- **Webots exit code.** It is 1 in every record: Webots was stopped at shutdown. This is not a
  start-up failure.
- **C1 in `suite_20261111_165048`.** The extern controllers did not stop on SIGINT, so the harness
  terminated its own process group (`sigterm_group`). This is a known `webots_ros2` teardown
  behaviour (`docs/PHASE4_REPORT.md` §8).
- **C2.** C2 ended in contact in both suites. This is the intended negative test.
- **A2.** The A2 repeats use the configured RRT* seed (`seed: 0`), so they show that the complete
  physics pipeline reproduces the same result: minimum clearance 0.449 m, tracking maximum
  0.075 m, goal error 0.031 m at the check. They are not a statistical evaluation across seeds.

Earlier sessions, recorded in `docs/RELEASE_AUDIT.md` §4, are not in this file:
- with the old harness, 4 of 7 Webots starts failed with exit code 5;
- the 5 start-up-only checks with the new harness were 5/5 READY.

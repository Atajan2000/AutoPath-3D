# AutoPath-3D Phase 4: ROS 2 Jazzy + Webots warehouse — final report

Date: 2026-11-11. Every number below was measured in the runs referenced; nothing is estimated
unless stated. Setup, launch commands and architecture: `ros2/README.md`.

**Scope statement.** This is a simulation integration. It does not demonstrate real-world flight
safety. The map is the known simulation geometry, not SLAM. Obstacles are scripted and "detected"
by the Webots supervisor with a fixed latency, not by onboard perception.

## 1. Outcome summary

| Demo | Scenario | Planner | Result (Webots physics) | Collisions | Min clearance* | Tracking max / RMS | Run |
|---|---|---|---|---|---|---|---|
| A | staging area → mezzanine landing (22.4 m, climb to 3.9 m) | A* | **GOAL_REACHED**, error 0.123 m at check, 0.002 m settled | 0 | 0.430 m | 0.132 / 0.024 m | `A_navigation/run_20261110_233401` |
| A2 | west aisle → aisle C around rack rows (19.1 m) | RRT* | **GOAL_REACHED**, error 0.030 m | 0 | 0.449 m | 0.075 / 0.022 m | `A2_around_racks/run_20261111_000540` |
| B | forklift drives into the cross aisle ahead | A* | brake → safe stop → replan → resume → **GOAL_REACHED** (0.057 m) | 0 | 0.479 m | 0.078 / 0.022 m | `B_replan/run_20261110_233951` |
| C1 | cage gate closes during the approach | A* | brake → safe stop → no route → **HOLDING_ABORTED (no_route_holding)**, hovering at 1.04 m clearance | 0 | 0.522 m | 0.040 / 0.029 m | `C1_no_route/run_20261110_234122` |
| C2 | load drops 0.55 m ahead at 0.77 m/s | A* | brake; stop **predicted unsafe** (both assessments); **physical contact** in Webots; **HOLDING_ABORTED (unavoidable_collision)** | 1 | 0.051 m | 0.019 / 0.013 m | `C2_unsafe_stop/run_20261110_234236` |

\* Minimum distance from the drone centre to the extracted collision geometry. The drone's physical
collision cylinder has a radius of 0.05 m; the planning envelope is 0.10 m (propeller tips 0.069 m).
The trajectory clearance requirement is 0.37 m; the nominal requirement is 0.20 m.

**What the clearances mean.** All values are distances from the drone centre:
- **Nominal clearance 0.20 m** = envelope 0.10 + safety margin 0.10.
- **Trajectory clearance 0.37 m** = nominal 0.20 + tracking allowance 0.17. Every reference
  trajectory is certified to keep this, so that a tracking deviation of up to 0.17 m still leaves
  the nominal clearance.
- **0.35 m in Phases 1–3** was the nominal clearance of the 0.25 m benchmark sphere (0.25 + 0.10).
  It does not apply to the Crazyflie, whose own nominal clearance is 0.20 m.
- **Measured minima of 0.43–0.52 m** (A–C1) lie above both 0.37 and 0.35. They track the
  certified trajectory minima (A 0.429, A2 0.436, B 0.459 after the replan, C1 0.521 m). The gap from the drone's physical
  collision cylinder to the geometry is therefore 0.38–0.47 m.

The C2 contact is the intended outcome of the unsafe-stop demo and is reported, not tuned away. It
lies inside the predicted bounds (§5). Results are in `results/phase4/webots/`. Earlier runs that
were invalidated by bugs found along the way are kept in `results/phase4/webots/superseded/`, with
the reason for each in §7.

Realtime factor: 0.90–0.98 (Webots GUI on, `basicTimeStep` 16 ms). Wall time per scenario,
including Webots start-up: 62–118 s.

Screenshot (one genuine Webots frame, exported by the scene manager at the end of a Demo B run):
`results/phase4/webots/screenshot_B_replan.jpg`. It shows the parked forklift blocking the cross
aisle, the original path (blue), the replanned path and the status overlay.

## 2. Versions, downloads, resources

| Item | Value |
|---|---|
| Host | Windows 11 Enterprise 10.0.26300, WSL2 `Ubuntu-24.04` (mirrored networking, DNS tunnelling) |
| ROS 2 | Jazzy: `ros-jazzy-ros-base` 0.11.0, `ros-dev-tools` 1.0.3, rclpy 7.1.12 |
| Webots | R2025a, the existing Windows install (no second copy) |
| webots_ros2 | `ros-jazzy-webots-ros2-driver` 2025.0.0, `ros-jazzy-webots-ros2-crazyflie` 2025.0.0 (released packages) |
| Python / numerics (WSL) | 3.12.3, numpy 1.26.4, scipy 1.11.4 (Ubuntu packages) |
| Installed | 465 apt packages; `/opt/ros/jazzy` 148 MB; WSL root disk in use 3.0 GB total |
| Webots asset cache | 44.8 MB (PROTOs and textures of tag R2025a, fetched by Webots through its proxy setting) |
| Not installed | RViz2, Gazebo and other simulators, Docker images, a second Ubuntu |
| Free space on C: afterwards | 86 GB (reserve 5 GB) |

Additional environment assets total about 0.2 GB, under the 500 MB budget. ROS was installed by
the user running `ros2/setup/install_ros2_jazzy.sh`, which does a dry-run size check before
installing. No firewall or security settings were changed. No remote was created and nothing was
published.

## 3. World

`ros2/autopath3d_ros/worlds/autopath_warehouse.wbt` is generated deterministically by
`tools/build_warehouse_world.py` from the installed official sample
`projects/samples/environments/factory/worlds/factory.wbt` (Cyberbotics, Apache-2.0).

**Kept from the factory:** the 20 × 16.4 m hall with 7 m walls, doors, mezzanine, stairs and
railings, pipes, valves, cabinets and barrels.

**Removed:** loose robots and tools (23 nodes).

**Added:**
- 8 rack segments with real collision bodies; their bracing is visual only.
- Official WoodenPallet, CardboardBox and WoodenBox cargo.
- 5 columns, an overhead ventilation duct, roof beams and workbenches.
- A storage cage with a 1.4 m doorway and a sliding gate.
- The official Forklift.
- The official Crazyflie.
- The `AutoPathSceneManager` supervisor robot.

The original mezzanine platform had no collision geometry and was given one.

**Collision extraction.** The scene manager traverses the running world and converts every
boundingObject into planning primitives:
- 662 records, with 0 unsupported geometry types.
- Oriented boxes, spheres and capsules become AABBs; vertical cylinders stay exact.

**Visual audit.** Each visual shape whose voxels are not covered by collision geometry is added to
the planning map as an obstacle: 89 parts, including the oil-barrel AABB corners and the visual mesh
under the stairs. Only the forklift's CAD visual cannot be audited; its collision body is 3 boxes up
to 1.78 m.

**Planning map.** It contains:
- 748 primitives on a 198 × 160 × 56 grid at 0.1 m resolution.
- 196,034 occupied voxels and 0 unknown voxels.

It is built in 0.12–0.26 s.

Licenses:

| Component | License |
|---|---|
| factory.wbt, and the derived `worlds/autopath_warehouse.wbt` (marked as modified) | Apache-2.0 |
| PROTOs | Webots asset license, "use only with Webots" (referenced by URL, not redistributed) |
| Bitcraze PID | MIT (imported at run time, not redistributed) |
| webots_ros2 | Apache-2.0 |
| Everything else in `ros2/autopath3d_ros` and `src/autopath3d` | AutoPath-3D, MIT |

Licensing map (added in Phase 5): `THIRD_PARTY_NOTICES.md`.

## 4. Drone, controller and characterisation

The drone is the Webots Crazyflie PROTO: 0.05 kg, collision cylinder r 0.05 m × h 0.03 m, motors
driven through Webots physics.

**Control stack:**
- **Third-party:** the attitude/altitude/velocity PID is Bitcraze's
  `pid_velocity_fixed_height_controller`, MIT, used unmodified from `webots_ros2_crazyflie`.
- **AutoPath (`tracking.py`, `crazyflie_bridge.py`):**
  - outer loop: `v_cmd = v_ref + 0.25·a_ref + 1.2·(p_ref − p)`, horizontal speed ≤ 1.0 m/s, altitude
    set-point = reference;
  - mode logic: takeoff, hold, track, brake, land;
  - rejection of late trajectories.

The drone is never positioned by the supervisor during flight. The only teleport is the initial
placement on the floor, which is refused if the drone is moving or above 0.05 m.

Characterisation (`results/phase4/webots/characterisation/characterisation_20261110_232847.json`;
flown in the free aisle, measured with ground truth):

| Flight | Max speed | Tracking error max / RMS |
|---|---|---|
| straight 0.4 / 0.6 / 0.8 / 1.0 m/s | 0.40 / 0.60 / 0.80 / 1.00 | 0.057 / 0.075 / 0.083 / 0.100, RMS 0.014–0.046 |
| straight 1.2 m/s requested | saturates at 1.0 (outer loop limit) | 0.355 |
| climb / descend 1.8 m at 0.5 m/s | 0.66 | 0.096 / 0.098 |
| diagonal up / down at 0.8 m/s | 0.80 | 0.085 / 0.085 |
| hover (4 s) | — | 0.021 (vertical) |

Braking after a brake command, measured at 0.6, 0.8 and 1.0 m/s:
- Deceleration onset: 0.208 s at every speed. ROS delivery of the command accounts for 0.016 s of it.
- Stop distance: 0.257, 0.372 and 0.504 m.
- Effective deceleration after onset: 1.36, 1.55 and 1.71 m/s².

**Values set from these measurements.** They are set once for all scenarios, never per scenario.

| Parameter | Value | Basis |
|---|---|---|
| `max_speed` | 0.8 m/s | 20 % below saturation |
| `max_acceleration` | 0.5 m/s² | — |
| tracking allowance | 0.17 m | 1.7 × the worst measured axis; see §7 for why not 0.15 |
| stopping assumptions: latency | 0.35 s | — |
| stopping assumptions: latency_min | 0.05 s | — |
| guaranteed braking | 1.2 m/s² | below the lowest measured 1.36 |
| best-case braking | 2.0 m/s² | — |
| disturbance | 0.1 m/s² | — |
| position error | 0.15 m | — |
| velocity error | 0.1 m/s | — |

A test checks that every measured stop lies inside [D_min, D_max] of these assumptions, and that
every characterised flight up to the speed limit stays inside the tracking allowance.

Tracking in the missions: maximum 0.132 m (Demo A, vertical, during the 3.9 m climb). That is inside
the 0.17 m allowance, which also holds in every other run.

## 5. Safety supervision and the conservative stopping assessment

When an obstacle update arrives (scene manager, 0.2 s after the obstacle reaches its final pose),
the supervisor:
1. **Decides whether to brake** with the Phase 2 step-1 criterion, unchanged: the continuous-time
   certificate of the remaining trajectory against the new obstacle at the trajectory clearance.
   Since §7 item 6 this is evaluated first. It takes 1–12 ms, and in all three event demos the brake
   left in the same simulation step as the update.
2. **Records** the Phase 4 conservative stopping assessment
   (`src/autopath3d/replanning/stopping_assessment.py`) and the unchanged Phase 2 `assess_update`
   prediction, which takes 0.5–0.9 s of wall time. A test and every run (`conflict_agrees_with_phase2`)
   confirm that the brake trigger equals Phase 2's conflict verdict.
3. **Verifies the stop** after `AT_REST` from the measured history against the updated map, then
   either replans from the measured rest point (unchanged `replan_from_rest`) or holds.

| Run | Speed at detection | Conservative class | Certified bounds [lower, upper] | Phase 2 prediction | Measured min clearance (appear → rest) | Consistent |
|---|---|---|---|---|---|---|
| B | 0.685 m/s | safe_stop_feasible | [0.588, 1.432] m | safe stop, 0.980–0.981 m | 0.990 m | yes |
| C1 | 0.647 m/s | safe_stop_feasible | [0.277, 1.396] m | safe stop, 1.009 m | 1.035 m | yes |
| C2 | 0.773 m/s | **collision_possible** | [−0.378, 0.608] m | **unavoidable_collision** | 0.051 m, **contact** | yes |

Stop distances from the brake command: 0.369 m (B), 0.290 m (C1) and 0.294 m (C2). In C2 the
contact happened 0.66 s after the update, during braking, at 0.44 m along the path from the
obstacle's appearance. That matches Phase 2's predicted distance-to-contact of 0.438 m.

The Phase 3 retro-analysis (`results/phase4/stopping_assessment_phase3.md`) is unchanged. Under
gusts, the conservative assessment classifies D2 as `collision_possible`, consistent with the
grazing contact that Phase 3 measured. The Phase 2 nominal prediction (`stop_margin_violation`)
did not capture it.

**Replanning.**
- **B:** A* replan with full validation: 2.43 s of sim time from rest to the new trajectory (A* 0.36 s,
  validation 0.95 s), then resume. The new trajectory's certified minimum clearance is 0.459 m.
- **C1:** A* exhausted the free space at 0.1 m resolution in 19.5 s and reported no route. The drone
  kept holding at 1.04 m clearance.

## 6. Tests

| Suite | Where | Result |
|---|---|---|
| Core Phase 1–3 + stopping assessment (`tests/`) | Windows venv | 112 passed |
| Core (`tests/`) | WSL, Ubuntu numpy 1.26 / scipy 1.11 | 108 passed, 4 viewer tests skipped (no rerun in WSL) |
| Package pure modules + offline warehouse (`ros2/autopath3d_ros/test`) | Windows and WSL | 22 passed |
| **End-to-end Webots + ROS 2** (`test_e2e_webots.py`, `AUTOPATH_E2E=1`) | WSL + Windows Webots | **4 passed** (A, B, C1, C2; 356 s), 2026-11-11 |

**The offline warehouse tests** run on the geometry dumped from the live simulation and on the
measured characterisation. They cover:
- extraction completeness, and zero unknown voxels in the map;
- mission endpoints at the trajectory clearance;
- A* routes passing the full trajectory pipeline (shortcut, generation, validation);
- the B detour after the forklift moves, and C1 having no route with the gate closed;
- equivalence of the brake trigger with Phase 2;
- the measured stops lying inside the stopping assumptions, and the tracking allowance covering
  the measured tracking error;
- a regression test for the A*/certification tie at c = 0.35, see §7.

### 6.1 End-to-end runs

Each test starts Webots, both controllers and the full stack, flies the scenario under physics and
asserts on the measured result:
- **A, B:** goal reached, no contact.
- **C1:** `no_route_holding`.
- **C2:** `HOLDING_ABORTED`; contact is allowed.

- First two full-suite runs: A, B and C2 passed. C1 failed both times; standalone it passed.
- Cause: the C1 test started while the previous Webots instance still held TCP port 1234, so the
  new instance moved to another port and the controllers never connected.
- Fix: a port per run (`AUTOPATH_WEBOTS_PORT`), plus waiting for the previous Webots to exit.
- **Final full-suite run after the fix: 4 passed (A, B, C1, C2) in 356 s.**
- Release audit (`docs/RELEASE_AUDIT.md` §4): that run kept no evidence and later suite runs hit
  intermittent Webots start-up exits (code 5). After a harness rewrite (`test/webots_session.py`)
  the suite passed 4/4 again in one run with its evidence kept; the start-up root cause is not
  isolated.

## 7. Problems found and fixed during integration

1. **Fast DDS over UDP between WSL processes delivered nothing.** The fix is SHM-only transport with
   SUBNET discovery (`ros2/env.sh`). LOCALHOST discovery combined with SHM-only discovers nothing.
2. **Orphaned `webots_ros2` driver processes.** Launch SIGTERMs `webots-controller`, whose `driver`
   child survives and stays on the DDS graph. Measured effect: reliable topics delayed by 4–6 s
   and samples lost. That is how Demo A's first trajectory reached the bridge 4 s late and started
   with a 0.42 m error (run moved to `superseded/`). Fixed on three levels:
   - a launch shutdown hook kills these drivers, and `ros2/scripts/cleanup.sh` does it by hand;
   - the bridge rejects a trajectory received after its start time (`TRAJECTORY_LATE`);
   - the supervisor then re-times the trajectory, at most 3 times.

   Measured after the fix: delivery in 4–25 ms for trajectories of 3–200 knots.
3. **A tie between A* and the segment certificate (core limitation, reported and not patched).**
   - The checker's clearance is a lower bound to blocked voxel cubes. From A* lattice nodes (cell
     centres) it takes only the values res/2 · √(sum of odd squares); at 0.1 m, c = 0.35 = 7 × 0.05
     is one of them. *(Corrected by the release audit, 2026-11-11: the original text attributed
     this to the warehouse faces lying on a 0.05 m grid. A synthetic corridor with every face off
     that grid shows the same tie, so it depends only on c and the resolution.)*
   - A* accepts nodes at exactly c (closed criterion, exact corner-cutting rule). The unchanged
     Lipschitz segment certificate can never certify an edge on which the bound stays flat at
     exactly c. The two certify the same predicate, but the second is incomplete at the boundary.
     The failure is fail-safe: a safe path is rejected, and no unsafe path is accepted.
   - Result: at c = 0.35 the shortcut stage rejected the A* path (Demo A, first attempt). The
     Phase 1 resolution study had already produced an A* path at clearance exactly 0.350 m at
     0.1 m. Phase 1 has no Lipschitz shortcut stage, so it was harmless there.
   - Fix: the tracking allowance 0.15 → 0.17 (c = 0.37). 0.37 is not a tie value; the nearest are
     0.354 and 0.381. 0.15 would have covered the measured tracking error, so the extra 0.02 m is
     the tie workaround, not a tracking requirement.
   - Remaining limitation: the core pair rejects A* paths whenever c is a tie value for the chosen
     resolution, on any map. It is not patched in the core because a margin in A* would change
     Phase 1 results. Regression tests pin the behaviour (`test_core_tie_limitation_at_c_035_is_documented`,
     `test_tie_depends_on_c_and_resolution_only`) and guard the shipped configuration
     (`test_trajectory_clearance_is_not_a_lattice_tie_value`).
4. **The 1.0 m cage doorway had no certifiable lattice node at c = 0.37** after conservative
   voxelisation of the visual-only jamb posts. The door was widened to 1.4 m and its posts were
   given collision geometry; the gate was widened to match.
5. **Altitude over-lead.** In Demo A the vertical error followed `0.42·vz − 0.33·az` (residual
   0.013 m). The 0.4 s altitude feed-forward lead made the drone run ahead of the reference, by
   0.20 m in descents. With the lead set to 0, the vertical error in Demo A fell from 0.203 m to
   0.128 m. Vertical and diagonal runs were added to the characterisation.
6. **The brake decision waited 0.74 s** for the Phase 2 stop prediction (first B run). This
   exceeded the 0.35 s latency the conservative assessment assumes. The node now brakes on the
   step-1 certificate and computes the prediction afterwards; measured decision latency is now
   0–0.016 s of sim time.
7. Two mission start points (C1, C2) were below the trajectory clearance from columns; caught by the
   offline tests and moved. The characterisation start placement was silently refused; the threshold
   was fixed, and refusals are now errors. One-shot requests now wait for DDS subscriber matching.

## 8. Limitations

- **Map:** known map from the simulation geometry, not SLAM or sensing. There are no unknown regions
  in this world, though the core still treats UNKNOWN as occupied.
- **Ideal sensing and no wind:** GPS is noise-free and there is no wind in Webots, so tracking and
  braking were measured without disturbances. The stopping assumptions include a
  0.1 m/s² disturbance bound and 0.15 m position error, but these are not validated against real
  disturbances.
- **Obstacles:** scripted and "detected" with a fixed 0.2 s latency. The forklift moves kinematically
  (not a physics vehicle). The dropped load appears instantly.
- **Single drone, fixed yaw:** the outer loop does not use yaw for planning.
- **Webots-run statistics:** each final scenario was flown once for the reported numbers, and A, B
  and C2 again in each end-to-end run. Runs are deterministic up to ROS timing, so these are single
  observations, not distributions.
- **Planning time:** A* at 0.1 m takes 13–21 s for the 22 m Demo A route and 19.5 s to prove C1 has
  no route (Python). The drone holds safely meanwhile; it is not a real-time replanner.
- **Phase 2 `assess_update`:** takes 0.5–0.9 s of wall time; now off the brake path, record only.
- **Teardown:** the stock `webots_ros2` driver process can crash (`double free`) on Ctrl-C after
  Webots exits. Harmless; covered by the cleanup.
- **Not visualised in ROS:** RViz2 was not installed. Visual status is the Webots overlay, the drawn
  paths and the screenshot.

## 9. Phase 5 readiness

**Ready:**
- standard ROS 2 interfaces: GetPlan, MultiDOFJointTrajectory, Detection3DArray, Odometry, TF and
  `/clock`;
- one launch file per scenario, with self-recording results;
- an opt-in end-to-end test;
- measured controller limits feeding a conservative safety assessment;
- the extraction and map pipeline, validated against the live world.

**Recommended next steps:**
- **Sensing:** replace the known map with sensor-based mapping (depth camera / range finder →
  occupancy with UNKNOWN), and replace supervisor-side detection with perception.
- **Disturbances:** add wind and GPS noise in Webots, then re-characterise and validate the stopping
  bounds statistically (repeated runs).
- **Performance:** move A* to a compiled implementation, or plan at a coarser resolution with
  refinement, to bring replanning under 1 s.
- **Core tie case:** resolve the A* / certificate tie in the core, for example with a planning slack
  in A*, under a regression review of Phase 1–3 results.
- **Testing:** run the end-to-end suite headless (`gui:=false`) in CI on a Linux Webots.

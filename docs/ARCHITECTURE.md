# Architecture

## Conventions

- **World frame `W`**: right-handed, x/y horizontal, **z up**, metres. The ground is the
  plane `z = 0`. Times are in seconds.
- **Scene bounds** `[lo, hi]` are an axis-aligned box. They act as physical walls,
  floor and ceiling (warehouse, building) or as a geofence (urban canyon). Leaving the
  bounds counts as a collision.
- **Voxel grid**: origin `o = lo`, resolution `r`, shape `(nx, ny, nz)` with
  `n = (hi - lo) / r` (bounds must be integer multiples of `r`). Voxel `(i, j, k)` is the
  closed cube `[o + i r, o + (i+1) r]` per axis; its centre is `o + (i + ½) r`.
  - `world_to_grid(p) = floor((p - o) / r)`
  - `grid_to_world(idx) = o + (idx + ½) r`
- **Voxel states**: `FREE = 0`, `OCCUPIED = 1`, `UNKNOWN = 2` (int8).

## Robot geometry

The quadrotor is modelled by a conservative **bounding sphere** of radius
`robot_radius` (frame + propellers) plus a `safety_margin`. The *required clearance* is
`c = robot_radius + safety_margin` (default 0.25 + 0.10 = 0.35 m). A configuration
(robot centre `p`) is valid iff `dist(p, blocked ∪ outside-bounds) ≥ c`. Orientation is
ignored, which is valid because the sphere encloses the vehicle in every attitude.

## Pipeline (Phase 1 scope)

```
Scene (analytic boxes / vertical cylinders, ground truth)
  └─ OccupancyGrid.from_scene     conservative voxelisation (voxel occupied iff its
  │                               interior intersects an obstacle's interior)
  └─ ClearanceField               exact distance from every voxel centre to the union
  │                               of blocked voxel cubes (separable min-convolution)
  └─ CollisionChecker             point / segment / path checks for the robot sphere
        ├─ AStarPlanner           lattice search over voxel centres
        └─ RRTStarPlanner         continuous sampling-based search
  └─ evaluation.validation        independent exact checks (ground truth + voxel map)
```

Algorithmic modules do not import visualisation, simulation or ROS code.

## Occupancy and unknown space

- Synthetic scenes are **known environments**: every voxel not touching an obstacle is
  FREE. Scenes may also declare `unknown` regions (UNKNOWN voxels) to emulate unobserved
  space.
- Planning uses the **blocked set** = OCCUPIED ∪ UNKNOWN (default
  `treat_unknown_as_occupied: true`). Treating unknown as free is possible only by an
  explicit flag and is documented as unsafe.
- Voxelisation is conservative: every obstacle point lies in some occupied closed cube,
  so a configuration that is safe w.r.t. the voxel map is safe w.r.t. the true geometry.
  The price is up to one voxel of lost free space per obstacle face.

## Collision checking (shared by both planners)

Let `d(p)` be the true distance from `p` to the blocked cubes and the bounds. `d` is
1-Lipschitz. The checker evaluates a **rigorous lower bound** `ℓ(p) ≤ d(p)`:

1. `F(v)`: exact (capped) distance from each voxel centre to the blocked cubes,
   computed for the whole grid by a separable per-axis min-convolution with kernel
   `g(t) = max(|t| − ½, 0)²` (exact for cubes, unlike a centre-to-centre EDT).
2. Fast bound: `max over the 8 surrounding centres v of F(v) − ‖p − c_v‖` (Lipschitz).
3. Near obstacles (fast bound `< c + r`) the bound is refined with exact point-to-cube
   distances to nearby *surface* blocked voxels (KD-tree over their centres).
4. Exact distance to the bounds; points inside a blocked voxel get `ℓ = 0`.

**Segment check** (no fixed-step point sampling assumption): sample the segment, require
`ℓ ≥ c` at samples, and certify every sub-interval of length `l` by
`(ℓ_i + ℓ_{i+1} − l) / 2 ≥ c` (Lipschitz). Uncertified intervals are bisected down to a
minimum length; anything still uncertified is rejected (conservative). Thin obstacles
between samples therefore cannot be missed.

## Planner A — 3D A*

- Graph: valid voxel centres (`F ≥ c` and bound distance `≥ c`), 6/18/26-connectivity.
- Edge cost: Euclidean centre distance. Heuristic: exact obstacle-free lattice distance
  (Manhattan for 6, 3D octile for 26) or Euclidean (18); all admissible and consistent.
- Edge validity: both endpoints valid **and** no corner cutting (all lattice cells inside
  the move's unit sub-cube must be valid) **and** the continuous segment is certified
  (fast Lipschitz test with endpoint `F`, fallback to the segment checker).
- Start/goal are continuous points connected to nearby valid centres by checked segments.
- Deterministic tie-breaking `(f, h, insertion counter)`.
- Optimality claim: optimal **only** on this lattice graph under these validity rules.

## Planner B — 3D RRT*

- Uniform sampling in the bounds shrunk by `c`, goal bias, seeded `numpy.random.Generator`.
- Nearest neighbour: brute-force vectorised NumPy (tree sizes ≤ ~10⁴; a spatial index is
  not justified before measurement).
- Steering with step `η`; near radius `min(γ (log n / n)^{1/3}, η)`, with
  `γ = γ_factor · 2 (1 + 1/3)^{1/3} (μ(X_free)/ζ_3)^{1/3}` (Karaman & Frazzoli).
- Parent selection: candidates sorted by `cost(n) + ‖n − x‖`, first collision-free wins
  (equivalent to the minimum-cost collision-free parent).
- Rewiring: strict cost improvement + collision-free edge; children lists are updated and
  costs of all descendants are recomputed exactly (BFS).
- Goal: any node within `η` of the goal with a collision-free connecting edge is a goal
  candidate; the best path cost is re-evaluated each iteration (rewiring can lower it).
- Budget: iteration count (reproducible) plus a wall-clock safety cap.
- No optimality guarantee is claimed for finite runs.

## Independent validation

`evaluation/validation.py` does **not** use the planner's clearance field or checker:

- **Ground truth**: for every path segment and every obstacle primitive (convex), the
  minimum of the convex function `t ↦ dist(a + t(b−a), primitive)` is found by
  golden-section search; bound distance is affine per face (minimum at endpoints).
- **Voxel map**: same exact segment-to-cube minimisation against every blocked voxel cube
  near the segment (candidates by brute-force index box, not KD-tree).
- A plan counts as a *validated success* only if both minima are `≥ c`.

## Phase 2: trajectories and replanning

```
geometric path (A* / RRT*)
  └─ trajectories.shortcut      greedy collision-checked shortcutting (certified segments)
  └─ trajectories.generator     piecewise quintic, C2, rest-to-rest, certified limits/clearance
  └─ evaluation.trajectory_validation   independent continuous-time validator
  └─ replanning.supervisor      conflict check -> emergency stop -> replan -> validate -> resume
```

**Clearance tiers.** Nominal geometric clearance `c_nom = r + margin = 0.35 m` (Phase 1).
Trajectories add a tracking allowance (0.10 m) for future controller error:
`c_traj = 0.45 m`. Trajectory pipelines plan, shortcut and certify with a checker built for
`c_traj`; the validator requires ground-truth clearance `≥ c_traj`. Phase 1 paths planned
at 0.35 m are *not* silently accepted at 0.45 m: shortcutting rejects an input path that is
not certified at the required clearance.

**Kinematic model.** Robot centre = translational double integrator, `|v| ≤ v_max`,
`|a| ≤ a_max` (Euclidean norms; emergency braking up to `a_dec ≥ a_max`). No attitude,
thrust or rotor dynamics: translational feasibility is *not* quadrotor feasibility.

**Trajectory construction.** One quintic Hermite piece per segment of the simplified
polyline; `a = 0` at all waypoints (C2), rest at start and goal. Interior waypoint
velocity: `λ · cos²(θ/2) · min(adjacent L/T)` along the corner bisector. Durations start
at the rest-to-rest minimum-jerk bounds and are stretched (×1.15) until certified bounds
hold. Curved pieces are certified collision-free (below); a rejected piece halves λ at its
end waypoints, and λ = 0 on both ends turns the piece into a rest-to-rest motion that lies
exactly on the certified polyline segment, so a valid trajectory always exists (fallback).

**Continuous-time certificates** (generator, supervisor and validator each use their own
implementation). A polynomial piece is a Bézier curve inside the convex hull of its
control points, hence inside the ball centred on the curve midpoint `m` with radius
`R = max |P_j − m|`. For a 1-Lipschitz distance `f`, `min f ≥ f(m) − R` on that piece;
de Casteljau subdivision tightens this until it is certified or refuted. The same hull
property bounds `max |v|` and `max |a|` from the derivative control points. The generator
certifies against the voxel-map lower bound `ℓ` at `c_traj`. The validator computes
certified `[lower, upper]` intervals against **exact ground-truth geometry** and also
checks knot continuity (p, v, a), boundary rest states, monotone timestamps, finite
coefficients and optional corridor deviation.

**Replanning protocol** (stop-and-replan, static obstacles only):
`t_appear` → (+ detection latency 0.2 s) map update → continuous conflict check of the
remaining trajectory against the new obstacles at `c_traj` → (+ decision latency 0.1 s)
constant-deceleration stop along the current velocity → classify everything flown after
`t_appear` against the updated geometry: attained clearance `< r` ⇒
`unavoidable_collision`; certified clearance `< c_nom` ⇒ `stop_margin_violation`; a
hovering robot whose hold point violates `c_nom` ⇒ `hold_unsafe`; otherwise safe stop.
From rest, the voxel map is rebuilt and the configured planner attempts run the full
pipeline; the robot resumes only with an independently validated trajectory. A* exhausting
its open set ⇒ `no_route_holding`; all attempts failing otherwise ⇒
`replan_failed_holding`. Baseline stopping distance: `d = v·t_L + v²/(2·a_dec)`.

## Timing

`planning_time_s` covers only the search (A*) or the sampling loop (RRT*).
Map construction, clearance field and lattice inflation are reported separately.
Peak memory is measured in a separate tracemalloc run so it does not distort timing.

## Phase 3: closed-loop simulation and viewer

```
validated trajectory (Phase 2 pipeline, unchanged)
  └─ simulation.navigator.Navigator     executive: FOLLOWING → (update) → BRAKING → HOLDING →
  │     ├─ simulation.controller        REPLANNING → FOLLOWING_REPLAN → GOAL_REACHED, or
  │     ├─ simulation.dynamics          HOLDING_ABORTED / COLLIDED
  │     ├─ simulation.safety            independent swept-step clearance monitor
  │     └─ replanning.supervisor        assess_update (actual state) / replan_from_rest (actual hold)
  └─ simulation.metrics                 tracking, limits, certified clearance, safety verdict
  └─ viz.rerun_viewer                   Rerun recording + blueprint from saved results only
```

`simulation` imports no visualisation code; only `viz/rerun_viewer.py` imports Rerun.

**Motion model.** Translational double integrator `p' = v, v' = a_cmd + d(t)` with an explicit
control period `dt = 0.01 s`, zero-order hold, exact integration per step. `d(t)` is a
deterministic disturbance: `ideal` (zero) or `gusts` (bias + two sinusoids per axis,
|d| ≤ 0.42 m/s², fixed before any scenario was simulated). The motion inside a step is the
quadratic Bézier `(p_k, p_k + v_k dt/2, p_{k+1})`. Not modelled: attitude, thrust/rotor
dynamics, drag, actuator lag, sensing/estimation (the controller reads the true state).

**Controller.** `a = a_ref(t_k + dt/2) + kp (p_ref − p) + kd (v_ref − v)`, kp = 9, kd = 6
(ω = 3 rad/s, ζ = 1), Euclidean saturation |a_cmd| ≤ 2 m/s² (Phase 2 nominal limit) while
tracking or holding, speed governor |v| ≤ 3 m/s (radial projection of the next velocity).
Emergency braking: `a = −3 v/|v|` m/s² (Phase 2 braking capability) until rest. With exact
feedforward and no saturation the error dynamics `ë = −kp e − kd ė + d` do not depend on the
reference, so all runs share the same tracking-error history except where saturation occurs.

**Safety monitor** (independent of the voxel map, checker and planners): exact ground-truth
distance to primitives and bounds (`WorldDistance`, equal to `Scene.distance`), evaluated on
each step's Bézier curve: `d(m) − R ≤ min d ≤ min(d(P0), d(m), d(P2))`, refined by de
Casteljau subdivision near thresholds and near the run minimum. Contact = certified lower
bound < robot radius (run terminated, no contact dynamics); margin violation = lower bound <
nominal clearance; the obstacle counts from `t_appear` (before detection). The bounds are
rigorous for the modelled piecewise-quadratic motion, not for a physical vehicle.

**Safety test.** A run passes only if: no contact, executed clearance certified ≥ 0.35 m,
tracking error ≤ 0.10 m (the reserved allowance) while tracking, |v| ≤ 3 m/s, |a_cmd| within
its mode's limit. Goal reached = within 0.10 m and 0.10 m/s after the reference ends.
Mission success = goal reached and safety pass; ERROR events always fail the run.

**Replanning integration.** The supervisor's `handle_update` was split (behaviour-preserving,
Phase 2 tests unchanged) into `assess_update` (conflict check, predicted stop, classification)
and `replan_from_rest`. At `t_detect` the executive calls `assess_update` with the *actual*
simulated state; at `t_brake` the vehicle executes its own braking; at rest the hold is
verified on the true updated geometry (hold point and everything flown since `t_appear` must
be ≥ 0.35 m), then `replan_from_rest` runs from the *actual* hold point. Simulated time
advances by the measured wall-clock compute (rounded up to dt) while the drone holds; an
optional fixed planning latency makes runs bit-reproducible (tests). Predicted unsafe stops
(collision, margin violation) are not replanned, as in Phase 2.

**Viewer.** `scripts/view.py` loads `results/phase3/sims/<run>.npz|json` and
`results/phase3/planning/<entry>.npz`, writes `results/phase3/recordings/<entry>__<profile>.rrd`
and opens it in the Rerun viewer (file input: no network server). All temporal data uses
timeline `sim_time` stamped with the integer-nanosecond value of `t_k = k dt`. Exploration
data comes from deterministic reruns of the actual planners with their recording flags,
checked to reproduce the visualised plan.

# Third-party notices and licensing map

AutoPath-3D combines original code with one file derived from Webots and with run-time
dependencies that are **not** redistributed here. This file says which licence applies to what.

## 1. Original AutoPath-3D material — MIT

Unless listed in §2 or §4, every file in this repository is original AutoPath-3D work, licensed
under the MIT License (`LICENSE`, Copyright (c) 2026 Atajan Garayev). This includes:

- `src/autopath3d/` (maps, collision checking, A*, RRT*, trajectories, replanning, simulation, viewer)
- `ros2/autopath3d_ros/` — all nodes, the Webots driver plugins (`crazyflie_bridge.py`,
  `scene_manager.py`), the world generator `tools/build_warehouse_world.py`, launch files,
  configuration, URDF plugin stubs and tests — **except** `worlds/autopath_warehouse.wbt` (§2)
- `ros2/env.sh`, `ros2/scripts/`, `ros2/setup/`, `scripts/`, `tests/`, `configs/`, `docs/`
- `results/` — measurements and figures produced by this project (see §4 for Webots imagery)

## 2. Webots-derived content — Apache License 2.0

| File | `ros2/autopath3d_ros/worlds/autopath_warehouse.wbt` |
|---|---|
| Upstream | Webots R2025a sample world `projects/samples/environments/factory/worlds/factory.wbt` |
| Source | https://github.com/cyberbotics/webots (tag `R2025a`); also shipped with every Webots R2025a installation |
| Original author | Cyberbotics Ltd. |
| Licence | Apache License 2.0 — full text in [`LICENSES/Apache-2.0.txt`](LICENSES/Apache-2.0.txt) |
| Upstream notices | The upstream `LICENSE` at tag R2025a is the plain Apache 2.0 text; the repository has no `NOTICE` file at that tag, and `factory.wbt` carries no copyright header (checked 2026-11-11). |
| Modified | **Yes**, by the AutoPath-3D project (2026). The file header states this. |

Modifications (generated deterministically by `tools/build_warehouse_world.py` from the installed
`factory.wbt`; the generator itself is MIT):

- removed: the tabletop robots (Darwin-OP, Aibo), the central table with its tools, and the loose
  boxes and pallets of the original scene (23 nodes);
- the visual-only mezzanine platform was given a `boundingObject`;
- added: 8 pallet-rack segments, cargo (official `WoodenPallet`, `CardboardBox`, `WoodenBox`
  PROTOs), steel columns, an overhead duct with hangers, roof beams, workbenches (official `Table`
  PROTO), a pallet stack, a fenced storage cage with a sliding gate, a parked `Forklift`, a hidden
  "dropped load", two lights, the `Crazyflie` (extern controller) and the AutoPath scene-manager
  supervisor robot;
- new `WorldInfo` and `Viewpoint`.

The derived world as a whole is distributed under Apache-2.0. It is not covered by the MIT licence.

## 3. Referenced, not redistributed — Webots PROTOs and textures

The world references 24 PROTO files and 3 textures by URL
(`https://raw.githubusercontent.com/cyberbotics/webots/R2025a/...`). Webots downloads them at run
time into its own cache. **None of these files is copied into this repository**, and they should
not be added to it.

- Most object and robot PROTOs (for example `Crazyflie`, `Forklift`, `WoodenBox`, `OilBarrel`,
  `TexturedBackground`, `Door`, `Wall`) carry the header
  *"Copyright Cyberbotics Ltd. Licensed for use only with Webots."* —
  licence URL: https://cyberbotics.com/webots_assets_license
- Some appearance PROTOs (for example `GlossyPaint`, `Roughcast`, `ThreadMetalPlate`) are Apache-2.0.
- The applicable terms are those in each file's own header at tag R2025a.

## 4. Material produced with Webots

These files are AutoPath-3D outputs, but they depict or are measured from Webots assets:

| File | What it is |
|---|---|
| `results/phase4/webots/screenshot_B_replan.jpg` | One frame rendered by Webots R2025a (Cyberbotics Ltd.) during a Demo B run, exported by the AutoPath scene manager. It shows Webots PROTO models and textures (© Cyberbotics Ltd., Webots asset terms). It comes from a Demo B run other than the archived one (its overlay shows goal error 0.009 m). |
| `ros2/autopath3d_ros/test/data/webots_warehouse_geometry.json` | Numeric axis-aligned boxes and cylinders extracted from the bounding objects of the running world. No meshes or textures. |
| `results/phase4/webots/**/run_*.json`, `*_series.npz`, `characterisation/*.json` | Flight measurements (positions, clearances, events) recorded in the simulation. |

The AutoPath-3D contribution to these files is MIT. Webots assets visible in the screenshot remain
subject to their own terms (§3). They are included as documentation of a simulation run.

## 5. Run-time dependencies (installed separately, not redistributed)

| Dependency | Use | Licence (upstream) |
|---|---|---|
| Webots R2025a | simulator | Apache-2.0 |
| `webots_ros2_driver` 2025.0.0 | Webots ↔ ROS 2 driver | Apache-2.0 |
| `webots_ros2_crazyflie` 2025.0.0 | provides Bitcraze `pid_velocity_fixed_height_controller`, imported at run time **unmodified** by `crazyflie_bridge.py` | package Apache-2.0; the PID file is MIT, Copyright 2023 Bitcraze |
| ROS 2 Jazzy (`rclpy`, `launch`, message packages, `tf2_ros`) | middleware | Apache-2.0 (per package) |
| NumPy, SciPy | numerics | BSD-3-Clause |
| PyYAML | configuration | MIT |
| rerun-sdk 0.41.0 (optional) | viewer | MIT OR Apache-2.0 |
| matplotlib (optional) | Phase 1 figure | Matplotlib licence (PSF-style) |
| pytest (tests only) | tests | MIT |

Check each project's own licence file for the authoritative terms.

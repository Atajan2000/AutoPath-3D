# AutoPath-3D Phase 4: ROS 2 Jazzy + Webots warehouse

The validated Phase 1–3 core (`src/autopath3d`: occupancy map, A*/RRT*, path validation,
shortcutting, certified quintic trajectories, Phase 2 replanning supervisor) runs unchanged inside
ROS 2 nodes and flies a physics-simulated Crazyflie in a Webots warehouse. Results and the full
report: `docs/PHASE4_REPORT.md`.

## Environment (tested 2026-11-10)

| Component | Version / location |
|---|---|
| Windows 11 + WSL2 | Ubuntu 24.04 (`Ubuntu-24.04`), mirrored networking, DNS tunnelling |
| ROS 2 | Jazzy, `ros-jazzy-ros-base` + `ros-dev-tools` (apt, packages.ros.org) |
| Webots | R2025a, native Windows install `C:\Program Files\Webots` (no second copy) |
| webots_ros2 | 2025.0.0 (`ros-jazzy-webots-ros2-driver`, `ros-jazzy-webots-ros2-crazyflie`) |
| Python (WSL) | 3.12, Ubuntu numpy 1.26.4, scipy 1.11.4 (core tests pass: 108 + 4 viewer tests skipped without rerun) |

Webots runs on Windows; ROS 2, the planner stack and the two Webots extern controllers
(Crazyflie bridge, scene manager) run in WSL and connect to Webots over TCP `127.0.0.1:1234`.

## One-time setup

```bash
# WSL (Ubuntu 24.04): working checkout. Either clone the repository, or - when you edit in a Windows
# checkout - clone that checkout so both sides stay in sync through git:
git clone <repository-url> ~/autopath3d
#   or: git clone "/mnt/c/<path-to-your-windows-checkout>/AutoPath-3D" ~/autopath3d

# Once: ROS 2 Jazzy + webots_ros2 (sudo prompts for your password; dry-run size check first)
bash ~/autopath3d/ros2/setup/install_ros2_jazzy.sh

# Workspace (only the ROS package is linked in; the core library comes through PYTHONPATH)
mkdir -p ~/ros2_ws/src && ln -sfn ~/autopath3d/ros2/autopath3d_ros ~/ros2_ws/src/autopath3d_ros
source ~/autopath3d/ros2/env.sh
cd ~/ros2_ws && colcon build --symlink-install && source ~/autopath3d/ros2/env.sh
```

`env.sh` defaults: `AP3D_ROOT=~/autopath3d`, `AP3D_WS=~/ros2_ws`,
`WEBOTS_HOME=/mnt/c/Program Files/Webots`; export any of them before sourcing to override.

Update after changes on Windows: `cd ~/autopath3d && git pull && cd ~/ros2_ws && colcon build --symlink-install`.
Only `ros2/autopath3d_ros` is in the workspace (symlink); the core library comes from
`$AP3D_ROOT/src` through `PYTHONPATH` (set by `env.sh`).

`env.sh` also sets two measured workarounds for this machine:
* `FASTDDS_BUILTIN_TRANSPORTS=SHM` — Fast DDS over UDP between two WSL processes delivered nothing
  (multicast and unicast, any discovery range); shared memory works. No firewall changes were made;
  with SHM only, no DDS traffic leaves the machine.
* `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET` — `LOCALHOST` combined with SHM-only discovers nothing.
* `AUTOPATH_WEBOTS_HOST=127.0.0.1` — webots_ros2 derives the host from `/etc/resolv.conf`, which under
  DNS tunnelling is `10.255.255.254` (not the Windows host).

## Run

```bash
source ~/autopath3d/ros2/env.sh
ros2 launch autopath3d_ros warehouse.launch.py                              # world + stack, idle
ros2 launch autopath3d_ros warehouse.launch.py scenario:=A_navigation       # Demo A
ros2 launch autopath3d_ros warehouse.launch.py scenario:=B_replan           # Demo B: forklift -> brake -> replan -> resume
ros2 launch autopath3d_ros warehouse.launch.py scenario:=C1_no_route        # Demo C1: gate closes -> no route -> hold
ros2 launch autopath3d_ros warehouse.launch.py scenario:=C2_unsafe_stop     # Demo C2: load drops ahead -> unsafe stop
ros2 launch autopath3d_ros warehouse.launch.py stack:=false characterise:=true   # Crazyflie characterisation
```

Each scenario run exits by itself and writes `~/autopath_results/<scenario>/run_<stamp>.json`
(+ `_series.npz`). Launch arguments: `scenario`, `results_dir`, `mode` (realtime|fast),
`gui` (true|false), `stack`, `characterise`, `config`.
After an interrupted run: `bash ~/autopath3d/ros2/scripts/cleanup.sh` (see Troubleshooting).

Tests:

```bash
# core (Phase 1-3 + stopping assessment). PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 is required once ROS is
# sourced: the ROS launch_testing pytest plugins otherwise make the run collect 0 tests and exit 0
# (measured 2026-11-11). Expected: 108 passed, test_viewer.py skipped (no rerun in WSL).
cd ~/autopath3d && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q tests
python3 -m pytest -q ros2/autopath3d_ros/test                      # package: pure modules + offline warehouse tests
AUTOPATH_E2E=1 python3 -m pytest -v ros2/autopath3d_ros/test/test_e2e_webots.py   # end-to-end, starts Webots
AUTOPATH_E2E=1 AUTOPATH_E2E_SCENARIOS=A_navigation,B_replan,C1_no_route,C2_unsafe_stop python3 -m pytest -v ros2/autopath3d_ros/test/test_e2e_webots.py
# e2e evidence (launch logs, session records, results, summary.jsonl) is kept under ~/autopath_e2e/suite_<stamp>/
python3 ros2/autopath3d_ros/test/webots_session.py --startup-check 5   # Webots start-up reliability only (no flight)
```

## Architecture

```
 Webots R2025a (Windows)                         WSL Ubuntu 24.04 / ROS 2 Jazzy
 ┌──────────────────────────────┐   TCP 1234    ┌────────────────────────────────────────────────────┐
 │ autopath_warehouse.wbt        │◄────────────►│ webots_ros2_driver: AutoPathCrazyflie (bridge)       │
 │  Crazyflie (extern)           │              │   Bitcraze PID (unchanged) + AutoPath outer loop     │
 │  AutoPathSceneManager         │◄────────────►│ webots_ros2_driver: AutoPathSceneManager (supervisor)│
 │  (extern, supervisor)         │              │   /clock, geometry extraction, ground truth,         │
 └──────────────────────────────┘              │   scripted obstacles, obstacle updates, path drawing │
                                                │ map_node  planner_node  trajectory_node              │
                                                │ supervisor_node  mission_node / characterise_node    │
                                                └────────────────────────────────────────────────────┘
```

| Node | Role | Core code used (unchanged) |
|---|---|---|
| `AutoPathSceneManager` (Webots supervisor plugin) | publishes `/clock`; extracts every boundingObject of the running world into primitives (`webots_extract.py`); ground-truth odometry and contacts; executes scripted obstacle motions and publishes the resulting geometry as obstacle updates after the detection latency; draws paths | — |
| `map_node` | collision primitives in the geofence + rack envelopes + visual parts without collision geometry (conservative) + updates → `/autopath/map/obstacles`, voxel cloud, map info | `OccupancyGrid`, `surface_mask` |
| `planner_node` | `GetPlan` services `/autopath/plan/{astar,rrt_star}`, independent path validation | `trajectory_checker`, `make_planner`, `validate_path` |
| `trajectory_node` | shortcut → quintic trajectory → continuous-time validation | `run_pipeline(path=…)` |
| `supervisor_node` | mission executive; obstacle updates: Phase 2 step-1 conflict certificate → brake, conservative stopping assessment, Phase 2 `assess_update`; replanning from the measured rest point | `ReplanningSupervisor`, `assess_stopping` |
| `AutoPathCrazyflie` (Webots robot plugin) | follows `/autopath/active_trajectory`; takeoff/hold/brake/land; odometry, TF, tracking error, events | `PiecewisePolynomial` |
| `mission_node` | scenario runner + recorder (placement, takeoff, request, scenario trigger from measured state, metrics) | — |

Interfaces use standard messages: `nav_msgs/Path`, `nav_msgs/Odometry`, `nav_msgs/srv/GetPlan`,
`trajectory_msgs/MultiDOFJointTrajectory` (lossless: position, velocity and acceleration at every
knot → quintic Hermite pieces), `vision_msgs/Detection3DArray` (boxes / vertical cylinders),
`sensor_msgs/PointCloud2`, `rosgraph_msgs/Clock`, `tf2` (`map → base_link`); JSON in `std_msgs/String`
for events, state and reports. Frames: `map` = Webots world frame, ENU, metres; `base_link` FLU.
All nodes use simulation time (`use_sim_time`, `/clock` from the scene manager). No NED anywhere.

Map: this phase uses the **known simulation map** (collision geometry extracted from Webots), not
SLAM or sensor-based mapping. Every voxel not touching extracted geometry is FREE because it is
known; there are no unknown regions in this world. The core still treats UNKNOWN as occupied.

## What is third-party and what is AutoPath

| Item | Origin | License |
|---|---|---|
| `factory.wbt` (building, walls, mezzanine, stairs, pipes, doors) | Cyberbotics, Webots R2025a sample | Apache-2.0 |
| PROTOs (Crazyflie, Forklift, WoodenPallet, CardboardBox, WoodenBox, OilBarrel, …) | Cyberbotics webots repository, tag R2025a (downloaded by Webots, ~13 MB cache) | Webots asset license (use with Webots) |
| `pid_velocity_fixed_height_controller` (attitude/altitude/velocity PID) | Bitcraze, via `webots_ros2_crazyflie` | MIT |
| webots_ros2 driver, launcher, `webots-controller` | Cyberbotics | Apache-2.0 |
| `worlds/autopath_warehouse.wbt` (generated, modified `factory.wbt`) | AutoPath-3D modifications of the Cyberbotics sample | Apache-2.0 (`worlds/LICENSE.md`) |
| Everything else in `ros2/autopath3d_ros` (world generator, extractor, bridge outer loop and mode logic, scene manager, nodes, tests) and `src/autopath3d` | AutoPath-3D | MIT (`LICENSE`) |

Full licensing map: `THIRD_PARTY_NOTICES.md`.

`tools/build_warehouse_world.py` turns the installed `factory.wbt` into
`worlds/autopath_warehouse.wbt` deterministically (removes the loose tools/robots, adds racks,
pallets, columns, duct, cage + gate, forklift, Crazyflie, scene manager; gives the original
mezzanine platform a collision box — it had none).

## Troubleshooting

* **Topics delayed by seconds / messages lost**: orphaned `webots_ros2_driver/driver` processes of
  an earlier run are still on the DDS graph (launch SIGTERMs `webots-controller`, which does not
  forward it). The launch file now kills them on shutdown; `ros2/scripts/cleanup.sh` does it by hand.
* **Webots stays on "loading"** on first start: PROTO textures download through the Webots proxy
  setting; restarting Webots retries failed downloads.
* **Controllers print `Cannot connect to Webots instance`** for a few seconds: normal while Webots
  starts; they retry for 50 s.

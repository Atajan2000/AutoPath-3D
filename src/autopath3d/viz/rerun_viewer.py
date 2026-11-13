"""Rerun recording + blueprint for a saved Phase 3 simulation run.

Everything time-dependent is logged on the timeline `sim_time` using the simulation's own
time stamps t_k = k dt, so the 3D view, the plots, the state lane and the event log are
synchronised by construction (verified in tests by reading the .rrd back).
Nothing here simulates or plans: it only reads saved results (SimResult + exploration data).
"""

from __future__ import annotations

import math

import numpy as np
import rerun as rr
import rerun.blueprint as rrb

from ..environments.primitives import Box, VerticalCylinder
from ..simulation.navigator import HOLD_MODES, M, MODES, TRACKING_MODES, SimResult
from ..trajectories.polynomial import PiecewisePolynomial

TIMELINE = "sim_time"
SOLID = rr.components.FillMode.Solid
WIRE = rr.components.FillMode.MajorWireframe

C = {  # stable colours (RGBA)
    "obstacle": (110, 122, 145, 85), "thin": (200, 175, 120, 220), "bounds": (150, 150, 160, 90),
    "new_obstacle": (235, 95, 45, 255), "new_detected": (220, 40, 40, 255),
    "start": (60, 210, 90, 255), "goal": (250, 200, 40, 255),
    "path_raw": (140, 140, 150, 255), "waypoints": (230, 230, 235, 255),
    "reference": (70, 150, 255, 255), "reference_invalid": (70, 110, 170, 110),
    "replanned": (0, 215, 180, 255), "desired": (90, 170, 255, 255),
    "error_line": (255, 70, 200, 255), "velocity": (120, 235, 120, 255), "nearest": (255, 255, 255, 120),
    "pred_brake": (255, 220, 60, 255), "stop_dist": (255, 150, 30, 255), "contact": (255, 60, 60, 255),
    "body": (40, 44, 52, 255), "rotor": (200, 205, 215, 255), "margin": (255, 255, 255, 50),
    "astar": (255, 150, 50, 255), "rrt_tree": (80, 200, 220, 110), "rrt_nodes": (120, 220, 240, 255),
    "rrt_path": (250, 200, 40, 255), "astar_path": (255, 120, 40, 255),
}
MODE_COLORS = {"FOLLOWING": (70, 150, 255, 255), "BRAKING": (255, 170, 30, 255), "HOLDING": (200, 200, 90, 255),
               "REPLANNING": (190, 120, 255, 255), "FOLLOWING_REPLAN": (0, 215, 180, 255),
               "GOAL_REACHED": (60, 210, 90, 255), "HOLDING_ABORTED": (255, 120, 60, 255),
               "COLLIDED": (230, 30, 30, 255), "TIMEOUT": (230, 30, 30, 255)}
MODE_LABELS = {"FOLLOWING": "FOLLOWING", "BRAKING": "BRAKING", "HOLDING": "HOLDING (verify stop)",
               "REPLANNING": "REPLANNING (holding)", "FOLLOWING_REPLAN": "RESUMED: following new route",
               "GOAL_REACHED": "GOAL REACHED", "HOLDING_ABORTED": "HOLDING: mission aborted",
               "COLLIDED": "COLLISION: run terminated", "TIMEOUT": "TIMEOUT"}
ENVELOPE_CLASSES = [  # (min clearance, colour, meaning) on the certified per-step lower bound
    (0.45, (80, 210, 120, 255), "clearance >= trajectory tier"),
    (0.35, (240, 210, 60, 255), "inside tracking allowance band"),
    (0.25, (255, 140, 0, 255), "safety margin violated"),
    (-1e9, (235, 30, 30, 255), "contact"),
]


def _ns(t) -> np.ndarray:
    """Simulation seconds -> exact integer nanoseconds (t_k = k dt is stored without float drift)."""
    return np.round(np.asarray(t, float) * 1e9).astype(np.int64).astype("timedelta64[ns]")


def _tcol(t) -> rr.TimeColumn:
    return rr.TimeColumn(TIMELINE, duration=_ns(t))


def _at(rec: rr.RecordingStream, t: float) -> None:
    rec.set_time(TIMELINE, duration=_ns([t])[0])


def _traj(d: dict) -> PiecewisePolynomial:
    hp = d.get("hold_point")
    return PiecewisePolynomial(np.asarray(d["knots"], float), np.asarray(d["coeffs"], float).reshape(-1, 6, 3),
                               None if hp is None else np.asarray(hp, float))


def _sample(tr: PiecewisePolynomial, step: float = 0.05) -> np.ndarray:
    if tr.n_pieces == 0:
        return tr.hold_point[None]
    t = np.append(np.arange(tr.t_start, tr.t_end, step), tr.t_end)
    return tr.evaluate(t)


def _ramp(n: int) -> np.ndarray:
    """Expansion-order colour ramp (dark purple -> orange -> yellow), uint8 RGBA."""
    u = np.linspace(0.0, 1.0, max(n, 1))[:, None]
    stops = np.array([[60, 30, 110], [200, 60, 90], [255, 150, 40], [255, 235, 120]], float)
    x = u * (len(stops) - 1)
    i = np.minimum(x.astype(int), len(stops) - 2)
    f = x - i
    rgb = stops[i[:, 0]] * (1 - f) + stops[i[:, 0] + 1] * f
    return np.hstack([rgb, np.full((len(rgb), 1), 255)]).astype(np.uint8)


# ---------------------------------------------------------------- static scene
def log_scene(rec: rr.RecordingStream, scene) -> None:
    rec.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    lo, hi = np.asarray(scene.lo), np.asarray(scene.hi)
    rec.log("world/scene/bounds", rr.Boxes3D(mins=[lo], sizes=[hi - lo], colors=[C["bounds"]], fill_mode=WIRE,
                                             radii=0.02), static=True)
    boxes = [o for o in scene.obstacles if isinstance(o, Box)]
    if boxes:
        mins = np.array([b.lo for b in boxes])
        sizes = np.array([b.hi for b in boxes]) - mins
        thin = sizes.min(axis=1) < 0.15
        colors = [C["thin"] if t else C["obstacle"] for t in thin]
        rec.log("world/scene/obstacles", rr.Boxes3D(mins=mins, sizes=sizes, colors=colors, fill_mode=SOLID,
                                                    labels=[b.name for b in boxes], show_labels=False), static=True)
    cyl = [o for o in scene.obstacles if isinstance(o, VerticalCylinder)]
    if cyl:
        rec.log("world/scene/cylinders", rr.Cylinders3D(
            lengths=[c.z_hi - c.z_lo for c in cyl], radii=[c.radius for c in cyl],
            centers=[(c.center_xy[0], c.center_xy[1], 0.5 * (c.z_lo + c.z_hi)) for c in cyl],
            colors=[C["obstacle"]] * len(cyl), fill_mode=SOLID, labels=[c.name for c in cyl], show_labels=False),
            static=True)


def log_drone_model(rec: rr.RecordingStream, radius: float, nominal: float) -> None:
    arm = 0.17
    rec.log("world/drone/body", rr.Boxes3D(
        centers=[(0, 0, 0), (0, 0, 0), (0, 0, 0)],
        half_sizes=[(0.07, 0.05, 0.03), (arm * 1.05, 0.012, 0.012), (0.012, arm * 1.05, 0.012)],
        quaternions=[rr.Quaternion(xyzw=(0, 0, 0, 1)), rr.Quaternion(xyzw=(0, 0, math.sin(math.pi / 8), math.cos(math.pi / 8))),
                     rr.Quaternion(xyzw=(0, 0, math.sin(math.pi / 8), math.cos(math.pi / 8)))],
        colors=[C["body"], (60, 64, 72, 255), (60, 64, 72, 255)], fill_mode=SOLID), static=True)
    a = arm / math.sqrt(2)
    rec.log("world/drone/rotors", rr.Cylinders3D(
        lengths=[0.01] * 4, radii=[0.075] * 4, centers=[(a, a, 0.02), (-a, a, 0.02), (-a, -a, 0.02), (a, -a, 0.02)],
        colors=[(230, 80, 60, 255), (230, 80, 60, 255), C["rotor"], C["rotor"]], fill_mode=SOLID), static=True)
    rec.log("world/drone/heading", rr.Arrows3D(vectors=[(0.32, 0, 0)], origins=[(0, 0, 0.03)],
                                               colors=[(230, 80, 60, 255)], radii=0.008), static=True)
    rec.log("world/drone/margin", rr.Ellipsoids3D(radii=[nominal], centers=[(0, 0, 0)], colors=[C["margin"]],
                                                  fill_mode=WIRE, line_radii=0.003), static=True)


# ---------------------------------------------------------------- per-run data
def log_run(rec: rr.RecordingStream, res: SimResult, scene) -> None:
    A, meta = res.arrays, res.meta
    pol = meta["policy"]
    t_state, t_step = A["t"], A["t"][:-1]
    p, v = A["p"], A["v"]
    mode = A["mode"]
    n = len(t_step)
    goal = np.asarray(meta["goal"], float)
    start = np.asarray(meta.get("start", p[0]), float)

    rec.log("world/markers/start", rr.Points3D([start], radii=0.12, colors=[C["start"]], labels=["start"]), static=True)
    rec.log("world/markers/goal", rr.Points3D([goal], radii=0.14, colors=[C["goal"]], labels=["goal"]), static=True)

    # reference / plans
    _at(rec, 0.0)
    if meta.get("reference_path"):
        rec.log("world/plan/planner_path", rr.LineStrips3D([meta["reference_path"]], radii=0.015, colors=[C["path_raw"]],
                                                           ))
    if meta.get("reference_simplified"):
        rec.log("world/plan/waypoints", rr.Points3D(meta["reference_simplified"], radii=0.06, colors=[C["waypoints"]]))
    tr0 = _traj(res.trajectories[0])
    rec.log("world/plan/reference", rr.LineStrips3D([_sample(tr0)], radii=0.03, colors=[C["reference"]]))
    ev = {e.name: e for e in res.events}
    if "CONFLICT" in ev:
        _at(rec, ev["CONFLICT"].t)
        rec.log("world/plan/reference", rr.LineStrips3D([_sample(tr0)], radii=0.02, colors=[C["reference_invalid"]]))
    if len(res.trajectories) > 1:
        tr1 = _traj(res.trajectories[1])
        _at(rec, tr1.t_start)
        rec.log("world/plan/replanned", rr.LineStrips3D([_sample(tr1)], radii=0.035, colors=[C["replanned"]]))
        kn = tr1.evaluate(tr1.knots)
        rec.log("world/plan/replanned_waypoints", rr.Points3D(kn, radii=0.06, colors=[C["replanned"]]))

    # new obstacles
    upd = meta.get("update")
    if upd:
        mins = np.array([o["lo"] for o in upd["obstacles"]])
        sizes = np.array([o["hi"] for o in upd["obstacles"]]) - mins
        _at(rec, upd["t_appear"])
        rec.log("world/new_obstacles", rr.Boxes3D(mins=mins, sizes=sizes, colors=[C["new_obstacle"]] * len(mins),
                                                  fill_mode=SOLID,
                                                  labels=["NEW OBSTACLE (not yet detected)"] * len(mins)))
        if "OBSTACLE_DETECTED" in ev:
            _at(rec, ev["OBSTACLE_DETECTED"].t)
            rec.log("world/new_obstacles", rr.Boxes3D(mins=mins, sizes=sizes, colors=[C["new_detected"]] * len(mins),
                                                      fill_mode=SOLID,
                                                      labels=[f"NEW OBSTACLE (detected {ev['OBSTACLE_DETECTED'].t:.2f} s)"] * len(mins)))

    # stopping-distance explanation and predicted braking (supervisor, made at detection)
    sup = res.supervisor
    if sup and "CONFLICT" in ev:
        kd = int(round(ev["OBSTACLE_DETECTED"].t / meta["dt"]))
        ka = int(round(upd["t_appear"] / meta["dt"]))
        va = v[ka] / max(np.linalg.norm(v[ka]), 1e-9)
        d_stop = sup["stopping_distance_baseline_m"]
        d_contact = sup.get("distance_to_contact_m")
        _at(rec, ev["OBSTACLE_DETECTED"].t)
        rec.log("world/braking/required_stop", rr.Arrows3D(
            vectors=[va * d_stop], origins=[p[ka] + (0, 0, -0.5)], colors=[C["stop_dist"]], radii=0.025,
            labels=[f"needs {d_stop:.2f} m to stop at {sup['speed_at_appear']:.2f} m/s"]))
        if d_contact is not None:
            rec.log("world/braking/contact_distance", rr.Points3D(
                [p[ka] + va * d_contact + (0, 0, -0.5)], radii=0.08, colors=[C["contact"]],
                labels=[f"contact in {d_contact:.2f} m"]))
        if "stop_point" in sup:
            tb = sup["t_brake"]
            kb = int(round(tb / meta["dt"]))
            _at(rec, tb)
            rec.log("world/braking/predicted", rr.LineStrips3D(
                [[p[kb].tolist(), sup["stop_point"]]], radii=0.02, colors=[C["pred_brake"]]))
            pc = sup["stop_min_clearance"][0]
            rec.log("world/braking/predicted_stop", rr.Points3D(
                [sup["stop_point"]], radii=0.07, colors=[C["pred_brake"]],
                labels=[f"predicted stop ({max(pc, 0.0):.2f} m)"]))
        _ = kd
    if "STOPPED" in ev:
        e = ev["STOPPED"]
        _at(rec, e.t)
        rec.log("world/braking/actual_stop", rr.Points3D([e.data["stop_point"]], radii=0.07, colors=[(255, 255, 255, 255)],
                                                         labels=["actual stop"]))
    if "COLLISION" in ev:
        e = ev["COLLISION"]
        _at(rec, round(e.t + meta["dt"], 9))
        rec.log("world/braking/contact", rr.Points3D([p[-1]], radii=0.1, colors=[C["contact"]],
                                                     labels=[f"CONTACT at {e.data['speed']:.2f} m/s"]))

    # drone pose (display yaw from horizontal velocity; translation is the simulated state)
    yaw = np.zeros(len(p))
    cur = 0.0
    if len(p) > 1:
        d0 = (p[-1] - p[0])[:2]
        cur = math.atan2(d0[1], d0[0]) if np.linalg.norm(d0) > 1e-6 else 0.0
    for i in range(len(p)):
        h = v[i, :2]
        if np.hypot(*h) > 0.3:
            cur = math.atan2(h[1], h[0])
        yaw[i] = cur
    quat = np.stack([np.zeros_like(yaw), np.zeros_like(yaw), np.sin(yaw / 2), np.cos(yaw / 2)], axis=1)
    rec.send_columns("world/drone", indexes=[_tcol(t_state)],
                     columns=rr.Transform3D.columns(translation=p, quaternion=quat))

    # envelope colour = certified clearance class of the current step
    cls = np.full(n, len(ENVELOPE_CLASSES) - 1)
    for j in range(len(ENVELOPE_CLASSES) - 1, -1, -1):
        cls[A["clr_lo"] >= ENVELOPE_CLASSES[j][0]] = j
    ch = np.flatnonzero(np.r_[True, cls[1:] != cls[:-1]])
    for k in ch:
        _at(rec, float(t_step[k]))
        rec.log("world/drone/envelope", rr.Ellipsoids3D(
            radii=[pol["robot_radius"]], centers=[(0, 0, 0)], colors=[ENVELOPE_CLASSES[cls[k]][1]],
            fill_mode=WIRE, line_radii=0.006))

    rec.send_columns("world/drone_state/velocity", indexes=[_tcol(t_state)],
                     columns=rr.Arrows3D.columns(vectors=v * 0.6, origins=p))
    trk = np.isin(mode, TRACKING_MODES)
    lens = trk.astype(int)
    rec.send_columns("world/drone_state/desired", indexes=[_tcol(t_step)],
                     columns=rr.Points3D.columns(positions=A["p_ref"][trk]).partition(lens))
    seg = np.stack([A["p_ref"][trk], p[:-1][trk]], axis=1)
    rec.send_columns("world/drone_state/tracking_error", indexes=[_tcol(t_step)],
                     columns=rr.LineStrips3D.columns(strips=list(seg)).partition(lens))

    # nearest obstacle line + status label at 20 Hz
    from ..simulation.safety import WorldDistance
    from ..replanning.supervisor import ObstacleUpdate, updated_scene
    worlds = [WorldDistance.of_scene(scene)]
    if upd:
        obs = [Box(tuple(o["lo"]), tuple(o["hi"]), o["name"]) for o in upd["obstacles"]]
        worlds.append(WorldDistance.of_scene(updated_scene(scene, ObstacleUpdate(upd["name"], upd["t_appear"], obs))))
    stride = max(1, int(round(0.05 / meta["dt"])))
    ks = np.arange(0, n, stride)
    near_q = np.zeros((len(ks), 3))
    near_d = np.zeros(len(ks))
    for wid, w in enumerate(worlds):
        s = A["world"][ks] == wid
        if s.any():
            q, d, _ = w.nearest(p[ks][s])
            near_q[s], near_d[s] = q, d
    rec.send_columns("world/drone_state/nearest_obstacle", indexes=[_tcol(t_step[ks])],
                     columns=rr.LineStrips3D.columns(strips=list(np.stack([p[ks], near_q], axis=1)),
                                                     labels=[f"{d:.2f} m" for d in near_d]))
    labels = [f"{MODE_LABELS[MODES[mode[k]]]} | {np.linalg.norm(v[k]):.2f} m/s | clearance {near_d[i]:.2f} m"
              for i, k in enumerate(ks)]
    rec.send_columns("world/drone_state/status", indexes=[_tcol(t_step[ks])],
                     columns=rr.Points3D.columns(positions=p[ks] + (0, 0, 0.9), labels=labels))

    # executed trail: full history at 2 Hz (coloured by mode) + per-step tail
    def strips_until(k):
        out_s, out_c = [], []
        runs = np.flatnonzero(np.r_[True, mode[1:k] != mode[:k - 1]]) if k > 1 else np.array([0])
        bounds = list(runs) + [k]
        for a, b in zip(bounds[:-1], bounds[1:]):
            out_s.append(p[a:b + 1])
            out_c.append(MODE_COLORS[MODES[mode[a]]])
        return out_s, out_c
    hstride = max(1, int(round(0.5 / meta["dt"])))
    for k in list(range(hstride, n, hstride)) + [n]:
        s_, c_ = strips_until(k)
        _at(rec, float(t_state[k]))
        rec.log("world/executed/history", rr.LineStrips3D(s_, colors=c_, radii=0.025))
    tail = hstride + 10
    tails = [p[max(0, k - tail):k + 1] for k in range(1, n + 1)]
    rec.send_columns("world/executed/recent", indexes=[_tcol(t_state[1:])],
                     columns=rr.LineStrips3D.columns(strips=tails, colors=[(255, 255, 255, 230)] * n,
                                                     radii=[0.02] * n))

    # ---------------------------------------------------------------- time series
    names = ["x", "y", "z"]
    axis_col = [(235, 85, 85, 255), (90, 210, 110, 255), (90, 140, 255, 255)]
    axis_ref = [(255, 170, 170, 255), (170, 240, 180, 255), (170, 200, 255, 255)]
    for i, nm in enumerate(names):
        rec.log(f"plots/position/{nm}/actual", rr.SeriesLines(colors=[axis_col[i]], names=[f"{nm} actual"], widths=[1.5]), static=True)
        rec.send_columns(f"plots/position/{nm}/actual", indexes=[_tcol(t_state)], columns=rr.Scalars.columns(scalars=p[:, i]))
    for rid, tag in ((0, "original"), (1, "replanned")):
        s = trk & (A["ref_id"] == rid)
        if not s.any():
            continue
        for i, nm in enumerate(names):
            ent = f"plots/position/{nm}/desired_{tag}"
            rec.log(ent, rr.SeriesLines(colors=[axis_ref[i]], names=[f"{nm} desired ({tag})"], widths=[3.0]), static=True)
            rec.send_columns(ent, indexes=[_tcol(t_step[s])], columns=rr.Scalars.columns(scalars=A["p_ref"][s, i]))
        ent = f"plots/tracking/position_error_{tag}"
        rec.log(ent, rr.SeriesLines(colors=[(255, 90, 200, 255)], names=[f"|p - p_ref| ({tag}) [m]"], widths=[1.5]), static=True)
        rec.send_columns(ent, indexes=[_tcol(t_step[s])],
                         columns=rr.Scalars.columns(scalars=np.linalg.norm(p[:-1][s] - A["p_ref"][s], axis=1)))
        ent = f"plots/tracking/velocity_error_{tag}"
        rec.log(ent, rr.SeriesLines(colors=[(120, 200, 255, 255)], names=[f"|v - v_ref| ({tag}) [m/s]"], widths=[1.0]), static=True)
        rec.send_columns(ent, indexes=[_tcol(t_step[s])],
                         columns=rr.Scalars.columns(scalars=np.linalg.norm(v[:-1][s] - A["v_ref"][s], axis=1)))
        ent = f"plots/speed/desired_{tag}"
        rec.log(ent, rr.SeriesLines(colors=[(120, 170, 255, 255)], names=[f"desired speed ({tag})"], widths=[3.0]), static=True)
        rec.send_columns(ent, indexes=[_tcol(t_step[s])], columns=rr.Scalars.columns(scalars=np.linalg.norm(A["v_ref"][s], axis=1)))
    hold = np.isin(mode, HOLD_MODES)
    if hold.any():
        rec.log("plots/tracking/hold_drift", rr.SeriesLines(colors=[(220, 220, 100, 255)], names=["|p - hold point| [m]"]), static=True)
        rec.send_columns("plots/tracking/hold_drift", indexes=[_tcol(t_step[hold])],
                         columns=rr.Scalars.columns(scalars=np.linalg.norm(p[:-1][hold] - A["hold"][hold], axis=1)))

    def const(ent, value, color, name):
        rec.log(ent, rr.SeriesLines(colors=[color], names=[name], widths=[1.0]), static=True)
        rec.send_columns(ent, indexes=[_tcol([0.0, float(t_state[-1])])], columns=rr.Scalars.columns(scalars=[value, value]))

    const("plots/tracking/allowance", pol["tracking_allowance"], (255, 60, 60, 255),
          f"tracking allowance {pol['tracking_allowance']:.2f} m")
    rec.log("plots/speed/actual", rr.SeriesLines(colors=[(255, 255, 255, 255)], names=["actual speed"], widths=[1.5]), static=True)
    rec.send_columns("plots/speed/actual", indexes=[_tcol(t_state)], columns=rr.Scalars.columns(scalars=np.linalg.norm(v, axis=1)))
    const("plots/speed/limit", meta["controller"]["v_max"], (255, 60, 60, 255), f"v_max {meta['controller']['v_max']:.1f} m/s")
    rec.log("plots/acceleration/commanded", rr.SeriesLines(colors=[(255, 200, 80, 255)], names=["|a_cmd| (controller)"], widths=[1.5]), static=True)
    rec.send_columns("plots/acceleration/commanded", indexes=[_tcol(t_step)],
                     columns=rr.Scalars.columns(scalars=np.linalg.norm(A["a_cmd"], axis=1)))
    rec.log("plots/acceleration/net", rr.SeriesLines(colors=[(160, 160, 170, 255)], names=["|a_cmd + wind|"], widths=[1.0]), static=True)
    rec.send_columns("plots/acceleration/net", indexes=[_tcol(t_step)],
                     columns=rr.Scalars.columns(scalars=np.linalg.norm(A["a_cmd"] + A["a_dist"], axis=1)))
    const("plots/acceleration/limit_nominal", meta["controller"]["a_track_max"], (255, 60, 60, 255),
          f"tracking limit {meta['controller']['a_track_max']:.1f} m/s²")
    const("plots/acceleration/limit_braking", meta["controller"]["a_brake_max"], (180, 40, 40, 255),
          f"braking capability {meta['controller']['a_brake_max']:.1f} m/s²")
    rec.log("plots/clearance/executed", rr.SeriesLines(colors=[(255, 255, 255, 255)], names=["executed clearance (certified lower bound)"], widths=[1.5]), static=True)
    rec.send_columns("plots/clearance/executed", indexes=[_tcol(t_step)], columns=rr.Scalars.columns(scalars=A["clr_lo"]))
    const("plots/clearance/trajectory_tier", pol["trajectory"], (80, 210, 120, 255), f"trajectory clearance {pol['trajectory']:.2f} m")
    const("plots/clearance/nominal", pol["nominal"], (240, 210, 60, 255), f"nominal clearance {pol['nominal']:.2f} m")
    const("plots/clearance/contact", pol["robot_radius"], (235, 30, 30, 255), f"contact (robot radius {pol['robot_radius']:.2f} m)")

    # ---------------------------------------------------------------- navigation state + events
    rec.log("nav/mode", rr.StateConfiguration(values=MODES, labels=[MODE_LABELS[m] for m in MODES],
                                              colors=[MODE_COLORS[m] for m in MODES]), static=True)
    final_k = len(t_state) - 1
    trans = np.flatnonzero(np.r_[True, mode[1:] != mode[:-1]])
    st_t = [float(t_step[k]) for k in trans]
    st_v = [MODES[mode[k]] for k in trans]
    final_mode = meta["final_mode"]
    if final_mode != st_v[-1]:
        st_t.append(float(t_state[final_k]))
        st_v.append(final_mode)
    rec.send_columns("nav/mode", indexes=[_tcol(st_t)], columns=rr.StateChange.columns(state=st_v))
    lvl = {"INFO": rr.TextLogLevel.INFO, "WARN": rr.TextLogLevel.WARN, "ERROR": rr.TextLogLevel.ERROR}
    for e in res.events:
        _at(rec, e.t)
        rec.log("nav/events", rr.TextLog(f"{e.name}: {e.message}", level=lvl[e.level]))


def log_exploration(rec: rr.RecordingStream, ex: dict) -> None:
    pts = ex.get("astar_expanded")
    if pts is not None and len(pts):
        rec.log("planning/astar/expanded", rr.Points3D(pts, radii=0.035, colors=_ramp(len(pts))), static=True)
    if len(ex.get("astar_path", [])):
        rec.log("planning/astar/path", rr.LineStrips3D([ex["astar_path"]], radii=0.05, colors=[C["astar_path"]],
                                                       labels=["A* path"]), static=True)
    tp, par = ex.get("rrt_tree_pos"), ex.get("rrt_tree_parent")
    if tp is not None and len(tp) > 1:
        kids = np.flatnonzero(par >= 0)
        segs = np.stack([tp[par[kids]], tp[kids]], axis=1)
        rec.log("planning/rrt/tree", rr.LineStrips3D(segs, radii=0.008, colors=[C["rrt_tree"]]), static=True)
        rec.log("planning/rrt/nodes", rr.Points3D(tp, radii=0.03, colors=[C["rrt_nodes"]]), static=True)
    if len(ex.get("rrt_path", [])):
        rec.log("planning/rrt/path", rr.LineStrips3D([ex["rrt_path"]], radii=0.05, colors=[C["rrt_path"]],
                                                     labels=["RRT* path (seed 0)"]), static=True)
    for k in range(4):
        pl = ex.get(f"replan{k}_planner")
        if pl is None:
            continue
        base = f"planning/replan_attempt_{k + 1}_{pl}"
        if pl == "astar":
            q = ex.get(f"replan{k}_astar_expanded")
            if q is not None and len(q):
                rec.log(f"{base}/expanded", rr.Points3D(q, radii=0.035, colors=_ramp(len(q))[:, [2, 1, 0, 3]]), static=True)
            if len(ex.get(f"replan{k}_astar_path", [])):
                rec.log(f"{base}/path", rr.LineStrips3D([ex[f"replan{k}_astar_path"]], radii=0.05, colors=[C["replanned"]],
                                                        labels=[f"replan attempt {k + 1}: A* path"]), static=True)
        else:
            tp, par = ex[f"replan{k}_rrt_tree_pos"], ex[f"replan{k}_rrt_tree_parent"]
            kids = np.flatnonzero(par >= 0)
            if len(kids):
                rec.log(f"{base}/tree", rr.LineStrips3D(np.stack([tp[par[kids]], tp[kids]], axis=1), radii=0.012,
                                                        colors=[(255, 80, 80, 200)]), static=True)
            rec.log(f"{base}/nodes", rr.Points3D(tp, radii=0.05, colors=[(255, 80, 80, 255)],
                                                 labels=[f"attempt {k + 1}: RRT* {ex[f'replan{k}_rrt_iterations']} it., "
                                                         f"{ex[f'replan{k}_rrt_status']}"] + [""] * (len(tp) - 1)),
                    static=True)


# ---------------------------------------------------------------- documents
def summary_markdown(res: SimResult, entry: str, ex: dict | None) -> str:
    m, meta = res.metrics, res.meta
    tr = m["tracking"]
    verdict = ("PASS" if m["safety_pass"] else "FAIL")
    lines = [f"# {entry}", "", f"**{meta.get('description', meta.get('label', ''))}**", "",
             f"Disturbance profile: `{meta['disturbance']}` · dt = {meta['dt']} s · simulated duration "
             f"{m['execution_duration_s']:.2f} s", "",
             f"## Safety test: **{verdict}** · goal reached: **{'yes' if m['goal_reached'] else 'no'}**", ""]
    if meta.get("kind") == "scenario":
        lines += [f"- expected (Phase 2): `{', '.join(meta['expected'])}`",
                  f"- supervisor prediction at detection: `{meta['supervisor_prediction']}`",
                  f"- simulated outcome: **`{meta['outcome']}`** "
                  f"({'matches' if meta['matches_expected'] else 'DIFFERS FROM'} the Phase 2 expectation)", ""]
    failed = [k for k, val in m["checks"].items() if not val]
    brk_s = "–" if m["max_accel_cmd_braking"] is None else f"{m['max_accel_cmd_braking']:.3f}"
    drift_s = "–" if m["hold_drift_max"] is None else f"{m['hold_drift_max']:.3f} m"
    lines += ["| check | result |", "|---|---|"] + [f"| {k} | {'ok' if val else '**FAIL**'} |" for k, val in m["checks"].items()]
    lines += ["", "| metric | value |", "|---|---|",
              f"| max / RMS position tracking error | {tr['position_error']['max']:.4f} / {tr['position_error']['rms']:.4f} m |",
              f"| max / RMS velocity tracking error | {tr['velocity_error']['max']:.4f} / {tr['velocity_error']['rms']:.4f} m/s |",
              f"| max speed | {m['max_speed']:.3f} m/s |",
              f"| max commanded accel (tracking / braking) | {m['max_accel_cmd_tracking_hold']:.3f} / {brk_s} m/s² |",
              f"| max net accel incl. wind | {m['max_accel_net']:.3f} m/s² |",
              f"| min executed clearance (certified) | [{m['clearance']['lower']:.4f}, {m['clearance']['upper']:.4f}] m |",
              f"| final goal error | {m['goal_error_final']:.3f} m |",
              f"| hold drift (max) | {drift_s} |", ""]
    if failed:
        lines += [f"Failed checks: {', '.join(failed)}.", ""]
    lines += ["## Events", "", "| sim time [s] | event | detail |", "|---|---|---|"]
    lines += [f"| {e.t:.2f} | {e.name} | {e.message} |" for e in res.events]
    rp = res.replan
    if rp:
        lines += ["", "## Simulated vs computation time", "",
                  f"Replanning started at t = {rp['t_hold']:.2f} s (simulated). Measured wall-clock compute "
                  f"{rp['measured_compute_s']:.2f} s (map rebuild {rp['map_update_s']:.2f} s + attempts "
                  + ", ".join(f"{a['planner']} {a['status']} {a['compute_s']:.2f} s" for a in rp["attempts"])
                  + f"); the drone held position for {rp['sim_planning_s']:.2f} s of simulated time "
                  "(compute rounded up to the control period). Playback speed in the viewer is independent of both."]
    lines += ["", "## Reading the views", "",
              "- Blue line: desired (validated) trajectory; teal: replanned trajectory; trail: executed path "
              "coloured by navigation mode; magenta segment: desired → actual position (tracking error).",
              "- Wireframe sphere around the drone: 0.25 m collision envelope, coloured green (≥ 0.45 m clearance), "
              "yellow (0.35–0.45), orange (< 0.35, margin violated), red (contact). Faint sphere: 0.35 m.",
              "- Orange/red boxes: obstacles added at run time (orange = exists but not yet detected).",
              "", "## Limitations", "",
              "Translational double integrator only: no attitude, thrust, rotor or aerodynamic dynamics; perfect "
              "state knowledge. Clearance bounds are rigorous for the simulated piecewise-quadratic motion, not a "
              "real-world flight guarantee."]
    return "\n".join(lines) + "\n"


def planning_markdown(ex: dict | None) -> str:
    if not ex:
        return "# Planning exploration\n\nNo exploration data saved for this entry.\n"
    lines = ["# Planning exploration", "",
             "Data from deterministic reruns of the actual planners with their recording flags, on the same map "
             "and 0.45 m trajectory clearance as the visualised plan (no decorative search).", "",
             f"- **A\\*** (26-connected): {ex['astar_nodes_expanded']} expanded nodes "
             f"(colour = expansion order, dark → bright), status `{ex['astar_status']}`, path {ex['astar_length']:.2f} m; "
             f"identical to the executed plan: **{'yes' if ex.get('astar_matches_pipeline') else 'no'}**.",
             f"- **RRT\\*** (seed {ex['rrt_seed']}, {ex['rrt_iterations']} iterations): {ex['rrt_tree_nodes']} tree "
             f"nodes, {ex['rrt_rewires']} rewires, first solution at iteration {ex['rrt_first_solution_iteration']}, "
             f"final path {ex['rrt_length']:.2f} m (shown for comparison; missions execute the A* plan).", ""]
    hist = np.asarray(ex.get("rrt_cost_history", np.zeros((0, 3))))
    if len(hist):
        lines += ["RRT\\* best-cost improvements (convergence):", "", "| iteration | best cost [m] |", "|---|---|"]
        idx = sorted(set(np.linspace(0, len(hist) - 1, min(len(hist), 12)).astype(int)))
        lines += [f"| {int(hist[i, 0])} | {hist[i, 1]:.2f} |" for i in idx]
        lines += [""]
    for k in range(4):
        pl = ex.get(f"replan{k}_planner")
        if pl == "astar":
            lines.append(f"- Replanning attempt {k + 1} (A\\*, from the actual hold point, updated map): "
                         f"{ex[f'replan{k}_astar_nodes_expanded']} expansions, status `{ex[f'replan{k}_astar_status']}`"
                         f"; matches the attempt executed in the run: **{'yes' if ex.get(f'replan{k}_matches_attempt') else 'no'}**.")
        elif pl == "rrt_star":
            lines.append(f"- Replanning attempt {k + 1} (RRT\\*, {ex[f'replan{k}_rrt_iterations']} iterations): "
                         f"{ex[f'replan{k}_rrt_tree_nodes']} nodes, status `{ex[f'replan{k}_rrt_status']}` (red tree).")
    return "\n".join(lines) + "\n"


# ---------------------------------------------------------------- blueprint
def blueprint(scene, has_exploration: bool) -> rrb.Blueprint:
    lo, hi = np.asarray(scene.lo, float), np.asarray(scene.hi, float)
    ctr, size = 0.5 * (lo + hi), hi - lo
    # 55 deg elevation from the -x/-y quadrant, distance scaled to the scene footprint
    el, az = math.radians(55.0), math.radians(-115.0)
    dist = 1.15 * max(size[:2])
    eye = ctr + dist * np.array([math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el)])
    cam = rrb.EyeControls3D(kind=rrb.Eye3DKind.Orbital, position=eye.tolist(), look_target=ctr.tolist(),
                            eye_up=[0, 0, 1])
    nav3d = rrb.Spatial3DView(name="Navigation 3D", origin="/world", eye_controls=cam,
                              line_grid=rrb.LineGrid3D(visible=False))
    views3d = [nav3d]
    if has_exploration:
        views3d.append(rrb.Spatial3DView(
            name="Planning exploration", origin="/",
            contents=["+ /world/scene/**", "+ /world/markers/**", "+ /world/new_obstacles/**", "+ /planning/**"],
            eye_controls=rrb.EyeControls3D(kind=rrb.Eye3DKind.Orbital, position=eye.tolist(),
                                           look_target=ctr.tolist(), eye_up=[0, 0, 1]),
            line_grid=rrb.LineGrid3D(visible=False)))
    left = rrb.Vertical(
        rrb.Tabs(*views3d, active_tab=0),
        rrb.StateTimelineView(name="Navigation state", origin="/nav/mode"),
        row_shares=[6, 1])
    plots = rrb.Vertical(
        rrb.TimeSeriesView(name="Position: desired vs actual [m]", origin="/plots/position"),
        rrb.TimeSeriesView(name="Tracking error", origin="/plots/tracking"),
        rrb.TimeSeriesView(name="Speed [m/s]", origin="/plots/speed"),
        rrb.TimeSeriesView(name="Acceleration [m/s²]", origin="/plots/acceleration"),
        rrb.TimeSeriesView(name="Clearance [m]", origin="/plots/clearance",
                           axis_y=rrb.ScalarAxis(range=(0.0, 1.5))),
        rrb.Tabs(rrb.TextLogView(name="Events", origin="/nav/events"),
                 rrb.TextDocumentView(name="Run summary", origin="/docs/summary"),
                 rrb.TextDocumentView(name="Planning summary", origin="/docs/planning"), active_tab=0),
        row_shares=[1, 1, 1, 1, 1, 1.4])
    return rrb.Blueprint(rrb.Horizontal(left, plots, column_shares=[3, 2]),
                         rrb.TimePanel(expanded=True, timeline=TIMELINE),
                         rrb.SelectionPanel(state="collapsed"), rrb.BlueprintPanel(state="collapsed"))


def build_recording(rec: rr.RecordingStream, res: SimResult, scene, entry: str, ex: dict | None) -> rrb.Blueprint:
    log_scene(rec, scene)
    log_drone_model(rec, res.meta["policy"]["robot_radius"], res.meta["policy"]["nominal"])
    log_run(rec, res, scene)
    if ex:
        log_exploration(rec, ex)
    rec.log("docs/summary", rr.TextDocument(summary_markdown(res, entry, ex), media_type=rr.MediaType.MARKDOWN), static=True)
    rec.log("docs/planning", rr.TextDocument(planning_markdown(ex), media_type=rr.MediaType.MARKDOWN), static=True)
    return blueprint(scene, ex is not None)

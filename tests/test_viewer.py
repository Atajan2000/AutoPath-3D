"""Simulation-to-visualisation synchronisation: the saved .rrd is read back and compared with the SimResult."""

import numpy as np
import pytest

rr = pytest.importorskip("rerun")
pa = pytest.importorskip("pyarrow")
from rerun.chunk import RrdReader  # noqa: E402

from autopath3d.environments import Box  # noqa: E402
from autopath3d.pipeline import ClearancePolicy  # noqa: E402
from autopath3d.replanning import ObstacleUpdate, ReplanningSupervisor, SupervisorConfig  # noqa: E402
from autopath3d.simulation import GUSTS, MODES, Navigator, load_result, save_result  # noqa: E402
from autopath3d.trajectories import TrajectoryLimits  # noqa: E402
from autopath3d.viz.rerun_viewer import build_recording  # noqa: E402

from conftest import make_scene  # noqa: E402

POL, LIM = ClearancePolicy(), TrajectoryLimits(3.0, 2.0, 3.0)


@pytest.fixture(scope="module")
def recorded(tmp_path_factory):
    sc = make_scene([Box((4.8, 0.0, 0.0), (5.2, 1.8, 4.0))], hi=(10, 6, 4))
    sup = ReplanningSupervisor(sc, POL, LIM, SupervisorConfig())
    nom = sup.plan_nominal([1.0, 3.0, 2.0], [9.0, 3.0, 2.0], {"planner": "astar"})
    upd = ObstacleUpdate("w", 1.0, [Box((6.0, 1.8, 0.0), (6.4, 4.5, 4.0), "wall")])
    res = Navigator(sc, POL, LIM, disturbance=GUSTS, supervisor=sup).run(nom.trajectory, [9.0, 3.0, 2.0], upd, 1.0)
    d = tmp_path_factory.mktemp("rrd")
    save_result(res, d, "run")
    res = load_result(d, "run")                       # the viewer works from saved results
    path = d / "run.rrd"
    rec = rr.RecordingStream("test_sync")
    rec.save(str(path))
    build_recording(rec, res, sc, "test", None)
    rec.flush()
    rec.disconnect()
    rows = {}
    for c in RrdReader(str(path)).stream():
        if "sim_time" not in c.timeline_names:
            continue
        rb = c.to_record_batch()
        ns = rb.column(rb.schema.get_field_index("sim_time")).cast(pa.int64()).to_numpy()
        for key in rb.schema.names:
            if ":" not in key:                       # index columns (sim_time, log_time, ...) and RowId
                continue
            vals = rb.column(rb.schema.get_field_index(key)).to_pylist()
            rows.setdefault((c.entity_path, key), []).extend(zip(ns.tolist(), vals))
    return res, rows


def _series(rows, entity, comp):
    """(time stamps in integer ns, values), sorted by time."""
    pts = sorted(rows[(entity, comp)], key=lambda x: x[0])
    return np.array([t for t, _ in pts], dtype=np.int64), [v for _, v in pts]


def _ns(t):
    return np.round(np.asarray(t, float) * 1e9).astype(np.int64)


def test_drone_pose_matches_simulated_state(recorded):
    res, rows = recorded
    t, vals = _series(rows, "/world/drone", "Transform3D:translation")
    A = res.arrays
    assert np.array_equal(t, _ns(A["t"]))
    assert np.allclose(np.array([v[0] for v in vals]), A["p"], atol=1e-5)    # float32 storage


def test_plots_match_simulation_outputs(recorded):
    res, rows = recorded
    A = res.arrays
    t, vals = _series(rows, "/plots/clearance/executed", "Scalars:scalars")
    assert np.array_equal(t, _ns(A["t"][:-1]))
    assert np.allclose([v[0] for v in vals], A["clr_lo"])
    t, vals = _series(rows, "/plots/speed/actual", "Scalars:scalars")
    assert np.allclose([v[0] for v in vals], np.linalg.norm(A["v"], axis=1))


def test_state_lane_and_event_log_use_event_times(recorded):
    res, rows = recorded
    A = res.arrays
    t, vals = _series(rows, "/nav/mode", "StateChange:state")
    states = [v[0] for v in vals]
    trans = np.flatnonzero(np.r_[True, A["mode"][1:] != A["mode"][:-1]])
    assert np.array_equal(t[: len(trans)], _ns(A["t"][trans]))
    assert states[: len(trans)] == [MODES[A["mode"][k]] for k in trans]
    assert states[-1] == res.meta["final_mode"] == "GOAL_REACHED"
    t, vals = _series(rows, "/nav/events", "TextLog:text")
    assert np.array_equal(t, _ns([e.t for e in res.events]))
    assert [v.split(":")[0] if isinstance(v, str) else v[0].split(":")[0] for v in vals] == res.event_names()


def test_new_obstacle_appears_only_at_its_time(recorded):
    res, rows = recorded
    t, _ = _series(rows, "/world/new_obstacles", "Boxes3D:half_sizes")
    assert t.min() == _ns(res.meta["update"]["t_appear"])

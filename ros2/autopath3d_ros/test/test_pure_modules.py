"""Pure-Python tests of the Phase 4 integration modules (no ROS / Webots needed)."""

import math

import numpy as np
import pytest

from autopath3d.collision.checker import RobotModel
from autopath3d.environments.primitives import Box
from autopath3d.environments.scenes import Scene
from autopath3d.pipeline import ClearancePolicy, run_pipeline, trajectory_checker
from autopath3d.replanning.braking import braking_trajectory
from autopath3d.trajectories.generator import TrajectoryLimits
from autopath3d_ros import geometry, trajcodec, tracking
from autopath3d_ros.webots_extract import axis_angle_matrix, box_aabb, cylinder_aabb, local_transform


# ------------------------------------------------------------------ trajectory codec
def _small_traj():
    sc = Scene("t", "", np.array([0.0, 0, 0]), np.array([6.0, 4, 3]), [Box((2.5, 0, 0), (3.5, 2.6, 3), "w")], [])
    pol = ClearancePolicy(0.1, 0.1, 0.1)
    lim = TrajectoryLimits(1.0, 0.8, 1.0)
    _, ck = trajectory_checker(sc, 0.1, pol)
    res = run_pipeline(sc, ck, (1.0, 1.0, 1.0), (5.0, 1.0, 1.5), {"planner": "astar"}, lim, pol)
    assert res.status == "success", res.reason
    return res.trajectory


def test_codec_reproduces_certified_trajectory_exactly():
    tr = _small_traj()
    t, P, V, A = trajcodec.encode(tr)
    back = trajcodec.decode(t, P, V, A)
    assert back.n_pieces == tr.n_pieces
    assert trajcodec.max_deviation(tr, back) < 1e-9
    ts = np.linspace(tr.t_start, tr.t_end, 777)
    assert np.abs(tr.evaluate(ts, 1) - back.evaluate(ts, 1)).max() < 1e-8
    assert np.abs(tr.evaluate(ts, 2) - back.evaluate(ts, 2)).max() < 1e-7


def test_codec_braking_and_stationary():
    br = braking_trajectory([1, 2, 3], [0.6, -0.8, 0.0], 1.5, 10.0)
    back = trajcodec.decode(*trajcodec.encode(br))
    assert trajcodec.max_deviation(br, back) < 1e-12
    st = trajcodec.decode(*trajcodec.encode(braking_trajectory([1, 2, 3], [0, 0, 0], 1.5, 4.0)))
    assert st.n_pieces == 0 and np.allclose(st.hold_point, [1, 2, 3]) and st.t_start == 4.0


def test_codec_rejects_unordered_knots():
    with pytest.raises(ValueError):
        trajcodec.decode([0, 1, 1], np.zeros((3, 3)), np.zeros((3, 3)), np.zeros((3, 3)))


# ------------------------------------------------------------------ outer loop
def test_outer_loop_frames_and_bounds():
    g = tracking.OuterLoopGains(kp_xy=1.0, accel_lead_s=0.0, z_lead_s=0.0, v_xy_max=0.8)
    # reference 0.5 m/s east, drone yawed +90 deg (facing north): east is body -y (right)
    c = tracking.outer_loop([0, 0, 1], [0.5, 0, 0], [0, 0, 0], [0, 0, 1], math.pi / 2, g, yaw_ref=math.pi / 2)
    assert c.vx_body == pytest.approx(0.0, abs=1e-12) and c.vy_body == pytest.approx(-0.5)
    # large error saturates to v_xy_max, direction preserved
    c = tracking.outer_loop([10, 10, 1], [0, 0, 0], [0, 0, 0], [0, 0, 1], 0.0, g)
    assert c.saturated and math.hypot(c.vx_body, c.vy_body) == pytest.approx(0.8)
    assert c.vx_body == pytest.approx(c.vy_body)
    # altitude set-point bounded around the current altitude
    c = tracking.outer_loop([0, 0, 5], [0, 0, 0], [0, 0, 0], [0, 0, 1], 0.0, g)
    assert c.z_setpoint == pytest.approx(1.0 + g.z_step_max)
    # yaw-rate bound and wrapping (target -179 deg from +179 deg is +2 deg, not -358)
    c = tracking.outer_loop([0, 0, 1], [0, 0, 0], [0, 0, 0], [0, 0, 1], math.radians(179), g,
                            yaw_ref=math.radians(-179))
    assert 0 < c.yaw_rate <= g.yaw_rate_max


def test_brake_command_zero_velocity():
    g = tracking.OuterLoopGains()
    c = tracking.brake_command(np.array([1, 2, 1.5]), 1.5, 0.3, g)
    assert (c.vx_body, c.vy_body) == (0.0, 0.0) and c.z_setpoint == 1.5


def test_invalid_gains():
    with pytest.raises(ValueError):
        tracking.OuterLoopGains(v_xy_max=0.0)


# ------------------------------------------------------------------ geometry / extraction math
def test_axis_angle_and_aabb():
    R = axis_angle_matrix((0, 0, 1, math.pi / 2))
    assert np.allclose(R @ [1, 0, 0], [0, 1, 0])
    T = local_transform((1, 2, 3), (0, 0, 1, math.pi / 2))
    lo, hi = box_aabb(T, (2.0, 0.2, 1.0))           # rotated 90 deg about z: x-extent 0.2, y-extent 2
    assert np.allclose(lo, [0.9, 1.0, 2.5]) and np.allclose(hi, [1.1, 3.0, 3.5])
    # horizontal cylinder (axis rotated to x): AABB extends +-h/2 in x, +-r in y, z
    T = local_transform((0, 0, 1), (0, 1, 0, math.pi / 2))
    lo, hi = cylinder_aabb(T, 0.1, 2.0)
    assert np.allclose(lo, [-1, -0.1, 0.9]) and np.allclose(hi, [1, 0.1, 1.1])


def test_scene_from_records_and_envelopes():
    recs = [
        {"object": "rack 0", "solid": "rack 0", "kind": "box", "lo": [0, 0, 0], "hi": [0.1, 1, 3], "source": "collision"},
        {"object": "rack 0", "solid": "rack 0", "kind": "box", "lo": [2.9, 0, 0], "hi": [3, 1, 3], "source": "collision"},
        {"object": "barrel", "solid": "barrel", "kind": "cylinder", "center": [5, 5], "radius": 0.3, "z": [0, 0.9],
         "source": "collision"},
        {"object": "hidden", "solid": "hidden", "kind": "box", "lo": [0, 0, -6], "hi": [1, 1, -4], "source": "collision"},
        {"object": "thin", "solid": "thin", "kind": "box", "lo": [1, 1, 1], "hi": [2, 2, 1], "source": "collision"},
    ]
    sc = geometry.scene_from_records(recs, [-1, -1, 0], [8, 8, 4], envelope_patterns=["rack *"])
    names = [o.name for o in sc.obstacles]
    assert len(sc.obstacles) == 5            # 2 posts + barrel + thin + envelope; hidden dropped
    env = [o for o in sc.obstacles if o.name.endswith("rack 0/rack 0") and o.hi[0] == 3]
    assert env and env[-1].lo == (0.0, 0.0, 0.0)
    # bay between the posts is free physically but occupied in the planning scene
    assert sc.distance(np.array([[1.5, 0.5, 1.5]]))[0] == 0.0
    thin = [o for o in sc.obstacles if "thin" in o.name][0]
    assert thin.hi[2] - thin.lo[2] > 0
    assert names


def test_compare_records():
    a = [{"object": "o", "solid": "s", "kind": "box", "lo": [0, 0, 0], "hi": [1, 1, 1]}]
    b = [{"object": "o", "solid": "s", "kind": "box", "lo": [0, 0, 0], "hi": [1, 1, 1.01]}]
    assert geometry.compare_records(a, a) == []
    assert geometry.compare_records(a, b)

"""Phase 4: conservative, disturbance-aware stopping assessment."""

import numpy as np
import pytest

from autopath3d.environments.primitives import Box
from autopath3d.replanning.stopping_assessment import StopClass, StoppingAssumptions, assess_stopping

WALL = Box((10.0, -5.0, -5.0), (10.5, 5.0, 5.0), "wall")      # face at x = 10


def dist(p):
    return WALL.distance(p)


def nominal(**kw):
    a = dict(latency_s=0.3, latency_min_s=0.3, brake_decel=3.0, brake_decel_max=3.0,
             disturbance_accel=0.0, tracking_accel=0.0, position_error=0.0)
    a.update(kw)
    return StoppingAssumptions(**a)


def test_exact_model_matches_phase2_baseline_stopping_distance():
    # no disturbance, no uncertainty: D = v t_L + v^2 / (2 a)
    r = assess_stopping([0, 0, 0], [2.0, 0, 0], dist, 0.25, 0.35, nominal())
    assert r.worst_stop_distance == pytest.approx(2.0 * 0.3 + 4.0 / 6.0)
    assert r.best_stop_distance == pytest.approx(r.worst_stop_distance)
    assert r.lateral_radius == 0.0
    assert r.clearance_lower == pytest.approx(10.0 - r.worst_stop_distance, abs=0.011)
    assert r.classification == StopClass.SAFE_STOP_FEASIBLE


# v = 2.5: v_L = 2.5 + 2.4*0.3 = 3.22, D_max = 0.858 + 3.22^2/4.2 = 3.327, T = 1.833,
# rho = 0.05 + 0.2*T^2 = 0.722  -> lower = gap - 4.049;  D_min = 0.5 + 6.25/7.8 = 1.301,
# upper = gap - 0.579.  Thresholds: SAFE gap >= 4.399, CNG >= 4.299, POSSIBLE >= 0.829.
@pytest.mark.parametrize("gap,expected", [
    (4.60, StopClass.SAFE_STOP_FEASIBLE),          # lower bound >= 0.35
    (4.35, StopClass.CLEARANCE_NOT_GUARANTEED),    # 0.25 <= lower < 0.35
    (2.00, StopClass.COLLISION_POSSIBLE),          # lower < 0.25, best case still clear
    # gap 0.7 m: the best case reaches the wall, but rho = 0.72 m of admitted lateral drift
    # exceeds the robot radius, so "unavoidable" cannot be certified -> still POSSIBLE
    (0.70, StopClass.COLLISION_POSSIBLE),
])
def test_classes_ordered_by_gap(gap, expected):
    a = nominal(latency_min_s=0.2, brake_decel=2.5, brake_decel_max=3.5, disturbance_accel=0.4,
                tracking_accel=2.0, position_error=0.05)
    p = np.array([10.0 - gap, 0, 0])
    r = assess_stopping(p, [2.5, 0, 0], dist, 0.25, 0.35, a)
    assert r.clearance_lower <= r.clearance_upper
    assert r.classification == expected, r.to_dict()


def test_unavoidable_needs_tight_bounds():
    a = nominal(latency_min_s=0.2, brake_decel=2.5, brake_decel_max=3.5, disturbance_accel=0.05,
                tracking_accel=0.0, position_error=0.02)
    r = assess_stopping([9.3, 0, 0], [2.5, 0, 0], dist, 0.25, 0.35, a)
    assert r.clearance_upper < 0.25
    assert r.classification == StopClass.COLLISION_UNAVOIDABLE


def test_bounds_contain_simulated_worst_and_best_realisations():
    """Simulate straight-line stops at the extreme parameter values; the attained minimum
    clearance must lie inside [clearance_lower, clearance_upper]."""
    a = nominal(latency_min_s=0.15, brake_decel=2.0, brake_decel_max=3.0, disturbance_accel=0.4,
                tracking_accel=1.0, position_error=0.03)
    p0, v0 = np.array([6.0, 0.3, 0.0]), np.array([2.0, 0.0, 0.0])
    r = assess_stopping(p0, v0, dist, 0.25, 0.35, a)
    for lat, acc, dec, dis in [(a.latency_s, a.tracking_accel, a.brake_decel, a.disturbance_accel),
                               (a.latency_min_s, 0.0, a.brake_decel_max, -a.disturbance_accel)]:
        dt, p, v, t, dmin = 1e-3, p0.copy(), v0.copy(), 0.0, np.inf
        while True:
            if t < lat:
                acc_t = acc + dis
            else:
                if v[0] <= 0:
                    break
                acc_t = -dec + dis
            v = v + np.array([acc_t * dt, 0, 0]); p = p + v * dt; t += dt
            dmin = min(dmin, dist(p[None])[0])
        assert r.clearance_lower - 1e-3 <= dmin <= r.clearance_upper + 1e-3


def test_braking_not_guaranteed_when_disturbance_exceeds_braking():
    r = assess_stopping([0, 0, 0], [1.0, 0, 0], dist, 0.25, 0.35,
                        nominal(brake_decel=0.3, disturbance_accel=0.5))
    assert r.classification == StopClass.BRAKING_NOT_GUARANTEED


def test_hovering_robot_uses_position_error_only():
    r = assess_stopping([9.5, 0, 0], [0, 0, 0], dist, 0.25, 0.35, nominal(position_error=0.1))
    assert r.worst_stop_distance == 0.0
    assert r.clearance_lower == pytest.approx(0.5 - 0.1, abs=1e-9)
    assert r.classification == StopClass.SAFE_STOP_FEASIBLE


def test_invalid_assumptions_rejected():
    with pytest.raises(ValueError):
        nominal(latency_min_s=0.5)
    with pytest.raises(ValueError):
        nominal(brake_decel=4.0)

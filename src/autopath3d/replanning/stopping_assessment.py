"""Conservative, disturbance-aware stopping assessment (Phase 4).

The Phase 2 supervisor predicts one nominal stop (exact latency, exact braking deceleration, no
disturbance, perfect state). Phase 3 scenario D2 showed that prediction can be optimistic by
several centimetres under wind. This module does NOT change that supervisor or its results; it
adds a separate assessment that bounds every motion admitted by an explicit set of assumptions.

Model (translational, straight-line braking policy, measured state)
-------------------------------------------------------------------
  p, v           measured position and velocity when the update is received
  e_pos          bound on |true position - measured position| (estimation + tracking error)
  e_vel          bound on |true velocity - measured velocity|
  t_lat          bound on the time until braking deceleration is actually applied
                 (detection + decision + communication + actuator response), [t_lat_min, t_lat]
  a_track        max commanded acceleration while still tracking during the latency
  d_max          bound on the disturbance / unmodelled acceleration, any direction
  a_brk          guaranteed braking deceleration (worst case), a_brk_max best case
Worst case along the velocity direction:
  v_L   = |v| + e_vel + (a_track + d_max) t_lat
  D_max = (|v| + e_vel) t_lat + (a_track + d_max) t_lat^2 / 2 + v_L^2 / (2 (a_brk - d_max))
  lateral drift   delta = d_max T^2 / 2 + e_vel T,  T = t_lat + v_L / (a_brk - d_max)
Every admitted motion stays in the capsule  C = {p + s u : 0 <= s <= D_max} (+) ball(rho),
u = v/|v|, rho = e_pos + delta. Because the true distance d to the geometry is 1-Lipschitz:
  clearance_lower = min_{s in [0, D_max]} d(p + s u) - rho              (certified lower bound)
Best case (minimum latency, best braking, helpful disturbance, no lateral drift):
  D_min = max(|v| - e_vel, 0) t_lat_min + max(|v| - e_vel, 0)^2 / (2 (a_brk_max + d_max))
Every admitted motion passes along-track progress s for all s <= D_min, while staying within
rho of the axis, hence
  clearance_upper = min_{s in [0, D_min]} d(p + s u) + rho              (bound for ALL motions)

Classification
  SAFE_STOP_FEASIBLE          clearance_lower >= required clearance (r + margin)
  CLEARANCE_NOT_GUARANTEED    robot radius <= clearance_lower < required clearance
  COLLISION_POSSIBLE          clearance_lower < radius <= clearance_upper
  COLLISION_UNAVOIDABLE       clearance_upper < radius (every admitted motion makes contact)
  BRAKING_NOT_GUARANTEED      a_brk <= d_max: the model cannot bound the stopping distance
A route-level NO_SAFE_ALTERNATIVE_ROUTE is decided by the replanner, not here.

Limits: straight-line braking only (no evasive steering), static obstacles, the assumption
bounds must be justified for the vehicle (see the Webots characterisation); a warning is not an
avoidance manoeuvre. Point-mass parameters do not automatically hold for a physical quadrotor.
"""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass
from enum import Enum

import numpy as np


class StopClass(str, Enum):
    SAFE_STOP_FEASIBLE = "safe_stop_feasible"
    CLEARANCE_NOT_GUARANTEED = "clearance_not_guaranteed"
    COLLISION_POSSIBLE = "collision_possible"
    COLLISION_UNAVOIDABLE = "collision_unavoidable"
    BRAKING_NOT_GUARANTEED = "braking_not_guaranteed"


@dataclass(frozen=True)
class StoppingAssumptions:
    latency_s: float               # worst-case time until braking deceleration acts
    latency_min_s: float           # best-case
    brake_decel: float             # guaranteed (worst-case) braking deceleration, m/s^2
    brake_decel_max: float         # best-case braking deceleration, m/s^2
    disturbance_accel: float       # bound on disturbance / unmodelled acceleration, m/s^2
    tracking_accel: float          # max commanded acceleration during the latency, m/s^2
    position_error: float          # bound on true-vs-measured position, m
    velocity_error: float = 0.0    # bound on true-vs-measured velocity, m/s

    def __post_init__(self):
        if self.latency_min_s > self.latency_s or self.brake_decel > self.brake_decel_max:
            raise ValueError("best-case bounds must not be worse than worst-case bounds")
        if min(self.latency_min_s, self.brake_decel, self.disturbance_accel, self.tracking_accel,
               self.position_error, self.velocity_error) < 0:
            raise ValueError("assumption bounds must be non-negative")


@dataclass
class StoppingAssessment:
    classification: StopClass
    speed: float
    direction: list
    worst_stop_distance: float      # D_max along the velocity direction
    best_stop_distance: float       # D_min
    lateral_radius: float           # rho
    worst_stop_time: float
    clearance_lower: float          # certified lower bound over all admitted motions
    clearance_upper: float          # upper bound valid for every admitted motion
    required_clearance: float
    robot_radius: float
    worst_stop_point: list
    assumptions: dict

    def to_dict(self) -> dict:
        d = asdict(self)
        d["classification"] = self.classification.value
        return d


def _segment_min(dist_fn, p: np.ndarray, u: np.ndarray, length: float, step: float) -> float:
    """Certified lower bound of min_{s in [0, length]} d(p + s u) for a 1-Lipschitz d."""
    if length <= 0.0:
        return float(dist_fn(p[None])[0])
    n = max(2, int(math.ceil(length / step)) + 1)
    s = np.linspace(0.0, length, n)
    d = np.asarray(dist_fn(p[None] + s[:, None] * u[None]), float)
    h = length / (n - 1)
    # on [s_i, s_i+1]: d >= (d_i + d_i+1 - h) / 2
    return float(min(d.min(), np.min(0.5 * (d[:-1] + d[1:] - h))))


def _segment_min_upper(dist_fn, p: np.ndarray, u: np.ndarray, length: float, step: float) -> float:
    """Upper bound of min_{s in [0, length]} d(p + s u): the smallest sampled value."""
    n = max(2, int(math.ceil(max(length, 0.0) / step)) + 1)
    s = np.linspace(0.0, max(length, 0.0), n)
    return float(np.min(dist_fn(p[None] + s[:, None] * u[None])))


def assess_stopping(position, velocity, dist_fn, robot_radius: float, required_clearance: float,
                    a: StoppingAssumptions, step: float = 0.01) -> StoppingAssessment:
    """Classify the emergency stop from the measured state against the geometry `dist_fn`
    (exact distance to obstacles, 1-Lipschitz, e.g. Scene.distance of the updated scene)."""
    p = np.asarray(position, float)
    v = np.asarray(velocity, float)
    speed = float(np.linalg.norm(v))
    u = v / speed if speed > 1e-9 else np.array([1.0, 0.0, 0.0])
    eff = a.brake_decel - a.disturbance_accel
    if eff <= 0:
        lower = _segment_min(dist_fn, p, u, 0.0, step) - a.position_error
        return StoppingAssessment(StopClass.BRAKING_NOT_GUARANTEED, speed, u.tolist(), math.inf, 0.0, math.inf,
                                  math.inf, lower, math.inf, required_clearance, robot_radius,
                                  [math.nan] * 3, asdict(a))
    v0 = speed + a.velocity_error
    acc_l = a.tracking_accel + a.disturbance_accel
    v_l = v0 + acc_l * a.latency_s
    d_lat = v0 * a.latency_s + 0.5 * acc_l * a.latency_s ** 2
    t_brk = v_l / eff
    d_max = d_lat + v_l * v_l / (2.0 * eff)
    T = a.latency_s + t_brk
    rho = a.position_error + 0.5 * a.disturbance_accel * T * T + a.velocity_error * T
    if speed <= 1e-9 and a.velocity_error == 0.0 and acc_l == 0.0:
        d_max, rho = 0.0, a.position_error
    v_best = max(speed - a.velocity_error, 0.0)
    d_min = v_best * a.latency_min_s + v_best * v_best / (2.0 * (a.brake_decel_max + a.disturbance_accel))
    lower = _segment_min(dist_fn, p, u, d_max, step) - rho
    upper = _segment_min_upper(dist_fn, p, u, d_min, step) + rho
    if lower >= required_clearance:
        cls = StopClass.SAFE_STOP_FEASIBLE
    elif lower >= robot_radius:
        cls = StopClass.CLEARANCE_NOT_GUARANTEED
    elif upper >= robot_radius:
        cls = StopClass.COLLISION_POSSIBLE
    else:
        cls = StopClass.COLLISION_UNAVOIDABLE
    return StoppingAssessment(cls, speed, u.tolist(), d_max, d_min, rho, T, lower, upper, required_clearance,
                              robot_radius, (p + d_max * u).tolist(), asdict(a))

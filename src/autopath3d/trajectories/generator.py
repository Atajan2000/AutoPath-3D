"""Time-parameterised, collision-certified trajectories through a certified polyline.

Kinematic model (documented assumption): the robot centre is a translational double
integrator with |v| <= v_max and |a| <= a_max (Euclidean norms). Attitude, thrust limits
and rotor dynamics are NOT modelled.

Construction:
  * one quintic Hermite piece per polyline segment; acceleration is zero at every
    waypoint, so position, velocity and acceleration are continuous (C2);
  * start and goal are rest states (v = a = 0);
  * interior waypoint velocity = lambda_j * cos^2(theta_j / 2) * min(adjacent L/T) along
    the corner bisector (theta = turning angle; U-turns get zero speed);
  * durations start from the rest-to-rest minimum-jerk bounds and are stretched until the
    certified speed/acceleration bounds hold;
  * every curved piece is certified collision-free in continuous time (Bezier hull +
    1-Lipschitz distance, branch and bound). A rejected piece halves lambda at its
    endpoints; lambda = 0 on both ends makes the piece a rest-to-rest motion lying
    exactly on the (already certified) polyline segment — the conservative fallback.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field

import numpy as np

from ..collision.checker import CollisionChecker
from .polynomial import PiecewisePolynomial, bezier_derivative, power_to_bernstein, quintic_hermite, split_half
from .shortcut import dedupe

# rest-to-rest minimum-jerk peaks: v = 1.875 L/T, a = (10/sqrt(3)) L/T^2
_KV, _KA = 1.875, 10.0 / math.sqrt(3.0)


@dataclass(frozen=True)
class TrajectoryLimits:
    max_speed: float = 3.0           # m/s
    max_acceleration: float = 2.0    # m/s^2, bound on |a| for nominal trajectories
    max_deceleration: float = 3.0    # m/s^2, emergency braking capability (>= max_acceleration)
    limit_margin: float = 0.97       # nominal design targets this fraction of the limits

    def __post_init__(self):
        if self.max_deceleration < self.max_acceleration:
            raise ValueError("max_deceleration must be >= max_acceleration (|a| bound covers braking)")


@dataclass
class GenerationResult:
    success: bool
    trajectory: PiecewisePolynomial | None
    reason: str = ""
    generation_time_s: float = 0.0
    rejected_candidates: int = 0       # curved pieces rejected by the collision certificate
    timing_adjustments: int = 0        # duration stretches to satisfy limits
    rounds: int = 0
    fallback_pieces: int = 0           # pieces executed as rest-to-rest straight motions
    lambdas: list = field(default_factory=list)
    speed_bound: float = float("nan")  # certified upper bounds over the whole trajectory
    accel_bound: float = float("nan")
    duration: float = float("nan")

    def summary(self) -> dict:
        d = dict(self.__dict__)
        d.pop("trajectory")
        return d


# ---------------------------------------------------------------- certified bounds
def max_norm_bound(Q: np.ndarray, limit: float, max_depth: int = 8) -> float:
    """Certified upper bound on max_u |B(u)| for a Bezier curve Q (convex-hull property).

    Subdivides only while the bound exceeds `limit`, so the result is either <= limit
    (certified) or the tightest bound found at max_depth.
    """
    nodes = Q[None]
    ub_done = 0.0
    for depth in range(max_depth + 1):
        ub = np.linalg.norm(nodes, axis=2).max(axis=1)
        ok = ub <= limit
        if ok.any():
            ub_done = max(ub_done, float(ub[ok].max()))
        if ok.all() or depth == max_depth:
            return max(ub_done, float(ub.max()))
        l, r, _ = split_half(nodes[~ok])
        nodes = np.concatenate([l, r])
    return float("inf")


def certify_curve(P: np.ndarray, dist_fn, clearance: float, max_depth: int = 16,
                  min_radius: float = 1e-3) -> bool:
    """True iff the Bezier curve P provably keeps dist_fn >= clearance everywhere.

    dist_fn must return a lower bound of a 1-Lipschitz distance. Each node is enclosed in
    a ball centred on its curve midpoint m with radius R = max |P_j - m|; the node is
    certified when dist(m) - R >= clearance. A curve point with dist < clearance rejects.
    """
    nodes = P[None]
    for _ in range(max_depth + 1):
        l, r, m = split_half(nodes)
        R = np.linalg.norm(nodes - m[:, None, :], axis=2).max(axis=1)
        d = dist_fn(m)
        if np.any(d < clearance):
            return False
        todo = d - R < clearance
        if not todo.any():
            return True
        if np.any(R[todo] < min_radius):
            return False
        nodes = np.concatenate([l[todo], r[todo]])
    return False


def curve_min_bounds(P: np.ndarray, dist_fn, tol: float = 1e-3, max_nodes: int = 200_000) -> tuple[float, float]:
    """Certified [lower, upper] on min dist_fn over a Bezier curve (dist_fn 1-Lipschitz)."""
    nodes = P[None]
    upper, lower_done, used = np.inf, np.inf, 0
    while len(nodes):
        l, r, m = split_half(nodes)
        R = np.linalg.norm(nodes - m[:, None, :], axis=2).max(axis=1)
        d = dist_fn(m)
        upper = min(upper, float(d.min()), float(dist_fn(nodes[:, 0]).min()), float(dist_fn(nodes[:, -1]).min()))
        lb = d - R
        keep = lb < upper - tol
        if (~keep).any():
            lower_done = min(lower_done, float(lb[~keep].min()))
        used += len(nodes)
        if used > max_nodes:
            return min(lower_done, float(lb.min())), upper
        nodes = np.concatenate([l[keep], r[keep]])
    return min(lower_done, upper), upper


# ---------------------------------------------------------------- generator
def _waypoint_velocities(W, L, D, T, lam):
    m = len(L)
    V = np.zeros((m + 1, 3))
    for j in range(1, m):
        if lam[j] <= 0.0:
            continue
        b = D[j - 1] + D[j]
        nb = np.linalg.norm(b)
        if nb < 1e-9:
            continue
        corner = 0.5 * (1.0 + float(D[j - 1] @ D[j]))         # cos^2(theta/2)
        speed = lam[j] * corner * min(L[j - 1] / T[j - 1], L[j] / T[j])
        V[j] = speed * b / nb
    return V


def generate_trajectory(waypoints: np.ndarray, checker: CollisionChecker, limits: TrajectoryLimits,
                        t0: float = 0.0, max_rounds: int = 25, max_stretch: int = 80) -> GenerationResult:
    """Rest-to-rest trajectory through `waypoints` (which must be certified by `checker`)."""
    tic = time.perf_counter()
    W = dedupe(waypoints)
    if len(W) == 1:
        tr = PiecewisePolynomial.stationary(W[0], t0)
        return GenerationResult(True, tr, "zero-length path", time.perf_counter() - tic,
                                speed_bound=0.0, accel_bound=0.0, duration=0.0)
    if not checker.path_free(W):
        return GenerationResult(False, None, "reference polyline not certified at required clearance",
                                time.perf_counter() - tic)
    vmax, amax = limits.max_speed, limits.max_acceleration
    vt, at = limits.limit_margin * vmax, limits.limit_margin * amax
    seg = np.diff(W, axis=0)
    L = np.linalg.norm(seg, axis=1)
    D = seg / L[:, None]
    m = len(L)
    T = np.maximum(_KV * L / vt, np.sqrt(_KA * L / at))
    lam = np.ones(m + 1)
    lam[0] = lam[m] = 0.0
    rejected = adjust = 0
    dist = checker.clearance
    c = checker.c

    for rnd in range(1, max_rounds + 1):
        if rnd == max_rounds:
            lam[:] = 0.0          # last round: guaranteed rest-to-rest fallback
        # 1) stretch durations until certified limits hold
        for _ in range(max_stretch):
            V = _waypoint_velocities(W, L, D, T, lam)
            bad = []
            for i in range(m):
                P = power_to_bernstein(quintic_hermite(W[i], V[i], 0, W[i + 1], V[i + 1], 0, T[i]), T[i])
                Pv = bezier_derivative(P, T[i])
                Pa = bezier_derivative(Pv, T[i])
                if max_norm_bound(Pv, vmax) > vmax or max_norm_bound(Pa, amax) > amax:
                    bad.append(i)
            if not bad:
                break
            T[bad] *= 1.15
            adjust += len(bad)
        else:
            lam[:] = 0.0          # cannot satisfy limits with corner speeds: rest-to-rest
            continue
        # 2) certify curved pieces against the voxel map at the required clearance
        failed = []
        for i in range(m):
            if not V[i].any() and not V[i + 1].any():
                continue          # straight rest-to-rest motion on a certified segment
            P = power_to_bernstein(quintic_hermite(W[i], V[i], 0, W[i + 1], V[i + 1], 0, T[i]), T[i])
            if not certify_curve(P, dist, c):
                failed.append(i)
        if not failed:
            pieces = [quintic_hermite(W[i], V[i], 0, W[i + 1], V[i + 1], 0, T[i]) for i in range(m)]
            tr = PiecewisePolynomial.from_pieces(pieces, list(T), t0)
            sb = ab = 0.0
            for i in range(m):
                P = tr.bernstein(i)
                Pv = bezier_derivative(P, T[i])
                sb = max(sb, max_norm_bound(Pv, vmax))
                ab = max(ab, max_norm_bound(bezier_derivative(Pv, T[i]), amax))
            fb = sum(1 for i in range(m) if not V[i].any() and not V[i + 1].any())
            return GenerationResult(True, tr, "", time.perf_counter() - tic, rejected, adjust, rnd, fb,
                                    lam.tolist(), sb, ab, tr.duration)
        rejected += len(failed)
        for i in failed:
            for j in (i, i + 1):
                lam[j] = 0.0 if lam[j] <= 0.25 else 0.5 * lam[j]
    return GenerationResult(False, None, "no certified trajectory within round budget",
                            time.perf_counter() - tic, rejected, adjust, max_rounds)

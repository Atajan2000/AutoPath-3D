"""Independent validation of piecewise-polynomial trajectories.

Uses only the raw knot times and power-basis coefficients; it re-implements polynomial
evaluation, the Bernstein conversion and subdivision itself and does not call the
generator, the CollisionChecker or the clearance field.

Continuous-time clearance (justification): on each piece the trajectory is a Bezier
curve contained in the convex hull of its control points, hence in the ball centred at
the curve midpoint m with radius R = max_j |P_j - m|. The exact ground-truth distance f
(obstacles + bounds) is 1-Lipschitz, so  min f >= f(m) - R  over that sub-curve, while
f(m) itself is attained. Branch-and-bound subdivision yields a certified interval
[lower, upper] for the minimum clearance. Speed and acceleration maxima are bounded the
same way from the hull of the derivative curves (norm is convex). No fixed-step sampling
is used as a guarantee.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from math import comb

import numpy as np


def _bern(c: np.ndarray, T: float) -> np.ndarray:
    n = len(c) - 1
    a = c * (T ** np.arange(n + 1))[:, None]
    B = np.array([[comb(i, k) / comb(n, k) if k <= i else 0.0 for k in range(n + 1)] for i in range(n + 1)])
    return B @ a


def _split(P: np.ndarray):
    n1 = P.shape[1]
    L, R = np.empty_like(P), np.empty_like(P)
    cur = P
    for r in range(n1):
        L[:, r] = cur[:, 0]
        R[:, n1 - 1 - r] = cur[:, -1]
        cur = 0.5 * (cur[:, :-1] + cur[:, 1:])
    return L, R


def _eval(c: np.ndarray, tau: float, order: int) -> np.ndarray:
    out = np.zeros(3)
    for k in range(order, len(c)):
        f = np.prod(np.arange(k - order + 1, k + 1)) if order else 1.0
        out += f * c[k] * tau ** (k - order)
    return out


def _min_over_curve(P, f, tol, max_nodes=400_000):
    """Certified [lower, upper] for min f over a Bezier curve; f is 1-Lipschitz."""
    nodes = P[None]
    upper = np.inf
    lower_done = np.inf
    used = 0
    while len(nodes):
        L, R = _split(nodes)
        m = L[:, -1]
        rad = np.linalg.norm(nodes - m[:, None, :], axis=2).max(axis=1)
        fm = f(m)
        upper = min(upper, float(fm.min()), float(f(nodes[:, 0]).min()), float(f(nodes[:, -1]).min()))
        lb = fm - rad
        keep = lb < upper - tol
        if (~keep).any():
            lower_done = min(lower_done, float(lb[~keep].min()))
        used += len(nodes)
        if used > max_nodes:
            return min(lower_done, float(lb.min())), upper
        nodes = np.concatenate([L[keep], R[keep]])
    return min(lower_done, upper), upper


def _max_norm(Q, tol, max_depth=30):
    """Certified [lower, upper] for max |Q(u)|."""
    nodes = Q[None]
    lower = float(max(np.linalg.norm(Q[0]), np.linalg.norm(Q[-1])))
    upper_done = 0.0
    for _ in range(max_depth):
        L, R = _split(nodes)
        lower = max(lower, float(np.linalg.norm(L[:, -1], axis=1).max()))
        ub = np.linalg.norm(nodes, axis=2).max(axis=1)
        keep = ub > lower + tol
        if (~keep).any():
            upper_done = max(upper_done, float(ub[~keep].max()))
        if not keep.any():
            return lower, max(upper_done, lower)
        nodes = np.concatenate([L[keep], R[keep]])
    return lower, max(upper_done, float(np.linalg.norm(nodes, axis=2).max()))


def _polyline_distance(path: np.ndarray):
    a, b = path[:-1], path[1:]
    ab = b - a
    ll = np.maximum((ab * ab).sum(1), 1e-300)

    def f(p):
        p = np.atleast_2d(p)
        t = np.clip(((p[:, None, :] - a[None]) * ab[None]).sum(2) / ll[None], 0, 1)
        q = a[None] + t[..., None] * ab[None]
        return np.linalg.norm(p[:, None, :] - q, axis=2).min(axis=1)
    return f


@dataclass
class TrajectoryValidation:
    valid: bool
    failures: list = field(default_factory=list)
    duration: float = float("nan")
    min_clearance_lower: float = float("nan")
    min_clearance_upper: float = float("nan")
    max_speed_lower: float = float("nan")
    max_speed_upper: float = float("nan")
    max_accel_lower: float = float("nan")
    max_accel_upper: float = float("nan")
    max_position_jump: float = float("nan")
    max_velocity_jump: float = float("nan")
    max_accel_jump: float = float("nan")
    max_deviation_upper: float | None = None

    def to_dict(self) -> dict:
        return asdict(self)


def validate_trajectory(knots, coeffs, distance_fn, required_clearance: float,
                        max_speed: float, max_acceleration: float,
                        start=None, goal=None, start_velocity=None, end_at_rest: bool = True,
                        require_accel_continuity: bool = True, reference_path=None,
                        corridor_halfwidth: float | None = None, hold_point=None,
                        clearance_tol: float = 1e-3, limit_tol: float = 1e-9,
                        continuity_tol: float = 1e-6) -> TrajectoryValidation:
    """Validate a trajectory given as raw arrays.

    distance_fn(points (N,3)) -> exact ground-truth distance to obstacles and bounds.
    """
    knots = np.asarray(knots, float)
    coeffs = np.asarray(coeffs, float)
    fails: list[str] = []
    rep = TrajectoryValidation(False, fails)
    if not (np.all(np.isfinite(knots)) and np.all(np.isfinite(coeffs))):
        fails.append("non-finite knots or coefficients")
        return rep
    if coeffs.ndim != 3 or coeffs.shape[1:] != (6, 3) or len(knots) != len(coeffs) + 1:
        fails.append("malformed trajectory arrays")
        return rep
    dts = np.diff(knots)
    rep.duration = float(knots[-1] - knots[0])
    if len(coeffs) == 0:                                   # stationary hold
        hp = np.asarray(hold_point, float)
        d = float(distance_fn(hp[None])[0])
        rep.min_clearance_lower = rep.min_clearance_upper = d
        rep.max_speed_lower = rep.max_speed_upper = rep.max_accel_lower = rep.max_accel_upper = 0.0
        if d < required_clearance:
            fails.append(f"hold clearance {d:.4f} < {required_clearance:.4f}")
        if start is not None and not np.allclose(hp, start, atol=1e-9):
            fails.append("hold point != start")
        if goal is not None and not np.allclose(hp, goal, atol=1e-9):
            fails.append("hold point != goal")
        rep.valid = not fails
        return rep
    if np.any(dts <= 0):
        fails.append("non-increasing timestamps")
        return rep

    # continuity at knots
    pj = vj = aj = 0.0
    for i in range(len(coeffs) - 1):
        T = dts[i]
        pj = max(pj, float(np.linalg.norm(_eval(coeffs[i], T, 0) - _eval(coeffs[i + 1], 0.0, 0))))
        vj = max(vj, float(np.linalg.norm(_eval(coeffs[i], T, 1) - _eval(coeffs[i + 1], 0.0, 1))))
        aj = max(aj, float(np.linalg.norm(_eval(coeffs[i], T, 2) - _eval(coeffs[i + 1], 0.0, 2))))
    rep.max_position_jump, rep.max_velocity_jump, rep.max_accel_jump = pj, vj, aj
    if pj > continuity_tol:
        fails.append(f"position discontinuity {pj:.2e} m")
    if vj > continuity_tol:
        fails.append(f"velocity discontinuity {vj:.2e} m/s")
    if require_accel_continuity and aj > continuity_tol:
        fails.append(f"acceleration discontinuity {aj:.2e} m/s^2")

    # boundary conditions
    p0, v0, a0 = (_eval(coeffs[0], 0.0, k) for k in range(3))
    pe, ve, ae = (_eval(coeffs[-1], dts[-1], k) for k in range(3))
    if start is not None and np.linalg.norm(p0 - np.asarray(start)) > continuity_tol:
        fails.append("start position mismatch")
    if goal is not None and np.linalg.norm(pe - np.asarray(goal)) > continuity_tol:
        fails.append("goal position mismatch")
    if start_velocity is not None and np.linalg.norm(v0 - np.asarray(start_velocity)) > continuity_tol:
        fails.append("start velocity mismatch")
    if end_at_rest and (np.linalg.norm(ve) > continuity_tol or np.linalg.norm(ae) > continuity_tol):
        fails.append("does not end at rest")

    # limits and clearance (certified intervals)
    s_lo = s_hi = a_lo = a_hi = 0.0
    c_lo = c_hi = np.inf
    dev_f = _polyline_distance(np.asarray(reference_path, float)) if reference_path is not None else None
    dev_hi = 0.0
    for i in range(len(coeffs)):
        T = dts[i]
        P = _bern(coeffs[i], T)
        Pv = 5 * np.diff(P, axis=0) / T
        Pa = 4 * np.diff(Pv, axis=0) / T
        lo, hi = _max_norm(Pv, 1e-6)
        s_lo, s_hi = max(s_lo, lo), max(s_hi, hi)
        lo, hi = _max_norm(Pa, 1e-6)
        a_lo, a_hi = max(a_lo, lo), max(a_hi, hi)
        lo, hi = _min_over_curve(P, distance_fn, clearance_tol)
        c_lo, c_hi = min(c_lo, lo), min(c_hi, hi)
        if dev_f is not None:
            lo, hi = _min_over_curve(P, lambda q: -dev_f(q), 1e-3)
            dev_hi = max(dev_hi, -lo)
    rep.max_speed_lower, rep.max_speed_upper = s_lo, s_hi
    rep.max_accel_lower, rep.max_accel_upper = a_lo, a_hi
    rep.min_clearance_lower, rep.min_clearance_upper = c_lo, c_hi
    if dev_f is not None:
        rep.max_deviation_upper = dev_hi
        if corridor_halfwidth is not None and dev_hi > corridor_halfwidth:
            fails.append(f"leaves corridor: deviation <= {dev_hi:.3f} > {corridor_halfwidth:.3f}")
    if s_hi > max_speed + limit_tol:
        fails.append(f"speed bound {s_hi:.4f} > {max_speed}")
    if a_hi > max_acceleration + limit_tol:
        fails.append(f"acceleration bound {a_hi:.4f} > {max_acceleration}")
    if c_lo < required_clearance:
        fails.append(f"clearance lower bound {c_lo:.4f} < {required_clearance:.4f} (upper {c_hi:.4f})")
    rep.valid = not fails
    return rep

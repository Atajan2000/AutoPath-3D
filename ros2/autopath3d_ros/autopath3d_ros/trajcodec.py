"""Lossless transport of AutoPath trajectories as knot states (pure Python, no ROS imports).

Every piece of a Phase 2 trajectory (and every braking piece) is a polynomial of degree <= 5.
A quintic is uniquely determined by position, velocity and acceleration at both ends of its
interval, so the knot states (t_k, p_k, v_k, a_k) carried by a standard
trajectory_msgs/MultiDOFJointTrajectory reproduce the certified polynomial exactly (up to
floating-point rounding); no resampling or re-interpolation error is introduced.
"""

from __future__ import annotations

import numpy as np

from autopath3d.trajectories.polynomial import PiecewisePolynomial, quintic_hermite


def encode(traj: PiecewisePolynomial) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """-> times (n+1,), positions, velocities, accelerations (n+1, 3). Stationary: one knot."""
    if traj.n_pieces == 0:
        p = np.asarray(traj.hold_point, float)[None]
        return np.array([traj.t_start]), p, np.zeros((1, 3)), np.zeros((1, 3))
    t = np.asarray(traj.knots, float)
    # evaluate each knot from the piece that starts there (the last from the end of the last piece)
    P, V, A = traj.evaluate(t, 0), traj.evaluate(t, 1), traj.evaluate(t, 2)
    return t, P, V, A


def decode(t, P, V, A) -> PiecewisePolynomial:
    t = np.asarray(t, float)
    P, V, A = (np.asarray(x, float).reshape(-1, 3) for x in (P, V, A))
    if len(t) == 1:
        return PiecewisePolynomial.stationary(P[0], float(t[0]))
    if np.any(np.diff(t) <= 0):
        raise ValueError("knot times must be strictly increasing")
    pieces = [quintic_hermite(P[i], V[i], A[i], P[i + 1], V[i + 1], A[i + 1], float(t[i + 1] - t[i]))
              for i in range(len(t) - 1)]
    return PiecewisePolynomial.from_pieces(pieces, list(np.diff(t)), float(t[0]))


def max_deviation(a: PiecewisePolynomial, b: PiecewisePolynomial, n: int = 2000) -> float:
    ts = np.linspace(max(a.t_start, b.t_start), min(a.t_end, b.t_end), n)
    return float(np.abs(a.evaluate(ts) - b.evaluate(ts)).max())


def shifted(traj: PiecewisePolynomial, dt: float) -> PiecewisePolynomial:
    if traj.n_pieces == 0:
        return PiecewisePolynomial.stationary(traj.hold_point, traj.t_start + dt)
    return PiecewisePolynomial(np.asarray(traj.knots) + dt, np.array(traj.coeffs), traj.hold_point)

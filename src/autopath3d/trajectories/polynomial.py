"""Piecewise-polynomial trajectories and Bernstein (Bezier) utilities.

Each piece i is a polynomial in local time tau in [0, T_i] (power basis, degree <= 5):
    p(tau) = sum_k c_k tau^k,   c_k in R^3.
Units: metres and seconds.

Bernstein form: with u = tau / T in [0, 1] the same polynomial is a Bezier curve whose
control points P_j enclose the curve (convex-hull property). Derivatives are again Bezier
curves (scaled differences of control points), which gives rigorous bounds on speed and
acceleration, and lets curves be subdivided exactly (de Casteljau).
"""

from __future__ import annotations

from dataclasses import dataclass
from math import comb

import numpy as np

DEG = 5


def quintic_hermite(p0, v0, a0, p1, v1, a1, T: float) -> np.ndarray:
    """Coefficients (6, 3) of the quintic matching position/velocity/acceleration at both ends."""
    p0, v0, a0, p1, v1, a1 = (np.asarray(x, float) for x in (p0, v0, a0, p1, v1, a1))
    dp = p1 - p0
    c = np.empty((6, 3))
    c[0], c[1], c[2] = p0, v0, 0.5 * a0
    c[3] = (20 * dp - (8 * v1 + 12 * v0) * T - (3 * a0 - a1) * T**2) / (2 * T**3)
    c[4] = (-30 * dp + (14 * v1 + 16 * v0) * T + (3 * a0 - 2 * a1) * T**2) / (2 * T**4)
    c[5] = (12 * dp - 6 * (v1 + v0) * T - (a0 - a1) * T**2) / (2 * T**5)
    return c


def power_to_bernstein(coeffs: np.ndarray, T: float) -> np.ndarray:
    """Control points (n+1, 3) of the polynomial (power coeffs in tau) over u = tau/T."""
    n = len(coeffs) - 1
    a = coeffs * (T ** np.arange(n + 1))[:, None]
    P = np.zeros_like(a)
    for i in range(n + 1):
        for k in range(i + 1):
            P[i] += comb(i, k) / comb(n, k) * a[k]
    return P


def bezier_derivative(P: np.ndarray, T: float) -> np.ndarray:
    """Control points of d/dtau of a Bezier curve given over u = tau/T."""
    n = P.shape[-2] - 1
    return n * np.diff(P, axis=-2) / T


def split_half(P: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """De Casteljau split at u = 0.5 for a batch (K, n+1, 3): (left, right, midpoint)."""
    n1 = P.shape[-2]
    left = np.empty_like(P)
    right = np.empty_like(P)
    cur = P.copy()
    for r in range(n1):
        left[..., r, :] = cur[..., 0, :]
        right[..., n1 - 1 - r, :] = cur[..., -1, :]
        cur = 0.5 * (cur[..., :-1, :] + cur[..., 1:, :])
    return left, right, left[..., -1, :].copy()


def split_at(P: np.ndarray, u: float) -> tuple[np.ndarray, np.ndarray]:
    """De Casteljau split of one curve (n+1, 3) at parameter u."""
    n1 = len(P)
    left, right = np.empty_like(P), np.empty_like(P)
    cur = P.copy()
    for r in range(n1):
        left[r] = cur[0]
        right[n1 - 1 - r] = cur[-1]
        cur = (1 - u) * cur[:-1] + u * cur[1:]
    return left, right


@dataclass
class PiecewisePolynomial:
    knots: np.ndarray        # (n+1,) strictly increasing times, knots[0] = t_start
    coeffs: np.ndarray       # (n, 6, 3) power-basis coefficients in local time
    hold_point: np.ndarray | None = None   # used when there are no pieces (stationary)

    @classmethod
    def stationary(cls, p, t0: float = 0.0) -> "PiecewisePolynomial":
        return cls(np.array([t0]), np.zeros((0, DEG + 1, 3)), np.asarray(p, float).copy())

    @classmethod
    def from_pieces(cls, pieces: list[np.ndarray], durations: list[float], t0: float = 0.0):
        knots = t0 + np.concatenate([[0.0], np.cumsum(durations)])
        co = np.zeros((len(pieces), DEG + 1, 3))
        for i, c in enumerate(pieces):
            co[i, : len(c)] = c
        return cls(knots, co)

    @property
    def n_pieces(self) -> int:
        return len(self.coeffs)

    @property
    def t_start(self) -> float:
        return float(self.knots[0])

    @property
    def t_end(self) -> float:
        return float(self.knots[-1])

    @property
    def duration(self) -> float:
        return self.t_end - self.t_start

    def durations(self) -> np.ndarray:
        return np.diff(self.knots)

    def evaluate(self, t, order: int = 0) -> np.ndarray:
        """Position (order 0), velocity (1), acceleration (2) or jerk (3) at times t."""
        t = np.atleast_1d(np.asarray(t, float))
        if self.n_pieces == 0:
            base = self.hold_point if order == 0 else np.zeros(3)
            return np.tile(base, (len(t), 1))
        tt = np.clip(t, self.t_start, self.t_end)
        idx = np.clip(np.searchsorted(self.knots, tt, side="right") - 1, 0, self.n_pieces - 1)
        tau = tt - self.knots[idx]
        c = self.coeffs[idx]                                        # (N, 6, 3)
        out = np.zeros((len(t), 3))
        for k in range(order, DEG + 1):
            f = 1.0
            for j in range(order):
                f *= k - j
            out += f * c[:, k, :] * (tau ** (k - order))[:, None]
        if order > 0:
            out[(t < self.t_start) | (t > self.t_end)] = 0.0
        return out

    def bernstein(self, i: int) -> np.ndarray:
        return power_to_bernstein(self.coeffs[i], float(self.knots[i + 1] - self.knots[i]))

    def bernstein_window(self, ta: float, tb: float) -> list[np.ndarray]:
        """Bezier control points of the trajectory restricted to [ta, tb] (piece-wise)."""
        out = []
        for i in range(self.n_pieces):
            k0, k1 = self.knots[i], self.knots[i + 1]
            lo, hi = max(ta, k0), min(tb, k1)
            if hi - lo <= 1e-12:
                continue
            P = self.bernstein(i)
            T = k1 - k0
            u0, u1 = (lo - k0) / T, (hi - k0) / T
            if u0 > 0:
                _, P = split_at(P, u0)
                u1 = (u1 - u0) / (1 - u0)
            if u1 < 1:
                P, _ = split_at(P, u1)
            out.append(P)
        return out

    def to_dict(self) -> dict:
        return {"knots": self.knots.tolist(), "coeffs": self.coeffs.tolist(),
                "hold_point": None if self.hold_point is None else self.hold_point.tolist()}

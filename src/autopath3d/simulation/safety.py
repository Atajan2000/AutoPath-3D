"""Independent clearance monitor for the executed (simulated) motion.

Geometry: exact ground-truth distance to the analytic primitives and the scene bounds,
re-implemented here in vectorised form (it does not use the voxel map, the planners'
clearance field or the collision checker). `WorldDistance(scene)(P)` equals
`scene.distance(P)` (tested). It is 1-Lipschitz.

Swept-step bound: under the zero-order-hold model each control step is a quadratic Bezier
curve (control points P0, P1, P2). The curve lies in the convex hull of its control points,
hence in the ball B(m, R) with m = curve midpoint and R = max_j |P_j - m|, so for the
1-Lipschitz distance d:
    lower_k = d(m) - R  <=  min over the step of d  <=  upper_k = min(d(P0), d(m), d(P2)).
`refine_min` tightens the interval around the global minimum by de Casteljau subdivision.

Scope of the guarantee: the bounds are rigorous (up to floating-point rounding) for the
modelled piecewise-quadratic motion of the robot centre. They are numerical checks of a
simulation, not a formal guarantee for a physical vehicle.
"""

from __future__ import annotations

import numpy as np

from ..environments.primitives import Box, VerticalCylinder


class WorldDistance:
    def __init__(self, lo, hi, obstacles) -> None:
        self.lo, self.hi = np.asarray(lo, float), np.asarray(hi, float)
        self.obstacles = list(obstacles)
        boxes = [o for o in self.obstacles if isinstance(o, Box)]
        cyls = [o for o in self.obstacles if isinstance(o, VerticalCylinder)]
        if len(boxes) + len(cyls) != len(self.obstacles):
            raise TypeError("unsupported primitive")
        self.box_names = [b.name for b in boxes]
        self.cyl_names = [c.name for c in cyls]
        self.blo = np.array([b.lo for b in boxes], float).reshape(-1, 3)
        self.bhi = np.array([b.hi for b in boxes], float).reshape(-1, 3)
        self.cc = np.array([c.center_xy for c in cyls], float).reshape(-1, 2)
        self.cr = np.array([c.radius for c in cyls], float)
        self.czl = np.array([c.z_lo for c in cyls], float)
        self.czh = np.array([c.z_hi for c in cyls], float)

    @classmethod
    def of_scene(cls, scene) -> "WorldDistance":
        return cls(scene.lo, scene.hi, scene.obstacles)

    def _box_d(self, P):
        q = np.maximum(np.maximum(self.blo[None] - P[:, None], 0.0), P[:, None] - self.bhi[None])
        return np.linalg.norm(q, axis=2)                                   # (N, B)

    def _cyl_d(self, P):
        dxy = np.maximum(np.linalg.norm(P[:, None, :2] - self.cc[None], axis=2) - self.cr[None], 0.0)
        dz = np.maximum(np.maximum(self.czl[None] - P[:, None, 2], 0.0), P[:, None, 2] - self.czh[None])
        return np.hypot(dxy, dz)                                            # (N, C)

    def __call__(self, points) -> np.ndarray:
        P = np.atleast_2d(np.asarray(points, float))
        d = np.minimum((P - self.lo).min(axis=1), (self.hi - P).min(axis=1))
        if len(self.blo):
            d = np.minimum(d, self._box_d(P).min(axis=1))
        if len(self.cc):
            d = np.minimum(d, self._cyl_d(P).min(axis=1))
        return d

    def nearest(self, points) -> tuple[np.ndarray, np.ndarray, list[str]]:
        """Closest point on the nearest obstacle / bound wall, its distance and its name."""
        P = np.atleast_2d(np.asarray(points, float))
        N = len(P)
        # bounds: nearest wall
        walls = np.concatenate([P - self.lo, self.hi - P], axis=1)          # (N, 6)
        w = np.argmin(walls, axis=1)
        best_d = walls[np.arange(N), w]
        best_q = P.copy()
        ax = w % 3
        best_q[np.arange(N), ax] = np.where(w < 3, self.lo[ax], self.hi[ax])
        names = np.array(["bounds"] * N, dtype=object)
        if len(self.blo):
            q = np.clip(P[:, None], self.blo[None], self.bhi[None])        # (N, B, 3)
            d = np.linalg.norm(P[:, None] - q, axis=2)
            j = np.argmin(d, axis=1)
            dj = d[np.arange(N), j]
            upd = dj < best_d
            best_d = np.where(upd, dj, best_d)
            best_q[upd] = q[np.arange(N), j][upd]
            names[upd] = np.array(self.box_names, dtype=object)[j[upd]]
        if len(self.cc):
            v = P[:, None, :2] - self.cc[None]
            r = np.linalg.norm(v, axis=2)
            s = np.minimum(1.0, self.cr[None] / np.maximum(r, 1e-12))
            qxy = self.cc[None] + v * s[..., None]
            qz = np.clip(P[:, None, 2], self.czl[None], self.czh[None])
            q = np.concatenate([qxy, qz[..., None]], axis=2)
            d = np.linalg.norm(P[:, None] - q, axis=2)
            j = np.argmin(d, axis=1)
            dj = d[np.arange(N), j]
            upd = dj < best_d
            best_d = np.where(upd, dj, best_d)
            best_q[upd] = q[np.arange(N), j][upd]
            names[upd] = np.array(self.cyl_names, dtype=object)[j[upd]]
        return best_q, best_d, list(names)


def _bezier2_mid(P: np.ndarray) -> np.ndarray:
    return 0.25 * P[..., 0, :] + 0.5 * P[..., 1, :] + 0.25 * P[..., 2, :]


def step_bounds(P: np.ndarray, dist: WorldDistance) -> tuple[np.ndarray, np.ndarray]:
    """Certified per-step [lower, upper] on min clearance for quadratic Bezier steps (K, 3, 3)."""
    P = np.asarray(P, float).reshape(-1, 3, 3)
    m = _bezier2_mid(P)
    R = np.linalg.norm(P - m[:, None, :], axis=2).max(axis=1)
    K = len(P)
    d = dist(np.concatenate([P[:, 0], m, P[:, 2]]))
    d0, dm, d2 = d[:K], d[K:2 * K], d[2 * K:]
    return dm - R, np.minimum(np.minimum(d0, dm), d2)


def _split2(P: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    a, b, c = P[:, 0], P[:, 1], P[:, 2]
    ab, bc = 0.5 * (a + b), 0.5 * (b + c)
    m = 0.5 * (ab + bc)
    return np.stack([a, ab, m], axis=1), np.stack([m, bc, c], axis=1)


def refine_min(P: np.ndarray, dist: WorldDistance, tol: float = 1e-4, max_depth: int = 30) -> tuple[float, float]:
    """Certified [lower, upper] on min_t d over the union of quadratic steps P (K, 3, 3)."""
    nodes = np.asarray(P, float).reshape(-1, 3, 3)
    if len(nodes) == 0:
        return float("inf"), float("inf")
    lb, ub = step_bounds(nodes, dist)
    upper = float(ub.min())
    lower_done = np.inf
    for _ in range(max_depth):
        keep = lb < upper - tol
        if (~keep).any():
            lower_done = min(lower_done, float(lb[~keep].min()))
        if not keep.any():
            return min(lower_done, upper), upper
        l, r = _split2(nodes[keep])
        nodes = np.concatenate([l, r])
        lb, ub = step_bounds(nodes, dist)
        upper = min(upper, float(ub.min()))
    return min(lower_done, float(lb.min())), upper

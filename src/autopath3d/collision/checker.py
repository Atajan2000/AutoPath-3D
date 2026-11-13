"""Shared collision checker for a spherical robot envelope on an occupancy grid.

All clearance values returned here are rigorous LOWER BOUNDS on the true distance from
the robot centre to (blocked voxel cubes  U  outside of the map bounds). A configuration
is valid iff its clearance is >= robot.clearance (radius + safety margin).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np
from scipy.spatial import cKDTree

from ..environments.primitives import bounds_distance
from ..maps.distance_field import centre_bounds_distance, cube_distance_field, surface_mask
from ..maps.occupancy import OCCUPIED, UNKNOWN, OccupancyGrid

_CORNERS = np.array([[i, j, k] for i in (0, 1) for j in (0, 1) for k in (0, 1)])


@dataclass(frozen=True)
class RobotModel:
    """Conservative bounding sphere of the quadrotor (frame + propellers)."""

    radius: float = 0.25
    safety_margin: float = 0.10

    @property
    def clearance(self) -> float:
        return self.radius + self.safety_margin


@dataclass
class CheckStats:
    point_queries: int = 0       # number of individual points evaluated
    segment_checks: int = 0
    refined_points: int = 0      # points that needed exact KD-tree refinement

    def reset(self) -> None:
        self.point_queries = self.segment_checks = self.refined_points = 0


class CollisionChecker:
    def __init__(
        self,
        grid: OccupancyGrid,
        robot: RobotModel,
        treat_unknown_as_occupied: bool = True,
        clearance_cap: float = 3.0,
        segment_step: float | None = None,
        min_interval: float = 1e-3,
        refine_k: int = 64,
    ) -> None:
        self.grid = grid
        self.robot = robot
        self.c = robot.clearance
        self.res = grid.resolution
        self.treat_unknown_as_occupied = treat_unknown_as_occupied
        self.step = segment_step if segment_step is not None else 0.5 * self.res
        self.min_interval = min_interval
        self.refine_k = refine_k
        self.stats = CheckStats()

        self.blocked = grid.blocked_mask(treat_unknown_as_occupied)
        # F_blk: exact distance from voxel centres to blocked cubes (capped lower bound)
        self.f_blocked, self.cap_eff = cube_distance_field(self.blocked, self.res, clearance_cap)
        # F_total: additionally bounded by the distance to the map boundary
        self.f_total = np.minimum(self.f_blocked, centre_bounds_distance(grid.shape, self.res))

        surf = np.argwhere(surface_mask(self.blocked))
        self._surf_centres = grid.grid_to_world(surf) if len(surf) else np.zeros((0, 3))
        self._tree = cKDTree(self._surf_centres) if len(surf) else None
        self._half_diag = 0.5 * math.sqrt(3.0) * self.res
        self._refine_margin = self.res
        self._refine_radius = self.c + self._refine_margin + self._half_diag
        self._shape = np.asarray(grid.shape)

    # ------------------------------------------------------------------ points
    def clearance(self, points: np.ndarray) -> np.ndarray:
        """Lower bound on distance to blocked space / bounds for each point (N,)."""
        p = np.atleast_2d(np.asarray(points, float))
        n = len(p)
        self.stats.point_queries += n
        g = self.grid
        q = (p - g.origin) / self.res - 0.5
        i0 = np.clip(np.floor(q).astype(np.int64), 0, self._shape - 2)
        idx = i0[:, None, :] + _CORNERS[None, :, :]                       # (N, 8, 3)
        centres = g.origin + (idx + 0.5) * self.res
        fv = self.f_blocked[idx[..., 0], idx[..., 1], idx[..., 2]]
        lb = (fv - np.linalg.norm(p[:, None, :] - centres, axis=2)).max(axis=1)

        need = lb < self.c + self._refine_margin
        if need.any() and self._tree is not None:
            lb[need] = np.maximum(lb[need], self._exact_near(p[need]))
            self.stats.refined_points += int(need.sum())

        # points inside a blocked voxel have zero clearance
        st = g.state_at(p)
        inside_blocked = (st == OCCUPIED) | ((st == UNKNOWN) & self.treat_unknown_as_occupied)
        lb[inside_blocked] = 0.0
        return np.minimum(lb, bounds_distance(p, g.lo, g.hi))

    def _exact_near(self, p: np.ndarray) -> np.ndarray:
        """Exact distance to the nearest surface cube, or a lower bound when far."""
        k = min(self.refine_k, len(self._surf_centres))
        d, ii = self._tree.query(p, k=k, distance_upper_bound=self._refine_radius)
        d = d.reshape(len(p), k)
        ii = ii.reshape(len(p), k)
        valid = np.isfinite(d)
        safe_ii = np.where(valid, ii, 0)
        cc = self._surf_centres[safe_ii]                                   # (M, k, 3)
        e = np.linalg.norm(np.maximum(np.abs(p[:, None, :] - cc) - 0.5 * self.res, 0.0), axis=2)
        e = np.where(valid, e, np.inf).min(axis=1)
        # cubes not returned: either beyond the radius, or beyond the k-th centre
        rest = np.where(valid[:, -1], d[:, -1], self._refine_radius) - self._half_diag
        return np.minimum(e, rest)

    def point_free(self, points: np.ndarray) -> np.ndarray:
        return self.clearance(points) >= self.c

    # ------------------------------------------------------------------ segments
    def segment_free(self, a: np.ndarray, b: np.ndarray) -> bool:
        """Certify that the whole segment a-b keeps clearance >= c.

        Uses the 1-Lipschitz property of the distance function: on an interval of
        length l with endpoint bounds d0, d1 the minimum is >= (d0 + d1 - l) / 2.
        Uncertified intervals are bisected; anything not certified at min_interval
        is rejected (conservative).
        """
        self.stats.segment_checks += 1
        a = np.asarray(a, float)
        b = np.asarray(b, float)
        v = b - a
        length = float(np.linalg.norm(v))
        c = self.c
        if length < 1e-12:
            return bool(self.clearance(a)[0] >= c)
        n = max(1, math.ceil(length / self.step))
        t = np.linspace(0.0, 1.0, n + 1)
        d = self.clearance(a + t[:, None] * v)
        if np.any(d < c):
            return False
        t0, t1, d0, d1 = t[:-1], t[1:], d[:-1], d[1:]
        while True:
            seg_len = (t1 - t0) * length
            bad = (d0 + d1 - seg_len) * 0.5 < c
            if not bad.any():
                return True
            t0, t1, d0, d1, seg_len = t0[bad], t1[bad], d0[bad], d1[bad], seg_len[bad]
            if np.any(seg_len < self.min_interval):
                return False
            tm = 0.5 * (t0 + t1)
            dm = self.clearance(a + tm[:, None] * v)
            if np.any(dm < c):
                return False
            t0, t1 = np.concatenate([t0, tm]), np.concatenate([tm, t1])
            d0, d1 = np.concatenate([d0, dm]), np.concatenate([dm, d1])

    def path_free(self, path: np.ndarray) -> bool:
        path = np.atleast_2d(path)
        if len(path) == 1:
            return bool(self.point_free(path)[0])
        return all(self.segment_free(path[i], path[i + 1]) for i in range(len(path) - 1))

    def sampled_min_clearance(self, path: np.ndarray, step: float = 0.02) -> float:
        """Minimum lower-bound clearance over densely sampled path points (diagnostic)."""
        pts = densify(path, step)
        return float(self.clearance(pts).min())


def densify(path: np.ndarray, step: float) -> np.ndarray:
    path = np.atleast_2d(np.asarray(path, float))
    if len(path) == 1:
        return path.copy()
    out = [path[:1]]
    for a, b in zip(path[:-1], path[1:]):
        n = max(1, math.ceil(np.linalg.norm(b - a) / step))
        t = np.linspace(0, 1, n + 1)[1:]
        out.append(a + t[:, None] * (b - a))
    return np.vstack(out)

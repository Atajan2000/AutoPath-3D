"""Independent path validation.

Deliberately shares no code with the planners' CollisionChecker or clearance field.
For a segment p(t) = a + t (b - a) and a convex solid S, t -> dist(p(t), S) is convex, so
its minimum over [0, 1] is found to ~1e-12 by golden-section search. Boxes, vertical
cylinders and voxel cubes are all convex; the distance to the bounding walls is affine
per face (minimum at an endpoint). The minimum clearance of a polyline is the minimum
over its segments and over every solid — this is exact, not sampled.
"""

from __future__ import annotations

from dataclasses import dataclass, asdict

import numpy as np

from ..environments.primitives import Box, VerticalCylinder

_INVPHI = (np.sqrt(5.0) - 1.0) / 2.0
_TOL = 1e-9


def _box_dist(p: np.ndarray, lo: np.ndarray, hi: np.ndarray) -> np.ndarray:
    """p (..., K, 3) vs boxes lo/hi (K, 3) -> (..., K)."""
    d = np.maximum(np.maximum(lo - p, 0.0), p - hi)
    return np.sqrt((d * d).sum(axis=-1))


def _cyl_dist(p, cxy, rad, zlo, zhi):
    dxy = np.maximum(np.sqrt(((p[..., :2] - cxy) ** 2).sum(axis=-1)) - rad, 0.0)
    dz = np.maximum(np.maximum(zlo - p[..., 2], 0.0), p[..., 2] - zhi)
    return np.sqrt(dxy * dxy + dz * dz)


def _segment_min(fn, a: np.ndarray, b: np.ndarray, k: int, iters: int = 64) -> np.ndarray:
    """Per-solid minimum of a convex distance over the segment a-b. fn: (K,3)->(K,)."""
    v = b - a
    lo = np.zeros(k)
    hi = np.ones(k)
    x1 = hi - _INVPHI * (hi - lo)
    x2 = lo + _INVPHI * (hi - lo)
    f1 = fn(a + x1[:, None] * v)
    f2 = fn(a + x2[:, None] * v)
    for _ in range(iters):
        left = f1 <= f2
        hi = np.where(left, x2, hi)
        lo = np.where(left, lo, x1)
        nx1 = hi - _INVPHI * (hi - lo)
        nx2 = lo + _INVPHI * (hi - lo)
        x1, x2 = nx1, nx2
        f1 = fn(a + x1[:, None] * v)
        f2 = fn(a + x2[:, None] * v)
    ends = np.minimum(fn(np.broadcast_to(a, (k, 3))), fn(np.broadcast_to(b, (k, 3))))
    return np.minimum(np.minimum(f1, f2), ends)


def _segments(path: np.ndarray):
    path = np.atleast_2d(np.asarray(path, float))
    if len(path) == 1:
        return [(path[0], path[0])]
    return list(zip(path[:-1], path[1:]))


def ground_truth_clearance(scene, path: np.ndarray) -> dict:
    """Exact minimum distance from the polyline to the scene's true geometry."""
    boxes = [o for o in scene.obstacles if isinstance(o, Box)]
    cyls = [o for o in scene.obstacles if isinstance(o, VerticalCylinder)]
    blo = np.array([b.lo for b in boxes]).reshape(-1, 3)
    bhi = np.array([b.hi for b in boxes]).reshape(-1, 3)
    cxy = np.array([c.center_xy for c in cyls]).reshape(-1, 2)
    crad = np.array([c.radius for c in cyls])
    czlo = np.array([c.z_lo for c in cyls])
    czhi = np.array([c.z_hi for c in cyls])
    names = [o.name for o in boxes] + [o.name for o in cyls]
    best_obs, best_name, best_seg = np.inf, None, -1
    best_bnd = np.inf
    for s, (a, b) in enumerate(_segments(path)):
        vals = []
        if len(boxes):
            vals.append(_segment_min(lambda p: _box_dist(p, blo, bhi), a, b, len(boxes)))
        if len(cyls):
            vals.append(_segment_min(lambda p: _cyl_dist(p, cxy, crad, czlo, czhi), a, b, len(cyls)))
        if vals:
            allv = np.concatenate(vals)
            k = int(np.argmin(allv))
            if allv[k] < best_obs:
                best_obs, best_name, best_seg = float(allv[k]), names[k], s
        for p in (a, b):
            bd = float(min((p - scene.lo).min(), (scene.hi - p).min()))
            best_bnd = min(best_bnd, bd)
    return {"min_clearance": min(best_obs, best_bnd), "min_obstacle_clearance": best_obs,
            "min_bounds_clearance": best_bnd, "closest_obstacle": best_name, "closest_segment": best_seg}


def voxel_map_clearance(grid, path: np.ndarray, search_radius: float,
                        treat_unknown_as_occupied: bool = True) -> dict:
    """Exact minimum distance from the polyline to the blocked voxel cubes and bounds.

    Only cubes within `search_radius` of a segment's AABB are examined, so values above
    `search_radius` are reported as `search_radius` (a lower bound).
    """
    states = grid.states
    blocked = states != 0 if treat_unknown_as_occupied else states == 1
    r, o = grid.resolution, grid.origin
    shape = np.asarray(states.shape)
    best = float(search_radius)
    best_bnd = np.inf
    for a, b in _segments(path):
        lo = np.minimum(a, b) - search_radius
        hi = np.maximum(a, b) + search_radius
        i0 = np.clip(np.floor((lo - o) / r).astype(int), 0, shape)
        i1 = np.clip(np.ceil((hi - o) / r).astype(int), 0, shape)
        sub = blocked[i0[0]:i1[0], i0[1]:i1[1], i0[2]:i1[2]]
        idx = np.argwhere(sub) + i0
        if len(idx):
            clo = o + idx * r
            chi = clo + r
            vals = _segment_min(lambda p: _box_dist(p, clo, chi), a, b, len(idx))
            best = min(best, float(vals.min()))
        for p in (a, b):
            best_bnd = min(best_bnd, float(min((p - grid.lo).min(), (grid.hi - p).min())))
    return {"min_clearance": min(best, best_bnd), "min_voxel_clearance": best,
            "min_bounds_clearance": best_bnd}


@dataclass
class ValidationReport:
    valid: bool
    required_clearance: float
    gt_min_clearance: float
    voxel_min_clearance: float
    closest_obstacle: str | None
    endpoints_match: bool

    def to_dict(self) -> dict:
        return asdict(self)


def validate_path(scene, grid, path, start, goal, required_clearance: float,
                  treat_unknown_as_occupied: bool = True) -> ValidationReport:
    path = np.atleast_2d(np.asarray(path, float))
    ends = bool(np.allclose(path[0], start, atol=1e-9) and np.allclose(path[-1], goal, atol=1e-9))
    gt = ground_truth_clearance(scene, path)
    vx = voxel_map_clearance(grid, path, required_clearance + grid.resolution, treat_unknown_as_occupied)
    ok = (ends and gt["min_clearance"] >= required_clearance - _TOL
          and vx["min_clearance"] >= required_clearance - _TOL)
    return ValidationReport(bool(ok), required_clearance, gt["min_clearance"], vx["min_clearance"],
                            gt["closest_obstacle"], ends)

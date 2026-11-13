"""Uniform 3D occupancy grid with FREE / OCCUPIED / UNKNOWN voxel states.

Voxel (i, j, k) is the closed cube [o + i r, o + (i+1) r] per axis, centre o + (i + 1/2) r.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..environments.primitives import Box, Primitive, VerticalCylinder

FREE: int = 0
OCCUPIED: int = 1
UNKNOWN: int = 2

_SNAP = 1e-9


def _snap(q: np.ndarray) -> np.ndarray:
    """Snap values within 1e-9 of an integer (removes float noise in index math)."""
    r = np.round(q)
    return np.where(np.abs(q - r) < _SNAP, r, q)


@dataclass
class OccupancyGrid:
    origin: np.ndarray          # (3,) world coords of the grid corner, metres
    resolution: float           # voxel edge length, metres
    states: np.ndarray          # (nx, ny, nz) int8
    known_environment: bool = True   # True: synthetic, fully known; False: sensor-derived

    @classmethod
    def empty(cls, lo, hi, resolution: float, fill: int = FREE) -> "OccupancyGrid":
        lo, hi = np.asarray(lo, float), np.asarray(hi, float)
        n = (hi - lo) / resolution
        n_int = np.round(n).astype(int)
        if np.any(np.abs(n - n_int) > 1e-6) or np.any(n_int < 2):
            raise ValueError(f"bounds extent {hi - lo} must be >= 2 integer multiples of {resolution}")
        return cls(lo.copy(), float(resolution), np.full(tuple(n_int), fill, dtype=np.int8))

    @classmethod
    def from_scene(cls, scene, resolution: float) -> "OccupancyGrid":
        grid = cls.empty(scene.lo, scene.hi, resolution, FREE)
        for box in scene.unknown_regions:
            grid.mark_primitive(box, UNKNOWN)
        for ob in scene.obstacles:
            grid.mark_primitive(ob, OCCUPIED)
        return grid

    # ------------------------------------------------------------------ geometry
    @property
    def shape(self) -> tuple[int, int, int]:
        return tuple(self.states.shape)

    @property
    def lo(self) -> np.ndarray:
        return self.origin

    @property
    def hi(self) -> np.ndarray:
        return self.origin + np.asarray(self.shape) * self.resolution

    def world_to_grid(self, points: np.ndarray) -> np.ndarray:
        """Integer voxel indices containing the points (may be out of range)."""
        q = (np.atleast_2d(np.asarray(points, float)) - self.origin) / self.resolution
        return np.floor(_snap(q)).astype(np.int64)

    def grid_to_world(self, idx: np.ndarray) -> np.ndarray:
        """World coordinates of voxel centres."""
        return self.origin + (np.atleast_2d(idx).astype(float) + 0.5) * self.resolution

    def index_in_bounds(self, idx: np.ndarray) -> np.ndarray:
        idx = np.atleast_2d(idx)
        return np.all((idx >= 0) & (idx < np.asarray(self.shape)), axis=1)

    def point_in_bounds(self, points: np.ndarray) -> np.ndarray:
        p = np.atleast_2d(points)
        return np.all((p >= self.lo) & (p <= self.hi), axis=1)

    # ------------------------------------------------------------------ queries
    def state_at(self, points: np.ndarray) -> np.ndarray:
        """Voxel state at each point; points outside the grid return UNKNOWN.

        Points on a shared face are assigned to the voxel given by floor(); points on
        the upper grid face are clamped into the last voxel.
        """
        p = np.atleast_2d(np.asarray(points, float))
        idx = self.world_to_grid(p)
        inside = self.point_in_bounds(p)
        idx = np.clip(idx, 0, np.asarray(self.shape) - 1)
        out = self.states[idx[:, 0], idx[:, 1], idx[:, 2]].copy()
        out[~inside] = UNKNOWN
        return out

    def blocked_mask(self, treat_unknown_as_occupied: bool = True) -> np.ndarray:
        if treat_unknown_as_occupied:
            return self.states != FREE
        return self.states == OCCUPIED

    # ------------------------------------------------------------------ insertion
    def _index_range(self, lo: np.ndarray, hi: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Voxel index range whose cube interiors overlap the open box (lo, hi)."""
        qlo = _snap((lo - self.origin) / self.resolution)
        qhi = _snap((hi - self.origin) / self.resolution)
        i0 = np.maximum(np.floor(qlo).astype(int), 0)
        i1 = np.minimum(np.ceil(qhi).astype(int), np.asarray(self.shape))   # exclusive
        return i0, i1

    def mark_primitive(self, prim: Primitive, state: int) -> int:
        """Mark voxels whose interior intersects the primitive's interior.

        This is conservative: every point of a positive-volume obstacle lies in some
        marked closed cube. UNKNOWN never overwrites OCCUPIED. Returns voxels marked.
        """
        lo, hi = prim.aabb()
        i0, i1 = self._index_range(lo, hi)
        if np.any(i1 <= i0):
            return 0
        sl = tuple(slice(a, b) for a, b in zip(i0, i1))
        if isinstance(prim, Box):
            mask = np.ones(tuple(i1 - i0), dtype=bool)
        elif isinstance(prim, VerticalCylinder):
            r = self.resolution
            xs = self.origin[0] + np.arange(i0[0], i1[0]) * r
            ys = self.origin[1] + np.arange(i0[1], i1[1]) * r
            cx, cy = prim.center_xy
            # distance from the axis to each closed voxel rectangle in xy
            dx = np.maximum(np.maximum(xs - cx, cx - (xs + r)), 0.0)
            dy = np.maximum(np.maximum(ys - cy, cy - (ys + r)), 0.0)
            m2 = (dx[:, None] ** 2 + dy[None, :] ** 2) < prim.radius ** 2
            mask = np.broadcast_to(m2[:, :, None], tuple(i1 - i0)).copy()
        else:
            raise TypeError(type(prim))
        view = self.states[sl]
        if state == UNKNOWN:
            mask &= view != OCCUPIED
        view[mask] = state
        return int(mask.sum())

    def mark_box(self, lo, hi, state: int) -> int:
        return self.mark_primitive(Box(tuple(lo), tuple(hi)), state)

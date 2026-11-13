"""Analytic convex obstacle primitives (ground-truth geometry).

All primitives are closed convex solids with positive volume. Distances are Euclidean
distances from a point to the solid (0 inside), in metres.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class Box:
    """Axis-aligned box [lo, hi]."""

    lo: tuple[float, float, float]
    hi: tuple[float, float, float]
    name: str = "box"

    def __post_init__(self) -> None:
        if not all(h > l for l, h in zip(self.lo, self.hi)):
            raise ValueError(f"Box {self.name} must have positive extent: {self.lo} {self.hi}")

    def distance(self, points: np.ndarray) -> np.ndarray:
        p = np.atleast_2d(points)
        lo, hi = np.asarray(self.lo), np.asarray(self.hi)
        d = np.maximum(np.maximum(lo - p, 0.0), p - hi)
        return np.linalg.norm(d, axis=1)

    def aabb(self) -> tuple[np.ndarray, np.ndarray]:
        return np.asarray(self.lo, float), np.asarray(self.hi, float)


@dataclass(frozen=True)
class VerticalCylinder:
    """Cylinder with vertical axis through (cx, cy), radius r, z in [z_lo, z_hi]."""

    center_xy: tuple[float, float]
    radius: float
    z_lo: float
    z_hi: float
    name: str = "cylinder"

    def __post_init__(self) -> None:
        if self.radius <= 0 or self.z_hi <= self.z_lo:
            raise ValueError(f"Cylinder {self.name} must have positive extent")

    def distance(self, points: np.ndarray) -> np.ndarray:
        p = np.atleast_2d(points)
        c = np.asarray(self.center_xy)
        dxy = np.maximum(np.linalg.norm(p[:, :2] - c, axis=1) - self.radius, 0.0)
        dz = np.maximum(np.maximum(self.z_lo - p[:, 2], 0.0), p[:, 2] - self.z_hi)
        return np.hypot(dxy, dz)

    def aabb(self) -> tuple[np.ndarray, np.ndarray]:
        cx, cy = self.center_xy
        r = self.radius
        return (np.array([cx - r, cy - r, self.z_lo]), np.array([cx + r, cy + r, self.z_hi]))


Primitive = Box | VerticalCylinder


def bounds_distance(points: np.ndarray, lo: np.ndarray, hi: np.ndarray) -> np.ndarray:
    """Signed distance to the walls of the bounding box (negative outside)."""
    p = np.atleast_2d(points)
    return np.minimum((p - lo).min(axis=1), (hi - p).min(axis=1))

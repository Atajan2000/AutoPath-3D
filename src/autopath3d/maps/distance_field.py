"""Exact clearance field: distance from voxel centres to the union of blocked cubes.

For a voxel centre q and a blocked cube with centre b (integer offset t = q - b per axis)
the exact distance is  r * sqrt(sum_i g(t_i)),  g(t) = max(|t| - 1/2, 0)^2.
Because the squared distance is a sum of per-axis terms, the minimum over all blocked
cubes is computed exactly by three separable 1-D min-convolutions (like an EDT, but with
the cube kernel instead of the centre-to-centre kernel). The window is truncated at
`cap`; values are therefore exact below `cap_effective` and a valid lower bound above.
"""

from __future__ import annotations

import math

import numpy as np


def _min_conv_axis(arr: np.ndarray, axis: int, w: int) -> np.ndarray:
    out = arr.copy()
    n = arr.shape[axis]
    for t in range(1, min(w, n - 1) + 1):
        gt = (t - 0.5) ** 2
        src_lo = [slice(None)] * 3
        dst_hi = [slice(None)] * 3
        src_lo[axis] = slice(0, n - t)
        dst_hi[axis] = slice(t, n)
        # voxel at index i+t sees blocked voxel at i (offset +t) and vice versa
        np.minimum(out[tuple(dst_hi)], arr[tuple(src_lo)] + gt, out=out[tuple(dst_hi)])
        np.minimum(out[tuple(src_lo)], arr[tuple(dst_hi)] + gt, out=out[tuple(src_lo)])
    return out


def cube_distance_field(blocked: np.ndarray, resolution: float, cap: float) -> tuple[np.ndarray, float]:
    """Return (F, cap_effective): F[v] = min(dist(centre_v, blocked cubes), cap_effective)."""
    w = int(math.ceil(cap / resolution)) + 1
    cap_eff = (w + 0.5) * resolution   # every cube outside the window is at least this far
    d2 = np.where(blocked, 0.0, np.inf)
    for axis in range(3):
        d2 = _min_conv_axis(d2, axis, w)
    f = np.sqrt(d2) * resolution
    np.minimum(f, cap_eff, out=f)
    return f, cap_eff


def centre_bounds_distance(shape: tuple[int, int, int], resolution: float) -> np.ndarray:
    """Distance from each voxel centre to the grid boundary faces."""
    out = np.full(shape, np.inf)
    for axis, n in enumerate(shape):
        i = np.arange(n)
        d = np.minimum(i + 0.5, n - i - 0.5) * resolution
        sh = [1, 1, 1]
        sh[axis] = n
        out = np.minimum(out, d.reshape(sh))
    return out


def surface_mask(blocked: np.ndarray) -> np.ndarray:
    """Blocked voxels with at least one 6-neighbour that is free (inside the grid).

    The closest point of the blocked union to any free point lies on a face of such a
    voxel, so exact nearest-cube queries only need these voxels.
    """
    free = ~blocked
    adj = np.zeros_like(blocked)
    for axis in range(3):
        n = blocked.shape[axis]
        a = [slice(None)] * 3
        b = [slice(None)] * 3
        a[axis], b[axis] = slice(0, n - 1), slice(1, n)
        adj[tuple(a)] |= free[tuple(b)]
        adj[tuple(b)] |= free[tuple(a)]
    return blocked & adj

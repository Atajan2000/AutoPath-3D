"""3D A* over the voxel-centre lattice.

Edge safety (no sampling needed): for a lattice move m with |m_i| <= 1, every point on
the segment between the two centres is at least as far from any blocked cube as one of
the lattice nodes of the unit sub-cube spanned by m (per axis, the integer cube offset t
satisfies |t - s| >= min(|t|, |t - 1|) for s in [0, 1]); the bound distance is affine per
face. Hence, if all nodes of that sub-cube have clearance >= c — exactly the
corner-cutting rule — the whole transition is collision-free for the robot sphere.
"""

from __future__ import annotations

import heapq
import itertools
import math
import time
from array import array

import numpy as np

from ..collision.checker import CollisionChecker
from .common import PlanResult, Status, path_length, validate_endpoints

_SQ2, _SQ3 = math.sqrt(2.0), math.sqrt(3.0)


def lattice_moves(connectivity: int) -> list[tuple[int, int, int]]:
    if connectivity not in (6, 18, 26):
        raise ValueError("connectivity must be 6, 18 or 26")
    moves = []
    for d in itertools.product((-1, 0, 1), repeat=3):
        nnz = sum(x != 0 for x in d)
        if nnz == 0:
            continue
        if (connectivity == 6 and nnz == 1) or (connectivity == 18 and nnz <= 2) or connectivity == 26:
            moves.append(d)
    return moves


def subcube_corners(move: tuple[int, int, int]) -> list[tuple[int, int, int]]:
    """Lattice nodes of the sub-cube spanned by `move`, excluding both endpoints."""
    axes = [i for i in range(3) if move[i] != 0]
    out = []
    for mask in itertools.product((0, 1), repeat=len(axes)):
        if 0 < sum(mask) < len(axes):
            u = [0, 0, 0]
            for a, bit in zip(axes, mask):
                u[a] = move[a] * bit
            out.append(tuple(u))
    return out


class AStarPlanner:
    name = "astar"

    def __init__(self, checker: CollisionChecker, connectivity: int = 26,
                 connect_radius_voxels: float = 3.0, record_expansions: bool = False) -> None:
        t0 = time.perf_counter()
        self.checker = checker
        self.grid = checker.grid
        self.connectivity = connectivity
        self.connect_radius = connect_radius_voxels * _SQ3 * self.grid.resolution
        self.base_ring = self.grid.resolution * {6: 1.0, 18: _SQ2, 26: _SQ3}[connectivity]
        self.record_expansions = record_expansions

        valid = checker.f_total >= checker.c                    # inflated free space
        self.valid_np = np.pad(valid, 1, constant_values=False)
        self.pshape = self.valid_np.shape
        self.s0 = self.pshape[1] * self.pshape[2]
        self.s1 = self.pshape[2]
        self.valid = bytearray(self.valid_np.ravel(order="C").astype(np.uint8).tobytes())
        self.n_valid = int(valid.sum())

        r = self.grid.resolution
        self.moves = []
        for m in lattice_moves(connectivity):
            off = m[0] * self.s0 + m[1] * self.s1 + m[2]
            cost = r * math.sqrt(sum(x * x for x in m))
            corners = tuple(u[0] * self.s0 + u[1] * self.s1 + u[2] for u in subcube_corners(m))
            self.moves.append((off, cost, corners))
        self.setup_time_s = time.perf_counter() - t0

    # ---------------------------------------------------------------- indexing
    def _flat(self, ijk) -> int:
        return (ijk[0] + 1) * self.s0 + (ijk[1] + 1) * self.s1 + (ijk[2] + 1)

    def _ijk(self, n: int) -> tuple[int, int, int]:
        i, rem = divmod(n, self.s0)
        j, k = divmod(rem, self.s1)
        return i - 1, j - 1, k - 1

    def node_centre(self, n: int) -> np.ndarray:
        return self.grid.grid_to_world(np.array(self._ijk(n)))[0]

    def _connectable(self, p: np.ndarray, reverse: bool) -> list[tuple[int, float]]:
        """Valid lattice nodes near p reachable by a certified straight segment.

        Tries the ring of radius r * sqrt(max move order) first (r, sqrt2 r, sqrt3 r for
        6/18/26; for p on a centre these are exactly its lattice neighbours, and p's own
        voxel centre is always inside), widening only if it is empty.
        """
        k = 1
        while k * self.base_ring <= self.connect_radius + 1e-12:
            out = self._connectable_within(p, reverse, k * self.base_ring)
            if out:
                return out
            k += 1
        return []

    def _connectable_within(self, p: np.ndarray, reverse: bool, radius: float) -> list[tuple[int, float]]:
        g = self.grid
        rad_vox = int(math.ceil(radius / g.resolution))
        base = g.world_to_grid(p)[0]
        rng = np.arange(-rad_vox, rad_vox + 1)
        cand = base + np.array(np.meshgrid(rng, rng, rng, indexing="ij")).reshape(3, -1).T
        cand = cand[g.index_in_bounds(cand)]
        if len(cand) == 0:
            return []
        cand = cand[self.checker.f_total[cand[:, 0], cand[:, 1], cand[:, 2]] >= self.checker.c]
        if len(cand) == 0:
            return []
        centres = g.grid_to_world(cand)
        dist = np.linalg.norm(centres - p, axis=1)
        keep = dist <= radius + 1e-12
        out = []
        for ijk, ctr, d in sorted(zip(cand[keep].tolist(), centres[keep], dist[keep]), key=lambda x: (x[2], x[0])):
            ok = self.checker.segment_free(ctr, p) if reverse else self.checker.segment_free(p, ctr)
            if ok:
                out.append((self._flat(ijk), float(d)))
        return out

    # ---------------------------------------------------------------- search
    def plan(self, start, goal) -> PlanResult:
        start = np.asarray(start, float)
        goal = np.asarray(goal, float)
        bad = validate_endpoints(self.checker, start, goal, self.name)
        if bad is not None:
            bad.setup_time_s = self.setup_time_s
            return bad
        stats0 = self.checker.stats.segment_checks
        t0 = time.perf_counter()
        if np.linalg.norm(goal - start) < 1e-9:
            return PlanResult(self.name, Status.SUCCESS, start[None, :].copy(), 0.0,
                              time.perf_counter() - t0, self.setup_time_s,
                              stats={"nodes_expanded": 0, "zero_length": True})

        r = self.grid.resolution
        o = self.grid.origin
        gx, gy, gz = goal
        GOAL = -1
        starts = self._connectable(start, reverse=False)
        goals = dict(self._connectable(goal, reverse=True))
        g_goal_ijk = self.grid.world_to_grid(goal)[0]
        D = 0.0
        if goals:
            D = max(self._lattice_h(self._ijk(n), g_goal_ijk) for n in goals)

        N = len(self.valid)
        g = array("d", [math.inf]) * N
        parent = array("q", [-3]) * N            # -2: predecessor is the start point
        closed = bytearray(N)
        valid = self.valid
        heap: list = []
        cnt = itertools.count()
        best_goal, goal_parent = math.inf, -3

        def h_of(n: int) -> float:
            i, j, k = self._ijk(n)
            x, y, z = o[0] + (i + 0.5) * r, o[1] + (j + 0.5) * r, o[2] + (k + 0.5) * r
            he = math.sqrt((x - gx) ** 2 + (y - gy) ** 2 + (z - gz) ** 2)
            if self.connectivity == 18:
                return he
            return max(he, self._lattice_h((i, j, k), g_goal_ijk) - D)

        for n, d in starts:
            if d < g[n]:
                g[n] = d
                parent[n] = -2
                hn = h_of(n)
                heapq.heappush(heap, (d + hn, hn, next(cnt), n))
        direct = float(np.linalg.norm(goal - start))
        if direct <= self.base_ring and self.checker.segment_free(start, goal):
            best_goal, goal_parent = direct, -2
            heapq.heappush(heap, (direct, 0.0, next(cnt), GOAL))

        expanded = 0
        expansions = [] if self.record_expansions else None
        moves = self.moves
        while heap:
            f, _, _, n = heapq.heappop(heap)
            if n == GOAL:
                break
            if closed[n]:
                continue
            closed[n] = 1
            expanded += 1
            if expansions is not None:
                expansions.append(n)
            gn = g[n]
            tc = goals.get(n)
            if tc is not None and gn + tc < best_goal:
                best_goal, goal_parent = gn + tc, n
                heapq.heappush(heap, (best_goal, 0.0, next(cnt), GOAL))
            for off, cost, corners in moves:
                m = n + off
                if not valid[m] or closed[m]:
                    continue
                cut = False
                for co in corners:
                    if not valid[n + co]:
                        cut = True
                        break
                if cut:
                    continue
                ng = gn + cost
                if ng < g[m]:
                    g[m] = ng
                    parent[m] = n
                    hm = h_of(m)
                    heapq.heappush(heap, (ng + hm, hm, next(cnt), m))
        elapsed = time.perf_counter() - t0
        stats = {"nodes_expanded": expanded, "valid_nodes": self.n_valid,
                 "start_connections": len(starts), "goal_connections": len(goals),
                 "segment_checks": self.checker.stats.segment_checks - stats0,
                 "connectivity": self.connectivity}
        if expansions is not None:
            stats["expansion_order"] = np.array([self._ijk(n) for n in expansions], dtype=np.int32)
        if not math.isfinite(best_goal):
            reason = ("start not connectable to lattice" if not starts else
                      "goal not connectable to lattice" if not goals else "open set exhausted")
            return PlanResult(self.name, Status.NO_PATH, None, float("nan"), elapsed, self.setup_time_s,
                              reason=reason, stats=stats)
        nodes = []
        n = goal_parent
        while n >= 0:
            nodes.append(n)
            n = parent[n]
        nodes.reverse()
        pts = [start] + [self.node_centre(n) for n in nodes] + [goal]
        path = np.array(pts)
        keep = np.ones(len(path), bool)
        keep[1:] = np.linalg.norm(np.diff(path, axis=0), axis=1) > 1e-12
        path = path[keep]
        stats["lattice_cost"] = best_goal
        return PlanResult(self.name, Status.SUCCESS, path, path_length(path), elapsed,
                          self.setup_time_s, stats=stats)

    def _lattice_h(self, a, b) -> float:
        """Exact obstacle-free lattice distance (6: Manhattan, 26: octile)."""
        d = sorted((abs(a[0] - b[0]), abs(a[1] - b[1]), abs(a[2] - b[2])), reverse=True)
        r = self.grid.resolution
        if self.connectivity == 6:
            return r * (d[0] + d[1] + d[2])
        return r * (d[0] + (_SQ2 - 1) * d[1] + (_SQ3 - _SQ2) * d[2])

"""3D RRT* (Karaman & Frazzoli 2011) in continuous space.

Every tree edge is certified collision-free by the shared CollisionChecker when it is
created (parent selection) or changed (rewiring). Costs are Euclidean path lengths from
the root; rewiring recomputes the cost of the whole affected subtree.
"""

from __future__ import annotations

import math
import time
from collections import deque

import numpy as np

from ..collision.checker import CollisionChecker
from .common import PlanResult, Status, path_length, validate_endpoints

_COST_EPS = 1e-9


class RRTStarTree:
    def __init__(self, root: np.ndarray, capacity: int) -> None:
        self.pos = np.empty((capacity, 3))
        self.parent = np.full(capacity, -1, dtype=np.int64)
        self.cost = np.empty(capacity)
        self.children: list[list[int]] = []
        self.n = 0
        self.add(np.asarray(root, float), -1, 0.0)

    def add(self, p: np.ndarray, parent: int, cost: float) -> int:
        if self.n == len(self.pos):
            grow = len(self.pos)
            self.pos = np.vstack([self.pos, np.empty((grow, 3))])
            self.parent = np.concatenate([self.parent, np.full(grow, -1, dtype=np.int64)])
            self.cost = np.concatenate([self.cost, np.empty(grow)])
        i = self.n
        self.pos[i] = p
        self.parent[i] = parent
        self.cost[i] = cost
        self.children.append([])
        if parent >= 0:
            self.children[parent].append(i)
        self.n += 1
        return i

    def edge_len(self, i: int, j: int) -> float:
        return float(np.linalg.norm(self.pos[i] - self.pos[j]))

    def set_parent(self, i: int, new_parent: int) -> int:
        """Re-attach node i under new_parent and recompute subtree costs.

        Returns the number of nodes whose cost was updated (i and all descendants).
        """
        old = int(self.parent[i])
        if old >= 0:
            self.children[old].remove(i)
        self.parent[i] = new_parent
        self.children[new_parent].append(i)
        self.cost[i] = self.cost[new_parent] + self.edge_len(new_parent, i)
        return 1 + self._propagate(i)

    def _propagate(self, i: int) -> int:
        q = deque([i])
        updated = 0
        while q:
            u = q.popleft()
            for ch in self.children[u]:
                self.cost[ch] = self.cost[u] + self.edge_len(u, ch)
                updated += 1
                q.append(ch)
        return updated

    def sq_dist(self, p: np.ndarray) -> np.ndarray:
        d = self.pos[: self.n] - p
        return np.einsum("ij,ij->i", d, d)

    def path_to_root(self, i: int) -> np.ndarray:
        idx = []
        while i >= 0:
            idx.append(i)
            i = int(self.parent[i])
        return self.pos[idx[::-1]].copy()

    def check_consistency(self, tol: float = 1e-6) -> list[str]:
        """Return a list of violated invariants (empty if the tree is consistent)."""
        errs = []
        if self.parent[0] != -1 or abs(self.cost[0]) > tol:
            errs.append("root")
        for i in range(1, self.n):
            p = int(self.parent[i])
            if not (0 <= p < self.n):
                errs.append(f"node {i} bad parent {p}")
                continue
            if i not in self.children[p]:
                errs.append(f"node {i} missing from children of {p}")
            exp = self.cost[p] + self.edge_len(p, i)
            if abs(self.cost[i] - exp) > tol:
                errs.append(f"node {i} cost {self.cost[i]:.6f} != {exp:.6f}")
        for p in range(self.n):
            for ch in self.children[p]:
                if self.parent[ch] != p:
                    errs.append(f"child {ch} of {p} has parent {self.parent[ch]}")
        # acyclicity: every node reaches the root within n steps
        for i in range(self.n):
            j, steps = i, 0
            while j > 0 and steps <= self.n:
                j = int(self.parent[j])
                steps += 1
            if j != 0:
                errs.append(f"node {i} does not reach root")
                break
        return errs


def choose_parent(tree: RRTStarTree, checker: CollisionChecker, x: np.ndarray,
                  near: np.ndarray) -> tuple[int, float, int]:
    """Minimum-cost collision-free parent among `near`.

    Candidates are tested in increasing order of cost-to-come through them, so the first
    collision-free one is the minimum. Returns (parent, cost, edge_checks); parent = -1
    if none is collision-free.
    """
    d = np.linalg.norm(tree.pos[near] - x, axis=1)
    c = tree.cost[near] + d
    order = np.lexsort((near, c))
    checks = 0
    for k in order:
        checks += 1
        if checker.segment_free(tree.pos[near[k]], x):
            return int(near[k]), float(c[k]), checks
    return -1, math.inf, checks


def rewire(tree: RRTStarTree, checker: CollisionChecker, new: int,
           near: np.ndarray) -> tuple[int, int, int]:
    """Re-parent near nodes through `new` when that strictly lowers their cost.

    Returns (rewires, edge_checks, cost_updates).
    """
    rewires = checks = updates = 0
    xn = tree.pos[new]
    for m in np.sort(near):
        m = int(m)
        if m == new or m == tree.parent[new]:
            continue
        nc = tree.cost[new] + float(np.linalg.norm(tree.pos[m] - xn))
        if nc < tree.cost[m] - _COST_EPS:
            checks += 1
            if checker.segment_free(xn, tree.pos[m]):
                updates += tree.set_parent(m, new)
                rewires += 1
    return rewires, checks, updates


class RRTStarPlanner:
    name = "rrt_star"

    def __init__(self, checker: CollisionChecker, max_iterations: int = 3000,
                 step_size: float = 1.5, goal_bias: float = 0.05, gamma_factor: float = 1.0,
                 time_limit_s: float = 120.0, seed: int = 0, record_tree: bool = False,
                 checkpoints: tuple[int, ...] = ()) -> None:
        t0 = time.perf_counter()
        self.checkpoints = tuple(int(c) for c in checkpoints)
        self.checker = checker
        self.max_iterations = int(max_iterations)
        self.step = float(step_size)
        self.goal_bias = float(goal_bias)
        self.time_limit_s = float(time_limit_s)
        self.seed = int(seed)
        self.record_tree = record_tree
        g = checker.grid
        c = checker.c
        self.sample_lo = g.lo + c
        self.sample_hi = g.hi - c
        # Lebesgue measure of the free configuration space, from inflated free voxels
        free_vol = float((checker.f_total >= c).sum()) * g.resolution ** 3
        zeta3 = 4.0 / 3.0 * math.pi
        self.gamma = gamma_factor * 2.0 * (1.0 + 1.0 / 3.0) ** (1.0 / 3.0) * (free_vol / zeta3) ** (1.0 / 3.0)
        self.free_volume = free_vol
        self.setup_time_s = time.perf_counter() - t0

    def near_radius(self, n: int) -> float:
        n = max(n, 2)
        return min(self.gamma * (math.log(n) / n) ** (1.0 / 3.0), self.step)

    def plan(self, start, goal) -> PlanResult:
        start = np.asarray(start, float)
        goal = np.asarray(goal, float)
        bad = validate_endpoints(self.checker, start, goal, self.name)
        if bad is not None:
            bad.setup_time_s = self.setup_time_s
            return bad
        ck = self.checker
        seg0, pts0 = ck.stats.segment_checks, ck.stats.point_queries
        t0 = time.perf_counter()
        if np.linalg.norm(goal - start) < 1e-9:
            return PlanResult(self.name, Status.SUCCESS, start[None, :].copy(), 0.0,
                              time.perf_counter() - t0, self.setup_time_s,
                              stats={"iterations": 0, "tree_nodes": 1, "zero_length": True})

        rng = np.random.default_rng(self.seed)
        tree = RRTStarTree(start, self.max_iterations + 1)
        goal_nodes: list[int] = []
        goal_dist: list[float] = []
        best_cost, best_node = math.inf, -1
        history: list[tuple[int, float, float]] = []   # (iteration, best cost, elapsed s)
        rejected_point = rejected_edge = rewires = cost_updates = 0
        first_solution_it = None
        cps = set(self.checkpoints)
        snapshots: list[dict] = []

        def snap(k: int) -> None:
            snapshots.append({"iteration": k, "best_cost": best_cost, "elapsed_s": time.perf_counter() - t0,
                              "tree_nodes": tree.n, "segment_checks": ck.stats.segment_checks - seg0})

        it = 0
        for it in range(1, self.max_iterations + 1):
            if (it - 1) in cps:
                snap(it - 1)
            if (it & 63) == 0 and time.perf_counter() - t0 > self.time_limit_s:
                it -= 1
                break
            if rng.random() < self.goal_bias:
                x_rand = goal
            else:
                x_rand = rng.uniform(self.sample_lo, self.sample_hi)
            d2 = tree.sq_dist(x_rand)
            i_near = int(np.argmin(d2))
            dn = math.sqrt(d2[i_near])
            if dn < 1e-12:
                continue
            x_new = x_rand if dn <= self.step else tree.pos[i_near] + (x_rand - tree.pos[i_near]) * (self.step / dn)
            if not ck.point_free(x_new)[0]:
                rejected_point += 1
                continue
            radius = self.near_radius(tree.n)
            dnew = tree.sq_dist(x_new)
            near = np.flatnonzero(dnew <= radius * radius)
            if i_near not in near:
                near = np.append(near, i_near)
            if np.min(dnew[near]) < 1e-12:          # duplicate of an existing node
                continue
            parent, cost, _ = choose_parent(tree, ck, x_new, near)
            if parent < 0:
                rejected_edge += 1
                continue
            new = tree.add(x_new, parent, cost)
            rw, _, upd = rewire(tree, ck, new, near)
            rewires += rw
            cost_updates += upd
            dg = float(np.linalg.norm(goal - x_new))
            if dg <= self.step and ck.segment_free(x_new, goal):
                goal_nodes.append(new)
                goal_dist.append(dg)
            if goal_nodes:
                tot = tree.cost[goal_nodes] + np.asarray(goal_dist)
                k = int(np.argmin(tot))
                if tot[k] < best_cost - _COST_EPS:
                    best_cost, best_node = float(tot[k]), goal_nodes[k]
                    history.append((it, best_cost, time.perf_counter() - t0))
                    if first_solution_it is None:
                        first_solution_it = it
                elif goal_nodes[k] != best_node or abs(tot[k] - best_cost) > _COST_EPS:
                    best_cost, best_node = float(tot[k]), goal_nodes[k]
        if it in cps and (not snapshots or snapshots[-1]["iteration"] != it):
            snap(it)
        elapsed = time.perf_counter() - t0
        stats = {
            "iterations": it, "samples": it, "tree_nodes": tree.n,
            "rejected_point": rejected_point, "rejected_edge": rejected_edge,
            "rewires": rewires, "cost_updates": cost_updates,
            "segment_checks": ck.stats.segment_checks - seg0,
            "point_queries": ck.stats.point_queries - pts0,
            "goal_connections": len(goal_nodes),
            "first_solution_iteration": first_solution_it,
            "cost_history": history, "checkpoints": snapshots, "seed": self.seed, "gamma": self.gamma,
            "tree_consistency_errors": tree.check_consistency() if tree.n <= 20000 else None,
        }
        if self.record_tree:
            stats["tree_pos"] = tree.pos[: tree.n].copy()
            stats["tree_parent"] = tree.parent[: tree.n].copy()
        if best_node < 0:
            return PlanResult(self.name, Status.BUDGET_EXHAUSTED, None, float("nan"), elapsed,
                              self.setup_time_s, reason="no goal connection within budget", stats=stats)
        path = tree.path_to_root(best_node)
        if np.linalg.norm(path[-1] - goal) > 1e-12:
            path = np.vstack([path, goal])
        return PlanResult(self.name, Status.SUCCESS, path, path_length(path), elapsed,
                          self.setup_time_s, stats=stats)

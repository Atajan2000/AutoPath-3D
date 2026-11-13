"""Collision-checked greedy shortcutting of a waypoint path.

A shortcut a -> b is accepted only if the shared CollisionChecker certifies the whole
segment (Lipschitz certificate, not endpoint checks). The required clearance is the
checker's robot clearance, so a stricter clearance is obtained by passing a checker built
with a larger safety margin. The input path itself must be certified first; otherwise the
result is rejected rather than "repaired".
"""

from __future__ import annotations

import time
from dataclasses import dataclass

import numpy as np

from ..collision.checker import CollisionChecker
from ..planners.common import path_length


@dataclass
class ShortcutResult:
    success: bool
    path: np.ndarray | None
    reason: str
    original_length: float
    length: float
    original_waypoints: int
    waypoints: int
    sampled_min_clearance: float      # checker lower bound along the result (diagnostic)
    time_s: float
    segment_checks: int
    rejected_shortcuts: int

    def summary(self) -> dict:
        d = dict(self.__dict__)
        d.pop("path")
        return d


def dedupe(path: np.ndarray, tol: float = 1e-9) -> np.ndarray:
    path = np.atleast_2d(np.asarray(path, float))
    keep = np.ones(len(path), bool)
    keep[1:] = np.linalg.norm(np.diff(path, axis=0), axis=1) > tol
    return path[keep]


def shortcut_path(path: np.ndarray, checker: CollisionChecker, max_passes: int = 4) -> ShortcutResult:
    t0 = time.perf_counter()
    seg0 = checker.stats.segment_checks
    raw = np.atleast_2d(np.asarray(path, float))
    p = dedupe(raw)
    L0 = path_length(raw)

    def result(ok, out, reason, rejected):
        return ShortcutResult(
            ok, out, reason, L0, path_length(out) if out is not None else float("nan"), len(raw),
            0 if out is None else len(out),
            checker.sampled_min_clearance(out) if out is not None else float("nan"),
            time.perf_counter() - t0, checker.stats.segment_checks - seg0, rejected)

    if not checker.path_free(p):
        return result(False, None, "input path not certified at the required clearance", 0)
    rejected = 0
    for _ in range(max_passes):
        out = [p[0]]
        i = 0
        n = len(p)
        while i < n - 1:
            j = i + 1
            while j + 1 < n and checker.segment_free(p[i], p[j + 1]):
                j += 1
            if j + 1 < n:
                rejected += 1
            out.append(p[j])
            i = j
        out = np.array(out)
        if len(out) == len(p):
            p = out
            break
        p = out
    return result(True, p, "", rejected)

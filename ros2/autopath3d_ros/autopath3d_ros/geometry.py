"""Collision geometry exchanged between Webots and AutoPath-3D (pure Python, no ROS imports).

A *primitive record* is a dict produced by the Webots extractor (webots_extract.py) from the
actual boundingObject geometry of the running simulation, in the world frame (= ROS `map`):
  {"object": top-level object name, "solid": owning Solid name, "kind": "box" | "cylinder",
   "lo": [x, y, z], "hi": [x, y, z]                       (kind == "box": axis-aligned box)
   "center": [x, y], "radius": r, "z": [z_lo, z_hi]       (kind == "cylinder": vertical axis)
   "source": "collision" | "envelope" | "visual"}
Oriented boxes, horizontal cylinders, spheres and capsules are converted by the extractor to
their axis-aligned bounding boxes (conservative). Only vertical cylinders stay exact.

`scene_from_records` turns records into the existing AutoPath `Scene` (core Box /
VerticalCylinder primitives), so map building, planning, trajectory generation, validation and
replanning all use the unchanged Phase 1-3 code.
"""

from __future__ import annotations

import fnmatch
import json
from dataclasses import dataclass

import numpy as np

from autopath3d.environments.primitives import Box, VerticalCylinder
from autopath3d.environments.scenes import Scene, Task

MIN_EXTENT = 1e-3     # core primitives need positive extent; thinner boxes are thickened


def record_aabb(r: dict) -> tuple[np.ndarray, np.ndarray]:
    if r["kind"] == "box":
        return np.asarray(r["lo"], float), np.asarray(r["hi"], float)
    c, rad = np.asarray(r["center"], float), float(r["radius"])
    return (np.array([c[0] - rad, c[1] - rad, r["z"][0]]), np.array([c[0] + rad, c[1] + rad, r["z"][1]]))


def to_primitive(r: dict, name: str | None = None):
    name = name or f"{r.get('object', '?')}/{r.get('solid', '?')}"
    if r["kind"] == "box":
        lo, hi = np.asarray(r["lo"], float), np.asarray(r["hi"], float)
        mid = 0.5 * (lo + hi)
        half = np.maximum(0.5 * (hi - lo), 0.5 * MIN_EXTENT)
        return Box(tuple(mid - half), tuple(mid + half), name)
    if r["kind"] == "cylinder":
        z0, z1 = r["z"]
        if z1 - z0 < MIN_EXTENT:
            z0, z1 = 0.5 * (z0 + z1) - 0.5 * MIN_EXTENT, 0.5 * (z0 + z1) + 0.5 * MIN_EXTENT
        return VerticalCylinder(tuple(r["center"]), float(r["radius"]), float(z0), float(z1), name)
    raise ValueError(f"unsupported primitive kind {r['kind']}")


def intersects(r: dict, lo, hi) -> bool:
    a, b = record_aabb(r)
    return bool(np.all(b > np.asarray(lo)) and np.all(a < np.asarray(hi)))


def envelope_records(records: list[dict], patterns: list[str]) -> list[dict]:
    """One conservative AABB per object whose name matches a pattern (e.g. "rack *"): the
    planner then treats e.g. rack bays as fully occupied (superset of posts/beams/decks/cargo)."""
    groups: dict[str, list] = {}
    for r in records:
        if any(fnmatch.fnmatch(r["object"], p) for p in patterns):
            groups.setdefault(r["object"], []).append(record_aabb(r))
    out = []
    for name, boxes in sorted(groups.items()):
        lo = np.min([b[0] for b in boxes], axis=0)
        hi = np.max([b[1] for b in boxes], axis=0)
        out.append({"object": name, "solid": name, "kind": "box", "lo": lo.tolist(), "hi": hi.tolist(),
                    "source": "envelope"})
    return out


def objects_in_records(records: list[dict]) -> set[str]:
    return {r["object"] for r in records}


@dataclass
class WorldGeometry:
    """Static map geometry: bounds (planning geofence) and primitive records."""
    lo: np.ndarray
    hi: np.ndarray
    records: list

    def scene(self, name: str = "webots_warehouse", tasks: list[Task] | None = None) -> Scene:
        prims = []
        for i, r in enumerate(self.records):
            if intersects(r, self.lo, self.hi):
                prims.append(to_primitive(r, f"{i}:{r['object']}/{r['solid']}"))
        return Scene(name, "Webots warehouse collision geometry (extracted boundingObjects)",
                     np.asarray(self.lo, float), np.asarray(self.hi, float), prims, tasks or [])

    def to_json(self) -> str:
        return json.dumps({"lo": list(map(float, self.lo)), "hi": list(map(float, self.hi)),
                           "records": self.records}, indent=1)

    @classmethod
    def from_json(cls, text: str) -> "WorldGeometry":
        d = json.loads(text)
        return cls(np.asarray(d["lo"], float), np.asarray(d["hi"], float), d["records"])


def scene_from_records(records: list[dict], lo, hi, envelope_patterns=(), exclude_objects=(),
                       tasks=None) -> Scene:
    recs = [r for r in records if r["object"] not in set(exclude_objects)]
    recs = recs + envelope_records(recs, list(envelope_patterns))
    return WorldGeometry(np.asarray(lo, float), np.asarray(hi, float), recs).scene(tasks=tasks)


def visual_audit(grid, visual: list[dict], lo, hi, min_height: float = 0.01) -> list[dict]:
    """Visual geometry inside the geofence whose voxels are not all OCCUPIED (i.e. visible
    parts without collision geometry). Visual records are AABBs (see webots_extract.py)."""
    from autopath3d.maps.occupancy import OCCUPIED
    out = []
    for r in visual:
        a, b = record_aabb(r)
        if not intersects(r, lo, hi) or (b[2] < 0.02 and b[2] - a[2] < min_height):
            continue            # flat decals on the floor (e.g. the start pad)
        i0, i1 = grid._index_range(np.maximum(a, lo), np.minimum(b, hi))
        if np.any(i1 <= i0):
            continue
        view = grid.states[tuple(slice(x, y) for x, y in zip(i0, i1))]
        n_free = int((view != OCCUPIED).sum())
        if n_free:
            out.append({**r, "uncovered_voxels": n_free})
    return out


def build_planning_records(base_records: list[dict], visual_records: list[dict], cfg: dict,
                           updates: list[dict] = ()) -> tuple[list[dict], list[dict]]:
    """The map node's planning geometry: collision records inside the geofence + envelopes of
    the configured objects + visual parts without collision geometry (conservative) + updates.
    Returns (records, visual_audit_additions)."""
    from autopath3d.maps.occupancy import OccupancyGrid
    m, b = cfg["map"], cfg["bounds"]
    lo, hi = np.asarray(b["lo"], float), np.asarray(b["hi"], float)
    recs = [r for r in base_records if intersects(r, lo, hi)]
    recs += envelope_records(recs, m.get("envelope_objects", []))
    grid = OccupancyGrid.from_scene(WorldGeometry(lo, hi, recs).scene(), m["resolution"])
    audit = visual_audit(grid, visual_records, lo, hi)
    for r in audit:
        recs.append({k: r[k] for k in ("object", "solid", "kind", "lo", "hi")} | {"source": "visual"})
    recs += [r for r in updates if intersects(r, lo, hi)]
    return recs, audit


def conflict_check(traj, update, t_from: float, c_traj: float) -> bool:
    """Brake trigger = the Phase 2 `assess_update` step 1 criterion, verbatim: the remaining
    trajectory is not certified (certify_curve over its Bernstein windows) to keep the trajectory
    clearance from the new obstacle. Run on its own so the brake is not delayed by the stop
    prediction (step 2). Equivalence with assess_update is unit-tested."""
    from autopath3d.trajectories.generator import certify_curve

    def new_dist(q):
        q = np.atleast_2d(q)
        return np.min([ob.distance(q) for ob in update.obstacles], axis=0)
    if traj.n_pieces == 0:
        return bool(new_dist(traj.hold_point)[0] < c_traj)
    return any(not certify_curve(P, new_dist, c_traj) for P in traj.bernstein_window(t_from, traj.t_end))


def compare_records(a: list[dict], b: list[dict], tol: float = 1e-4) -> list[str]:
    """Differences between two record lists (by object/solid/kind, in order). Empty = identical."""
    def key(r):
        return (r["object"], r["solid"], r["kind"])
    diffs = []
    ka, kb = sorted(a, key=lambda r: (key(r), record_aabb(r)[0].tolist())), \
        sorted(b, key=lambda r: (key(r), record_aabb(r)[0].tolist()))
    if len(ka) != len(kb):
        diffs.append(f"record count {len(ka)} != {len(kb)}")
    for ra, rb in zip(ka, kb):
        if key(ra) != key(rb):
            diffs.append(f"{key(ra)} != {key(rb)}")
            continue
        (a0, a1), (b0, b1) = record_aabb(ra), record_aabb(rb)
        err = max(np.abs(a0 - b0).max(), np.abs(a1 - b1).max())
        if err > tol:
            diffs.append(f"{key(ra)} differs by {err:.4g} m")
    return diffs

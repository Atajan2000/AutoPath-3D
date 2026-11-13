"""Run metrics from recorded Webots/ROS data (pure Python, unit-tested).

Inputs are plain arrays recorded by the mission runner:
  bridge  per physics step: t, mode, p (GPS = true position, noise-free in this world), p_ref, err
  gt      per physics step: t, p, v (supervisor ground truth)
  contacts: collision contact events from Webots physics (floor contacts while landed excluded)
Clearance is the distance from the drone centre to the extracted collision geometry (AABBs of the
real boundingObjects; vertical cylinders exact), so it is a lower bound of the distance to the
true collision shapes. Geometry added by an obstacle update counts from its t_appear.
"""

from __future__ import annotations

import numpy as np


def tracking_stats(err: np.ndarray) -> dict:
    if len(err) == 0:
        return {"n": 0}
    n = np.linalg.norm(err, axis=1)
    return {"n": int(len(n)), "max": float(n.max()), "rms": float(np.sqrt(np.mean(n * n))),
            "mean": float(n.mean()), "p95": float(np.percentile(n, 95)),
            "max_horizontal": float(np.linalg.norm(err[:, :2], axis=1).max()),
            "max_vertical": float(np.abs(err[:, 2]).max())}


def speed_accel(t: np.ndarray, v: np.ndarray, smooth_s: float = 0.048) -> dict:
    """Max speed and max acceleration (central difference of velocity over ~smooth_s)."""
    if len(t) < 3:
        return {}
    sp = np.linalg.norm(v, axis=1)
    dt = float(np.median(np.diff(t)))
    k = max(1, int(round(smooth_s / dt / 2)))
    a = (v[2 * k:] - v[:-2 * k]) / (t[2 * k:] - t[:-2 * k])[:, None]
    an = np.linalg.norm(a, axis=1)
    ah = np.linalg.norm(a[:, :2], axis=1)
    return {"max_speed": float(sp.max()), "max_accel": float(an.max()), "max_accel_horizontal": float(ah.max()),
            "accel_window_s": 2 * k * dt}


def clearance_series(p: np.ndarray, t: np.ndarray, base_dist, update_dists: list[tuple[float, object]]) -> np.ndarray:
    d = base_dist(p)
    for t_appear, fn in update_dists:
        m = t >= t_appear
        if m.any():
            d[m] = np.minimum(d[m], fn(p[m]))
    return d


def contact_events(contacts: list[dict]) -> dict:
    coll = [c for c in contacts if c.get("collision")]
    n = 0
    prev = None
    for c in coll:
        if prev is None or c["t"] - prev > 0.05:
            n += 1
        prev = c["t"]
    return {"collision_events": n, "collision_samples": len(coll),
            "first_collision_t": coll[0]["t"] if coll else None,
            "first_collision_points": coll[0]["points"] if coll else None}


def realtime_factor(sim_t: tuple[float, float], wall_t: tuple[float, float]) -> float | None:
    dw = wall_t[1] - wall_t[0]
    return (sim_t[1] - sim_t[0]) / dw if dw > 0 else None

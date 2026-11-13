"""Stopping-distance model for the translational double integrator.

Emergency stop: after a total latency t_L (detection + decision) during which the robot
keeps executing its current trajectory, it applies the constant deceleration a_dec
opposite to its velocity until rest. With speed v at the start of braking the braking
segment is a straight line of length v^2 / (2 a_dec) taking v / a_dec seconds.
Baseline (constant speed during latency):  d_stop = v t_L + v^2 / (2 a_dec).
"""

from __future__ import annotations

import numpy as np

from ..trajectories.polynomial import PiecewisePolynomial


def stopping_distance(speed: float, max_deceleration: float, latency_s: float) -> float:
    return speed * latency_s + speed * speed / (2.0 * max_deceleration)


def braking_trajectory(position, velocity, max_deceleration: float, t0: float) -> PiecewisePolynomial:
    """Straight-line constant-deceleration stop from (position, velocity) at time t0."""
    p = np.asarray(position, float)
    v = np.asarray(velocity, float)
    speed = float(np.linalg.norm(v))
    if speed < 1e-9:
        return PiecewisePolynomial.stationary(p, t0)
    T = speed / max_deceleration
    c = np.zeros((6, 3))
    c[0], c[1], c[2] = p, v, -0.5 * max_deceleration * v / speed
    return PiecewisePolynomial.from_pieces([c], [T], t0)

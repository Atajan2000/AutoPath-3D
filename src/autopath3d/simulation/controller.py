"""Trajectory-following controller: acceleration feedforward + PD feedback, bounded.

Tracking / holding:
    a = a_ref(t_k + dt/2) + kp (p_ref(t_k) - p_k) + kd (v_ref(t_k) - v_k)
    a <- a * min(1, a_max / |a|)                      (Euclidean norm saturation, a_max = 2 m/s^2)
    speed governor: if |v_k + a dt| > v_max, the next velocity is projected radially onto the
    ball |v| <= v_max (non-expansive projection, so |a| does not grow).
The midpoint reference acceleration is the second-order-accurate ZOH feedforward. The
reference trajectory itself is never modified.

Emergency braking (Phase 2 model, executed on the true state):
    a = -a_dec v / |v|   (a_dec = 3 m/s^2) until |v| <= a_dec dt; the final step brings the
    commanded velocity exactly to zero (|a| <= a_dec), after which the robot holds position.

Gains: kp = w^2, kd = 2 zeta w with w = 3 rad/s, zeta = 1 (critically damped, ~0.5 Hz
bandwidth, far below the 100 Hz control rate; discrete stability checked in tests).
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class ControllerConfig:
    kp: float = 9.0              # 1/s^2
    kd: float = 6.0              # 1/s
    a_track_max: float = 2.0     # m/s^2, |a_cmd| while tracking or holding (nominal limit)
    a_brake_max: float = 3.0     # m/s^2, emergency braking capability
    v_max: float = 3.0           # m/s


def clip_norm(a: np.ndarray, limit: float) -> tuple[np.ndarray, bool]:
    n = float(np.linalg.norm(a))
    if n > limit:
        return a * (limit / n), True
    return a, False


def speed_governor(v: np.ndarray, a: np.ndarray, dt: float, v_max: float) -> tuple[np.ndarray, bool]:
    u = v + a * dt
    n = float(np.linalg.norm(u))
    if n <= v_max:
        return a, False
    return (u * (v_max / n) - v) / dt, True


def discrete_spectral_radius(kp: float, kd: float, dt: float) -> float:
    """Spectral radius of the per-axis closed loop (ZOH double integrator + PD) at period dt."""
    A = np.array([[1.0 - 0.5 * kp * dt * dt, dt - 0.5 * kd * dt * dt],
                  [-kp * dt, 1.0 - kd * dt]])
    return float(np.max(np.abs(np.linalg.eigvals(A))))


class TrackingController:
    def __init__(self, cfg: ControllerConfig = ControllerConfig()) -> None:
        self.cfg = cfg

    def track(self, p, v, p_ref, v_ref, a_ref, dt: float) -> tuple[np.ndarray, bool, bool]:
        """Returns (a_cmd, saturated, governed)."""
        c = self.cfg
        a = a_ref + c.kp * (p_ref - p) + c.kd * (v_ref - v)
        a, sat = clip_norm(a, c.a_track_max)
        a, gov = speed_governor(v, a, dt, c.v_max)
        a, sat2 = clip_norm(a, c.a_track_max)
        return a, sat or sat2, gov

    def hold(self, p, v, p_hold, dt: float) -> tuple[np.ndarray, bool, bool]:
        z = np.zeros(3)
        return self.track(p, v, p_hold, z, z, dt)

    def brake(self, v, dt: float) -> tuple[np.ndarray, bool]:
        """Returns (a_cmd, finished): finished when this step brings the commanded velocity to 0."""
        a_dec = self.cfg.a_brake_max
        s = float(np.linalg.norm(v))
        if s <= a_dec * dt:
            return -v / dt, True
        return -a_dec * v / s, False

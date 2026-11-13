"""Translational double-integrator drone model used by the Phase 3 simulation.

State (p, v) in the world frame W (right-handed, x/y horizontal, z up; metres, seconds).
Input: the commanded acceleration a_cmd (bounded by the controller). The vehicle also
experiences an exogenous disturbance acceleration d(t) (a deterministic wind-gust model)
that the controller does not know:

    p' = v,    v' = a_cmd + d(t)

Discretisation: zero-order hold with an explicit control period dt. During step k
(t in [t_k, t_k + dt], t_k = k dt) a_cmd and d are held constant (d is sampled at the step
midpoint), and the step is integrated exactly:

    a_k     = a_cmd,k + d(t_k + dt/2)
    p_{k+1} = p_k + v_k dt + a_k dt^2 / 2
    v_{k+1} = v_k + a_k dt

So the modelled continuous-time motion inside a step is the quadratic
p_k + v_k tau + a_k tau^2 / 2, i.e. the quadratic Bezier curve with control points
(p_k, p_k + v_k dt / 2, p_{k+1}); the safety monitor bounds clearance on these curves.

NOT modelled: attitude, thrust and rotor dynamics, aerodynamic drag, actuator lag,
sensing noise and state estimation (the controller reads the true state). Results from
this model validate translational navigation only, not quadrotor flight.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class Disturbance:
    """Deterministic smooth disturbance acceleration [m/s^2]: bias + two sinusoids per axis.

    d_i(t) = bias_i + amp1_i sin(w1_i t + phi1_i) + amp2_i sin(w2_i t + phi2_i)
    """

    name: str = "ideal"
    bias: tuple = (0.0, 0.0, 0.0)
    amp1: tuple = (0.0, 0.0, 0.0)
    freq1: tuple = (0.0, 0.0, 0.0)     # rad/s
    phase1: tuple = (0.0, 0.0, 0.0)    # rad
    amp2: tuple = (0.0, 0.0, 0.0)
    freq2: tuple = (0.0, 0.0, 0.0)
    phase2: tuple = (0.0, 0.0, 0.0)

    def __call__(self, t: float) -> np.ndarray:
        b, a1, w1, f1 = (np.asarray(x, float) for x in (self.bias, self.amp1, self.freq1, self.phase1))
        a2, w2, f2 = (np.asarray(x, float) for x in (self.amp2, self.freq2, self.phase2))
        return b + a1 * np.sin(w1 * t + f1) + a2 * np.sin(w2 * t + f2)

    def axis_bound(self) -> np.ndarray:
        """Upper bound on |d_i(t)| per axis."""
        return np.abs(self.bias) + np.abs(self.amp1) + np.abs(self.amp2)

    def norm_bound(self) -> float:
        return float(np.linalg.norm(self.axis_bound()))


IDEAL = Disturbance("ideal")

# Light, smooth wind gusts (fixed before any scenario was simulated; not tuned per scenario).
# Horizontal per-axis peak 0.30 / 0.28 m/s^2, vertical 0.10 m/s^2, |d| <= 0.42 m/s^2.
GUSTS = Disturbance("gusts", bias=(0.05, -0.03, 0.0),
                    amp1=(0.15, 0.15, 0.06), freq1=(0.7, 0.5, 0.9), phase1=(0.3, 2.0, 0.7),
                    amp2=(0.10, 0.10, 0.04), freq2=(1.9, 1.3, 2.3), phase2=(1.1, 0.4, 2.5))

DISTURBANCES = {"ideal": IDEAL, "gusts": GUSTS}


def get_disturbance(name: str) -> Disturbance:
    if name not in DISTURBANCES:
        raise KeyError(f"unknown disturbance profile {name!r}; available: {sorted(DISTURBANCES)}")
    return DISTURBANCES[name]


def zoh_step(p: np.ndarray, v: np.ndarray, a: np.ndarray, dt: float) -> tuple[np.ndarray, np.ndarray]:
    """Exact integration of p' = v, v' = a over dt for constant a."""
    return p + v * dt + 0.5 * a * dt * dt, v + a * dt


def step_control_points(p: np.ndarray, v: np.ndarray, p_next: np.ndarray, dt: float) -> np.ndarray:
    """Quadratic Bezier control points (..., 3, 3) of the motion inside one ZOH step."""
    return np.stack([p, p + 0.5 * dt * v, p_next], axis=-2)

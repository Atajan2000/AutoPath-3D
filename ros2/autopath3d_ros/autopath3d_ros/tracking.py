"""Outer trajectory-tracking loop for the Webots Crazyflie (AutoPath-3D bridge code).

Cascade (only the outermost stage is AutoPath code):
  AutoPath reference p_r(t), v_r(t), a_r(t)            (validated trajectory, world/map frame)
    -> outer loop (this module): bounded velocity command  v_c = v_r + T_a a_r + K_p (p_r - p)
       and altitude set-point  z_c = z_r + T_z vz_r, yaw-rate command holding a fixed yaw
    -> Bitcraze `pid_velocity_fixed_height_controller` (third-party, webots_ros2_crazyflie,
       MIT): velocity PID -> attitude PID -> motor mixing   (body-frame vx/vy, altitude, yaw rate)
    -> Webots propeller physics (thrust/torque constants, mass, inertia, gravity)
No position is ever written to the simulator: the drone moves only through motor commands.

All gains and bounds are explicit; the command saturation is the controller command bound
checked by the tests. Pure Python, no Webots or ROS imports.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class OuterLoopGains:
    kp_xy: float = 1.2          # 1/s, horizontal position error -> velocity correction
    accel_lead_s: float = 0.25  # s, acceleration feed-forward (compensates the velocity-loop lag)
    z_lead_s: float = 0.4       # s, altitude set-point lead on the vertical reference velocity
    kp_z: float = 0.0           # extra altitude-error gain added to the set-point (PID has its own)
    v_xy_max: float = 1.0       # m/s, bound on the horizontal velocity command
    z_step_max: float = 0.6     # m, bound on |z_c - z| handed to the altitude PID
    yaw_kp: float = 1.0         # 1/s
    yaw_rate_max: float = 1.0   # rad/s

    def __post_init__(self):
        if min(self.v_xy_max, self.z_step_max, self.yaw_rate_max) <= 0:
            raise ValueError("command bounds must be positive")


@dataclass
class OuterCommand:
    vx_body: float       # forward velocity command (body x), m/s
    vy_body: float       # sideways velocity command (body y, left), m/s
    z_setpoint: float    # absolute altitude set-point, m
    yaw_rate: float      # rad/s
    saturated: bool      # horizontal command hit v_xy_max


def wrap_angle(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def world_to_body_xy(v_world_xy, yaw: float) -> tuple[float, float]:
    """Rotate a horizontal world (ENU map) vector into the yaw-aligned body frame (FLU)."""
    c, s = math.cos(yaw), math.sin(yaw)
    vx, vy = float(v_world_xy[0]), float(v_world_xy[1])
    return c * vx + s * vy, -s * vx + c * vy


def outer_loop(p_ref, v_ref, a_ref, p, yaw: float, g: OuterLoopGains, yaw_ref: float = 0.0) -> OuterCommand:
    p_ref, v_ref, a_ref, p = (np.asarray(x, float) for x in (p_ref, v_ref, a_ref, p))
    v_cmd = v_ref[:2] + g.accel_lead_s * a_ref[:2] + g.kp_xy * (p_ref[:2] - p[:2])
    n = float(np.linalg.norm(v_cmd))
    sat = n > g.v_xy_max
    if sat:
        v_cmd = v_cmd * (g.v_xy_max / n)
    bx, by = world_to_body_xy(v_cmd, yaw)
    z_c = p_ref[2] + g.z_lead_s * v_ref[2] + g.kp_z * (p_ref[2] - p[2])
    z_c = float(np.clip(z_c, p[2] - g.z_step_max, p[2] + g.z_step_max))
    yr = float(np.clip(g.yaw_kp * wrap_angle(yaw_ref - yaw), -g.yaw_rate_max, g.yaw_rate_max))
    return OuterCommand(bx, by, z_c, yr, sat)


def brake_command(p, z_hold: float, yaw: float, g: OuterLoopGains, yaw_ref: float = 0.0) -> OuterCommand:
    """Emergency stop: zero horizontal velocity command (the velocity PID decelerates with its
    full authority), altitude held at the brake-onset value."""
    z_c = float(np.clip(z_hold, p[2] - g.z_step_max, p[2] + g.z_step_max))
    yr = float(np.clip(g.yaw_kp * wrap_angle(yaw_ref - yaw), -g.yaw_rate_max, g.yaw_rate_max))
    return OuterCommand(0.0, 0.0, z_c, yr, False)

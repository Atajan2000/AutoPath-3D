"""Webots driver plugin: AutoPath-3D <-> Crazyflie flight-control bridge.

Loaded by webots_ros2_driver inside the Crazyflie's extern controller process (see
resource/crazyflie_autopath.urdf). Runs synchronously with the physics step (16 ms).

Third-party vs AutoPath code:
  * `pid_velocity_fixed_height_controller` and the sensor/motor sign conventions are imported
    unmodified from webots_ros2_crazyflie (Bitcraze, MIT licence) - the flight controller.
  * This file (AutoPath) adds: mode logic, the outer trajectory-tracking loop (tracking.py),
    state publication with simulation-time stamps, and trajectory/command subscriptions.
The drone is moved only through motor velocity commands; its pose is never written.

Topics (all stamps = Webots simulation time):
  pub  /autopath/odom                 nav_msgs/Odometry (map -> base_link, twist in base_link)
  pub  /tf                            map -> base_link
  pub  /autopath/bridge/state         std_msgs/String JSON (mode, reference, error, command), every step
  pub  /autopath/tracking_error       geometry_msgs/Vector3Stamped (p_ref - p, map frame)
  sub  /autopath/active_trajectory    trajectory_msgs/MultiDOFJointTrajectory (lossless knot states)
  sub  /autopath/bridge/command       std_msgs/String JSON {"cmd": takeoff|brake|hold|land|disarm, ...}
Modes: IDLE (motors off) -> TAKEOFF -> HOLD <-> TRACK, BRAKE -> HOLD, LAND -> IDLE.
"""

from __future__ import annotations

import json
import math
import os

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
from trajectory_msgs.msg import MultiDOFJointTrajectory
from webots_ros2_crazyflie.crazyflie_driver import pid_velocity_fixed_height_controller

from autopath3d.trajectories.polynomial import PiecewisePolynomial

from . import config as apcfg
from .rosconv import from_time, header, msg_to_traj
from .tracking import brake_command, outer_loop

LATE_TOLERANCE_S = 0.1     # a trajectory received later than t_start + this is not started
QOS_RELIABLE = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)
QOS_LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class AutoPathCrazyflie:
    def init(self, webots_node, properties):
        self.robot = webots_node.robot
        self.step_ms = int(self.robot.getBasicTimeStep())
        self.motors = []
        for i, sign in zip(range(1, 5), (-1, 1, -1, 1)):
            m = self.robot.getDevice(f"m{i}_motor")
            m.setPosition(float("inf"))
            m.setVelocity(0.0)
            self.motors.append((m, sign))
        self.imu = self.robot.getDevice("inertial_unit")
        self.gps = self.robot.getDevice("gps")
        self.gyro = self.robot.getDevice("gyro")
        for d in (self.imu, self.gps, self.gyro):
            d.enable(self.step_ms)

        self.cfg = apcfg.load(properties.get("config") or os.environ.get("AUTOPATH_CONFIG") or None)
        self.gains = apcfg.outer_gains(self.cfg)
        b = self.cfg["bridge"]
        self.rest_speed, self.rest_time, self.climb = b["rest_speed"], b["rest_time_s"], b["takeoff_climb_rate"]

        if not rclpy.ok():
            rclpy.init(args=None)
        self.node = rclpy.create_node("autopath_crazyflie_bridge")
        self.pub_odom = self.node.create_publisher(Odometry, "/autopath/odom", QOS_RELIABLE)
        self.pub_state = self.node.create_publisher(String, "/autopath/bridge/state", QoSProfile(depth=200))
        self.pub_event = self.node.create_publisher(String, "/autopath/events", QoSProfile(depth=100))
        self.pub_err = self.node.create_publisher(Vector3Stamped, "/autopath/tracking_error", QOS_RELIABLE)
        self.tf = TransformBroadcaster(self.node)
        self.node.create_subscription(MultiDOFJointTrajectory, "/autopath/active_trajectory", self._on_traj, QOS_LATCHED)
        self.node.create_subscription(String, "/autopath/bridge/command", self._on_cmd, QoSProfile(depth=20))

        self.mode = "IDLE"
        self.pid = None
        self.traj: PiecewisePolynomial | None = None
        self.traj_id = -1
        self.hold = None            # (3,) hold reference
        self.takeoff_target = None
        self.ramp_start = None
        self.z_brake = None
        self.rest_since = None
        self.pending = []
        self.yaw_ref = None
        self.t_prev = None
        self.n_steps = 0

    # ---------------------------------------------------------------- inputs
    def _on_traj(self, msg):
        self.pending.append(("traj", msg))

    def _on_cmd(self, msg):
        try:
            self.pending.append(("cmd", json.loads(msg.data)))
        except ValueError:
            self._event("BRIDGE_BAD_COMMAND", {"data": msg.data})

    def _event(self, name, data=None, t=None):
        t = self.robot.getTime() if t is None else t
        self.pub_event.publish(String(data=json.dumps({"t": t, "source": "bridge", "name": name, "data": data or {}})))

    def _apply_pending(self, t, p):
        for kind, item in self.pending:
            if kind == "traj":
                if self.mode not in ("HOLD", "TRACK"):
                    self._event("TRAJECTORY_REJECTED", {"reason": f"mode {self.mode}"}, t)
                    continue
                traj = msg_to_traj(item)
                if t > traj.t_start + LATE_TOLERANCE_S:
                    # starting now would put the reference ahead of the drone: keep the current
                    # reference and report; the supervisor re-times and republishes
                    self._event("TRAJECTORY_LATE", {"t_start": traj.t_start, "lateness_s": t - traj.t_start,
                                                    "stamp": from_time(item.header.stamp)}, t)
                    continue
                self.traj, self.traj_id, self.mode = traj, self.traj_id + 1, "TRACK"
                self._event("TRAJECTORY_ACCEPTED", {"traj_id": self.traj_id, "t_start": traj.t_start,
                                                    "t_end": traj.t_end, "stamp": from_time(item.header.stamp)}, t)
            else:
                c = item.get("cmd")
                if c == "takeoff" and self.mode == "IDLE":
                    self.pid = pid_velocity_fixed_height_controller()
                    self.takeoff_target = np.array([p[0], p[1], float(item.get("height", 1.0))])
                    self.ramp_start = (t, p.copy())
                    self.hold, self.mode = self.takeoff_target.copy(), "TAKEOFF"
                    self._event("TAKEOFF_START", {"target": self.takeoff_target.tolist()}, t)
                elif c == "brake" and self.mode in ("TRACK", "HOLD"):
                    self.mode, self.z_brake, self.rest_since = "BRAKE", float(p[2]), None
                    self._event("BRAKE_START", {"p": p.tolist(), "request_t": item.get("t")}, t)
                elif c == "hold" and self.mode in ("TRACK", "BRAKE"):
                    self.hold, self.mode = p.copy(), "HOLD"
                    self._event("HOLD", {"p": p.tolist()}, t)
                elif c == "land" and self.mode in ("HOLD",):
                    self.ramp_start = (t, p.copy())
                    self.mode = "LAND"
                    self._event("LAND_START", {"p": p.tolist()}, t)
                elif c == "disarm":
                    self.mode = "IDLE"
                    self._event("DISARMED", {}, t)
                else:
                    self._event("COMMAND_IGNORED", {"cmd": item, "mode": self.mode}, t)
        self.pending.clear()

    # ---------------------------------------------------------------- step
    def step(self):
        rclpy.spin_once(self.node, timeout_sec=0)
        t = self.robot.getTime()
        p = np.array(self.gps.getValues(), float)
        v = np.array(self.gps.getSpeedVector(), float)
        rpy = self.imu.getRollPitchYaw()
        q = self.imu.getQuaternion()                      # x, y, z, w (world frame, ENU)
        w_body = self.gyro.getValues()
        if not np.all(np.isfinite(p)):
            return
        roll, pitch, yaw = rpy[0], -rpy[1], rpy[2]        # sign conventions of the Bitcraze driver
        if self.yaw_ref is None:
            self.yaw_ref = yaw
        dt = self.step_ms / 1000.0
        self._apply_pending(t, p)

        p_ref = v_ref = a_ref = None
        cmd = None
        if self.mode == "TAKEOFF" or self.mode == "LAND":
            t0, p0 = self.ramp_start
            z_goal = self.takeoff_target[2] if self.mode == "TAKEOFF" else 0.0
            direction = 1.0 if z_goal >= p0[2] else -1.0
            z_r = p0[2] + direction * min(self.climb * (t - t0), abs(z_goal - p0[2]))
            p_ref = np.array([p0[0], p0[1], z_r])
            moving = abs(z_r - z_goal) > 1e-9
            v_ref = np.array([0.0, 0.0, direction * self.climb if moving else 0.0])
            a_ref = np.zeros(3)
            cmd = outer_loop(p_ref, v_ref, a_ref, p, yaw, self.gains, self.yaw_ref)
            if self.mode == "TAKEOFF" and not moving and abs(p[2] - z_goal) < 0.05 and np.linalg.norm(v) < 0.1:
                self.hold, self.mode = p_ref.copy(), "HOLD"
                self._event("TAKEOFF_COMPLETE", {"p": p.tolist()}, t)
            if self.mode == "LAND" and p[2] < 0.03 and np.linalg.norm(v) < 0.05:
                self.mode = "IDLE"
                self._event("LANDED", {"p": p.tolist()}, t)
        elif self.mode == "TRACK":
            tr = self.traj
            p_ref, v_ref, a_ref = tr.evaluate(t, 0)[0], tr.evaluate(t, 1)[0], tr.evaluate(t, 2)[0]
            if t < tr.t_start:
                v_ref = a_ref = np.zeros(3)
            cmd = outer_loop(p_ref, v_ref, a_ref, p, yaw, self.gains, self.yaw_ref)
            if t > tr.t_end:
                self.hold, self.mode = (tr.hold_point if tr.n_pieces == 0 else tr.evaluate(tr.t_end)[0]).copy(), "HOLD"
                self._event("TRAJECTORY_COMPLETE", {"traj_id": self.traj_id, "p": p.tolist()}, t)
        elif self.mode == "BRAKE":
            cmd = brake_command(p, self.z_brake, yaw, self.gains, self.yaw_ref)
            if np.linalg.norm(v) < self.rest_speed:
                self.rest_since = t if self.rest_since is None else self.rest_since
                if t - self.rest_since >= self.rest_time:
                    self.hold, self.mode = p.copy(), "HOLD"
                    self._event("AT_REST", {"p": p.tolist(), "v": v.tolist()}, t)
            else:
                self.rest_since = None
        elif self.mode == "HOLD":
            p_ref, v_ref, a_ref = self.hold, np.zeros(3), np.zeros(3)
            cmd = outer_loop(p_ref, v_ref, a_ref, p, yaw, self.gains, self.yaw_ref)

        if cmd is None:                                   # IDLE: motors off
            for m, _ in self.motors:
                m.setVelocity(0.0)
        else:
            c, s = math.cos(yaw), math.sin(yaw)
            vx_b, vy_b = c * v[0] + s * v[1], -s * v[0] + c * v[1]
            power = self.pid.pid(dt, cmd.vx_body, cmd.vy_body, cmd.yaw_rate, cmd.z_setpoint,
                                 roll, pitch, w_body[2], p[2], vx_b, vy_b)
            for (m, sign), w in zip(self.motors, power):
                m.setVelocity(sign * float(w))
        self._publish(t, p, v, q, w_body, yaw, p_ref, v_ref, a_ref, cmd)
        self.n_steps += 1

    # ---------------------------------------------------------------- outputs
    def _publish(self, t, p, v, q, w_body, yaw, p_ref, v_ref, a_ref, cmd):
        h = header(t, "map")
        od = Odometry(header=h, child_frame_id="base_link")
        od.pose.pose.position.x, od.pose.pose.position.y, od.pose.pose.position.z = (float(x) for x in p)
        o = od.pose.pose.orientation
        o.x, o.y, o.z, o.w = (float(x) for x in q)
        R = _quat_matrix(q)
        vb = R.T @ v
        od.twist.twist.linear.x, od.twist.twist.linear.y, od.twist.twist.linear.z = (float(x) for x in vb)
        od.twist.twist.angular.x, od.twist.twist.angular.y, od.twist.twist.angular.z = (float(x) for x in w_body)
        self.pub_odom.publish(od)
        tf = TransformStamped(header=h, child_frame_id="base_link")
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = (float(x) for x in p)
        tf.transform.rotation = od.pose.pose.orientation
        self.tf.sendTransform(tf)
        st = {"t": t, "k": self.n_steps, "mode": self.mode, "traj_id": self.traj_id, "p": p.tolist(), "v": v.tolist(),
              "yaw": yaw}
        if p_ref is not None:
            err = np.asarray(p_ref) - p
            st.update(p_ref=np.asarray(p_ref).tolist(), v_ref=np.asarray(v_ref).tolist(),
                      a_ref=np.asarray(a_ref).tolist(), err=err.tolist())
            e = Vector3Stamped(header=h)
            e.vector.x, e.vector.y, e.vector.z = (float(x) for x in err)
            self.pub_err.publish(e)
        if cmd is not None:
            st["cmd"] = [cmd.vx_body, cmd.vy_body, cmd.z_setpoint, cmd.yaw_rate]
            st["sat"] = cmd.saturated
        self.pub_state.publish(String(data=json.dumps(st)))


def _quat_matrix(q):
    x, y, z, w = (float(c) for c in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])

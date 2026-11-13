"""Trajectory node: validated path -> existing AutoPath-3D shortcut + quintic trajectory + validation.

in   /autopath/global_path            nav_msgs/Path (from the planner node)
out  /autopath/trajectory_candidate   trajectory_msgs/MultiDOFJointTrajectory, t0 = 0 (relative);
                                      the supervisor shifts it to the activation time
out  /autopath/trajectory/result      std_msgs/String JSON: status, shortcut/generation summaries,
                                      certified speed/acceleration bounds, independent validation
Infeasible or unvalidated trajectories are rejected (no candidate is published). Uses
autopath3d.pipeline.run_pipeline(path=...) unchanged: collision-checked shortcutting, piecewise
quintic generation with certified limits/clearance, independent continuous-time validation.
"""

from __future__ import annotations

import threading
import time

import rclpy
from nav_msgs.msg import Path
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from trajectory_msgs.msg import MultiDOFJointTrajectory

from autopath3d.pipeline import run_pipeline, trajectory_checker

from . import config as apcfg
from .common import QOS_EVENTS, AutoPathNode, MapClient, spin_node
from .rosconv import json_msg, msg_to_path, traj_to_msg


class TrajectoryNode(AutoPathNode):
    def __init__(self):
        super().__init__("autopath_trajectory")
        self.policy, self.limits = apcfg.policy(self.cfg), apcfg.limits(self.cfg)
        self.map = MapClient(self, MutuallyExclusiveCallbackGroup())
        self._ck = (None, None)
        self._lock = threading.Lock()
        self.pub_traj = self.create_publisher(MultiDOFJointTrajectory, "/autopath/trajectory_candidate", QOS_EVENTS)
        self.pub_result = self.create_publisher(String, "/autopath/trajectory/result", QOS_EVENTS)
        self.create_subscription(Path, "/autopath/global_path", self._on_path, QOS_EVENTS,
                                 callback_group=MutuallyExclusiveCallbackGroup())
        self.n = 0

    def _checker(self):
        with self.map.lock:
            ver, scene = self.map.version, self.map.scene
        with self._lock:
            if self._ck[0] != ver and scene is not None:
                _, ck = trajectory_checker(scene, self.cfg["map"]["resolution"], self.policy,
                                           self.cfg["map"]["clearance_cap"])
                self._ck = (ver, ck)
            return scene, self._ck[1]

    def _on_path(self, msg):
        self.n += 1
        path = msg_to_path(msg)
        scene, ck = self._checker()
        out = {"request": self.n, "t": self.now_s(), "path_waypoints": len(path)}
        if ck is None or len(path) < 2:
            out.update(status="rejected", reason="no map" if ck is None else "path too short")
            self.pub_result.publish(json_msg(out))
            return
        tic = time.perf_counter()
        res = run_pipeline(scene, ck, path[0], path[-1], {"planner": "given_path"}, self.limits, self.policy,
                           path=path)
        out.update(status=res.status, reason=res.reason, wall_s=time.perf_counter() - tic, record=res.record)
        if res.status == "success":
            tr = res.trajectory
            out.update(duration_s=tr.duration, pieces=tr.n_pieces)
            self.pub_traj.publish(traj_to_msg(tr))
        self.pub_result.publish(json_msg(out))
        gen = res.record.get("generation", {})
        self.event("TRAJECTORY_RESULT", {"request": self.n, "status": res.status, "reason": res.reason,
                                         "duration_s": gen.get("duration"),
                                         "generation_time_s": gen.get("generation_time_s")})


def main():
    rclpy.init()
    spin_node(TrajectoryNode(), MultiThreadedExecutor(num_threads=3))

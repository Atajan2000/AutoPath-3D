"""Global planning node: the existing AutoPath-3D A* / RRT* behind a standard ROS 2 service.

srv  /autopath/plan/astar, /autopath/plan/rrt_star   nav_msgs/srv/GetPlan (start, goal in `map`)
     -> response.plan: the validated geometric path (empty on failure)
out  /autopath/global_path       nav_msgs/Path (on success)
out  /autopath/planner/result    std_msgs/String JSON: status, reason, timings, path length,
                                 independent validation (ground truth + voxel map)
in   /autopath/map/obstacles     planning geometry (latched)
Planner and checker: autopath3d.pipeline.trajectory_checker / make_planner, at the trajectory
clearance (robot radius + margin + tracking allowance). Planning runs in its own callback group
so map updates are still received; a long plan never blocks the drone bridge (other process).
"""

from __future__ import annotations

import threading
import time

import numpy as np
import rclpy
from nav_msgs.msg import Path
from nav_msgs.srv import GetPlan
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String

from autopath3d.evaluation.validation import validate_path
from autopath3d.pipeline import make_planner, trajectory_checker

from . import config as apcfg
from .common import QOS_EVENTS, AutoPathNode, MapClient, spin_node
from .rosconv import json_msg, path_to_msg


class PlannerNode(AutoPathNode):
    def __init__(self):
        super().__init__("autopath_planner")
        self.policy = apcfg.policy(self.cfg)
        self._ck_lock = threading.Lock()
        self._ck = (None, None, None)          # (map version, grid, checker)
        self.map = MapClient(self, MutuallyExclusiveCallbackGroup())
        self.pub_path = self.create_publisher(Path, "/autopath/global_path", QOS_EVENTS)
        self.pub_result = self.create_publisher(String, "/autopath/planner/result", QOS_EVENTS)
        grp = MutuallyExclusiveCallbackGroup()
        for name in ("astar", "rrt_star"):
            self.create_service(GetPlan, f"/autopath/plan/{name}",
                                lambda req, res, n=name: self._plan(n, req, res), callback_group=grp)
        self.n_requests = 0

    def checker(self):
        with self.map.lock:
            ver, scene = self.map.version, self.map.scene
        if scene is None:
            return None, None, None
        with self._ck_lock:
            if self._ck[0] != ver:
                tic = time.perf_counter()
                grid, ck = trajectory_checker(scene, self.cfg["map"]["resolution"], self.policy,
                                              self.cfg["map"]["clearance_cap"])
                self._ck = (ver, grid, ck)
                self.event("CHECKER_BUILT", {"map_version": ver, "build_s": time.perf_counter() - tic})
            return scene, self._ck[1], self._ck[2]

    def _plan(self, name, req, res):
        self.n_requests += 1
        rid = self.n_requests
        start = np.array([req.start.pose.position.x, req.start.pose.position.y, req.start.pose.position.z])
        goal = np.array([req.goal.pose.position.x, req.goal.pose.position.y, req.goal.pose.position.z])
        scene, grid, ck = self.checker()
        out = {"request": rid, "planner": name, "start": start.tolist(), "goal": goal.tolist(), "t": self.now_s()}
        if ck is None:
            out.update(status="no_map", reason="map not received yet")
            self.pub_result.publish(json_msg(out))
            return res
        spec = dict(self.cfg["planners"][name])
        tic = time.perf_counter()
        pr = make_planner(ck, spec).plan(start, goal)
        wall = time.perf_counter() - tic
        out.update(status=pr.status.value, reason=pr.reason, planning_time_s=pr.planning_time_s,
                   setup_time_s=pr.setup_time_s, wall_s=wall, length_m=pr.length,
                   waypoints=0 if pr.path is None else len(pr.path), stats=pr.stats)
        if pr.success:
            val = validate_path(scene, grid, pr.path, start, goal, self.policy.trajectory)
            out["validation"] = val.to_dict()
            if val.valid:
                msg = path_to_msg(pr.path, self.now_s())
                res.plan = msg
                self.pub_path.publish(msg)
            else:
                out.update(status="validation_failed", reason=f"independent validation: {val.to_dict()}")
        self.pub_result.publish(json_msg(out))
        self.event("PLAN_RESULT", {k: out.get(k) for k in ("request", "planner", "status", "reason",
                                                         "planning_time_s", "length_m")})
        return res


def main():
    rclpy.init()
    spin_node(PlannerNode(), MultiThreadedExecutor(num_threads=3))

"""Scenario runner + recorder: drives one mission through ROS 2 and writes its measured results.

Sequence (all through topics; nothing is moved except the initial drone placement on the floor):
  wait for map/bridge/supervisor -> place drone on its start pad (scene manager, initial condition)
  -> takeoff (bridge) -> mission request (supervisor) -> scenario event when the measured state
  meets the trigger condition (scene manager) -> wait for a terminal supervisor mode -> hold 3 s
  -> metrics -> <results_dir>/<scenario>/run_<stamp>.json -> exit.
"""

from __future__ import annotations

import json
import os
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String
from trajectory_msgs.msg import MultiDOFJointTrajectory
from vision_msgs.msg import Detection3DArray

from .common import QOS_EVENTS, QOS_LATCHED, AutoPathNode
from .geometry import WorldGeometry, to_primitive
from .metrics import clearance_series, contact_events, realtime_factor, speed_accel, tracking_stats
from .rosconv import from_time, msg_to_records, msg_to_traj

TERMINAL = ("GOAL_REACHED", "HOLDING_ABORTED", "MISSION_FAILED")


class MissionNode(AutoPathNode):
    def __init__(self):
        super().__init__("autopath_mission")
        self.declare_parameter("scenario", "A_navigation")
        self.declare_parameter("results_dir", os.path.expanduser("~/autopath_results"))
        self.declare_parameter("timeout_s", 300.0)
        self.declare_parameter("screenshot", "")
        self.name = self.get_parameter("scenario").value
        self.mcfg = self.cfg["missions"][self.name]
        self.out_dir = os.path.join(self.get_parameter("results_dir").value, self.name)
        self.timeout = float(self.get_parameter("timeout_s").value)
        self.rec = {k: [] for k in ("bridge", "gt", "contacts", "events", "sup_state", "planner", "trajectory",
                                    "active", "updates")}
        self.map_info = self.geometry = self.report = self.sup_report = None
        self.pub_scene = self.create_publisher(String, "/autopath/scene/request", QOS_EVENTS)
        self.pub_cmd = self.create_publisher(String, "/autopath/bridge/command", QOS_EVENTS)
        self.pub_req = self.create_publisher(String, "/autopath/mission/request", QOS_EVENTS)
        s = self.create_subscription
        s(String, "/autopath/bridge/state", lambda m: self.rec["bridge"].append(json.loads(m.data)), 200)
        s(Odometry, "/autopath/ground_truth/odom", self._on_gt, 200)
        s(String, "/autopath/ground_truth/contacts", lambda m: self.rec["contacts"].append(json.loads(m.data)), QOS_EVENTS)
        s(String, "/autopath/events", lambda m: self.rec["events"].append(json.loads(m.data)), QOS_EVENTS)
        s(String, "/autopath/supervisor/state", self._on_sup, QOS_EVENTS)
        s(String, "/autopath/planner/result", lambda m: self.rec["planner"].append(json.loads(m.data)), QOS_EVENTS)
        s(String, "/autopath/trajectory/result", lambda m: self.rec["trajectory"].append(json.loads(m.data)), QOS_EVENTS)
        s(MultiDOFJointTrajectory, "/autopath/active_trajectory", self._on_active, QOS_LATCHED)
        s(Detection3DArray, "/autopath/obstacle_updates", lambda m: self.rec["updates"].append(
            {"t": from_time(m.header.stamp), "records": msg_to_records(m)}), QOS_EVENTS)
        s(String, "/autopath/map/info", lambda m: setattr(self, "map_info", json.loads(m.data)), QOS_LATCHED)
        s(Detection3DArray, "/autopath/world/collision_geometry",
          lambda m: setattr(self, "geometry", msg_to_records(m)), QOS_LATCHED)
        s(String, "/autopath/world/extraction_report", lambda m: setattr(self, "report", json.loads(m.data)), QOS_LATCHED)
        s(String, "/autopath/supervisor/report", lambda m: setattr(self, "sup_report", json.loads(m.data)), QOS_LATCHED)
        self.sup_mode = None
        self.gt_last = None

    def _on_gt(self, m):
        t = from_time(m.header.stamp)
        p = [m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z]
        v = [m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.linear.z]
        self.rec["gt"].append((t, p, v))
        self.gt_last = (t, np.array(p), np.array(v))

    def _on_sup(self, m):
        d = json.loads(m.data)
        self.rec["sup_state"].append(d)
        self.sup_mode = d["mode"]

    def _on_active(self, m):
        tr = msg_to_traj(m)
        self.rec["active"].append({"t_start": tr.t_start, "t_end": tr.t_end, "knots": tr.knots.tolist(),
                                   "coeffs": tr.coeffs.tolist()})

    # ---------------------------------------------------------------- helpers
    def spin_until(self, ex, cond, timeout, what):
        t0 = time.monotonic()
        while not cond():
            ex.spin_once(timeout_sec=0.05)
            if time.monotonic() - t0 > timeout:
                raise TimeoutError(f"timeout waiting for {what}")

    def has_event(self, name, source=None, after=-1.0):
        return any(e["name"] == name and (source is None or e["source"] == source) and e["t"] >= after
                   for e in self.rec["events"])

    def send(self, pub, obj):
        pub.publish(String(data=json.dumps(obj)))

    # ---------------------------------------------------------------- run
    def run(self, ex):
        wall0, m = time.time(), self.mcfg
        self.spin_until(ex, lambda: self.map_info and self.rec["bridge"] and self.sup_mode == "READY", 180,
                        "map, bridge and supervisor")
        self.wait_matched(ex, [self.pub_scene, self.pub_cmd, self.pub_req])
        start = m["start"]
        p0 = self.gt_last[1]
        if np.linalg.norm(p0[:2] - np.asarray(start)) > 0.05:
            self.send(self.pub_scene, {"name": "place_drone", "xy": start})
            self.spin_until(ex, lambda: self.has_event("DRONE_PLACED") or self.has_event("PLACE_REFUSED"), 20,
                            "drone placement")
            if self.has_event("PLACE_REFUSED"):
                raise RuntimeError("drone placement refused")
            t_pl = self.gt_last[0]
            self.spin_until(ex, lambda: self.gt_last[0] > t_pl + 0.5, 20, "settle")
        t_takeoff_cmd = self.gt_last[0]
        self.send(self.pub_cmd, {"cmd": "takeoff", "height": m["takeoff_height"]})
        self.spin_until(ex, lambda: self.has_event("TAKEOFF_COMPLETE", "bridge"), 60, "takeoff")
        t_tk = self.gt_last[0]
        self.spin_until(ex, lambda: self.gt_last[0] > t_tk + 1.0, 20, "hover settle")
        sim0 = self.gt_last[0]
        wall_m0 = time.time()
        self.send(self.pub_req, {"goal": m["goal"], "planner": m.get("planner", "astar")})
        ev, fired = m.get("event"), False
        self.spin_until(ex, lambda: self.sup_mode not in ("READY",), 30, "mission start")
        while self.sup_mode not in TERMINAL:
            ex.spin_once(timeout_sec=0.02)
            if ev and not fired and self.sup_mode in ("FOLLOWING",):
                fired = self._maybe_trigger(ev)
            if self.gt_last[0] - sim0 > self.timeout:
                break
        t_end_mode = self.gt_last[0]
        self.spin_until(ex, lambda: self.gt_last[0] > t_end_mode + 3.0, 30, "post-mission hold")
        shot = self.get_parameter("screenshot").value
        if shot:      # one genuine Webots frame (the path is on the Webots host, i.e. Windows)
            self.send(self.pub_scene, {"name": "screenshot", "path": shot, "quality": 92})
            try:
                self.spin_until(ex, lambda: self.has_event("SCREENSHOT"), 15, "screenshot")
            except TimeoutError as exc:
                self.get_logger().warning(str(exc))
        self.spin_until(ex, lambda: self.sup_report is not None or self.sup_mode not in TERMINAL, 5, "report")
        return self._write(sim0, (wall_m0, time.time()), wall0, t_takeoff_cmd, t_tk, fired)

    def _maybe_trigger(self, ev):
        t, p, v = self.gt_last
        if "when_y_above" in ev and p[1] < ev["when_y_above"]:
            return False
        if "when_x_above" in ev and p[0] < ev["when_x_above"]:
            return False
        req = {"name": ev["name"]}
        if ev["name"] == "drop_load":
            sp = np.linalg.norm(v[:2])
            u = v[:2] / sp if sp > 1e-6 else np.array([1.0, 0.0])
            # obstacle face `ahead_m` in front of the drone centre (0.4 m = half the load footprint)
            req["xy"] = (p[:2] + (ev["ahead_m"] + 0.4) * u).tolist()
        self.send(self.pub_scene, req)
        self.event("SCENARIO_EVENT_TRIGGERED", {"event": req, "p": p.tolist(), "v": v.tolist()}, t=t)
        return True

    def _write(self, sim0, wall_m, wall0, t_takeoff_cmd, t_tk, fired):
        b = self.cfg["bounds"]
        base = WorldGeometry(np.asarray(b["lo"]), np.asarray(b["hi"]), self.geometry).scene()
        upd = []
        for u in self.rec["updates"]:
            prims = [to_primitive(r) for r in u["records"]]
            t_app = u["t"] - self.cfg["supervisor"]["detection_latency_s"]
            upd.append((t_app, lambda q, prims=prims: np.min([pr.distance(q) for pr in prims], axis=0)))
        G = self.rec["gt"]
        tg = np.array([g[0] for g in G]); pg = np.array([g[1] for g in G]); vg = np.array([g[2] for g in G])
        flight = tg >= sim0
        clr = clearance_series(pg[flight], tg[flight], base.distance, upd)
        br = [s for s in self.rec["bridge"] if s["t"] >= sim0 and s.get("mode") == "TRACK" and "err" in s]
        err = np.array([s["err"] for s in br]).reshape(-1, 3)
        rep = self.sup_report or {}
        goal = np.asarray(self.mcfg["goal"], float)
        coll = contact_events([c for c in self.rec["contacts"] if c["t"] >= t_takeoff_cmd])
        plan = [r for r in self.rec["planner"]]
        traj = [r for r in self.rec["trajectory"]]
        res = {
            "scenario": self.name, "description": self.mcfg.get("description"), "mission": self.mcfg,
            "end_mode": rep.get("end_mode", self.sup_mode), "end_detail": rep.get("end_detail"),
            "event_fired": fired,
            "goal_reached": rep.get("end_mode") == "GOAL_REACHED" and coll["collision_events"] == 0,
            "final_position": pg[-1].tolist(), "final_goal_error_m": float(np.linalg.norm(pg[-1] - goal)),
            "flight_duration_s": float(rep.get("t_end", tg[-1]) - sim0),
            "takeoff_duration_s": t_tk - t_takeoff_cmd,
            "tracking_error_m": tracking_stats(err),
            "motion": speed_accel(tg[flight], vg[flight]),
            "min_clearance_m": float(clr.min()) if len(clr) else None,
            "min_clearance_position": pg[flight][int(np.argmin(clr))].tolist() if len(clr) else None,
            "clearance_reference": "drone centre to extracted collision geometry (AABB, lower bound)",
            "robot_envelope_radius_m": self.cfg["robot"]["radius"],
            "physical_collision_radius_m": 0.05,
            **coll,
            "planning": [{k: r.get(k) for k in ("planner", "status", "reason", "planning_time_s", "wall_s",
                                                 "length_m", "waypoints", "validation")} for r in plan],
            "trajectory": [{k: r.get(k) for k in ("status", "reason", "wall_s", "duration_s")} |
                           {"generation": (r.get("record") or {}).get("generation"),
                            "validation": (r.get("record") or {}).get("validation")} for r in traj],
            "supervisor_report": rep,
            "realtime_factor_mission": realtime_factor((sim0, float(tg[-1])), wall_m),
            "wall_total_s": time.time() - wall0,
            "map_info": self.map_info,
            "events": self.rec["events"],
        }
        os.makedirs(self.out_dir, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        path = os.path.join(self.out_dir, f"run_{stamp}.json")
        with open(path, "w") as f:
            json.dump(res, f, indent=1, default=str)
        np.savez_compressed(os.path.join(self.out_dir, f"run_{stamp}_series.npz"), t=tg, p=pg, v=vg,
                            clearance_t=tg[flight], clearance=clr,
                            bridge_t=np.array([s["t"] for s in self.rec["bridge"]]),
                            bridge_p=np.array([s["p"] for s in self.rec["bridge"]]),
                            bridge_mode=np.array([s["mode"] for s in self.rec["bridge"]]),
                            bridge_ref=np.array([s.get("p_ref", [np.nan] * 3) for s in self.rec["bridge"]]))
        self.get_logger().info(f"RESULT {self.name}: {res['end_mode']} goal_reached={res['goal_reached']} "
                               f"collisions={res['collision_events']} min_clearance={res['min_clearance_m']} "
                               f"tracking_rms={res['tracking_error_m'].get('rms')} -> {path}")
        return res


def main():
    rclpy.init()
    node = MissionNode()
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    code = 0
    try:
        node.run(ex)
    except Exception as exc:
        node.get_logger().error(f"mission runner failed: {exc!r}")
        code = 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    raise SystemExit(code)

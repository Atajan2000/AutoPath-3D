"""Mission executive and replanning supervisor (ROS 2 node around the existing Phase 2 supervisor).

in   /autopath/mission/request     String JSON {"goal": [x,y,z], "planner": "astar"|"rrt_star"}
in   /autopath/odom                nav_msgs/Odometry (bridge, measured)
in   /autopath/events              bridge events (AT_REST, TRAJECTORY_ACCEPTED, ...)
in   /autopath/obstacle_updates    vision_msgs/Detection3DArray (stamp = t_detect)
in   /autopath/trajectory_candidate, /autopath/trajectory/result, /autopath/map/obstacles
srv  /autopath/plan/{astar,rrt_star} (client)
out  /autopath/active_trajectory   MultiDOFJointTrajectory (latched): the only reference the bridge follows
out  /autopath/bridge/command      String JSON (brake)
out  /autopath/replanned_path      nav_msgs/Path (drawing)
out  /autopath/supervisor/state    String JSON {"mode", "detail"} (on change + 2 Hz)
out  /autopath/supervisor/report   String JSON (latched) at the end of a mission

Modes: WAIT_MAP -> READY -> PLANNING -> WAIT_TRAJECTORY -> FOLLOWING -> GOAL_REACHED
       obstacle update in conflict -> BRAKING -> (AT_REST, verified) -> REPLANNING -> FOLLOWING_REPLAN
       terminal failures: HOLDING_ABORTED (no route / unsafe stop / unsafe hold), MISSION_FAILED
Safety decisions use measured state and simulation time only. Two assessments are made on an
update, both from the measured state: the unchanged Phase 2 `assess_update` (nominal prediction)
and the Phase 4 conservative stopping assessment (assumption bounds from the config).
Replanning runs the unchanged Phase 2 `replan_from_rest` from the *measured* hold point in a worker
thread; the drone keeps holding (bridge) and odometry/safety callbacks keep running meanwhile.
"""

from __future__ import annotations

import json
import queue
import threading
import time
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from nav_msgs.srv import GetPlan
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from trajectory_msgs.msg import MultiDOFJointTrajectory
from vision_msgs.msg import Detection3DArray

from autopath3d.replanning.stopping_assessment import assess_stopping
from autopath3d.replanning.supervisor import (MotionState, ObstacleUpdate, Outcome, ReplanningSupervisor,
                                              updated_scene)

from . import config as apcfg
from .common import QOS_EVENTS, QOS_LATCHED, QOS_RELIABLE, AutoPathNode, MapClient, spin_node
from .geometry import conflict_check, to_primitive
from .rosconv import from_time, json_msg, msg_to_records, msg_to_traj, path_to_msg, traj_to_msg
from .trajcodec import shifted

TERMINAL = ("GOAL_REACHED", "HOLDING_ABORTED", "MISSION_FAILED")
ACTIVATION_DELAY_S = 0.5      # sim seconds between accepting a trajectory and its start
MAX_LATE_RETRIES = 3          # bridge reported TRAJECTORY_LATE -> re-time and republish at most this often


def _quat_rot(q, v):
    x, y, z, w = q
    R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                  [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                  [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    return R @ v


class SupervisorNode(AutoPathNode):
    def __init__(self):
        super().__init__("autopath_supervisor")
        self.policy, self.limits = apcfg.policy(self.cfg), apcfg.limits(self.cfg)
        self.sup_cfg, self.assume = apcfg.supervisor_config(self.cfg), apcfg.stopping_assumptions(self.cfg)
        self.det_latency = self.sup_cfg.detection_latency_s
        cb = ReentrantCallbackGroup()
        self.lock = threading.RLock()
        self.map = MapClient(self, cb)
        self.pub_active = self.create_publisher(MultiDOFJointTrajectory, "/autopath/active_trajectory", QOS_LATCHED)
        self.pub_cmd = self.create_publisher(String, "/autopath/bridge/command", QoSProfile_cmd())
        self.pub_state = self.create_publisher(String, "/autopath/supervisor/state", QOS_EVENTS)
        self.pub_report = self.create_publisher(String, "/autopath/supervisor/report", QOS_LATCHED)
        self.pub_rpath = self.create_publisher(Path, "/autopath/replanned_path", QOS_EVENTS)
        self.cli = {n: self.create_client(GetPlan, f"/autopath/plan/{n}", callback_group=cb) for n in ("astar", "rrt_star")}
        self.create_subscription(String, "/autopath/mission/request", self._on_request, QOS_EVENTS, callback_group=cb)
        self.create_subscription(Odometry, "/autopath/odom", self._on_odom, QOS_RELIABLE, callback_group=cb)
        self.create_subscription(String, "/autopath/events", self._on_event, QOS_EVENTS, callback_group=cb)
        self.create_subscription(Detection3DArray, "/autopath/obstacle_updates", self._on_update, QOS_EVENTS,
                                 callback_group=cb)
        self.create_subscription(MultiDOFJointTrajectory, "/autopath/trajectory_candidate", self._on_candidate,
                                 QOS_EVENTS, callback_group=cb)
        self.create_subscription(String, "/autopath/trajectory/result", self._on_traj_result, QOS_EVENTS,
                                 callback_group=cb)
        self.create_timer(0.05, self._tick, callback_group=cb)
        self.done = queue.Queue()
        self.mode, self.detail = "WAIT_MAP", ""
        self.odom = None                    # (t, p, v_world)
        self.hist = deque(maxlen=20000)     # measured (t, p, v) history
        self.mission = None
        self.sup: ReplanningSupervisor | None = None
        self.sup_version = None
        self._last_state_pub = 0.0

    # ---------------------------------------------------------------- state
    def set_mode(self, mode, detail=""):
        with self.lock:
            changed = (mode, detail) != (self.mode, self.detail)
            self.mode, self.detail = mode, detail
        if changed:
            self.event("SUPERVISOR_MODE", {"mode": mode, "detail": detail})
            self._publish_state()

    def _publish_state(self):
        self.pub_state.publish(json_msg({"t": self.now_s(), "mode": self.mode, "detail": self.detail}))

    def _finish(self, mode, detail, **extra):
        self.mission.update(extra, end_mode=mode, end_detail=detail, t_end=self.now_s())
        self.set_mode(mode, detail)
        self.pub_report.publish(json_msg(self.mission))

    # ---------------------------------------------------------------- inputs
    def _on_odom(self, msg):
        t = from_time(msg.header.stamp)
        p = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
        q = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z,
             msg.pose.pose.orientation.w)
        vb = np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z])
        v = _quat_rot(q, vb)
        self.odom = (t, p, v)
        self.hist.append((t, p, v))

    def _on_event(self, msg):
        try:
            e = json.loads(msg.data)
        except ValueError:
            return
        if e.get("source") != "bridge" or self.mission is None:
            return
        if e["name"] == "AT_REST" and self.mode == "BRAKING":
            self._after_rest(np.array(e["data"]["p"]), e["t"])
        elif e["name"] in ("TRAJECTORY_ACCEPTED", "TRAJECTORY_COMPLETE"):
            self.mission.setdefault("bridge_events", []).append(e)
        elif e["name"] == "TRAJECTORY_LATE" and self.mode in ("FOLLOWING", "FOLLOWING_REPLAN"):
            self.mission.setdefault("bridge_events", []).append(e)
            if abs(e["data"]["t_start"] - self.active.t_start) > 1e-6:
                return                       # an older activation; the current one is still in flight
            n = self.mission["late_retries"] = self.mission.get("late_retries", 0) + 1
            if n > MAX_LATE_RETRIES:
                return self._finish("MISSION_FAILED", f"trajectory delivered late {n} times")
            t, p, _ = self.odom
            if np.linalg.norm(self.active.evaluate(self.active.t_start)[0] - p) > 0.3:
                return self._finish("MISSION_FAILED", "re-timing refused: drone moved from the trajectory start")
            self._activate(shifted(self.active, t + ACTIVATION_DELAY_S - self.active.t_start), self.mode)

    def _on_request(self, msg):
        req = json.loads(msg.data)
        if self.mode not in ("READY",) + TERMINAL:
            self.event("MISSION_REJECTED", {"reason": f"mode {self.mode}", "request": req})
            return
        if self.odom is None:
            self.event("MISSION_REJECTED", {"reason": "no odometry"})
            return
        with self.map.lock:
            scene, ver = self.map.scene, self.map.version
        t, p, v = self.odom
        self.mission = {"request": req, "t_request": t, "start": p.tolist(), "goal": req["goal"],
                        "planner": req.get("planner", "astar"), "map_version": ver, "updates": []}
        self.base_scene = scene
        self.goal = np.asarray(req["goal"], float)
        # Phase 2 supervisor on the base map (built in the background; needed only for updates)
        if self.sup_version != ver:
            self.sup = None
            threading.Thread(target=self._build_supervisor, args=(scene, ver), daemon=True).start()
        self.set_mode("PLANNING", self.mission["planner"])
        cli = self.cli[self.mission["planner"]]
        if not cli.wait_for_service(timeout_sec=5.0):
            return self._finish("MISSION_FAILED", "planner service unavailable")
        r = GetPlan.Request()
        r.start = _pose(p, t)
        r.goal = _pose(self.goal, t)
        self.mission["t_plan_call"] = self.now_s()
        self._wall_plan = time.perf_counter()
        cli.call_async(r).add_done_callback(self._on_plan)

    def _build_supervisor(self, scene, ver):
        tic = time.perf_counter()
        sup = ReplanningSupervisor(scene, self.policy, self.limits, self.sup_cfg)
        with self.lock:
            self.sup, self.sup_version = sup, ver
        self.event("SUPERVISOR_READY", {"map_version": ver, "build_s": time.perf_counter() - tic})

    def _on_plan(self, fut):
        res = fut.result()
        self.mission["plan_service_wall_s"] = time.perf_counter() - self._wall_plan
        if res is None or len(res.plan.poses) < 2:
            return self._finish("MISSION_FAILED", "planning failed (see /autopath/planner/result)")
        self.mission["path_waypoints"] = len(res.plan.poses)
        self.set_mode("WAIT_TRAJECTORY")

    def _on_traj_result(self, msg):
        r = json.loads(msg.data)
        if self.mode != "WAIT_TRAJECTORY":
            return
        self.mission["trajectory_result"] = {k: r.get(k) for k in ("status", "reason", "wall_s", "duration_s")}
        if r.get("status") != "success":
            self._finish("MISSION_FAILED", f"trajectory rejected: {r.get('status')} {r.get('reason')}")

    def _on_candidate(self, msg):
        if self.mode != "WAIT_TRAJECTORY":
            return
        tr = msg_to_traj(msg)
        t, p, _ = self.odom
        if np.linalg.norm(tr.evaluate(tr.t_start)[0] - p) > 0.3:
            return self._finish("MISSION_FAILED", "trajectory start too far from the measured position")
        self._activate(shifted(tr, t + ACTIVATION_DELAY_S - tr.t_start), "FOLLOWING")

    def _activate(self, tr, mode):
        self.active = tr
        self.pub_active.publish(traj_to_msg(tr))
        self.mission.setdefault("activations", []).append({"t": self.now_s(), "t_start": tr.t_start,
                                                          "t_end": tr.t_end, "mode": mode})
        self.set_mode(mode)

    # ---------------------------------------------------------------- obstacle updates
    def _on_update(self, msg):
        recs = msg_to_records(msg)
        if not recs or self.mission is None:
            return
        t_detect = from_time(msg.header.stamp)
        name = recs[0]["object"]
        upd = ObstacleUpdate(name, t_detect - self.det_latency,
                             [to_primitive(r, f"{name}:{i}") for i, r in enumerate(recs)])
        t_recv, p, v = self.odom
        rec = {"name": name, "t_appear": upd.t_appear, "t_detect": t_detect, "t_received": t_recv,
               "state": {"p": p.tolist(), "v": v.tolist(), "speed": float(np.linalg.norm(v))},
               "mode_at_update": self.mode}
        self.mission["updates"].append(rec)
        self.update = upd
        new_scene = updated_scene(self.base_scene, upd)
        self.new_scene = new_scene
        if self.mode in ("FOLLOWING", "FOLLOWING_REPLAN"):
            # 1) brake decision = Phase 2 step 1 (continuous-time certificate of the remaining
            #    trajectory against the new obstacle at the trajectory clearance), evaluated first so
            #    the brake is not delayed by the (slower) stop prediction computed afterwards
            tic = time.perf_counter()
            conflict = conflict_check(self.active, upd, t_recv, self.policy.trajectory)
            rec["conflict_check_wall_s"] = time.perf_counter() - tic
            if conflict:
                self.pub_cmd.publish(String(data=json.dumps({"cmd": "brake", "t": self.now_s()})))
                rec["t_brake_command"] = self.now_s()
                rec["decision_latency_s"] = rec["t_brake_command"] - t_detect
                self.set_mode("BRAKING", name)
            # 2) for the record: conservative stopping assessment and the unchanged Phase 2 prediction
            sa = assess_stopping(p, v, new_scene.distance, self.policy.robot_radius, self.policy.nominal, self.assume)
            rec["stopping_assessment"] = sa.to_dict()
            sup = self._wait_supervisor()
            tic = time.perf_counter()
            pred = sup.assess_update(MotionState(t_recv, p, v, np.zeros(3)), self.active, upd)
            rec["phase2_prediction"] = {k: v_ for k, v_ in pred.items() if k not in ("new_trajectory",)}
            rec["assess_wall_s"] = time.perf_counter() - tic
            rec["conflict_agrees_with_phase2"] = bool(pred.get("conflict")) == conflict
            if not conflict:
                self.event("UPDATE_NO_CONFLICT", {"name": name})
                return
            self.event("STOP_ASSESSED", {"name": name, "conservative": sa.classification.value,
                                         "phase2_conflict": bool(pred.get("conflict"))})
        else:
            sa = assess_stopping(p, v, new_scene.distance, self.policy.robot_radius, self.policy.nominal, self.assume)
            rec["stopping_assessment"] = sa.to_dict()
            hold_clr = float(new_scene.distance(p[None])[0])
            rec["hold_clearance"] = hold_clr
            if hold_clr < self.policy.nominal:
                self._finish("HOLDING_ABORTED", Outcome.HOLD_UNSAFE.value)

    def _wait_supervisor(self, timeout=120.0):
        t0 = time.perf_counter()
        while self.sup is None or self.sup_version != self.mission["map_version"]:
            if time.perf_counter() - t0 > timeout:
                raise RuntimeError("Phase 2 supervisor not ready")
            time.sleep(0.05)
        return self.sup

    def _after_rest(self, p_hold, t_rest):
        rec = self.mission["updates"][-1]
        upd, ns, pol = self.update, self.new_scene, self.policy
        H = [(t, p) for t, p, _ in self.hist if t >= upd.t_appear - 1e-6 and t <= t_rest + 1e-6]
        P = np.array([p for _, p in H]) if H else p_hold[None]
        d = ns.distance(P)
        seg = np.linalg.norm(np.diff(P, axis=0), axis=1) if len(P) > 1 else np.zeros(1)
        min_meas = float(d.min())
        # sampled measured motion; between samples the distance can be lower by <= half a step
        min_lower = float(min(d.min(), np.min(0.5 * (d[:-1] + d[1:] - seg)) if len(P) > 1 else d.min()))
        hold_clr = float(ns.distance(p_hold[None])[0])
        rec.update(t_rest=t_rest, hold_point=p_hold.tolist(), hold_clearance=hold_clr,
                   measured_min_clearance=min_meas, measured_min_clearance_lower=min_lower,
                   stop_distance_from_brake=float(np.linalg.norm(p_hold - np.asarray(rec["state"]["p"]))))
        if min_lower < pol.robot_radius:
            outcome = Outcome.UNAVOIDABLE_COLLISION.value if min_meas < pol.robot_radius else "clearance_below_radius_bound"
            return self._finish("HOLDING_ABORTED", f"stop: {outcome}", stop_outcome=outcome)
        if min_lower < pol.nominal:
            return self._finish("HOLDING_ABORTED", Outcome.STOP_MARGIN_VIOLATION.value,
                                stop_outcome=Outcome.STOP_MARGIN_VIOLATION.value)
        if hold_clr < pol.nominal:
            return self._finish("HOLDING_ABORTED", Outcome.HOLD_UNSAFE.value, stop_outcome=Outcome.HOLD_UNSAFE.value)
        rec["stop_outcome"] = "safe_stop"
        self.set_mode("REPLANNING", upd.name)
        self.mission["t_replan_start"] = self.now_s()
        threading.Thread(target=self._replan, args=(p_hold, t_rest, (min_lower, min_meas)), daemon=True).start()

    def _replan(self, p_hold, t_rest, stop_clr):
        try:
            tic = time.perf_counter()
            rr = self._wait_supervisor().replan_from_rest(p_hold, self.goal, self.update, t_rest, stop_clr)
            rr["replan_wall_s"] = time.perf_counter() - tic
            self.done.put(("replan", rr))
        except Exception as exc:          # reported, never hidden
            self.done.put(("replan_error", repr(exc)))

    # ---------------------------------------------------------------- periodic
    def _tick(self):
        now = self.now_s()
        if self.mode == "WAIT_MAP" and self.map.scene is not None and self.odom is not None:
            self.set_mode("READY")
        while not self.done.empty():
            kind, payload = self.done.get()
            if kind == "replan_error":
                self._finish("HOLDING_ABORTED", f"replanning error: {payload}")
            else:
                self._replan_done(payload)
        if self.mode in ("FOLLOWING", "FOLLOWING_REPLAN") and self.odom is not None:
            t, p, v = self.odom
            if t > self.active.t_end:
                err, spd = float(np.linalg.norm(p - self.goal)), float(np.linalg.norm(v))
                if err < 0.15 and spd < 0.1:
                    self._finish("GOAL_REACHED", f"goal error {err:.3f} m", goal_error_m=err)
                elif t > self.active.t_end + 10.0:
                    self._finish("MISSION_FAILED", f"goal not reached: error {err:.3f} m", goal_error_m=err)
        if now - self._last_state_pub > 0.5:
            self._last_state_pub = now
            self._publish_state()

    def _replan_done(self, rr):
        out = {k: v for k, v in rr.items() if k != "new_trajectory"}
        self.mission["replan"] = out
        self.mission["t_replan_done"] = self.now_s()
        oc = rr["outcome"]
        oc = oc.value if hasattr(oc, "value") else oc
        if rr.get("new_trajectory") is None:
            return self._finish("HOLDING_ABORTED", oc)
        tr = rr["new_trajectory"]
        t, p, _ = self.odom
        if np.linalg.norm(tr.evaluate(tr.t_start)[0] - p) > 0.3:
            return self._finish("HOLDING_ABORTED", "replanned trajectory does not start at the measured hold point")
        tr = shifted(tr, t + ACTIVATION_DELAY_S - tr.t_start)
        ts = np.linspace(tr.t_start, tr.t_end, max(2, int(tr.duration / 0.2)))
        self.pub_rpath.publish(path_to_msg(tr.evaluate(ts), t))
        self.mission["t_resume"] = tr.t_start
        self._activate(tr, "FOLLOWING_REPLAN")


def QoSProfile_cmd():
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    return QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)


def _pose(p, t):
    from .rosconv import header
    ps = PoseStamped(header=header(t, "map"))
    ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = (float(x) for x in p)
    ps.pose.orientation.w = 1.0
    return ps


def main():
    rclpy.init()
    spin_node(SupervisorNode(), MultiThreadedExecutor(num_threads=4))

"""Webots supervisor plugin: AutoPath scene manager (environment side of the simulation).

Runs in the extern controller of the "AutoPathSceneManager" supervisor robot. It never
commands the drone. Responsibilities:
  * extract the actual collision geometry (boundingObjects) of the running world and publish it
    (/autopath/world/collision_geometry, Detection3DArray, latched) + an extraction report;
  * publish /clock (simulation time) and the ground-truth drone state
    (/autopath/ground_truth/odom) and physical contacts (/autopath/ground_truth/contacts);
  * scenario environment events on request (/autopath/scene/request, JSON): place the drone
    before takeoff (initial condition only, refused once the motors run), drive the forklift
    into the cross aisle, slide the cage gate shut, drop a load into an aisle. Moved objects
    are kinematic (no physics); once an object is in place (t_appear) its geometry is
    re-extracted from Webots and published as an obstacle update after the configured
    detection latency (/autopath/obstacle_updates, Detection3DArray, stamp = t_detect);
  * drawing: original/active/replanned routes, start/goal markers and a status overlay.
"""

from __future__ import annotations

import json
import math
import os

import numpy as np
import rclpy
from nav_msgs.msg import Odometry, Path
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from trajectory_msgs.msg import MultiDOFJointTrajectory
from vision_msgs.msg import Detection3DArray

from . import config as apcfg
from .rosconv import header, msg_to_path, msg_to_traj, records_to_msg, to_time
from .webots_extract import extract_node, extract_world

QOS_LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
QOS_EVENTS = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)

COLORS = {"original": (0.15, 0.45, 1.0), "active": (0.1, 0.9, 0.2), "replanned": (1.0, 0.55, 0.0)}


class AutoPathSceneManager:
    def init(self, webots_node, properties):
        self.sv = webots_node.robot
        self.step_ms = int(self.sv.getBasicTimeStep())
        self.cfg = apcfg.load(properties.get("config") or os.environ.get("AUTOPATH_CONFIG") or None)
        if not rclpy.ok():
            rclpy.init(args=None)
        self.node = rclpy.create_node("autopath_scene_manager")
        n = self.node
        self.pub_clock = n.create_publisher(Clock, "/clock", QoSProfile(depth=10))
        self.pub_geom = n.create_publisher(Detection3DArray, "/autopath/world/collision_geometry", QOS_LATCHED)
        self.pub_report = n.create_publisher(String, "/autopath/world/extraction_report", QOS_LATCHED)
        self.pub_gt = n.create_publisher(Odometry, "/autopath/ground_truth/odom", QoSProfile(depth=200))
        self.pub_contacts = n.create_publisher(String, "/autopath/ground_truth/contacts", QOS_EVENTS)
        self.pub_update = n.create_publisher(Detection3DArray, "/autopath/obstacle_updates", QOS_EVENTS)
        self.pub_event = n.create_publisher(String, "/autopath/events", QOS_EVENTS)
        n.create_subscription(String, "/autopath/scene/request", lambda m: self.requests.append(m.data), QOS_EVENTS)
        n.create_subscription(Path, "/autopath/global_path", lambda m: self._on_path(m, "original"), QOS_EVENTS)
        n.create_subscription(Path, "/autopath/replanned_path", lambda m: self._on_path(m, "replanned"), QOS_EVENTS)
        n.create_subscription(MultiDOFJointTrajectory, "/autopath/active_trajectory", self._on_traj, QOS_LATCHED)
        n.create_subscription(String, "/autopath/supervisor/state", self._on_state, QoSProfile(depth=10))
        self.requests: list[str] = []
        self.anims: list[dict] = []
        self.pending_updates: list[tuple[float, dict]] = []
        self.drone = self.sv.getFromDef("DRONE")
        self.vis = self.sv.getFromDef("AUTOPATH_VIS").getField("children")
        self.vis_ids: dict[str, int] = {}
        self.n_paths = 0
        self.in_contact = False
        self.contact_events = 0
        self.state_text = "INITIALISING"
        self.det_latency = float(self.cfg["supervisor"]["detection_latency_s"])
        self.geometry_dump = properties.get("geometry_dump") or os.environ.get("AUTOPATH_GEOMETRY_DUMP")
        self.extracted = False

    def _extract(self):
        """Run once, at the first step (the driver owns stepping; init() must not step)."""
        ex = extract_world(self.sv)
        self.records = ex.records
        report = {"t": self.sv.getTime(), "n_records": len(ex.records), "n_visual": len(ex.visual),
                  "objects": sorted({r["object"] for r in ex.records}), "planes": ex.planes,
                  "unsupported_bounding_geometry": ex.unsupported, "visual_unbounded": sorted(set(ex.visual_unbounded)),
                  "visual_records": ex.visual}
        self.pub_geom.publish(records_to_msg(ex.records, self.sv.getTime()))
        self.pub_report.publish(String(data=json.dumps(report)))
        out = self.geometry_dump
        if out:
            with open(out, "w") as f:
                json.dump({"records": ex.records, **{k: v for k, v in report.items() if k != "objects"}}, f, indent=1)
        self._event("WORLD_EXTRACTED", {"n_records": len(ex.records), "unsupported": len(ex.unsupported)})
        self._label()

    # ---------------------------------------------------------------- helpers
    def _event(self, name, data=None):
        self.pub_event.publish(String(data=json.dumps({"t": self.sv.getTime(), "source": "scene", "name": name,
                                                       "data": data or {}})))

    def _set_translation(self, node, xyz):
        node.getField("translation").setSFVec3f([float(c) for c in xyz])

    def _label(self):
        self.sv.setLabel(0, f"AutoPath-3D | {self.state_text}", 0.01, 0.01, 0.07, 0xFFFFFF, 0.0, "Arial")

    def _on_state(self, msg):
        try:
            s = json.loads(msg.data)
        except ValueError:
            return
        txt = s.get("mode", "?")
        if s.get("detail"):
            txt += f" - {s['detail']}"
        if txt != self.state_text:
            self.state_text = txt
            self._label()

    # ---------------------------------------------------------------- drawing
    def _draw(self, key, pts, color, width_note=""):
        pts = np.asarray(pts, float)
        if len(pts) < 2:
            return
        if key in self.vis_ids:
            idx = self._find_vis(key)
            if idx is not None:
                self.vis.removeMF(idx)
        coords = " ".join(f"{p[0]:.3f} {p[1]:.3f} {p[2]:.3f}" for p in pts)
        index = " ".join(str(i) for i in range(len(pts))) + " -1"
        r, g, b = color
        node = (f'DEF VIS_{key.upper()} Shape {{ appearance Appearance {{ material Material {{ diffuseColor {r} {g} {b} '
                f'emissiveColor {r} {g} {b} }} }} geometry IndexedLineSet {{ coord Coordinate {{ point [ {coords} ] }} '
                f'coordIndex [ {index} ] }} castShadows FALSE }}')
        self.vis.importMFNodeFromString(-1, node)
        self.vis_ids[key] = 1

    def _find_vis(self, key):
        for i in range(self.vis.getCount()):
            if self.vis.getMFNode(i).getDef() == f"VIS_{key.upper()}":
                return i
        return None

    def _marker(self, key, p, color, radius=0.12):
        if key in self.vis_ids:
            idx = self._find_vis(key)
            if idx is not None:
                self.vis.removeMF(idx)
        r, g, b = color
        self.vis.importMFNodeFromString(-1, f'DEF VIS_{key.upper()} Pose {{ translation {p[0]:.3f} {p[1]:.3f} {p[2]:.3f} '
                                            f'children [ Shape {{ appearance PBRAppearance {{ baseColor {r} {g} {b} '
                                            f'emissiveColor {r} {g} {b} transparency 0.3 }} geometry Sphere {{ radius {radius} }} '
                                            f'castShadows FALSE }} ] }}')
        self.vis_ids[key] = 1

    def _on_path(self, msg, key):
        pts = msg_to_path(msg)
        self.n_paths += 1
        self._draw(key, pts, COLORS[key])
        if len(pts):
            self._marker("goal", pts[-1], (1.0, 0.1, 0.1))
            if key == "original":
                self._marker("start", pts[0], (0.1, 0.8, 0.2))

    def _on_traj(self, msg):
        tr = msg_to_traj(msg)
        if tr.n_pieces == 0:
            return
        ts = np.linspace(tr.t_start, tr.t_end, max(2, int(tr.duration / 0.1)))
        self._draw("active", tr.evaluate(ts), COLORS["active"])

    # ---------------------------------------------------------------- scenario events
    def _handle_request(self, raw):
        try:
            r = json.loads(raw)
        except ValueError:
            self._event("BAD_REQUEST", {"data": raw}); return
        name, t = r.get("name"), self.sv.getTime()
        if name == "place_drone":
            # initial condition only: refused while the propellers turn (drone flying)
            speed, z = float(np.linalg.norm(self.drone.getVelocity()[:3])), float(self.drone.getPosition()[2])
            if speed > 0.02 or z > 0.05:
                self._event("PLACE_REFUSED", {"reason": "drone not at rest on the floor", "speed": speed, "z": z})
                return
            x, y = r["xy"]
            self._set_translation(self.drone, (x, y, 0.015))
            self.drone.getField("rotation").setSFRotation([0, 0, 1, float(r.get("yaw", 0.0))])
            self.drone.resetPhysics()
            self._event("DRONE_PLACED", {"xy": [x, y]})
        elif name == "forklift":
            node = self.sv.getFromDef("FORKLIFT")
            z = node.getPosition()[2]
            fp = self.cfg["scene_events"]["forklift"]
            self.anims.append({"name": "forklift", "node": node, "t0": t, "T": fp["duration"],
                               "a": np.array([*fp["from"], z]), "b": np.array([*fp["to"], z])})
            self._event("OBSTACLE_MOVING", {"object": "forklift"})
        elif name == "gate":
            node = self.sv.getFromDef("CAGE_GATE")
            a = np.array(node.getPosition())
            gs = self.cfg["scene_events"]["gate"]
            self.anims.append({"name": "gate", "node": node, "t0": t, "T": gs["duration"], "a": a,
                               "b": a + np.array([0, gs["dy"], 0])})
            self._event("OBSTACLE_MOVING", {"object": "gate"})
        elif name == "drop_load":
            node = self.sv.getFromDef("DROP_LOAD")
            p = np.array([r["xy"][0], r["xy"][1], 0.0])
            self.anims.append({"name": "drop_load", "node": node, "t0": t, "T": 0.0, "a": p, "b": p})
            self._event("OBSTACLE_MOVING", {"object": "drop_load", "xy": r["xy"]})
        elif name == "screenshot":
            self.sv.exportImage(r["path"], int(r.get("quality", 90)))
            self._event("SCREENSHOT", {"path": r["path"]})
        elif name == "viewpoint":
            vp = self.sv.getFromDef("VIEWPOINT") or self.sv.getRoot().getField("children").getMFNode(1)
            if r.get("position"):
                vp.getField("position").setSFVec3f(r["position"])
            if r.get("orientation"):
                vp.getField("orientation").setSFRotation(r["orientation"])
        else:
            self._event("UNKNOWN_REQUEST", {"request": r})

    def _animate(self, t):
        done = []
        for a in self.anims:
            s = 1.0 if a["T"] <= 0 else min(1.0, (t - a["t0"]) / a["T"])
            s = 0.5 - 0.5 * math.cos(math.pi * s)            # smooth start/stop
            self._set_translation(a["node"], a["a"] + s * (a["b"] - a["a"]))
            if s >= 1.0:
                done.append(a)
        for a in done:
            self.anims.remove(a)
            # obstacle fully in place: t_appear. Geometry re-extracted from the simulator.
            ex = extract_node(a["node"])
            recs = [dict(r, object=a["name"]) for r in ex.records]
            self._event("OBSTACLE_APPEARED", {"object": a["name"], "t_appear": t, "records": recs})
            self.pending_updates.append((t + self.det_latency, {"name": a["name"], "t_appear": t, "records": recs}))

    # ---------------------------------------------------------------- step
    def step(self):
        rclpy.spin_once(self.node, timeout_sec=0)
        t = self.sv.getTime()
        self.pub_clock.publish(Clock(clock=to_time(t)))
        if not self.extracted:
            self.extracted = True
            self._extract()
        while self.requests:
            self._handle_request(self.requests.pop(0))
        self._animate(t)
        for item in [u for u in self.pending_updates if u[0] <= t + 1e-9]:
            self.pending_updates.remove(item)
            msg = records_to_msg(item[1]["records"], t)
            msg.header.frame_id = "map"
            for d in msg.detections:
                d.id = f"{item[1]['name']}|{d.id.split('|')[1]}|update"
            self.pub_update.publish(msg)
            self._event("OBSTACLE_UPDATE_PUBLISHED", {"object": item[1]["name"], "t_appear": item[1]["t_appear"],
                                                      "t_detect": t})
        # ground truth
        pos = self.drone.getPosition()
        vel = self.drone.getVelocity()
        R = np.array(self.drone.getOrientation()).reshape(3, 3)
        od = Odometry(header=header(t, "map"), child_frame_id="base_link_ground_truth")
        od.pose.pose.position.x, od.pose.pose.position.y, od.pose.pose.position.z = (float(c) for c in pos)
        qx, qy, qz, qw = _matrix_quat(R)
        od.pose.pose.orientation.x, od.pose.pose.orientation.y = qx, qy
        od.pose.pose.orientation.z, od.pose.pose.orientation.w = qz, qw
        od.twist.twist.linear.x, od.twist.twist.linear.y, od.twist.twist.linear.z = (float(c) for c in vel[:3])
        self.pub_gt.publish(od)       # NOTE: twist here is in the map frame (ground truth)
        cps = self.drone.getContactPoints(True)
        pts = [list(c.getPoint()) for c in cps]
        floor_only = all(p[2] < 0.02 for p in pts) and pos[2] < 0.06
        contact = bool(pts) and not floor_only
        if contact and not self.in_contact:
            self.contact_events += 1
        if pts:
            self.pub_contacts.publish(String(data=json.dumps({"t": t, "points": pts, "floor_only": floor_only,
                                                              "collision": contact, "count": self.contact_events})))
        self.in_contact = contact


def _matrix_quat(R):
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = 0.5 / math.sqrt(tr + 1.0)
        return ((R[2, 1] - R[1, 2]) * s, (R[0, 2] - R[2, 0]) * s, (R[1, 0] - R[0, 1]) * s, 0.25 / s)
    i = int(np.argmax([R[0, 0], R[1, 1], R[2, 2]]))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = 2.0 * math.sqrt(1.0 + R[i, i] - R[j, j] - R[k, k])
    q = [0.0, 0.0, 0.0, 0.0]
    q[i] = 0.25 * s
    q[j] = (R[j, i] + R[i, j]) / s
    q[k] = (R[k, i] + R[i, k]) / s
    q[3] = (R[k, j] - R[j, k]) / s
    return tuple(float(c) for c in q)

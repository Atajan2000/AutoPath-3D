"""Conversions between AutoPath-3D objects and standard ROS 2 messages.

  trajectory      trajectory_msgs/MultiDOFJointTrajectory: one point per knot carrying
                  position, velocity and acceleration (lossless, see trajcodec.py);
                  header.stamp = absolute start time (simulation time), joint_names = [base_link]
  geometry        vision_msgs/Detection3DArray: one detection per primitive; bbox = AABB
                  (orientation identity), results[0].hypothesis.class_id = "box" | "cylinder",
                  id = "<object>|<solid>|<source>" (cylinder radius = bbox.size.x / 2)
  path            nav_msgs/Path (frame map)
  voxels          sensor_msgs/PointCloud2 (x, y, z float32) of occupied voxel centres
"""

from __future__ import annotations

import json

import numpy as np
from builtin_interfaces.msg import Duration, Time
from geometry_msgs.msg import PoseStamped, Transform, Twist
from nav_msgs.msg import Path
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header, String
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose

from autopath3d.trajectories.polynomial import PiecewisePolynomial

from . import trajcodec


# ---------------------------------------------------------------- time
def to_time(t: float) -> Time:
    sec = int(np.floor(t))
    return Time(sec=sec, nanosec=int(round((t - sec) * 1e9)) % 1_000_000_000)


def from_time(m) -> float:
    return float(m.sec) + 1e-9 * float(m.nanosec)


def to_duration(t: float) -> Duration:
    sec = int(np.floor(t))
    return Duration(sec=sec, nanosec=int(round((t - sec) * 1e9)) % 1_000_000_000)


def header(t: float, frame: str = "map") -> Header:
    return Header(stamp=to_time(t), frame_id=frame)


# ---------------------------------------------------------------- trajectory
def traj_to_msg(traj: PiecewisePolynomial, frame: str = "map", child: str = "base_link") -> MultiDOFJointTrajectory:
    t, P, V, A = trajcodec.encode(traj)
    msg = MultiDOFJointTrajectory(header=header(float(t[0]), frame), joint_names=[child])
    for k in range(len(t)):
        tf = Transform()
        tf.translation.x, tf.translation.y, tf.translation.z = (float(x) for x in P[k])
        tf.rotation.w = 1.0
        tw, ta = Twist(), Twist()
        tw.linear.x, tw.linear.y, tw.linear.z = (float(x) for x in V[k])
        ta.linear.x, ta.linear.y, ta.linear.z = (float(x) for x in A[k])
        msg.points.append(MultiDOFJointTrajectoryPoint(transforms=[tf], velocities=[tw], accelerations=[ta],
                                                       time_from_start=to_duration(float(t[k] - t[0]))))
    return msg


def msg_to_traj(msg: MultiDOFJointTrajectory) -> PiecewisePolynomial:
    t0 = from_time(msg.header.stamp)
    t = np.array([t0 + msg.points[k].time_from_start.sec + 1e-9 * msg.points[k].time_from_start.nanosec
                  for k in range(len(msg.points))])
    P = np.array([[p.transforms[0].translation.x, p.transforms[0].translation.y, p.transforms[0].translation.z]
                  for p in msg.points])
    V = np.array([[p.velocities[0].linear.x, p.velocities[0].linear.y, p.velocities[0].linear.z] for p in msg.points])
    A = np.array([[p.accelerations[0].linear.x, p.accelerations[0].linear.y, p.accelerations[0].linear.z]
                  for p in msg.points])
    return trajcodec.decode(t, P, V, A)


# ---------------------------------------------------------------- geometry
def records_to_msg(records: list[dict], t: float, frame: str = "map") -> Detection3DArray:
    out = Detection3DArray(header=header(t, frame))
    for r in records:
        d = Detection3D(header=out.header)
        d.id = f"{r['object']}|{r['solid']}|{r.get('source', 'collision')}"
        h = ObjectHypothesisWithPose()
        h.hypothesis.class_id = r["kind"]
        h.hypothesis.score = 1.0
        d.results.append(h)
        if r["kind"] == "box":
            lo, hi = np.asarray(r["lo"]), np.asarray(r["hi"])
            c, s = 0.5 * (lo + hi), hi - lo
        else:
            c = np.array([r["center"][0], r["center"][1], 0.5 * (r["z"][0] + r["z"][1])])
            s = np.array([2 * r["radius"], 2 * r["radius"], r["z"][1] - r["z"][0]])
        d.bbox.center.position.x, d.bbox.center.position.y, d.bbox.center.position.z = (float(x) for x in c)
        d.bbox.center.orientation.w = 1.0
        d.bbox.size.x, d.bbox.size.y, d.bbox.size.z = (float(x) for x in s)
        out.detections.append(d)
    return out


def msg_to_records(msg: Detection3DArray) -> list[dict]:
    recs = []
    for d in msg.detections:
        obj, solid, src = (d.id.split("|") + ["", "", "collision"])[:3]
        kind = d.results[0].hypothesis.class_id if d.results else "box"
        c = np.array([d.bbox.center.position.x, d.bbox.center.position.y, d.bbox.center.position.z])
        s = np.array([d.bbox.size.x, d.bbox.size.y, d.bbox.size.z])
        if kind == "cylinder":
            recs.append({"object": obj, "solid": solid, "kind": "cylinder", "center": [c[0], c[1]],
                         "radius": s[0] / 2, "z": [c[2] - s[2] / 2, c[2] + s[2] / 2], "source": src})
        else:
            recs.append({"object": obj, "solid": solid, "kind": "box", "lo": (c - s / 2).tolist(),
                         "hi": (c + s / 2).tolist(), "source": src})
    return recs


# ---------------------------------------------------------------- path / clouds / json
def path_to_msg(path: np.ndarray, t: float, frame: str = "map") -> Path:
    msg = Path(header=header(t, frame))
    for p in np.asarray(path, float):
        ps = PoseStamped(header=msg.header)
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = (float(x) for x in p)
        ps.pose.orientation.w = 1.0
        msg.poses.append(ps)
    return msg


def msg_to_path(msg: Path) -> np.ndarray:
    return np.array([[p.pose.position.x, p.pose.position.y, p.pose.position.z] for p in msg.poses]).reshape(-1, 3)


def points_to_cloud(points: np.ndarray, t: float, frame: str = "map") -> PointCloud2:
    pts = np.asarray(points, np.float32).reshape(-1, 3)
    fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1) for i, n in enumerate("xyz")]
    return PointCloud2(header=header(t, frame), height=1, width=len(pts), fields=fields, is_bigendian=False,
                       point_step=12, row_step=12 * len(pts), data=pts.tobytes(), is_dense=True)


def cloud_to_points(msg: PointCloud2) -> np.ndarray:
    return np.frombuffer(bytes(msg.data), np.float32).reshape(-1, 3)


def json_msg(obj) -> String:
    return String(data=json.dumps(obj, default=_jsonable))


def _jsonable(o):
    if isinstance(o, np.ndarray):
        return o.tolist()
    if isinstance(o, (np.floating, np.integer)):
        return o.item()
    if isinstance(o, np.bool_):
        return bool(o)
    if hasattr(o, "value"):
        return o.value
    return str(o)

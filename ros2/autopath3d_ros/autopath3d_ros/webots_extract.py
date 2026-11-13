"""Extract the actual collision geometry (boundingObjects) from a running Webots world.

Runs inside a Webots supervisor controller (the scene-manager plugin). The traversal uses only
the Supervisor node/field API, so it can be unit-tested with fake nodes.

For every top-level node it walks Solid children recursively, composes local transforms
(translation, axis-angle rotation, Transform scale) from the *base node* fields of PROTO
instances (i.e. the values Webots actually simulates), and converts every boundingObject
geometry to world-frame primitive records (see geometry.py):
  Box, Sphere, Capsule, horizontal/tilted Cylinder, IndexedFaceSet  -> AABB box  (conservative)
  Cylinder whose axis is vertical (within 1e-6)                    -> vertical cylinder (exact)
  Plane                                                            -> recorded separately
Unsupported bounding geometry is reported, never silently dropped.

It also records the AABB of *visual* Shape geometry (Box/Cylinder/Sphere/Capsule/
IndexedFaceSet) per object so that visual parts without collision geometry can be audited.
CadShape/Mesh visuals (external meshes) cannot be bounded this way and are listed as such.
"""

from __future__ import annotations

import math

import numpy as np

SOLID_TYPES = {"Solid", "Robot", "Charger", "Connector", "Display", "Emitter", "Receiver", "Pen", "Propeller",
               "Track", "TrackWheel", "Altimeter", "Accelerometer", "Camera", "Compass", "DistanceSensor", "GPS",
               "Gyro", "InertialUnit", "LED", "Lidar", "LightSensor", "Radar", "RangeFinder", "TouchSensor",
               "Speaker", "VacuumGripper", "Brake", "Skin"}
GROUP_TYPES = {"Group", "Pose", "Transform", "Billboard", "Slot"}
VISUAL_GEOMS = {"Box", "Cylinder", "Sphere", "Capsule", "IndexedFaceSet", "Cone", "Plane", "ElevationGrid"}


def axis_angle_matrix(r) -> np.ndarray:
    x, y, z, a = (float(v) for v in r)
    n = math.sqrt(x * x + y * y + z * z)
    if n < 1e-12 or abs(a) < 1e-15:
        return np.eye(3)
    x, y, z = x / n, y / n, z / n
    c, s, C = math.cos(a), math.sin(a), 1 - math.cos(a)
    return np.array([[c + x * x * C, x * y * C - z * s, x * z * C + y * s],
                     [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
                     [z * x * C - y * s, z * y * C + x * s, c + z * z * C]])


def local_transform(translation, rotation, scale=(1, 1, 1)) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = axis_angle_matrix(rotation) @ np.diag(np.asarray(scale, float))
    T[:3, 3] = np.asarray(translation, float)
    return T


def box_aabb(T: np.ndarray, size) -> tuple[np.ndarray, np.ndarray]:
    h = 0.5 * np.asarray(size, float)
    corners = np.array([[sx * h[0], sy * h[1], sz * h[2]] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
    w = corners @ T[:3, :3].T + T[:3, 3]
    return w.min(axis=0), w.max(axis=0)


def cylinder_aabb(T: np.ndarray, radius: float, height: float) -> tuple[np.ndarray, np.ndarray]:
    """Exact AABB of a (possibly scaled/tilted) cylinder with local axis z."""
    M, c = T[:3, :3], T[:3, 3]
    ex, ey, ez = M[:, 0] * radius, M[:, 1] * radius, M[:, 2] * (0.5 * height)
    half = np.sqrt(ex ** 2 + ey ** 2) + np.abs(ez)
    return c - half, c + half


def _fld(node, name):
    try:
        f = node.getBaseNodeField(name) if node.isProto() else node.getField(name)
    except Exception:
        f = None
    if f is None:
        try:
            f = node.getField(name)
        except Exception:
            f = None
    return f


def _vec(node, name, default):
    f = _fld(node, name)
    if f is None:
        return default
    try:
        return f.getSFVec3f()
    except Exception:
        return default


def _rot(node):
    f = _fld(node, "rotation")
    return f.getSFRotation() if f is not None else (0, 0, 1, 0)


def _float(node, name, default=None):
    f = _fld(node, name)
    return f.getSFFloat() if f is not None else default


def _sfnode(node, name):
    f = _fld(node, name)
    return f.getSFNode() if f is not None else None


def _mfnodes(node, name):
    f = _fld(node, name)
    if f is None:
        return []
    return [f.getMFNode(i) for i in range(f.getCount())]


def _name(node, fallback):
    f = _fld(node, "name")
    try:
        return f.getSFString() if f is not None else fallback
    except Exception:
        return fallback


class Extractor:
    def __init__(self, vertical_tol: float = 1e-6):
        self.vertical_tol = vertical_tol
        self.records: list[dict] = []
        self.visual: list[dict] = []          # visual AABB records ("source": "visual")
        self.planes: list[dict] = []
        self.unsupported: list[str] = []
        self.visual_unbounded: list[str] = []  # objects with CadShape/Mesh visuals

    # ---------------------------------------------------------------- traversal
    def extract_top(self, node, obj_name: str | None = None):
        base = node.getBaseTypeName()
        name = obj_name or _name(node, node.getTypeName())
        self._visit(node, np.eye(4), name, name, base)

    def _visit(self, node, T_parent, obj, solid, base=None):
        base = base or node.getBaseTypeName()
        if base in SOLID_TYPES or base in GROUP_TYPES:
            scale = _vec(node, "scale", (1, 1, 1)) if base == "Transform" else (1, 1, 1)
            T = T_parent @ local_transform(_vec(node, "translation", (0, 0, 0)) if base != "Group" else (0, 0, 0),
                                           _rot(node) if base != "Group" else (0, 0, 1, 0), scale)
            if base in SOLID_TYPES:
                solid = _name(node, solid)
                bo = _sfnode(node, "boundingObject")
                if bo is not None:
                    self._bounding(bo, T, obj, solid)
            for ch in _mfnodes(node, "children"):
                if ch is not None:
                    self._visit(ch, T, obj, solid)
            for slot in ("endPoint", "fastHelix", "slowHelix"):      # joints / propellers
                ch = _sfnode(node, slot)
                if ch is not None:
                    self._visit(ch, T, obj, solid)
        elif base == "Shape":
            self._geometry(_sfnode(node, "geometry"), T_parent, obj, solid, visual=True)
        elif base == "CadShape":
            self.visual_unbounded.append(f"{obj}/{solid}: CadShape")
        elif base in ("HingeJoint", "SliderJoint", "BallJoint", "Hinge2Joint"):
            ep = _sfnode(node, "endPoint")
            if ep is not None:
                self._visit(ep, T_parent, obj, solid)

    def _bounding(self, bo, T, obj, solid):
        base = bo.getBaseTypeName()
        if base in GROUP_TYPES:
            Tl = T if base == "Group" else T @ local_transform(
                _vec(bo, "translation", (0, 0, 0)), _rot(bo),
                _vec(bo, "scale", (1, 1, 1)) if base == "Transform" else (1, 1, 1))
            for ch in _mfnodes(bo, "children"):
                if ch is not None:
                    self._bounding(ch, Tl, obj, solid)
        elif base == "Shape":
            self._geometry(_sfnode(bo, "geometry"), T, obj, solid, visual=False)
        else:
            self._geometry(bo, T, obj, solid, visual=False)

    # ---------------------------------------------------------------- geometry
    def _add(self, lo, hi, obj, solid, visual):
        rec = {"object": obj, "solid": solid, "kind": "box", "lo": [float(v) for v in lo],
               "hi": [float(v) for v in hi], "source": "visual" if visual else "collision"}
        (self.visual if visual else self.records).append(rec)

    def _geometry(self, g, T, obj, solid, visual):
        if g is None:
            return
        base = g.getBaseTypeName()
        if base == "Box":
            self._add(*box_aabb(T, _vec(g, "size", (2, 2, 2))), obj, solid, visual)
        elif base in ("Cylinder", "Cone"):
            r = _float(g, "radius", 1.0) if base == "Cylinder" else _float(g, "bottomRadius", 1.0)
            h = _float(g, "height", 2.0)
            M = T[:3, :3]
            axis = M[:, 2] / max(np.linalg.norm(M[:, 2]), 1e-12)
            sx, sy = np.linalg.norm(M[:, 0]), np.linalg.norm(M[:, 1])
            lo, hi = cylinder_aabb(T, r, h)
            if (not visual and base == "Cylinder" and abs(abs(axis[2]) - 1) < self.vertical_tol
                    and abs(sx - sy) < 1e-9):
                self.records.append({"object": obj, "solid": solid, "kind": "cylinder",
                                     "center": [float(T[0, 3]), float(T[1, 3])], "radius": float(r * sx),
                                     "z": [float(lo[2]), float(hi[2])], "source": "collision"})
            else:
                self._add(lo, hi, obj, solid, visual)
        elif base == "Sphere":
            r = _float(g, "radius", 1.0)
            s = np.linalg.norm(T[:3, :3], axis=0).max()
            self._add(T[:3, 3] - r * s, T[:3, 3] + r * s, obj, solid, visual)
        elif base == "Capsule":
            r, h = _float(g, "radius", 1.0), _float(g, "height", 2.0)
            s = np.linalg.norm(T[:3, :3], axis=0).max()
            lo, hi = cylinder_aabb(T, r, h)
            self._add(lo - r * s, hi + r * s, obj, solid, visual)
        elif base == "IndexedFaceSet":
            coord = _sfnode(g, "coord")
            f = _fld(coord, "point") if coord is not None else None
            if f is None or f.getCount() == 0:
                (self.visual_unbounded if visual else self.unsupported).append(f"{obj}/{solid}: empty IndexedFaceSet")
                return
            P = np.array([f.getMFVec3f(i) for i in range(f.getCount())], float)
            W = P @ T[:3, :3].T + T[:3, 3]
            self._add(W.min(axis=0), W.max(axis=0), obj, solid, visual)
        elif base == "Plane":
            if not visual:
                size = _fld(g, "size")
                sz = size.getSFVec2f() if size is not None else [1, 1]
                self.planes.append({"object": obj, "solid": solid, "origin": T[:3, 3].tolist(),
                                    "normal": (T[:3, :3] @ np.array([0, 0, 1.0])).tolist(), "size": list(sz)})
        elif visual:
            self.visual_unbounded.append(f"{obj}/{solid}: {base}")
        else:
            self.unsupported.append(f"{obj}/{solid}: {base}")


def extract_world(supervisor, skip_names=("Crazyflie", "AutoPathSceneManager"), skip_types=("Viewpoint",
                  "WorldInfo", "TexturedBackground", "TexturedBackgroundLight", "PointLight", "DirectionalLight",
                  "SpotLight", "Fog", "Background")) -> Extractor:
    ex = Extractor()
    root = supervisor.getRoot()
    ch = root.getField("children")
    for i in range(ch.getCount()):
        n = ch.getMFNode(i)
        if n is None or n.getTypeName() in skip_types or n.getBaseTypeName() in skip_types:
            continue
        name = _name(n, n.getTypeName())
        if name in skip_names:
            continue
        ex.extract_top(n, name)
    return ex


def extract_node(node) -> Extractor:
    ex = Extractor()
    ex.extract_top(node)
    return ex

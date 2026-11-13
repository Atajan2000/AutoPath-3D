"""Warehouse map node: Webots collision geometry -> AutoPath planning geometry and voxel map.

in   /autopath/world/collision_geometry   Detection3DArray (latched), from the scene manager
in   /autopath/world/extraction_report    String JSON (latched)
in   /autopath/obstacle_updates           Detection3DArray
out  /autopath/map/obstacles              Detection3DArray (latched): planning primitives =
                                          collision primitives inside the geofence + rack envelopes
                                          + uncovered visual parts + received updates
out  /autopath/map/voxels                 PointCloud2 (latched): occupied surface voxel centres
out  /autopath/map/info                   String JSON (latched): resolution, shape, counts, audit
The occupancy grid is built with the unchanged AutoPath OccupancyGrid (FREE/OCCUPIED/UNKNOWN,
world<->grid transforms). This first integration uses the known simulation map (not SLAM):
every voxel not touching extracted geometry is FREE; nothing is marked UNKNOWN.
"""

from __future__ import annotations

import json
import time

import numpy as np
import rclpy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from vision_msgs.msg import Detection3DArray

from autopath3d.maps.distance_field import surface_mask
from autopath3d.maps.occupancy import OCCUPIED, UNKNOWN, OccupancyGrid

from .common import QOS_EVENTS, QOS_LATCHED, AutoPathNode, spin_node
from .geometry import WorldGeometry, build_planning_records
from .rosconv import from_time, json_msg, msg_to_records, points_to_cloud, records_to_msg


class MapNode(AutoPathNode):
    def __init__(self):
        super().__init__("autopath_map")
        self.pub_obs = self.create_publisher(Detection3DArray, "/autopath/map/obstacles", QOS_LATCHED)
        self.pub_vox = self.create_publisher(PointCloud2, "/autopath/map/voxels", QOS_LATCHED)
        self.pub_info = self.create_publisher(String, "/autopath/map/info", QOS_LATCHED)
        self.report = None
        self.base_records = None
        self.updates: list = []
        self.create_subscription(String, "/autopath/world/extraction_report", self._on_report, QOS_LATCHED)
        self.create_subscription(Detection3DArray, "/autopath/world/collision_geometry", self._on_geom, QOS_LATCHED)
        self.create_subscription(Detection3DArray, "/autopath/obstacle_updates", self._on_update, QOS_EVENTS)

    def _on_report(self, msg):
        self.report = json.loads(msg.data)
        if self.base_records is not None:
            self._build(self._stamp)

    def _on_geom(self, msg):
        self.base_records = msg_to_records(msg)
        self._stamp = from_time(msg.header.stamp)
        if self.report is not None:
            self._build(self._stamp)

    def _on_update(self, msg):
        self.updates += msg_to_records(msg)
        self._build(from_time(msg.header.stamp))

    def _build(self, stamp):
        tic = time.perf_counter()
        m, b = self.cfg["map"], self.cfg["bounds"]
        lo, hi = np.asarray(b["lo"], float), np.asarray(b["hi"], float)
        recs, audit = build_planning_records(self.base_records, self.report.get("visual_records", []),
                                             self.cfg, self.updates)
        geo = WorldGeometry(lo, hi, recs)
        grid = OccupancyGrid.from_scene(geo.scene(), m["resolution"])
        occ = grid.states == OCCUPIED
        surf = np.argwhere(surface_mask(occ))
        build_s = time.perf_counter() - tic
        self.pub_obs.publish(records_to_msg(recs, stamp))
        self.pub_vox.publish(points_to_cloud(grid.grid_to_world(surf) if len(surf) else np.zeros((0, 3)), stamp))
        info = {"stamp": stamp, "resolution": grid.resolution, "shape": list(grid.shape), "lo": lo.tolist(),
                "hi": hi.tolist(), "n_voxels": int(grid.states.size), "n_occupied": int(occ.sum()),
                "n_unknown": int((grid.states == UNKNOWN).sum()), "n_primitives": len(recs),
                "n_collision": sum(r.get("source") == "collision" for r in recs),
                "n_envelope": sum(r.get("source") == "envelope" for r in recs),
                "n_visual_added": len(audit), "visual_added": [f"{r['object']}/{r['solid']}" for r in audit],
                "n_updates": len(self.updates), "build_s": build_s,
                "unsupported_bounding_geometry": self.report.get("unsupported_bounding_geometry", []),
                "visual_unbounded": self.report.get("visual_unbounded", [])}
        self.pub_info.publish(json_msg(info))
        self.event("MAP_PUBLISHED", {k: info[k] for k in ("n_occupied", "n_primitives", "n_visual_added",
                                                          "n_updates", "build_s")}, t=stamp)


def main():
    rclpy.init()
    spin_node(MapNode())

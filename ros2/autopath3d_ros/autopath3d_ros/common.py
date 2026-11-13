"""Shared helpers for the AutoPath ROS 2 nodes."""

from __future__ import annotations

import json
import threading

import numpy as np
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from vision_msgs.msg import Detection3DArray

from autopath3d.environments.scenes import Scene

from . import config as apcfg
from .geometry import WorldGeometry
from .rosconv import from_time, json_msg, msg_to_records

QOS_LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
QOS_RELIABLE = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)
QOS_EVENTS = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)


class AutoPathNode(Node):
    """Node with the AutoPath config, sim-time helpers and an /autopath/events publisher."""

    def __init__(self, name: str):
        super().__init__(name)
        self.declare_parameter("config", "")
        path = self.get_parameter("config").get_parameter_value().string_value or None
        self.cfg = apcfg.load(path)
        self.pub_events = self.create_publisher(String, "/autopath/events", QOS_EVENTS)

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def wait_matched(self, executor, publishers, timeout: float = 30.0):
        """Block until every publisher has at least one matched subscriber, so one-shot requests
        (placement, takeoff, mission) are not lost while DDS discovery is still in progress."""
        import time as _t
        t0 = _t.monotonic()
        while any(p.get_subscription_count() == 0 for p in publishers):
            executor.spin_once(timeout_sec=0.05)
            if _t.monotonic() - t0 > timeout:
                missing = [p.topic_name for p in publishers if p.get_subscription_count() == 0]
                raise TimeoutError(f"no subscriber for {missing}")

    def event(self, name: str, data: dict | None = None, t: float | None = None):
        self.pub_events.publish(json_msg({"t": self.now_s() if t is None else t, "source": self.get_name(),
                                          "name": name, "data": data or {}}))
        self.get_logger().info(f"{name} {json.dumps(data or {}, default=str)[:300]}")


def spin_node(node, executor=None):
    """Spin until Ctrl-C / launch shutdown. rclpy's SIGINT handler may already have shut the
    context down, after which destroy_node() raises; teardown is then skipped (the process exits)."""
    import rclpy
    from rclpy.executors import ExternalShutdownException
    try:
        if executor is None:
            rclpy.spin(node)
        else:
            executor.add_node(node)
            executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            try:
                node.destroy_node()
            except Exception:
                pass
        rclpy.try_shutdown()


class MapClient:
    """Subscribes to /autopath/map/obstacles (planning geometry, latched) and keeps the core Scene."""

    def __init__(self, node: AutoPathNode, callback_group=None, on_map=None):
        self._node = node
        self.lock = threading.RLock()
        self.scene: Scene | None = None
        self.records: list = []
        self.version = 0
        self.stamp = None
        self._on_map = on_map
        node.create_subscription(Detection3DArray, "/autopath/map/obstacles", self._cb, QOS_LATCHED,
                                 callback_group=callback_group)

    def _cb(self, msg):
        recs = msg_to_records(msg)
        b = self._node.cfg["bounds"]
        scene = WorldGeometry(np.asarray(b["lo"], float), np.asarray(b["hi"], float), recs).scene()
        with self.lock:
            self.records, self.scene, self.stamp = recs, scene, from_time(msg.header.stamp)
            self.version += 1
        if self._on_map:
            self._on_map(scene)

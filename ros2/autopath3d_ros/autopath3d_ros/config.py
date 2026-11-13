"""Load the Webots warehouse configuration and build the unchanged AutoPath-3D core objects."""

from __future__ import annotations

import os

import yaml

from autopath3d.pipeline import ClearancePolicy
from autopath3d.replanning.stopping_assessment import StoppingAssumptions
from autopath3d.replanning.supervisor import SupervisorConfig
from autopath3d.trajectories.generator import TrajectoryLimits

from .tracking import OuterLoopGains


def default_config_path() -> str:
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory("autopath3d_ros"), "config", "webots_warehouse.yaml")
    except Exception:
        return os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "config",
                            "webots_warehouse.yaml")


def load(path: str | None = None) -> dict:
    with open(path or default_config_path(), encoding="utf-8") as f:
        return yaml.safe_load(f)


def policy(cfg: dict) -> ClearancePolicy:
    r = cfg["robot"]
    return ClearancePolicy(r["radius"], r["safety_margin"], r["tracking_allowance"])


def limits(cfg: dict) -> TrajectoryLimits:
    return TrajectoryLimits(**cfg["limits"])


def supervisor_config(cfg: dict) -> SupervisorConfig:
    s, m = cfg["supervisor"], cfg["map"]
    return SupervisorConfig(detection_latency_s=s["detection_latency_s"], decision_latency_s=s["decision_latency_s"],
                            resolution=m["resolution"], clearance_cap=m["clearance_cap"],
                            replan_attempts=[dict(a) for a in s["replan_attempts"]])


def stopping_assumptions(cfg: dict) -> StoppingAssumptions:
    return StoppingAssumptions(**cfg["stopping_assumptions"])


def outer_gains(cfg: dict) -> OuterLoopGains:
    b = cfg["bridge"]
    keys = OuterLoopGains.__dataclass_fields__.keys()
    return OuterLoopGains(**{k: b[k] for k in keys if k in b})

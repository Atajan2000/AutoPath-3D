"""Phase 3: translational double-integrator drone simulation, closed-loop tracking and safety monitor.

No visualisation or ROS imports here; the Rerun viewer lives in autopath3d.viz.
"""

from .controller import ControllerConfig, TrackingController, discrete_spectral_radius
from .dynamics import DISTURBANCES, GUSTS, IDEAL, Disturbance, get_disturbance, step_control_points, zoh_step
from .metrics import compute_metrics, executed_clearance, load_result, save_result
from .navigator import MODES, Event, Navigator, SimConfig, SimResult
from .safety import WorldDistance, refine_min, step_bounds

__all__ = ["ControllerConfig", "TrackingController", "discrete_spectral_radius", "DISTURBANCES", "GUSTS", "IDEAL",
           "Disturbance", "get_disturbance", "step_control_points", "zoh_step", "compute_metrics",
           "executed_clearance", "load_result", "save_result", "MODES", "Event", "Navigator", "SimConfig",
           "SimResult", "WorldDistance", "refine_min", "step_bounds"]

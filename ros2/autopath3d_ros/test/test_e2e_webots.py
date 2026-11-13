"""End-to-end test: Webots (Windows, via WSL interop) + the full AutoPath ROS 2 stack.

Launches warehouse.launch.py with a scenario, waits for the mission runner to finish and checks
the measured result (physics-based Crazyflie flight, ground-truth contacts and clearance).
Nothing is mocked: the drone is flown by the Bitcraze PID through the AutoPath bridge, from a
trajectory produced by the planner/trajectory/supervisor nodes.

Opt-in (needs Webots and takes minutes):
    source ros2/env.sh && AUTOPATH_E2E=1 python3 -m pytest -v ros2/autopath3d_ros/test/test_e2e_webots.py
    AUTOPATH_E2E_SCENARIOS=A_navigation,B_replan,C1_no_route,C2_unsafe_stop   (default: A_navigation)

Session handling (webots_session.py): one Webots at a time; readiness = both extern controllers
connected (no fixed sleeps); a start that does not reach READY fails as "STARTUP FAILURE" with the
Webots exit code, separately from navigation failures, and is not retried; only the session's own
processes are stopped. Every attempt is appended to <suite dir>/summary.jsonl and its launch log,
session record and results are kept under ~/autopath_e2e/<suite stamp>/.
"""

import glob
import json
import os
import time

import pytest

from webots_session import WebotsSession, wait_no_webots

pytestmark = pytest.mark.skipif(os.environ.get("AUTOPATH_E2E") != "1",
                                reason="end-to-end Webots test: set AUTOPATH_E2E=1 (needs Webots + ROS 2)")

SCENARIOS = os.environ.get("AUTOPATH_E2E_SCENARIOS", "A_navigation").split(",")
SUITE_DIR = os.path.join(os.path.expanduser(os.environ.get("AUTOPATH_E2E_DIR", "~/autopath_e2e")),
                         "suite_" + time.strftime("%Y%m%d_%H%M%S"))
FLIGHT_TIMEOUT_S = 900.0

EXPECT = {
    # scenario: (end modes accepted, collisions allowed)
    "A_navigation": ({"GOAL_REACHED"}, False),
    "A2_around_racks": ({"GOAL_REACHED"}, False),
    "B_replan": ({"GOAL_REACHED"}, False),
    "C1_no_route": ({"HOLDING_ABORTED"}, False),
    # unsafe stop: the outcome is reported, a contact is a legitimate (measured) result
    "C2_unsafe_stop": ({"HOLDING_ABORTED"}, True),
}
# Webots ports 1240.. (one per scenario; below the 1250-1349 range Windows excludes on the test host)
PORTS = {name: 1240 + i for i, name in enumerate(EXPECT)}


def _summary(rec: dict):
    os.makedirs(SUITE_DIR, exist_ok=True)
    with open(os.path.join(SUITE_DIR, "summary.jsonl"), "a") as f:
        f.write(json.dumps(rec) + "\n")


def _run(scenario: str) -> dict:
    out = os.path.join(SUITE_DIR, scenario)
    if not wait_no_webots(90.0):
        _summary({"scenario": scenario, "startup": "WEBOTS_ALREADY_RUNNING"})
        pytest.fail(f"STARTUP FAILURE ({scenario}): another Webots instance is still running; not started")
    s = WebotsSession(out, [f"scenario:={scenario}", f"results_dir:={out}"], PORTS[scenario])
    try:
        status = s.start()
        if status != "READY":
            raise AssertionError(f"STARTUP FAILURE ({scenario}): {status}, Webots exit code {s.webots_exit}; "
                                 f"no flight. log {s.log_path}\n{s.tail()}")
        rc = s.wait(FLIGHT_TIMEOUT_S)
        s.record["flight_launch_exit"] = rc
    finally:
        rec = s.close()
        runs = sorted(glob.glob(os.path.join(out, scenario, "run_*.json")))
        rec.update(scenario=scenario, result=runs[-1] if runs else None)
        _summary({k: rec.get(k) for k in ("scenario", "port", "startup", "t_startup_s", "webots_exit_code",
                                          "flight_launch_exit", "result", "cleanup_steps", "leftover_wsl_group",
                                          "leftover_windows", "log")})
    assert not rec["leftover_wsl_group"] and not rec["leftover_windows"], ("processes left behind", rec)
    if rc is None:
        raise AssertionError(f"NAVIGATION TIMEOUT ({scenario}): no exit within {FLIGHT_TIMEOUT_S} s; log {s.log_path}")
    if not runs:
        raise AssertionError(f"NO RESULT ({scenario}): launch exit {rc}, Webots exit {s.webots_exit}; "
                             f"log {s.log_path}\n{s.tail()}")
    with open(runs[-1]) as f:
        res = json.load(f)
    res["_dir"] = out
    return res


@pytest.mark.parametrize("scenario", SCENARIOS)
def test_scenario_end_to_end(scenario):
    res = _run(scenario)
    modes, coll_ok = EXPECT[scenario]
    assert res["end_mode"] in modes, (res["end_mode"], res["end_detail"])
    if not coll_ok:
        assert res["collision_events"] == 0, res.get("first_collision_points")
    # the drone actually flew under physics: tracking samples and a measured flight duration
    assert res["tracking_error_m"]["n"] > 100
    assert res["flight_duration_s"] > 5.0
    if res["end_mode"] == "GOAL_REACHED":
        assert res["goal_reached"] and res["final_goal_error_m"] < 0.15
        # measured centre clearance must stay above the nominal clearance (radius + margin); the
        # trajectory clearance additionally contains the tracking allowance
        assert res["min_clearance_m"] >= 0.20, res["min_clearance_m"]
    if scenario == "B_replan":
        upd = res["supervisor_report"]["updates"]
        assert upd and upd[0]["stop_outcome"] == "safe_stop"
        assert res["supervisor_report"]["replan"]["outcome"] in ("replanned", "replanned_after_retry")
    if scenario == "C1_no_route":
        assert res["end_detail"] in ("no_route_holding", "replan_failed_holding"), res["end_detail"]
    if scenario == "C2_unsafe_stop":
        # negative test: HOLDING_ABORTED alone could also come from a failed replan; require that the
        # supervisor braked and that both assessments predicted the stop to be unsafe. A contact is
        # the expected physical outcome and is reported (collision_events), not asserted away.
        upd = res["supervisor_report"]["updates"]
        assert upd and upd[0].get("t_brake_command") is not None
        assert upd[0]["stopping_assessment"]["classification"] == "collision_possible"
        assert upd[0]["phase2_prediction"]["outcome"] == "unavoidable_collision"
        assert "unavoidable_collision" in str(res["end_detail"]), res["end_detail"]
        print(f"C2 collision_events={res['collision_events']} min_clearance={res['min_clearance_m']:.3f}")
    print("result:", res["_dir"])

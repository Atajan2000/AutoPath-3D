"""AutoPath-3D Phase 4: Webots warehouse (Windows) + Crazyflie bridge + AutoPath ROS 2 stack (WSL).

  ros2 launch autopath3d_ros warehouse.launch.py                       # simulation + stack, idle
  ros2 launch autopath3d_ros warehouse.launch.py scenario:=A_navigation   # run a mission, then exit
  ros2 launch autopath3d_ros warehouse.launch.py stack:=false characterise:=true

Webots runs natively on Windows (started through WSL interop by webots_ros2_driver's
WebotsLauncher). The two extern controllers (Crazyflie bridge, scene manager) connect to it over
TCP. Under WSL mirrored networking the Windows host is 127.0.0.1; webots_ros2's default host
detection reads /etc/resolv.conf, which with DNS tunnelling is 10.255.255.254 (not the host), so
the controllers are started here with an explicit address (env AUTOPATH_WEBOTS_HOST, default
127.0.0.1).
"""

import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, RegisterEventHandler
from launch.conditions import IfCondition, LaunchConfigurationNotEquals
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from webots_ros2_driver.webots_launcher import WebotsLauncher


# Webots TCP port (env AUTOPATH_WEBOTS_PORT). If the port is still held by a Webots instance that is
# closing, a new Webots silently falls back to port+1 and the controllers never connect, so
# back-to-back runs (e2e tests) use distinct ports.
PORT = os.environ.get("AUTOPATH_WEBOTS_PORT", "1234")
DRIVER_PATTERN ="[w]ebots_ros2_driver/driver.*autopath3d_ros"   # [w]: the bash -c line must not match itself


def _controller(robot_name, urdf, env):
    driver = get_package_share_directory("webots_ros2_driver")
    host = os.environ.get("AUTOPATH_WEBOTS_HOST", "127.0.0.1")
    return ExecuteProcess(
        cmd=[os.path.join(driver, "scripts", "webots-controller"), f"--robot-name={robot_name}", "--protocol=tcp",
             f"--ip-address={host}", f"--port={PORT}", "ros2", "--ros-args", "-p", f"robot_description:={urdf}",
             "-p", "use_sim_time:=true"],
        additional_env={"WEBOTS_HOME": get_package_prefix("webots_ros2_driver"), **env},
        output="screen", name=f"controller_{robot_name}")


def generate_launch_description():
    share = get_package_share_directory("autopath3d_ros")
    cfg = LaunchConfiguration("config")
    results = LaunchConfiguration("results_dir")
    env = {"AUTOPATH_CONFIG": os.environ.get("AUTOPATH_CONFIG", os.path.join(share, "config", "webots_warehouse.yaml"))}
    if os.environ.get("AUTOPATH_GEOMETRY_DUMP"):
        env["AUTOPATH_GEOMETRY_DUMP"] = os.environ["AUTOPATH_GEOMETRY_DUMP"]
    webots = WebotsLauncher(world=PathJoinSubstitution([share, "worlds", "autopath_warehouse.wbt"]),
                            mode=LaunchConfiguration("mode"), gui=LaunchConfiguration("gui"), port=PORT)
    sim = {"use_sim_time": True}
    common = [sim, {"config": cfg}]
    stack = IfCondition(LaunchConfiguration("stack"))
    nodes = [
        Node(package="autopath3d_ros", executable="map_node", parameters=common, output="screen", condition=stack),
        Node(package="autopath3d_ros", executable="planner_node", parameters=common, output="screen", condition=stack),
        Node(package="autopath3d_ros", executable="trajectory_node", parameters=common, output="screen", condition=stack),
        Node(package="autopath3d_ros", executable="supervisor_node", parameters=common, output="screen", condition=stack),
    ]
    mission = Node(package="autopath3d_ros", executable="mission_node", output="screen",
                   parameters=common + [{"scenario": LaunchConfiguration("scenario"), "results_dir": results,
                                         "screenshot": LaunchConfiguration("screenshot")}],
                   condition=LaunchConfigurationNotEquals("scenario", ""))
    charac = Node(package="autopath3d_ros", executable="characterise_node", output="screen",
                  parameters=common + [{"results_dir": results}],
                  condition=IfCondition(LaunchConfiguration("characterise")))
    end = [EmitEvent(event=Shutdown())]
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=env["AUTOPATH_CONFIG"]),
        DeclareLaunchArgument("mode", default_value="realtime"),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("stack", default_value="true"),
        DeclareLaunchArgument("scenario", default_value=""),
        DeclareLaunchArgument("characterise", default_value="false"),
        DeclareLaunchArgument("screenshot", default_value="",
                              description="after the mission: Webots exportImage to this path (Webots host path)"),
        DeclareLaunchArgument("results_dir", default_value=os.path.expanduser("~/autopath_results")),
        webots,
        _controller("Crazyflie", os.path.join(share, "resource", "crazyflie_autopath.urdf"), env),
        _controller("AutoPathSceneManager", os.path.join(share, "resource", "scene_manager.urdf"), env),
        *nodes, mission, charac,
        RegisterEventHandler(OnProcessExit(target_action=webots, on_exit=end)),
        RegisterEventHandler(OnProcessExit(target_action=mission, on_exit=end)),
        RegisterEventHandler(OnProcessExit(target_action=charac, on_exit=end)),
        # webots-controller -> `ros2 run` -> driver: SIGTERM on the wrapper orphans the driver, which
        # then stays on the DDS graph and stalls reliable topics of the next run (measured 2026-11-10:
        # 4-6 s delivery delays, lost samples). Kill this package's drivers on shutdown.
        # (a fresh action per shutdown event: launch may emit Shutdown more than once)
        RegisterEventHandler(OnShutdown(on_shutdown=lambda event, context: [ExecuteProcess(
            cmd=["bash", "-c", f"pkill -f '{DRIVER_PATTERN}'; sleep 1; pkill -9 -f '{DRIVER_PATTERN}'; true"],
            output="log")])),
    ])

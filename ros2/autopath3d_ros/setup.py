from glob import glob

from setuptools import setup

PKG = "autopath3d_ros"
setup(
    name=PKG,
    version="0.1.0",
    packages=[PKG],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + PKG]),
        ("share/" + PKG, ["package.xml"]),
        ("share/" + PKG + "/launch", glob("launch/*.py")),
        ("share/" + PKG + "/config", glob("config/*.yaml")),
        ("share/" + PKG + "/worlds", glob("worlds/*.wbt")),
        ("share/" + PKG + "/resource", glob("resource/*.urdf")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Garayev Atajan",
    description="AutoPath-3D ROS 2 + Webots integration",
    license="MIT AND Apache-2.0",   # Apache-2.0: worlds/autopath_warehouse.wbt (modified Webots sample)
    tests_require=["pytest"],
    entry_points={"console_scripts": [
        f"map_node = {PKG}.map_node:main",
        f"planner_node = {PKG}.planner_node:main",
        f"trajectory_node = {PKG}.trajectory_node:main",
        f"supervisor_node = {PKG}.supervisor_node:main",
        f"mission_node = {PKG}.mission_node:main",
        f"characterise_node = {PKG}.characterise_node:main",
    ]},
)
